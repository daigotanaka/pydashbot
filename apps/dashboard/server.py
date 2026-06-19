"""Serve or render a Dash room-map animation dashboard.

Static mode replays the robot's exploration leg by leg: the path is animated with smooth
interpolation while the conservative-exploration territories, their 4x4 cell
grids, and each cell's resolved state (visited / frontier / blocked /
unreachable) update in step with the robot's progress. Wall and obstacle
observations, inferred wall segments, and a live statistics panel are drawn on a
zoomable HTML5 canvas. The output is a single HTML file with all data and code
embedded -- no server, no media files, no external dependencies -- so it can be
opened locally or published to the web as-is.

Dashboard mode runs an HTTP server. POST JSON moves to /move and open / in a
browser to watch the robot pose animate live. GET /animation.html exports the
live session as a standalone static HTML replay.
"""

import argparse
import json
import math
import threading
from pathlib import Path
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

# The dashboard depends only on the mapping library's geometry helpers; it does
# not import the mapping application's runtime, and the mapping app does not
# import the dashboard. The two communicate at runtime over HTTP (POST /move).
from apps.map.policies.conservative_exploration import (
    GRID_CELLS,
    TERRITORY_MM,
    densify_path,
    territory_resolution,
)
from apps.map.exploration_walls import inferred_wall_segments


def accepted_runs(data):
    return [
        run
        for run in data.get('runs', [])
        if run.get('status', 'accepted') in {'accepted', 'partial'}
    ]


def latest_policy(runs):
    """Return the most recent run's conservative-exploration metadata."""
    return next(
        (
            run['conservative_exploration']
            for run in reversed(runs)
            if run.get('conservative_exploration')
        ),
        {},
    )


def cell_state_lookup(resolution):
    """Map each (x, y) cell in a territory to its single resolved state name."""
    states = {}
    for name in ('visited', 'blocked', 'unreachable', 'frontier'):
        for cell in resolution[name]:
            states[cell] = name
    return states


def build_frames(runs, territories, walls_rev, obstacles_rev, segments_rev,
                 territory_mm):
    """Precompute robot pose and per-territory cell states at every path node.

    Each accepted run contributes one frame per pose in its path. Cell
    resolution is recomputed against the densified path prefix that has been
    traversed *so far* and only the blockers/segments discovered up to that
    frame, so replaying reproduces how the explored region -- and the walls it
    reveals -- grew over time. `walls_rev`/`obstacles_rev` are `[x, y, reveal]`
    and `segments_rev` is `[(start, end), reveal]`, where `reveal` is the global
    frame index at which the observation was sensed.
    """
    grid_mm = territory_mm / GRID_CELLS
    frames = []
    traversed = []  # densified points accumulated across runs and legs
    for run_index, run in enumerate(runs):
        path = run.get('path', [])
        timestamp = run.get('timestamp', '')[:19]
        for node_index, pose in enumerate(path):
            gi = len(frames)  # global frame index this node will occupy
            x = float(pose[0])
            y = float(pose[1])
            heading = float(pose[2]) if len(pose) > 2 else 0.0
            # Extend the traversed trail with the densified segment leading
            # into this node so cell coverage matches the explorer's own logic.
            if node_index > 0:
                prev = path[node_index - 1]
                segment = densify_path([prev, pose], grid_mm / 2)
                traversed.extend(segment[1:] if traversed else segment)
            elif not traversed:
                traversed.append((x, y))

            blockers = [
                (b[0], b[1]) for b in walls_rev if b[2] <= gi
            ] + [
                (b[0], b[1]) for b in obstacles_rev if b[2] <= gi
            ]
            wall_segments = [seg for seg, reveal in segments_rev if reveal <= gi]

            cells = {}
            for territory in territories:
                resolution = territory_resolution(
                    territory, traversed, blockers, wall_segments, territory_mm
                )
                states = cell_state_lookup(resolution)
                cells[f'{territory[0]},{territory[1]}'] = {
                    f'{cx},{cy}': states.get((cx, cy), 'frontier')
                    for cx in range(GRID_CELLS)
                    for cy in range(GRID_CELLS)
                }
            frames.append({
                'run': run_index,
                'node': node_index,
                'timestamp': timestamp,
                'x': round(x, 2),
                'y': round(y, 2),
                'heading': round(heading, 2),
                'cells': cells,
            })
    return frames


# Real-world motion timing, mirrored from dash/motion.py so that 1x playback
# tracks wall-clock duration. Obstacle-aware moves (what exploration uses) cap
# at 200 mm/s forward and 100 mm/s reverse; turns run at 85.9 deg/s with a
# 0.05 s settle (MotionController.turn / move). A small fallback covers legs
# with no recorded event and the gap between separate runs.
MOVE_SPEED_MMPS = 200
REVERSE_SPEED_MMPS = 100
TURN_SPEED_DPS = 85.9
TURN_SETTLE_SECONDS = 0.05
DEFAULT_LEG_SECONDS = 0.3
RUN_BOUNDARY_SECONDS = 0.5

DEFAULT_DASHBOARD_HOST = '127.0.0.1'
DEFAULT_DASHBOARD_PORT = 8000


def leg_duration(event):
    """Wall-clock seconds a single command (one path transition) took.

    Turns are timed off the *requested* angle (the robot drives the commanded
    angle even when it under-rotates); forward/reverse legs off the *measured*
    distance, so a leg that stopped early on a wall takes proportionally less
    time.
    """
    if not event:
        return DEFAULT_LEG_SECONDS
    if event.get('action') == 'turn':
        degrees = event.get('requested') or event.get('heading_delta') or 0.0
        return abs(degrees) / TURN_SPEED_DPS + TURN_SETTLE_SECONDS
    distance = event.get('distance_mm')
    if distance is None:
        distance = event.get('requested') or 0.0
    speed = MOVE_SPEED_MMPS if distance >= 0 else REVERSE_SPEED_MMPS
    return abs(distance) / speed


def discovery_frame(point, run_path, offset):
    """Global frame index of the path node nearest `point` within its run.

    A wall or obstacle is recorded just ahead of the robot when a leg ends, so
    the node closest to the observation is the moment it was sensed.
    """
    best_i, best_d = 0, float('inf')
    for i, pose in enumerate(run_path):
        d = (float(pose[0]) - point[0]) ** 2 + (float(pose[1]) - point[1]) ** 2
        if d < best_d:
            best_d, best_i = d, i
    return offset + best_i


def build_payload(data, territory_mm_override=None):
    runs = accepted_runs(data)
    if not runs:
        raise SystemExit('map has no accepted runs to animate')

    # Inferred wall segments are derived from the full wall set; the per-frame
    # reveal below decides when each one becomes visible.
    walls = [
        (float(point[0]), float(point[1]))
        for run in runs
        for point in run.get('walls', [])
    ]

    # Tag each observation with the global frame index at which the robot was
    # closest to it -- i.e. the leg end where it was sensed -- so the animation
    # can reveal walls and obstacles only as the robot bumps into them rather
    # than showing the whole map up front. Frames are concatenated per run in
    # `runs` order, so we accumulate a global offset across runs.
    walls_out, obstacles_out = [], []
    offset = 0
    for run in runs:
        rpath = run.get('path', [])
        for point in run.get('walls', []):
            p = (float(point[0]), float(point[1]))
            walls_out.append(
                [round(p[0], 2), round(p[1], 2), discovery_frame(p, rpath, offset)]
            )
        for point in run.get('obstacles', []):
            p = (float(point[0]), float(point[1]))
            obstacles_out.append(
                [round(p[0], 2), round(p[1], 2), discovery_frame(p, rpath, offset)]
            )
        offset += len(rpath)

    def nearest_wall_reveal(point):
        best, best_d = 0, float('inf')
        for wx, wy, reveal in walls_out:
            d = (wx - point[0]) ** 2 + (wy - point[1]) ** 2
            if d < best_d:
                best_d, best = d, reveal
        return best

    policy = latest_policy(runs)
    focus = tuple(policy.get('focus_territory', (0, 0)))
    territory_mm = float(
        territory_mm_override
        or policy.get('territory_size_mm', TERRITORY_MM)
    )
    grid_mm = territory_mm / GRID_CELLS

    territories = [tuple(t) for t in policy.get('territories', [focus])]
    if focus not in territories:
        territories.append(focus)

    wall_segments = inferred_wall_segments(walls, max_distance=grid_mm)
    # An inferred segment is "discovered" once both of its endpoint walls are.
    segments_rev = [
        (seg, max(nearest_wall_reveal(seg[0]), nearest_wall_reveal(seg[1])))
        for seg in wall_segments
    ]

    frames = build_frames(
        runs, territories, walls_out, obstacles_out, segments_rev, territory_mm
    )

    # Whole path, flattened across runs, for the persistent trail line.
    full_path = [
        [round(float(p[0]), 2), round(float(p[1]), 2)]
        for run in runs
        for p in run.get('path', [])
    ]

    # Real-world seconds for each frame-to-frame transition, so 1x playback
    # matches how long the robot actually took. One transition per path-node
    # gap within a run (timed from that command's event), plus a short gap
    # bridging consecutive runs. Length is len(frames) - 1.
    durations = []
    for run_index, run in enumerate(runs):
        path = run.get('path', [])
        events = run.get('events', [])
        for k in range(max(0, len(path) - 1)):
            event = events[k] if k < len(events) else None
            durations.append(round(leg_duration(event), 3))
        if run_index < len(runs) - 1:
            durations.append(RUN_BOUNDARY_SECONDS)

    return {
        'territory_mm': territory_mm,
        'grid_cells': GRID_CELLS,
        'grid_mm': grid_mm,
        'focus': list(focus),
        'territories': [list(t) for t in territories],
        'walls': walls_out,
        'obstacles': obstacles_out,
        'wall_segments': [
            [
                [round(seg[0][0], 2), round(seg[0][1], 2)],
                [round(seg[1][0], 2), round(seg[1][1], 2)],
                reveal,
            ]
            for seg, reveal in segments_rev
        ],
        'path': full_path,
        'frames': frames,
        'durations': durations,
        'run_count': len(runs),
    }


def render_html(payload, title):
    """Embed the payload into the standalone HTML/JS template."""
    data_json = json.dumps(payload, separators=(',', ':'))
    return HTML_TEMPLATE.replace('__TITLE__', title).replace(
        '__DATA__', data_json
    )


def empty_payload(territory_mm=TERRITORY_MM):
    territory_mm = float(territory_mm)
    return {
        'territory_mm': territory_mm,
        'grid_cells': GRID_CELLS,
        'grid_mm': territory_mm / GRID_CELLS,
        'focus': [0, 0],
        'territories': [],
        'walls': [],
        'obstacles': [],
        'wall_segments': [],
        'path': [],
        'frames': [],
        'durations': [],
        'run_count': 1,
        # Prior-run coverage seeded on a resume (see apply_seed): densified path
        # points that prime visited/unreachable resolution without creating
        # animation frames.
        'seed_path': [],
    }


def frame_from_move(move, index):
    """Normalize a posted move JSON object into an animation frame."""
    pose = move.get('pose')
    if pose is None:
        pose = [move.get('x'), move.get('y'), move.get('heading', 0.0)]
    if not isinstance(pose, (list, tuple)) or len(pose) < 2:
        raise ValueError("move requires pose [x, y, heading] or x/y fields")
    x = float(pose[0])
    y = float(pose[1])
    heading = float(pose[2]) if len(pose) > 2 and pose[2] is not None else 0.0
    return {
        'run': int(move.get('run', 0)),
        'node': int(move.get('node', index)),
        'timestamp': str(move.get('timestamp', '')),
        'x': round(x, 2),
        'y': round(y, 2),
        'heading': round(heading, 2),
        'cells': {},
    }


def _live_territory(x, y, territory_mm):
    """Territory (column, row) index holding world point (x, y)."""
    return (math.floor(x / territory_mm), math.floor(y / territory_mm))


def recompute_wall_segments(payload):
    """Rebuild inferred wall segments from the walls observed so far.

    Produces the same shape the static payload carries -- each segment is
    `[[x, y], [x, y], reveal]` and is revealed once both of its endpoint walls
    have been (the later of the two reveal frames).
    """
    walls = payload['walls']
    wall_points = [(w[0], w[1]) for w in walls]
    segments = inferred_wall_segments(wall_points, max_distance=payload['grid_mm'])

    def nearest_wall_reveal(point):
        best, best_d = 0, float('inf')
        for wx, wy, reveal in walls:
            d = (wx - point[0]) ** 2 + (wy - point[1]) ** 2
            if d < best_d:
                best_d, best = d, reveal
        return best

    payload['wall_segments'] = [
        [
            [round(seg[0][0], 2), round(seg[0][1], 2)],
            [round(seg[1][0], 2), round(seg[1][1], 2)],
            max(nearest_wall_reveal(seg[0]), nearest_wall_reveal(seg[1])),
        ]
        for seg in segments
    ]


def resolve_live_cells(payload):
    """Recompute live territories, focus, and per-frame cell states.

    Mirrors the static `build_frames` cumulative resolution so the live
    dashboard colours cells (visited / frontier / blocked / unreachable)
    exactly like the saved animation. Each frame resolves the densified path
    *prefix* traversed so far against only the blockers and wall segments
    revealed up to that frame, so a visited cell stays visited as the robot
    moves on rather than reverting to frontier.
    """
    territory_mm = payload['territory_mm']
    grid_mm = payload['grid_mm']
    frames = payload['frames']
    seed_path = [(p[0], p[1]) for p in payload.get('seed_path', [])]

    # Territories with coverage, in first-seen order: any seeded prior-run cell
    # plus any the robot has entered this run, then the focus (the territory
    # holding the most recent pose).
    territories = []
    for x, y in seed_path:
        territory = _live_territory(x, y, territory_mm)
        if territory not in territories:
            territories.append(territory)
    for frame in frames:
        territory = _live_territory(frame['x'], frame['y'], territory_mm)
        if territory not in territories:
            territories.append(territory)
    if frames:
        focus = _live_territory(frames[-1]['x'], frames[-1]['y'], territory_mm)
    elif territories:
        focus = territories[-1]
    else:
        focus = tuple(payload.get('focus', (0, 0)))
    if focus not in territories:
        territories.append(focus)
    if not territories:
        territories = [(0, 0)]

    segments_rev = [
        (((seg[0][0], seg[0][1]), (seg[1][0], seg[1][1])), seg[2])
        for seg in payload.get('wall_segments', [])
    ]

    # Prior-run coverage is present from the first frame on, so it primes the
    # traversed trail that every frame's resolution accumulates onto.
    traversed = list(seed_path)
    for gi, frame in enumerate(frames):
        if gi == 0:
            traversed.append((frame['x'], frame['y']))
        else:
            prev = frames[gi - 1]
            segment = densify_path(
                [(prev['x'], prev['y']), (frame['x'], frame['y'])], grid_mm / 2
            )
            traversed.extend(segment[1:])

        blockers = [
            (b[0], b[1]) for b in payload['walls'] if b[2] <= gi
        ] + [
            (b[0], b[1]) for b in payload['obstacles'] if b[2] <= gi
        ]
        wall_segments = [seg for seg, reveal in segments_rev if reveal <= gi]

        cells = {}
        for territory in territories:
            resolution = territory_resolution(
                territory, traversed, blockers, wall_segments, territory_mm
            )
            states = cell_state_lookup(resolution)
            cells[f'{territory[0]},{territory[1]}'] = {
                f'{cx},{cy}': states.get((cx, cy), 'frontier')
                for cx in range(GRID_CELLS)
                for cy in range(GRID_CELLS)
            }
        frame['cells'] = cells

    payload['territories'] = [list(t) for t in territories]
    payload['focus'] = list(focus)


def apply_live_move(payload, move):
    """Append a posted move to a live animation payload."""
    frame = frame_from_move(move, len(payload['frames']))
    payload['frames'].append(frame)
    payload['path'].append([frame['x'], frame['y']])
    if len(payload['frames']) > 1:
        duration = move.get('duration')
        if duration is None and isinstance(move.get('event'), dict):
            duration = leg_duration(move['event'])
        if duration is None:
            duration = DEFAULT_LEG_SECONDS
        payload['durations'].append(round(float(duration), 3))
    for name in ('walls', 'obstacles'):
        for point in move.get(name, []):
            payload[name].append(
                [round(float(point[0]), 2), round(float(point[1]), 2),
                 len(payload['frames']) - 1]
            )
    # Recompute inferred wall geometry and the cumulative cell resolution so the
    # live payload matches what the static animation would render for the same
    # poses (see resolve_live_cells).
    recompute_wall_segments(payload)
    resolve_live_cells(payload)
    return frame


def amend_last_move(payload, move):
    """Amend the most recent frame in place (PUT /move).

    The mapper POSTs each leg's predicted target before driving, then PUTs the
    robot's measured pose here once the leg ends -- so the live map converges on
    what actually happened (e.g. a leg that stopped short of a wall, or a turn
    that under-rotated). Walls/obstacles observed on the leg are appended at this
    frame's index, then cells are re-resolved.
    """
    if not payload['frames']:
        return apply_live_move(payload, move)
    frame = payload['frames'][-1]
    pose = move.get('pose')
    if pose is None:
        pose = [move.get('x'), move.get('y'), move.get('heading')]
    if pose and pose[0] is not None:
        frame['x'] = round(float(pose[0]), 2)
    if pose and len(pose) > 1 and pose[1] is not None:
        frame['y'] = round(float(pose[1]), 2)
    if pose and len(pose) > 2 and pose[2] is not None:
        frame['heading'] = round(float(pose[2]), 2)
    if payload['path']:
        payload['path'][-1] = [frame['x'], frame['y']]
    if move.get('duration') is not None and payload['durations']:
        payload['durations'][-1] = round(float(move['duration']), 3)
    index = len(payload['frames']) - 1
    for name in ('walls', 'obstacles'):
        for point in move.get(name, []):
            payload[name].append(
                [round(float(point[0]), 2), round(float(point[1]), 2), index]
            )
    recompute_wall_segments(payload)
    resolve_live_cells(payload)
    return frame


def apply_seed(payload, seed):
    """Prime a live payload with prior-run knowledge for a resume.

    Seeds already-explored coverage (`path`) and known blockers (`walls`,
    `obstacles`) so the live map shows prior visited / blocked / unreachable
    cells before the robot moves. Unlike a move, a seed creates no animation
    frame -- it only sets the cumulative state that resolve_live_cells applies
    to every frame. Blockers are revealed from frame 0 (reveal index 0) since
    they were already known when this run began. Replaces any earlier seed.
    """
    payload['seed_path'] = [
        [round(float(p[0]), 2), round(float(p[1]), 2)]
        for p in seed.get('path', [])
    ]
    payload['walls'] = [
        [round(float(p[0]), 2), round(float(p[1]), 2), 0]
        for p in seed.get('walls', [])
    ]
    payload['obstacles'] = [
        [round(float(p[0]), 2), round(float(p[1]), 2), 0]
        for p in seed.get('obstacles', [])
    ]
    recompute_wall_segments(payload)
    resolve_live_cells(payload)


# How long the replay dwells on each retraced prior-coverage point. Kept short
# so seeded history fast-forwards rather than dragging out the saved animation.
EXPORT_SEED_FRAME_SECONDS = 0.04


def export_payload(payload):
    """Build a standalone-animation payload that reveals progressively.

    The live dashboard seeds prior-run coverage as `seed_path` plus blockers
    revealed at frame 0, so the *live* view shows that context immediately. In a
    saved replay that looks wrong -- every wall, obstacle, and visited/blocked/
    unreachable cell is already present before the robot moves. Here the seeded
    path becomes leading frames the replay retraces, and each wall/obstacle is
    re-revealed at the frame whose pose lies nearest it, so the map fills in as
    the robot reaches each feature, matching how the static map animation plays.
    """
    export = json.loads(json.dumps(payload))  # private deep copy to mutate
    seed = export.pop('seed_path', None) or []

    seed_frames = []
    for i, point in enumerate(seed):
        x, y = round(float(point[0]), 2), round(float(point[1]), 2)
        if i + 1 < len(seed):
            heading = math.degrees(
                math.atan2(seed[i + 1][1] - point[1], seed[i + 1][0] - point[0])
            )
        else:
            heading = seed_frames[-1]['heading'] if seed_frames else 0.0
        seed_frames.append({
            'run': 0, 'node': i, 'timestamp': '',
            'x': x, 'y': y, 'heading': round(heading, 2), 'cells': {},
        })

    live_frames = export['frames']
    frames = seed_frames + live_frames
    if not frames:
        frames = [{
            'run': 0, 'node': 0, 'timestamp': '',
            'x': 0, 'y': 0, 'heading': 0.0, 'cells': {},
        }]
    export['frames'] = frames
    export['path'] = [[f['x'], f['y']] for f in frames]
    export['seed_path'] = []

    # Re-reveal every blocker at the frame whose pose is nearest it, across the
    # combined retrace + live path, so seeded blockers no longer flash in at
    # frame 0.
    full_path = [(f['x'], f['y']) for f in frames]
    for name in ('walls', 'obstacles'):
        export[name] = [
            [point[0], point[1], discovery_frame((point[0], point[1]), full_path, 0)]
            for point in export[name]
        ]

    # One duration per frame transition (len(frames) - 1): a quick step through
    # each retraced seed frame, then the live run's own recorded timing.
    live_durations = export['durations']
    if seed_frames and live_frames:
        export['durations'] = (
            [EXPORT_SEED_FRAME_SECONDS] * len(seed_frames) + live_durations
        )
    elif seed_frames:
        export['durations'] = [EXPORT_SEED_FRAME_SECONDS] * (len(seed_frames) - 1)
    else:
        export['durations'] = live_durations

    recompute_wall_segments(export)
    resolve_live_cells(export)
    return export


def live_payload_to_map(payload):
    """Synthesize a minimal single-run map from a live payload.

    Used for export when no authoritative map has been uploaded/imported -- it
    captures the path and observed blockers (and is round-trippable through
    build_payload) but, unlike a real mapping-run file, has no per-leg events or
    quality metadata.
    """
    seed = payload.get('seed_path', []) or []
    frames = payload.get('frames', [])
    path = (
        [[p[0], p[1]] for p in seed]
        + [[f['x'], f['y'], f['heading']] for f in frames]
    )
    return {
        'schema_version': 2,
        'runs': [{
            'status': 'accepted',
            'path': path,
            'walls': [[w[0], w[1]] for w in payload.get('walls', [])],
            'obstacles': [[o[0], o[1]] for o in payload.get('obstacles', [])],
            'conservative_exploration': {
                'territory_size_mm': payload.get('territory_mm', TERRITORY_MM),
                'territories': payload.get('territories', []),
                'focus_territory': payload.get('focus', [0, 0]),
            },
        }],
    }


class LiveDashboard:
    def __init__(self, title='Dash Live Map Dashboard', territory_mm=TERRITORY_MM):
        self.title = title
        self.payload = empty_payload(territory_mm)
        self.map_data = None  # authoritative map JSON (uploaded or imported)
        self.lock = threading.Lock()

    def snapshot(self):
        with self.lock:
            return json.loads(json.dumps(self.payload))

    def post_move(self, move):
        with self.lock:
            return apply_live_move(self.payload, move)

    def amend_move(self, move):
        with self.lock:
            return amend_last_move(self.payload, move)

    def seed(self, seed):
        with self.lock:
            apply_seed(self.payload, seed)

    def load_map(self, data):
        """Replace the dashboard state with a full map JSON (import / finalize).

        Renders the map exactly like the standalone animation (build_payload) and
        keeps the raw data so it can be exported verbatim.
        """
        if not isinstance(data, dict):
            raise ValueError('map must be a JSON object')
        try:
            payload = build_payload(data)
        except SystemExit as exc:  # build_payload rejects maps with no runs
            raise ValueError(str(exc))
        with self.lock:
            self.payload = payload
            self.map_data = data
        return payload

    def map_for_export(self):
        with self.lock:
            if self.map_data is not None:
                return self.map_data
            return live_payload_to_map(self.payload)

    def static_html(self):
        return render_html(export_payload(self.snapshot()), self.title)


def dashboard_page(dashboard):
    """Return the live page using the same UI shell as the static animation."""
    payload = dashboard.snapshot()
    territory_mm = payload.get('territory_mm', TERRITORY_MM)
    initial_payload = empty_payload(territory_mm)
    apply_live_move(initial_payload, {'pose': [0, 0, 0], 'duration': 0})
    html = render_html(initial_payload, dashboard.title)
    html = html.replace(
        'button.ghost {',
        'button.ghost, a.ghost {',
    ).replace(
        'button.ghost:hover {',
        'button.ghost:hover, a.ghost:hover {',
    ).replace(
        '  .hint {',
        '  a.ghost { display: inline-flex; align-items: center;\n'
        '            justify-content: center; text-decoration: none; }\n'
        '  .io-buttons { display: flex; flex-direction: column; gap: 8px; }\n'
        '  .io-buttons .ghost { width: 100%; }\n'
        '  .hint {',
    ).replace(
        '    </aside>',
        '      <div class="section io-buttons">\n'
        '        <button class="ghost" id="importMap" '
        'title="Load a map JSON into the dashboard">Import map</button>\n'
        '        <a class="ghost" id="saveMap" href="/map.json" '
        'title="Download the map as JSON">Save map</a>\n'
        '        <a class="ghost" id="saveAnimation" href="/animation.html" '
        'title="Save standalone animation">Save animation</a>\n'
        '        <input type="file" id="importMapFile" '
        'accept="application/json,.json" style="display:none">\n'
        '      </div>\n'
        '    </aside>',
    )
    return html.replace('</body>', LIVE_DASHBOARD_SCRIPT + '\n</body>')


def make_handler(dashboard):
    class DashboardHandler(BaseHTTPRequestHandler):
        server_version = 'DashMapDashboard/0.1'

        def log_message(self, format, *args):
            print(f'{self.address_string()} - {format % args}')

        def send_bytes(self, body, content_type, status=HTTPStatus.OK):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, payload, status=HTTPStatus.OK):
            self.send_bytes(
                json.dumps(payload, separators=(',', ':')).encode('utf-8'),
                'application/json',
                status,
            )

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path in ('/', '/index.html'):
                self.send_bytes(
                    dashboard_page(dashboard).encode('utf-8'),
                    'text/html; charset=utf-8',
                )
            elif parsed.path == '/state':
                self.send_json(dashboard.snapshot())
            elif parsed.path == '/animation.html':
                body = dashboard.static_html().encode('utf-8')
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Disposition', 'attachment; filename="dash_map_animation.html"')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif parsed.path == '/map.json':
                body = json.dumps(
                    dashboard.map_for_export(), indent=2
                ).encode('utf-8')
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Disposition', 'attachment; filename="dash_map.json"')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif parsed.path == '/move':
                query = parse_qs(parsed.query)
                self.send_json({
                    'endpoint': 'POST /move',
                    'example': {
                        'pose': [100, 200, 90],
                        'duration': 0.5,
                        'timestamp': '2026-06-18T20:00:00',
                    },
                    'received': int(query.get('since', [0])[0] or 0),
                })
            else:
                self.send_json({'error': 'not found'}, HTTPStatus.NOT_FOUND)

        def do_POST(self):
            path = urlparse(self.path).path
            if path not in ('/move', '/moves', '/seed', '/map'):
                self.send_json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                body = self.rfile.read(length).decode('utf-8')
                data = json.loads(body) if body else {}
                if path == '/map':
                    payload = dashboard.load_map(data)
                    self.send_json({'ok': True, 'frames': len(payload['frames'])})
                    return
                if path == '/seed':
                    dashboard.seed(data if isinstance(data, dict) else {})
                    self.send_json({'ok': True, 'seeded': True})
                    return
                moves = data if isinstance(data, list) else data.get('moves', [data])
                frames = [dashboard.post_move(move) for move in moves]
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                self.send_json({'error': str(exc)}, HTTPStatus.BAD_REQUEST)
                return
            self.send_json({'ok': True, 'frames': frames})

        def do_PUT(self):
            if urlparse(self.path).path != '/move':
                self.send_json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                body = self.rfile.read(length).decode('utf-8')
                data = json.loads(body) if body else {}
                frame = dashboard.amend_move(data if isinstance(data, dict) else {})
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                self.send_json({'error': str(exc)}, HTTPStatus.BAD_REQUEST)
                return
            self.send_json({'ok': True, 'frame': frame})

    return DashboardHandler


def serve_dashboard(host, port, title, territory_mm):
    server, dashboard, _thread = start_dashboard_server(
        host, port, title, territory_mm, daemon=False
    )
    print(f'Dashboard listening on http://{host}:{port}')
    print('POST moves to /move as {"pose":[x,y,heading],"duration":0.5}')
    print('Import/export the map JSON via POST /map and GET /map.json')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nDashboard stopped.')
    finally:
        server.server_close()


def start_dashboard_server(host, port, title, territory_mm, daemon=True):
    dashboard = LiveDashboard(title=title, territory_mm=territory_mm)
    server = ThreadingHTTPServer((host, port), make_handler(dashboard))
    thread = threading.Thread(target=server.serve_forever, daemon=daemon)
    if daemon:
        thread.start()
    return server, dashboard, thread


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_file', type=Path, nargs='?')
    parser.add_argument(
        '--host',
        default=None,
        help=f'run live dashboard HTTP server on HOST (default: {DEFAULT_DASHBOARD_HOST})',
    )
    parser.add_argument(
        '--port',
        type=int,
        default=None,
        help=f'run live dashboard HTTP server on PORT (default: {DEFAULT_DASHBOARD_PORT})',
    )
    parser.add_argument(
        '--output',
        type=Path,
        help='HTML output path (default: <map>_animation.html beside the map)',
    )
    parser.add_argument(
        '--territory-size',
        type=float,
        help='override the territory size (mm) recorded in the map',
    )
    parser.add_argument(
        '--title',
        default=None,
        help='page title shown in the animation header',
    )
    options = parser.parse_args()

    if options.host is not None or options.port is not None or options.map_file is None:
        serve_dashboard(
            options.host or DEFAULT_DASHBOARD_HOST,
            options.port or DEFAULT_DASHBOARD_PORT,
            options.title or 'Dash Live Map Dashboard',
            options.territory_size or TERRITORY_MM,
        )
        return

    data = json.loads(options.map_file.read_text())
    payload = build_payload(data, territory_mm_override=options.territory_size)
    title = options.title or f'Dash Room Map -- {options.map_file.stem}'
    html = render_html(payload, title)

    output = options.output or options.map_file.with_name(
        f'{options.map_file.stem}_animation.html'
    )
    output.write_text(html)
    print(
        f'Animation saved -> {output} '
        f'({len(payload["frames"])} frames, '
        f'{len(payload["territories"])} territories)'
    )


HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
  :root {
    --bg: #0b0f1a;
    --panel: #141a2b;
    --panel-2: #1b2236;
    --ink: #e6ecff;
    --muted: #8b96b8;
    --accent: #5b8cff;
    --grid: #2a3354;
    --visited: #57d093;
    --frontier: #f2c94c;
    --blocked: #ef5d6b;
    --unreachable: #6b7494;
    --robot: #6fb3ff;
    --wall: #ff5d5d;
    --obstacle: #ffa64d;
  }
  * { box-sizing: border-box; }
  html, body {
    margin: 0; height: 100%;
    background: radial-gradient(1200px 800px at 70% -10%, #182338, transparent),
                var(--bg);
    background-color: var(--bg);
    color: var(--ink);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica,
                 Arial, sans-serif;
    -webkit-font-smoothing: antialiased;
  }
  .app { display: flex; flex-direction: column; height: 100vh; }
  header {
    padding: 14px 22px; display: flex; align-items: baseline; gap: 16px;
    border-bottom: 1px solid var(--grid);
    background: linear-gradient(180deg, var(--panel), transparent);
  }
  header h1 { font-size: 16px; margin: 0; font-weight: 650; letter-spacing: .2px; }
  header .sub { color: var(--muted); font-size: 12.5px; }
  .stage { flex: 1; display: flex; min-height: 0; }
  .canvas-wrap { flex: 1; position: relative; min-width: 0; }
  canvas { display: block; width: 100%; height: 100%; cursor: grab; }
  canvas.dragging { cursor: grabbing; }
  aside {
    width: 270px; flex-shrink: 0; padding: 18px; overflow-y: auto;
    border-left: 1px solid var(--grid);
    background: var(--panel);
  }
  aside h2 {
    font-size: 11px; text-transform: uppercase; letter-spacing: 1.2px;
    color: var(--muted); margin: 0 0 10px;
  }
  .section { margin-bottom: 22px; }
  .legend-row {
    display: flex; align-items: center; gap: 9px; font-size: 13px;
    margin-bottom: 7px;
  }
  .swatch {
    width: 14px; height: 14px; border-radius: 3px; flex-shrink: 0;
    border: 1px solid rgba(255,255,255,.18);
  }
  .stat-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
  .stat {
    background: var(--panel-2); border-radius: 9px; padding: 10px 11px;
    border: 1px solid var(--grid);
  }
  .stat .num { font-size: 21px; font-weight: 680; line-height: 1; }
  .stat .lbl {
    font-size: 10.5px; color: var(--muted); margin-top: 5px;
    text-transform: uppercase; letter-spacing: .6px;
  }
  .stat.visited .num { color: var(--visited); }
  .stat.frontier .num { color: var(--frontier); }
  .stat.blocked .num { color: var(--blocked); }
  .stat.unreachable .num { color: var(--unreachable); }
  .meta { font-size: 12.5px; color: var(--muted); line-height: 1.7; }
  .meta b { color: var(--ink); font-weight: 600; }
  footer {
    border-top: 1px solid var(--grid); padding: 12px 22px;
    background: linear-gradient(0deg, var(--panel), transparent);
    display: flex; align-items: center; gap: 16px;
  }
  button.ctrl {
    background: var(--accent); color: #fff; border: none; border-radius: 9px;
    width: 44px; height: 44px; font-size: 17px; cursor: pointer;
    display: grid; place-items: center; transition: filter .15s;
  }
  button.ctrl:hover { filter: brightness(1.12); }
  button.ghost {
    background: var(--panel-2); color: var(--ink); border: 1px solid var(--grid);
    border-radius: 8px; height: 34px; padding: 0 12px; cursor: pointer;
    font-size: 13px;
  }
  button.ghost:hover { border-color: var(--accent); }
  .scrub { flex: 1; display: flex; flex-direction: column; gap: 5px; }
  .scrub-row { display: flex; align-items: center; gap: 8px; }
  .scrub-row input[type=range] { flex: 1; min-width: 0; accent-color: var(--accent); }
  button.jump { width: 30px; padding: 0; display: grid; place-items: center;
                font-size: 10px; flex-shrink: 0; }
  .scrub .ticks {
    display: flex; justify-content: space-between; font-size: 11px;
    color: var(--muted);
  }
  .speed { display: flex; align-items: center; gap: 8px; font-size: 12.5px;
           color: var(--muted); }
  .speed select {
    background: var(--panel-2); color: var(--ink); border: 1px solid var(--grid);
    border-radius: 7px; padding: 6px 8px; font-size: 12.5px;
  }
  .hint {
    position: absolute; bottom: 12px; left: 12px; font-size: 11.5px;
    color: var(--muted); background: rgba(11,15,26,.7); padding: 6px 10px;
    border-radius: 7px; border: 1px solid var(--grid); pointer-events: none;
  }
</style>
</head>
<body>
<div class="app">
  <header>
    <h1>__TITLE__</h1>
    <span class="sub" id="headerSub"></span>
  </header>
  <div class="stage">
    <div class="canvas-wrap">
      <canvas id="view"></canvas>
      <div class="hint">drag to pan &middot; scroll to zoom &middot; double-click to reset</div>
    </div>
    <aside>
      <div class="section">
        <h2>Focus territory progress</h2>
        <div class="stat-grid" id="stats"></div>
      </div>
      <div class="section">
        <h2>Cell states</h2>
        <div class="legend-row"><span class="swatch" style="background:var(--visited)"></span>Visited</div>
        <div class="legend-row"><span class="swatch" style="background:var(--frontier)"></span>Frontier (reachable, unvisited)</div>
        <div class="legend-row"><span class="swatch" style="background:var(--blocked)"></span>Blocked (wall / obstacle)</div>
        <div class="legend-row"><span class="swatch" style="background:var(--unreachable)"></span>Unreachable</div>
      </div>
      <div class="section">
        <h2>Map features</h2>
        <div class="legend-row"><span class="swatch" style="background:var(--wall)"></span>Wall observation</div>
        <div class="legend-row"><span class="swatch" style="background:var(--obstacle)"></span>Obstacle observation</div>
        <div class="legend-row"><span class="swatch" style="background:var(--robot)"></span>Robot &amp; path</div>
      </div>
      <div class="section">
        <h2>Current state</h2>
        <div class="meta" id="meta"></div>
      </div>
    </aside>
  </div>
  <footer>
    <button class="ctrl" id="play" title="Play / pause">&#9658;</button>
    <div class="scrub">
      <div class="scrub-row">
        <button class="ghost jump" id="toStart" title="Jump to beginning">&#9664;</button>
        <input type="range" id="seek" min="0" max="0" value="0" step="1">
        <button class="ghost jump" id="toEnd" title="Jump to end">&#9654;</button>
      </div>
      <div class="ticks">
        <span id="frameLabel">frame 0</span>
        <span id="poseLabel"></span>
      </div>
    </div>
    <div class="speed">
      speed
      <select id="speed">
        <option value="0.5">0.5&times;</option>
        <option value="1" selected>1&times;</option>
        <option value="2">2&times;</option>
        <option value="4">4&times;</option>
      </select>
      <button class="ghost" id="rotate" title="Rotate 90&deg; clockwise">&#10227; rotate</button>
    </div>
  </footer>
</div>

<script id="payload" type="application/json">__DATA__</script>
<script>
const DATA = JSON.parse(document.getElementById('payload').textContent);
const GRID = DATA.grid_cells;
const TMM = DATA.territory_mm;
const GMM = DATA.grid_mm;
const CELL_COLORS = {
  visited: '#57d093', frontier: '#f2c94c',
  blocked: '#ef5d6b', unreachable: '#6b7494',
};

const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d');

// ---- World <-> screen transform (with pan/zoom) ----------------------------
// World is in mm. y grows toward the open room (negative); we flip y for a
// natural "into the room is up" view.
let view = { scale: 1, ox: 0, oy: 0 };  // ox/oy in screen px
let bounds = computeBounds();
let rotationSteps = 1;  // 90-degree clockwise steps applied to the map view (start rotated 90° CW)

function computeBounds() {
  let minX = -250, maxX = TMM + 250, minY = -250, maxY = TMM + 250;
  for (const t of DATA.territories) {
    minX = Math.min(minX, t[0] * TMM - 250);
    maxX = Math.max(maxX, (t[0] + 1) * TMM + 250);
    minY = Math.min(minY, t[1] * TMM - 250);
    maxY = Math.max(maxY, (t[1] + 1) * TMM + 250);
  }
  for (const p of DATA.path) {
    minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
    minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
  }
  return { minX, maxX, minY, maxY };
}

function fitView() {
  const w = canvas.clientWidth, h = canvas.clientHeight, pad = 40;
  const exX = bounds.maxX - bounds.minX, exY = bounds.maxY - bounds.minY;
  // Transpose swaps which world axis spans the screen horizontal/vertical; an
  // odd rotation swaps them again.
  let hExt = TRANSPOSE ? exY : exX;
  let vExt = TRANSPOSE ? exX : exY;
  if (rotationSteps % 2) { const t = hExt; hExt = vExt; vExt = t; }
  view.scale = Math.min((w - pad * 2) / hExt, (h - pad * 2) / vExt);
  // Center the content midpoint at the canvas center (rotation pivots there).
  const midX = (bounds.minX + bounds.maxX) / 2, midY = (bounds.minY + bounds.maxY) / 2;
  const hW = TRANSPOSE ? midY : midX, vW = TRANSPOSE ? midX : midY;
  view.ox = w / 2 - hW * view.scale;
  view.oy = h / 2 + vW * view.scale;
}

function sx(x) { return view.ox + x * view.scale; }
function sy(y) { return view.oy - y * view.scale; }

// Axis transpose: draw world +x up the page and world +y across it, matching
// how the room reads from the dock. Positions only -- coordinate labels keep
// their true map indices. TX/TY take a full (x, y) pair since the transpose
// couples the two axes.
const TRANSPOSE = true;
function TX(x, y) { return sx(TRANSPOSE ? y : x); }
function TY(x, y) { return sy(TRANSPOSE ? x : y); }

// Screen-space axis-aligned square covering the world square [wx,wx+side] x
// [wy,wy+side]; returns [minX, minY, sidePx]. Works under transpose because a
// transpose keeps axis-aligned squares axis-aligned (only the min corner moves).
function squareRect(wx, wy, side) {
  const corners = [[wx, wy], [wx + side, wy], [wx + side, wy + side], [wx, wy + side]];
  let minX = Infinity, minY = Infinity;
  for (const [a, b] of corners) { minX = Math.min(minX, TX(a, b)); minY = Math.min(minY, TY(a, b)); }
  return [minX, minY, side * view.scale];
}

// Map a base (un-rotated) screen point through the active map rotation, which
// pivots about the canvas center. Used to place upright labels at the rotated
// positions without rotating the glyphs themselves.
function rotateScreen(px, py) {
  const k = ((rotationSteps % 4) + 4) % 4;
  if (!k) return [px, py];
  const w = canvas.clientWidth, h = canvas.clientHeight;
  const a = k * Math.PI / 2;
  const c = Math.cos(a), s = Math.sin(a);
  const dx = px - w / 2, dy = py - h / 2;
  return [w / 2 + dx * c - dy * s, h / 2 + dx * s + dy * c];
}

function resize() {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = canvas.clientWidth * dpr;
  canvas.height = canvas.clientHeight * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

// ---- Animation state -------------------------------------------------------
const frames = DATA.frames;
// Real-world seconds per frame transition (from the robot's actual move/turn
// timing); 1x playback advances the playhead in step with these.
const DURATIONS = DATA.durations || [];
const TOTAL_SECONDS = DURATIONS.reduce((a, b) => a + b, 0);
let pos = 0;           // fractional frame index for smooth interpolation
let playing = true;
let speed = 1;

const seek = document.getElementById('seek');
seek.max = frames.length - 1;

function lerp(a, b, t) { return a + (b - a) * t; }
function lerpAngle(a, b, t) {
  let d = ((b - a + 540) % 360) - 180;
  return a + d * t;
}

function interpolatedPose() {
  const i = Math.min(frames.length - 1, Math.floor(pos));
  const j = Math.min(frames.length - 1, i + 1);
  const t = pos - i;
  const f0 = frames[i], f1 = frames[j];
  return {
    x: lerp(f0.x, f1.x, t),
    y: lerp(f0.y, f1.y, t),
    heading: lerpAngle(f0.heading, f1.heading, t),
    frame: f0,
    index: i,
  };
}

// ---- Drawing ---------------------------------------------------------------
function draw() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  ctx.clearRect(0, 0, w, h);

  const pose = interpolatedPose();
  const frame = pose.frame;

  // Rotate the whole map (geometry + robot) about the canvas center; labels are
  // drawn upright afterwards so the text never tilts.
  const k = ((rotationSteps % 4) + 4) % 4;
  ctx.save();
  if (k) {
    ctx.translate(w / 2, h / 2);
    ctx.rotate(k * Math.PI / 2);
    ctx.translate(-w / 2, -h / 2);
  }
  drawCells(frame);
  drawTerritoryBorders();
  drawDockWalls();
  drawWallSegments(pose.index);
  drawPath(pose.index, frame.run);
  drawWalls(pose.index);
  drawCorner();
  drawRobot(pose);
  ctx.restore();

  drawCellLabels(frame);
  drawTerritoryLabels();
}

function drawCells(frame) {
  ctx.font = '600 ' + Math.max(8, 10 * view.scale * 0).toFixed(0) + 'px sans-serif';
  const labelPx = Math.min(13, Math.max(6, GMM * view.scale * 0.16));
  for (const t of DATA.territories) {
    const key = t[0] + ',' + t[1];
    const grid = frame.cells[key];
    if (!grid) continue;
    const isFocus = (t[0] === DATA.focus[0] && t[1] === DATA.focus[1]);
    for (let cx = 0; cx < GRID; cx++) {
      for (let cy = 0; cy < GRID; cy++) {
        const state = grid[cx + ',' + cy] || 'frontier';
        const wx = t[0] * TMM + cx * GMM;
        const wy = t[1] * TMM + cy * GMM;
        const [x, y, s] = squareRect(wx, wy, GMM);  // axis-aligned cell square
        ctx.fillStyle = CELL_COLORS[state];
        ctx.globalAlpha = isFocus ? 0.30 : 0.18;
        ctx.fillRect(x, y, s, s);
        ctx.globalAlpha = 1;
        ctx.strokeStyle = 'rgba(180,200,255,0.12)';
        ctx.lineWidth = 1;
        ctx.strokeRect(x, y, s, s);
      }
    }
  }
}

// Cell coordinate labels, drawn upright after the rotated geometry pass so they
// stay readable at any rotation. Placed at each cell's (rotated) center.
function drawCellLabels(frame) {
  const labelPx = Math.min(13, Math.max(6, GMM * view.scale * 0.16));
  if (labelPx < 7) return;
  ctx.fillStyle = 'rgba(233,239,255,0.72)';
  ctx.font = labelPx.toFixed(0) + 'px ui-monospace, monospace';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  for (const t of DATA.territories) {
    const key = t[0] + ',' + t[1];
    if (!frame.cells[key]) continue;
    for (let cx = 0; cx < GRID; cx++) {
      for (let cy = 0; cy < GRID; cy++) {
        const wx = t[0] * TMM + (cx + 0.5) * GMM;
        const wy = t[1] * TMM + (cy + 0.5) * GMM;
        const [px, py] = rotateScreen(TX(wx, wy), TY(wx, wy));
        ctx.fillText(cx + ',' + cy, px, py);
      }
    }
  }
}

function drawTerritoryBorders() {
  for (const t of DATA.territories) {
    const isFocus = (t[0] === DATA.focus[0] && t[1] === DATA.focus[1]);
    const [x, y, s] = squareRect(t[0] * TMM, t[1] * TMM, TMM);
    ctx.strokeStyle = isFocus ? '#7da2ff' : 'rgba(125,162,255,0.4)';
    ctx.lineWidth = isFocus ? 2.5 : 1.5;
    ctx.strokeRect(x, y, s, s);
  }
}

// Territory labels, drawn upright after the rotated geometry pass and anchored
// to the top-right corner of each territory's on-screen box (so the label sits
// top-right at any rotation).
function drawTerritoryLabels() {
  for (const t of DATA.territories) {
    const isFocus = (t[0] === DATA.focus[0] && t[1] === DATA.focus[1]);
    const corners = [
      [t[0] * TMM, t[1] * TMM],
      [(t[0] + 1) * TMM, t[1] * TMM],
      [(t[0] + 1) * TMM, (t[1] + 1) * TMM],
      [t[0] * TMM, (t[1] + 1) * TMM],
    ].map(([wx, wy]) => rotateScreen(TX(wx, wy), TY(wx, wy)));
    // Top-right on screen = largest (x - y).
    let best = corners[0];
    for (const c of corners) {
      if (c[0] - c[1] > best[0] - best[1]) best = c;
    }
    const s = TMM * view.scale;
    const fs = Math.min(15, Math.max(9, s * 0.07));
    ctx.fillStyle = isFocus ? '#a9c2ff' : 'rgba(139,150,184,0.85)';
    ctx.font = '650 ' + fs.toFixed(0) + 'px ui-monospace, monospace';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'top';
    ctx.fillText('T ' + t[0] + ',' + t[1] + (isFocus ? '  (focus)' : ''),
                 best[0] - 6, best[1] + 5);
  }
}

function drawDockWalls() {
  ctx.strokeStyle = 'rgba(230,236,255,0.55)';
  ctx.lineWidth = 3;
  // side dock wall along y=0, x in [0, TMM]
  ctx.beginPath();
  ctx.moveTo(TX(0, 0), TY(0, 0)); ctx.lineTo(TX(TMM, 0), TY(TMM, 0));
  ctx.stroke();
  // rear dock wall along x=0, into the open room
  const startY = DATA.path.length ? DATA.path[0][1] : -1;
  const wallY = startY < 0 ? -TMM : TMM;
  ctx.beginPath();
  ctx.moveTo(TX(0, 0), TY(0, 0)); ctx.lineTo(TX(0, wallY), TY(0, wallY));
  ctx.stroke();
}

function drawWallSegments(revealIndex) {
  ctx.strokeStyle = 'rgba(255,93,93,0.35)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([6, 5]);
  for (const seg of DATA.wall_segments) {
    if (seg[2] > revealIndex) continue;  // both endpoints discovered yet?
    ctx.beginPath();
    ctx.moveTo(TX(seg[0][0], seg[0][1]), TY(seg[0][0], seg[0][1]));
    ctx.lineTo(TX(seg[1][0], seg[1][1]), TY(seg[1][0], seg[1][1]));
    ctx.stroke();
  }
  ctx.setLineDash([]);
}

function drawPath(uptoIndex, currentRun) {
  // Draw the traversed path as a glowing trail up to the current frame.
  const pts = [];
  for (let k = 0; k <= uptoIndex && k < frames.length; k++) {
    pts.push([frames[k].x, frames[k].y]);
  }
  if (pts.length < 2) return;
  ctx.lineJoin = 'round'; ctx.lineCap = 'round';
  // soft glow underlay
  ctx.strokeStyle = 'rgba(111,179,255,0.18)';
  ctx.lineWidth = 7;
  strokePoly(pts);
  ctx.strokeStyle = 'rgba(111,179,255,0.9)';
  ctx.lineWidth = 2.2;
  strokePoly(pts);
}

function strokePoly(pts) {
  ctx.beginPath();
  ctx.moveTo(TX(pts[0][0], pts[0][1]), TY(pts[0][0], pts[0][1]));
  for (let k = 1; k < pts.length; k++) {
    ctx.lineTo(TX(pts[k][0], pts[k][1]), TY(pts[k][0], pts[k][1]));
  }
  ctx.stroke();
}

// Observations are revealed only once the robot has reached the frame where it
// sensed them (p[2]). A point flashes an expanding ring for a few frames right
// after it is discovered.
function discoveryPulse(x, y, color, age) {
  if (age < 0 || age > 3) return;
  const t = age / 3;            // 0 just discovered -> 1 settled
  ctx.strokeStyle = color;
  ctx.globalAlpha = (1 - t) * 0.8;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(x, y, 6 + t * 18, 0, Math.PI * 2);
  ctx.stroke();
  ctx.globalAlpha = 1;
}

function drawWalls(revealIndex) {
  ctx.lineWidth = 2;
  for (const p of DATA.walls) {
    if (p[2] > revealIndex) continue;
    const x = TX(p[0], p[1]), y = TY(p[0], p[1]), r = 5;
    ctx.strokeStyle = '#ff5d5d';
    ctx.beginPath();
    ctx.moveTo(x - r, y - r); ctx.lineTo(x + r, y + r);
    ctx.moveTo(x + r, y - r); ctx.lineTo(x - r, y + r);
    ctx.stroke();
    discoveryPulse(x, y, '#ff5d5d', revealIndex - p[2]);
  }
  for (const p of DATA.obstacles) {
    if (p[2] > revealIndex) continue;
    const x = TX(p[0], p[1]), y = TY(p[0], p[1]), r = 6;
    ctx.fillStyle = '#ffa64d';
    ctx.beginPath();
    ctx.moveTo(x, y - r); ctx.lineTo(x + r, y + r); ctx.lineTo(x - r, y + r);
    ctx.closePath(); ctx.fill();
    discoveryPulse(x, y, '#ffa64d', revealIndex - p[2]);
  }
}

function drawCorner() {
  const x = TX(0, 0), y = TY(0, 0), r = 9;
  ctx.strokeStyle = '#e6ecff';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(x - r, y); ctx.lineTo(x + r, y);
  ctx.moveTo(x, y - r); ctx.lineTo(x, y + r);
  ctx.stroke();
}

// Top-view Dash avatar: three blue spheres in a triangle (two front, one
// tail) around the head dome with its orange eye. Forward (the two front
// spheres + eye) points toward -y in the SVG's own frame; drawRobot rotates it
// to the live heading. Embedded as an SVG data URI so the marker stays a
// crisp vector asset at any zoom.
const DASH_SVG =
  '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 120 120">' +
  '<defs>' +
    '<radialGradient id="body" cx="38%" cy="30%" r="78%">' +
      '<stop offset="0%" stop-color="#9fe4fb"/>' +
      '<stop offset="46%" stop-color="#26a6da"/>' +
      '<stop offset="100%" stop-color="#0a6b9a"/></radialGradient>' +
    '<radialGradient id="head" cx="40%" cy="26%" r="80%">' +
      '<stop offset="0%" stop-color="#b6edff"/>' +
      '<stop offset="48%" stop-color="#34b4e6"/>' +
      '<stop offset="100%" stop-color="#0c79ad"/></radialGradient>' +
    '<radialGradient id="amber" cx="40%" cy="32%" r="72%">' +
      '<stop offset="0%" stop-color="#ffc978"/>' +
      '<stop offset="58%" stop-color="#f5882a"/>' +
      '<stop offset="100%" stop-color="#d4660f"/></radialGradient>' +
  '</defs>' +
  // tail sphere (back)
  '<circle cx="60" cy="88" r="24" fill="url(#body)" stroke="#063f5c" stroke-width="1.2"/>' +
  '<ellipse cx="52" cy="80" rx="9" ry="6" fill="#ffffff" opacity="0.30"/>' +
  // front-left and front-right spheres
  '<circle cx="34" cy="46" r="24" fill="url(#body)" stroke="#063f5c" stroke-width="1.2"/>' +
  '<ellipse cx="27" cy="38" rx="9" ry="6" fill="#ffffff" opacity="0.32"/>' +
  '<circle cx="86" cy="46" r="24" fill="url(#body)" stroke="#063f5c" stroke-width="1.2"/>' +
  '<ellipse cx="79" cy="38" rx="9" ry="6" fill="#ffffff" opacity="0.32"/>' +
  // orange caps on the forward face of each front sphere
  '<circle cx="32" cy="33" r="8.5" fill="url(#amber)" stroke="#9c4a08" stroke-width="0.8"/>' +
  '<circle cx="88" cy="33" r="8.5" fill="url(#amber)" stroke="#9c4a08" stroke-width="0.8"/>' +
  // dark neck collar under the head
  '<circle cx="60" cy="56" r="22" fill="#10222e" opacity="0.92"/>' +
  // head dome (center, on top)
  '<circle cx="60" cy="55" r="19.5" fill="url(#head)" stroke="#063f5c" stroke-width="1.2"/>' +
  '<ellipse cx="52" cy="47" rx="8" ry="5" fill="#ffffff" opacity="0.38"/>' +
  // orange eye ring toward the front
  '<circle cx="60" cy="46" r="7.5" fill="url(#amber)" stroke="#9c4a08" stroke-width="0.9"/>' +
  '<circle cx="60" cy="46" r="3.4" fill="#10222e" opacity="0.85"/>' +
  '</svg>';
const dashImg = new Image();
let dashReady = false;
dashImg.onload = () => { dashReady = true; };
dashImg.src = 'data:image/svg+xml;charset=utf-8,' + encodeURIComponent(DASH_SVG);

function drawRobot(pose) {
  const x = TX(pose.x, pose.y), y = TY(pose.x, pose.y);
  const rad = pose.heading * Math.PI / 180;
  // Derive the avatar's on-screen facing from the projected heading vector, so
  // the transpose (and any rotation) carries the orientation automatically.
  const ax = TX(pose.x + Math.cos(rad), pose.y + Math.sin(rad));
  const ay = TY(pose.x + Math.cos(rad), pose.y + Math.sin(rad));
  const screenAng = Math.atan2(ay - y, ax - x);
  // footprint sized to Dash's body (~200 mm across the triangle)
  const half = Math.max(11, 115 * view.scale);
  // soft glow under the robot
  const grad = ctx.createRadialGradient(x, y, 0, x, y, half * 1.7);
  grad.addColorStop(0, 'rgba(111,179,255,0.40)');
  grad.addColorStop(1, 'rgba(111,179,255,0)');
  ctx.fillStyle = grad;
  ctx.beginPath(); ctx.arc(x, y, half * 1.7, 0, Math.PI * 2); ctx.fill();

  ctx.save();
  ctx.translate(x, y);
  // rotate so the SVG's "up" (forward) aligns with the on-screen heading
  ctx.rotate(screenAng + Math.PI / 2);
  if (dashReady) {
    ctx.drawImage(dashImg, -half, -half, half * 2, half * 2);
  } else {
    // fallback until the SVG decodes
    ctx.fillStyle = '#26a6da';
    ctx.beginPath(); ctx.arc(0, 0, half * 0.6, 0, Math.PI * 2); ctx.fill();
  }
  ctx.restore();
}

// ---- HUD / panels ----------------------------------------------------------
function countFocus(frame) {
  const key = DATA.focus[0] + ',' + DATA.focus[1];
  const grid = frame.cells[key] || {};
  const c = { visited: 0, frontier: 0, blocked: 0, unreachable: 0 };
  for (const k in grid) c[grid[k]]++;
  return c;
}

const statsEl = document.getElementById('stats');
const metaEl = document.getElementById('meta');
const headerSub = document.getElementById('headerSub');
headerSub.textContent =
  DATA.territories.length + ' territories · ' +
  DATA.run_count + ' run' + (DATA.run_count > 1 ? 's' : '') + ' · ' +
  TMM + ' mm territory · ' + GRID + '×' + GRID + ' cells · ' +
  '~' + Math.round(TOTAL_SECONDS) + ' s at 1×';

function updateHUD(pose) {
  const f = pose.frame;
  const c = countFocus(f);
  statsEl.innerHTML =
    statBox('visited', c.visited) + statBox('frontier', c.frontier) +
    statBox('blocked', c.blocked) + statBox('unreachable', c.unreachable);
  metaEl.innerHTML =
    'pose <b>(' + Math.round(pose.x) + ', ' + Math.round(pose.y) + ')</b> mm<br>' +
    'heading <b>' + Math.round(((pose.heading % 360) + 360) % 360) + '&deg;</b><br>' +
    'run <b>' + (f.run + 1) + '</b> &middot; node <b>' + f.node + '</b><br>' +
    (f.timestamp ? 'time <b>' + f.timestamp.replace('T', ' ') + '</b>' : '');
  document.getElementById('frameLabel').textContent =
    'frame ' + pose.index + ' / ' + (frames.length - 1);
  document.getElementById('poseLabel').textContent =
    'focus T ' + DATA.focus[0] + ',' + DATA.focus[1];
}

function statBox(cls, n) {
  return '<div class="stat ' + cls + '"><div class="num">' + n +
         '</div><div class="lbl">' + cls + '</div></div>';
}

// ---- Main loop -------------------------------------------------------------
let last = null;
function tick(now) {
  // Clamp the per-frame step: the first frame (and any frame after the tab was
  // inactive) can report a huge or negative wall-clock delta, which would make
  // the playhead jump to the end or go negative and look like it "never
  // started". Cap it to a little over one display frame.
  const dt = last === null ? 0 : Math.min(50, Math.max(0, now - last));
  last = now;
  if (playing && frames.length > 1) {
    // Advance by real time: dt seconds covers dt/segDur of the current leg,
    // where segDur is that leg's wall-clock duration.
    const i = Math.min(DURATIONS.length - 1, Math.floor(pos));
    const segDur = Math.max(0.04, DURATIONS[i] || 0.3);
    pos += (dt / 1000) * speed / segDur;
    if (pos >= frames.length - 1) {
      pos = frames.length - 1;
      playing = false;
      playBtn.innerHTML = '&#9658;';
    }
    seek.value = Math.floor(pos);
  }
  const pose = interpolatedPose();
  draw();
  updateHUD(pose);
  requestAnimationFrame(tick);
}

// ---- Controls --------------------------------------------------------------
const playBtn = document.getElementById('play');
playBtn.addEventListener('click', () => {
  if (!playing && pos >= frames.length - 1) pos = 0;
  playing = !playing;
  playBtn.innerHTML = playing ? '&#10073;&#10073;' : '&#9658;';
});
document.getElementById('toStart').addEventListener('click', () => {
  pos = 0; seek.value = 0;
  playing = false; playBtn.innerHTML = '&#9658;';
});
document.getElementById('toEnd').addEventListener('click', () => {
  pos = frames.length - 1; seek.value = pos;
  playing = false; playBtn.innerHTML = '&#9658;';
});
document.getElementById('rotate').addEventListener('click', () => {
  rotationSteps = (rotationSteps + 1) % 4;
  resize();
  fitView();  // refit for the swapped aspect; recenters the view
});
seek.addEventListener('input', () => {
  pos = parseFloat(seek.value);
  playing = false; playBtn.innerHTML = '&#9658;';
});
document.getElementById('speed').addEventListener('change', (e) => {
  speed = parseFloat(e.target.value);
});

// pan & zoom
let drag = null;
canvas.addEventListener('mousedown', (e) => {
  drag = { x: e.clientX, y: e.clientY, ox: view.ox, oy: view.oy };
  canvas.classList.add('dragging');
});
window.addEventListener('mousemove', (e) => {
  if (!drag) return;
  // The map is drawn rotated about the canvas center, so a screen-space drag
  // must be inverse-rotated for the content to follow the cursor.
  const mdx = e.clientX - drag.x, mdy = e.clientY - drag.y;
  const a = (((rotationSteps % 4) + 4) % 4) * Math.PI / 2;
  const c = Math.cos(a), s = Math.sin(a);
  view.ox = drag.ox + mdx * c + mdy * s;
  view.oy = drag.oy - mdx * s + mdy * c;
});
window.addEventListener('mouseup', () => {
  drag = null; canvas.classList.remove('dragging');
});
canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const w = canvas.clientWidth, h = canvas.clientHeight;
  // Inverse-rotate the cursor about the canvas center so zoom keeps the point
  // actually under the cursor fixed when the map is rotated.
  const a = (((rotationSteps % 4) + 4) % 4) * Math.PI / 2;
  const c = Math.cos(a), s = Math.sin(a);
  const dx = (e.clientX - rect.left) - w / 2, dy = (e.clientY - rect.top) - h / 2;
  const mx = w / 2 + dx * c + dy * s;
  const my = h / 2 - dx * s + dy * c;
  const factor = e.deltaY < 0 ? 1.1 : 1 / 1.1;
  // zoom toward cursor
  view.ox = mx - (mx - view.ox) * factor;
  view.oy = my - (my - view.oy) * factor;
  view.scale *= factor;
}, { passive: false });
canvas.addEventListener('dblclick', () => { resize(); fitView(); });

// Initial layout can race the render loop: when the page is opened directly
// (file://) the flex container may not be measured yet, leaving the canvas at
// zero size so nothing draws. A ResizeObserver fires once the canvas has a
// real size and fits the view then; subsequent observed resizes (e.g. window
// changes) just resize without clobbering the user's pan/zoom.
let didFit = false;
function handleResize() {
  resize();
  if (!didFit && canvas.clientWidth > 0 && canvas.clientHeight > 0) {
    fitView();
    didFit = true;
  }
}
if (typeof ResizeObserver !== 'undefined') {
  new ResizeObserver(handleResize).observe(canvas);
}
window.addEventListener('resize', handleResize);

handleResize();
requestAnimationFrame(tick);
</script>
</body>
</html>
"""


LIVE_DASHBOARD_SCRIPT = r"""<script>
// Live dashboard adapter: keep the animation UI, but replace its payload as
// /state delivers new robot poses. The server resolves cells, territories,
// focus, and wall segments exactly like the static animation, so this adapter
// only swaps the data in and nudges the playhead -- the static template
// intentionally owns all drawing, pan/zoom, rotate, scrub, speed, and the
// Dash-avatar code.
let liveFrameCount = 0;

function liveReplaceArray(target, source) {
  target.length = 0;
  for (const item of source) target.push(item);
}

async function livePoll() {
  try {
    const res = await fetch('/state', { cache: 'no-store' });
    const next = await res.json();
    // Always keep at least one frame and one territory so the static template's
    // draw/HUD code never indexes into empty arrays before the first move.
    const nextFrames = (next.frames && next.frames.length)
      ? next.frames
      : [{ run: 0, node: 0, timestamp: '', x: 0, y: 0, heading: 0, cells: {} }];
    const previousCount = liveFrameCount;
    liveFrameCount = next.frames ? next.frames.length : 0;

    liveReplaceArray(DATA.focus, next.focus || [0, 0]);
    liveReplaceArray(
      DATA.territories,
      (next.territories && next.territories.length) ? next.territories : [[0, 0]]
    );
    liveReplaceArray(DATA.frames, nextFrames);
    liveReplaceArray(
      DATA.path,
      (next.path && next.path.length) ? next.path : [[0, 0]]
    );
    liveReplaceArray(DATA.durations, next.durations || []);
    liveReplaceArray(DATA.walls, next.walls || []);
    liveReplaceArray(DATA.obstacles, next.obstacles || []);
    liveReplaceArray(DATA.wall_segments, next.wall_segments || []);
    DATA.run_count = next.run_count || 1;

    bounds = computeBounds();
    seek.max = Math.max(0, DATA.frames.length - 1);
    if (pos > DATA.frames.length - 1) pos = DATA.frames.length - 1;

    // When new poses arrive and the playhead was riding the live edge, advance
    // it so the freshly added leg animates; if the user has scrubbed back, leave
    // their position alone.
    if (liveFrameCount > previousCount) {
      if (previousCount === 0 || pos >= previousCount - 1) {
        pos = Math.max(0, liveFrameCount - 2);
        playing = true;
        playBtn.innerHTML = '&#10073;&#10073;';
      }
      fitView();
    }

    const status = liveFrameCount ? 'live' : 'waiting for moves';
    const liveSeconds = DATA.durations.reduce((a, b) => a + b, 0);
    headerSub.textContent =
      status + ' · ' +
      DATA.territories.length + ' territories · ' +
      DATA.run_count + ' run' + (DATA.run_count > 1 ? 's' : '') + ' · ' +
      TMM + ' mm territory · ' + GRID + '×' + GRID + ' cells · ' +
      '~' + Math.round(liveSeconds) + ' s at 1×';
  } catch (err) {
    headerSub.textContent = 'server unavailable';
  }
}

// ---- Import a map JSON -----------------------------------------------------
// Posting to /map replaces the dashboard state with the full map render; reload
// afterwards so the page re-reads the imported map's territory size and grid.
const importBtn = document.getElementById('importMap');
const importFile = document.getElementById('importMapFile');
if (importBtn && importFile) {
  importBtn.addEventListener('click', () => importFile.click());
  importFile.addEventListener('change', async () => {
    const file = importFile.files[0];
    importFile.value = '';
    if (!file) return;
    try {
      const text = await file.text();
      const res = await fetch('/map', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: text,
      });
      if (!res.ok) {
        const err = await res.json().catch(() => ({}));
        alert('Import failed: ' + (err.error || res.status));
        return;
      }
      location.reload();
    } catch (err) {
      alert('Import failed: ' + err);
    }
  });
}

// Poll fast enough that a predicted leg and its measured-pose amend are picked
// up close to when the robot actually moves (live-sync, see the mapper).
setInterval(livePoll, 200);
livePoll();
</script>"""


if __name__ == '__main__':
    main()
