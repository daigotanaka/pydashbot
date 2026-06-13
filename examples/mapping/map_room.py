#!/usr/bin/env python3
"""Explore a room through the WebSocket server and build a 2D map.

Starting ritual (run every session for a consistent origin):
  1. Place the robot in a corner, back roughly facing one wall, side roughly
     facing the adjacent wall.
  2. The dock routine will back into the rear wall, then crawl into the side
     wall, establishing (0, 0) at the corner with heading 0° pointing into
     the room.
"""

import argparse
import json
import math
import random
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

from dash.ws_client import send_command

CAL_FILE_PATTERN = 'calibration_????????-??-??-??.json'
MAP_FILE_PATTERN = 'room_map_????????-??-??-??.json'
LEGACY_MAP_FILE = Path('room_map.json')

# --- Tunable parameters ---
PROX_THRESHOLD    = 15
REAR_THRESHOLD    = 20    # rear sensor fires slightly differently
DOCK_SPEED        = 50    # mm/s-equivalent drive speed for docking
FORWARD_DISTANCE_MM = 3000
FORWARD_SPEED_MMPS  = 500  # obstacle-aware move() caps this to a sensor-safe speed
SENSOR_SAFE_SPEED_MMPS = 200
MIN_FORWARD_DISTANCE_MM = 200
BACK_AWAY_MM        = 200
BACK_AWAY_SPEED_MMPS = 100
POLL_INTERVAL     = 0.05
PITCH_TILT_THRESHOLD = 40
DURATION          = 60
WALL_OFFSET_MM    = 150
OBSTACLE_OFFSET_MM = 100
DOCK_CLEARANCE_MM = 80    # back off this far from each wall after contact
INITIAL_HEADING_ANGLES = [0, -30, 30, -60, 60, -90, 90, -120, 120, -150, 150, 180]
REDIRECT_ANGLES = [-45, 45, -60, 60, -90, 90, -120, 120, -150, 150, 180]
STRATEGY_SAMPLE_DISTANCES = [400, 800, 1200, 1600]
BLOCKER_CORRIDOR_MM = 275
ODOMETRY_SIGN_TOLERANCE_MM = 40
ODOMETRY_MAX_MOVE_HEADING_DEG = 45
ODOMETRY_MAX_TURN_DISTANCE_MM = 150

WALL_SOUNDS   = ['ohno', 'ayayay', 'huh', 'confused2', 'confused3']
TILT_SOUNDS   = ['ayayay', 'ohno', 'confused5', 'confused8']
RESUME_SOUNDS = ['okay', 'wee']


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        '--resume',
        nargs='?',
        const='latest',
        metavar='MAP_FILE',
        help=(
            "skip corner docking and append to MAP_FILE from its final saved pose; "
            "without MAP_FILE, use the newest room map"
        ),
    )
    mode.add_argument(
        '--start-with-map',
        nargs='?',
        const='latest',
        metavar='MAP_FILE',
        help=(
            "corner-dock at the normal starting point, then append to MAP_FILE "
            "using its knowledge to choose exploration headings"
        ),
    )
    parser.add_argument(
        '--calibration',
        metavar='CAL_FILE',
        help="override the calibration scales with CAL_FILE",
    )
    parser.add_argument(
        '--output',
        metavar='FILE_PATH',
        help=(
            "write map JSON to FILE_PATH and its image beside it; "
            "otherwise use the selected map or a timestamped filename"
        ),
    )
    return parser.parse_args(args)


def wrap_delta(prev, curr, bits):
    half = 1 << (bits - 1)
    full = 1 << bits
    d = curr - prev
    if d > half:
        d -= full
    elif d < -half:
        d += full
    return d


def angle_delta(target, current):
    """Return the shortest signed turn from current to target heading."""
    return (target - current + 180) % 360 - 180


def normalize_heading(heading):
    """Normalize a heading to [-180, 180)."""
    return (heading + 180) % 360 - 180


def validate_odometry(action, requested, distance_mm, heading_delta):
    """Return reasons an odometry transition is implausible for its command."""
    issues = []
    requested_signed = requested
    requested = abs(requested)
    if action == 'forward':
        if distance_mm < -ODOMETRY_SIGN_TOLERANCE_MM:
            issues.append('forward move measured negative distance')
        if distance_mm > requested * 1.35 + 100:
            issues.append('forward distance exceeded requested distance')
        if abs(heading_delta) > ODOMETRY_MAX_MOVE_HEADING_DEG:
            issues.append('forward move measured excessive heading change')
    elif action == 'reverse':
        if distance_mm > ODOMETRY_SIGN_TOLERANCE_MM:
            issues.append('reverse move measured positive distance')
        if abs(distance_mm) > requested * 1.5 + 100:
            issues.append('reverse distance exceeded requested distance')
        if abs(heading_delta) > ODOMETRY_MAX_MOVE_HEADING_DEG:
            issues.append('reverse move measured excessive heading change')
    elif action == 'turn':
        if abs(distance_mm) > ODOMETRY_MAX_TURN_DISTANCE_MM:
            issues.append('turn measured excessive wheel distance')
        if abs(heading_delta) > requested * 1.5 + 45:
            issues.append('turn measured excessive heading change')
        if requested < 170 and heading_delta * requested_signed < -10:
            issues.append('turn measured the wrong direction')
    return issues


def accepted_runs(data):
    """Return runs whose pose tracking remained trustworthy."""
    return [
        run
        for run in data.get('runs', [])
        if run.get('status', 'accepted') in {'accepted', 'partial'}
    ]


def map_knowledge(data):
    """Extract path and blocker points used by the exploration strategy."""
    runs = accepted_runs(data)
    path_points = [
        (float(point[0]), float(point[1]))
        for run in runs
        for point in run.get('path', [])
    ]
    if data.get('schema_version', 1) >= 2:
        blocker_source = [
            point
            for run in runs
            for point in run.get('walls', []) + run.get('obstacles', [])
        ]
    else:
        blocker_source = data.get('walls', []) + data.get('obstacles', [])
    blockers = [(float(point[0]), float(point[1])) for point in blocker_source]
    return path_points, blockers


def heading_score(
    x,
    y,
    heading,
    turn_angle,
    path_points,
    blockers,
    blocked_left=0,
    blocked_right=0,
):
    """Score a candidate heading for clearance and expected map knowledge."""
    hr = math.radians(heading + turn_angle)
    ux, uy = math.cos(hr), math.sin(hr)
    score = -abs(turn_angle) * 0.35

    for distance in STRATEGY_SAMPLE_DISTANCES:
        sx, sy = x + distance * ux, y + distance * uy
        if path_points:
            nearest = min(math.hypot(sx - px, sy - py) for px, py in path_points)
            score += min(nearest, 800) * 0.35
        else:
            score += 280

    for bx, by in blockers:
        dx, dy = bx - x, by - y
        forward = dx * ux + dy * uy
        lateral = abs(dx * uy - dy * ux)
        if 0 < forward < 2000 and lateral < BLOCKER_CORRIDOR_MM:
            score -= (BLOCKER_CORRIDOR_MM - lateral) * 8
            score -= (2000 - forward) * 1.5

    if blocked_left >= PROX_THRESHOLD and turn_angle > 0:
        score -= 2500
    if blocked_right >= PROX_THRESHOLD and turn_angle < 0:
        score -= 2500
    if blocked_left >= PROX_THRESHOLD and blocked_right >= PROX_THRESHOLD:
        score += abs(turn_angle) * 8
    return score


def choose_exploration_angle(
    x,
    y,
    heading,
    path_points,
    blockers,
    blocked_left=0,
    blocked_right=0,
    require_turn=False,
):
    """Choose the highest-value relative turn using saved and live knowledge."""
    candidates = REDIRECT_ANGLES if require_turn else INITIAL_HEADING_ANGLES
    return max(
        candidates,
        key=lambda turn: heading_score(
            x,
            y,
            heading,
            turn,
            path_points,
            blockers,
            blocked_left,
            blocked_right,
        ),
    )


def forward_distance_for_remaining(remaining_seconds):
    """Choose a forward leg that fits the remaining exploration time."""
    return int(min(
        FORWARD_DISTANCE_MM,
        max(MIN_FORWARD_DISTANCE_MM, remaining_seconds * SENSOR_SAFE_SPEED_MMPS),
    ))


def read_settled(getter, stable=3, tol=2, timeout=2.0, poll=0.05):
    """Return a sensor reading after its post-motion transient settles."""
    history = []
    start = time.time()
    while time.time() - start < timeout:
        history.append(send_command(getter)['result'])
        recent = history[-stable:]
        if len(recent) == stable and max(recent) - min(recent) <= tol:
            return recent[-1]
        time.sleep(poll)
    return Counter(history).most_common(1)[0][0]


def timestamped_path(stem, suffix, now=None):
    """Return a path with a `_YYYYMMDD-HH-MM-SS` timestamp suffix."""
    now = now or datetime.now()
    return Path(f"{stem}_{now.strftime('%Y%m%d-%H-%M-%S')}{suffix}")


def latest_calibration_file(directory=Path('.')):
    """Return the newest timestamped calibration file."""
    files = sorted(directory.glob(CAL_FILE_PATTERN))
    if not files:
        raise FileNotFoundError(
            f"No {CAL_FILE_PATTERN} file found. "
            "Run uv run examples/mapping/calibrate.py first."
        )
    return files[-1]


def latest_map_file(directory=Path('.')):
    """Return the newest timestamped map, falling back to the legacy filename."""
    files = list(directory.glob(MAP_FILE_PATTERN))
    legacy = directory / LEGACY_MAP_FILE
    if legacy.exists():
        files.append(legacy)
    if not files:
        raise FileNotFoundError(
            "No room_map_YYYYMMDD-HH-MM-SS.json or room_map.json file found."
        )
    return max(files, key=lambda path: path.stat().st_mtime)


def load_map_data(map_file):
    """Load a map and validate the calibration needed to extend it."""
    data = json.loads(map_file.read_text())
    calibration = data.get('calibration', {})
    if 'deg_per_yaw' not in calibration or 'mm_per_wd' not in calibration:
        raise ValueError(f"{map_file} does not contain calibration scales")
    return data


def load_resume_state(map_file):
    """Return calibration and final pose from a map that can be resumed."""
    data = load_map_data(map_file)
    calibration = data['calibration']
    runs = accepted_runs(data)
    if not runs or not runs[-1].get('path'):
        raise ValueError(f"{map_file} does not contain a final robot pose")
    x, y, heading = runs[-1]['path'][-1]
    print(
        f"=== Resuming {map_file} from "
        f"({x:.0f}, {y:.0f}) mm, heading {heading:.1f}° ==="
    )
    return (
        calibration['deg_per_yaw'],
        calibration['mm_per_wd'],
        float(x),
        float(y),
        float(heading),
    )


def map_start_pose(data):
    """Return the first pose recorded in a map."""
    runs = accepted_runs(data)
    if not runs or not runs[0].get('path'):
        raise ValueError("map does not contain a starting robot pose")
    x, y, heading = runs[0]['path'][0]
    return float(x), float(y), float(heading)


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------
def load_calibration(cal_file=None):
    cal_file = cal_file or latest_calibration_file()
    cal = json.loads(cal_file.read_text())
    deg_per_yaw = cal['deg_per_yaw'] * cal.get('yaw_sign', 1)
    mm_per_wd   = cal['mm_per_wd']   * cal.get('wd_sign',  1)
    print(f"=== Calibration loaded from {cal_file} (recorded {cal.get('timestamp', 'unknown')}) ===")
    print(f"  deg_per_yaw={deg_per_yaw:.4f}  mm_per_wd={mm_per_wd:.4f}")
    return deg_per_yaw, mm_per_wd


# ---------------------------------------------------------------------------
# Corner dock — establishes (0, 0) as the corner, robot ends up facing room
# ---------------------------------------------------------------------------
def dock_to_corner(deg_per_yaw, mm_per_wd):
    """Back into rear wall, then crawl into left side wall to find corner."""
    print("\n=== Corner Dock ===")
    print("  Place robot near a corner, back toward one wall, left side toward")
    print("  the adjacent wall. Starting in 15 seconds...")
    for i in range(15, 0, -1):
        print(f'  {i}...', end='\r')
        if i == 5:
            send_command('say', 'beep')
        time.sleep(1)
    print()

    # -- Step 1: back into rear wall --
    print("  Backing into rear wall...")
    send_command('drive', -DOCK_SPEED)
    while True:
        rear = send_command('get_prox_rear')['result']
        print(f'    prox_rear={rear}', end='\r')
        if rear >= REAR_THRESHOLD:
            send_command('stop')
            print(f'\n  Rear wall contact (prox_rear={rear})')
            break
        time.sleep(POLL_INTERVAL)

    send_command('say', 'okay')
    time.sleep(0.3)

    # Clear slightly from rear wall
    send_command('move', DOCK_CLEARANCE_MM, 80)
    time.sleep(0.2)

    # -- Step 2: turn left, crawl into side wall --
    print("  Turning left to find side wall...")
    send_command('turn', -90)
    time.sleep(0.2)

    print("  Crawling into side wall...")
    send_command('drive', DOCK_SPEED)
    while True:
        l = send_command('get_prox_left')['result']
        r = send_command('get_prox_right')['result']
        print(f'    prox L={l} R={r}', end='\r')
        if l >= PROX_THRESHOLD or r >= PROX_THRESHOLD:
            send_command('stop')
            print(f'\n  Side wall contact (prox L={l} R={r})')
            break
        time.sleep(POLL_INTERVAL)

    send_command('say', 'okay')
    time.sleep(0.3)

    # Clear slightly from side wall
    send_command('move', -DOCK_CLEARANCE_MM, 80)
    time.sleep(0.2)

    # -- Step 3: turn right to face into room --
    print("  Turning to face room...")
    send_command('turn', 90)
    time.sleep(0.3)

    # Robot is now at approximately (DOCK_CLEARANCE_MM, DOCK_CLEARANCE_MM)
    # relative to the corner, facing into the room (heading 0°).
    x0 = float(DOCK_CLEARANCE_MM)
    y0 = float(DOCK_CLEARANCE_MM)
    print(f"  Docked. Starting position: ({x0:.0f}, {y0:.0f}) mm from corner, heading 0°")
    send_command('neck_color', '#00ffff')
    return x0, y0


# ---------------------------------------------------------------------------
# Exploration
# ---------------------------------------------------------------------------
def explore(deg_per_yaw, mm_per_wd, x0, y0, heading0=0.0, strategy_map=None):
    yaw_prev = send_command('get_yaw')['result']
    wd_prev  = send_command('get_wheel_distance')['result']
    pitch_samples = [send_command('get_pitch')['result'] for _ in range(5)]
    baseline_pitch = sum(pitch_samples) / len(pitch_samples)

    heading = normalize_heading(heading0)
    x, y = x0, y0

    path      = [(x, y, heading)]
    walls     = []
    obstacles = []
    events = []
    quality = {
        'accepted_updates': 0,
        'rejected_updates': 0,
        'tracking_lost': False,
        'issues': [],
    }
    known_path, known_blockers = map_knowledge(strategy_map or {})
    known_path.append((x, y))

    def update_pose(action, requested):
        nonlocal heading, x, y, yaw_prev, wd_prev
        yaw_now = read_settled('get_yaw')
        wd_now = read_settled('get_wheel_distance')
        d_yaw = wrap_delta(yaw_prev, yaw_now, 16)
        d_dist = wrap_delta(wd_prev, wd_now, 20)
        heading_delta = d_yaw * deg_per_yaw
        d_mm = d_dist * mm_per_wd
        yaw_prev = yaw_now
        wd_prev = wd_now
        issues = validate_odometry(action, requested, d_mm, heading_delta)
        event = {
            'action': action,
            'requested': requested,
            'raw_yaw_delta': d_yaw,
            'raw_wheel_delta': d_dist,
            'heading_delta': heading_delta,
            'distance_mm': d_mm,
            'accepted': not issues and not quality['tracking_lost'],
        }
        if issues:
            event['issues'] = issues
            quality['rejected_updates'] += 1
            quality['tracking_lost'] = True
            quality['issues'].extend(issues)
            print(f"\n  [odometry rejected] {'; '.join(issues)}")
        elif quality['tracking_lost']:
            event['issues'] = ['pose tracking was already lost']
        else:
            heading = normalize_heading(heading + heading_delta)
            hr = math.radians(heading)
            x += d_mm * math.cos(hr)
            y += d_mm * math.sin(hr)
            path.append((x, y, heading))
            known_path.append((x, y))
            quality['accepted_updates'] += 1
        events.append(event)
        if not event['accepted']:
            return None
        return d_mm

    def turn_toward_knowledge(reason, left=0, right=0, require_turn=False):
        turn_angle = choose_exploration_angle(
            x,
            y,
            heading,
            known_path,
            known_blockers,
            blocked_left=left,
            blocked_right=right,
            require_turn=require_turn,
        )
        print(f'\n  [{reason}] map-guided turn {turn_angle:+.0f}°')
        if turn_angle:
            send_command('turn', turn_angle)
            update_pose('turn', turn_angle)

    def redirect(reason, sounds=None, back_away=False, left=0, right=0):
        print(f'\n  [{reason}] changing direction')
        send_command('stop')
        send_command('neck_color', '#ff0000')
        if sounds:
            send_command('say', random.choice(sounds))
        if back_away:
            send_command('move', -BACK_AWAY_MM, BACK_AWAY_SPEED_MMPS)
            update_pose('reverse', BACK_AWAY_MM)
        turn_toward_knowledge(reason, left, right, require_turn=True)
        send_command('say', random.choice(RESUME_SOUNDS))
        send_command('neck_color', '#00ff00')

    def mark_ahead(points, offset):
        if quality['tracking_lost']:
            return
        hr = math.radians(heading)
        point = (x + offset * math.cos(hr), y + offset * math.sin(hr))
        points.append(point)
        known_blockers.append(point)

    def report_leg(remaining, traveled, left, right, tilt):
        print(
            f'  [{remaining:4.1f}s] ({x:6.0f},{y:6.0f})mm  '
            f'hdg={heading:6.1f}°  leg={traveled:5.0f}mm  '
            f'prox L={left:2d} R={right:2d}  tilt={tilt:+.0f}',
            end='\r',
        )

    def handle_leg_end(traveled, requested_distance):
        left = send_command('get_prox_left')['result']
        right = send_command('get_prox_right')['result']
        pitch = send_command('get_pitch')['result']
        tilt = pitch - baseline_pitch

        if left >= PROX_THRESHOLD or right >= PROX_THRESHOLD:
            mark_ahead(walls, WALL_OFFSET_MM)
            return left, right, tilt, 'wall', WALL_SOUNDS, True
        if abs(tilt) > PITCH_TILT_THRESHOLD:
            mark_ahead(obstacles, OBSTACLE_OFFSET_MM)
            return left, right, tilt, f'tilt {tilt:+.0f}', TILT_SOUNDS, True
        if abs(traveled) < requested_distance * 0.8:
            mark_ahead(obstacles, OBSTACLE_OFFSET_MM)
            return left, right, tilt, 'early stop', WALL_SOUNDS, True
        return left, right, tilt, 'forward leg complete', None, False

    def stop_safely():
        send_command('stop')
        send_command('say', 'bye')
        send_command('neck_color', '#ffffff')

    print(f"\n=== Exploring for {DURATION}s ===")
    print(
        f"  Repeatedly trying {FORWARD_DISTANCE_MM}mm forward legs; "
        "walls and tilt stop each leg early."
    )
    send_command('say', 'hi')
    send_command('neck_color', '#00ff00')
    turn_toward_knowledge('initial strategy')

    end_time = time.time() + DURATION

    try:
        while time.time() < end_time and not quality['tracking_lost']:
            remaining = end_time - time.time()
            requested_distance = forward_distance_for_remaining(remaining)
            send_command(
                'move',
                requested_distance,
                FORWARD_SPEED_MMPS,
                wall_stop_sound=None,
            )
            traveled = update_pose('forward', requested_distance)
            remaining = max(0.0, end_time - time.time())
            if traveled is None:
                left = send_command('get_prox_left')['result']
                right = send_command('get_prox_right')['result']
                tilt = send_command('get_pitch')['result'] - baseline_pitch
                reason, sounds, back_away = 'odometry rejected', None, False
                traveled = 0.0
                report_leg(remaining, traveled, left, right, tilt)
                break
            else:
                left, right, tilt, reason, sounds, back_away = handle_leg_end(
                    traveled, requested_distance
                )
            report_leg(remaining, traveled, left, right, tilt)

            if time.time() < end_time:
                redirect(reason, sounds, back_away, left, right)

    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        stop_safely()

    print(f'\nDone. Path={len(path)} pts  Walls={len(walls)}  Obstacles={len(obstacles)}')
    return {
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        'status': 'partial' if quality['tracking_lost'] else 'accepted',
        'quality': quality,
        'path': path,
        'walls': walls,
        'obstacles': obstacles,
        'events': events,
    }


# ---------------------------------------------------------------------------
# Persist
# ---------------------------------------------------------------------------
def save_map(
    deg_per_yaw,
    mm_per_wd,
    run,
    map_file,
    base_data=None,
    replace_existing=False,
):
    if map_file.exists() and not replace_existing:
        existing = json.loads(map_file.read_text())
    elif base_data:
        existing = base_data
    else:
        existing = {}

    if existing:
        all_runs = existing.get('runs', [])
        if existing.get('schema_version', 1) < 2 and all_runs:
            all_runs[0].setdefault('walls', existing.get('walls', []))
            all_runs[0].setdefault('obstacles', existing.get('obstacles', []))
    else:
        all_runs = []

    all_runs.append(run)
    accepted = [
        item
        for item in all_runs
        if item.get('status', 'accepted') in {'accepted', 'partial'}
    ]
    all_walls = [point for item in accepted for point in item.get('walls', [])]
    all_obstacles = [
        point for item in accepted for point in item.get('obstacles', [])
    ]

    data = {
        'schema_version': 2,
        'calibration': {'deg_per_yaw': deg_per_yaw, 'mm_per_wd': mm_per_wd},
        'runs':        all_runs,
        'walls':       all_walls,
        'obstacles':   all_obstacles,
    }
    map_file.parent.mkdir(parents=True, exist_ok=True)
    temp_file = map_file.with_suffix(map_file.suffix + '.tmp')
    temp_file.write_text(json.dumps(data, indent=2))
    temp_file.replace(map_file)
    print(f'Map saved → {map_file}  '
          f'(run #{len(all_runs)} {run["status"]}, '
          f'{len(all_walls)} total wall pts, '
          f'{len(all_obstacles)} total obstacle pts)')
    return all_runs, all_walls, all_obstacles


# ---------------------------------------------------------------------------
# Visualise
# ---------------------------------------------------------------------------
def visualise(all_runs, all_walls, all_obstacles, img_path):
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    COLORS = ['#4477cc', '#44aa77', '#cc7744', '#aa44aa', '#cc4444']

    fig, ax = plt.subplots(figsize=(11, 10))

    # Corner origin marker
    ax.plot(0, 0, 'k+', markersize=16, markeredgewidth=2, zorder=8, label='Corner (0,0)')

    for i, run in enumerate(all_runs):
        if run.get('status', 'accepted') not in {'accepted', 'partial'}:
            continue
        rpath = run['path']
        if not rpath:
            continue
        color = COLORS[i % len(COLORS)]
        px = [p[0] for p in rpath]
        py = [p[1] for p in rpath]
        label = f'Run {i+1} ({run["timestamp"][:10]})'
        ax.plot(px, py, '-', color=color, alpha=0.4, linewidth=1, label=label)
        ax.plot(px[0],  py[0],  'o', color=color, markersize=8,  zorder=6)
        ax.plot(px[-1], py[-1], 's', color=color, markersize=7,  zorder=6)
        step = max(1, len(rpath) // 15)
        for j in range(0, len(rpath) - step, step):
            dx = px[j+step] - px[j]
            dy = py[j+step] - py[j]
            if math.hypot(dx, dy) > 1:
                ax.annotate('', xy=(px[j+step], py[j+step]), xytext=(px[j], py[j]),
                            arrowprops=dict(arrowstyle='->', color=color, lw=1.0))

    if all_walls:
        wx = [w[0] for w in all_walls]
        wy = [w[1] for w in all_walls]
        ax.scatter(wx, wy, c='red', s=80, marker='x', linewidths=2,
                   label=f'Wall ({len(all_walls)} pts)', zorder=7)

    if all_obstacles:
        ox = [o[0] for o in all_obstacles]
        oy = [o[1] for o in all_obstacles]
        ax.scatter(ox, oy, c='orange', s=80, marker='^',
                   label=f'Obstacle ({len(all_obstacles)} pts)', zorder=7)

    # Draw the two dock walls as reference lines
    accepted = [
        run
        for run in all_runs
        if run.get('status', 'accepted') in {'accepted', 'partial'}
    ]
    all_x = [p[0] for run in accepted for p in run['path']] + [w[0] for w in all_walls] + [0]
    all_y = [p[1] for run in accepted for p in run['path']] + [w[1] for w in all_walls] + [0]
    max_x = max(all_x) * 1.05 if all_x else 1000
    max_y = max(all_y) * 1.05 if all_y else 1000
    ax.plot([0, max_x], [0, 0],      'k-', linewidth=2, alpha=0.5, label='Dock walls')
    ax.plot([0, 0],     [0, max_y],  'k-', linewidth=2, alpha=0.5)

    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)
    ax.set_title(
        f'Room Map — {len(accepted)} accepted / {len(all_runs)} total run(s)',
        fontsize=14,
    )
    ax.set_xlabel('x (mm)')
    ax.set_ylabel('y (mm)')
    plt.tight_layout()

    plt.savefig(img_path, dpi=150, bbox_inches='tight')
    print(f'Map image saved → {img_path}')
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(args=None):
    options = parse_args(args)
    run_started = datetime.now()
    send_command('stop')
    time.sleep(1.0)
    calibration_override = (
        load_calibration(Path(options.calibration))
        if options.calibration
        else None
    )

    strategy_map = {}
    source_map_file = None
    if options.resume:
        source_map_file = (
            latest_map_file()
            if options.resume == 'latest'
            else Path(options.resume)
        )
        strategy_map = load_map_data(source_map_file)
        deg_per_yaw, mm_per_wd, x0, y0, heading0 = load_resume_state(source_map_file)
        if calibration_override:
            deg_per_yaw, mm_per_wd = calibration_override
    elif options.start_with_map:
        source_map_file = (
            latest_map_file()
            if options.start_with_map == 'latest'
            else Path(options.start_with_map)
        )
        strategy_map = load_map_data(source_map_file)
        calibration = strategy_map['calibration']
        deg_per_yaw = calibration['deg_per_yaw']
        mm_per_wd = calibration['mm_per_wd']
        if calibration_override:
            deg_per_yaw, mm_per_wd = calibration_override
        print(f"=== Starting from dock with knowledge from {source_map_file} ===")
        dock_to_corner(deg_per_yaw, mm_per_wd)
        x0, y0, heading0 = map_start_pose(strategy_map)
        print(
            f"  Anchored to saved starting pose: "
            f"({x0:.0f}, {y0:.0f}) mm, heading {heading0:.1f}°"
        )
    else:
        deg_per_yaw, mm_per_wd = calibration_override or load_calibration()
        x0, y0 = dock_to_corner(deg_per_yaw, mm_per_wd)
        heading0 = 0.0

    if options.output:
        map_file = Path(options.output)
    elif source_map_file:
        map_file = source_map_file
    else:
        map_file = timestamped_path('room_map', '.json', run_started)
    img_path = map_file.with_suffix('.png')

    run = explore(
        deg_per_yaw, mm_per_wd, x0, y0, heading0, strategy_map
    )
    all_runs, all_walls, all_obstacles = save_map(
        deg_per_yaw,
        mm_per_wd,
        run,
        map_file,
        base_data=strategy_map if source_map_file != map_file else None,
        replace_existing=source_map_file is None,
    )
    visualise(all_runs, all_walls, all_obstacles, img_path)


if __name__ == "__main__":
    try:
        main()
    finally:
        try:
            send_command("stop")
        except Exception:
            pass
