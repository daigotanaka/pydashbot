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
import heapq
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
LOOP_CLOSURE_MATCH_RADIUS_MM = 350
LOOP_CLOSURE_PATH_RADIUS_MM = 700
LOOP_CLOSURE_MAX_CORRECTION_MM = 250
LOOP_CLOSURE_GAIN = 0.6
HOME_ROUTE_LINK_RADIUS_MM = 250
HOME_ROUTE_COLLINEAR_DEG = 12
HOME_MAX_LEG_MM = 1000
HOME_POSITION_TOLERANCE_MM = 100
BLOCKED_EDGE_TOLERANCE_MM = 150
HOME_WALL_CLEARANCE_MM = 120        # short nudge to step off a wall just turned from
HOME_CLEARANCE_SPEED_MMPS = 80
HOME_CLEARANCE_MIN_TURN_DEG = 120   # only a near-reversal turn faces a wall left behind
# Relaxed obstacle criteria while retracing a proven corridor home. A higher
# threshold and a longer confirmation streak let Dash graze walls it already
# drove past, while a solid head-on wall (which reads far higher) still stops it.
HOME_RETRACE_PROX_THRESHOLD = 22
HOME_RETRACE_CONFIRM_COUNT = 6
MAX_TRACKED_TURN_DEG = 90
MIN_TRACKED_TURN_DEG = 20
HOME_CLEAR_RETRY_DELAY = 0.3

WALL_SOUNDS   = ['ohno', 'ayayay', 'huh', 'confused2', 'confused3']
TILT_SOUNDS   = ['ayayay', 'ohno', 'confused5', 'confused8']
RESUME_SOUNDS = ['okay', 'wee']


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        '--go-home',
        nargs='?',
        const='latest',
        metavar='MAP_FILE',
        help=(
            "return from MAP_FILE's final saved pose to its initial pose and "
            "orientation; without MAP_FILE, use the newest room map"
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
    parser.add_argument(
        '--duration',
        type=positive_seconds,
        default=DURATION,
        metavar='SECONDS',
        help=f"exploration run time in seconds (default: {DURATION})",
    )
    return parser.parse_args(args)


def positive_seconds(value):
    seconds = float(value)
    if seconds <= 0:
        raise argparse.ArgumentTypeError("duration must be greater than zero")
    return seconds


def wrap_delta(prev, curr, bits):
    half = 1 << (bits - 1)
    full = 1 << bits
    return (curr - prev + half) % full - half


def wheel_translation_delta(left_prev, left_now, right_prev, right_now):
    """Return translation ticks while canceling opposite wheel motion in turns."""
    return (
        wrap_delta(left_prev, left_now, 16)
        + wrap_delta(right_prev, right_now, 16)
    ) / 2


def angle_delta(target, current):
    """Return the shortest signed turn from current to target heading."""
    return (target - current + 180) % 360 - 180


def normalize_heading(heading):
    """Normalize a heading to [-180, 180)."""
    return (heading + 180) % 360 - 180


def tracked_turn_steps(degrees):
    """Split a turn so each yaw delta remains unambiguous."""
    count = max(1, math.ceil(abs(degrees) / MAX_TRACKED_TURN_DEG))
    return [degrees / count] * count


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


def run_pose_trustworthy(run):
    """Return whether a run's final saved pose is safe to navigate from."""
    quality = run.get('quality', {})
    if not quality.get('tracking_lost', False):
        return True
    return (
        run.get('mode') == 'go_home'
        and quality.get('rejected_updates', 0) == 0
        and quality.get('issues') == ['go-home leg stopped early']
    )


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


def revisit_pose_correction(
    x,
    y,
    observed_point,
    path_points,
    landmarks,
):
    """Return a bounded XY correction when an observation closes a known loop."""
    if not path_points or not landmarks:
        return None
    if min(math.hypot(x - px, y - py) for px, py in path_points) > LOOP_CLOSURE_PATH_RADIUS_MM:
        return None

    ox, oy = observed_point
    target = min(landmarks, key=lambda point: math.hypot(ox - point[0], oy - point[1]))
    dx, dy = target[0] - ox, target[1] - oy
    distance = math.hypot(dx, dy)
    if distance == 0 or distance > LOOP_CLOSURE_MATCH_RADIUS_MM:
        return None

    correction_scale = min(
        LOOP_CLOSURE_GAIN,
        LOOP_CLOSURE_MAX_CORRECTION_MM / distance,
    )
    return dx * correction_scale, dy * correction_scale, target, distance


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


def home_leg_distance(distance_mm):
    """Return an integer distance accepted by Dash's move packet encoder."""
    return max(1, int(round(min(HOME_MAX_LEG_MM, distance_mm))))


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
    if 'deg_per_yaw' not in calibration or not (
        'mm_per_wheel_tick' in calibration or 'mm_per_wd' in calibration
    ):
        raise ValueError(f"{map_file} does not contain calibration scales")
    return data


def load_resume_state(map_file):
    """Return calibration and the map's final saved pose."""
    data = load_map_data(map_file)
    calibration = data['calibration']
    runs = accepted_runs(data)
    if not runs or not runs[-1].get('path'):
        raise ValueError(f"{map_file} does not contain a final robot pose")
    x, y, heading = runs[-1]['path'][-1]
    print(
        f"=== Using final pose from {map_file}: "
        f"({x:.0f}, {y:.0f}) mm, heading {heading:.1f}° ==="
    )
    return (
        calibration['deg_per_yaw'],
        calibration.get('mm_per_wheel_tick', calibration.get('mm_per_wd')),
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


def simplify_home_route(route):
    """Remove duplicate and nearly straight waypoints without creating shortcuts."""
    deduped = []
    for point in route:
        point = (float(point[0]), float(point[1]))
        if not deduped or math.dist(point, deduped[-1]) > 1:
            deduped.append(point)

    simplified = []
    for point in deduped:
        simplified.append(point)
        while len(simplified) >= 3:
            a, b, c = simplified[-3:]
            ab = math.atan2(b[1] - a[1], b[0] - a[0])
            bc = math.atan2(c[1] - b[1], c[0] - b[0])
            turn = (bc - ab + math.pi) % (2 * math.pi) - math.pi
            turn_degrees = abs(math.degrees(turn))
            if (
                turn_degrees > HOME_ROUTE_COLLINEAR_DEG
                and turn_degrees < 180 - HOME_ROUTE_COLLINEAR_DEG
            ):
                break
            simplified.pop(-2)
    return simplified


def collect_blocked_edges(data):
    """Return blocked-route segments recorded by prior aborted go-home runs.

    Each segment is ((from_x, from_y), (to_x, to_y)) for a proven path leg that
    a go-home attempt found physically obstructed.
    """
    edges = []
    for run in data.get('runs', []):
        for edge in run.get('blocked_edges', []):
            if 'from' in edge and 'to' in edge:
                edges.append(
                    (
                        (float(edge['from'][0]), float(edge['from'][1])),
                        (float(edge['to'][0]), float(edge['to'][1])),
                    )
                )
    return edges


def _point_near_segment(point, seg_start, seg_end, tolerance):
    """Return whether point lies within tolerance of segment seg_start-seg_end."""
    px, py = point
    ax, ay = seg_start
    bx, by = seg_end
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0:
        return math.hypot(px - ax, py - ay) <= tolerance
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy) <= tolerance


def edge_is_blocked(
    first_point,
    second_point,
    blocked_edges,
    tolerance=BLOCKED_EDGE_TOLERANCE_MM,
):
    """Return whether a graph edge runs along a recorded blocked segment.

    The edge must overlap the blocked corridor (its midpoint falls near the
    segment) and run roughly parallel to it. A perpendicular crossing into a
    different corridor stays usable, and a degenerate coincident link is kept so
    the proximity graph remains connected at the blocked corridor's endpoints.
    """
    ex, ey = second_point[0] - first_point[0], second_point[1] - first_point[1]
    edge_len = math.hypot(ex, ey)
    if edge_len < 1:
        return False
    midpoint = (
        (first_point[0] + second_point[0]) / 2,
        (first_point[1] + second_point[1]) / 2,
    )
    for seg_start, seg_end in blocked_edges:
        if not _point_near_segment(midpoint, seg_start, seg_end, tolerance):
            continue
        sx, sy = seg_end[0] - seg_start[0], seg_end[1] - seg_start[1]
        seg_len = math.hypot(sx, sy)
        if seg_len < 1:
            return True
        alignment = abs(ex * sx + ey * sy) / (edge_len * seg_len)
        if alignment >= math.cos(math.radians(30)):
            return True
    return False


def plan_home_route(data):
    """Return the shortest route home along previously traversed path segments."""
    latest_run = data.get('runs', [])[-1] if data.get('runs') else None
    if not latest_run or latest_run.get('status', 'accepted') not in {
        'accepted',
        'partial',
    }:
        raise ValueError("go-home requires an accepted or safely aborted latest run")
    if not run_pose_trustworthy(latest_run):
        raise ValueError("go-home requires a trustworthy final saved pose")

    runs = [run for run in accepted_runs(data) if run.get('path')]
    if not runs:
        raise ValueError("map does not contain an accepted path home")

    blocked_edges = collect_blocked_edges(data)

    nodes = []
    adjacency = {}
    point_lookup = {}

    def link(first, second, distance):
        if edge_is_blocked(point_lookup[first], point_lookup[second], blocked_edges):
            return
        adjacency[first].append((second, distance))
        adjacency[second].append((first, distance))

    for run_index, run in enumerate(runs):
        run_nodes = []
        for point_index, point in enumerate(run['path']):
            node = (run_index, point_index)
            position = (float(point[0]), float(point[1]))
            nodes.append((node, position))
            point_lookup[node] = position
            adjacency[node] = []
            run_nodes.append(node)
        for first, second in zip(run_nodes, run_nodes[1:]):
            link(first, second, math.dist(point_lookup[first], point_lookup[second]))

    for index, (first, first_point) in enumerate(nodes):
        for second, second_point in nodes[index + 1:]:
            if first[0] == second[0] and abs(first[1] - second[1]) <= 1:
                continue
            distance = math.dist(first_point, second_point)
            if distance <= HOME_ROUTE_LINK_RADIUS_MM:
                link(first, second, distance)

    start = (len(runs) - 1, len(runs[-1]['path']) - 1)
    goal = (0, 0)
    distances = {start: 0.0}
    previous = {}
    queue = [(0.0, start)]
    while queue:
        distance, node = heapq.heappop(queue)
        if node == goal:
            break
        if distance != distances.get(node):
            continue
        for neighbor, edge_distance in adjacency[node]:
            candidate = distance + edge_distance
            if candidate < distances.get(neighbor, math.inf):
                distances[neighbor] = candidate
                previous[neighbor] = node
                heapq.heappush(queue, (candidate, neighbor))

    if goal not in distances:
        if blocked_edges:
            raise ValueError(
                "no unblocked proven route home remains; "
                f"{len(blocked_edges)} known route segment(s) are blocked"
            )
        raise ValueError("accepted map paths do not connect back to the starting pose")

    route_nodes = [goal]
    while route_nodes[-1] != start:
        route_nodes.append(previous[route_nodes[-1]])
    route_nodes.reverse()
    return simplify_home_route([point_lookup[node] for node in route_nodes])


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------
def load_calibration(cal_file=None):
    cal_file = cal_file or latest_calibration_file()
    cal = json.loads(cal_file.read_text())
    deg_per_yaw = cal['deg_per_yaw'] * cal.get('yaw_sign', 1)
    mm_per_wd = cal.get('mm_per_wheel_tick', cal.get('mm_per_wd'))
    mm_per_wd *= cal.get('wd_sign', 1)
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
    send_command('turn', 90)
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
    send_command('turn', -90)
    time.sleep(0.3)

    # Robot is now at approximately (DOCK_CLEARANCE_MM, DOCK_CLEARANCE_MM)
    # relative to the corner, facing into the room (heading 0°).
    x0 = float(DOCK_CLEARANCE_MM)
    y0 = float(DOCK_CLEARANCE_MM)
    print(f"  Docked. Starting position: ({x0:.0f}, {y0:.0f}) mm from corner, heading 0°")
    send_command('neck_color', '#00ffff')
    return x0, y0


def describe_halt(outcome, deg_per_yaw=None):
    """Render the motion layer's halt outcome as a human-readable reason.

    The motion layer returns a dict describing why a move or turn stopped (an
    obstacle with its sensor readings, a tilt, a mechanical stall, or a gyro
    that registered no rotation). This turns that into a one-line explanation,
    converting raw yaw counts to degrees when a calibration scale is supplied.
    """
    if not isinstance(outcome, dict):
        return 'reason not reported'
    halt = outcome.get('halt')
    if halt == 'obstacle' and outcome.get('side') == 'front':
        return (
            f"obstacle ahead (prox L={outcome.get('prox_left')} "
            f"R={outcome.get('prox_right')}, threshold {PROX_THRESHOLD})"
        )
    if halt == 'obstacle' and outcome.get('side') == 'rear':
        return f"obstacle behind (prox_rear={outcome.get('prox_rear')})"
    if halt == 'tilt':
        return f"tilt detected (pitch change {outcome.get('pitch_delta')})"
    if halt in ('stalled', 'no_yaw_response', 'executed'):
        yaw = outcome.get('yaw_delta')
        measured = (
            f", ~{yaw * deg_per_yaw:.0f}° measured"
            if deg_per_yaw and yaw is not None
            else ""
        )
        wheels = (
            f"wheels L={outcome.get('left_wheel_delta')} "
            f"R={outcome.get('right_wheel_delta')} ticks, yaw={yaw} counts{measured}"
        )
        commanded = outcome.get('commanded_deg')
        if halt == 'stalled':
            return f"wheels did not move — mechanical stall (commanded {commanded}°; {wheels})"
        if halt == 'no_yaw_response':
            return (
                f"wheels moved but gyro registered no rotation "
                f"(commanded {commanded}°; {wheels})"
            )
        return f"turn under target (commanded {commanded}°; {wheels})"
    if halt == 'completed':
        return 'completed full distance'
    if halt == 'invalid':
        return f"invalid command ({outcome.get('commanded_deg')}°)"
    return str(outcome)


def needs_wall_clearance(leg_turn_deg, prox_left, prox_right, threshold=PROX_THRESHOLD):
    """Whether a leg should begin with a clearance nudge off a wall just left.

    A near-reversal turn at the start of a go-home leg puts a wall that sat
    behind Dash during outbound travel directly ahead, where the forward
    proximity sensors read it even though Dash just traversed that space. In that
    case a short, bounded forward nudge lets Dash step off the known wall before
    normal obstacle stopping resumes. A smaller turn, or a clear front, does not
    qualify, so genuine head-on obstacles still stop the robot.
    """
    front = max(prox_left or 0, prox_right or 0)
    return abs(leg_turn_deg) >= HOME_CLEARANCE_MIN_TURN_DEG and front >= threshold


def go_home(data, deg_per_yaw, mm_per_wd):
    """Follow the shortest known-safe route back to the map's initial pose."""
    route = plan_home_route(data)
    start_x, start_y, start_heading = map_start_pose(data)
    current_x, current_y, current_heading = accepted_runs(data)[-1]['path'][-1]
    x, y = float(current_x), float(current_y)
    heading = normalize_heading(float(current_heading))
    yaw_prev = send_command('get_yaw')['result']
    left_prev = send_command('get_left_wheel')['result']
    right_prev = send_command('get_right_wheel')['result']
    path = [(x, y, heading)]
    events = []
    issues = []
    blocked_edges = []
    halt_reason = None
    odometry_rejected = False

    def update_pose(action, requested):
        nonlocal x, y, heading, yaw_prev, left_prev, right_prev, odometry_rejected
        yaw_now = read_settled('get_yaw')
        left_now = read_settled('get_left_wheel')
        right_now = read_settled('get_right_wheel')
        d_yaw = wrap_delta(yaw_prev, yaw_now, 12)
        left_delta = wrap_delta(left_prev, left_now, 16)
        right_delta = wrap_delta(right_prev, right_now, 16)
        distance_mm = ((left_delta + right_delta) / 2) * mm_per_wd
        heading_delta = d_yaw * deg_per_yaw
        event_issues = validate_odometry(
            action,
            requested,
            distance_mm,
            heading_delta,
        )
        yaw_prev, left_prev, right_prev = yaw_now, left_now, right_now
        event = {
            'action': action,
            'requested': requested,
            'raw_yaw_delta': d_yaw,
            'raw_left_wheel_delta': left_delta,
            'raw_right_wheel_delta': right_delta,
            'heading_delta': heading_delta,
            'distance_mm': distance_mm,
            'accepted': not event_issues,
        }
        if event_issues:
            event['issues'] = event_issues
            issues.extend(event_issues)
            odometry_rejected = True
        else:
            heading = normalize_heading(heading + heading_delta)
            hr = math.radians(heading)
            x += distance_mm * math.cos(hr)
            y += distance_mm * math.sin(hr)
            path.append((x, y, heading))
        events.append(event)
        return None if event_issues else distance_mm

    def turn_to(target_heading):
        nonlocal halt_reason
        turn = angle_delta(target_heading, heading)
        if abs(turn) < MIN_TRACKED_TURN_DEG:
            return True
        for step in tracked_turn_steps(turn):
            outcome = send_command('turn', step).get('result')
            previous_heading = heading
            if update_pose('turn', step) is None:
                return False
            if abs(angle_delta(heading, previous_heading)) < abs(step) * 0.5:
                issues.append('go-home turn did not execute')
                halt_reason = outcome
                print(f"\n  Turn did not execute — {describe_halt(outcome, deg_per_yaw)}")
                return False
        return True

    print("\n=== Going home ===")
    print(
        f"  Planned {sum(math.dist(a, b) for a, b in zip(route, route[1:])):.0f}mm "
        f"along {len(route)} proven-route waypoints."
    )
    send_command('say', 'okay')
    send_command('neck_color', '#00ffff')
    completed = True

    try:
        prev_waypoint = route[0]
        for waypoint_x, waypoint_y in route[1:]:
            leg_first_move = True
            while math.hypot(waypoint_x - x, waypoint_y - y) > HOME_POSITION_TOLERANCE_MM:
                target_heading = math.degrees(math.atan2(waypoint_y - y, waypoint_x - x))
                intended_turn = angle_delta(target_heading, heading)
                if not turn_to(target_heading):
                    completed = False
                    break
                if leg_first_move:
                    leg_first_move = False
                    prox_left = send_command('get_prox_left')['result']
                    prox_right = send_command('get_prox_right')['result']
                    if needs_wall_clearance(intended_turn, prox_left, prox_right):
                        clearance = min(
                            HOME_WALL_CLEARANCE_MM,
                            home_leg_distance(
                                math.hypot(waypoint_x - x, waypoint_y - y)
                            ),
                        )
                        print(
                            f"\n  Stepping off wall just turned from "
                            f"(prox L={prox_left} R={prox_right}); "
                            f"nudging {clearance:.0f}mm with detection off"
                        )
                        clearance_response = send_command(
                            'move',
                            clearance,
                            HOME_CLEARANCE_SPEED_MMPS,
                            stop_at_obstacle=False,
                            wall_stop_sound=None,
                        )
                        if clearance_response.get('ok', True) is not False:
                            update_pose('forward', clearance)
                        continue
                requested = home_leg_distance(
                    math.hypot(waypoint_x - x, waypoint_y - y)
                )
                response = send_command(
                    'move',
                    requested,
                    FORWARD_SPEED_MMPS,
                    wall_stop_sound=None,
                    proximity_threshold=HOME_RETRACE_PROX_THRESHOLD,
                    proximity_confirm_count=HOME_RETRACE_CONFIRM_COUNT,
                )
                if response.get('ok', True) is False:
                    issues.append(f"go-home move command failed: {response['error']}")
                    completed = False
                    break
                move_outcome = response.get('result')
                traveled = update_pose('forward', requested)
                if traveled is not None and abs(traveled) < 1:
                    time.sleep(HOME_CLEAR_RETRY_DELAY)
                    left = send_command('get_prox_left')['result']
                    right = send_command('get_prox_right')['result']
                    if left < HOME_RETRACE_PROX_THRESHOLD and right < HOME_RETRACE_PROX_THRESHOLD:
                        response = send_command(
                            'move',
                            requested,
                            FORWARD_SPEED_MMPS,
                            wall_stop_sound=None,
                            proximity_threshold=HOME_RETRACE_PROX_THRESHOLD,
                            proximity_confirm_count=HOME_RETRACE_CONFIRM_COUNT,
                        )
                        if response.get('ok', True) is False:
                            issues.append(
                                f"go-home move command failed: {response['error']}"
                            )
                            completed = False
                            break
                        move_outcome = response.get('result')
                        traveled = update_pose('forward', requested)
                if traveled is None or traveled < requested * 0.8:
                    issues.append('go-home leg stopped early')
                    halt_reason = move_outcome
                    print(
                        f"\n  Leg stopped early — {describe_halt(move_outcome, deg_per_yaw)}"
                    )
                    if traveled is not None:
                        blocked_edges.append({
                            'from': [float(prev_waypoint[0]), float(prev_waypoint[1])],
                            'to': [float(waypoint_x), float(waypoint_y)],
                            'stop': [round(x, 1), round(y, 1)],
                            'reason': move_outcome,
                            'timestamp': datetime.now().isoformat(timespec='seconds'),
                        })
                    completed = False
                    break
                print(
                    f"  home distance={math.hypot(start_x - x, start_y - y):.0f}mm  "
                    f"pose=({x:.0f},{y:.0f}) heading={heading:.1f}°",
                    end='\r',
                )
            if not completed:
                break
            prev_waypoint = (waypoint_x, waypoint_y)

        if completed:
            completed = turn_to(start_heading)
        if completed and math.hypot(start_x - x, start_y - y) > HOME_POSITION_TOLERANCE_MM:
            issues.append('final pose remained outside home tolerance')
            completed = False
    except KeyboardInterrupt:
        issues.append('go-home interrupted')
        completed = False
        print('\nInterrupted.')
    finally:
        send_command('stop')
        send_command('say', 'bye' if completed else 'ohno')
        send_command('neck_color', '#ffffff')

    print(
        f"\nGo-home {'complete' if completed else 'aborted'} at "
        f"({x:.0f}, {y:.0f}) mm, heading {heading:.1f}°"
    )
    if not completed and halt_reason is not None:
        print(f"  Halt reason: {describe_halt(halt_reason, deg_per_yaw)}")
    return {
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        'mode': 'go_home',
        'status': 'accepted' if completed else 'partial',
        'quality': {
            'accepted_updates': sum(event['accepted'] for event in events),
            'rejected_updates': sum(not event['accepted'] for event in events),
            'tracking_lost': odometry_rejected,
            'issues': issues,
            'halt_reason': halt_reason,
        },
        'planned_route': route,
        'path': path,
        'walls': [],
        'obstacles': [],
        'blocked_edges': blocked_edges,
        'events': events,
    }


# ---------------------------------------------------------------------------
# Exploration
# ---------------------------------------------------------------------------
def explore(
    deg_per_yaw,
    mm_per_wd,
    x0,
    y0,
    heading0=0.0,
    strategy_map=None,
    duration=DURATION,
):
    yaw_prev = send_command('get_yaw')['result']
    left_prev = send_command('get_left_wheel')['result']
    right_prev = send_command('get_right_wheel')['result']
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
        'loop_closures': 0,
        'loop_closure_correction_mm': 0.0,
        'tracking_lost': False,
        'issues': [],
    }
    known_path, known_blockers = map_knowledge(strategy_map or {})
    known_walls = [
        (float(point[0]), float(point[1]))
        for run in accepted_runs(strategy_map or {})
        for point in run.get('walls', [])
    ]
    known_obstacles = [
        (float(point[0]), float(point[1]))
        for run in accepted_runs(strategy_map or {})
        for point in run.get('obstacles', [])
    ]
    known_path.append((x, y))

    def update_pose(action, requested):
        nonlocal heading, x, y, yaw_prev, left_prev, right_prev
        yaw_now = read_settled('get_yaw')
        left_now = read_settled('get_left_wheel')
        right_now = read_settled('get_right_wheel')
        d_yaw = wrap_delta(yaw_prev, yaw_now, 12)
        left_delta = wrap_delta(left_prev, left_now, 16)
        right_delta = wrap_delta(right_prev, right_now, 16)
        d_dist = (left_delta + right_delta) / 2
        heading_delta = d_yaw * deg_per_yaw
        d_mm = d_dist * mm_per_wd
        yaw_prev = yaw_now
        left_prev = left_now
        right_prev = right_now
        issues = validate_odometry(action, requested, d_mm, heading_delta)
        event = {
            'action': action,
            'requested': requested,
            'raw_yaw_delta': d_yaw,
            'raw_left_wheel_delta': left_delta,
            'raw_right_wheel_delta': right_delta,
            'translation_wheel_delta': d_dist,
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
        for step in tracked_turn_steps(turn_angle):
            if not step:
                continue
            send_command('turn', step)
            if update_pose('turn', step) is None:
                break

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

    def mark_ahead(points, landmarks, offset):
        nonlocal x, y
        if quality['tracking_lost']:
            return
        hr = math.radians(heading)
        point = (x + offset * math.cos(hr), y + offset * math.sin(hr))
        correction = revisit_pose_correction(
            x,
            y,
            point,
            known_path[:-1],
            landmarks,
        )
        if correction:
            dx, dy, target, mismatch = correction
            x += dx
            y += dy
            path[-1] = (x, y, heading)
            known_path[-1] = (x, y)
            point = (x + offset * math.cos(hr), y + offset * math.sin(hr))
            correction_mm = math.hypot(dx, dy)
            quality['loop_closures'] += 1
            quality['loop_closure_correction_mm'] += correction_mm
            events[-1]['loop_closure'] = {
                'matched_landmark': target,
                'observation_mismatch_mm': mismatch,
                'pose_correction': (dx, dy),
            }
            print(
                f'\n  [revisit] corrected pose by {correction_mm:.0f}mm '
                f'toward known landmark'
            )
        points.append(point)
        landmarks.append(point)
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
            mark_ahead(walls, known_walls, WALL_OFFSET_MM)
            return left, right, tilt, 'wall', WALL_SOUNDS, True
        if abs(tilt) > PITCH_TILT_THRESHOLD:
            mark_ahead(obstacles, known_obstacles, OBSTACLE_OFFSET_MM)
            return left, right, tilt, f'tilt {tilt:+.0f}', TILT_SOUNDS, True
        if abs(traveled) < requested_distance * 0.8:
            mark_ahead(obstacles, known_obstacles, OBSTACLE_OFFSET_MM)
            return left, right, tilt, 'early stop', WALL_SOUNDS, True
        return left, right, tilt, 'forward leg complete', None, False

    def stop_safely():
        send_command('stop')
        send_command('say', 'bye')
        send_command('neck_color', '#ffffff')

    print(f"\n=== Exploring for {duration:g}s ===")
    print(
        f"  Repeatedly trying {FORWARD_DISTANCE_MM}mm forward legs; "
        "walls and tilt stop each leg early."
    )
    send_command('say', 'hi')
    send_command('neck_color', '#00ff00')
    turn_toward_knowledge('initial strategy')

    end_time = time.time() + duration

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
        'duration_seconds': duration,
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
        'calibration': {
            'deg_per_yaw': deg_per_yaw,
            'mm_per_wheel_tick': mm_per_wd,
        },
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
    if options.go_home:
        source_map_file = (
            latest_map_file()
            if options.go_home == 'latest'
            else Path(options.go_home)
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
        mm_per_wd = calibration.get(
            'mm_per_wheel_tick', calibration.get('mm_per_wd')
        )
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

    if options.go_home:
        try:
            run = go_home(strategy_map, deg_per_yaw, mm_per_wd)
        except ValueError as exc:
            print(f"Go-home cannot proceed: {exc}")
            return
    else:
        run = explore(
            deg_per_yaw,
            mm_per_wd,
            x0,
            y0,
            heading0,
            strategy_map,
            duration=options.duration,
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
