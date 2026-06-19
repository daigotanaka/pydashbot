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

import yaml

from dash.remote.client import send_command
try:
    from apps.map.policies.conservative_exploration import (
        ConservativeExploration,
        GRID_CELLS,
        TERRITORY_MM,
        densify_path,
        territory_cell,
    )
    from apps.map.policies.coverage_exploration import CoverageExploration
    from apps.map.policies.exploration_policy import NoveltyExplorationPolicy
    from apps.map.exploration_walls import (
        WALL_SEGMENT_AVOID_MM,
        inferred_wall_segments,
        point_segment_distance,
    )
    from apps.map.policies.exploration_policies import load_exploration_policy
    from apps.map.strategies.go_home_strategies import (
        DStarLiteStrategy,
        HardBlockedEdgeStrategy,
        edge_is_blocked,
    )
except ModuleNotFoundError:
    from policies.conservative_exploration import (
        ConservativeExploration,
        GRID_CELLS,
        TERRITORY_MM,
        densify_path,
        territory_cell,
    )
    from policies.coverage_exploration import CoverageExploration
    from policies.exploration_policy import NoveltyExplorationPolicy
    from exploration_walls import (
        WALL_SEGMENT_AVOID_MM,
        inferred_wall_segments,
        point_segment_distance,
    )
    from policies.exploration_policies import load_exploration_policy
    from strategies.go_home_strategies import (
        DStarLiteStrategy,
        HardBlockedEdgeStrategy,
        edge_is_blocked,
    )

CAL_FILE_PATTERN = 'calibration_????????-??-??-??.json'

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
DOCK_CLEARANCE_MM = 310    # back off this far from each wall after contact
DOCK_WALL_SEARCH_MM = 500  # max travel to find each wall during docking
INITIAL_HEADING_ANGLES = [0, -30, 30, -60, 60, -90, 90, -120, 120, -150, 150, 180]
REDIRECT_ANGLES = [-45, 45, -60, 60, -90, 90, -120, 120, -150, 150, 180]
STRATEGY_SAMPLE_DISTANCES = [400, 800, 1200, 1600]
BLOCKER_CORRIDOR_MM = 275
INFERRED_WALL_PENALTY = 500000
ODOMETRY_SIGN_TOLERANCE_MM = 40
ODOMETRY_MAX_MOVE_HEADING_DEG = 45
ODOMETRY_MAX_TURN_DISTANCE_MM = 150
# Wheel-vs-gyro slip detection. The left/right wheel-tick difference implies a
# heading change through the track width; if it diverges from the heading the
# gyro actually measured by more than this, one wheel slipped (it spun without
# the robot turning, or vice versa) and the transition is rejected before it
# corrupts the pose. Track width was backed out from clean in-place turns
# (heading = (right - left) * mm_per_wheel_tick / track_width); see README.
TRACK_WIDTH_MM = 87.0
ODOMETRY_SLIP_HEADING_DEG = 45
COMMON_MODE_SLIP_MIN_DISTANCE_MM = 150
COMMON_MODE_SLIP_FULL_REQUEST_RATIO = 0.9
COMMON_MODE_SLIP_LEGACY_MAX_REQUEST_MM = 500
# Loop closure must be a small drift nudge against a genuinely co-located wall,
# not a teleport toward a different wall. A wide match radius binds an
# observation to the wrong wall and a large max correction then jumps the pose
# hundreds of mm, corrupting the map (observed up to 327 mm "matches" snapping
# the pose ~250 mm). Keep the match tight (same wall, modest drift) and the
# correction genuinely bounded; under-correcting drift is far safer than
# snapping to the wrong landmark.
LOOP_CLOSURE_MATCH_RADIUS_MM = 120
LOOP_CLOSURE_PATH_RADIUS_MM = 400
LOOP_CLOSURE_MAX_CORRECTION_MM = 50
LOOP_CLOSURE_GAIN = 0.6
HOME_ROUTE_LINK_RADIUS_MM = 250
HOME_ROUTE_COLLINEAR_DEG = 12
HOME_MAX_LEG_MM = 1000
HOME_POSITION_TOLERANCE_MM = 100
HOME_OBSTACLE_ACCEPT_MM = 400       # an obstacle this close to home triggers final approach
BLOCKED_EDGE_TOLERANCE_MM = 150
# Final approach once within HOME_OBSTACLE_ACCEPT_MM of home: creep straight at
# home with a relaxed front threshold and slow short steps for precision, then
# re-reference to the rear wall (mirroring docking) to nail the rear axis.
HOME_FINAL_CRAWL_STEP_MM = 80
HOME_FINAL_CRAWL_SPEED_MMPS = 60
HOME_FINAL_CRAWL_PROX_THRESHOLD = 40
HOME_REAR_BACKUP_MAX_MM = 250
HOME_REAR_BACKUP_SPEED_MMPS = 60
GO_HOME_MAX_RETRIES = 3              # replan around blockages this many times
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

LEGACY_GO_HOME_STRATEGY = HardBlockedEdgeStrategy(
    HOME_ROUTE_LINK_RADIUS_MM,
    BLOCKED_EDGE_TOLERANCE_MM,
    HOME_ROUTE_COLLINEAR_DEG,
)
ACTIVE_GO_HOME_STRATEGY = DStarLiteStrategy(
    HOME_ROUTE_LINK_RADIUS_MM,
    HOME_ROUTE_COLLINEAR_DEG,
)
GO_HOME_STRATEGIES = {
    ACTIVE_GO_HOME_STRATEGY.name: ACTIVE_GO_HOME_STRATEGY,
    LEGACY_GO_HOME_STRATEGY.name: LEGACY_GO_HOME_STRATEGY,
}
# Heading policy selected by name in the config.
# `novelty` is the unconstrained default base; `conservative` and `coverage`
# are bounded-territory policies (subclasses of ConservativeExploration).
EXPLORATION_POLICIES = {
    'novelty': NoveltyExplorationPolicy,
    'conservative': ConservativeExploration,
    'coverage': CoverageExploration,
}
DEFAULT_EXPLORATION_POLICY = 'conservative'
MAPPING_CONFIG_KEYS = {
    'map_file',
    'calibration',
    'duration_seconds',
    'exploration_policy',
    'territory_size_mm',
    'docking',
    'policy',
    'dashboard',
}
DEFAULT_DOCKING_CONFIG = {
    'init': True,
    'go-home-strategy': ACTIVE_GO_HOME_STRATEGY.name,
}
DEFAULT_DASHBOARD_CONFIG = {
    'active': False,
    'host': '0.0.0.0',
    'port': 8000,
}
DEFAULT_MAPPING_CONFIG = Path(__file__).with_name('config') / 'config.yaml'

WALL_SOUNDS   = ['ohno', 'ayayay', 'huh', 'confused2', 'confused3']
TILT_SOUNDS   = ['ayayay', 'ohno', 'confused5', 'confused8']
RESUME_SOUNDS = ['okay', 'wee']


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'mode',
        choices=('start', 'resume', 'dock'),
        help=(
            "start a docked mapping run, resume from the map's final pose, "
            "or return home from the map's final pose"
        ),
    )
    parser.add_argument(
        '--config',
        metavar='CONFIG_FILE',
        default=str(DEFAULT_MAPPING_CONFIG),
        help=f"load YAML settings from CONFIG_FILE (default: {DEFAULT_MAPPING_CONFIG})",
    )
    options = parser.parse_args(args)
    config = load_mapping_config(Path(options.config), parser)
    apply_mapping_config(options, config, parser)
    return options


def load_mapping_config(config_file, parser):
    """Load and validate the mapper's human-editable YAML configuration."""
    try:
        config = yaml.safe_load(config_file.read_text())
    except OSError as exc:
        parser.error(f"cannot read config file {config_file}: {exc}")
    except yaml.YAMLError as exc:
        parser.error(f"invalid YAML in config file {config_file}: {exc}")
    if not isinstance(config, dict):
        parser.error(f"config file {config_file} must contain a YAML mapping")
    unknown = sorted(set(config) - MAPPING_CONFIG_KEYS)
    if unknown:
        parser.error(f"unknown config setting(s): {', '.join(unknown)}")
    return config


def apply_mapping_config(options, config, parser):
    """Apply validated config values to parsed run-mode options."""
    map_file = config.get('map_file')
    if not isinstance(map_file, str) or not map_file.strip():
        parser.error("config map_file must be a non-empty file path")
    options.map_file = map_file
    options.calibration = config.get('calibration')
    options.duration = config.get('duration_seconds', DURATION)
    options.territory_size = config.get('territory_size_mm', TERRITORY_MM)
    options.docking = validate_docking_config(config.get('docking', {}), parser)
    options.go_home_strategy = options.docking['go-home-strategy']
    options.policy = validate_policy_config(config.get('policy', []), parser)
    options.dashboard = validate_dashboard_config(
        config.get('dashboard', {}), parser
    )

    options.exploration_policy = config.get(
        'exploration_policy', DEFAULT_EXPLORATION_POLICY
    )

    try:
        options.duration = positive_seconds(options.duration)
        options.territory_size = positive_mm(options.territory_size)
    except argparse.ArgumentTypeError as exc:
        parser.error(str(exc))
    if options.exploration_policy not in EXPLORATION_POLICIES:
        parser.error(
            "config exploration_policy must be one of: "
            f"{', '.join(EXPLORATION_POLICIES)}"
        )


def validate_docking_config(docking, parser):
    """Validate dock initialization and go-home settings."""
    if docking is None:
        docking = {}
    if not isinstance(docking, dict):
        parser.error("config docking must be a mapping")
    unknown = sorted(set(docking) - set(DEFAULT_DOCKING_CONFIG))
    if unknown:
        parser.error(f"unknown docking setting(s): {', '.join(unknown)}")
    merged = {**DEFAULT_DOCKING_CONFIG, **docking}
    if not isinstance(merged['init'], bool):
        parser.error("config docking.init must be true or false")
    if merged['go-home-strategy'] not in GO_HOME_STRATEGIES:
        parser.error(
            "config docking.go-home-strategy must be one of: "
            f"{', '.join(GO_HOME_STRATEGIES)}"
        )
    return merged


def validate_dashboard_config(dashboard, parser):
    """Validate live dashboard settings."""
    if dashboard is None:
        dashboard = {}
    if not isinstance(dashboard, dict):
        parser.error("config dashboard must be a mapping")
    unknown = sorted(set(dashboard) - set(DEFAULT_DASHBOARD_CONFIG))
    if unknown:
        parser.error(f"unknown dashboard setting(s): {', '.join(unknown)}")
    merged = {**DEFAULT_DASHBOARD_CONFIG, **dashboard}
    if not isinstance(merged['active'], bool):
        parser.error("config dashboard.active must be true or false")
    if not isinstance(merged['host'], str) or not merged['host'].strip():
        parser.error("config dashboard.host must be a non-empty string")
    try:
        port = int(merged['port'])
    except (TypeError, ValueError):
        parser.error("config dashboard.port must be an integer")
    if port <= 0 or port > 65535:
        parser.error("config dashboard.port must be between 1 and 65535")
    merged['port'] = port
    return merged


def validate_policy_config(policy, parser):
    """Validate ordered command-policy configuration."""
    if policy is None:
        return []
    if not isinstance(policy, list):
        parser.error("config policy must be a list")
    for index, item in enumerate(policy):
        if not isinstance(item, dict):
            parser.error(f"config policy item {index + 1} must be a mapping")
        name = item.get('name')
        if not isinstance(name, str) or not name.strip():
            parser.error(f"config policy item {index + 1} requires a non-empty name")
    return policy


def positive_seconds(value):
    seconds = float(value)
    if seconds <= 0:
        raise argparse.ArgumentTypeError("duration must be greater than zero")
    return seconds


def positive_mm(value):
    millimeters = float(value)
    if millimeters <= 0:
        raise argparse.ArgumentTypeError("territory size must be greater than zero")
    return millimeters


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


def validate_odometry(
    action,
    requested,
    distance_mm,
    heading_delta,
    left_delta=None,
    right_delta=None,
    mm_per_wheel_tick=None,
):
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
    if (
        action in ('forward', 'reverse')
        and left_delta is not None
        and right_delta is not None
        and mm_per_wheel_tick
    ):
        # On a straight move the wheel-tick difference implies a heading change;
        # if it disagrees with what the gyro actually measured, one wheel slipped
        # and the averaged distance that updates position is bogus. This catches
        # slips the gyro-only checks above miss, because they trust the measured
        # heading. Turns are excluded: there the gyro is the source of truth for
        # heading and translation is ~0, so a wheel over-spin cannot corrupt the
        # pose.
        wheel_heading_delta = math.degrees(
            (right_delta - left_delta) * mm_per_wheel_tick / TRACK_WIDTH_MM
        )
        if abs(wheel_heading_delta - heading_delta) > ODOMETRY_SLIP_HEADING_DEG:
            issues.append('wheel rotation inconsistent with gyro (suspected wheel slip)')
    return issues


def blocked_common_mode_slip_issue(
    action,
    requested,
    distance_mm,
    motion_outcome=None,
    front_blocked=False,
):
    """Return a common-mode slip issue when a blocked leg logged bogus travel."""
    if action != 'forward':
        return None
    if abs(distance_mm) < COMMON_MODE_SLIP_MIN_DISTANCE_MM:
        return None

    requested = abs(requested)
    halt = (motion_outcome or {}).get('halt')
    phase = (motion_outcome or {}).get('phase')
    if halt == 'obstacle' and phase == 'before_start':
        return 'front obstacle before start but wheels measured forward travel'
    if (
        halt == 'obstacle'
        and phase == 'moving'
        and abs(distance_mm) >= requested * COMMON_MODE_SLIP_FULL_REQUEST_RATIO
    ):
        return 'front obstacle stop with near-full wheel travel (suspected common-mode slip)'

    # Older map files did not record the move() outcome. Keep a narrow fallback
    # for the original failure mode: a short boundary-limited probe that ends
    # blocked but claims essentially the full requested distance.
    if (
        motion_outcome is None
        and front_blocked
        and requested <= COMMON_MODE_SLIP_LEGACY_MAX_REQUEST_MM
        and abs(distance_mm) >= requested * COMMON_MODE_SLIP_FULL_REQUEST_RATIO
    ):
        return 'front obstacle with near-full wheel travel (suspected common-mode slip)'
    return None


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


def map_knowledge(data, path_sample_mm=None):
    """Extract path and blocker points used by the exploration strategy."""
    runs = accepted_runs(data)
    if path_sample_mm is None:
        path_points = [
            (float(point[0]), float(point[1]))
            for run in runs
            for point in run.get('path', [])
        ]
    else:
        path_points = [
            point
            for run in runs
            for point in densify_path(run.get('path', []), path_sample_mm)
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
    blockers,
    blocked_left=0,
    blocked_right=0,
    point_allowed=None,
    heading_preference=None,
    wall_segments=(),
):
    """Score a candidate heading as policy preference minus physical penalties.

    All *positive* (preferred) scoring comes from ``heading_preference`` (an
    ``ExplorationPolicy`` hook); everything here is a physical-constraint
    penalty: turn cost, leaving the allowed region, known blockers, inferred
    walls, and live proximity.
    """
    hr = math.radians(heading + turn_angle)
    ux, uy = math.cos(hr), math.sin(hr)
    score = -abs(turn_angle) * 0.35

    for distance in STRATEGY_SAMPLE_DISTANCES:
        sx, sy = x + distance * ux, y + distance * uy
        if point_allowed is not None and not point_allowed(sx, sy):
            score -= 5000

    for bx, by in blockers:
        dx, dy = bx - x, by - y
        forward = dx * ux + dy * uy
        lateral = abs(dx * uy - dy * ux)
        if 0 < forward < 2000 and lateral < BLOCKER_CORRIDOR_MM:
            score -= (BLOCKER_CORRIDOR_MM - lateral) * 8
            score -= (2000 - forward) * 1.5

    for distance in STRATEGY_SAMPLE_DISTANCES:
        point = x + distance * ux, y + distance * uy
        if any(
            point_segment_distance(point, start, end) <= WALL_SEGMENT_AVOID_MM
            for start, end in wall_segments
        ):
            score -= INFERRED_WALL_PENALTY
            break

    if blocked_left >= PROX_THRESHOLD and turn_angle > 0:
        score -= 2500
    if blocked_right >= PROX_THRESHOLD and turn_angle < 0:
        score -= 2500
    if blocked_left >= PROX_THRESHOLD and blocked_right >= PROX_THRESHOLD:
        score += abs(turn_angle) * 8
    if heading_preference is not None:
        score += heading_preference(x, y, normalize_heading(heading + turn_angle))
    return score


def choose_exploration_angle(
    x,
    y,
    heading,
    blockers,
    blocked_left=0,
    blocked_right=0,
    require_turn=False,
    point_allowed=None,
    heading_preference=None,
    wall_segments=(),
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
            blockers,
            blocked_left,
            blocked_right,
            point_allowed,
            heading_preference,
            wall_segments,
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
            "Run uv run apps/map/calibrate.py first."
        )
    return files[-1]


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


def plan_home_route(data, strategy=ACTIVE_GO_HOME_STRATEGY):
    """Return the route selected by the requested go-home strategy."""
    return strategy.plan_route(data, accepted_runs, run_pose_trustworthy)


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
# Corner dock — establishes (0, 0) as the corner, rear and left walls detected
# ---------------------------------------------------------------------------
def dock_to_corner(deg_per_yaw, mm_per_wd, wait_sec=15):
    """Back into rear wall, turn CCW to face left wall, advance into it, turn CW.

    Placement: robot at the dock corner with a wall behind it and a wall to its
    left.  Both walls may be out of sensor range until the robot moves toward
    them.  The CCW-then-CW turns leave the robot facing back into the room (net
    heading unchanged) with both walls now within known sensor-intensity
    ranges, giving a consistent origin regardless of initial placement.
    """
    print("\n=== Corner Dock ===")
    print("  Place robot at dock corner: wall behind the robot AND wall to its left.")
    print("  Both walls may be out of sensor range initially.")
    print(f"  Starting in {wait_sec} seconds...")
    for i in range(wait_sec, 0, -1):
        print(f'  {i}...', end='\r')
        if i == 5:
            send_command('say', 'beep')
        time.sleep(1)
    print()

    # -- Step 1: back into rear wall (max DOCK_WALL_SEARCH_MM travel) --
    print(f"  Backing toward rear wall (max {DOCK_WALL_SEARCH_MM}mm)...")
    outcome = send_command(
        'move', -DOCK_WALL_SEARCH_MM, DOCK_SPEED,
        wall_stop_sound=None,
    ).get('result')
    found_rear = (
        isinstance(outcome, dict)
        and outcome.get('halt') == 'obstacle'
        and outcome.get('side') == 'rear'
    )
    if not found_rear:
        print(
            f"  *** Rear wall not detected within {DOCK_WALL_SEARCH_MM}mm — "
            "check robot placement! ***"
        )
        send_command('say', 'ohno')
    else:
        rear = send_command('get_prox_rear')['result']
        print(f'\n  Rear wall contact (prox_rear={rear})')
        send_command('say', 'okay')
    time.sleep(0.3)

    # -- Step 2: turn 90° CCW to face the left wall --
    print("  Turning 90° counter-clockwise to face left wall...")
    send_command('turn', 90)
    time.sleep(0.2)

    # -- Step 3: advance into left wall (max DOCK_WALL_SEARCH_MM travel) --
    print(f"  Moving toward left wall (max {DOCK_WALL_SEARCH_MM}mm)...")
    outcome = send_command(
        'move', DOCK_WALL_SEARCH_MM, DOCK_SPEED,
        wall_stop_sound=None,
    ).get('result')
    found_left = (
        isinstance(outcome, dict)
        and outcome.get('halt') == 'obstacle'
        and outcome.get('side') == 'front'
    )
    if not found_left:
        print(
            f"  *** Left wall not detected within {DOCK_WALL_SEARCH_MM}mm — "
            "check robot placement! ***"
        )
        send_command('say', 'ohno')
    else:
        l = send_command('get_prox_left')['result']
        r = send_command('get_prox_right')['result']
        print(f'\n  Left wall contact (prox L={l} R={r})')
        send_command('say', 'okay')
    time.sleep(0.3)

    # -- Step 4: turn 90° CW — back to facing into the room (net 0° turn) --
    print("  Turning 90° clockwise to face room...")
    send_command('turn', -90)
    time.sleep(0.3)

    # Both walls were within known proximity-sensor ranges at the moment of
    # contact in steps 1 and 3 (rear, then left). This gives a consistent
    # corner reference regardless of where within the acceptable placement zone
    # the user set the robot down initially.
    # The robot's rotation axis sits ~310mm from each wall, measured physically.
    # The map frame places the start in the positive quadrant so the starting
    # territory is (0,0) and the room grows toward +x/+y; see update_pose, which
    # mirrors the gyro's handedness into this frame.
    x0 = 310.0
    y0 = 310.0
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


def obstacle_arrival_near_home(distance_to_home, move_outcome, tolerance=HOME_OBSTACLE_ACCEPT_MM):
    """Whether an obstacle halt this close to home should count as arrival.

    Returning to the starting corner means approaching its walls head-on, so the
    forward sensors stop Dash a short distance out. When the halt is an obstacle
    and Dash is already within tolerance of the home pose, treat it as arrival
    rather than a blocked route, and finish by turning to the start orientation.
    """
    return (
        distance_to_home <= tolerance
        and isinstance(move_outcome, dict)
        and move_outcome.get('halt') == 'obstacle'
    )


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


def go_home(data, deg_per_yaw, mm_per_wd, strategy=ACTIVE_GO_HOME_STRATEGY):
    """Follow the route selected by a go-home strategy."""
    route = plan_home_route(data, strategy)
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
            left_delta,
            right_delta,
            mm_per_wd,
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
            # Map frame mirrors the gyro handedness (see the class update_pose):
            # accumulate the negated heading delta while turn commands are
            # negated to physical at the send_command('turn', ...) sites.
            heading = normalize_heading(heading - heading_delta)
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
            # Negate the map-frame angle to a physical turn command (see
            # update_pose); update_pose records the physical step for odometry.
            physical_step = -step
            outcome = send_command('turn', physical_step).get('result')
            previous_heading = heading
            if update_pose('turn', physical_step) is None:
                return False
            if abs(angle_delta(heading, previous_heading)) < abs(step) * 0.5:
                issues.append('go-home turn did not execute')
                halt_reason = outcome
                print(f"\n  Turn did not execute — {describe_halt(outcome, deg_per_yaw)}")
                return False
        return True

    def crawl_home():
        """Creep straight at home with relaxed detection and conservative motion."""
        while math.hypot(start_x - x, start_y - y) > HOME_POSITION_TOLERANCE_MM:
            if not turn_to(math.degrees(math.atan2(start_y - y, start_x - x))):
                return
            step = home_leg_distance(
                min(HOME_FINAL_CRAWL_STEP_MM, math.hypot(start_x - x, start_y - y))
            )
            response = send_command(
                'move',
                step,
                HOME_FINAL_CRAWL_SPEED_MMPS,
                wall_stop_sound=None,
                proximity_threshold=HOME_FINAL_CRAWL_PROX_THRESHOLD,
                proximity_confirm_count=HOME_RETRACE_CONFIRM_COUNT,
            )
            if response.get('ok', True) is False:
                return
            traveled = update_pose('forward', step)
            print(
                f"  Crawling home: {math.hypot(start_x - x, start_y - y):.0f}mm  "
                f"pose=({x:.0f},{y:.0f})",
                end='\r',
            )
            if traveled is None or traveled < step * 0.8:
                return  # blocked even with relaxed detection — as close as we get

    def rear_reference():
        """Re-reference to the rear wall for final precision, mirroring docking."""
        nonlocal x, y
        rear = send_command('get_prox_rear')['result']
        if rear is not None and rear >= REAR_THRESHOLD:
            return  # already against the rear wall
        print("\n  Re-referencing to rear wall for final precision...")
        # Reverse until the rear wall is detected (rear-obstacle-aware, bounded).
        outcome = send_command(
            'move', -HOME_REAR_BACKUP_MAX_MM, HOME_REAR_BACKUP_SPEED_MMPS,
            wall_stop_sound=None,
        ).get('result')
        found_wall = (
            isinstance(outcome, dict)
            and outcome.get('halt') == 'obstacle'
            and outcome.get('side') == 'rear'
        )
        if not found_wall:
            print("  Rear wall not found within range; skipping re-reference.")
            return
        # Step forward to match the docked clearance and record the start pose.
        send_command(
            'move', DOCK_CLEARANCE_MM, HOME_REAR_BACKUP_SPEED_MMPS,
            wall_stop_sound=None, stop_at_obstacle=False,
        )
        x, y = float(start_x), float(start_y)
        path.append((x, y, heading))

    print("\n=== Going home ===")
    print(f"  Strategy: {strategy.name}")
    print(
        f"  Planned {sum(math.dist(a, b) for a, b in zip(route, route[1:])):.0f}mm "
        f"along {len(route)} proven-route waypoints."
    )
    send_command('say', 'okay')
    send_command('neck_color', '#00ffff')
    completed = True
    arrived = False

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
                    distance_home = math.hypot(start_x - x, start_y - y)
                    if traveled is not None and obstacle_arrival_near_home(
                        distance_home, move_outcome
                    ):
                        arrived = True
                        print(
                            f"\n  Obstacle {distance_home:.0f}mm from home "
                            f"({describe_halt(move_outcome, deg_per_yaw)}) — "
                            f"close enough, treating as arrival."
                        )
                        break
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
            if arrived or not completed:
                break
            prev_waypoint = (waypoint_x, waypoint_y)

        if arrived:
            crawl_home()
        if completed:
            completed = turn_to(start_heading)
        if arrived and completed:
            rear_reference()
        home_tolerance = HOME_OBSTACLE_ACCEPT_MM if arrived else HOME_POSITION_TOLERANCE_MM
        if completed and math.hypot(start_x - x, start_y - y) > home_tolerance:
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
        'strategy': strategy.name,
        'status': 'accepted' if completed else 'partial',
        'quality': {
            'accepted_updates': sum(event['accepted'] for event in events),
            'rejected_updates': sum(not event['accepted'] for event in events),
            'tracking_lost': odometry_rejected,
            'issues': issues,
            'halt_reason': halt_reason,
            'arrived_blocked_near_home': arrived,
        },
        'planned_route': route,
        'path': path,
        'walls': [],
        'obstacles': [],
        'blocked_edges': blocked_edges,
        'events': events,
    }


def go_home_with_retries(
    data,
    deg_per_yaw,
    mm_per_wd,
    max_retries=GO_HOME_MAX_RETRIES,
    strategy=ACTIVE_GO_HOME_STRATEGY,
):
    """Drive home, replanning around blockages until arrival or retries run out.

    Each aborted attempt that records a blocked edge and keeps a trustworthy pose
    lets the next attempt replan around the blockage from where Dash stopped, so
    a return that halts far from home can keep trying alternative proven routes.
    Returns the list of run records produced; each should be saved to the map.
    """
    runs = []
    data = {**data, 'runs': list(data.get('runs', []))}
    for attempt in range(max_retries + 1):
        try:
            run = go_home(data, deg_per_yaw, mm_per_wd, strategy)
        except ValueError as exc:
            print(f"Go-home cannot proceed: {exc}")
            break
        runs.append(run)
        data['runs'].append(run)
        if run.get('status') == 'accepted':
            break
        if not run_pose_trustworthy(run):
            print("  Stopping retries: final pose is no longer trustworthy.")
            break
        if not run.get('blocked_edges'):
            print("  Stopping retries: halt was not a blockage to route around.")
            break
        if attempt < max_retries:
            print(
                f"\n  Retry {attempt + 1}/{max_retries}: "
                f"replanning around the blockage..."
            )
        else:
            print(f"\n  Reached max retries ({max_retries}) without arriving home.")
    return runs


# ---------------------------------------------------------------------------
# Exploration
# ---------------------------------------------------------------------------
class _ExplorationRun:
    """Mutable state and behaviour for a single exploration run.

    ``explore`` below is the high-level driver; everything that mutates the
    live pose, the accumulated map, or the run quality lives here as methods,
    so the driver stays short and the flow is easy to read.
    """

    def __init__(
        self,
        deg_per_yaw,
        mm_per_wd,
        x0,
        y0,
        heading0=0.0,
        strategy_map=None,
        duration=DURATION,
        territory_mm=TERRITORY_MM,
        command_policy=None,
        exploration_policy=DEFAULT_EXPLORATION_POLICY,
        dashboard=None,
    ):
        self.deg_per_yaw = deg_per_yaw
        self.mm_per_wd = mm_per_wd
        self.duration = duration
        self.command_policy = command_policy
        self.dashboard = dashboard

        self.yaw_prev = send_command('get_yaw')['result']
        self.left_prev = send_command('get_left_wheel')['result']
        self.right_prev = send_command('get_right_wheel')['result']
        pitch_samples = [send_command('get_pitch')['result'] for _ in range(5)]
        self.baseline_pitch = sum(pitch_samples) / len(pitch_samples)

        self.heading = normalize_heading(heading0)
        self.x, self.y = x0, y0

        self.path = [(self.x, self.y, self.heading)]
        self.walls = []
        self.obstacles = []
        self.events = []
        self.quality = {
            'accepted_updates': 0,
            'rejected_updates': 0,
            'loop_closures': 0,
            'loop_closure_correction_mm': 0.0,
            'tracking_lost': False,
            'issues': [],
        }
        self.policy_commands_completed = 0
        self.cell_mm = territory_mm / GRID_CELLS
        self.path_sample_mm = self.cell_mm / 2
        self.known_path, self.known_blockers = map_knowledge(
            strategy_map or {}, path_sample_mm=self.path_sample_mm
        )
        self.known_walls = [
            (float(point[0]), float(point[1]))
            for run in accepted_runs(strategy_map or {})
            for point in run.get('walls', [])
        ]
        self.known_obstacles = [
            (float(point[0]), float(point[1]))
            for run in accepted_runs(strategy_map or {})
            for point in run.get('obstacles', [])
        ]
        # Link wall observations within one reachability cell, so a smaller
        # territory infers continuous walls at a proportionally finer scale
        # (cell = territory / grid). Replaces the former fixed 300 mm threshold.
        self.known_wall_segments = inferred_wall_segments(
            self.known_walls, max_distance=self.cell_mm
        )
        # The robot docks in a room corner at world (0,0): a wall behind it
        # (along the x=0 axis) and a wall to its side (along the y=0 axis). Those
        # walls are implied by the docking sequence and never observed, so the
        # policy would otherwise propose expanding into the unreachable space
        # behind them (e.g. territory (0,-1)/(-1,0)). Record them as known
        # segments extending from the corner into the explorable quadrant (the
        # sign of the start pose), so expansion-crossing checks and the
        # reachability BFS reject crossings of either axis. They lie on the
        # start territory's edges, clear of its cell centers, so no within-
        # territory connection is falsely blocked.
        dock_wall_length = 20 * territory_mm
        sign_x = math.copysign(1.0, self.x) if self.x else 1.0
        sign_y = math.copysign(1.0, self.y) if self.y else 1.0
        self.known_wall_segments = self.known_wall_segments + [
            ((0.0, 0.0), (0.0, sign_y * dock_wall_length)),  # rear wall (x=0)
            ((0.0, 0.0), (sign_x * dock_wall_length, 0.0)),  # side wall (y=0)
        ]
        # There is always a policy, selected by name. Bounded-territory policies
        # (subclasses of ConservativeExploration) take the territory state;
        # the unconstrained novelty default takes only the live path.
        policy_class = EXPLORATION_POLICIES[exploration_policy]
        if issubclass(policy_class, ConservativeExploration):
            self.policy = policy_class(
                accepted_runs(strategy_map or {}),
                (self.x, self.y),
                self.known_path,
                self.known_blockers,
                self.known_wall_segments,
                territory_mm,
            )
        else:
            self.policy = policy_class(
                self.known_path, STRATEGY_SAMPLE_DISTANCES
            )
        self.known_path.append((self.x, self.y))
        self.publish_dashboard_pose(duration=0.0)

    # -- pose tracking -----------------------------------------------------
    def publish_dashboard_pose(self, event=None, duration=None):
        if self.dashboard is None:
            return
        try:
            move = {
                'pose': [self.x, self.y, self.heading],
                'timestamp': datetime.now().isoformat(timespec='seconds'),
            }
            if event is not None:
                move['event'] = event
            if duration is not None:
                move['duration'] = duration
            self.dashboard.post_move(move)
        except Exception as exc:
            print(f"\n  [dashboard warning] failed to publish pose: {exc}")

    def update_pose(self, action, requested):
        yaw_now = read_settled('get_yaw')
        left_now = read_settled('get_left_wheel')
        right_now = read_settled('get_right_wheel')
        d_yaw = wrap_delta(self.yaw_prev, yaw_now, 12)
        left_delta = wrap_delta(self.left_prev, left_now, 16)
        right_delta = wrap_delta(self.right_prev, right_now, 16)
        d_dist = (left_delta + right_delta) / 2
        heading_delta = d_yaw * self.deg_per_yaw
        d_mm = d_dist * self.mm_per_wd
        self.yaw_prev = yaw_now
        self.left_prev = left_now
        self.right_prev = right_now
        issues = validate_odometry(
            action,
            requested,
            d_mm,
            heading_delta,
            left_delta,
            right_delta,
            self.mm_per_wd,
        )
        event = {
            'action': action,
            'requested': requested,
            'raw_yaw_delta': d_yaw,
            'raw_left_wheel_delta': left_delta,
            'raw_right_wheel_delta': right_delta,
            'translation_wheel_delta': d_dist,
            'heading_delta': heading_delta,
            'distance_mm': d_mm,
            'accepted': not issues and not self.quality['tracking_lost'],
        }
        if issues:
            event['issues'] = issues
            self.quality['rejected_updates'] += 1
            self.quality['tracking_lost'] = True
            self.quality['issues'].extend(issues)
            print(f"\n  [odometry rejected] {'; '.join(issues)}")
        elif self.quality['tracking_lost']:
            event['issues'] = ['pose tracking was already lost']
        else:
            previous_position = (self.x, self.y)
            # Map frame mirrors the gyro's handedness so the room grows toward
            # +y (start territory (0,0)). The gyro/odometry stay in the physical
            # frame -- validate_odometry above and the recorded event keep the
            # physical heading_delta -- but the stored pose accumulates its
            # negation. Map-frame turn angles are negated back to physical at the
            # send_command('turn', ...) sites, keeping the control loop
            # consistent (command -theta -> measure -theta -> heading += theta).
            self.heading = normalize_heading(self.heading - heading_delta)
            hr = math.radians(self.heading)
            self.x += d_mm * math.cos(hr)
            self.y += d_mm * math.sin(hr)
            self.path.append((self.x, self.y, self.heading))
            self.known_path.extend(
                densify_path(
                    [previous_position, (self.x, self.y)], self.path_sample_mm
                )[1:]
            )
            self.quality['accepted_updates'] += 1
            self.policy.report_progress()
            self.publish_dashboard_pose(event)
        self.events.append(event)
        if not event['accepted']:
            return None
        return d_mm

    # -- steering ----------------------------------------------------------
    def turn_toward_knowledge(self, reason, left=0, right=0, require_turn=False):
        self.policy.unlock_if_complete()
        turn_angle = choose_exploration_angle(
            self.x,
            self.y,
            self.heading,
            self.known_blockers,
            blocked_left=left,
            blocked_right=right,
            require_turn=require_turn,
            point_allowed=self.policy.allows_point,
            heading_preference=self.policy.heading_preference,
            wall_segments=self.known_wall_segments,
        )
        print(f'\n  [{reason}] map-guided turn {turn_angle:+.0f}°')
        for step in tracked_turn_steps(turn_angle):
            if not step:
                continue
            # Negate the map-frame angle to a physical turn command (see
            # update_pose); update_pose records the physical step for odometry.
            physical_step = -step
            send_command('turn', physical_step)
            if self.update_pose('turn', physical_step) is None:
                break

    def redirect(self, reason, sounds=None, back_away=False, left=0, right=0):
        print(f'\n  [{reason}] changing direction')
        send_command('stop')
        send_command('neck_color', '#ff0000')
        if sounds:
            send_command('say', random.choice(sounds))
        if back_away:
            send_command('move', -BACK_AWAY_MM, BACK_AWAY_SPEED_MMPS)
            self.update_pose('reverse', BACK_AWAY_MM)
        self.turn_toward_knowledge(reason, left, right, require_turn=True)
        send_command('say', random.choice(RESUME_SOUNDS))
        send_command('neck_color', '#00ff00')

    # -- observations ------------------------------------------------------
    def reject_blocked_territory_transition(self, previous_position, known_path_len):
        """Discard translation into another territory when the leg was blocked."""
        if not isinstance(self.policy, ConservativeExploration):
            return False
        previous_territory = territory_cell(
            previous_position[0], previous_position[1], self.policy.territory_mm
        )
        current_territory = territory_cell(
            self.x, self.y, self.policy.territory_mm
        )
        if current_territory == previous_territory:
            return False

        self.x, self.y = previous_position
        self.path[-1] = (self.x, self.y, self.heading)
        del self.known_path[known_path_len:]
        self.events[-1]['territory_transition_rejected'] = {
            'from': previous_territory,
            'to': current_territory,
        }
        print(
            f'\n  [territory transition rejected] blocked leg cannot move '
            f'from {previous_territory} to {current_territory}'
        )
        return True

    def reject_blocked_common_mode_slip(
        self,
        previous_position,
        known_path_len,
        traveled,
        requested_distance,
        motion_outcome,
        left,
        right,
    ):
        """Discard a blocked straight leg whose wheels likely spun in place."""
        issue = blocked_common_mode_slip_issue(
            'forward',
            requested_distance,
            traveled,
            motion_outcome=motion_outcome,
            front_blocked=left >= PROX_THRESHOLD or right >= PROX_THRESHOLD,
        )
        if not issue:
            return False

        previous_x, previous_y = previous_position
        self.x, self.y = previous_x, previous_y
        self.path[-1] = (self.x, self.y, self.heading)
        del self.known_path[known_path_len:]

        event = self.events[-1]
        event['accepted'] = False
        event.setdefault('issues', []).append(issue)
        event['common_mode_slip_rejected'] = {
            'previous_position': previous_position,
            'distance_mm': traveled,
            'requested': requested_distance,
            'prox_left': left,
            'prox_right': right,
            **({'motion_outcome': motion_outcome} if motion_outcome else {}),
        }
        self.quality['rejected_updates'] += 1
        self.quality['tracking_lost'] = True
        self.quality['issues'].append(issue)
        print(f"\n  [odometry rejected] {issue}")
        return True

    def mark_ahead(self, points, landmarks, offset):
        if self.quality['tracking_lost']:
            return
        hr = math.radians(self.heading)
        point = (self.x + offset * math.cos(hr), self.y + offset * math.sin(hr))
        correction = revisit_pose_correction(
            self.x,
            self.y,
            point,
            self.known_path[:-1],
            landmarks,
        )
        if correction:
            dx, dy, target, mismatch = correction
            if (
                isinstance(self.policy, ConservativeExploration)
                and territory_cell(
                    self.x + dx, self.y + dy, self.policy.territory_mm
                )
                != territory_cell(self.x, self.y, self.policy.territory_mm)
            ):
                correction = None
        if correction:
            dx, dy, target, mismatch = correction
            self.x += dx
            self.y += dy
            self.path[-1] = (self.x, self.y, self.heading)
            self.known_path[-1] = (self.x, self.y)
            point = (
                self.x + offset * math.cos(hr),
                self.y + offset * math.sin(hr),
            )
            correction_mm = math.hypot(dx, dy)
            self.quality['loop_closures'] += 1
            self.quality['loop_closure_correction_mm'] += correction_mm
            self.events[-1]['loop_closure'] = {
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
        self.known_blockers.append(point)
        if points is self.walls:
            self.known_wall_segments[:] = inferred_wall_segments(
                self.known_walls, max_distance=self.cell_mm
            )
        self.policy.report_progress()

    def report_leg(self, remaining, traveled, left, right, tilt):
        print(
            f'  [{remaining:4.1f}s] ({self.x:6.0f},{self.y:6.0f})mm  '
            f'hdg={self.heading:6.1f}°  leg={traveled:5.0f}mm  '
            f'prox L={left:2d} R={right:2d}  tilt={tilt:+.0f}',
            end='\r',
        )

    def handle_leg_end(
        self,
        traveled,
        requested_distance,
        policy_limit_reached=False,
        record_early_stop_obstacle=True,
        previous_position=None,
        known_path_len=None,
        motion_outcome=None,
    ):
        left = send_command('get_prox_left')['result']
        right = send_command('get_prox_right')['result']
        pitch = send_command('get_pitch')['result']
        tilt = pitch - self.baseline_pitch

        if left >= PROX_THRESHOLD or right >= PROX_THRESHOLD:
            if previous_position is not None:
                if self.reject_blocked_common_mode_slip(
                    previous_position,
                    known_path_len,
                    traveled,
                    requested_distance,
                    motion_outcome,
                    left,
                    right,
                ):
                    return left, right, tilt, 'odometry rejected', None, False
                self.reject_blocked_territory_transition(
                    previous_position, known_path_len
                )
            self.mark_ahead(self.walls, self.known_walls, WALL_OFFSET_MM)
            self.policy.add_blocked_territory_expansion(
                self.x, self.y, self.heading
            )
            return left, right, tilt, 'wall', WALL_SOUNDS, True
        if abs(tilt) > PITCH_TILT_THRESHOLD:
            if previous_position is not None:
                self.reject_blocked_territory_transition(
                    previous_position, known_path_len
                )
            self.mark_ahead(self.obstacles, self.known_obstacles, OBSTACLE_OFFSET_MM)
            self.policy.add_blocked_territory_expansion(
                self.x, self.y, self.heading
            )
            return left, right, tilt, f'tilt {tilt:+.0f}', TILT_SOUNDS, True
        if abs(traveled) < requested_distance * 0.8:
            if record_early_stop_obstacle:
                if previous_position is not None:
                    self.reject_blocked_territory_transition(
                        previous_position, known_path_len
                    )
                self.mark_ahead(
                    self.obstacles, self.known_obstacles, OBSTACLE_OFFSET_MM
                )
                self.policy.add_blocked_territory_expansion(
                    self.x, self.y, self.heading
                )
            return left, right, tilt, 'early stop', WALL_SOUNDS, True
        if policy_limit_reached:
            return left, right, tilt, 'exploration boundary', None, False
        return left, right, tilt, 'forward leg complete', None, False

    def stop_safely(self):
        send_command('stop')
        send_command('say', 'bye')
        send_command('neck_color', '#ffffff')

    # -- run phases --------------------------------------------------------
    def announce(self):
        if self.command_policy:
            print(f"\n=== Exploring preset course ===")
        else:
            print(f"\n=== Exploring for {self.duration:g}s ===")
            print(
                f"  Repeatedly trying {FORWARD_DISTANCE_MM}mm forward legs; "
                "walls and tilt stop each leg early."
            )
        print(self.policy.describe())
        send_command('say', 'hi')
        send_command('neck_color', '#00ff00')
        self.policy.report_progress()
        if self.command_policy:
            print(
                f"  Command policy: {self.command_policy.name} "
                f"({len(self.command_policy.commands)} commands)"
            )

    def drive_preset_course(self):
        command_policy = self.command_policy
        for index, command in enumerate(command_policy.commands, start=1):
            if self.quality['tracking_lost']:
                break
            name, value = command['command'], command['value']
            print(
                f"\n  [preset {index}/{len(command_policy.commands)}] "
                f"{name} {value:g}"
            )
            if name == 'turn':
                for step in tracked_turn_steps(value):
                    # Preset angles are map-frame; negate to a physical turn
                    # command (see update_pose).
                    physical_step = -step
                    send_command('turn', physical_step)
                    if self.update_pose('turn', physical_step) is None:
                        break
                if self.quality['tracking_lost']:
                    break
                self.policy_commands_completed = index
                continue
            response = send_command(
                'move',
                value,
                SENSOR_SAFE_SPEED_MMPS,
                wall_stop_sound=None,
                stop_at_obstacle=command['stop_at_obstacle'],
            )
            if not response.get('ok', False):
                self.quality['tracking_lost'] = True
                issue = f"preset move command failed: {response.get('error', 'unknown error')}"
                self.quality['issues'].append(issue)
                self.events.append(
                    {
                        'action': 'forward',
                        'requested': value,
                        'accepted': False,
                        'issues': [issue],
                        'motion_response': response,
                    }
                )
                print(f"\n  [preset halted] {issue}")
                break
            previous_position = (self.x, self.y)
            known_path_len = len(self.known_path)
            traveled = self.update_pose('forward', value)
            self.events[-1]['motion_outcome'] = response['result']
            if traveled is None:
                break
            left, right, tilt, reason, _, back_away = self.handle_leg_end(
                traveled,
                value,
                record_early_stop_obstacle=command['stop_at_obstacle'],
                previous_position=previous_position,
                known_path_len=known_path_len,
                motion_outcome=response['result'],
            )
            self.report_leg(0.0, traveled, left, right, tilt)
            if back_away:
                print(f"\n  [preset halted] {reason}")
                break
            self.policy_commands_completed = index
        if self.policy_commands_completed == len(command_policy.commands):
            print("\n  [preset complete] final action finished")

    def explore_until(self, end_time):
        while time.time() < end_time and not self.quality['tracking_lost']:
            if self.policy.is_complete():
                print('\n  [coverage complete] no open territory expansion remains')
                break
            remaining = end_time - time.time()
            desired_distance = forward_distance_for_remaining(remaining)
            requested_distance = self.policy.forward_distance(
                self.x, self.y, self.heading, desired_distance
            )
            policy_limit_reached = requested_distance < desired_distance
            if requested_distance < MIN_FORWARD_DISTANCE_MM:
                # Pinned against the invisible territory wall. If the territory
                # the robot is in is fully mapped, unlock the one ahead and
                # re-evaluate instead of turning back.
                if self.policy.expand_past_boundary(
                    self.x, self.y, self.heading
                ):
                    continue
                self.redirect('exploration boundary')
                continue
            response = send_command(
                'move',
                requested_distance,
                FORWARD_SPEED_MMPS,
                wall_stop_sound=None,
            )
            previous_position = (self.x, self.y)
            known_path_len = len(self.known_path)
            traveled = self.update_pose('forward', requested_distance)
            self.events[-1]['motion_outcome'] = response['result']
            remaining = max(0.0, end_time - time.time())
            if traveled is None:
                left = send_command('get_prox_left')['result']
                right = send_command('get_prox_right')['result']
                tilt = send_command('get_pitch')['result'] - self.baseline_pitch
                reason, sounds, back_away = 'odometry rejected', None, False
                traveled = 0.0
                self.report_leg(remaining, traveled, left, right, tilt)
                break
            else:
                left, right, tilt, reason, sounds, back_away = self.handle_leg_end(
                    traveled,
                    requested_distance,
                    policy_limit_reached,
                    previous_position=previous_position,
                    known_path_len=known_path_len,
                    motion_outcome=response['result'],
                )
            self.report_leg(remaining, traveled, left, right, tilt)

            if (
                time.time() < end_time
                and (back_away or not self.policy.is_complete())
            ):
                self.redirect(reason, sounds, back_away, left, right)

    def result(self):
        return {
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'duration_seconds': self.duration,
            'status': 'partial' if self.quality['tracking_lost'] else 'accepted',
            'quality': self.quality,
            'path': self.path,
            'walls': self.walls,
            'obstacles': self.obstacles,
            **(
                {self.policy.metadata_key: self.policy.metadata()}
            ),
            **(
                {
                    'exploration_policy': {
                        **self.command_policy.metadata(),
                        'commands_completed': self.policy_commands_completed,
                        'completed': (
                            self.policy_commands_completed
                            == len(self.command_policy.commands)
                        ),
                    }
                }
                if self.command_policy
                else {}
            ),
            'events': self.events,
        }


def explore(
    deg_per_yaw,
    mm_per_wd,
    x0,
    y0,
    heading0=0.0,
    strategy_map=None,
    duration=DURATION,
    territory_mm=TERRITORY_MM,
    command_policy=None,
    exploration_policy=DEFAULT_EXPLORATION_POLICY,
    dashboard=None,
):
    """Drive one exploration run and return its recorded map contribution.

    High-level flow: set up the run, announce it, then either replay a preset
    course or explore on a time budget, and always stop the robot safely.
    """
    run = _ExplorationRun(
        deg_per_yaw,
        mm_per_wd,
        x0,
        y0,
        heading0,
        strategy_map,
        duration,
        territory_mm,
        command_policy,
        exploration_policy,
        dashboard,
    )
    run.announce()
    if not command_policy:
        run.turn_toward_knowledge('initial strategy')
        end_time = time.time() + duration

    try:
        if command_policy:
            run.drive_preset_course()
        else:
            run.explore_until(end_time)
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        run.stop_safely()

    print(
        f'\nDone. Path={len(run.path)} pts  '
        f'Walls={len(run.walls)}  Obstacles={len(run.obstacles)}'
    )
    return run.result()


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
# Main
# ---------------------------------------------------------------------------
def main(args=None):
    options = parse_args(args)
    map_file = Path(options.map_file)
    if options.mode in {'resume', 'dock'} and not map_file.exists():
        raise ValueError(f"{options.mode} requires existing map file {map_file}")
    command_policy = (
        load_exploration_policy(options.policy)
        if options.mode != 'dock'
        else None
    )

    send_command('stop')
    time.sleep(1.0)
    calibration_override = (
        load_calibration(Path(options.calibration))
        if options.calibration
        else None
    )

    strategy_map = {}
    source_map_file = None
    if options.mode in {'resume', 'dock'}:
        source_map_file = map_file
        strategy_map = load_map_data(source_map_file)
        deg_per_yaw, mm_per_wd, x0, y0, heading0 = load_resume_state(source_map_file)
        if calibration_override:
            deg_per_yaw, mm_per_wd = calibration_override
    elif map_file.exists():
        source_map_file = map_file
        strategy_map = load_map_data(source_map_file)
        calibration = strategy_map['calibration']
        deg_per_yaw = calibration['deg_per_yaw']
        mm_per_wd = calibration.get(
            'mm_per_wheel_tick', calibration.get('mm_per_wd')
        )
        if calibration_override:
            deg_per_yaw, mm_per_wd = calibration_override
        print(f"=== Starting from dock with knowledge from {source_map_file} ===")
        if options.docking['init']:
            dock_to_corner(deg_per_yaw, mm_per_wd)
        else:
            print("  Skipping initial dock positioning sequence (docking.init=false)")
        x0, y0, heading0 = map_start_pose(strategy_map)
        print(
            f"  Anchored to saved starting pose: "
            f"({x0:.0f}, {y0:.0f}) mm, heading {heading0:.1f}°"
        )
    else:
        deg_per_yaw, mm_per_wd = calibration_override or load_calibration()
        if options.docking['init']:
            x0, y0 = dock_to_corner(deg_per_yaw, mm_per_wd)
        else:
            print("=== Skipping initial dock positioning sequence (docking.init=false) ===")
            x0, y0 = DOCK_CLEARANCE_MM, DOCK_CLEARANCE_MM
        heading0 = 0.0

    img_path = map_file.with_suffix('.png')
    dashboard_server = None
    live_dashboard = None
    if options.dashboard['active'] and options.mode != 'dock':
        try:
            from apps.map.dashboard import start_dashboard_server
        except ModuleNotFoundError:
            from dashboard import start_dashboard_server
        dashboard_server, live_dashboard, _dashboard_thread = start_dashboard_server(
            options.dashboard['host'],
            options.dashboard['port'],
            'Dash Live Map Dashboard',
            options.territory_size,
        )
        print(
            f"Dashboard listening on "
            f"http://{options.dashboard['host']}:{options.dashboard['port']}"
        )

    try:
        if options.mode == 'dock':
            produced_runs = go_home_with_retries(
                strategy_map,
                deg_per_yaw,
                mm_per_wd,
                strategy=GO_HOME_STRATEGIES[options.go_home_strategy],
            )
            if not produced_runs:
                return
        else:
            produced_runs = [
                explore(
                    deg_per_yaw,
                    mm_per_wd,
                    x0,
                    y0,
                    heading0,
                    strategy_map,
                    duration=options.duration,
                    territory_mm=options.territory_size,
                    command_policy=command_policy,
                    exploration_policy=options.exploration_policy,
                    dashboard=live_dashboard,
                )
            ]
    finally:
        if dashboard_server is not None:
            dashboard_server.shutdown()
            dashboard_server.server_close()
    for run in produced_runs:
        save_map(
            deg_per_yaw,
            mm_per_wd,
            run,
            map_file,
            base_data=strategy_map if source_map_file != map_file else None,
            replace_existing=source_map_file is None,
        )
    try:
        from apps.map.visualize_cells import render_cell_map
    except ModuleNotFoundError:
        from visualize_cells import render_cell_map
    render_cell_map(json.loads(map_file.read_text()), img_path)


if __name__ == "__main__":
    try:
        main()
    finally:
        try:
            send_command("stop")
        except Exception:
            pass
