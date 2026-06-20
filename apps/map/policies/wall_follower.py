"""Arc-based wall-following exploration.

Unlike the heading policies (conservative / coverage / novelty), which only pick
a direction, this is a self-contained driving loop. It steers the active
``_ExplorationRun`` through its motion primitives:

1. drive forward until a wall stops a leg,
2. face along the nearest inferred wall segment (wall kept on the chosen side),
3. arc around it until the obstacle-aware arc halts at the wall ahead,
4. measure how much of the sweep ran,
5. a full circle with no obstacle means the wall fell away -> drive forward to
   find the next one,
6. otherwise re-face and arc again.

Geometry here is self-contained (no import from main) to keep the dependency
one-way: main imports this module, not the reverse.
"""

import math
import time

try:
    from apps.map.policies.exploration_policy_base import ExplorationPolicy
except ModuleNotFoundError:
    from policies.exploration_policy_base import ExplorationPolicy

DEFAULT_RADIUS_MM = 250
DEFAULT_ARC_DEG = 360
DEFAULT_WALL_ON_LEFT = True  # keep the wall on the left -> arc clockwise (CW)

# Corner-escape guard: an arc that halts on an obstacle after sweeping less than
# WEDGE_FRACTION of its commanded angle made effectively no progress. That many
# in a row means the robot is wedged in a corner (re-facing then yields a tiny
# turn and it grinds in place), so it backs off and turns toward open space.
WEDGE_FRACTION = 0.05
WEDGE_STALL_LIMIT = 2


def _normalize_heading(heading):
    """Normalize a heading to [-180, 180)."""
    return (heading + 180) % 360 - 180


def arc_pose_delta(start_heading_deg, dtheta_deg, arc_length_mm):
    """World-frame ``(dx, dy, new_heading_deg)`` for a constant-radius arc.

    The robot starts at ``start_heading_deg`` and sweeps ``dtheta_deg`` (map
    frame, +CCW) while travelling ``arc_length_mm`` along the curve. Integrates
    the curved path exactly -- a plain move()/turn() can't, which is why arc
    odometry needs this -- and reduces to a straight step as ``dtheta -> 0``.
    """
    h0 = math.radians(start_heading_deg)
    dth = math.radians(dtheta_deg)
    if abs(dth) < 1e-6:
        forward, lateral = arc_length_mm, 0.0
    else:
        radius = arc_length_mm / dth
        forward = radius * math.sin(dth)
        lateral = radius * (1.0 - math.cos(dth))
    dx = forward * math.cos(h0) - lateral * math.sin(h0)
    dy = forward * math.sin(h0) + lateral * math.cos(h0)
    return dx, dy, _normalize_heading(start_heading_deg + dtheta_deg)


def nearest_point_on_segment(point, segment):
    """Closest point on a line segment to ``point`` (all (x, y) tuples)."""
    (ax, ay), (bx, by) = segment
    tx, ty = bx - ax, by - ay
    length_sq = tx * tx + ty * ty
    if length_sq == 0:
        return (ax, ay)
    s = ((point[0] - ax) * tx + (point[1] - ay) * ty) / length_sq
    s = max(0.0, min(1.0, s))
    return (ax + s * tx, ay + s * ty)


def wall_follow_heading(robot_xy, segment, wall_on_left=True):
    """Heading (deg) to run tangent to ``segment`` with the wall on one side.

    Of the wall's two tangent directions, pick the one that keeps the wall on
    the robot's left (or right) -- ``segment`` is the nearest inferred wall.
    """
    (ax, ay), (bx, by) = segment
    theta = math.atan2(by - ay, bx - ax)
    near = nearest_point_on_segment(robot_xy, segment)
    to_wall = (near[0] - robot_xy[0], near[1] - robot_xy[1])
    left_normal = (-math.sin(theta), math.cos(theta))
    on_left = to_wall[0] * left_normal[0] + to_wall[1] * left_normal[1] > 0
    if on_left != wall_on_left:
        theta += math.pi
    return _normalize_heading(math.degrees(theta))


class WallFollower(ExplorationPolicy):
    """Drive an exploration run along walls using arcs (see module docstring).

    It is a self-driving controller rather than a heading-preference policy, but
    inherits ``ExplorationPolicy`` so it shares the common base. The abstract
    ``heading_preference`` is unused (the loop in ``follow`` decides motion), so
    it returns a neutral 0.
    """

    name = 'wall-follower'
    metadata_key = 'wall_follower'

    def __init__(
        self,
        radius_mm=DEFAULT_RADIUS_MM,
        arc_deg=DEFAULT_ARC_DEG,
        wall_on_left=DEFAULT_WALL_ON_LEFT,
    ):
        self.radius_mm = radius_mm
        self.arc_deg = arc_deg
        self.wall_on_left = wall_on_left

    def heading_preference(self, x, y, heading):
        """Unused: the wall follower drives itself via follow()."""
        return 0.0

    def describe(self):
        side = 'left' if self.wall_on_left else 'right'
        return (
            f"  Wall follower: arc R={self.radius_mm}mm around walls "
            f"(wall on the {side})."
        )

    def metadata(self):
        return {
            'radius_mm': self.radius_mm,
            'arc_deg': self.arc_deg,
            'wall_on_left': self.wall_on_left,
        }

    def target_heading(self, robot_xy, wall_segments, max_distance=None):
        """Heading to face along the nearest wall, or None if none is near.

        ``max_distance`` (mm) restricts the match to walls close to the robot --
        a far dock wall is not the one it just bumped, so it shouldn't steer by
        it. With no limit, the single nearest segment wins.
        """
        best, best_distance = None, float('inf')
        if max_distance is not None:
            best_distance = max_distance ** 2
        for segment in wall_segments:
            near = nearest_point_on_segment(robot_xy, segment)
            distance = (near[0] - robot_xy[0]) ** 2 + (near[1] - robot_xy[1]) ** 2
            if distance < best_distance:
                best_distance, best = distance, segment
        if best is None:
            return None
        return wall_follow_heading(robot_xy, best, self.wall_on_left)

    def drive(self, run, duration):
        """Self-driving policy: run the wall-following loop (no initial orient
        turn -- it just drives forward to the first wall)."""
        self.follow(run, time.time() + duration)

    def follow(self, run, end_time):
        """Run the wall-following loop against ``run`` until ``end_time``."""
        # Wall on the left -> arc clockwise (negative command angle).
        arc_angle = -self.arc_deg if self.wall_on_left else self.arc_deg
        while time.time() < end_time and not run.quality['tracking_lost']:
            if not run.drive_forward_to_wall(end_time):
                break
            stalls = 0
            while time.time() < end_time and not run.quality['tracking_lost']:
                # Only steer by a wall close enough to be the one just hit; a far
                # dock wall would aim us back across the room.
                target = self.target_heading(
                    (run.x, run.y), run.known_wall_segments, max_distance=self.radius_mm * 2
                )
                if target is None:
                    # First contact (or a still-sparse map): a head-on stop means
                    # the wall is ~perpendicular to our heading. Turn 90 deg to
                    # put it on the chosen side, then arc along it. wall_on_left
                    # -> turn right (CW); wall_on_right -> turn left (CCW).
                    turn = -90.0 if self.wall_on_left else 90.0
                    target = _normalize_heading(run.heading + turn)
                run.turn_to_heading(target, reason='face wall')
                outcome = run.arc_leg(self.radius_mm, arc_angle)
                print(
                    f"  [wall-follow] arc {outcome.get('completed_angle_deg')}"
                    f"\N{DEGREE SIGN} ({outcome.get('completed_fraction', 1):.0%}), "
                    f"halt={outcome.get('halt')}"
                )
                # A full circle with no obstacle: the wall fell away -> go back
                # to driving forward to find the next one.
                if outcome.get('halt') != 'obstacle':
                    break
                # An arc that barely swept before hitting the wall made no
                # progress; a run of them means we're wedged in a corner. Escape
                # rather than grind, then go back to driving forward.
                if outcome.get('completed_fraction', 1.0) < WEDGE_FRACTION:
                    stalls += 1
                    if stalls >= WEDGE_STALL_LIMIT:
                        self.escape(run)
                        break
                else:
                    stalls = 0

    def escape(self, run):
        """Unwedge from a corner: back off, then turn toward open space (away
        from the followed wall) so the next forward leg heads somewhere new."""
        print("  [wall-follow] wedged in a corner -- backing off to escape")
        run.back_away()
        # Wall kept on the left -> open space is to the right; turn that way.
        turn = -90.0 if self.wall_on_left else 90.0
        run.turn_to_heading(_normalize_heading(run.heading + turn), reason='escape')
