import math
import time
import unittest

from apps.map.policies.exploration.wall_follower import (
    WallFollower,
    arc_pose_delta,
    nearest_point_on_segment,
    wall_follow_heading,
)


class WallFollowerGeometryTests(unittest.TestCase):
    def test_arc_pose_delta_curves_and_reduces_to_straight(self):
        # 90 deg left arc of radius 100 (arc length R*pi/2) from origin/heading 0
        # ends at (100, 100) facing +90.
        dx, dy, h = arc_pose_delta(0, 90, 100 * math.pi / 2)
        self.assertAlmostEqual(dx, 100, places=3)
        self.assertAlmostEqual(dy, 100, places=3)
        self.assertAlmostEqual(h, 90)
        # The same arc started facing +90 (north) curves to the west.
        dx, dy, h = arc_pose_delta(90, 90, 100 * math.pi / 2)
        self.assertAlmostEqual(dx, -100, places=3)
        self.assertAlmostEqual(dy, 100, places=3)
        self.assertAlmostEqual(h, -180)  # normalize is half-open [-180, 180)
        # dtheta -> 0 is a straight step along the heading.
        dx, dy, h = arc_pose_delta(0, 0, 50)
        self.assertAlmostEqual((dx, dy, h), (50.0, 0.0, 0.0))

    def test_nearest_point_on_segment_clamps_to_ends(self):
        seg = ((0.0, 0.0), (100.0, 0.0))
        self.assertEqual(nearest_point_on_segment((50.0, 20.0), seg), (50.0, 0.0))
        self.assertEqual(nearest_point_on_segment((-30.0, 5.0), seg), (0.0, 0.0))
        self.assertEqual(nearest_point_on_segment((130.0, 5.0), seg), (100.0, 0.0))

    def test_wall_follow_heading_keeps_wall_on_chosen_side(self):
        wall = ((0.0, 0.0), (100.0, 0.0))  # wall along +x
        # Robot below the wall -> to keep the wall on its LEFT it faces +x (0).
        self.assertAlmostEqual(wall_follow_heading((50.0, -10.0), wall, True), 0.0)
        # Robot above the wall -> faces -x (180) to keep the wall on its left.
        self.assertAlmostEqual(abs(wall_follow_heading((50.0, 10.0), wall, True)), 180.0)
        # Wall on the right flips the direction.
        self.assertAlmostEqual(abs(wall_follow_heading((50.0, -10.0), wall, False)), 180.0)


class FakeRun:
    """Minimal stand-in exposing what WallFollower.follow drives."""

    def __init__(self, forward_results, arc_results, wall_segments):
        self.quality = {'tracking_lost': False}
        self.x, self.y = 50.0, -10.0
        self.heading = 0.0
        self.known_wall_segments = wall_segments
        self._forward = iter(forward_results)
        self._arcs = iter(arc_results)
        self.forward_calls = 0
        self.back_away_calls = 0
        self.turns = []
        self.arcs = []

    def drive_forward_to_wall(self, end_time):
        self.forward_calls += 1
        try:
            return next(self._forward)
        except StopIteration:
            return False

    def turn_to_heading(self, target, reason='align'):
        self.turns.append(target)

    def arc_leg(self, radius_mm, angle_deg):
        self.arcs.append((radius_mm, angle_deg))
        return next(self._arcs)

    def back_away(self, distance_mm=200):
        self.back_away_calls += 1


class WallFollowerLoopTests(unittest.TestCase):
    def test_target_heading_uses_nearest_wall(self):
        follower = WallFollower(wall_on_left=True)
        far = ((0.0, 500.0), (100.0, 500.0))
        near = ((0.0, 0.0), (100.0, 0.0))
        # Robot just south of `near`; should align along it (+x), wall on left.
        self.assertAlmostEqual(
            follower.target_heading((50.0, -10.0), [far, near]), 0.0
        )
        self.assertIsNone(follower.target_heading((0.0, 0.0), []))

    def test_follow_arcs_until_a_full_circle_then_drives_on(self):
        wall = [((0.0, 0.0), (100.0, 0.0))]
        run = FakeRun(
            forward_results=[True],  # find a wall once, then time out
            arc_results=[
                {'halt': 'obstacle', 'completed_angle_deg': 40, 'completed_fraction': 0.4},
                {'halt': 'obstacle', 'completed_angle_deg': 55, 'completed_fraction': 0.6},
                {'halt': 'completed', 'completed_angle_deg': 360, 'completed_fraction': 1.0},
            ],
            wall_segments=wall,
        )
        WallFollower().follow(run, time.time() + 100)

        # Re-faced and arced until the full-circle (no obstacle) arc, then the
        # next forward returned False (timeout) and the loop ended.
        self.assertEqual(len(run.arcs), 3)
        self.assertEqual(len(run.turns), 3)
        # Wall on the left -> arc clockwise (negative command angle).
        self.assertTrue(all(angle < 0 for _radius, angle in run.arcs))
        self.assertEqual(run.forward_calls, 2)

    def test_follow_escapes_a_corner_after_repeated_no_progress_arcs(self):
        wall = [((0.0, 0.0), (100.0, 0.0))]
        run = FakeRun(
            forward_results=[True],  # find a wall once, then time out
            arc_results=[
                {'halt': 'obstacle', 'completed_angle_deg': 3, 'completed_fraction': 0.01},
                {'halt': 'obstacle', 'completed_angle_deg': 4, 'completed_fraction': 0.012},
            ],
            wall_segments=wall,
        )
        WallFollower().follow(run, time.time() + 100)

        # Two no-progress arcs in a row trip the wedge guard: back off once and
        # break out to driving forward (which then times out and ends the run).
        self.assertEqual(len(run.arcs), 2)
        self.assertEqual(run.back_away_calls, 1)
        self.assertEqual(run.forward_calls, 2)

    def test_a_productive_arc_resets_the_wedge_counter(self):
        wall = [((0.0, 0.0), (100.0, 0.0))]
        run = FakeRun(
            forward_results=[True],
            arc_results=[
                {'halt': 'obstacle', 'completed_angle_deg': 3, 'completed_fraction': 0.01},
                {'halt': 'obstacle', 'completed_angle_deg': 40, 'completed_fraction': 0.4},
                {'halt': 'obstacle', 'completed_angle_deg': 3, 'completed_fraction': 0.01},
                {'halt': 'completed', 'completed_angle_deg': 360, 'completed_fraction': 1.0},
            ],
            wall_segments=wall,
        )
        WallFollower().follow(run, time.time() + 100)

        # Single stalls separated by a productive arc never reach the limit, so
        # the robot keeps following instead of escaping.
        self.assertEqual(run.back_away_calls, 0)
        self.assertEqual(len(run.arcs), 4)

    def test_follow_stops_when_tracking_is_lost(self):
        run = FakeRun([True], [], [((0.0, 0.0), (100.0, 0.0))])
        run.quality['tracking_lost'] = True
        WallFollower().follow(run, time.time() + 100)
        self.assertEqual(run.arcs, [])


if __name__ == '__main__':
    unittest.main()
