import unittest

from examples.mapping import conservative_exploration as conservative
from examples.mapping import exploration_walls
from examples.mapping import map_room


class ConservativeExplorationTests(unittest.TestCase):
    def test_fresh_dock_unlocks_territory_inside_room(self):
        policy = conservative.ConservativeExploration([], (80, -80), [], [])

        self.assertEqual(policy.territories, [(0, -1)])
        self.assertEqual(policy.focus, (0, -1))
        self.assertTrue(policy.allows_point(500, -500))
        self.assertFalse(policy.allows_point(500, 500))

    def test_forward_distance_stops_inside_unlocked_territory(self):
        policy = conservative.ConservativeExploration([], (1000, 1000), [], [])
        self.assertEqual(policy.forward_distance(1000, 1000, 0, 3000), 825)

    def test_territory_completes_when_majority_is_unreachable(self):
        visited = [(100, y) for y in (100, 600, 1100, 1600)]
        barrier = [(600, y) for y in (100, 600, 1100, 1600)]
        resolution = conservative.territory_resolution(
            (0, 0), visited, barrier, territory_mm=2000
        )
        self.assertEqual(len(resolution["visited"]), 4)
        self.assertEqual(len(resolution["blocked"]), 4)
        self.assertEqual(len(resolution["unreachable"]), 8)
        self.assertEqual(resolution["frontier"], set())
        self.assertTrue(
            conservative.territory_sufficiently_mapped(
                (0, 0), visited, barrier, territory_mm=2000
            )
        )

    def test_territory_stays_open_when_barrier_has_a_gap(self):
        visited = [(100, y) for y in (100, 600, 1100, 1600)]
        barrier_with_gap = [(600, y) for y in (100, 600, 1600)]
        resolution = conservative.territory_resolution(
            (0, 0),
            visited,
            barrier_with_gap,
            territory_mm=2000,
        )
        self.assertTrue(resolution["frontier"])
        self.assertFalse(
            conservative.territory_sufficiently_mapped(
                (0, 0),
                visited,
                barrier_with_gap,
                territory_mm=2000,
            )
        )

    def test_resume_drift_does_not_unlock_a_new_territory(self):
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, 0]],
                    "focus_territory": [0, 0],
                }
            }
        ]
        policy = conservative.ConservativeExploration(
            runs,
            (611, -78),
            [(611, -78)],
            [],
        )
        self.assertEqual(policy.territories, [(0, 0)])
        self.assertEqual(policy.focus, (0, 0))

    def test_resume_restores_persisted_negative_territory(self):
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, 0], [0, -1]],
                    "focus_territory": [0, -1],
                }
            }
        ]
        policy = conservative.ConservativeExploration(
            runs,
            (1691, -255),
            [(1691, -255)],
            [],
        )
        self.assertEqual(policy.territories, [(0, 0), (0, -1)])
        self.assertEqual(policy.focus, (0, -1))

    def test_heading_preference_targets_reachable_unvisited_cells(self):
        path = [(250, 250), (750, 250)]
        blockers = [(250, 750), (750, 750)]
        policy = conservative.ConservativeExploration(
            [], (250, 250), path, blockers, territory_mm=2000
        )
        self.assertGreater(
            policy.heading_preference(750, 250, 0),
            policy.heading_preference(750, 250, 180),
        )

    def test_heading_preference_excludes_unreachable_cells(self):
        path = [(250, y) for y in (250, 750, 1250, 1750)]
        barrier = [(750, y) for y in (250, 750, 1250, 1750)]
        policy = conservative.ConservativeExploration(
            [], (250, 250), path, barrier, territory_mm=2000
        )
        self.assertLess(
            policy.heading_preference(250, 250, 0),
            policy.heading_preference(250, 250, 90),
        )

    def test_frontier_preference_recovers_from_saved_boundary_turn_loop(self):
        x, y, heading = 158.9, -162.0, -166.0
        path = [(250, 250), (750, 250), (1250, 250), (1750, 250)]
        policy = conservative.ConservativeExploration([], (x, y), path, [])
        angle = map_room.choose_exploration_angle(
            x,
            y,
            heading,
            path,
            [],
            require_turn=True,
            point_allowed=policy.allows_point,
            heading_preference=policy.heading_preference,
        )
        chosen_heading = map_room.normalize_heading(heading + angle)
        self.assertGreaterEqual(
            policy.forward_distance(x, y, chosen_heading, 2000),
            conservative.MIN_USEFUL_FORWARD_MM,
        )
        self.assertNotIn(angle, {-45, 45})

    def test_nearby_wall_points_form_shared_planning_segment(self):
        nearby = [(500, 100), (500, 400)]
        distant = [(500, 100), (500, 401)]
        self.assertEqual(len(exploration_walls.inferred_wall_segments(nearby)), 1)
        self.assertEqual(exploration_walls.inferred_wall_segments(distant), [])

    def test_inferred_wall_segment_blocks_crossing_without_blocking_around_it(self):
        wall_segments = exploration_walls.inferred_wall_segments(
            [(500, 100), (500, 400)]
        )
        self.assertTrue(
            exploration_walls.segment_crosses_wall(
                (250, 250),
                (750, 250),
                wall_segments,
            )
        )
        self.assertFalse(
            exploration_walls.segment_crosses_wall(
                (250, 750),
                (750, 750),
                wall_segments,
            )
        )

    def test_inferred_wall_near_transition_start_blocks_crossing(self):
        wall_segments = [((150, 190), (350, 190))]
        self.assertTrue(
            exploration_walls.segment_crosses_wall(
                (250, 250),
                (250, 750),
                wall_segments,
            )
        )

    def test_wall_samples_resolve_cells_behind_wall_and_unlock_south(self):
        bottom_row = [(x, 250) for x in (250, 750, 1250, 1750)]
        south_path = [(750, -250)]
        wall_segments = [
            ((x - 100, 190), (x + 100, 190))
            for x in (250, 750, 1250, 1750)
        ]
        resolution = conservative.territory_resolution(
            (0, 0),
            bottom_row + south_path,
            [],
            wall_segments,
            territory_mm=2000,
        )
        self.assertEqual(resolution["visited"], {(0, 0), (1, 0), (2, 0), (3, 0)})
        self.assertEqual(len(resolution["unreachable"]), 12)
        self.assertFalse(resolution["frontier"])

        policy = conservative.ConservativeExploration(
            [],
            bottom_row[0],
            bottom_row + south_path,
            [],
            wall_segments,
            territory_mm=2000,
        )
        policy.unlock_if_complete()
        self.assertEqual(policy.focus, (0, -1))

    def test_completed_focus_selects_existing_unfinished_south_territory(self):
        bottom_row = [(x, 250) for x in (250, 750, 1250, 1750)]
        south_path = [(750, -250)]
        wall_segments = [
            ((x - 100, 190), (x + 100, 190))
            for x in (250, 750, 1250, 1750)
        ]
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, 0], [0, -1]],
                    "focus_territory": [0, 0],
                }
            }
        ]
        policy = conservative.ConservativeExploration(
            runs,
            bottom_row[0],
            bottom_row + south_path,
            [],
            wall_segments,
            territory_mm=2000,
        )
        policy.unlock_if_complete()
        self.assertEqual(policy.focus, (0, -1))
        self.assertEqual(policy.territories, [(0, 0), (0, -1)])

    def test_heading_preference_avoids_inferred_wall_segment(self):
        path = [(250, 250)]
        walls = [(500, 350), (700, 350)]
        policy = conservative.ConservativeExploration(
            [],
            (250, 250),
            path,
            walls,
            exploration_walls.inferred_wall_segments(walls),
            territory_mm=2000,
        )
        self.assertLess(
            policy.heading_preference(250, 250, 0),
            policy.heading_preference(250, 250, 90),
        )


if __name__ == "__main__":
    unittest.main()
