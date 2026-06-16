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

    def test_cells_crossed_between_path_endpoints_are_visited(self):
        sparse_path = [(100, -100), (900, -100)]
        dense_path = conservative.densify_path(sparse_path, 125)

        resolution = conservative.territory_resolution(
            (0, -1), dense_path, [], territory_mm=1000
        )

        self.assertEqual(
            resolution["visited"],
            {(0, 3), (1, 3), (2, 3), (3, 3)},
        )

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

    def test_inferred_wall_segments_do_not_fill_cluster_with_chords(self):
        points = [(0, 0), (100, 0), (200, 0)]

        self.assertEqual(
            exploration_walls.inferred_wall_segments(points),
            [((0, 0), (100, 0)), ((100, 0), (200, 0))],
        )

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

    def test_inferred_wall_crossing_near_transition_start_blocks_connection(self):
        wall_segments = [((150, 260), (350, 260))]
        self.assertTrue(
            exploration_walls.segment_intersects_wall(
                (250, 250),
                (250, 750),
                wall_segments,
            )
        )

    def test_disjoint_collinear_wall_does_not_block_connection(self):
        self.assertFalse(
            exploration_walls.segment_intersects_wall(
                (0, 0),
                (100, 0),
                [((200, 0), (300, 0))],
            )
        )

    def test_nearby_wall_does_not_cut_off_reachable_neighbor(self):
        # Regression from data/room_map.json: this short wall segment is wholly
        # inside neighboring cell (2, 3), ~136mm from the (1, 3)-(1, 2)
        # connection -- still outside CORRIDOR_HALF_CLEARANCE_MM (125mm), so it
        # is close enough for conservative motion avoidance, but it does not
        # separate visited (1, 3) from (1, 2).
        wall_segments = [
            ((510.9, -1176.9), (542.7, -1079.2)),
        ]
        resolution = conservative.territory_resolution(
            (0, -2),
            [(375, -1125)],
            [(625, -1125)],
            wall_segments,
            territory_mm=1000,
        )

        self.assertIn((1, 2), resolution["frontier"])
        self.assertIn((2, 2), resolution["frontier"])
        self.assertIn((1, 1), resolution["frontier"])

    def test_wall_too_close_to_visited_cell_cuts_off_territory(self):
        # From data/room_map.json: this short wall sits only ~12-21mm from the
        # connections out of the visited entry cell. The robot is measured at
        # 180-190mm wide and needs a real MIN_CORRIDOR_OPENING_MM-wide gap (see
        # conservative_exploration.py) to trust a passage, so a wall this close
        # genuinely blocks both connections -- it does not merely sit "beside"
        # the path with room to spare.
        wall_segments = [
            ((-148.3, 14.5), (-142.6, -113.2)),
        ]
        resolution = conservative.territory_resolution(
            (-1, -1),
            [(-125, -125)],
            [],
            wall_segments,
            territory_mm=1000,
        )

        self.assertIn((2, 3), resolution["unreachable"])
        self.assertIn((3, 2), resolution["unreachable"])
        self.assertIn((0, 0), resolution["unreachable"])

    def test_wall_samples_resolve_cells_behind_wall_and_unlock_south(self):
        bottom_row = [(x, 250) for x in (250, 750, 1250, 1750)]
        south_path = [(750, -250)]
        wall_segments = [
            ((x - 100, 500), (x + 100, 500))
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
            ((x - 100, 500), (x + 100, 500))
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

    def test_expands_into_territory_ahead_when_current_is_complete(self):
        # Reproduces the saved scenario: focus (0, 0) still has frontier, but
        # the territory the robot is in, (0, -1), is fully mapped. Heading into
        # the un-unlocked (0, -2) must unlock it rather than stall at the wall.
        covered = [
            (x, y)
            for x in (125, 375, 625, 875)
            for y in (-875, -625, -375, -125)
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
            covered[0],
            covered,
            [],
            territory_mm=1000,
        )

        unlocked = policy.expand_past_boundary(500, -950, -90)

        self.assertTrue(unlocked)
        self.assertEqual(policy.territories, [(0, 0), (0, -1), (0, -2)])
        self.assertEqual(policy.focus, (0, -2))

    def test_does_not_expand_when_current_territory_incomplete(self):
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, 0], [0, -1]],
                    "focus_territory": [0, 0],
                }
            }
        ]
        # Only one cell of (0, -1) visited: it still has frontier.
        policy = conservative.ConservativeExploration(
            runs,
            (500, -500),
            [(500, -500)],
            [],
            territory_mm=1000,
        )

        self.assertFalse(policy.expand_past_boundary(500, -950, -90))
        self.assertEqual(policy.territories, [(0, 0), (0, -1)])
        self.assertEqual(policy.focus, (0, 0))

    def test_does_not_re_expand_into_already_unlocked_territory(self):
        # (0, -1) complete and unlocked; heading back into it must not re-add it.
        covered = [
            (x, y)
            for x in (125, 375, 625, 875)
            for y in (-875, -625, -375, -125)
        ]
        policy = conservative.ConservativeExploration(
            [],
            covered[0],
            covered,
            [],
            territory_mm=1000,
        )
        policy.territories = [(0, 0), (0, -1)]
        policy.focus = (0, 0)

        # From (0, -1) heading north (+90) crosses into already-unlocked (0, 0).
        self.assertFalse(policy.expand_past_boundary(500, -50, 90))
        self.assertEqual(policy.territories, [(0, 0), (0, -1)])

    def test_completed_dead_end_focus_expands_from_best_explored_territory(self):
        # (0, -1) is fully explored and reachable; the focus (0, 0) "completed"
        # by going unreachable (3 visited, the rest blocked). Unlocking the next
        # territory must grow from (0, -1), not march further north past (0, 0).
        explored = [
            (x, y)
            for x in (125, 375, 625, 875)
            for y in (-875, -625, -375, -125)
        ]
        dead_end_visited = [(125, 125), (375, 125), (625, 125)]
        # Block every other cell of (0, 0) so it has no frontier left.
        dead_end_blocked = [
            (cx * 250 + 125, cy * 250 + 125)
            for cx in range(4)
            for cy in range(4)
            if (cx, cy) not in {(0, 0), (1, 0), (2, 0)}
        ]
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, -1], [0, 0]],
                    "focus_territory": [0, 0],
                }
            }
        ]
        policy = conservative.ConservativeExploration(
            runs,
            explored[0],
            explored + dead_end_visited,
            dead_end_blocked,
            territory_mm=1000,
        )
        self.assertTrue(
            conservative.territory_sufficiently_mapped(
                (0, 0), policy.path_points, policy.blockers, policy.wall_segments,
                1000,
            )
        )

        policy.unlock_if_complete()

        north_dead_region = {(1, 0), (-1, 0), (0, 1)}
        self.assertNotIn(policy.focus, north_dead_region)
        self.assertIn(policy.focus, {(0, -2), (1, -1), (-1, -1)})

    def test_focus_with_no_frontier_unlocks_despite_few_visited_cells(self):
        # Reproduces the stuck run: focus (0, 0) has only 2 reachable cells (the
        # rest unreachable behind a wall), so frontier is empty but visited < 3.
        # It must still count as done so exploration can move on, instead of
        # trapping focus on the dead-end forever.
        explored = [
            (x, y)
            for x in (125, 375, 625, 875)
            for y in (-875, -625, -375, -125)
        ]
        dead_end_visited = [(125, 125), (375, 125)]  # only 2 cells of (0, 0)
        dead_end_blocked = [
            (cx * 250 + 125, cy * 250 + 125)
            for cx in range(4)
            for cy in range(4)
            if (cx, cy) not in {(0, 0), (1, 0)}
        ]
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, -1], [0, 0]],
                    "focus_territory": [0, 0],
                }
            }
        ]
        policy = conservative.ConservativeExploration(
            runs,
            explored[0],
            explored + dead_end_visited,
            dead_end_blocked,
            territory_mm=1000,
        )
        # Strict reporting bar still treats it as not-sufficiently-mapped...
        self.assertFalse(
            conservative.territory_sufficiently_mapped(
                (0, 0), policy.path_points, policy.blockers, policy.wall_segments,
                1000,
            )
        )
        # ...but it is "explored" (no reachable frontier), so unlock fires.
        self.assertTrue(policy.territory_explored((0, 0)))

        policy.unlock_if_complete()

        self.assertGreater(len(policy.territories), 2)
        self.assertNotIn(policy.focus, {(0, 0), (0, -1)})
        self.assertNotIn(policy.focus, {(1, 0), (-1, 0), (0, 1)})

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
