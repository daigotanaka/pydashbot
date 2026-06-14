import unittest

from examples.mapping import conservative_exploration as conservative
from examples.mapping.coverage_exploration import STALL_LEGS, CoverageExploration
from examples.mapping.exploration_policy import ExplorationPolicy

FULLY_EXPLORED_SOUTH = [
    (x, y)
    for x in (125, 375, 625, 875)
    for y in (-875, -625, -375, -125)
]


class CoverageExplorationTests(unittest.TestCase):
    def test_is_an_exploration_policy(self):
        policy = CoverageExploration([], (80, -80), [], [], territory_mm=1000)
        self.assertIsInstance(policy, ExplorationPolicy)
        self.assertIsInstance(policy, conservative.ConservativeExploration)

    def test_abandons_a_focus_it_cannot_enter(self):
        # Reproduces the stuck run: focus (0, 0) is north of the dock wall, so
        # the robot never reaches a cell there (visited stays 0, cells stay
        # frontier). The unreachable-marking rule cannot relinquish it; the
        # no-progress signal must, after STALL_LEGS legs with no new cells.
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, -1], [0, 0]],
                    "focus_territory": [0, 0],
                }
            }
        ]
        policy = CoverageExploration(
            runs,
            FULLY_EXPLORED_SOUTH[0],
            FULLY_EXPLORED_SOUTH,
            [],
            territory_mm=1000,
        )
        self.assertEqual(policy.focus, (0, 0))

        for _ in range(STALL_LEGS):
            policy.unlock_if_complete()

        self.assertIn((0, 0), policy.abandoned)
        # Territory creation is position-driven (expand_past_boundary), so a
        # stalled focus must NOT abstractly unlock new (likely unreachable)
        # territories -- that runaway is what trapped the robot.
        self.assertEqual(policy.territories, [(0, -1), (0, 0)])

    def test_resets_stall_when_focus_gains_a_cell(self):
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, -1]],
                    "focus_territory": [0, -1],
                }
            }
        ]
        # Start with one visited cell; the focus has a real frontier.
        path = [(125, -875)]
        policy = CoverageExploration(runs, path[0], path, [], territory_mm=1000)
        policy.unlock_if_complete()
        policy.unlock_if_complete()
        # A new cell gets visited (progress) -> the stall counter resets.
        path.append((375, -875))
        policy.unlock_if_complete()
        self.assertEqual(policy._stall_legs, 0)
        self.assertNotIn((0, -1), policy.abandoned)

    def test_heading_preference_prefers_entering_a_new_cell(self):
        # Two cells visited along the south edge; the cell to the north is a
        # reachable frontier. Heading north (into the new cell) must beat
        # heading back over the already-visited cell to the west.
        path = [(375, -875), (625, -875)]
        policy = CoverageExploration([], path[0], path, [], territory_mm=1000)
        toward_new_cell = policy.heading_preference(375, -875, 90)   # +y, into frontier
        over_visited = policy.heading_preference(375, -875, 180)     # -x, over visited
        self.assertGreater(toward_new_cell, over_visited)

    def test_forward_leg_stops_before_reentering_completed_territory(self):
        # (0, -1) is fully explored; the robot stands in still-uncharted (0, -2).
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, -2], [0, -1]],
                    "focus_territory": [0, -2],
                }
            }
        ]
        policy = CoverageExploration(
            runs, (500, -1500), FULLY_EXPLORED_SOUTH, [], territory_mm=1000
        )
        self.assertTrue(policy.territory_explored((0, -1)))

        parent_forward_distance = conservative.ConservativeExploration.forward_distance

        # Heading north re-enters completed (0, -1): the leg is clamped at the
        # boundary, well short of where the unlocked-region clamp would allow.
        north = policy.forward_distance(500, -1500, 90, 2000)
        north_unclamped = parent_forward_distance(policy, 500, -1500, 90, 2000)
        self.assertLess(north, north_unclamped)
        self.assertLess(north, 600)

        # Heading deeper into the uncharted current territory is not extra-clamped.
        south = policy.forward_distance(500, -1500, -90, 2000)
        south_unclamped = parent_forward_distance(policy, 500, -1500, -90, 2000)
        self.assertEqual(south, south_unclamped)

    def test_heading_into_completed_territory_scores_as_no_progress(self):
        # The clamp makes a heading into finished territory read as no-progress,
        # so it loses to a heading into the uncharted current territory.
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, -2], [0, -1]],
                    "focus_territory": [0, -2],
                }
            }
        ]
        policy = CoverageExploration(
            runs, (500, -1500), FULLY_EXPLORED_SOUTH, [], territory_mm=1000
        )
        into_completed = policy.heading_preference(500, -1100, 90)   # north into (0,-1)
        into_uncharted = policy.heading_preference(500, -1100, -90)  # south, stays in (0,-2)
        self.assertGreater(into_uncharted, into_completed)

    def test_does_not_unlock_new_territories_abstractly(self):
        # Both unlocked territories are finished and there is no adjacent
        # unfinished one; coverage must not abstractly grow the territory set
        # (that is left to position-driven expand_past_boundary).
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, -1], [0, 0]],
                    "focus_territory": [0, 0],
                }
            }
        ]
        both_filled = FULLY_EXPLORED_SOUTH + [
            (cx * 250 + 125, cy * 250 + 125)
            for cx in range(4)
            for cy in range(4)
        ]
        policy = CoverageExploration(
            runs, FULLY_EXPLORED_SOUTH[0], both_filled, [], territory_mm=1000
        )
        before = list(policy.territories)

        policy.unlock_if_complete()

        self.assertEqual(policy.territories, before)

    def test_aims_toward_live_frontier_not_abandoned_direction(self):
        # (0, -1) fully explored and is the focus; nothing left to gain there.
        # With north (0, 0) given up, heading from the center toward the live
        # south frontier (0, -2) must beat heading toward the abandoned north.
        policy = CoverageExploration(
            [], FULLY_EXPLORED_SOUTH[0], FULLY_EXPLORED_SOUTH, [], territory_mm=1000
        )
        self.assertTrue(policy.territory_explored((0, -1)))
        policy.abandoned.add((0, 0))

        toward_live_south = policy.heading_preference(500, -500, -90)
        toward_abandoned_north = policy.heading_preference(500, -500, 90)
        self.assertGreater(toward_live_south, toward_abandoned_north)

    def test_metadata_records_objective_and_abandoned(self):
        policy = CoverageExploration([], (80, -80), [], [], territory_mm=1000)
        policy.abandoned.add((9, 9))
        meta = policy.metadata()
        self.assertEqual(meta['objective'], 'coverage')
        self.assertIn([9, 9], [list(cell) for cell in meta['abandoned']])


if __name__ == "__main__":
    unittest.main()
