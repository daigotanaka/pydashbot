import math
import unittest

from apps.map.policies import conservative_exploration as conservative
from apps.map.policies.coverage_exploration import STALL_LEGS, CoverageExploration
from apps.map.policies.exploration_policy_base import ExplorationPolicy

FULLY_EXPLORED_SOUTH = [
    (x, y)
    for x in (125, 375, 625, 875)
    for y in (-875, -625, -375, -125)
]

# Start territory (0,0) fully covered, in the map frame where the room grows
# toward +x/+y. The two dock walls lie along the axes through the corner.
FULLY_EXPLORED_START = [
    (x, y)
    for x in (125, 375, 625, 875)
    for y in (125, 375, 625, 875)
]
DOCK_WALLS = [((0.0, 0.0), (0.0, 20000.0)), ((0.0, 0.0), (20000.0, 0.0))]


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

    def test_does_not_abandon_a_physically_entered_territory(self):
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, -2]],
                    "focus_territory": [0, -2],
                }
            }
        ]
        path = [(375, -1125)]
        policy = CoverageExploration(runs, path[0], path, [], territory_mm=1000)

        for _ in range(STALL_LEGS + 1):
            policy.unlock_if_complete()

        self.assertNotIn((0, -2), policy.abandoned)

    def test_resume_reconsiders_abandoned_territory_that_was_entered(self):
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, -2]],
                    "focus_territory": [0, -2],
                    "abandoned": [[0, -2]],
                }
            }
        ]

        policy = CoverageExploration(
            runs, (375, -1125), [(375, -1125)], [], territory_mm=1000
        )

        self.assertNotIn((0, -2), policy.abandoned)
        self.assertFalse(policy.territory_explored((0, -2)))

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

    def test_can_transit_into_completed_active_expansion_source(self):
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, -1], [0, 0]],
                    "focus_territory": [0, -1],
                    "active_territory_expansion": [[0, 0], [-1, 0]],
                }
            }
        ]
        north_filled = [
            (cx * 250 + 125, cy * 250 + 125)
            for cx in range(4)
            for cy in range(4)
        ]
        path = list(FULLY_EXPLORED_SOUTH) + north_filled + [(500, -500)]
        policy = CoverageExploration(
            runs, path[-1], path, [], territory_mm=1000
        )

        self.assertTrue(policy.territory_explored((0, 0)))
        self.assertGreater(
            policy.forward_distance(500, -500, 90, 2000),
            1000,
        )

    def test_does_not_plan_expansion_while_unlocked_territory_is_unfinished(self):
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, -1], [0, 0]],
                    "focus_territory": [0, 0],
                    "active_territory_expansion": [[0, -1], [-1, -1]],
                }
            }
        ]
        path = list(FULLY_EXPLORED_SOUTH) + [(500, 250)]
        policy = CoverageExploration(
            runs, path[-1], path, [], territory_mm=1000
        )

        policy.unlock_if_complete()

        self.assertIsNone(policy.active_territory_expansion)
        self.assertLess(
            policy.forward_distance(500, 250, -90, 2000),
            500,
        )

    def test_does_not_expand_behind_dock_walls(self):
        # Start territory (0,0) is fully explored and the dock walls lie along
        # the x=0 and y=0 axes. Expansions across them (into (-1,0)/(0,-1)) are
        # unreachable space behind the walls and must be rejected; only the
        # in-room neighbors (1,0)/(0,1) remain.
        policy = CoverageExploration(
            [], (310, 310), FULLY_EXPLORED_START, [], DOCK_WALLS, territory_mm=1000
        )
        self.assertEqual(policy.focus, (0, 0))
        self.assertTrue(policy.territory_explored((0, 0)))
        self.assertEqual(
            policy.get_territory_expansions(),
            {((0, 0), (1, 0)), ((0, 0), (0, 1))},
        )

    def test_without_dock_walls_all_neighbors_would_expand(self):
        # Contrast: without the dock-wall segments the same fully-explored
        # territory offers all four neighbors -- the behind-wall (-1,0)/(0,-1)
        # included. This is exactly the spurious expansion the dock walls fix.
        policy = CoverageExploration(
            [], (310, 310), FULLY_EXPLORED_START, [], territory_mm=1000
        )
        self.assertEqual(
            policy.get_territory_expansions(),
            {
                ((0, 0), (1, 0)),
                ((0, 0), (-1, 0)),
                ((0, 0), (0, 1)),
                ((0, 0), (0, -1)),
            },
        )

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
        # With north (0, 0) given up, the selected expansion must be live and
        # its crossing heading must beat the opposite direction.
        policy = CoverageExploration(
            [], FULLY_EXPLORED_SOUTH[0], FULLY_EXPLORED_SOUTH, [], territory_mm=1000
        )
        self.assertTrue(policy.territory_explored((0, -1)))
        policy.abandoned.add((0, 0))
        policy.unlock_if_complete()

        self.assertNotEqual(policy.active_territory_expansion[1], (0, 0))
        x, y = 500, -500
        tx, ty = policy.active_expansion_crossing
        toward_active = math.degrees(math.atan2(ty - y, tx - x))
        self.assertGreater(
            policy.heading_preference(x, y, toward_active),
            policy.heading_preference(x, y, toward_active + 180),
        )

    def test_commits_to_a_boundary_to_expand_even_at_low_clearance(self):
        # (0, -1) finished; the robot sits just inside the south boundary. The
        # only way to expand into (0, -2) is to drive at that boundary and get
        # pinned (low clearance). That heading must score positive, not as
        # no-progress, or expand_past_boundary can never fire.
        path = list(FULLY_EXPLORED_SOUTH) + [(875, -950)]
        policy = CoverageExploration(
            [], path[0], path, [], territory_mm=1000
        )
        self.assertTrue(policy.territory_explored((0, -1)))
        policy.blocked_territory_expansions.update({
            ((0, -1), (-1, -1)),
            ((0, -1), (0, 0)),
            ((0, -1), (1, -1)),
        })
        policy.unlock_if_complete()
        self.assertEqual(
            policy.active_territory_expansion, ((0, -1), (0, -2))
        )
        # Near the south edge: heading south has almost no clearance.
        self.assertLess(policy.forward_distance(875, -950, -90, 3000), 200)
        self.assertGreater(policy.heading_preference(875, -950, -90), 0)

    def test_physical_stop_eliminates_one_crossing_without_blocking_expansion(self):
        policy = CoverageExploration(
            [], FULLY_EXPLORED_SOUTH[0], list(FULLY_EXPLORED_SOUTH), [],
            territory_mm=1000,
        )
        policy.blocked_territory_expansions.update({
            ((0, -1), (-1, -1)),
            ((0, -1), (0, 0)),
            ((0, -1), (1, -1)),
        })
        policy.unlock_if_complete()
        expansion = policy.active_territory_expansion
        crossing = policy.active_expansion_crossing

        policy.blockers.append(crossing)
        policy.add_blocked_territory_expansion(crossing[0], -850, -90)

        self.assertNotIn(expansion, policy.blocked_territory_expansions)
        self.assertEqual(policy.active_territory_expansion, expansion)
        self.assertNotEqual(policy.active_expansion_crossing, crossing)

    def test_blocks_expansion_after_every_crossing_is_physically_blocked(self):
        policy = CoverageExploration(
            [], FULLY_EXPLORED_SOUTH[0], list(FULLY_EXPLORED_SOUTH), [],
            territory_mm=1000,
        )
        policy.blocked_territory_expansions.update({
            ((0, -1), (-1, -1)),
            ((0, -1), (0, 0)),
            ((0, -1), (1, -1)),
        })
        policy.unlock_if_complete()
        expansion = policy.active_territory_expansion
        crossings = policy._territory_expansion_crossings(*expansion)
        policy.blockers.extend(crossings)

        policy.add_blocked_territory_expansion(500, -850, -90)

        self.assertIn(expansion, policy.blocked_territory_expansions)
        self.assertIsNone(policy.active_territory_expansion)
        self.assertTrue(policy.is_complete())

    def test_active_and_blocked_expansions_persist_across_runs(self):
        runs = [
            {
                "conservative_exploration": {
                    "territories": [[0, -1]],
                    "focus_territory": [0, -1],
                    "active_territory_expansion": [[0, -1], [0, -2]],
                    "blocked_territory_expansions": [
                        [[0, -1], [-1, -1]],
                    ],
                }
            }
        ]
        policy = CoverageExploration(
            runs, FULLY_EXPLORED_SOUTH[0], list(FULLY_EXPLORED_SOUTH), [],
            territory_mm=1000,
        )

        policy.unlock_if_complete()

        self.assertEqual(
            policy.active_territory_expansion, ((0, -1), (0, -2))
        )
        self.assertIn(
            ((0, -1), (-1, -1)), policy.blocked_territory_expansions
        )

    def test_active_expansion_unlocks_the_selected_territory(self):
        path = list(FULLY_EXPLORED_SOUTH) + [(875, -950)]
        policy = CoverageExploration([], path[0], path, [], territory_mm=1000)
        policy.blocked_territory_expansions.update({
            ((0, -1), (-1, -1)),
            ((0, -1), (0, 0)),
            ((0, -1), (1, -1)),
        })
        policy.unlock_if_complete()

        self.assertTrue(policy.expand_past_boundary(875, -950, -90))
        self.assertIn((0, -2), policy.territories)
        self.assertEqual(policy.focus, (0, -2))
        self.assertIsNone(policy.active_territory_expansion)

    def test_coverage_complete_when_every_boundary_crossing_is_blocked(self):
        policy = CoverageExploration(
            [], FULLY_EXPLORED_SOUTH[0], list(FULLY_EXPLORED_SOUTH), [],
            territory_mm=1000,
        )
        expansions = policy.get_territory_expansions()
        for expansion in expansions:
            policy.blockers.extend(policy._territory_expansion_crossings(*expansion))

        self.assertTrue(policy.is_complete())

    def test_metadata_records_objective_and_abandoned(self):
        policy = CoverageExploration([], (80, -80), [], [], territory_mm=1000)
        policy.abandoned.add((9, 9))
        policy.blocked_territory_expansions.add(((0, -1), (0, -2)))
        meta = policy.metadata()
        self.assertEqual(meta['objective'], 'coverage')
        self.assertIn([9, 9], [list(cell) for cell in meta['abandoned']])
        self.assertIn(
            [[0, -1], [0, -2]],
            [[list(source), list(target)] for source, target in meta[
                'blocked_territory_expansions'
            ]],
        )


if __name__ == "__main__":
    unittest.main()
