import unittest

from apps.map.policies.navigation.d_star_lite import DStarLitePolicy
from apps.map.policies.navigation.hard_blocked_edge import HardBlockedEdgePolicy
from apps.map.policies.navigation.navigation_policy_base import NavigationPolicy


class NavigationPolicyStructureTests(unittest.TestCase):
    def test_navigation_policy_is_an_abstract_base(self):
        with self.assertRaises(TypeError):
            NavigationPolicy()  # abstract plan_route is unimplemented

    def test_strategies_are_navigation_policies(self):
        for strategy, name in (
            (DStarLitePolicy, "d-star-lite"),
            (HardBlockedEdgePolicy, "hard-blocked-edge"),
        ):
            self.assertTrue(issubclass(strategy, NavigationPolicy))
            self.assertEqual(strategy.name, name)

    def test_strategies_plan_a_route_home_over_proven_paths(self):
        # One run straight out and partway back; both planners return the route
        # to the start node (0, 0) over the proven path graph.
        data = {
            "runs": [
                {
                    "status": "accepted",
                    "path": [[0, 0], [250, 0], [500, 0]],
                    "blocked_edges": [],
                }
            ]
        }
        accepted_runs = lambda d: d["runs"]
        trustworthy = lambda run: True

        for strategy in (DStarLitePolicy(), HardBlockedEdgePolicy()):
            route = strategy.plan_route(data, accepted_runs, trustworthy)
            self.assertEqual(route[0], (500.0, 0.0))
            self.assertEqual(route[-1], (0.0, 0.0))

    def test_plan_route_targets_the_nearest_node_to_a_point(self):
        data = {
            "runs": [
                {
                    "status": "accepted",
                    "path": [[0, 0], [250, 0], [500, 0]],
                    "blocked_edges": [],
                }
            ]
        }
        accepted_runs = lambda d: d["runs"]
        trustworthy = lambda run: True

        for strategy in (DStarLitePolicy(), HardBlockedEdgePolicy()):
            # No target -> route home, to the start node.
            self.assertEqual(
                strategy.plan_route(data, accepted_runs, trustworthy)[-1], (0.0, 0.0)
            )
            # A target near the middle node routes to that node, not home.
            to_mid = strategy.plan_route(
                data, accepted_runs, trustworthy, target_xy=(240, 10)
            )
            self.assertEqual(to_mid[-1], (250.0, 0.0))


if __name__ == "__main__":
    unittest.main()
