"""Legacy go-home planner: permanently exclude every corridor marked blocked."""

import heapq
import math

try:
    from apps.map.policies.navigation.navigation_policy_base import (
        NavigationPolicy,
        _build_graph,
        _collect_blocked_records,
        _simplify_route,
        _validate_runs,
        edge_is_blocked,
    )
except ModuleNotFoundError:
    from policies.navigation.navigation_policy_base import (
        NavigationPolicy,
        _build_graph,
        _collect_blocked_records,
        _simplify_route,
        _validate_runs,
        edge_is_blocked,
    )


class HardBlockedEdgePolicy(NavigationPolicy):
    """Legacy planner: permanently exclude every corridor marked blocked."""

    name = "hard-blocked-edge"

    def __init__(self, link_radius_mm=250, blocked_tolerance_mm=150, collinear_degrees=12):
        self.link_radius_mm = link_radius_mm
        self.blocked_tolerance_mm = blocked_tolerance_mm
        self.collinear_degrees = collinear_degrees

    def plan_route(self, data, accepted_runs, run_pose_trustworthy):
        runs = _validate_runs(data, accepted_runs, run_pose_trustworthy)
        records = _collect_blocked_records(data)
        blocked_edges = [(record["from"], record["to"]) for record in records]

        def edge_cost(first, second, distance, actual):
            if edge_is_blocked(
                first, second, blocked_edges, tolerance=self.blocked_tolerance_mm
            ):
                return math.inf
            return distance

        positions, adjacency = _build_graph(runs, self.link_radius_mm, edge_cost)
        start = (len(runs) - 1, len(runs[-1]["path"]) - 1)
        route = _dijkstra_route(adjacency, start, (0, 0))
        if route is None:
            if blocked_edges:
                raise ValueError(
                    "no unblocked proven route home remains; "
                    f"{len(blocked_edges)} known route segment(s) are blocked"
                )
            raise ValueError(
                "accepted map paths do not connect back to the starting pose"
            )
        return _simplify_route([positions[node] for node in route], self.collinear_degrees)


def _dijkstra_route(adjacency, start, goal):
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
        return None
    route = [goal]
    while route[-1] != start:
        route.append(previous[route[-1]])
    route.reverse()
    return route
