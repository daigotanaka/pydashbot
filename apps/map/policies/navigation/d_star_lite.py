"""D* Lite go-home planner: replan with localized soft costs for failed approaches."""

import heapq
import math

try:
    from apps.map.policies.navigation.navigation_policy_base import (
        NavigationPolicy,
        _build_graph,
        _collect_blocked_records,
        _point_near_segment,
        _simplify_route,
        _validate_runs,
    )
except ModuleNotFoundError:
    from policies.navigation.navigation_policy_base import (
        NavigationPolicy,
        _build_graph,
        _collect_blocked_records,
        _point_near_segment,
        _simplify_route,
        _validate_runs,
    )


class DStarLite:
    """D* Lite shortest-path search over a directed weighted graph."""

    def __init__(self, adjacency, positions, start, goal):
        self.adjacency = adjacency
        self.positions = positions
        self.start = start
        self.goal = goal
        self.predecessors = {node: [] for node in adjacency}
        for node, edges in adjacency.items():
            for neighbor, cost in edges:
                self.predecessors[neighbor].append((node, cost))
        self.g = {node: math.inf for node in adjacency}
        self.rhs = {node: math.inf for node in adjacency}
        self.rhs[goal] = 0.0
        self.queue = []
        self._push(goal)

    def _heuristic(self, node):
        return math.dist(self.positions[self.start], self.positions[node])

    def _key(self, node):
        best = min(self.g[node], self.rhs[node])
        return best + self._heuristic(node), best

    def _push(self, node):
        heapq.heappush(self.queue, (*self._key(node), node))

    def _update_vertex(self, node):
        if node != self.goal:
            self.rhs[node] = min(
                (cost + self.g[successor] for successor, cost in self.adjacency[node]),
                default=math.inf,
            )
        if self.g[node] != self.rhs[node]:
            self._push(node)

    def compute_shortest_path(self):
        while self.queue:
            top_key = self.queue[0][:2]
            if (
                top_key >= self._key(self.start)
                and self.rhs[self.start] == self.g[self.start]
            ):
                break
            old_first, old_second, node = heapq.heappop(self.queue)
            old_key = (old_first, old_second)
            if old_key < self._key(node):
                self._push(node)
            elif self.g[node] == self.rhs[node]:
                # A newer queue entry already made this node consistent.
                continue
            elif self.g[node] > self.rhs[node]:
                self.g[node] = self.rhs[node]
                for predecessor, _ in self.predecessors[node]:
                    self._update_vertex(predecessor)
            else:
                self.g[node] = math.inf
                self._update_vertex(node)
                for predecessor, _ in self.predecessors[node]:
                    self._update_vertex(predecessor)

    def route(self):
        self.compute_shortest_path()
        if not math.isfinite(self.g[self.start]):
            return None
        route = [self.start]
        seen = {self.start}
        while route[-1] != self.goal:
            node = route[-1]
            candidates = [
                (cost + self.g[neighbor], cost, neighbor)
                for neighbor, cost in self.adjacency[node]
                if math.isfinite(self.g[neighbor]) and neighbor not in seen
            ]
            if not candidates:
                return None
            _, _, neighbor = min(candidates)
            route.append(neighbor)
            seen.add(neighbor)
        return route


class DStarLitePolicy(NavigationPolicy):
    """D* Lite replanning with localized soft costs for failed approaches."""

    name = "d-star-lite"

    def __init__(
        self,
        link_radius_mm=250,
        collinear_degrees=12,
        risk_radius_mm=225,
        blocked_approach_penalty=4000,
        inferred_link_penalty=75,
    ):
        self.link_radius_mm = link_radius_mm
        self.collinear_degrees = collinear_degrees
        self.risk_radius_mm = risk_radius_mm
        self.blocked_approach_penalty = blocked_approach_penalty
        self.inferred_link_penalty = inferred_link_penalty

    def plan_route(self, data, accepted_runs, run_pose_trustworthy):
        runs = _validate_runs(data, accepted_runs, run_pose_trustworthy)
        blocked_records = _collect_blocked_records(data)

        def edge_cost(first, second, distance, actual):
            cost = distance + (0 if actual else self.inferred_link_penalty)
            ex, ey = second[0] - first[0], second[1] - first[1]
            edge_len = math.hypot(ex, ey)
            for record in blocked_records:
                fx = record["to"][0] - record["from"][0]
                fy = record["to"][1] - record["from"][1]
                failed_len = math.hypot(fx, fy)
                if edge_len < 1 or failed_len < 1:
                    continue
                same_direction = (ex * fx + ey * fy) / (edge_len * failed_len)
                if same_direction < math.cos(math.radians(45)):
                    continue
                if not _point_near_segment(
                    record["stop"], first, second, self.risk_radius_mm
                ):
                    continue
                cost += self.blocked_approach_penalty
            return cost

        positions, adjacency = _build_graph(runs, self.link_radius_mm, edge_cost)
        start = (len(runs) - 1, len(runs[-1]["path"]) - 1)
        goal = (0, 0)
        route = DStarLite(adjacency, positions, start, goal).route()
        if route is None:
            raise ValueError(
                "proven-route graph does not connect back to the starting pose"
            )
        return _simplify_route([positions[node] for node in route], self.collinear_degrees)
