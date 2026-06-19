"""Swappable route-planning strategies for returning a mapped robot home."""

import heapq
import math


def _point_near_segment(point, seg_start, seg_end, tolerance):
    px, py = point
    ax, ay = seg_start
    bx, by = seg_end
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0:
        return math.hypot(px - ax, py - ay) <= tolerance
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy) <= tolerance


def edge_is_blocked(first_point, second_point, blocked_edges, tolerance=150):
    """Return whether an edge overlaps and runs along a blocked route segment."""
    ex, ey = second_point[0] - first_point[0], second_point[1] - first_point[1]
    edge_len = math.hypot(ex, ey)
    if edge_len < 1:
        return False
    midpoint = (
        (first_point[0] + second_point[0]) / 2,
        (first_point[1] + second_point[1]) / 2,
    )
    for seg_start, seg_end in blocked_edges:
        sx, sy = seg_end[0] - seg_start[0], seg_end[1] - seg_start[1]
        seg_len_sq = sx * sx + sy * sy
        if seg_len_sq == 0:
            if math.dist(midpoint, seg_start) <= tolerance:
                return True
            continue
        t = (
            (midpoint[0] - seg_start[0]) * sx
            + (midpoint[1] - seg_start[1]) * sy
        ) / seg_len_sq
        if t < 0 or t > 1:
            continue
        projection = (seg_start[0] + t * sx, seg_start[1] + t * sy)
        if math.dist(midpoint, projection) > tolerance:
            continue
        alignment = abs(ex * sx + ey * sy) / (edge_len * math.sqrt(seg_len_sq))
        if alignment >= math.cos(math.radians(30)):
            return True
    return False


def _collect_blocked_records(data):
    records = []
    for run in data.get("runs", []):
        for edge in run.get("blocked_edges", []):
            if "from" not in edge or "to" not in edge:
                continue
            records.append(
                {
                    "from": (float(edge["from"][0]), float(edge["from"][1])),
                    "to": (float(edge["to"][0]), float(edge["to"][1])),
                    "stop": (
                        float(edge.get("stop", edge["to"])[0]),
                        float(edge.get("stop", edge["to"])[1]),
                    ),
                }
            )
    return records


def _build_graph(runs, link_radius_mm, edge_cost):
    positions = {}
    adjacency = {}
    nodes = []

    def add_link(first, second, actual):
        distance = math.dist(positions[first], positions[second])
        forward = edge_cost(positions[first], positions[second], distance, actual)
        reverse = edge_cost(positions[second], positions[first], distance, actual)
        if math.isfinite(forward):
            adjacency[first].append((second, forward))
        if math.isfinite(reverse):
            adjacency[second].append((first, reverse))

    for run_index, run in enumerate(runs):
        run_nodes = []
        for point_index, point in enumerate(run["path"]):
            node = (run_index, point_index)
            positions[node] = (float(point[0]), float(point[1]))
            adjacency[node] = []
            nodes.append(node)
            run_nodes.append(node)
        for first, second in zip(run_nodes, run_nodes[1:]):
            add_link(first, second, actual=True)

    for index, first in enumerate(nodes):
        for second in nodes[index + 1:]:
            if first[0] == second[0] and abs(first[1] - second[1]) <= 1:
                continue
            if math.dist(positions[first], positions[second]) <= link_radius_mm:
                add_link(first, second, actual=False)
    return positions, adjacency


def _simplify_route(route, collinear_degrees):
    deduped = []
    for point in route:
        if not deduped or math.dist(point, deduped[-1]) > 1:
            deduped.append(point)

    simplified = []
    for point in deduped:
        simplified.append(point)
        while len(simplified) >= 3:
            a, b, c = simplified[-3:]
            ab = math.atan2(b[1] - a[1], b[0] - a[0])
            bc = math.atan2(c[1] - b[1], c[0] - b[0])
            turn = (bc - ab + math.pi) % (2 * math.pi) - math.pi
            turn_degrees = abs(math.degrees(turn))
            if collinear_degrees < turn_degrees < 180 - collinear_degrees:
                break
            simplified.pop(-2)
    return simplified


def _validate_runs(data, accepted_runs, run_pose_trustworthy):
    latest = data.get("runs", [])[-1] if data.get("runs") else None
    if not latest or latest.get("status", "accepted") not in {"accepted", "partial"}:
        raise ValueError("go-home requires an accepted or safely aborted latest run")
    if not run_pose_trustworthy(latest):
        raise ValueError("go-home requires a trustworthy final saved pose")
    runs = [run for run in accepted_runs(data) if run.get("path")]
    if not runs:
        raise ValueError("map does not contain an accepted path home")
    return runs


class HardBlockedEdgeStrategy:
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


class DStarLiteStrategy:
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
