"""Navigation-policy interface (abstract base) and shared planning helpers.

A navigation policy plans a route that returns a mapped robot to its starting
pose over the *proven* graph of accepted run paths. Concrete policies live in
their own module (1 file, 1 class) -- e.g. ``HardBlockedEdgePolicy`` in
``hard_blocked_edge.py`` and ``DStarLitePolicy`` in ``d_star_lite.py``; this
module holds the abstract base they share plus the graph/route helpers both
build on.
"""

import math
from abc import ABC, abstractmethod


class NavigationPolicy(ABC):
    """Pluggable route planner that returns the robot home.

    Concrete policies build a graph from the accepted run paths and search it
    for a route from the latest pose back to the start, differing in how they
    treat corridors that previously failed (hard exclusion vs. soft cost).
    """

    #: stable key used to select this policy by name in config.
    name = 'navigation'

    @abstractmethod
    def plan_route(self, data, accepted_runs, run_pose_trustworthy, target_xy=None):
        """Plan a route over the proven graph from the latest saved pose.

        ``target_xy`` is the destination point; the route ends at the proven
        node nearest it. ``None`` targets the starting pose (go-home). Raises
        ``ValueError`` if no route over the proven paths remains.
        """


def goal_node(positions, target_xy):
    """The graph node to route to: the start node ``(0, 0)`` when ``target_xy``
    is ``None`` (go-home), otherwise the proven node nearest the target point."""
    if target_xy is None:
        return (0, 0)
    return min(positions, key=lambda node: math.dist(positions[node], target_xy))


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
