"""Shared inferred-wall geometry for exploration planning."""

import math


WALL_LINK_MAX_MM = 300
WALL_SEGMENT_AVOID_MM = 150
REACHABILITY_WALL_CLEARANCE_MM = 10
WALL_SEGMENT_SAMPLE_MM = 100


def inferred_wall_segments(wall_points, max_distance=WALL_LINK_MAX_MM):
    """Join observations into a sparse relative-neighborhood graph.

    Connecting every nearby pair fills clusters with inferred chords that need
    not follow a physical wall. Keep an edge only when no third observation is
    closer to both endpoints; this preserves chains while avoiding unsupported
    shortcuts.
    """
    return [
        (start, end)
        for index, start in enumerate(wall_points)
        for end_index, end in enumerate(wall_points[index + 1:], index + 1)
        if (
            0
            < (distance := math.hypot(start[0] - end[0], start[1] - end[1]))
            <= max_distance
            and not any(
                other_index not in (index, end_index)
                and math.hypot(start[0] - other[0], start[1] - other[1])
                < distance
                and math.hypot(end[0] - other[0], end[1] - other[1])
                < distance
                for other_index, other in enumerate(wall_points)
            )
        )
    ]


def point_segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    scale = max(
        0,
        min(
            1,
            ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
            / length_sq,
        ),
    )
    nearest = start[0] + scale * dx, start[1] + scale * dy
    return math.hypot(point[0] - nearest[0], point[1] - nearest[1])


def segment_segment_distance(start1, end1, start2, end2):
    """Return the minimum distance between two line segments."""
    epsilon = 1e-9

    def orientation(a, b, c):
        return (
            (b[0] - a[0]) * (c[1] - a[1])
            - (b[1] - a[1]) * (c[0] - a[0])
        )

    def on_segment(a, b, point):
        return (
            min(a[0], b[0]) - epsilon <= point[0] <= max(a[0], b[0]) + epsilon
            and min(a[1], b[1]) - epsilon <= point[1] <= max(a[1], b[1]) + epsilon
        )

    o1 = orientation(start1, end1, start2)
    o2 = orientation(start1, end1, end2)
    o3 = orientation(start2, end2, start1)
    o4 = orientation(start2, end2, end1)
    if (
        ((o1 < -epsilon and o2 > epsilon) or (o2 < -epsilon and o1 > epsilon))
        and (
            (o3 < -epsilon and o4 > epsilon)
            or (o4 < -epsilon and o3 > epsilon)
        )
    ) or (
        abs(o1) <= epsilon and on_segment(start1, end1, start2)
    ) or (
        abs(o2) <= epsilon and on_segment(start1, end1, end2)
    ) or (
        abs(o3) <= epsilon and on_segment(start2, end2, start1)
    ) or (
        abs(o4) <= epsilon and on_segment(start2, end2, end1)
    ):
        return 0.0
    return min(
        point_segment_distance(start1, start2, end2),
        point_segment_distance(end1, start2, end2),
        point_segment_distance(start2, start1, end1),
        point_segment_distance(end2, start1, end1),
    )


def segment_intersects_wall(
    start, end, wall_segments, clearance_mm=REACHABILITY_WALL_CLEARANCE_MM
):
    """Whether inferred wall geometry separates the endpoints topologically."""
    return any(
        segment_segment_distance(start, end, wall_start, wall_end)
        <= clearance_mm
        for wall_start, wall_end in wall_segments
    )


def segment_crosses_wall(
    start, end, wall_segments, clearance_mm=WALL_SEGMENT_AVOID_MM
):
    for point in (start, end):
        if any(
            point_segment_distance(point, wall_start, wall_end)
            <= clearance_mm
            for wall_start, wall_end in wall_segments
        ):
            return True

    distance = WALL_SEGMENT_SAMPLE_MM
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    while distance < length:
        scale = distance / length
        point = (
            start[0] + scale * (end[0] - start[0]),
            start[1] + scale * (end[1] - start[1]),
        )
        if any(
            point_segment_distance(point, wall_start, wall_end)
            <= clearance_mm
            for wall_start, wall_end in wall_segments
        ):
            return True
        distance += WALL_SEGMENT_SAMPLE_MM
    return False
