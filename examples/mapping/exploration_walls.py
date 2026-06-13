"""Shared inferred-wall geometry for exploration planning."""

import math


WALL_LINK_MAX_MM = 300
WALL_SEGMENT_AVOID_MM = 150
WALL_SEGMENT_SAMPLE_MM = 100


def inferred_wall_segments(wall_points, max_distance=WALL_LINK_MAX_MM):
    """Join nearby real wall observations without altering saved wall points."""
    return [
        (start, end)
        for index, start in enumerate(wall_points)
        for end in wall_points[index + 1:]
        if 0 < math.hypot(start[0] - end[0], start[1] - end[1]) <= max_distance
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


def segment_crosses_wall(start, end, wall_segments):
    for point in (start, end):
        if any(
            point_segment_distance(point, wall_start, wall_end)
            <= WALL_SEGMENT_AVOID_MM
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
            <= WALL_SEGMENT_AVOID_MM
            for wall_start, wall_end in wall_segments
        ):
            return True
        distance += WALL_SEGMENT_SAMPLE_MM
    return False
