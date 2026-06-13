"""Experimental bounded-territory policy for room-map exploration."""

import math

try:
    from examples.mapping.exploration_walls import (
        WALL_SEGMENT_AVOID_MM,
        point_segment_distance,
        segment_crosses_wall,
    )
except ModuleNotFoundError:
    from exploration_walls import (
        WALL_SEGMENT_AVOID_MM,
        point_segment_distance,
        segment_crosses_wall,
    )

TERRITORY_MM = 2000
GRID_MM = 500
GRID_CELLS = 4
MIN_VISITED_CELLS = 3
WALL_CLEARANCE_MM = 150
WALL_SAMPLE_MM = 25
WALL_RECOVERY_MM = 300
MIN_USEFUL_FORWARD_MM = 200
FRONTIER_HEADING_WEIGHT = 12000
NO_PROGRESS_PENALTY = 1000000
WALL_SEGMENT_PENALTY = 500000
WALL_SEGMENT_CLEARANCE_WEIGHT = 100


def territory_cell(x, y, size=TERRITORY_MM):
    return math.floor(x / size), math.floor(y / size)


def territory_coverage(cell, points):
    cx, cy = cell
    x0 = cx * TERRITORY_MM
    y0 = cy * TERRITORY_MM
    return {
        (
            min(GRID_CELLS - 1, max(0, int((x - x0) // GRID_MM))),
            min(GRID_CELLS - 1, max(0, int((y - y0) // GRID_MM))),
        )
        for x, y in points
        if territory_cell(x, y) == cell
    }


def grid_cell_center(territory, cell):
    return (
        territory[0] * TERRITORY_MM + (cell[0] + 0.5) * GRID_MM,
        territory[1] * TERRITORY_MM + (cell[1] + 0.5) * GRID_MM,
    )


def local_grid_cell(territory, x, y):
    if territory_cell(x, y) != territory:
        return None
    return (
        min(
            GRID_CELLS - 1,
            max(0, int((x - territory[0] * TERRITORY_MM) // GRID_MM)),
        ),
        min(
            GRID_CELLS - 1,
            max(0, int((y - territory[1] * TERRITORY_MM) // GRID_MM)),
        ),
    )


def territory_resolution(cell, path_points, blockers, wall_segments=()):
    all_cells = {
        (x, y)
        for x in range(GRID_CELLS)
        for y in range(GRID_CELLS)
    }
    visited = territory_coverage(cell, path_points)
    blocked = territory_coverage(cell, blockers) - visited
    if not visited:
        return {
            'visited': set(),
            'blocked': blocked,
            'unreachable': set(),
            'frontier': all_cells - blocked,
            'resolved': blocked,
        }
    reachable = set(visited)
    pending = list(visited)
    while pending:
        x, y = pending.pop()
        for neighbor in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            connection = (
                grid_cell_center(cell, (x, y)),
                grid_cell_center(cell, neighbor),
            )
            if (
                neighbor in all_cells
                and neighbor not in blocked
                and neighbor not in reachable
                and not segment_crosses_wall(*connection, wall_segments)
            ):
                reachable.add(neighbor)
                pending.append(neighbor)
    frontier = reachable - visited
    unreachable = all_cells - reachable - blocked
    return {
        'visited': visited,
        'blocked': blocked,
        'unreachable': unreachable,
        'frontier': frontier,
        'resolved': visited | blocked | unreachable,
    }


def territory_sufficiently_mapped(cell, path_points, blockers=(), wall_segments=()):
    resolution = territory_resolution(cell, path_points, blockers, wall_segments)
    return len(resolution['visited']) >= MIN_VISITED_CELLS and not resolution['frontier']


class ConservativeExploration:
    """Optional policy that confines exploration to unlocked 2 m territories."""

    metadata_key = 'conservative_exploration'

    def __init__(self, runs, start, path_points, blockers, wall_segments=None):
        self.path_points = path_points
        self.blockers = blockers
        self.wall_segments = wall_segments if wall_segments is not None else []
        self.territories = []
        self.focus = None
        for run in runs:
            state = run.get(self.metadata_key, {})
            for cell in state.get('territories', []):
                normalized = tuple(int(value) for value in cell)
                if normalized not in self.territories:
                    self.territories.append(normalized)
            if state.get('focus_territory') is not None:
                candidate = tuple(int(value) for value in state['focus_territory'])
                self.focus = candidate
        start_cell = territory_cell(*start)
        if not self.territories:
            self.territories.append(start_cell)
        if self.focus not in self.territories:
            self.focus = self.territories[-1]

        prior_path = list(path_points)
        prior_blockers = list(blockers)
        self.reported_cells = {
            (cell, local_cell)
            for cell in self.territories
            for local_cell in territory_resolution(
                cell, prior_path, prior_blockers, self.wall_segments
            )['resolved']
        }
        self.completed_territories = {
            cell
            for cell in self.territories
            if territory_sufficiently_mapped(
                cell, prior_path, prior_blockers, self.wall_segments
            )
        }

    def allows_point(self, x, y):
        return territory_cell(x, y) in set(self.territories)

    def forward_distance(self, x, y, heading, desired_distance):
        hr = math.radians(heading)
        ux, uy = math.cos(hr), math.sin(hr)
        distance = 0.0
        if not self.allows_point(x, y):
            while distance <= min(desired_distance, WALL_RECOVERY_MM):
                sx, sy = x + distance * ux, y + distance * uy
                if self.allows_point(sx, sy):
                    break
                distance += WALL_SAMPLE_MM
            else:
                return 0
        while distance <= desired_distance:
            sx, sy = x + distance * ux, y + distance * uy
            if not self.allows_point(sx, sy):
                return int(max(0, distance - WALL_SAMPLE_MM - WALL_CLEARANCE_MM))
            distance += WALL_SAMPLE_MM
        return int(desired_distance)

    def heading_preference(self, x, y, heading):
        """Reward headings toward reachable unvisited cells with room to move."""
        clearance = self.forward_distance(x, y, heading, TERRITORY_MM)
        if clearance < MIN_USEFUL_FORWARD_MM:
            return -NO_PROGRESS_PENALTY + clearance

        resolution = territory_resolution(
            self.focus,
            self.path_points,
            self.blockers,
            self.wall_segments,
        )
        forbidden = resolution['blocked'] | resolution['unreachable']
        hr = math.radians(heading)
        ux, uy = math.cos(hr), math.sin(hr)
        for distance in (GRID_MM / 2, GRID_MM, GRID_MM * 1.5):
            point = x + distance * ux, y + distance * uy
            if any(
                point_segment_distance(point, wall_start, wall_end)
                <= WALL_SEGMENT_AVOID_MM
                for wall_start, wall_end in self.wall_segments
            ):
                return (
                    -WALL_SEGMENT_PENALTY
                    + clearance * WALL_SEGMENT_CLEARANCE_WEIGHT
                )
            cell = local_grid_cell(
                self.focus,
                *point,
            )
            if cell in forbidden:
                return -NO_PROGRESS_PENALTY + clearance

        targets = [
            grid_cell_center(self.focus, cell)
            for cell in resolution['frontier']
        ]
        if not targets:
            return clearance

        best_alignment = max(
            ((tx - x) * ux + (ty - y) * uy) / math.hypot(tx - x, ty - y)
            for tx, ty in targets
            if math.hypot(tx - x, ty - y) > 0
        )
        return best_alignment * FRONTIER_HEADING_WEIGHT + min(clearance, GRID_MM)

    def report_progress(self):
        for cell in self.territories:
            resolution = territory_resolution(
                cell, self.path_points, self.blockers, self.wall_segments
            )
            for local_cell in sorted(resolution['resolved']):
                key = (cell, local_cell)
                if key in self.reported_cells:
                    continue
                if local_cell in resolution['visited']:
                    status = 'visited'
                elif local_cell in resolution['blocked']:
                    status = 'blocked by real observation'
                else:
                    status = 'unreachable behind real blockers'
                print(f'\n  [cell complete] territory {cell} cell {local_cell}: {status}')
                self.reported_cells.add(key)
            if (
                cell not in self.completed_territories
                and territory_sufficiently_mapped(
                    cell, self.path_points, self.blockers, self.wall_segments
                )
            ):
                print(
                    f'\n  [territory complete] {cell}: '
                    f'{len(resolution["visited"])} visited, '
                    f'{len(resolution["blocked"])} blocked, '
                    f'{len(resolution["unreachable"])} unreachable, '
                    'no reachable frontier remains'
                )
                self.completed_territories.add(cell)

    def unlock_if_complete(self):
        if not territory_sufficiently_mapped(
            self.focus, self.path_points, self.blockers, self.wall_segments
        ):
            return
        adjacent_unfinished = [
            cell
            for cell in self.territories
            if (
                cell != self.focus
                and abs(cell[0] - self.focus[0]) + abs(cell[1] - self.focus[1]) == 1
                and not territory_sufficiently_mapped(
                    cell, self.path_points, self.blockers, self.wall_segments
                )
            )
        ]
        if adjacent_unfinished:
            self.focus = max(
                adjacent_unfinished,
                key=lambda cell: len(territory_coverage(cell, self.path_points)),
            )
            print(
                f'\n  [adjacent territory selected] moving exploration '
                f'focus to {self.focus}'
            )
            return
        allowed = set(self.territories)
        candidates = [
            cell
            for cell in (
                (self.focus[0] + 1, self.focus[1]),
                (self.focus[0], self.focus[1] + 1),
                (self.focus[0] - 1, self.focus[1]),
                (self.focus[0], self.focus[1] - 1),
            )
            if cell not in allowed
        ]
        if not candidates:
            return
        next_territory = min(
            candidates,
            key=lambda cell: (
                -len(territory_coverage(cell, self.path_points)),
                len(territory_coverage(cell, self.blockers)),
                cell[0] ** 2 + cell[1] ** 2,
            ),
        )
        self.territories.append(next_territory)
        self.focus = next_territory
        print(
            f'\n  [adjacent territory unlocked] moving exploration '
            f'focus to {self.focus}'
        )

    def describe(self):
        return (
            f"  Conservative territory {self.focus}: "
            f"{TERRITORY_MM}mm square, {len(self.territories)} unlocked."
        )

    def metadata(self):
        resolution = territory_resolution(
            self.focus, self.path_points, self.blockers, self.wall_segments
        )
        return {
            'territory_size_mm': TERRITORY_MM,
            'territories': self.territories,
            'focus_territory': self.focus,
            'focus_resolution': {
                key: len(value) for key, value in resolution.items()
            },
            'focus_coverage_cells': len(territory_coverage(self.focus, self.path_points)),
        }
