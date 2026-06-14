"""Experimental bounded-territory policy for room-map exploration."""

import math

try:
    from examples.mapping.exploration_policy import ExplorationPolicy
    from examples.mapping.exploration_walls import (
        WALL_SEGMENT_AVOID_MM,
        point_segment_distance,
        segment_crosses_wall,
    )
except ModuleNotFoundError:
    from exploration_policy import ExplorationPolicy
    from exploration_walls import (
        WALL_SEGMENT_AVOID_MM,
        point_segment_distance,
        segment_crosses_wall,
    )

TERRITORY_MM = 1000
GRID_CELLS = 4
MIN_VISITED_CELLS = 3
WALL_CLEARANCE_MM = 150
WALL_SAMPLE_MM = 25
WALL_RECOVERY_MM = 300
MIN_USEFUL_FORWARD_MM = 200
FRONTIER_HEADING_WEIGHT = 12000
FRONTIER_ENTRY_BONUS = 15000
REVISIT_PENALTY = 8000
NO_PROGRESS_PENALTY = 1000000
WALL_SEGMENT_PENALTY = 500000
WALL_SEGMENT_CLEARANCE_WEIGHT = 100


def territory_cell(x, y, size=TERRITORY_MM):
    return math.floor(x / size), math.floor(y / size)


def densify_path(points, max_step_mm):
    """Return path points sampled densely enough to cover traversed cells."""
    if not points:
        return []
    dense = [(float(points[0][0]), float(points[0][1]))]
    for start, end in zip(points, points[1:]):
        sx, sy = float(start[0]), float(start[1])
        ex, ey = float(end[0]), float(end[1])
        distance = math.hypot(ex - sx, ey - sy)
        steps = max(1, math.ceil(distance / max_step_mm))
        dense.extend(
            (
                sx + (ex - sx) * step / steps,
                sy + (ey - sy) * step / steps,
            )
            for step in range(1, steps + 1)
        )
    return dense


def territory_coverage(cell, points, territory_mm=TERRITORY_MM):
    grid_mm = territory_mm / GRID_CELLS
    cx, cy = cell
    x0 = cx * territory_mm
    y0 = cy * territory_mm
    return {
        (
            min(GRID_CELLS - 1, max(0, int((x - x0) // grid_mm))),
            min(GRID_CELLS - 1, max(0, int((y - y0) // grid_mm))),
        )
        for x, y in points
        if territory_cell(x, y, territory_mm) == cell
    }


def grid_cell_center(territory, cell, territory_mm=TERRITORY_MM):
    grid_mm = territory_mm / GRID_CELLS
    return (
        territory[0] * territory_mm + (cell[0] + 0.5) * grid_mm,
        territory[1] * territory_mm + (cell[1] + 0.5) * grid_mm,
    )


def local_grid_cell(territory, x, y, territory_mm=TERRITORY_MM):
    grid_mm = territory_mm / GRID_CELLS
    if territory_cell(x, y, territory_mm) != territory:
        return None
    return (
        min(
            GRID_CELLS - 1,
            max(0, int((x - territory[0] * territory_mm) // grid_mm)),
        ),
        min(
            GRID_CELLS - 1,
            max(0, int((y - territory[1] * territory_mm) // grid_mm)),
        ),
    )


def territory_resolution(
    cell, path_points, blockers, wall_segments=(), territory_mm=TERRITORY_MM
):
    all_cells = {
        (x, y)
        for x in range(GRID_CELLS)
        for y in range(GRID_CELLS)
    }
    visited = territory_coverage(cell, path_points, territory_mm)
    blocked = territory_coverage(cell, blockers, territory_mm) - visited
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
                grid_cell_center(cell, (x, y), territory_mm),
                grid_cell_center(cell, neighbor, territory_mm),
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


def territory_sufficiently_mapped(
    cell, path_points, blockers=(), wall_segments=(), territory_mm=TERRITORY_MM
):
    resolution = territory_resolution(
        cell, path_points, blockers, wall_segments, territory_mm
    )
    return len(resolution['visited']) >= MIN_VISITED_CELLS and not resolution['frontier']


class ConservativeExploration(ExplorationPolicy):
    """Policy that confines exploration to unlocked square territories."""

    metadata_key = 'conservative_exploration'

    def __init__(
        self,
        runs,
        start,
        path_points,
        blockers,
        wall_segments=None,
        territory_mm=TERRITORY_MM,
    ):
        self.path_points = path_points
        self.blockers = blockers
        self.wall_segments = wall_segments if wall_segments is not None else []
        self.territory_mm = territory_mm
        self.grid_mm = territory_mm / GRID_CELLS
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
        start_cell = territory_cell(start[0], start[1], self.territory_mm)
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
                cell, prior_path, prior_blockers, self.wall_segments,
                self.territory_mm,
            )['resolved']
        }
        self.completed_territories = {
            cell
            for cell in self.territories
            if territory_sufficiently_mapped(
                cell, prior_path, prior_blockers, self.wall_segments,
                self.territory_mm,
            )
        }

    def allows_point(self, x, y):
        return territory_cell(x, y, self.territory_mm) in set(self.territories)

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
        """Reward headings that drive into reachable, still-unvisited cells.

        Aiming toward a frontier cell is not enough: a frontier cell behind a
        wall keeps drawing Dash back to re-drive visited cells along that wall.
        So we reward headings whose forward leg actually *enters* a reachable
        unvisited cell (testing its reachability) and penalise headings that
        only re-traverse already-visited cells.
        """
        clearance = self.forward_distance(x, y, heading, self.territory_mm)
        if clearance < MIN_USEFUL_FORWARD_MM:
            return -NO_PROGRESS_PENALTY + clearance

        resolution = territory_resolution(
            self.focus,
            self.path_points,
            self.blockers,
            self.wall_segments,
            self.territory_mm,
        )
        forbidden = resolution['blocked'] | resolution['unreachable']
        frontier = resolution['frontier']
        visited = resolution['visited']
        hr = math.radians(heading)
        ux, uy = math.cos(hr), math.sin(hr)
        enters_frontier = False
        leaves_visited = False
        for distance in (self.grid_mm / 2, self.grid_mm, self.grid_mm * 1.5):
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
                point[0],
                point[1],
                self.territory_mm,
            )
            if cell in forbidden:
                return -NO_PROGRESS_PENALTY + clearance
            # Only count cells the leg can actually reach this move.
            if distance <= clearance:
                if cell in frontier:
                    enters_frontier = True
                if cell is not None and cell not in visited:
                    leaves_visited = True

        targets = [
            grid_cell_center(self.focus, cell, self.territory_mm)
            for cell in frontier
        ]
        if not targets:
            return clearance

        best_alignment = max(
            ((tx - x) * ux + (ty - y) * uy) / math.hypot(tx - x, ty - y)
            for tx, ty in targets
            if math.hypot(tx - x, ty - y) > 0
        )
        score = (
            best_alignment * FRONTIER_HEADING_WEIGHT
            + min(clearance, self.grid_mm)
        )
        if enters_frontier:
            score += FRONTIER_ENTRY_BONUS
        elif not leaves_visited:
            score -= REVISIT_PENALTY
        return score

    def report_progress(self):
        for cell in self.territories:
            resolution = territory_resolution(
                cell, self.path_points, self.blockers, self.wall_segments,
                self.territory_mm,
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
                    cell, self.path_points, self.blockers, self.wall_segments,
                    self.territory_mm,
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

    def territory_explored(self, cell):
        """True when nothing reachable is left to explore in ``cell``.

        ``territory_sufficiently_mapped`` also requires ``MIN_VISITED_CELLS``,
        which is the right bar for *reporting* a territory as mapped but wrong
        for deciding when to move on: a territory that is mostly blocked or
        unreachable (few reachable cells) has no frontier left yet never clears
        that bar, which would trap the exploration focus on it forever. Here we
        only require that the territory was entered and has no reachable
        frontier remaining.
        """
        resolution = territory_resolution(
            cell, self.path_points, self.blockers, self.wall_segments,
            self.territory_mm,
        )
        return bool(resolution['visited']) and not resolution['frontier']

    def unlock_if_complete(self):
        if not self.territory_explored(self.focus):
            return
        adjacent_unfinished = [
            cell
            for cell in self.territories
            if (
                cell != self.focus
                and abs(cell[0] - self.focus[0]) + abs(cell[1] - self.focus[1]) == 1
                and not self.territory_explored(cell)
            )
        ]
        if adjacent_unfinished:
            self.focus = max(
                adjacent_unfinished,
                key=lambda cell: len(
                    territory_coverage(cell, self.path_points, self.territory_mm)
                ),
            )
            print(
                f'\n  [adjacent territory selected] moving exploration '
                f'focus to {self.focus}'
            )
            return
        allowed = set(self.territories)
        # Grow the explored region from ANY unlocked territory, not just the
        # focus. A focus that completed by going unreachable (e.g. behind a
        # wall) must not keep unlocking territories further in that dead
        # direction. Prefer a frontier territory adjacent to the best-explored
        # one, so expansion follows where the robot actually made progress.
        candidates = {
            (cell[0] + dx, cell[1] + dy)
            for cell in self.territories
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
            if (cell[0] + dx, cell[1] + dy) not in allowed
        }
        if not candidates:
            return

        def neighbor_coverage(cell):
            return max(
                (
                    len(territory_coverage(
                        neighbor, self.path_points, self.territory_mm
                    ))
                    for neighbor in (
                        (cell[0] + 1, cell[1]), (cell[0] - 1, cell[1]),
                        (cell[0], cell[1] + 1), (cell[0], cell[1] - 1),
                    )
                    if neighbor in allowed
                ),
                default=0,
            )

        # Stable final tie-break: keep the original focus-relative preference
        # (+x, +y, -x, -y) so equal candidates resolve the same way as before.
        direction_rank = {(1, 0): 0, (0, 1): 1, (-1, 0): 2, (0, -1): 3}

        def focus_direction(cell):
            offset = (cell[0] - self.focus[0], cell[1] - self.focus[1])
            return direction_rank.get(offset, len(direction_rank))

        next_territory = min(
            candidates,
            key=lambda cell: (
                -len(territory_coverage(cell, self.path_points, self.territory_mm)),
                -neighbor_coverage(cell),
                len(territory_coverage(cell, self.blockers, self.territory_mm)),
                cell[0] ** 2 + cell[1] ** 2,
                focus_direction(cell),
                cell,
            ),
        )
        self.territories.append(next_territory)
        self.focus = next_territory
        print(
            f'\n  [adjacent territory unlocked] moving exploration '
            f'focus to {self.focus}'
        )

    def territory_ahead(self, x, y, heading):
        """Return the first territory along ``heading`` that differs from here."""
        current = territory_cell(x, y, self.territory_mm)
        hr = math.radians(heading)
        ux, uy = math.cos(hr), math.sin(hr)
        distance = WALL_SAMPLE_MM
        limit = self.territory_mm + WALL_RECOVERY_MM
        while distance <= limit:
            cell = territory_cell(
                x + distance * ux, y + distance * uy, self.territory_mm
            )
            if cell != current:
                return cell
            distance += WALL_SAMPLE_MM
        return None

    def expand_past_boundary(self, x, y, heading):
        """Unlock the territory ahead when the one the robot is in is mapped.

        Called when the invisible territory wall blocks forward progress.
        Unlike ``unlock_if_complete`` (which only acts on ``self.focus``), this
        is driven by the territory the robot physically occupies: once that one
        is sufficiently mapped, the robot is free to cross into the adjacent
        territory it is heading toward. The previous territory stays unlocked,
        so the robot may return to it later. Returns ``True`` when a new
        territory was unlocked.
        """
        current = territory_cell(x, y, self.territory_mm)
        if not self.territory_explored(current):
            return False
        ahead = self.territory_ahead(x, y, heading)
        if ahead is None or ahead in self.territories:
            return False
        self.territories.append(ahead)
        self.focus = ahead
        print(
            f'\n  [territory expanded] {current} fully mapped; unlocking '
            f'{ahead} ahead and continuing exploration'
        )
        return True

    def describe(self):
        return (
            f"  Conservative territory {self.focus}: "
            f"{self.territory_mm}mm square, {len(self.territories)} unlocked."
        )

    def metadata(self):
        resolution = territory_resolution(
            self.focus, self.path_points, self.blockers, self.wall_segments,
            self.territory_mm,
        )
        return {
            'territory_size_mm': self.territory_mm,
            'territories': self.territories,
            'focus_territory': self.focus,
            'focus_resolution': {
                key: len(value) for key, value in resolution.items()
            },
            'focus_coverage_cells': len(
                territory_coverage(self.focus, self.path_points, self.territory_mm)
            ),
        }
