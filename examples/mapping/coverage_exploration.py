"""Coverage-maximizing variant of the conservative-territory policy.

This is the "second version" of :class:`ConservativeExploration`, built around
the redefined objective:

    Maximize the number of cells marked *visited*, subject to the conservative
    unlock constraint (a new territory may only be entered after the current
    one is finished).

It reuses the parent's territory machinery (the invisible-wall constraint via
``allows_point`` / ``forward_distance``, reachability resolution, unlocking and
expansion) and changes two things:

1. ``heading_preference`` rewards the **count of new reachable, unvisited cells
   a leg would enter**, a direct proxy for the objective, instead of the
   parent's "alignment to the nearest frontier cell" heuristic.

2. A **stateful no-progress signal**: a focus that yields no new cells over
   several legs is abandoned. This catches the case the unreachable-marking
   rule cannot -- a territory the robot can never physically *enter* (0 visited
   cells, so its cells stay classified ``frontier`` forever and keep tugging
   the robot toward space it cannot reach).
"""

import math

try:
    from examples.mapping.conservative_exploration import (
        ConservativeExploration,
        FRONTIER_HEADING_WEIGHT,
        MIN_USEFUL_FORWARD_MM,
        NO_PROGRESS_PENALTY,
        REVISIT_PENALTY,
        TERRITORY_MM,
        WALL_CLEARANCE_MM,
        WALL_SAMPLE_MM,
        WALL_SEGMENT_CLEARANCE_WEIGHT,
        WALL_SEGMENT_PENALTY,
        grid_cell_center,
        local_grid_cell,
        territory_cell,
        territory_coverage,
        territory_resolution,
    )
    from examples.mapping.exploration_walls import (
        WALL_SEGMENT_AVOID_MM,
        point_segment_distance,
    )
except ModuleNotFoundError:
    from conservative_exploration import (
        ConservativeExploration,
        FRONTIER_HEADING_WEIGHT,
        MIN_USEFUL_FORWARD_MM,
        NO_PROGRESS_PENALTY,
        REVISIT_PENALTY,
        TERRITORY_MM,
        WALL_CLEARANCE_MM,
        WALL_SAMPLE_MM,
        WALL_SEGMENT_CLEARANCE_WEIGHT,
        WALL_SEGMENT_PENALTY,
        grid_cell_center,
        local_grid_cell,
        territory_cell,
        territory_coverage,
        territory_resolution,
    )
    from exploration_walls import (
        WALL_SEGMENT_AVOID_MM,
        point_segment_distance,
    )

# Reward per new reachable cell a candidate leg would enter. Set above
# FRONTIER_HEADING_WEIGHT so "actually enters a new cell" beats "merely points
# at one", and so a leg covering more new cells outscores one covering fewer.
COVERAGE_CELL_WEIGHT = 20000

# Legs aimed at the focus without gaining a cell before it is abandoned.
STALL_LEGS = 3


class CoverageExploration(ConservativeExploration):
    """Conservative exploration that maximizes newly-visited cells per leg."""

    def __init__(
        self,
        runs,
        start,
        path_points,
        blockers,
        wall_segments=None,
        territory_mm=TERRITORY_MM,
    ):
        super().__init__(
            runs, start, path_points, blockers, wall_segments, territory_mm
        )
        # Territories given up as unreachable-in-practice (carried across runs).
        self.abandoned = set()
        for run in runs:
            state = run.get(self.metadata_key, {})
            for cell in state.get('abandoned', []):
                self.abandoned.add(tuple(int(value) for value in cell))
        self._tracked_focus = self.focus
        self._focus_visited = self._focus_coverage()
        self._stall_legs = 0

    def _focus_coverage(self):
        return len(
            territory_coverage(self.focus, self.path_points, self.territory_mm)
        )

    def forward_distance(self, x, y, heading, desired_distance):
        # Beyond the parent's unlocked-region clamp, stop the leg at the boundary
        # of a *completed* territory the robot is about to re-enter (one it is
        # not already standing in). This keeps legs from sailing through the
        # finished start territory between forays, and -- because
        # heading_preference reads clearance -- it makes directions into
        # finished territory score as no-progress while directions into
        # still-uncharted (frontier-bearing) territory stay open.
        base = super().forward_distance(x, y, heading, desired_distance)
        current = territory_cell(x, y, self.territory_mm)
        hr = math.radians(heading)
        ux, uy = math.cos(hr), math.sin(hr)
        last_cell = current
        distance = WALL_SAMPLE_MM
        while distance <= base:
            cell = territory_cell(
                x + distance * ux, y + distance * uy, self.territory_mm
            )
            if cell != last_cell:
                last_cell = cell
                if cell != current and self.territory_explored(cell):
                    return int(
                        max(0, distance - WALL_SAMPLE_MM - WALL_CLEARANCE_MM)
                    )
            distance += WALL_SAMPLE_MM
        return int(base)

    def territory_explored(self, cell):
        # An abandoned territory is "done" for the purpose of moving focus on.
        return cell in self.abandoned or super().territory_explored(cell)

    def unlock_if_complete(self):
        # Runs once per turn decision. Account for progress first so a focus the
        # robot cannot enter is abandoned, then let the parent reselect/expand.
        self._note_progress()
        super().unlock_if_complete()

    def _note_progress(self):
        if self.focus != self._tracked_focus:
            # Focus moved (expansion/unlock); restart the stall accounting.
            self._tracked_focus = self.focus
            self._focus_visited = self._focus_coverage()
            self._stall_legs = 0
            return
        coverage = self._focus_coverage()
        if coverage > self._focus_visited:
            self._focus_visited = coverage
            self._stall_legs = 0
            return
        self._stall_legs += 1
        if (
            self._stall_legs >= STALL_LEGS
            and self.focus not in self.abandoned
            and not super().territory_explored(self.focus)
        ):
            # Has a frontier on paper but we keep gaining nothing: give it up.
            self.abandoned.add(self.focus)
            print(
                f"\n  [focus abandoned] {self.focus}: no new cells in "
                f"{STALL_LEGS} legs; treating as unreachable in practice"
            )
            self._stall_legs = 0

    def heading_preference(self, x, y, heading):
        clearance = self.forward_distance(x, y, heading, self.territory_mm)
        if clearance < MIN_USEFUL_FORWARD_MM:
            return -NO_PROGRESS_PENALTY + clearance
        if self.focus in self.abandoned:
            # Don't be tugged toward a territory we've given up reaching.
            return clearance

        resolution = territory_resolution(
            self.focus,
            self.path_points,
            self.blockers,
            self.wall_segments,
            self.territory_mm,
        )
        frontier = resolution['frontier']
        forbidden = resolution['blocked'] | resolution['unreachable']
        if not frontier:
            # Focus exhausted; unlock_if_complete will move focus on next turn.
            return clearance

        hr = math.radians(heading)
        ux, uy = math.cos(hr), math.sin(hr)
        new_cells = set()
        step = self.grid_mm / 2
        distance = step
        while distance <= clearance:
            point = (x + distance * ux, y + distance * uy)
            if any(
                point_segment_distance(point, wall_start, wall_end)
                <= WALL_SEGMENT_AVOID_MM
                for wall_start, wall_end in self.wall_segments
            ):
                return (
                    -WALL_SEGMENT_PENALTY
                    + clearance * WALL_SEGMENT_CLEARANCE_WEIGHT
                )
            cell = local_grid_cell(self.focus, point[0], point[1], self.territory_mm)
            if cell in forbidden:
                return -NO_PROGRESS_PENALTY + clearance
            if cell in frontier:
                new_cells.add(cell)
            distance += step

        if new_cells:
            # The objective: how many new reachable cells this leg would visit.
            return len(new_cells) * COVERAGE_CELL_WEIGHT + min(clearance, self.grid_mm)

        # No new cell reachable this leg: keep a weak pull toward the nearest
        # frontier so the robot still closes on it, and discourage pure revisits.
        targets = [
            grid_cell_center(self.focus, cell, self.territory_mm)
            for cell in frontier
        ]
        best_alignment = max(
            ((tx - x) * ux + (ty - y) * uy) / math.hypot(tx - x, ty - y)
            for tx, ty in targets
            if math.hypot(tx - x, ty - y) > 0
        )
        return best_alignment * FRONTIER_HEADING_WEIGHT - REVISIT_PENALTY

    def metadata(self):
        data = super().metadata()
        data['objective'] = 'coverage'
        data['abandoned'] = sorted(self.abandoned)
        return data
