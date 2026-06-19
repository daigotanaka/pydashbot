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
    from apps.map.policies.conservative_exploration import (
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
    from apps.map.exploration_walls import (
        WALL_SEGMENT_AVOID_MM,
        point_segment_distance,
    )
except ModuleNotFoundError:
    from policies.conservative_exploration import (
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

# A shared territory boundary is tested at the center of each reachability cell.
# One physical stop eliminates one crossing area, not the whole expansion.
EXPANSION_CROSSINGS = 4


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
        self.blocked_territory_expansions = set()
        self.active_territory_expansion = None
        for run in runs:
            state = run.get(self.metadata_key, {})
            for cell in state.get('abandoned', []):
                self.abandoned.add(tuple(int(value) for value in cell))
            for expansion in state.get('blocked_territory_expansions', []):
                source, target = expansion
                self.blocked_territory_expansions.add(
                    (
                        tuple(int(value) for value in source),
                        tuple(int(value) for value in target),
                    )
                )
            if state.get('active_territory_expansion') is not None:
                self.active_territory_expansion = tuple(
                    tuple(int(value) for value in cell)
                    for cell in state['active_territory_expansion']
                )
        # Abandonment exists only for a territory the robot cannot physically
        # enter. Once any cell was visited, normal reachability owns its state.
        self.abandoned = {
            cell
            for cell in self.abandoned
            if not territory_coverage(cell, self.path_points, self.territory_mm)
        }
        self.active_expansion_crossing = None
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
                if (
                    cell != current
                    and self.territory_explored(cell)
                    and (
                        self.active_territory_expansion is None
                        or cell != self.active_territory_expansion[0]
                    )
                ):
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
        # robot cannot enter is abandoned, then let the parent reselect focus.
        self._note_progress()
        super().unlock_if_complete()
        self._select_territory_expansion()

    def _unlock_new_territory(self):
        # Coverage creates territories only where the robot physically reaches a
        # boundary (expand_past_boundary), never abstractly. Abstract unlocking
        # cascades into territories the robot cannot reach once a focus stalls,
        # and -- with the boundary clamp -- walls the robot into the start
        # territory. Leaving focus put lets heading_preference steer toward the
        # nearest expandable frontier instead.
        return

    def _territory_expansion_crossings(self, source, target):
        """Return plausible crossing points along the shared territory boundary."""
        dx, dy = target[0] - source[0], target[1] - source[1]
        if abs(dx) + abs(dy) != 1:
            return []
        step = self.territory_mm / EXPANSION_CROSSINGS
        offsets = [(index + 0.5) * step for index in range(EXPANSION_CROSSINGS)]
        if dx:
            x = (source[0] + (1 if dx > 0 else 0)) * self.territory_mm
            points = [(x, source[1] * self.territory_mm + offset) for offset in offsets]
        else:
            y = (source[1] + (1 if dy > 0 else 0)) * self.territory_mm
            points = [(source[0] * self.territory_mm + offset, y) for offset in offsets]
        return [
            point
            for point in points
            if not any(
                math.hypot(point[0] - bx, point[1] - by) <= WALL_SEGMENT_AVOID_MM
                for bx, by in self.blockers
            )
            and not any(
                point_segment_distance(point, wall_start, wall_end)
                <= WALL_SEGMENT_AVOID_MM
                for wall_start, wall_end in self.wall_segments
            )
        ]

    def get_territory_expansions(self):
        """Return open directed expansions from explored unlocked territories."""
        allowed = set(self.territories)
        expansions = set()
        for source in self.territories:
            if source in self.abandoned or not self.territory_explored(source):
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                target = (source[0] + dx, source[1] + dy)
                expansion = (source, target)
                if (
                    target not in allowed
                    and target not in self.abandoned
                    and expansion not in self.blocked_territory_expansions
                    and self._territory_expansion_crossings(source, target)
                ):
                    expansions.add(expansion)
        return expansions

    def _select_territory_expansion(self):
        """Keep the active expansion stable, or select the nearest open crossing."""
        if any(not self.territory_explored(cell) for cell in self.territories):
            # Finish unlocked work before planning growth. Otherwise a future
            # expansion's completed source is exempted by forward_distance and
            # can pull the robot out of the territory it is still covering.
            self.active_territory_expansion = None
            self.active_expansion_crossing = None
            return
        expansions = self.get_territory_expansions()
        x, y = self.path_points[-1] if self.path_points else (0.0, 0.0)
        if self.active_territory_expansion in expansions:
            crossings = self._territory_expansion_crossings(
                *self.active_territory_expansion
            )
            if self.active_expansion_crossing in crossings:
                return
            self.active_expansion_crossing = min(
                crossings,
                key=lambda crossing: math.hypot(
                    crossing[0] - x, crossing[1] - y
                ),
            )
            return
        self.active_territory_expansion = None
        self.active_expansion_crossing = None
        if not expansions:
            return
        _, source, target, crossing = min(
            (
                math.hypot(crossing[0] - x, crossing[1] - y),
                source,
                target,
                crossing,
            )
            for source, target in expansions
            for crossing in self._territory_expansion_crossings(source, target)
        )
        self.active_territory_expansion = (source, target)
        self.active_expansion_crossing = crossing
        print(
            f'\n  [territory expansion selected] {source} -> {target} '
            f'via ({crossing[0]:.0f}, {crossing[1]:.0f})'
        )

    def _aim_toward_expansion(self, x, y, heading, clearance):
        """Score a heading by how well it points at the active boundary crossing.

        Keeping one target stable prevents every heading from receiving a strong
        score merely because there is some unopened territory in that direction.
        """
        self._select_territory_expansion()
        if self.active_expansion_crossing is None:
            return clearance
        hr = math.radians(heading)
        ux, uy = math.cos(hr), math.sin(hr)
        tx, ty = self.active_expansion_crossing
        distance = math.hypot(tx - x, ty - y)
        if not distance:
            return FRONTIER_HEADING_WEIGHT + min(clearance, self.grid_mm)
        alignment = ((tx - x) * ux + (ty - y) * uy) / distance
        return alignment * FRONTIER_HEADING_WEIGHT + min(clearance, self.grid_mm)

    def add_blocked_territory_expansion(self, x, y, heading):
        """Close an expansion only after physical evidence blocks every crossing."""
        expansion = self.active_territory_expansion
        if expansion is None:
            return
        source, target = expansion
        if territory_cell(x, y, self.territory_mm) != source:
            return
        if self.territory_ahead(x, y, heading) != target:
            return
        if self._territory_expansion_crossings(source, target):
            self.active_expansion_crossing = None
            self._select_territory_expansion()
            return
        self.blocked_territory_expansions.add(expansion)
        self.active_territory_expansion = None
        self.active_expansion_crossing = None
        print(f'\n  [territory expansion blocked] {source} -> {target}')
        self._select_territory_expansion()

    def expand_past_boundary(self, x, y, heading):
        self._select_territory_expansion()
        current = territory_cell(x, y, self.territory_mm)
        ahead = self.territory_ahead(x, y, heading)
        if self.active_territory_expansion != (current, ahead):
            return False
        expanded = super().expand_past_boundary(x, y, heading)
        if expanded:
            self.active_territory_expansion = None
            self.active_expansion_crossing = None
        return expanded

    def is_complete(self):
        return (
            bool(self.territories)
            and all(self.territory_explored(cell) for cell in self.territories)
            and not self.get_territory_expansions()
        )

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
            and coverage == 0
            and not super().territory_explored(self.focus)
        ):
            # Has a frontier on paper but was never entered: give it up.
            self.abandoned.add(self.focus)
            print(
                f"\n  [focus abandoned] {self.focus}: no new cells in "
                f"{STALL_LEGS} legs; treating as unreachable in practice"
            )
            self._stall_legs = 0

    def heading_preference(self, x, y, heading):
        clearance = self.forward_distance(x, y, heading, self.territory_mm)
        resolution = territory_resolution(
            self.focus,
            self.path_points,
            self.blockers,
            self.wall_segments,
            self.territory_mm,
        )
        frontier = resolution['frontier']

        if self.focus in self.abandoned or not frontier:
            # Nothing left in the focus: steer toward the nearest expandable
            # frontier. Reaching a boundary means low clearance, and that is
            # exactly what triggers expand_past_boundary -- so this case must be
            # handled BEFORE the low-clearance no-progress penalty below, or the
            # robot would never commit to a boundary and could never expand.
            return self._aim_toward_expansion(x, y, heading, clearance)

        if clearance < MIN_USEFUL_FORWARD_MM:
            return -NO_PROGRESS_PENALTY + clearance

        forbidden = resolution['blocked'] | resolution['unreachable']
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
        data['blocked_territory_expansions'] = sorted(
            (list(source), list(target))
            for source, target in self.blocked_territory_expansions
        )
        if self.active_territory_expansion is not None:
            data['active_territory_expansion'] = [
                list(cell) for cell in self.active_territory_expansion
            ]
        return data
