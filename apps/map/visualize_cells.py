"""Render a saved room map with conservative-exploration cell states."""

import argparse
import json
import math
from pathlib import Path

import matplotlib.patches as patches
import matplotlib.pyplot as plt

try:
    from apps.map.policies.conservative_exploration import (
        GRID_CELLS,
        TERRITORY_MM,
        densify_path,
        territory_resolution,
    )
    from apps.map.exploration_walls import inferred_wall_segments
    from apps.map.main import plan_home_route
except ModuleNotFoundError:
    from policies.conservative_exploration import (
        GRID_CELLS,
        TERRITORY_MM,
        densify_path,
        territory_resolution,
    )
    from exploration_walls import inferred_wall_segments
    from main import plan_home_route


COLORS = ['#4477cc', '#44aa77', '#cc7744', '#aa44aa', '#cc4444']
CELL_COLORS = {
    'visited': '#6fcf97',
    'frontier': '#f2c94c',
    'blocked': '#eb5757',
    'unreachable': '#bdbdbd',
}


def accepted_runs(data):
    return [
        run
        for run in data.get('runs', [])
        if run.get('status', 'accepted') in {'accepted', 'partial'}
    ]


def render_cell_map(data, output, home_route=False):
    runs = accepted_runs(data)
    walls = [
        (float(point[0]), float(point[1]))
        for run in runs
        for point in run.get('walls', [])
    ]
    obstacles = [
        (float(point[0]), float(point[1]))
        for run in runs
        for point in run.get('obstacles', [])
    ]
    blockers = walls + obstacles
    policy = next(
        (
            run['conservative_exploration']
            for run in reversed(runs)
            if run.get('conservative_exploration')
        ),
        {},
    )
    focus = tuple(policy.get('focus_territory', (0, 0)))
    territory_mm = float(policy.get('territory_size_mm', TERRITORY_MM))
    grid_mm = territory_mm / GRID_CELLS
    path_points = [
        point
        for run in runs
        for point in densify_path(run.get('path', []), grid_mm / 2)
    ]
    # Match the explorer: link wall observations within one reachability cell.
    wall_segments = inferred_wall_segments(walls, max_distance=grid_mm)
    territories = [
        tuple(territory)
        for territory in policy.get('territories', [focus])
    ]
    if focus not in territories:
        territories.append(focus)
    resolutions = {
        territory: territory_resolution(
            territory, path_points, blockers, wall_segments, territory_mm
        )
        for territory in territories
    }

    fig, ax = plt.subplots(figsize=(13, 11))
    ax.plot(0, 0, 'k+', markersize=16, markeredgewidth=2, zorder=8,
            label='Corner (0,0)')

    for index, run in enumerate(runs):
        rpath = run.get('path', [])
        if not rpath:
            continue
        color = COLORS[index % len(COLORS)]
        # Axes are transposed: world x is drawn vertical, world y horizontal,
        # to match how the room reads from the dock.
        px = [point[0] for point in rpath]
        py = [point[1] for point in rpath]
        ax.plot(py, px, '-', color=color, alpha=0.4, linewidth=1,
                label=f'Run {index + 1} ({run["timestamp"][:10]})')
        ax.plot(py[0], px[0], 'o', color=color, markersize=8, zorder=6)
        ax.plot(py[-1], px[-1], 's', color=color, markersize=7, zorder=6)
        step = max(1, len(rpath) // 15)
        for point_index in range(0, len(rpath) - step, step):
            start = rpath[point_index]
            end = rpath[point_index + step]
            if math.hypot(end[0] - start[0], end[1] - start[1]) > 1:
                ax.annotate(
                    '', xy=(end[1], end[0]), xytext=(start[1], start[0]),
                    arrowprops=dict(arrowstyle='->', color=color, lw=1),
                )

    for territory, resolution in resolutions.items():
        x0 = territory[0] * territory_mm
        y0 = territory[1] * territory_mm
        for cell_x in range(GRID_CELLS):
            for cell_y in range(GRID_CELLS):
                cell = (cell_x, cell_y)
                status = next(
                    name
                    for name in ('visited', 'blocked', 'unreachable', 'frontier')
                    if cell in resolution[name]
                )
                rectangle = patches.Rectangle(
                    (y0 + cell_y * grid_mm, x0 + cell_x * grid_mm),
                    grid_mm,
                    grid_mm,
                    facecolor=CELL_COLORS[status],
                    edgecolor='#111111' if territory == focus else '#555555',
                    linewidth=2.25 if territory == focus else 1.5,
                    alpha=0.22,
                    zorder=1,
                )
                ax.add_patch(rectangle)
                ax.text(
                    y0 + (cell_y + 0.5) * grid_mm,
                    x0 + (cell_x + 0.5) * grid_mm,
                    f'{territory} cell {cell_x},{cell_y}\nvisited: '
                    f'{"yes" if cell in resolution["visited"] else "no"}\n{status}',
                    ha='center',
                    va='center',
                    fontsize=7,
                    weight='bold' if cell in resolution['visited'] else 'normal',
                    zorder=2,
                )

    if wall_segments:
        for index, (start, end) in enumerate(wall_segments):
            ax.plot(
                [start[1], end[1]], [start[0], end[0]],
                color='#d62728', linestyle='--', linewidth=0.8, alpha=0.25,
                label='Inferred continuous walls' if index == 0 else None,
                zorder=3,
            )
    if walls:
        wx, wy = zip(*walls)
        ax.scatter(wy, wx, c='red', s=80, marker='x', linewidths=2,
                   label=f'Wall ({len(walls)} pts)', zorder=7)
    if obstacles:
        ox_, oy_ = zip(*obstacles)
        ax.scatter(oy_, ox_, c='orange', s=80, marker='^',
                   label=f'Obstacle ({len(obstacles)} pts)', zorder=7)

    if home_route:
        home_route = plan_home_route(data)
        route_x = [point[0] for point in home_route]
        route_y = [point[1] for point in home_route]
        ax.plot(
            route_y,
            route_x,
            color='#0057ff',
            linewidth=4,
            marker='o',
            markersize=6,
            label=f'Planned go-home route ({len(home_route)} waypoints)',
            zorder=9,
        )
        for index, point in enumerate(home_route):
            ax.annotate(
                str(index),
                xy=(point[1], point[0]),
                xytext=(7, 7),
                textcoords='offset points',
                color='#003399',
                fontsize=9,
                weight='bold',
                zorder=10,
            )

    start_y = float(runs[0]['path'][0][1]) if runs and runs[0].get('path') else -1
    wall_y = -territory_mm if start_y < 0 else territory_mm
    # Transposed axes: pass (horizontal=world y, vertical=world x).
    ax.plot([0, 0], [0, territory_mm], 'k-', linewidth=2, alpha=0.5,
            label='Dock walls')
    ax.plot([0, wall_y], [0, 0], 'k-', linewidth=2, alpha=0.5)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.2)
    ax.legend(fontsize=8, loc='upper left', bbox_to_anchor=(1.01, 1))
    ax.set_title(
        f'Room Map with {len(territories)} Unlocked Territory Cell Grids - '
        f'Focus {focus}'
        f'{" - Planned Go-Home Route" if home_route else ""}',
        fontsize=14,
    )
    ax.set_xlabel('y (mm)')
    ax.set_ylabel('x (mm)')
    min_x = min(territory[0] * territory_mm for territory in territories)
    max_x = max((territory[0] + 1) * territory_mm for territory in territories)
    min_y = min(territory[1] * territory_mm for territory in territories)
    max_y = max((territory[1] + 1) * territory_mm for territory in territories)
    # Horizontal axis is world y, vertical axis is world x.
    ax.set_xlim(min(-200, min_y - 200), max(territory_mm + 200, max_y + 200))
    ax.set_ylim(min(-200, min_x - 200), max(territory_mm + 200, max_x + 200))
    plt.tight_layout()

    plt.savefig(output, dpi=180, bbox_inches='tight')
    print(f'Cell map image saved -> {output}')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_file', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument(
        '--home-route',
        action='store_true',
        help='overlay the route the current go-home planner would follow',
    )
    options = parser.parse_args()

    data = json.loads(options.map_file.read_text())
    output = options.output or options.map_file.with_name(
        f'{options.map_file.stem}_cells.png'
    )
    render_cell_map(data, output, home_route=options.home_route)


if __name__ == '__main__':
    main()
