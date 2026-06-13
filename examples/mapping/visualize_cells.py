"""Render a saved room map with conservative-exploration cell states."""

import argparse
import json
import math
from pathlib import Path

import matplotlib.patches as patches
import matplotlib.pyplot as plt

try:
    from examples.mapping.conservative_exploration import (
        GRID_CELLS,
        GRID_MM,
        TERRITORY_MM,
        territory_resolution,
    )
    from examples.mapping.exploration_walls import inferred_wall_segments
    from examples.mapping.map_room import plan_home_route
except ModuleNotFoundError:
    from conservative_exploration import (
        GRID_CELLS,
        GRID_MM,
        TERRITORY_MM,
        territory_resolution,
    )
    from exploration_walls import inferred_wall_segments
    from map_room import plan_home_route


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
    runs = accepted_runs(data)
    path_points = [
        (float(point[0]), float(point[1]))
        for run in runs
        for point in run.get('path', [])
    ]
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
    wall_segments = inferred_wall_segments(walls)
    policy = next(
        (
            run['conservative_exploration']
            for run in reversed(runs)
            if run.get('conservative_exploration')
        ),
        {},
    )
    focus = tuple(policy.get('focus_territory', (0, 0)))
    territories = [
        tuple(territory)
        for territory in policy.get('territories', [focus])
    ]
    if focus not in territories:
        territories.append(focus)
    resolutions = {
        territory: territory_resolution(
            territory, path_points, blockers, wall_segments
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
        px = [point[0] for point in rpath]
        py = [point[1] for point in rpath]
        ax.plot(px, py, '-', color=color, alpha=0.4, linewidth=1,
                label=f'Run {index + 1} ({run["timestamp"][:10]})')
        ax.plot(px[0], py[0], 'o', color=color, markersize=8, zorder=6)
        ax.plot(px[-1], py[-1], 's', color=color, markersize=7, zorder=6)
        step = max(1, len(rpath) // 15)
        for point_index in range(0, len(rpath) - step, step):
            start = rpath[point_index]
            end = rpath[point_index + step]
            if math.hypot(end[0] - start[0], end[1] - start[1]) > 1:
                ax.annotate(
                    '', xy=end[:2], xytext=start[:2],
                    arrowprops=dict(arrowstyle='->', color=color, lw=1),
                )

    for territory, resolution in resolutions.items():
        x0 = territory[0] * TERRITORY_MM
        y0 = territory[1] * TERRITORY_MM
        for cell_x in range(GRID_CELLS):
            for cell_y in range(GRID_CELLS):
                cell = (cell_x, cell_y)
                status = next(
                    name
                    for name in ('visited', 'blocked', 'unreachable', 'frontier')
                    if cell in resolution[name]
                )
                rectangle = patches.Rectangle(
                    (x0 + cell_x * GRID_MM, y0 + cell_y * GRID_MM),
                    GRID_MM,
                    GRID_MM,
                    facecolor=CELL_COLORS[status],
                    edgecolor='#111111' if territory == focus else '#555555',
                    linewidth=2.25 if territory == focus else 1.5,
                    alpha=0.22,
                    zorder=1,
                )
                ax.add_patch(rectangle)
                ax.text(
                    x0 + (cell_x + 0.5) * GRID_MM,
                    y0 + (cell_y + 0.5) * GRID_MM,
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
                [start[0], end[0]], [start[1], end[1]],
                color='#d62728', linestyle='--', linewidth=0.8, alpha=0.25,
                label='Inferred continuous walls' if index == 0 else None,
                zorder=3,
            )
    if walls:
        ax.scatter(*zip(*walls), c='red', s=80, marker='x', linewidths=2,
                   label=f'Wall ({len(walls)} pts)', zorder=7)
    if obstacles:
        ax.scatter(*zip(*obstacles), c='orange', s=80, marker='^',
                   label=f'Obstacle ({len(obstacles)} pts)', zorder=7)

    if options.home_route:
        home_route = plan_home_route(data)
        route_x = [point[0] for point in home_route]
        route_y = [point[1] for point in home_route]
        ax.plot(
            route_x,
            route_y,
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
                xy=point,
                xytext=(7, 7),
                textcoords='offset points',
                color='#003399',
                fontsize=9,
                weight='bold',
                zorder=10,
            )

    ax.plot([0, TERRITORY_MM], [0, 0], 'k-', linewidth=2, alpha=0.5,
            label='Dock walls')
    ax.plot([0, 0], [0, TERRITORY_MM], 'k-', linewidth=2, alpha=0.5)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.2)
    ax.legend(fontsize=8, loc='upper left', bbox_to_anchor=(1.01, 1))
    ax.set_title(
        f'Room Map with {len(territories)} Unlocked Territory Cell Grids - '
        f'Focus {focus}'
        f'{" - Planned Go-Home Route" if options.home_route else ""}',
        fontsize=14,
    )
    ax.set_xlabel('x (mm)')
    ax.set_ylabel('y (mm)')
    min_x = min(territory[0] * TERRITORY_MM for territory in territories)
    max_x = max((territory[0] + 1) * TERRITORY_MM for territory in territories)
    min_y = min(territory[1] * TERRITORY_MM for territory in territories)
    max_y = max((territory[1] + 1) * TERRITORY_MM for territory in territories)
    ax.set_xlim(min(-200, min_x - 200), max(TERRITORY_MM + 200, max_x + 200))
    ax.set_ylim(min(-200, min_y - 200), max(TERRITORY_MM + 200, max_y + 200))
    plt.tight_layout()

    output = options.output or options.map_file.with_name(
        f'{options.map_file.stem}_cells.png'
    )
    plt.savefig(output, dpi=180, bbox_inches='tight')
    print(f'Cell map image saved -> {output}')


if __name__ == '__main__':
    main()
