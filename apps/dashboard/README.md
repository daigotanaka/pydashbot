# Map Dashboard App

`apps.dashboard` serves a live browser dashboard for mapping sessions and
renders saved map JSON files into standalone HTML animations.

The dashboard does not talk to Dash directly. In live mode, it receives poses
and observations from `apps.map` over HTTP. By default it also starts
`apps.map` as a subprocess so one command can launch both the dashboard and the
mapping run.

## Quick Start

Start the robot WebSocket server in one terminal:

```bash
uv run dash.remote.server
```

Create `data/config.yaml` with mapper and dashboard settings:

```yaml
map_file: data/room_map.json
calibration: data/calibration/calibration.json
duration_seconds: 60
territory_size_mm: 1000

policies:
  exploration: conservative
  navigation: d-star-lite

docking:
  init: true

dashboard:
  active: true
  host: 127.0.0.1
  port: 8000
```

Run the live dashboard and map app together:

```bash
uv run apps.dashboard --config data/config.yaml
```

Open `http://127.0.0.1:8000`.

Resume an existing map through the dashboard:

```bash
uv run apps.dashboard --config data/config.yaml --map-mode resume
```

Serve only the dashboard, without launching the map app:

```bash
uv run apps.dashboard --no-map --host 127.0.0.1 --port 8000
```

Render a saved map to a standalone animation:

```bash
uv run apps.dashboard data/room_map.json
```

That writes `data/room_map_animation.html` unless `--output` is provided.

## Live Mode

In live mode the dashboard starts an HTTP server and keeps an in-memory payload
for the current session. The page polls `/state` and redraws the robot pose,
path, territories, cell states, wall observations, obstacle observations, and
inferred wall segments.

When `--no-map` is not set, the dashboard:

1. Reads the mapping config from `--config` (`data/config.yaml` by default).
2. Uses `dashboard.host`, `dashboard.port`, and `territory_size_mm` from that
   config unless overridden by CLI flags.
3. Starts its HTTP server first.
4. Launches `python -m apps.map <mode> --config <config>`.
5. Keeps serving after the map process exits so the result can be inspected or
   exported.

The map app must have `dashboard.active: true` in the same config if you want
live poses to appear.

## Static Export Mode

Passing a map JSON path with no live-server flags renders an HTML replay:

```bash
uv run apps.dashboard data/room_map.json --output out/map.html
```

The output file embeds all map data, CSS, and JavaScript. It can be opened
locally or shared without running the dashboard server.

Useful static options:

- `--output FILE`: choose the HTML output path.
- `--territory-size MM`: override the territory size recorded in the map.
- `--title TEXT`: set the page title shown in the animation.

## Configuration

The dashboard reads the mapper config rather than having its own config file.
These keys are most relevant:

```yaml
territory_size_mm: 1000

dashboard:
  active: true
  host: 127.0.0.1
  port: 8000
```

`dashboard.host` is the address the live server binds to. Use `127.0.0.1` for
local-only access. Use `0.0.0.0` only on a trusted network.

`dashboard.port` is the HTTP port. If another process is using it, choose a
different port in the config or pass `--port`.

`territory_size_mm` controls the dashboard grid and should match the mapper's
exploration setting.

## HTTP API

The mapper uses these endpoints automatically:

- `GET /`: live dashboard page.
- `GET /state`: current dashboard payload.
- `POST /move`: append a predicted or measured robot frame.
- `PUT /move`: amend the most recent frame after the robot finishes a leg.
- `POST /seed`: seed prior map coverage when resuming.
- `POST /map`: replace the dashboard state with an authoritative map JSON.
- `GET /map.json`: download the current map JSON.
- `GET /animation.html`: download a standalone HTML animation.

`POST /move` accepts either one move object or `{"moves": [...]}`. A move can
use `pose: [x, y, heading]` or separate `x`, `y`, and `heading` fields. Optional
fields include `duration`, `timestamp`, `walls`, `obstacles`, `event`, and
`arc`.

## Important Concepts

Predicted frames make the live view responsive. The map app posts the expected
end pose before a leg runs, then amends that frame with the measured pose and
observations when the leg finishes.

Seeded coverage lets resumed sessions show prior map knowledge immediately.
Seeds do not create animation frames; they prime the visited path and blockers
used to resolve cell states.

Cell states mirror the mapper's territory resolution. The dashboard recomputes
visited, frontier, blocked, and unreachable cells from the cumulative path and
observations so the live view and saved replay agree.

Static replays reveal observations progressively. A saved map is converted into
frames so walls, obstacles, inferred wall segments, and cell states appear at
the point in the path where they were discovered.

## Files

- `apps/dashboard/server.py`: CLI, live HTTP server, map import/export, payload
  construction, HTML rendering, and embedded browser UI.
- `apps/dashboard/__main__.py`: module entry point for `uv run apps.dashboard`.
- `apps/map/main.py`: publisher used by live mapping sessions.
- `apps/map/policies/exploration/conservative_exploration.py`: shared grid and
  territory helpers used by the dashboard renderer.

## Commands

```bash
uv run apps.dashboard --help
uv run apps.dashboard --config data/config.yaml
uv run apps.dashboard --config data/config.yaml --map-mode resume
uv run apps.dashboard --no-map --host 127.0.0.1 --port 8000
uv run apps.dashboard data/room_map.json --output data/room_map_animation.html
```
