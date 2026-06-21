# Room Mapping App

`apps.map` calibrates Dash's odometry, explores a room, saves a 2D map, and can
return the robot to its starting pose. It talks to Dash through the WebSocket
robot server, not directly over Bluetooth.

Keep Dash on the floor in a clear area, away from stairs and fragile objects,
and be ready to stop it. Proximity, tilt, and odometry checks are best-effort
safeguards.

## Quick Start

Start the robot WebSocket server in one terminal:

```bash
uv run dash.remote.server
```

Calibrate in open space:

```bash
mkdir -p data/calibration
uv run apps/map/calibrate.py --output data/calibration/calibration.json
```

Create or edit a mapping config:

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
  active: false
  host: 127.0.0.1
  port: 8000
```

Run from that config:

```bash
uv run apps.map start --config data/config.yaml
uv run apps.map resume --config data/config.yaml
uv run apps.map dock --config data/config.yaml
```

The bundled example config lives at `apps/map/config/config.yaml`. If you omit
`--config`, `apps.map` uses that file.

## Run Modes

`start` begins from the dock pose. If `map_file` does not exist, it creates a
new map. If the file already exists, it starts from the dock pose again while
reusing the saved map knowledge.

`resume` requires an existing `map_file` and continues from the final saved
pose in that map. Use this only when Dash has not been manually moved since the
map was saved, or when you have deliberately placed it at that saved pose.

`dock` requires an existing `map_file` and returns home from the final saved
pose. It plans over previously traversed path segments and does not invent
shortcuts through unknown space. `dock` is experimental feature, and it might
not work at all.

## Configuration

`map_file` is required. It is the JSON file where runs, wall observations,
obstacle observations, calibration scales, and policy metadata are saved.

`calibration` is optional. When present, it points to a JSON file produced by
`apps/map/calibrate.py`. When omitted for a fresh map, the mapper uses the
newest timestamped calibration file in the current directory.

`duration_seconds` controls how long exploration runs before saving. It must be
positive. The `preset` exploration policy ignores the time budget and stops
after its command list is exhausted.

`territory_size_mm` controls the size of a square exploration territory. The
default is `1000`. Conservative and coverage exploration use this to keep the
map growth bounded and compact.

`policies.exploration` can be a policy name:

- `conservative`: bounded territories and frontier-oriented exploration.
- `coverage`: bounded territories with stronger preference for newly visited
  cells.
- `novelty`: free-roaming preference for unexplored space.
- `wall-follower`: arc-based wall following.
- `preset`: replay a fixed JSON course.

Policy options use a mapping with `name`. For example:

```yaml
policies:
  exploration:
    name: preset
    input_file: data/preset_courses/course.json
```

Preset course files contain either a command list or an object with a
`commands` list:

```json
{
  "commands": [
    {"command": "move", "value": 250},
    {"command": "turn", "value": 90},
    {"command": "move", "value": 250, "stop_at_obstacle": true}
  ]
}
```

`policies.navigation` selects the route planner used by `dock`:

- `d-star-lite`: replans with soft costs around failed approaches.
- `hard-blocked-edge`: excludes blocked corridors more aggressively.

## Custom Policies

Exploration and navigation policies are ordinary Python classes selected by
name from the YAML config. Add a new policy when you want to experiment with
the robot's decision making without changing the CLI, map format, calibration,
or persistence code.

Custom exploration policies live in `apps/map/policies/exploration/`. Subclass
`ExplorationPolicy` from `exploration_policy_base.py`, give the class a stable
`metadata_key`, and implement `heading_preference(x, y, heading)`. The mapper
scores candidate headings by combining that positive preference with its shared
physical safety penalties. Use `from_context(cls, context)` when the policy
needs live path points, blockers, wall segments, territory size, or
policy-specific YAML options.

```python
from apps.map.policies.exploration.exploration_policy_base import ExplorationPolicy


class MyExplorationPolicy(ExplorationPolicy):
    metadata_key = "my_exploration"

    @classmethod
    def from_context(cls, context):
        return cls(context.known_path, context.exploration_options)

    def __init__(self, known_path, options):
        self.known_path = known_path
        self.weight = float(options.get("weight", 1.0))

    def heading_preference(self, x, y, heading):
        return self.weight
```

Bounded or self-driving exploration policies can also override hooks such as
`allows_point`, `forward_distance`, `unlock_if_complete`, `is_complete`, and
`drive`. See `conservative_exploration.py`, `coverage_exploration.py`, and
`wall_follower.py` for examples of those stronger control surfaces.

Register the exploration policy in `EXPLORATION_POLICIES` in `apps/map/main.py`
and select it from config:

```python
EXPLORATION_POLICIES = {
    "my-policy": MyExplorationPolicy,
    ...
}
```

```yaml
policies:
  exploration:
    name: my-policy
    weight: 2.0
```

Custom navigation policies live in `apps/map/policies/navigation/`. Subclass
`NavigationPolicy` from `navigation_policy_base.py`, set a unique `name`, and
implement `plan_route(data, accepted_runs, run_pose_trustworthy,
target_xy=None)`. Return a list of `(x, y)` route points over proven map paths,
or raise `ValueError` when no safe route exists. The helpers in
`navigation_policy_base.py` build and simplify the proven path graph used by
the bundled planners.

```python
from apps.map.policies.navigation.navigation_policy_base import NavigationPolicy


class MyNavigationPolicy(NavigationPolicy):
    name = "my-navigation"

    def plan_route(self, data, accepted_runs, run_pose_trustworthy, target_xy=None):
        raise ValueError("no route found")
```

Register an instance in `GO_HOME_STRATEGIES` in `apps/map/main.py` and select
it from config:

```python
GO_HOME_STRATEGIES = {
    "my-navigation": MyNavigationPolicy(),
    ...
}
```

```yaml
policies:
  navigation: my-navigation
```

Add focused tests next to the existing policy tests before running the robot.
`tests/test_coverage_exploration.py`, `tests/test_wall_follower.py`, and
`tests/test_navigation.py` are good templates.

`docking.init` controls whether `start` runs the physical docking sequence. Keep
it `true` for ordinary fresh runs. Set it to `false` only when you have manually
placed Dash at the established start pose.

`dashboard.active` makes the mapper publish live poses to an already running
dashboard server. `dashboard.host` and `dashboard.port` must match the dashboard
listener. `dock` does not publish live dashboard poses.

## Calibration

Calibration measures two scales:

- `deg_per_yaw`: converts Dash yaw sensor units into degrees.
- `mm_per_wheel_tick`: converts wheel encoder ticks into millimeters.

Run calibration on a flat surface with at least 300 mm of clear floor ahead and
enough room to turn. Recalibrate after changing floor surfaces, wheels, or when
turns and distances are consistently off.

Without `--output`, the calibrator writes a timestamped file such as
`calibration_20260620-14-30-00.json` in the current directory.

## Generated Files

The map JSON uses `schema_version: 2` and stores:

- `calibration`: the scales used for the saved map.
- `runs`: every exploration or go-home run, including status, path, events,
  walls, obstacles, and policy metadata.
- `walls` and `obstacles`: aggregate observations from accepted and partial
  runs.

During save, the mapper writes through a temporary `*.tmp` file and then
replaces the target map file.

The dashboard app can turn a saved map JSON into a standalone HTML animation:

```bash
uv run apps.dashboard data/room_map.json
```

## Important Concepts

The dock pose defines the map frame. The docking sequence backs into one wall,
turns to find the adjacent wall, and establishes a repeatable start. The current
start position is approximately `(310, 310)` mm with heading `0` pointing into
the room.

Odometry is trusted only while it remains plausible. The mapper compares wheel
ticks, gyro heading, commanded motion, obstacle stops, and loop-closure
landmarks. If a run looks corrupted by slip or implausible motion, it is stopped
or marked so later planning does not silently depend on bad pose data.

Territories are square regions of the map. Bounded policies unlock adjacent
territories as reachable cells are resolved, which keeps exploration from
wandering too far before the local area is understood.

Cell states are recomputed from the path and blockers:

- `visited`: Dash has driven through the cell.
- `frontier`: reachable but not yet visited.
- `blocked`: wall or obstacle evidence blocks it.
- `unreachable`: not safely reachable from the known traversed area.

Go-home routing uses the proven path graph. A failed approach can raise route
costs or block a corridor depending on the selected navigation policy, then the
planner retries within bounded limits.

## Files

- `apps/map/main.py`: CLI, config validation, docking, exploration, go-home,
  odometry validation, dashboard publishing, and map persistence.
- `apps/map/calibrate.py`: yaw and wheel-distance calibration script.
- `apps/map/exploration_walls.py`: wall-segment inference and wall geometry
  helpers.
- `apps/map/config/config.yaml`: example mapper configuration.
- `apps/map/policies/exploration/`: exploration policy implementations.
- `apps/map/policies/navigation/`: go-home route planning policies.
- `tests/test_mapping.py`, `tests/test_navigation.py`, and related policy tests:
  regression coverage for config, mapping behavior, and planners.

## Commands

```bash
uv run apps.map --help
uv run apps.map start --config data/config.yaml
uv run apps.map resume --config data/config.yaml
uv run apps.map dock --config data/config.yaml
uv run apps/map/calibrate.py --help
```
