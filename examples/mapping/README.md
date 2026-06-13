# Room Mapping

These tools calibrate Dash's odometry, autonomously explore a room, preserve a
2D map, and return the robot to its starting pose.

They communicate with physical hardware through the running WebSocket server.
Keep Dash on the floor in a clear area, away from stairs, and be ready to stop
it. Proximity and tilt detection are best-effort safeguards and cannot detect
every obstacle.

## Setup

Install the project and the optional Matplotlib dependency used to render map
images:

```bash
uv sync --extra tools
```

Start the WebSocket server in a separate terminal and leave it running:

```bash
uv run pydashbot-server
```

Only one process can maintain the Bluetooth connection to Dash. The calibration
and mapping scripts use the running server instead of connecting directly.

## Calibration

Place Dash in open space with at least 300 mm of clear floor ahead and enough
room to turn. Run:

```bash
uv run examples/mapping/calibrate.py --output data/calibration.json
```

The calibrator:

1. Turns 90 degrees to measure the yaw scale.
2. Moves forward 300 mm to measure the wheel-distance scale.
3. Moves backward and turns back to approximately its starting pose.
4. Writes the measured scales to JSON.

Without `--output`, it creates a timestamped file such as
`calibration_20260612-17-23-26.json`.

Calibration assumes the wheels move freely and do not slip. Recalibrate after
changing the wheels, using Dash on a substantially different floor surface, or
observing consistently inaccurate turns or distances.

## Fresh Exploration

For a consistent map origin, place Dash near a room corner:

1. Point its back roughly toward one wall.
2. Put its left side roughly toward the adjacent wall. Dash always turns left
   during docking, so this turn should make its front face the adjacent wall.
3. Leave roughly 100-200 mm of clearance from both walls for docking.
4. Point its front diagonally into the open room.
5. Ensure the head tip and proximity sensors are unobstructed.
6. Clear nearby cables, small objects, stairs, and drop-offs.

The mapper uses this fixed docking sequence to establish the starting pose and
orientation:

1. Reverse until the rear proximity sensor detects the first wall.
2. Move forward 80 mm to clear that wall.
3. Turn left 90 degrees.
4. Drive forward until a front proximity sensor detects the adjacent wall.
5. Reverse 80 mm to clear that wall.
6. Turn right 90 degrees to face into the room.

It then explores until the requested duration ends:

```bash
uv run --extra tools examples/mapping/map_room.py \
  --calibration data/calibration.json \
  --output data/room_map.json \
  --duration 300
```

`--duration` is measured in seconds and defaults to 60. A fresh run with
`--output` replaces an existing file at that path. Without `--output`, the
mapper creates a timestamped file such as
`room_map_20260612-17-23-26.json`.

The mapper also renders a PNG beside the JSON using the same basename.

After exploration, do not manually move Dash before running `--go-home`.

## Go Home

After a fresh exploration, do not manually move the robot. Return it to the
map's initial position and orientation with:

```bash
uv run --extra tools examples/mapping/map_room.py \
  --go-home data/room_map.json
```

Go-home chooses the shortest route along previously traversed path segments. It
does not invent shortcuts through unknown space. Movement remains
obstacle-aware, each leg is limited to 1 meter, and the return aborts if a leg
stops early or odometry becomes implausible.

The command refuses to start when the map's latest run does not have a
trustworthy final pose. The completed or aborted return is appended to the map.

Omit the filename after `--go-home` to use the newest room map in the current
directory.

## Start With A Map

To physically start again from the original corner while reusing prior map
knowledge:

```bash
uv run --extra tools examples/mapping/map_room.py \
  --start-with-map data/room_map.json \
  --duration 300
```

The mapper performs the corner-docking routine, anchors itself to the saved
starting pose, and chooses headings that favor unexplored space while avoiding
known walls and obstacles. The new run is appended to the selected map.

Omit the filename after `--start-with-map` to use the newest room map in the
current directory.

## Recommended Explore-And-Return Session

1. Start the WebSocket server.
2. Place Dash at the starting corner.
3. Run a fresh exploration:

```bash
uv run --extra tools examples/mapping/map_room.py \
  --calibration data/calibration.json \
  --output data/room_map.json \
  --duration 300
```

4. Confirm Dash has not been manually moved.
5. Return home:

```bash
uv run --extra tools examples/mapping/map_room.py \
  --go-home data/room_map.json
```

## Map Data And Quality

Room-map JSON files preserve every run, including:

- The calibration scales used by the map
- Robot path poses
- Wall and obstacle observations
- Raw odometry events
- Quality checks and loop-closure corrections
- Go-home planned routes and results

The mapper validates forward, reverse, and turn odometry after every command.
An implausible transition stops the run immediately and marks it `partial`.
The validated path prefix and observations remain available, while the rejected
transition and later data are isolated. Explicitly rejected runs remain in the
JSON for diagnosis but are excluded from strategy and visualization.

When the mapper revisits a known area, matching nearby landmarks can apply a
small, bounded correction to accumulated position drift. Large mismatches are
not automatically snapped together because they may represent unrelated walls
or severe wheel slip.

Wheel odometry measures wheel rotation, not physical movement. If Dash is
blocked while its wheels spin, the saved pose can become inaccurate. Stop the
run and avoid go-home if this happens.

## Command Reference

Show all mapper options:

```bash
uv run --extra tools examples/mapping/map_room.py --help
```

Show all calibration options:

```bash
uv run examples/mapping/calibrate.py --help
```
