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

When a leg stops early on an obstacle, go-home records that route segment as a
temporarily blocked edge in the map (its endpoints, the observed stop position,
and a timestamp). Subsequent go-home attempts exclude blocked edges when
planning, so the robot reroutes around a known obstruction instead of retrying
the same corridor. When every proven route home is blocked, the command reports
this clearly and refuses to move rather than retreating and retrying.

Because Dash is retracing space it already traversed, go-home relaxes its
obstacle criteria while following a proven corridor: it uses a higher proximity
threshold and a longer confirmation streak, so it can graze past walls it
already drove past while a solid head-on wall still stops it. When a leg begins
with a near-reversal turn that faces a wall Dash just drove away from, it takes a
short bounded clearance nudge to step off that wall before normal stopping
resumes.

Returning to the starting corner means approaching its walls head-on, so the
forward sensors stop Dash a short distance out. Once an obstacle halt occurs
within the near-home zone, go-home runs a final approach for precision:

1. It crawls straight at the home pose with a relaxed front threshold and slow,
   short steps, closing the remaining distance until it reaches position
   tolerance or is genuinely blocked.
2. It turns to the starting orientation.
3. It re-references to the rear wall, mirroring docking: reverse until the rear
   sensor finds the wall, then step forward by the dock clearance. This nails the
   rear axis. If the rear wall is not found within range, this step is skipped.

The run then completes at the starting pose and orientation.

Each motion reports why it halted (an obstacle with the triggering sensor
readings, a tilt, or a stalled or under-rotated turn), and go-home prints that
reason and stores it on the run.

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
- Blocked route segments discovered during go-home

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

## Blocked Go-Home Routes

Go-home remembers which route segments caused an early stop. Each safely aborted
attempt records the blocked segment in the map, and later attempts plan around
it. This avoids the earlier failure where a subsequent attempt re-selected the
same blocked corridor, retreating and retrying without progress.

How it works:

1. A go-home leg that stops early on an obstacle is recorded as a temporarily
   blocked edge, including its endpoints, timestamp, and observed stop position.
2. Blocked-edge data is preserved per run in the map JSON, so it survives across
   safely aborted go-home runs.
3. Planning excludes graph edges that run along a blocked corridor. A
   perpendicular crossing into a different corridor stays usable, so the robot
   reroutes through proven space when an alternative exists.
4. When no unblocked proven route home remains, go-home reports this and refuses
   to move.

Safety constraints honored by the implementation:

- Never infer an arbitrary shortcut through unknown space.
- Keep obstacle-aware movement enabled.
- Abort on rejected odometry or an early stop.
- Only continue from a partial go-home run when its final pose remains
  trustworthy.
- Avoid repeatedly commanding motion into the same blocked segment.

### Next Challenge: Local Detour Exploration

The planner reroutes only through space it has already proven. When every known
route is blocked it stops. A future implementation could perform bounded local
exploration to discover a new connection around an obstruction, while always
preserving a proven retreat route back to known-safe space.

### Next Challenge: Path-Aware Obstacle Handling on the Way Home

Dash's obstacle sensing for forward motion only looks ahead (the left and right
proximity sensors; see `_obstacle_in_path` in `dash/motion.py`). A wall that sat
behind Dash during outbound travel becomes a wall directly ahead after the turn
home, even though Dash already traversed that space. Go-home currently mitigates
this with blunt, geometry-free heuristics — a relaxed proximity threshold and
confirmation streak while retracing, a clearance nudge after a near-reversal
turn, and accepting a near-home obstacle as arrival. These help, but they trade
away safety margin uniformly rather than reasoning about what the obstacle is.

The remaining work is to make the obstacle handling *path-aware*: distinguish a
*known* wall already in the map near the proven segment from a *new* unmapped
obstacle. Compare the triggering stop position (the halt outcome carries the
exact prox readings and stop position) against `run['walls']` along the current
leg. If the detected wall coincides with a mapped wall on a segment Dash already
traversed, continue at normal sensitivity; otherwise stop as today. This would
let go-home keep full obstacle sensitivity for genuinely new obstructions while
no longer halting on walls it has already proven passable — replacing the blunt
relaxation with a principled decision.

Safety constraints:

- Never disable tilt stopping.
- Always stop for new or unmapped obstacles, and for anything not on the proven
  path.
- Keep obstacle-aware movement enabled; do not blanket-disable forward detection.
- Continue recording genuinely blocked segments as blocked edges.

Relevant code: `go_home` (the move loop, `needs_wall_clearance`,
`obstacle_arrival_near_home`) and `describe_halt` in
`examples/mapping/map_room.py`; `move()`, `_obstacle_in_path`, and the
`PROXIMITY_*` constants in `dash/motion.py`; mapped walls in each run's `walls`
list in the room-map JSON.

### Next Challenge: Detecting Wheel Slip

Wheel odometry measures wheel *rotation*, not physical movement. On a low bump,
threshold, or slick spot, the tires can keep turning while Dash makes little or
no real progress. The encoders advance, so the pose estimate moves even though
the robot did not — and every later pose, map observation, and go-home route is
then built on a wrong position. Dash has no independent position sensor (no
optical flow or external reference), which makes slip genuinely hard to detect.
The available sensors are the two wheel encoders, the gyro (yaw), pitch and roll,
the accelerometer, and the proximity sensors.

There are two distinct cases, with different detectability:

1. **Differential slip (one wheel).** One tire grips while the other slips or
   spins free. This is detectable by cross-checking the wheels against the gyro:
   the left/right encoder difference implies a heading change (through the track
   width and `mm_per_wheel_tick`), which can be compared to the heading change
   the gyro actually measured. A persistent mismatch — the wheels imply a turn
   the gyro does not see, or vice versa — indicates a slipping wheel. This reuses
   data the mapper already reads every step (`get_left_wheel`, `get_right_wheel`,
   `get_yaw`). Measuring the raw left/right odometer difference, as a first cut,
   is exactly this signal.
2. **Common-mode slip (both wheels).** Both tires spin equally while Dash sits
   still or is high-centered on a bump. This is the hard case: equal left and
   right deltas with no rotation look like a perfectly valid straight move, so
   the wheel-vs-gyro check cannot see it. Catching it needs a signal independent
   of the wheels:
   - **Accelerometer / pitch.** A real forward move produces acceleration
     transients and, climbing a bump, a pitch spike. Wheels advancing with a flat
     accelerometer (no motion) or a pitch event (stuck on a bump) is suspicious.
     Noisy, but Dash exposes `get_acceleration` and `get_pitch`.
   - **Proximity / landmark change.** When moving toward a known wall, the
     expected proximity reading should change as Dash closes in. Wheels advancing
     while a nearby landmark distance does not change suggests no real movement.
     This only works near mapped features (it is a stricter cousin of the
     existing loop-closure revisit correction).

A practical implementation could start with the differential wheel-vs-gyro check
(cheap, always available, catches one-wheel slip) and add an accelerometer-based
"wheels turning but not accelerating" heuristic for the high-centered case. On
suspected slip, stop the run, mark the segment uncertain (mirroring how
implausible odometry is already isolated), and either re-reference against a
known wall or require re-docking rather than trusting the drifted pose.

Safety and correctness constraints:

- Prefer a false stop over silently trusting a slipped pose; a corrupted pose
  poisons all later mapping and any go-home route.
- Keep the existing odometry validation and loop-closure corrections; slip
  detection augments them rather than replacing them.

Relevant code: `update_pose` and `validate_odometry` in
`examples/mapping/map_room.py`; the `get_left_wheel`, `get_right_wheel`,
`get_yaw`, `get_pitch`, and `get_acceleration` sensors in `dash/sensors.py`.

## Command Reference

Show all mapper options:

```bash
uv run --extra tools examples/mapping/map_room.py --help
```

Show all calibration options:

```bash
uv run examples/mapping/calibrate.py --help
```
