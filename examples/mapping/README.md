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

By default the mapper explores with the conservative-territory policy (see
[Conservative Exploration](#conservative-exploration)): it confines itself to an
unlocked square territory and repeatedly picks a forward heading that drives
*into* a reachable, still-unvisited cell to test whether it is reachable, rather
than re-driving cells it has already visited. Each leg stops early on a wall, a
tilt, or a territory boundary; odometry is validated after every command and
loop-closure corrections against known landmarks bound the drift. As cells are
resolved the policy unlocks adjacent territory so the explored region grows
compactly. Pass `--no-conservative-exploration` for undirected forward legs, or
`--territory-size MM` to change the territory granularity.

`--duration` is measured in seconds and defaults to 60. A fresh run with
`--output` replaces an existing file at that path. Without `--output`, the
mapper creates a timestamped file such as
`room_map_20260612-17-23-26.json`.

The mapper renders a PNG beside the JSON using the same basename: a cell-grid
overlay showing the conservative-exploration territories with each cell's
visited / blocked / unreachable / frontier state, the robot path, and the wall
observations.

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

A single `--go-home` invocation replans automatically: if an attempt aborts on a
blockage while keeping a trustworthy pose, it records the blocked edge and tries
an alternative proven route from where Dash stopped, up to a few times, until it
reaches home or no unblocked route remains. Retries stop early if the pose is no
longer trustworthy or the halt was not a blockage (for example a turn that did
not execute), since replanning would not help those cases.

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
- Conservative-exploration territory progress
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

### Next Challenge: Blocked Edges Over-Block Parallel Proven Corridors

A blocked edge can still remove a *different*, already-driven corridor from the
plan, producing a false "no unblocked route home" even when a proven path
exists. `edge_is_blocked` excludes any graph edge whose midpoint falls within
`BLOCKED_EDGE_TOLERANCE_MM` of the blocked segment and runs roughly parallel to
it. The endpoint-sharing case is already guarded: a candidate edge is excluded
only when its midpoint projects onto the *interior* of the blocked segment, so a
corridor that merely shares an endpoint with it, or runs past its far end,
survives. What remains is collinear bleed — a genuinely parallel corridor that
overlaps the blocked segment's interior is still excluded even though Dash
already drove it.

Remaining directions:

1. **Tighten the blocked region further.** Block a small zone around the actual
   stop position rather than the whole route leg, reducing collinear bleed onto
   genuinely parallel neighboring corridors that still overlap interior-wise.
2. **Soft costs instead of hard exclusion.** Distinguish edges Dash actually
   drove (exploration path segments) from inferred proximity links, and
   downweight a proven-driven corridor that overlaps a blockage so it becomes a
   last resort instead of removing it — "we already drove this" is the strongest
   evidence of passability. This also guarantees a route whenever the proven
   graph is connected.

Safety constraints:

- Still avoid re-driving the same blocked approach that failed.
- Prefer trying a proven corridor at reduced confidence over falsely reporting no
  route, but keep obstacle-aware movement enabled so a genuine obstruction still
  stops Dash.

Relevant code: `edge_is_blocked`, `_point_near_segment`, `collect_blocked_edges`,
and `plan_home_route` in `examples/mapping/map_room.py`; the per-run
`blocked_edges` records in the room-map JSON.

### Conservative Exploration

The mapper limits exploration to an initial square territory (2 x 2 meters by
default; change the side length with `--territory-size MM`). When a forward leg
approaches the edge of an unlocked territory, it stops short and chooses a new
direction as though it had reached a mental wall. Smaller territories unlock
sooner and resolve walled-off cells at finer granularity, because each cell is
`territory-size / 4` on a side and wall observations are then less likely to
fall in the same cell Dash drove through.

Mental walls are planning constraints only. They are stored as
`conservative_exploration` territory metadata on each run and are never added
to the map's `walls` observations or rendered as real walls.

Each territory is divided into a 4 x 4 reachability grid. Cells Dash traverses
are visited. Cells containing real wall or obstacle observations are blocked,
unless Dash has also traversed them. A flood fill from visited cells identifies
the remaining reachable frontier and cells cut off behind physical blockers.
The mapper unlocks one adjacent territory after Dash has visited at least 3
cells and no reachable unresolved frontier remains. This lets a physically
small or enclosed area complete even when most of its 16 cells are unreachable,
without treating mental walls as physical evidence.

Heading selection strongly prefers headings whose forward leg actually *enters*
a reachable, unvisited frontier cell, so Dash tests a cell's reachability
instead of merely aiming at frontier it cannot reach. A heading that only
re-traverses already-visited cells is penalized. (Aiming toward a frontier cell
behind a wall is not enough on its own: it draws Dash back to re-drive the
visited cells along that wall, re-detecting known walls instead of making
progress.) It still rejects directions with less than 200 mm of usable forward
clearance and directions leading into cells classified as physically blocked or
unreachable. This prevents repeated turn-only loops near a mental boundary.

Real wall observations within 300 mm of each other are treated as a continuous
wall segment by the core exploration planner, including when conservative
exploration is disabled. These inferred segments prevent repeated sampling
between nearby observations, but remain planning-only evidence and are never
added to the JSON `walls` points. Conservative reachability consumes the same
shared segments when enabled.

The mapper reports major progress to stdout with `[cell complete]`,
`[territory complete]`, and `[adjacent territory unlocked]` messages. Territory
progress persists across `--start-with-map` runs, and expansion favors nearby
uncharted territory so the explored region grows compactly.

This feature is experimental and isolated in
`examples/mapping/conservative_exploration.py`. Disable it without changing
other planning behavior:

```bash
uv run --extra tools examples/mapping/map_room.py \
  --calibration data/calibration.json \
  --no-conservative-exploration
```

The core explorer interacts with the policy only through optional hooks for
heading constraints, forward-distance limits, progress reporting, territory
unlocking, and run metadata.

### Next Challenge: A Better Retry Planner

The current retry loop excludes the exact blocked edge, replans the shortest
proven route, and repeats. In testing this over-blocked parallel corridors,
gave up too early, and produced routes full of stall-prone small turns. A manual
drive home from the same stuck pose succeeded by doing almost the opposite:
backing into open space, committing to one long straight run, and re-referencing
to a wall. The outline below captures that experience as a target algorithm.

1. **Retreat to maneuver room before replanning.** When a leg stops blocked,
   reverse a bounded distance into space just traversed (known clear) before
   planning the next attempt. Manual control needed roughly 300 mm of clearance
   before Dash could turn at all; planning a turn while wedged against the wall
   only stalls.
2. **Soft costs, not hard exclusions.** Do not delete the blocked corridor from
   the graph. Keep edges Dash actually drove during exploration as a trusted
   graph, and raise the cost of the specific blocked approach (a small zone
   around the stop position) so the planner prefers alternatives but can still
   fall back to a proven corridor as a last resort. Never let a blocked segment
   exclude a parallel or endpoint-sharing proven corridor (see the over-blocking
   challenge above).
3. **Prefer few long straight legs over many short ones.** The manual success
   came from one long straight drive (a 600 mm leg that completed in full) down a
   clear line, not from nibbling forward. Favor routes with long, wall-parallel
   straight segments and the fewest turns, since each turn from rest is a failure
   risk.
4. **Make turns reliable.** Small turns (~15-35 degrees) stalled from rest in
   testing, while large turns executed. Snap heading changes to a minimum
   effective turn, or add a brief kick or wiggle to break static friction, and
   verify rotation with the closed-loop turn outcome, compensating for the
   under-rotation that was also observed (a 40-degree command yielding ~11
   degrees).
5. **Re-reference to walls to bound drift.** Pure dead reckoning over ~10 moves
   drifted to a 165-675 mm position uncertainty. Periodically, and at the end,
   re-reference against a known wall (the rear-wall docking move, or a side wall)
   to reset accumulated error instead of trusting odometry.
6. **Generous near-home acceptance, then fine-tune.** Manual driving reached
   about 200 mm from the dock with only a final orientation tweak needed. Once
   within a near-home band, hand off to the existing crawl plus rear-reference
   final approach and a closing orientation correction.
7. **Bound and report.** Cap the retries, report the halt reason and the chosen
   alternative each time, and when only an over-blocked proven corridor remains,
   try it at reduced confidence rather than declaring no route.

Relevant code: `go_home_with_retries`, `go_home`, `plan_home_route`,
`edge_is_blocked`, and the final-approach helpers (`crawl_home`,
`rear_reference`) in `examples/mapping/map_room.py`.

### Detecting Wheel Slip

Wheel odometry measures wheel *rotation*, not physical movement. On a low bump,
threshold, or slick spot, the tires can keep turning while Dash makes little or
no real progress. The encoders advance, so the pose estimate moves even though
the robot did not — and every later pose, map observation, and go-home route is
then built on a wrong position. Dash has no independent position sensor (no
optical flow or external reference), which makes slip genuinely hard to detect.
The available sensors are the two wheel encoders, the gyro (yaw), pitch and roll,
the accelerometer, and the proximity sensors.

There are two distinct cases, with different detectability:

1. **Differential slip (one wheel) — detected.** One tire grips while the other
   slips or spins free. On a straight move this is now caught in
   `validate_odometry`: the left/right encoder difference implies a heading
   change through the track width
   (`(right_delta - left_delta) * mm_per_wheel_tick / TRACK_WIDTH_MM`, with
   `TRACK_WIDTH_MM` ≈ 87 mm backed out from clean in-place turns), and when it
   diverges from the heading the gyro actually measured by more than
   `ODOMETRY_SLIP_HEADING_DEG` (45°) the transition is rejected — stopping the
   run before the bogus averaged distance corrupts the pose. The check is scoped
   to forward/reverse moves: on a turn the gyro is the source of truth for
   heading and translation is ~0, so a wheel over-spin there cannot corrupt the
   pose. Across a captured 156-event session it flagged exactly the one real slip
   with no false positives on the 78 clean translation legs.

   Recorded example that is now rejected (a real forward leg from a captured
   run's `events`): the right wheel turned ~2.2x the left (1366 vs 2954 ticks)
   while the gyro measured only 4.6°. The wheel difference implies ~208° of
   rotation; the gyro saw 5 — the slip. Previously `validate_odometry` accepted
   it (the gyro heading 4.6° < its 45° limit) and averaged the wheels into a
   bogus 429 mm of travel, corrupting every later pose and leaving
   `run_pose_trustworthy` wrongly calling the final pose safe. It is now rejected.

2. **Common-mode slip (both wheels) — still open.** Both tires spin equally while
   Dash sits still or is high-centered on a bump. This is the hard case: equal
   left and right deltas with no rotation look like a perfectly valid straight
   move, so the wheel-vs-gyro check above cannot see it. Catching it needs a
   signal independent of the wheels:
   - **Accelerometer / pitch.** A real forward move produces acceleration
     transients and, climbing a bump, a pitch spike. Wheels advancing with a flat
     accelerometer (no motion) or a pitch event (stuck on a bump) is suspicious.
     Noisy, but Dash exposes `get_acceleration` and `get_pitch`.
   - **Proximity / landmark change.** When moving toward a known wall, the
     expected proximity reading should change as Dash closes in. Wheels advancing
     while a nearby landmark distance does not change suggests no real movement.
     This only works near mapped features (it is a stricter cousin of the
     existing loop-closure revisit correction).

   On suspected common-mode slip the response should mirror the differential
   case: stop the run, mark the segment uncertain (as implausible odometry is
   already isolated), and re-reference against a known wall or require re-docking
   rather than trusting the drifted pose.

Safety and correctness constraints:

- Prefer a false stop over silently trusting a slipped pose; a corrupted pose
  poisons all later mapping and any go-home route.
- Keep the existing odometry validation and loop-closure corrections; slip
  detection augments them rather than replacing them.

Relevant code: `validate_odometry` (and the `TRACK_WIDTH_MM` /
`ODOMETRY_SLIP_HEADING_DEG` constants) and `update_pose` in
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
