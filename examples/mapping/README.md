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

## Configuration

The provided [`config/config.yaml`](config/config.yaml) stores the map path and
reusable mapping settings:

```yaml
map_file: data/room_map.json
calibration: data/calibration/calibration_20260612.json
duration_seconds: 60
exploration_policy: conservative
territory_size_mm: 1000
policy:
  - name: preset
    input_file: data/course.json
go_home_strategy: d-star-lite
```

`exploration_policy` selects the heading policy by name (like
`go_home_strategy`): `conservative` (bounded territories, frontier-alignment
objective; the default), `coverage` (bounded territories, maximize
newly-visited cells per leg — see
[The coverage objective](#step-5-done-the-coverage-objective--coverageexploration)),
or `novelty` (no territory limit, head toward unexplored space). The separate
`policy:` list below is the *command* (preset course) policy and overrides
heading selection while it runs.

Exploration policies are ordered by priority. When `policy` is present, the
first policy drives the robot. The `preset` policy reads a JSON command course
and executes it through the normal mapping odometry and safety checks. Preset
exploration finishes immediately after its final action; `duration_seconds`
does not keep it running. Remove `policy` to use the normal map-guided
exploration strategy.

A preset course like this one runs a cell-conversion check from the dock pose
(currently `(310,310)`):

```text
move 250, turn 90, move 250, turn 90, move 250
```

With ideal motion and the current dock pose, it visits `(0,0)` cells `(1,1)`,
`(2,1)`, `(2,2)`, and ends in `(1,2)`. Course angles are in the map frame
(positive turns head into the room toward +y). Recompute this if `x0`/`y0` in
`dock_to_corner` (`examples/mapping/map_room.py`) or `territory_size_mm`
changes.

The first move sets `stop_at_obstacle: false` because it intentionally departs
parallel to the known dock wall, which can remain visible to a front proximity
sensor and otherwise prevent the move from starting. Use this override only for
bounded moves whose path has been physically verified. Preset moves remain
limited to the sensor-safe `200 mm/s` speed even when obstacle monitoring is
disabled.

Choose one positional run mode:

```bash
uv run --extra tools examples/mapping/map_room.py start
uv run --extra tools examples/mapping/map_room.py resume
uv run --extra tools examples/mapping/map_room.py dock
```

- `start`: dock at the starting corner. If `map_file` exists, reuse and extend
  it; otherwise create a new map at that path.
- `resume`: require `map_file` and continue exploring from its final saved pose.
- `dock`: require `map_file` and return home from its final saved pose.

Use `--config FILE` only to select a different config file.

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
2. Put its left side roughly toward the adjacent wall. Both walls may be out
   of proximity-sensor range at this point — the docking sequence actively
   seeks each one rather than assuming it is already detectable.
3. Point its front diagonally into the open room.
4. Ensure the head tip and proximity sensors are unobstructed.
5. Clear nearby cables, small objects, stairs, and drop-offs.

The mapper uses this fixed docking sequence to establish the starting pose and
orientation:

1. Reverse, seeking the rear wall, up to 500 mm. If the rear proximity sensor
   never crosses its threshold within that distance, the mapper warns (prints
   and speaks "ohno") and continues anyway — check robot placement if this
   happens.
2. Turn 90 degrees counter-clockwise to face the left wall.
3. Drive forward, seeking that wall, up to 500 mm, with the same 500 mm cap
   and warning if it is not found.
4. Turn 90 degrees clockwise — net zero rotation overall, so Dash faces back
   into the room exactly as it started.

Unlike the wall-search distance, the result is *not* a clearance offset from
each wall: Dash ends up snug against both, body against the corner. The
starting pose is fixed at Dash's measured rotation-axis offset from that
corner — `(310, 310)` for the current physical robot (180-190 mm wide; see
`DOCK_CLEARANCE_MM`/`x0`/`y0` in `dock_to_corner`, `examples/mapping/map_room.py`).
Re-measure and update those constants if the robot's body or sensor mounting
changes.

The resulting map coordinate frame uses heading `0°` along `+x` into the room
and places the start in the positive quadrant, so the open room — and the
territories Dash unlocks — grow toward `+x` and `+y`. The two dock walls lie
along the axes through the corner: the side wall at `y=0` (for `x ≥ 0`) and the
rear wall at `x=0` (for `y ≥ 0`); the space behind them (negative `x` or `y`)
is never explorable, and the mapper records those walls so it does not try to
expand past them. Fresh maps therefore start at approximately `(310, 310)` in
territory `(0,0)`. This frame mirrors the gyro's handedness (see `update_pose`
in `examples/mapping/map_room.py`), so the physical motion is unchanged while
map-frame turn angles are negated to physical turn commands. Existing maps
retain their saved start pose and (older) coordinate frame — do not append
new-frame runs to a map created before this change; start a fresh map instead.

It then explores until the requested duration ends:

```bash
uv run --extra tools examples/mapping/map_room.py start
```

By default the mapper explores with the conservative-territory policy (see
[Conservative Exploration](#conservative-exploration)): it confines itself to an
unlocked square territory and repeatedly picks a forward heading that drives
*into* a reachable, still-unvisited cell to test whether it is reachable, rather
than re-driving cells it has already visited. Each leg stops early on a wall, a
tilt, or a territory boundary; odometry is validated after every command and
loop-closure corrections against known landmarks bound the drift. As cells are
resolved the policy unlocks adjacent territory so the explored region grows
compactly. Set `exploration_policy: novelty` for undirected forward legs, or
change `territory_size_mm` to adjust the territory granularity.

`duration_seconds` defaults to 60. `start` creates `map_file` when it does not
exist and appends a new run when it does.

The mapper renders a PNG beside the JSON using the same basename: a cell-grid
overlay showing the conservative-exploration territories with each cell's
visited / blocked / unreachable / frontier state, the robot path, and the wall
observations.

After exploration, do not manually move Dash before running `dock`.

## Dock

After a fresh exploration, do not manually move the robot. Return it to the
map's initial position and orientation with:

```bash
uv run --extra tools examples/mapping/map_room.py dock
```

Go-home plans along previously traversed path segments. It does not invent
shortcuts through unknown space. Movement remains obstacle-aware, each leg is
limited to 1 meter, and the return aborts if a leg stops early or odometry
becomes implausible.

The default planner is **D* Lite**, based on Koenig and Likhachev's
[D* Lite algorithm](https://idm-lab.org/bib/abstracts/papers/aaai02b.pdf) for a
robot moving toward a fixed goal while observed route costs change. When a leg
stops early, the mapper records the failed approach and raises the cost of graph
edges that approach the actual stop position from the same direction. The edge
remains available as a last resort, avoiding false "no route home" failures
caused by deleting broad overlapping corridors.

Set `go_home_strategy` to `hard-blocked-edge` in the config to use the previous
hard-exclusion strategy for A/B testing.

A single `dock` invocation replans automatically: if an attempt aborts on a
blockage while keeping a trustworthy pose, it records the failed approach and
tries a lower-cost proven route from where Dash stopped, up to a few times.
Retries stop early if the pose is no longer trustworthy or the halt was not a
blockage (for example a turn that did not execute).

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

## Start Or Resume

To physically start again from the original corner, creating or extending the
configured map:

```bash
uv run --extra tools examples/mapping/map_room.py start
```

When the configured map exists, the mapper performs the corner-docking routine,
anchors itself to the saved starting pose, and appends a new run. When it does
not exist, the mapper creates a fresh map.

To continue from the robot's final physical pose without docking:

```bash
uv run --extra tools examples/mapping/map_room.py resume
```

## Recommended Explore-And-Return Session

1. Start the WebSocket server.
2. Set `map_file`, `calibration`, and `duration_seconds` in
   `examples/mapping/config/config.yaml`.
3. Place Dash at the starting corner and run:

```bash
uv run --extra tools examples/mapping/map_room.py start
```

4. Confirm Dash has not been manually moved.
5. Return home:

```bash
uv run --extra tools examples/mapping/map_room.py dock
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

## Visualizing A Map

Two tools render a saved map. Both read the JSON the mapper writes and need no
hardware or running server.

[`visualize_cells.py`](visualize_cells.py) renders a static PNG — a cell-grid
overlay with each cell's visited / blocked / unreachable / frontier state, the
robot path, and wall observations:

```bash
uv run --extra tools examples/mapping/visualize_cells.py data/room_map.json
```

It writes `<map>_cells.png` beside the map. Pass `--output FILE` to choose the
path, or `--home-route` to overlay the route the current go-home planner would
follow.

[`animation.py`](animation.py) renders an interactive replay as a single,
self-contained HTML file — all map data and rendering code are embedded, so it
has no external dependencies and can be opened locally or published to the web
as-is:

```bash
uv run --extra tools examples/mapping/animation.py data/room_map.json
```

It writes `<map>_animation.html` beside the map (override with `--output`,
retitle with `--title`, or override the recorded territory size with
`--territory-size`). The animation:

- Replays the robot leg by leg as a top-view Dash avatar (the three-sphere
  body, head dome, and orange eye), with a glowing path trail.
- Draws the conservative-exploration territories and their 4×4 cell grids,
  labeling each cell's coordinate and coloring it by live state as the run
  progresses.
- Reveals wall and obstacle observations only as the robot senses them — each
  flashes a discovery pulse — and resolves cells using just the blockers found
  so far, so the map fills in over time rather than appearing all at once.
- Paces 1× playback to Dash's real motion timing (200 mm/s obstacle-aware
  moves, 85.9 deg/s turns; see `dash/motion.py`), so a long forward leg takes
  proportionally longer than a turn. The header shows the total wall-clock
  motion time, and a speed selector offers 0.5×–4×.
- Supports play/pause, scrubbing, drag-to-pan, scroll-to-zoom, and
  double-click to reset the view.

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
3. D* Lite raises the cost of nearby graph edges traveling in the failed
   direction. Reverse travel away from the obstruction remains cheap.
4. Failed approaches remain available at high cost as a last resort, so broad
   blockage records cannot disconnect an otherwise connected proven graph.

Safety constraints honored by the implementation:

- Never infer an arbitrary shortcut through unknown space.
- Keep obstacle-aware movement enabled.
- Abort on rejected odometry or an early stop.
- Only continue from a partial go-home run when its final pose remains
  trustworthy.
- Avoid repeatedly commanding motion into the same blocked segment.

### Legacy Strategy: Blocked Edges Over-Block Parallel Proven Corridors

The optional `hard-blocked-edge` strategy preserves the original planner for
comparison. It can remove a different, already-driven parallel corridor and
produce a false "no unblocked route home." The default D* Lite strategy fixes
this by applying localized directional costs instead of hard exclusions.

Relevant code: `HardBlockedEdgeStrategy` and `DStarLiteStrategy` in
`examples/mapping/go_home_strategies.py`.

## Conservative Exploration

The mapper limits exploration to an initial square territory (1 x 1 meter by
default; change `territory_size_mm` in the config). When a forward leg
approaches the edge of an unlocked territory, it stops short and chooses a new
direction as though it had reached a mental wall. Smaller territories unlock
sooner and resolve walled-off cells at finer granularity, because each cell is
`territory-size / 4` on a side and wall observations are then less likely to
fall in the same cell Dash drove through; that is why 1 m is the default.

Mental walls are planning constraints only. They are stored as
`conservative_exploration` territory metadata on each run and are never added
to the map's `walls` observations or rendered as real walls.

Each territory is divided into a 4 x 4 reachability grid. Cells Dash traverses
are visited, including cells crossed between saved motion-command endpoints.
Cells containing real wall or obstacle observations are blocked, unless Dash
has also traversed them. A flood fill from visited cells identifies the
remaining reachable frontier and cells cut off behind physical blockers.
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

Real wall observations within one reachability cell of each other (the cell
size, `territory-size / 4` — e.g. 250 mm for the 1 m default, 500 mm for a
2 m territory) are treated as a continuous wall segment by the core exploration
planner, including when conservative exploration is disabled. Tying the link
distance to the cell size keeps wall inference at the same scale as the
reachability grid, so a smaller territory links walls more finely. The links
form a sparse relative-neighborhood graph: nearby observations make continuous
chains, but an observation between two points suppresses the unsupported chord
across that cluster. Inferred segments prevent repeated sampling between nearby
observations, but remain planning-only evidence and are never added to the JSON
`walls` points.
Conservative reachability consumes the same shared segments when enabled, and
marks a cell-center connection blocked when a wall comes within
`CORRIDOR_HALF_CLEARANCE_MM` of it (half of `MIN_CORRIDOR_OPENING_MM`, 250 mm
by default). That 250 mm figure comes from Dash's measured body width
(180-190 mm) plus margin for actuator and sensor slop — the minimum gap
trusted as passable. Motion planning keeps 150 mm away from a wall for live
steering, a separate, smaller margin than the corridor-opening check used for
reachability classification. A wall right next to (parallel to) a connection
but not actually narrowing the gap can still register as blocking under this
distance-only check, since orientation relative to the path is not modeled —
see [Corridor-Clearance Check Does Not Model Wall
Orientation](#corridor-clearance-check-does-not-model-wall-orientation) below.

The mapper reports major progress to stdout with `[cell complete]`,
`[territory complete]`, and `[adjacent territory unlocked]` messages. Territory
progress persists across `start` runs, and expansion favors nearby
uncharted territory so the explored region grows compactly.

This feature is experimental and isolated in
`examples/mapping/conservative_exploration.py`. Set
`exploration_policy: novelty` to disable it.

The core explorer interacts with the policy only through optional hooks for
heading constraints, forward-distance limits, progress reporting, territory
unlocking, and run metadata.

## Exploration Objective And Policy Architecture (work in progress)

This section captures an in-progress redesign so it survives across sessions.

### The objective we are optimizing for

The robot should be incentivized to **maximize the number of cells marked
visited**, subject to the **conservative-exploration constraint**: it may only
enter a new territory after finishing (unlocking from) the current one. That
constraint is enforced as a hard gate (`allows_point` / `forward_distance`);
the objective is the *positive* part of the score.

### Turn decision = `heading_score` (argmax over candidate angles)

`choose_exploration_angle` picks the next turn by maximizing
`heading_score(...)` over a fixed candidate set. We are refactoring so that:

> `heading_score = policy.heading_preference (positive/preference)
>                  − physical_constraints (negative penalties)`

- **Physical constraints stay in `heading_score`** for now: turn cost,
  `point_allowed` (territory wall), known-blocker corridor, inferred-wall
  crossing, and live-proximity penalties. These are all negative.
- **All positive (preference) scoring moves into a policy class.** A policy
  exposes `heading_preference(x, y, heading)` returning the positive signal.
  `heading_score` no longer contains any reward term (the old novelty reward is
  relocated — see below) and no longer takes `path_points`.

### Policy class hierarchy

- `ExplorationPolicy` (abstract base, `exploration_policy.py`): defines the
  interface the explorer/`heading_score` use, with safe "unconstrained"
  defaults — `allows_point→True`, `forward_distance→desired`,
  `unlock_if_complete`/`report_progress`→no-op, `expand_past_boundary→False`,
  `metadata→{}`. The one abstract method is `heading_preference`.
- `NoveltyExplorationPolicy` (the **default** policy, also in
  `exploration_policy.py`): `heading_preference` is the relocated novelty
  reward `Σ min(nearest_path_dist, 800) * 0.35` over the sample distances. Used
  when conservative exploration is disabled. No territory constraint.
- `ConservativeExploration` (`conservative_exploration.py`): now subclasses
  `ExplorationPolicy`. Each concrete policy lives in its own file (1 file, 1
  class), except the ABC and the default `NoveltyExplorationPolicy` which share
  `exploration_policy.py`.

`_ExplorationRun` always holds a policy now (Novelty when not conservative), so
the old `if policy is None` guards are gone.

**Known behavior change from this refactor:** conservative mode no longer adds
the novelty term — its only positive signal is
`ConservativeExploration.heading_preference`. When the focus territory has a
frontier this barely matters (frontier ≫ novelty); when the focus has *no*
frontier the explore pull collapses to bare `clearance`. That no-frontier
regime is owned by the next step.

### Step 5 (done): the coverage objective — `CoverageExploration`

`coverage_exploration.py` adds `CoverageExploration`, a subclass of
`ConservativeExploration` that reflects the redefined objective directly.
Enable it with `exploration_policy: coverage` in the config. It reuses the
parent's territory constraint machinery and changes six things:

- **Objective in `heading_preference`:** rewards the **count of new reachable,
  unvisited cells a leg would enter** (`COVERAGE_CELL_WEIGHT` per cell) instead
  of the parent's alignment-to-nearest-frontier heuristic, with a weak
  alignment fallback when no new cell is reachable this leg.
- **Stateful no-progress signal:** a focus that remains physically unentered
  after `STALL_LEGS` (3) turn decisions is added to `abandoned`, relinquished,
  and reselected. This catches the **unreachable-in-practice** case (visited =
  0, cells stay `frontier` because reachability needs a visited seed) that the
  exhausted-focus rule (`territory_explored`, visited ≥ 1 and frontier = 0)
  cannot. A territory with any visited cell is never abandoned; normal
  reachability owns it. `abandoned` persists in run metadata, and stale
  abandonment is discarded on resume if the territory was later entered.
- **Boundary-aware forward legs:** `forward_distance` stops a leg at the
  boundary of a *completed* territory it would re-enter (one the robot is not
  already standing in), instead of sailing through it to the unlocked-region
  edge. This keeps the robot from drifting back through the finished start
  territory between forays, and — because `heading_preference` reads
  `clearance` — a heading into finished territory then scores as no-progress
  while a heading into still-uncharted (frontier-bearing) territory stays open.
  Transit into uncharted neighbors is unaffected (they are not "completed").
  The selected expansion's source territory is also exempt, so Dash can return
  to that completed territory to reach its committed boundary crossing.
- **Position-driven territory creation.** `_unlock_new_territory` is overridden
  to a no-op, so the policy never unlocks a territory *abstractly*. New
  territories are created only by `expand_past_boundary` — when the robot is
  physically pinned at a boundary heading into un-unlocked space.
- **Committed territory expansion.** After finishing a territory, the policy
  first finishes every currently unlocked territory, then selects one directed
  territory expansion and one plausible crossing area on the shared boundary.
  It keeps pursuing that expansion across turns and resumed runs instead of
  rewarding whichever adjacent territory happens to align with each candidate
  heading. Expansion remains dormant while any unlocked territory still has
  reachable frontier, preventing a future expansion from pulling Dash back out
  of its current coverage work.
- **Physical-blockage learning and completion.** A wall or obstacle encountered
  during expansion removes the attempted crossing area. The directed expansion
  is marked blocked only after no plausible crossing remains, and blocked
  expansions persist across runs. When every unlocked territory is explored
  and no open expansion remains, coverage ends instead of driving until the
  time limit.
- **Territory-transition validation.** A forward leg that reports a wall,
  obstacle, or early stop cannot commit an odometry transition into another
  territory. Its translated pose is rolled back before the blocker is recorded.
  Loop-closure correction is likewise prevented from moving a bounded-policy
  pose across a territory boundary.

Why position-driven: abstract unlocking (the parent's behavior) plus premature
abandonment plus the boundary clamp interact catastrophically — a focus the
robot has not yet reached gets abandoned, the clamp then walls it off, the next
abstract unlock picks another unreachable direction, and the cascade traps the
robot in the start territory while spawning a dozen unreachable territories
(observed: 16 unlocked / 12 abandoned, robot stuck in `(0,-1)`). Creating
territories only where the robot actually reaches a boundary keeps the unlocked
set equal to what has been physically visited or directly attempted.

Relevant code: `heading_score` / `choose_exploration_angle` in
`examples/mapping/map_room.py`; `exploration_policy.py`;
`conservative_exploration.py`; `coverage_exploration.py`.

## Next Challenges

This section is the single list of current mapping and navigation challenges.
Completed behavior and historical design decisions remain documented in their
respective sections above.

### Generalized Corridor Routing: Unify Go-Home And Frontier Navigation

Today, navigating from the dock to a distant frontier (for example, `start`
with an existing map whose `focus_territory` is several territories away)
does **not** use a corridor-aware path plan. `CoverageExploration.heading_preference`
only reasons about frontier cells *inside* `self.focus`; once a forward leg
cannot directly enter one of those cells (because they are out of this leg's
reach), it falls back to scoring headings by straight-line alignment toward
the nearest frontier cell's center (`best_alignment * FRONTIER_HEADING_WEIGHT`,
and `_aim_toward_expansion`'s similar alignment-to-crossing-point scoring).
That alignment score is combined with `heading_score`'s reactive,
sensor-driven obstacle avoidance — there is no notion of a previously-proven
corridor, so Dash re-discovers the route to a distant focus territory leg by
leg, the same way it explores unknown space, even though the map already
records exactly how it got there the first time.

This is a different mechanism from go-home's `DStarLiteStrategy`
(`examples/mapping/go_home_strategies.py`), which builds a graph from path
segments already driven, links nearby/collinear segments, and plans a route
through that graph — explicitly preferring known-clear corridors and
replanning around recorded blockages. Go-home has this; frontier-seeking
during `start`/`resume` does not.

**Proposed generalization:** lift the corridor-graph machinery out of its
current "return-to-dock" framing and expose it as a reusable point-to-point
router: given a start `(territory, cell)` and a target `(territory, cell)`,
return a route along the known-corridor graph, replanning around blocked
edges exactly as go-home does today. Both call sites would use the same
router:

- **Go-home** keeps using it with the goal fixed at the saved start pose
  (today's behavior, just routed through the generalized interface instead of
  a `go_home`-specific code path).
- **Frontier/expansion approach** would call it with the goal set to the
  current focus territory's frontier (or the active territory-expansion's
  crossing point), replacing the straight-line alignment fallback in
  `heading_preference`/`_aim_toward_expansion` with an actual route through
  previously-driven corridors, only falling back to undirected/reactive
  exploration once the route runs out — i.e. once Dash is genuinely off the
  known map and must explore to find the rest of the way.

This is flagged as two pieces of work, not one:

1. **Refactoring.** The route-planning strategy (`GoHomeStrategy` /
   `DStarLiteStrategy` / `HardBlockedEdgeStrategy`) is currently parameterized
   around "drive home" specifics (start pose, `accepted_runs`,
   `run_pose_trustworthy`). Generalizing it means separating "build a corridor
   graph from the map" from "plan a route between two arbitrary points on that
   graph" from "drive the route, handling blockages," so a policy can call the
   planning/driving pieces with any goal, not just home.
2. **Algorithm work.** Today's corridor graph is built at the whole-map
   path-segment scale (`HOME_ROUTE_LINK_RADIUS_MM`, `HOME_ROUTE_COLLINEAR_DEG`),
   while frontier targets are expressed at the conservative-exploration cell
   grid scale. Deciding how a route should terminate at (or hand off to) a
   specific cell inside a specific territory — and how blocked-edge learning,
   the obstacle-relaxation heuristics used while retracing, and the safety
   constraints already established for go-home (never invent a shortcut
   through unknown space, abort on rejected odometry, etc.) carry over to a
   goal that is not "home" — needs design before implementation.

Relevant code: `DStarLiteStrategy`, `HardBlockedEdgeStrategy`, `plan_route` in
`examples/mapping/go_home_strategies.py`; `go_home`, `go_home_with_retries` in
`examples/mapping/map_room.py`; `CoverageExploration.heading_preference`,
`_aim_toward_expansion`, `_select_territory_expansion` in
`examples/mapping/coverage_exploration.py`.

### Local Detour Exploration

The planner reroutes only through space it has already proven. When every known
route is blocked it stops. A future implementation could perform bounded local
exploration to discover a new connection around an obstruction, while always
preserving a proven retreat route back to known-safe space.

### Path-Aware Obstacle Handling On The Way Home

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

### Better Retry Planning

The D* Lite strategy now applies localized soft costs and keeps blocked proven
corridors available as a last resort. The remaining retry problem is physical:
replanning while Dash is pressed against a wall can still select a route that it
cannot begin. A manual drive home from the same stuck pose succeeded by backing
into open space, committing to one long straight run, and re-referencing to a
wall.

1. **Retreat to maneuver room before replanning.** When a leg stops blocked,
   reverse a bounded distance into space just traversed (known clear) before
   planning the next attempt. Manual control needed roughly 300 mm of clearance
   before Dash could turn at all; planning a turn while wedged against the wall
   only stalls.
2. **Prefer few long straight legs over many short ones.** The manual success
   came from one long straight drive (a 600 mm leg that completed in full) down a
   clear line, not from nibbling forward. Favor routes with long, wall-parallel
   straight segments and the fewest turns, since each turn from rest is a failure
   risk.
3. **Make turns reliable.** Small turns (~15-35 degrees) stalled from rest in
   testing, while large turns executed. Snap heading changes to a minimum
   effective turn, or add a brief kick or wiggle to break static friction, and
   verify rotation with the closed-loop turn outcome, compensating for the
   under-rotation that was also observed (a 40-degree command yielding ~11
   degrees).
4. **Re-reference to walls to bound drift.** Pure dead reckoning over ~10 moves
   drifted to a 165-675 mm position uncertainty. Periodically, and at the end,
   re-reference against a known wall (the rear-wall docking move, or a side wall)
   to reset accumulated error instead of trusting odometry.
5. **Generous near-home acceptance, then fine-tune.** Manual driving reached
   about 200 mm from the dock with only a final orientation tweak needed. Once
   within a near-home band, hand off to the existing crawl plus rear-reference
   final approach and a closing orientation correction.
6. **Bound and report.** Continue reporting each halt and chosen alternative,
   while keeping the retry count bounded.

Relevant code: `go_home_with_retries`, `go_home`, `plan_home_route`,
`DStarLiteStrategy`, and the final-approach helpers (`crawl_home`,
`rear_reference`).

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

2. **Common-mode slip (both wheels) — partially detected.** Both tires spin
   equally while Dash sits still or is high-centered on a bump. This is the hard
   case: equal left and right deltas with no rotation look like a perfectly
   valid straight move, so the wheel-vs-gyro check above cannot see it. The
   mapper now rejects one important wall-contact case: if an obstacle-aware
   forward move reports a front obstacle stop, but wheel odometry still claims a
   near-full requested leg (or a before-start obstacle still logs substantial
   travel), the just-committed translation is rolled back, the event is marked
   `common_mode_slip_rejected`, and the run stops with `tracking_lost` rather
   than continuing from a drifted pose. Call paths that do not provide a move
   outcome get a narrow fallback for short, blocked, near-full-distance probes.
   Catching the broader cases still needs another signal independent of the
   wheels:
   - **Accelerometer / pitch.** A real forward move produces acceleration
     transients and, climbing a bump, a pitch spike. Wheels advancing with a flat
     accelerometer (no motion) or a pitch event (stuck on a bump) is suspicious.
     Noisy, but Dash exposes `get_acceleration` and `get_pitch`.
   - **Proximity / landmark change.** When moving toward a known wall, the
     expected proximity reading should change as Dash closes in. Wheels advancing
     while a nearby landmark distance does not change suggests no real movement.
     This only works near mapped features (it is a stricter cousin of the
     existing loop-closure revisit correction).

   On suspected common-mode slip the response mirrors the differential case:
   stop the run, mark the segment uncertain (as implausible odometry is already
   isolated), and re-reference against a known wall or require re-docking rather
   than trusting the drifted pose.

Safety and correctness constraints:

- Prefer a false stop over silently trusting a slipped pose; a corrupted pose
  poisons all later mapping and any go-home route.
- Keep the existing odometry validation and loop-closure corrections; slip
  detection augments them rather than replacing them.

Relevant code: `validate_odometry` (and the `TRACK_WIDTH_MM` /
`ODOMETRY_SLIP_HEADING_DEG` constants) and `update_pose` in
`examples/mapping/map_room.py`; the `get_left_wheel`, `get_right_wheel`,
`get_yaw`, `get_pitch`, and `get_acceleration` sensors in `dash/sensors.py`.

### Corridor-Clearance Check Does Not Model Wall Orientation

`territory_resolution`'s reachability BFS (see [Conservative
Exploration](#conservative-exploration)) blocks a cell-center connection when
any point on it comes within `CORRIDOR_HALF_CLEARANCE_MM` (125 mm) of an
inferred wall segment, regardless of that wall's orientation relative to the
connection. A wall running *parallel* to a connection, off to the side, can
register as "too close" even though it does not actually narrow the path
along the direction of travel — only a wall that crosses *transversely* near
the connection should block it. The 125 mm threshold (half of
`MIN_CORRIDOR_OPENING_MM`, derived from Dash's measured 180-190 mm body width)
was chosen and validated against exactly one real map
(`data/room_map.json`) plus two prior regression scenarios; it has not yet
been exercised against a wider variety of corridor geometries (sharp corners,
short furniture legs at odd angles, etc.), so both false-blocks and
false-opens beyond those three cases are unverified.

A more accurate model would test whether the wall, projected onto the
connection's direction of travel, actually overlaps the connection's extent
within the clearance band — effectively treating the connection as a
rectangle (`MIN_CORRIDOR_OPENING_MM` wide, leg-length long) and checking for
intersection, rather than measuring undirected point-to-segment distance.

Relevant code: `territory_resolution` and `MIN_CORRIDOR_OPENING_MM` /
`CORRIDOR_HALF_CLEARANCE_MM` in `examples/mapping/conservative_exploration.py`;
`segment_crosses_wall` in `examples/mapping/exploration_walls.py`; regression
tests in `tests/test_conservative_exploration.py`.

### Verifying Wall Detections Against Transient Tilt

When a forward leg stops early with elevated proximity readings, the mapper
records it as a wall (`mark_ahead`/`handle_leg_end` in
`examples/mapping/map_room.py`). Investigating a run where several such wall
points seemed implausible (`data/room_map.json`, no physical wall at the
recorded location) showed:

- The stop was **not** an artifact of the conservative-territory boundary
  clamp: in every case the robot's *actual* traveled distance (from wheel
  odometry) fell well short of even the policy-clamped *requested* distance
  (ratios of roughly 0.3-0.8), meaning live proximity sensing — not the
  synthetic territory edge — ended the leg early.
- Whether a brief, sub-threshold tilt (a bump too small to cross
  `PITCH_TILT_THRESHOLD`) caused a spurious proximity spike could not be ruled
  in or out: pitch is sampled once, *after* each leg ends
  (`_ExplorationRun.handle_leg_end`), not continuously during it. A momentary
  bump that settles back to flat before the leg finishes leaves no trace in
  the saved data.

Adding per-leg min/max pitch (and ideally min/max proximity) sampled
throughout the move, not just at the end, would make this verifiable from the
saved JSON instead of inferred after the fact, and could let the mapper
distinguish a genuine wall from a bump-induced false reading automatically
(e.g. discount a wall recorded during a leg whose pitch briefly excursed past
some lower bump-detection threshold even if it never crossed
`PITCH_TILT_THRESHOLD`).

Relevant code: `handle_leg_end`, `mark_ahead`, `report_leg`, and
`PITCH_TILT_THRESHOLD` in `examples/mapping/map_room.py`; `get_pitch` in
`dash/sensors.py`.

## Command Reference

Run from the mapping config:

```bash
uv run --extra tools examples/mapping/map_room.py start
uv run --extra tools examples/mapping/map_room.py resume
uv run --extra tools examples/mapping/map_room.py dock
```

Use a different config:

```bash
uv run --extra tools examples/mapping/map_room.py start --config path/to/config.yaml
```

Show mapper usage:

```bash
uv run --extra tools examples/mapping/map_room.py --help
```

Show all calibration options:

```bash
uv run examples/mapping/calibrate.py --help
```

Render a saved map (see [Visualizing A Map](#visualizing-a-map)):

```bash
uv run --extra tools examples/mapping/visualize_cells.py data/room_map.json
uv run --extra tools examples/mapping/animation.py data/room_map.json
```
