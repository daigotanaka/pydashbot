# Dash Python API

This package contains the Python control layers for Wonder Workshop Dash:

- `dash.core`: asynchronous robot connection, actuator commands, sensor
  decoding, motion control, constants, and command dispatch
- `dash.control`: synchronous helpers for scripts and interactive sessions
- `dash.remote`: WebSocket server and client used by the command line, map app,
  and dashboard

## Quick Start

The synchronous API is the simplest way to control Dash from a script or
interactive Python session:

```python
from dash.control.interactive import discover_and_connect_sync

robot = discover_and_connect_sync()  # Discovers the default name "Doodle"
if robot is None:
    raise RuntimeError("Dash was not found")

robot.say("hi")
robot.neck_color("blue")
robot.move(200)
robot.turn(90)

robot.stop()
robot.disconnect()
```

For an asynchronous application:

```python
import asyncio

from dash.core.robot import discover_and_connect


async def main():
    robot = await discover_and_connect(name="Doodle")
    if robot is None:
        raise RuntimeError("Dash was not found")
    try:
        await robot.say("hi")
        await robot.move(200)
        await robot.turn(90)
    finally:
        await robot.stop()
        await robot.disconnect()


asyncio.run(main())
```

The default discovery name is `Doodle`. You can select another Bluetooth name
or skip discovery and connect directly using a Bluetooth address:

```python
robot = discover_and_connect_sync(name="Workshop Dash")
robot = discover_and_connect_sync(address="AA:BB:CC:DD:EE:FF")
```

Direct addresses are platform-specific. Linux and Windows commonly use a MAC
address, while macOS may expose a Bluetooth UUID instead.

## Motion

### Bounded motion

`move()` travels a requested distance in millimeters and waits until the motion
finishes:

```python
robot.move(200)          # forward 200 mm
robot.move(-150)         # backward 150 mm
robot.move(300, 150)     # 300 mm at 150 mm/s
```

`turn()` rotates a requested number of degrees in place:

```python
robot.turn(90)           # counterclockwise
robot.turn(-90)          # clockwise
robot.turn(360)
```

### Obstacle-aware movement

Obstacle and tilt stopping are enabled by default for `move()`.

While moving forward, Dash monitors its left and right proximity sensors.
While reversing, it monitors the rear proximity sensor. A sustained obstacle
reading or excessive tilt stops both wheels before the requested distance is
complete. Dash says `confused8` when a safety stop occurs.

```python
robot.move(500)

# Move without the safety-stop sound.
robot.move(500, wall_stop_sound=None)

# Disable obstacle and tilt stopping.
robot.move(500, stop_at_obstacle=False)
```

When obstacle detection is enabled, speed is capped at a level that gives the
sensor loop time to react: 200 mm/s forward and 100 mm/s in reverse.

### Continuous motion

`drive()` and `spin()` continue until `stop()` is called:

```python
robot.drive(100)         # drive forward continuously
robot.spin(100)          # spin counterclockwise continuously
robot.stop()
```

Use `move()` and `turn()` when you want bounded motion.

### Arc motion

`arc()` drives a smooth circular arc using Dash's native pose command. The first
argument is the arc radius in millimeters and the second is the heading sweep in
degrees. Positive angles curve left; negative angles curve right.

```python
robot.arc(120, 60)       # 120 mm radius, 60 degrees left
robot.arc(120, -60)      # 120 mm radius, 60 degrees right
robot.arc(180, 180)      # split into smaller arc segments automatically
```

By default, arcs run at 150 mm/s, ease in and out, split large sweeps into
90-degree segments, and monitor proximity sensors like `move()`. The return
value is a dict with fields such as `halt`, `requested_angle_deg`,
`completed_angle_deg`, `completed_fraction`, `segments`, `rel_pose_mm`, and
`yaw_delta`.

Useful options:

```python
robot.arc(120, 90, speed_mmps=100)
robot.arc(120, 90, stop_at_obstacle=False)
robot.arc(120, 90, max_segment_deg=45)
robot.arc(120, 90, wall_stop_sound="confused8")
```

For backward arcs, pass `direction=PoseDirection.BACKWARD` from
`dash.core.actuators`.

## Actuators

The public actuator API is available on both the asynchronous robot returned by
`dash.core.robot.discover_and_connect()` and the synchronous facade returned by
`dash.control.interactive.discover_and_connect_sync()`.

```python
robot.say("hi")
robot.beep()

robot.eye(0x0FFF)
robot.eye_brightness(255)
robot.neck_color("purple")
robot.left_ear_color("red")
robot.right_ear_color("blue")
robot.head_color("white")
robot.tail_brightness(255)

robot.head_yaw(30)
robot.head_pitch(-10)
robot.pose(x=10, y=0, theta=0, time=1)

robot.drive(100)
robot.spin(-100)
robot.stop()
robot.reset()
```

For `pose()`, `x` and `y` are centimeters, `theta` is radians, and `time` is
seconds. The optional pose controls are `mode`, `direction`, `wrap_theta`, and
`ease`; import `PoseMode` and `PoseDirection` from `dash.core.actuators` when
you need them.

`eye()` takes a 12-bit mask selecting Dash's eye LEDs. Neck and ear colors may
be color names understood by the `colour` package or `colour.Color` objects.
Head yaw is clamped to `-53..53` degrees, head pitch to `-5..10` degrees, tail
brightness to `0..255`, and continuous drive/spin speeds are clamped to the
firmware ranges.

## Sounds

Play a built-in sound by name:

```python
robot.say("hi")
robot.say("confused8")
```

To list all known sound names:

```python
from dash.core.constants import NOISES

print(*NOISES, sep="\n")
```

Dash also exposes the pre-existing custom sound slots `my1` through `my10`.
This project does not upload new audio to those slots.

## Sensors

The getter API returns the latest decoded BLE sensor values. Getters return
`None` until the corresponding sensor stream has produced data. Most motion,
pose, wheel, and proximity values are raw firmware units; calibrate them for
physical measurements when precision matters.

Shared Dot/Dash stream getters:

```python
robot.get_time()
robot.get_index()
robot.get_pitch()
robot.get_roll()
robot.get_acceleration()

robot.is_button_white_pressed()
robot.is_button_1_pressed()
robot.is_button_2_pressed()
robot.is_button_3_pressed()

robot.is_moving()
robot.is_picked_up()
robot.is_hit()
robot.is_on_side()
robot.is_nominal()

robot.has_heard_clap()
robot.get_mic_level()
robot.get_sound_direction()

robot.is_dot_left_of_dash()
robot.is_dot_right_of_dash()
robot.get_robot()
```

Dash-specific stream getters:

```python
robot.get_dash_time()
robot.get_dash_index()

robot.get_pitch_delta()
robot.get_roll_delta()
robot.get_yaw()
robot.get_yaw_delta()

robot.get_prox_left()
robot.get_prox_right()
robot.get_prox_rear()

robot.get_left_wheel()
robot.get_right_wheel()
robot.get_wheel_distance()
robot.get_head_pitch()
robot.get_head_yaw()
```

`get_time()` and `get_dash_time()` are host timestamps for the latest sensor
packet, useful for detecting whether fresh readings have arrived. `get_robot()`
reports the connected robot type once startup has identified the sensor
streams.

## WebSocket Server

For repeated commands, the WebSocket server keeps one process connected to Dash
and accepts commands from other terminals or computers.

Start it with the default address, `127.0.0.1:8765`:

```bash
uv run dash.remote.server
```

Choose another host or port:

```bash
uv run dash.remote.server --host 192.168.1.10 --port 9000
```

Discover a robot with another Bluetooth name:

```bash
uv run dash.remote.server --name "Workshop Dash"
```

Connect directly without scanning:

```bash
uv run dash.remote.server --address AA:BB:CC:DD:EE:FF
```

`--name` and `--address` are mutually exclusive. The default name is `Doodle`.

Suppress all robot sounds for the whole session, including ones requested by
clients and internal safety sounds:

```bash
uv run dash.remote.server --silent
```

Dash says `hi` after the server connects to it and `bye` when the server shuts
down and disconnects, unless `--silent` is used.

Use the WebSocket client:

```bash
uv run pydashbot say hi
uv run pydashbot move 200
uv run pydashbot turn 360
uv run pydashbot get_prox_rear
uv run pydashbot stop
```

Specify a server address:

```bash
uv run pydashbot --host 192.168.1.10 --port 9000 move 200
```

Suppress the default safety-stop sound for a move:

```bash
uv run pydashbot move 200 --no-wall-sound
```

The server has no authentication. Binding it to `0.0.0.0` or a LAN address
allows other devices on that network to control the robot. Only expose it on a
trusted network.

### Command protocol

The WebSocket client sends a method name followed by positional arguments. The
server returns JSON:

```json
{"ok": true, "result": null}
```

Errors use:

```json
{"ok": false, "error": "error description"}
```

## Architecture

The codebase separates hardware access, motion behavior, synchronous helpers,
and remote control:

```text
dash/
  core/
    actuators.py         Low-level actuator command encoding
    command_protocol.py  Command parsing and dispatch
    constants.py         Bluetooth UUIDs, sounds, and protocol constants
    motion.py            Bounded motion and obstacle/tilt safety
    robot.py             BLE connection and hardware composition
    sensors.py           Low-level sensor decoding

  control/
    interactive.py       Synchronous facade over the async API

  remote/
    client.py            WebSocket client and pydashbot CLI entry point
    server.py            Persistent robot WebSocket server
```

This layering keeps transport details out of motion control and keeps safety
logic out of low-level actuator encoding.

## Examples and Tools

Hardware examples:

```bash
uv run examples/simple_demo/hardware_demo.py
uv run examples/simple_demo/sensor_demo.py
uv run examples/simple_demo/light_demo.py
uv run examples/simple_demo/sound_follow.py
```

Each hardware example accepts `--name` or `--address`:

```bash
uv run examples/simple_demo/hardware_demo.py --name "Workshop Dash"
uv run examples/simple_demo/sensor_demo.py --address AA:BB:CC:DD:EE:FF
```

The [room-mapping app](../apps/map/README.md) covers calibration, autonomous
exploration, map-guided exploration, and returning to the starting pose.

## Tests

The automated test suite does not require a connected robot:

```bash
uv run python -m unittest discover -s tests -p "test*.py"
```

The suite covers actuator encoding, motion safety, command dispatch,
architecture boundaries, sensor decoding, dashboard behavior, mapping policies,
navigation, and WebSocket behavior.

## Troubleshooting

### Dash is not discovered

- Confirm Dash is powered on and nearby.
- Disconnect the official Wonder Workshop app and any other controller.
- Confirm your terminal has Bluetooth permission.
- Turn Dash off and on, then retry.

### Connection fails or commands stop responding

Only one process can connect to Dash at a time. Stop other Python clients and
servers before reconnecting.

### A server is still running

Press `Ctrl+C` in its terminal. During shutdown, the server sends a stop command
before disconnecting from Dash.

### Motion stops before reaching the requested distance

This usually means the obstacle or tilt safeguard triggered. Check the floor,
clear nearby objects, and keep Dash upright. Sensor thresholds can be adjusted
through the Python `move()` API when necessary.
