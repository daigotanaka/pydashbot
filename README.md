# pydashbot

Python control software for the Wonder Workshop Dash robot, built on
[Bleak](https://github.com/hbldh/bleak).

This project provides:

- Direct asynchronous and synchronous Python APIs
- Obstacle-aware, bounded movement
- Access to Dash's sensors, lights, head, wheels, and sounds
- Persistent WebSocket servers
- Command-line clients for controlling a connected robot
- Hardware examples, calibration tools, and automated tests

The project focuses exclusively on **Dash**. It does not aim to provide a
general API for Dot, Cue, or other Wonder Workshop robots.

## Safety

Dash is a physical robot. Always test movement on the floor in a clear area,
keep it away from stairs, and be ready to stop it.

Obstacle and tilt detection are best-effort safeguards. Proximity sensors
cannot reliably detect every material, shape, or approach angle.

## Credits and Project History

Early protocol knowledge used by this project came from:

- [chubbykat's bleak-dash](https://github.com/mewmix/bleak-dash)
- [Ilya Sukhanov's morseapi](https://github.com/IlyaSukhanov/morseapi)
- [Russ Buchanan's python-dash-robot](https://github.com/havnfun/python-dash-robot)

Those projects made independent control of Wonder Workshop robots possible and
deserve credit for documenting much of the low-level behavior.

`pydashbot` has since become an independently structured and reimplemented
project. It is centered on Dash, uses Bleak for Bluetooth Low Energy, and adds
layered motion safety, persistent communication servers, WebSocket control,
tests, and hardware tools.

## Requirements

- Python 3.11
- [uv](https://docs.astral.sh/uv/)
- A Bluetooth Low Energy adapter
- A Wonder Workshop Dash robot

On macOS, allow your terminal application to use Bluetooth in **System
Settings > Privacy & Security > Bluetooth**.

## Installation

```bash
git clone https://github.com/daigotanaka/pydashbot.git
cd pydashbot

uv sync
```

`uv sync` creates `.venv`, installs the project, and uses the checked-in
`uv.lock` file for reproducible dependencies.

Turn on Dash before connecting. Only one process can maintain the Bluetooth
connection at a time.

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

## Actuators

Common actuator methods include:

```python
robot.say("hi")
robot.eye(0x0FFF)
robot.eye_brightness(255)
robot.neck_color("purple")
robot.left_ear_color("red")
robot.right_ear_color("blue")
robot.tail_brightness(255)

robot.head_yaw(30)
robot.head_pitch(-10)
robot.pose(x=10, y=0, theta=0, time=1)

robot.stop()
robot.reset()
```

For `pose()`, `x` and `y` are centimeters, `theta` is radians, and `time` is
seconds.

`eye()` takes a 12-bit mask selecting Dash's eye LEDs. Neck and ear colors may
be color names understood by the `colour` package or `colour.Color` objects.

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

Examples:

```python
robot.get_pitch()
robot.get_roll()
robot.get_yaw()
robot.get_acceleration()

robot.get_prox_left()
robot.get_prox_right()
robot.get_prox_rear()

robot.get_left_wheel()
robot.get_right_wheel()
robot.get_wheel_distance()

robot.get_head_pitch()
robot.get_head_yaw()
robot.get_sound_direction()

robot.is_moving()
robot.is_picked_up()
robot.is_hit()
```

Button, microphone, and clap-related getters are also available. See
[`dash/sensors.py`](dash/sensors.py) for the complete sensor API.

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

The codebase separates hardware access, motion behavior, and communication:

```text
dash/
  sensors.py           Low-level sensor decoding
  actuators.py         Low-level actuator command encoding
  robot.py             BLE connection and hardware composition

  motion.py            Bounded motion and obstacle/tilt safety

  command_protocol.py  Command parsing and dispatch
  server.py         WebSocket server
  client.py         WebSocket client

  interactive.py       Synchronous facade over the async API
  constants.py         Bluetooth UUIDs, sounds, and protocol constants
```

This layering keeps transport details out of motion control and keeps safety
logic out of low-level actuator encoding.

## Examples and Tools

Hardware examples:

```bash
uv run examples/hardware_demo.py
uv run examples/sensor_monitor.py
uv run examples/lightshow.py
```

Each hardware example accepts `--name` or `--address`:

```bash
uv run examples/hardware_demo.py --name "Workshop Dash"
uv run examples/sensor_monitor.py --address AA:BB:CC:DD:EE:FF
```

The [room-mapping examples](apps/map/README.md) cover calibration,
autonomous exploration, map-guided exploration, and returning to the starting
pose.

## Tests

The automated test suite does not require a connected robot:

```bash
uv run python -m unittest discover -s tests -p "test*.py"
```

The suite covers actuator encoding, motion safety, command dispatch,
architecture boundaries, sensor decoding, and WebSocket behavior.

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

## License

[Apache 2.0 License](LICENSE)
