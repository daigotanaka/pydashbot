# pydashbot

## What is it?

`pydashbot` is Python control software for the Wonder Workshop Dash robot via BLE (Bluetooth Low Energy)

It provides:

- A persistent WebSocket server that keeps one Bluetooth connection open
- A command-line client for sending robot commands
- A live room-mapping dashboard
- Direct synchronous and asynchronous Python APIs
- Obstacle-aware, bounded movement
- Access to Dash's sensors, lights, head, wheels, and sounds
- Hardware examples, calibration tools, and automated tests

The project focuses exclusively on **Dash**. It does not aim to provide a
general API for Dot, Cue, or other Wonder Workshop robots.

## Demo Application (map + dashboard)

![Demo](images/pydashbot.gif)

The demo application explores a room with Dash, builds a 2D map, and streams
the robot's pose to a browser dashboard. The dashboard can also export the map
JSON and a standalone HTML animation replay.

At a high level:

1. The WebSocket server connects to Dash over Bluetooth.
2. The map app sends movement and sensor commands through that server.
3. The dashboard runs in a browser and receives live map updates over HTTP.

## Requirements

- Python 3.11
- uv ([How to install uv](https://docs.astral.sh/uv/getting-started/installation/))
- A Bluetooth Low Energy adapter (Any modern laptop should have it already)
- A [Wonder Workshop Dash robot](https://www.makewonder.com/dash/)

`pydashbot` is tested on macOS and Ubuntu.

On macOS, allow your terminal application to use Bluetooth in **System
Settings > Privacy & Security > Bluetooth**.

## Quick Start

Install the project:

```bash
git clone https://github.com/daigotanaka/pydashbot.git
cd pydashbot
uv sync
```

Turn on Dash before connecting. Only one process can maintain the Bluetooth
connection at a time. Make sure Dash is within Bluetooth range of your computer.

Start the WebSocket server in one terminal:

```bash
uv run dash.remote.server --name <your Dash robot's name>
```

If your dash name has not changed from default names (Dash, Dashet), you can
ommit `--name` argument. When connected, Dash will stop spontaneous movements,
and the console will display a message like:

```
Connected to XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXX
```

Place Dash near a room corner with its back roughly facing one wall (20 to 50cm away)
and its left side roughly facing the adjacent wall (20 to 50cm away):

![Docking position](images/docking-position.jpg)

Start the dashboard and map app:

```bash
uv run apps.dashboard --config data/sample_config.yaml
```

Open [http://127.0.0.1:8000](http://127.0.0.1:8000) to watch the live map.

For more details, read:
- [apps/dashboard/README.md](apps/dashboard/README.md)
- [apps/map/README.md](apps/maps/README.md)

## Safety

Dash is a physical robot. Always test movement on the floor in a clear area,
keep it away from stairs, and be ready to stop it.

Obstacle and tilt detection are best-effort safeguards. Proximity sensors
cannot reliably detect every material, shape, or approach angle.

The WebSocket server has no authentication. Binding it to `0.0.0.0` or a LAN
address allows other devices on that network to control the robot. Only expose
it on a trusted network.

## Programming your Dash with Python

Want to develop your application with Python? See [dash/README.md](dash/README.md).

## Letting an AI agent control Dash

With pydashbot, AI agents like OpenClaw, Hermes, Claude Code, and Codex can control Dash.
The easiest way to get started is to let them read this doc.
Just note that your agent might not have a direct access to Bluetooth adapter.
You may need to start the WebSocket server from a command prompt.
AI agents should be able to start dashboard and map apps, or they can write scripts to control Dash.

## Credits

Early protocol knowledge used by this project came from:

- [Bleak](https://github.com/hbldh/bleak) for BLE control
- [chubbykat's bleak-dash](https://github.com/mewmix/bleak-dash)
- [Ilya Sukhanov's morseapi](https://github.com/IlyaSukhanov/morseapi)
- [Russ Buchanan's python-dash-robot](https://github.com/havnfun/python-dash-robot)

Those projects made independent control of Wonder Workshop robots possible and
deserve credit for documenting much of the low-level behavior.

`pydashbot` has since become an independently structured and reimplemented
project. It is centered on Dash, uses Bleak for Bluetooth Low Energy, and adds
layered motion safety, persistent communication servers, WebSocket control,
tests, and hardware tools.

## Disclaimer

`pydashbot` is an independent fan project and is not affiliated with, endorsed
by, sponsored by, or otherwise associated with Wonder Workshop, Inc.

Use of this software is at your own risk. You are solely responsible for how
you install, configure, operate, or modify the software and for any resulting
effects on your robot, devices, property, network, or surroundings. To the
maximum extent permitted by applicable law, the project authors and contributors
disclaim liability for any damages, losses, injuries, or other consequences
arising from use of this software.

## License

[Apache 2.0 License](LICENSE)
