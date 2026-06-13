"""Background server that keeps a robot connected and accepts WebSocket commands."""

import argparse
import asyncio
import json
import os
import signal
import sys
import threading

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosedError, ConnectionClosedOK

from dash.command_protocol import execute_json
from dash.interactive import discover_and_connect_sync
from dash.robot import DEFAULT_ROBOT_NAME

HOST = "127.0.0.1"
PORT = 8765
PID_PATH = "/tmp/dash_robot_ws.pid"


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description="Keep a robot connected and accept WebSocket commands."
    )
    parser.add_argument("--host", default=HOST, help=f"Host IP to bind (default: {HOST})")
    parser.add_argument(
        "--port", type=int, default=PORT, help=f"Port to bind (default: {PORT})"
    )
    parser.add_argument(
        "--silent",
        action="store_true",
        help="Suppress all robot sounds for the session, including ones requested "
        "by clients and internal safety sounds",
    )
    robot = parser.add_mutually_exclusive_group()
    robot.add_argument(
        "--name",
        default=DEFAULT_ROBOT_NAME,
        help=f"Bluetooth name to discover (default: {DEFAULT_ROBOT_NAME})",
    )
    robot.add_argument(
        "--address",
        help="Bluetooth address to connect to directly without discovery",
    )
    return parser.parse_args(args)


def silence_robot(robot):
    """Replace say() with a no-op so no sound plays for the rest of the session.

    The replacement lives on the underlying async robot, so it covers client
    "say" commands, the server's own greetings, and the motion layer's internal
    safety sounds, which all route through the same method.
    """
    async def _silent_say(*args, **kwargs):
        return None

    robot.async_robot.say = _silent_say


def greet_robot(robot):
    robot.say("hi")


def say_goodbye_and_disconnect(robot):
    try:
        robot.stop()
    except Exception as error:
        print(f"Failed to stop robot during shutdown: {error}", flush=True)
    try:
        robot.say("bye")
    finally:
        robot.disconnect()


def execute_request(message, robot, command_lock):
    """Execute one JSON command and return a JSON-compatible response."""
    with command_lock:
        return execute_json(message, robot)


def stop_robot(robot, command_lock):
    """Best-effort halt of the robot, holding the command lock.

    A client opens a fresh connection per command, so a normal exchange closes
    cleanly and never reaches here. This runs only when a client disconnects
    abruptly (for example killed mid-command), so a move that was in flight
    cannot leave the robot driving with no client able to stop it.
    """
    with command_lock:
        try:
            robot.stop()
        except Exception as error:
            print(f"Failed to stop robot after disconnect: {error}", flush=True)


async def handle_client(websocket, robot, command_lock):
    try:
        async for message in websocket:
            response = await asyncio.to_thread(
                execute_request, message, robot, command_lock
            )
            await websocket.send(json.dumps(response))
    except ConnectionClosedOK:
        # Client closed cleanly mid-exchange; nothing left to do.
        pass
    except ConnectionClosedError:
        # Abrupt disconnect (no close frame): halt the robot as a safety net.
        await asyncio.to_thread(stop_robot, robot, command_lock)


async def run_server(robot, host=HOST, port=PORT):
    stop = asyncio.Event()
    command_lock = threading.Lock()
    loop = asyncio.get_running_loop()

    for signum in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(signum, stop.set)

    async with serve(
        lambda websocket: handle_client(websocket, robot, command_lock),
        host,
        port,
    ):
        print(f"Robot WebSocket server ready on ws://{host}:{port}", flush=True)
        await stop.wait()


def main():
    args = parse_args()
    print("Connecting to robot...", flush=True)
    robot = discover_and_connect_sync(
        retry_attempts=3,
        retry_delay=5,
        name=args.name,
        address=args.address,
    )
    if robot is None:
        print("No robot found.", flush=True)
        sys.exit(1)
    print(f"Connected: {robot._robot.address} ({type(robot._robot).__name__})", flush=True)

    if args.silent:
        silence_robot(robot)
        print("Silent mode: robot sounds are suppressed.", flush=True)

    try:
        greet_robot(robot)
        with open(PID_PATH, "w") as pid_file:
            pid_file.write(str(os.getpid()))
        asyncio.run(run_server(robot, host=args.host, port=args.port))
    finally:
        print("Shutting down...", flush=True)
        try:
            say_goodbye_and_disconnect(robot)
        finally:
            if os.path.exists(PID_PATH):
                os.remove(PID_PATH)


if __name__ == "__main__":
    main()
