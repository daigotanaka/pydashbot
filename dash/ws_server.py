"""Background server that keeps a robot connected and accepts WebSocket commands."""

import argparse
import asyncio
import json
import os
import signal
import sys
import threading
import time

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosedError, ConnectionClosedOK

from dash.command_protocol import execute_json
from dash.interactive import discover_and_connect_sync
from dash.robot import DEFAULT_ROBOT_NAME

HOST = "127.0.0.1"
PORT = 8765
PID_PATH = "/tmp/dash_robot_ws.pid"

# Liveness check: the sensor stream stamps `dash_time` with the wall-clock time
# of each packet, so seconds-since-`dash_time` is how long the robot has been
# silent. A heartbeat polls it every HEARTBEAT_INTERVAL_S; after
# HEARTBEAT_MAX_MISSES consecutive intervals with no packet, the link is treated
# as lost and a reconnect is attempted.
HEARTBEAT_INTERVAL_S = 15
HEARTBEAT_MAX_MISSES = 2


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


def center_head(robot):
    """Return the head to its neutral angle (level, facing forward).

    Done once on the first connection, not on reconnects, so a recovered link
    does not disturb a head pose a client has since set.
    """
    robot.head_yaw(0)
    robot.head_pitch(0)


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


def robot_silence_seconds(robot):
    """Seconds since the robot's last sensor packet (its heartbeat).

    Returns ``inf`` when no packet has ever arrived or the sensor read fails
    (for example mid-reconnect), so those count as a missed heartbeat.
    """
    try:
        last_packet = robot.get_dash_time()
    except Exception:
        return float("inf")
    if not last_packet:
        return float("inf")
    return max(0.0, time.time() - last_packet)


def reconnect_robot(robot, command_lock):
    """Re-establish a dropped BLE link, holding the command lock."""
    with command_lock:
        try:
            robot.reconnect()
        except Exception as error:
            print(f"Robot reconnect failed: {error}", flush=True)
            return False
    return True


async def monitor_heartbeat(
    robot,
    command_lock,
    stop,
    interval=HEARTBEAT_INTERVAL_S,
    max_misses=HEARTBEAT_MAX_MISSES,
):
    """Poll the robot's heartbeat and reconnect after repeated misses."""
    misses = 0
    while True:
        try:
            await asyncio.wait_for(stop.wait(), timeout=interval)
            return  # shutdown requested
        except asyncio.TimeoutError:
            pass

        silence = await asyncio.to_thread(robot_silence_seconds, robot)
        if silence < interval:
            if misses:
                print("Robot heartbeat recovered.", flush=True)
            misses = 0
            continue

        misses += 1
        print(
            f"Robot heartbeat missed ({misses}/{max_misses}); "
            f"no sensor data for {silence:.0f}s.",
            flush=True,
        )
        if misses < max_misses:
            continue

        print("Robot connection lost; attempting to reconnect...", flush=True)
        if await asyncio.to_thread(reconnect_robot, robot, command_lock):
            print("Robot reconnected.", flush=True)
            misses = 0
        else:
            # Stay tripped so the next interval retries the reconnect.
            misses = max_misses


async def run_server(robot, host=HOST, port=PORT):
    stop = asyncio.Event()
    command_lock = threading.Lock()
    loop = asyncio.get_running_loop()

    for signum in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(signum, stop.set)

    heartbeat = asyncio.create_task(monitor_heartbeat(robot, command_lock, stop))
    try:
        async with serve(
            lambda websocket: handle_client(websocket, robot, command_lock),
            host,
            port,
        ):
            print(f"Robot WebSocket server ready on ws://{host}:{port}", flush=True)
            await stop.wait()
    finally:
        heartbeat.cancel()
        try:
            await heartbeat
        except asyncio.CancelledError:
            pass


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
        center_head(robot)
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
