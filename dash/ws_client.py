"""Send a command to the running robot WebSocket server."""

import argparse
import asyncio
import json

from websockets.asyncio.client import connect

from dash.command_protocol import create_request, parse_cli_values

HOST = "127.0.0.1"
PORT = 8765
URI = f"ws://{HOST}:{PORT}"


def build_uri(host=HOST, port=PORT):
    return f"ws://{host}:{port}"


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description="Send a command to the running robot WebSocket server."
    )
    parser.add_argument("--host", default=HOST, help=f"Server IP (default: {HOST})")
    parser.add_argument(
        "--port", type=int, default=PORT, help=f"Server port (default: {PORT})"
    )
    parser.add_argument(
        "--no-wall-sound",
        action="store_true",
        help="Suppress the sound when move stops for a wall",
    )
    parser.add_argument("method", help="Robot method to call")
    parser.add_argument("args", nargs="*", help="Arguments passed to the robot method")
    return parser.parse_args(args)


async def send_command_async(method, *args, uri=URI, **kwargs):
    request = json.dumps(create_request(method, args, kwargs))
    async with connect(uri) as websocket:
        await websocket.send(request)
        response = await websocket.recv()
    return json.loads(response)


def send_command(method, *args, uri=URI, **kwargs):
    return asyncio.run(send_command_async(method, *args, uri=uri, **kwargs))


def main():
    options = parse_args()
    command_args = parse_cli_values(options.args)
    kwargs = {}
    if options.no_wall_sound:
        if options.method != "move":
            raise SystemExit("--no-wall-sound can only be used with move")
        kwargs["wall_stop_sound"] = None
    result = send_command(
        options.method,
        *command_args,
        uri=build_uri(options.host, options.port),
        **kwargs,
    )
    if result["ok"]:
        print(result.get("result"))
    else:
        raise SystemExit(f"Error: {result['error']}")


if __name__ == "__main__":
    main()
