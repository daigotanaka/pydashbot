import asyncio
import json
import threading
import unittest

from dash.ws_client import build_uri, parse_args as parse_client_args
from dash.ws_server import (
    execute_request,
    greet_robot,
    parse_args as parse_server_args,
    say_goodbye_and_disconnect,
    silence_robot,
)


class FakeRobot:
    def __init__(self):
        self.calls = []

    def add(self, left, right=0):
        return left + right

    def say(self, sound):
        self.calls.append(("say", sound))

    def disconnect(self):
        self.calls.append(("disconnect",))


class WebSocketProtocolTests(unittest.TestCase):
    def setUp(self):
        self.robot = FakeRobot()
        self.lock = threading.Lock()

    def test_execute_request_with_args_and_kwargs(self):
        message = json.dumps(
            {"method": "add", "args": [2], "kwargs": {"right": 3}}
        )

        response = execute_request(message, self.robot, self.lock)

        self.assertEqual(response, {"ok": True, "result": 5})

    def test_execute_request_returns_error(self):
        message = json.dumps({"method": "missing"})

        response = execute_request(message, self.robot, self.lock)

        self.assertFalse(response["ok"])
        self.assertIn("missing", response["error"])


class WebSocketArgumentTests(unittest.TestCase):
    def test_server_uses_default_host_and_port(self):
        args = parse_server_args([])

        self.assertEqual(args.host, "127.0.0.1")
        self.assertEqual(args.port, 8765)
        self.assertEqual(args.name, "Doodle")
        self.assertIsNone(args.address)

    def test_server_accepts_host_and_port(self):
        args = parse_server_args(["--host", "0.0.0.0", "--port", "9000"])

        self.assertEqual(args.host, "0.0.0.0")
        self.assertEqual(args.port, 9000)

    def test_server_accepts_robot_name(self):
        args = parse_server_args(["--name", "Workshop Dash"])

        self.assertEqual(args.name, "Workshop Dash")
        self.assertIsNone(args.address)

    def test_server_accepts_direct_robot_address(self):
        args = parse_server_args(["--address", "AA:BB:CC:DD:EE:FF"])

        self.assertEqual(args.address, "AA:BB:CC:DD:EE:FF")

    def test_server_rejects_name_and_address_together(self):
        with self.assertRaises(SystemExit):
            parse_server_args(
                ["--name", "Workshop Dash", "--address", "AA:BB:CC:DD:EE:FF"]
            )

    def test_client_accepts_host_port_and_command_args(self):
        args = parse_client_args(
            ["--host", "192.0.2.1", "--port", "9000", "move", "100", "50"]
        )

        self.assertEqual(build_uri(args.host, args.port), "ws://192.0.2.1:9000")
        self.assertEqual(args.method, "move")
        self.assertEqual(args.args, ["100", "50"])
        self.assertFalse(args.no_wall_sound)

    def test_client_accepts_negative_command_args_after_separator(self):
        args = parse_client_args(["move", "--", "-100", "50"])

        self.assertEqual(args.args, ["-100", "50"])

    def test_client_accepts_wall_sound_suppression(self):
        args = parse_client_args(["--no-wall-sound", "move", "200"])

        self.assertTrue(args.no_wall_sound)
        self.assertEqual(args.method, "move")

    def test_server_silent_defaults_off_and_accepts_flag(self):
        self.assertFalse(parse_server_args([]).silent)
        self.assertTrue(parse_server_args(["--silent"]).silent)


class SilentModeTests(unittest.IsolatedAsyncioTestCase):
    async def test_silence_robot_makes_say_a_no_op(self):
        class AsyncRobot:
            def __init__(self):
                self.said = []

            async def say(self, sound):
                self.said.append(sound)

        class Facade:
            def __init__(self, robot):
                self.async_robot = robot

        robot = AsyncRobot()
        facade = Facade(robot)

        silence_robot(facade)
        result = await robot.say("confused8")

        self.assertIsNone(result)
        self.assertEqual(robot.said, [])


class WebSocketLifecycleTests(unittest.TestCase):
    def test_robot_says_hi_after_connecting(self):
        robot = FakeRobot()

        greet_robot(robot)

        self.assertEqual(robot.calls, [("say", "hi")])

    def test_robot_says_bye_before_disconnect(self):
        robot = FakeRobot()

        say_goodbye_and_disconnect(robot)

        self.assertEqual(robot.calls, [("say", "bye"), ("disconnect",)])

    def test_robot_disconnects_when_saying_bye_fails(self):
        class FailingRobot(FakeRobot):
            def say(self, sound):
                super().say(sound)
                raise RuntimeError("sound failed")

        robot = FailingRobot()

        with self.assertRaisesRegex(RuntimeError, "sound failed"):
            say_goodbye_and_disconnect(robot)

        self.assertEqual(robot.calls, [("say", "bye"), ("disconnect",)])


class WebSocketClientTests(unittest.IsolatedAsyncioTestCase):
    async def test_client_round_trip(self):
        try:
            from websockets.asyncio.server import serve
            from dash.ws_client import send_command_async
        except ImportError:
            self.skipTest("websockets is not installed")

        async def echo_command(websocket):
            request = json.loads(await websocket.recv())
            await websocket.send(
                json.dumps({"ok": True, "result": request["args"]})
            )

        async with serve(echo_command, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            response = await send_command_async(
                "echo", 1, "two", uri=f"ws://127.0.0.1:{port}"
            )

        self.assertEqual(response, {"ok": True, "result": [1, "two"]})


if __name__ == "__main__":
    unittest.main()
