import asyncio
import json
import threading
import time
import unittest

from websockets.exceptions import ConnectionClosedError, ConnectionClosedOK

from dash.ws_client import build_uri, parse_args as parse_client_args
from dash.ws_server import (
    center_head,
    execute_request,
    greet_robot,
    handle_client,
    monitor_heartbeat,
    parse_args as parse_server_args,
    reconnect_robot,
    robot_silence_seconds,
    say_goodbye_and_disconnect,
    silence_robot,
    stop_robot,
)


class FakeRobot:
    def __init__(self):
        self.calls = []

    def add(self, left, right=0):
        return left + right

    def say(self, sound):
        self.calls.append(("say", sound))

    def stop(self):
        self.calls.append(("stop",))

    def disconnect(self):
        self.calls.append(("disconnect",))

    def head_yaw(self, angle):
        self.calls.append(("head_yaw", angle))

    def head_pitch(self, angle):
        self.calls.append(("head_pitch", angle))


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

    def test_center_head_returns_head_to_neutral(self):
        robot = FakeRobot()

        center_head(robot)

        self.assertEqual(robot.calls, [("head_yaw", 0), ("head_pitch", 0)])

    def test_robot_stops_and_says_bye_before_disconnect(self):
        robot = FakeRobot()

        say_goodbye_and_disconnect(robot)

        self.assertEqual(
            robot.calls, [("stop",), ("say", "bye"), ("disconnect",)]
        )

    def test_robot_disconnects_when_saying_bye_fails(self):
        class FailingRobot(FakeRobot):
            def say(self, sound):
                super().say(sound)
                raise RuntimeError("sound failed")

        robot = FailingRobot()

        with self.assertRaisesRegex(RuntimeError, "sound failed"):
            say_goodbye_and_disconnect(robot)

        self.assertEqual(
            robot.calls, [("stop",), ("say", "bye"), ("disconnect",)]
        )


class FakeWebSocket:
    """Minimal async-iterable websocket for handle_client tests."""

    def __init__(self, messages, iter_exc=None, send_exc=None):
        self._messages = list(messages)
        self._iter_exc = iter_exc
        self._send_exc = send_exc
        self.sent = []

    def __aiter__(self):
        return self

    async def __anext__(self):
        if self._messages:
            return self._messages.pop(0)
        if self._iter_exc is not None:
            raise self._iter_exc
        raise StopAsyncIteration

    async def send(self, data):
        if self._send_exc is not None:
            raise self._send_exc
        self.sent.append(data)


class HandleClientDisconnectTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.robot = FakeRobot()
        self.lock = threading.Lock()

    async def test_clean_exchange_does_not_stop_robot(self):
        message = json.dumps({"method": "add", "args": [2], "kwargs": {"right": 3}})
        websocket = FakeWebSocket([message])

        await handle_client(websocket, self.robot, self.lock)

        self.assertEqual(websocket.sent, [json.dumps({"ok": True, "result": 5})])
        self.assertNotIn(("stop",), self.robot.calls)

    async def test_abrupt_disconnect_on_send_stops_robot(self):
        message = json.dumps({"method": "add", "args": [1]})
        websocket = FakeWebSocket(
            [message], send_exc=ConnectionClosedError(None, None)
        )

        await handle_client(websocket, self.robot, self.lock)

        self.assertEqual(self.robot.calls, [("stop",)])

    async def test_abrupt_disconnect_while_waiting_stops_robot(self):
        websocket = FakeWebSocket([], iter_exc=ConnectionClosedError(None, None))

        await handle_client(websocket, self.robot, self.lock)

        self.assertEqual(self.robot.calls, [("stop",)])

    async def test_clean_close_mid_send_does_not_stop_robot(self):
        message = json.dumps({"method": "add", "args": [1]})
        websocket = FakeWebSocket([message], send_exc=ConnectionClosedOK(None, None))

        await handle_client(websocket, self.robot, self.lock)

        self.assertNotIn(("stop",), self.robot.calls)

    async def test_stop_robot_swallows_errors(self):
        class FailingRobot(FakeRobot):
            def stop(self):
                super().stop()
                raise RuntimeError("ble dropped")

        robot = FailingRobot()

        stop_robot(robot, self.lock)

        self.assertEqual(robot.calls, [("stop",)])


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


class HeartbeatRobot:
    """Fake robot whose last-packet time and reconnect are controllable."""

    def __init__(self, healthy=True):
        self.healthy = healthy
        self.reconnects = 0
        self.on_reconnect = None

    def get_dash_time(self):
        # Fresh wall-clock time when healthy; 0 (never) when silent.
        return time.time() if self.healthy else 0

    def reconnect(self):
        self.reconnects += 1
        self.healthy = True
        if self.on_reconnect:
            self.on_reconnect()


class HeartbeatTests(unittest.TestCase):
    def test_silence_seconds_reports_inf_when_never_seen_or_failing(self):
        class NeverSeen:
            def get_dash_time(self):
                return 0

        class Failing:
            def get_dash_time(self):
                raise RuntimeError("ble dropped")

        self.assertEqual(robot_silence_seconds(NeverSeen()), float("inf"))
        self.assertEqual(robot_silence_seconds(Failing()), float("inf"))

    def test_silence_seconds_measures_time_since_last_packet(self):
        class SeenAt:
            def __init__(self, when):
                self.when = when

            def get_dash_time(self):
                return self.when

        self.assertLess(robot_silence_seconds(SeenAt(time.time())), 1.0)
        self.assertGreater(robot_silence_seconds(SeenAt(time.time() - 100)), 50)

    def test_reconnect_robot_reports_success_and_failure(self):
        lock = threading.Lock()
        robot = HeartbeatRobot()
        self.assertTrue(reconnect_robot(robot, lock))
        self.assertEqual(robot.reconnects, 1)

        def boom():
            raise RuntimeError("ble down")

        robot.reconnect = boom
        self.assertFalse(reconnect_robot(robot, lock))


class HeartbeatLoopTests(unittest.IsolatedAsyncioTestCase):
    async def test_reconnects_after_max_missed_heartbeats(self):
        stop = asyncio.Event()
        robot = HeartbeatRobot(healthy=False)  # silent from the start
        robot.on_reconnect = stop.set  # end the loop once it recovers

        await asyncio.wait_for(
            monitor_heartbeat(
                robot, threading.Lock(), stop, interval=0.01, max_misses=2
            ),
            timeout=2,
        )
        self.assertEqual(robot.reconnects, 1)

    async def test_healthy_robot_is_never_reconnected(self):
        stop = asyncio.Event()
        robot = HeartbeatRobot(healthy=True)

        async def shutdown_soon():
            await asyncio.sleep(0.1)
            stop.set()

        await asyncio.gather(
            monitor_heartbeat(
                robot, threading.Lock(), stop, interval=0.01, max_misses=2
            ),
            shutdown_soon(),
        )
        self.assertEqual(robot.reconnects, 0)


if __name__ == "__main__":
    unittest.main()
