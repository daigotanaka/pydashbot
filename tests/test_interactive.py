import asyncio
import threading
import unittest
from unittest.mock import patch

from dash.interactive import discover_and_connect_sync


class FakeRobot:
    def __init__(self):
        self.address = "test-address"
        self.calls = []
        self.thread_ids = []

    async def move(self, distance, speed):
        self.calls.append(("move", distance, speed))
        self.thread_ids.append(threading.get_ident())
        await asyncio.sleep(0)
        return "moved"

    async def disconnect(self):
        self.calls.append(("disconnect",))
        self.thread_ids.append(threading.get_ident())


class InteractiveRobotTest(unittest.TestCase):
    def test_runs_async_methods_on_persistent_background_loop(self):
        fake_robot = FakeRobot()

        async def fake_discover(**kwargs):
            return fake_robot

        with patch("dash.interactive.discover_and_connect", fake_discover):
            robot = discover_and_connect_sync(retry_attempts=1, retry_delay=0)
            self.assertEqual(robot.address, "test-address")
            self.assertEqual(robot.move(100, 100), "moved")
            robot.disconnect()

        self.assertEqual(
            fake_robot.calls,
            [("move", 100, 100), ("disconnect",)],
        )
        self.assertEqual(len(set(fake_robot.thread_ids)), 1)
        self.assertNotEqual(fake_robot.thread_ids[0], threading.get_ident())

    def test_returns_none_and_stops_loop_when_discovery_fails(self):
        async def fake_discover(**kwargs):
            return None

        with patch("dash.interactive.discover_and_connect", fake_discover):
            robot = discover_and_connect_sync(retry_attempts=1, retry_delay=0)

        self.assertIsNone(robot)

    def test_forwards_name_and_address_to_async_connection(self):
        received = {}

        async def fake_discover(**kwargs):
            received.update(kwargs)
            return None

        with patch("dash.interactive.discover_and_connect", fake_discover):
            discover_and_connect_sync(
                retry_attempts=1,
                retry_delay=0,
                name="Workshop Dash",
                address="AA:BB:CC:DD:EE:FF",
            )

        self.assertEqual(
            received,
            {
                "retry_attempts": 1,
                "retry_delay": 0,
                "name": "Workshop Dash",
                "address": "AA:BB:CC:DD:EE:FF",
            },
        )


if __name__ == "__main__":
    unittest.main()
