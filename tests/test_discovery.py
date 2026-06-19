import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch

from dash.core.robot import discover_and_connect


class DiscoveryTests(unittest.IsolatedAsyncioTestCase):
    async def test_default_name_is_doodle(self):
        device = SimpleNamespace(name="Doodle", address="robot-address")
        robot = SimpleNamespace(connect=AsyncMock())

        with patch(
            "dash.core.robot.BleakScanner.discover",
            new=AsyncMock(return_value=[device]),
        ):
            with patch("dash.core.robot.DashRobot", return_value=robot):
                result = await discover_and_connect(retry_attempts=1, retry_delay=0)

        self.assertIs(result, robot)

    async def test_discovers_robot_by_custom_name(self):
        device = SimpleNamespace(name="Workshop Dash", address="robot-address")
        robot = SimpleNamespace(connect=AsyncMock())

        with patch(
            "dash.core.robot.BleakScanner.discover",
            new=AsyncMock(return_value=[device]),
        ) as discover:
            with patch("dash.core.robot.DashRobot", return_value=robot) as robot_class:
                result = await discover_and_connect(
                    retry_attempts=1,
                    retry_delay=0,
                    name="Workshop Dash",
                )

        self.assertIs(result, robot)
        discover.assert_awaited_once()
        robot_class.assert_called_once_with("robot-address")
        robot.connect.assert_awaited_once()

    async def test_connects_directly_by_address_without_discovery(self):
        robot = SimpleNamespace(connect=AsyncMock())

        with patch(
            "dash.core.robot.BleakScanner.discover",
            new=AsyncMock(),
        ) as discover:
            with patch("dash.core.robot.DashRobot", return_value=robot) as robot_class:
                result = await discover_and_connect(
                    retry_attempts=1,
                    retry_delay=0,
                    address="AA:BB:CC:DD:EE:FF",
                )

        self.assertIs(result, robot)
        discover.assert_not_awaited()
        robot_class.assert_called_once_with("AA:BB:CC:DD:EE:FF")
        robot.connect.assert_awaited_once()


if __name__ == "__main__":
    unittest.main()
