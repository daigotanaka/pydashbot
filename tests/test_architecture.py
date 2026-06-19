import ast
import unittest
from pathlib import Path
from unittest.mock import AsyncMock

from dash.core.actuators import CommonActuators, DashActuators
from dash.core.command_protocol import create_request, execute_json, execute_request

ROOT = Path(__file__).parents[1]


def imported_dash_modules(path):
    tree = ast.parse(path.read_text())
    modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            if node.module.startswith("dash."):
                modules.add(node.module)
        elif isinstance(node, ast.Import):
            modules.update(
                name.name for name in node.names if name.name.startswith("dash.")
            )
    return modules


class LayerBoundaryTests(unittest.TestCase):
    def test_low_level_actuators_do_not_import_higher_layers(self):
        modules = imported_dash_modules(ROOT / "dash" / "core" / "actuators.py")

        self.assertEqual(modules, {"dash.core.constants"})

    def test_motion_depends_only_on_low_level_actuators(self):
        modules = imported_dash_modules(ROOT / "dash" / "core" / "motion.py")

        self.assertEqual(modules, {"dash.core.actuators"})

    def test_sensors_depend_only_on_low_level_constants(self):
        modules = imported_dash_modules(ROOT / "dash" / "core" / "sensors.py")

        self.assertEqual(modules, {"dash.core.constants"})

    def test_command_protocol_does_not_depend_on_robot_layers(self):
        modules = imported_dash_modules(
            ROOT / "dash" / "core" / "command_protocol.py"
        )

        self.assertEqual(modules, set())


class CommandProtocolTests(unittest.TestCase):
    class Robot:
        def add(self, left, right=0):
            return left + right

    def test_request_creation_and_execution(self):
        request = create_request("add", [2], {"right": 3})

        self.assertEqual(
            execute_request(request, self.Robot()),
            {"ok": True, "result": 5},
        )

    def test_invalid_json_returns_error_response(self):
        response = execute_json("{", self.Robot())

        self.assertFalse(response["ok"])
        self.assertIn("Expecting", response["error"])


class ActuatorCompositionTests(unittest.IsolatedAsyncioTestCase):
    async def test_common_actuator_sends_stop_command(self):
        actuator = CommonActuators()
        actuator.command = AsyncMock()

        await actuator.stop()

        actuator.command.assert_awaited_once_with("drive", bytearray([0, 0, 0]))

    async def test_dash_actuator_sends_drive_command(self):
        actuator = DashActuators()
        actuator.command = AsyncMock()

        await actuator.drive(100)

        actuator.command.assert_awaited_once_with(
            "drive", bytearray([100, 0, 0])
        )
