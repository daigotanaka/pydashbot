import math
import unittest
from itertools import count
from unittest.mock import AsyncMock, patch

from dash.core.actuators import compensate_turn
from dash.core.robot import DashRobot


def decode_turn_centiradians(packet):
    value = packet[2] | (((packet[5] >> 6) & 0x03) << 8)
    return value - 0x400 if packet[6] & 0xC0 == 0xC0 else value


class TurnTests(unittest.IsolatedAsyncioTestCase):
    def make_robot(self, yaw=(0, 0), left=(0, 0), right=(0, 0)):
        robot = DashRobot.__new__(DashRobot)
        robot.command = AsyncMock()
        robot.stop = AsyncMock()
        robot.say = AsyncMock()
        robot.get_yaw = lambda values=iter(yaw): next(values)
        robot.get_left_wheel = lambda values=iter(left): next(values)
        robot.get_right_wheel = lambda values=iter(right): next(values)
        return robot

    async def test_turns_encode_the_direction_compensated_angle_and_duration(self):
        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()) as sleep:
            robot = self.make_robot()
            await robot.turn(90)
            left_packet = robot.command.await_args.args[1]
            left_duration = sleep.await_args_list[0].args[0]

            robot = self.make_robot()
            await robot.turn(-90)
            right_packet = robot.command.await_args.args[1]
            right_duration = sleep.await_args_list[-2].args[0]

        # Each direction encodes its own deadband-compensated angle, and the
        # duration is timed off that same compensated angle (default 85.9 dps).
        self.assertEqual(
            decode_turn_centiradians(left_packet),
            int(math.radians(compensate_turn(90)) * 100),
        )
        self.assertEqual(
            decode_turn_centiradians(right_packet),
            int(math.radians(compensate_turn(-90)) * 100),
        )
        self.assertAlmostEqual(left_duration, abs(compensate_turn(90)) / 85.9)
        self.assertAlmostEqual(right_duration, abs(compensate_turn(-90)) / 85.9)
        # Direction sign flag still rides in byte 6.
        self.assertEqual(left_packet[6], 0x00)
        self.assertEqual(right_packet[6], 0xC0)

    async def test_turn_reports_executed_when_gyro_and_wheels_move(self):
        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            robot = self.make_robot(yaw=(0, 300), left=(0, 90), right=(0, -90))
            outcome = await robot.turn(21)
        self.assertEqual(outcome["halt"], "executed")
        self.assertEqual(outcome["yaw_delta"], 300)

    async def test_turn_reports_stall_when_wheels_do_not_move(self):
        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            robot = self.make_robot(yaw=(0, 1), left=(0, 2), right=(0, -1))
            outcome = await robot.turn(21)
        self.assertEqual(outcome["halt"], "stalled")

    async def test_turn_reports_no_yaw_response_when_only_wheels_move(self):
        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            robot = self.make_robot(yaw=(0, 2), left=(0, 90), right=(0, -90))
            outcome = await robot.turn(21)
        self.assertEqual(outcome["halt"], "no_yaw_response")


class ObstacleAwareMoveTests(unittest.IsolatedAsyncioTestCase):
    def make_robot(self, left=0, right=0, rear=0, pitch=0):
        robot = DashRobot.__new__(DashRobot)
        robot.command = AsyncMock()
        robot.stop = AsyncMock()
        robot.say = AsyncMock()
        robot.get_prox_left = lambda: left
        robot.get_prox_right = lambda: right
        robot.get_prox_rear = lambda: rear
        robot.get_pitch = lambda: pitch
        dash_times = count()
        dot_times = count()
        robot.get_dash_time = lambda: next(dash_times)
        robot.get_time = lambda: next(dot_times)
        return robot

    async def test_forward_move_stops_before_starting_when_wall_is_detected(self):
        robot = self.make_robot(left=15)

        await robot.move(200)

        robot.command.assert_not_awaited()
        robot.stop.assert_awaited_once()
        robot.say.assert_awaited_once_with("confused8")

    async def test_move_reports_obstacle_outcome_with_sensor_readings(self):
        robot = self.make_robot(left=99)

        outcome = await robot.move(200)

        self.assertEqual(outcome["halt"], "obstacle")
        self.assertEqual(outcome["side"], "front")
        self.assertEqual(outcome["prox_left"], 99)

    async def test_move_reports_completed_when_path_stays_clear(self):
        robot = self.make_robot()

        with patch("dash.core.motion.asyncio.get_running_loop") as get_loop:
            get_loop.return_value.time.side_effect = [0, 2]
            outcome = await robot.move(200)

        self.assertEqual(outcome["halt"], "completed")

    async def test_backward_move_checks_rear_sensor(self):
        robot = self.make_robot(left=255, right=255, rear=0)

        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.core.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 2]
                await robot.move(-200)

        robot.command.assert_awaited_once()
        robot.stop.assert_not_awaited()

    async def test_backward_move_stops_for_rear_wall(self):
        robot = self.make_robot(rear=20)

        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.core.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 0.01, 0.02, 0.03]
                await robot.move(-200)

        robot.command.assert_awaited_once()
        robot.stop.assert_awaited_once()
        robot.say.assert_awaited_once_with("confused8")

    async def test_backward_move_ignores_rear_sensor_idle_noise(self):
        robot = self.make_robot(rear=12)

        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.core.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 2]
                await robot.move(-200)

        robot.command.assert_awaited_once()
        robot.stop.assert_not_awaited()

    async def test_backward_move_requires_sustained_rear_detection(self):
        robot = self.make_robot()
        readings = iter([0, 20, 0, 20, 20, 20])
        robot.get_prox_rear = lambda: next(readings)

        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.core.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [
                    0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06
                ]
                await robot.move(-200)

        robot.stop.assert_awaited_once()
        robot.say.assert_awaited_once_with("confused8")

    async def test_backward_move_does_not_count_same_sensor_frame_repeatedly(self):
        robot = self.make_robot(rear=20)
        sensor_times = iter([0, 1, 1, 1])
        robot.get_dash_time = lambda: next(sensor_times)

        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.core.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 0.01, 0.02, 2]
                await robot.move(-200)

        robot.command.assert_awaited_once()
        robot.stop.assert_not_awaited()

    async def test_backward_move_uses_slower_sensor_safe_speed(self):
        robot = self.make_robot(rear=0)

        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.core.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 2]
                await robot.move(-200, speed_mmps=1000)

        packet = robot.command.await_args.args[1]
        encoded_time_ms = (packet[3] << 8) | packet[4]
        self.assertEqual(encoded_time_ms, 2000)

    async def test_move_stops_when_wall_appears_during_travel(self):
        robot = self.make_robot()
        readings = iter([0, 15, 15, 15])
        robot.get_prox_left = lambda: next(readings)

        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.core.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 0.01, 0.02, 0.03]
                await robot.move(200)

        robot.command.assert_awaited_once()
        robot.stop.assert_awaited_once()
        robot.say.assert_awaited_once_with("confused8")

    async def test_forward_move_ignores_single_proximity_spike(self):
        robot = self.make_robot()
        readings = iter([0, 15, 0])
        robot.get_prox_left = lambda: next(readings)

        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.core.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 0.01, 1]
                await robot.move(200)

        robot.stop.assert_not_awaited()
        robot.say.assert_not_awaited()

    async def test_obstacle_stop_can_be_disabled(self):
        robot = self.make_robot(left=255, right=255)

        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()) as sleep:
            await robot.move(200, stop_at_obstacle=False)

        robot.command.assert_awaited_once()
        sleep.assert_awaited_once()
        robot.stop.assert_not_awaited()
        robot.say.assert_not_awaited()

    async def test_obstacle_aware_move_caps_speed_for_sensor_response_time(self):
        robot = self.make_robot()

        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.core.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 1]
                await robot.move(200, speed_mmps=1000)

        packet = robot.command.await_args.args[1]
        encoded_time_ms = (packet[3] << 8) | packet[4]
        self.assertEqual(encoded_time_ms, 1000)

    async def test_wall_stop_sound_can_be_suppressed(self):
        robot = self.make_robot(left=15)

        await robot.move(200, wall_stop_sound=None)

        robot.stop.assert_awaited_once()
        robot.say.assert_not_awaited()

    async def test_move_stops_after_sustained_tilt(self):
        robot = self.make_robot()
        pitches = iter([0, 50, 50])
        robot.get_pitch = lambda: next(pitches)

        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.core.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 0.01, 0.02]
                await robot.move(200, tilt_confirm_count=2)

        robot.stop.assert_awaited_once()
        robot.say.assert_awaited_once_with("confused8")

    async def test_single_tilt_reading_does_not_stop_move(self):
        robot = self.make_robot()
        pitches = iter([0, 50, 0])
        robot.get_pitch = lambda: next(pitches)

        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.core.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 0.01, 1]
                await robot.move(200, tilt_confirm_count=2)

        robot.stop.assert_not_awaited()
        robot.say.assert_not_awaited()


class ArcTests(unittest.IsolatedAsyncioTestCase):
    def make_robot(self):
        robot = DashRobot.__new__(DashRobot)
        robot.command = AsyncMock()
        return robot

    def test_arc_relative_pose_geometry(self):
        from dash.core.motion import arc_relative_pose

        # 90 deg left arc of radius 100 ends at (100, 100) facing +90.
        x, y, t = arc_relative_pose(100, 90)
        self.assertAlmostEqual(x, 100)
        self.assertAlmostEqual(y, 100)
        self.assertEqual(t, 90)
        # Mirror image to the right.
        x, y, t = arc_relative_pose(100, -90)
        self.assertAlmostEqual(x, 100)
        self.assertAlmostEqual(y, -100)
        self.assertEqual(t, -90)
        # A 180 deg arc doubles back to (0, 2R).
        x, y, t = arc_relative_pose(100, 180)
        self.assertAlmostEqual(x, 0, places=6)
        self.assertAlmostEqual(y, 200)
        # No sweep is no motion.
        self.assertEqual(arc_relative_pose(100, 0), (0.0, 0.0, 0.0))

    async def test_arc_sends_pose_with_forward_and_lateral_displacement(self):
        from dash.core.motion import ARC_SPEED_MMPS, arc_relative_pose

        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()) as sleep:
            robot = self.make_robot()
            outcome = await robot.arc(100, 90)  # quarter circle, R=100, left

        method, packet = robot.command.await_args.args
        self.assertEqual(method, "pose")
        # Unlike a plain move, the packet carries BOTH a forward (x) and a
        # lateral (y) component -- that is what makes it an arc; a left arc
        # bends to +lateral.
        x_field = packet[0] | ((packet[5] & 0x3F) << 8)
        y_field = packet[1] | ((packet[6] & 0x3F) << 8)
        self.assertEqual(x_field, 100)  # x_mm = 100 -> 10 cm * 10
        self.assertEqual(y_field, 100)  # y_mm = 100 (left -> +lateral)
        # Duration is timed off the arc length (R * phi) at the arc speed.
        self.assertAlmostEqual(
            sleep.await_args.args[0], (100 * math.pi / 2) / ARC_SPEED_MMPS
        )
        self.assertEqual(outcome["rel_pose_mm"], [100.0, 100.0, 90.0])
        self.assertEqual(outcome["segments"], 1)

    async def test_arc_splits_wide_sweeps_into_subarcs(self):
        with patch("dash.core.motion.asyncio.sleep", new=AsyncMock()):
            robot = self.make_robot()
            outcome = await robot.arc(100, 180)  # half-circle -> two 90 deg arcs

        # A single 180 deg arc's target is nearly pure-lateral and stalls; it is
        # split into forward-dominant sub-arcs instead.
        self.assertEqual(outcome["segments"], 2)
        self.assertEqual(robot.command.await_count, 2)
        for call in robot.command.await_args_list:
            method, packet = call.args
            self.assertEqual(method, "pose")
            x_field = packet[0] | ((packet[5] & 0x3F) << 8)
            self.assertEqual(x_field, 100)  # each sub-arc is a 90 deg, x=100
        # The reported pose is still the full intended arc.
        self.assertEqual(outcome["rel_pose_mm"][2], 180.0)


if __name__ == "__main__":
    unittest.main()
