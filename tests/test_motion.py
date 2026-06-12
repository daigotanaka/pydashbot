import unittest
from itertools import count
from unittest.mock import AsyncMock, patch

from dash.robot import DashRobot


def decode_turn_centiradians(packet):
    value = packet[2] | (((packet[5] >> 6) & 0x03) << 8)
    return value - 0x400 if packet[6] & 0xC0 == 0xC0 else value


class TurnTests(unittest.IsolatedAsyncioTestCase):
    async def test_left_and_right_turns_use_equal_angle_and_duration(self):
        robot = DashRobot.__new__(DashRobot)
        robot.command = AsyncMock()
        robot.stop = AsyncMock()
        robot.say = AsyncMock()

        with patch("dash.motion.asyncio.sleep", new=AsyncMock()) as sleep:
            await robot.turn(90)
            left_packet = robot.command.await_args.args[1]
            left_duration = sleep.await_args.args[0]

            robot.command.reset_mock()
            sleep.reset_mock()

            await robot.turn(-90)
            right_packet = robot.command.await_args.args[1]
            right_duration = sleep.await_args.args[0]

        self.assertEqual(left_duration, right_duration)
        self.assertEqual(
            decode_turn_centiradians(left_packet),
            -decode_turn_centiradians(right_packet),
        )
        self.assertEqual(left_packet[3:5], right_packet[3:5])
        self.assertEqual(left_packet[6], 0x00)
        self.assertEqual(right_packet[6], 0xC0)


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

    async def test_backward_move_checks_rear_sensor(self):
        robot = self.make_robot(left=255, right=255, rear=0)

        with patch("dash.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 2]
                await robot.move(-200)

        robot.command.assert_awaited_once()
        robot.stop.assert_not_awaited()

    async def test_backward_move_stops_for_rear_wall(self):
        robot = self.make_robot(rear=20)

        with patch("dash.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 0.01, 0.02, 0.03]
                await robot.move(-200)

        robot.command.assert_awaited_once()
        robot.stop.assert_awaited_once()
        robot.say.assert_awaited_once_with("confused8")

    async def test_backward_move_ignores_rear_sensor_idle_noise(self):
        robot = self.make_robot(rear=12)

        with patch("dash.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 2]
                await robot.move(-200)

        robot.command.assert_awaited_once()
        robot.stop.assert_not_awaited()

    async def test_backward_move_requires_sustained_rear_detection(self):
        robot = self.make_robot()
        readings = iter([0, 20, 0, 20, 20, 20])
        robot.get_prox_rear = lambda: next(readings)

        with patch("dash.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.motion.asyncio.get_running_loop") as get_loop:
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

        with patch("dash.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 0.01, 0.02, 2]
                await robot.move(-200)

        robot.command.assert_awaited_once()
        robot.stop.assert_not_awaited()

    async def test_backward_move_uses_slower_sensor_safe_speed(self):
        robot = self.make_robot(rear=0)

        with patch("dash.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 2]
                await robot.move(-200, speed_mmps=1000)

        packet = robot.command.await_args.args[1]
        encoded_time_ms = (packet[3] << 8) | packet[4]
        self.assertEqual(encoded_time_ms, 2000)

    async def test_move_stops_when_wall_appears_during_travel(self):
        robot = self.make_robot()
        readings = iter([0, 15, 15, 15])
        robot.get_prox_left = lambda: next(readings)

        with patch("dash.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 0.01, 0.02, 0.03]
                await robot.move(200)

        robot.command.assert_awaited_once()
        robot.stop.assert_awaited_once()
        robot.say.assert_awaited_once_with("confused8")

    async def test_forward_move_ignores_single_proximity_spike(self):
        robot = self.make_robot()
        readings = iter([0, 15, 0])
        robot.get_prox_left = lambda: next(readings)

        with patch("dash.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 0.01, 1]
                await robot.move(200)

        robot.stop.assert_not_awaited()
        robot.say.assert_not_awaited()

    async def test_obstacle_stop_can_be_disabled(self):
        robot = self.make_robot(left=255, right=255)

        with patch("dash.motion.asyncio.sleep", new=AsyncMock()) as sleep:
            await robot.move(200, stop_at_obstacle=False)

        robot.command.assert_awaited_once()
        sleep.assert_awaited_once()
        robot.stop.assert_not_awaited()
        robot.say.assert_not_awaited()

    async def test_obstacle_aware_move_caps_speed_for_sensor_response_time(self):
        robot = self.make_robot()

        with patch("dash.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.motion.asyncio.get_running_loop") as get_loop:
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

        with patch("dash.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 0.01, 0.02]
                await robot.move(200, tilt_confirm_count=2)

        robot.stop.assert_awaited_once()
        robot.say.assert_awaited_once_with("confused8")

    async def test_single_tilt_reading_does_not_stop_move(self):
        robot = self.make_robot()
        pitches = iter([0, 50, 0])
        robot.get_pitch = lambda: next(pitches)

        with patch("dash.motion.asyncio.sleep", new=AsyncMock()):
            with patch("dash.motion.asyncio.get_running_loop") as get_loop:
                get_loop.return_value.time.side_effect = [0, 0.01, 1]
                await robot.move(200, tilt_confirm_count=2)

        robot.stop.assert_not_awaited()
        robot.say.assert_not_awaited()


if __name__ == "__main__":
    unittest.main()
