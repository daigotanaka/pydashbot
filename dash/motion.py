"""High-level motion control built on robot actuators and sensor readings."""

import asyncio
import logging

from dash.actuators import encode_move

PROXIMITY_STOP_THRESHOLD = 15
PROXIMITY_CONFIRM_COUNT = 3
REAR_PROXIMITY_STOP_THRESHOLD = 20
REAR_PROXIMITY_CONFIRM_COUNT = 3
PROXIMITY_POLL_INTERVAL = 0.02
MAX_OBSTACLE_AWARE_SPEED_MMPS = 200
MAX_REVERSE_OBSTACLE_AWARE_SPEED_MMPS = 100
TILT_STOP_THRESHOLD = 40
TILT_CONFIRM_COUNT = 15


class MotionController:
    """High-level bounded motion policy for a Dash-compatible actuator."""

    async def turn(self, degrees, speed_dps=85.9):
        """Turn a bounded number of degrees and then stop."""
        if abs(degrees) > 360:
            return
        seconds = abs(degrees) / speed_dps
        await self.command("move", encode_move(0, degrees, seconds))
        await asyncio.sleep(seconds)
        await self.stop()

    async def move(
        self,
        distance_mm,
        speed_mmps=1000,
        no_turn=True,
        stop_at_obstacle=True,
        proximity_threshold=PROXIMITY_STOP_THRESHOLD,
        proximity_confirm_count=PROXIMITY_CONFIRM_COUNT,
        rear_proximity_threshold=REAR_PROXIMITY_STOP_THRESHOLD,
        rear_proximity_confirm_count=REAR_PROXIMITY_CONFIRM_COUNT,
        wall_stop_sound="confused8",
        tilt_threshold=TILT_STOP_THRESHOLD,
        tilt_confirm_count=TILT_CONFIRM_COUNT,
    ):
        """Move a bounded distance, optionally stopping for walls or tilt."""
        speed_mmps = abs(speed_mmps)
        if stop_at_obstacle:
            speed_mmps = min(speed_mmps, MAX_OBSTACLE_AWARE_SPEED_MMPS)
            if distance_mm < 0:
                speed_mmps = min(speed_mmps, MAX_REVERSE_OBSTACLE_AWARE_SPEED_MMPS)
        seconds = abs(distance_mm / speed_mmps)

        if stop_at_obstacle and distance_mm >= 0 and self._obstacle_in_path(
            distance_mm, proximity_threshold, rear_proximity_threshold
        ):
            await self._stop_for_obstacle(wall_stop_sound)
            return

        baseline_pitch = self.get_pitch() if stop_at_obstacle else None
        last_proximity_time = self.get_dash_time() if stop_at_obstacle else None
        last_tilt_time = self.get_time() if stop_at_obstacle else None
        tilt_streak = 0
        proximity_streak = 0
        rear_proximity_streak = 0

        flags = 0x81 if no_turn and distance_mm < 0 else 0x80
        await self.command("move", encode_move(distance_mm, 0, seconds, flags))
        logging.debug("Moving for %s seconds", seconds)
        if not stop_at_obstacle:
            await asyncio.sleep(seconds)
            return

        loop = asyncio.get_running_loop()
        deadline = loop.time() + seconds
        while True:
            remaining = deadline - loop.time()
            if remaining <= 0:
                return
            await asyncio.sleep(min(PROXIMITY_POLL_INTERVAL, remaining))

            proximity_time = self.get_dash_time()
            if proximity_time is None or proximity_time != last_proximity_time:
                last_proximity_time = proximity_time
                obstacle_detected = self._obstacle_in_path(
                    distance_mm, proximity_threshold, rear_proximity_threshold
                )
                if distance_mm < 0:
                    rear_proximity_streak = (
                        rear_proximity_streak + 1 if obstacle_detected else 0
                    )
                    if rear_proximity_streak >= rear_proximity_confirm_count:
                        await self._stop_for_obstacle(wall_stop_sound)
                        return
                else:
                    proximity_streak = (
                        proximity_streak + 1 if obstacle_detected else 0
                    )
                    if proximity_streak >= proximity_confirm_count:
                        await self._stop_for_obstacle(wall_stop_sound)
                        return

            tilt_time = self.get_time()
            if tilt_time is None or tilt_time != last_tilt_time:
                last_tilt_time = tilt_time
                pitch = self.get_pitch()
                if (
                    baseline_pitch is not None
                    and pitch is not None
                    and abs(pitch - baseline_pitch) > tilt_threshold
                ):
                    tilt_streak += 1
                    if tilt_streak >= tilt_confirm_count:
                        await self._stop_for_obstacle(wall_stop_sound)
                        return
                else:
                    tilt_streak = 0

    async def _stop_for_obstacle(self, wall_stop_sound):
        await self.stop()
        if wall_stop_sound:
            await self.say(wall_stop_sound)

    def _obstacle_in_path(
        self, distance_mm, proximity_threshold, rear_proximity_threshold
    ):
        if distance_mm < 0:
            readings = (self.get_prox_rear(),)
            threshold = rear_proximity_threshold
        elif distance_mm > 0:
            readings = (self.get_prox_left(), self.get_prox_right())
            threshold = proximity_threshold
        else:
            return False
        return any(
            reading is not None and reading >= threshold
            for reading in readings
        )

    _get_move_byte_array = staticmethod(encode_move)
