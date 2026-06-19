"""High-level motion control built on robot actuators and sensor readings."""

import asyncio
import logging

from dash.core.actuators import compensate_turn, encode_move

PROXIMITY_STOP_THRESHOLD = 15
PROXIMITY_CONFIRM_COUNT = 3
REAR_PROXIMITY_STOP_THRESHOLD = 20
REAR_PROXIMITY_CONFIRM_COUNT = 3
PROXIMITY_POLL_INTERVAL = 0.02
MAX_OBSTACLE_AWARE_SPEED_MMPS = 200
MAX_REVERSE_OBSTACLE_AWARE_SPEED_MMPS = 100
TILT_STOP_THRESHOLD = 40
TILT_CONFIRM_COUNT = 15

# Closed-loop turn verification. After commanding a turn we read the gyro and
# wheel encoders to report *why* a turn did or did not take effect, instead of
# returning silently. Thresholds are in raw sensor counts.
TURN_SETTLE_SECONDS = 0.05
TURN_STALL_WHEEL_COUNTS = 20   # below this, wheels effectively did not move
TURN_STALL_YAW_COUNTS = 12     # below this, the gyro registered no rotation

# Largest turn the move packet can carry. The angle is sent as centiradians in
# a 10-bit magnitude field, so it rolls over at 1024 centiradians (~587 deg);
# we cap just below that to keep the firmware seeing the angle we commanded.
MAX_TURN_DEGREES = 585


def to_packet_int(value, name):
    """Coerce ``value`` to the int the packet encoder requires.

    Dash's move packet packs ``distance_mm`` with bitwise operations, so a
    float would raise ``TypeError``. We round to the nearest int and warn if
    the rounded value differs from the original, since that loses precision.
    """
    converted = int(round(value))
    if converted != value:
        logging.warning(
            "%s=%r is not an integer; encoding as %d (Dash packets require "
            "integer values)",
            name,
            value,
            converted,
        )
    return converted


def wrap_signed_delta(previous, current, bits):
    """Signed change of a counter that wraps at ``2 ** bits``."""
    if previous is None or current is None:
        return None
    span = 1 << bits
    delta = (current - previous) % span
    if delta >= span // 2:
        delta -= span
    return delta


class MotionController:
    """High-level bounded motion policy for a Dash-compatible actuator."""

    async def turn(self, degrees, speed_dps=85.9):
        """Turn a bounded number of degrees, then stop and report the outcome.

        Returns a dict describing whether the turn took effect, distinguishing a
        mechanical stall (wheels did not move) from a gyro that registered no
        rotation. ``yaw_delta`` and the wheel deltas are raw sensor counts; the
        caller converts them to degrees with its own calibration.
        """
        if abs(degrees) > MAX_TURN_DEGREES:
            return {"halt": "invalid", "commanded_deg": degrees}
        yaw_before = self.get_yaw()
        left_before = self.get_left_wheel()
        right_before = self.get_right_wheel()
        # Time the turn off the deadband-compensated angle so the commanded
        # angular speed stays at speed_dps; encode_move applies the same
        # compensation to the packet angle.
        seconds = abs(compensate_turn(degrees)) / speed_dps
        await self.command("move", encode_move(0, degrees, seconds))
        await asyncio.sleep(seconds)
        await self.stop()
        await asyncio.sleep(TURN_SETTLE_SECONDS)
        yaw_delta = wrap_signed_delta(yaw_before, self.get_yaw(), 12)
        left_delta = wrap_signed_delta(left_before, self.get_left_wheel(), 16)
        right_delta = wrap_signed_delta(right_before, self.get_right_wheel(), 16)
        return self._turn_outcome(degrees, yaw_delta, left_delta, right_delta)

    @staticmethod
    def _turn_outcome(degrees, yaw_delta, left_delta, right_delta):
        wheels_moved = max(abs(left_delta or 0), abs(right_delta or 0))
        yaw_moved = abs(yaw_delta or 0)
        if wheels_moved < TURN_STALL_WHEEL_COUNTS and yaw_moved < TURN_STALL_YAW_COUNTS:
            reason = "stalled"           # wheels did not move (mechanical/surface)
        elif yaw_moved < TURN_STALL_YAW_COUNTS:
            reason = "no_yaw_response"   # wheels moved but the gyro saw no rotation
        else:
            reason = "executed"          # the turn took effect; caller checks size
        return {
            "halt": reason,
            "commanded_deg": degrees,
            "yaw_delta": yaw_delta,
            "left_wheel_delta": left_delta,
            "right_wheel_delta": right_delta,
        }

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

        if stop_at_obstacle and distance_mm >= 0:
            detected, readings = self._obstacle_in_path(
                distance_mm, proximity_threshold, rear_proximity_threshold
            )
            if detected:
                await self._stop_for_obstacle(wall_stop_sound)
                return self._obstacle_outcome(
                    "before_start", readings,
                    elapsed_seconds=0.0, speed_mmps=speed_mmps,
                )

        baseline_pitch = self.get_pitch() if stop_at_obstacle else None
        last_proximity_time = self.get_dash_time() if stop_at_obstacle else None
        last_tilt_time = self.get_time() if stop_at_obstacle else None
        tilt_streak = 0
        proximity_streak = 0
        rear_proximity_streak = 0

        flags = 0x81 if no_turn and distance_mm < 0 else 0x80
        distance_mm = to_packet_int(distance_mm, "distance_mm")
        await self.command("move", encode_move(distance_mm, 0, seconds, flags))
        logging.debug("Moving for %s seconds", seconds)
        if not stop_at_obstacle:
            await asyncio.sleep(seconds)
            return {"halt": "completed", "monitored": False}

        loop = asyncio.get_running_loop()
        deadline = loop.time() + seconds
        while True:
            remaining = deadline - loop.time()
            if remaining <= 0:
                return {"halt": "completed", "monitored": True}
            # Time spent moving so far; the obstacle halts below report it so the
            # mapper can compare logged travel against the commanded speed.
            elapsed = seconds - remaining
            await asyncio.sleep(min(PROXIMITY_POLL_INTERVAL, remaining))

            proximity_time = self.get_dash_time()
            if proximity_time is None or proximity_time != last_proximity_time:
                last_proximity_time = proximity_time
                obstacle_detected, readings = self._obstacle_in_path(
                    distance_mm, proximity_threshold, rear_proximity_threshold
                )
                if distance_mm < 0:
                    rear_proximity_streak = (
                        rear_proximity_streak + 1 if obstacle_detected else 0
                    )
                    if rear_proximity_streak >= rear_proximity_confirm_count:
                        await self._stop_for_obstacle(wall_stop_sound)
                        return self._obstacle_outcome(
                            "moving", readings,
                            elapsed_seconds=elapsed, speed_mmps=speed_mmps,
                        )
                else:
                    proximity_streak = (
                        proximity_streak + 1 if obstacle_detected else 0
                    )
                    if proximity_streak >= proximity_confirm_count:
                        await self._stop_for_obstacle(wall_stop_sound)
                        return self._obstacle_outcome(
                            "moving", readings,
                            elapsed_seconds=elapsed, speed_mmps=speed_mmps,
                        )

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
                        return {
                            "halt": "tilt",
                            "phase": "moving",
                            "pitch": pitch,
                            "baseline_pitch": baseline_pitch,
                            "pitch_delta": pitch - baseline_pitch,
                        }
                else:
                    tilt_streak = 0

    async def _stop_for_obstacle(self, wall_stop_sound):
        await self.stop()
        if wall_stop_sound:
            await self.say(wall_stop_sound)

    @staticmethod
    def _obstacle_outcome(phase, readings, elapsed_seconds=None, speed_mmps=None):
        """Build an obstacle halt outcome from the readings that triggered it.

        `elapsed_seconds` (time spent moving before the halt) and `speed_mmps`
        (the effective commanded speed) let callers tell a genuine near-target
        stop from a common-mode slip: the wheels can only log distance at the
        commanded speed, so travel implying a much higher speed is an encoder
        over-read while blocked.
        """
        outcome = {"halt": "obstacle", "phase": phase, **readings}
        if elapsed_seconds is not None:
            outcome["elapsed_seconds"] = round(elapsed_seconds, 3)
        if speed_mmps is not None:
            outcome["speed_mmps"] = speed_mmps
        return outcome

    def _obstacle_in_path(
        self, distance_mm, proximity_threshold, rear_proximity_threshold
    ):
        """Return (detected, readings) where readings names the sensors used.

        The readings are returned so a halt can report the exact values that
        triggered it without issuing a second sensor read.
        """
        if distance_mm < 0:
            rear = self.get_prox_rear()
            detected = rear is not None and rear >= rear_proximity_threshold
            return detected, {"side": "rear", "prox_rear": rear}
        if distance_mm > 0:
            left = self.get_prox_left()
            right = self.get_prox_right()
            detected = any(
                reading is not None and reading >= proximity_threshold
                for reading in (left, right)
            )
            return detected, {"side": "front", "prox_left": left, "prox_right": right}
        return False, {"side": "none"}

    _get_move_byte_array = staticmethod(encode_move)
