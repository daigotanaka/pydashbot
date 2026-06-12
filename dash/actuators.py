"""Low-level actuator value and BLE packet encoders."""

import math
import struct
from enum import IntEnum

from colour import Color

from dash.constants import NOISES


class PoseMode(IntEnum):
    """Pose interpretation modes."""

    GLOBAL = 0
    RELATIVE_COMMAND = 1
    RELATIVE_MEASURED = 2
    SET_GLOBAL = 3


class PoseDirection(IntEnum):
    """Movement direction."""

    FORWARD = 0
    BACKWARD = 1
    INFERRED = 2


def one_byte_array(value):
    return bytearray(struct.pack(">B", value))


def two_byte_array(value):
    return bytearray(struct.pack(">H", value))


def color_byte_array(color_value):
    color = Color(color_value)
    return bytearray([
        int(round(color.get_red() * 255)),
        int(round(color.get_green() * 255)),
        int(round(color.get_blue() * 255)),
    ])


def angle_array(angle):
    if angle < 0:
        angle = (abs(angle) ^ 0xFF) + 1
    return bytearray([angle & 0xFF])


def encode_pose(
    x=0.0,
    y=0.0,
    theta=0.0,
    time=0.0,
    mode=PoseMode.RELATIVE_MEASURED,
    direction=PoseDirection.INFERRED,
    wrap_theta=True,
    ease=True,
):
    """Encode a Dash pose command."""
    x_encoded = max(-8192, min(8191, int(round(x * 10.0))))
    y_encoded = max(-8192, min(8191, int(round(y * 10.0))))
    theta_encoded = max(-2048, min(2047, int(round(theta * 100.0))))
    time_ms = max(0, min(65535, int(round(time * 1000.0))))
    mode = 3 if mode == 5 else (int(mode) & 0x03)

    return struct.pack(
        "BBBBBBBB",
        x_encoded & 0xFF,
        y_encoded & 0xFF,
        theta_encoded & 0xFF,
        (time_ms >> 8) & 0xFF,
        time_ms & 0xFF,
        ((x_encoded >> 8) & 0x3F) | ((theta_encoded >> 2) & 0xC0),
        ((y_encoded >> 8) & 0x3F) | ((theta_encoded >> 4) & 0xC0),
        (mode << 6)
        | ((int(ease) & 0x01) << 5)
        | ((int(wrap_theta) & 0x01) << 4)
        | (int(direction) & 0x0F),
    )


def encode_move(distance_mm=0, degrees=0, seconds=1.0, flags=0x80):
    """Encode a low-level Dash move packet."""
    if distance_mm and degrees:
        raise NotImplementedError("Concurrent move and turn not supported.")

    distance_low_byte = distance_mm & 0xFF
    distance_high_byte = (distance_mm >> 8) & 0x3F
    sixth_byte = distance_high_byte

    centiradians = int(math.radians(degrees) * 100)
    turn_low_byte = centiradians & 0xFF
    turn_high_byte = (centiradians >> 8) & 0x03
    sixth_byte |= turn_high_byte << 6
    seventh_byte = 0xC0 if centiradians < 0 else 0x00

    time_ms = int(seconds * 1000)
    return bytearray([
        distance_low_byte,
        0x00,
        turn_low_byte,
        (time_ms >> 8) & 0xFF,
        time_ms & 0xFF,
        sixth_byte,
        seventh_byte,
        flags,
    ])


class CommonActuators:
    """Actuator commands shared by Dash, Dot, and Cue."""

    async def reset(self, mode=4):
        await self.command("reset", bytearray([mode]))

    async def eye(self, value):
        await self.command("eye", two_byte_array(value))

    async def eye_brightness(self, value):
        await self.command("eye_brightness", one_byte_array(value))

    async def neck_color(self, color):
        await self.command("neck_color", color_byte_array(color))

    async def left_ear_color(self, color):
        await self.command("left_ear_color", color_byte_array(color))

    async def right_ear_color(self, color):
        await self.command("right_ear_color", color_byte_array(color))

    async def stop(self):
        await self.command("drive", bytearray([0, 0, 0]))

    async def say(self, sound_name):
        if sound_name in NOISES:
            await self.command("say", bytearray(NOISES[sound_name]))
        else:
            print(f"Sound '{sound_name}' not found in sound bank.")


class DashActuators:
    """Low-level Dash-specific actuator commands."""

    async def tail_brightness(self, value):
        await self.command("tail_brightness", one_byte_array(max(0, min(255, value))))

    async def head_yaw(self, angle):
        await self.command("head_yaw", angle_array(max(-53, min(53, angle))))

    async def head_pitch(self, angle):
        await self.command("head_pitch", angle_array(max(-5, min(10, angle))))

    async def pose(
        self,
        x=0.0,
        y=0.0,
        theta=0.0,
        time=0.0,
        mode=PoseMode.RELATIVE_MEASURED,
        direction=PoseDirection.INFERRED,
        wrap_theta=True,
        ease=True,
    ):
        await self.command(
            "pose",
            encode_pose(x, y, theta, time, mode, direction, wrap_theta, ease),
        )

    async def drive(self, speed):
        speed = max(-2048, min(2048, speed))
        if speed < 0:
            speed = 0x8000 + abs(speed)
        await self.command(
            "drive",
            bytearray([speed & 0xFF, (speed >> 8) & 0xFF, 0x00]),
        )

    async def spin(self, speed):
        speed = max(-255, min(255, speed))
        await self.command("drive", bytearray([0x00, speed & 0xFF, 0x00]))
