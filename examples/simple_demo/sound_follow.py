"""React to a loud sound: turn head toward it, say 'huh?', then drive forward.

Requires the robot WebSocket server to be running:
    uv run python -m dash.remote.server

Then run this script:
    uv run python examples/simple_demo/sound_follow.py
"""

import argparse
import asyncio

from dash.remote.client import URI, build_uri, send_command_async

# Mic level (0-255) that counts as a "loud enough" sound event.
MIC_THRESHOLD = 40

# How long to collect sound-direction samples after a loud event (seconds).
DIRECTION_SAMPLE_WINDOW = 0.5

# Maximum distance to drive toward the sound (mm).
MAX_DRIVE_DISTANCE = 500

# Head yaw hardware limits (degrees).
HEAD_YAW_MAX = 53


def sound_direction_to_degrees(raw: int) -> float:
    """Convert raw sound_direction sensor value to signed degrees.

    The BLE packet yields a 16-bit value (0–65535).  Based on community
    reverse-engineering the value is a 0–359 degree bearing where 0 is
    straight ahead and the angle increases clockwise (right = positive,
    left = negative when mapped to ±180).
    """
    deg = raw % 360
    if deg > 180:
        deg -= 360
    return float(deg)


async def cmd(method, *args, uri=URI, **kwargs):
    """Send one command to the WS server and return the result value."""
    response = await send_command_async(method, *args, uri=uri, **kwargs)
    if not response.get("ok"):
        raise RuntimeError(f"{method} failed: {response.get('error')}")
    return response.get("result")


async def wait_for_loud_sound(threshold: int, uri: str) -> None:
    print(f"Listening… (mic threshold={threshold})")
    while True:
        level = await cmd("get_mic_level", uri=uri) or 0
        if level >= threshold:
            print(f"Sound detected! mic_level={level}")
            return
        await asyncio.sleep(0.05)


async def sample_sound_direction(window: float, uri: str) -> float | None:
    samples = []
    deadline = asyncio.get_event_loop().time() + window
    while asyncio.get_event_loop().time() < deadline:
        raw = await cmd("get_sound_direction", uri=uri)
        if raw is not None:
            samples.append(sound_direction_to_degrees(raw))
        await asyncio.sleep(0.05)

    if not samples:
        return None
    samples.sort()
    return samples[len(samples) // 2]


async def react_to_sound(threshold: int, uri: str) -> None:
    await wait_for_loud_sound(threshold, uri)

    direction_deg = await sample_sound_direction(DIRECTION_SAMPLE_WINDOW, uri)

    if direction_deg is None:
        print("Could not determine sound direction — skipping.")
        return

    print(f"Sound direction: {direction_deg:+.1f}°")

    # Turn head toward the sound (clamped to hardware limits).
    head_angle = max(-HEAD_YAW_MAX, min(HEAD_YAW_MAX, direction_deg))
    print(f"Turning head to {head_angle:+.1f}°")
    await cmd("head_yaw", head_angle, uri=uri)
    await asyncio.sleep(0.4)

    # Say "huh?"
    await cmd("say", "confused2", uri=uri)
    await asyncio.sleep(1.2)

    # Return head to neutral before body movement.
    print("Returning head to neutral")
    await cmd("head_yaw", 0, uri=uri)
    await asyncio.sleep(0.3)

    # Turn body toward the sound.
    print(f"Turning body {direction_deg:+.1f}°")
    await cmd("turn", direction_deg, uri=uri)
    await asyncio.sleep(0.3)

    # Drive forward (obstacle-aware on the server side, capped at MAX_DRIVE_DISTANCE).
    print(f"Driving forward up to {MAX_DRIVE_DISTANCE} mm")
    await cmd("move", MAX_DRIVE_DISTANCE, 150, uri=uri)

    print("Done.")


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="Server host")
    parser.add_argument("--port", type=int, default=8765, help="Server port")
    parser.add_argument(
        "--threshold", type=int, default=MIC_THRESHOLD,
        help=f"Mic level threshold 0-255 (default {MIC_THRESHOLD})"
    )
    return parser.parse_args(args)


LOOP_DURATION = 60  # seconds


async def main(host="127.0.0.1", port=8765, threshold=MIC_THRESHOLD):
    uri = build_uri(host, port)
    deadline = asyncio.get_event_loop().time() + LOOP_DURATION
    print(f"Running for {LOOP_DURATION}s — make some noise!")
    while asyncio.get_event_loop().time() < deadline:
        remaining = deadline - asyncio.get_event_loop().time()
        if remaining <= 0:
            break
        await asyncio.wait_for(react_to_sound(threshold, uri), timeout=remaining)
    print("Time's up.")


if __name__ == "__main__":
    opts = parse_args()
    asyncio.run(main(host=opts.host, port=opts.port, threshold=opts.threshold))
