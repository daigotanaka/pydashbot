#!/usr/bin/env python3
"""Calibrate yaw and wheel-distance scales through the WebSocket server.

Start ``uv run dash.remote.server`` first and place the robot in open space.
"""

import argparse
import json
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

from dash.remote.client import send_command

CAL_DISTANCE_MM = 300


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output',
        metavar='FILE_PATH',
        help="write calibration JSON to FILE_PATH instead of a timestamped filename",
    )
    return parser.parse_args(args)


def timestamped_path(stem, suffix, now=None):
    """Return a path with a `_YYYYMMDD-HH-MM-SS` timestamp suffix."""
    now = now or datetime.now()
    return Path(f"{stem}_{now.strftime('%Y%m%d-%H-%M-%S')}{suffix}")


def read_settled(getter, stable=5, tol=2, timeout=4.0, poll=0.1):
    """Poll a sensor until its reading settles, then return it.

    The wheel_distance stream emits a sustained transient (~1s) right after the
    wheels stop, so a fixed sleep + single read can land on a wrong value. At
    true rest the reads are dead stable, so we wait until `stable` consecutive
    reads agree within `tol`. Falls back to the mode if it never settles.
    """
    hist = []
    t0 = time.time()
    while time.time() - t0 < timeout:
        hist.append(send_command(getter)['result'])
        recent = hist[-stable:]
        if len(recent) == stable and max(recent) - min(recent) <= tol:
            return recent[-1]
        time.sleep(poll)
    return Counter(hist).most_common(1)[0][0]


def wrap_delta(prev, curr, bits):
    half = 1 << (bits - 1)
    full = 1 << bits
    return (curr - prev + half) % full - half


def wheel_translation_delta(left_before, left_after, right_before, right_after):
    """Return translation ticks while canceling opposite wheel motion in turns."""
    left_delta = wrap_delta(left_before, left_after, 16)
    right_delta = wrap_delta(right_before, right_after, 16)
    return (left_delta + right_delta) / 2


def main(args=None):
    options = parse_args(args)
    print("=== Calibration ===")
    send_command('stop')
    time.sleep(1.5)

    yaw0 = read_settled('get_yaw')
    print("Turning 90° to calibrate yaw...")
    send_command('turn', 90)
    yaw90 = read_settled('get_yaw')

    yaw_delta = wrap_delta(yaw0, yaw90, 12)
    if abs(yaw_delta) < 3:
        print("  Warning: yaw delta too small, using default scale")
        deg_per_yaw = 0.076
        yaw_sign = 1
    else:
        deg_per_yaw = 90.0 / abs(yaw_delta)
        yaw_sign = 1 if yaw_delta > 0 else -1

    print(f"  Yaw: {yaw0} -> {yaw90}  delta={yaw_delta}  scale={deg_per_yaw:.4f} deg/unit  sign={yaw_sign:+d}")

    print(f"Moving {CAL_DISTANCE_MM}mm forward to calibrate distance...")
    left_before = read_settled('get_left_wheel')
    right_before = read_settled('get_right_wheel')
    send_command('move', CAL_DISTANCE_MM, 100)
    left_after = read_settled('get_left_wheel')
    right_after = read_settled('get_right_wheel')
    wheel_delta = wheel_translation_delta(
        left_before, left_after, right_before, right_after
    )
    if abs(wheel_delta) < 3:
        print("  Warning: distance delta too small, using default scale")
        mm_per_wheel_tick = 0.200
        wd_sign = 1
    else:
        mm_per_wheel_tick = float(CAL_DISTANCE_MM) / abs(wheel_delta)
        wd_sign = 1 if wheel_delta > 0 else -1

    print(
        f"  Wheel ticks: L {left_before}->{left_after}, "
        f"R {right_before}->{right_after}  average delta={wheel_delta:.1f}  "
        f"scale={mm_per_wheel_tick:.4f} mm/tick  sign={wd_sign:+d}"
    )

    print("Returning to start...")
    send_command('move', -CAL_DISTANCE_MM, 100)
    send_command('turn', -90)
    time.sleep(0.5)
    send_command('stop')

    cal = {
        'deg_per_yaw': deg_per_yaw,
        'yaw_sign': yaw_sign,
        'mm_per_wheel_tick': mm_per_wheel_tick,
        'wd_sign': wd_sign,
        'timestamp': datetime.now().isoformat(timespec='seconds'),
    }
    cal_file = (
        Path(options.output)
        if options.output
        else timestamped_path('calibration', '.json')
    )
    cal_file.parent.mkdir(parents=True, exist_ok=True)
    cal_file.write_text(json.dumps(cal, indent=2))
    print(f"\nCalibration saved -> {cal_file}")
    print("\nRun uv run apps.map start to start mapping.")


if __name__ == "__main__":
    try:
        main()
    finally:
        try:
            send_command("stop")
        except Exception:
            pass
