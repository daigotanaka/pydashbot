#!/usr/bin/env python3
"""Explore a room through the WebSocket server and build a 2D map.

Starting ritual (run every session for a consistent origin):
  1. Place the robot in a corner, back roughly facing one wall, side roughly
     facing the adjacent wall.
  2. The dock routine will back into the rear wall, then crawl into the side
     wall, establishing (0, 0) at the corner with heading 0° pointing into
     the room.
"""

import json
import math
import random
import time
from datetime import datetime
from pathlib import Path

from dash.ws_client import send_command

MAP_FILE = Path('room_map.json')
CAL_FILE = Path('calibration.json')

# --- Tunable parameters ---
PROX_THRESHOLD    = 15
REAR_THRESHOLD    = 20    # rear sensor fires slightly differently
DOCK_SPEED        = 50    # mm/s-equivalent drive speed for docking
SPEED             = 100
POLL_INTERVAL     = 0.05
PITCH_TILT_THRESHOLD = 40
TILT_CONFIRM_COUNT   = 6
COOLDOWN          = 1.5
DURATION          = 60
WALL_OFFSET_MM    = 150
OBSTACLE_OFFSET_MM = 100
DOCK_CLEARANCE_MM = 80    # back off this far from each wall after contact

WALL_SOUNDS   = ['ohno', 'ayayay', 'huh', 'confused2', 'confused3']
TILT_SOUNDS   = ['ayayay', 'ohno', 'confused5', 'confused8']
RESUME_SOUNDS = ['okay', 'wee']


def wrap_delta(prev, curr, bits):
    half = 1 << (bits - 1)
    full = 1 << bits
    d = curr - prev
    if d > half:
        d -= full
    elif d < -half:
        d += full
    return d


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------
def load_calibration():
    if not CAL_FILE.exists():
        raise FileNotFoundError(
            f"No calibration file found. Run uv run tools/calibrate.py first."
        )
    cal = json.loads(CAL_FILE.read_text())
    deg_per_yaw = cal['deg_per_yaw'] * cal.get('yaw_sign', 1)
    mm_per_wd   = cal['mm_per_wd']   * cal.get('wd_sign',  1)
    print(f"=== Calibration loaded from {CAL_FILE} (recorded {cal.get('timestamp', 'unknown')}) ===")
    print(f"  deg_per_yaw={deg_per_yaw:.4f}  mm_per_wd={mm_per_wd:.4f}")
    return deg_per_yaw, mm_per_wd


# ---------------------------------------------------------------------------
# Corner dock — establishes (0, 0) as the corner, robot ends up facing room
# ---------------------------------------------------------------------------
def dock_to_corner(deg_per_yaw, mm_per_wd):
    """Back into rear wall, then crawl into left side wall to find corner."""
    print("\n=== Corner Dock ===")
    print("  Place robot near a corner, back toward one wall, left side toward")
    print("  the adjacent wall. Starting in 15 seconds...")
    for i in range(15, 0, -1):
        print(f'  {i}...', end='\r')
        if i == 5:
            send_command('say', 'beep')
        time.sleep(1)
    print()

    # -- Step 1: back into rear wall --
    print("  Backing into rear wall...")
    send_command('drive', -DOCK_SPEED)
    while True:
        rear = send_command('get_prox_rear')['result']
        print(f'    prox_rear={rear}', end='\r')
        if rear >= REAR_THRESHOLD:
            send_command('stop')
            print(f'\n  Rear wall contact (prox_rear={rear})')
            break
        time.sleep(POLL_INTERVAL)

    send_command('say', 'okay')
    time.sleep(0.3)

    # Clear slightly from rear wall
    send_command('move', DOCK_CLEARANCE_MM, 80)
    time.sleep(0.2)

    # -- Step 2: turn left, crawl into side wall --
    print("  Turning left to find side wall...")
    send_command('turn', -90)
    time.sleep(0.2)

    print("  Crawling into side wall...")
    send_command('drive', DOCK_SPEED)
    while True:
        l = send_command('get_prox_left')['result']
        r = send_command('get_prox_right')['result']
        print(f'    prox L={l} R={r}', end='\r')
        if l >= PROX_THRESHOLD or r >= PROX_THRESHOLD:
            send_command('stop')
            print(f'\n  Side wall contact (prox L={l} R={r})')
            break
        time.sleep(POLL_INTERVAL)

    send_command('say', 'okay')
    time.sleep(0.3)

    # Clear slightly from side wall
    send_command('move', -DOCK_CLEARANCE_MM, 80)
    time.sleep(0.2)

    # -- Step 3: turn right to face into room --
    print("  Turning to face room...")
    send_command('turn', 90)
    time.sleep(0.3)

    # Robot is now at approximately (DOCK_CLEARANCE_MM, DOCK_CLEARANCE_MM)
    # relative to the corner, facing into the room (heading 0°).
    x0 = float(DOCK_CLEARANCE_MM)
    y0 = float(DOCK_CLEARANCE_MM)
    print(f"  Docked. Starting position: ({x0:.0f}, {y0:.0f}) mm from corner, heading 0°")
    send_command('neck_color', '#00ffff')
    return x0, y0


# ---------------------------------------------------------------------------
# Exploration
# ---------------------------------------------------------------------------
def explore(deg_per_yaw, mm_per_wd, x0, y0):
    yaw_prev = send_command('get_yaw')['result']
    wd_prev  = send_command('get_wheel_distance')['result']
    pitch_samples = [send_command('get_pitch')['result'] for _ in range(5)]
    baseline_pitch = sum(pitch_samples) / len(pitch_samples)

    heading = 0.0
    x, y = x0, y0

    path      = [(x, y, heading)]
    walls     = []
    obstacles = []

    def recover(reason, sounds):
        nonlocal heading, x, y, yaw_prev, wd_prev
        sound = random.choice(sounds)
        print(f'\n  [{reason}] → "{sound}"')
        send_command('stop')
        send_command('neck_color', '#ff0000')
        send_command('say', sound)
        send_command('move', -200, 150)
        angle = random.choice([-120, -90, 90, 120])
        send_command('turn', angle)
        send_command('say', random.choice(RESUME_SOUNDS))
        send_command('neck_color', '#00ff00')
        send_command('drive', SPEED)
        yaw_prev = send_command('get_yaw')['result']
        wd_prev  = send_command('get_wheel_distance')['result']

    print(f"\n=== Exploring for {DURATION}s ===")
    send_command('say', 'hi')
    send_command('neck_color', '#00ff00')
    send_command('drive', SPEED)

    end_time       = time.time() + DURATION
    cooldown_until = 0.0
    tilt_streak    = 0

    try:
        while time.time() < end_time:
            now       = time.time()
            remaining = end_time - now

            l       = send_command('get_prox_left')['result']
            r       = send_command('get_prox_right')['result']
            pitch   = send_command('get_pitch')['result']
            yaw_now = send_command('get_yaw')['result']
            wd_now  = send_command('get_wheel_distance')['result']

            d_yaw  = wrap_delta(yaw_prev, yaw_now, 16)
            d_dist = wrap_delta(wd_prev,  wd_now,  20)  # wheel_distance is 20-bit
            heading += d_yaw * deg_per_yaw
            d_mm    = d_dist * mm_per_wd
            hr      = math.radians(heading)
            x += d_mm * math.cos(hr)
            y += d_mm * math.sin(hr)
            yaw_prev = yaw_now
            wd_prev  = wd_now
            path.append((x, y, heading))

            tilt = pitch - baseline_pitch
            print(f'  [{remaining:4.1f}s] ({x:6.0f},{y:6.0f})mm  hdg={heading:6.1f}°  '
                  f'prox L={l:2d} R={r:2d}  tilt={tilt:+.0f} streak={tilt_streak}',
                  end='\r')

            if l > PROX_THRESHOLD or r > PROX_THRESHOLD:
                walls.append((x + WALL_OFFSET_MM * math.cos(hr),
                              y + WALL_OFFSET_MM * math.sin(hr)))
                tilt_streak = 0
                recover('wall', WALL_SOUNDS)
                cooldown_until = time.time() + COOLDOWN

            elif now > cooldown_until:
                if abs(tilt) > PITCH_TILT_THRESHOLD:
                    tilt_streak += 1
                else:
                    tilt_streak = 0

                if tilt_streak >= TILT_CONFIRM_COUNT:
                    obstacles.append((x + OBSTACLE_OFFSET_MM * math.cos(hr),
                                      y + OBSTACLE_OFFSET_MM * math.sin(hr)))
                    tilt_streak = 0
                    recover(f'tilt {tilt:+.0f}', TILT_SOUNDS)
                    cooldown_until = time.time() + COOLDOWN
            else:
                tilt_streak = 0

            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print('\nInterrupted.')

    send_command('stop')
    send_command('say', 'bye')
    send_command('neck_color', '#ffffff')
    print(f'\nDone. Path={len(path)} pts  Walls={len(walls)}  Obstacles={len(obstacles)}')
    return path, walls, obstacles


# ---------------------------------------------------------------------------
# Persist
# ---------------------------------------------------------------------------
def save_map(deg_per_yaw, mm_per_wd, path, walls, obstacles):
    if MAP_FILE.exists():
        existing      = json.loads(MAP_FILE.read_text())
        all_runs      = existing.get('runs', [])
        all_walls     = existing.get('walls', [])
        all_obstacles = existing.get('obstacles', [])
    else:
        all_runs, all_walls, all_obstacles = [], [], []

    all_runs.append({
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        'path': path,
    })
    all_walls     += walls
    all_obstacles += obstacles

    MAP_FILE.write_text(json.dumps({
        'calibration': {'deg_per_yaw': deg_per_yaw, 'mm_per_wd': mm_per_wd},
        'runs':        all_runs,
        'walls':       all_walls,
        'obstacles':   all_obstacles,
    }))
    print(f'Map saved → {MAP_FILE}  '
          f'(run #{len(all_runs)}, '
          f'{len(all_walls)} total wall pts, '
          f'{len(all_obstacles)} total obstacle pts)')
    return all_runs, all_walls, all_obstacles


# ---------------------------------------------------------------------------
# Visualise
# ---------------------------------------------------------------------------
def visualise(all_runs, all_walls, all_obstacles):
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    COLORS = ['#4477cc', '#44aa77', '#cc7744', '#aa44aa', '#cc4444']

    fig, ax = plt.subplots(figsize=(11, 10))

    # Corner origin marker
    ax.plot(0, 0, 'k+', markersize=16, markeredgewidth=2, zorder=8, label='Corner (0,0)')

    for i, run in enumerate(all_runs):
        rpath = run['path']
        if not rpath:
            continue
        color = COLORS[i % len(COLORS)]
        px = [p[0] for p in rpath]
        py = [p[1] for p in rpath]
        label = f'Run {i+1} ({run["timestamp"][:10]})'
        ax.plot(px, py, '-', color=color, alpha=0.4, linewidth=1, label=label)
        ax.plot(px[0],  py[0],  'o', color=color, markersize=8,  zorder=6)
        ax.plot(px[-1], py[-1], 's', color=color, markersize=7,  zorder=6)
        step = max(1, len(rpath) // 15)
        for j in range(0, len(rpath) - step, step):
            dx = px[j+step] - px[j]
            dy = py[j+step] - py[j]
            if math.hypot(dx, dy) > 1:
                ax.annotate('', xy=(px[j+step], py[j+step]), xytext=(px[j], py[j]),
                            arrowprops=dict(arrowstyle='->', color=color, lw=1.0))

    if all_walls:
        wx = [w[0] for w in all_walls]
        wy = [w[1] for w in all_walls]
        ax.scatter(wx, wy, c='red', s=80, marker='x', linewidths=2,
                   label=f'Wall ({len(all_walls)} pts)', zorder=7)

    if all_obstacles:
        ox = [o[0] for o in all_obstacles]
        oy = [o[1] for o in all_obstacles]
        ax.scatter(ox, oy, c='orange', s=80, marker='^',
                   label=f'Obstacle ({len(all_obstacles)} pts)', zorder=7)

    # Draw the two dock walls as reference lines
    all_x = [p[0] for run in all_runs for p in run['path']] + [w[0] for w in all_walls] + [0]
    all_y = [p[1] for run in all_runs for p in run['path']] + [w[1] for w in all_walls] + [0]
    max_x = max(all_x) * 1.05 if all_x else 1000
    max_y = max(all_y) * 1.05 if all_y else 1000
    ax.plot([0, max_x], [0, 0],      'k-', linewidth=2, alpha=0.5, label='Dock walls')
    ax.plot([0, 0],     [0, max_y],  'k-', linewidth=2, alpha=0.5)

    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)
    ax.set_title(f'Room Map — {len(all_runs)} run(s), corner-docked', fontsize=14)
    ax.set_xlabel('x (mm)')
    ax.set_ylabel('y (mm)')
    plt.tight_layout()

    img_path = Path('room_map.png')
    plt.savefig(img_path, dpi=150, bbox_inches='tight')
    print(f'Map image saved → {img_path}')
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    deg_per_yaw, mm_per_wd = load_calibration()
    send_command('stop')
    time.sleep(1.0)
    x0, y0 = dock_to_corner(deg_per_yaw, mm_per_wd)
    path, walls, obstacles = explore(deg_per_yaw, mm_per_wd, x0, y0)
    all_runs, all_walls, all_obstacles = save_map(
        deg_per_yaw, mm_per_wd, path, walls, obstacles
    )
    visualise(all_runs, all_walls, all_obstacles)


if __name__ == "__main__":
    try:
        main()
    finally:
        try:
            send_command("stop")
        except Exception:
            pass
