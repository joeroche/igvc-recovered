#!/usr/bin/env python3

"""
RPLidar A1 - Standalone Terminal Test
--------------------------------------
Prints raw scan data to the terminal without requiring ROS2.
Useful for verifying the sensor is working before running the full driver.

Usage:
    python3 rplidar_test.py
    python3 rplidar_test.py --port /dev/ttyACM0
    python3 rplidar_test.py --port //dev/ttyACM0-- scans 5
"""

import argparse
import math
import time

from rplidar import RPLidar, RPLidarException


# ── ANSI colors for terminal output ──────────────────────────────────────────
GREEN  = '\033[92m'
YELLOW = '\033[93m'
RED    = '\033[91m'
CYAN   = '\033[96m'
RESET  = '\033[0m'
BOLD   = '\033[1m'


def direction_label(angle_deg: float) -> str:
    """Convert an angle in degrees to a compass direction label."""
    dirs = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW', 'N']
    idx  = int((angle_deg + 22.5) / 45.0) % 8
    return dirs[idx]


def print_scan_summary(scan_number: int, measurements: list):
    """Print a concise summary of one full 360° scan."""
    valid = [(q, a, d / 1000.0) for q, a, d in measurements
             if q > 0 and d > 0]

    if not valid:
        print(f"{RED}Scan {scan_number}: no valid measurements.{RESET}")
        return

    distances   = [d for _, _, d in valid]
    min_dist    = min(distances)
    max_dist    = max(distances)
    avg_dist    = sum(distances) / len(distances)
    min_reading = min(valid, key=lambda x: x[2])


    
    max_reading = max(valid, key=lambda x: x[2])

    print(f"\n{BOLD}{CYAN}{'─' * 55}{RESET}")
    print(f"{BOLD}Scan #{scan_number}{RESET}  |  {len(valid)} valid points  "
          f"(of {len(measurements)} total)")
    print(f"{CYAN}{'─' * 55}{RESET}")
    print(f"  {'Closest:':<12} {GREEN}{min_dist:.3f} m{RESET}  "
          f"@ {min_reading[1]:.1f}°  ({direction_label(min_reading[1])})")
    print(f"  {'Farthest:':<12} {YELLOW}{max_dist:.3f} m{RESET}  "
          f"@ {max_reading[1]:.1f}°  ({direction_label(max_reading[1])})")
    print(f"  {'Average:':<12} {avg_dist:.3f} m")


def print_scan_detail(scan_number: int, measurements: list, max_rows: int = 20):
    """Print individual measurements — capped to keep output readable."""
    valid = [(q, a, d / 1000.0) for q, a, d in measurements if q > 0 and d > 0]
    valid.sort(key=lambda x: x[1])  # sort by angle

    print(f"\n{BOLD}Scan #{scan_number} — per-point detail "
          f"(showing up to {max_rows} of {len(valid)} points){RESET}")
    print(f"  {'Angle (°)':<12} {'Distance (m)':<16} {'Dir':<6} {'Quality'}")
    print(f"  {'─' * 45}")

    step = max(1, len(valid) // max_rows)
    for q, angle, dist in valid[::step]:
        bar   = '█' * min(int(dist * 3), 20)
        color = GREEN if dist < 1.0 else YELLOW if dist < 5.0 else RESET
        print(f"  {angle:<12.1f} {color}{dist:<16.3f}{RESET} "
              f"{direction_label(angle):<6} {q}  {color}{bar}{RESET}")


def run_test(port: str, baudrate: int, num_scans: int, detail: bool):
    print(f"\n{BOLD}RPLidar A1 — Terminal Test{RESET}")
    print(f"Port: {port}  |  Baudrate: {baudrate}")
    print(f"Collecting {num_scans} scan(s) …\n")

    lidar = RPLidar(port, baudrate=baudrate)

    try:
        # ── Device info ───────────────────────────────────────────────────
        info   = lidar.get_info()
        health = lidar.get_health()

        print(f"{BOLD}Device Info{RESET}")
        for k, v in info.items():
            print(f"  {k:<20} {v}")

        status_color = GREEN if health[0] == 'Good' else YELLOW if health[0] == 'Warning' else RED
        print(f"\n{BOLD}Health{RESET}")
        print(f"  Status: {status_color}{health[0]}{RESET}  |  Error code: {health[1]}")

        lidar.start_motor()
        print(f"\n{YELLOW}Motor started — warming up …{RESET}")
        time.sleep(1.0)  # brief warm-up so first scan is clean

        # ── Scan loop ─────────────────────────────────────────────────────
        collected = 0
        for scan in lidar.iter_scans(max_buf_meas=500):
            collected += 1
            print_scan_summary(collected, scan)
            if detail:
                print_scan_detail(collected, scan)
            if collected >= num_scans:
                break

        print(f"\n{GREEN}Done — {collected} scan(s) collected.{RESET}\n")

    except RPLidarException as exc:
        print(f"\n{RED}RPLidar error: {exc}{RESET}")
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Interrupted by user.{RESET}")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("Lidar disconnected.")


# ─────────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='RPLidar A1 terminal test.')
    parser.add_argument('--port',     default='/dev/ttyACM0',
                        help='Serial port (default: /dev/ttyACM0')
    parser.add_argument('--baudrate', default=115200, type=int,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--scans',    default=3, type=int,
                        help='Number of full scans to collect (default: 3)')
    parser.add_argument('--detail',   action='store_true',
                        help='Print per-point breakdown for each scan')
    args = parser.parse_args()

    run_test(args.port, args.baudrate, args.scans, args.detail)