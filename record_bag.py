#!/usr/bin/env python3
"""
RE-RASSOR Bag Recorder
----------------------
Records all critical pipeline topics to a timestamped bag file.
Run alongside the rover stack when investigating failures.

Usage:
    python3 record_bag.py                  # records to ./bags/YYYY-MM-DD_HH-MM-SS/
    python3 record_bag.py --out /tmp/bags  # custom output directory
    python3 record_bag.py --no-depth       # skip raw depth images (saves space)

Press Ctrl-C to stop.  The bag is usable immediately after stopping.

Replay later:
    ros2 bag play bags/<bag_name>
    ros2 bag info bags/<bag_name>
"""

import argparse
import os
import signal
import subprocess
import sys
from datetime import datetime

# ── Topics to record ──────────────────────────────────────────────────────────

# Always recorded — lightweight, essential for diagnosing every failure mode
CORE_TOPICS = [
    # Odometry pipeline
    "/rtabmap/odom",           # ICP odometry raw
    "/odom",                   # ICP odom post-relay
    "/odometry/wheel",         # Wheel encoder odom
    "/odometry/fused",         # What Nav2 actually uses

    # Map pipeline
    "/rtabmap/map",            # SLAM map raw
    "/map",                    # Map post-relay (or fake_map)

    # Navigation
    "/cmd_vel",                # Nav2 velocity commands
    "/navigate_to_pose/_action/status",
    "/navigate_to_pose/_action/feedback",

    # TF (essential for replaying any ROS bag)
    "/tf",
    "/tf_static",

    # Camera info (tiny, needed to reconstruct pointclouds)
    "/camera/depth/camera_info",

    # PointCloud (moderate size — key input to ICP)
    "/camera/depth/points",

    # Node diagnostics
    "/rosout",
]

# Recorded unless --no-depth is passed (raw depth images are large)
DEPTH_TOPICS = [
    "/camera/depth/image_raw",
]

# ── ANSI ─────────────────────────────────────────────────────────────────────
BOLD  = "\033[1m"
GREEN = "\033[92m"
CYAN  = "\033[96m"
DIM   = "\033[2m"
RESET = "\033[0m"


def main():
    parser = argparse.ArgumentParser(description="RE-RASSOR bag recorder")
    parser.add_argument("--out",      default="bags",
                        help="Parent directory for bag files (default: ./bags)")
    parser.add_argument("--no-depth", action="store_true",
                        help="Skip /camera/depth/image_raw (reduces bag size ~70%%)")
    args = parser.parse_args()

    stamp    = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    bag_path = os.path.join(args.out, stamp)
    os.makedirs(args.out, exist_ok=True)

    topics = CORE_TOPICS[:]
    if not args.no_depth:
        topics += DEPTH_TOPICS

    print(f"\n{BOLD}RE-RASSOR Bag Recorder{RESET}")
    print(f"  Output : {CYAN}{bag_path}{RESET}")
    print(f"  Topics : {len(topics)} ({GREEN}{'w/' if not args.no_depth else 'w/o'} raw depth{RESET})")
    print(f"  Stop   : {DIM}Ctrl-C{RESET}\n")
    for t in sorted(topics):
        print(f"    {DIM}{t}{RESET}")
    print()

    cmd = [
        "ros2", "bag", "record",
        "--output", bag_path,
        "--storage", "sqlite3",
    ] + topics

    try:
        proc = subprocess.run(cmd)
    except KeyboardInterrupt:
        pass

    # ros2 bag record exits cleanly on SIGINT — nothing extra needed
    print(f"\n{BOLD}Bag saved:{RESET}  {CYAN}{bag_path}{RESET}")
    print(f"  Replay : {DIM}ros2 bag play {bag_path}{RESET}")
    print(f"  Info   : {DIM}ros2 bag info {bag_path}{RESET}\n")


if __name__ == "__main__":
    main()
