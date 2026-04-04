#!/usr/bin/env python3
"""
RE-RASSOR Preflight Check
-------------------------
Run before a mission.  Subscribes to all critical topics for COLLECT_SECS,
then prints a GO / NO-GO checklist and exits.

Usage:
    python3 preflight.py            # default 12-second collection window
    python3 preflight.py --secs 20  # longer window for slow hardware

Exit code: 0 = GO, 1 = NO-GO (at least one hard failure).
"""

import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
)
import tf2_ros

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from tf2_msgs.msg import TFMessage

# ── Thresholds ────────────────────────────────────────────────────────────────
DEPTH_MIN_HZ    = 5.0    # depth image must publish at least this fast
PC_MIN_HZ       = 5.0    # pointcloud must publish at least this fast
SCAN_MIN_HZ     = 5.0    # laser scan must publish at least this fast

# Startup grace: if system was just launched, rates will be low.
# Default collection window is 20 s to give nodes time to warm up.
DEFAULT_SECS    = 20.0
PC_MIN_POINTS   = 1000   # fewer points than this = sparse cloud (Nav2 costmap warning)

SERIAL_PORTS = ["/dev/arduino_wheel", "/dev/arduino_drum"]

REQUIRED_NODES = [
    ("motor_ctrl",    "serial_motor_controller"),
    ("mission_ctrl",  "mission_control"),
    ("slam_toolbox",  "slam_toolbox"),
    ("controller",    "controller_server"),
    ("planner",       "planner_server"),
    ("bt_navigator",  "bt_navigator"),
]

OPTIONAL_NODES = [
    ("depth_scan",    "depth_to_laserscan"),
    ("depth_pc",      "depth_to_pointcloud"),
]

TF_CHAIN = [
    ("odom",        "base_link"),
    ("base_link",   "camera_link"),
    ("camera_link", "camera_depth_optical_frame"),
]

# ── ANSI ─────────────────────────────────────────────────────────────────────
RESET  = "\033[0m"
BOLD   = "\033[1m"
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
DIM    = "\033[2m"


def _ok(msg):    return f"{GREEN}[  OK  ]{RESET}  {msg}"
def _warn(msg):  return f"{YELLOW}[ WARN ]{RESET}  {msg}"
def _fail(msg):  return f"{RED}[ FAIL ]{RESET}  {msg}"
def _skip(msg):  return f"{DIM}[ SKIP ]{RESET}  {msg}"


# ── Data-collection node ──────────────────────────────────────────────────────

class _PreflightNode(Node):
    def __init__(self):
        super().__init__("re_rassor_preflight")

        self.depth_count    = 0
        self.pc_count       = 0
        self.pc_points_max  = 0
        self.scan_count     = 0
        self.scan_ranges_ok = 0      # scans with at least some finite ranges
        self.map_received   = False
        self.map_is_blank   = True   # all-zero = still the fake_map
        self.odom_received  = False
        self.wheel_odom_received = False

        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        transient_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Image,         "/camera/depth/image_raw", self._cb_depth,      sensor_qos)
        self.create_subscription(PointCloud2,  "/camera/depth/points",    self._cb_pc,         sensor_qos)
        self.create_subscription(LaserScan,    "/scan",                   self._cb_scan,       sensor_qos)
        self.create_subscription(Odometry,     "/odom",                   self._cb_odom,       10)
        self.create_subscription(Odometry,     "/odometry/wheel",         self._cb_wheel_odom, 10)
        self.create_subscription(OccupancyGrid, "/map",                   self._cb_map,        transient_qos)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _cb_depth(self, _):
        self.depth_count += 1

    def _cb_pc(self, msg: PointCloud2):
        self.pc_count += 1
        n = len(bytes(msg.data)) // max(msg.point_step, 1)
        self.pc_points_max = max(self.pc_points_max, n)

    def _cb_scan(self, msg: LaserScan):
        self.scan_count += 1
        finite = sum(1 for r in msg.ranges if not (r == float('inf') or r != r))
        if finite > 0:
            self.scan_ranges_ok += 1

    def _cb_odom(self, _):
        self.odom_received = True

    def _cb_wheel_odom(self, _):
        self.wheel_odom_received = True

    def _cb_map(self, msg: OccupancyGrid):
        self.map_received = True
        self.map_is_blank = all(c == 0 for c in msg.data)

    def lookup_tf(self, parent, child):
        try:
            return self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception:
            return None


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="RE-RASSOR preflight check")
    parser.add_argument("--secs", type=float, default=DEFAULT_SECS,
                        help=f"Seconds to collect data (default {DEFAULT_SECS:.0f})")
    args = parser.parse_args()

    rclpy.init()
    node = _PreflightNode()

    print(f"\n{BOLD}{'═'*58}{RESET}")
    print(f"{BOLD}  RE-RASSOR Preflight Check{RESET}")
    print(f"{BOLD}{'═'*58}{RESET}")
    print(f"  Collecting data for {args.secs:.0f} s …  (Ctrl-C to abort)\n")

    # Spin and show a simple countdown
    t0 = time.monotonic()
    interval = 1.0
    next_tick = t0 + interval
    while time.monotonic() - t0 < args.secs:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if now >= next_tick:
            elapsed = now - t0
            remaining = args.secs - elapsed
            print(f"  {DIM}{remaining:.0f}s remaining …{RESET}", end="\r", flush=True)
            next_tick += interval

    print(f"  {DIM}Collection done.{RESET}                    \n")

    elapsed  = time.monotonic() - t0
    results  = []   # list of (label, line, is_hard_fail)
    failures = 0
    warns    = 0

    # ── 1. Serial ports ───────────────────────────────────────────────────────
    for port in SERIAL_PORTS:
        name = port.split("/")[-1]
        if os.path.exists(port):
            results.append((_ok(f"Serial port {port}"), False))
        else:
            results.append((_fail(f"Serial port {port} not found — Arduino not connected?"), True))
            failures += 1

    # ── 2. Depth camera ───────────────────────────────────────────────────────
    depth_hz = node.depth_count / elapsed
    if depth_hz >= DEPTH_MIN_HZ:
        results.append((_ok(f"Depth image  {depth_hz:.1f} Hz"), False))
    elif node.depth_count > 0:
        results.append((_warn(f"Depth image  {depth_hz:.1f} Hz < {DEPTH_MIN_HZ} Hz — camera slow, USB bandwidth limited, or still warming up"), False))
        warns += 1
    else:
        results.append((_fail("Depth image  0 Hz — camera not publishing.  Check USB connection and astra_camera node"), True))
        failures += 1

    # ── 3. PointCloud ─────────────────────────────────────────────────────────
    pc_hz = node.pc_count / elapsed
    if pc_hz >= PC_MIN_HZ and node.pc_points_max >= PC_MIN_POINTS:
        results.append((_ok(f"PointCloud   {pc_hz:.1f} Hz  max={node.pc_points_max} pts"), False))
    elif node.pc_count == 0:
        results.append((_fail("PointCloud   0 Hz — depth_image_proc not running or not receiving depth"), True))
        failures += 1
    elif node.pc_points_max < PC_MIN_POINTS:
        results.append((_warn(f"PointCloud   {pc_hz:.1f} Hz but only {node.pc_points_max} pts — ICP may fail with sparse cloud"), False))
        warns += 1
    else:
        results.append((_warn(f"PointCloud   {pc_hz:.1f} Hz < {PC_MIN_HZ} Hz — still warming up, re-run preflight after 30 s if system just started"), False))
        warns += 1

    # ── 4. Laser scan (/scan from depthimage_to_laserscan) ───────────────────
    scan_hz = node.scan_count / elapsed
    if node.scan_count == 0:
        results.append((_fail("/scan        0 Hz — depthimage_to_laserscan not running or not receiving depth"), True))
        failures += 1
    elif scan_hz < SCAN_MIN_HZ:
        results.append((_warn(f"/scan        {scan_hz:.1f} Hz < {SCAN_MIN_HZ} Hz — still warming up"), False))
        warns += 1
    elif node.scan_ranges_ok == 0:
        results.append((_fail(f"/scan        {scan_hz:.1f} Hz but all ranges are inf — camera not seeing anything in range"), True))
        failures += 1
    else:
        results.append((_ok(f"/scan        {scan_hz:.1f} Hz  ({node.scan_ranges_ok}/{node.scan_count} scans with valid ranges)"), False))

    # ── 5. Wheel odometry (check before TF — explains odom→base_link if missing) ──
    if node.wheel_odom_received:
        results.append((_ok("/odometry/wheel  receiving (Arduino connected)"), False))
    else:
        results.append((_fail("/odometry/wheel  nothing received — Arduino not sending.  "
                              "Check USB cable and serial_motor_controller node"), True))
        failures += 1

    # ── 6. TF chain ───────────────────────────────────────────────────────────
    # map→odom      : published by slam_toolbox (localization)
    # odom→base_link: published by mission_control (wheel odometry)
    # base_link→camera_link and camera_link→camera_depth_optical_frame:
    #   static TFs published by re_rassor_full.launch.py at startup.
    TF_CHAIN = [
        ("map",         "odom"),
        ("odom",        "base_link"),
        ("base_link",   "camera_link"),
        ("camera_link", "camera_depth_optical_frame"),
    ]
    TF_SOURCES = {
        ("map",         "odom"):                           "slam_toolbox (needs /scan + odom TF)",
        ("odom",        "base_link"):                      "mission_control (needs wheel odom)",
        ("base_link",   "camera_link"):                    "static_transform_publisher in launch",
        ("camera_link", "camera_depth_optical_frame"):     "static_transform_publisher in launch",
    }
    for parent, child in TF_CHAIN:
        tf = node.lookup_tf(parent, child)
        if tf is not None:
            t = tf.transform.translation
            results.append((_ok(f"TF {parent}→{child}  ({t.x:+.2f},{t.y:+.2f},{t.z:+.2f})"), False))
        else:
            source = TF_SOURCES.get((parent, child), "unknown")
            hint = ""
            if parent == "map" and node.scan_count == 0:
                hint = " (root cause: /scan not publishing — fix depthimage_to_laserscan first)"
            elif parent == "odom" and not node.wheel_odom_received:
                hint = " (root cause: wheel odom dead — fix /odometry/wheel first)"
            results.append((_fail(f"TF {parent}→{child}  MISSING — published by {source}{hint}"), True))
            failures += 1

    # ── 7. /map ───────────────────────────────────────────────────────────────
    if not node.map_received:
        results.append((_fail("/map         not received — fake_map.py not running and slam_toolbox has no map yet"), True))
        failures += 1
    elif node.map_is_blank:
        results.append((_warn("/map         received but all-free — still on fake_map, slam_toolbox has not built a real map yet"), False))
        warns += 1
    else:
        results.append((_ok("/map         received with SLAM content (slam_toolbox active)"), False))

    # ── 8. Required nodes ─────────────────────────────────────────────────────
    node_names = [n for n, _ in node.get_node_names_and_namespaces()]
    for label, ros_name in REQUIRED_NODES:
        if ros_name in node_names:
            results.append((_ok(f"Node {label:<14} {ros_name}"), False))
        else:
            results.append((_fail(f"Node {label:<14} {ros_name} not found in node list"), True))
            failures += 1

    for label, ros_name in OPTIONAL_NODES:
        if ros_name in node_names:
            results.append((_ok(f"Node {label:<14} {ros_name}  (optional)"), False))
        else:
            results.append((_warn(f"Node {label:<14} {ros_name} not found — relay may be missing"), False))
            warns += 1

    # ── Print results ─────────────────────────────────────────────────────────
    print(f"  {BOLD}Results:{RESET}")
    print(f"  {'─'*54}")
    for line, _ in results:
        print(f"  {line}")
    print(f"  {'─'*54}")

    # ── Verdict ───────────────────────────────────────────────────────────────
    print()
    if failures == 0 and warns == 0:
        print(f"  {GREEN}{BOLD}{'▶  GO — all systems nominal':^54}{RESET}")
    elif failures == 0:
        print(f"  {YELLOW}{BOLD}{'▶  GO (with warnings) — check WARN items above':^54}{RESET}")
    else:
        print(f"  {RED}{BOLD}{'✖  NO-GO — fix FAIL items before the mission':^54}{RESET}")
    print(f"\n  {DIM}{failures} failure(s)   {warns} warning(s){RESET}")
    print(f"{BOLD}{'═'*58}{RESET}\n")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if failures == 0 else 1)


if __name__ == "__main__":
    main()
