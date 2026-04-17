#!/usr/bin/env python3
"""
RE-RASSOR Debug Monitor
-----------------------
Run this on the rover (after sourcing ROS2) to see Nav2, motor, odometry,
depth-camera, laser scan, and slam_toolbox status in real-time.

Usage:
    python3 debug_monitor.py [--rover-name <name>]

If --rover-name is omitted it tries to auto-detect from the running nodes.
Press Ctrl+C to exit.
"""

import argparse
import math
import os
import socket
import struct
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from tf2_msgs.msg import TFMessage
import tf2_ros

# ── ANSI colours ──────────────────────────────────────────────────────────────
RESET   = "\033[0m"
BOLD    = "\033[1m"
RED     = "\033[91m"
GREEN   = "\033[92m"
YELLOW  = "\033[93m"
CYAN    = "\033[96m"
MAGENTA = "\033[95m"
DIM     = "\033[2m"

ROUTINE_NAMES = {
    0:  "NONE",
    2:  "DIG",
    4:  "DUMP",
    8:  "EXTEND_FRONT",
    16: "EXTEND_BACK",
    32: "STOP",
}

NAV2_STATUS = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}

# Nodes that must be alive for the rover to navigate.
# Format: (display_name, ros_node_name)
REQUIRED_NODES = [
    ("motor_ctrl",    "serial_motor_controller"),
    ("mission_ctrl",  "mission_control"),
    ("slam_toolbox",  "slam_toolbox"),
    ("controller",    "controller_server"),
    ("planner",       "planner_server"),
    ("bt_navigator",  "bt_navigator"),
    ("behavior_srv",  "behavior_server"),
    ("depth_scan",    "depth_to_laserscan"),
    ("depth_pc",      "depth_to_pointcloud"),
]

# Serial ports we care about
SERIAL_PORTS = ["/dev/arduino_wheel", "/dev/arduino_drum"]

# Dead-band parameters (must match serial_motor_controller.cpp)
DEADBAND_LIN_MAX = 0.08   # |lin_x| < this AND lin_x < 0 → STOP
DEADBAND_ANG_MAX = 0.01   # |ang_z| < this (angular must also be small)

# How long (s) without an odom→base_link TF before we warn
TF_STALE_WARN_S = 0.5

# How long (s) without a slam_toolbox /map update before we warn
MAP_STALE_WARN_S = 30.0

# Full TF chain required for navigation + SLAM
REQUIRED_TF_CHAIN = [
    ("map",         "odom"),
    ("odom",        "base_link"),
    ("base_link",   "camera_link"),
    ("camera_link", "camera_depth_optical_frame"),
]


def ts() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def fmt_twist(t: Twist) -> str:
    return f"lin_x={t.linear.x:+.3f}  ang_z={t.angular.z:+.3f}"


def fmt_pos(p) -> str:
    return f"x={p.x:+.3f}  y={p.y:+.3f}"


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def is_dead_banded(lin_x: float, ang_z: float) -> bool:
    """Return True if serial_motor_controller will silently convert this to STOP."""
    return lin_x < 0.0 and abs(lin_x) < DEADBAND_LIN_MAX and abs(ang_z) < DEADBAND_ANG_MAX


class DebugMonitor(Node):
    def __init__(self, rover_name: str):
        super().__init__("re_rassor_debug_monitor")
        self._rover_name = rover_name
        self._last_cmd_vel    = None
        self._last_wheel      = None
        self._last_routine    = None
        self._last_nav2_states = {}

        # Odom pulse counters
        self._wheel_odom_count  = 0
        self._visual_odom_count = 0
        self._fused_odom_count  = 0

        # TF staleness tracking
        self._last_odom_base_tf_time: float = 0.0  # monotonic seconds

        # Track last known fused position
        self._last_fused_pos = None

        # Nav2 feedback throttle
        self._nav2_fb_print_interval = 2.0
        self._last_nav2_fb_print     = 0.0

        # Fused odom print throttle
        self._fused_print_interval = 5.0
        self._last_fused_print     = 0.0

        # ── Depth camera / slam_toolbox state ──────────────────────────────────
        # Depth image
        self._depth_img_count   = 0
        self._last_depth_time   = 0.0   # monotonic; 0 = never seen

        # PointCloud (for Nav2 costmaps)
        self._pc_count          = 0
        self._last_pc_time      = 0.0
        self._last_pc_n_points  = 0

        # Laser scan (/scan — input to slam_toolbox)
        self._scan_count        = 0
        self._last_scan_time    = 0.0
        self._scan_finite_count = 0     # scans with at least one finite range

        # slam_toolbox /map output
        self._slam_map_count    = 0
        self._last_map_time     = 0.0
        self._last_map_w        = 0
        self._last_map_h        = 0
        self._map_is_fake       = True   # assume fake until slam_toolbox pushes

        # TF buffer for full-chain checks
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        print(f"\n{BOLD}{'─'*60}{RESET}")
        print(f"{BOLD}  RE-RASSOR Debug Monitor{RESET}  (rover: {CYAN}{rover_name}{RESET})")
        print(f"{BOLD}{'─'*60}{RESET}")
        print(f"  {BOLD}Nav2 / motor:{RESET}")
        print(f"    {GREEN}/cmd_vel{RESET}               ← Nav2 velocity output")
        print(f"    {GREEN}/{rover_name}/wheel_instructions{RESET} ← motor controller")
        print(f"    {GREEN}/ezrassor/routine_actions{RESET} ← Stop/Dig/Dump signals")
        print(f"    {GREEN}/navigate_to_pose/...{RESET}  ← Nav2 action status + distance")
        print(f"  {BOLD}Odometry:{RESET}")
        print(f"    {GREEN}/odometry/wheel{RESET}        ← Wheel encoder odometry")
        print(f"    {GREEN}/odometry/fused{RESET}        ← Fused position (what Nav2 uses)")
        print(f"  {BOLD}Depth camera → slam_toolbox pipeline:{RESET}")
        print(f"    {GREEN}/camera/depth/image_raw{RESET} ← Astra Pro depth frames")
        print(f"    {GREEN}/camera/depth/points{RESET}   ← depth_image_proc PointCloud2 (Nav2 costmaps)")
        print(f"    {GREEN}/scan{RESET}                  ← depthimage_to_laserscan (slam_toolbox input)")
        print(f"    {GREEN}/map{RESET}                   ← slam_toolbox SLAM map (or fake_map bootstrap)")
        print(f"  {BOLD}TF chain:{RESET}  map→odom→base_link→camera_link→camera_depth_optical_frame")
        print(f"{BOLD}{'─'*60}{RESET}")
        print(f"  {DIM}Pulse counts + health printed every 10s — zero = dead{RESET}")
        print(f"  {DIM}Map stale >{MAP_STALE_WARN_S}s = slam_toolbox not building map{RESET}\n")

        # ── Subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(Twist, "/cmd_vel", self._cb_cmd_vel, 10)

        self.create_subscription(
            Twist, f"/{rover_name}/wheel_instructions", self._cb_wheel, 10)

        self.create_subscription(Int8, "/ezrassor/routine_actions", self._cb_routine, 10)

        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._cb_nav2_status, 10)

        self.create_subscription(
            NavigateToPose_FeedbackMessage,
            "/navigate_to_pose/_action/feedback",
            self._cb_nav2_feedback, 10)

        self.create_subscription(Odometry, "/odometry/wheel", self._cb_wheel_odom, 10)
        self.create_subscription(Odometry, "/odom",           self._cb_visual_odom, 10)
        self.create_subscription(Odometry, "/odometry/fused", self._cb_fused_odom,  10)

        # TF — use best-effort so we don't miss latched transforms
        tf_qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(TFMessage, "/tf", self._cb_tf, tf_qos)

        # ── Depth camera / slam_toolbox subscriptions ──────────────────────────
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

        self.create_subscription(Image,        "/camera/depth/image_raw", self._cb_depth_img,  sensor_qos)
        self.create_subscription(PointCloud2,  "/camera/depth/points",    self._cb_pointcloud,  sensor_qos)
        self.create_subscription(LaserScan,    "/scan",                   self._cb_laserscan,   sensor_qos)
        self.create_subscription(OccupancyGrid, "/map",                   self._cb_slam_map,    transient_qos)

        # ── Timers ─────────────────────────────────────────────────────────────
        self.create_timer(10.0, self._odom_health_report)
        self.create_timer(10.0, self._node_and_serial_health_report)
        self.create_timer(10.0, self._depth_slam_health_report)
        self.create_timer(10.0, self._tf_chain_health_report)

    # ── Velocity callbacks ─────────────────────────────────────────────────────

    def _cb_cmd_vel(self, msg: Twist):
        same = (
            self._last_cmd_vel is not None
            and abs(msg.linear.x  - self._last_cmd_vel.linear.x)  < 0.001
            and abs(msg.angular.z - self._last_cmd_vel.angular.z) < 0.001
        )
        self._last_cmd_vel = msg
        if same:
            return

        moving = abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001
        colour = RED if moving else GREEN
        label  = "MOVING" if moving else "STOPPED"

        # Warn if the motor controller dead-band will silently block this command
        db_warn = ""
        if is_dead_banded(msg.linear.x, msg.angular.z):
            db_warn = (
                f"  {YELLOW}{BOLD}[DEAD-BAND — motor ctrl will send STOP]{RESET}"
            )

        print(
            f"[{DIM}{ts()}{RESET}] {BOLD}/cmd_vel{RESET}       "
            f"{colour}{fmt_twist(msg)}{RESET}  [{colour}{label}{RESET}]{db_warn}"
        )

    def _cb_wheel(self, msg: Twist):
        same = (
            self._last_wheel is not None
            and abs(msg.linear.x  - self._last_wheel.linear.x)  < 0.001
            and abs(msg.angular.z - self._last_wheel.angular.z) < 0.001
        )
        self._last_wheel = msg
        if same:
            return

        colour = YELLOW if (abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001) else GREEN
        print(
            f"[{DIM}{ts()}{RESET}] {BOLD}wheel_instr{RESET}    "
            f"{colour}{fmt_twist(msg)}{RESET}"
        )

    def _cb_routine(self, msg: Int8):
        if self._last_routine == msg.data:
            return
        self._last_routine = msg.data
        name   = ROUTINE_NAMES.get(msg.data, f"UNKNOWN({msg.data})")
        colour = RED if msg.data == 32 else CYAN
        print(
            f"[{DIM}{ts()}{RESET}] {BOLD}routine_actions{RESET} "
            f"{colour}{name} (value={msg.data}){RESET}"
        )

    # ── Nav2 callbacks ─────────────────────────────────────────────────────────

    def _cb_nav2_status(self, msg: GoalStatusArray):
        for gs in msg.status_list:
            gid = bytes(gs.goal_info.goal_id.uuid).hex()[:8]
            if self._last_nav2_states.get(gid) == gs.status:
                continue
            self._last_nav2_states[gid] = gs.status

            status_str = NAV2_STATUS.get(gs.status, f"STATUS_{gs.status}")
            if gs.status in (4, 5, 6):
                colour = GREEN if gs.status == 4 else RED
            elif gs.status in (1, 2):
                colour = CYAN
            else:
                colour = YELLOW

            print(
                f"[{DIM}{ts()}{RESET}] {BOLD}Nav2 status{RESET}    "
                f"goal={gid}  {colour}{BOLD}{status_str}{RESET}"
            )

    def _cb_nav2_feedback(self, msg: NavigateToPose_FeedbackMessage):
        now = time.monotonic()
        if now - self._last_nav2_fb_print < self._nav2_fb_print_interval:
            return
        self._last_nav2_fb_print = now
        fb = msg.feedback
        dist = fb.distance_remaining
        colour = GREEN if dist < 0.3 else YELLOW
        print(
            f"[{DIM}{ts()}{RESET}] {DIM}Nav2 feedback  "
            f"distance_remaining={colour}{dist:.3f}m{RESET}"
        )

    # ── Odometry callbacks ─────────────────────────────────────────────────────

    def _cb_wheel_odom(self, msg: Odometry):
        self._wheel_odom_count += 1

    def _cb_visual_odom(self, msg: Odometry):
        self._visual_odom_count += 1

    def _cb_fused_odom(self, msg: Odometry):
        self._fused_odom_count += 1
        now = time.monotonic()
        if now - self._last_fused_print < self._fused_print_interval:
            return
        self._last_fused_print = now

        p   = msg.pose.pose.position
        yaw = math.degrees(yaw_from_quat(msg.pose.pose.orientation))

        frozen = (
            self._last_fused_pos is not None
            and abs(p.x - self._last_fused_pos[0]) < 0.005
            and abs(p.y - self._last_fused_pos[1]) < 0.005
        )
        self._last_fused_pos = (p.x, p.y)

        frozen_tag = f"  {RED}[FROZEN — odom not updating!]{RESET}" if frozen else ""
        print(
            f"[{DIM}{ts()}{RESET}] {DIM}odom/fused     "
            f"{fmt_pos(p)}  yaw={yaw:+.1f}°{RESET}{frozen_tag}"
        )

    # ── TF callback ───────────────────────────────────────────────────────────

    def _cb_tf(self, msg: TFMessage):
        for tf in msg.transforms:
            if tf.header.frame_id == "odom" and tf.child_frame_id == "base_link":
                self._last_odom_base_tf_time = time.monotonic()
                break

    # ── Periodic health reports ────────────────────────────────────────────────

    def _odom_health_report(self):
        def status(count):
            if count == 0:
                return f"{RED}DEAD (0 msgs){RESET}"
            elif count < 5:
                return f"{YELLOW}SLOW ({count} msgs){RESET}"
            else:
                return f"{GREEN}OK ({count} msgs){RESET}"

        # TF staleness
        tf_age = time.monotonic() - self._last_odom_base_tf_time
        if self._last_odom_base_tf_time == 0.0:
            tf_str = f"{RED}NEVER SEEN{RESET}"
        elif tf_age > TF_STALE_WARN_S:
            tf_str = f"{RED}STALE ({tf_age:.1f}s ago){RESET}"
        else:
            tf_str = f"{GREEN}OK ({tf_age:.2f}s ago){RESET}"

        print(
            f"[{DIM}{ts()}{RESET}] {BOLD}Odom health (last 10s){RESET}  "
            f"wheel={status(self._wheel_odom_count)}  "
            f"visual/odom={status(self._visual_odom_count)}  "
            f"fused={status(self._fused_odom_count)}  "
            f"odom→base_link TF={tf_str}"
        )
        self._wheel_odom_count  = 0
        self._visual_odom_count = 0
        self._fused_odom_count  = 0

    def _node_and_serial_health_report(self):
        # Node health — check the ROS graph
        node_names = [n for n, _ in self.get_node_names_and_namespaces()]

        node_parts = []
        for display, ros_name in REQUIRED_NODES:
            alive = ros_name in node_names
            colour = GREEN if alive else RED
            mark   = "✓" if alive else "✗"
            node_parts.append(f"{colour}{mark}{display}{RESET}")

        # Serial port health — check device file existence
        serial_parts = []
        for port in SERIAL_PORTS:
            short = port.split("/")[-1]
            if os.path.exists(port):
                serial_parts.append(f"{GREEN}✓{short}{RESET}")
            else:
                serial_parts.append(f"{RED}✗{short} MISSING{RESET}")

        print(
            f"[{DIM}{ts()}{RESET}] {BOLD}Node health{RESET}    "
            + "  ".join(node_parts)
        )
        print(
            f"[{DIM}{ts()}{RESET}] {BOLD}Serial ports{RESET}   "
            + "  ".join(serial_parts)
        )


    # ── Depth camera callbacks ─────────────────────────────────────────────────

    def _cb_depth_img(self, msg: Image):
        self._depth_img_count += 1
        self._last_depth_time = time.monotonic()

    def _cb_pointcloud(self, msg: PointCloud2):
        self._pc_count += 1
        self._last_pc_time = time.monotonic()
        self._last_pc_n_points = len(bytes(msg.data)) // max(msg.point_step, 1)

    # ── Laser scan callback ────────────────────────────────────────────────────

    def _cb_laserscan(self, msg: LaserScan):
        self._scan_count += 1
        self._last_scan_time = time.monotonic()
        finite = sum(1 for r in msg.ranges if not (r == float('inf') or r != r))
        if finite > 0:
            self._scan_finite_count += 1

    # ── slam_toolbox /map callback ─────────────────────────────────────────────

    def _cb_slam_map(self, msg: OccupancyGrid):
        self._slam_map_count += 1
        now = time.monotonic()
        has_content = any(c != 0 for c in msg.data)
        self._map_is_fake = not has_content

        w, h = msg.info.width, msg.info.height
        if w != self._last_map_w or h != self._last_map_h:
            known_cells = sum(1 for c in msg.data if c >= 0)
            src = "slam_toolbox" if has_content else "fake_map"
            print(
                f"[{DIM}{ts()}{RESET}] {CYAN}{BOLD}/map updated ({src}){RESET}  "
                f"{w}×{h} cells  res={msg.info.resolution:.3f}m  "
                f"known={known_cells}/{w*h}  frame={msg.header.frame_id}"
            )
            self._last_map_w = w
            self._last_map_h = h

        self._last_map_time = now

    # ── Depth / slam_toolbox health report (every 10 s) ───────────────────────

    def _depth_slam_health_report(self):
        now = time.monotonic()

        def rate_status(count):
            if count == 0:
                return f"{RED}DEAD  (0 msgs/10s){RESET}"
            elif count < 3:
                return f"{YELLOW}SLOW  ({count} msgs/10s){RESET}"
            else:
                return f"{GREEN}OK    ({count} msgs/10s){RESET}"

        # Depth image
        depth_age = now - self._last_depth_time if self._last_depth_time else None
        depth_str = rate_status(self._depth_img_count)
        if depth_age and depth_age > 2.0:
            depth_str += f"  {RED}STALE {depth_age:.1f}s{RESET}"

        # PointCloud (Nav2 costmaps)
        pc_str = rate_status(self._pc_count)
        if self._pc_count > 0:
            pc_str += f"  {DIM}last={self._last_pc_n_points} pts{RESET}"

        # Laser scan (slam_toolbox input)
        scan_str = rate_status(self._scan_count)
        if self._scan_count > 0 and self._scan_finite_count == 0:
            scan_str += f"  {RED}all ranges inf — camera sees nothing in range{RESET}"
        elif self._scan_count > 0:
            scan_str += f"  {DIM}{self._scan_finite_count}/{self._scan_count} with valid ranges{RESET}"

        # slam_toolbox /map
        if self._slam_map_count == 0:
            map_str = f"{RED}DEAD  (no /map yet — fake_map or slam_toolbox not started){RESET}"
        else:
            map_age = now - self._last_map_time
            if map_age > MAP_STALE_WARN_S:
                map_str = (
                    f"{YELLOW}STALE ({map_age:.0f}s since last update)  "
                    f"{self._last_map_w}×{self._last_map_h}{RESET}"
                )
            else:
                src = f"{YELLOW}fake_map{RESET}" if self._map_is_fake else f"{GREEN}slam_toolbox{RESET}"
                map_str = (
                    f"{GREEN}OK    ({self._slam_map_count} updates){RESET}  source={src}  "
                    f"{self._last_map_w}×{self._last_map_h}"
                )

        print(
            f"[{DIM}{ts()}{RESET}] {BOLD}Camera health (last 10s){RESET}  "
            f"depth_img={depth_str}  points={pc_str}"
        )
        print(
            f"[{DIM}{ts()}{RESET}] {BOLD}SLAM health (last 10s){RESET}  "
            f"scan={scan_str}  map={map_str}"
        )

        # Reset interval counters
        self._depth_img_count   = 0
        self._pc_count          = 0
        self._scan_count        = 0
        self._scan_finite_count = 0
        self._slam_map_count    = 0

    # ── TF chain health report (every 10 s) ────────────────────────────────────

    def _tf_chain_health_report(self):
        parts = []
        for parent, child in REQUIRED_TF_CHAIN:
            try:
                tf = self._tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1))
                t = tf.transform.translation
                parts.append(
                    f"{GREEN}✓{parent}→{child}{RESET}"
                    f"{DIM}({t.x:+.2f},{t.y:+.2f},{t.z:+.2f}){RESET}"
                )
            except Exception:
                parts.append(f"{RED}✗{parent}→{child} MISSING{RESET}")

        print(
            f"[{DIM}{ts()}{RESET}] {BOLD}TF chain{RESET}    "
            + "  ".join(parts)
        )


def auto_detect_rover_name() -> str:
    try:
        hostname = socket.gethostname()
        ip = socket.gethostbyname(hostname)
        return "ip_" + ip.replace(".", "_")
    except Exception:
        return "re_rassor"


def main():
    parser = argparse.ArgumentParser(description="RE-RASSOR debug topic monitor")
    parser.add_argument(
        "--rover-name",
        default=None,
        help="Rover namespace (e.g. ip_192_168_1_10). Auto-detected if omitted.",
    )
    args = parser.parse_args()

    rover_name = args.rover_name or auto_detect_rover_name()

    rclpy.init()
    node = DebugMonitor(rover_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{DIM}[monitor] shutting down{RESET}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
