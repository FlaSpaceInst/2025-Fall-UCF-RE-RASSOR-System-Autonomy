#!/usr/bin/env python3
"""
RE-RASSOR Debug Monitor
-----------------------
Run this on the rover (after sourcing ROS2) to see Nav2, motor, odometry,
depth-camera, ICP odometry, and RTAB-Map status in real-time.

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
from sensor_msgs.msg import Image, PointCloud2
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
    ("controller",    "controller_server"),
    ("planner",       "planner_server"),
    ("bt_navigator",  "bt_navigator"),
    ("behavior_srv",  "behavior_server"),
    ("icp_odom",      "icp_odometry"),
    ("rtabmap",       "rtabmap"),
    ("odom_relay",    "odom_relay"),
    ("map_relay",     "map_relay"),
]

# Serial ports we care about
SERIAL_PORTS = ["/dev/arduino_wheel", "/dev/arduino_drum"]

# Dead-band parameters (must match serial_motor_controller.cpp)
DEADBAND_LIN_MAX = 0.08   # |lin_x| < this AND lin_x < 0 → STOP
DEADBAND_ANG_MAX = 0.01   # |ang_z| < this (angular must also be small)

# How long (s) without an odom→base_link TF before we warn
TF_STALE_WARN_S = 0.5

# ICP covariance threshold — diagonal value above this means ICP lost tracking
ICP_COV_WARN = 0.5

# How long (s) without a /rtabmap/map update before we warn
MAP_STALE_WARN_S = 30.0

# TF chain that must exist for the depth pipeline to work
REQUIRED_TF_CHAIN = [
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

        # ── Depth camera / RTAB-Map state ──────────────────────────────────────
        # Depth image
        self._depth_img_count   = 0
        self._last_depth_time   = 0.0   # monotonic; 0 = never seen

        # PointCloud
        self._pc_count          = 0
        self._last_pc_time      = 0.0
        self._last_pc_n_points  = 0

        # ICP odometry (/rtabmap/odom)
        self._icp_odom_count    = 0
        self._last_icp_cov_max  = 0.0   # largest diagonal of pose covariance
        self._icp_lost_tracking = False  # True when covariance explodes

        # ICP reset detection — mirrors mission_control.cpp near_reset logic
        self._last_icp_pos      = None  # (x, y) of last accepted ICP message
        self._icp_reset_count   = 0     # how many resets detected this session

        # RTAB-Map SLAM (/rtabmap/map)
        self._rtabmap_map_count  = 0
        self._last_map_time      = 0.0
        self._last_map_w         = 0
        self._last_map_h         = 0

        # /map relay output — track whether it's the fake map or a real one
        self._relay_map_count    = 0
        self._map_is_fake        = True   # assume fake until rtabmap pushes

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
        print(f"    {GREEN}/rtabmap/odom{RESET}          ← ICP odometry (raw, pre-relay)")
        print(f"    {GREEN}/odom{RESET}                  ← ICP odometry (post-relay)")
        print(f"    {GREEN}/odometry/fused{RESET}        ← Fused position (what Nav2 uses)")
        print(f"  {BOLD}Depth camera → RTAB-Map pipeline:{RESET}")
        print(f"    {GREEN}/camera/depth/image_raw{RESET} ← Astra Pro depth frames")
        print(f"    {GREEN}/camera/depth/points{RESET}   ← depth_image_proc PointCloud2")
        print(f"    {GREEN}/rtabmap/map{RESET}           ← SLAM map (pre-relay)")
        print(f"    {GREEN}/map{RESET}                   ← SLAM map (post-relay / fake_map)")
        print(f"  {BOLD}TF chain:{RESET}  odom→base_link→camera_link→camera_depth_optical_frame")
        print(f"{BOLD}{'─'*60}{RESET}")
        print(f"  {DIM}Pulse counts + health printed every 10s — zero = dead{RESET}")
        print(f"  {DIM}ICP covariance spike = lost tracking.  Map stale >{MAP_STALE_WARN_S}s = SLAM stuck{RESET}\n")

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

        # ── Depth camera / RTAB-Map subscriptions ──────────────────────────────
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

        self.create_subscription(Image,        "/camera/depth/image_raw", self._cb_depth_img, sensor_qos)
        self.create_subscription(PointCloud2,  "/camera/depth/points",    self._cb_pointcloud, sensor_qos)
        self.create_subscription(Odometry,     "/rtabmap/odom",           self._cb_icp_odom,   10)
        self.create_subscription(OccupancyGrid, "/rtabmap/map",           self._cb_rtabmap_map, transient_qos)
        self.create_subscription(OccupancyGrid, "/map",                   self._cb_relay_map,   transient_qos)

        # ── Timers ─────────────────────────────────────────────────────────────
        self.create_timer(10.0, self._odom_health_report)
        self.create_timer(10.0, self._node_and_serial_health_report)
        self.create_timer(10.0, self._depth_rtabmap_health_report)
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
            f"visual/rtabmap={status(self._visual_odom_count)}  "
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

    # ── ICP odometry callback ──────────────────────────────────────────────────

    def _cb_icp_odom(self, msg: Odometry):
        self._icp_odom_count += 1
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Largest diagonal of the 6×6 pose covariance matrix
        cov = msg.pose.covariance
        diag = [cov[i * 7] for i in range(6)]   # indices 0,7,14,21,28,35
        max_cov = max(abs(v) for v in diag)
        self._last_icp_cov_max = max_cov

        was_lost = self._icp_lost_tracking
        self._icp_lost_tracking = max_cov > ICP_COV_WARN

        # Alert on tracking loss / recovery
        if self._icp_lost_tracking and not was_lost:
            print(
                f"[{DIM}{ts()}{RESET}] {RED}{BOLD}ICP LOST TRACKING{RESET}  "
                f"max_cov={max_cov:.3f} > {ICP_COV_WARN}  "
                f"{YELLOW}— rover pose is unreliable, map may drift{RESET}"
            )
        elif not self._icp_lost_tracking and was_lost:
            print(
                f"[{DIM}{ts()}{RESET}] {GREEN}{BOLD}ICP tracking recovered{RESET}  "
                f"max_cov={max_cov:.3f}"
            )

        # ── Reset detection — mirrors mission_control.cpp near_reset logic ──
        # mission_control rejects ICP messages that jump near origin (0,0)
        # while the fused pose says we're >0.5 m away.  This is a silent
        # rejection — the only sign is rtabmap reporting a fresh origin.
        # We can't see the fused pose here, so we detect the jump directly:
        # if ICP suddenly moves >0.5 m toward origin, flag it.
        near_origin = (abs(x) < 0.05 and abs(y) < 0.05)
        if near_origin and self._last_icp_pos is not None:
            prev_x, prev_y = self._last_icp_pos
            dist_jumped = math.hypot(x - prev_x, y - prev_y)
            if dist_jumped > 0.5:
                self._icp_reset_count += 1
                print(
                    f"[{DIM}{ts()}{RESET}] {RED}{BOLD}ICP RESET DETECTED (#{self._icp_reset_count}){RESET}  "
                    f"jumped {dist_jumped:.2f} m back to origin ({x:.3f},{y:.3f})  "
                    f"{YELLOW}mission_control will REJECT this — fused odom unaffected "
                    f"but /map may desync{RESET}"
                )

        self._last_icp_pos = (x, y)

    # ── RTAB-Map map callbacks ─────────────────────────────────────────────────

    def _cb_rtabmap_map(self, msg: OccupancyGrid):
        self._rtabmap_map_count += 1
        now = time.monotonic()

        w, h = msg.info.width, msg.info.height
        if w != self._last_map_w or h != self._last_map_h:
            known_cells = sum(1 for c in msg.data if c >= 0)
            print(
                f"[{DIM}{ts()}{RESET}] {CYAN}{BOLD}/rtabmap/map updated{RESET}  "
                f"{w}×{h} cells  res={msg.info.resolution:.3f}m  "
                f"known={known_cells}/{w*h}  "
                f"frame={msg.header.frame_id}"
            )
            self._last_map_w = w
            self._last_map_h = h

        self._last_map_time = now

    def _cb_relay_map(self, msg: OccupancyGrid):
        self._relay_map_count += 1
        # If the map has any non-free (-1 or >0) cells it's a real SLAM map
        has_content = any(c != 0 for c in msg.data)
        self._map_is_fake = not has_content

    # ── Depth / RTAB-Map health report (every 10 s) ────────────────────────────

    def _depth_rtabmap_health_report(self):
        now = time.monotonic()

        def rate_status(count, label):
            if count == 0:
                return f"{RED}DEAD  (0 msgs/10s){RESET}"
            elif count < 3:
                return f"{YELLOW}SLOW  ({count} msgs/10s){RESET}"
            else:
                return f"{GREEN}OK    ({count} msgs/10s){RESET}"

        # Depth image
        depth_age = now - self._last_depth_time if self._last_depth_time else None
        depth_str = rate_status(self._depth_img_count, "depth_img")
        if depth_age and depth_age > 2.0:
            depth_str += f"  {RED}STALE {depth_age:.1f}s{RESET}"

        # PointCloud
        pc_str = rate_status(self._pc_count, "points")
        if self._pc_count > 0:
            pc_str += f"  {DIM}last={self._last_pc_n_points} pts{RESET}"

        # ICP odometry
        icp_str = rate_status(self._icp_odom_count, "icp_odom")
        if self._icp_odom_count > 0:
            cov_colour = RED if self._icp_lost_tracking else GREEN
            icp_str += f"  {cov_colour}cov={self._last_icp_cov_max:.3f}{RESET}"
            if self._icp_lost_tracking:
                icp_str += f"  {RED}{BOLD}⚠ LOST TRACKING{RESET}"
            if self._icp_reset_count > 0:
                icp_str += f"  {RED}resets={self._icp_reset_count}{RESET}"

        # RTAB-Map map
        if self._rtabmap_map_count == 0:
            map_str = f"{RED}DEAD  (no /rtabmap/map yet){RESET}"
        else:
            map_age = now - self._last_map_time
            if map_age > MAP_STALE_WARN_S:
                map_str = (
                    f"{YELLOW}STALE ({map_age:.0f}s since last update)  "
                    f"{self._last_map_w}×{self._last_map_h}{RESET}"
                )
            else:
                map_str = (
                    f"{GREEN}OK    ({self._rtabmap_map_count} updates)  "
                    f"{self._last_map_w}×{self._last_map_h}{RESET}"
                )

        # /map relay
        map_src = f"{YELLOW}fake_map{RESET}" if self._map_is_fake else f"{GREEN}rtabmap SLAM{RESET}"
        relay_str = (
            f"{GREEN}relaying{RESET}" if self._relay_map_count > 0
            else f"{RED}no /map received{RESET}"
        )

        print(
            f"[{DIM}{ts()}{RESET}] {BOLD}Camera health (last 10s){RESET}  "
            f"depth_img={depth_str}  points={pc_str}"
        )
        print(
            f"[{DIM}{ts()}{RESET}] {BOLD}RTAB-Map health (last 10s){RESET}  "
            f"icp_odom={icp_str}  map={map_str}"
        )
        print(
            f"[{DIM}{ts()}{RESET}] {BOLD}/map status{RESET}  "
            f"source={map_src}  relay={relay_str}"
        )

        # Reset counters
        self._depth_img_count  = 0
        self._pc_count         = 0
        self._icp_odom_count   = 0
        self._rtabmap_map_count = 0
        self._relay_map_count  = 0

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
