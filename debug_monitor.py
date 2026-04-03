#!/usr/bin/env python3
"""
RE-RASSOR Debug Monitor
-----------------------
Run this on the rover (after sourcing ROS2) to see exactly what Nav2 and
the controller server are sending in real-time.

Usage:
    python3 debug_monitor.py [--rover-name <name>]

If --rover-name is omitted it tries to auto-detect from the running nodes.
Press Ctrl+C to exit.
"""

import argparse
import math
import os
import socket
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from tf2_msgs.msg import TFMessage

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
]

# Serial ports we care about
SERIAL_PORTS = ["/dev/arduino_wheel", "/dev/arduino_drum"]

# Dead-band parameters (must match serial_motor_controller.cpp)
DEADBAND_LIN_MAX = 0.08   # |lin_x| < this AND lin_x < 0 → STOP
DEADBAND_ANG_MAX = 0.01   # |ang_z| < this (angular must also be small)

# How long (s) without an odom→base_link TF before we warn
TF_STALE_WARN_S = 0.5


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

        print(f"\n{BOLD}{'─'*60}{RESET}")
        print(f"{BOLD}  RE-RASSOR Debug Monitor{RESET}  (rover: {CYAN}{rover_name}{RESET})")
        print(f"{BOLD}{'─'*60}{RESET}")
        print(f"  Monitoring topics:")
        print(f"    {GREEN}/cmd_vel{RESET}               ← Nav2 velocity output")
        print(f"    {GREEN}/{rover_name}/wheel_instructions{RESET} ← Controller-server stop")
        print(f"    {GREEN}/ezrassor/routine_actions{RESET} ← Stop/Dig/Dump signals")
        print(f"    {GREEN}/navigate_to_pose/...{RESET}  ← Nav2 action status + distance")
        print(f"    {GREEN}/odometry/wheel{RESET}        ← Wheel encoder odometry")
        print(f"    {GREEN}/odom{RESET}                  ← RTAB-Map visual odometry")
        print(f"    {GREEN}/odometry/fused{RESET}        ← Fused position (what Nav2 uses)")
        print(f"    {GREEN}/tf{RESET}                    ← odom→base_link TF staleness")
        print(f"{BOLD}{'─'*60}{RESET}")
        print(f"  {DIM}Odom pulse counts printed every 10s — zero means that source is dead{RESET}")
        print(f"  {DIM}Node + serial health printed every 10s{RESET}\n")

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

        # ── Timers ─────────────────────────────────────────────────────────────
        self.create_timer(10.0, self._odom_health_report)
        self.create_timer(10.0, self._node_and_serial_health_report)

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
