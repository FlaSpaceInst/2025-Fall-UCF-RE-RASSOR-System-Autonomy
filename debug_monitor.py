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
import socket
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatusArray

# ── ANSI colours ──────────────────────────────────────────────────────────────
RESET  = "\033[0m"
BOLD   = "\033[1m"
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
DIM    = "\033[2m"

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


def ts() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def fmt_twist(t: Twist) -> str:
    return f"lin_x={t.linear.x:+.3f}  ang_z={t.angular.z:+.3f}"


class DebugMonitor(Node):
    def __init__(self, rover_name: str):
        super().__init__("re_rassor_debug_monitor")
        self._rover_name = rover_name
        self._last_cmd_vel = None
        self._last_wheel   = None
        self._last_routine = None

        print(f"\n{BOLD}{'─'*60}{RESET}")
        print(f"{BOLD}  RE-RASSOR Debug Monitor{RESET}  (rover: {CYAN}{rover_name}{RESET})")
        print(f"{BOLD}{'─'*60}{RESET}")
        print(f"  Monitoring topics:")
        print(f"    {GREEN}/cmd_vel{RESET}                  ← Nav2 velocity output")
        print(f"    {GREEN}/{rover_name}/wheel_instructions{RESET} ← Controller-server stop")
        print(f"    {GREEN}/ezrassor/routine_actions{RESET}  ← Stop/Dig/Dump signals")
        print(f"    {GREEN}/navigate_to_pose/...{RESET}     ← Nav2 action status")
        print(f"    {GREEN}/odometry/fused{RESET}           ← Position estimate")
        print(f"{BOLD}{'─'*60}{RESET}\n")

        # /cmd_vel — what Nav2 actually commands to the motors
        self.create_subscription(
            Twist, "/cmd_vel", self._cb_cmd_vel, 10
        )

        # /{rover_name}/wheel_instructions — what controller server sends
        self.create_subscription(
            Twist,
            f"/{rover_name}/wheel_instructions",
            self._cb_wheel,
            10,
        )

        # /ezrassor/routine_actions — stop / dig / dump signals
        self.create_subscription(
            Int8, "/ezrassor/routine_actions", self._cb_routine, 10
        )

        # Nav2 action goal status array
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._cb_nav2_status,
            10,
        )

        # Fused odometry — current position
        self.create_subscription(
            Odometry, "/odometry/fused", self._cb_odom, 10
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_cmd_vel(self, msg: Twist):
        same = (
            self._last_cmd_vel is not None
            and abs(msg.linear.x  - self._last_cmd_vel.linear.x)  < 0.001
            and abs(msg.angular.z - self._last_cmd_vel.angular.z) < 0.001
        )
        self._last_cmd_vel = msg
        if same:
            return  # suppress repeated identical values to reduce noise

        colour = RED if (abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001) else GREEN
        label  = "MOVING" if colour == RED else "STOPPED"
        print(
            f"[{DIM}{ts()}{RESET}] {BOLD}/cmd_vel{RESET}  "
            f"{colour}{fmt_twist(msg)}{RESET}  [{colour}{label}{RESET}]"
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
            f"[{DIM}{ts()}{RESET}] {BOLD}wheel_instr{RESET} "
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

    def _cb_nav2_status(self, msg: GoalStatusArray):
        for gs in msg.status_list:
            status_str = NAV2_STATUS.get(gs.status, f"STATUS_{gs.status}")
            gid = bytes(gs.goal_info.goal_id.uuid).hex()[:8]

            if gs.status in (4, 5, 6):   # terminal states
                colour = GREEN if gs.status == 4 else RED
            elif gs.status in (1, 2):
                colour = CYAN
            else:
                colour = YELLOW

            print(
                f"[{DIM}{ts()}{RESET}] {BOLD}Nav2 action{RESET}  "
                f"goal={gid}  {colour}{status_str}{RESET}"
            )

    _odom_print_interval = 5.0
    _last_odom_print     = 0.0

    def _cb_odom(self, msg: Odometry):
        now = time.monotonic()
        if now - self._last_odom_print < self._odom_print_interval:
            return
        self._last_odom_print = now
        p = msg.pose.pose.position
        print(
            f"[{DIM}{ts()}{RESET}] {DIM}odom/fused   "
            f"x={p.x:+.3f}  y={p.y:+.3f}  z={p.z:+.3f}{RESET}"
        )


def auto_detect_rover_name() -> str:
    """Best-effort: derive rover name the same way server.py does."""
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
