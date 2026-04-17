#!/usr/bin/env python3
"""
nav2_debug.py — Real-time Nav2 path + status printer for RE-RASSOR simulation.

Subscribes to:
  /plan                  — global path from planner_server (nav_msgs/Path)
  /local_plan            — local path from RPP controller (nav_msgs/Path)
  /cmd_vel               — velocity commands to rover (geometry_msgs/Twist)
  /odometry/wheel        — current odom pose (nav_msgs/Odometry)
  /goal_pose             — latest goal sent to Nav2 (geometry_msgs/PoseStamped)
  /navigate_to_pose/_action/feedback — Nav2 action feedback (distance to goal, ETA)

Run while simulation is active:
  python3 src/re_rassor_simulation/re_rassor_simulation/nav2_debug.py
or via ros2 run:
  ros2 run re_rassor_simulation nav2_debug
"""

import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path

try:
    from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
    _NAV2_FB_OK = True
except ImportError:
    _NAV2_FB_OK = False


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _dist(p1, p2) -> float:
    return math.hypot(p1.x - p2.x, p1.y - p2.y)


def _path_length(poses) -> float:
    total = 0.0
    for i in range(1, len(poses)):
        total += _dist(poses[i - 1].pose.position, poses[i].pose.position)
    return total


def _clear_line():
    sys.stdout.write('\r\033[K')


def _header(text: str):
    cols = 72
    bar  = '─' * cols
    print(f'\n┌{bar}┐')
    print(f'│  {text:<{cols - 2}}│')
    print(f'└{bar}┘')


# ─────────────────────────────────────────────────────────────────────────────
# Debug node
# ─────────────────────────────────────────────────────────────────────────────

class Nav2Debug(Node):

    def __init__(self):
        super().__init__('nav2_debug')
        # use_sim_time is auto-declared by rclpy.Node — do NOT re-declare it.
        # Pass via: ros2 run re_rassor_simulation nav2_debug --ros-args -p use_sim_time:=true

        self._lock = threading.Lock()

        # State
        self._global_plan:  'Path | None'          = None
        self._local_plan:   'Path | None'           = None
        self._cmd_vel:      'Twist | None'          = None
        self._odom:         'Odometry | None'       = None
        self._goal:         'PoseStamped | None'    = None
        self._feedback:     'object | None'         = None
        self._global_plan_t: float                  = 0.0
        self._local_plan_t:  float                  = 0.0
        self._plan_printed:  bool                   = False  # print new plan once

        # QoS: transient local for /goal_pose so we catch goals published before us
        tl_qos = QoSProfile(depth=1,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Path,         '/plan',       self._plan_cb,     10)
        self.create_subscription(Path,         '/local_plan', self._local_cb,    10)
        self.create_subscription(Twist,        '/cmd_vel',    self._vel_cb,      10)
        self.create_subscription(Odometry,     '/odometry/wheel', self._odom_cb, 10)
        self.create_subscription(PoseStamped,  '/goal_pose',  self._goal_cb,     tl_qos)

        # Nav2 action feedback topic (raw sub — avoids needing action client)
        if _NAV2_FB_OK:
            try:
                self.create_subscription(
                    NavigateToPose_FeedbackMessage,
                    '/navigate_to_pose/_action/feedback',
                    self._feedback_cb,
                    10,
                )
            except Exception as e:
                self.get_logger().warn(f'Nav2 feedback sub failed: {e}')
        else:
            self.get_logger().warn('nav2_msgs not found — no action feedback')

        # Print status at 2 Hz
        self.create_timer(0.5, self._print_status)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _plan_cb(self, msg: Path):
        with self._lock:
            self._global_plan   = msg
            self._global_plan_t = time.time()
            self._plan_printed  = False  # trigger one-shot full print

    def _local_cb(self, msg: Path):
        with self._lock:
            self._local_plan   = msg
            self._local_plan_t = time.time()

    def _vel_cb(self, msg: Twist):
        with self._lock:
            self._cmd_vel = msg

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom = msg

    def _goal_cb(self, msg: PoseStamped):
        with self._lock:
            self._goal         = msg
            self._plan_printed = False  # new goal → reprint plan when it arrives

    def _feedback_cb(self, msg):
        with self._lock:
            self._feedback = msg.feedback

    # ── Periodic print ────────────────────────────────────────────────────────

    def _print_status(self):
        with self._lock:
            plan      = self._global_plan
            local     = self._local_plan
            vel       = self._cmd_vel
            odom      = self._odom
            goal      = self._goal
            feedback  = self._feedback
            need_full = not self._plan_printed and plan is not None
            if need_full:
                self._plan_printed = True

        # ── One-shot: print full global plan when new plan arrives ────────────
        if need_full and plan:
            _header('NEW GLOBAL PLAN')
            poses = plan.poses
            n     = len(poses)
            length = _path_length(poses)
            print(f'  Frame  : {plan.header.frame_id}')
            print(f'  Points : {n}')
            print(f'  Length : {length:.3f} m')
            if n > 0:
                s = poses[0].pose.position
                e = poses[-1].pose.position
                print(f'  Start  : ({s.x:.3f}, {s.y:.3f})')
                print(f'  End    : ({e.x:.3f}, {e.y:.3f})')
            print(f'  {"#":>4}   {"x":>8}   {"y":>8}   {"yaw_deg":>9}')
            print(f'  {"─"*4}   {"─"*8}   {"─"*8}   {"─"*9}')
            step = max(1, n // 20)  # print up to 20 waypoints
            for i in range(0, n, step):
                p   = poses[i].pose
                yaw = math.degrees(_yaw(p.orientation))
                print(f'  {i:>4}   {p.position.x:>8.3f}   {p.position.y:>8.3f}'
                      f'   {yaw:>8.1f}°')
            if (n - 1) % step != 0:
                p   = poses[-1].pose
                yaw = math.degrees(_yaw(p.orientation))
                print(f'  {n-1:>4}   {p.position.x:>8.3f}   {p.position.y:>8.3f}'
                      f'   {yaw:>8.1f}° ← goal')
            print()

        # ── 2 Hz rolling status line ──────────────────────────────────────────
        parts = []

        if odom:
            p   = odom.pose.pose.position
            yaw = math.degrees(_yaw(odom.pose.pose.orientation))
            vx  = odom.twist.twist.linear.x
            wz  = odom.twist.twist.angular.z
            parts.append(f'odom ({p.x:.2f},{p.y:.2f}) yaw={yaw:.1f}°'
                         f'  vx={vx:.3f} wz={wz:.3f}')
        else:
            parts.append('odom: waiting…')

        if vel:
            parts.append(f'cmd_vel  lx={vel.linear.x:.3f}  az={vel.angular.z:.3f}')
        else:
            parts.append('cmd_vel: –')

        if goal and odom:
            gx = goal.pose.position.x
            gy = goal.pose.position.y
            rx = odom.pose.pose.position.x
            ry = odom.pose.pose.position.y
            d  = math.hypot(gx - rx, gy - ry)
            parts.append(f'goal ({gx:.2f},{gy:.2f}) dist={d:.3f}m')
        elif goal:
            gx = goal.pose.position.x
            gy = goal.pose.position.y
            parts.append(f'goal ({gx:.2f},{gy:.2f})')

        if feedback:
            try:
                dtg = feedback.distance_remaining
                eta = feedback.estimated_time_remaining.sec
                parts.append(f'nav2 dist_remaining={dtg:.3f}m ETA={eta}s')
            except AttributeError:
                pass

        if plan:
            age = time.time() - self._global_plan_t
            n   = len(plan.poses)
            parts.append(f'global_plan {n}pts age={age:.1f}s')

        if local:
            age = time.time() - self._local_plan_t
            n   = len(local.poses)
            parts.append(f'local_plan {n}pts age={age:.1f}s')

        line = '  |  '.join(parts)
        _clear_line()
        sys.stdout.write(line)
        sys.stdout.flush()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Debug()
    print('nav2_debug active — waiting for topics…')
    print('  /plan               → prints full path on new plan')
    print('  /cmd_vel            → rolling status line')
    print('  /odometry/wheel     → rolling status line')
    print('  /goal_pose          → distance to goal')
    print('  /navigate_to_pose/… → Nav2 feedback\n')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[nav2_debug] stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
