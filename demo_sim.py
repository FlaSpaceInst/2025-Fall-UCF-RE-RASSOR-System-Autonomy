#!/usr/bin/env python3
"""
demo_sim.py — RE-RASSOR Demo Simulator (ROS 2 node)
─────────────────────────────────────────────────────
Replaces the two hardware-dependent nodes so the full autonomy stack can run
on a dev machine without the physical rover:

  serial_motor_controller  →  subscribes to /{rover_name}/wheel_instructions,
                               dead-reckons pose, publishes /odometry/wheel

  astra_camera (depth)     →  publishes an empty PointCloud2 on
                               /camera/depth/points so Nav2 costmaps
                               initialise cleanly (zero obstacles)

Run via:
  ros2 launch re_rassor_bringup demo_sim.launch.py

Dependencies
────────────
  pip install flask flask-cors flask-socketio   (for controller_server)
  # All ROS deps (rclpy, nav_msgs, etc.) come from the workspace.
"""

import math
import socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField


# ── Helpers ────────────────────────────────────────────────────────────────────

def _get_rover_name() -> str:
    """Match the IP-based name used by controller_server and motor_controller."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return 'ip_' + ip.replace('.', '_')
    except Exception:
        return 'ezrassor'


def _quat_from_yaw(yaw: float):
    """Return (x, y, z, w) unit quaternion for a pure Z-axis rotation."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _normalize_angle(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


# ── ROS 2 node ─────────────────────────────────────────────────────────────────

class DemoSim(Node):

    ODOM_HZ   = 50.0   # wheel odometry publish rate
    CLOUD_HZ  =  2.0   # empty point-cloud publish rate
    CMD_TIMEOUT_S = 0.5  # zero velocity after this many seconds without a command

    def __init__(self):
        super().__init__('demo_sim')

        self._rover_name = _get_rover_name()

        # ── Pose state (protected by lock) ────────────────────────────────────
        self._lock  = threading.Lock()
        self._x     = 0.0
        self._y     = 0.0
        self._yaw   = 0.0
        self._lin   = 0.0   # last commanded linear velocity  (m/s)
        self._ang   = 0.0   # last commanded angular velocity (rad/s)
        self._last_cmd_time = self.get_clock().now()

        # ── Subscribers ───────────────────────────────────────────────────────
        # Manual commands from the controller app
        wheel_topic = f'/{self._rover_name}/wheel_instructions'
        self.create_subscription(Twist, wheel_topic, self._wheel_cb, 10)
        # Autonomous commands from Nav2 (controller_server → velocity_smoother)
        self.create_subscription(Twist, '/cmd_vel', self._wheel_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, '/odometry/wheel', 10)

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._cloud_pub = self.create_publisher(
            PointCloud2, '/camera/depth/points', sensor_qos)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / self.ODOM_HZ,  self._odom_tick)
        self.create_timer(1.0 / self.CLOUD_HZ, self._cloud_tick)

        # Pre-build the static empty cloud (only stamp changes each tick)
        self._empty_cloud = self._build_empty_cloud()

        self.get_logger().info(
            f'\n'
            f'  ╔══════════════════════════════════════════╗\n'
            f'  ║         RE-RASSOR  Demo Simulator        ║\n'
            f'  ║                                          ║\n'
            f'  ║  rover name : {self._rover_name:<26} ║\n'
            f'  ║  wheel sub  : /{self._rover_name}/wheel_instructions\n'
            f'  ║  odom pub   : /odometry/wheel            ║\n'
            f'  ║  cloud pub  : /camera/depth/points       ║\n'
            f'  ╚══════════════════════════════════════════╝'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _wheel_cb(self, msg: Twist):
        with self._lock:
            self._lin = float(msg.linear.x)
            self._ang = float(msg.angular.z)
            self._last_cmd_time = self.get_clock().now()

    def _odom_tick(self):
        dt  = 1.0 / self.ODOM_HZ
        now = self.get_clock().now()

        with self._lock:
            # Command timeout — stop if controller goes silent
            age = (now - self._last_cmd_time).nanoseconds * 1e-9
            if age > self.CMD_TIMEOUT_S:
                self._lin = 0.0
                self._ang = 0.0

            lin = self._lin
            ang = self._ang

            # Unicycle dead-reckoning
            dth = ang * dt
            if abs(ang) < 1e-6:
                self._x += lin * math.cos(self._yaw) * dt
                self._y += lin * math.sin(self._yaw) * dt
            else:
                r = lin / ang
                self._x += r * (math.sin(self._yaw + dth) - math.sin(self._yaw))
                self._y += r * (math.cos(self._yaw) - math.cos(self._yaw + dth))

            self._yaw = _normalize_angle(self._yaw + dth)
            x, y, yaw = self._x, self._y, self._yaw

        # Build and publish Odometry
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = _quat_from_yaw(yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = lin
        odom.twist.twist.angular.z = ang

        # Diagonal covariance — small but non-zero so EKF/fuser accepts it
        odom.pose.covariance[0]  = 0.01   # x–x
        odom.pose.covariance[7]  = 0.01   # y–y
        odom.pose.covariance[35] = 0.01   # yaw–yaw
        odom.twist.covariance[0]  = 0.001
        odom.twist.covariance[35] = 0.001

        self._odom_pub.publish(odom)

    def _cloud_tick(self):
        self._empty_cloud.header.stamp = self.get_clock().now().to_msg()
        self._cloud_pub.publish(self._empty_cloud)

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _build_empty_cloud(self) -> PointCloud2:
        """Zero-point cloud so Nav2 costmap layers subscribe and stay clear."""
        msg = PointCloud2()
        msg.header.frame_id = 'camera_depth_optical_frame'
        msg.height      = 1
        msg.width       = 0
        msg.is_dense    = True
        msg.is_bigendian = False
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step   = 0
        msg.data       = []
        return msg


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = DemoSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
