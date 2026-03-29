"""
test_e2e_gazebo.py — re_rassor_test
────────────────────────────────────────────────────────────────────────────
End-to-end pytest suite launched via launch_testing.

What is validated
─────────────────
Camera (simulated depth):
  TC-CAM-01  /camera/depth/image_raw topic is being published
  TC-CAM-02  Depth image dimensions are non-zero (height > 0, width > 0)
  TC-CAM-03  Depth image encoding is 32FC1 (single-precision float metres)
             or 16UC1 (uint16 millimetres) — both are legal Astra outputs
  TC-CAM-04  /camera/depth/points (PointCloud2) is being published
  TC-CAM-05  PointCloud2 has at least one field named 'z'
  TC-CAM-06  /camera/color/image_raw topic is being published

Hardware commands (serial_motor_controller protocol):
  TC-HW-01   /hw_commands/wheel_instructions is advertised after the relay
             node starts
  TC-HW-02   Publishing a forward Twist to /ezrassor/wheel_instructions
             results in the same message appearing on /hw_commands/wheel_instructions
  TC-HW-03   Publishing to /cmd_vel is relayed to /hw_commands/cmd_vel
  TC-HW-04   front_arm_instructions (Float64) round-trips through the relay
  TC-HW-05   back_arm_instructions  (Float64) round-trips through the relay
  TC-HW-06   front_drum_instructions (Float64) round-trips through the relay
  TC-HW-07   back_drum_instructions  (Float64) round-trips through the relay
  TC-HW-08   shoulder_instructions (Twist) round-trips through the relay
  TC-HW-09   routine_actions (Int8) round-trips through the relay

Obstacle-detection integration:
  TC-OBS-01  /odometry/wheel is published (serial_motor_controller odom)
  TC-OBS-02  /map topic is published by rtabmap
  TC-OBS-03  /odom topic is published by rgbd_odometry

Run via:
    colcon test --packages-select re_rassor_test
    colcon test-result --verbose

Or directly (with the stack already running):
    pytest test/e2e/test_e2e_gazebo.py -v
"""

import math
import time
import threading
import unittest

import launch_testing
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float64, Int8

# ── QoS helpers ─────────────────────────────────────────────────────────────

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    durability=QoSDurabilityPolicy.VOLATILE,
)

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# ── Timeouts (seconds) ───────────────────────────────────────────────────────
TOPIC_TIMEOUT    = 20.0   # max wait for a topic to appear
MESSAGE_TIMEOUT  = 25.0   # max wait for a single message to arrive
RELAY_TIMEOUT    = 5.0    # relay round-trip (same machine, no serial)

# ── Pytest mark required by launch_testing ───────────────────────────────────
pytestmark = pytest.mark.launch_test


# ════════════════════════════════════════════════════════════════════════════
# Helpers
# ════════════════════════════════════════════════════════════════════════════

class _Subscriber:
    """Thin wrapper: subscribes to a topic and stores the latest message."""

    def __init__(self, node: Node, topic: str, msg_type, qos=RELIABLE_QOS):
        self._msg   = None
        self._lock  = threading.Lock()
        self._sub   = node.create_subscription(
            msg_type, topic,
            lambda m: self._store(m),
            qos,
        )

    def _store(self, msg):
        with self._lock:
            self._msg = msg

    def latest(self):
        with self._lock:
            return self._msg

    def wait(self, timeout: float = MESSAGE_TIMEOUT):
        """Block until a message arrives or timeout expires. Returns the msg."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = self.latest()
            if m is not None:
                return m
            time.sleep(0.05)
        return None


def _topic_exists(node: Node, topic: str, timeout: float = TOPIC_TIMEOUT) -> bool:
    """Poll until a topic with the given name appears in the graph."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        names_and_types = node.get_topic_names_and_types()
        if any(name == topic for name, _ in names_and_types):
            return True
        time.sleep(0.2)
    return False


def _publish_and_relay(
    pub_node: Node,
    sub_node: Node,
    pub_topic: str,
    sub_topic: str,
    msg_type,
    msg,
    qos=RELIABLE_QOS,
    timeout: float = RELAY_TIMEOUT,
):
    """
    Publish *msg* on *pub_topic* and return the first matching message that
    arrives on *sub_topic* within *timeout* seconds, or None on failure.
    """
    received = []
    lock = threading.Lock()

    def _cb(m):
        with lock:
            if not received:
                received.append(m)

    sub = sub_node.create_subscription(msg_type, sub_topic, _cb, qos)
    pub = pub_node.create_publisher(msg_type, pub_topic, qos)

    # Give DDS a moment to wire up
    time.sleep(0.3)

    deadline = time.time() + timeout
    while time.time() < deadline:
        pub.publish(msg)
        time.sleep(0.1)
        with lock:
            if received:
                break

    sub_node.destroy_subscription(sub)
    pub_node.destroy_publisher(pub)
    return received[0] if received else None


# ════════════════════════════════════════════════════════════════════════════
# Test class
# ════════════════════════════════════════════════════════════════════════════

class TestE2EGazebo(unittest.TestCase):
    """
    End-to-end tests.  rclpy is initialised once per class; all tests share
    two nodes (one for publishing, one for subscribing) to avoid port clashes.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.pub_node = rclpy.create_node("e2e_pub_node")
        cls.sub_node = rclpy.create_node("e2e_sub_node")

        # Spin both nodes in background threads
        cls._executor = rclpy.executors.MultiThreadedExecutor()
        cls._executor.add_node(cls.pub_node)
        cls._executor.add_node(cls.sub_node)
        cls._spin_thread = threading.Thread(
            target=cls._executor.spin, daemon=True)
        cls._spin_thread.start()

        # Subscribe to persistent topics immediately so we don't miss bursts
        cls._depth_img_sub   = _Subscriber(cls.sub_node,
            "/camera/depth/image_raw", Image, SENSOR_QOS)
        cls._color_img_sub   = _Subscriber(cls.sub_node,
            "/camera/color/image_raw", Image, SENSOR_QOS)
        cls._pointcloud_sub  = _Subscriber(cls.sub_node,
            "/camera/depth/points", PointCloud2, SENSOR_QOS)
        cls._wheel_odom_sub  = _Subscriber(cls.sub_node,
            "/odometry/wheel", Odometry)
        cls._map_sub         = _Subscriber(cls.sub_node,
            "/map", OccupancyGrid)
        cls._odom_sub        = _Subscriber(cls.sub_node,
            "/odom", Odometry)

    @classmethod
    def tearDownClass(cls):
        cls._executor.shutdown()
        cls.pub_node.destroy_node()
        cls.sub_node.destroy_node()
        rclpy.shutdown()

    # ── Camera tests (TC-CAM-*) ───────────────────────────────────────────

    def test_cam_01_depth_topic_published(self):
        """TC-CAM-01: /camera/depth/image_raw must appear in the topic graph."""
        self.assertTrue(
            _topic_exists(self.sub_node, "/camera/depth/image_raw"),
            "/camera/depth/image_raw was never advertised — "
            "check ros_gz_bridge startup",
        )

    def test_cam_02_depth_image_has_valid_dimensions(self):
        """TC-CAM-02: Depth image must have non-zero height and width."""
        msg = self._depth_img_sub.wait(MESSAGE_TIMEOUT)
        self.assertIsNotNone(
            msg,
            "/camera/depth/image_raw: no message received within "
            f"{MESSAGE_TIMEOUT}s — Gazebo may not be publishing",
        )
        self.assertGreater(msg.height, 0,
            f"Depth image height={msg.height} is zero")
        self.assertGreater(msg.width, 0,
            f"Depth image width={msg.width} is zero")

    def test_cam_03_depth_image_encoding(self):
        """TC-CAM-03: Depth encoding must be 32FC1 or 16UC1."""
        msg = self._depth_img_sub.wait(MESSAGE_TIMEOUT)
        self.assertIsNotNone(msg, "No depth image received")
        self.assertIn(
            msg.encoding, ("32FC1", "16UC1"),
            f"Unexpected depth encoding '{msg.encoding}'; "
            "expected 32FC1 (float metres) or 16UC1 (uint16 mm)",
        )

    def test_cam_04_pointcloud_topic_published(self):
        """TC-CAM-04: /camera/depth/points must appear in the topic graph."""
        self.assertTrue(
            _topic_exists(self.sub_node, "/camera/depth/points"),
            "/camera/depth/points was never advertised",
        )

    def test_cam_05_pointcloud_has_z_field(self):
        """TC-CAM-05: PointCloud2 must contain a 'z' field."""
        msg = self._pointcloud_sub.wait(MESSAGE_TIMEOUT)
        self.assertIsNotNone(msg, "No PointCloud2 message received")
        field_names = [f.name for f in msg.fields]
        self.assertIn("z", field_names,
            f"PointCloud2 fields {field_names} do not include 'z'")

    def test_cam_06_color_topic_published(self):
        """TC-CAM-06: /camera/color/image_raw must appear."""
        self.assertTrue(
            _topic_exists(self.sub_node, "/camera/color/image_raw"),
            "/camera/color/image_raw was never advertised",
        )

    # ── Hardware-command relay tests (TC-HW-*) ────────────────────────────

    def test_hw_01_relay_wheel_topic_advertised(self):
        """TC-HW-01: /hw_commands/wheel_instructions must be advertised."""
        self.assertTrue(
            _topic_exists(self.sub_node, "/hw_commands/wheel_instructions",
                          timeout=TOPIC_TIMEOUT),
            "/hw_commands/wheel_instructions not found — "
            "hw_command_relay may not have started yet",
        )

    def test_hw_02_wheel_instructions_round_trip(self):
        """TC-HW-02: Twist published on /ezrassor/wheel_instructions must
        appear on /hw_commands/wheel_instructions with identical values."""
        msg_out = Twist()
        msg_out.linear.x  =  0.5
        msg_out.angular.z = -0.3

        relayed = _publish_and_relay(
            self.pub_node, self.sub_node,
            "/ezrassor/wheel_instructions",
            "/hw_commands/wheel_instructions",
            Twist, msg_out,
        )
        self.assertIsNotNone(relayed,
            "No message relayed on /hw_commands/wheel_instructions")
        self.assertAlmostEqual(relayed.linear.x,  msg_out.linear.x,  places=4)
        self.assertAlmostEqual(relayed.angular.z, msg_out.angular.z, places=4)

    def test_hw_03_cmd_vel_round_trip(self):
        """TC-HW-03: /cmd_vel must be relayed to /hw_commands/cmd_vel."""
        msg_out = Twist()
        msg_out.linear.x = 1.0

        relayed = _publish_and_relay(
            self.pub_node, self.sub_node,
            "/cmd_vel",
            "/hw_commands/cmd_vel",
            Twist, msg_out,
        )
        self.assertIsNotNone(relayed,
            "No message relayed on /hw_commands/cmd_vel")
        self.assertAlmostEqual(relayed.linear.x, 1.0, places=4)

    def test_hw_04_front_arm_round_trip(self):
        """TC-HW-04: front_arm_instructions (Float64) round-trip."""
        msg_out = Float64()
        msg_out.data = 0.75

        relayed = _publish_and_relay(
            self.pub_node, self.sub_node,
            "/ezrassor/front_arm_instructions",
            "/hw_commands/front_arm_instructions",
            Float64, msg_out,
        )
        self.assertIsNotNone(relayed,
            "No message relayed on /hw_commands/front_arm_instructions")
        self.assertAlmostEqual(relayed.data, 0.75, places=4)

    def test_hw_05_back_arm_round_trip(self):
        """TC-HW-05: back_arm_instructions (Float64) round-trip."""
        msg_out = Float64()
        msg_out.data = -0.5

        relayed = _publish_and_relay(
            self.pub_node, self.sub_node,
            "/ezrassor/back_arm_instructions",
            "/hw_commands/back_arm_instructions",
            Float64, msg_out,
        )
        self.assertIsNotNone(relayed,
            "No message relayed on /hw_commands/back_arm_instructions")
        self.assertAlmostEqual(relayed.data, -0.5, places=4)

    def test_hw_06_front_drum_round_trip(self):
        """TC-HW-06: front_drum_instructions (Float64) round-trip."""
        msg_out = Float64()
        msg_out.data = 1.0

        relayed = _publish_and_relay(
            self.pub_node, self.sub_node,
            "/ezrassor/front_drum_instructions",
            "/hw_commands/front_drum_instructions",
            Float64, msg_out,
        )
        self.assertIsNotNone(relayed,
            "No message relayed on /hw_commands/front_drum_instructions")
        self.assertAlmostEqual(relayed.data, 1.0, places=4)

    def test_hw_07_back_drum_round_trip(self):
        """TC-HW-07: back_drum_instructions (Float64) round-trip."""
        msg_out = Float64()
        msg_out.data = -1.0

        relayed = _publish_and_relay(
            self.pub_node, self.sub_node,
            "/ezrassor/back_drum_instructions",
            "/hw_commands/back_drum_instructions",
            Float64, msg_out,
        )
        self.assertIsNotNone(relayed,
            "No message relayed on /hw_commands/back_drum_instructions")
        self.assertAlmostEqual(relayed.data, -1.0, places=4)

    def test_hw_08_shoulder_instructions_round_trip(self):
        """TC-HW-08: shoulder_instructions (Twist) round-trip."""
        msg_out = Twist()
        msg_out.linear.y  = 0.3
        msg_out.angular.y = 0.0

        relayed = _publish_and_relay(
            self.pub_node, self.sub_node,
            "/ezrassor/shoulder_instructions",
            "/hw_commands/shoulder_instructions",
            Twist, msg_out,
        )
        self.assertIsNotNone(relayed,
            "No message relayed on /hw_commands/shoulder_instructions")
        self.assertAlmostEqual(relayed.linear.y, 0.3, places=4)

    def test_hw_09_routine_actions_round_trip(self):
        """TC-HW-09: routine_actions (Int8) round-trip."""
        msg_out = Int8()
        msg_out.data = 0b100000  # STOP routine per serial_motor_controller

        relayed = _publish_and_relay(
            self.pub_node, self.sub_node,
            "/ezrassor/routine_actions",
            "/hw_commands/routine_actions",
            Int8, msg_out,
        )
        self.assertIsNotNone(relayed,
            "No message relayed on /hw_commands/routine_actions")
        self.assertEqual(relayed.data, msg_out.data)

    # ── Obstacle / navigation integration (TC-OBS-*) ─────────────────────

    def test_obs_01_wheel_odometry_published(self):
        """TC-OBS-01: /odometry/wheel must be published by the odom estimator."""
        msg = self._wheel_odom_sub.wait(MESSAGE_TIMEOUT)
        self.assertIsNotNone(
            msg,
            "/odometry/wheel: no message received — "
            "serial_motor_controller odom loop may not be running in sim",
        )
        self.assertEqual(msg.header.frame_id, "odom",
            f"Unexpected frame_id '{msg.header.frame_id}' (expected 'odom')")
        self.assertEqual(msg.child_frame_id, "base_link",
            f"Unexpected child_frame_id '{msg.child_frame_id}'")

    def test_obs_02_map_published(self):
        """TC-OBS-02: /map (OccupancyGrid) must be published by rtabmap."""
        msg = self._map_sub.wait(MESSAGE_TIMEOUT + 10.0)  # rtabmap takes a moment
        self.assertIsNotNone(
            msg,
            "/map: no OccupancyGrid received — rtabmap SLAM may not be running",
        )
        self.assertEqual(msg.header.frame_id, "map",
            f"Unexpected map frame_id '{msg.header.frame_id}'")

    def test_obs_03_odom_published(self):
        """TC-OBS-03: /odom must be published by rgbd_odometry."""
        msg = self._odom_sub.wait(MESSAGE_TIMEOUT)
        self.assertIsNotNone(
            msg,
            "/odom: no Odometry received — rgbd_odometry may not be running",
        )
        self.assertEqual(msg.header.frame_id, "odom")

    # ── Known obstacle sanity check ───────────────────────────────────────

    def test_cam_07_pointcloud_contains_near_obstacle(self):
        """
        TC-CAM-07 (integration): PointCloud2 must contain at least one point
        with z in [0.5 m, 3.0 m] — consistent with the closest obstacle
        (C at 1.04 m) being visible from the origin.

        NOTE: This test only checks that *some* near-range points exist.
        It does NOT assert exact obstacle coordinates (that belongs to
        test_obstacle_detection_node.py).
        """
        msg = self._pointcloud_sub.wait(MESSAGE_TIMEOUT)
        self.assertIsNotNone(msg, "No PointCloud2 received")

        import struct
        import numpy as np

        # Parse PointCloud2 to a flat float array of (x, y, z, …) per point
        # Works for both XYZ and XYZRGB organised clouds.
        point_step = msg.point_step
        field_map  = {f.name: f.offset for f in msg.fields}

        if "z" not in field_map:
            self.fail(f"PointCloud2 has no 'z' field; fields={list(field_map)}")

        z_offset = field_map["z"]
        data     = bytes(msg.data)
        n_points = len(data) // point_step

        self.assertGreater(n_points, 0, "PointCloud2 is empty")

        near_count = 0
        for i in range(n_points):
            base = i * point_step + z_offset
            z_val, = struct.unpack_from("<f", data, base)
            if math.isfinite(z_val) and 0.5 <= z_val <= 3.0:
                near_count += 1
                if near_count >= 10:   # enough evidence
                    break

        self.assertGreaterEqual(
            near_count, 10,
            f"Expected ≥10 points with z in [0.5, 3.0] m but found {near_count}. "
            "Obstacle may not be in camera FOV or Gazebo depth plugin is misconfigured.",
        )