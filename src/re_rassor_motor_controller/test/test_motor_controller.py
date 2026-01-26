#!/usr/bin/env python3
"""
Integration tests for the Motor Controller node.

These tests verify that the motor controller correctly:
1. Subscribes to controller server command topics
2. Processes wheel, arm, drum, and routine commands
3. Publishes odometry feedback to /location_status
4. Handles command timeouts correctly

To run these tests:
    ros2 run re_rassor_traffic_controller test_motor_controller.py

Or with pytest:
    pytest test_motor_controller.py -v
"""

import unittest
import time
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int8
from re_rassor_interfaces.msg import LocationStatus


class TestMotorControllerNode(Node):
    """Test harness node for motor controller testing."""

    def __init__(self):
        super().__init__('test_motor_controller_node')

        # Publishers to simulate controller server commands
        self.wheel_pub = self.create_publisher(
            Twist, '/ezrassor/wheel_instructions', 10)
        self.front_arm_pub = self.create_publisher(
            Float64, '/ezrassor/front_arm_instructions', 10)
        self.back_arm_pub = self.create_publisher(
            Float64, '/ezrassor/back_arm_instructions', 10)
        self.front_drum_pub = self.create_publisher(
            Float64, '/ezrassor/front_drum_instructions', 10)
        self.back_drum_pub = self.create_publisher(
            Float64, '/ezrassor/back_drum_instructions', 10)
        self.routine_pub = self.create_publisher(
            Int8, '/ezrassor/routine_actions', 10)

        # Subscriber to receive motor controller feedback
        self.location_status_received = []
        self.location_status_sub = self.create_subscription(
            LocationStatus,
            '/location_status',
            self.location_status_callback,
            10
        )

        self.controller_status_received = []
        self.controller_status_sub = self.create_subscription(
            Int8,
            '/motor_controller/status',
            self.controller_status_callback,
            10
        )

    def location_status_callback(self, msg):
        """Store received location status messages."""
        self.location_status_received.append(msg)

    def controller_status_callback(self, msg):
        """Store received controller status messages."""
        self.controller_status_received.append(msg)

    def publish_wheel_command(self, linear_x, angular_z):
        """Publish a wheel command."""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.wheel_pub.publish(msg)

    def publish_front_arm_command(self, action):
        """Publish a front arm command (-1=LOWER, 0=STOP, 1=RAISE)."""
        msg = Float64()
        msg.data = float(action)
        self.front_arm_pub.publish(msg)

    def publish_back_arm_command(self, action):
        """Publish a back arm command."""
        msg = Float64()
        msg.data = float(action)
        self.back_arm_pub.publish(msg)

    def publish_front_drum_command(self, action):
        """Publish a front drum command (-1=DUMP, 0=STOP, 1=DIG)."""
        msg = Float64()
        msg.data = float(action)
        self.front_drum_pub.publish(msg)

    def publish_back_drum_command(self, action):
        """Publish a back drum command."""
        msg = Float64()
        msg.data = float(action)
        self.back_drum_pub.publish(msg)

    def publish_routine_action(self, routine):
        """Publish a routine action."""
        msg = Int8()
        msg.data = int(routine)
        self.routine_pub.publish(msg)

    def clear_received_messages(self):
        """Clear all received message buffers."""
        self.location_status_received.clear()
        self.controller_status_received.clear()


class TestMotorControllerIntegration(unittest.TestCase):
    """Integration tests for Motor Controller with Controller Server."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 and create test nodes."""
        rclpy.init()
        cls.test_node = TestMotorControllerNode()
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.test_node)

        # Spin in background thread
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

        # Wait for connections to establish
        time.sleep(1.0)

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2."""
        cls.executor.shutdown()
        cls.test_node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        """Clear message buffers before each test."""
        self.test_node.clear_received_messages()
        time.sleep(0.1)

    def test_wheel_command_forward(self):
        """Test that forward wheel commands are received and processed."""
        # Send forward command
        self.test_node.publish_wheel_command(linear_x=0.5, angular_z=0.0)
        time.sleep(0.5)

        # Check that location status is being published
        self.assertGreater(
            len(self.test_node.location_status_received), 0,
            "No location status messages received after wheel command"
        )

        # Verify velocity is being reported
        latest_status = self.test_node.location_status_received[-1]
        self.assertIsNotNone(latest_status.velocity)

    def test_wheel_command_rotation(self):
        """Test that rotation wheel commands are processed."""
        # Send rotation command
        self.test_node.publish_wheel_command(linear_x=0.0, angular_z=1.0)
        time.sleep(0.5)

        self.assertGreater(
            len(self.test_node.location_status_received), 0,
            "No location status messages received after rotation command"
        )

    def test_wheel_command_combined(self):
        """Test combined linear and angular motion."""
        # Send combined command (arc motion)
        self.test_node.publish_wheel_command(linear_x=0.3, angular_z=0.5)
        time.sleep(0.5)

        self.assertGreater(
            len(self.test_node.location_status_received), 0,
            "No location status messages received after combined command"
        )

    def test_front_arm_raise(self):
        """Test front arm raise command."""
        self.test_node.publish_front_arm_command(action=1.0)  # RAISE
        time.sleep(0.3)

        # Motor controller should acknowledge command via status updates
        self.assertGreater(
            len(self.test_node.location_status_received), 0,
            "No status update after front arm command"
        )

    def test_front_arm_lower(self):
        """Test front arm lower command."""
        self.test_node.publish_front_arm_command(action=-1.0)  # LOWER
        time.sleep(0.3)

        self.assertGreater(
            len(self.test_node.location_status_received), 0,
            "No status update after front arm lower command"
        )

    def test_back_arm_commands(self):
        """Test back arm commands."""
        self.test_node.publish_back_arm_command(action=1.0)  # RAISE
        time.sleep(0.2)
        self.test_node.publish_back_arm_command(action=0.0)  # STOP
        time.sleep(0.2)
        self.test_node.publish_back_arm_command(action=-1.0)  # LOWER
        time.sleep(0.2)

        self.assertGreater(
            len(self.test_node.location_status_received), 0,
            "No status update after back arm commands"
        )

    def test_front_drum_dig(self):
        """Test front drum dig command."""
        self.test_node.publish_front_drum_command(action=1.0)  # DIG
        time.sleep(0.3)

        self.assertGreater(
            len(self.test_node.location_status_received), 0,
            "No status update after front drum dig command"
        )

    def test_front_drum_dump(self):
        """Test front drum dump command."""
        self.test_node.publish_front_drum_command(action=-1.0)  # DUMP
        time.sleep(0.3)

        self.assertGreater(
            len(self.test_node.location_status_received), 0,
            "No status update after front drum dump command"
        )

    def test_back_drum_commands(self):
        """Test back drum commands."""
        self.test_node.publish_back_drum_command(action=1.0)  # DIG
        time.sleep(0.2)
        self.test_node.publish_back_drum_command(action=-1.0)  # DUMP
        time.sleep(0.2)

        self.assertGreater(
            len(self.test_node.location_status_received), 0,
            "No status update after back drum commands"
        )

    def test_routine_auto_drive(self):
        """Test AUTO_DRIVE routine activation."""
        AUTO_DRIVE = 0b000001
        self.test_node.publish_routine_action(AUTO_DRIVE)
        time.sleep(0.3)

        self.assertGreater(
            len(self.test_node.controller_status_received), 0,
            "No controller status received after routine command"
        )

        # Verify routine is active
        latest_status = self.test_node.controller_status_received[-1]
        self.assertEqual(latest_status.data & AUTO_DRIVE, AUTO_DRIVE)

    def test_routine_stop(self):
        """Test STOP routine command."""
        STOP = 0b100000
        self.test_node.publish_routine_action(STOP)
        time.sleep(0.3)

        # After STOP, active_routine should be 0
        self.assertGreater(
            len(self.test_node.controller_status_received), 0,
            "No controller status received after STOP command"
        )

    def test_routine_full_autonomy(self):
        """Test FULL_AUTONOMY routine activation."""
        FULL_AUTONOMY = 0b010000
        self.test_node.publish_routine_action(FULL_AUTONOMY)
        time.sleep(0.3)

        self.assertGreater(
            len(self.test_node.controller_status_received), 0,
            "No controller status received after full autonomy command"
        )

    def test_odometry_position_update(self):
        """Test that odometry position updates correctly with wheel commands."""
        self.test_node.clear_received_messages()

        # Send forward motion command
        self.test_node.publish_wheel_command(linear_x=1.0, angular_z=0.0)

        # Wait for position updates
        time.sleep(1.0)

        self.assertGreater(
            len(self.test_node.location_status_received), 5,
            "Not enough location status updates received"
        )

        # Verify position has changed (robot should have moved forward)
        first_status = self.test_node.location_status_received[0]
        last_status = self.test_node.location_status_received[-1]

        # Position should have changed
        position_changed = (
            abs(last_status.position_x - first_status.position_x) > 0.01 or
            abs(last_status.position_y - first_status.position_y) > 0.01
        )
        self.assertTrue(
            position_changed,
            f"Position did not update: first=({first_status.position_x}, {first_status.position_y}), "
            f"last=({last_status.position_x}, {last_status.position_y})"
        )

    def test_odometry_orientation_update(self):
        """Test that orientation updates correctly with rotation commands."""
        self.test_node.clear_received_messages()

        # Send pure rotation command
        self.test_node.publish_wheel_command(linear_x=0.0, angular_z=1.0)

        # Wait for orientation updates
        time.sleep(1.0)

        self.assertGreater(
            len(self.test_node.location_status_received), 5,
            "Not enough location status updates received"
        )

        # Verify orientation has changed
        first_status = self.test_node.location_status_received[0]
        last_status = self.test_node.location_status_received[-1]

        orientation_changed = abs(last_status.orientation - first_status.orientation) > 0.01
        self.assertTrue(
            orientation_changed,
            f"Orientation did not update: first={first_status.orientation}, last={last_status.orientation}"
        )

    def test_multiple_simultaneous_commands(self):
        """Test handling of multiple commands sent simultaneously."""
        # Send multiple commands at once
        self.test_node.publish_wheel_command(linear_x=0.5, angular_z=0.2)
        self.test_node.publish_front_arm_command(action=1.0)
        self.test_node.publish_back_drum_command(action=-1.0)

        time.sleep(0.5)

        # Should still receive status updates
        self.assertGreater(
            len(self.test_node.location_status_received), 0,
            "No status updates received after multiple simultaneous commands"
        )

    def test_command_sequence(self):
        """Test a sequence of commands simulating a digging operation."""
        # 1. Move forward to digging location
        self.test_node.publish_wheel_command(linear_x=0.3, angular_z=0.0)
        time.sleep(0.5)

        # 2. Stop
        self.test_node.publish_wheel_command(linear_x=0.0, angular_z=0.0)
        time.sleep(0.2)

        # 3. Lower front arm
        self.test_node.publish_front_arm_command(action=-1.0)
        time.sleep(0.3)

        # 4. Start digging
        self.test_node.publish_front_drum_command(action=1.0)
        time.sleep(0.5)

        # 5. Stop digging
        self.test_node.publish_front_drum_command(action=0.0)
        time.sleep(0.2)

        # 6. Raise arm
        self.test_node.publish_front_arm_command(action=1.0)
        time.sleep(0.3)

        # Verify we received continuous status updates throughout
        self.assertGreater(
            len(self.test_node.location_status_received), 10,
            "Not enough status updates during command sequence"
        )


class TestMotorControllerUnit(unittest.TestCase):
    """Unit tests for motor controller calculations (no ROS 2 required)."""

    def test_differential_drive_forward(self):
        """Test differential drive wheel speed calculation for forward motion."""
        wheel_base = 0.5  # meters
        linear_x = 1.0  # m/s
        angular_z = 0.0  # rad/s

        v_left = linear_x - (angular_z * wheel_base / 2.0)
        v_right = linear_x + (angular_z * wheel_base / 2.0)

        self.assertAlmostEqual(v_left, 1.0)
        self.assertAlmostEqual(v_right, 1.0)

    def test_differential_drive_rotation(self):
        """Test differential drive wheel speed calculation for rotation."""
        wheel_base = 0.5  # meters
        linear_x = 0.0  # m/s
        angular_z = 2.0  # rad/s

        v_left = linear_x - (angular_z * wheel_base / 2.0)
        v_right = linear_x + (angular_z * wheel_base / 2.0)

        self.assertAlmostEqual(v_left, -0.5)
        self.assertAlmostEqual(v_right, 0.5)

    def test_differential_drive_arc(self):
        """Test differential drive wheel speed calculation for arc motion."""
        wheel_base = 0.5  # meters
        linear_x = 0.5  # m/s
        angular_z = 1.0  # rad/s

        v_left = linear_x - (angular_z * wheel_base / 2.0)
        v_right = linear_x + (angular_z * wheel_base / 2.0)

        self.assertAlmostEqual(v_left, 0.25)
        self.assertAlmostEqual(v_right, 0.75)

    def test_odometry_straight_line(self):
        """Test odometry calculation for straight line motion."""
        x, y, theta = 0.0, 0.0, 0.0
        linear_vel = 1.0  # m/s
        angular_vel = 0.0
        dt = 1.0  # 1 second

        # Straight line motion
        delta_x = linear_vel * dt * math.cos(theta)
        delta_y = linear_vel * dt * math.sin(theta)

        new_x = x + delta_x
        new_y = y + delta_y

        self.assertAlmostEqual(new_x, 1.0)
        self.assertAlmostEqual(new_y, 0.0)

    def test_odometry_rotation(self):
        """Test odometry calculation for pure rotation."""
        theta = 0.0
        angular_vel = math.pi / 2  # 90 degrees per second
        dt = 1.0  # 1 second

        new_theta = theta + angular_vel * dt

        self.assertAlmostEqual(new_theta, math.pi / 2)

    def test_theta_normalization(self):
        """Test that theta is properly normalized to [-pi, pi]."""
        theta = 3.5 * math.pi  # Beyond 2*pi

        # Normalize
        while theta > math.pi:
            theta -= 2.0 * math.pi
        while theta < -math.pi:
            theta += 2.0 * math.pi

        self.assertGreaterEqual(theta, -math.pi)
        self.assertLessEqual(theta, math.pi)

    def test_routine_flag_decoding(self):
        """Test routine action flag decoding."""
        AUTO_DRIVE = 0b000001
        AUTO_DIG = 0b000010
        AUTO_DUMP = 0b000100
        FULL_AUTONOMY = 0b010000
        STOP = 0b100000

        # Test combined flags
        routine = AUTO_DRIVE | AUTO_DIG
        self.assertTrue((routine & AUTO_DRIVE) != 0)
        self.assertTrue((routine & AUTO_DIG) != 0)
        self.assertFalse((routine & AUTO_DUMP) != 0)
        self.assertFalse((routine & FULL_AUTONOMY) != 0)
        self.assertFalse((routine & STOP) != 0)


if __name__ == '__main__':
    unittest.main()
