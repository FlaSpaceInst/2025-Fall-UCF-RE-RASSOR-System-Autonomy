#!/usr/bin/env python3
"""
End-to-End Integration Tests for Controller Server + Motor Controller.

These tests verify the complete communication flow:
HTTP Request -> Controller Server -> ROS 2 Topics -> Motor Controller -> Location Feedback

Prerequisites:
- Both ezrassor_controller_server and motor_controller nodes must be running
- Flask server must be accessible at http://localhost:5000

To run:
    # Terminal 1: Start controller server
    ros2 run ezrassor_controller_server controller_server

    # Terminal 2: Start motor controller
    ros2 run re_rassor_traffic_controller motor_controller

    # Terminal 3: Run tests
    pytest test_integration_e2e.py -v
"""

import unittest
import time
import threading
import json

import requests
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from re_rassor_interfaces.msg import LocationStatus
from std_msgs.msg import Int8

# Controller server URL
CONTROLLER_SERVER_URL = "http://localhost:5000/"


class E2ETestNode(Node):
    """Test node for monitoring motor controller outputs."""

    def __init__(self):
        super().__init__('e2e_test_node')

        self.location_status_msgs = []
        self.controller_status_msgs = []

        self.location_sub = self.create_subscription(
            LocationStatus,
            '/location_status',
            lambda msg: self.location_status_msgs.append(msg),
            10
        )

        self.status_sub = self.create_subscription(
            Int8,
            '/motor_controller/status',
            lambda msg: self.controller_status_msgs.append(msg),
            10
        )

    def clear(self):
        """Clear all received messages."""
        self.location_status_msgs.clear()
        self.controller_status_msgs.clear()


class TestE2EIntegration(unittest.TestCase):
    """End-to-End integration tests."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 and test infrastructure."""
        rclpy.init()
        cls.test_node = E2ETestNode()
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.test_node)

        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

        # Wait for connections
        time.sleep(1.0)

        # Check if controller server is running
        try:
            requests.post(CONTROLLER_SERVER_URL, json={}, timeout=2)
            cls.server_available = True
        except requests.exceptions.RequestException:
            cls.server_available = False
            print("\nWARNING: Controller server not available at", CONTROLLER_SERVER_URL)
            print("Some E2E tests will be skipped.\n")

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2."""
        cls.executor.shutdown()
        cls.test_node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        """Clear message buffers before each test."""
        self.test_node.clear()
        time.sleep(0.1)

    def send_command(self, command_dict):
        """Send a command to the controller server via HTTP POST."""
        if not self.server_available:
            self.skipTest("Controller server not available")

        response = requests.post(
            CONTROLLER_SERVER_URL,
            json=command_dict,
            headers={'Content-Type': 'application/json'},
            timeout=5
        )
        return response

    def test_e2e_wheel_forward(self):
        """Test wheel command from HTTP through to motor controller."""
        command = {
            "wheel_action": {
                "linear_x": "0.5",
                "angular_z": "0.0"
            }
        }

        response = self.send_command(command)
        self.assertEqual(response.status_code, 200)

        # Wait for motor controller to process
        time.sleep(0.5)

        # Verify motor controller received and processed the command
        self.assertGreater(
            len(self.test_node.location_status_msgs), 0,
            "Motor controller did not publish location status after wheel command"
        )

    def test_e2e_wheel_rotation(self):
        """Test rotation command flow."""
        command = {
            "wheel_action": {
                "linear_x": "0.0",
                "angular_z": "1.0"
            }
        }

        response = self.send_command(command)
        self.assertEqual(response.status_code, 200)

        time.sleep(0.5)

        self.assertGreater(len(self.test_node.location_status_msgs), 0)

    def test_e2e_front_arm_raise(self):
        """Test front arm raise command flow."""
        command = {
            "front_arm_action": "RAISE"
        }

        response = self.send_command(command)
        self.assertEqual(response.status_code, 200)

        time.sleep(0.3)
        self.assertGreater(len(self.test_node.location_status_msgs), 0)

    def test_e2e_front_arm_lower(self):
        """Test front arm lower command flow."""
        command = {
            "front_arm_action": "LOWER"
        }

        response = self.send_command(command)
        self.assertEqual(response.status_code, 200)

        time.sleep(0.3)
        self.assertGreater(len(self.test_node.location_status_msgs), 0)

    def test_e2e_back_arm_commands(self):
        """Test back arm command flow."""
        for action in ["RAISE", "STOP", "LOWER"]:
            self.test_node.clear()
            command = {"back_arm_action": action}

            response = self.send_command(command)
            self.assertEqual(response.status_code, 200)

            time.sleep(0.2)

    def test_e2e_front_drum_dig(self):
        """Test front drum dig command flow."""
        command = {
            "front_drum_action": "DIG"
        }

        response = self.send_command(command)
        self.assertEqual(response.status_code, 200)

        time.sleep(0.3)
        self.assertGreater(len(self.test_node.location_status_msgs), 0)

    def test_e2e_front_drum_dump(self):
        """Test front drum dump command flow."""
        command = {
            "front_drum_action": "DUMP"
        }

        response = self.send_command(command)
        self.assertEqual(response.status_code, 200)

        time.sleep(0.3)
        self.assertGreater(len(self.test_node.location_status_msgs), 0)

    def test_e2e_back_drum_commands(self):
        """Test back drum command flow."""
        for action in ["DIG", "STOP", "DUMP"]:
            self.test_node.clear()
            command = {"back_drum_action": action}

            response = self.send_command(command)
            self.assertEqual(response.status_code, 200)

            time.sleep(0.2)

    def test_e2e_routine_auto_drive(self):
        """Test AUTO_DRIVE routine command flow."""
        command = {
            "routine_action": "AUTO_DRIVE"
        }

        response = self.send_command(command)
        self.assertEqual(response.status_code, 200)

        time.sleep(0.3)

        # Check controller status shows active routine
        self.assertGreater(len(self.test_node.controller_status_msgs), 0)

    def test_e2e_routine_full_autonomy(self):
        """Test FULL_AUTONOMY routine command flow."""
        command = {
            "routine_action": "FULL_AUTONOMY"
        }

        response = self.send_command(command)
        self.assertEqual(response.status_code, 200)

        time.sleep(0.3)
        self.assertGreater(len(self.test_node.controller_status_msgs), 0)

    def test_e2e_routine_stop(self):
        """Test STOP routine command flow."""
        command = {
            "routine_action": "STOP"
        }

        response = self.send_command(command)
        self.assertEqual(response.status_code, 200)

        time.sleep(0.3)
        self.assertGreater(len(self.test_node.controller_status_msgs), 0)

    def test_e2e_combined_commands(self):
        """Test multiple commands in single request."""
        command = {
            "wheel_action": {
                "linear_x": "0.3",
                "angular_z": "0.1"
            },
            "front_arm_action": "RAISE",
            "back_drum_action": "DIG"
        }

        response = self.send_command(command)
        self.assertEqual(response.status_code, 200)

        time.sleep(0.5)
        self.assertGreater(len(self.test_node.location_status_msgs), 0)

    def test_e2e_digging_sequence(self):
        """Test a complete digging operation sequence."""
        sequence = [
            # 1. Move to dig location
            {"wheel_action": {"linear_x": "0.5", "angular_z": "0.0"}},
            # 2. Stop
            {"wheel_action": {"linear_x": "0.0", "angular_z": "0.0"}},
            # 3. Lower front arm
            {"front_arm_action": "LOWER"},
            # 4. Start digging
            {"front_drum_action": "DIG"},
            # 5. Stop digging
            {"front_drum_action": "STOP"},
            # 6. Raise arm with payload
            {"front_arm_action": "RAISE"},
            # 7. Move to dump location
            {"wheel_action": {"linear_x": "0.3", "angular_z": "0.5"}},
            # 8. Stop
            {"wheel_action": {"linear_x": "0.0", "angular_z": "0.0"}},
            # 9. Dump payload
            {"front_drum_action": "DUMP"},
            # 10. Stop dump
            {"front_drum_action": "STOP"},
        ]

        for i, command in enumerate(sequence):
            self.test_node.clear()

            response = self.send_command(command)
            self.assertEqual(
                response.status_code, 200,
                f"Step {i+1} failed: {command}"
            )

            time.sleep(0.3)

    def test_e2e_invalid_command_rejected(self):
        """Test that invalid commands are rejected by controller server."""
        # Invalid arm action
        command = {
            "front_arm_action": "INVALID_ACTION"
        }

        response = self.send_command(command)
        self.assertEqual(response.status_code, 200)  # Server returns 200 with status 400 in body

        response_data = response.json()
        self.assertEqual(response_data.get('status'), 400)

    def test_e2e_unknown_key_rejected(self):
        """Test that unknown keys are rejected."""
        command = {
            "unknown_action": "something"
        }

        response = self.send_command(command)
        response_data = response.json()
        self.assertEqual(response_data.get('status'), 400)

    def test_e2e_position_tracking(self):
        """Test that position is properly tracked during movement."""
        self.test_node.clear()

        # Move forward for a while
        command = {
            "wheel_action": {
                "linear_x": "1.0",
                "angular_z": "0.0"
            }
        }

        self.send_command(command)
        time.sleep(1.0)

        # Stop
        stop_command = {
            "wheel_action": {
                "linear_x": "0.0",
                "angular_z": "0.0"
            }
        }
        self.send_command(stop_command)
        time.sleep(0.2)

        # Verify position changed
        self.assertGreater(len(self.test_node.location_status_msgs), 5)

        first_pos = self.test_node.location_status_msgs[0]
        last_pos = self.test_node.location_status_msgs[-1]

        # Position should have changed
        position_diff = abs(last_pos.position_x - first_pos.position_x)
        self.assertGreater(
            position_diff, 0.1,
            f"Position did not change enough: diff={position_diff}"
        )


class TestControllerServerDirectly(unittest.TestCase):
    """Tests for controller server HTTP interface (no ROS 2 required)."""

    @classmethod
    def setUpClass(cls):
        """Check server availability."""
        try:
            requests.post(CONTROLLER_SERVER_URL, json={}, timeout=2)
            cls.server_available = True
        except requests.exceptions.RequestException:
            cls.server_available = False

    def test_server_accepts_empty_request(self):
        """Test that server accepts empty request."""
        if not self.server_available:
            self.skipTest("Server not available")

        response = requests.post(
            CONTROLLER_SERVER_URL,
            json={},
            timeout=5
        )
        self.assertEqual(response.status_code, 200)

    def test_server_accepts_wheel_action(self):
        """Test wheel action format."""
        if not self.server_available:
            self.skipTest("Server not available")

        response = requests.post(
            CONTROLLER_SERVER_URL,
            json={"wheel_action": {"linear_x": "0.5", "angular_z": "0.0"}},
            timeout=5
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json().get('status'), 200)

    def test_server_validates_arm_actions(self):
        """Test that server validates arm action values."""
        if not self.server_available:
            self.skipTest("Server not available")

        # Valid actions
        for action in ["RAISE", "STOP", "LOWER"]:
            response = requests.post(
                CONTROLLER_SERVER_URL,
                json={"front_arm_action": action},
                timeout=5
            )
            self.assertEqual(
                response.json().get('status'), 200,
                f"Valid action {action} was rejected"
            )

    def test_server_validates_drum_actions(self):
        """Test that server validates drum action values."""
        if not self.server_available:
            self.skipTest("Server not available")

        for action in ["DIG", "STOP", "DUMP"]:
            response = requests.post(
                CONTROLLER_SERVER_URL,
                json={"front_drum_action": action},
                timeout=5
            )
            self.assertEqual(
                response.json().get('status'), 200,
                f"Valid action {action} was rejected"
            )

    def test_server_validates_routine_actions(self):
        """Test that server validates routine action values."""
        if not self.server_available:
            self.skipTest("Server not available")

        valid_routines = [
            "AUTO_DRIVE", "AUTO_DIG", "AUTO_DUMP",
            "AUTO_DOCK", "FULL_AUTONOMY", "STOP"
        ]

        for routine in valid_routines:
            response = requests.post(
                CONTROLLER_SERVER_URL,
                json={"routine_action": routine},
                timeout=5
            )
            self.assertEqual(
                response.json().get('status'), 200,
                f"Valid routine {routine} was rejected"
            )


if __name__ == '__main__':
    unittest.main()
