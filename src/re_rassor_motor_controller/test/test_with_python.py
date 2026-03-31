#!/usr/bin/env python3
# Copyright 2025 UCF RE-RASSOR
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""Wheel commander node for manual RE-RASSOR testing."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class WheelCommander(Node):
    """Publishes wheel commands for manual motor controller testing."""

    def __init__(self):
        """Initialize the WheelCommander node."""
        super().__init__('wheel_commander')
        # Publisher to the motor controller
        self.pub = self.create_publisher(Twist, '/ezrassor/wheel_instructions', 10)
        self.get_logger().info('Wheel Commander Node started')

    def send_command(self, linear_x=0.0, angular_z=0.0, duration=1.0):
        """Send a wheel command for the given duration.

        :param linear_x: forward/backward speed
        :param angular_z: rotational speed
        :param duration: how long to keep sending the command (seconds)
        """
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z

        start = time.time()
        rate = self.create_rate(10)  # 10 Hz
        while time.time() - start < duration:
            self.pub.publish(msg)
            self.get_logger().info(f'Publishing: linear_x={linear_x}, angular_z={angular_z}')
            rate.sleep()


def main(args=None):
    """Run the wheel commander node."""
    rclpy.init(args=args)
    node = WheelCommander()

    try:
        # Example: move forward at 0.5 m/s for 2 seconds
        node.send_command(linear_x=0.5, angular_z=0.0, duration=2.0)

        # Example: turn in place at 0.3 rad/s for 1.5 seconds
        node.send_command(linear_x=0.0, angular_z=0.3, duration=1.5)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
