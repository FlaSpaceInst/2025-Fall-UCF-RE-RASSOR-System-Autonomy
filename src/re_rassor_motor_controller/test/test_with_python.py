#!/usr/bin/env python3
"""Wheel commander utility node for manual motor controller testing."""

import time

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node


class WheelCommander(Node):
    """ROS2 node that publishes wheel commands to the motor controller."""

    def __init__(self):
        """Initialize the wheel commander publisher."""
        super().__init__('wheel_commander')
        # Publisher to the motor controller
        self.pub = self.create_publisher(Twist, '/ezrassor/wheel_instructions', 10)
        self.get_logger().info('Wheel Commander Node started')

    def send_command(self, linear_x=0.0, angular_z=0.0, duration=1.0):
        """Send a wheel command.

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
    """Entry point for the wheel commander node."""
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
