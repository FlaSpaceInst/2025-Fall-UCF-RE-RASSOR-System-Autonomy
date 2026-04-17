#!/usr/bin/env python3
"""Publish a blank 10x10m occupancy grid to /map with transient_local QoS."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

WIDTH_M  = 10.0   # metres
RES      = 0.05   # metres/cell
CELLS    = int(WIDTH_M / RES)  # 200


class FakeMapPublisher(Node):
    def __init__(self):
        super().__init__('fake_map_publisher')

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(OccupancyGrid, '/map', qos)

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()

        msg.info.resolution        = RES
        msg.info.width             = CELLS
        msg.info.height            = CELLS
        msg.info.origin.position.x = -WIDTH_M / 2.0   # centre on origin
        msg.info.origin.position.y = -WIDTH_M / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = [0] * (CELLS * CELLS)   # 0 = free

        self.pub.publish(msg)
        self.get_logger().info(
            f'Blank {WIDTH_M}x{WIDTH_M} m map published to /map '
            f'({CELLS}x{CELLS} cells @ {RES} m/cell)'
        )


def main():
    rclpy.init()
    node = FakeMapPublisher()
    # spin briefly so transient_local latches the message for late subscribers
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
