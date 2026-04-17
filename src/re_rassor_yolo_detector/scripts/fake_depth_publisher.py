#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

FAKE_DEPTH_METRES = 2.0 #fake depth data to test sensor fusion
WIDTH  = 640
HEIGHT = 480

class FakeDepthPublisher(Node):
    def __init__(self):
        super().__init__('fake_depth_publisher')
        self._depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self._info_pub  = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        self.create_timer(0.1, self._publish)
        depth_mm = int(FAKE_DEPTH_METRES * 1000)
        arr = np.full((HEIGHT, WIDTH), depth_mm, dtype=np.uint16)
        self._depth_msg          = Image()
        self._depth_msg.height   = HEIGHT
        self._depth_msg.width    = WIDTH
        self._depth_msg.encoding = '16UC1'
        self._depth_msg.step     = WIDTH * 2
        self._depth_msg.data     = arr.tobytes()
        self._info_msg                 = CameraInfo()
        self._info_msg.height          = HEIGHT
        self._info_msg.width           = WIDTH
        self._info_msg.distortion_model = 'plumb_bob'
        self._info_msg.k               = [570.0, 0.0, 320.0, 0.0, 570.0, 240.0, 0.0, 0.0, 1.0]
        self._info_msg.header.frame_id = 'camera_depth_optical_frame'
        self.get_logger().info(f'Publishing fake depth: {FAKE_DEPTH_METRES}m')

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        self._depth_msg.header.stamp = stamp
        self._info_msg.header.stamp  = stamp
        self._depth_pub.publish(self._depth_msg)
        self._info_pub.publish(self._info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeDepthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
