#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

IMAGE_PATH = None  # set this to the path of your test image e.g. "/path/to/image.jpg"

class StaticImagePublisher(Node):
    def __init__(self):
        super().__init__('static_image_publisher')
        self._pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self._timer = self.create_timer(0.1, self._publish)

	if IMAGE_PATH is None:
            raise RuntimeError(
                'IMAGE_PATH is not set. '
                'Edit scripts/image_publisher.py and set IMAGE_PATH to your image path.'
            )

        frame = cv2.imread(IMAGE_PATH)
        if frame is None:
            raise RuntimeError(f'Could not load image: {IMAGE_PATH}')
        frame = cv2.resize(frame, (640, 480))
        self._msg = Image()
        self._msg.height   = frame.shape[0]
        self._msg.width    = frame.shape[1]
        self._msg.encoding = 'bgr8'
        self._msg.step     = frame.shape[1] * 3
        self._msg.data     = frame.tobytes()
        self.get_logger().info(f'Publishing {IMAGE_PATH} at 10 Hz')

    def _publish(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)

def main(args=None):
    rclpy.init(args=args)
    node = StaticImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
