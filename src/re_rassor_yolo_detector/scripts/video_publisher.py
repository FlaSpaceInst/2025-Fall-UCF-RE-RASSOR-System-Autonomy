#!/usr/bin/env python3
import cv2
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

IMAGE_FOLDER = None  # set this to the path of your folder of test images e.g. "/path/to/image/directory/"

class FolderImagePublisher(Node):
    def __init__(self):
        super().__init__('folder_image_publisher')
        self._pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self._timer = self.create_timer(2.0, self._publish)

        if IMAGE_FOLDER is None:
            raise RuntimeError(
                'IMAGE_FOLDER is not set. '
                'Edit scripts/video_publisher.py and set IMAGE_FOLDER to your image folder path.'
            )

        extensions = ('.jpg', '.jpeg', '.png')
        self._images = sorted([
            os.path.join(IMAGE_FOLDER, f)
            for f in os.listdir(IMAGE_FOLDER)
            if f.lower().endswith(extensions)
        ])
        if not self._images:
            raise RuntimeError(f'No images found in {IMAGE_FOLDER}')
        self._index = 0
        self.get_logger().info(f'Found {len(self._images)} images in {IMAGE_FOLDER}')

    def _publish(self):
        path = self._images[self._index]
        frame = cv2.imread(path)
        if frame is None:
            return
        frame = cv2.resize(frame, (640, 480))
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height   = frame.shape[0]
        msg.width    = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.step     = frame.shape[1] * 3
        msg.data     = frame.tobytes()
        self._pub.publish(msg)
        self.get_logger().info(f'Published {os.path.basename(path)} ({self._index + 1}/{len(self._images)})')
        self._index = (self._index + 1) % len(self._images)

def main(args=None):
    rclpy.init(args=args)
    node = FolderImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
