#!/usr/bin/env python3
"""
RE-RASSOR YOLO Obstacle Detector
----------------------------------
Subscribes to the Astra Pro color and depth streams, runs YOLO inference,
and publishes detected obstacles as YoloMsgArray with distance data.

Subscriptions:
  /camera/color/image_raw     (sensor_msgs/Image)
  /camera/depth/image_raw     (sensor_msgs/Image)
  /camera/depth/camera_info   (sensor_msgs/CameraInfo)

Publications:
  /re_rassor/vision/yolo_detections   (re_rassor_interfaces/YoloMsgArray)
  /re_rassor/vision/annotated_image   (sensor_msgs/Image)
"""

import math
import time

import cv2
import numpy as np
import rclpy
import rclpy.node
from sensor_msgs.msg import Image, CameraInfo
from re_rassor_interfaces.msg import YoloMsg, YoloMsgArray

try:
    from ultralytics import YOLO
except ImportError:
    raise ImportError(
        "ultralytics not installed.\n"
        "Run: pip3 install ultralytics --break-system-packages"
    )

DEFAULT_MODEL     = 'yolov8n.pt'
DEFAULT_CONF      = 0.4
DEFAULT_IMG_SIZE  = 640
DEFAULT_PATCH     = 7
DEFAULT_MIN_DEPTH = 0.3
DEFAULT_MAX_DEPTH = 6.0


class YoloDetectorNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('model_path',  DEFAULT_MODEL)
        self.declare_parameter('confidence',  DEFAULT_CONF)
        self.declare_parameter('img_size',    DEFAULT_IMG_SIZE)
        self.declare_parameter('patch_size',  DEFAULT_PATCH)
        self.declare_parameter('min_depth_m', DEFAULT_MIN_DEPTH)
        self.declare_parameter('max_depth_m', DEFAULT_MAX_DEPTH)

        self._conf    = self.get_parameter('confidence').value
        self._size    = self.get_parameter('img_size').value
        self._patch   = self.get_parameter('patch_size').value
        self._min_d   = self.get_parameter('min_depth_m').value
        self._max_d   = self.get_parameter('max_depth_m').value
        model_path    = self.get_parameter('model_path').value

        # Camera intrinsics — filled in once from /camera/depth/camera_info
        self._fx = None
        self._fy = None
        self._cx = None
        self._cy = None
        self._img_width  = None

        # Latest depth image — updated every frame
        self._depth_img = None

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self._model = YOLO(model_path)
        self.get_logger().info('YOLO model ready')

        # Subscriptions
        self.create_subscription(
            Image, '/camera/color/image_raw', self._cb_color, 10)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._cb_depth, 10)
        self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self._cb_info, 10)

        # Publishers
        self._pub_detections = self.create_publisher(
            YoloMsgArray, '/re_rassor/vision/yolo_detections', 10)
        self._pub_annotated = self.create_publisher(
            Image, '/re_rassor/vision/annotated_image', 1)

        self.get_logger().info(
            'YOLO detector ready\n'
            '  Subscribing : /camera/color/image_raw\n'
            '  Subscribing : /camera/depth/image_raw\n'
            '  Publishing  : /re_rassor/vision/yolo_detections\n'
            '  Publishing  : /re_rassor/vision/annotated_image'
        )

    # ── camera info ───────────────────────────────────────────────────────────

    def _cb_info(self, msg: CameraInfo):
        if self._fx is not None:
            return
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]
        self._img_width = msg.width
        self.get_logger().info(
            f'Camera intrinsics received: '
            f'fx={self._fx:.1f} fy={self._fy:.1f} '
            f'cx={self._cx:.1f} cy={self._cy:.1f}'
        )

    # ── depth image ───────────────────────────────────────────────────────────

    def _cb_depth(self, msg: Image):
        raw = bytes(msg.data)
        self._depth_img = np.frombuffer(raw, dtype=np.uint16).reshape(
            (msg.height, msg.width))

    # ── image conversion helpers (same pattern as aruco_detector_node) ────────

    def _ros_to_bgr(self, msg: Image):
        raw = bytes(msg.data)
        enc = msg.encoding
        if enc == 'rgb8':
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if enc == 'bgr8':
            return np.frombuffer(raw, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
        if enc in ('mono8', 'gray8'):
            gray = np.frombuffer(raw, dtype=np.uint8).reshape(
                (msg.height, msg.width))
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        raise ValueError(f'Unsupported encoding: {enc}')

    def _bgr_to_ros(self, frame: np.ndarray, header) -> Image:
        out = Image()
        out.header   = header
        out.height   = frame.shape[0]
        out.width    = frame.shape[1]
        out.encoding = 'bgr8'
        out.step     = frame.shape[1] * 3
        out.data     = frame.tobytes()
        return out

    # ── depth sampling ────────────────────────────────────────────────────────

    def _sample_depth(self, u: int, v: int) -> float:
        """Return median depth in metres at pixel (u, v). -1.0 if unavailable."""
        if self._depth_img is None:
            return -1.0
        h, w = self._depth_img.shape
        half = self._patch // 2
        u0, u1 = max(0, u - half), min(w, u + half + 1)
        v0, v1 = max(0, v - half), min(h, v + half + 1)
        patch  = self._depth_img[v0:v1, u0:u1].astype(np.float32) / 1000.0
        valid  = patch[(patch >= self._min_d) & (patch <= self._max_d)]
        if valid.size == 0:
            return -1.0
        return float(np.median(valid))

    # ── angle helpers ─────────────────────────────────────────────────────────

    def _pixel_to_angle(self, u: int) -> float:
        """Horizontal angle in radians of pixel column u from camera centre."""
        if self._fx is None:
            return 0.0
        return math.atan2(u - self._cx, self._fx)

    # ── main color callback ───────────────────────────────────────────────────

    def _cb_color(self, msg: Image):
        try:
            frame = self._ros_to_bgr(msg)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=10.0)
            return

        results = self._model.predict(
            source=frame,
            conf=self._conf,
            imgsz=self._size,
            verbose=False,
        )

        annotated  = frame.copy()
        yolo_array = YoloMsgArray()
        yolo_array.stamp = msg.header.stamp

        img_h, img_w = frame.shape[:2]

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                w  = x2 - x1
                h  = y2 - y1

                label    = self._model.names[int(box.cls)]
                conf     = float(box.conf)
                distance = self._sample_depth(cx, cy)

                # Normalised coordinates (0.0 – 1.0)
                norm_x = cx / img_w
                norm_y = cy / img_h
                norm_w = w  / img_w
                norm_h = h  / img_h

                # Horizontal angular extent of the bounding box
                min_angle = self._pixel_to_angle(x1)
                max_angle = self._pixel_to_angle(x2)

                yolo_msg             = YoloMsg()
                yolo_msg.x           = float(cx)
                yolo_msg.y           = float(cy)
                yolo_msg.width       = float(w)
                yolo_msg.height      = float(h)
                yolo_msg.norm_x      = norm_x
                yolo_msg.norm_y      = norm_y
                yolo_msg.norm_width  = norm_w
                yolo_msg.norm_height = norm_h
                yolo_msg.min_angle   = min_angle
                yolo_msg.max_angle   = max_angle
                yolo_msg.name        = label
                yolo_msg.distance    = distance if distance > 0 else -1.0
                yolo_array.yolo_objects.append(yolo_msg)

                # Draw bounding box and label on annotated image
                dist_str = f'{distance:.2f}m' if distance > 0 else 'no depth'
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    annotated,
                    f'{label} {conf:.2f} {dist_str}',
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                )
                cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)

                self.get_logger().debug(
                    f'{label} ({conf:.2f}) at ({cx},{cy}) '
                    f'distance={dist_str}'
                )

        self._pub_detections.publish(yolo_array)
        self._pub_annotated.publish(
            self._bgr_to_ros(annotated, msg.header))

        if yolo_array.yolo_objects:
            self.get_logger().info(
                f'{len(yolo_array.yolo_objects)} detection(s) published'
            )


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
