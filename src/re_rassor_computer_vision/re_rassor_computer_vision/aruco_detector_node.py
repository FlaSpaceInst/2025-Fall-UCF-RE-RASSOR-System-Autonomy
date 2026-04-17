"""
RE-RASSOR ArUco Detector Node
─────────────────────────────
Subscribes to the rover RGB camera, detects ArUco markers, and publishes:
  /re_rassor/vision/annotated_image  (sensor_msgs/Image)  — frame with bounding boxes drawn
  /re_rassor/vision/detections       (std_msgs/String)    — JSON detection report

Based on 2023-RE-RASSOR-Extension cameraController / arucoGenerator by Noah Gregory et al.

Parameters (all ROS params):
  image_topic          (string, default '/camera/color/image_raw')
  marker_dict          (string, default 'DICT_5X5_100')
  detection_timeout_sec (float, default 7.5)  — seconds without a hit before resetting report
"""

import json
import time

import cv2
import numpy as np
import rclpy
import rclpy.node
from sensor_msgs.msg import Image
from std_msgs.msg import String

ARUCO_DICTS = {
    'DICT_4X4_50':          cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100':         cv2.aruco.DICT_4X4_100,
    'DICT_4X4_250':         cv2.aruco.DICT_4X4_250,
    'DICT_4X4_1000':        cv2.aruco.DICT_4X4_1000,
    'DICT_5X5_50':          cv2.aruco.DICT_5X5_50,
    'DICT_5X5_100':         cv2.aruco.DICT_5X5_100,
    'DICT_5X5_250':         cv2.aruco.DICT_5X5_250,
    'DICT_5X5_1000':        cv2.aruco.DICT_5X5_1000,
    'DICT_6X6_50':          cv2.aruco.DICT_6X6_50,
    'DICT_6X6_100':         cv2.aruco.DICT_6X6_100,
    'DICT_6X6_250':         cv2.aruco.DICT_6X6_250,
    'DICT_6X6_1000':        cv2.aruco.DICT_6X6_1000,
    'DICT_7X7_50':          cv2.aruco.DICT_7X7_50,
    'DICT_7X7_100':         cv2.aruco.DICT_7X7_100,
    'DICT_7X7_250':         cv2.aruco.DICT_7X7_250,
    'DICT_7X7_1000':        cv2.aruco.DICT_7X7_1000,
    'DICT_ARUCO_ORIGINAL':  cv2.aruco.DICT_ARUCO_ORIGINAL,
    'DICT_APRILTAG_16h5':   cv2.aruco.DICT_APRILTAG_16h5,
    'DICT_APRILTAG_25h9':   cv2.aruco.DICT_APRILTAG_25h9,
    'DICT_APRILTAG_36h10':  cv2.aruco.DICT_APRILTAG_36h10,
    'DICT_APRILTAG_36h11':  cv2.aruco.DICT_APRILTAG_36h11,
}


class ArucoDetectorNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('marker_dict', 'DICT_5X5_100')
        self.declare_parameter('detection_timeout_sec', 7.5)

        image_topic  = self.get_parameter('image_topic').get_parameter_value().string_value
        dict_key     = self.get_parameter('marker_dict').get_parameter_value().string_value
        self._timeout = self.get_parameter('detection_timeout_sec').get_parameter_value().double_value

        aruco_dict   = cv2.aruco.getPredefinedDictionary(
            ARUCO_DICTS.get(dict_key, cv2.aruco.DICT_5X5_100))
        aruco_params = cv2.aruco.DetectorParameters()

        # OpenCV >= 4.7 uses ArucoDetector class; older versions use the
        # legacy detectMarkers() free function directly.
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self._detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            self._detect_fn    = self._detector.detectMarkers
        else:
            self._aruco_dict   = aruco_dict
            self._aruco_params = aruco_params
            self._detect_fn    = self._legacy_detect

        self._annotated_pub = self.create_publisher(
            Image, '/re_rassor/vision/annotated_image', 1)
        self._detection_pub = self.create_publisher(
            String, '/re_rassor/vision/detections', 10)

        self._sub = self.create_subscription(
            Image, image_topic, self._image_cb, 1)

        self._last_detection_time = 0.0
        self._last_report = ''

        self.get_logger().info(
            f'ArUco detector ready — topic: {image_topic}, dict: {dict_key}')

    def _legacy_detect(self, frame):
        """Adapter for OpenCV < 4.7 detectMarkers free function."""
        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame, self._aruco_dict, parameters=self._aruco_params)
        return corners, ids, rejected

    # ── helpers ──────────────────────────────────────────────────────────────

    def _ros_to_bgr(self, msg: Image):
        """Convert a sensor_msgs/Image to an OpenCV BGR ndarray."""
        raw = bytes(msg.data)
        enc = msg.encoding
        if enc in ('rgb8',):
            arr = np.frombuffer(raw, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if enc in ('bgr8',):
            return np.frombuffer(raw, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        if enc in ('mono8', 'gray8'):
            gray = np.frombuffer(raw, dtype=np.uint8).reshape((msg.height, msg.width))
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if enc in ('bgra8',):
            arr = np.frombuffer(raw, dtype=np.uint8).reshape((msg.height, msg.width, 4))
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        if enc in ('rgba8',):
            arr = np.frombuffer(raw, dtype=np.uint8).reshape((msg.height, msg.width, 4))
            return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        raise ValueError(f'Unsupported encoding: {enc}')

    def _bgr_to_ros(self, frame: np.ndarray, header) -> Image:
        """Pack an OpenCV BGR ndarray into a sensor_msgs/Image."""
        out = Image()
        out.header   = header
        out.height   = frame.shape[0]
        out.width    = frame.shape[1]
        out.encoding = 'bgr8'
        out.step     = frame.shape[1] * 3
        out.data     = frame.tobytes()
        return out

    # ── main callback ─────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        try:
            frame = self._ros_to_bgr(msg)
        except ValueError as e:
            self.get_logger().warning(str(e), throttle_duration_sec=10.0)
            return
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        corners, ids, _ = self._detect_fn(frame)
        annotated = frame.copy()
        now = time.time()

        cx, cy = 0, 0
        if ids is not None and len(ids) > 0:
            self._last_detection_time = now
            for marker_corners, mid in zip(corners, ids.flatten()):
                pts = marker_corners.reshape((4, 2)).astype(int)
                tl, tr, br, bl = pts[0], pts[1], pts[2], pts[3]

                cv2.line(annotated, tuple(tl), tuple(tr), (0, 255, 0), 2)
                cv2.line(annotated, tuple(tr), tuple(br), (0, 255, 0), 2)
                cv2.line(annotated, tuple(br), tuple(bl), (0, 255, 0), 2)
                cv2.line(annotated, tuple(bl), tuple(tl), (0, 255, 0), 2)

                cx = int((tl[0] + br[0]) / 2)
                cy = int((tl[1] + br[1]) / 2)
                cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(annotated, str(mid),
                            (tl[0], tl[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            report = json.dumps({
                'detection': str(int(ids[0][0])),
                'time':      now,
                'x':         cx,
                'y':         cy,
                'z':         0,   # depth unavailable from RGB only
            })
            if report != self._last_report:
                self._last_report = report
                det_msg = String()
                det_msg.data = report
                self._detection_pub.publish(det_msg)
                self.get_logger().info(
                    f'ArUco id={ids[0][0]} at ({cx},{cy})')

        elif (now - self._last_detection_time) > self._timeout:
            report = json.dumps({'detection': 'false', 'time': now})
            if report != self._last_report:
                self._last_report = report
                det_msg = String()
                det_msg.data = report
                self._detection_pub.publish(det_msg)

        self._annotated_pub.publish(self._bgr_to_ros(annotated, msg.header))


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
