#!/usr/bin/env python3
"""
synthetic_camera_publisher.py
─────────────────────────────────────────────────────────────────────────────
Replaces the Gazebo + ros_gz_bridge camera pipeline in the e2e test.
Publishes at 10 Hz immediately on startup — no rendering engine required.

Topics published:
  /camera/color/image_raw     sensor_msgs/Image      RGB8    640×480
  /camera/color/camera_info   sensor_msgs/CameraInfo Astra Pro intrinsics
  /camera/depth/image_raw     sensor_msgs/Image      32FC1   640×480
  /camera/depth/camera_info   sensor_msgs/CameraInfo Astra Pro intrinsics
  /camera/depth/points        sensor_msgs/PointCloud2 XYZ float32
  /odometry/wheel             nav_msgs/Odometry      stationary at origin

Scene:
  depth map: 2.0 m background, 1.0 m box obstacle (rows 100-200, cols 200-440)
  color image: 40-px checkerboard for rich visual features (rtabmap / rgbd_odometry)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

ASTRA_W  = 640
ASTRA_H  = 480
ASTRA_FX = 554.26
ASTRA_FY = 554.26
ASTRA_CX = 320.0
ASTRA_CY = 240.0

SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


def _make_scene():
    depth = np.full((ASTRA_H, ASTRA_W), 2.0, dtype=np.float32)
    depth[100:200, 200:440] = 1.0
    rows = np.arange(ASTRA_H)[:, None]
    cols = np.arange(ASTRA_W)[None, :]
    gray = (((rows // 40 + cols // 40) % 2 == 0) * 255).astype(np.uint8)
    color = np.stack([gray, gray, gray], axis=2)
    return depth, color


class SyntheticCameraPublisher(Node):
    def __init__(self):
        super().__init__("synthetic_camera_publisher")

        self._depth_map, self._color_img = _make_scene()

        # Pre-compute constant byte blobs
        self._depth_bytes = self._depth_map.tobytes()
        self._color_bytes = self._color_img.tobytes()
        self._pc_data, self._pc_n = self._make_pointcloud_data()

        self._depth_pub  = self.create_publisher(
            Image,       "/camera/depth/image_raw",  SENSOR_QOS)
        self._color_pub  = self.create_publisher(
            Image,       "/camera/color/image_raw",  SENSOR_QOS)
        self._depth_info = self.create_publisher(
            CameraInfo,  "/camera/depth/camera_info", 10)
        self._color_info = self.create_publisher(
            CameraInfo,  "/camera/color/camera_info", 10)
        self._pc_pub     = self.create_publisher(
            PointCloud2, "/camera/depth/points",     SENSOR_QOS)
        self._odom_pub   = self.create_publisher(
            Odometry,    "/odometry/wheel",           10)

        self.create_timer(0.1, self._publish)   # 10 Hz
        self.get_logger().info(
            "SyntheticCameraPublisher ready — publishing camera + odometry at 10 Hz")

    # ── helpers ──────────────────────────────────────────────────────────────

    def _make_pointcloud_data(self):
        stride = 4
        rs = np.arange(0, ASTRA_H, stride)
        cs = np.arange(0, ASTRA_W, stride)
        c, r = np.meshgrid(cs, rs)
        z = self._depth_map[r, c].astype(np.float32)
        x = ((c - ASTRA_CX) * z / ASTRA_FX).astype(np.float32)
        y = ((r - ASTRA_CY) * z / ASTRA_FY).astype(np.float32)
        pts = np.stack([x.ravel(), y.ravel(), z.ravel()], axis=1)
        return pts.tobytes(), pts.shape[0]

    def _camera_info(self, stamp, frame_id: str) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp     = stamp
        msg.header.frame_id  = frame_id
        msg.width            = ASTRA_W
        msg.height           = ASTRA_H
        msg.k                = [ASTRA_FX, 0.0, ASTRA_CX,
                                  0.0, ASTRA_FY, ASTRA_CY,
                                  0.0, 0.0, 1.0]
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [ASTRA_FX, 0.0, ASTRA_CX, 0.0,
                  0.0, ASTRA_FY, ASTRA_CY, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return msg

    # ── timer callback ───────────────────────────────────────────────────────

    def _publish(self):
        now = self.get_clock().now().to_msg()

        depth = Image()
        depth.header.stamp    = now
        depth.header.frame_id = "camera_depth_optical_frame"
        depth.width    = ASTRA_W
        depth.height   = ASTRA_H
        depth.encoding = "32FC1"
        depth.step     = ASTRA_W * 4
        depth.data     = self._depth_bytes
        self._depth_pub.publish(depth)

        color = Image()
        color.header.stamp    = now
        color.header.frame_id = "camera_color_optical_frame"
        color.width    = ASTRA_W
        color.height   = ASTRA_H
        color.encoding = "rgb8"
        color.step     = ASTRA_W * 3
        color.data     = self._color_bytes
        self._color_pub.publish(color)

        self._depth_info.publish(
            self._camera_info(now, "camera_depth_optical_frame"))
        self._color_info.publish(
            self._camera_info(now, "camera_color_optical_frame"))

        pc = PointCloud2()
        pc.header.stamp    = now
        pc.header.frame_id = "camera_depth_optical_frame"
        pc.height     = 1
        pc.width      = self._pc_n
        pc.fields     = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        pc.is_bigendian = False
        pc.point_step   = 12
        pc.row_step     = 12 * self._pc_n
        pc.data         = self._pc_data
        pc.is_dense     = True
        self._pc_pub.publish(pc)

        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.pose.pose.orientation.w = 1.0
        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticCameraPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
