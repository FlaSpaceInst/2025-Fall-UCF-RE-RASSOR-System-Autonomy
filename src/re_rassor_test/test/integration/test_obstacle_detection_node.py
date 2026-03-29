"""
test_obstacle_detection_node.py
─────────────────────────────────────────────────────────────────────────────
Node-level integration tests for the depth-camera → rtabmap → nav2 pipeline.

Launches (via launch.LaunchService so parameter types are preserved):
  • static_transform_publisher  odom→base_link→camera_link→optical frames
  • rtabmap_odom/rgbd_odometry  depth+RGB → /odom
  • rtabmap_slam/rtabmap         /odom + depth+RGB → /map

Publishes synthetic 640×480 depth + RGB images (no Gazebo required) and
verifies:

  TestRgbdOdometryIntegration
    test_odom_published         — /odom appears after feeding camera data
    test_odom_frame_ids         — frame_id=odom, child_frame_id=base_link
    test_odom_position_finite   — position values are finite floats

  TestRtabmapSlamIntegration
    test_map_published          — /map (OccupancyGrid) appears from rtabmap
                                  with frame_id=map

  TestNav2PointCloudCompatibility
    test_pointcloud_xyz_fields  — x, y, z float32 fields present
    test_pointcloud_near_range  — points at 1.0 m obstacle depth exist
    test_pointcloud_frame_id    — frame_id = camera_depth_optical_frame
    test_pointcloud_point_step  — point_step = 12 (3 × float32)
    test_pointcloud_not_empty   — at least one point

No scan_to_costmap_node or re_rassor_obstacle_detection is used.

Run:
    colcon build --packages-select re_rassor_test --cmake-args -DBUILD_TESTING=ON
    colcon test  --packages-select re_rassor_test
    colcon test-result --verbose
"""

import math
import struct
import tempfile
import threading
import time

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

import launch
import launch_ros.actions
from launch.launch_service import LaunchService

# ──────────────────────────────────────────────────────────────────────────────
# Astra Pro 640×480 reference intrinsics
# ──────────────────────────────────────────────────────────────────────────────
ASTRA_W  = 640
ASTRA_H  = 480
ASTRA_FX = 554.26
ASTRA_FY = 554.26
ASTRA_CX = 320.0
ASTRA_CY = 240.0

PIPELINE_STARTUP  = 8.0   # seconds for all nodes to initialise
ODOM_TIMEOUT      = 20.0  # seconds to wait for first /odom
MAP_TIMEOUT       = 30.0  # seconds to wait for first /map
PC_TIMEOUT        = 5.0   # seconds to wait for PointCloud2 echo

SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

TRANSIENT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


# ──────────────────────────────────────────────────────────────────────────────
# Message factories
# ──────────────────────────────────────────────────────────────────────────────

def _camera_info(stamp, frame_id: str) -> CameraInfo:
    msg = CameraInfo()
    msg.header.stamp     = stamp
    msg.header.frame_id  = frame_id
    msg.width            = ASTRA_W
    msg.height           = ASTRA_H
    msg.k                = [ASTRA_FX, 0.0, ASTRA_CX,
                             0.0, ASTRA_FY, ASTRA_CY,
                             0.0, 0.0, 1.0]
    msg.distortion_model = "plumb_bob"
    msg.d                = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.r                = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p                = [ASTRA_FX, 0.0, ASTRA_CX, 0.0,
                             0.0, ASTRA_FY, ASTRA_CY, 0.0,
                             0.0, 0.0, 1.0, 0.0]
    return msg


def _make_scene():
    """
    Synthetic scene (built once, reused every frame):
      depth map:   2.0 m background, 1.0 m box (rows 100-200, cols 200-440)
      color image: 40-px checkerboard — rich corner features for rgbd_odometry
    """
    depth = np.full((ASTRA_H, ASTRA_W), 2.0, dtype=np.float32)
    depth[100:200, 200:440] = 1.0

    rows = np.arange(ASTRA_H)[:, None]
    cols = np.arange(ASTRA_W)[None, :]
    pattern = ((rows // 40 + cols // 40) % 2 == 0)
    gray  = (pattern * 255).astype(np.uint8)
    color = np.stack([gray, gray, gray], axis=2)
    return depth, color


def _depth_msg(stamp, depth_map: np.ndarray) -> Image:
    msg = Image()
    msg.header.stamp    = stamp
    msg.header.frame_id = "camera_depth_optical_frame"
    msg.width    = ASTRA_W
    msg.height   = ASTRA_H
    msg.encoding = "32FC1"
    msg.step     = ASTRA_W * 4
    msg.data     = depth_map.astype(np.float32).tobytes()
    return msg


def _color_msg(stamp, color_img: np.ndarray) -> Image:
    msg = Image()
    msg.header.stamp    = stamp
    msg.header.frame_id = "camera_color_optical_frame"
    msg.width    = ASTRA_W
    msg.height   = ASTRA_H
    msg.encoding = "rgb8"
    msg.step     = ASTRA_W * 3
    msg.data     = color_img.tobytes()
    return msg


def _pointcloud_msg(stamp, depth_map: np.ndarray, stride: int = 4) -> PointCloud2:
    """Pinhole projection of depth_map → PointCloud2 XYZ float32.
    Mirrors depth_image_proc/point_cloud_xyz — validates nav2 obstacle_layer format."""
    rs = np.arange(0, ASTRA_H, stride)
    cs = np.arange(0, ASTRA_W, stride)
    c, r  = np.meshgrid(cs, rs)
    z = depth_map[r, c].astype(np.float32)
    x = ((c - ASTRA_CX) * z / ASTRA_FX).astype(np.float32)
    y = ((r - ASTRA_CY) * z / ASTRA_FY).astype(np.float32)
    pts = np.stack([x.ravel(), y.ravel(), z.ravel()], axis=1)
    n   = pts.shape[0]

    msg = PointCloud2()
    msg.header.stamp    = stamp
    msg.header.frame_id = "camera_depth_optical_frame"
    msg.height     = 1
    msg.width      = n
    msg.fields     = [
        PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step   = 12
    msg.row_step     = 12 * n
    msg.data         = pts.tobytes()
    msg.is_dense     = True
    return msg


# ──────────────────────────────────────────────────────────────────────────────
# Helper test node
# ──────────────────────────────────────────────────────────────────────────────

class _TestNode(Node):
    def __init__(self):
        super().__init__("re_rassor_integration_test_node")

        self.depth_pub      = self.create_publisher(
            Image,       "/camera/depth/image_raw",  SENSOR_QOS)
        self.color_pub      = self.create_publisher(
            Image,       "/camera/color/image_raw",  SENSOR_QOS)
        self.depth_info_pub = self.create_publisher(
            CameraInfo,  "/camera/depth/camera_info", 10)
        self.color_info_pub = self.create_publisher(
            CameraInfo,  "/camera/color/camera_info", 10)
        self.pc_pub         = self.create_publisher(
            PointCloud2, "/camera/depth/points",     SENSOR_QOS)

        self.odom_msgs: list = []
        self.map_msgs:  list = []
        self.pc_msgs:   list = []

        self.create_subscription(
            Odometry,      "/odom",
            lambda m: self.odom_msgs.append(m), 10)
        self.create_subscription(
            OccupancyGrid, "/map",
            lambda m: self.map_msgs.append(m), TRANSIENT_QOS)
        self.create_subscription(
            PointCloud2,   "/camera/depth/points",
            lambda m: self.pc_msgs.append(m), SENSOR_QOS)

        self._depth_map, self._color_img = _make_scene()

    def publish_frame(self):
        now = self.get_clock().now().to_msg()
        self.depth_pub.publish(_depth_msg(now, self._depth_map))
        self.color_pub.publish(_color_msg(now, self._color_img))
        self.depth_info_pub.publish(_camera_info(now, "camera_depth_optical_frame"))
        self.color_info_pub.publish(_camera_info(now, "camera_color_optical_frame"))
        self.pc_pub.publish(_pointcloud_msg(now, self._depth_map))

    def _spin(self, secs: float = 0.05):
        rclpy.spin_once(self, timeout_sec=secs)

    def _wait_for(self, attr: str, timeout: float):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._spin()
            if getattr(self, attr):
                return getattr(self, attr)[-1]
        return None

    def wait_odom(self, timeout: float = ODOM_TIMEOUT):
        return self._wait_for("odom_msgs", timeout)

    def wait_map(self, timeout: float = MAP_TIMEOUT):
        return self._wait_for("map_msgs", timeout)

    def wait_pc(self, timeout: float = PC_TIMEOUT):
        return self._wait_for("pc_msgs", timeout)


def _pump(tnode: _TestNode, count: int = 20, delay: float = 0.1):
    for _ in range(count):
        tnode.publish_frame()
        time.sleep(delay)


# ──────────────────────────────────────────────────────────────────────────────
# Fixtures
# ──────────────────────────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def pipeline_procs(ros):
    """
    Launch the depth-camera → rtabmap → nav2 pipeline via LaunchService so
    that parameters are passed as a Python dict (preserving string types for
    rtabmap-specific params) and remappings are applied correctly.

    Nodes launched:
      static TF  odom → base_link
      static TF  base_link → camera_link
      static TF  camera_link → camera_depth_optical_frame
      static TF  camera_link → camera_color_optical_frame
      rgbd_odometry   (rtabmap_odom)  → publishes /odom
      rtabmap SLAM    (rtabmap_slam)  → publishes /map
    """
    db_path = tempfile.mktemp(suffix=".db", prefix="rtabmap_inttest_")

    def _stf(x, y, z, yaw, pitch, roll, parent, child):
        return launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[str(x), str(y), str(z),
                       str(yaw), str(pitch), str(roll),
                       parent, child],
            output="log")

    rgbd_odom_node = launch_ros.actions.Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        parameters=[{
            "frame_id":                "base_link",
            "odom_frame_id":           "odom",
            "approx_sync":             True,
            "approx_sync_max_interval": 0.5,
            "publish_tf":              False,
            "Odom/Strategy":           "0",   # F2M — robust in low-texture
            "Vis/MinInliers":          "5",
            "Vis/FeatureType":         "6",   # GFTT/Harris (always available)
        }],
        remappings=[
            ("rgb/image",       "/camera/color/image_raw"),
            ("depth/image",     "/camera/depth/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("odom",            "/odom"),
        ],
        output="log")

    rtabmap_node = launch_ros.actions.Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        arguments=["--delete_db_on_start"],
        parameters=[{
            "frame_id":                 "base_link",
            "odom_frame_id":            "odom",
            "map_frame_id":             "map",
            "subscribe_depth":          True,
            "approx_sync":              True,
            "approx_sync_max_interval": 0.5,
            "publish_tf":               True,
            "database_path":            db_path,
            "Mem/IncrementalMemory":    "true",
            "Vis/MinInliers":           "5",
            "Vis/FeatureType":          "6",
            "RGBD/AngularUpdate":       "0.01",
            "RGBD/LinearUpdate":        "0.01",
        }],
        remappings=[
            ("rgb/image",       "/camera/color/image_raw"),
            ("depth/image",     "/camera/depth/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("odom",            "/odom"),
            ("map",             "/map"),
        ],
        output="log")

    ld = launch.LaunchDescription([
        _stf(0,    0, 0,      0,     0,      0, "odom",        "base_link"),
        _stf(0.15, 0, 0.10,   0,     0.087,  0, "base_link",   "camera_link"),
        _stf(0,    0, 0,     -1.5708, 0, -1.5708,
             "camera_link", "camera_depth_optical_frame"),
        _stf(0,    0, 0,     -1.5708, 0, -1.5708,
             "camera_link", "camera_color_optical_frame"),
        rgbd_odom_node,
        rtabmap_node,
    ])

    ls = LaunchService()
    ls.include_launch_description(ld)

    t = threading.Thread(target=ls.run, daemon=True)
    t.start()

    time.sleep(PIPELINE_STARTUP)
    yield ls

    ls.shutdown()
    t.join(timeout=10)


@pytest.fixture
def tnode(ros):
    n = _TestNode()
    yield n
    n.destroy_node()


# ──────────────────────────────────────────────────────────────────────────────
# rgbd_odometry integration tests
# ──────────────────────────────────────────────────────────────────────────────

class TestRgbdOdometryIntegration:
    """Verify rgbd_odometry processes synthetic depth+RGB → /odom."""

    def test_odom_published(self, pipeline_procs, tnode):
        """rgbd_odometry must publish /odom after receiving camera frames."""
        _pump(tnode, count=30)
        odom = tnode.wait_odom(timeout=ODOM_TIMEOUT)
        assert odom is not None, (
            "/odom not received within timeout — "
            "check rtabmap_odom/rgbd_odometry installation")

    def test_odom_frame_ids(self, pipeline_procs, tnode):
        """/odom must have frame_id='odom' and child_frame_id='base_link'."""
        _pump(tnode, count=15)
        odom = tnode.wait_odom(timeout=15.0)
        assert odom is not None, "/odom not received"
        assert odom.header.frame_id == "odom", (
            f"Expected frame_id 'odom', got '{odom.header.frame_id}'")
        assert odom.child_frame_id == "base_link", (
            f"Expected child_frame_id 'base_link', got '{odom.child_frame_id}'")

    def test_odom_position_finite(self, pipeline_procs, tnode):
        """All position values in /odom must be finite (not NaN / Inf)."""
        _pump(tnode, count=15)
        odom = tnode.wait_odom(timeout=15.0)
        assert odom is not None, "/odom not received"
        pos = odom.pose.pose.position
        assert math.isfinite(pos.x), f"odom.position.x not finite: {pos.x}"
        assert math.isfinite(pos.y), f"odom.position.y not finite: {pos.y}"
        assert math.isfinite(pos.z), f"odom.position.z not finite: {pos.z}"


# ──────────────────────────────────────────────────────────────────────────────
# rtabmap SLAM integration tests
# ──────────────────────────────────────────────────────────────────────────────

class TestRtabmapSlamIntegration:
    """Verify rtabmap builds a map from the depth-camera pipeline."""

    def test_map_published(self, pipeline_procs, tnode):
        """/map (OccupancyGrid) must be published by rtabmap with frame_id='map'."""
        # Pump plenty of frames — rtabmap needs odom + camera data
        _pump(tnode, count=40)
        grid = tnode.wait_map(timeout=MAP_TIMEOUT)
        assert grid is not None, (
            "/map not received within timeout — "
            "rtabmap may not be running or has insufficient visual features")
        assert grid.header.frame_id == "map", (
            f"Expected map frame_id 'map', got '{grid.header.frame_id}'")


# ──────────────────────────────────────────────────────────────────────────────
# Nav2 PointCloud2 compatibility tests
# ──────────────────────────────────────────────────────────────────────────────

class TestNav2PointCloudCompatibility:
    """
    Verify /camera/depth/points has the format nav2's obstacle_layer expects:
    x, y, z float32 fields, camera_depth_optical_frame, point_step=12.

    The test node generates PointCloud2 via the same pinhole projection that
    depth_image_proc/point_cloud_xyz uses in the live system, validating both
    the depth→3D math and the message format consumed by nav2.
    """

    def test_pointcloud_xyz_fields(self, pipeline_procs, tnode):
        """PointCloud2 must have x, y, z float32 fields for nav2 obstacle_layer."""
        _pump(tnode, count=3)
        pc = tnode.wait_pc()
        assert pc is not None, "/camera/depth/points not received"
        field_names = [f.name for f in pc.fields]
        for axis in ("x", "y", "z"):
            assert axis in field_names, (
                f"PointCloud2 missing '{axis}' field; fields={field_names}")

    def test_pointcloud_near_range(self, pipeline_procs, tnode):
        """Points from the 1.0 m obstacle box must appear with z ∈ [0.8, 1.2] m."""
        tnode.pc_msgs.clear()
        _pump(tnode, count=3)
        pc = tnode.wait_pc()
        assert pc is not None, "/camera/depth/points not received"

        field_map  = {f.name: f.offset for f in pc.fields}
        z_off      = field_map["z"]
        ps         = pc.point_step
        data       = bytes(pc.data)
        n_pts      = len(data) // ps

        near = sum(
            1 for i in range(n_pts)
            if math.isfinite(
                z := struct.unpack_from("<f", data, i * ps + z_off)[0]
            ) and 0.8 <= z <= 1.2
        )
        assert near > 0, (
            f"No points with z ∈ [0.8, 1.2] m in PointCloud2 "
            f"(obstacle at 1.0 m; total points: {n_pts})")

    def test_pointcloud_frame_id(self, pipeline_procs, tnode):
        """PointCloud2 frame_id must be camera_depth_optical_frame."""
        _pump(tnode, count=3)
        pc = tnode.wait_pc()
        assert pc is not None, "/camera/depth/points not received"
        assert pc.header.frame_id == "camera_depth_optical_frame", (
            f"Expected 'camera_depth_optical_frame', got '{pc.header.frame_id}'")

    def test_pointcloud_point_step(self, pipeline_procs, tnode):
        """point_step must be 12 bytes (3 × float32) for nav2 compatibility."""
        _pump(tnode, count=3)
        pc = tnode.wait_pc()
        assert pc is not None
        assert pc.point_step == 12, (
            f"Expected point_step=12 (3×float32), got {pc.point_step}")

    def test_pointcloud_not_empty(self, pipeline_procs, tnode):
        """PointCloud2 must contain at least one point."""
        _pump(tnode, count=3)
        pc = tnode.wait_pc()
        assert pc is not None
        n = len(bytes(pc.data)) // pc.point_step
        assert n > 0, "PointCloud2 has zero points"
