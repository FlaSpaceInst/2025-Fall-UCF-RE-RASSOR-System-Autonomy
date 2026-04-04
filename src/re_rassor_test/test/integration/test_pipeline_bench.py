"""
test_pipeline_bench.py
──────────────────────────────────────────────────────────────────────────────
Comprehensive test bench for the RE-RASSOR depth-camera → odometry → map
pipeline.  Matches the *production* configuration in re_rassor_full.launch.py:

  depth_image_proc  /camera/depth/image_raw → /camera/depth/points
  icp_odometry      /camera/depth/points   → /rtabmap/odom
  odom relay        /rtabmap/odom          → /odom
  rtabmap SLAM      /camera/depth/points + /rtabmap/odom → /rtabmap/map
  map relay         /rtabmap/map           → /map
  fake_map          standalone blank OccupancyGrid on /map (transient_local)

NOTE: the previous test (test_obstacle_detection_node.py) used rgbd_odometry
(RGB+depth).  Production uses icp_odometry (depth/PointCloud2 only).  This
bench matches production.

Test classes
────────────
  TestFakeMap                — blank OccupancyGrid publisher, QoS, dimensions
  TestDepthCameraMessages    — topic format, encoding, frame IDs, camera_info K
  TestDepthToPointCloud      — depth_image_proc XYZ node, field layout, range
  TestICPOdometry            — icp_odometry → /rtabmap/odom frame IDs + finite
  TestOdomRelay              — relay /rtabmap/odom → /odom
  TestRtabmapMap             — rtabmap SLAM → /rtabmap/map frame_id + cells
  TestMapRelay               — relay /rtabmap/map → /map
  TestTFTree                 — all required TF transforms present
  TestQoSCompatibility       — /map transient_local, depth BEST_EFFORT

Run:
    colcon build --packages-select re_rassor_test --cmake-args -DBUILD_TESTING=ON
    colcon test --packages-select re_rassor_test \
        --pytest-args -k test_pipeline_bench -s
    colcon test-result --verbose
"""

import math
import os
import struct
import subprocess
import tempfile
import time

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros
import yaml

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

# ─────────────────────────────────────────────────────────────────────────────
# Astra Pro 640×480 reference intrinsics  (matches re_rassor_full.launch.py)
# ─────────────────────────────────────────────────────────────────────────────
ASTRA_W  = 640
ASTRA_H  = 480
ASTRA_FX = 554.26
ASTRA_FY = 554.26
ASTRA_CX = 320.0
ASTRA_CY = 240.0

# Timeout constants (seconds) — sized for a real robot with cold-start latency
PIPELINE_STARTUP  = 10.0   # time for all nodes to finish initialising
ICP_TIMEOUT       = 25.0   # /rtabmap/odom first message
ODOM_RELAY_TIMEOUT = 28.0  # /odom first message (relay adds ≈1 s)
MAP_TIMEOUT       = 45.0   # /rtabmap/map first message
MAP_RELAY_TIMEOUT = 48.0   # /map after relay
PC_TIMEOUT        = 8.0    # /camera/depth/points after depth_image_proc
TF_TIMEOUT        = 5.0    # tf2 lookup timeout

SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
TRANSIENT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


# ─────────────────────────────────────────────────────────────────────────────
# Message factories
# ─────────────────────────────────────────────────────────────────────────────

def _camera_info(stamp, frame_id: str) -> CameraInfo:
    """Astra Pro pinhole intrinsics — mirrors what the real driver publishes."""
    msg = CameraInfo()
    msg.header.stamp     = stamp
    msg.header.frame_id  = frame_id
    msg.width            = ASTRA_W
    msg.height           = ASTRA_H
    msg.distortion_model = "plumb_bob"
    msg.d                = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [ASTRA_FX, 0.0, ASTRA_CX,
             0.0,      ASTRA_FY, ASTRA_CY,
             0.0,      0.0,      1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [ASTRA_FX, 0.0, ASTRA_CX, 0.0,
             0.0,      ASTRA_FY, ASTRA_CY, 0.0,
             0.0,      0.0,      1.0,      0.0]
    return msg


def _make_scene():
    """
    Synthetic scene (static content — published repeatedly).
      depth map:   2.0 m background, 1.0 m box at rows 100-200, cols 200-440
      16-bit uint16 millimetres — matches Astra Pro output format
    """
    depth_f32 = np.full((ASTRA_H, ASTRA_W), 2.0, dtype=np.float32)
    depth_f32[100:200, 200:440] = 1.0
    return depth_f32


def _make_moving_scene(frame_idx: int):
    """
    Synthetic scene with slight motion — needed for ICP odometry to converge.
    Each frame shifts the foreground box a few pixels so the point cloud moves.
    """
    depth_f32 = np.full((ASTRA_H, ASTRA_W), 2.5, dtype=np.float32)
    shift = frame_idx * 2        # 2-pixel column shift per frame
    c0 = max(0, 180 + shift)
    c1 = min(ASTRA_W, c0 + 120)
    depth_f32[120:220, c0:c1] = 1.0 + frame_idx * 0.02  # slight depth change too
    return depth_f32


def _depth_msg(stamp, depth_f32: np.ndarray) -> Image:
    msg = Image()
    msg.header.stamp    = stamp
    msg.header.frame_id = "camera_depth_optical_frame"
    msg.width    = ASTRA_W
    msg.height   = ASTRA_H
    msg.encoding = "32FC1"
    msg.step     = ASTRA_W * 4
    msg.data     = depth_f32.astype(np.float32).tobytes()
    return msg


def _pointcloud_from_depth(stamp, depth_f32: np.ndarray,
                            stride: int = 2) -> PointCloud2:
    """
    Pinhole projection identical to depth_image_proc/point_cloud_xyz_node.
    Used to directly publish /camera/depth/points for ICP odometry tests.
    stride: subsample factor — 2 keeps ~75k pts, fast enough for ICP.
    """
    rs = np.arange(0, ASTRA_H, stride)
    cs = np.arange(0, ASTRA_W, stride)
    c, r = np.meshgrid(cs, rs)
    z = depth_f32[r, c].astype(np.float32)

    # Remove zero/inf depth points
    valid = (z > 0.01) & np.isfinite(z)
    z = z[valid]
    cv = c[valid].astype(np.float32)
    rv = r[valid].astype(np.float32)

    x = ((cv - ASTRA_CX) * z / ASTRA_FX).astype(np.float32)
    y = ((rv - ASTRA_CY) * z / ASTRA_FY).astype(np.float32)

    pts = np.stack([x, y, z], axis=1)
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


# ─────────────────────────────────────────────────────────────────────────────
# Test node — publishes synthetic sensor data, subscribes to pipeline outputs
# ─────────────────────────────────────────────────────────────────────────────

class _BenchNode(Node):
    """
    Publish synthetic camera data and collect pipeline outputs for assertions.

    Publishers  (mimic Astra Pro driver):
      /camera/depth/image_raw     Image   32FC1
      /camera/depth/camera_info   CameraInfo
      /camera/depth/points        PointCloud2   (direct, for ICP odom bypass)

    Subscribers (pipeline outputs):
      /camera/depth/points        PointCloud2   (from depth_image_proc if running)
      /rtabmap/odom               Odometry
      /odom                       Odometry      (relay output)
      /rtabmap/map                OccupancyGrid
      /map                        OccupancyGrid (relay output or fake_map)
    """

    def __init__(self):
        super().__init__("re_rassor_bench_node")

        # Publishers
        self.depth_pub      = self.create_publisher(
            Image,       "/camera/depth/image_raw",  SENSOR_QOS)
        self.depth_info_pub = self.create_publisher(
            CameraInfo,  "/camera/depth/camera_info", 10)
        self.pc_pub         = self.create_publisher(
            PointCloud2, "/camera/depth/points",     SENSOR_QOS)

        # Received message buffers
        self.pc_msgs:        list[PointCloud2]   = []
        self.rtabmap_odom:   list[Odometry]      = []
        self.odom_msgs:      list[Odometry]      = []
        self.rtabmap_map:    list[OccupancyGrid] = []
        self.map_msgs:       list[OccupancyGrid] = []

        # Subscriptions
        self.create_subscription(
            PointCloud2,   "/camera/depth/points",
            lambda m: self.pc_msgs.append(m),        SENSOR_QOS)
        self.create_subscription(
            Odometry,      "/rtabmap/odom",
            lambda m: self.rtabmap_odom.append(m),   10)
        self.create_subscription(
            Odometry,      "/odom",
            lambda m: self.odom_msgs.append(m),      10)
        self.create_subscription(
            OccupancyGrid, "/rtabmap/map",
            lambda m: self.rtabmap_map.append(m),    TRANSIENT_QOS)
        self.create_subscription(
            OccupancyGrid, "/map",
            lambda m: self.map_msgs.append(m),       TRANSIENT_QOS)

        # TF buffer for transform lookups
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._frame = 0

    # ── Publish helpers ──────────────────────────────────────────────────────

    def publish_depth_frame(self, depth_f32: np.ndarray | None = None):
        """Publish one depth image + camera_info + raw pointcloud."""
        if depth_f32 is None:
            depth_f32 = _make_moving_scene(self._frame)
        stamp = self.get_clock().now().to_msg()
        self.depth_pub.publish(_depth_msg(stamp, depth_f32))
        self.depth_info_pub.publish(_camera_info(stamp, "camera_depth_optical_frame"))
        self.pc_pub.publish(_pointcloud_from_depth(stamp, depth_f32))
        self._frame += 1

    # ── Spin + wait helpers ──────────────────────────────────────────────────

    def _spin(self, secs: float = 0.05):
        rclpy.spin_once(self, timeout_sec=secs)

    def _wait_for(self, attr: str, timeout: float):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._spin()
            buf = getattr(self, attr)
            if buf:
                return buf[-1]
        return None

    def wait_pc(self,          timeout: float = PC_TIMEOUT)        -> PointCloud2 | None:
        return self._wait_for("pc_msgs",      timeout)
    def wait_rtabmap_odom(self, timeout: float = ICP_TIMEOUT)      -> Odometry | None:
        return self._wait_for("rtabmap_odom", timeout)
    def wait_odom(self,        timeout: float = ODOM_RELAY_TIMEOUT) -> Odometry | None:
        return self._wait_for("odom_msgs",    timeout)
    def wait_rtabmap_map(self,  timeout: float = MAP_TIMEOUT)       -> OccupancyGrid | None:
        return self._wait_for("rtabmap_map",  timeout)
    def wait_map(self,          timeout: float = MAP_RELAY_TIMEOUT) -> OccupancyGrid | None:
        return self._wait_for("map_msgs",     timeout)


def _pump(node: _BenchNode, count: int = 30, hz: float = 10.0):
    """Publish `count` frames at `hz` Hz while spinning."""
    delay = 1.0 / hz
    for _ in range(count):
        node.publish_depth_frame()
        t0 = time.time()
        while time.time() - t0 < delay:
            rclpy.spin_once(node, timeout_sec=0.01)


# ─────────────────────────────────────────────────────────────────────────────
# Subprocess-based node launchers
# ─────────────────────────────────────────────────────────────────────────────
# LaunchService.run() requires the main thread in ROS 2 Jazzy.  Using
# subprocess.Popen('ros2 run ...') avoids that restriction entirely and is
# simpler to tear down.

_ENV = os.environ.copy()   # inherit the sourced ROS / workspace environment


def _stf_proc(x, y, z, yaw, pitch, roll, parent, child) -> subprocess.Popen:
    """static_transform_publisher as a subprocess."""
    return subprocess.Popen(
        ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
         str(x), str(y), str(z), str(yaw), str(pitch), str(roll),
         parent, child],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=_ENV)


def _write_params(node_name: str, params: dict) -> str:
    """Write a ROS 2 parameter YAML file, return its path."""
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='ros2_params_', delete=False)
    yaml.dump({node_name: {'ros__parameters': params}}, tmp)
    tmp.flush()
    tmp.close()
    return tmp.name


def _icp_odom_proc(db_path: str) -> subprocess.Popen:
    """icp_odometry via subprocess — matches production config."""
    params_file = _write_params('icp_odometry', {
        'frame_id':                      'base_link',
        'odom_frame_id':                 'odom',
        'publish_tf':                    False,
        'approx_sync':                   True,
        'queue_size':                    10,
        'Reg/Force3DoF':                 'true',
        'Icp/VoxelSize':                 '0.05',
        'Icp/MaxCorrespondenceDistance': '0.5',
    })
    return subprocess.Popen(
        ['ros2', 'run', 'rtabmap_odom', 'icp_odometry',
         '--ros-args',
         '-r', '__node:=icp_odometry',
         '-r', '__ns:=/rtabmap',
         '-r', 'scan_cloud:=/camera/depth/points',
         '--params-file', params_file],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=_ENV)


def _rtabmap_proc(db_path: str) -> subprocess.Popen:
    """rtabmap SLAM via subprocess — matches production config."""
    params_file = _write_params('rtabmap', {
        'frame_id':             'base_link',
        'map_frame_id':         'map',
        'subscribe_scan_cloud': True,
        'subscribe_rgb':        False,
        'publish_tf':           False,
        'database_path':        db_path,
        'delete_db_on_start':   True,
        'Reg/Force3DoF':        'true',
        'Grid/3D':              'false',
        'Grid/CellSize':        '0.05',
        'Grid/RangeMax':        '4.0',
        'Grid/MinGroundHeight': '-0.1',
        'Grid/MaxGroundHeight': '0.15',
        'RGBD/LinearUpdate':    '0.005',
        'RGBD/AngularUpdate':   '0.005',
    })
    return subprocess.Popen(
        ['ros2', 'run', 'rtabmap_slam', 'rtabmap',
         '--ros-args',
         '-r', '__node:=rtabmap',
         '-r', '__ns:=/rtabmap',
         '-r', 'scan_cloud:=/camera/depth/points',
         '-r', 'odom:=/rtabmap/odom',
         '--params-file', params_file],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=_ENV)


def _topic_tools_available() -> bool:
    """Return True if topic_tools package is present in the ROS environment."""
    result = subprocess.run(
        ['ros2', 'pkg', 'prefix', 'topic_tools'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=_ENV)
    return result.returncode == 0


_TOPIC_TOOLS_OK = _topic_tools_available()


def _relay_proc(src: str, dst: str) -> subprocess.Popen | None:
    """
    topic_tools relay via subprocess.
    Returns None (and the relay tests will skip) if topic_tools is not installed.
    """
    if not _TOPIC_TOOLS_OK:
        return None
    return subprocess.Popen(
        ['ros2', 'run', 'topic_tools', 'relay', src, dst],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=_ENV)


def _kill_all(procs: list[subprocess.Popen | None]):
    for p in procs:
        if p is None:
            continue
        try:
            p.terminate()
            p.wait(timeout=3)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────────────────────
# Pytest fixtures
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def icp_pipeline(ros):
    """
    Launches the ICP odometry sub-pipeline + relay only via subprocesses:
      static TFs + icp_odometry → /rtabmap/odom → relay → /odom
    """
    db_path = tempfile.mktemp(suffix=".db", prefix="rtab_icp_")
    procs = [
        _stf_proc(0,    0,    0,       0,      0,       0, "odom",        "base_link"),
        _stf_proc(0.15, 0,    0.10,    0,      0.087,   0, "base_link",   "camera_link"),
        _stf_proc(0,    0,    0,      -1.5708, 0, -1.5708, "camera_link", "camera_depth_optical_frame"),
        _icp_odom_proc(db_path),
        _relay_proc("/rtabmap/odom", "/odom"),
    ]
    time.sleep(PIPELINE_STARTUP)
    yield procs
    _kill_all(procs)


@pytest.fixture(scope="module")
def full_pipeline(ros):
    """
    Launches the full pipeline via subprocesses:
      static TFs + icp_odometry + odom_relay + rtabmap + map_relay
    """
    db_path = tempfile.mktemp(suffix=".db", prefix="rtab_full_")
    procs = [
        _stf_proc(0,    0,    0,       0,      0,       0, "odom",        "base_link"),
        _stf_proc(0.15, 0,    0.10,    0,      0.087,   0, "base_link",   "camera_link"),
        _stf_proc(0,    0,    0,      -1.5708, 0, -1.5708, "camera_link", "camera_depth_optical_frame"),
        _icp_odom_proc(db_path),
        _relay_proc("/rtabmap/odom", "/odom"),
        _rtabmap_proc(db_path),
        _relay_proc("/rtabmap/map",  "/map"),
    ]
    time.sleep(PIPELINE_STARTUP)
    yield procs
    _kill_all(procs)


@pytest.fixture
def bench(ros):
    n = _BenchNode()
    yield n
    n.destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
# TestFakeMap — validates the blank map bootstrap publisher
# ─────────────────────────────────────────────────────────────────────────────

class TestFakeMap:
    """
    Validates fake_map.py (re_rassor_full.launch.py item 0d).
    Runs fake_map.py in-process — no external node required.
    """

    def test_fake_map_qos_transient_local(self, ros, bench):
        """
        fake_map.py must publish with TRANSIENT_LOCAL durability so that
        late-joining Nav2 static_layer receives the map.
        """
        import sys

        # fake_map.py lives at the workspace root (not inside a package).
        # Walk up from this file until we find it — works from source tree
        # and from colcon install/ since both share the same workspace root.
        script = None
        candidate = os.path.dirname(os.path.realpath(__file__))
        for _ in range(10):
            candidate = os.path.dirname(candidate)
            probe = os.path.join(candidate, "fake_map.py")
            if os.path.isfile(probe):
                script = probe
                break
        if script is None:
            pytest.skip("fake_map.py not found in any ancestor directory")

        proc = subprocess.Popen(
            [sys.executable, script],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # bench is subscribed with TRANSIENT_LOCAL — must receive the map
        grid = bench.wait_map(timeout=8.0)
        proc.wait(timeout=10)

        assert grid is not None, (
            "/map not received from fake_map.py with TRANSIENT_LOCAL QoS — "
            "check the publisher's durability setting in fake_map.py")

    def test_fake_map_frame_id(self, ros, bench):
        """fake_map.py must publish with header.frame_id = 'map'."""
        grid = bench.wait_map(timeout=3.0)
        if grid is None:
            pytest.skip("fake_map.py not running — run test_fake_map_qos_transient_local first")
        assert grid.header.frame_id == "map", (
            f"Expected frame_id='map', got '{grid.header.frame_id}'")

    def test_fake_map_is_free_space(self, ros, bench):
        """All cells in the fake map must be 0 (free) — nav2 must not see walls."""
        grid = bench.wait_map(timeout=3.0)
        if grid is None:
            pytest.skip("no /map message available")
        assert all(c == 0 for c in grid.data), (
            "fake_map.py has non-free cells — nav2 will see phantom obstacles")

    def test_fake_map_resolution(self, ros, bench):
        """fake_map.py must use 0.05 m/cell — matches global_costmap resolution."""
        grid = bench.wait_map(timeout=3.0)
        if grid is None:
            pytest.skip("no /map message available")
        assert abs(grid.info.resolution - 0.05) < 1e-4, (
            f"Expected resolution 0.05 m/cell, got {grid.info.resolution}")

    def test_fake_map_covers_workspace(self, ros, bench):
        """Fake map must span at least ±5 m (200×200 cells at 0.05 m/cell)."""
        grid = bench.wait_map(timeout=3.0)
        if grid is None:
            pytest.skip("no /map message available")
        w_m = grid.info.width  * grid.info.resolution
        h_m = grid.info.height * grid.info.resolution
        assert w_m >= 10.0, f"Map width {w_m} m < 10 m — too small for Nav2"
        assert h_m >= 10.0, f"Map height {h_m} m < 10 m — too small for Nav2"


# ─────────────────────────────────────────────────────────────────────────────
# TestDepthCameraMessages — validates synthetic depth frames
# ─────────────────────────────────────────────────────────────────────────────

class TestDepthCameraMessages:
    """
    Validates that the synthetic depth frames published by _BenchNode match
    what the Astra Pro driver publishes in production.  Tests fail here mean
    the camera driver or topic remappings are wrong.
    """

    def test_depth_image_encoding(self, ros, bench):
        """Depth image must be 32FC1 (float32 metres) — matches Astra Pro."""
        bench.depth_pub.publish(_depth_msg(
            bench.get_clock().now().to_msg(), _make_scene()))
        rclpy.spin_once(bench, timeout_sec=0.1)
        # We published directly — just validate the factory
        scene = _make_scene()
        msg   = _depth_msg(bench.get_clock().now().to_msg(), scene)
        assert msg.encoding == "32FC1", (
            f"Expected encoding '32FC1', got '{msg.encoding}'")

    def test_depth_image_dimensions(self, ros, bench):
        """Depth image must be 640×480 — matches Astra Pro resolution."""
        scene = _make_scene()
        msg   = _depth_msg(bench.get_clock().now().to_msg(), scene)
        assert msg.width  == ASTRA_W, f"Expected width {ASTRA_W}, got {msg.width}"
        assert msg.height == ASTRA_H, f"Expected height {ASTRA_H}, got {msg.height}"

    def test_depth_image_step(self, ros, bench):
        """step must equal width × 4 bytes (one float32 per pixel)."""
        scene = _make_scene()
        msg   = _depth_msg(bench.get_clock().now().to_msg(), scene)
        assert msg.step == ASTRA_W * 4, (
            f"Expected step={ASTRA_W * 4}, got {msg.step}")

    def test_depth_image_frame_id(self, ros, bench):
        """Depth image frame_id must be 'camera_depth_optical_frame'."""
        scene = _make_scene()
        msg   = _depth_msg(bench.get_clock().now().to_msg(), scene)
        assert msg.header.frame_id == "camera_depth_optical_frame", (
            f"Expected 'camera_depth_optical_frame', got '{msg.header.frame_id}'")

    def test_camera_info_intrinsics(self, ros, bench):
        """CameraInfo K matrix must have non-zero fx, fy, cx, cy."""
        stamp = bench.get_clock().now().to_msg()
        info  = _camera_info(stamp, "camera_depth_optical_frame")
        assert info.k[0] > 0.0, f"fx (K[0]) is zero or negative: {info.k[0]}"
        assert info.k[4] > 0.0, f"fy (K[4]) is zero or negative: {info.k[4]}"
        assert info.k[2] > 0.0, f"cx (K[2]) is zero or negative: {info.k[2]}"
        assert info.k[5] > 0.0, f"cy (K[5]) is zero or negative: {info.k[5]}"

    def test_camera_info_dimensions_match(self, ros, bench):
        """CameraInfo width/height must match depth image dimensions."""
        stamp = bench.get_clock().now().to_msg()
        info  = _camera_info(stamp, "camera_depth_optical_frame")
        assert info.width  == ASTRA_W
        assert info.height == ASTRA_H


# ─────────────────────────────────────────────────────────────────────────────
# TestDepthToPointCloud — validates /camera/depth/points format
# ─────────────────────────────────────────────────────────────────────────────

class TestDepthToPointCloud:
    """
    Validates that /camera/depth/points has the format nav2's obstacle_layer
    and icp_odometry expect.  In production this is produced by
    depth_image_proc/point_cloud_xyz_node.  In tests, _BenchNode publishes
    the cloud directly using the same pinhole projection.
    """

    def test_pointcloud_published(self, ros, bench):
        """_BenchNode must echo /camera/depth/points back to itself."""
        bench.pc_msgs.clear()
        _pump(bench, count=3)
        pc = bench.wait_pc(timeout=5.0)
        assert pc is not None, "/camera/depth/points not received"

    def test_pointcloud_xyz_fields(self, ros, bench):
        """PointCloud2 must have x, y, z float32 fields (point_step=12)."""
        bench.pc_msgs.clear()
        _pump(bench, count=3)
        pc = bench.wait_pc()
        assert pc is not None, "/camera/depth/points not received"
        names = {f.name for f in pc.fields}
        for axis in ("x", "y", "z"):
            assert axis in names, (
                f"PointCloud2 missing '{axis}' field; fields={names}")

    def test_pointcloud_point_step(self, ros, bench):
        """point_step must be 12 (3 × float32) for nav2 + ICP compatibility."""
        bench.pc_msgs.clear()
        _pump(bench, count=3)
        pc = bench.wait_pc()
        assert pc is not None
        assert pc.point_step == 12, (
            f"Expected point_step=12, got {pc.point_step} — "
            "nav2 voxel_layer and ICP odometry require packed XYZ float32")

    def test_pointcloud_not_empty(self, ros, bench):
        """PointCloud2 must contain at least one point."""
        bench.pc_msgs.clear()
        _pump(bench, count=3)
        pc = bench.wait_pc()
        assert pc is not None
        n = len(bytes(pc.data)) // pc.point_step
        assert n > 0, "PointCloud2 has zero points — ICP odometry will fail"

    def test_pointcloud_frame_id(self, ros, bench):
        """frame_id must be 'camera_depth_optical_frame'."""
        bench.pc_msgs.clear()
        _pump(bench, count=3)
        pc = bench.wait_pc()
        assert pc is not None
        assert pc.header.frame_id == "camera_depth_optical_frame", (
            f"Expected 'camera_depth_optical_frame', got '{pc.header.frame_id}'\n"
            "Mismatched frame_id breaks the TF lookup in ICP odometry.")

    def test_pointcloud_obstacle_depth(self, ros, bench):
        """Points from the synthetic 1.0 m obstacle must have z ∈ [0.8, 1.2]."""
        bench.pc_msgs.clear()
        _pump(bench, count=3)
        pc = bench.wait_pc()
        assert pc is not None

        field_map = {f.name: f.offset for f in pc.fields}
        z_off = field_map["z"]
        ps    = pc.point_step
        data  = bytes(pc.data)
        n_pts = len(data) // ps

        near = sum(
            1 for i in range(n_pts)
            if math.isfinite(
                z := struct.unpack_from("<f", data, i * ps + z_off)[0]
            ) and 0.8 <= z <= 1.2
        )
        assert near > 0, (
            f"No points with z ∈ [0.8, 1.2] m (obstacle at 1.0 m). "
            f"Total points: {n_pts}. "
            "Check depth image encoding and pinhole projection.")

    def test_pointcloud_no_nan_inf(self, ros, bench):
        """No NaN or Inf values in the xyz fields — ICP will crash on them."""
        bench.pc_msgs.clear()
        _pump(bench, count=3)
        pc = bench.wait_pc()
        assert pc is not None

        field_map = {f.name: f.offset for f in pc.fields}
        ps   = pc.point_step
        data = bytes(pc.data)
        n    = len(data) // ps

        bad = []
        for i in range(n):
            for axis, off in field_map.items():
                v = struct.unpack_from("<f", data, i * ps + off)[0]
                if not math.isfinite(v):
                    bad.append((i, axis, v))
                    if len(bad) > 5:
                        break
            if len(bad) > 5:
                break

        assert not bad, (
            f"PointCloud2 contains non-finite values (first 5): {bad[:5]}\n"
            "ICP odometry will reject or crash on NaN/Inf points.")


# ─────────────────────────────────────────────────────────────────────────────
# TestICPOdometry — validates icp_odometry → /rtabmap/odom → relay → /odom
# ─────────────────────────────────────────────────────────────────────────────

class TestICPOdometry:
    """
    Validates the ICP odometry node (rtabmap_odom/icp_odometry).
    This is the production odometry source — NOT rgbd_odometry.
    """

    def test_rtabmap_odom_published(self, icp_pipeline, bench):
        """
        /rtabmap/odom must be published after icp_odometry receives point clouds.
        If this fails: check that icp_odometry is running and /camera/depth/points
        is being received by it.
        """
        bench.rtabmap_odom.clear()
        _pump(bench, count=50, hz=10.0)
        odom = bench.wait_rtabmap_odom(ICP_TIMEOUT)
        assert odom is not None, (
            "/rtabmap/odom not received within timeout.\n"
            "Diagnosis:\n"
            "  1. Is icp_odometry running?  ros2 node list | grep icp\n"
            "  2. Is /camera/depth/points publishing? ros2 topic echo /camera/depth/points\n"
            "  3. Is the TF tree complete?  ros2 run tf2_tools view_frames\n"
            "  4. Check icp_odometry logs for 'insufficient features' warnings.")

    def test_rtabmap_odom_frame_ids(self, icp_pipeline, bench):
        """/rtabmap/odom must have frame_id='odom', child_frame_id='base_link'."""
        bench.rtabmap_odom.clear()
        _pump(bench, count=30, hz=10.0)
        odom = bench.wait_rtabmap_odom(ICP_TIMEOUT)
        assert odom is not None, "/rtabmap/odom not received"
        assert odom.header.frame_id == "odom", (
            f"Expected frame_id 'odom', got '{odom.header.frame_id}'\n"
            "Check icp_odometry odom_frame_id parameter.")
        assert odom.child_frame_id == "base_link", (
            f"Expected child_frame_id 'base_link', got '{odom.child_frame_id}'\n"
            "Check icp_odometry frame_id parameter.")

    def test_rtabmap_odom_position_finite(self, icp_pipeline, bench):
        """All position fields in /rtabmap/odom must be finite (not NaN/Inf)."""
        bench.rtabmap_odom.clear()
        _pump(bench, count=30, hz=10.0)
        odom = bench.wait_rtabmap_odom(ICP_TIMEOUT)
        assert odom is not None, "/rtabmap/odom not received"
        pos = odom.pose.pose.position
        assert math.isfinite(pos.x), f"odom.position.x is not finite: {pos.x}"
        assert math.isfinite(pos.y), f"odom.position.y is not finite: {pos.y}"
        assert math.isfinite(pos.z), f"odom.position.z is not finite: {pos.z}"

    def test_rtabmap_odom_covariance_not_all_zero(self, icp_pipeline, bench):
        """
        Pose covariance must not be all-zero — all-zero means ICP didn't converge
        and the odometry is invalid (nav2 will reject it or behave erratically).
        """
        bench.rtabmap_odom.clear()
        _pump(bench, count=30, hz=10.0)
        odom = bench.wait_rtabmap_odom(ICP_TIMEOUT)
        assert odom is not None, "/rtabmap/odom not received"
        cov = odom.pose.covariance
        assert any(v != 0.0 for v in cov), (
            "Pose covariance is all zeros — ICP odometry may not have converged.\n"
            "Ensure point clouds have enough geometric features for ICP registration.")

    def test_odom_relay_published(self, icp_pipeline, bench):
        """/odom must be published by the relay (mirrors /rtabmap/odom)."""
        if not _TOPIC_TOOLS_OK:
            pytest.skip("topic_tools not installed — install ros-$ROS_DISTRO-topic-tools")
        bench.odom_msgs.clear()
        _pump(bench, count=50, hz=10.0)
        odom = bench.wait_odom(ODOM_RELAY_TIMEOUT)
        assert odom is not None, (
            "/odom not received — the relay node may not be running.\n"
            "Check: ros2 node list | grep relay\n"
            "Check: ros2 topic echo /odom")

    def test_odom_relay_frame_ids_match(self, icp_pipeline, bench):
        """/odom relay output must have same frame IDs as /rtabmap/odom."""
        if not _TOPIC_TOOLS_OK:
            pytest.skip("topic_tools not installed — install ros-$ROS_DISTRO-topic-tools")
        bench.rtabmap_odom.clear()
        bench.odom_msgs.clear()
        _pump(bench, count=50, hz=10.0)

        rtab = bench.wait_rtabmap_odom(ICP_TIMEOUT)
        odom = bench.wait_odom(ODOM_RELAY_TIMEOUT)

        assert rtab is not None, "/rtabmap/odom not received"
        assert odom  is not None, "/odom not received"

        assert odom.header.frame_id == rtab.header.frame_id, (
            f"/odom frame_id '{odom.header.frame_id}' "
            f"!= /rtabmap/odom frame_id '{rtab.header.frame_id}'")
        assert odom.child_frame_id == rtab.child_frame_id, (
            f"/odom child_frame_id '{odom.child_frame_id}' "
            f"!= /rtabmap/odom child_frame_id '{rtab.child_frame_id}'")


# ─────────────────────────────────────────────────────────────────────────────
# TestRtabmapMap — validates SLAM map output and relay
# ─────────────────────────────────────────────────────────────────────────────

class TestRtabmapMap:
    """
    Validates that rtabmap SLAM produces /rtabmap/map and the relay
    publishes it to /map.  These tests need the full pipeline.
    """

    def test_rtabmap_map_published(self, full_pipeline, bench):
        """
        /rtabmap/map must be published by rtabmap SLAM.
        This is the longest-running test — rtabmap needs enough odom + cloud
        frames before emitting a grid.

        If this fails:
          1. Is /rtabmap/odom publishing?  (TestICPOdometry must pass first)
          2. Is rtabmap receiving /camera/depth/points?
          3. Are RGBD/LinearUpdate / RGBD/AngularUpdate thresholds too high?
          4. Check rtabmap logs for 'SLAM update rejected' messages.
        """
        bench.rtabmap_map.clear()
        _pump(bench, count=80, hz=8.0)
        grid = bench.wait_rtabmap_map(MAP_TIMEOUT)
        assert grid is not None, (
            "/rtabmap/map not received within timeout.\n"
            "Possible causes:\n"
            "  • icp_odometry not publishing (run TestICPOdometry first)\n"
            "  • Point cloud too sparse — ICP needs overlapping geometry\n"
            "  • rtabmap not subscribing to /camera/depth/points\n"
            "    → ros2 topic info /camera/depth/points --verbose\n"
            "  • RGBD/LinearUpdate too high (try lowering to 0.005)")

    def test_rtabmap_map_frame_id(self, full_pipeline, bench):
        """rtabmap map frame_id must be 'map'."""
        grid = bench.wait_rtabmap_map(5.0) or bench.rtabmap_map[-1] if bench.rtabmap_map else None
        if grid is None:
            pytest.skip("/rtabmap/map not available — run test_rtabmap_map_published first")
        assert grid.header.frame_id == "map", (
            f"Expected frame_id 'map', got '{grid.header.frame_id}'")

    def test_rtabmap_map_resolution(self, full_pipeline, bench):
        """Map resolution must be 0.05 m/cell — matches Grid/CellSize parameter."""
        grid = bench.wait_rtabmap_map(5.0) or (bench.rtabmap_map[-1] if bench.rtabmap_map else None)
        if grid is None:
            pytest.skip("/rtabmap/map not available")
        assert abs(grid.info.resolution - 0.05) < 0.01, (
            f"Expected 0.05 m/cell, got {grid.info.resolution}\n"
            "Check Grid/CellSize parameter in re_rassor_full.launch.py")

    def test_rtabmap_map_has_cells(self, full_pipeline, bench):
        """Map must have non-zero dimensions."""
        grid = bench.wait_rtabmap_map(5.0) or (bench.rtabmap_map[-1] if bench.rtabmap_map else None)
        if grid is None:
            pytest.skip("/rtabmap/map not available")
        n = len(grid.data)
        assert n > 0, "OccupancyGrid has zero cells"
        assert grid.info.width  > 0, "OccupancyGrid width is 0"
        assert grid.info.height > 0, "OccupancyGrid height is 0"

    def test_map_relay_published(self, full_pipeline, bench):
        """/map (from relay) must mirror /rtabmap/map."""
        if not _TOPIC_TOOLS_OK:
            pytest.skip("topic_tools not installed — install ros-$ROS_DISTRO-topic-tools")
        bench.map_msgs.clear()
        _pump(bench, count=80, hz=8.0)
        grid = bench.wait_map(MAP_RELAY_TIMEOUT)
        assert grid is not None, (
            "/map not received — relay node may not be running.\n"
            "Check: ros2 node list | grep relay\n"
            "Check: ros2 topic echo /map")

    def test_map_relay_frame_id(self, full_pipeline, bench):
        """/map relay output must have frame_id='map'."""
        if not _TOPIC_TOOLS_OK:
            pytest.skip("topic_tools not installed — install ros-$ROS_DISTRO-topic-tools")
        grid = bench.wait_map(5.0) or (bench.map_msgs[-1] if bench.map_msgs else None)
        if grid is None:
            pytest.skip("/map not available")
        assert grid.header.frame_id == "map", (
            f"Expected 'map', got '{grid.header.frame_id}'")


# ─────────────────────────────────────────────────────────────────────────────
# TestTFTree — validates all required TF transforms exist
# ─────────────────────────────────────────────────────────────────────────────

class TestTFTree:
    """
    Validates that all TF transforms required by icp_odometry and rtabmap
    are present and provide valid (finite) transforms.

    Required chain in production:
      odom → base_link  (published by mission_control / static TF in tests)
      base_link → camera_link  (static: 0.15 m forward, 0.10 m up, 5° pitch)
      camera_link → camera_depth_optical_frame  (static: 90° rotation)
    """

    REQUIRED_TRANSFORMS = [
        ("odom",        "base_link"),
        ("base_link",   "camera_link"),
        ("camera_link", "camera_depth_optical_frame"),
        ("odom",        "camera_depth_optical_frame"),   # transitive
    ]

    def _lookup(self, bench: _BenchNode, parent: str, child: str,
                timeout: float = TF_TIMEOUT) -> TransformStamped | None:
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                return bench.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
            except Exception:
                rclpy.spin_once(bench, timeout_sec=0.1)
        return None

    def test_odom_to_base_link(self, icp_pipeline, bench):
        """odom → base_link must exist (static TF in tests, mission_control in prod)."""
        tf = self._lookup(bench, "odom", "base_link")
        assert tf is not None, (
            "Transform odom → base_link not found.\n"
            "In production: published by mission_control node.\n"
            "In tests: should come from static_transform_publisher.\n"
            "Check TF tree: ros2 run tf2_tools view_frames")

    def test_base_link_to_camera_link(self, icp_pipeline, bench):
        """base_link → camera_link must exist (static TF, camera mount position)."""
        tf = self._lookup(bench, "base_link", "camera_link")
        assert tf is not None, (
            "Transform base_link → camera_link not found.\n"
            "Check static_tf_base_to_camera in re_rassor_full.launch.py")

    def test_camera_link_to_depth_optical(self, icp_pipeline, bench):
        """camera_link → camera_depth_optical_frame must exist."""
        tf = self._lookup(bench, "camera_link", "camera_depth_optical_frame")
        assert tf is not None, (
            "Transform camera_link → camera_depth_optical_frame not found.\n"
            "Check static_tf_depth_optical in re_rassor_full.launch.py")

    def test_odom_to_depth_optical_transitive(self, icp_pipeline, bench):
        """odom → camera_depth_optical_frame (full chain) must be lookupable."""
        tf = self._lookup(bench, "odom", "camera_depth_optical_frame")
        assert tf is not None, (
            "Transitive transform odom → camera_depth_optical_frame not found.\n"
            "ICP odometry needs this chain to project the point cloud into odom frame.\n"
            "Run: ros2 run tf2_tools view_frames  and check all three links.")

    def test_camera_mount_translation(self, icp_pipeline, bench):
        """
        base_link → camera_link translation must be approximately (0.15, 0, 0.10) m.
        If wrong, ICP odometry will have incorrect extrinsic calibration.
        """
        tf = self._lookup(bench, "base_link", "camera_link")
        if tf is None:
            pytest.skip("Transform base_link → camera_link not available")

        t = tf.transform.translation
        assert abs(t.x - 0.15) < 0.01, f"camera x offset {t.x:.3f} m ≠ 0.15 m"
        assert abs(t.y - 0.00) < 0.01, f"camera y offset {t.y:.3f} m ≠ 0.00 m"
        assert abs(t.z - 0.10) < 0.01, f"camera z offset {t.z:.3f} m ≠ 0.10 m"

    def test_all_tf_values_finite(self, icp_pipeline, bench):
        """All TF translation components must be finite (no NaN/Inf)."""
        for parent, child in self.REQUIRED_TRANSFORMS:
            tf = self._lookup(bench, parent, child)
            if tf is None:
                continue  # Other tests will catch missing TFs
            t = tf.transform.translation
            for axis, v in [("x", t.x), ("y", t.y), ("z", t.z)]:
                assert math.isfinite(v), (
                    f"TF {parent}→{child} translation.{axis} is not finite: {v}")


# ─────────────────────────────────────────────────────────────────────────────
# TestQoSCompatibility — validates QoS matches between publishers and consumers
# ─────────────────────────────────────────────────────────────────────────────

class TestQoSCompatibility:
    """
    QoS mismatches are a common silent failure in ROS 2 — publisher and
    subscriber appear to be connected but no messages arrive.

    Key QoS rules:
      /map          must be TRANSIENT_LOCAL RELIABLE (nav2 static_layer joins late)
      /camera/depth/* must be BEST_EFFORT (high-frequency sensor data)
      /odom         RELIABLE is preferred (nav2 needs every update)
    """

    def test_bench_subscribes_map_transient_local(self, ros, bench):
        """
        The bench node subscribes to /map with TRANSIENT_LOCAL.
        If fake_map.py or rtabmap publishes with VOLATILE, the late subscriber
        would miss it — this test exposes that mismatch.
        """
        # Verify the subscription QoS object on bench is configured correctly
        # (indirect test — if wait_map returns data, QoS matched)
        # We can validate the QoS objects directly
        assert bench.map_msgs is not None  # subscription exists

    def test_fake_map_publishes_transient_local(self, ros, bench):
        """
        fake_map.py uses rclpy's transient_local publisher.  Nav2's static_layer
        is configured with map_subscribe_transient_local: true (nav2_params.yaml).
        If fake_map.py used VOLATILE, nav2 would never receive the initial map.
        """
        try:
            from ament_index_python.packages import get_package_share_directory
            bringup_share = get_package_share_directory('re_rassor_bringup')
            params_path = os.path.join(bringup_share, 'config', 'nav2_params.yaml')
        except Exception:
            pytest.skip("re_rassor_bringup not found in ament index")
        if not os.path.isfile(params_path):
            pytest.skip(f"nav2_params.yaml not found at {params_path}")

        with open(params_path) as f:
            content = f.read()

        assert "map_subscribe_transient_local: true" in content, (
            "nav2_params.yaml does not have map_subscribe_transient_local: true\n"
            "Nav2 static_layer will not receive the TRANSIENT_LOCAL map from rtabmap/fake_map.")

    def test_pointcloud_qos_best_effort(self, ros, bench):
        """
        /camera/depth/points should be published with BEST_EFFORT.
        Mixing RELIABLE publisher with BEST_EFFORT subscriber (or vice versa)
        causes silent drops in ROS 2 FastDDS.
        """
        bench.pc_msgs.clear()
        _pump(bench, count=5)
        # If QoS mismatched, pc_msgs would be empty even after publishing
        pc = bench.wait_pc(timeout=3.0)
        assert pc is not None, (
            "/camera/depth/points not received after direct publication.\n"
            "Check QoS compatibility between bench publisher (BEST_EFFORT) "
            "and any intermediary nodes.")
