"""
test_pipeline_bench.py
──────────────────────────────────────────────────────────────────────────────
Test bench for the RE-RASSOR depth-camera → laser-scan → SLAM pipeline.
Matches the production configuration in re_rassor_full.launch.py:

  depthimage_to_laserscan   /camera/depth/image_raw → /scan
  slam_toolbox              /scan + odom→base_link TF → /map + map→odom TF
  fake_map                  blank OccupancyGrid on /map (Nav2 bootstrap)
  depth_image_proc          /camera/depth/image_raw → /camera/depth/points
                            (used by Nav2 costmaps, not SLAM)

Test classes
────────────
  TestFakeMap               — blank OccupancyGrid publisher, QoS, dimensions
  TestDepthCameraMessages   — topic format, encoding, frame IDs, camera_info K
  TestDepthToLaserScan      — depthimage_to_laserscan → /scan validity
  TestSlamToolboxStartup    — slam_toolbox map→odom TF and /map published
  TestSlamToolboxMapping    — /map has SLAM content, correct frame / resolution
  TestTFChain               — full map→odom→base_link→camera chain present

Run:
    colcon build --packages-select re_rassor_test --cmake-args -DBUILD_TESTING=ON
    colcon test --packages-select re_rassor_test \\
        --pytest-args -k test_pipeline_bench -s
    colcon test-result --verbose
"""

import math
import os
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

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image, LaserScan

# ─────────────────────────────────────────────────────────────────────────────
# Astra Pro 320×240 intrinsics  (matches re_rassor_full.launch.py)
# ─────────────────────────────────────────────────────────────────────────────
ASTRA_W  = 320
ASTRA_H  = 240
ASTRA_FX = 277.13
ASTRA_FY = 277.13
ASTRA_CX = 160.0
ASTRA_CY = 120.0

# ─────────────────────────────────────────────────────────────────────────────
# Timing (seconds)
# ─────────────────────────────────────────────────────────────────────────────
SCAN_STARTUP     = 5.0    # depthimage_to_laserscan cold-start
SLAM_STARTUP     = 15.0   # slam_toolbox initialise + first map publish
COLLECT_SECS     = 3.0    # observation window per test
TF_LOOKUP_SECS   = 1.0    # tf2 lookup timeout

# ─────────────────────────────────────────────────────────────────────────────
# QoS profiles
# ─────────────────────────────────────────────────────────────────────────────
SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
TRANSIENT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# ─────────────────────────────────────────────────────────────────────────────
# Locate fake_map.py by walking up the directory tree
# ─────────────────────────────────────────────────────────────────────────────
_FAKE_MAP_SCRIPT: str | None = None
_candidate = os.path.dirname(os.path.realpath(__file__))
for _ in range(12):
    _candidate = os.path.dirname(_candidate)
    _probe = os.path.join(_candidate, "fake_map.py")
    if os.path.isfile(_probe):
        _FAKE_MAP_SCRIPT = _probe
        break


# ─────────────────────────────────────────────────────────────────────────────
# Message factories
# ─────────────────────────────────────────────────────────────────────────────

def _camera_info(stamp, frame_id: str) -> CameraInfo:
    msg = CameraInfo()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame_id
    msg.width           = ASTRA_W
    msg.height          = ASTRA_H
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [ASTRA_FX, 0.0,     ASTRA_CX,
             0.0,      ASTRA_FY, ASTRA_CY,
             0.0,      0.0,      1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [ASTRA_FX, 0.0,     ASTRA_CX, 0.0,
             0.0,      ASTRA_FY, ASTRA_CY, 0.0,
             0.0,      0.0,      1.0,      0.0]
    return msg


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


def _make_scene() -> np.ndarray:
    """
    Curved wall scene: background at 2.0 m, a box at 1.0 m in the centre.
    Gives depthimage_to_laserscan a mix of ranges to produce a textured scan.
    """
    depth = np.full((ASTRA_H, ASTRA_W), 2.0, dtype=np.float32)
    depth[80:160, 80:240] = 1.0
    return depth


def _make_moving_scene(frame_idx: int) -> np.ndarray:
    """
    Vary the box position per frame so slam_toolbox sees genuine scan change.
    """
    depth = np.full((ASTRA_H, ASTRA_W), 2.0, dtype=np.float32)
    shift = (frame_idx * 3) % (ASTRA_W // 2)
    c0 = max(0, 60 + shift)
    c1 = min(ASTRA_W, c0 + 160)
    depth[80:160, c0:c1] = 1.0 + frame_idx * 0.02
    return depth


# ─────────────────────────────────────────────────────────────────────────────
# Test node
# ─────────────────────────────────────────────────────────────────────────────

class _BenchNode(Node):
    """
    Publishes synthetic depth data and collects pipeline outputs.

    Publishers  (mimic Astra Pro driver):
      /camera/depth/image_raw     Image 32FC1
      /camera/depth/camera_info   CameraInfo

    Subscribers (pipeline outputs):
      /scan       LaserScan   (from depthimage_to_laserscan)
      /map        OccupancyGrid (from slam_toolbox or fake_map)
    """

    def __init__(self):
        super().__init__("re_rassor_bench_node")

        self.depth_pub      = self.create_publisher(
            Image,      "/camera/depth/image_raw",   SENSOR_QOS)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, "/camera/depth/camera_info", 10)

        self.scan_msgs: list[LaserScan]    = []
        self.map_msgs:  list[OccupancyGrid] = []

        self.create_subscription(
            LaserScan,    "/scan", lambda m: self.scan_msgs.append(m), SENSOR_QOS)
        self.create_subscription(
            OccupancyGrid, "/map", lambda m: self.map_msgs.append(m),  TRANSIENT_QOS)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._frame = 0

    def publish_depth_frame(self, depth: np.ndarray | None = None):
        if depth is None:
            depth = _make_moving_scene(self._frame)
        stamp = self.get_clock().now().to_msg()
        self.depth_pub.publish(_depth_msg(stamp, depth))
        self.depth_info_pub.publish(
            _camera_info(stamp, "camera_depth_optical_frame"))
        self._frame += 1

    def lookup_tf(self, parent: str, child: str):
        try:
            return self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=TF_LOOKUP_SECS))
        except Exception:
            return None


def _pump(node: _BenchNode, count: int = 30, hz: float = 10.0):
    """Publish `count` depth frames at `hz` Hz while spinning the node."""
    delay = 1.0 / hz
    for _ in range(count):
        node.publish_depth_frame()
        t0 = time.monotonic()
        while time.monotonic() - t0 < delay:
            rclpy.spin_once(node, timeout_sec=0.01)


def _spin(node: _BenchNode, secs: float, hz: float = 10.0):
    """Spin node for `secs` seconds, publishing depth frames continuously."""
    _pump(node, count=int(secs * hz), hz=hz)


def _wait_scan(node: _BenchNode, timeout: float) -> LaserScan | None:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        node.publish_depth_frame()
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.scan_msgs:
            return node.scan_msgs[-1]
    return None


def _wait_map(node: _BenchNode, timeout: float) -> OccupancyGrid | None:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        node.publish_depth_frame()
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.map_msgs:
            return node.map_msgs[-1]
    return None


# ─────────────────────────────────────────────────────────────────────────────
# Subprocess launchers
# ─────────────────────────────────────────────────────────────────────────────

_ENV = os.environ.copy()


def _stf_proc(x, y, z, yaw, pitch, roll, parent, child) -> subprocess.Popen:
    return subprocess.Popen(
        ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
         str(x), str(y), str(z), str(yaw), str(pitch), str(roll),
         parent, child],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=_ENV)


def _write_params(node_name: str, params: dict) -> str:
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='ros2_params_', delete=False)
    yaml.dump({node_name: {'ros__parameters': params}}, tmp)
    tmp.flush()
    tmp.close()
    return tmp.name


def _depth_laserscan_proc() -> subprocess.Popen:
    """depthimage_to_laserscan — mirrors production re_rassor_full.launch.py."""
    return subprocess.Popen(
        ['ros2', 'run', 'depthimage_to_laserscan', 'depthimage_to_laserscan_node',
         '--ros-args',
         '-r', 'depth:=/camera/depth/image_raw',
         '-r', 'depth_camera_info:=/camera/depth/camera_info',
         '-r', 'scan:=/scan',
         '-p', 'scan_height:=1',
         '-p', 'range_min:=0.45',
         '-p', 'range_max:=4.0',
         '-p', 'output_frame:=camera_link'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=_ENV)


def _slam_proc() -> subprocess.Popen:
    """
    async_slam_toolbox_node with test-friendly params:
    - minimum_travel_distance/heading = 0.0  → accept every scan (no movement needed)
    - minimum_time_interval = 0.2 s          → moderate rate limit
    - map_update_interval = 2.0 s            → fast map publishes for tests
    """
    params_file = _write_params('slam_toolbox', {
        'odom_frame':               'odom',
        'map_frame':                'map',
        'base_frame':               'base_link',
        'scan_topic':               '/scan',
        'mode':                     'mapping',
        'use_scan_matching':        True,
        'resolution':               0.05,
        'max_laser_range':          4.0,
        'minimum_time_interval':    0.2,
        'minimum_travel_distance':  0.0,
        'minimum_travel_heading':   0.0,
        'transform_publish_period': 0.1,
        'map_update_interval':      2.0,
        'debug_logging':            False,
        'stack_size_to_use':        40000000,
    })
    return subprocess.Popen(
        ['ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node',
         '--ros-args', '--params-file', params_file],
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


def _pkg_available(pkg: str) -> bool:
    return subprocess.run(
        ['ros2', 'pkg', 'prefix', pkg],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=_ENV,
    ).returncode == 0


def _slam_toolbox_functional() -> bool:
    """
    Returns True only if slam_toolbox is installed AND actually starts.

    On Jazzy/Ubuntu 24.04, slam_toolbox 2.8.3 hangs silently at startup
    (Ceres plugin loader deadlock) — it is installed but produces zero output.
    This check starts the node, waits up to 8 s for any stdout/stderr output,
    and returns False if none arrives (i.e. the node is non-functional here).
    """
    if not _pkg_available('slam_toolbox'):
        return False
    try:
        p = subprocess.Popen(
            ['ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=_ENV,
        )
        deadline = time.monotonic() + 8.0
        produced_output = False
        while time.monotonic() < deadline:
            # Non-blocking read: check if any bytes are ready
            import select
            ready, _, _ = select.select([p.stdout], [], [], 0.2)
            if ready:
                chunk = p.stdout.read1(4096)  # type: ignore[attr-defined]
                if chunk:
                    produced_output = True
                    break
            if p.poll() is not None:
                # Node exited on its own (crash counts as "responding")
                produced_output = True
                break
        try:
            p.terminate()
            p.wait(timeout=3)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass
        return produced_output
    except Exception:
        return False


_DEPTH_LASERSCAN_OK = _pkg_available('depthimage_to_laserscan')
_SLAM_TOOLBOX_OK    = _slam_toolbox_functional()


# ─────────────────────────────────────────────────────────────────────────────
# Fixtures
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def bench(ros):
    node = _BenchNode()
    yield node
    node.destroy_node()


@pytest.fixture(scope="module")
def static_tfs(ros):
    """Static TFs that must exist for the full pipeline."""
    procs = [
        _stf_proc(0,    0,    0,       0,      0,       0, "odom",        "base_link"),
        _stf_proc(0.15, 0,    0.10,    0,      0.087,   0, "base_link",   "camera_link"),
        _stf_proc(0,    0,    0,      -1.5708, 0, -1.5708, "camera_link", "camera_depth_optical_frame"),
    ]
    time.sleep(1.5)   # let /tf_static go out before any test tries to look them up
    yield procs
    _kill_all(procs)


@pytest.fixture(scope="module")
def depth_scan_pipeline(bench, static_tfs):
    """depthimage_to_laserscan running with synthetic depth input."""
    if not _DEPTH_LASERSCAN_OK:
        pytest.skip("depthimage_to_laserscan not installed")
    procs = [_depth_laserscan_proc()]
    _spin(bench, SCAN_STARTUP)     # publish depth while node warms up
    yield procs
    _kill_all(procs)


@pytest.fixture(scope="module")
def full_slam_pipeline(bench, depth_scan_pipeline):
    """slam_toolbox stacked on top of the already-running depth_scan_pipeline."""
    if not _SLAM_TOOLBOX_OK:
        pytest.skip("slam_toolbox not installed or non-functional on this platform")
    procs = [_slam_proc()]
    _spin(bench, SLAM_STARTUP)     # keep publishing; slam_toolbox needs scan input
    yield procs
    _kill_all(procs)


# ─────────────────────────────────────────────────────────────────────────────
# TestFakeMap
# ─────────────────────────────────────────────────────────────────────────────

class TestFakeMap:
    """
    Validates fake_map.py — the blank OccupancyGrid publisher used to bootstrap
    Nav2 before slam_toolbox builds a real map.
    """

    def test_fake_map_script_found(self):
        assert _FAKE_MAP_SCRIPT is not None, (
            "fake_map.py not found in any ancestor directory. "
            "Ensure it lives at the workspace root."
        )

    def test_fake_map_published(self, bench):
        if _FAKE_MAP_SCRIPT is None:
            pytest.skip("fake_map.py not found")
        bench.map_msgs.clear()
        proc = subprocess.Popen(
            ['python3', _FAKE_MAP_SCRIPT],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=_ENV)
        try:
            msg = _wait_map(bench, timeout=6.0)
            assert msg is not None, "/map not received from fake_map.py"
        finally:
            proc.terminate()
            proc.wait(timeout=3)

    def test_fake_map_all_free(self, bench):
        """Blank map must have all cells == 0 (free space)."""
        if not bench.map_msgs:
            pytest.skip("No /map received — run test_fake_map_published first")
        msg = bench.map_msgs[-1]
        non_free = [c for c in msg.data if c != 0]
        assert len(non_free) == 0, (
            f"fake_map has {len(non_free)} non-free cells — "
            "it should be a blank free-space grid"
        )

    def test_fake_map_qos_transient_local(self, bench):
        """
        /map must use TRANSIENT_LOCAL so Nav2 static_layer receives it even
        if Nav2 subscribes after fake_map.py published.
        We verify this indirectly: the bench node subscribes with TRANSIENT_LOCAL
        QoS and must have received at least one map message.
        """
        assert bench.map_msgs, (
            "/map never received with TRANSIENT_LOCAL QoS — "
            "fake_map.py may be publishing with VOLATILE"
        )

    def test_fake_map_has_positive_dimensions(self, bench):
        if not bench.map_msgs:
            pytest.skip("No /map received")
        msg = bench.map_msgs[-1]
        assert msg.info.width > 0
        assert msg.info.height > 0
        assert msg.info.resolution > 0.0

    def test_fake_map_frame_is_map(self, bench):
        if not bench.map_msgs:
            pytest.skip("No /map received")
        assert bench.map_msgs[-1].header.frame_id == "map"


# ─────────────────────────────────────────────────────────────────────────────
# TestDepthCameraMessages
# ─────────────────────────────────────────────────────────────────────────────

class TestDepthCameraMessages:
    """
    Validates the synthetic depth publisher output — the same format the
    Astra Pro driver produces.  No external nodes needed.
    """

    def test_depth_image_published(self, bench):
        bench.publish_depth_frame()
        rclpy.spin_once(bench, timeout_sec=0.1)

    def test_depth_encoding_32fc1(self, bench):
        stamp = bench.get_clock().now().to_msg()
        msg = _depth_msg(stamp, _make_scene())
        assert msg.encoding == "32FC1"

    def test_depth_dimensions(self, bench):
        stamp = bench.get_clock().now().to_msg()
        msg = _depth_msg(stamp, _make_scene())
        assert msg.width  == ASTRA_W
        assert msg.height == ASTRA_H

    def test_depth_step_matches_width(self, bench):
        stamp = bench.get_clock().now().to_msg()
        msg = _depth_msg(stamp, _make_scene())
        assert msg.step == ASTRA_W * 4   # 4 bytes per float32

    def test_depth_data_length(self, bench):
        stamp = bench.get_clock().now().to_msg()
        msg = _depth_msg(stamp, _make_scene())
        assert len(msg.data) == ASTRA_W * ASTRA_H * 4

    def test_depth_frame_id(self, bench):
        stamp = bench.get_clock().now().to_msg()
        msg = _depth_msg(stamp, _make_scene())
        assert msg.header.frame_id == "camera_depth_optical_frame"

    def test_camera_info_intrinsics(self, bench):
        stamp = bench.get_clock().now().to_msg()
        info = _camera_info(stamp, "camera_depth_optical_frame")
        assert abs(info.k[0] - ASTRA_FX) < 0.01   # fx
        assert abs(info.k[4] - ASTRA_FY) < 0.01   # fy
        assert abs(info.k[2] - ASTRA_CX) < 0.01   # cx
        assert abs(info.k[5] - ASTRA_CY) < 0.01   # cy

    def test_camera_info_dimensions(self, bench):
        stamp = bench.get_clock().now().to_msg()
        info = _camera_info(stamp, "camera_depth_optical_frame")
        assert info.width  == ASTRA_W
        assert info.height == ASTRA_H

    def test_scene_contains_foreground_and_background(self):
        depth = _make_scene()
        assert np.any(depth < 1.5), "scene should have near objects"
        assert np.any(depth > 1.5), "scene should have far background"


# ─────────────────────────────────────────────────────────────────────────────
# TestDepthToLaserScan
# ─────────────────────────────────────────────────────────────────────────────

class TestDepthToLaserScan:
    """
    Validates depthimage_to_laserscan:
      /camera/depth/image_raw  →  /scan
    """

    def test_scan_published(self, bench, depth_scan_pipeline):
        bench.scan_msgs.clear()
        msg = _wait_scan(bench, timeout=5.0)
        assert msg is not None, (
            "/scan not received — depthimage_to_laserscan may not be running "
            "or not receiving depth images"
        )

    def test_scan_frame_id(self, bench, depth_scan_pipeline):
        if not bench.scan_msgs:
            pytest.skip("No /scan received")
        assert bench.scan_msgs[-1].header.frame_id == "camera_link", (
            "scan frame_id should be camera_link (output_frame param)"
        )

    def test_scan_has_finite_ranges(self, bench, depth_scan_pipeline):
        """At least some ranges must be finite — scene has objects in range."""
        if not bench.scan_msgs:
            pytest.skip("No /scan received")
        msg = bench.scan_msgs[-1]
        finite = [r for r in msg.ranges if math.isfinite(r)]
        assert len(finite) > 0, (
            "all scan ranges are inf — depthimage_to_laserscan sees nothing "
            "within [range_min, range_max]. Check depth image content."
        )

    def test_scan_ranges_within_bounds(self, bench, depth_scan_pipeline):
        """Finite ranges must be within [0.45, 4.0] m (production config)."""
        if not bench.scan_msgs:
            pytest.skip("No /scan received")
        msg = bench.scan_msgs[-1]
        for r in msg.ranges:
            if math.isfinite(r):
                assert 0.45 <= r <= 4.0, f"range {r:.3f} m outside [0.45, 4.0]"

    def test_scan_angle_coverage(self, bench, depth_scan_pipeline):
        """Scan should span at least 30° — a 320-pixel-wide image gives ~63°."""
        if not bench.scan_msgs:
            pytest.skip("No /scan received")
        msg = bench.scan_msgs[-1]
        span = abs(msg.angle_max - msg.angle_min)
        assert span > math.radians(30), (
            f"scan spans only {math.degrees(span):.1f}° — expected ≥30°"
        )

    def test_scan_hz_above_threshold(self, bench, depth_scan_pipeline):
        """Scan rate must be ≥ 5 Hz — our depth publisher runs at 10 Hz."""
        bench.scan_msgs.clear()
        _spin(bench, COLLECT_SECS)
        hz = len(bench.scan_msgs) / COLLECT_SECS
        assert hz >= 5.0, (
            f"/scan published at {hz:.1f} Hz — expected ≥5 Hz. "
            "depthimage_to_laserscan may be dropping depth messages."
        )


# ─────────────────────────────────────────────────────────────────────────────
# TestSlamToolboxStartup
# ─────────────────────────────────────────────────────────────────────────────

class TestSlamToolboxStartup:
    """
    Validates that slam_toolbox starts correctly and immediately publishes
    the map→odom TF and /map topic.
    """

    def test_map_received(self, bench, full_slam_pipeline):
        bench.map_msgs.clear()
        msg = _wait_map(bench, timeout=10.0)
        assert msg is not None, (
            "/map not received from slam_toolbox — check that /scan is "
            "publishing and the odom→base_link TF is present"
        )

    def test_map_odom_tf_published(self, bench, full_slam_pipeline):
        """slam_toolbox must publish map→odom TF (its primary output)."""
        _spin(bench, 2.0)   # give TF time to arrive
        tf = bench.lookup_tf("map", "odom")
        assert tf is not None, (
            "map→odom TF not found — slam_toolbox is not publishing its "
            "localisation transform"
        )

    def test_map_frame_id_is_map(self, bench, full_slam_pipeline):
        if not bench.map_msgs:
            pytest.skip("No /map received")
        assert bench.map_msgs[-1].header.frame_id == "map"

    def test_map_resolution_matches_config(self, bench, full_slam_pipeline):
        """Resolution must match slam_toolbox_params.yaml (0.05 m)."""
        if not bench.map_msgs:
            pytest.skip("No /map received")
        assert abs(bench.map_msgs[-1].info.resolution - 0.05) < 1e-4

    def test_map_has_positive_dimensions(self, bench, full_slam_pipeline):
        if not bench.map_msgs:
            pytest.skip("No /map received")
        msg = bench.map_msgs[-1]
        assert msg.info.width  > 0
        assert msg.info.height > 0


# ─────────────────────────────────────────────────────────────────────────────
# TestSlamToolboxMapping
# ─────────────────────────────────────────────────────────────────────────────

class TestSlamToolboxMapping:
    """
    Validates that slam_toolbox builds a map with actual SLAM content
    (occupied / unknown cells) after receiving laser scans.
    """

    def test_map_has_slam_content(self, bench, full_slam_pipeline):
        """
        /map must have at least some non-free cells (occupied or unknown).
        All-zero = still blank / fake map.  Any non-zero = slam_toolbox is
        marking obstacles and/or unknown space.
        """
        # Wait for a map update after the startup window
        bench.map_msgs.clear()
        msg = _wait_map(bench, timeout=15.0)
        assert msg is not None, "No /map received after pipeline startup"
        non_free = [c for c in msg.data if c != 0]
        assert len(non_free) > 0, (
            "slam_toolbox /map is all-free (all zeros) — SLAM is not marking "
            "obstacles. Check /scan has valid ranges and odom→base_link TF exists."
        )

    def test_map_cell_count_nonzero(self, bench, full_slam_pipeline):
        if not bench.map_msgs:
            pytest.skip("No /map received")
        msg = bench.map_msgs[-1]
        assert len(msg.data) == msg.info.width * msg.info.height

    def test_map_updates_over_time(self, bench, full_slam_pipeline):
        """
        slam_toolbox must publish more than one /map message during the test —
        confirming it is actively processing scans, not stuck after the first.
        """
        bench.map_msgs.clear()
        _spin(bench, 10.0)    # 10 s of continuous scan + spin
        assert len(bench.map_msgs) >= 2, (
            f"slam_toolbox published only {len(bench.map_msgs)} /map message(s) "
            "in 10 s — expected ≥2 (map_update_interval=2.0 s in test config)"
        )

    def test_map_odom_tf_stable(self, bench, full_slam_pipeline):
        """map→odom TF must be present throughout the test, not just at startup."""
        _spin(bench, 2.0)
        tf = bench.lookup_tf("map", "odom")
        assert tf is not None, "map→odom TF disappeared after initial publish"


# ─────────────────────────────────────────────────────────────────────────────
# TestTFChain
# ─────────────────────────────────────────────────────────────────────────────

class TestTFChain:
    """
    Validates the complete TF chain required for Nav2:
      map → odom → base_link → camera_link → camera_depth_optical_frame

    map→odom        : slam_toolbox (localisation)
    odom→base_link  : static_transform_publisher (wheel odom in production)
    base_link→camera: static_transform_publisher in launch file
    """

    def test_odom_base_link_tf(self, bench, static_tfs):
        _spin(bench, 1.0)
        tf = bench.lookup_tf("odom", "base_link")
        assert tf is not None, "odom→base_link TF missing"

    def test_base_link_camera_link_tf(self, bench, static_tfs):
        _spin(bench, 1.0)
        tf = bench.lookup_tf("base_link", "camera_link")
        assert tf is not None, "base_link→camera_link TF missing"
        t = tf.transform.translation
        assert abs(t.x - 0.15) < 0.01, f"camera_link x translation wrong: {t.x}"
        assert abs(t.z - 0.10) < 0.01, f"camera_link z translation wrong: {t.z}"

    def test_camera_link_optical_tf(self, bench, static_tfs):
        _spin(bench, 1.0)
        tf = bench.lookup_tf("camera_link", "camera_depth_optical_frame")
        assert tf is not None, "camera_link→camera_depth_optical_frame TF missing"

    def test_map_odom_tf_from_slam(self, bench, full_slam_pipeline):
        """map→odom TF must come from slam_toolbox (not a static publisher)."""
        _spin(bench, 2.0)
        tf = bench.lookup_tf("map", "odom")
        assert tf is not None, (
            "map→odom TF missing — slam_toolbox must publish this "
            "(mission_control no longer publishes a static map→odom)"
        )

    def test_full_chain_map_to_optical(self, bench, full_slam_pipeline):
        """
        Full chain map→camera_depth_optical_frame must be lookupable —
        this is what Nav2 uses to project obstacles from the depth frame
        into the map frame.
        """
        _spin(bench, 2.0)
        tf = bench.lookup_tf("map", "camera_depth_optical_frame")
        assert tf is not None, (
            "Cannot look up map→camera_depth_optical_frame — one or more "
            "links in the TF chain are broken"
        )
