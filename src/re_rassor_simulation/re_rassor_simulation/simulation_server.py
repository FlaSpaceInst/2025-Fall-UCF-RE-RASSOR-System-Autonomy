#!/usr/bin/env python3
"""
RE-RASSOR Simulation Server
============================
Textbook Nav2 + slam_toolbox + Gazebo Harmonic launch pattern.

Key design decisions (matching official Nav2 documentation):
  1. Gazebo DiffDrive publishes odom→base_footprint TF.
     A fixed joint base_footprint→base_link is handled by RSP.
     slam_toolbox base_frame must be base_footprint, NOT base_link.

  2. slam_toolbox is a lifecycle node. It must be configure+activated
     before it will subscribe to /scan or publish /map.
     The lifecycle manager handles this automatically when autostart:true.

  3. Nav2 lifecycle manager is separate from slam_toolbox lifecycle manager.
     Two managers: one for slam_toolbox, one for Nav2 stack.

  4. Scan filter replaces .inf ranges with range_max so slam_toolbox
     never receives an all-invalid scan.

  5. Launch order (textbook):
       t= 0s  Gazebo + bridge
       t= 3s  RSP (needs bridge up for /clock)
       t= 5s  depth→pointcloud, depth→scan, scan_filter
       t= 8s  slam_toolbox lifecycle manager (autostart → configure+activate)
       t=15s  mission_control (odom fusion)
       t=35s  Nav2 lifecycle manager (after slam has had time to build map)

Run with:
  ros2 run re_rassor_simulation simulation_server
"""

import base64
import math
import os
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import Float64, Int8, String
from action_msgs.srv import CancelGoal as ActionCancelGoal
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

try:
    import sensor_msgs_py.point_cloud2 as pc2
    _PC2_OK = True
except ImportError:
    _PC2_OK = False

try:
    import numpy as np
    _NP_OK = True
except ImportError:
    _NP_OK = False

try:
    import cv2 as _cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

try:
    from flask import (Flask, Response, request as flask_request,
                       jsonify, stream_with_context)
    from flask_cors import CORS
    from flask_socketio import SocketIO, emit
    _FLASK_OK = True
except ImportError:
    _FLASK_OK = False

try:
    import re_rassor_controller_server as _rcs
    _RCS_OK = True
except ImportError:
    _RCS_OK = False

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

FLASK_PORT  = 5000
QUEUE_SIZE  = 11
ODOM_SKIP   = 5
DEPTH_SKIP  = 3
MAX_POINTS  = 512

ODOM_TOPIC      = '/odometry/fused'
MAP_TOPIC       = '/map'
COSTMAP_TOPIC   = '/global_costmap/costmap'
POINTS_TOPIC    = '/camera/depth/points'
DEPTH_TOPIC     = '/camera/depth/image_raw'
COLOR_TOPIC     = '/camera/color/image_raw'
GOAL_POSE_TOPIC = '/goal_pose'
CALIBRATE_SRV   = '/re_rassor/calibrate'
AUTONOMY_TOPIC  = '/ezrassor/autonomy_instructions'
FRONT_ARM_TOPIC = '/ezrassor/front_arm_instructions'
BACK_ARM_TOPIC  = '/ezrassor/back_arm_instructions'
ROUTINE_TOPIC   = '/ezrassor/routine_actions'

# ─────────────────────────────────────────────────────────────────────────────
# Module-level shared state
# ─────────────────────────────────────────────────────────────────────────────

_sio:           'SocketIO | None' = None
_latest_odom:   'dict | None'    = None
_latest_detect: str               = '{"detection":"false"}'
_latest_frame:  'bytes | None'   = None

_subscribers: set            = set()
_sub_lock:    threading.Lock = threading.Lock()
_video_lock:  threading.Lock = threading.Lock()

_odom_ctr    = 0
_depth_ctr   = 0
_last_map_key = None
_last_cost_t  = 0.0

_nav_proc: 'subprocess.Popen | None' = None
_nav_lock:  threading.Lock           = threading.Lock()


# ─────────────────────────────────────────────────────────────────────────────
# Pure helpers
# ─────────────────────────────────────────────────────────────────────────────

def _local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return '127.0.0.1'


def _rover_ns(ip: str) -> str:
    return 'ip_' + ip.replace('.', '_')


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _downsample(msg, max_pts=MAX_POINTS):
    if not _PC2_OK:
        return []
    try:
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                   skip_nans=True))
    except Exception:
        return []
    if not pts:
        return []
    stride = max(1, len(pts) // max_pts)
    out = []
    for i in range(0, len(pts), stride):
        x, y, z = pts[i]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            out.append([round(x, 3), round(y, 3), round(z, 3)])
        if len(out) >= max_pts:
            break
    return out


def _depth_b64(msg) -> 'dict | None':
    if not _NP_OK:
        return None
    try:
        W, H = msg.width, msg.height
        if W == 0 or H == 0:
            return None
        raw = bytes(msg.data)
        enc = msg.encoding
        if enc in ('16UC1', 'mono16'):
            arr = np.frombuffer(raw, dtype=np.uint16).reshape((H, W))
            px  = np.clip(arr.astype(np.float32) * (255.0 / 5000.0),
                          0, 255).astype(np.uint8)
        elif enc == '32FC1':
            arr = np.frombuffer(raw, dtype=np.float32).reshape((H, W))
            fin = np.isfinite(arr) & (arr > 0.0)
            px  = np.where(fin, np.clip(arr * (255.0 / 8.0), 0, 255),
                           0).astype(np.uint8)
        else:
            return None
        return {
            'data':   base64.b64encode(px.tobytes()).decode('ascii'),
            'width':  W, 'height': H,
            'stamp':  msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }
    except Exception:
        return None


def _to_jpeg(msg) -> 'bytes | None':
    if not (_CV2_OK and _NP_OK):
        return None
    try:
        raw = bytes(msg.data)
        enc = msg.encoding
        if enc == 'rgb8':
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
            bgr = _cv2.cvtColor(arr, _cv2.COLOR_RGB2BGR)
        elif enc == 'bgr8':
            bgr = np.frombuffer(raw, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
        elif enc in ('bgra8', 'rgba8'):
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(
                (msg.height, msg.width, 4))
            code = (_cv2.COLOR_BGRA2BGR if enc == 'bgra8'
                    else _cv2.COLOR_RGBA2BGR)
            bgr = _cv2.cvtColor(arr, code)
        elif enc in ('mono8', 'gray8'):
            g = np.frombuffer(raw, dtype=np.uint8).reshape(
                (msg.height, msg.width))
            bgr = _cv2.cvtColor(g, _cv2.COLOR_GRAY2BGR)
        else:
            return None
        _, buf = _cv2.imencode('.jpg', bgr, [_cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes()
    except Exception:
        return None


def _rpy_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy)


def _static_tf(parent, child, x=0., y=0., z=0.,
               roll=0., pitch=0., yaw=0.) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp    = Time(sec=0, nanosec=0)
    t.header.frame_id = parent
    t.child_frame_id  = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    qw, qx, qy, qz = _rpy_quat(roll, pitch, yaw)
    t.transform.rotation.w = qw
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    return t


def _drain(proc: subprocess.Popen, label: str):
    for raw in proc.stderr:
        line = raw.decode('utf-8', errors='replace').rstrip()
        if line:
            print(f'[{label}] {line}', file=sys.stderr, flush=True)


def _delayed(secs: float, fn, *args):
    def _run():
        time.sleep(secs)
        fn(*args)
    threading.Thread(target=_run, daemon=True).start()


# ─────────────────────────────────────────────────────────────────────────────
# Proximity goal halt node (fallback)
# ─────────────────────────────────────────────────────────────────────────────

# ─────────────────────────────────────────────────────────────────────────────
# Main node
# ─────────────────────────────────────────────────────────────────────────────

class SimulationServer(Node):

    def __init__(self):
        super().__init__('simulation_server')

        if not _FLASK_OK:
            self.get_logger().error(
                'Missing Flask deps:\n  pip3 install flask flask-cors flask-socketio')
            raise RuntimeError('flask not installed')

        self._procs:     list = []
        self._tmp_files: list = []

        # ── identity ───────────────────────────────────────────────────────
        self._ip = _local_ip()
        self._rv = _rover_ns(self._ip)
        wheel_t    = f'/{self._rv}/wheel_instructions'
        shoulder_t = f'/{self._rv}/shoulder_instructions'
        fdrum_t    = f'/{self._rv}/front_drum_instructions'
        bdrum_t    = f'/{self._rv}/back_drum_instructions'

        # ── package paths ──────────────────────────────────────────────────
        sim_share         = get_package_share_directory('re_rassor_simulation')
        self._world       = os.path.join(sim_share, 'worlds', 'rassor_world.sdf')
        self._bridge_cfg  = os.path.join(sim_share, 'config', 'ros_gz_bridge.yaml')
        self._nav2_params = os.path.join(sim_share, 'config', 'nav2_params_sim.yaml')
        self._slam_params = os.path.join(sim_share, 'config',
                                         'slam_toolbox_params_sim.yaml')

        try:
            desc = get_package_share_directory('ezrassor_description')
            self._urdf = os.path.join(desc, 'urdf', 'ezrassor.urdf.xacro')
        except Exception:
            self._urdf = None

        # ── publishers ─────────────────────────────────────────────────────
        q = QoSProfile(depth=QUEUE_SIZE)
        self._cmd_vel_pub  = self.create_publisher(Twist,       '/cmd_vel',      q)
        self._wheel_pub    = self.create_publisher(Twist,       wheel_t,         q)
        self._shoulder_pub = self.create_publisher(Twist,       shoulder_t,      q)
        self._fdrum_pub    = self.create_publisher(Twist,       fdrum_t,         q)
        self._bdrum_pub    = self.create_publisher(Twist,       bdrum_t,         q)
        self._autonomy_pub = self.create_publisher(Twist,       AUTONOMY_TOPIC,  q)
        self._farm_pub     = self.create_publisher(Float64,     FRONT_ARM_TOPIC, q)
        self._barm_pub     = self.create_publisher(Float64,     BACK_ARM_TOPIC,  q)
        self._routine_pub  = self.create_publisher(Int8,        ROUTINE_TOPIC,   q)
        self._goal_pub     = self.create_publisher(PoseStamped, GOAL_POSE_TOPIC, q)

        # Scan filter: inf → range_max before slam_toolbox sees /scan
        self._scan_filtered_pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

        # Wheel odometry publisher — computed by integrating /cmd_vel
        self._odom_wheel_pub  = self.create_publisher(Odometry, '/odometry/wheel',  10)
        # Fused odometry = wheel only (no visual source in simulation)
        self._odom_fused_pub  = self.create_publisher(Odometry, '/odometry/fused',  10)

        # ── subscribers ────────────────────────────────────────────────────
        self.create_subscription(Twist,         wheel_t,       self._wheel_instr_cb, q)
        # Sim: subscribe to wheel odom directly (no mission_control fusion node)
        self.create_subscription(Odometry,      '/odometry/wheel', self._odom_cb,   10)
        self.create_subscription(OccupancyGrid, MAP_TOPIC,     self._map_cb,        10)
        self.create_subscription(OccupancyGrid, COSTMAP_TOPIC, self._cost_cb,        1)
        self.create_subscription(PointCloud2,   POINTS_TOPIC,  self._pts_cb,         1)
        self.create_subscription(Image,         DEPTH_TOPIC,   self._depth_cb,       1)
        self.create_subscription(Image,         COLOR_TOPIC,   self._color_cb,       1)
        self.create_subscription(LaserScan,     '/scan',       self._scan_filter_cb, 10)
        # Integrate /cmd_vel → /odometry/wheel using skid-steer kinematics
        # SDF geometry: wheel radius=0.10m, half-track=0.35m (track width=0.70m)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_odom_cb, 10)

        self._cal_client = self.create_client(Trigger, CALIBRATE_SRV)

        # ── wheel odometry integration state ───────────────────────────────
        self._wo_x          = 0.0
        self._wo_y          = 0.0
        self._wo_theta      = 0.0
        self._wo_vx         = 0.0   # latest commanded linear.x
        self._wo_wz         = 0.0   # latest commanded angular.z
        # Sim time from Gazebo /clock — used to stamp TF/odom AND to compute
        # integration dt.  Using sim time for dt ensures odom tracks actual physics
        # regardless of real-time factor (RTF).  Wall-clock dt at RTF<1 would
        # integrate faster than physics, placing the robot TF ahead of its physical
        # position so the goal checker could never fire.
        self._sim_stamp:      'Time | None' = None
        self._wo_last_sim_t:  float         = 0.0  # sim-time seconds at last tick
        # True once Gazebo bridge delivers /odometry/gz_wheel — disables cmd_vel
        # integration fallback so physics-accurate odom takes priority.
        self._gz_odom_received: bool = False
        self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        # Subscribe to Gazebo DiffDrive physics odometry (bridged via ros_gz_bridge).
        # This is the primary odometry source — more accurate than cmd_vel integration
        # because it tracks actual joint velocities including acceleration dynamics.
        self.create_subscription(Odometry, '/odometry/gz_wheel',
                                 self._gz_odom_cb, 10)
        # 20 Hz timer: cmd_vel integration fallback if bridge is not yet up
        self.create_timer(0.05, self._wheel_odom_timer_cb)

        # ── proximity goal halt (fallback) ─────────────────────────────────
        # Monitors distance to active Nav2 goal.  If within HALT_RADIUS before
        # Nav2's own goal checker fires, cancels the action and stops the rover.
        self._halt_radius  = 0.5    # metres
        self._halt_x:      float = 0.0
        self._halt_y:      float = 0.0
        self._halt_goal_x: float = 0.0
        self._halt_goal_y: float = 0.0
        self._halt_active: bool  = False   # True while a goal is being tracked
        self._halt_done:   bool  = False   # per-goal cooldown
        self._halt_lock = threading.Lock()
        # Subscribe to fused odom for current pose (separate from _odom_cb which
        # throttles for WS; this one needs every message for accurate distance)
        self.create_subscription(Odometry, '/odometry/fused',
                                 self._halt_odom_cb, 10)
        # bt_navigator publishes accepted goal on /goal_pose.  We also publish
        # to this topic ourselves in the navigate() route so the monitor fires
        # immediately without waiting for Nav2 to accept the action.
        self.create_subscription(PoseStamped, GOAL_POSE_TOPIC,
                                 self._halt_goal_cb, 10)
        # 10 Hz proximity check
        self.create_timer(0.1, self._halt_check_cb)
        # Service client to cancel active Nav2 action goals.
        # /navigate_to_pose/_action/cancel_goal cancels ALL active goals when
        # called with an empty request — reliable unlike the CLI which needs a UUID.
        self._nav_cancel_cli = self.create_client(
            ActionCancelGoal, '/navigate_to_pose/_action/cancel_goal')
        # Tracks how many more zero-vel ticks to publish after a halt fires
        # to override Nav2's controller server (which keeps publishing cmd_vel
        # until it receives the cancel acknowledgement and shuts down).
        self._halt_override_ticks: int = 0
        print(f'[proximity_halt] ready — halt radius {self._halt_radius} m',
              flush=True)

        # ── TF broadcasters ────────────────────────────────────────────────
        # Dynamic: odom→base_footprint published by _wheel_odom_timer_cb
        #          (TransformBroadcaster, updated at 20 Hz from odom math).
        # Static:  base_footprint→base_link, base_link→camera_link + optical frames.
        # Full chain: map→odom (slam_toolbox) → odom→base_footprint (this node)
        #           → base_footprint→base_link (static) → base_link→camera_link (static)
        self._tf_dyn = TransformBroadcaster(self)
        self._tf_bc  = StaticTransformBroadcaster(self)
        self._tf_bc.sendTransform([
            # map→odom = identity: slam_toolbox is configured to publish
            # map_slam→odom instead of map→odom, so we own this TF.
            # This makes nav2 plan in odom frame (accurate Gazebo physics)
            # rather than slam_toolbox's drifted map frame.
            _static_tf('map', 'odom'),
            _static_tf('base_footprint', 'base_link', z=0.325),
            _static_tf('base_link', 'camera_link', z=0.195),
            _static_tf('camera_link', 'camera_depth_optical_frame',
                       roll=-math.pi/2, yaw=-math.pi/2),
            _static_tf('camera_link', 'camera_color_optical_frame',
                       roll=-math.pi/2, yaw=-math.pi/2),
            _static_tf('camera_link', 'camera_depth_frame'),
            _static_tf('camera_link', 'camera_color_frame'),
            # rassor/camera_link/rgbd_camera is the frame_id Gazebo puts in
            # camera_info, so depthimage_to_laserscan uses it as the LaserScan
            # frame_id.
            #
            # depthimage_to_laserscan computes scan angle = atan2(x_opt, z_opt)
            # in the camera optical frame (+X_opt=right, +Z_opt=forward).
            # Result: angle=0 = boresight (forward), angle>0 = camera's RIGHT,
            # angle<0 = camera's LEFT.
            #
            # LaserScan consumers (slam_toolbox, costmaps) place each beam at
            # (r·cos θ, r·sin θ, 0) in the scan frame_id, i.e. angle=0 → +X,
            # angle>0 → +Y.  So the scan frame must satisfy:
            #   +X_scan = rover forward  (so angle=0 → forward)
            #   +Y_scan = rover RIGHT    (so angle>0 → rover's right)
            #
            # camera_link (ROS body) has +X=forward, +Y=LEFT.  With identity,
            # positive scan angles (camera right) land on +Y_scan = rover LEFT
            # → left/right mirror flip on the costmap.
            #
            # roll=π rotates around camera_link's X axis:
            #   +X_scan =  +X_camera_link = forward  ✓
            #   +Y_scan = −Y_camera_link = RIGHT    ✓
            #   +Z_scan = −Z_camera_link = down (irrelevant for 2-D scan)
            # This eliminates the mirror flip.
            _static_tf('camera_link', 'rassor/camera_link/rgbd_camera',
                       roll=math.pi),
        ])

        # ── textbook launch sequence ───────────────────────────────────────
        # slam_toolbox is NOT launched: it publishes map_slam→odom TF which
        # conflicts with our static map→odom=identity TF (both have child=odom,
        # giving odom two parents and breaking the TF tree).
        # Nav2 plans entirely in odom frame (map≡odom via identity TF).
        # The global costmap uses obstacle_layer from live scan — no static map needed.
        self._start_gazebo()
        self._start_bridge()
        _delayed(4.0,  self._start_sensors)
        _delayed(12.0, self._start_nav2)  # no slam → no /map wait needed

        self._setup_flask()

        headless = not bool(os.environ.get('DISPLAY', ''))
        self.get_logger().info(
            f'\n'
            f'  ╔══════════════════════════════════════════════════════╗\n'
            f'  ║     RE-RASSOR Simulation + Controller Server         ║\n'
            f'  ║  IP   : {self._ip:<45}║\n'
            f'  ║  HTTP : http://{self._ip}:{FLASK_PORT}/              \n'
            f'  ║  WS   : ws://{self._ip}:{FLASK_PORT}/socket.io       \n'
            f'  ║  Gz   : {"headless (-s)" if headless else "GUI"}     \n'
            f'  ║  t= 0s  Gazebo + bridge (/tf bridged → odom→base_fp)║\n'
            f'  ║  t= 4s  depthimage_to_laserscan → /scan → filter   ║\n'
            f'  ║  t=12s  Nav2 (no slam — map≡odom identity TF)        ║\n'
            f'  ║  TF: map→odom(static id)→base_footprint(gz)→base_link║\n'
            f'  ╚══════════════════════════════════════════════════════╝'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Subprocess launchers
    # ─────────────────────────────────────────────────────────────────────────

    def _spawn(self, cmd: list, label: str) -> subprocess.Popen:
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                stderr=subprocess.PIPE)
        threading.Thread(target=_drain, args=(proc, label),
                         daemon=True).start()
        self._procs.append(proc)
        self.get_logger().info(f'[sim] started: {label}')
        return proc

    def _start_gazebo(self):
        has_display = bool(os.environ.get('DISPLAY', ''))
        cmd = (['gz', 'sim', '-r', self._world] if has_display
               else ['gz', 'sim', '-s', '-r', self._world])
        self._spawn(cmd, 'gazebo')

    def _start_bridge(self):
        self._spawn([
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '--ros-args', '-p', f'config_file:={self._bridge_cfg}',
        ], 'gz_bridge')

    def _start_rsp(self):
        if not self._urdf:
            self.get_logger().warn('[sim] No URDF — using static TF fallback only')
            return
        try:
            r = subprocess.run(['xacro', self._urdf],
                               capture_output=True, text=True, check=True)
            urdf = r.stdout.strip()
        except (FileNotFoundError, subprocess.CalledProcessError) as e:
            self.get_logger().error(f'[sim] xacro failed: {e}')
            return
        params = {'robot_state_publisher': {'ros__parameters': {
            'robot_description': urdf, 'use_sim_time': True}}}
        tmp = tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', delete=False, prefix='/tmp/rassor_rsp_')
        yaml.dump(params, tmp, default_flow_style=False, allow_unicode=True)
        tmp.flush(); tmp.close()
        self._tmp_files.append(tmp.name)
        self._spawn([
            'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
            '--ros-args', '--params-file', tmp.name,
        ], 'rsp')

    def _start_sensors(self):
        """depth→pointcloud and depth→laserscan. Scan filter is inline in node."""
        self._spawn([
            'ros2', 'run', 'depth_image_proc', 'point_cloud_xyz_node',
            '--ros-args',
            '-r', 'image_rect:=/camera/depth/image_raw',
            '-r', 'camera_info:=/camera/depth/camera_info',
            '-r', 'points:=/camera/depth/points',
            '-p', 'use_sim_time:=true',
            '-p', 'queue_size:=20',
        ], 'depth_to_pointcloud')

        self._spawn([
            'ros2', 'run', 'depthimage_to_laserscan',
            'depthimage_to_laserscan_node',
            '--ros-args',
            '-r', 'depth:=/camera/depth/image_raw',
            '-r', 'depth_camera_info:=/camera/depth/camera_info',
            '-r', 'scan:=/scan',
            '-p', 'use_sim_time:=true',
            '-p', 'range_min:=0.10',
            '-p', 'range_max:=8.0',
            '-p', 'scan_height:=60',
        ], 'depth2scan')

    def _start_slam_lifecycle(self):
        """
        Textbook slam_toolbox startup via lifecycle manager.

        slam_toolbox_params_sim.yaml must have:
          slam_toolbox:
            ros__parameters:
              use_sim_time: true
              base_frame: base_footprint
              odom_frame: odom
              map_frame: map
              scan_topic: /scan_filtered
              mode: mapping
              map_start_at_dock: false
              map_update_interval: 5.0
              resolution: 0.05
              minimum_travel_distance: 0.1
              minimum_travel_heading: 0.1
        """
        self._spawn([
            'ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node',
            '--ros-args',
            '--params-file', self._slam_params,
            '-p', 'use_sim_time:=true',
        ], 'slam_toolbox')

        def _activate():
            time.sleep(2.0)
            self._spawn([
                'ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager',
                '--ros-args',
                '-r', '__node:=lifecycle_manager_slam',
                '-p', 'use_sim_time:=true',
                '-p', 'autostart:=true',
                '-p', 'bond_timeout:=0.0',
                '-p', 'node_names:=["slam_toolbox"]',
            ], 'lifecycle_mgr_slam')
            self.get_logger().info('[sim] slam_toolbox lifecycle manager started')

        threading.Thread(target=_activate, daemon=True).start()

    def _start_mission_ctrl(self):
        self._spawn([
            'ros2', 'run', 're_rassor_mission_control', 'mission_control',
            '--ros-args',
            '-p', 'wheel_odom_topic:=/odometry/wheel',
            '-p', 'visual_odom_topic:=/odom',
            '-p', 'fused_odom_topic:=/odometry/fused',
            '-p', 'visual_weight:=0.0',
            '-p', 'use_sim_time:=true',
        ], 'mission_ctrl')

    def _start_nav2_when_map_ready(self):
        self.get_logger().info('[sim] Waiting for /map before starting Nav2…')
        deadline = time.time() + 90.0
        while time.time() < deadline:
            if _last_map_key is not None and _last_map_key[0] > 10:
                self.get_logger().info(
                    f'[sim] /map ready ({_last_map_key[0]}×{_last_map_key[1]}'
                    f' cells) — starting Nav2')
                break
            time.sleep(3.0)
        else:
            self.get_logger().warn('[sim] /map not ready after 90s — starting Nav2 anyway')
        self._start_nav2()

    def _start_nav2(self):
        """
        Nav2 full stack.
        nav2_params_sim.yaml must have:
          global_costmap:
            rolling_window: false
            static_layer:
              map_subscribe_transient_local: true
          lifecycle_manager_navigation:
            node_names: [controller_server, planner_server,
                         behavior_server, bt_navigator, velocity_smoother]
        """
        for pkg, exe in [
            ('nav2_controller',        'controller_server'),
            ('nav2_planner',           'planner_server'),
            ('nav2_behaviors',         'behavior_server'),
            ('nav2_bt_navigator',      'bt_navigator'),
            ('nav2_velocity_smoother', 'velocity_smoother'),
        ]:
            self._spawn([
                'ros2', 'run', pkg, exe,
                '--ros-args', '--params-file', self._nav2_params,
            ], exe)

        time.sleep(2.0)

        self._spawn([
            'ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager',
            '--ros-args',
            '-r', '__node:=lifecycle_manager_navigation',
            '--params-file', self._nav2_params,
        ], 'lifecycle_mgr_nav2')

        self.get_logger().info('[sim] Nav2 full stack launched')

    # ─────────────────────────────────────────────────────────────────────────
    # ROS callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _scan_filter_cb(self, msg: LaserScan):
        """
        Replace inf/nan/out-of-range values with range_max.
        slam_toolbox discards scans that are entirely invalid.
        Replacing inf with range_max gives it free-space evidence at max range.
        """
        rmax = msg.range_max
        rmin = msg.range_min
        msg.ranges = [
            r if (math.isfinite(r) and rmin <= r <= rmax) else rmax
            for r in msg.ranges
        ]
        self._scan_filtered_pub.publish(msg)

    # ── proximity goal halt callbacks ─────────────────────────────────────────

    def _halt_odom_cb(self, msg: Odometry):
        with self._halt_lock:
            self._halt_x = msg.pose.pose.position.x
            self._halt_y = msg.pose.pose.position.y

    def _halt_goal_cb(self, msg: PoseStamped):
        with self._halt_lock:
            self._halt_goal_x = msg.pose.position.x
            self._halt_goal_y = msg.pose.position.y
            self._halt_active = True
            self._halt_done   = False
        print(f'[proximity_halt] tracking goal '
              f'({self._halt_goal_x:.2f}, {self._halt_goal_y:.2f})', flush=True)

    def _halt_check_cb(self):
        # Drain override ticks — keeps publishing zero cmd_vel after a halt
        # to override Nav2's controller server while cancel propagates.
        if self._halt_override_ticks > 0:
            self._cmd_vel_pub.publish(Twist())
            self._halt_override_ticks -= 1

        with self._halt_lock:
            if not self._halt_active or self._halt_done:
                return
            cx, cy = self._halt_x, self._halt_y
            gx, gy = self._halt_goal_x, self._halt_goal_y

        # In this simulation slam_toolbox keeps map≈odom (no significant drift),
        # so odom-frame pose and map-frame goal are directly comparable.
        dx   = cx - gx
        dy   = cy - gy
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self._halt_radius:
            print(f'[proximity_halt] DID REACH GOAL — '
                  f'dist={dist:.3f}m < {self._halt_radius}m | '
                  f'pose=({cx:.2f},{cy:.2f}) goal=({gx:.2f},{gy:.2f}) — halting',
                  flush=True)
            with self._halt_lock:
                self._halt_done   = True
                self._halt_active = False
            # Publish zero immediately
            self._cmd_vel_pub.publish(Twist())
            # Keep publishing zero for 3 s (30 ticks @ 10 Hz) to override
            # the controller server until cancel acknowledgement arrives.
            self._halt_override_ticks = 30
            # Cancel via the action cancel service — works without a goal UUID.
            if self._nav_cancel_cli.service_is_ready():
                future = self._nav_cancel_cli.call_async(ActionCancelGoal.Request())
                future.add_done_callback(self._halt_cancel_done_cb)
            else:
                print('[proximity_halt] cancel service not ready — rover may drift',
                      flush=True)
        else:
            print(f'[proximity_halt] DID NOT REACH GOAL — '
                  f'dist={dist:.3f}m | '
                  f'pose=({cx:.2f},{cy:.2f}) goal=({gx:.2f},{gy:.2f})',
                  flush=True)

    def _halt_cancel_done_cb(self, future):
        try:
            result = future.result()
            # returning_goals list is non-empty when cancel was accepted
            n = len(result.goals_canceling)
            print(f'[proximity_halt] cancel accepted — {n} goal(s) cancelling',
                  flush=True)
        except Exception as e:
            print(f'[proximity_halt] cancel service error: {e}', flush=True)

    # ─────────────────────────────────────────────────────────────────────────

    def _wheel_instr_cb(self, msg: Twist):
        self._cmd_vel_pub.publish(msg)

    def _clock_cb(self, msg: Clock):
        """Track Gazebo sim time so TF/odom stamps match use_sim_time nodes."""
        self._sim_stamp = msg.clock

    def _cmd_vel_odom_cb(self, msg: Twist):
        """Store latest commanded velocity for wheel odometry integration."""
        self._wo_vx = msg.linear.x
        self._wo_wz = msg.angular.z

    def _gz_odom_cb(self, msg: Odometry):
        """
        Primary odometry source: Gazebo DiffDrive physics odometry bridged to ROS.

        DiffDrive integrates actual joint velocities (not commanded velocities), so
        it correctly accounts for acceleration ramp-up/ramp-down as defined in the
        SDF (min/max_acceleration ±1.0 m/s²).  cmd_vel integration ignores this —
        at t=0 it assumes the rover is already at the commanded speed, causing the
        TF position to run ahead of actual Gazebo physics.  When Nav2's goal checker
        compares the TF pose to the goal, it thinks the rover has passed the goal,
        the controller tries to back up (allow_reversing=false → it U-turns instead),
        producing the "wildly off course" symptom.

        Using physics odom also gives more accurate heading integration — Gazebo
        physics computes differential steering torques on 4 wheels, so the heading
        drift is the physical simulation's actual drift, not a numerical integration
        artifact.
        """
        self._gz_odom_received = True

        # Mirror state into _wo_* so cmd_vel integration fallback stays consistent.
        self._wo_x     = msg.pose.pose.position.x
        self._wo_y     = msg.pose.pose.position.y
        self._wo_theta = _yaw(msg.pose.pose.orientation)
        self._wo_vx    = msg.twist.twist.linear.x
        self._wo_wz    = msg.twist.twist.angular.z

        # Re-publish on the standard topics that slam_toolbox / Nav2 consume.
        self._odom_wheel_pub.publish(msg)
        self._odom_fused_pub.publish(msg)

        # Broadcast odom→base_footprint TF.  frame_id and child_frame_id come
        # from the SDF (<frame_id>odom</frame_id>, <child_frame_id>base_footprint).
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = msg.header.stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id  = 'base_footprint'
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation      = msg.pose.pose.orientation
        self._tf_dyn.sendTransform(tf_msg)

    def _wheel_odom_timer_cb(self):
        """
        Fallback: cmd_vel integration when Gazebo bridge not yet delivering odom.
        Disabled automatically once /odometry/gz_wheel arrives (_gz_odom_received).

        dt is derived from Gazebo sim time (/clock), not wall clock.  This is
        critical: if wall-clock dt were used and RTF < 1.0, odom would accumulate
        faster than actual physics — the robot's TF position would be ahead of
        where it physically is, Nav2 would think the rover passed the goal, and
        the goal checker would never fire (rover goes wildly off course trying to
        U-turn back to a "missed" goal).  Sim-time dt keeps TF aligned with physics
        at any RTF.

        The timer still fires at 20 Hz wall-clock as a polling rate.  When
        _sim_stamp hasn't advanced (duplicate /clock tick), dt==0 and we skip to
        avoid duplicate-timestamp TF warnings in tf2.
        """
        # Gazebo physics odom takes over once bridge is up — no duplicate TF.
        if self._gz_odom_received:
            return

        stamp = self._sim_stamp
        if stamp is None:
            return  # /clock not yet bridged — wait before publishing TF

        current_sim_t = stamp.sec + stamp.nanosec * 1e-9
        dt = current_sim_t - self._wo_last_sim_t

        if dt <= 0.0:
            # Sim time hasn't advanced between wall-clock ticks — skip to avoid
            # publishing duplicate-timestamp TF transforms that confuse tf2.
            return
        if dt > 0.5:
            # Sim was paused or just resumed after a long gap — don't integrate
            # the stale velocity over a large dt jump; just reset the baseline.
            self._wo_last_sim_t = current_sim_t
            return

        self._wo_last_sim_t = current_sim_t

        vx = self._wo_vx
        wz = self._wo_wz
        th = self._wo_theta

        # Skid-steer forward kinematics.
        self._wo_x     +=  vx * math.cos(th) * dt
        self._wo_y     +=  vx * math.sin(th) * dt
        self._wo_theta += wz * dt

        half_th = self._wo_theta / 2.0

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x = self._wo_x
        odom.pose.pose.position.y = self._wo_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(half_th)
        odom.pose.pose.orientation.w = math.cos(half_th)

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = wz

        self._odom_wheel_pub.publish(odom)
        self._odom_fused_pub.publish(odom)

        # Publish dynamic odom→base_footprint TF so slam_toolbox + Nav2
        # can look up the robot pose without relying on Gazebo DiffDrive.
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id  = 'base_footprint'
        tf_msg.transform.translation.x = self._wo_x
        tf_msg.transform.translation.y = self._wo_y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.sin(half_th)
        tf_msg.transform.rotation.w = math.cos(half_th)
        self._tf_dyn.sendTransform(tf_msg)

    def _odom_cb(self, msg: Odometry):
        global _odom_ctr, _latest_odom
        payload = {
            'x':     round(msg.pose.pose.position.x, 4),
            'y':     round(msg.pose.pose.position.y, 4),
            'z':     round(msg.pose.pose.position.z, 4),
            'yaw':   round(_yaw(msg.pose.pose.orientation), 4),
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }
        _latest_odom = payload
        _odom_ctr += 1
        if _odom_ctr % ODOM_SKIP != 0:
            return
        with _sub_lock:
            if 'odom' not in _subscribers or _sio is None:
                return
        try:
            _sio.emit('odom', payload)
        except Exception as e:
            self.get_logger().warning(f'odom WS emit: {e}')

    def _map_cb(self, msg: OccupancyGrid):
        global _last_map_key
        key = (msg.info.width, msg.info.height, msg.header.stamp.sec)
        _last_map_key = key
        with _sub_lock:
            if 'map' not in _subscribers or _sio is None:
                return
        try:
            _sio.emit('map', {
                'width':      msg.info.width,
                'height':     msg.info.height,
                'resolution': msg.info.resolution,
                'origin_x':   round(msg.info.origin.position.x, 4),
                'origin_y':   round(msg.info.origin.position.y, 4),
                'data':       list(msg.data),
                'stamp':      msg.header.stamp.sec,
            })
            self.get_logger().info(
                f'[sim] /map → WS: {msg.info.width}×{msg.info.height} '
                f'@ {msg.info.resolution:.3f}m/cell')
        except Exception as e:
            self.get_logger().warning(f'map WS emit: {e}')

    def _cost_cb(self, msg: OccupancyGrid):
        global _last_cost_t
        with _sub_lock:
            if 'costmap' not in _subscribers or _sio is None:
                return
        now = time.time()
        if now - _last_cost_t < 1.0:
            return
        _last_cost_t = now
        try:
            _sio.emit('costmap', {
                'width':      msg.info.width,
                'height':     msg.info.height,
                'resolution': msg.info.resolution,
                'origin_x':   round(msg.info.origin.position.x, 4),
                'origin_y':   round(msg.info.origin.position.y, 4),
                'data':       list(msg.data),
                'stamp':      msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'source':     COSTMAP_TOPIC,
            })
        except Exception as e:
            self.get_logger().warning(f'costmap WS emit: {e}')

    def _pts_cb(self, msg: PointCloud2):
        with _sub_lock:
            if 'points' not in _subscribers or _sio is None:
                return
        pts = _downsample(msg)
        if not pts:
            return
        try:
            _sio.emit('points', {
                'points': pts,
                'stamp':  msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'frame':  msg.header.frame_id,
                'count':  len(pts),
            })
        except Exception as e:
            self.get_logger().warning(f'points WS emit: {e}')

    def _depth_cb(self, msg: Image):
        global _depth_ctr
        _depth_ctr += 1
        if _depth_ctr % DEPTH_SKIP != 0:
            return
        with _sub_lock:
            if 'depth_image' not in _subscribers or _sio is None:
                return
        payload = _depth_b64(msg)
        if payload is None:
            return
        try:
            _sio.emit('depth_image', payload)
        except Exception as e:
            self.get_logger().warning(f'depth_image WS emit: {e}')

    def _color_cb(self, msg: Image):
        global _latest_frame
        jpeg = _to_jpeg(msg)
        if jpeg is not None:
            with _video_lock:
                _latest_frame = jpeg

    # ─────────────────────────────────────────────────────────────────────────
    # Flask + Socket.IO
    # ─────────────────────────────────────────────────────────────────────────

    def _setup_flask(self):
        global _sio

        node = self
        rv   = self._rv

        app = Flask(__name__)
        CORS(app)
        _sio = SocketIO(app, cors_allowed_origins='*', async_mode='threading',
                        logger=False, engineio_logger=False)

        @app.after_request
        def _cors(r):
            r.headers['Access-Control-Allow-Origin']  = '*'
            r.headers['Access-Control-Allow-Methods'] = 'GET,POST,OPTIONS'
            r.headers['Access-Control-Allow-Headers'] = 'Content-Type,Authorization'
            return r

        def _cancel_nav():
            global _nav_proc
            # Cancel via action cancel service (works without a goal UUID).
            # subprocess CLI requires a UUID and silently does nothing without one.
            if node._nav_cancel_cli.service_is_ready():
                future = node._nav_cancel_cli.call_async(ActionCancelGoal.Request())
                future.add_done_callback(
                    lambda f: print(
                        f'[cancel_nav] cancel accepted — '
                        f'{len(f.result().goals_canceling)} goal(s) cancelling',
                        flush=True)
                    if not f.exception() else
                    print(f'[cancel_nav] cancel error: {f.exception()}', flush=True)
                )
            else:
                print('[cancel_nav] cancel service not ready', flush=True)
            # Also kill the send_goal subprocess if still running
            with _nav_lock:
                if _nav_proc is not None and _nav_proc.poll() is None:
                    _nav_proc.terminate()
                _nav_proc = None
            # Override cmd_vel for 3 s in case controller server lags
            node._halt_override_ticks = max(node._halt_override_ticks, 30)

        def _dispatch(data: dict):
            if data.get('wheel_instruction') == 'none':
                _cancel_nav()
                node._cmd_vel_pub.publish(Twist())
                node._wheel_pub.publish(Twist())
                return
            if _RCS_OK:
                _rcs.verify(data)
                cmd = _rcs.create_command(data)
                if cmd.wheel_action is not None:
                    _cancel_nav()
                    msg = Twist()
                    msg.linear.x  = float(cmd.wheel_action.linear_x)
                    msg.angular.z = float(cmd.wheel_action.angular_z)
                    node._wheel_pub.publish(msg)
                    node._cmd_vel_pub.publish(msg)
                if cmd.shoulder_action is not None:
                    msg = Twist()
                    msg.linear.y  = float(cmd.shoulder_action.linear_y)
                    msg.angular.y = float(cmd.shoulder_action.angular_y)
                    msg.linear.x  = float(cmd.shoulder_action.linear_x)
                    msg.angular.z = float(cmd.shoulder_action.angular_z)
                    node._shoulder_pub.publish(msg)
                if cmd.front_drum_action is not None:
                    m = Twist(); m.linear.x = float(cmd.front_drum_action.linear_x)
                    node._fdrum_pub.publish(m)
                if cmd.back_drum_action is not None:
                    m = Twist(); m.linear.x = float(cmd.back_drum_action.linear_x)
                    node._bdrum_pub.publish(m)
                if cmd.target_coordinate is not None:
                    m = Twist()
                    m.linear.x = float(cmd.target_coordinate.x)
                    m.linear.y = float(cmd.target_coordinate.y)
                    node._autonomy_pub.publish(m)
                if cmd.front_arm_action is not None:
                    m = Float64(); m.data = float(cmd.front_arm_action.value)
                    node._farm_pub.publish(m)
                if cmd.back_arm_action is not None:
                    m = Float64(); m.data = float(cmd.back_arm_action.value)
                    node._barm_pub.publish(m)
                if cmd.routine_action is not None:
                    m = Int8(); m.data = int(cmd.routine_action.value)
                    node._routine_pub.publish(m)
                return
            if 'wheel_action' in data:
                _cancel_nav()
                wa = data['wheel_action']
                msg = Twist()
                msg.linear.x  = float(wa.get('linear_x', 0.0))
                msg.angular.z = float(wa.get('angular_z', 0.0))
                node._wheel_pub.publish(msg)
                node._cmd_vel_pub.publish(msg)
            if 'target_coordinate' in data:
                tc = data['target_coordinate']
                m = Twist()
                m.linear.x = float(tc.get('x', 0.0))
                m.linear.y = float(tc.get('y', 0.0))
                node._autonomy_pub.publish(m)
            if 'routine_action' in data:
                _R = {'AUTO_DRIVE': 1, 'AUTO_DIG': 2, 'AUTO_DUMP': 4,
                      'AUTO_DOCK': 8, 'FULL_AUTONOMY': 16, 'STOP': 32}
                m = Int8(); m.data = _R.get(data['routine_action'], 0)
                node._routine_pub.publish(m)

        @app.route('/', methods=['OPTIONS'])
        def opts():
            return jsonify({'status': 200})

        @app.route('/', methods=['GET'])
        def health():
            return jsonify({'status': 200, 'node': 'simulation_server',
                            'ip': self._ip, 'ws_port': FLASK_PORT, 'sim': True})

        @app.route('/', methods=['POST'])
        def command():
            try:
                _dispatch(flask_request.get_json(force=True) or {})
                return jsonify({'status': 200})
            except Exception as e:
                node.get_logger().error(f'POST / error: {e}')
                return jsonify({'status': 400, 'error': str(e)}), 400

        @app.route('/navigate', methods=['POST'])
        def navigate():
            try:
                d     = flask_request.get_json(force=True) or {}
                x     = float(d.get('x', 0.0))
                y     = float(d.get('y', 0.0))
                theta = float(d.get('theta', 0.0))
                qz    = math.sin(theta / 2.0)
                qw    = math.cos(theta / 2.0)
                yaml_goal = (
                    f'pose:\n  header:\n    frame_id: map\n'
                    f'  pose:\n    position:\n      x: {x}\n      y: {y}\n'
                    f'      z: 0.0\n    orientation:\n      x: 0.0\n'
                    f'      y: 0.0\n      z: {qz}\n      w: {qw}\n'
                )
                global _nav_proc
                _cancel_nav()
                with _nav_lock:
                    _nav_proc = subprocess.Popen([
                        'ros2', 'action', 'send_goal',
                        '/navigate_to_pose',
                        'nav2_msgs/action/NavigateToPose',
                        yaml_goal,
                    ])
                node.get_logger().info(
                    f'Navigate: x={x:.3f} y={y:.3f} θ={theta:.3f}')
                # Notify proximity halt monitor immediately — don't wait for
                # bt_navigator to publish /goal_pose after action is accepted.
                goal_msg = PoseStamped()
                goal_msg.header.frame_id = 'map'
                goal_msg.pose.position.x = x
                goal_msg.pose.position.y = y
                goal_msg.pose.orientation.z = qz
                goal_msg.pose.orientation.w = qw
                node._goal_pub.publish(goal_msg)
                return jsonify({'status': 200, 'x': x, 'y': y, 'theta': theta})
            except Exception as e:
                return jsonify({'status': 500, 'error': str(e)}), 500

        @app.route('/stop', methods=['POST', 'OPTIONS'])
        def stop():
            if flask_request.method == 'OPTIONS':
                return jsonify({'status': 200})
            _cancel_nav()
            node._cmd_vel_pub.publish(Twist())
            node._wheel_pub.publish(Twist())
            return jsonify({'status': 200, 'serial_halt': False, 'sim': True})

        @app.route('/calibrate', methods=['POST'])
        def calibrate():
            try:
                if not node._cal_client.wait_for_service(timeout_sec=2.0):
                    return jsonify({'status': 503,
                                    'error': 'calibrate service not available'}), 503
                future   = node._cal_client.call_async(Trigger.Request())
                deadline = time.time() + 3.0
                while not future.done() and time.time() < deadline:
                    time.sleep(0.05)
                if future.done():
                    res = future.result()
                    if _sio:
                        _sio.emit('calibrated', {'x': 0, 'y': 0, 'yaw': 0})
                    return jsonify({'status': 200, 'message': res.message,
                                    'success': res.success})
                return jsonify({'status': 504, 'error': 'timeout'}), 504
            except Exception as e:
                return jsonify({'status': 500, 'error': str(e)}), 500

        @app.route('/odom', methods=['GET'])
        def odom_get():
            if _latest_odom is None:
                return jsonify({'status': 503, 'error': 'no odometry yet'}), 503
            return jsonify({'status': 200, **_latest_odom})

        @app.route(f'/{rv}/command_status', methods=['GET'])
        def cmd_status():
            return jsonify({'status': 200, 'rover': rv, 'sim': True})

        @app.route('/video_feed')
        def video_feed():
            def _gen():
                while True:
                    with _video_lock:
                        frame = _latest_frame
                    if frame:
                        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                               + frame + b'\r\n')
                    else:
                        time.sleep(0.05)
            return Response(stream_with_context(_gen()),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route('/is_detection')
        def is_detection():
            return Response(_latest_detect, mimetype='application/json')

        @_sio.on('connect')
        def on_connect():
            node.get_logger().info('WS client connected')
            emit('connected', {'status': 'ok', 'node': 'simulation_server',
                               'ip': self._ip, 'sim': True})

        @_sio.on('disconnect')
        def on_disconnect():
            node.get_logger().info('WS client disconnected')

        def _sub(name):
            with _sub_lock:
                _subscribers.add(name)
            node.get_logger().info(f'WS subscribed: {name}')
            emit('subscribed', {'topic': name})

        def _unsub(name):
            with _sub_lock:
                _subscribers.discard(name)
            emit('unsubscribed', {'topic': name})

        @_sio.on('subscribe_odom')
        def on_sub_odom():              _sub('odom')
        @_sio.on('unsubscribe_odom')
        def on_unsub_odom():            _unsub('odom')
        @_sio.on('subscribe_map')
        def on_sub_map():               _sub('map')
        @_sio.on('unsubscribe_map')
        def on_unsub_map():             _unsub('map')
        @_sio.on('subscribe_costmap')
        def on_sub_cost():              _sub('costmap')
        @_sio.on('unsubscribe_costmap')
        def on_unsub_cost():            _unsub('costmap')
        @_sio.on('subscribe_points')
        def on_sub_pts():               _sub('points')
        @_sio.on('unsubscribe_points')
        def on_unsub_pts():             _unsub('points')
        @_sio.on('subscribe_depth_image')
        def on_sub_depth():             _sub('depth_image')
        @_sio.on('unsubscribe_depth_image')
        def on_unsub_depth():           _unsub('depth_image')

        @_sio.on('request_map')
        def on_req_map():
            try:
                subprocess.Popen([
                    'ros2', 'service', 'call', '/slam_toolbox/save_map',
                    'slam_toolbox/srv/SaveMap',
                    '{name: {data: /tmp/rassor_map}}',
                ])
                emit('map_requested', {'status': 'ok'})
            except Exception as e:
                emit('error', {'message': str(e)})

        @_sio.on('request_tf_frames')
        def on_tf():
            try:
                from tf2_ros import Buffer, TransformListener
                buf = Buffer()
                TransformListener(buf, node)
                time.sleep(0.5)
                raw    = buf.all_frames_as_string()
                frames = [ln.split()[1] for ln in raw.split('\n')
                          if 'Frame' in ln and 'exists' in ln
                          and len(ln.split()) > 1]
                emit('tf_frames', {'frames': frames})
            except Exception as e:
                emit('tf_frames', {'frames': [], 'error': str(e)})

        threading.Thread(
            target=lambda: _sio.run(
                app, host='0.0.0.0', port=FLASK_PORT,
                debug=False, use_reloader=False,
                log_output=False, allow_unsafe_werkzeug=True),
            daemon=True).start()

    # ─────────────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info('[sim] Shutting down…')
        for proc in self._procs:
            try:
                proc.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
        for proc in self._procs:
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()
        for path in self._tmp_files:
            try:
                os.unlink(path)
            except OSError:
                pass
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SimulationServer()
    except RuntimeError:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()