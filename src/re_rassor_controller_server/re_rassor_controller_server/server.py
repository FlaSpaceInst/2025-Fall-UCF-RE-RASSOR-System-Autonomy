"""
RE-RASSOR Controller Server
────────────────────────────
Flask HTTP + WebSocket server that bridges the EZRassor controller app to
ROS2 topics, and streams live robot data to any connected browser/client.

HTTP endpoints (original):
  POST /           — wheel / shoulder / arm / drum / routine / target_coordinate
  GET  /           — health check
  OPTIONS /        — CORS preflight
  POST /navigate   — send a Nav2 goal pose (x, y, theta)
  POST /calibrate  — reset/calibrate odometry to the current pose as origin

WebSocket events (new):
  Client → Server:
    subscribe_odom      — start streaming /odom at ~10 Hz
    subscribe_map       — start streaming /map updates when map changes
    subscribe_points       — start streaming /camera/depth/points as
                             downsampled XYZ array for 3D visualisation
    subscribe_depth_image  — start streaming /camera/depth/image_raw as
                             base64-encoded 8-bit grayscale at ~10 Hz
    unsubscribe_*          — stop the corresponding stream

  Server → Client:
    odom                — {x, y, z, yaw, stamp}
    map                 — {width, height, resolution, origin_x, origin_y, data[]}
    points              — {points: [[x,y,z], ...], stamp}  (downsampled to ≤512 pts)
    tf_frames           — {frames: [...]}  list of active TF frames
    error               — {message}
"""

import math
import os
import socket
import subprocess
import sys
import termios
import threading
import time

import rclpy
import rclpy.node
import geometry_msgs.msg
import std_msgs.msg
import std_srvs.srv
from std_msgs.msg import Empty

try:
    import tf2_ros
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False

try:
    import sensor_msgs_py.point_cloud2 as pc2
    PC2_AVAILABLE = True
except ImportError:
    PC2_AVAILABLE = False

try:
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import PointCloud2, Image as DepthImage
    NAV_MSGS_AVAILABLE = True
except ImportError:
    NAV_MSGS_AVAILABLE = False

try:
    from tf2_ros import Buffer, TransformListener
    import tf_transformations
    TF_TRANSFORMS_AVAILABLE = True
except ImportError:
    TF_TRANSFORMS_AVAILABLE = False

try:
    from flask import Flask, request, jsonify
    from flask_cors import CORS
    import flask_socketio
    from flask_socketio import SocketIO, emit
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False

import re_rassor_controller_server as server

# ── ROS topic names ───────────────────────────────────────────────────────────
QUEUE_SIZE = 11
NODE_NAME  = 'controller_server'

AUTONOMY_TOPIC     = '/ezrassor/autonomy_instructions'
FRONT_ARM_TOPIC    = '/ezrassor/front_arm_instructions'
BACK_ARM_TOPIC     = '/ezrassor/back_arm_instructions'
ROUTINE_TOPIC      = '/ezrassor/routine_actions'
GOAL_POSE_TOPIC    = '/goal_pose'
CALIBRATE_SERVICE  = '/re_rassor/calibrate'

ODOM_TOPIC         = '/odometry/fused'
MAP_TOPIC          = '/map'
COSTMAP_TOPIC      = '/map'
POINTS_TOPIC       = '/camera/depth/points'
ANNOTATED_IMG_TOPIC = '/re_rassor/vision/annotated_image'
RAW_CAM_TOPIC       = '/camera/color/image_raw'   # fallback if vision node not running
DETECTIONS_TOPIC    = '/re_rassor/vision/detections'

# Max points to send per WebSocket frame (downsampled from full cloud)
MAX_POINTS         = 512
# How many odom messages to skip between WS broadcasts (~10 Hz at 50 Hz odom)
ODOM_SKIP          = 5
# Skip N-1 out of every N depth frames for the image stream (~10 Hz at 30 Hz)
DEPTH_IMG_SKIP     = 3

# ── Globals shared between ROS callbacks and Flask/SocketIO ──────────────────
_socketio          = None          # set in main()
_odom_counter      = 0
_last_map_seq      = -1
_last_costmap_time = 0.0           # wall-clock time of last costmap broadcast
_depth_img_counter = 0
_subscribers       = set()         # active WebSocket subscription types
_sub_lock          = threading.Lock()
_latest_odom       = None          # last odom payload dict, updated every message
_nav_proc          = None          # subprocess handle for active ros2 action send_goal
_nav_lock          = threading.Lock()

# ── Video feed globals ────────────────────────────────────────────────────────
_latest_video_frame = None         # latest JPEG bytes for /video_feed
_video_lock         = threading.Lock()
_latest_detection   = '{"detection":"false"}'  # JSON string from /re_rassor/vision/detections


def _get_ip_address():
    """Return formatted rover IP string used for namespaced topics."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return 'ip_' + ip.replace('.', '_')
    except Exception:
        return 'ezrassor'


def _serial_halt(port: str) -> bool:
    """
    Write 0xFF (HALT) directly to the wheel Arduino serial port, bypassing ROS.
    Uses the same baud / termios settings as serial_motor_controller.cpp.
    Returns True on success, False if the port could not be opened.
    """
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            tty = termios.tcgetattr(fd)
            # 115200 8N1, raw mode
            tty[0] = 0                         # iflag
            tty[1] = 0                         # oflag
            tty[2] = (termios.CS8 | termios.CLOCAL | termios.CREAD)  # cflag
            tty[3] = 0                         # lflag
            tty[4] = termios.B115200           # ispeed
            tty[5] = termios.B115200           # ospeed
            tty[6][termios.VMIN]  = 0
            tty[6][termios.VTIME] = 1
            termios.tcsetattr(fd, termios.TCSANOW, tty)
            os.write(fd, bytes([0xFF]))        # HALT command
            return True
        finally:
            os.close(fd)
    except OSError:
        return False


def _yaw_from_quaternion(q):
    """Extract yaw (radians) from a geometry_msgs Quaternion."""
    # ROS quaternion: (x, y, z, w)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _downsample_pointcloud(cloud_msg, max_pts=MAX_POINTS):
    """
    Extract XYZ points from a PointCloud2 message and return a downsampled
    list of [x, y, z] triples suitable for JSON serialisation.

    Downsampling uses uniform stride — fast, no extra dependencies.
    NaN / inf values are filtered out.
    """
    if not PC2_AVAILABLE:
        return []
    try:
        pts = list(pc2.read_points(
            cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True))
    except Exception:
        return []

    if not pts:
        return []

    stride = max(1, len(pts) // max_pts)
    result = []
    for i in range(0, len(pts), stride):
        x, y, z = pts[i]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            result.append([round(x, 3), round(y, 3), round(z, 3)])
        if len(result) >= max_pts:
            break
    return result


def main(passed_args=None):
    if not FLASK_AVAILABLE:
        print('[ERROR] flask, flask-cors, and flask-socketio are required:\n'
              '  pip3 install flask flask-cors flask-socketio',
              file=sys.stderr)
        return

    rclpy.init(args=passed_args)
    node = rclpy.create_node(NODE_NAME)

    # ── Parameters ────────────────────────────────────────────────────────────
    node.declare_parameter('wheel_port', '/dev/arduino_wheel')
    node.declare_parameter('drum_port',  '/dev/arduino_drum')
    node.declare_parameter('baud_rate',  115200)
    node.declare_parameter('sim_mode',   False)

    wheel_port = node.get_parameter('wheel_port').get_parameter_value().string_value
    drum_port  = node.get_parameter('drum_port').get_parameter_value().string_value
    baud_rate  = node.get_parameter('baud_rate').get_parameter_value().integer_value
    sim_mode   = node.get_parameter('sim_mode').get_parameter_value().bool_value

    # ── Launch re_rassor_full on startup (skipped in sim_mode) ───────────────
    if sim_mode:
        launch_proc = None
        node.get_logger().info(
            'sim_mode=True — skipping re_rassor_full launch '
            '(demo_sim.launch.py manages the full stack)'
        )
    else:
        launch_proc = subprocess.Popen([
            'ros2', 'launch', 're_rassor_bringup', 're_rassor_full.launch.py',
            f'wheel_port:={wheel_port}',
            f'drum_port:={drum_port}',
            f'baud_rate:={baud_rate}',
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    rover_name = _get_ip_address()
    rover_ip   = rover_name[3:].replace('_', '.') if rover_name.startswith('ip_') else rover_name

    WHEEL_TOPIC        = f'/{rover_name}/wheel_instructions'
    SHOULDER_TOPIC     = f'/{rover_name}/shoulder_instructions'
    FRONT_DRUM_TOPIC   = f'/{rover_name}/front_drum_instructions'
    BACK_DRUM_TOPIC    = f'/{rover_name}/back_drum_instructions'

    # ── Command publishers ────────────────────────────────────────────────────
    wheel_pub      = node.create_publisher(geometry_msgs.msg.Twist, WHEEL_TOPIC,      QUEUE_SIZE)
    shoulder_pub   = node.create_publisher(geometry_msgs.msg.Twist, SHOULDER_TOPIC,   QUEUE_SIZE)
    front_drum_pub = node.create_publisher(geometry_msgs.msg.Twist, FRONT_DRUM_TOPIC, QUEUE_SIZE)
    back_drum_pub  = node.create_publisher(geometry_msgs.msg.Twist, BACK_DRUM_TOPIC,  QUEUE_SIZE)
    autonomy_pub   = node.create_publisher(geometry_msgs.msg.Twist, AUTONOMY_TOPIC,   QUEUE_SIZE)
    front_arm_pub  = node.create_publisher(std_msgs.msg.Float64,    FRONT_ARM_TOPIC,  QUEUE_SIZE)
    back_arm_pub   = node.create_publisher(std_msgs.msg.Float64,    BACK_ARM_TOPIC,   QUEUE_SIZE)
    routine_pub    = node.create_publisher(std_msgs.msg.Int8,       ROUTINE_TOPIC,    QUEUE_SIZE)
    goal_pub       = node.create_publisher(
        geometry_msgs.msg.PoseStamped, GOAL_POSE_TOPIC, QUEUE_SIZE)
    cmd_vel_pub    = node.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', QUEUE_SIZE)

    # ── ROS → WebSocket bridge subscribers ───────────────────────────────────

    def _odom_cb(msg):
        """Cache latest odom and broadcast to subscribed WebSocket clients."""
        global _odom_counter, _latest_odom
        payload = {
            'x':     round(msg.pose.pose.position.x, 4),
            'y':     round(msg.pose.pose.position.y, 4),
            'z':     round(msg.pose.pose.position.z, 4),
            'yaw':   round(_yaw_from_quaternion(msg.pose.pose.orientation), 4),
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }
        _latest_odom = payload

        _odom_counter += 1
        if _odom_counter % ODOM_SKIP != 0:
            return
        with _sub_lock:
            if 'odom' not in _subscribers:
                return
        if _socketio is None:
            return
        try:
            _socketio.emit('odom', payload)
        except Exception as e:
            node.get_logger().warning(f'odom WS emit error: {e}')

    def _map_cb(msg):
        """Broadcast occupancy grid to subscribed clients when it changes."""
        global _last_map_seq
        with _sub_lock:
            if 'map' not in _subscribers:
                return
        if _socketio is None:
            return
        # Only send when map actually updates (width/height/stamp changed)
        stamp_sec = msg.header.stamp.sec
        if stamp_sec == _last_map_seq:
            return
        _last_map_seq = stamp_sec
        try:
            payload = {
                'width':      msg.info.width,
                'height':     msg.info.height,
                'resolution': msg.info.resolution,
                'origin_x':   round(msg.info.origin.position.x, 4),
                'origin_y':   round(msg.info.origin.position.y, 4),
                'data':       list(msg.data),   # int8[] — -1 unknown, 0 free, 100 occupied
                'stamp':      stamp_sec,
            }
            _socketio.emit('map', payload)
            node.get_logger().info(
                f'Map broadcast: {msg.info.width}x{msg.info.height} '
                f'@ {msg.info.resolution}m/cell')
        except Exception as e:
            node.get_logger().warning(f'map WS emit error: {e}')

    def _costmap_cb(msg):
        """Broadcast Nav2 global costmap to subscribed clients at most once per second."""
        global _last_costmap_time
        with _sub_lock:
            if 'costmap' not in _subscribers:
                return
        if _socketio is None:
            return
        now = time.time()
        if now - _last_costmap_time < 1.0:
            return
        _last_costmap_time = now
        try:
            payload = {
                'width':      msg.info.width,
                'height':     msg.info.height,
                'resolution': msg.info.resolution,
                'origin_x':   round(msg.info.origin.position.x, 4),
                'origin_y':   round(msg.info.origin.position.y, 4),
                'data':       list(msg.data),   # int8[] — -1 unknown, 0 free, 1-99 inflated, 100 lethal
                'stamp':      msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            }
            _socketio.emit('costmap', payload)
            node.get_logger().debug(
                f'Costmap broadcast: {msg.info.width}x{msg.info.height} '
                f'@ {msg.info.resolution}m/cell')
        except Exception as e:
            node.get_logger().warning(f'costmap WS emit error: {e}')

    def _points_cb(msg):
        """Broadcast downsampled depth point cloud to subscribed clients."""
        with _sub_lock:
            if 'points' not in _subscribers:
                return
        if _socketio is None:
            return
        try:
            pts = _downsample_pointcloud(msg)
            if not pts:
                return
            payload = {
                'points': pts,
                'stamp':  msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'frame':  msg.header.frame_id,
                'count':  len(pts),
            }
            _socketio.emit('points', payload)
        except Exception as e:
            node.get_logger().warning(f'points WS emit error: {e}')

    def _depth_image_cb(msg):
        """
        Normalise a 16UC1 (mm) or 32FC1 (m) depth image to 8-bit grayscale
        (0 = 0 m, 255 = 5 m) and broadcast raw bytes as base64 at ~10 Hz.
        """
        global _depth_img_counter
        _depth_img_counter += 1
        if _depth_img_counter % DEPTH_IMG_SKIP != 0:
            return
        with _sub_lock:
            if 'depth_image' not in _subscribers:
                return
        if _socketio is None:
            return
        try:
            import base64
            import numpy as np
            W, H = msg.width, msg.height
            if W == 0 or H == 0:
                return
            raw = bytes(msg.data)
            if msg.encoding in ('16UC1', 'mono16'):
                arr = np.frombuffer(raw, dtype=np.uint16).reshape((H, W))
                pixels = np.clip(arr.astype(np.float32) * (255.0 / 5000.0), 0, 255).astype(np.uint8)
            elif msg.encoding == '32FC1':
                arr = np.frombuffer(raw, dtype=np.float32).reshape((H, W))
                finite = np.isfinite(arr) & (arr > 0.0)
                pixels = np.where(finite, np.clip(arr * (255.0 / 5.0), 0, 255), 0).astype(np.uint8)
            else:
                return
            b64 = base64.b64encode(pixels.tobytes()).decode('ascii')
            _socketio.emit('depth_image', {
                'data':   b64,
                'width':  W,
                'height': H,
                'stamp':  msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            })
        except Exception as e:
            node.get_logger().warning(f'depth_image WS emit error: {e}')

    def _wheel_instr_cb(msg):
        """Relay wheel_instructions to /cmd_vel so manual commands override Nav2."""
        cmd_vel_pub.publish(msg)

    wheel_instr_relay_sub = node.create_subscription(
        geometry_msgs.msg.Twist, WHEEL_TOPIC, _wheel_instr_cb, QUEUE_SIZE)

    # ── Video frame callbacks ─────────────────────────────────────────────────

    def _image_to_jpeg(msg) -> bytes:
        """Convert a sensor_msgs/Image to JPEG bytes using numpy (no cv_bridge)."""
        try:
            import cv2 as _cv2
            import numpy as _np
            raw = bytes(msg.data)
            enc = msg.encoding
            if enc in ('rgb8',):
                arr = _np.frombuffer(raw, dtype=_np.uint8).reshape(
                    (msg.height, msg.width, 3))
                bgr = _cv2.cvtColor(arr, _cv2.COLOR_RGB2BGR)
            elif enc in ('bgr8',):
                bgr = _np.frombuffer(raw, dtype=_np.uint8).reshape(
                    (msg.height, msg.width, 3))
            elif enc in ('bgra8',):
                arr = _np.frombuffer(raw, dtype=_np.uint8).reshape(
                    (msg.height, msg.width, 4))
                bgr = _cv2.cvtColor(arr, _cv2.COLOR_BGRA2BGR)
            elif enc in ('rgba8',):
                arr = _np.frombuffer(raw, dtype=_np.uint8).reshape(
                    (msg.height, msg.width, 4))
                bgr = _cv2.cvtColor(arr, _cv2.COLOR_RGBA2BGR)
            elif enc in ('mono8', 'gray8'):
                gray = _np.frombuffer(raw, dtype=_np.uint8).reshape(
                    (msg.height, msg.width))
                bgr = _cv2.cvtColor(gray, _cv2.COLOR_GRAY2BGR)
            else:
                return None
            _, buf = _cv2.imencode('.jpg', bgr, [_cv2.IMWRITE_JPEG_QUALITY, 80])
            return buf.tobytes()
        except Exception as e:
            node.get_logger().warning(f'video frame encode error: {e}',
                                      throttle_duration_sec=5.0)
            return None

    def _annotated_img_cb(msg):
        """Store latest annotated (ArUco overlay) frame for /video_feed."""
        global _latest_video_frame
        jpeg = _image_to_jpeg(msg)
        if jpeg is not None:
            with _video_lock:
                _latest_video_frame = jpeg

    def _raw_cam_cb(msg):
        """Fallback: store raw camera frame when vision node is not running."""
        global _latest_video_frame
        # Only use raw if no annotated frame has arrived yet
        with _video_lock:
            if _latest_video_frame is not None:
                return
        jpeg = _image_to_jpeg(msg)
        if jpeg is not None:
            with _video_lock:
                _latest_video_frame = jpeg

    def _detections_cb(msg):
        """Cache latest ArUco detection JSON."""
        global _latest_detection
        _latest_detection = msg.data

    # Create ROS subscribers for streaming topics
    odom_sub         = node.create_subscription(Odometry,      ODOM_TOPIC,                _odom_cb,           10)
    map_sub          = node.create_subscription(OccupancyGrid, MAP_TOPIC,                 _map_cb,            10)
    costmap_sub      = node.create_subscription(OccupancyGrid, COSTMAP_TOPIC,             _costmap_cb,         1)
    points_sub       = node.create_subscription(PointCloud2,   POINTS_TOPIC,              _points_cb,          1)
    depth_img_sub    = node.create_subscription(DepthImage,    '/camera/depth/image_raw', _depth_image_cb,     1)
    annotated_sub    = node.create_subscription(DepthImage,    ANNOTATED_IMG_TOPIC,       _annotated_img_cb,   1)
    raw_cam_sub      = node.create_subscription(DepthImage,    RAW_CAM_TOPIC,             _raw_cam_cb,         1)
    detections_sub   = node.create_subscription(std_msgs.msg.String, DETECTIONS_TOPIC,   _detections_cb,     10)

    # ── Flask + SocketIO app ──────────────────────────────────────────────────
    app = Flask(__name__)
    CORS(app)

    global _socketio
    _socketio = SocketIO(
        app,
        cors_allowed_origins='*',
        async_mode='threading',
        logger=False,
        engineio_logger=False,
    )

    @app.after_request
    def _cors(response):
        response.headers['Access-Control-Allow-Origin']  = '*'
        response.headers['Access-Control-Allow-Methods'] = 'GET,POST,OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = 'Content-Type,Authorization'
        return response

    @app.route('/', methods=['OPTIONS'])
    def handle_options():
        return jsonify({'status': 200})

    @app.route('/', methods=['GET'])
    def default_get():
        return jsonify({'status': 200, 'node': NODE_NAME, 'ws_port': 5000})

    def _cancel_nav():
        """Cancel the active Nav2 goal and clean up the send_goal subprocess."""
        global _nav_proc
        # Tell the Nav2 action server to abort the goal.  Sending no goal-id
        # cancels all active goals on /navigate_to_pose.
        try:
            subprocess.run(
                ['ros2', 'action', 'cancel', '/navigate_to_pose'],
                timeout=2.0,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            node.get_logger().info('Nav2 goal cancelled via ros2 action cancel')
        except Exception as exc:
            node.get_logger().warning(f'ros2 action cancel failed: {exc}')
        # Also reap the send_goal subprocess so it doesn't become a zombie.
        with _nav_lock:
            if _nav_proc is not None and _nav_proc.poll() is None:
                _nav_proc.terminate()
            _nav_proc = None

    # ── POST / — main command endpoint ───────────────────────────────────────
    @app.route('/', methods=['POST'])
    def handle_request():
        try:
            data = request.get_json()

            # Legacy allStop format: {wheel_instruction: "none", ...}
            if isinstance(data, dict) and data.get('wheel_instruction') == 'none':
                _cancel_nav()
                stop = geometry_msgs.msg.Twist()
                wheel_pub.publish(stop)
                return jsonify({'status': 200})

            server.verify(data)
            cmd = server.create_command(data)

            if cmd.wheel_action is not None:
                _cancel_nav()   # manual wheel command preempts any active Nav2 goal
                msg = geometry_msgs.msg.Twist()
                msg.linear.x  = float(cmd.wheel_action.linear_x)
                msg.angular.z = float(cmd.wheel_action.angular_z)
                wheel_pub.publish(msg)

            if cmd.shoulder_action is not None:
                msg = geometry_msgs.msg.Twist()
                msg.linear.y  = float(cmd.shoulder_action.linear_y)
                msg.angular.y = float(cmd.shoulder_action.angular_y)
                msg.linear.x  = float(cmd.shoulder_action.linear_x)
                msg.angular.z = float(cmd.shoulder_action.angular_z)
                shoulder_pub.publish(msg)

            if cmd.front_drum_action is not None:
                msg = geometry_msgs.msg.Twist()
                msg.linear.x = float(cmd.front_drum_action.linear_x)
                front_drum_pub.publish(msg)

            if cmd.back_drum_action is not None:
                msg = geometry_msgs.msg.Twist()
                msg.linear.x = float(cmd.back_drum_action.linear_x)
                back_drum_pub.publish(msg)

            if cmd.target_coordinate is not None:
                msg = geometry_msgs.msg.Twist()
                msg.linear.x = float(cmd.target_coordinate.x)
                msg.linear.y = float(cmd.target_coordinate.y)
                autonomy_pub.publish(msg)

            if cmd.front_arm_action is not None:
                msg = std_msgs.msg.Float64()
                msg.data = cmd.front_arm_action.value
                front_arm_pub.publish(msg)

            if cmd.back_arm_action is not None:
                msg = std_msgs.msg.Float64()
                msg.data = cmd.back_arm_action.value
                back_arm_pub.publish(msg)

            if cmd.routine_action is not None:
                msg = std_msgs.msg.Int8()
                msg.data = cmd.routine_action.value
                routine_pub.publish(msg)

            return jsonify({'status': 200})

        except server.VerificationError as e:
            node.get_logger().error(f'Verification error: {e.message}')
            return jsonify({'status': 400, 'error': e.message}), 400
        except Exception as e:
            node.get_logger().error(f'Unexpected error: {e}')
            return jsonify({'status': 500, 'error': str(e)}), 500

    # ── POST /navigate ────────────────────────────────────────────────────────
    @app.route('/navigate', methods=['POST'])
    def handle_navigate():
        try:
            data = request.get_json()
            if data is None:
                return jsonify({'status': 400, 'error': 'missing JSON body'}), 400

            x     = float(data.get('x', 0.0))
            y     = float(data.get('y', 0.0))
            theta = float(data.get('theta', 0.0))

            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)
            # Send directly to Nav2 in map frame — frontend coordinates are
            # already map-frame so no rover→map conversion is applied.
            goal_yaml = (
                f'pose:\n'
                f'  header:\n'
                f'    frame_id: map\n'
                f'  pose:\n'
                f'    position:\n'
                f'      x: {x}\n'
                f'      y: {y}\n'
                f'      z: 0.0\n'
                f'    orientation:\n'
                f'      x: 0.0\n'
                f'      y: 0.0\n'
                f'      z: {qz}\n'
                f'      w: {qw}\n'
            )
            global _nav_proc
            _cancel_nav()   # cancel any existing goal before sending a new one
            with _nav_lock:
                _nav_proc = subprocess.Popen([
                    'ros2', 'action', 'send_goal',
                    '/navigate_to_pose',
                    'nav2_msgs/action/NavigateToPose',
                    goal_yaml,
                ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            node.get_logger().info(
                f'Navigate: map goal x={x:.3f} y={y:.3f} theta={theta:.3f}')

            # Log send_goal output in a background thread so we can see errors
            def _log_nav_output(proc):
                out, err = proc.communicate()
                if out:
                    node.get_logger().info(f'[send_goal stdout] {out.decode().strip()}')
                if err:
                    node.get_logger().error(f'[send_goal stderr] {err.decode().strip()}')
            import threading
            threading.Thread(target=_log_nav_output, args=(_nav_proc,), daemon=True).start()
            return jsonify({'status': 200, 'x': x, 'y': y, 'theta': theta})

        except Exception as e:
            node.get_logger().error(f'/navigate error: {e}')
            return jsonify({'status': 500, 'error': str(e)}), 500

    # ── POST /stop — force-stop: cancel Nav2 goal + halt wheels ─────────────
    @app.route('/stop', methods=['POST', 'OPTIONS'])
    def handle_stop():
        if request.method == 'OPTIONS':
            return jsonify({'status': 200})
        _cancel_nav()
        # Low-level serial HALT (0xFF) — skipped in sim_mode (no Arduino).
        serial_ok = False if sim_mode else _serial_halt(wheel_port)
        if not sim_mode and not serial_ok:
            node.get_logger().warning(
                f'Serial HALT failed on {wheel_port} — falling back to /cmd_vel only')
        # Also publish zero Twist so the ROS pipeline stays consistent.
        stop = geometry_msgs.msg.Twist()
        wheel_pub.publish(stop)
        cmd_vel_pub.publish(stop)
        node.get_logger().info(
            f'Force stop — serial HALT {"sent" if serial_ok else "FAILED"}, '
            'Nav2 cancelled, /cmd_vel zeroed')
        return jsonify({'status': 200, 'serial_halt': serial_ok})

    # ── GET /video_feed — MJPEG stream from annotated (or raw) camera ────────
    @app.route('/video_feed')
    def video_feed():
        def _generate():
            while True:
                with _video_lock:
                    frame = _latest_video_frame
                if frame:
                    yield (
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
                    )
                else:
                    time.sleep(0.05)
        from flask import Response, stream_with_context
        return Response(
            stream_with_context(_generate()),
            mimetype='multipart/x-mixed-replace; boundary=frame',
        )

    # ── GET /is_detection — latest ArUco detection JSON ───────────────────────
    @app.route('/is_detection')
    def is_detection():
        from flask import Response as _Resp
        return _Resp(_latest_detection, mimetype='application/json')

    # ── POST /calibrate ───────────────────────────────────────────────────────
    calibrate_client = node.create_client(std_srvs.srv.Trigger, CALIBRATE_SERVICE)

    @app.route('/calibrate', methods=['POST'])
    def handle_calibrate():
        try:
            if not calibrate_client.wait_for_service(timeout_sec=2.0):
                node.get_logger().warning('/re_rassor/calibrate service not available')
                return jsonify({'status': 503, 'error': 'calibrate service not available'}), 503

            future = calibrate_client.call_async(std_srvs.srv.Trigger.Request())
            # Wait up to 3 s for the service to respond
            deadline = time.time() + 3.0
            while not future.done() and time.time() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)

            if future.done():
                result = future.result()
                node.get_logger().info(
                    f'Calibrate: {result.message}')
                # Push reset event to all connected WebSocket clients
                if _socketio is not None:
                    _socketio.emit('calibrated', {'x': 0, 'y': 0, 'yaw': 0})
                return jsonify({'status': 200, 'message': result.message,
                                'success': result.success})
            else:
                node.get_logger().warning('Calibrate service call timed out')
                return jsonify({'status': 504, 'error': 'service call timed out'}), 504

        except Exception as e:
            node.get_logger().error(f'/calibrate error: {e}')
            return jsonify({'status': 500, 'error': str(e)}), 500

    # ── GET /odom ─────────────────────────────────────────────────────────────
    @app.route('/odom', methods=['GET'])
    def get_odom():
        if _latest_odom is None:
            return jsonify({'status': 503, 'error': 'no odometry received yet'}), 503
        return jsonify({'status': 200, **_latest_odom})

    # ── GET /{rover_name}/command_status ────────────────────��────────────────
    # Accepts any rover-name prefix so localhost connections (ip_127_0_0_1)
    # resolve to the same handler as the real IP (ip_10_x_x_x).
    @app.route('/<string:any_rover>/command_status', methods=['GET'])
    def command_status(any_rover):
        return jsonify({'status': 200, 'rover': rover_name})

    # ── WebSocket subscription handlers ──────────────────────────────────────

    @_socketio.on('connect')
    def on_connect():
        node.get_logger().info(f'WebSocket client connected')
        emit('connected', {'status': 'ok', 'node': NODE_NAME})

    @_socketio.on('disconnect')
    def on_disconnect():
        node.get_logger().info('WebSocket client disconnected')

    @_socketio.on('subscribe_odom')
    def on_subscribe_odom():
        with _sub_lock:
            _subscribers.add('odom')
        node.get_logger().info('WS: subscribed to odom')
        emit('subscribed', {'topic': 'odom'})

    @_socketio.on('unsubscribe_odom')
    def on_unsubscribe_odom():
        with _sub_lock:
            _subscribers.discard('odom')
        emit('unsubscribed', {'topic': 'odom'})

    @_socketio.on('subscribe_map')
    def on_subscribe_map():
        with _sub_lock:
            _subscribers.add('map')
        node.get_logger().info('WS: subscribed to map')
        emit('subscribed', {'topic': 'map'})

    @_socketio.on('unsubscribe_map')
    def on_unsubscribe_map():
        with _sub_lock:
            _subscribers.discard('map')
        emit('unsubscribed', {'topic': 'map'})

    @_socketio.on('subscribe_costmap')
    def on_subscribe_costmap():
        with _sub_lock:
            _subscribers.add('costmap')
        node.get_logger().info('WS: subscribed to global costmap')
        emit('subscribed', {'topic': 'costmap'})

    @_socketio.on('unsubscribe_costmap')
    def on_unsubscribe_costmap():
        with _sub_lock:
            _subscribers.discard('costmap')
        emit('unsubscribed', {'topic': 'costmap'})

    @_socketio.on('subscribe_points')
    def on_subscribe_points():
        with _sub_lock:
            _subscribers.add('points')
        node.get_logger().info('WS: subscribed to depth point cloud')
        emit('subscribed', {'topic': 'points'})

    @_socketio.on('unsubscribe_points')
    def on_unsubscribe_points():
        with _sub_lock:
            _subscribers.discard('points')
        emit('unsubscribed', {'topic': 'points'})

    @_socketio.on('subscribe_depth_image')
    def on_subscribe_depth_image():
        with _sub_lock:
            _subscribers.add('depth_image')
        node.get_logger().info('WS: subscribed to depth image stream')
        emit('subscribed', {'topic': 'depth_image'})

    @_socketio.on('unsubscribe_depth_image')
    def on_unsubscribe_depth_image():
        with _sub_lock:
            _subscribers.discard('depth_image')
        emit('unsubscribed', {'topic': 'depth_image'})

    @_socketio.on('request_map')
    def on_request_map():
        """Client can request an immediate map save via slam_toolbox service."""
        try:
            import subprocess
            subprocess.Popen([
                'ros2', 'service', 'call',
                '/slam_toolbox/save_map',
                'slam_toolbox/srv/SaveMap',
                '{name: {data: /tmp/rassor_map}}',
            ])
            emit('map_requested', {'status': 'ok'})
        except Exception as e:
            emit('error', {'message': str(e)})

    @_socketio.on('request_tf_frames')
    def on_request_tf_frames():
        """Return list of known TF frame IDs."""
        if not TF2_AVAILABLE:
            emit('tf_frames', {'frames': []})
            return
        try:
            buf = Buffer()
            listener = TransformListener(buf, node)
            time.sleep(0.5)
            frames = buf.all_frames_as_string()
            # Parse frame names from the verbose string output
            frame_list = []
            for line in frames.split('\n'):
                if 'Frame' in line and 'exists' in line:
                    parts = line.split()
                    if parts:
                        frame_list.append(parts[1])
            emit('tf_frames', {'frames': frame_list})
        except Exception as e:
            emit('tf_frames', {'frames': [], 'error': str(e)})

    # ── Start Flask+SocketIO in background thread ─────────────────────────────
    def _run_server():
        _socketio.run(
            app,
            host='0.0.0.0',
            port=5000,
            debug=False,
            use_reloader=False,
            log_output=False,
            allow_unsafe_werkzeug=True,
        )

    flask_thread = threading.Thread(target=_run_server, daemon=True)
    flask_thread.start()

    node.get_logger().info(
        f'\n'
        f'  ╔══════════════════════════════════════════════╗\n'
        f'  ║         RE-RASSOR Controller Server          ║\n'
        f'  ║                                              ║\n'
        f'  ║  Rover IP  : {rover_ip:<32} ║\n'
        f'  ║  HTTP      : http://{rover_ip}:5000/        \n'
        f'  ║  WebSocket : ws://{rover_ip}:5000/socket.io \n'
        f'  ║  Namespace : /{rover_name:<29} ║\n'
        f'  ║  Launch PID: {"sim_mode (no launch)" if launch_proc is None else launch_proc.pid:<32} ║\n'
        f'  ╚══════════════════════════════════════════════╝'
    )

    # ── Spin ROS node ─────────────────────────────────────────────────────────
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if launch_proc is not None:
            launch_proc.terminate()


if __name__ == '__main__':
    main()