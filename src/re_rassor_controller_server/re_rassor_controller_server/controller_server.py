#!/usr/bin/env python3
# Copyright 2025 UCF RE-RASSOR
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""HTTP controller server bridging the EZRassor tablet app to ROS2 topics."""

import threading

import rclpy
from flask import Flask, jsonify, request
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int8

# ── ROS topic names ───────────────────────────────────────────────────────────
_WHEEL_TOPIC      = '/ezrassor/wheel_instructions'
_FRONT_ARM_TOPIC  = '/ezrassor/front_arm_instructions'
_BACK_ARM_TOPIC   = '/ezrassor/back_arm_instructions'
_FRONT_DRUM_TOPIC = '/ezrassor/front_drum_instructions'
_BACK_DRUM_TOPIC  = '/ezrassor/back_drum_instructions'
_ROUTINE_TOPIC    = '/ezrassor/routine_instructions'

# ── Valid action value maps ───────────────────────────────────────────────────
_ARM_ACTIONS = {'RAISE': 1.0, 'STOP': 0.0, 'LOWER': -1.0}
_DRUM_ACTIONS = {'DIG': 1.0, 'STOP': 0.0, 'DUMP': -1.0}
_ROUTINE_ACTIONS = {
    'AUTO_DRIVE':    1,
    'AUTO_DIG':      2,
    'AUTO_DUMP':     4,
    'AUTO_DOCK':     8,
    'FULL_AUTONOMY': 16,
    'STOP':          32,
}
_VALID_KEYS = {
    'wheel_action', 'front_arm_action', 'back_arm_action',
    'front_drum_action', 'back_drum_action', 'routine_action',
}


def _validate(data):
    """Validate request data. Raises ValueError on bad input."""
    for key, value in data.items():
        if key not in _VALID_KEYS:
            raise ValueError(f'unknown key: {key}')
        if key == 'wheel_action':
            if not isinstance(value, dict):
                raise ValueError(f'{key} must be a dict with linear_x and angular_z')
            if 'linear_x' not in value or 'angular_z' not in value:
                raise ValueError(f'{key} must contain linear_x and angular_z')
            try:
                float(value['linear_x'])
                float(value['angular_z'])
            except (TypeError, ValueError):
                raise ValueError(f'{key}: linear_x and angular_z must be numeric')
        elif key in ('front_arm_action', 'back_arm_action'):
            if value not in _ARM_ACTIONS:
                raise ValueError(
                    f'{key} must be one of {sorted(_ARM_ACTIONS)}')
        elif key in ('front_drum_action', 'back_drum_action'):
            if value not in _DRUM_ACTIONS:
                raise ValueError(
                    f'{key} must be one of {sorted(_DRUM_ACTIONS)}')
        elif key == 'routine_action':
            if value not in _ROUTINE_ACTIONS:
                raise ValueError(
                    f'routine_action must be one of {sorted(_ROUTINE_ACTIONS)}')


def main(args=None):
    """Entry point for the controller_server ROS2 node."""
    rclpy.init(args=args)
    node = rclpy.create_node('controller_server')

    # ── Publishers ────────────────────────────────────────────────────────────
    wheel_pub      = node.create_publisher(Twist,   _WHEEL_TOPIC,      10)
    front_arm_pub  = node.create_publisher(Float64, _FRONT_ARM_TOPIC,  10)
    back_arm_pub   = node.create_publisher(Float64, _BACK_ARM_TOPIC,   10)
    front_drum_pub = node.create_publisher(Float64, _FRONT_DRUM_TOPIC, 10)
    back_drum_pub  = node.create_publisher(Float64, _BACK_DRUM_TOPIC,  10)
    routine_pub    = node.create_publisher(Int8,    _ROUTINE_TOPIC,    10)

    # ── Flask app ─────────────────────────────────────────────────────────────
    app = Flask(__name__)

    @app.route('/', methods=['GET', 'OPTIONS'])
    def health():
        return jsonify({'status': 200})

    @app.route('/', methods=['POST'])
    def handle_command():
        data = request.get_json(silent=True)
        if data is None:
            data = {}

        try:
            _validate(data)
        except ValueError as exc:
            node.get_logger().warning(f'Bad request: {exc}')
            return jsonify({'status': 400, 'error': str(exc)})

        if 'wheel_action' in data:
            msg = Twist()
            msg.linear.x  = float(data['wheel_action']['linear_x'])
            msg.angular.z = float(data['wheel_action']['angular_z'])
            wheel_pub.publish(msg)

        if 'front_arm_action' in data:
            msg = Float64()
            msg.data = _ARM_ACTIONS[data['front_arm_action']]
            front_arm_pub.publish(msg)

        if 'back_arm_action' in data:
            msg = Float64()
            msg.data = _ARM_ACTIONS[data['back_arm_action']]
            back_arm_pub.publish(msg)

        if 'front_drum_action' in data:
            msg = Float64()
            msg.data = _DRUM_ACTIONS[data['front_drum_action']]
            front_drum_pub.publish(msg)

        if 'back_drum_action' in data:
            msg = Float64()
            msg.data = _DRUM_ACTIONS[data['back_drum_action']]
            back_drum_pub.publish(msg)

        if 'routine_action' in data:
            msg = Int8()
            msg.data = _ROUTINE_ACTIONS[data['routine_action']]
            routine_pub.publish(msg)

        return jsonify({'status': 200})

    # ── Start Flask in a daemon background thread ─────────────────────────────
    flask_thread = threading.Thread(
        target=lambda: app.run(
            host='0.0.0.0',
            port=5000,
            use_reloader=False,
            threaded=True,
        ),
        daemon=True,
    )
    flask_thread.start()

    node.get_logger().info('controller_server listening on http://0.0.0.0:5000/')

    # ── Spin ROS node in the main thread ──────────────────────────────────────
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
