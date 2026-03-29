# Copyright (c) 2022 Florida Space Institute (MIT License)
# Ported and updated for ROS2 Jazzy / RE-RASSOR 2025 by UCF RE-RASSOR Team.
#
# Original: rassor_serial_fwd/rassor_driver_subscriber.py
#
# Subscribes to wheel/shoulder/drum Twist topics and sends single binary
# command bytes to the Arduino RAMPS 1.4 boards over pyserial.
#
# Wire protocol (matches 2023 arduino_wheels.ino / arduino_shoulder.ino):
#   Wheel Arduino (/dev/arduino_wheel):
#     0x00 STOP  0x01 FWD  0x02 REV  0x03 LEFT  0x04 RIGHT
#
#   Drum/Shoulder Arduino (/dev/arduino_drum):
#     0x00 STOP
#     0x05 RAISE_FRONT  0x06 LOWER_FRONT
#     0x07 RAISE_BACK   0x08 LOWER_BACK
#     0x01 DUMP         0x02 DIG

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class SerialDriver(Node):
    """ROS2 node that forwards Twist commands to Arduino via pyserial."""

    def __init__(self):
        super().__init__('serial_driver')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('wheel_port',      '/dev/arduino_wheel')
        self.declare_parameter('drum_port',       '/dev/arduino_drum')
        self.declare_parameter('baud_rate',       115200)
        self.declare_parameter('rover_namespace', 'ezrassor')

        wheel_port = self.get_parameter('wheel_port').value
        drum_port  = self.get_parameter('drum_port').value
        baud_rate  = self.get_parameter('baud_rate').value
        ns         = '/' + self.get_parameter('rover_namespace').value

        if not SERIAL_AVAILABLE:
            self.get_logger().error(
                'pyserial not installed — install with: pip3 install pyserial')

        # ── Open wheel serial ─────────────────────────────────────────────────
        self.wheel_serial = None
        if SERIAL_AVAILABLE:
            try:
                self.wheel_serial = serial.Serial(
                    port=wheel_port,
                    baudrate=baud_rate,
                    write_timeout=0.1,
                )
                self.get_logger().info(f'Wheel serial opened: {wheel_port} @ {baud_rate}')
            except Exception as e:
                self.get_logger().error(f'Failed to open wheel serial {wheel_port}: {e}')
                self.wheel_serial = None

        # ── Open drum/shoulder serial (optional) ──────────────────────────────
        self.drum_serial = None
        if SERIAL_AVAILABLE and os.path.exists(drum_port):
            try:
                self.drum_serial = serial.Serial(
                    port=drum_port,
                    baudrate=baud_rate,
                    write_timeout=0.1,
                )
                self.get_logger().info(f'Drum serial opened: {drum_port} @ {baud_rate}')
            except Exception as e:
                self.get_logger().error(f'Failed to open drum serial {drum_port}: {e}')

        # ── Status publisher ──────────────────────────────────────────────────
        self.status_pub = self.create_publisher(String, ns + '/serial_status', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Twist, ns + '/wheel_instructions',
            self.wheel_callback, 100)

        self.create_subscription(
            Twist, ns + '/shoulder_instructions',
            self.shoulder_callback, 100)

        self.create_subscription(
            Twist, ns + '/front_drum_instructions',
            self.front_drum_callback, 100)

        self.create_subscription(
            Twist, ns + '/back_drum_instructions',
            self.back_drum_callback, 100)

        self.get_logger().info('RE-RASSOR serial driver started')

    # ── Serial write ──────────────────────────────────────────────────────────

    def _send(self, usb, byte_cmd: int):
        if usb is None or not usb.is_open:
            return
        try:
            usb.write(bytearray([byte_cmd]))
            self._publish_status('OK')
        except Exception as e:
            if SERIAL_AVAILABLE and isinstance(e, serial.SerialTimeoutException):
                self.get_logger().error(f'Write timeout: {e}')
                self._publish_status('Timeout')
            else:
                self.get_logger().error(f'Serial error: {e}')
                self._publish_status('Error')
            try:
                usb.flushInput()
                usb.flushOutput()
            except Exception:
                pass

    def _publish_status(self, msg: str):
        m = String()
        m.data = msg
        self.status_pub.publish(m)

    # ── Command encoding ──────────────────────────────────────────────────────

    def _wheel_byte(self, lin_x: float, ang_z: float) -> int:
        if lin_x == 0.0 and ang_z == 0.0:
            return 0x00  # STOP
        elif lin_x > 0:
            return 0x01  # FWD
        elif lin_x < 0:
            return 0x02  # REV
        elif ang_z > 0:
            return 0x03  # LEFT
        else:
            return 0x04  # RIGHT

    def _shoulder_byte(self, lin_y: float, ang_y: float) -> int:
        if lin_y == 0.0 and ang_y == 0.0:
            return 0x00
        elif lin_y > 0:
            return 0x05  # RAISE_FRONT
        elif lin_y < 0:
            return 0x06  # LOWER_FRONT
        elif ang_y > 0:
            return 0x07  # RAISE_BACK
        else:
            return 0x08  # LOWER_BACK

    def _drum_byte(self, lin_x: float) -> int:
        if lin_x == 0.0:
            return 0x00  # STOP
        elif lin_x > 0:
            return 0x01  # DUMP
        else:
            return 0x02  # DIG

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def wheel_callback(self, msg: Twist):
        cmd = self._wheel_byte(msg.linear.x, msg.angular.z)
        self.get_logger().debug(f'wheel_callback: lin={msg.linear.x:.2f} ang={msg.angular.z:.2f} → 0x{cmd:02X}')
        self._send(self.wheel_serial, cmd)

    def shoulder_callback(self, msg: Twist):
        cmd = self._shoulder_byte(msg.linear.y, msg.angular.y)
        self.get_logger().debug(f'shoulder_callback: lin_y={msg.linear.y:.2f} ang_y={msg.angular.y:.2f} → 0x{cmd:02X}')
        self._send(self.drum_serial, cmd)

    def front_drum_callback(self, msg: Twist):
        cmd = self._drum_byte(msg.linear.x)
        self.get_logger().debug(f'front_drum_callback: lin_x={msg.linear.x:.2f} → 0x{cmd:02X}')
        self._send(self.drum_serial, cmd)

    def back_drum_callback(self, msg: Twist):
        cmd = self._drum_byte(msg.linear.x)
        self.get_logger().debug(f'back_drum_callback: lin_x={msg.linear.x:.2f} → 0x{cmd:02X}')
        self._send(self.drum_serial, cmd)

    def destroy_node(self):
        if self.wheel_serial and self.wheel_serial.is_open:
            self.wheel_serial.write(bytearray([0x00]))  # STOP before closing
            self.wheel_serial.close()
        if self.drum_serial and self.drum_serial.is_open:
            self.drum_serial.write(bytearray([0x00]))
            self.drum_serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
