#!/usr/bin/env python3
"""
hw_command_relay.py — re_rassor_motor_controller
──────────────────────────────────────────────────────────────────────────────
Thin relay node used exclusively during simulation / CI testing.

On real hardware serial_motor_controller.cpp directly writes bytes to the
Arduino RAMPS boards.  In Gazebo there is no RAMPS board, so the pytest
e2e tests cannot observe hardware-level commands by reading a serial port.

This node bridges that gap: it subscribes to every topic that
SerialMotorController subscribes to and re-publishes them unchanged on
/hw_commands/* so test assertions have a clean, ROS-native endpoint.

Topic mapping (rover_namespace default = "ezrassor"):
  /ezrassor/wheel_instructions      → /hw_commands/wheel_instructions
  /ezrassor/shoulder_instructions   → /hw_commands/shoulder_instructions
  /ezrassor/front_arm_instructions  → /hw_commands/front_arm_instructions
  /ezrassor/back_arm_instructions   → /hw_commands/back_arm_instructions
  /ezrassor/front_drum_instructions → /hw_commands/front_drum_instructions
  /ezrassor/back_drum_instructions  → /hw_commands/back_drum_instructions
  /ezrassor/routine_actions         → /hw_commands/routine_actions
  /cmd_vel                          → /hw_commands/cmd_vel

Parameters:
  rover_namespace  (string, default "ezrassor")
  use_sim_time     (bool,   default true)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int8

_RELIABLE = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class HwCommandRelay(Node):

    def __init__(self):
        super().__init__("hw_command_relay")

        self.declare_parameter("rover_namespace", "ezrassor")
        ns = self.get_parameter("rover_namespace").get_parameter_value().string_value
        src = f"/{ns}"
        dst = "/hw_commands"

        self.get_logger().info(
            f"HwCommandRelay: mirroring {src}/* → {dst}/*"
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._pub = {
            "wheel":        self.create_publisher(Twist,  f"{dst}/wheel_instructions",      _RELIABLE),
            "shoulder":     self.create_publisher(Twist,  f"{dst}/shoulder_instructions",   _RELIABLE),
            "front_arm":    self.create_publisher(Float64, f"{dst}/front_arm_instructions", _RELIABLE),
            "back_arm":     self.create_publisher(Float64, f"{dst}/back_arm_instructions",  _RELIABLE),
            "front_drum":   self.create_publisher(Float64, f"{dst}/front_drum_instructions",_RELIABLE),
            "back_drum":    self.create_publisher(Float64, f"{dst}/back_drum_instructions", _RELIABLE),
            "routine":      self.create_publisher(Int8,   f"{dst}/routine_actions",         _RELIABLE),
            "cmd_vel":      self.create_publisher(Twist,  f"{dst}/cmd_vel",                 _RELIABLE),
        }

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(Twist,  f"{src}/wheel_instructions",      lambda m: self._pub["wheel"].publish(m),      _RELIABLE)
        self.create_subscription(Twist,  f"{src}/shoulder_instructions",   lambda m: self._pub["shoulder"].publish(m),   _RELIABLE)
        self.create_subscription(Float64, f"{src}/front_arm_instructions", lambda m: self._pub["front_arm"].publish(m),  _RELIABLE)
        self.create_subscription(Float64, f"{src}/back_arm_instructions",  lambda m: self._pub["back_arm"].publish(m),   _RELIABLE)
        self.create_subscription(Float64, f"{src}/front_drum_instructions",lambda m: self._pub["front_drum"].publish(m), _RELIABLE)
        self.create_subscription(Float64, f"{src}/back_drum_instructions", lambda m: self._pub["back_drum"].publish(m),  _RELIABLE)
        self.create_subscription(Int8,   f"{src}/routine_actions",         lambda m: self._pub["routine"].publish(m),    _RELIABLE)
        self.create_subscription(Twist,  "/cmd_vel",                       lambda m: self._pub["cmd_vel"].publish(m),    _RELIABLE)


def main(args=None):
    rclpy.init(args=args)
    node = HwCommandRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()