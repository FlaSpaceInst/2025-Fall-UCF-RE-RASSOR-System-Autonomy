"""
demo_sim.launch.py — RE-RASSOR Full Autonomy Stack (No Hardware)
─────────────────────────────────────────────────────────────────
Brings up the complete autonomy stack on a dev machine without the
physical rover.  Hardware nodes are replaced by demo_sim.py:

  Real stack                  →  Demo stack
  ──────────────────────────────────────────
  serial_motor_controller        demo_sim.py  (fake motor + empty point cloud)
  astra_camera                   (skipped)
  depth_to_pointcloud            (skipped — demo_sim publishes empty cloud)
  depth_to_laserscan             (skipped)
  slam_toolbox                   (skipped — static map→odom TF used instead)

Everything else runs exactly as on the rover:
  fake_map, static TFs, mission_control, full Nav2 stack, controller_server

Usage
─────
  # In a sourced workspace:
  ros2 launch re_rassor_bringup demo_sim.launch.py

  # Then connect the Electron controller app to this machine's IP on port 5000.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, GroupAction, ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # ── Launch args ───────────────────────────────────────────────────────────
    declare_args = [
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 for debugging',
        ),
    ]

    # ── Param file paths ──────────────────────────────────────────────────────
    nav2_params = PathJoinSubstitution([
        FindPackageShare('re_rassor_bringup'), 'config', 'nav2_params.yaml',
    ])

    # ── Resolve demo_sim.py path (same walk-up logic as fake_map in re_rassor_full) ──
    _demo_sim_script = None
    _candidate = os.path.dirname(os.path.realpath(__file__))
    for _ in range(10):
        _candidate = os.path.dirname(_candidate)
        _probe = os.path.join(_candidate, 'demo_sim.py')
        if os.path.isfile(_probe):
            _demo_sim_script = _probe
            break
    if _demo_sim_script is None:
        raise RuntimeError(
            'demo_sim.py not found in any ancestor directory of this launch file. '
            'Ensure demo_sim.py is at the workspace root alongside fake_map.py.'
        )

    _fake_map_script = None
    _candidate = os.path.dirname(os.path.realpath(__file__))
    for _ in range(10):
        _candidate = os.path.dirname(_candidate)
        _probe = os.path.join(_candidate, 'fake_map.py')
        if os.path.isfile(_probe):
            _fake_map_script = _probe
            break
    if _fake_map_script is None:
        raise RuntimeError(
            'fake_map.py not found in any ancestor directory of this launch file.'
        )

    # ── 0. Blank occupancy grid → /map (transient-local, latched) ────────────
    # Gives Nav2 a valid map immediately so costmaps initialise.
    fake_map = ExecuteProcess(
        cmd=['python3', _fake_map_script],
        output='screen',
    )

    # ── 1. Static TF: base_link → camera_link ────────────────────────────────
    # Needed so Nav2 costmap can transform the point cloud into the map frame.
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.15', '0', '0.10', '0', '0.087', '0',
                   'base_link', 'camera_link'],
        output='screen',
    )

    # ── 2. Static TF: camera_link → camera_depth_optical_frame ───────────────
    static_tf_depth_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_optical_tf',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                   'camera_link', 'camera_depth_optical_frame'],
        output='screen',
    )

    # ── 3. Static TF: map → odom (identity) ──────────────────────────────────
    # Replaces slam_toolbox.  Without real laser data slam_toolbox cannot
    # initialise; a fixed identity transform anchors the map frame to the
    # odometry frame so Nav2 path planning works correctly.
    # The rover starts at the map origin and moves purely by dead-reckoning.
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    # ── 4. Demo simulator node ────────────────────────────────────────────────
    # Replaces serial_motor_controller + astra_camera.
    # Publishes: /odometry/wheel  (dead-reckoned from wheel commands)
    #            /camera/depth/points  (empty cloud — Nav2 costmaps stay clear)
    demo_sim = ExecuteProcess(
        cmd=['python3', _demo_sim_script],
        output='screen',
    )

    # ── 5. Mission control (2 s delay) ────────────────────────────────────────
    # Fuses /odometry/wheel → /odometry/fused, broadcasts odom→base_link TF.
    # Delayed slightly so demo_sim is publishing odom before fusion starts.
    mission_control = TimerAction(
        period=2.0,
        actions=[Node(
            package='re_rassor_mission_control',
            executable='mission_control',
            name='mission_control',
            output='screen',
            parameters=[{
                'wheel_odom_topic':  '/odometry/wheel',
                'visual_odom_topic': '/odom',
                'fused_odom_topic':  '/odometry/fused',
                'visual_weight':     0.0,
            }],
        )],
    )

    # ── 6. Nav2 stack (8 s delay) ─────────────────────────────────────────────
    # Needs: /map (from fake_map), map→odom TF, odom→base_link TF,
    #        /odometry/fused (from mission_control), /camera/depth/points.
    nav2_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[nav2_params],
        ),
    ]

    nav2 = TimerAction(period=8.0, actions=[GroupAction(nav2_nodes)])

    # ── 7. Controller server (10 s delay) ─────────────────────────────────────
    # HTTP + WebSocket bridge between the Electron app and ROS.
    # sim_mode=true prevents it from auto-launching re_rassor_full.launch.py
    # (which would try to open serial ports and the camera).
    controller_server = TimerAction(
        period=10.0,
        actions=[Node(
            package='re_rassor_controller_server',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[{'sim_mode': True}],
        )],
    )

    return LaunchDescription([
        *declare_args,
        fake_map,
        static_tf_base_to_camera,
        static_tf_depth_optical,
        static_tf_map_to_odom,
        demo_sim,
        mission_control,
        nav2,
        controller_server,
    ])
