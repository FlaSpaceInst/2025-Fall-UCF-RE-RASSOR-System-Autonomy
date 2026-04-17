"""
e2e_obstacle_detection.launch.py
─────────────────────────────────────────────────────────────────────────────
End-to-end test launch for the depth-camera → rtabmap → nav2 pipeline.

Camera data comes from synthetic_camera_publisher (re_rassor_test package)
instead of Gazebo.  Gazebo's camera sensor plugin requires a rendering engine
(OGRE2) that silently fails in headless environments; the synthetic publisher
starts instantly and is guaranteed to produce /camera/* topics.

TF chain:
  map ──(rtabmap)──> odom ──(static)──> base_link ──(static)──> camera_link
                                                                   ├──> camera_color_optical_frame
                                                                   └──> camera_depth_optical_frame

Start sequence:
  t= 0 s  Static TF: odom → base_link  (identity)
  t= 0 s  Static TFs: base_link→camera_link, →color_optical, →depth_optical
  t= 0 s  synthetic_camera_publisher — immediately publishes all /camera/* topics
           and /odometry/wheel at 10 Hz; aborts launch if it fails to start
  t= 3 s  rgbd_odometry  — publish_tf:=false (static TF owns odom→base_link)
  t= 5 s  rtabmap SLAM   — builds /map, publishes map→odom TF
  t= 8 s  hw_command_relay — echoes /ezrassor/* → /hw_commands/*
  t=10 s  Nav2 stack
  t=11 s  RViz2 (optional, rviz:=true)

Usage (plain launch):
    ros2 launch re_rassor_test e2e_obstacle_detection.launch.py

Usage (with pytest — launch_testing):
    colcon test --packages-select re_rassor_test

Usage (with RViz):
    ros2 launch re_rassor_test e2e_obstacle_detection.launch.py rviz:=true
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest


@pytest.mark.launch_test
def generate_test_description():
    """Entry-point used by launch_testing / colcon test."""
    ld, context = _build_launch_description()
    ld.add_action(
        TimerAction(
            period=15.0,
            actions=[launch_testing.actions.ReadyToTest()],
        )
    )
    return ld, context


def generate_launch_description():
    """Entry-point used by plain `ros2 launch` (no testing)."""
    ld, _ = _build_launch_description()
    return ld


def _build_launch_description():
    nav2_params = PathJoinSubstitution([
        FindPackageShare("re_rassor_bringup"), "config", "nav2_params.yaml"
    ])

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="false",
        description="Launch RViz2 for visual debugging")

    # ── 1. Static TF: odom → base_link (identity) ────────────────────────────
    static_tf_odom_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_odom_base_link",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        output="screen")

    # ── 2. Static TFs: camera chain ──────────────────────────────────────────
    static_tf_base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        arguments=["0.15", "0", "0.10", "0", "0.087", "0",
                   "base_link", "camera_link"],
        output="screen")

    static_tf_color_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_color_optical_tf",
        arguments=["0", "0", "0", "-1.5708", "0", "-1.5708",
                   "camera_link", "camera_color_optical_frame"],
        output="screen")

    static_tf_depth_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_depth_optical_tf",
        arguments=["0", "0", "0", "-1.5708", "0", "-1.5708",
                   "camera_link", "camera_depth_optical_frame"],
        output="screen")

    # ── 3. Synthetic camera publisher ────────────────────────────────────────
    # Publishes /camera/color/image_raw, /camera/depth/image_raw,
    # /camera/color/camera_info, /camera/depth/camera_info,
    # /camera/depth/points, /odometry/wheel — all at 10 Hz from t=0.
    # on_exit=Shutdown() ensures the whole launch aborts if this node dies.
    synthetic_camera = Node(
        package="re_rassor_test",
        executable="synthetic_camera_publisher",
        name="synthetic_camera_publisher",
        output="screen",
    )

    # ── 4. rgbd_odometry ─────────────────────────────────────────────────────
    rgbd_odometry = TimerAction(
        period=3.0,
        actions=[Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            output="screen",
            parameters=[{
                "frame_id":                 "base_link",
                "odom_frame_id":            "odom",
                "approx_sync":              True,
                "approx_sync_max_interval": 0.5,
                "publish_tf":               False,
                "Odom/Strategy":            "0",
                "Vis/MinInliers":           "5",
                "Vis/FeatureType":          "6",
            }],
            remappings=[
                ("rgb/image",       "/camera/color/image_raw"),
                ("depth/image",     "/camera/depth/image_raw"),
                ("rgb/camera_info", "/camera/color/camera_info"),
                ("odom",            "/odom"),
            ],
        )])

    # ── 5. rtabmap SLAM ───────────────────────────────────────────────────────
    rtabmap_slam = TimerAction(
        period=5.0,
        actions=[Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            arguments=["--delete_db_on_start"],
            parameters=[{
                "frame_id":                 "base_link",
                "odom_frame_id":            "odom",
                "map_frame_id":             "map",
                "subscribe_depth":          True,
                "approx_sync":              True,
                "approx_sync_max_interval": 0.5,
                "publish_tf":               True,
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
        )])

    # ── 6. Hardware-command relay ─────────────────────────────────────────────
    hw_command_relay = TimerAction(
        period=8.0,
        actions=[Node(
            package="re_rassor_test",
            executable="hw_command_relay",
            name="hw_command_relay",
            output="screen",
            parameters=[{"rover_namespace": "ezrassor"}],
        )])

    # ── 7. Nav2 stack ─────────────────────────────────────────────────────────
    nav2_nodes = [
        Node(package="nav2_controller",        executable="controller_server",
             name="controller_server",         output="screen",
             parameters=[nav2_params]),
        Node(package="nav2_planner",           executable="planner_server",
             name="planner_server",            output="screen",
             parameters=[nav2_params]),
        Node(package="nav2_behaviors",         executable="behavior_server",
             name="behavior_server",           output="screen",
             parameters=[nav2_params]),
        Node(package="nav2_bt_navigator",      executable="bt_navigator",
             name="bt_navigator",              output="screen",
             parameters=[nav2_params]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother",         output="screen",
             parameters=[nav2_params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[nav2_params]),
    ]

    nav2 = TimerAction(period=10.0, actions=[GroupAction(nav2_nodes)])

    # ── 8. RViz2 (optional) ───────────────────────────────────────────────────
    rviz2 = TimerAction(
        period=11.0,
        actions=[Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")))])

    ld = LaunchDescription([
        rviz_arg,
        static_tf_odom_to_base,
        static_tf_base_to_camera,
        static_tf_color_optical,
        static_tf_depth_optical,
        synthetic_camera,
        rgbd_odometry,
        rtabmap_slam,
        hw_command_relay,
        nav2,
        rviz2,
    ])

    context = {
        "synthetic_camera": synthetic_camera,
        "hw_command_relay": hw_command_relay,
    }

    return ld, context
