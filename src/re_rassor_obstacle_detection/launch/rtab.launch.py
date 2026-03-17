"""
rtabmap_astra.launch.py
────────────────────────
Launches:
  0. static_transform_publisher   base_link → camera_link  (REQUIRED by rtabmap)
  1. rtabmap_odom/rgbd_odometry   visual odometry → /odom + TF odom→base_link
  2. rtabmap_slam/rtabmap         SLAM map        → /map  + TF map→odom

FIX: "base_link does not exist" warning
  rtabmap's rgbd_odometry looks up the TF from the sensor frame (camera_link)
  to the robot base frame (base_link) on startup.  Without a static transform
  connecting these two frames the lookup fails with the warning you saw.
  The static_transform_publisher below creates that link immediately on launch.

Camera mount convention (Astra Pro 3D on RE-RASSOR):
  The camera is assumed to be mounted 15 cm forward and 10 cm above base_link,
  pointing forward with a slight downward tilt (~5 degrees / 0.087 rad).
  Tune x/y/z/roll/pitch/yaw to match your actual robot geometry.

Usage:
  ros2 launch re_rassor_obstacle_detection rtabmap_astra.launch.py

Optional args:
  localization:=true   — localization-only mode (map already built)
  rviz:=true           — open RViz2

Camera mount args (all in metres / radians, default = forward-facing):
  cam_x cam_y cam_z cam_roll cam_pitch cam_yaw
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("localization", default_value="false",
                              description="true = localization-only (reuse saved map)"),
        DeclareLaunchArgument("rviz",         default_value="false",
                              description="true = launch RViz2"),

        # Camera extrinsics — adjust to your robot's actual camera mount
        DeclareLaunchArgument("cam_x",     default_value="0.15",
                              description="Camera X offset from base_link (m), forward"),
        DeclareLaunchArgument("cam_y",     default_value="0.0",
                              description="Camera Y offset from base_link (m), left"),
        DeclareLaunchArgument("cam_z",     default_value="0.10",
                              description="Camera Z offset from base_link (m), up"),
        DeclareLaunchArgument("cam_roll",  default_value="0.0",
                              description="Camera roll  (rad)"),
        DeclareLaunchArgument("cam_pitch", default_value="0.087",
                              description="Camera pitch (rad), ~5 deg downward tilt"),
        DeclareLaunchArgument("cam_yaw",   default_value="0.0",
                              description="Camera yaw   (rad)"),
    ]

    localization = LaunchConfiguration("localization")
    rviz         = LaunchConfiguration("rviz")

    # ── Node 0: static transform  base_link → camera_link ────────────────
    # This is the transform rtabmap's rgbd_odometry was failing to find.
    # It tells rtabmap where the camera sits relative to the robot body.
    # Arguments order: x y z yaw pitch roll parent_frame child_frame
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_camera_link",
        output="screen",
        arguments=[
            LaunchConfiguration("cam_x"),
            LaunchConfiguration("cam_y"),
            LaunchConfiguration("cam_z"),
            LaunchConfiguration("cam_yaw"),    # tf2 stp order: yaw pitch roll
            LaunchConfiguration("cam_pitch"),
            LaunchConfiguration("cam_roll"),
            "base_link",      # parent frame
            "camera_link",    # child frame
        ],
    )

    # ── Shared rtabmap parameters ─────────────────────────────────────────
    # frame_id      = the robot base frame
    # guess_frame_id = the sensor frame; rtabmap uses the TF to transform
    #                  sensor data into the base frame before processing.
    #                  Must match the frame_id used in the published images.
    rtabmap_params = {
        "frame_id":       "base_link",    # robot body frame
        "odom_frame_id":  "odom",
        "approx_sync":    True,
        "queue_size":     10,
    }

    # ── Node 1: rgbd_odometry ─────────────────────────────────────────────
    # Publishes /odom (nav_msgs/Odometry) and TF odom → base_link
    rgbd_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        parameters=[rtabmap_params],
        remappings=[
            ("rgb/image",       "/camera/image/rgb"),
            ("depth/image",     "/camera/depth/image"),   # 32FC1, metres
            ("rgb/camera_info", "/camera/camera_info"),
            ("odom",            "/odom"),
        ],
    )

    # ── Node 2: rtabmap SLAM ──────────────────────────────────────────────
    # Publishes /map and TF map → odom (loop-closure corrected)
    rtabmap_slam = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[{
            **rtabmap_params,
            "subscribe_depth":       True,
            "subscribe_scan":        True,
            "use_action_for_goal":   False,
            "database_path":         "~/.ros/rtabmap_re_rassor.db",
            # Incremental mapping vs localization-only
            "Mem/IncrementalMemory": PythonExpression([
                "'false' if '", localization, "' == 'true' else 'true'"
            ]),
            "Mem/InitWMWithAllNodes": localization,
            # Tuning for Astra Pro (short range, structured environments)
            "Reg/Strategy":          "1",     # 1 = ICP
            "Vis/MinInliers":        "6",     # lowered from 10 — Astra has limited range
            "RGBD/AngularUpdate":    "0.01",
            "RGBD/LinearUpdate":     "0.01",
        }],
        arguments=["--delete_db_on_start"],   # remove this line to resume a saved map
        remappings=[
            ("rgb/image",       "/camera/image/rgb"),
            ("depth/image",     "/camera/depth/image"),
            ("rgb/camera_info", "/camera/camera_info"),
            ("scan",            "/camera/scan"),
            ("odom",            "/odom"),
            ("map",             "/map"),
        ],
    )

    # ── Node 3: RViz (optional) ───────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        *args,
        static_tf,       # MUST come first so TF is available before rtabmap starts
        rgbd_odometry,
        rtabmap_slam,
        rviz_node,
    ])