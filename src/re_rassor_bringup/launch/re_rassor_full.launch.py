"""
re_rassor_full.launch.py
─────────────────────────
Starts the complete RE-RASSOR stack in the correct order:

  0. static_transform_publisher   base_link → camera_link
  1. motor_controller             /odometry/wheel
  2. depth_costmap_node           /camera/image/rgb
                                  /camera/depth/image   (32FC1)
                                  /camera/camera_info
                                  /camera/scan
                                  /camera/depth/points
  3. scan_to_costmap_node         /scan_costmap
  4. rgbd_odometry (rtabmap)      /odom  (no TF — mission_control owns it)
  5. rtabmap SLAM                 /map   + map→odom TF
  6. Nav2 bringup                 /navigate_to_pose action server
  7. mission_control              /odometry/fused  + odom→base_link TF
                                  /re_rassor/calibrate service
  8. RViz (optional)

Usage:
  ros2 launch re_rassor_bringup re_rassor_full.launch.py

  # Override server IP if different:
  ros2 launch re_rassor_bringup re_rassor_full.launch.py server_ip:=10.127.234.196

  # Open RViz:
  ros2 launch re_rassor_bringup re_rassor_full.launch.py rviz:=true

  # After launch — calibrate (hold robot still, facing forward):
  ros2 service call /re_rassor/calibrate std_srvs/srv/Trigger

  # Navigate (rover frame: +X=right, +Y=forward, theta=CCW rad):
  ros2 run re_rassor_mission_control navigate_cli -- 0.0 3.0 0.0
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "server_ip", default_value="10.127.234.196",
            description="IP of the Windows depth_server.py host"),
        DeclareLaunchArgument(
            "server_port", default_value="5000",
            description="HTTP rover controller port"),
        DeclareLaunchArgument(
            "ws_port", default_value="8765",
            description="WebSocket depth stream port"),
        DeclareLaunchArgument(
            "localization", default_value="false",
            description="rtabmap: true=localization only, false=mapping"),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Launch RViz2"),
        DeclareLaunchArgument(
            "visual_weight", default_value="0.0",  # 0.0=wheel only; set >0 when camera moving
            description="Visual odom weight in fusion [0.0=wheel only, 1.0=visual only]"),
        # Camera mount extrinsics (metres / radians)
        DeclareLaunchArgument("cam_x",     default_value="0.15"),
        DeclareLaunchArgument("cam_y",     default_value="0.0"),
        DeclareLaunchArgument("cam_z",     default_value="0.10"),
        DeclareLaunchArgument("cam_roll",  default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.087"),
        DeclareLaunchArgument("cam_yaw",   default_value="0.0"),
    ]

    server_ip     = LaunchConfiguration("server_ip")
    server_port   = LaunchConfiguration("server_port")
    ws_port_str   = LaunchConfiguration("ws_port")     # string from CLI
    localization  = LaunchConfiguration("localization")
    rviz          = LaunchConfiguration("rviz")
    visual_weight = LaunchConfiguration("visual_weight")

    # ── 0. Static TF: base_link → camera_link ────────────────────────────
    # Must be up before rtabmap tries any TF lookups.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_camera_link",
        output="screen",
        # stp argument order: x y z yaw pitch roll parent child
        arguments=[
            LaunchConfiguration("cam_x"),
            LaunchConfiguration("cam_y"),
            LaunchConfiguration("cam_z"),
            LaunchConfiguration("cam_yaw"),
            LaunchConfiguration("cam_pitch"),
            LaunchConfiguration("cam_roll"),
            "base_link",
            "camera_link",
        ],
    )

    # ── 1. Motor controller ───────────────────────────────────────────────
    # server_port=8000: depth_server.py Flask runs on 8000 (merged controller).
    # server_ip is a LaunchConfiguration string substitution — pass it in a
    # separate dict so ROS2 handles the type correctly at launch time.
    motor_controller = Node(
        package="re_rassor_motor_controller",
        executable="motor_controller",
        name="motor_controller",
        output="screen",
        parameters=[
            {"server_ip":       server_ip},   # substitution resolved at launch
            {
                "server_port":      8000,     # depth_server.py HTTP port
                "update_rate":      50.0,
                "skid_correction":  0.7,
                "command_timeout":  1.0,
            },
        ],
    )

    # ── 2. Depth costmap node ─────────────────────────────────────────────
    # ws_port must be passed as an integer parameter.
    # LaunchConfiguration returns a string, so we pass the default int here
    # and let the user override via --ros-args if needed.
    depth_costmap = Node(
        package="re_rassor_obstacle_detection",
        executable="depth_costmap_node",
        name="depth_costmap_node",
        output="screen",
        parameters=[{
            "ws_host": server_ip,
            "ws_port": 8765,       # int literal — declare_parameter expects int
            "fx":      525.0,
            "fy":      525.0,
            "cx":      320.0,
            "cy":      240.0,
            "width":   640,
            "height":  480,
        }],
    )

    # ── 3. Scan → costmap ─────────────────────────────────────────────────
    scan_costmap = Node(
        package="re_rassor_obstacle_detection",
        executable="scan_to_costmap_node",
        name="scan_to_costmap_node",
        output="screen",
    )

    # ── 4. rtabmap rgbd_odometry ──────────────────────────────────────────
    # publish_tf=false: mission_control owns the odom→base_link TF.
    # Delayed 3 s to give depth_costmap_node time to connect to the server
    # and start publishing camera topics before rtabmap starts waiting.
    rtabmap_odom = TimerAction(
        period=3.0,
        actions=[Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            output="screen",
            parameters=[{
                "frame_id":                "base_link",
                "odom_frame_id":           "odom",
                "approx_sync":             True,
                "queue_size":              10,
                "publish_tf":              False,
                "Vis/MinInliers":          "6",
                "Vis/FeatureType":         "8",
                "Vis/CorNNDRRatio":        "0.8",
                "OdomF2M/BundleAdjustment":"0",
            }],
            remappings=[
                ("rgb/image",       "/camera/image/rgb"),
                ("depth/image",     "/camera/depth/image"),
                ("rgb/camera_info", "/camera/camera_info"),
                ("odom",            "/odom"),
            ],
        )],
    )

    # ── 5. rtabmap SLAM ───────────────────────────────────────────────────
    rtabmap_slam = TimerAction(
        period=3.0,
        actions=[Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[{
                "frame_id":           "base_link",
                "odom_frame_id":      "odom",
                "approx_sync":        True,
                "sync_queue_size":    10,
                "subscribe_depth":    True,
                "subscribe_scan":     True,
                "database_path":      "~/.ros/rtabmap_re_rassor.db",
                # rtabmap string params must be passed as strings, not bools.
                # Localization flag is passed via --Mem/IncrementalMemory CLI arg.
                "Reg/Strategy":       "1",
                "Vis/MinInliers":     "8",   # rtabmap enforces minimum of 8
                "RGBD/AngularUpdate": "0.01",
                "RGBD/LinearUpdate":  "0.01",
            }],
            arguments=["--delete_db_on_start"],
            remappings=[
                ("rgb/image",       "/camera/image/rgb"),
                ("depth/image",     "/camera/depth/image"),
                ("rgb/camera_info", "/camera/camera_info"),
                ("scan",            "/camera/scan"),
                ("odom",            "/odom"),
                ("map",             "/map"),
            ],
        )],
    )

    # ── 6. Nav2 bringup ───────────────────────────────────────────────────
    # Launch only the nodes listed in lifecycle_manager/node_names.
    # navigation_launch.py from nav2_bringup also starts route_server,
    # opennav_docking, and collision_monitor which fail on some Jazzy
    # installs and cause lifecycle_manager to abort the whole bringup.
    # We launch each node individually so we control the exact set.
    nav2_params_file = PathJoinSubstitution([
        FindPackageShare("re_rassor_bringup"),
        "config",
        "nav2_params.yaml",
    ])

    nav2_nodes = [
        Node(package="nav2_controller",     executable="controller_server",
             name="controller_server",      output="screen",
             parameters=[nav2_params_file]),
        Node(package="nav2_smoother",        executable="smoother_server",
             name="smoother_server",         output="screen",
             parameters=[nav2_params_file]),
        Node(package="nav2_planner",         executable="planner_server",
             name="planner_server",          output="screen",
             parameters=[nav2_params_file]),
        Node(package="nav2_behaviors",       executable="behavior_server",
             name="behavior_server",         output="screen",
             parameters=[nav2_params_file]),
        Node(package="nav2_bt_navigator",    executable="bt_navigator",
             name="bt_navigator",            output="screen",
             parameters=[nav2_params_file]),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower",
             name="waypoint_follower",         output="screen",
             parameters=[nav2_params_file]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother",          output="screen",
             parameters=[nav2_params_file]),
        # lifecycle_manager: node_names and autostart come from nav2_params.yaml
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[nav2_params_file]),
    ]

    nav2 = TimerAction(period=5.0, actions=[GroupAction(actions=nav2_nodes)])

    # ── 7. Mission control ────────────────────────────────────────────────
    # Delayed 6 s — after Nav2 has started its action server.
    mission_control = TimerAction(
        period=6.0,
        actions=[Node(
            package="re_rassor_mission_control",
            executable="mission_control",
            name="mission_control",
            output="screen",
            parameters=[{
                "wheel_odom_topic":  "/odometry/wheel",
                "visual_odom_topic": "/odom",
                "fused_odom_topic":  "/odometry/fused",
                "visual_weight":     0.0,   # override with visual_weight:=0.3 when SLAM running
            }],
        )],
    )

    # ── 8. RViz (optional) ────────────────────────────────────────────────
    rviz_node = TimerAction(
        period=7.0,
        actions=[Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(rviz),
        )],
    )

    return LaunchDescription([
        *args,
        static_tf,          # immediate — TF must exist before everything
        motor_controller,   # immediate — start publishing wheel odom
        depth_costmap,      # immediate — start connecting to WebSocket
        scan_costmap,       # immediate — waits for /camera/scan internally
        rtabmap_odom,       # +3 s — camera topics should be flowing by now
        rtabmap_slam,       # +3 s — same
        nav2,               # +5 s — rtabmap odom→base_link TF should exist
        mission_control,    # +6 s — Nav2 action server should be ready
        rviz_node,          # +7 s — everything up
    ])