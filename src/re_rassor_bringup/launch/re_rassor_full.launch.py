from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, GroupAction, IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():

    # ── Launch args ──────────────────────────────────────────────────────────
    rviz = LaunchConfiguration("rviz")

    declare_args = [
        DeclareLaunchArgument(
            "wheel_port",
            default_value="/dev/arduino_wheel",
            description="Serial port for wheel Arduino",
        ),
        DeclareLaunchArgument(
            "drum_port",
            default_value="/dev/arduino_drum",
            description="Serial port for drum/shoulder Arduino",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Serial baud rate",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Launch RViz2",
        ),
    ]

    # ── Nav2 params ──────────────────────────────────────────────────────────
    nav2_params = PathJoinSubstitution([
        FindPackageShare("re_rassor_bringup"),
        "config",
        "nav2_params.yaml",
    ])

    # ── 0a. Static TF: base_link → camera_link ───────────────────────────────
    # 0.15 m forward, 0.10 m up, 5° downward pitch (0.087 rad).
    static_tf_base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        arguments=["0.15", "0", "0.10", "0", "0.087", "0",
                   "base_link", "camera_link"],
        output="screen",
    )

    # ── 0b. Static TF: camera_link → camera_color_optical_frame ─────────────
    # Published statically — prevents "extrapolation into the future" TF
    # errors when RTAB-Map looks up the transform at exact image timestamps.
    static_tf_color_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_color_optical_tf",
        arguments=["0", "0", "0", "-1.5708", "0", "-1.5708",
                   "camera_link", "camera_color_optical_frame"],
        output="screen",
    )

    # ── 0c. Static TF: camera_link → camera_depth_optical_frame ─────────────
    static_tf_depth_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_depth_optical_tf",
        arguments=["0", "0", "0", "-1.5708", "0", "-1.5708",
                   "camera_link", "camera_depth_optical_frame"],
        output="screen",
    )

    # ── 1. Motor controller (serial) ─────────────────────────────────────────
    motor = Node(
        package="re_rassor_motor_controller",
        executable="serial_motor_controller",
        name="serial_motor_controller",
        output="screen",
        parameters=[{
            "wheel_port": LaunchConfiguration("wheel_port"),
            "drum_port":  LaunchConfiguration("drum_port"),
            "baud_rate":  LaunchConfiguration("baud_rate"),
        }],
    )

    # ── 2. Astra Pro RGBD camera ─────────────────────────────────────────────
    # Uses official astra_pro launch — configures UVC colour + OpenNI2 depth.
    # publish_tf=false: static TFs above replace the driver's dynamic ones.
    astra_camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("astra_camera"),
                "launch",
                "astra_pro.launch.xml",
            ])
        ]),
        launch_arguments={"publish_tf": "false"}.items(),
    )

    # ── 3. RTAB-Map (SLAM + odometry) ────────────────────────────────────────
    # rtabmap_launch runs rgbd_odometry + rtabmap inside the /rtabmap namespace.
    # Internally they communicate correctly via /rtabmap/odom_info.
    # Externally their topics are /rtabmap/odom and /rtabmap/map.
    # Relay nodes below (3b, 3c) bridge these to /odom and /map for Nav2.
    rtabmap = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("rtabmap_launch"),
                    "launch",
                    "rtabmap.launch.py",
                ])
            ]),
            launch_arguments={
                "rgb_topic":            "/camera/color/image_raw",
                "depth_topic":          "/camera/depth/image_raw",
                "camera_info_topic":    "/camera/color/camera_info",
                "approx_sync":          "true",
                "frame_id":             "base_link",
                "odom_frame_id":        "odom",
                "rviz":                 "false",
                "rtabmap_viz":          "false",
                # RTAB-Map internal params must go inside the args string.
                # Passing them as launch arguments crashes Humble (unknown args).
                "args": (
                    "--delete_db_on_start"
                    " --Reg/Force3DoF true"
                    " --Grid/FromDepth true"
                    " --Grid/3D false"
                    " --Grid/CellSize 0.05"
                    " --Grid/RangeMax 4.0"
                    " --Grid/MinGroundHeight -0.1"
                    " --Grid/MaxGroundHeight 0.15"
                ),
                "use_sim_time":    "false",
                "publish_tf_odom": "true",
                "publish_tf_map":  "true",
            }.items(),
        )],
    )

    # ── 3b. Relay: /rtabmap/odom → /odom ─────────────────────────────────────
    # Humble topic_tools relay takes positional CLI arguments.
    odom_relay = TimerAction(
        period=4.0,
        actions=[Node(
            package="topic_tools",
            executable="relay",
            name="odom_relay",
            output="screen",
            arguments=["/rtabmap/odom", "/odom"],
        )],
    )

    # ── 3c. Relay: /rtabmap/map → /map ───────────────────────────────────────
    # Published with default QoS — Nav2 static_layer uses transient_local
    # so set map_subscribe_transient_local: false in nav2_params.yaml.
    map_relay = TimerAction(
        period=4.0,
        actions=[Node(
            package="topic_tools",
            executable="relay",
            name="map_relay",
            output="screen",
            arguments=["/rtabmap/map", "/map"],
        )],
    )

    # ── 4. Mission control ───────────────────────────────────────────────────
    # Fuses /odometry/wheel + /odom → /odometry/fused.
    # Broadcasts odom→base_link TF.
    mission_control = TimerAction(
        period=5.0,
        actions=[Node(
            package="re_rassor_mission_control",
            executable="mission_control",
            name="mission_control",
            output="screen",
            parameters=[{
                "wheel_odom_topic":  "/odometry/wheel",
                "visual_odom_topic": "/odom",
                "fused_odom_topic":  "/odometry/fused",
                "visual_weight":     0.3,
            }],
        )],
    )

    # ── 5. Nav2 stack ────────────────────────────────────────────────────────
    # Delayed 10 s — RTAB-Map and relays must be publishing /map and /odom
    # before Nav2 costmaps and bt_navigator initialize.
    #
    # global costmap: static_layer reads /map (from relay) + obstacle_layer
    #                 reads /camera/depth/points for live obstacles.
    # local costmap:  voxel_layer reads /camera/depth/points.
    nav2_nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[nav2_params],
        ),
    ]

    nav2 = TimerAction(period=10.0, actions=[GroupAction(nav2_nodes)])

    # ── 6. RViz2 (optional) ──────────────────────────────────────────────────
    rviz_node = TimerAction(
        period=11.0,
        actions=[Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(rviz),
        )],
    )

    return LaunchDescription([
        *declare_args,
        static_tf_base_to_camera,   # base_link  → camera_link
        static_tf_color_optical,    # camera_link → camera_color_optical_frame
        static_tf_depth_optical,    # camera_link → camera_depth_optical_frame
        motor,
        astra_camera,
        rtabmap,
        odom_relay,                 # /rtabmap/odom → /odom
        map_relay,                  # /rtabmap/map  → /map
        mission_control,
        nav2,
        rviz_node,
    ])