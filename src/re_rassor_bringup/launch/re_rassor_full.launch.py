import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, GroupAction, IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
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

    # ── 0b. Static TF: camera_link → camera_depth_optical_frame ────────────
    static_tf_depth_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_depth_optical_tf",
        arguments=["0", "0", "0", "-1.5708", "0", "-1.5708",
                   "camera_link", "camera_depth_optical_frame"],
        output="screen",
    )

    # ── 0d. Fake map — publish a blank free-space OccupancyGrid on /map ────────
    # Gives Nav2 a valid latched map immediately so costmaps initialise before
    # RTAB-Map builds a real one.
    _fake_map_script = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..', '..', '..', '..', 'fake_map.py',
    )
    fake_map = ExecuteProcess(
        cmd=['python3', _fake_map_script],
        output='screen',
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

    # ── 2. Astra Pro camera — depth only, 320×240 ───────────────────────────
    # Color disabled: Le Potato USB 2.0 cannot sustain both streams simultaneously.
    # publish_tf=false: static TFs above replace the driver's dynamic ones.
    # 320×240 depth: 4× fewer points than 640×480 — critical for ICP performance
    # on the Le Potato ARM CPU.  ICP convergence rate improves from ~0.2 Hz to
    # 5+ Hz.  Spatial resolution is still sufficient for obstacle detection at
    # the rover's operating range (< 4 m).
    astra_camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("astra_camera"),
                "launch",
                "astra_pro.launch.xml",
            ])
        ]),
        launch_arguments={
            "publish_tf":    "false",
            "enable_color":  "false",
            "depth_width":   "320",
            "depth_height":  "240",
        }.items(),
    )

    # ── 3. RTAB-Map — depth-only (ICP odometry + SLAM) ──────────────────────
    # Direct nodes instead of rtabmap.launch.py wrapper, which always forces
    # RGB subscription regardless of subscribe_rgb argument.
    #
    # icp_odometry: PointCloud2 → /rtabmap/odom  (no RGB needed)
    # rtabmap:      PointCloud2 + /rtabmap/odom → /rtabmap/map
    # Relay nodes (3b, 3c) bridge /rtabmap/odom → /odom and /rtabmap/map → /map.

    icp_odom = TimerAction(
        period=3.0,
        actions=[Node(
            package="rtabmap_odom",
            executable="icp_odometry",
            name="icp_odometry",
            namespace="rtabmap",
            output="screen",
            remappings=[
                ("scan_cloud", "/camera/depth/points"),
            ],
            parameters=[{
                "frame_id":                      "base_link",
                "odom_frame_id":                 "odom",
                "publish_tf":                    False,
                "approx_sync":                   True,
                "queue_size":                    5,
                "Reg/Force3DoF":                 "true",
                # ── Performance tuning for Le Potato ARM ──────────────────────
                # Voxel-filter the input cloud before ICP.  At 0.05 m voxels a
                # 320×240 cloud (~76k pts) shrinks to ~3-8k pts — ICP runs
                # 10-20× faster with negligible accuracy loss at rover speeds.
                "Icp/VoxelSize":                 "0.05",
                # Prune correspondences beyond 30 cm — faster matching, also
                # prevents ICP from latching onto distant background noise.
                "Icp/MaxCorrespondenceDistance": "0.3",
                # 10 iterations is sufficient for the slow rover (<0.3 m/s).
                "Icp/Iterations":                "10",
            }],
        )],
    )

    rtabmap_slam = TimerAction(
        period=3.5,
        actions=[Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            namespace="rtabmap",
            output="screen",
            remappings=[
                ("scan_cloud", "/camera/depth/points"),
                ("odom",       "/rtabmap/odom"),
            ],
            parameters=[{
                "subscribe_scan_cloud":  True,
                "subscribe_rgb":         False,
                "frame_id":              "base_link",
                "publish_tf":            False,
                "database_path":         "/tmp/rtabmap.db",
                "delete_db_on_start":    True,
                "Reg/Force3DoF":         "true",
                "Grid/3D":               "false",
                "Grid/CellSize":         "0.05",
                "Grid/RangeMax":         "4.0",
                "Grid/MinGroundHeight":  "-0.1",
                "Grid/MaxGroundHeight":  "0.15",
            }],
        )],
    )

    # ── 3b. Relay: /rtabmap/odom → /odom ─────────────────────────────────────
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

    # ── 3d. Depth → PointCloud2 (depth_image_proc) ───────────────────────────
    # Converts /camera/depth/image_raw + /camera/depth/camera_info
    # into /camera/depth/points (PointCloud2) for Nav2 costmaps and RViz2.
    # Runs independently of rtabmap so Nav2 gets obstacle data even if SLAM fails.
    depth_to_pointcloud = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        name="depth_to_pointcloud",
        output="screen",
        remappings=[
            ("image_rect",    "/camera/depth/image_raw"),
            ("camera_info",   "/camera/depth/camera_info"),
            ("points",        "/camera/depth/points"),
        ],
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
                "visual_weight":     0.0,
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
        fake_map,                   # /map ← blank free-space grid (latched)
        static_tf_base_to_camera,   # base_link  → camera_link
        static_tf_depth_optical,    # camera_link → camera_depth_optical_frame
        motor,
        astra_camera,
        depth_to_pointcloud,        # /camera/depth/image_raw → /camera/depth/points
        icp_odom,                   # /camera/depth/points → /rtabmap/odom
        rtabmap_slam,               # /camera/depth/points + odom → /rtabmap/map
        odom_relay,                 # /rtabmap/odom → /odom
        map_relay,                  # /rtabmap/map  → /map
        mission_control,
        nav2,
        rviz_node,
    ])