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

    # ── Params paths ─────────────────────────────────────────────────────────
    nav2_params = PathJoinSubstitution([
        FindPackageShare("re_rassor_bringup"),
        "config",
        "nav2_params.yaml",
    ])

    slam_params = PathJoinSubstitution([
        FindPackageShare("re_rassor_bringup"),
        "config",
        "slam_toolbox_params.yaml",
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
    # slam_toolbox builds a real one.
    # Walk up from this file until fake_map.py is found — works from both the
    # source tree and the colcon install/ tree (which are at different depths).
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
            "fake_map.py not found in any ancestor directory of this launch file. "
            "Ensure fake_map.py is at the workspace root.")
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
    # 320×240 depth: 4× fewer points than 640×480 — sufficient for obstacle
    # detection at the rover's operating range (< 4 m).
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

    # ── 3. Depth → PointCloud2 (depth_image_proc) ───────────────────────────
    # Converts /camera/depth/image_raw + /camera/depth/camera_info
    # into /camera/depth/points (PointCloud2).
    # Required by Nav2 costmaps:
    #   - local_costmap  voxel_layer    reads /camera/depth/points
    #   - global_costmap obstacle_layer reads /camera/depth/points
    depth_to_pointcloud = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        name="depth_to_pointcloud",
        output="screen",
        remappings=[
            ("image_rect",  "/camera/depth/image_raw"),
            ("camera_info", "/camera/depth/camera_info"),
            ("points",      "/camera/depth/points"),
        ],
    )

    # ── 4. Depth → LaserScan (depthimage_to_laserscan) ──────────────────────
    # Converts a horizontal slice of the depth image into a 2D laser scan.
    # This is the input to slam_toolbox.
    #
    # scan_height=1: single-row scan — fast and sufficient for 2D SLAM.
    # range_min=0.45: Astra Pro minimum reliable depth distance.
    # output_frame=camera_link: scan is in the camera optical frame; the
    #   static TF chain (base_link→camera_link→camera_depth_optical_frame)
    #   lets slam_toolbox transform it to base_link automatically.
    depth_to_laserscan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depth_to_laserscan",
        output="screen",
        remappings=[
            ("depth",             "/camera/depth/image_raw"),
            ("depth_camera_info", "/camera/depth/camera_info"),
            ("scan",              "/scan"),
        ],
        parameters=[{
            "scan_height":   1,
            "range_min":     0.45,
            "range_max":     4.0,
            "output_frame":  "camera_link",
        }],
    )

    # ── 5. Mission control ───────────────────────────────────────────────────
    # Fuses /odometry/wheel + /odom → /odometry/fused.
    # Broadcasts odom→base_link TF.
    # visual_weight=0.0: pure wheel odometry (no visual correction).
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

    # ── 6. slam_toolbox — async mapping ────────────────────────────────────
    # Receives /scan (from depthimage_to_laserscan) and the odom→base_link TF
    # (from mission_control / wheel odometry) to build a 2D occupancy map.
    #
    # slam_toolbox owns the map→odom TF (localization).
    # mission_control owns odom→base_link TF (wheel odometry).
    #
    # Delayed 6 s — needs the TF chain (static TFs + odom→base_link from
    # mission_control at t=5 s) and laser scans to be available first.
    slam = TimerAction(
        period=6.0,
        actions=[Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params],
        )],
    )

    # ── 7. Nav2 stack ────────────────────────────────────────────────────────
    # Delayed 12 s — slam_toolbox must be publishing /map and map→odom TF
    # before Nav2 costmaps and bt_navigator initialize.
    #
    # global costmap: static_layer reads /map (from slam_toolbox) +
    #                 obstacle_layer reads /camera/depth/points (live obstacles)
    # local costmap:  voxel_layer reads /camera/depth/points
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

    nav2 = TimerAction(period=12.0, actions=[GroupAction(nav2_nodes)])

    # ── 8. RViz2 (optional) ──────────────────────────────────────────────────
    rviz_node = TimerAction(
        period=13.0,
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
        depth_to_pointcloud,        # /camera/depth/image_raw → /camera/depth/points (Nav2 costmaps)
        depth_to_laserscan,         # /camera/depth/image_raw → /scan (slam_toolbox input)
        mission_control,            # wheel odom → /odometry/fused + odom→base_link TF
        slam,                       # /scan + odom TF → /map + map→odom TF
        nav2,
        rviz_node,
    ])
