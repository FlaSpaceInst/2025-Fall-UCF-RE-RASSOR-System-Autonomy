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

    # -- Launch args ----------------------------------------------------------
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

    # -- Nav2 params ----------------------------------------------------------
    nav2_params = PathJoinSubstitution([
        FindPackageShare("re_rassor_bringup"),
        "config",
        "nav2_params.yaml",
    ])

    # -- 0a. Static TF: base_link -> camera_link ------------------------------
    # 0.15 m forward, 0.10 m up, 5deg downward pitch (0.087 rad).
    static_tf_base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        arguments=["0.15", "0", "0.10", "0", "0.087", "0",
                   "base_link", "camera_link"],
        output="screen",
    )

    # -- 0b. Static TF: camera_link -> camera_color_optical_frame -------------
    # Published statically -- prevents "extrapolation into the future" TF
    # errors when RTAB-Map looks up the transform at exact image timestamps.
    static_tf_color_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_color_optical_tf",
        arguments=["0", "0", "0", "-1.5708", "0", "-1.5708",
                   "camera_link", "camera_color_optical_frame"],
        output="screen",
    )

    # -- 0c. Static TF: camera_link -> camera_depth_optical_frame -------------
    static_tf_depth_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_depth_optical_tf",
        arguments=["0", "0", "0", "-1.5708", "0", "-1.5708",
                   "camera_link", "camera_depth_optical_frame"],
        output="screen",
    )

    # -- 1. Motor controller (serial) -----------------------------------------
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

    # -- 2. Astra Pro RGBD camera ---------------------------------------------
    # Uses the official astra_pro launch file which correctly configures
    # UVC colour + OpenNI2 depth and publishes all required topics:
    #   /camera/color/image_raw
    #   /camera/color/camera_info
    #   /camera/depth/image_raw
    #   /camera/depth/camera_info
    #   /camera/depth/points      <- Nav2 costmaps
    # publish_tf overridden to false -- static TFs above replace it.
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

    # -- 3. RTAB-Map (SLAM + odometry) ----------------------------------------
    # Uses the official rtabmap_launch package -- this is the configuration
    # that is confirmed working from manual testing.
    #
    # rtabmap_launch internally starts:
    #   - rgbd_odometry  -> publishes /odom + odom->base_link TF
    #   - rtabmap        -> publishes /map (OccupancyGrid) + map->odom TF
    #
    # approx_sync=true handles the Astra Pro's independent UVC/OpenNI2 clocks.
    # frame_id=base_link ties the map to the robot body frame.
    #
    # Delayed 3 s to ensure camera topics are live before RTAB-Map subscribes.
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
                "rgb_topic":         "/camera/color/image_raw",
                "depth_topic":       "/camera/depth/image_raw",
                "camera_info_topic": "/camera/color/camera_info",
                "approx_sync":       "true",
                "frame_id":          "base_link",
                "odom_frame_id":     "odom",
                # Let rtabmap_launch manage its own rviz
                "rviz":              "false",
                # Use depth for grid, 2D mode for ground robot
                "args":              "--delete_db_on_start",
                "use_sim_time":      "false",
            }.items(),
        )],
    )

    # -- 4. Mission control ---------------------------------------------------
    # Fuses /odometry/wheel + /odom -> /odometry/fused.
    # Broadcasts odom->base_link TF.
    # visual_odom_topic=/odom -- picks up rtabmap_launch's odometry directly.
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
                "visual_weight":     0.0,   # wheel-only: no visual correction
            }],
        )],
    )

    # -- 5. Nav2 stack --------------------------------------------------------
    # Delayed 10 s -- RTAB-Map must be publishing /map and /odom before
    # Nav2 costmaps and bt_navigator initialize.
    #
    # map_server is absent -- rtabmap_launch owns /map and map->odom TF.
    # lifecycle_manager does not manage rtabmap (not a Nav2 lifecycle node).
    #
    # Both costmaps use rolling_window + depth point cloud only:
    #   local:  VoxelLayer  from /camera/depth/points  (6x6m)
    #   global: ObstacleLayer from /camera/depth/points (20x20m)
    #
    # odom_topic=/odom in bt_navigator and velocity_smoother -- uses
    # rtabmap_launch's odometry directly, no dependency on mission_control
    # fused odom for Nav2 core functions.
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

    # -- 6. RViz2 (optional) --------------------------------------------------
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
        static_tf_base_to_camera,   # base_link  -> camera_link
        static_tf_color_optical,    # camera_link -> camera_color_optical_frame
        static_tf_depth_optical,    # camera_link -> camera_depth_optical_frame
        motor,
        astra_camera,
        rtabmap,
        mission_control,
        nav2,
        rviz_node,
    ])