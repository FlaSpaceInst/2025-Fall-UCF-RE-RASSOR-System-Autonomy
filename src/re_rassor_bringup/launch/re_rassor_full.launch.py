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

    # ── 0b. Static TF: camera_link → camera_depth_optical_frame ─────────────
    # Collapsed the intermediate camera_depth_frame hop — depth_image_proc only
    # needs camera_link → camera_depth_optical_frame to stamp point clouds
    # correctly.  Fewer TF hops = less lookup latency on the slow ARM CPU.
    # Standard optical-frame rotation: z-forward, x-right, y-down.
    # args order: x y z yaw pitch roll parent child
    static_tf_depth_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_depth_optical_tf",
        arguments=["0", "0", "0", "-1.5708", "0", "-1.5708",
                   "camera_link", "camera_depth_optical_frame"],
        output="screen",
    )

    # ── 0c. Static TF: map → odom (identity) ─────────────────────────────────
    # In mapless/wheel-odom-only mode there is no SLAM node publishing the
    # map→odom transform.  Nav2 requires the full chain:
    #   map → odom → base_link
    # The identity static TF here satisfies that requirement — it tells Nav2
    # "the map frame and the odom frame are coincident", which is correct for a
    # robot that starts at the origin and uses wheel odometry as its sole pose
    # source.  mission_control owns odom→base_link.
    static_tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    # ── 0d. Fake map ─────────────────────────────────────────────────────────
    # Publishes a blank free-space OccupancyGrid on /map so Nav2's global
    # costmap static_layer initialises immediately.  In mapless mode the global
    # costmap's static_layer is essentially a no-op — obstacles are handled
    # entirely by the local costmap's voxel_layer / obstacle_layer fed by
    # /camera/depth/points.  The fake map prevents Nav2 from stalling at boot
    # waiting for a /map message that would never arrive.
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

    # ── 2. Astra Pro camera — raw depth stream only, 320×240 ─────────────────
    # Color disabled: Le Potato USB 2.0 cannot sustain both streams.
    # 320×240 depth: 4× fewer points than 640×480 — critical for point cloud
    # processing performance on the Le Potato ARM CPU.
    # enable_point_cloud=false: we generate the cloud ourselves via
    # depth_image_proc (node 2b) — the driver's built-in generator stalls at
    # 0 Hz when color is disabled because it still needs color camera_info.
    # publish_tf=false: static TFs (0a, 0b) replace the driver's 10 Hz TFs,
    # eliminating TF timeout errors against the 30 Hz depth stream.
    astra_camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("astra_camera"),
                "launch",
                "astra_pro.launch.xml",
            ])
        ]),
        launch_arguments={
            "publish_tf":         "false",
            "enable_color":       "false",
            "enable_point_cloud": "false",
            "depth_width":        "320",
            "depth_height":       "240",
        }.items(),
    )

    # ── 2b. depth_image_proc: depth image → PointCloud2 ──────────────────────
    # Converts /camera/depth/image_raw + /camera/depth/camera_info into
    # /camera/depth/points.  Works without the color stream.
    # At 320×240 this produces ~76k points before voxel filtering — well within
    # the Le Potato's budget for Nav2 voxel_layer processing.
    # No timer needed: depth_image_proc is a passive subscriber and will simply
    # wait until the Astra driver begins publishing depth frames.
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
        parameters=[{
            "queue_size": 5,
        }],
    )

    # ── 3. Mission control ───────────────────────────────────────────────────
    # Sole odometry source: wheel encoders only.
    # visual_weight=0.0 disables any visual odometry fusion — there is no ICP
    # or SLAM node in this stack, so visual_odom_topic will receive no messages.
    # publish_tf=True: mission_control is the single owner of the
    # odom→base_link TF.  No ICP node fights for this broadcast.
    # Delayed 3 s to ensure the motor controller is publishing /odometry/wheel
    # before mission_control subscribes.
    mission_control = TimerAction(
        period=3.0,
        actions=[Node(
            package="re_rassor_mission_control",
            executable="mission_control",
            name="mission_control",
            output="screen",
            parameters=[{
                "wheel_odom_topic":  "/odometry/wheel",
                "visual_odom_topic": "/odom",       # unused — no visual odom
                "fused_odom_topic":  "/odometry/fused",
                "visual_weight":     0.0,            # wheel odom only
                "publish_tf":        True,           # owns odom→base_link TF
            }],
        )],
    )

    # ── 4. Nav2 stack ─────────────────────────────────────────────────────────
    # Delayed 8 s — mission_control must be publishing /odometry/fused and the
    # odom→base_link TF before Nav2's controller_server and bt_navigator init.
    #
    # nav2_params.yaml should be configured for mapless/reactive operation:
    #
    #   global costmap:
    #     - static_layer:   reads /map (fake free-space grid — obstacles never
    #                       written here, only the local costmap matters)
    #     - obstacle_layer: reads /camera/depth/points for global inflation
    #     rolling_window: false   (global map stays fixed at origin)
    #
    #   local costmap:
    #     - voxel_layer:    reads /camera/depth/points, clears dynamically
    #     rolling_window: true    (follows the robot)
    #     width:  3.0             (3 m × 3 m window is plenty for a slow rover)
    #     height: 3.0
    #
    #   controller_server:
    #     - use DWB or RPP controller; set min_vel_x low for slow rover
    #
    #   bt_navigator:
    #     - odom_topic: /odometry/fused
    #
    # With this config Nav2 plans globally on the static free-space grid and
    # avoids obstacles reactively using the live local costmap point cloud.
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

    nav2 = TimerAction(period=8.0, actions=[GroupAction(nav2_nodes)])

    # ── 5. RViz2 (optional) ──────────────────────────────────────────────────
    rviz_node = TimerAction(
        period=9.0,
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
        # ── Static TFs (immediate) ───────────────────────────────────────────
        static_tf_base_to_camera,   # base_link → camera_link
        static_tf_depth_optical,    # camera_link → camera_depth_optical_frame
        static_tf_map_odom,         # map → odom (identity, no SLAM node present)
        # ── Map placeholder (immediate) ──────────────────────────────────────
        fake_map,                   # /map ← blank free-space OccupancyGrid
        # ── Hardware (immediate) ─────────────────────────────────────────────
        motor,                      # wheel encoders → /odometry/wheel
        astra_camera,               # raw 320×240 depth stream
        depth_to_pointcloud,        # depth image → /camera/depth/points
        # ── Odometry + TF (t+3s) ─────────────────────────────────────────────
        mission_control,            # wheel odom → /odometry/fused + odom→base_link TF
        # ── Navigation (t+8s) ────────────────────────────────────────────────
        nav2,                       # full Nav2 stack, reactive local costmap
        # ── Visualisation (t+9s, optional) ───────────────────────────────────
        rviz_node,
    ])