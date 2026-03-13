# full stack launch
# brings up robot_state_publisher -> camera -> odometry -> static map->odom tf
# -> nav2 -> motor controller -> mission control

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction #,TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    bringup_pkg = FindPackageShare("re_rassor_bringup")
    nav2_bringup_pkg = FindPackageShare("nav2_bringup")

    # toggles
    launch_rsp = LaunchConfiguration("launch_rsp")
    #launch_camera = LaunchConfiguration("launch_camera")
    launch_odometry = LaunchConfiguration("launch_odometry")
    launch_nav2 = LaunchConfiguration("launch_nav2")
    launch_motor_controller = LaunchConfiguration("launch_motor_controller")
    launch_mission_control = LaunchConfiguration("launch_mission_control")

    # default map + params
    default_map = PathJoinSubstitution([bringup_pkg, "maps", "my_map.yaml"])
    default_params = PathJoinSubstitution([bringup_pkg, "config", "nav2_params.yaml"])

    # 1) robot_state_publisher include
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_pkg, "launch", "robot_state_publisher.launch.py"])
        ),
        condition=IfCondition(launch_rsp),
    )

    # 2) camera node
    #replace package/executable with real camera driver, im not sure if we need this
    #camera_node = Node(
    #    package="astra_camera",
    #    executable="astra_camera_node",
    #    name="astra_camera",
    #    output="screen",
    #    parameters=[{
    #        "use_sim_time": use_sim_time,
    #    }],
    #    condition=IfCondition(launch_camera),
    #)

    # 3) odometry include
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_pkg, "launch", "odometry.launch.py"])
        ),
        condition=IfCondition(launch_odometry),
    )

    # 4) static map -> odom transform publisher
    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    # 5) Nav2 bringup with cmd_vel remap
    nav2_launch = GroupAction([
        SetRemap(src="/cmd_vel", dst="/ezrassor/wheel_instructions"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav2_bringup_pkg, "launch", "navigation_launch.py"])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "map": map_yaml,
                "params_file": params_file,
            }.items(),
            condition=IfCondition(launch_nav2),
        ),
    ])

    # 6) motor controller
    # replace package/executable with real motor controller package if need
    motor_controller_node = Node(
        package="re_rassor_motor_controller",
        executable="motor_controller",
        name="motor_controller",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
        condition=IfCondition(launch_motor_controller),
    )

    # 7) mission control
    mission_control_node = Node(
        package="nav2_first_steps",
        executable="mission_control",
        name="mission_control",
        output="screen",
        parameters=[{
            "goal_x": LaunchConfiguration("goal_x"),
            "goal_y": LaunchConfiguration("goal_y"),
            "goal_theta": LaunchConfiguration("goal_theta"),
        }],
        condition=IfCondition(launch_mission_control),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Path to map YAML file",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to Nav2 params file",
        ),

        DeclareLaunchArgument(
            "launch_rsp",
            default_value="true",
            description="Launch robot_state_publisher",
        ),
        #DeclareLaunchArgument(
        #   "launch_camera",
        #    default_value="true",
        #    description="Launch camera driver",
        #),
        DeclareLaunchArgument(
            "launch_odometry",
            default_value="true",
            description="Launch odometry stack",
        ),
        DeclareLaunchArgument(
            "launch_nav2",
            default_value="true",
            description="Launch Nav2",
        ),
        DeclareLaunchArgument(
            "launch_motor_controller",
            default_value="true",
            description="Launch motor controller",
        ),
        DeclareLaunchArgument(
            "launch_mission_control",
            default_value="false",
            description="Launch mission control node",
        ),

        DeclareLaunchArgument("goal_x", default_value="0.0"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        DeclareLaunchArgument("goal_theta", default_value="0.0"),

        rsp_launch,
        #camera_node,
        odometry_launch,
        static_map_to_odom,
        nav2_launch,
        motor_controller_node,
        mission_control_node,
    ])