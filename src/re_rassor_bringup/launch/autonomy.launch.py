#full stack launch 
#new launch file with tf launch and mission control. will update as needed 

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    bringup_pkg = FindPackageShare("re_rassor_bringup")
    nav2_bringup_pkg = FindPackageShare("nav2_bringup")

    # toggles
    launch_rsp = LaunchConfiguration("launch_rsp")
    launch_initial_pose_pub = LaunchConfiguration("launch_initial_pose_pub")
    launch_nav2_client = LaunchConfiguration("launch_nav2_client")
    launch_mission_control = LaunchConfiguration("launch_mission_control")

    # default map + params 
    default_map = PathJoinSubstitution([bringup_pkg, "maps", "my_map.yaml"])
    default_params = PathJoinSubstitution([bringup_pkg, "config", "nav2_params.yaml"])

    # robot_state_publisher include 
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_pkg, "launch", "robot_state_publisher.launch.py"])
        ),
        condition=IfCondition(launch_rsp),
    )
    # nav2 include
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_pkg, "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_yaml,
            "params_file": params_file,
        }.items(),
    )
    # helper nodes, assumes nav2_first_steps package
    initial_pose_pub_node = Node(
        package="nav2_first_steps",
        executable="initial_pose_pub",
        name="initial_pose_publisher",
        output="screen",
        condition=IfCondition(launch_initial_pose_pub),
    )

    nav2_client_node = Node(
        package="nav2_first_steps",
        executable="nav2_client",
        name="nav2_client",
        output="screen",
        condition=IfCondition(launch_nav2_client),
    )

    mission_control_node = TimerAction(
        period=3.0,
        actions=[
            Node(
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
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Path to map YAML file",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the ROS2 params file for Nav2",
            ),

            DeclareLaunchArgument(
                "launch_rsp",
                default_value="true",
                description="Launch robot_state_publisher.launch.py",
            ),
            DeclareLaunchArgument(
                "launch_initial_pose_pub",
                default_value="false",
                description="Launch initial_pose_pub helper node",
            ),
            DeclareLaunchArgument(
                "launch_nav2_client",
                default_value="false",
                description="Launch nav2_client helper node",
            ),
            DeclareLaunchArgument(
                "launch_mission_control",
                default_value="false",
                description="Launch mission_control node",
            ),

            # mission control goal params
            DeclareLaunchArgument("goal_x", default_value="0.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("goal_theta", default_value="0.0"),

            # launch order
            rsp_launch,
            nav2_launch,
            initial_pose_pub_node,
            nav2_client_node,
            mission_control_node,
        ]
    )
