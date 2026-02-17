#full stack launch 
#temp launch file, 

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    bringup_pkg = FindPackageShare("re_rassor_bringup")
    nav2_bringup_pkg = FindPackageShare("nav2_bringup")

    # default map + params 
    default_map = PathJoinSubstitution([bringup_pkg, "maps", "my_map.yaml"])
    default_params = PathJoinSubstitution([bringup_pkg, "config", "nav2_params.yaml"])

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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="use simulation (gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="path to map YAML file",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="path to the ros2 params file for all launched nodes",
            ),
            nav2_launch,
        ]
    )
