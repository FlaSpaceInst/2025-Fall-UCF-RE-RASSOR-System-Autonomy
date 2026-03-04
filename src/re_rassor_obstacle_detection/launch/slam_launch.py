from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('re_rassor_obstacle_detection')
    rviz_config = os.path.join(pkg_share, 'rviz', 'costmap.rviz')

    depth_node = Node(
        package='re_rassor_obstacle_detection',
        executable='depth_costmap_node',
        name='depth_costmap_node',
        output='screen'
    )

    scan_costmap_node = Node(
        package='re_rassor_obstacle_detection',
        executable='scan_to_costmap_node',
        name='scan_to_costmap_node',
        output='screen'
    )

    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','base_link','camera_link']
    )

    return LaunchDescription([
        depth_node,
        scan_costmap_node,
        static_tf,
        rviz
    ])