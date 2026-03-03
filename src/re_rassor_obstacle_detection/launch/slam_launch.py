from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    depth_node = Node(
        package='re_rassor_obstacle_detection',
        executable='depth_costmap_node',
        output='screen'
    )

    return LaunchDescription([
        slam,
        depth_node
    ])