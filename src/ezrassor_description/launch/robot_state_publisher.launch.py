"""
Launch robot_state_publisher with the EZRASSOR URDF.

Processes the ezrassor.urdf.xacro and publishes the full TF tree:
  base_footprint -> base_link -> camera_link -> camera_depth_frame
                                             -> camera_color_frame
                                             -> laser_frame (virtual)

Primary sensor: Orbbec Astra Pro 3D depth camera (no physical LiDAR).
The virtual laser_frame is used by depthimage_to_laserscan for /scan output.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('ezrassor_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'ezrassor.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        robot_state_publisher_node,
    ])
