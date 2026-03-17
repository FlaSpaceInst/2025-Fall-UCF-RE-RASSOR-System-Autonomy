from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    bringup_dir = get_package_share_directory('re_rassor_bringup')
    rf2o_params = os.path.join(bringup_dir, 'config', 'rf2o_params.yaml')

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o',
        output='screen',
        parameters=[rf2o_params]
    )

    return LaunchDescription([
        rf2o_node
    ])