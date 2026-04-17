from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    camera_container = ComposableNodeContainer(
        name='camera_container',
        namespace='camera',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[

            ComposableNode(
                package='openni2_camera',
                plugin='openni2_wrapper::OpenNI2Driver',
                name='camera',
                parameters=[{
                    'depth_registration': True,
                    'use_device_time': False,
                }],
                remappings=[
                    ('rgb/image_raw', 'image/rgb'),
                    ('rgb/camera_info', 'camera_info'),
                    ('depth_registered/image_raw', 'depth/image'),
                    ('depth_registered/points', 'depth/points'),
                ],
            ),

            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb',
                remappings=[
                    ('rgb/image_rect_color', 'image/rgb'),
                    ('depth_registered/image_rect', 'depth/image'),
                    ('depth_registered/points', 'depth/points'),
                ],
            ),
        ],
    )

    return LaunchDescription([
        camera_container
    ])