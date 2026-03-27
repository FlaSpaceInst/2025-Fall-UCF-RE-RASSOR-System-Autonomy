"""
sensors.launch.py
─────────────────
Starts the Orbbec Astra Pro camera (via openni2_camera driver) and
depth-to-scan conversion for the RE-RASSOR autonomy stack.

  1. openni2_wrapper::OpenNI2Driver  (ComposableNode, namespace /camera)
     Publishes raw RGB, depth-registered images and a coloured point cloud.
     Launched as a composable node so that camera_with_cloud.launch.py's
     extra static_transform_publishers do NOT enter the TF tree.

  2. depth_image_proc::PointCloudXyzrgbNode  (ComposableNode, same container)
     Generates a coloured point cloud from registered depth + colour images.

  3. depthimage_to_laserscan_node  (regular Node)
     Converts the depth image to a virtual laser scan on /camera/scan.

Topic remapping summary
───────────────────────
  openni2 default topic              → stack expected topic
  /camera/rgb/image_raw              → /camera/image/rgb
  /camera/depth_registered/image_raw → /camera/depth/image
  /camera/depth/camera_info          → /camera/camera_info
  /camera/depth_registered/points    → /camera/depth/points
  depthimage_to_laserscan publishes  → /camera/scan

NOTE: No static_transform_publisher is added here.  The URDF already
      defines the full TF tree via robot_state_publisher:
        base_footprint → base_link → camera_link
          → camera_depth_frame / camera_color_frame / laser_frame
      A duplicate transform would cause TF conflicts.
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # ── 1 & 2. Camera driver + point cloud (composable, shared container) ──
    # Launching the OpenNI2Driver composable node directly avoids the extra
    # static_transform_publisher nodes that camera_with_cloud.launch.py adds.
    camera_container = ComposableNodeContainer(
        name='camera_container',
        namespace='camera',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[

            # OpenNI2 driver for Orbbec Astra Pro
            ComposableNode(
                package='openni2_camera',
                plugin='openni2_wrapper::OpenNI2Driver',
                name='camera',
                namespace='camera',
                parameters=[{
                    'depth_registration': True,
                    'use_device_time':    False,
                }],
                remappings=[
                    ('/camera/rgb/image_raw',              '/camera/image/rgb'),
                    ('/camera/rgb/camera_info',            '/camera/camera_info'),
                    ('/camera/depth_registered/image_raw', '/camera/depth/image'),
                    ('/camera/depth_registered/points',    '/camera/depth/points'),
                ],
            ),

            # Coloured point cloud from registered depth + colour
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb',
                namespace='camera',
                remappings=[
                    ('rgb/image_rect_color',        '/camera/image/rgb'),
                    ('depth_registered/image_rect',  '/camera/depth/image'),
                    ('depth_registered/camera_info', '/camera/depth/camera_info'),
                    ('depth_registered/points',      '/camera/depth/points'),
                ],
            ),
        ],
    )

    # ── 3. Depth image → virtual laser scan ─────────────────────────────────
    depthimage_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'scan_height':     1,
            'scan_time':       0.033,
            'range_min':       0.1,
            'range_max':       10.0,
            'output_frame_id': 'laser_frame',
        }],
        remappings=[
            ('depth',             '/camera/depth/image'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan',              '/camera/scan'),
        ],
    )

    return LaunchDescription([
        camera_container,
        depthimage_to_laserscan,
    ])
