"""
Depth image → LaserScan for frontal obstacles (/scan_depth).
Use alongside LiDAR /scan for Nav2 multi-source costmaps.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/depth/image_rect_raw',
            description='Registered depth image (sensor_msgs/Image).',
        ),
        DeclareLaunchArgument(
            'depth_info_topic',
            default_value='/camera/camera/depth/camera_info',
            description='Matching CameraInfo for the depth image.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan_depth',
            description='Output LaserScan topic.',
        ),
        DeclareLaunchArgument(
            'range_min',
            default_value='0.35',
            description='Minimum range (m); above RealSense blind zone.',
        ),
        DeclareLaunchArgument(
            'range_max',
            default_value='8.0',
            description='Maximum range (m).',
        ),
        DeclareLaunchArgument(
            'scan_height',
            default_value='8',
            description='Number of image rows centered vertically (narrower = more ground-focused).',
        ),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[{
                'scan_height': LaunchConfiguration('scan_height'),
                'scan_time': 0.033,
                'range_min': LaunchConfiguration('range_min'),
                'range_max': LaunchConfiguration('range_max'),
            }],
            remappings=[
                ('depth', LaunchConfiguration('depth_topic')),
                ('depth_camera_info', LaunchConfiguration('depth_info_topic')),
                ('scan', LaunchConfiguration('scan_topic')),
            ],
        ),
    ])
