"""Static transform: base_link → unilidar_lidar.

Measured mount: 0.20 m forward (+X), 0.60 m up (+Z), 38° nose-down pitch
(~-0.663 rad; REP-103, same sign as lidar→camera downward tilt).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'base_to_lidar_x', default_value='0.20',
            description='base_link → unilidar_lidar translation X forward (m)'),
        DeclareLaunchArgument(
            'base_to_lidar_y', default_value='0.0',
            description='base_link → unilidar_lidar translation Y (m)'),
        DeclareLaunchArgument(
            'base_to_lidar_z', default_value='0.60',
            description='base_link → unilidar_lidar translation Z up (m)'),
        DeclareLaunchArgument(
            'base_to_lidar_yaw', default_value='0.0',
            description='Yaw (rad), Euler ZYX after translation'),
        DeclareLaunchArgument(
            'base_to_lidar_pitch', default_value='0.0',
            description='Pitch (rad); 0° forward/down from horizontal'),
        DeclareLaunchArgument(
            'base_to_lidar_roll', default_value='0.0',
            description='Roll (rad)'),
    ]

    tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_unilidar_tf',
        arguments=[
            '--x', LaunchConfiguration('base_to_lidar_x'),
            '--y', LaunchConfiguration('base_to_lidar_y'),
            '--z', LaunchConfiguration('base_to_lidar_z'),
            '--yaw',   LaunchConfiguration('base_to_lidar_yaw'),
            '--pitch', LaunchConfiguration('base_to_lidar_pitch'),
            '--roll',  LaunchConfiguration('base_to_lidar_roll'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'unilidar_lidar',
        ],
    )

    return LaunchDescription(args + [tf_base_to_lidar])
