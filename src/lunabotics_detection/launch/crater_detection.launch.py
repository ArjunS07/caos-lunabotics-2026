import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lunabotics_detection'),
        'config',
        'crater_detection.yaml',
    )

    return LaunchDescription([
        Node(
            package='lunabotics_detection',
            executable='crater_detector',
            name='crater_detector',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='lunabotics_detection',
            executable='crater_visualizer',
            name='crater_visualizer',
            output='screen',
        ),
    ])
