import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg    = get_package_share_directory('lunabotics_detection')
    config = os.path.join(pkg, 'config', 'crater_detection.yaml')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    sim = LaunchConfiguration('use_sim_time')

    detector_node = Node(
        package='lunabotics_detection',
        executable='crater_detector',
        name='crater_detector_node',
        output='screen',
        parameters=[config, {'use_sim_time': sim}],
    )

    cloud_pub_node = Node(
        package='lunabotics_detection',
        executable='crater_cloud_pub',
        name='crater_cloud_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': sim}],
    )

    return LaunchDescription([use_sim_time, detector_node, cloud_pub_node])
