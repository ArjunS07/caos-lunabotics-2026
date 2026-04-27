import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('lunabotics_icp_localization')
    config = os.path.join(pkg, 'config', 'icp_localization.yaml')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    icp_node = Node(
        package='lunabotics_icp_localization',
        executable='icp_localization_node',
        name='icp_localization_node',
        output='screen',
        parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([use_sim_time, icp_node])
