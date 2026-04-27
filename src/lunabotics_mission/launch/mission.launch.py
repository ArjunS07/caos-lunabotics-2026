import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg    = get_package_share_directory('lunabotics_mission')
    config = os.path.join(pkg, 'config', 'mission.yaml')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    mission_node = Node(
        package='lunabotics_mission',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([use_sim_time, mission_node])
