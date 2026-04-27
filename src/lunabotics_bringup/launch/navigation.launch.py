import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('lunabotics_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params,
            'map_subscribe_transient_local': 'true',
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([use_sim_time, nav2])
