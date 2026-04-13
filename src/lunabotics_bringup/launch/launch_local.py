"""
Local machine launch file.
Runs only RViz. All sensor topics come from the Jetson over DDS.

Prerequisites on local machine:
  export ROS_DOMAIN_ID=42
  export ROS_LOCALHOST_ONLY=0

Copy the RViz config from the Jetson once (or scp it):
  scp <jetson_user>@<jetson_ip>:/path/to/unitree_lidar_ros2/rviz/view.rviz ~/view.rviz

Then either:
  a) Install the lunabotics_bringup package locally so ros2 pkg prefix finds it, OR
  b) Point RVIZ_CONFIG_PATH env var at the copied file:
       export RVIZ_CONFIG_PATH=~/view.rviz
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Allow overriding the config path via environment variable so you don't
    # need the full package installed locally.
    rviz_config = os.environ.get(
        'RVIZ_CONFIG_PATH',
        os.path.expanduser('~/view.rviz'),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([rviz_node])
