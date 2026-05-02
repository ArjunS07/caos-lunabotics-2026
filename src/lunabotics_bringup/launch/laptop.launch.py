"""
Laptop-side launch file.
Runs the Mission FSM and RViz. All sensor and navigation topics come from
the Jetson over DDS (start robot.launch.py there first).

The Mission FSM is offloaded here because it is lightweight Python that
communicates with Nav2 purely over ROS 2 action/service interfaces — no
hardware access or latency-sensitive loops. Running it on the laptop lets
the operator monitor mission state and trigger autonomy without an SSH
session into the Jetson.

Prerequisites on laptop:
  export ROS_DOMAIN_ID=42
  export ROS_LOCALHOST_ONLY=0

Trigger autonomy after both sides are up:
  ros2 service call /autonomy_start std_srvs/srv/Trigger
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('lunabotics_bringup')

    # ── Mission FSM ───────────────────────────────────────────────────────────
    # Lightweight: waits for /autonomy_start, sends Nav2 NavigateToPose goal,
    # publishes /mission_state (IDLE → TRAVERSING → ARRIVED).

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('lunabotics_mission'),
                'launch', 'mission.launch.py'),
        ]),
    )

    # ── RViz ─────────────────────────────────────────────────────────────────

    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    return LaunchDescription([
        mission,
        rviz_node,
    ])
