"""
Bench / cart MVP: LiDAR only (no RealSense), rf2o odometry + octomap + RViz.

Use when the sensor is on a box/cart and you push it around to validate
TF, /scan, /odom_rf2o, and mapping — without IMU fusion (same as Jetson stack + RViz).

Pose output: /odom_rf2o and TF odom→base_link (no /odometry/filtered unless you enable IMU).
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    tf_base_link = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('lunabotics_bringup'),
                'launch',
                'tf_base_link.launch.py',
            ),
        ]),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('lunabotics_localization'),
                'launch',
                'localization.launch.py',
            ),
        ]),
        launch_arguments={'use_imu_fusion': 'false'}.items(),
    )

    lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[
            {'initialize_type': 2},
            {'work_mode': 0},
            {'use_system_timestamp': True},
            {'range_min': 0.0},
            {'range_max': 100.0},
            {'cloud_scan_num': 10},
            {'serial_port': '/dev/ttyACM0'},
            {'baudrate': 4000000},
            {'lidar_port': 6101},
            {'lidar_ip': '192.168.1.62'},
            {'local_port': 6201},
            {'local_ip': '192.168.1.2'},
            {'cloud_frame': 'unilidar_lidar'},
            {'cloud_topic': 'unilidar/cloud'},
            {'imu_frame': 'unilidar_imu'},
            {'imu_topic': 'unilidar/imu'},
        ],
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'unilidar_lidar',
            'transform_tolerance': 0.01,
            'min_height': -0.1,
            'max_height': 1.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00872,
            'range_min': 0.1,
            'range_max': 50.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/unilidar/cloud'),
            ('scan', '/scan'),
        ],
    )

    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[{
            'resolution': 0.1,
            'frame_id': 'unilidar_lidar',
            'sensor_model/max_range': 5.0,
            'occupancy_min_z': 0.05,
            'occupancy_max_z': 0.5,
            'filter_speckles': True,
            'filter_ground': False,
            'sensor_model/hit': 0.9,
            'sensor_model/miss': 0.2,
            'sensor_model/min': 0.12,
            'sensor_model/max': 0.97,
        }],
        remappings=[('cloud_in', '/unilidar/cloud')],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('lunabotics_bringup'), 'rviz', 'view.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='log',
    )

    return LaunchDescription([
        tf_base_link,
        lidar_node,
        pointcloud_to_laserscan_node,
        localization,
        octomap_node,
        rviz_node,
    ])
