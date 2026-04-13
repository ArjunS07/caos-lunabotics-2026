"""
TK variant launch file (single-machine, all nodes + RViz).
Differs from launch.py in the static TF arguments (no pitch offset).
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # 1. RealSense Driver
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'pointcloud.enable': 'true',          # Required for 3D data
            'depth_module.profile': '640x480x30', # Optimized for Orin Nano
            'ordered_pc': 'true'
        }.items()
    )

    # 2. Unitree Lidar node
    node1 = Node(
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
            {'cloud_frame': "unilidar_lidar"},
            {'cloud_topic': "unilidar/cloud"},
            {'imu_frame': "unilidar_imu"},
            {'imu_topic': "unilidar/imu"},
        ]
    )

    # 3. Static Transform: Lidar → Camera (no pitch offset variant)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_camera_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'unilidar_lidar', 'camera_link']
    )

    # 4. Convert PointCloud2 → LaserScan
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
        ]
    )

    # 5. Octomap Server
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
        remappings=[('cloud_in', '/unilidar/cloud')]
    )

    # 6. RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('lunabotics_bringup'), 'rviz', 'view.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='log'
    )

    return LaunchDescription([
        realsense_node,
        node1,
        static_tf,
        pointcloud_to_laserscan_node,
        octomap_node,
        rviz_node,
    ])
