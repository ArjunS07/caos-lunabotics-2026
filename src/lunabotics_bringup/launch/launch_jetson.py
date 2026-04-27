"""
Jetson-side launch file.
Runs all sensor and processing nodes. Does NOT start RViz.
RViz runs on the local machine via launch_local.py.

Prerequisites on Jetson:
  export ROS_DOMAIN_ID=42
  export ROS_LOCALHOST_ONLY=0
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'pointcloud.enable': 'true',
            'depth_module.profile': '640x480x30',
            'ordered_pc': 'true'
        }.items()
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_camera_tf',
        arguments=[
            '0.30',    # X: 30cm forward
            '0.0',     # Y: centered
            '0.0',     # Z: same flat surface (adjust if frame origins differ in height)
            '0.0',     # Yaw: facing same direction as lidar
            '-0.2618', # Pitch: -15° (nose down; standard ROS: negative pitch = downward)
            '0.0',     # Roll: no tilt
            'unilidar_lidar',
            'camera_link'
        ]
    )
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
            {'cloud_scan_num': 18},
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
        ]
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
        ]
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
        remappings=[('cloud_in', '/unilidar/cloud')]
    )

    return LaunchDescription([
        realsense_node,
        static_tf,
        tf_base_link,
        lidar_node,
        pointcloud_to_laserscan_node,
        localization,
        octomap_node,
    ])
