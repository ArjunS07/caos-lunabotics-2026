"""
Lidar odometry (rf2o) + optional EKF (robot_localization) fusing RealSense IMU.

When use_imu_fusion is false (e.g. Jetson without camera), rf2o publishes odom→base_link TF.
When true, rf2o publishes /odom_rf2o only; ekf_filter_node publishes smoothed /odom and TF.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabotics_localization')
    ekf_yaml = os.path.join(pkg_share, 'config', 'ekf.yaml')

    use_imu = LaunchConfiguration('use_imu_fusion')

    declared = [
        DeclareLaunchArgument(
            'use_imu_fusion',
            default_value='true',
            description='If true, run EKF and fuse /odom_rf2o + IMU. If false, rf2o publishes TF alone.',
        ),
    ]

    rf2o_fused = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[
            {'laser_scan_topic': '/scan'},
            {'odom_topic': '/odom_rf2o'},
            {'base_frame_id': 'base_link'},
            {'odom_frame_id': 'odom'},
            {'publish_tf': False},
            {'init_pose_from_topic': ''},
            {'freq': 20.0},
        ],
        condition=IfCondition(use_imu),
    )

    rf2o_standalone = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[
            {'laser_scan_topic': '/scan'},
            {'odom_topic': '/odom_rf2o'},
            {'base_frame_id': 'base_link'},
            {'odom_frame_id': 'odom'},
            {'publish_tf': True},
            {'init_pose_from_topic': ''},
            {'freq': 20.0},
        ],
        condition=UnlessCondition(use_imu),
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],
        condition=IfCondition(use_imu),
    )

    return LaunchDescription(declared + [rf2o_fused, rf2o_standalone, ekf_node])
