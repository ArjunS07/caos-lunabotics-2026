"""
Jetson-side launch file.
Runs all sensor, processing, navigation, and mission nodes. Does NOT start RViz.
RViz runs on the local machine via launch_local.py.

Prerequisites on Jetson:
  export ROS_DOMAIN_ID=42
  export ROS_LOCALHOST_ONLY=0

Start autonomy after launch:
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

    # ── Sensors ──────────────────────────────────────────────────────────────

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'pointcloud.enable': 'true',
            'depth_module.profile': '640x480x30',
            'ordered_pc': 'true',
        }.items(),
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
        ],
    )

    # ── TF tree ───────────────────────────────────────────────────────────────

    tf_base_link = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, 'launch', 'tf_base_link.launch.py'),
        ]),
    )

    # Static TF: unilidar_lidar → camera_link
    # Camera is 30 cm forward of LiDAR, tilted 15° downward
    static_tf_lidar_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_camera_tf',
        arguments=[
            '0.30',    # X: 30 cm forward
            '0.0',     # Y: centred
            '0.0',     # Z: same height
            '0.0',     # Yaw
            '-0.2618', # Pitch: -15°
            '0.0',     # Roll
            'unilidar_lidar',
            'camera_link',
        ],
    )

    # ── Localization (ICP) ───────────────────────────────────────────────────

    icp_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('lunabotics_icp_localization'),
                'launch', 'icp_localization.launch.py'),
        ]),
    )

    # ── Terrain mapping ───────────────────────────────────────────────────────

    elevation_mapping_node = Node(
        package='elevation_mapping',
        executable='elevation_mapping_node',
        name='elevation_mapping',
        output='screen',
        parameters=[os.path.join(bringup_dir, 'config', 'elevation_mapping.yaml')],
    )

    # ── Crater detection ──────────────────────────────────────────────────────

    crater_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('lunabotics_detection'),
                'launch', 'crater_detection.launch.py'),
        ]),
    )

    # ── Navigation (Nav2) ─────────────────────────────────────────────────────

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, 'launch', 'navigation.launch.py'),
        ]),
    )

    # ── Mission FSM ───────────────────────────────────────────────────────────

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('lunabotics_mission'),
                'launch', 'mission.launch.py'),
        ]),
    )

    return LaunchDescription([
        # Sensors first
        realsense_node,
        lidar_node,
        # TF
        tf_base_link,
        static_tf_lidar_camera,
        # Localization
        icp_localization,
        # Terrain + detection
        elevation_mapping_node,
        crater_detection,
        # Navigation
        navigation,
        # Mission controller
        mission,
    ])
