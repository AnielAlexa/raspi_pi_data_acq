#!/usr/bin/env python3
"""
Launch file for VIO sensor system - TWO NODE ARCHITECTURE

This launches:
1. camera_only_node: Lightweight camera capture (publishes /camera/image_raw)
2. imu_sync_node: IMU reader + synchronization (publishes /vio/imu/data_raw, /vio/camera/image_synced)

This 2-node architecture prevents camera frame drops by isolating camera capture
from IMU/sync processing in separate processes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Pico IMU data'
    )

    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='Camera image width'
    )

    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='Camera image height'
    )

    # Node 1: Camera-only (lightweight, fast)
    camera_node = Node(
        package='vio_sensor',
        executable='camera_only_node',
        name='camera_only_node',
        output='screen',
        parameters=[{
            'width': LaunchConfiguration('camera_width'),
            'height': LaunchConfiguration('camera_height'),
        }]
    )

    # Node 2: IMU + Sync (subscribes to camera)
    imu_sync_node = Node(
        package='vio_sensor',
        executable='imu_sync_node',
        name='imu_sync_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        camera_width_arg,
        camera_height_arg,
        camera_node,
        imu_sync_node
    ])
