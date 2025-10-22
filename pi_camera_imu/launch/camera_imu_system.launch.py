#!/usr/bin/env python3
"""
Launch file for complete Camera + IMU system
Starts camera node and IMU sync node together
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera Node (Python)
        Node(
            package='pi_camera_imu',
            executable='camera_node.py',
            name='camera_node',
            output='screen',
            parameters=[{
                'width': 640,
                'height': 480,
                'frame_id': 'camera',
                'exposure_time': 3000,  # 3ms
                'analog_gain': 2.0,
            }]
        ),

        # IMU Camera Sync Node (C++)
        Node(
            package='pi_camera_imu',
            executable='imu_camera_sync_node',
            name='imu_camera_sync_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'serial_baudrate': 921600,
                'imu_frame_id': 'imu',
            }]
        ),
    ])
