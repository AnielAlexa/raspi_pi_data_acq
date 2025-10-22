#!/usr/bin/env python3
"""
Launch file for VIO sensor node
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

    # VIO sensor node
    vio_sensor_node = Node(
        package='vio_sensor',
        executable='vio_sensor_node',
        name='vio_sensor_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'camera_width': LaunchConfiguration('camera_width'),
            'camera_height': LaunchConfiguration('camera_height'),
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        camera_width_arg,
        camera_height_arg,
        vio_sensor_node
    ])
