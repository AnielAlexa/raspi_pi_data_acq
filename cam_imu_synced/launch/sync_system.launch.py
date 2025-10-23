#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch IMU + Trigger and Camera components in a single container with intra-process comms."""

    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Pico IMU/trigger data'
    )

    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Camera index (0 = first camera)'
    )

    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Camera image width'
    )

    height_arg = DeclareLaunchArgument(
        'height',
        default_value='480',
        description='Camera image height'
    )

    disable_ae_arg = DeclareLaunchArgument(
        'disable_ae',
        default_value='false',
        description='Disable auto-exposure'
    )

    disable_awb_arg = DeclareLaunchArgument(
        'disable_awb',
        default_value='false',
        description='Disable auto-white-balance'
    )

    imu_frame_id_arg = DeclareLaunchArgument(
        'imu_frame_id',
        default_value='imu_link',
        description='Frame ID for IMU messages'
    )

    camera_frame_id_arg = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='camera_link',
        description='Frame ID for camera messages'
    )

    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    camera_index = LaunchConfiguration('camera_index')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    disable_ae = LaunchConfiguration('disable_ae')
    disable_awb = LaunchConfiguration('disable_awb')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    camera_frame_id = LaunchConfiguration('camera_frame_id')

    # Create composable node container with intra-process comms enabled
    container = ComposableNodeContainer(
        name='cam_imu_sync_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # IMU + Trigger component
            ComposableNode(
                package='cam_imu_synced',
                plugin='cam_imu_synced::ImuTriggerComponent',
                name='imu_trigger_component',
                parameters=[{
                    'serial_port': serial_port,
                    'imu_frame_id': imu_frame_id,
                    'trigger_frame_id': camera_frame_id,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Camera component
            ComposableNode(
                package='cam_imu_synced',
                plugin='cam_imu_synced::CameraComponent',
                name='camera_component',
                parameters=[{
                    'camera_index': camera_index,
                    'width': width,
                    'height': height,
                    'camera_frame_id': camera_frame_id,
                    'disable_ae': disable_ae,
                    'disable_awb': disable_awb,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        camera_index_arg,
        width_arg,
        height_arg,
        disable_ae_arg,
        disable_awb_arg,
        imu_frame_id_arg,
        camera_frame_id_arg,
        container,
    ])
