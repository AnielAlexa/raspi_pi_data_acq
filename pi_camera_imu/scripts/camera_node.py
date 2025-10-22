#!/usr/bin/env python3
"""
ROS2 Camera Node using picamera2
Optimized for 20 FPS capture with minimal overhead
NO metadata fetching for maximum performance

Publishes:
  - /camera/image_raw (sensor_msgs/Image)
  - /camera/camera_info (sensor_msgs/CameraInfo)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import cv2
import numpy as np
from picamera2 import Picamera2
import time

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declare parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('exposure_time', 3000)  # microseconds
        self.declare_parameter('analog_gain', 2.0)

        # Get parameters
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.frame_id = self.get_parameter('frame_id').value
        self.exposure_time = self.get_parameter('exposure_time').value
        self.analog_gain = self.get_parameter('analog_gain').value

        # Create publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Initialize camera
        self.get_logger().info('Initializing picamera2...')
        self.picam2 = Picamera2()

        # Minimal configuration for maximum speed
        config = self.picam2.create_video_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"},
            buffer_count=4
        )

        self.picam2.configure(config)
        self.picam2.start()

        # Wait for camera to stabilize
        time.sleep(0.5)

        # Set manual controls for VIO (fixed exposure/gain)
        self.picam2.set_controls({
            "AeEnable": False,           # Disable auto-exposure
            "AwbEnable": False,          # Disable auto-white-balance
            "ExposureTime": self.exposure_time,
            "AnalogueGain": self.analog_gain,
        })

        time.sleep(0.5)

        self.get_logger().info(f'Camera started: {self.width}x{self.height}')
        self.get_logger().info(f'Exposure: {self.exposure_time}us, Gain: {self.analog_gain}')
        self.get_logger().info('Auto-exposure and auto-white-balance DISABLED for VIO')

        # Initialize camera info
        self.camera_info = self.create_camera_info()

        # Create timer for capture (as fast as possible)
        # Using 10ms timer (~100Hz) but actual rate limited by camera
        self.timer = self.create_timer(0.01, self.capture_and_publish)

        # Statistics
        self.frame_count = 0
        self.last_log_time = self.get_clock().now()

    def create_camera_info(self):
        """Create camera info message with default calibration"""
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = "plumb_bob"

        # Default calibration (user should calibrate and load actual values)
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Distortion coefficients

        # Camera matrix K
        info.k = [
            float(self.width), 0.0, float(self.width / 2),
            0.0, float(self.width), float(self.height / 2),
            0.0, 0.0, 1.0
        ]

        # Rectification matrix R (identity)
        info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix P
        info.p = [
            float(self.width), 0.0, float(self.width / 2), 0.0,
            0.0, float(self.width), float(self.height / 2), 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        return info

    def capture_and_publish(self):
        """Capture frame and publish - MINIMAL overhead version"""
        try:
            # Capture frame (NO metadata fetching!)
            frame = self.picam2.capture_array("main")

            # Get timestamp
            timestamp = self.get_clock().now().to_msg()

            # Convert RGB to BGR for ROS (sensor_msgs/Image expects BGR8)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Create Image message
            img_msg = Image()
            img_msg.header = Header()
            img_msg.header.stamp = timestamp
            img_msg.header.frame_id = self.frame_id
            img_msg.height = self.height
            img_msg.width = self.width
            img_msg.encoding = 'bgr8'
            img_msg.is_bigendian = False
            img_msg.step = self.width * 3
            img_msg.data = frame_bgr.tobytes()

            # Publish image
            self.image_pub.publish(img_msg)

            # Publish camera info with same timestamp
            self.camera_info.header.stamp = timestamp
            self.info_pub.publish(self.camera_info)

            self.frame_count += 1

            # Log statistics every second
            now = self.get_clock().now()
            elapsed = (now - self.last_log_time).nanoseconds / 1e9
            if elapsed >= 1.0:
                fps = self.frame_count / elapsed
                self.get_logger().info(f'Publishing at {fps:.2f} FPS')
                self.frame_count = 0
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f'Capture error: {e}')

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Stopping camera...')
        self.picam2.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
