#!/usr/bin/env python3
"""
All-in-one ROS 2 node (Python) for Raspberry Pi:
- Reads Pico (IMU @400Hz + trigger @20Hz) via serial
- Converts Pico timestamps to ROS time (sliding median offset + wrap handling)
- Publishes IMU + trigger topics
- Captures camera frames (picamera2) and stamps them with the Pico trigger ROS time (FIFO)
Compatible with ROS 2 Jazzy: uses qos_profile_sensor_data instead of SensorDataQoS.
"""

import threading
import collections
import time
import struct
import serial
from statistics import median

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,   # <-- use this preset for sensor streams
)

from sensor_msgs.msg import Imu, Image, CameraInfo
from std_msgs.msg import UInt16, UInt32
from builtin_interfaces.msg import Time as RosTime
from std_msgs.msg import Header

import cv2
import numpy as np
from picamera2 import Picamera2

# ---------------------- CRC16 (Modbus/A001) ----------------------
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

# Packet sizes and struct formats (little-endian, packed)
IMU_FMT = "<H I f f f f f f H"     # header(0xAA55), ts_us(u32), ax ay az gx gy gz, crc16
TRIG_FMT = "<H I H H H"            # header(0xBB66), ts_us(u32), frame_id(u16), reserved(u16), crc16
IMU_SIZE = struct.calcsize(IMU_FMT)    # 32 bytes
TRIG_SIZE = struct.calcsize(TRIG_FMT)  # 12 bytes

class ImuCamAllInOne(Node):
    def __init__(self):
        super().__init__('imu_cam_all_in_one')

        # ---------------- Parameters ----------------
        # Serial
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baudrate', 921600)

        # Time sync
        self.declare_parameter('offset_window_size', 200)  # sliding window for median
        self.declare_parameter('min_init_samples', 100)    # before first publish
        self.declare_parameter('offset_log_every', 20)
        self.declare_parameter('publish_before_sync', False)

        # Camera
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('exposure_time', 3000)  # us
        self.declare_parameter('analog_gain', 2.0)
        self.declare_parameter('exposure_offset_us', 0)       # shift stamp (e.g., +exposure/2)
        self.declare_parameter('use_approx_now_if_missing', True)
        self.declare_parameter('trigger_queue_limit', 200)

        # IMU
        self.declare_parameter('imu_frame_id', 'imu')
        self.declare_parameter('imu_cov_lin', 0.01)
        self.declare_parameter('imu_cov_ang', 0.01)

        # Get params
        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = int(self.get_parameter('serial_baudrate').value)

        self.offset_window_size = max(10, int(self.get_parameter('offset_window_size').value))
        self.min_init_samples = max(20, int(self.get_parameter('min_init_samples').value))
        self.offset_log_every = max(1, int(self.get_parameter('offset_log_every').value))
        self.publish_before_sync = bool(self.get_parameter('publish_before_sync').value)

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.exposure_time = int(self.get_parameter('exposure_time').value)
        self.analog_gain = float(self.get_parameter('analog_gain').value)
        self.exposure_offset_us = int(self.get_parameter('exposure_offset_us').value)
        self.use_approx_now_if_missing = bool(self.get_parameter('use_approx_now_if_missing').value)
        self.trigger_queue_limit = int(self.get_parameter('trigger_queue_limit').value)

        self.imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self.imu_cov_lin = float(self.get_parameter('imu_cov_lin').value)
        self.imu_cov_ang = float(self.get_parameter('imu_cov_ang').value)

        # ---------------- Publishers ----------------
        # Use qos_profile_sensor_data (BEST_EFFORT, KEEP_LAST small depth) for fast sensor streams
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
        self.trig_ts_pub = self.create_publisher(UInt32, '/sync/trigger_timestamp', 10)
        self.trig_fid_pub = self.create_publisher(UInt16, '/sync/trigger_frame_id', 10)
        self.trig_ros_pub = self.create_publisher(RosTime, '/sync/trigger_time_ros', 10)

        self.image_pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.info_pub  = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # ---------------- Serial ----------------
        try:
            self.ser = serial.Serial(self.serial_port, self.serial_baud, timeout=0.01)
        except Exception as e:
            self.get_logger().fatal(f"Failed to open serial {self.serial_port} @ {self.serial_baud}: {e}")
            raise SystemExit(1)

        self.buffer = bytearray()
        self.serial_running = True
        self.serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.serial_thread.start()

        # ---------------- Time sync state ----------------
        self.time_offset_ns = 0
        self.time_offset_initialized = False
        self.offset_samples = []
        self.offset_lock = threading.Lock()

        # 32-bit Pico wrap handling
        self.pico_epoch_us = 0
        self.last_pico_us = None
        self.wrap_guard_us = 500_000  # 0.5 s
        self.wrap_lock = threading.Lock()

        # Latest trigger ROS time queue (FIFO for camera stamping)
        self.trig_lock = threading.Lock()
        self.trigger_fifo = collections.deque(maxlen=self.trigger_queue_limit)

        # warn throttle state (since rclpy lacks warn_once/ throttle)
        self._last_warn_no_trigger = 0.0

        # ---------------- Camera ----------------
        self.get_logger().info('Initializing picamera2...')
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"},
            buffer_count=4
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(0.5)
        self.picam2.set_controls({
            "AeEnable": False,
            "AwbEnable": False,
            "ExposureTime": self.exposure_time,
            "AnalogueGain": self.analog_gain,
        })
        time.sleep(0.5)
        self.get_logger().info(f'Camera started: {self.width}x{self.height}, exposure {self.exposure_time}us, gain {self.analog_gain}')

        self.camera_info = self._make_camera_info()

        # Capture timer (~100 Hz; camera limits to ~20 FPS)
        self.frame_count = 0
        self.last_log_time = self.get_clock().now()
        self.create_timer(0.01, self._capture_once)

        self.get_logger().info('All-in-one IMU+Trigger+Camera node running')

    # ---------------- Utility: Pico time expansion ----------------
    def _expand_pico_us(self, pico_us32: int) -> int:
        with self.wrap_lock:
            if self.last_pico_us is None:
                self.last_pico_us = pico_us32
                return self.pico_epoch_us + pico_us32
            if pico_us32 + self.wrap_guard_us < self.last_pico_us:
                # wrap detected
                self.pico_epoch_us += (1 << 32)
                self.get_logger().warn('Detected Pico timestamp wrap (~71.6 min). Epoch advanced by 2^32 µs')
            self.last_pico_us = pico_us32
            return self.pico_epoch_us + pico_us32

    def _maybe_update_offset(self, pico_full_us: int):
        # sample: ros_now = pico_now + offset
        ros_ns = self.get_clock().now().nanoseconds
        pico_ns = int(pico_full_us) * 1000
        sample = ros_ns - pico_ns
        with self.offset_lock:
            self.offset_samples.append(sample)
            if len(self.offset_samples) > self.offset_window_size:
                self.offset_samples.pop(0)

            if not self.time_offset_initialized:
                if len(self.offset_samples) % self.offset_log_every == 0:
                    self.get_logger().info(f'Collecting time offset samples: {len(self.offset_samples)}/{self.min_init_samples}')
                if len(self.offset_samples) >= self.min_init_samples:
                    med = int(median(self.offset_samples))
                    self.time_offset_ns = med
                    self.time_offset_initialized = True
                    rng = max(self.offset_samples) - min(self.offset_samples)
                    self.get_logger().info(f'✓ Time offset calibration complete. median={med/1e6:.3f} ms, range={rng/1e6:.3f} ms')
            else:
                # continuous refresh
                self.time_offset_ns = int(median(self.offset_samples))

    def _pico_full_us_to_ros_time(self, pico_full_us: int):
        ros_ns = int(pico_full_us) * 1000 + int(self.time_offset_ns)
        sec = ros_ns // 1_000_000_000
        nsec = ros_ns % 1_000_000_000
        t = RosTime()
        t.sec = int(sec)
        t.nanosec = int(nsec)
        return t

    # ---------------- Serial reading & parsing ----------------
    def _serial_loop(self):
        imu_count = 0
        trig_count = 0
        crc_errors = 0
        last_stats = time.time()

        while self.serial_running and rclpy.ok():
            try:
                chunk = self.ser.read(1024)
                if chunk:
                    self.buffer.extend(chunk)
                    i = 0
                    # scan buffer
                    while i + 1 < len(self.buffer):
                        header = self.buffer[i] | (self.buffer[i+1] << 8)
                        if header == 0xAA55 and i + IMU_SIZE <= len(self.buffer):
                            pkt = bytes(self.buffer[i:i+IMU_SIZE])
                            calc = crc16_modbus(pkt[:-2])
                            crc = struct.unpack_from('<H', pkt, IMU_SIZE - 2)[0]
                            if calc == crc:
                                _, ts_us, ax, ay, az, gx, gy, gz, _ = struct.unpack(IMU_FMT, pkt)
                                pico_full_us = self._expand_pico_us(ts_us)
                                self._maybe_update_offset(pico_full_us)

                                if self.time_offset_initialized or self.publish_before_sync:
                                    msg = Imu()
                                    ros_time = self._pico_full_us_to_ros_time(pico_full_us)
                                    msg.header = Header()
                                    msg.header.stamp = ros_time
                                    msg.header.frame_id = self.imu_frame_id
                                    msg.linear_acceleration.x = ax
                                    msg.linear_acceleration.y = ay
                                    msg.linear_acceleration.z = az
                                    msg.angular_velocity.x = gx
                                    msg.angular_velocity.y = gy
                                    msg.angular_velocity.z = gz
                                    msg.orientation_covariance[0] = -1.0
                                    msg.orientation_covariance[4] = -1.0
                                    msg.orientation_covariance[8] = -1.0
                                    msg.linear_acceleration_covariance[0] = self.imu_cov_lin
                                    msg.linear_acceleration_covariance[4] = self.imu_cov_lin
                                    msg.linear_acceleration_covariance[8] = self.imu_cov_lin
                                    msg.angular_velocity_covariance[0] = self.imu_cov_ang
                                    msg.angular_velocity_covariance[4] = self.imu_cov_ang
                                    msg.angular_velocity_covariance[8] = self.imu_cov_ang
                                    self.imu_pub.publish(msg)

                                imu_count += 1
                                del self.buffer[:i+IMU_SIZE]
                                i = 0
                                continue
                            else:
                                crc_errors += 1
                                del self.buffer[i]
                                continue

                        elif header == 0xBB66 and i + TRIG_SIZE <= len(self.buffer):
                            pkt = bytes(self.buffer[i:i+TRIG_SIZE])
                            calc = crc16_modbus(pkt[:-2])
                            crc = struct.unpack_from('<H', pkt, TRIG_SIZE - 2)[0]
                            if calc == crc:
                                _, ts_us, frame_id, _reserved, _ = struct.unpack(TRIG_FMT, pkt)
                                pico_full_us = self._expand_pico_us(ts_us)
                                self._maybe_update_offset(pico_full_us)

                                # publish debug topics
                                u32 = UInt32(); u32.data = ts_us
                                self.trig_ts_pub.publish(u32)
                                u16 = UInt16(); u16.data = frame_id
                                self.trig_fid_pub.publish(u16)

                                # publish ROS trigger time and enqueue for camera
                                ros_time = self._pico_full_us_to_ros_time(pico_full_us)
                                self.trig_ros_pub.publish(ros_time)
                                with self.trig_lock:
                                    self.trigger_fifo.append(ros_time)

                                trig_count += 1
                                del self.buffer[:i+TRIG_SIZE]
                                i = 0
                                continue
                            else:
                                crc_errors += 1
                                del self.buffer[i]
                                continue
                        else:
                            i += 1

                    # avoid unbounded buffer
                    if len(self.buffer) > 4096:
                        self.buffer = self.buffer[-512:]
                # stats
                now = time.time()
                if now - last_stats >= 1.0:
                    self.get_logger().info(
                        f'IMU: {imu_count} Hz, Triggers: {trig_count} Hz, CRC errors: {crc_errors}'
                        + ('' if self.time_offset_initialized else ' (calibrating...)')
                    )
                    imu_count = trig_count = crc_errors = 0
                    last_stats = now
            except Exception as e:
                self.get_logger().error(f'Serial loop error: {e}')
                time.sleep(0.01)

    # ---------------- Camera capture ----------------
    def _make_camera_info(self) -> CameraInfo:
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [
            float(self.width), 0.0, float(self.width / 2),
            0.0, float(self.width), float(self.height / 2),
            0.0, 0.0, 1.0
        ]
        info.r = [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]
        info.p = [
            float(self.width), 0.0, float(self.width / 2), 0.0,
            0.0, float(self.width), float(self.height / 2), 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        return info

    def _throttled_warn(self, key: str, msg: str, period_s: float = 5.0):
        now = time.monotonic()
        if key == 'no_trigger':
            if now - self._last_warn_no_trigger >= period_s:
                self.get_logger().warn(msg)
                self._last_warn_no_trigger = now

    def _capture_once(self):
        try:
            frame = self.picam2.capture_array("main")  # RGB888
            # Convert to BGR for ROS (to match 'bgr8'); to save CPU you can publish 'rgb8' and skip this
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Get trigger-based timestamp (FIFO)
            stamp = None
            with self.trig_lock:
                if self.trigger_fifo:
                    stamp = self.trigger_fifo.popleft()

            if stamp is None:
                if self.use_approx_now_if_missing:
                    stamp = self.get_clock().now().to_msg()
                else:
                    self._throttled_warn('no_trigger', 'No trigger available; dropping frame (fallback disabled)')
                    return

            # Optional exposure midpoint shift
            if self.exposure_offset_us != 0:
                total_ns = stamp.sec * 1_000_000_000 + stamp.nanosec + self.exposure_offset_us * 1000
                if total_ns < 0:
                    total_ns = 0
                stamp.sec = total_ns // 1_000_000_000
                stamp.nanosec = total_ns % 1_000_000_000

            # Image msg
            img = Image()
            img.header = Header()
            img.header.stamp = stamp
            img.header.frame_id = self.frame_id
            img.height = self.height
            img.width = self.width
            img.encoding = 'bgr8'
            img.is_bigendian = False
            img.step = self.width * 3
            img.data = frame_bgr.tobytes()
            self.image_pub.publish(img)

            # CameraInfo with SAME timestamp
            self.camera_info.header.stamp = stamp
            self.camera_info.header.frame_id = self.frame_id
            self.info_pub.publish(self.camera_info)

            # Stats
            self.frame_count += 1
            now = self.get_clock().now()
            elapsed = (now - self.last_log_time).nanoseconds / 1e9
            if elapsed >= 1.0:
                fps = self.frame_count / elapsed
                self.get_logger().info(f'Publishing at {fps:.2f} FPS')
                self.frame_count = 0
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f'Capture error: {e}')

    # ---------------- Cleanup ----------------
    def destroy_node(self):
        self.get_logger().info('Stopping...')
        try:
            self.serial_running = False
            try:
                self.ser.close()
            except Exception:
                pass
            try:
                self.picam2.stop()
            except Exception:
                pass
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuCamAllInOne()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
