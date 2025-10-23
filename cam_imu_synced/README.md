# cam_imu_synced

Synchronized IMU and camera capture using composable ROS2 components with intra-process communication.

## Architecture

This package implements a low-latency, synchronized IMU-camera capture system using two composable components running in a single process:

```
┌──────────────────────────────────────────────────────────────────────┐
│           rclcpp component container (intra-process ON)              │
│                                                                      │
│  ┌──────────────────────┐                     ┌────────────────────┐ │
│  │  IMU + Trigger       │   (fast, no ser.)   │    Camera          │ │
│  │  component (C++)     │ ─────────────────▶  │    component (C++) │ │
│  │                      │       (I.P.C.)      │                    │ │
│  │  • Reads Pico via    │                     │  • Subscribes to   │ │
│  │    serial (/dev/tty) │                     │    trigger times   │ │
│  │  • CRC validation    │                     │  • Opens libcamera │ │
│  │  • Time offset       │                     │  • FIFO matching   │ │
│  │    (median filter)   │                     │    trigger→frame   │ │
│  │                      │                     │                    │ │
│  │  Publishes:          │                     │  Publishes:        │ │
│  │   /imu/data_raw      │                     │   /camera/image_   │ │
│  │   /sync/trigger_     │                     │     synced (rgb8)  │ │
│  │     time_ros         │                     │   /camera/camera_  │ │
│  │   /sync/trigger_     │                     │     info           │ │
│  │     frame_id         │                     │                    │ │
│  └──────────────────────┘                     └────────────────────┘ │
└──────────────────────────────────────────────────────────────────────┘
```

## Features

- **Intra-process communication**: Zero-copy message passing between components for minimal latency
- **FIFO frame matching**: Simple and fast trigger-to-frame association
- **Median time offset**: Robust time synchronization between Pico and ROS clock
- **Minimal threading**: Publish directly in callbacks for lowest latency
- **Based on proven code**: Camera capture based on standalone_camera_test.cpp (20Hz proven)

## Components

### 1. ImuTriggerComponent

Reads serial data from Raspberry Pi Pico and publishes:
- `/imu/data_raw` (sensor_msgs/Imu): IMU measurements
- `/sync/trigger_time_ros` (std_msgs/Header): Trigger timestamps in ROS time
- `/sync/trigger_frame_id` (std_msgs/UInt16): Frame IDs from Pico

**Time synchronization**: Uses a rolling median filter (50 samples, updates every 10) to compute robust time offset between Pico microsecond timestamps and ROS time.

### 2. CameraComponent

Captures frames from Raspberry Pi camera and publishes:
- `/camera/image_synced` (sensor_msgs/Image): RGB8 images with trigger timestamps
- `/camera/camera_info` (sensor_msgs/CameraInfo): Camera intrinsics

**Frame matching**: Uses FIFO queue (size 5) to match incoming frames with trigger timestamps. When a frame arrives, the oldest trigger is popped and used for timestamping.

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select cam_imu_synced --symlink-install
source install/setup.bash
```

## Usage

### Launch the synchronized capture system:

```bash
ros2 launch cam_imu_synced sync_system.launch.py
```

### Launch with custom parameters:

```bash
ros2 launch cam_imu_synced sync_system.launch.py \
  serial_port:=/dev/ttyACM0 \
  width:=640 \
  height:=480 \
  disable_ae:=false \
  disable_awb:=false
```

### Available launch parameters:

- `serial_port`: Serial port for Pico (default: `/dev/ttyACM0`)
- `camera_index`: Camera index (default: `0`)
- `width`: Image width (default: `640`)
- `height`: Image height (default: `480`)
- `disable_ae`: Disable auto-exposure (default: `false`)
- `disable_awb`: Disable auto-white-balance (default: `false`)
- `imu_frame_id`: IMU frame ID (default: `imu_link`)
- `camera_frame_id`: Camera frame ID (default: `camera_link`)

## Recording Data

Record synchronized data with rosbag2:

```bash
ros2 bag record /camera/image_synced /imu/data_raw /camera/camera_info \
  --compression-mode file --compression-format zstd
```

## Pico Serial Protocol

The components expect the following packet formats from the Pico:

### IMU Packet (32 bytes):
```
[header:1][timestamp_us:4][accel_xyz:12][gyro_xyz:12][crc:1]
header = 0xAA
```

### Trigger Packet (8 bytes):
```
[header:1][timestamp_us:4][frame_id:2][crc:1]
header = 0xBB
```

**Note**: The current implementation uses simple XOR checksum. Replace `validateCRC()` in [imu_trigger_component.cpp](src/imu_trigger_component.cpp) with your actual CRC implementation if needed.

## Performance

Expected performance:
- **Frame rate**: 20 Hz (based on standalone_camera_test.cpp baseline)
- **Latency**: Minimal due to:
  - Intra-process comms (no serialization)
  - Direct publishing in libcamera callback
  - FIFO matching (no search or sorting)

## Topics

### Published by ImuTriggerComponent:
- `/imu/data_raw` (sensor_msgs/Imu): IMU measurements
- `/sync/trigger_time_ros` (std_msgs/Header): Trigger timestamps
- `/sync/trigger_frame_id` (std_msgs/UInt16): Frame IDs

### Published by CameraComponent:
- `/camera/image_synced` (sensor_msgs/Image, rgb8): Synchronized camera frames
- `/camera/camera_info` (sensor_msgs/CameraInfo): Camera calibration

## Troubleshooting

### Serial port permissions:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Check camera access:
```bash
libcamera-hello --list-cameras
```

### Monitor topics:
```bash
# Check IMU data
ros2 topic echo /imu/data_raw

# Check trigger timestamps
ros2 topic echo /sync/trigger_time_ros

# Check camera images (rate only)
ros2 topic hz /camera/image_synced
```

### Debug logging:
```bash
ros2 launch cam_imu_synced sync_system.launch.py --ros-args --log-level debug
```

## Next Steps

1. **Calibrate camera**: Replace placeholder intrinsics in CameraComponent with actual calibration
2. **Tune time offset**: Adjust `offset_window_size_` and `offset_update_interval_` if needed
3. **Validate sync accuracy**: Measure actual timestamp alignment between triggers and frames
4. **Implement actual CRC**: Replace XOR checksum with your Pico's CRC algorithm

## License

Apache-2.0
