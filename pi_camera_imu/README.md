# Pi Camera IMU - ROS2 Package

ROS2 package for synchronized IMX296 Global Shutter camera and BMI088 IMU data capture for VIO applications.

## Hardware Setup

- **Raspberry Pi 4** with ROS2 Jazzy
- **IMX296 Global Shutter Camera** connected to Pi 4 CSI port
- **Raspberry Pi Pico** with BMI088 IMU connected via USB (`/dev/ttyACM0`)
  - Pico sends IMU data @ 400Hz
  - Pico sends camera trigger timestamps @ 20Hz
  - Camera is triggered externally by Pico GPIO 22

## System Architecture

```
Pico (BMI088 IMU) --USB--> Pi 4
        |                    |
        |                    +-- camera_node.py (Python)
        +-- GPIO 22              Captures @ 20 FPS
              |                  Publishes /camera/image_raw
              |
              v
        IMX296 Camera -------> Pi 4
                               |
                               +-- imu_camera_sync_node (C++)
                                   Reads Pico serial data
                                   Syncs timestamps
                                   Publishes:
                                     /imu/data_raw @ 400Hz
                                     /camera/image_synced @ 20Hz
```

## Pico Firmware

Flash the Pico with `bmi088_camera_trigger_updated.ino` (included in workspace).

**Key features:**
- IMU packets (header 0xAA55, 32 bytes) @ 400Hz
- Trigger packets (header 0xBB66, 12 bytes) @ 20Hz
- All packets include microsecond timestamps and CRC16

## Building

```bash
cd /home/pi/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pi_camera_imu
source install/setup.bash
```

## Running

### Full System (Recommended)

```bash
ros2 launch pi_camera_imu camera_imu_system.launch.py
```

This starts:
- `camera_node.py` - Camera capture @ 20 FPS
- `imu_camera_sync_node` - IMU parsing and synchronization

### Individual Nodes

**Camera only:**
```bash
ros2 run pi_camera_imu camera_node.py
```

**IMU sync only:**
```bash
ros2 run pi_camera_imu imu_camera_sync_node
```

## Published Topics

- `/camera/image_raw` (sensor_msgs/Image) - Raw camera frames @ 20Hz
- `/camera/camera_info` (sensor_msgs/CameraInfo) - Camera calibration
- `/camera/image_synced` (sensor_msgs/Image) - Frames with synced Pico timestamps
- `/imu/data_raw` (sensor_msgs/Imu) - IMU data @ 400Hz with synced timestamps
- `/sync/trigger_timestamp` (std_msgs/UInt32) - Pico trigger timestamps (debug)
- `/sync/trigger_frame_id` (std_msgs/UInt16) - Pico frame counter (debug)

## Monitoring

**Check topic rates:**
```bash
ros2 topic hz /camera/image_synced
ros2 topic hz /imu/data_raw
```

**View camera:**
```bash
ros2 run rqt_image_view rqt_image_view
```

**View IMU data:**
```bash
ros2 topic echo /imu/data_raw
```

## Recording for VIO

```bash
ros2 bag record /camera/image_synced /imu/data_raw /camera/camera_info
```

## Camera Settings (VIO Optimized)

- **Resolution:** 640x480 (configurable)
- **Frame rate:** 20 FPS (hardware triggered)
- **Exposure:** 3ms (fixed, no auto-exposure)
- **Gain:** 2.0 (fixed, no auto-gain)
- **White balance:** OFF

Fixed settings are **critical for VIO** - auto-exposure causes brightness changes that break feature tracking.

## Time Synchronization

The system uses Pico microsecond timestamps as the time reference:

1. First IMU/trigger packet establishes time offset
2. All subsequent data uses: `ros_time = pico_timestamp_us * 1000 + offset`
3. Camera frames are stamped with the Pico trigger timestamp
4. IMU data is stamped with Pico sample timestamp

This ensures camera and IMU are perfectly synchronized in the same time domain.

## Parameters

### camera_node.py

- `width` (int, default: 640) - Frame width
- `height` (int, default: 480) - Frame height
- `frame_id` (string, default: "camera") - TF frame ID
- `exposure_time` (int, default: 3000) - Exposure in microseconds
- `analog_gain` (float, default: 2.0) - Camera gain

### imu_camera_sync_node

- `serial_port` (string, default: "/dev/ttyACM0") - Pico serial port
- `serial_baudrate` (int, default: 921600) - Baud rate
- `imu_frame_id` (string, default: "imu") - IMU TF frame ID

## Troubleshooting

**Camera not publishing:**
- Check picamera2 is installed: `python3 -c "import picamera2"`
- Check camera detected: `rpicam-hello --list-cameras`

**No IMU data:**
- Check Pico is connected: `ls /dev/ttyACM*`
- Check Pico firmware is running: `screen /dev/ttyACM0 921600`
- Verify binary data is being sent (should see garbage characters)

**Low frame rate:**
- Ensure no metadata fetching in camera loop (current version is optimized)
- Check CPU usage: `htop`

**Timestamps not synced:**
- Check time offset was initialized (look for log message)
- Verify trigger packets are being received

## Performance

- **Camera:** 19.5-20 FPS @ 640x480
- **IMU:** 400 Hz
- **Camera triggers:** 20 Hz
- **CPU usage:** ~30-40% on Pi 4

## For VIO Applications

This package outputs data compatible with:
- VINS-Mono
- ORB-SLAM3 with IMU
- OpenVINS
- Kimera-VIO

**Next steps:**
1. Calibrate camera: `ros2 run camera_calibration cameracalibrator`
2. Calibrate IMU: Determine noise parameters
3. Run VIO algorithm with recorded bag files

