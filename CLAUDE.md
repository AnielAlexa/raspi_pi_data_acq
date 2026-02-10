# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# Build the package (from workspace root)
cd ~/ros2_ws && colcon build --packages-select camera_display_node

# Build with verbose output
cd ~/ros2_ws && colcon build --packages-select camera_display_node --event-handlers console_direct+

# Source the workspace after building
source ~/ros2_ws/install/setup.bash

# Run the main camera node (free-run mode, no Pico trigger)
ros2 run camera_display_node camera_display_node --ros-args -p trigger_mode:=false

# Run with Pico trigger sync (default)
ros2 run camera_display_node camera_display_node --ros-args -p camera_index:=0 -p serial_port:=/dev/ttyAMA0

# Run the FPS monitoring subscriber
ros2 run camera_display_node camera_fps_subscriber

# Lint (via ament)
cd ~/ros2_ws && colcon test --packages-select camera_display_node
colcon test-result --verbose
```

## Architecture

This is a **synchronized sensor data acquisition system** for Visual Inertial Odometry (VIO). It couples a **Raspberry Pi Pico** (microcontroller with sensors) to a **Jetson Orin Nano** (running ROS2) to capture time-synchronized camera frames, IMU, and altimeter data.

### Hardware Data Flow

```
Pico (Arduino firmware)              Jetson Orin Nano (ROS2)
├── BMI088 IMU (SPI, 400 Hz)    ──►  /imu/data_raw
├── BMP388 altimeter (I2C, 10 Hz) ►  /altimeter/range
└── Camera trigger (GPIO, 20 Hz) ──►  trigger_map → timestamp lookup
         │                                     ↓
    Serial 230400 baud ──────────►  SerialSync thread
                                           ↓
                              camera_display_node (V4L2)
                              └── /camera/image_mono (20 Hz, Mono8, 1280x720)
```

### Key Synchronization Mechanism

The Pico generates a hardware trigger pulse for the Arducam JetVariety camera at 20 Hz and sends a `TriggerPacket` (frame_id + timestamp) over serial. When a V4L2 frame arrives on the Jetson, `camera_display_node` calibrates the V4L2 `buf.sequence` to the Pico `frame_id` on the first triggered frame, then looks up the frame_id in `trigger_map_` to stamp it with the Pico-synchronized ROS time. Time calibration uses the median of 100 offset samples collected at startup.

### Threading Model (camera_display_node)

1. **V4L2 capture thread** — `captureThreadLoop()`: `select(v4l2_fd_)` blocks with zero CPU until frame ready, DQBUF, Y16→mono8 conversion (OpenCV normalize), swaps pre-allocated buffer, notifies publisher, QBUF requeue
2. **Mono publisher thread** — event-driven via condition variable, publishes every frame (20 Hz)
3. **SerialSync thread** — reads serial packets, validates CRC16, dispatches callbacks, maintains trigger map (last 20 entries)

All threads sleep when idle (zero CPU). Buffers are pre-allocated to avoid per-frame allocations.

### Serial Protocol

Three packet types, all with CRC16 validation:
- `ImuPacket` (header 0xAA55, 44 bytes): timestamp, accel[3], gyro[3]
- `TriggerPacket` (header 0xBB66, 12 bytes): timestamp, frame_id
- `AltimeterPacket` (header 0xCC77, 16 bytes): timestamp, altitude

### Source Layout

- `bmi088_camera_trigger_updated.ino` — Pico firmware: sensor reading, trigger generation, serial packet transmission
- `camera_display_node/src/camera_display.cpp` — Main ROS2 node: V4L2 capture, Y16→mono8 conversion, publishing
- `camera_display_node/src/serial_sync.cpp` — Serial protocol handler: packet parsing, CRC validation, Pico→ROS time synchronization
- `camera_display_node/src/camera_fps_subscriber.cpp` — Standalone FPS monitor with OpenCV display

### Node Parameters

| Parameter | Default | Description |
|---|---|---|
| `camera_index` | 0 | V4L2 device index (/dev/videoN) |
| `width` | 1280 | Capture width |
| `height` | 720 | Capture height |
| `serial_port` | /dev/ttyAMA0 | Pico UART device |
| `enable_pico_sync` | true | Enable Pico time synchronization |
| `exposure` | 700 | V4L2 exposure control value |
| `analogue_gain` | 400 | V4L2 analogue gain control value |
| `trigger_mode` | true | Enable Arducam external trigger mode |

### Dependencies

- **ROS2**: rclcpp, sensor_msgs, ament_cmake
- **System**: OpenCV (V4L2 is kernel-provided)
- **Camera**: Arducam JetVariety (V4L2, Y16 pixel format)
- **C++ standard**: C++17
