/**
 * @file packet_defs.h
 * @brief Pico packet structures for IMU and camera trigger data
 */

#ifndef VIO_SENSOR_PACKET_DEFS_H
#define VIO_SENSOR_PACKET_DEFS_H

#include <cstdint>

namespace vio_sensor {

// Packet structures matching Pico firmware
#pragma pack(push, 1)

struct ImuPacket {
    uint16_t header;        // 0xAA55
    uint32_t timestamp_us;  // Microseconds since Pico boot
    float ax, ay, az;       // Linear acceleration (m/s²)
    float gx, gy, gz;       // Angular velocity (rad/s)
    uint16_t crc16;         // CRC checksum
};

struct TriggerPacket {
    uint16_t header;        // 0xBB66
    uint32_t timestamp_us;  // Microseconds when camera trigger fired
    uint16_t frame_id;      // Frame counter
    uint16_t reserved;      // Reserved for future use
    uint16_t crc16;         // CRC checksum
};

#pragma pack(pop)

// Packet header constants
constexpr uint16_t IMU_HEADER = 0xAA55;
constexpr uint16_t TRIGGER_HEADER = 0xBB66;

// Serial port configuration
constexpr int SERIAL_BAUDRATE = 921600;
constexpr const char* DEFAULT_SERIAL_PORT = "/dev/ttyACM0";

// Time sync calibration
constexpr size_t TIME_OFFSET_SAMPLES = 100;

}  // namespace vio_sensor

#endif  // VIO_SENSOR_PACKET_DEFS_H
