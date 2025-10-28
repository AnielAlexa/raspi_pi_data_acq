/**
 * @file serial_sync.h
 * @brief Minimal serial synchronization for Pico camera triggers
 * Extracted and adapted from vio_sensor for independent operation
 */

#ifndef CAMERA_DISPLAY_NODE_SERIAL_SYNC_H
#define CAMERA_DISPLAY_NODE_SERIAL_SYNC_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

namespace camera_display_node {

// Packet structures matching Pico firmware
#pragma pack(push, 1)

struct ImuPacket {
    uint16_t header;        // 0xAA55
    uint64_t timestamp_us;  // Microseconds since Pico boot
    float ax, ay, az;       // Linear acceleration (m/s²)
    float gx, gy, gz;       // Angular velocity (rad/s)
    uint16_t crc16;         // CRC checksum
};

struct TriggerPacket {
    uint16_t header;        // 0xBB66
    uint64_t timestamp_us;  // Microseconds when camera trigger fired
    uint16_t frame_id;      // Frame counter
    uint16_t reserved;      // Reserved for future use
    uint16_t crc16;         // CRC checksum
};

#pragma pack(pop)

// Packet header constants
constexpr uint16_t IMU_HEADER = 0xAA55;
constexpr uint16_t TRIGGER_HEADER = 0xBB66;
constexpr int SERIAL_BAUDRATE = 230400;
constexpr size_t TIME_OFFSET_SAMPLES = 100;

class SerialSync {
public:
    using ImuCallback = std::function<void(uint32_t timestamp_us, float ax, float ay, float az, float gx, float gy, float gz)>;
    using TriggerCallback = std::function<void(uint32_t timestamp_us, uint16_t frame_id)>;

    SerialSync(rclcpp::Node* node,
               const std::string& port,
               ImuCallback imu_cb,
               TriggerCallback trigger_cb);
    ~SerialSync();

    void start();
    void stop();

    rclcpp::Time pico_to_ros_time(uint32_t pico_us) const;
    bool is_calibrated() const { return time_offset_initialized_.load(); }

private:
    bool open_serial();
    void serial_read_loop();
    void process_imu_packet(const ImuPacket& pkt);
    void process_trigger_packet(const TriggerPacket& pkt);
    void initialize_time_offset(uint32_t pico_us);
    uint16_t calculate_crc16(const uint8_t* data, size_t len);
    int64_t calculate_median(std::vector<int64_t>& samples);

    rclcpp::Node* node_;
    std::string port_;
    int serial_fd_;

    std::thread serial_thread_;
    std::atomic<bool> running_;

    ImuCallback imu_callback_;
    TriggerCallback trigger_callback_;

    std::atomic<bool> time_offset_initialized_;
    int64_t time_offset_ns_;
    std::vector<int64_t> offset_samples_;
};

}  // namespace camera_display_node

#endif  // CAMERA_DISPLAY_NODE_SERIAL_SYNC_H
