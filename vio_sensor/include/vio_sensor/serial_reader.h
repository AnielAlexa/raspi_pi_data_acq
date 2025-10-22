/**
 * @file serial_reader.h
 * @brief Serial reader for IMU and trigger packets from Pico
 */

#ifndef VIO_SENSOR_SERIAL_READER_H
#define VIO_SENSOR_SERIAL_READER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <atomic>
#include <thread>
#include <vector>
#include <functional>

#include "vio_sensor/packet_defs.h"

namespace vio_sensor {

class SerialReader {
public:
    using ImuCallback = std::function<void(const sensor_msgs::msg::Imu&)>;
    using TriggerCallback = std::function<void(uint32_t timestamp_us, uint16_t frame_id)>;

    SerialReader(rclcpp::Node* node,
                 const std::string& port,
                 ImuCallback imu_cb,
                 TriggerCallback trigger_cb);
    ~SerialReader();

    // Start/stop serial reading thread
    void start();
    void stop();

    // Get latest trigger timestamp (ROS time)
    rclcpp::Time get_latest_trigger_time() const;

    // Check if time calibration is complete
    bool is_calibrated() const { return time_offset_initialized_; }

private:
    // Serial port operations
    bool open_serial();
    void serial_read_loop();

    // Packet processing
    void process_imu_packet(const ImuPacket& pkt);
    void process_trigger_packet(const TriggerPacket& pkt);

    // Time synchronization
    void initialize_time_offset(uint32_t pico_us);
    rclcpp::Time pico_to_ros_time(uint32_t pico_us) const;
    int64_t calculate_median(std::vector<int64_t>& samples);

    // CRC validation
    uint16_t calculate_crc16(const uint8_t* data, size_t len);

    // Node for logging and time
    rclcpp::Node* node_;

    // Serial port
    std::string port_;
    int serial_fd_;

    // Thread management
    std::thread serial_thread_;
    std::atomic<bool> running_;

    // Callbacks
    ImuCallback imu_callback_;
    TriggerCallback trigger_callback_;

    // Time synchronization
    std::atomic<bool> time_offset_initialized_;
    int64_t time_offset_ns_;
    std::vector<int64_t> offset_samples_;

    // Latest trigger timestamp (atomic for lock-free access)
    std::atomic<int64_t> latest_trigger_time_ns_;

    // Frame ID
    std::string imu_frame_id_;
};

}  // namespace vio_sensor

#endif  // VIO_SENSOR_SERIAL_READER_H
