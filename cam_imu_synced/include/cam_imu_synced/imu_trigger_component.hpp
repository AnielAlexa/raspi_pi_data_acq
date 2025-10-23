#ifndef CAM_IMU_SYNCED__IMU_TRIGGER_COMPONENT_HPP_
#define CAM_IMU_SYNCED__IMU_TRIGGER_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace cam_imu_synced
{

// Pico packet structures (must match firmware exactly)
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

class ImuTriggerComponent : public rclcpp::Node
{
public:
  explicit ImuTriggerComponent(const rclcpp::NodeOptions & options);
  virtual ~ImuTriggerComponent();

private:
  // Serial reading thread
  void serialReadThread();

  // Parse incoming serial data
  void parseSerialData(const uint8_t * data, size_t len);

  // Compute ROS time offset from Pico microsecond timestamps
  void updateTimeOffset(uint32_t pico_us, const rclcpp::Time & ros_time);
  rclcpp::Time picoUsToRosTime(uint32_t pico_us);

  // CRC validation
  bool validateCRC(const uint8_t * data, size_t len);

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr trigger_time_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr trigger_id_pub_;

  // Serial port parameters
  std::string serial_port_;
  int serial_fd_;

  // Time offset estimation (median filter)
  std::mutex offset_mutex_;
  std::deque<int64_t> offset_samples_;  // nanoseconds
  int64_t current_offset_ns_;
  size_t offset_window_size_;
  size_t offset_update_interval_;
  size_t offset_sample_count_;

  // Serial reading thread
  std::thread serial_thread_;
  std::atomic<bool> running_;

  // IMU frame ID
  std::string imu_frame_id_;
  std::string trigger_frame_id_;
};

}  // namespace cam_imu_synced

#endif  // CAM_IMU_SYNCED__IMU_TRIGGER_COMPONENT_HPP_
