#include "cam_imu_synced/imu_trigger_component.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <iostream>

namespace cam_imu_synced
{

ImuTriggerComponent::ImuTriggerComponent(const rclcpp::NodeOptions & options)
: Node("imu_trigger_component", options),
  serial_fd_(-1),
  current_offset_ns_(0),
  offset_window_size_(50),
  offset_update_interval_(10),
  offset_sample_count_(0),
  running_(true)
{
  // Declare parameters
  serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  imu_frame_id_ = this->declare_parameter<std::string>("imu_frame_id", "imu_link");
  trigger_frame_id_ = this->declare_parameter<std::string>("trigger_frame_id", "camera_link");

  // Create publishers with intra-process communication
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "/imu/data_raw",
    rclcpp::QoS(10)
  );

  trigger_time_pub_ = this->create_publisher<std_msgs::msg::Header>(
    "/sync/trigger_time_ros",
    rclcpp::QoS(10)
  );

  trigger_id_pub_ = this->create_publisher<std_msgs::msg::UInt16>(
    "/sync/trigger_frame_id",
    rclcpp::QoS(10)
  );

  // Open serial port
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
    throw std::runtime_error("Failed to open serial port");
  }

  // Configure serial port (921600 8N1 - matching Pico)
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get serial attributes");
    close(serial_fd_);
    throw std::runtime_error("Failed to configure serial port");
  }

  cfsetospeed(&tty, B921600);
  cfsetispeed(&tty, B921600);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
  tty.c_iflag &= ~IGNBRK;                      // disable break processing
  tty.c_lflag = 0;                             // no signaling chars, no echo, no canonical processing
  tty.c_oflag = 0;                             // no remapping, no delays
  tty.c_cc[VMIN] = 0;                          // read doesn't block
  tty.c_cc[VTIME] = 5;                         // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);      // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);             // ignore modem controls, enable reading
  tty.c_cflag &= ~(PARENB | PARODD);           // shut off parity
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set serial attributes");
    close(serial_fd_);
    throw std::runtime_error("Failed to configure serial port");
  }

  RCLCPP_INFO(this->get_logger(), "Opened serial port: %s", serial_port_.c_str());

  // Start serial reading thread
  serial_thread_ = std::thread(&ImuTriggerComponent::serialReadThread, this);

  RCLCPP_INFO(this->get_logger(), "ImuTriggerComponent initialized");
}

ImuTriggerComponent::~ImuTriggerComponent()
{
  running_ = false;
  if (serial_thread_.joinable()) {
    serial_thread_.join();
  }
  if (serial_fd_ >= 0) {
    close(serial_fd_);
  }
  RCLCPP_INFO(this->get_logger(), "ImuTriggerComponent shut down");
}

void ImuTriggerComponent::serialReadThread()
{
  std::vector<uint8_t> buffer;
  buffer.reserve(512);

  while (running_) {
    uint8_t read_buf[128];
    ssize_t n = read(serial_fd_, read_buf, sizeof(read_buf));

    if (n > 0) {
      buffer.insert(buffer.end(), read_buf, read_buf + n);

      // Process complete packets by looking for headers
      size_t i = 0;
      while (i + 2 <= buffer.size()) {  // Need at least 2 bytes for header
        uint16_t header;
        memcpy(&header, &buffer[i], 2);

        // IMU packet
        if (header == IMU_HEADER && i + sizeof(ImuPacket) <= buffer.size()) {
          ImuPacket pkt;
          memcpy(&pkt, &buffer[i], sizeof(ImuPacket));

          if (validateCRC((uint8_t*)&pkt, sizeof(ImuPacket))) {
            parseSerialData((uint8_t*)&pkt, sizeof(ImuPacket));
            buffer.erase(buffer.begin(), buffer.begin() + i + sizeof(ImuPacket));
            i = 0;
          } else {
            buffer.erase(buffer.begin(), buffer.begin() + i + 1);
            i = 0;
          }
        }
        // Trigger packet
        else if (header == TRIGGER_HEADER && i + sizeof(TriggerPacket) <= buffer.size()) {
          TriggerPacket pkt;
          memcpy(&pkt, &buffer[i], sizeof(TriggerPacket));

          if (validateCRC((uint8_t*)&pkt, sizeof(TriggerPacket))) {
            parseSerialData((uint8_t*)&pkt, sizeof(TriggerPacket));
            buffer.erase(buffer.begin(), buffer.begin() + i + sizeof(TriggerPacket));
            i = 0;
          } else {
            buffer.erase(buffer.begin(), buffer.begin() + i + 1);
            i = 0;
          }
        }
        // No valid header found, advance
        else {
          i++;
        }
      }
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", strerror(errno));
      break;
    }

    // Prevent buffer from growing indefinitely
    if (buffer.size() > 1024) {
      RCLCPP_WARN(this->get_logger(), "Serial buffer overflow, clearing");
      buffer.erase(buffer.begin(), buffer.end() - 256);
    }
  }
}

void ImuTriggerComponent::parseSerialData(const uint8_t * data, size_t len)
{
  rclcpp::Time now = this->now();

  if (len == sizeof(ImuPacket)) {
    // Parse IMU packet using struct
    ImuPacket pkt;
    memcpy(&pkt, data, sizeof(ImuPacket));

    if (pkt.header != IMU_HEADER) return;

    // Update time offset
    updateTimeOffset(pkt.timestamp_us, now);

    // Publish IMU message
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = picoUsToRosTime(pkt.timestamp_us);
    imu_msg->header.frame_id = imu_frame_id_;

    imu_msg->linear_acceleration.x = pkt.ax;
    imu_msg->linear_acceleration.y = pkt.ay;
    imu_msg->linear_acceleration.z = pkt.az;

    imu_msg->angular_velocity.x = pkt.gx;
    imu_msg->angular_velocity.y = pkt.gy;
    imu_msg->angular_velocity.z = pkt.gz;

    // Set covariance to unknown
    imu_msg->orientation_covariance[0] = -1.0;

    imu_pub_->publish(std::move(imu_msg));

  } else if (len == sizeof(TriggerPacket)) {
    // Parse trigger packet using struct
    TriggerPacket pkt;
    memcpy(&pkt, data, sizeof(TriggerPacket));

    if (pkt.header != TRIGGER_HEADER) return;

    // Convert to ROS time
    rclcpp::Time trigger_time = picoUsToRosTime(pkt.timestamp_us);

    // Publish trigger timestamp
    auto header_msg = std::make_unique<std_msgs::msg::Header>();
    header_msg->stamp = trigger_time;
    header_msg->frame_id = trigger_frame_id_;
    trigger_time_pub_->publish(std::move(header_msg));

    // Publish frame ID
    auto id_msg = std::make_unique<std_msgs::msg::UInt16>();
    id_msg->data = pkt.frame_id;
    trigger_id_pub_->publish(std::move(id_msg));

    RCLCPP_DEBUG(this->get_logger(), "Trigger: frame_id=%u, time=%f",
                 pkt.frame_id, trigger_time.seconds());
  }
}

void ImuTriggerComponent::updateTimeOffset(uint32_t pico_us, const rclcpp::Time & ros_time)
{
  // Compute offset: ros_time_ns - pico_us_ns
  int64_t pico_ns = static_cast<int64_t>(pico_us) * 1000LL;
  int64_t ros_ns = ros_time.nanoseconds();
  int64_t offset = ros_ns - pico_ns;

  std::lock_guard<std::mutex> lock(offset_mutex_);

  offset_samples_.push_back(offset);
  offset_sample_count_++;

  // Keep window size limited
  if (offset_samples_.size() > offset_window_size_) {
    offset_samples_.pop_front();
  }

  // Update offset using median every N samples
  if (offset_sample_count_ >= offset_update_interval_) {
    offset_sample_count_ = 0;

    // Compute median
    std::vector<int64_t> sorted_samples(offset_samples_.begin(), offset_samples_.end());
    std::sort(sorted_samples.begin(), sorted_samples.end());
    size_t mid = sorted_samples.size() / 2;

    if (sorted_samples.size() % 2 == 0) {
      current_offset_ns_ = (sorted_samples[mid - 1] + sorted_samples[mid]) / 2;
    } else {
      current_offset_ns_ = sorted_samples[mid];
    }

    RCLCPP_DEBUG(this->get_logger(), "Time offset updated: %ld ns (%.3f ms)",
                 current_offset_ns_, current_offset_ns_ / 1e6);
  }

  // Use first sample as initial offset
  if (current_offset_ns_ == 0) {
    current_offset_ns_ = offset;
  }
}

rclcpp::Time ImuTriggerComponent::picoUsToRosTime(uint32_t pico_us)
{
  std::lock_guard<std::mutex> lock(offset_mutex_);
  int64_t pico_ns = static_cast<int64_t>(pico_us) * 1000LL;
  int64_t ros_ns = pico_ns + current_offset_ns_;
  return rclcpp::Time(ros_ns);
}

bool ImuTriggerComponent::validateCRC(const uint8_t * data, size_t len)
{
  // CRC16 matching Pico implementation (Modbus CRC16)
  if (len < 2) return false;

  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len - 2; i++) {  // len-2 because CRC is last 2 bytes
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  // Extract CRC from packet (little-endian uint16_t)
  uint16_t packet_crc;
  memcpy(&packet_crc, data + len - 2, 2);

  return crc == packet_crc;
}

}  // namespace cam_imu_synced

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cam_imu_synced::ImuTriggerComponent)
