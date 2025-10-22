/**
 * @file serial_reader.cpp
 * @brief Implementation of SerialReader class
 */

#include "vio_sensor/serial_reader.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>

namespace vio_sensor {

SerialReader::SerialReader(rclcpp::Node* node,
                           const std::string& port,
                           ImuCallback imu_cb,
                           TriggerCallback trigger_cb)
    : node_(node),
      port_(port),
      serial_fd_(-1),
      running_(false),
      imu_callback_(imu_cb),
      trigger_callback_(trigger_cb),
      time_offset_initialized_(false),
      time_offset_ns_(0),
      latest_trigger_time_ns_(0),
      imu_frame_id_("imu")
{
    offset_samples_.reserve(TIME_OFFSET_SAMPLES);
}

SerialReader::~SerialReader()
{
    stop();
    if (serial_fd_ >= 0) {
        close(serial_fd_);
    }
}

void SerialReader::start()
{
    if (!open_serial()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open serial port: %s", port_.c_str());
        return;
    }

    running_ = true;
    serial_thread_ = std::thread(&SerialReader::serial_read_loop, this);
    RCLCPP_INFO(node_->get_logger(), "Serial reader started on %s @ %d baud",
                port_.c_str(), SERIAL_BAUDRATE);
}

void SerialReader::stop()
{
    running_ = false;
    if (serial_thread_.joinable()) {
        serial_thread_.join();
    }
}

bool SerialReader::open_serial()
{
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd_ < 0) {
        return false;
    }

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        return false;
    }

    // Set baud rate
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    // 8N1 mode, no flow control
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        return false;
    }

    return true;
}

uint16_t SerialReader::calculate_crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

int64_t SerialReader::calculate_median(std::vector<int64_t>& samples)
{
    if (samples.empty()) return 0;

    std::vector<int64_t> sorted = samples;
    std::sort(sorted.begin(), sorted.end());

    size_t mid = sorted.size() / 2;
    if (sorted.size() % 2 == 0) {
        return (sorted[mid - 1] + sorted[mid]) / 2;
    } else {
        return sorted[mid];
    }
}

rclcpp::Time SerialReader::pico_to_ros_time(uint32_t pico_us) const
{
    int64_t pico_ns = static_cast<int64_t>(pico_us) * 1000LL;
    int64_t ros_ns = pico_ns + time_offset_ns_;
    return rclcpp::Time(ros_ns);
}

void SerialReader::initialize_time_offset(uint32_t pico_us)
{
    if (time_offset_initialized_.load()) {
        return;
    }

    // Calculate offset sample
    auto ros_now = node_->now();
    int64_t ros_ns = ros_now.nanoseconds();
    int64_t pico_ns = static_cast<int64_t>(pico_us) * 1000LL;
    int64_t offset_sample = ros_ns - pico_ns;

    offset_samples_.push_back(offset_sample);

    // Log progress every 20 samples
    if (offset_samples_.size() % 20 == 0) {
        RCLCPP_INFO(node_->get_logger(), "Calibrating time offset: %zu/%zu",
                   offset_samples_.size(), TIME_OFFSET_SAMPLES);
    }

    // Once we have enough samples, calculate median
    if (offset_samples_.size() >= TIME_OFFSET_SAMPLES) {
        time_offset_ns_ = calculate_median(offset_samples_);

        // Calculate statistics
        int64_t min_offset = *std::min_element(offset_samples_.begin(), offset_samples_.end());
        int64_t max_offset = *std::max_element(offset_samples_.begin(), offset_samples_.end());
        int64_t range = max_offset - min_offset;

        RCLCPP_INFO(node_->get_logger(), "Time offset calibration complete:");
        RCLCPP_INFO(node_->get_logger(), "  Median offset: %.3f ms", time_offset_ns_ / 1e6);
        RCLCPP_INFO(node_->get_logger(), "  Sample range: %.3f ms", range / 1e6);

        // Set flag and clear samples
        time_offset_initialized_.store(true);
        offset_samples_.clear();
        offset_samples_.shrink_to_fit();
    }
}

void SerialReader::process_imu_packet(const ImuPacket& pkt)
{
    // Verify CRC
    uint16_t calc_crc = calculate_crc16((uint8_t*)&pkt, sizeof(ImuPacket) - 2);
    if (calc_crc != pkt.crc16) {
        return;  // Silently drop bad packets
    }

    // Initialize time offset
    initialize_time_offset(pkt.timestamp_us);

    // Only publish if calibrated
    if (!time_offset_initialized_.load()) {
        return;
    }

    // Create IMU message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = pico_to_ros_time(pkt.timestamp_us);
    imu_msg.header.frame_id = imu_frame_id_;

    // Linear acceleration
    imu_msg.linear_acceleration.x = pkt.ax;
    imu_msg.linear_acceleration.y = pkt.ay;
    imu_msg.linear_acceleration.z = pkt.az;

    // Angular velocity
    imu_msg.angular_velocity.x = pkt.gx;
    imu_msg.angular_velocity.y = pkt.gy;
    imu_msg.angular_velocity.z = pkt.gz;

    // Covariances (unknown orientation, known acc/gyro)
    imu_msg.orientation_covariance[0] = -1.0;
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[0] = 0.01;

    // Publish via callback
    imu_callback_(imu_msg);
}

void SerialReader::process_trigger_packet(const TriggerPacket& pkt)
{
    // Verify CRC
    uint16_t calc_crc = calculate_crc16((uint8_t*)&pkt, sizeof(TriggerPacket) - 2);
    if (calc_crc != pkt.crc16) {
        return;
    }

    // Initialize time offset
    initialize_time_offset(pkt.timestamp_us);

    // Only process if calibrated
    if (!time_offset_initialized_.load()) {
        return;
    }

    // Store latest trigger time (atomic)
    rclcpp::Time trigger_time = pico_to_ros_time(pkt.timestamp_us);
    latest_trigger_time_ns_.store(trigger_time.nanoseconds());

    // Call trigger callback
    trigger_callback_(pkt.timestamp_us, pkt.frame_id);
}

void SerialReader::serial_read_loop()
{
    std::vector<uint8_t> buffer;
    buffer.reserve(1024);

    uint32_t imu_count = 0;
    uint32_t trigger_count = 0;
    auto last_stats = node_->now();

    while (running_.load()) {
        uint8_t byte;
        int n = read(serial_fd_, &byte, 1);

        if (n > 0) {
            buffer.push_back(byte);

            // Scan for packet headers
            for (size_t i = 0; i + 1 < buffer.size(); i++) {
                uint16_t header = buffer[i] | (buffer[i+1] << 8);

                // IMU packet
                if (header == IMU_HEADER && i + sizeof(ImuPacket) <= buffer.size()) {
                    ImuPacket pkt;
                    std::memcpy(&pkt, &buffer[i], sizeof(ImuPacket));

                    uint16_t calc_crc = calculate_crc16((uint8_t*)&pkt, sizeof(ImuPacket) - 2);
                    if (calc_crc == pkt.crc16) {
                        process_imu_packet(pkt);
                        imu_count++;
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
                    std::memcpy(&pkt, &buffer[i], sizeof(TriggerPacket));

                    uint16_t calc_crc = calculate_crc16((uint8_t*)&pkt, sizeof(TriggerPacket) - 2);
                    if (calc_crc == pkt.crc16) {
                        process_trigger_packet(pkt);
                        trigger_count++;
                        buffer.erase(buffer.begin(), buffer.begin() + i + sizeof(TriggerPacket));
                        i = 0;
                    } else {
                        buffer.erase(buffer.begin(), buffer.begin() + i + 1);
                        i = 0;
                    }
                }
            }

            // Prevent buffer overflow
            if (buffer.size() > 512) {
                buffer.erase(buffer.begin(), buffer.end() - 256);
            }

            // Stats every second (only after calibration)
            if (time_offset_initialized_.load()) {
                auto now = node_->now();
                if ((now - last_stats).seconds() >= 1.0) {
                    RCLCPP_INFO(node_->get_logger(), "IMU: %d Hz | Triggers: %d Hz",
                               imu_count, trigger_count);
                    imu_count = 0;
                    trigger_count = 0;
                    last_stats = now;
                }
            }
        }
    }
}

rclcpp::Time SerialReader::get_latest_trigger_time() const
{
    return rclcpp::Time(latest_trigger_time_ns_.load());
}

}  // namespace vio_sensor
