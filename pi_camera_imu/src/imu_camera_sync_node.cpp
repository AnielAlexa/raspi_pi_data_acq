/**
 * @file imu_camera_sync_node.cpp
 * @brief Combined IMU + Camera trigger synchronization node
 *
 * This node:
 * - Reads serial data from Pico (IMU @ 400Hz + trigger @ 20Hz)
 * - Converts Pico timestamps to ROS time domain using median-based calibration
 * - Publishes synchronized data with corrected ROS timestamps
 *
 * Published Topics:
 *   /imu/data_raw - Raw IMU data @ 400Hz (ROS timestamps)
 *   /sync/trigger_timestamp - Camera trigger timestamp in Pico microseconds
 *   /sync/trigger_frame_id - Frame counter from Pico
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <deque>
#include <thread>
#include <atomic>
#include <mutex>
#include <algorithm>

// Packet structures from Pico code
#pragma pack(push, 1)
struct ImuPacket {
    uint16_t header;        // 0xAA55
    uint32_t timestamp_us;  // Microseconds
    float ax, ay, az;       // m/s²
    float gx, gy, gz;       // rad/s
    uint16_t crc16;
};

struct TriggerPacket {
    uint16_t header;        // 0xBB66
    uint32_t timestamp_us;  // Microseconds when trigger fired
    uint16_t frame_id;      // Frame counter
    uint16_t reserved;
    uint16_t crc16;
};
#pragma pack(pop)

class ImuCameraSyncNode : public rclcpp::Node
{
public:
    ImuCameraSyncNode() : Node("imu_camera_sync_node"),
                          serial_fd_(-1),
                          running_(false),
                          time_offset_ns_(0),
                          time_offset_initialized_(false)
    {
        // Parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("serial_baudrate", 921600);
        this->declare_parameter<std::string>("imu_frame_id", "imu");

        serial_port_ = this->get_parameter("serial_port").as_string();
        baudrate_ = this->get_parameter("serial_baudrate").as_int();
        imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();

        // Publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 100);
        trigger_timestamp_pub_ = this->create_publisher<std_msgs::msg::UInt32>("/sync/trigger_timestamp", 10);
        trigger_frame_id_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/sync/trigger_frame_id", 10);

        // Open serial port
        if (!open_serial()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
            rclcpp::shutdown();
            return;
        }

        // Start serial reading thread
        running_ = true;
        serial_thread_ = std::thread(&ImuCameraSyncNode::serial_read_loop, this);

        RCLCPP_INFO(this->get_logger(), "IMU Camera Sync Node started");
        RCLCPP_INFO(this->get_logger(), "Serial port: %s @ %d baud", serial_port_.c_str(), baudrate_);
        RCLCPP_INFO(this->get_logger(), "Waiting for Pico data...");
    }

    ~ImuCameraSyncNode()
    {
        running_ = false;
        if (serial_thread_.joinable()) {
            serial_thread_.join();
        }
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    bool open_serial()
    {
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open %s: %s",
                        serial_port_.c_str(), strerror(errno));
            return false;
        }

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr error");
            return false;
        }

        // Set baud rate
        speed_t speed = B921600;
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        // 8N1 mode
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
            RCLCPP_ERROR(this->get_logger(), "tcsetattr error");
            return false;
        }

        return true;
    }

    uint16_t calculate_crc16(const uint8_t* data, size_t len)
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

    int64_t calculate_median(std::vector<int64_t>& samples)
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

    rclcpp::Time pico_to_ros_time(uint32_t pico_us)
    {
        // Convert Pico microseconds to ROS time with offset
        int64_t pico_ns = static_cast<int64_t>(pico_us) * 1000LL;
        int64_t ros_ns = pico_ns + time_offset_ns_;
        return rclcpp::Time(ros_ns);
    }

    void initialize_time_offset(uint32_t pico_us)
    {
        if (!time_offset_initialized_) {
            // Calculate offset sample: ros_time = pico_time + offset
            auto ros_now = this->now();
            int64_t ros_ns = ros_now.nanoseconds();
            int64_t pico_ns = static_cast<int64_t>(pico_us) * 1000LL;
            int64_t offset_sample = ros_ns - pico_ns;

            offset_samples_.push_back(offset_sample);

            // Log progress every 20 samples
            if (offset_samples_.size() % 20 == 0) {
                RCLCPP_INFO(this->get_logger(), "Collecting time offset samples: %zu/%zu",
                           offset_samples_.size(), NUM_OFFSET_SAMPLES);
            }

            // Once we have enough samples, calculate median
            if (offset_samples_.size() >= NUM_OFFSET_SAMPLES) {
                time_offset_ns_ = calculate_median(offset_samples_);
                time_offset_initialized_ = true;

                // Calculate statistics for logging
                int64_t min_offset = *std::min_element(offset_samples_.begin(), offset_samples_.end());
                int64_t max_offset = *std::max_element(offset_samples_.begin(), offset_samples_.end());
                int64_t range = max_offset - min_offset;

                RCLCPP_INFO(this->get_logger(),
                           "✓ Time offset calibration complete:");
                RCLCPP_INFO(this->get_logger(),
                           "  Samples: %zu", offset_samples_.size());
                RCLCPP_INFO(this->get_logger(),
                           "  Median offset: %ld ns (%.3f ms)",
                           time_offset_ns_, time_offset_ns_ / 1e6);
                RCLCPP_INFO(this->get_logger(),
                           "  Range: %ld ns (%.3f ms)",
                           range, range / 1e6);

                // Clear samples to free memory
                offset_samples_.clear();
                offset_samples_.shrink_to_fit();
            }
        }
    }

    void process_imu_packet(const ImuPacket& pkt)
    {
        // Verify CRC
        uint16_t calc_crc = calculate_crc16((uint8_t*)&pkt, sizeof(ImuPacket) - 2);
        if (calc_crc != pkt.crc16) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "IMU CRC error");
            return;
        }

        // Initialize time offset on first packet
        initialize_time_offset(pkt.timestamp_us);

        // Create IMU message
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = pico_to_ros_time(pkt.timestamp_us);
        imu_msg.header.frame_id = imu_frame_id_;

        // Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = pkt.ax;
        imu_msg.linear_acceleration.y = pkt.ay;
        imu_msg.linear_acceleration.z = pkt.az;

        // Angular velocity (rad/s)
        imu_msg.angular_velocity.x = pkt.gx;
        imu_msg.angular_velocity.y = pkt.gy;
        imu_msg.angular_velocity.z = pkt.gz;

        // Covariances (unknown, set to -1)
        imu_msg.orientation_covariance[0] = -1.0;
        imu_msg.linear_acceleration_covariance[0] = 0.01;
        imu_msg.angular_velocity_covariance[0] = 0.01;

        imu_pub_->publish(imu_msg);
    }

    void process_trigger_packet(const TriggerPacket& pkt)
    {
        // Verify CRC
        uint16_t calc_crc = calculate_crc16((uint8_t*)&pkt, sizeof(TriggerPacket) - 2);
        if (calc_crc != pkt.crc16) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Trigger CRC error");
            return;
        }

        // Initialize time offset if needed
        initialize_time_offset(pkt.timestamp_us);

        // Store latest trigger info
        {
            std::lock_guard<std::mutex> lock(trigger_mutex_);
            latest_trigger_timestamp_us_ = pkt.timestamp_us;
            latest_trigger_frame_id_ = pkt.frame_id;
            latest_trigger_ros_time_ = pico_to_ros_time(pkt.timestamp_us);
        }

        // Publish trigger info for debugging
        auto ts_msg = std_msgs::msg::UInt32();
        ts_msg.data = pkt.timestamp_us;
        trigger_timestamp_pub_->publish(ts_msg);

        auto fid_msg = std_msgs::msg::UInt16();
        fid_msg.data = pkt.frame_id;
        trigger_frame_id_pub_->publish(fid_msg);

        RCLCPP_DEBUG(this->get_logger(), "Trigger: frame_id=%d, timestamp=%u us",
                    pkt.frame_id, pkt.timestamp_us);
    }

    void serial_read_loop()
    {
        std::vector<uint8_t> buffer;
        buffer.reserve(1024);

        uint32_t imu_count = 0;
        uint32_t trigger_count = 0;
        uint32_t crc_errors = 0;
        auto last_stats = this->now();

        while (running_ && rclcpp::ok()) {
            uint8_t byte;
            int n = read(serial_fd_, &byte, 1);

            if (n > 0) {
                buffer.push_back(byte);

                // Look for packet headers - scan through entire buffer
                for (size_t i = 0; i + 1 < buffer.size(); i++) {
                    // Read header as little-endian
                    uint16_t header = buffer[i] | (buffer[i+1] << 8);

                    // IMU packet (32 bytes)
                    if (header == 0xAA55 && i + sizeof(ImuPacket) <= buffer.size()) {
                        ImuPacket pkt;
                        memcpy(&pkt, &buffer[i], sizeof(ImuPacket));

                        // Verify CRC
                        uint16_t calc_crc = calculate_crc16((uint8_t*)&pkt, sizeof(ImuPacket) - 2);
                        if (calc_crc == pkt.crc16) {
                            process_imu_packet(pkt);
                            imu_count++;

                            // Remove processed packet
                            buffer.erase(buffer.begin(), buffer.begin() + i + sizeof(ImuPacket));
                            i = 0;  // Restart search
                        } else {
                            crc_errors++;
                            // Move past bad header
                            buffer.erase(buffer.begin(), buffer.begin() + i + 1);
                            i = 0;
                        }
                    }
                    // Trigger packet (12 bytes)
                    else if (header == 0xBB66 && i + sizeof(TriggerPacket) <= buffer.size()) {
                        TriggerPacket pkt;
                        memcpy(&pkt, &buffer[i], sizeof(TriggerPacket));

                        // Verify CRC
                        uint16_t calc_crc = calculate_crc16((uint8_t*)&pkt, sizeof(TriggerPacket) - 2);
                        if (calc_crc == pkt.crc16) {
                            process_trigger_packet(pkt);
                            trigger_count++;

                            // Remove processed packet
                            buffer.erase(buffer.begin(), buffer.begin() + i + sizeof(TriggerPacket));
                            i = 0;  // Restart search
                        } else {
                            crc_errors++;
                            // Move past bad header
                            buffer.erase(buffer.begin(), buffer.begin() + i + 1);
                            i = 0;
                        }
                    }
                }

                // Prevent buffer overflow - keep only last 256 bytes if too large
                if (buffer.size() > 512) {
                    buffer.erase(buffer.begin(), buffer.end() - 256);
                }

                // Stats every second
                auto now = this->now();
                if ((now - last_stats).seconds() >= 1.0) {
                    RCLCPP_INFO(this->get_logger(), "IMU: %d Hz, Triggers: %d Hz, CRC errors: %d",
                               imu_count, trigger_count, crc_errors);
                    imu_count = 0;
                    trigger_count = 0;
                    crc_errors = 0;
                    last_stats = now;
                }
            }
        }
    }


    // Serial
    std::string serial_port_;
    int baudrate_;
    int serial_fd_;
    std::thread serial_thread_;
    std::atomic<bool> running_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr trigger_timestamp_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr trigger_frame_id_pub_;

    // Time sync
    int64_t time_offset_ns_;
    bool time_offset_initialized_;
    std::vector<int64_t> offset_samples_;
    static constexpr size_t NUM_OFFSET_SAMPLES = 100;

    // Trigger info
    std::mutex trigger_mutex_;
    uint32_t latest_trigger_timestamp_us_;
    uint16_t latest_trigger_frame_id_;
    rclcpp::Time latest_trigger_ros_time_;

    // Frame IDs
    std::string imu_frame_id_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuCameraSyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
