/**
 * @file imu_sync_node_v2.cpp
 * @brief IMU + synchronization node (subscribes to camera images)
 *
 * This node:
 * - Reads IMU + trigger packets from Pico serial
 * - Subscribes to camera images from camera_only_node
 * - Re-timestamps camera images with trigger timestamps
 * - Publishes synced IMU + camera data
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "vio_sensor/serial_reader.h"

namespace vio_sensor {

class ImuSyncNode : public rclcpp::Node {
public:
    ImuSyncNode() : Node("imu_sync_node"), frame_count_(0)
    {
        // Parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        serial_port_ = this->get_parameter("serial_port").as_string();

        // Publishers with SensorDataQoS
        auto qos_imu = rclcpp::SensorDataQoS().keep_last(10);
        auto qos_camera = rclcpp::SensorDataQoS().keep_last(1);

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/vio/imu/data_raw", qos_imu);

        image_synced_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/vio/camera/image_synced", qos_camera);

        // Subscriber to raw camera images
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", qos_camera,
            std::bind(&ImuSyncNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "IMU Sync Node started");
        RCLCPP_INFO(this->get_logger(), "  Serial port: %s", serial_port_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Subscribing to: /camera/image_raw");
        RCLCPP_INFO(this->get_logger(), "  Publishing to: /vio/imu/data_raw, /vio/camera/image_synced");

        // Create serial reader
        serial_reader_ = std::make_unique<SerialReader>(
            this,
            serial_port_,
            std::bind(&ImuSyncNode::on_imu_data, this, std::placeholders::_1),
            std::bind(&ImuSyncNode::on_trigger, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Start serial reader
        serial_reader_->start();

        // Wait for calibration
        RCLCPP_INFO(this->get_logger(), "Waiting for time offset calibration...");
        while (rclcpp::ok() && !serial_reader_->is_calibrated()) {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        if (!rclcpp::ok()) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Time calibration complete - ready for syncing");

        last_stats_time_ = this->now();
        imu_count_ = 0;
    }

    ~ImuSyncNode()
    {
        if (serial_reader_) {
            serial_reader_->stop();
        }
    }

private:
    void on_imu_data(const sensor_msgs::msg::Imu& msg)
    {
        // Publish IMU message
        imu_pub_->publish(msg);
        imu_count_++;
    }

    void on_trigger(uint32_t timestamp_us, uint16_t frame_id)
    {
        // Trigger received - just for logging/debugging
        (void)timestamp_us;
        (void)frame_id;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Get latest trigger timestamp from serial reader
        rclcpp::Time trigger_time = serial_reader_->get_latest_trigger_time();

        // FAST PATH: Just count frames, no publish (to test if publish is the bottleneck)
        (void)trigger_time;  // Suppress unused warning
        (void)msg;           // Suppress unused warning

        frame_count_++;

        // Stats every second
        auto now = this->now();
        if ((now - last_stats_time_).seconds() >= 1.0) {
            double dt = (now - last_stats_time_).seconds();
            double imu_hz = imu_count_ / dt;
            double cam_hz = frame_count_ / dt;

            RCLCPP_INFO(this->get_logger(), "IMU: %.1f Hz | Camera: %.1f Hz",
                       imu_hz, cam_hz);

            imu_count_ = 0;
            frame_count_ = 0;
            last_stats_time_ = now;
        }
    }

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_synced_pub_;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Serial reader
    std::unique_ptr<SerialReader> serial_reader_;

    // Parameters
    std::string serial_port_;

    // Stats
    rclcpp::Time last_stats_time_;
    int imu_count_;
    int frame_count_;
};

}  // namespace vio_sensor

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vio_sensor::ImuSyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
