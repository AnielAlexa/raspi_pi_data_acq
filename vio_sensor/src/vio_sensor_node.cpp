/**
 * @file vio_sensor_node.cpp
 * @brief Implementation of VioSensorNode
 */

#include "vio_sensor/vio_sensor_node.h"
#include "vio_sensor/packet_defs.h"

namespace vio_sensor {

VioSensorNode::VioSensorNode()
    : Node("vio_sensor_node"),
      imu_count_(0),
      camera_count_(0)
{
    // Declare parameters
    this->declare_parameter<std::string>("serial_port", DEFAULT_SERIAL_PORT);
    this->declare_parameter<int>("camera_width", 640);
    this->declare_parameter<int>("camera_height", 480);

    serial_port_ = this->get_parameter("serial_port").as_string();
    camera_width_ = this->get_parameter("camera_width").as_int();
    camera_height_ = this->get_parameter("camera_height").as_int();

    // Create publishers with SensorDataQoS (BEST_EFFORT)
    auto qos_imu = rclcpp::SensorDataQoS().keep_last(10);
    auto qos_camera = rclcpp::SensorDataQoS().keep_last(1);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/vio/imu/data_raw", qos_imu);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/vio/camera/image_raw", qos_camera);

    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/vio/camera/camera_info", qos_camera);

    RCLCPP_INFO(this->get_logger(), "VIO Sensor Node started");
    RCLCPP_INFO(this->get_logger(), "  Serial port: %s", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Camera: %dx%d", camera_width_, camera_height_);
    RCLCPP_INFO(this->get_logger(), "  QoS: SensorDataQoS (BEST_EFFORT)");

    // Create serial reader with callbacks
    serial_reader_ = std::make_unique<SerialReader>(
        this,
        serial_port_,
        std::bind(&VioSensorNode::on_imu_data, this, std::placeholders::_1),
        std::bind(&VioSensorNode::on_trigger, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Create camera capture with callback
    camera_capture_ = std::make_unique<CameraCapture>(
        this,
        std::bind(&VioSensorNode::on_camera_frame, this,
                 std::placeholders::_1, std::placeholders::_2)
    );

    // Initialize camera
    if (!camera_capture_->init(camera_width_, camera_height_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
        rclcpp::shutdown();
        return;
    }

    // Start serial reader thread
    serial_reader_->start();

    // Wait for time calibration before starting camera
    RCLCPP_INFO(this->get_logger(), "Waiting for time offset calibration...");
    while (rclcpp::ok() && !serial_reader_->is_calibrated()) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (!rclcpp::ok()) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Time calibration complete, starting camera...");

    // Start camera
    camera_capture_->start();

    last_stats_time_ = this->now();
}

VioSensorNode::~VioSensorNode()
{
    if (camera_capture_) {
        camera_capture_->stop();
    }
    if (serial_reader_) {
        serial_reader_->stop();
    }
}

void VioSensorNode::on_imu_data(const sensor_msgs::msg::Imu& msg)
{
    // Publish IMU message
    imu_pub_->publish(msg);
    imu_count_++;
}

void VioSensorNode::on_trigger(uint32_t timestamp_us, uint16_t frame_id)
{
    // Trigger received - camera frame will arrive shortly
    // (Currently just logging, could be used for synchronization validation)
}

void VioSensorNode::on_camera_frame(const sensor_msgs::msg::Image& img,
                                   const sensor_msgs::msg::CameraInfo& info)
{
    // Get latest trigger timestamp from serial reader
    rclcpp::Time trigger_time = serial_reader_->get_latest_trigger_time();

    // Create messages with trigger timestamp
    auto img_msg = img;
    auto info_msg = info;

    img_msg.header.stamp = trigger_time;
    info_msg.header.stamp = trigger_time;

    // Publish
    image_pub_->publish(img_msg);
    camera_info_pub_->publish(info_msg);

    camera_count_++;

    // Stats logging every second
    auto now = this->now();
    if ((now - last_stats_time_).seconds() >= 1.0) {
        double dt = (now - last_stats_time_).seconds();
        double imu_hz = imu_count_ / dt;
        double cam_hz = camera_count_ / dt;

        RCLCPP_INFO(this->get_logger(), "IMU: %.1f Hz | Camera: %.1f Hz",
                   imu_hz, cam_hz);

        imu_count_ = 0;
        camera_count_ = 0;
        last_stats_time_ = now;
    }
}

}  // namespace vio_sensor

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vio_sensor::VioSensorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
