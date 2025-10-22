/**
 * @file vio_sensor_node.h
 * @brief Main VIO sensor node combining camera and IMU
 */

#ifndef VIO_SENSOR_VIO_SENSOR_NODE_H
#define VIO_SENSOR_VIO_SENSOR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <memory>

#include "vio_sensor/serial_reader.h"
#include "vio_sensor/camera_capture.h"

namespace vio_sensor {

class VioSensorNode : public rclcpp::Node {
public:
    VioSensorNode();
    ~VioSensorNode();

private:
    // Callbacks
    void on_imu_data(const sensor_msgs::msg::Imu& msg);
    void on_trigger(uint32_t timestamp_us, uint16_t frame_id);
    void on_camera_frame(const sensor_msgs::msg::Image& img,
                        const sensor_msgs::msg::CameraInfo& info);

    // Publishers with SensorDataQoS
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    // Sensor components
    std::unique_ptr<SerialReader> serial_reader_;
    std::unique_ptr<CameraCapture> camera_capture_;

    // Parameters
    std::string serial_port_;
    int camera_width_;
    int camera_height_;

    // Statistics
    rclcpp::Time last_stats_time_;
    int imu_count_;
    int camera_count_;
};

}  // namespace vio_sensor

#endif  // VIO_SENSOR_VIO_SENSOR_NODE_H
