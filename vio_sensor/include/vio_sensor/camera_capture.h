/**
 * @file camera_capture.h
 * @brief libcamera-based camera capture with zero-copy access (reusable messages)
 */

#ifndef VIO_SENSOR_CAMERA_CAPTURE_H
#define VIO_SENSOR_CAMERA_CAPTURE_H

#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

namespace vio_sensor {

struct MappedPlane { void* addr=nullptr; size_t length=0; };
struct MappedBuffer { std::vector<MappedPlane> planes; };

class CameraCapture {
public:
    using FrameCallback = std::function<void(
        sensor_msgs::msg::Image&, sensor_msgs::msg::CameraInfo&)>;

    CameraCapture(rclcpp::Node* node, FrameCallback cb);
    ~CameraCapture();

    bool init(int width=640, int height=480);
    void start();
    void stop();
    void process_events();

private:
    void request_completed(libcamera::Request* req);

    rclcpp::Node* node_;
    FrameCallback frame_callback_;

    std::unique_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream* stream_{nullptr};

    std::unordered_map<libcamera::FrameBuffer*, MappedBuffer> mappings_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;

    int width_{640};
    int height_{480};
    std::string camera_frame_id_{"camera"};
    bool running_{false};

    // Reusable preallocated messages
    sensor_msgs::msg::Image img_msg_;
    sensor_msgs::msg::CameraInfo info_msg_;
};

} // namespace vio_sensor

#endif
