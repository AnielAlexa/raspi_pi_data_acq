/**
 * @file camera_capture.h
 * @brief libcamera-based camera capture with zero-copy access
 */

#ifndef VIO_SENSOR_CAMERA_CAPTURE_H
#define VIO_SENSOR_CAMERA_CAPTURE_H

#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <memory>
#include <functional>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <condition_variable>

namespace vio_sensor {

struct MappedPlane {
    void* addr = nullptr;
    size_t length = 0;
};

struct MappedBuffer {
    std::vector<MappedPlane> planes;
};

class CameraCapture {
public:
    using FrameCallback = std::function<void(const sensor_msgs::msg::Image&,
                                             const sensor_msgs::msg::CameraInfo&)>;

    CameraCapture(rclcpp::Node* node, FrameCallback callback);
    ~CameraCapture();

    // Initialize and start camera
    bool init(int width = 640, int height = 480);
    void start();
    void stop();

    // Process camera events (call from main loop)
    void process_events();

private:
    void request_completed(libcamera::Request* req);

    rclcpp::Node* node_;
    FrameCallback frame_callback_;

    // libcamera objects
    std::unique_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream* stream_;

    // Memory mapped buffers
    std::unordered_map<libcamera::FrameBuffer*, MappedBuffer> mappings_;

    // Request pool
    std::vector<std::unique_ptr<libcamera::Request>> requests_;

    // Camera configuration
    int width_;
    int height_;
    std::string camera_frame_id_;

    // Running state
    bool running_;
};

}  // namespace vio_sensor

#endif  // VIO_SENSOR_CAMERA_CAPTURE_H
