/**
 * @file camera_display_node.h
 * @brief ROS2 node for synchronized camera and IMU capture from Pico
 */

#ifndef CAMERA_DISPLAY_NODE_CAMERA_DISPLAY_NODE_H
#define CAMERA_DISPLAY_NODE_CAMERA_DISPLAY_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>

#include "camera_display_node/serial_sync.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

struct MappedPlane {
    void *addr = nullptr;
    size_t length = 0;
};

struct MappedBuffer {
    std::vector<MappedPlane> planes;
};

class CameraDisplayNode : public rclcpp::Node {
public:
    explicit CameraDisplayNode();
    virtual ~CameraDisplayNode();

private:
    // Initialization
    bool initCamera();
    void initPicoSync(const std::string& serial_port);

    // Callbacks
    void onImuPacket(uint32_t timestamp_us, float ax, float ay, float az,
                     float gx, float gy, float gz);
    void onTriggerPacket(uint32_t timestamp_us, uint16_t frame_id);
    void onRequestCompleted(libcamera::Request *req);

    // Utilities
    rclcpp::Time getFrameTimestamp(uint16_t frame_id);
    void logSyncStats();

    // ============================================================
    // Publishers
    // ============================================================
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // ============================================================
    // Camera Configuration
    // ============================================================
    int camera_index_;
    int width_;
    int height_;

    // ============================================================
    // libcamera Objects
    // ============================================================
    std::unique_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::unordered_map<libcamera::FrameBuffer *, MappedBuffer> mappings_;

    libcamera::Stream *stream_;
    unsigned int stride_;

    // ============================================================
    // FPS Tracking
    // ============================================================
    std::chrono::steady_clock::time_point last_frame_time_;
    std::chrono::steady_clock::time_point last_log_time_;
    double smoothed_fps_;
    int frames_since_log_;

    // ============================================================
    // Pico Synchronization
    // ============================================================
    bool enable_pico_sync_;
    std::unique_ptr<camera_display_node::SerialSync> serial_sync_;

    std::mutex trigger_map_mutex_;
    std::unordered_map<uint16_t, rclcpp::Time> trigger_map_;  // frame_id → timestamp
    rclcpp::Time latest_trigger_time_;                         // Latest trigger timestamp for sync
    size_t trigger_map_max_size_;
    std::atomic<uint16_t> expected_frame_id_;

    // ============================================================
    // Synchronization Statistics
    // ============================================================
    std::atomic<uint32_t> frames_received_;
    std::atomic<uint32_t> frames_matched_;
    std::atomic<uint32_t> frame_drops_;
};

#endif  // CAMERA_DISPLAY_NODE_CAMERA_DISPLAY_NODE_H
