/**
 * @file camera_display_node.h
 * @brief ROS2 node for synchronized camera and IMU capture from Pico
 *
 * ARCHITECTURE:
 * This node uses event-driven asynchronous publishing to minimize camera callback overhead.
 *
 * Thread 1 (libcamera callback): onRequestCompleted()
 *   - Captures frame from camera (~800µs)
 *   - Looks up trigger timestamp from trigger_map
 *   - Copies frame data to pre-allocated buffer
 *   - Swaps to pending buffer and notifies publisher
 *   - Immediately requeues camera buffer (no blocking)
 *
 * Thread 2 (publisher): publisherThreadLoop()
 *   - Waits on condition variable (zero CPU when idle)
 *   - Woken instantly when frame ready (event-driven)
 *   - Publishes pending frames (~700µs)
 *   - Runs independently, doesn't block camera
 *
 * Thread 3 (serial reader): SerialSync
 *   - Receives trigger packets from Pico
 *   - Stores timestamps in trigger_map
 *   - Publishes IMU data immediately
 *
 * SYNCHRONIZATION:
 * - IMU-camera sync preserved: timestamps captured in Thread 1 before async publish
 * - Thread coordination: condition variable for event-driven notification
 * - Mutex protects shared frame buffer during swap
 */

#ifndef CAMERA_DISPLAY_NODE_CAMERA_DISPLAY_NODE_H
#define CAMERA_DISPLAY_NODE_CAMERA_DISPLAY_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>

#include "camera_display_node/serial_sync.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
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
    void onAltimeterPacket(uint32_t timestamp_us, float altitude_m);
    void onRequestCompleted(libcamera::Request *req);

    // Publisher threads
    void publisherThreadLoopColor();
    void publisherThreadLoopMono();

    // Utilities
    rclcpp::Time getFrameTimestamp(uint16_t frame_id);
    void logSyncStats();

    // ============================================================
    // Publishers
    // ============================================================
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_color_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_mono_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;

    // Pre-allocated message buffers to reduce allocation overhead
    sensor_msgs::msg::Image reusable_msg_color_;
    sensor_msgs::msg::Image reusable_msg_mono_;

    // Event-driven async publish mechanism - Color publisher (2 Hz)
    std::thread publisher_thread_color_;
    std::mutex publish_mutex_color_;
    std::condition_variable publish_cv_color_;
    bool frame_ready_to_publish_color_;
    std::atomic<bool> publisher_running_color_;
    sensor_msgs::msg::Image pending_msg_color_;

    // Event-driven async publish mechanism - Mono publisher (20 Hz)
    std::thread publisher_thread_mono_;
    std::mutex publish_mutex_mono_;
    std::condition_variable publish_cv_mono_;
    bool frame_ready_to_publish_mono_;
    std::atomic<bool> publisher_running_mono_;
    sensor_msgs::msg::Image pending_msg_mono_;
    sensor_msgs::msg::Image pending_bgr_for_mono_;  // BGR frame to convert

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

    size_t trigger_map_max_size_;
    std::atomic<uint16_t> expected_frame_id_;
    std::mutex trigger_map_mutex_;
    std::unordered_map<uint16_t, rclcpp::Time> trigger_map_;  // frame_id → timestamp
    rclcpp::Time latest_trigger_time_;                         // Latest trigger timestamp for sync

    // Altimeter baseline (zero-reset on first sample)
    std::atomic<bool> altitude_baseline_set_;
    float altitude_baseline_m_;

    // ============================================================
    // Synchronization Statistics
    // ============================================================
    std::atomic<uint32_t> frames_received_;
    std::atomic<uint32_t> frames_matched_;
    std::atomic<uint32_t> frame_drops_;

    // ============================================================
    // Performance Metrics
    // ============================================================
    double callback_time_us_;
    double memcpy_time_us_;
    double convert_time_us_;
    double alloc_time_us_;
    double publish_time_mono_us_;
    double publish_time_color_us_;
    std::atomic<uint32_t> slow_callbacks_;  // Count of callbacks > 5ms
    std::atomic<uint32_t> frames_skipped_mono_;
    std::atomic<uint32_t> frames_skipped_color_;
    std::atomic<uint32_t> frame_counter_;  // For color decimation (every 10th frame)
};

#endif  // CAMERA_DISPLAY_NODE_CAMERA_DISPLAY_NODE_H
