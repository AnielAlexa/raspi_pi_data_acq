/**
 * @file camera_display_node.h
 * @brief ROS2 node for synchronized camera and IMU capture from Pico
 *
 * ARCHITECTURE:
 * This node uses event-driven V4L2 capture with select() for zero-CPU blocking.
 *
 * Thread 1 (V4L2 capture): captureThreadLoop()
 *   - select(v4l2_fd_) blocks until frame ready (zero CPU when idle)
 *   - DQBUF, Y16→mono8 conversion, swap to pending buffer, notify publisher
 *   - QBUF to requeue buffer, then blocks again on select()
 *
 * Thread 2 (publisher): publisherThreadLoopMono()
 *   - Waits on condition variable (zero CPU when idle)
 *   - Woken instantly when frame ready (event-driven)
 *   - Publishes pending mono8 frame (~700µs)
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

#include <linux/videodev2.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "camera_display_node/serial_sync.h"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

class CameraDisplayNode : public rclcpp::Node {
public:
    explicit CameraDisplayNode();
    virtual ~CameraDisplayNode();

private:
    // Initialization
    bool initV4L2();
    void enableTriggerMode();
    void initPicoSync(const std::string& serial_port);

    // Capture thread
    void captureThreadLoop();
    void cleanupV4L2();

    // Callbacks
    void onImuPacket(uint32_t timestamp_us, float ax, float ay, float az,
                     float gx, float gy, float gz);
    void onTriggerPacket(uint32_t timestamp_us, uint16_t frame_id);
    void onAltimeterPacket(uint32_t timestamp_us, float altitude_m);

    // Publisher thread
    void publisherThreadLoopMono();

    // Utilities
    rclcpp::Time getFrameTimestamp(uint16_t frame_id);
    void logSyncStats();

    // ============================================================
    // Publishers
    // ============================================================
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_mono_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;

    // Pre-allocated message buffers
    sensor_msgs::msg::Image reusable_msg_mono_;

    // Event-driven async publish mechanism - Mono publisher (20 Hz)
    std::thread publisher_thread_mono_;
    std::mutex publish_mutex_mono_;
    std::condition_variable publish_cv_mono_;
    bool frame_ready_to_publish_mono_;
    std::atomic<bool> publisher_running_mono_;
    sensor_msgs::msg::Image pending_msg_mono_;

    // ============================================================
    // Camera Configuration
    // ============================================================
    int camera_index_;
    int width_;
    int height_;

    // ============================================================
    // V4L2 Objects
    // ============================================================
    int v4l2_fd_ = -1;
    static constexpr int NUM_V4L2_BUFFERS = 4;
    static constexpr int DRAIN_FRAME_COUNT = 5;

    struct V4L2Buffer { void *start = nullptr; size_t length = 0; };
    std::array<V4L2Buffer, NUM_V4L2_BUFFERS> v4l2_buffers_;
    unsigned int v4l2_stride_ = 0;

    // Capture thread
    std::thread capture_thread_;
    std::atomic<bool> capture_running_{false};

    // Sequence calibration (V4L2 buf.sequence → Pico frame_id offset)
    std::atomic<bool> sequence_calibrated_{false};
    int32_t sequence_to_frame_id_offset_ = 0;

    // Parameters
    int exposure_;
    int analogue_gain_;
    bool trigger_mode_enabled_;

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
    std::unordered_map<uint16_t, rclcpp::Time> trigger_map_;
    rclcpp::Time latest_trigger_time_;

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
    double publish_time_mono_us_;
    std::atomic<uint32_t> slow_callbacks_;
    std::atomic<uint32_t> frames_skipped_mono_;
};

#endif  // CAMERA_DISPLAY_NODE_CAMERA_DISPLAY_NODE_H
