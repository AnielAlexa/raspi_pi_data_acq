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
 * Thread 4 (auto-exposure): autoExposureThreadLoop()
 *   - Waits on condition variable (zero CPU when idle)
 *   - Woken by capture thread after computing frame mean
 *   - P-controller adjusts V4L2 exposure via ioctl
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
#include <queue>
#include <string>
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
    void onImuPacket(uint64_t timestamp_us, float ax, float ay, float az,
                     float gx, float gy, float gz);
    void onTriggerPacket(uint64_t timestamp_us, uint16_t frame_id);
    void onAltimeterPacket(uint64_t timestamp_us, float altitude_m);

    // Publisher threads
    void publisherThreadLoopMono();
    void publisherThreadLoopSmall();

    // IMU publish thread
    void imuPublishThreadLoop();

    // Auto-exposure
    void autoExposureThreadLoop();
    bool loadAEConfig(const std::string& path);

    // Adaptive frame rate
    void setFrameRate(int fps);

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
    sensor_msgs::msg::Imu reusable_imu_msg_;
    sensor_msgs::msg::Range reusable_range_msg_;

    // Event-driven async publish mechanism - Mono publisher (20 Hz)
    std::thread publisher_thread_mono_;
    std::mutex publish_mutex_mono_;
    std::condition_variable publish_cv_mono_;
    bool frame_ready_to_publish_mono_;
    std::atomic<bool> publisher_running_mono_;
    sensor_msgs::msg::Image pending_msg_mono_;

    // Small image publisher (6-7 Hz, 320x320 cropped+resized)
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_small_;
    std::thread publisher_thread_small_;
    std::mutex publish_mutex_small_;
    std::condition_variable publish_cv_small_;
    bool frame_ready_to_publish_small_{false};
    std::atomic<bool> publisher_running_small_{false};
    sensor_msgs::msg::Image pending_msg_small_;
    uint32_t small_pub_counter_{0};
    static constexpr uint32_t SMALL_PUB_DECIMATION = 2;  // every 2nd frame → 10 Hz

    // IMU publish thread (decoupled from serial thread)
    std::thread imu_publish_thread_;
    std::atomic<bool> imu_publish_running_{false};
    std::mutex imu_queue_mutex_;
    std::condition_variable imu_queue_cv_;
    std::queue<sensor_msgs::msg::Imu> imu_queue_;

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

    // Cached copy of mmap'd frame (DMA buffers are uncacheable on Tegra)
    std::vector<uint8_t> cached_frame_buf_;

    // Capture thread
    std::thread capture_thread_;
    std::atomic<bool> capture_running_{false};

    // Sequence calibration (V4L2 buf.sequence → Pico frame_id offset)
    std::atomic<bool> sequence_calibrated_{false};
    int32_t sequence_to_frame_id_offset_ = 0;

    // Calibration validation: confirm offset is correct over first N frames
    uint32_t calibration_validation_frames_{0};
    uint32_t calibration_validation_hits_{0};
    static constexpr uint32_t CALIBRATION_VALIDATION_COUNT = 40;

    // Parameters
    int exposure_;
    int frame_rate_;
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

    // V4L2 sequence tracking for accurate frame drop detection
    uint32_t last_v4l2_sequence_{0};
    bool last_v4l2_sequence_valid_{false};

    // ============================================================
    // Synchronization Statistics
    // ============================================================
    std::atomic<uint32_t> frames_received_;
    std::atomic<uint32_t> frames_matched_;
    std::atomic<uint32_t> frame_drops_;

    // ============================================================
    // Drop Detection & Offset Correction
    // ============================================================

    // Layer 1: Dual-gate drop detection (buf.timestamp + wall-clock)
    bool v4l2_ts_available_{false};
    bool v4l2_ts_probed_{false};
    struct timeval last_v4l2_ts_{0, 0};
    bool last_v4l2_ts_valid_{false};
    std::chrono::steady_clock::time_point last_dqbuf_wall_;
    bool last_dqbuf_wall_valid_{false};

    // Layer 2: Pipeline latency validation (fallback if buf.timestamp unavailable)
    double latency_ema_ns_{0.0};
    bool latency_baseline_valid_{false};
    uint32_t latency_warmup_count_{0};
    static constexpr uint32_t LATENCY_WARMUP_FRAMES = 60;
    static constexpr double LATENCY_EMA_ALPHA = 0.02;
    static constexpr double LATENCY_WARMUP_ALPHA = 0.1;
    static constexpr double LATENCY_JUMP_THRESHOLD_NS = 30'000'000.0;  // 30ms

    // Shared counters
    std::atomic<uint32_t> latency_corrections_{0};
    std::atomic<uint32_t> interval_drops_detected_{0};

    // ============================================================
    // Performance Metrics
    // ============================================================
    double callback_time_us_;
    double convert_time_us_;
    double publish_time_mono_us_;
    std::atomic<uint32_t> slow_callbacks_;
    std::atomic<uint32_t> frames_skipped_mono_;

    // ============================================================
    // Auto-Exposure P-Controller
    // ============================================================
    std::thread ae_thread_;
    std::mutex ae_mutex_;
    std::condition_variable ae_cv_;
    bool ae_frame_ready_ = false;
    std::atomic<bool> ae_running_{false};

    // AE config (loaded from file)
    bool ae_enabled_ = false;
    double ae_target_mean_ = 120.0;
    double ae_kp_ = 0.5;
    double ae_deadband_ = 3.0;  // |error| <= deadband → no update (prevents steady-state chatter)
    int ae_min_exposure_ = 1;
    int ae_max_exposure_ = 65523;

    // Shared state
    double ae_current_mean_ = 0.0;
    std::atomic<int> current_exposure_{700};
    double ae_exposure_acc_ = 0.0;  // float accumulator — prevents sub-integer step loss

    // ============================================================
    // Adaptive Frame Rate
    // ============================================================
    struct AfpsBracket { int max_exp; int fps; };

    bool afps_enabled_ = false;
    std::vector<AfpsBracket> afps_brackets_;
    int afps_active_fps_ = -1;  // -1 = not yet set; avoids redundant ioctl
};

#endif  // CAMERA_DISPLAY_NODE_CAMERA_DISPLAY_NODE_H
