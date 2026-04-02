// camera_display.cpp
// ROS2 node that captures video from V4L2 (Arducam JetVariety), synced with Pico triggers.
// Event-driven capture using select() on V4L2 fd — zero CPU when idle.

#include "camera_display_node/camera_display_node.h"

#include <fcntl.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

// ============================================================
// Constructor
// ============================================================
CameraDisplayNode::CameraDisplayNode() : Node("camera_display_node"),
                                         enable_pico_sync_(true),
                                         trigger_map_max_size_(20),
                                         expected_frame_id_(0),
                                         altitude_baseline_set_(false),
                                         altitude_baseline_m_(0.0f),
                                         frames_received_(0),
                                         frames_matched_(0),
                                         frame_drops_(0)
{
    // Declare parameters
    camera_index_ = this->declare_parameter<int>("camera_index", 0);
    width_ = this->declare_parameter<int>("width", 1280);
    height_ = this->declare_parameter<int>("height", 720);
    std::string serial_port = this->declare_parameter<std::string>("serial_port", "/dev/ttyTHS1");
    enable_pico_sync_ = this->declare_parameter<bool>("enable_pico_sync", true);
    exposure_ = this->declare_parameter<int>("exposure", 20);  // range 1-65523, driver default=681
    frame_rate_ = this->declare_parameter<int>("frame_rate", 22);
    trigger_mode_enabled_ = this->declare_parameter<bool>("trigger_mode", true);

    rclcpp::QoS mono_qos(
    rclcpp::QoSInitialization(
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        2
    )
    );
    mono_qos.reliable();
    mono_qos.durability_volatile();

    rclcpp::QoS imu_qos(
    rclcpp::QoSInitialization(
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        50
    )
    );
    imu_qos.best_effort();
    imu_qos.durability_volatile();

    rclcpp::QoS range_qos(
    rclcpp::QoSInitialization(
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        10
    )
    );
    range_qos.reliable();
    range_qos.durability_volatile();

    // Create publishers
    image_pub_mono_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/image_mono", mono_qos);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/imu/data_raw", imu_qos);
    range_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
        "/altimeter/range", range_qos);

    if (!initV4L2()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize V4L2 camera");
        throw std::runtime_error("Failed to initialize V4L2 camera");
    }

    // Pre-allocate cached frame buffer (mmap'd V4L2 DMA memory is uncacheable on Tegra)
    cached_frame_buf_.resize(static_cast<size_t>(v4l2_stride_) * static_cast<size_t>(height_));

    // Pre-allocate message buffers
    const size_t frame_size_mono = static_cast<size_t>(width_) * static_cast<size_t>(height_);

    reusable_msg_mono_.header.frame_id = "camera_link";
    reusable_msg_mono_.encoding = "mono8";
    reusable_msg_mono_.is_bigendian = false;
    reusable_msg_mono_.width = width_;
    reusable_msg_mono_.height = height_;
    reusable_msg_mono_.step = width_;
    reusable_msg_mono_.data.resize(frame_size_mono);

    pending_msg_mono_ = reusable_msg_mono_;
    pending_msg_mono_.data.resize(frame_size_mono);

    // Start event-driven publisher thread
    publisher_running_mono_ = true;
    frame_ready_to_publish_mono_ = false;
    publisher_thread_mono_ = std::thread(&CameraDisplayNode::publisherThreadLoopMono, this);

    // Give publisher thread high priority too, slightly below capture thread
    // to reduce occasional scheduling delays that can cause 100ms publish gaps.
    struct sched_param pub_param;
    pub_param.sched_priority = 48;
    int pub_ret = pthread_setschedparam(publisher_thread_mono_.native_handle(), SCHED_FIFO, &pub_param);
    if (pub_ret != 0) {
        if (pub_ret == EPERM || pub_ret == EACCES) {
            RCLCPP_INFO(this->get_logger(),
                       "Publisher RT priority not applied (%s). Continuing with normal scheduler.",
                       strerror(pub_ret));
        } else {
            RCLCPP_WARN(this->get_logger(),
                       "Failed to set RT priority on publisher thread: %s",
                       strerror(pub_ret));
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Publisher thread set to SCHED_FIFO priority 48");
    }

    // Start capture thread with real-time priority to minimize jitter
    capture_running_ = true;
    capture_thread_ = std::thread(&CameraDisplayNode::captureThreadLoop, this);

    // Set SCHED_FIFO on capture thread for deterministic frame timing
    struct sched_param param;
    param.sched_priority = 49;  // Below kernel threads (50+), above normal user tasks
    int ret = pthread_setschedparam(capture_thread_.native_handle(), SCHED_FIFO, &param);
    if (ret != 0) {
        if (ret == EPERM || ret == EACCES) {
            RCLCPP_INFO(this->get_logger(),
                       "Capture RT priority not applied (%s). Continuing with normal scheduler.",
                       strerror(ret));
        } else {
            RCLCPP_WARN(this->get_logger(),
                       "Failed to set RT priority on capture thread: %s",
                       strerror(ret));
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Capture thread set to SCHED_FIFO priority 49");
    }

    RCLCPP_INFO(this->get_logger(), "Camera ready: %dx%d | Mono@20Hz (event-driven V4L2, %d bufs)",
               width_, height_, NUM_V4L2_BUFFERS);

    // Initialize FPS tracking
    last_frame_time_ = std::chrono::steady_clock::now();
    last_log_time_ = last_frame_time_;
    smoothed_fps_ = 0.0;
    frames_since_log_ = 0;

    // Initialize performance metrics
    callback_time_us_ = 0.0;
    convert_time_us_ = 0.0;
    publish_time_mono_us_ = 0.0;
    slow_callbacks_ = 0;
    frames_skipped_mono_ = 0;

    // Pre-allocate IMU message (static fields set once)
    reusable_imu_msg_.header.frame_id = "imu_link";
    reusable_imu_msg_.orientation_covariance[0] = -1.0;
    reusable_imu_msg_.linear_acceleration_covariance[0] = 0.01;
    reusable_imu_msg_.angular_velocity_covariance[0] = 0.01;

    // Pre-allocate Range message (static fields set once)
    reusable_range_msg_.header.frame_id = "altimeter";
    reusable_range_msg_.radiation_type = sensor_msgs::msg::Range::INFRARED;
    reusable_range_msg_.field_of_view = 0.0;
    reusable_range_msg_.min_range = -500.0;
    reusable_range_msg_.max_range = 9000.0;

    // Start IMU publish thread (decoupled from serial thread)
    imu_publish_running_ = true;
    imu_publish_thread_ = std::thread(&CameraDisplayNode::imuPublishThreadLoop, this);

    struct sched_param imu_pub_param;
    imu_pub_param.sched_priority = 46;
    int imu_pub_ret = pthread_setschedparam(imu_publish_thread_.native_handle(), SCHED_FIFO, &imu_pub_param);
    if (imu_pub_ret != 0) {
        if (imu_pub_ret == EPERM || imu_pub_ret == EACCES) {
            RCLCPP_INFO(this->get_logger(),
                       "IMU publisher RT priority not applied (%s). Continuing with normal scheduler.",
                       strerror(imu_pub_ret));
        } else {
            RCLCPP_WARN(this->get_logger(),
                       "Failed to set RT priority on IMU publisher thread: %s",
                       strerror(imu_pub_ret));
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "IMU publisher thread set to SCHED_FIFO priority 46");
    }

    // Initialize Pico serial synchronization if enabled
    if (enable_pico_sync_) {
        initPicoSync(serial_port);
    }

    // Auto-exposure P-controller
    std::string ae_default_path = ament_index_cpp::get_package_share_directory("camera_display_node")
                                  + "/config/auto_exposure.yaml";
    std::string ae_config_path = this->declare_parameter<std::string>("ae_config_path", ae_default_path);
    loadAEConfig(ae_config_path);
    current_exposure_.store(exposure_);

    if (ae_enabled_) {
        ae_running_ = true;
        ae_thread_ = std::thread(&CameraDisplayNode::autoExposureThreadLoop, this);
        RCLCPP_INFO(this->get_logger(), "Auto-exposure enabled: target=%.0f, Kp=%.4f, deadband=±%.1f, range=[%d,%d]",
                    ae_target_mean_, ae_kp_, ae_deadband_, ae_min_exposure_, ae_max_exposure_);
    }
}

// ============================================================
// Destructor
// ============================================================
CameraDisplayNode::~CameraDisplayNode() {
    // Stop auto-exposure thread
    if (ae_running_) {
        ae_running_ = false;
        ae_cv_.notify_one();
        if (ae_thread_.joinable()) ae_thread_.join();
    }

    // Stop capture thread
    if (capture_running_) {
        capture_running_ = false;

        // Unblock capture thread if it is waiting in select() forever.
        // cleanupV4L2() is idempotent and safe to call again later.
        cleanupV4L2();

        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
    }

    // Stop publisher thread
    if (publisher_running_mono_) {
        publisher_running_mono_ = false;
        publish_cv_mono_.notify_one();
        if (publisher_thread_mono_.joinable()) {
            publisher_thread_mono_.join();
        }
    }

    // Stop IMU publish thread before stopping serial (which feeds the queue)
    if (imu_publish_running_) {
        imu_publish_running_ = false;
        imu_queue_cv_.notify_one();
        if (imu_publish_thread_.joinable()) {
            imu_publish_thread_.join();
        }
    }

    if (serial_sync_) {
        serial_sync_->stop();
    }

    cleanupV4L2();

    // Log final synchronization statistics
    if (enable_pico_sync_) {
        logSyncStats();
    }

    RCLCPP_INFO(this->get_logger(), "Camera display node shut down");
}

// ============================================================
// V4L2 Initialization
// ============================================================
bool CameraDisplayNode::initV4L2() {
    // Open V4L2 device
    std::string dev_path = "/dev/video" + std::to_string(camera_index_);
    v4l2_fd_ = open(dev_path.c_str(), O_RDWR);
    if (v4l2_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open %s: %s",
                    dev_path.c_str(), strerror(errno));
        return false;
    }

    // Verify capabilities
    struct v4l2_capability cap;
    if (ioctl(v4l2_fd_, VIDIOC_QUERYCAP, &cap) < 0) {
        RCLCPP_ERROR(this->get_logger(), "VIDIOC_QUERYCAP failed: %s", strerror(errno));
        return false;
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        RCLCPP_ERROR(this->get_logger(), "Device does not support video capture");
        return false;
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        RCLCPP_ERROR(this->get_logger(), "Device does not support streaming");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "V4L2 device: %s (%s)", cap.card, cap.driver);

    // Set pixel format
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width_;
    fmt.fmt.pix.height = height_;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(v4l2_fd_, VIDIOC_S_FMT, &fmt) < 0) {
        RCLCPP_ERROR(this->get_logger(), "VIDIOC_S_FMT failed: %s", strerror(errno));
        return false;
    }

    // Update from driver response (may differ from request)
    width_ = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;
    v4l2_stride_ = fmt.fmt.pix.bytesperline;
    RCLCPP_INFO(this->get_logger(), "V4L2 format: %dx%d, stride=%u, pixfmt=0x%08X",
               width_, height_, v4l2_stride_, fmt.fmt.pix.pixelformat);

    // Request mmap buffers
    struct v4l2_requestbuffers reqbufs;
    memset(&reqbufs, 0, sizeof(reqbufs));
    reqbufs.count = NUM_V4L2_BUFFERS;
    reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbufs.memory = V4L2_MEMORY_MMAP;

    if (ioctl(v4l2_fd_, VIDIOC_REQBUFS, &reqbufs) < 0) {
        RCLCPP_ERROR(this->get_logger(), "VIDIOC_REQBUFS failed: %s", strerror(errno));
        return false;
    }
    if (static_cast<int>(reqbufs.count) < NUM_V4L2_BUFFERS) {
        RCLCPP_WARN(this->get_logger(), "Requested %d buffers, got %u",
                   NUM_V4L2_BUFFERS, reqbufs.count);
    }

    // Query and mmap each buffer
    for (int i = 0; i < static_cast<int>(reqbufs.count) && i < NUM_V4L2_BUFFERS; ++i) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(v4l2_fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            RCLCPP_ERROR(this->get_logger(), "VIDIOC_QUERYBUF[%d] failed: %s",
                        i, strerror(errno));
            return false;
        }

        v4l2_buffers_[i].length = buf.length;
        v4l2_buffers_[i].start = mmap(nullptr, buf.length,
                                       PROT_READ | PROT_WRITE, MAP_SHARED,
                                       v4l2_fd_, buf.m.offset);
        if (v4l2_buffers_[i].start == MAP_FAILED) {
            RCLCPP_ERROR(this->get_logger(), "mmap[%d] failed: %s", i, strerror(errno));
            v4l2_buffers_[i].start = nullptr;
            return false;
        }
    }

    // Queue all buffers
    for (int i = 0; i < static_cast<int>(reqbufs.count) && i < NUM_V4L2_BUFFERS; ++i) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(v4l2_fd_, VIDIOC_QBUF, &buf) < 0) {
            RCLCPP_ERROR(this->get_logger(), "VIDIOC_QBUF[%d] failed: %s",
                        i, strerror(errno));
            return false;
        }
    }

    // Set exposure and gain
    struct v4l2_control ctrl;

    ctrl.id = V4L2_CID_EXPOSURE;
    ctrl.value = 130;//std::max(1, std::min(50000, exposure_));
    if (ioctl(v4l2_fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to set exposure=%d: %s",
                   exposure_, strerror(errno));
    } else {
        RCLCPP_INFO(this->get_logger(), "Exposure set to %d ", exposure_);
    }

    // Start streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(v4l2_fd_, VIDIOC_STREAMON, &type) < 0) {
        RCLCPP_ERROR(this->get_logger(), "VIDIOC_STREAMON failed: %s", strerror(errno));
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "V4L2 streaming started, draining %d initial frames...",
               DRAIN_FRAME_COUNT);

    // Drain initial stale frames
    for (int i = 0; i < DRAIN_FRAME_COUNT; ++i) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(v4l2_fd_, &fds);
        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        int ret = select(v4l2_fd_ + 1, &fds, nullptr, nullptr, &tv);
        if (ret <= 0) {
            RCLCPP_WARN(this->get_logger(), "Drain frame %d: select timeout or error", i);
            continue;
        }

        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(v4l2_fd_, VIDIOC_DQBUF, &buf) < 0) {
            RCLCPP_WARN(this->get_logger(), "Drain DQBUF failed: %s", strerror(errno));
            continue;
        }
        if (ioctl(v4l2_fd_, VIDIOC_QBUF, &buf) < 0) {
            RCLCPP_WARN(this->get_logger(), "Drain QBUF failed: %s", strerror(errno));
        }
    }

    RCLCPP_INFO(this->get_logger(), "Drain complete");

    // Enable trigger mode if requested
    if (trigger_mode_enabled_) {
        usleep(1000000);  // 1s settle time before switching to trigger mode
        enableTriggerMode();
    }

    // Set frame rate after trigger mode (Arducam user control 0x00981906, min=5 max=80 step=1)
    {
        struct v4l2_control ctrl;
        memset(&ctrl, 0, sizeof(ctrl));
        ctrl.id    = 0x00981906;
        ctrl.value = frame_rate_;
        if (ioctl(v4l2_fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to set frame_rate=%d: %s",
                       frame_rate_, strerror(errno));
        } else {
            RCLCPP_INFO(this->get_logger(), "Frame rate set to %d fps", frame_rate_);
        }
    }

    return true;
}

// ============================================================
// Enable Arducam Trigger Mode
// ============================================================
void CameraDisplayNode::enableTriggerMode() {
    // Control IDs from: v4l2-ctl -d /dev/video0 -l
    // User Controls (0x0098xxxx)
    static constexpr uint32_t ARDUCAM_TRIGGER_MODE_ID        = 0x00981901;  // trigger_mode bool
    static constexpr uint32_t ARDUCAM_DISABLE_FRAME_TIMEOUT  = 0x00981902;  // disable_frame_timeout bool
    static constexpr uint32_t ARDUCAM_FRAME_TIMEOUT_ID       = 0x00981903;  // frame_timeout int (ms, 100–12000)

    struct v4l2_control ctrl;

    // Enable trigger mode
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id    = ARDUCAM_TRIGGER_MODE_ID;
    ctrl.value = 1;
    if (ioctl(v4l2_fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to enable trigger_mode (0x%08X): %s",
                     ARDUCAM_TRIGGER_MODE_ID, strerror(errno));
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Trigger mode enabled");

    // Keep frame-timeout active (do NOT set disable_frame_timeout)
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id    = ARDUCAM_DISABLE_FRAME_TIMEOUT;
    ctrl.value = 0;
    if (ioctl(v4l2_fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
        RCLCPP_WARN(this->get_logger(),
                    "Could not clear disable_frame_timeout (0x%08X): %s",
                    ARDUCAM_DISABLE_FRAME_TIMEOUT, strerror(errno));
    }

    // Set frame timeout (ms) — how long driver waits before reporting no frame
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id    = ARDUCAM_FRAME_TIMEOUT_ID;
    ctrl.value = 2000;
    if (ioctl(v4l2_fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to set frame_timeout (0x%08X): %s",
                    ARDUCAM_FRAME_TIMEOUT_ID, strerror(errno));
    } else {
        RCLCPP_INFO(this->get_logger(), "Frame timeout set to 2000 ms");
    }
}

// ============================================================
// V4L2 Cleanup
// ============================================================
void CameraDisplayNode::cleanupV4L2() {
    if (v4l2_fd_ < 0) return;

    // Stop streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(v4l2_fd_, VIDIOC_STREAMOFF, &type);

    // Unmap buffers
    for (auto& buf : v4l2_buffers_) {
        if (buf.start && buf.start != MAP_FAILED) {
            munmap(buf.start, buf.length);
            buf.start = nullptr;
        }
    }

    close(v4l2_fd_);
    v4l2_fd_ = -1;
}

// ============================================================
// V4L2 Capture Thread Loop (Event-Driven)
// ============================================================
void CameraDisplayNode::captureThreadLoop() {
    while (capture_running_) {
        // select() blocks until a frame is ready — zero CPU when idle
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(v4l2_fd_, &fds);

        // 200ms timeout lets the capture thread check capture_running_ on shutdown
        // without burning CPU — a blocked select() would delay join() up to one frame period.
        struct timeval tv { 0, 200000 };
        int ret = select(v4l2_fd_ + 1, &fds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            RCLCPP_ERROR(this->get_logger(), "select() failed: %s", strerror(errno));
            break;
        }
        if (ret == 0) continue;  // timeout — loop back to check capture_running_

        auto callback_start = std::chrono::steady_clock::now();

        // Dequeue frame
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(v4l2_fd_, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) continue;
            RCLCPP_ERROR(this->get_logger(), "VIDIOC_DQBUF failed: %s", strerror(errno));
            break;
        }

        frames_received_++;

        // --- V4L2 buf.timestamp probe & drop detection (Layer 1) ---
        if (!v4l2_ts_probed_) {
            v4l2_ts_probed_ = true;
            v4l2_ts_available_ = (buf.timestamp.tv_sec != 0 || buf.timestamp.tv_usec != 0);
            RCLCPP_INFO(this->get_logger(), "V4L2 buf.timestamp %s",
                        v4l2_ts_available_ ? "available — using for drop detection"
                                           : "not populated by driver");
        }

        if (v4l2_ts_available_ && enable_pico_sync_ && sequence_calibrated_.load()) {
            if (last_v4l2_ts_valid_ && last_dqbuf_wall_valid_) {
                double v4l2_interval_ms =
                    (buf.timestamp.tv_sec - last_v4l2_ts_.tv_sec) * 1000.0
                  + (buf.timestamp.tv_usec - last_v4l2_ts_.tv_usec) / 1000.0;
                double wall_interval_ms =
                    std::chrono::duration<double, std::milli>(
                        callback_start - last_dqbuf_wall_).count();
                // Dual-gate: BOTH buf.timestamp AND wall-clock must exceed 1.5× period.
                // Prevents false positives from kernel timestamp jitter under GPU load.
                if (v4l2_interval_ms > 75.0 && wall_interval_ms > 75.0) {
                    int dropped = std::max(1,
                        static_cast<int>(std::round(wall_interval_ms / 50.0)) - 1);
                    sequence_to_frame_id_offset_ += dropped;
                    interval_drops_detected_ += dropped;
                    RCLCPP_WARN(this->get_logger(),
                        "V4L2 drop detected: v4l2=%.1fms wall=%.1fms, %d frame(s) dropped, "
                        "offset adjusted to %d",
                        v4l2_interval_ms, wall_interval_ms, dropped,
                        sequence_to_frame_id_offset_);
                } else if (v4l2_interval_ms > 75.0) {
                    RCLCPP_DEBUG(this->get_logger(),
                        "buf.timestamp glitch (v4l2=%.1fms, wall=%.1fms) — ignored",
                        v4l2_interval_ms, wall_interval_ms);
                }
            }
            last_v4l2_ts_ = buf.timestamp;
            last_v4l2_ts_valid_ = true;
        }
        last_dqbuf_wall_ = callback_start;
        last_dqbuf_wall_valid_ = true;

        // --- Sequence → Frame ID calibration ---
        uint16_t frame_id;
        if (enable_pico_sync_) {
            if (!sequence_calibrated_.load()) {
                uint16_t expected = expected_frame_id_.load();
                if (expected > 0) {
                    // Pico has sent at least one trigger — calibrate
                    sequence_to_frame_id_offset_ = static_cast<int32_t>(expected) - static_cast<int32_t>(buf.sequence);
                    sequence_calibrated_ = true;
                    calibration_validation_frames_ = 0;
                    calibration_validation_hits_ = 0;
                    RCLCPP_INFO(this->get_logger(),
                               "Sequence calibrated: V4L2 seq=%u → frame_id=%u (offset=%d)",
                               buf.sequence, expected, sequence_to_frame_id_offset_);
                }
            }
            frame_id = static_cast<uint16_t>((static_cast<int32_t>(buf.sequence) + sequence_to_frame_id_offset_) & 0xFFFF);
        } else {
            frame_id = static_cast<uint16_t>(buf.sequence & 0xFFFF);
        }

        // --- Frame drop detection (via V4L2 sequence gaps) ---
        // NOTE: On the Arducam JetVariety driver, buf.sequence does NOT skip for
        // dropped frames. The buf.timestamp interval detector (Layer 1) or the
        // pipeline latency validator (Layer 2) handles offset correction instead.
        if (last_v4l2_sequence_valid_) {
            uint32_t gap = buf.sequence - last_v4l2_sequence_;
            if (gap > 1) {
                frame_drops_ += gap - 1;
                RCLCPP_WARN(this->get_logger(),
                           "V4L2 frame drop: seq %u→%u (%u dropped)",
                           last_v4l2_sequence_, buf.sequence, gap - 1);
            }
        }
        last_v4l2_sequence_ = buf.sequence;
        last_v4l2_sequence_valid_ = true;

        // --- FPS Calculation ---
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_frame_time_).count();
        last_frame_time_ = now;

        if (dt > 0.0) {
            double inst_fps = 1.0 / dt;
            double alpha = 0.1;
            smoothed_fps_ = (smoothed_fps_ == 0.0) ? inst_fps :
                           (alpha * inst_fps + (1.0 - alpha) * smoothed_fps_);
        }
        frames_since_log_++;

        // Log stats every 5 seconds
        if (std::chrono::duration<double>(now - last_log_time_).count() >= 5.0) {
            if (enable_pico_sync_) {
                uint32_t matched = frames_matched_.load();
                uint32_t received = frames_received_.load();
                double match_rate = (received > 0) ? (100.0 * matched / received) : 0.0;
                uint32_t int_drops = interval_drops_detected_.load();
                uint32_t lat_corr = latency_corrections_.load();
                if (ae_enabled_) {
                    RCLCPP_INFO(this->get_logger(),
                               "FPS: %.1f | Mean: %.0f Exp: %d | Sync: %.0f%% | Drops: %u+%u | CB: %.0fµs [Conv:%.0f] | Mono: %.0fµs",
                               smoothed_fps_, ae_current_mean_, current_exposure_.load(),
                               match_rate, int_drops, lat_corr,
                               callback_time_us_,
                               convert_time_us_, publish_time_mono_us_);
                } else {
                    RCLCPP_INFO(this->get_logger(),
                               "FPS: %.1f | Sync: %.0f%% | Drops: %u+%u | CB: %.0fµs [Conv:%.0f] | Mono: %.0fµs",
                               smoothed_fps_, match_rate, int_drops, lat_corr,
                               callback_time_us_,
                               convert_time_us_, publish_time_mono_us_);
                }
            } else {
                if (ae_enabled_) {
                    RCLCPP_INFO(this->get_logger(),
                               "FPS: %.1f | Mean: %.0f Exp: %d | CB: %.0fµs [Conv:%.0f] | Mono: %.0fµs",
                               smoothed_fps_, ae_current_mean_, current_exposure_.load(),
                               callback_time_us_, convert_time_us_, publish_time_mono_us_);
                } else {
                    RCLCPP_INFO(this->get_logger(),
                               "FPS: %.1f | CB: %.0fµs [Conv:%.0f] | Mono: %.0fµs",
                               smoothed_fps_, callback_time_us_,
                               convert_time_us_, publish_time_mono_us_);
                }
            }
            frames_since_log_ = 0;
            last_log_time_ = now;
        }

        // --- Timestamp lookup ---
        rclcpp::Time frame_timestamp;
        if (enable_pico_sync_) {
            rclcpp::Time frame_time = getFrameTimestamp(frame_id);
            bool hit = (frame_time.nanoseconds() > 0);
            if (hit) {
                frame_timestamp = frame_time;
                frames_matched_++;
            } else {
                frame_timestamp = this->now();
            }

            // Calibration validation: if the offset is wrong by ±1, we'll miss consistently.
            // After CALIBRATION_VALIDATION_COUNT frames, if hit rate < 80% adjust by -1 and retry.
            if (sequence_calibrated_.load() &&
                calibration_validation_frames_ < CALIBRATION_VALIDATION_COUNT)
            {
                calibration_validation_frames_++;
                if (hit) calibration_validation_hits_++;

                if (calibration_validation_frames_ == CALIBRATION_VALIDATION_COUNT) {
                    double hit_rate = static_cast<double>(calibration_validation_hits_) /
                                     CALIBRATION_VALIDATION_COUNT;
                    if (hit_rate < 0.80) {
                        // Offset is likely off by 1 — decrement and recalibrate
                        sequence_to_frame_id_offset_--;
                        calibration_validation_frames_ = 0;
                        calibration_validation_hits_ = 0;
                        RCLCPP_WARN(this->get_logger(),
                                   "Calibration hit rate %.0f%% < 80%% — adjusting offset to %d",
                                   hit_rate * 100.0, sequence_to_frame_id_offset_);
                    } else {
                        RCLCPP_INFO(this->get_logger(),
                                   "Calibration validated: %.0f%% hit rate over %u frames",
                                   hit_rate * 100.0, CALIBRATION_VALIDATION_COUNT);
                    }
                }
            }

            // --- Layer 2: Pipeline latency EMA update (when buf.timestamp unavailable) ---
            if (!v4l2_ts_available_ && hit &&
                calibration_validation_frames_ >= CALIBRATION_VALIDATION_COUNT)
            {
                double latency_ns = static_cast<double>(
                    (this->now() - frame_timestamp).nanoseconds());

                if (!latency_baseline_valid_) {
                    latency_warmup_count_++;
                    if (latency_ema_ns_ == 0.0) {
                        latency_ema_ns_ = latency_ns;
                    } else {
                        latency_ema_ns_ += LATENCY_WARMUP_ALPHA *
                            (latency_ns - latency_ema_ns_);
                    }
                    if (latency_warmup_count_ >= LATENCY_WARMUP_FRAMES) {
                        latency_baseline_valid_ = true;
                        RCLCPP_INFO(this->get_logger(),
                            "Latency baseline converged: %.2f ms",
                            latency_ema_ns_ / 1e6);
                    }
                } else {
                    double deviation = std::abs(latency_ns - latency_ema_ns_);
                    if (deviation < LATENCY_JUMP_THRESHOLD_NS) {
                        latency_ema_ns_ += LATENCY_EMA_ALPHA *
                            (latency_ns - latency_ema_ns_);
                    }
                }
            }
        } else {
            frame_timestamp = this->now();
        }

        // --- GREY/Y8 → mono8 copy (cached copy + row memcpy) ---
        auto convert_start = std::chrono::steady_clock::now();

        reusable_msg_mono_.header.stamp = frame_timestamp;

        // Step 1: bulk-copy from uncacheable DMA mmap → cached heap buffer
        const size_t frame_bytes = static_cast<size_t>(v4l2_stride_) * static_cast<size_t>(height_);
        std::memcpy(cached_frame_buf_.data(), v4l2_buffers_[buf.index].start, frame_bytes);

        // Step 2: cached buffer → message buffer
        const uint8_t* src_base = cached_frame_buf_.data();
        uint8_t* dst_base = reusable_msg_mono_.data.data();

        // Fast path: tightly packed frame (stride == width)
        if (v4l2_stride_ == static_cast<unsigned int>(width_)) {
            std::memcpy(dst_base, src_base,
                       static_cast<size_t>(width_) * static_cast<size_t>(height_));
        } else {
            for (int r = 0; r < height_; ++r) {
                const uint8_t* src_row = src_base + r * v4l2_stride_;
                uint8_t* dst_row = dst_base + r * width_;
                std::memcpy(dst_row, src_row, static_cast<size_t>(width_));
            }
        }

        auto convert_end = std::chrono::steady_clock::now();
        convert_time_us_ = std::chrono::duration<double>(convert_end - convert_start).count() * 1e6;

        // --- Compute frame mean for auto-exposure (data already in CPU cache) ---
        if (ae_enabled_) {
            const uint8_t* pixels = reusable_msg_mono_.data.data();
            const size_t n = reusable_msg_mono_.data.size();
            uint64_t sum = 0;
            for (size_t i = 0; i < n; ++i) sum += pixels[i];
            double mean = static_cast<double>(sum) / static_cast<double>(n);

            std::lock_guard<std::mutex> ae_lock(ae_mutex_);
            ae_current_mean_ = mean;
            ae_frame_ready_ = true;
            ae_cv_.notify_one();
        }

        // --- Notify publisher ---
        {
            std::lock_guard<std::mutex> lock(publish_mutex_mono_);
            if (!frame_ready_to_publish_mono_) {
                std::swap(pending_msg_mono_, reusable_msg_mono_);
                frame_ready_to_publish_mono_ = true;
            } else {
                // Publisher still busy: overwrite pending with newest frame.
                // This reduces staleness and helps avoid apparent 100ms gaps.
                std::swap(pending_msg_mono_, reusable_msg_mono_);
                frames_skipped_mono_++;
            }
            publish_cv_mono_.notify_one();
        }

        // --- Requeue buffer ---
        if (ioctl(v4l2_fd_, VIDIOC_QBUF, &buf) < 0) {
            RCLCPP_ERROR(this->get_logger(), "VIDIOC_QBUF requeue failed: %s", strerror(errno));
        }

        // --- Measure callback time ---
        auto callback_end = std::chrono::steady_clock::now();
        callback_time_us_ = std::chrono::duration<double>(callback_end - callback_start).count() * 1e6;

        if (callback_time_us_ > 5000.0) {
            slow_callbacks_++;
            RCLCPP_DEBUG(this->get_logger(),
                        "Slow callback: %.1f µs (convert: %.1f µs)",
                        callback_time_us_, convert_time_us_);
        }
    }
}

// ============================================================
// Pico Synchronization Initialization
// ============================================================
void CameraDisplayNode::initPicoSync(const std::string& serial_port) {
    serial_sync_ = std::make_unique<camera_display_node::SerialSync>(
        this,
        serial_port,
        std::bind(&CameraDisplayNode::onImuPacket, this,
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                 std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
                 std::placeholders::_7),
        std::bind(&CameraDisplayNode::onTriggerPacket, this,
                 std::placeholders::_1, std::placeholders::_2),
        std::bind(&CameraDisplayNode::onAltimeterPacket, this,
                 std::placeholders::_1, std::placeholders::_2)
    );

    serial_sync_->start();

    uint32_t wait_count = 0;
    while (rclcpp::ok() && !serial_sync_->is_calibrated() && wait_count < 50) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        wait_count++;
    }

    if (!serial_sync_->is_calibrated()) {
        RCLCPP_WARN(this->get_logger(), "Pico time sync timeout");
    }

    trigger_map_.reserve(trigger_map_max_size_);
}

// ============================================================
// IMU Packet Callback
// ============================================================
void CameraDisplayNode::onImuPacket(uint64_t timestamp_us, float ax, float ay, float az,
                                     float gx, float gy, float gz) {
    if (!serial_sync_ || !imu_pub_) return;

    sensor_msgs::msg::Imu msg = reusable_imu_msg_;
    msg.header.stamp = serial_sync_->pico_to_ros_time(timestamp_us);

    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;

    msg.angular_velocity.x = gx;
    msg.angular_velocity.y = gy;
    msg.angular_velocity.z = gz;

    {
        std::lock_guard<std::mutex> lock(imu_queue_mutex_);
        if (imu_queue_.size() >= 200) {
            imu_queue_.pop();  // drop oldest
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "IMU queue overflow (200) — dropping oldest sample");
        }
        imu_queue_.push(std::move(msg));
    }
    imu_queue_cv_.notify_one();
}

// ============================================================
// IMU Publish Thread
// ============================================================
void CameraDisplayNode::imuPublishThreadLoop() {
    std::vector<sensor_msgs::msg::Imu> batch;
    batch.reserve(32);

    while (imu_publish_running_ || !imu_queue_.empty()) {
        {
            std::unique_lock<std::mutex> lock(imu_queue_mutex_);
            imu_queue_cv_.wait(lock, [this] {
                return !imu_queue_.empty() || !imu_publish_running_;
            });

            while (!imu_queue_.empty()) {
                batch.push_back(std::move(imu_queue_.front()));
                imu_queue_.pop();
            }
        }

        for (auto& msg : batch) {
            imu_pub_->publish(msg);
        }
        batch.clear();
    }
}

// ============================================================
// Altimeter Packet Callback
// ============================================================
void CameraDisplayNode::onAltimeterPacket(uint64_t timestamp_us, float altitude_m) {
    if (!serial_sync_ || !range_pub_) return;

    if (!altitude_baseline_set_.load()) {
        altitude_baseline_m_ = altitude_m;
        altitude_baseline_set_.store(true);
        RCLCPP_INFO(this->get_logger(), "Altimeter baseline set: %.3f m (will be zeroed)", altitude_m);
    }

    float relative_altitude_m = altitude_m - altitude_baseline_m_;

    reusable_range_msg_.header.stamp = serial_sync_->pico_to_ros_time(timestamp_us);
    reusable_range_msg_.range = relative_altitude_m;

    range_pub_->publish(reusable_range_msg_);
}

// ============================================================
// Trigger Packet Callback
// ============================================================
void CameraDisplayNode::onTriggerPacket(uint64_t timestamp_us, uint16_t frame_id) {
    if (!serial_sync_) return;

    // serial_sync only dispatches trigger callbacks after calibration, so timestamp is always valid
    rclcpp::Time trigger_time = serial_sync_->pico_to_ros_time(timestamp_us);

    {
        std::lock_guard<std::mutex> lock(trigger_map_mutex_);
        trigger_map_[frame_id] = trigger_time;
        latest_trigger_time_ = trigger_time;

        if (trigger_map_.size() > trigger_map_max_size_) {
            // Evict entries older than current frame_id (accounting for uint16 wrap)
            auto it = trigger_map_.begin();
            while (it != trigger_map_.end()) {
                int32_t age = static_cast<int32_t>(frame_id) - static_cast<int32_t>(it->first);
                if (age < 0) age += 0x10000;  // handle wrap
                if (age > static_cast<int32_t>(trigger_map_max_size_ / 2)) {
                    it = trigger_map_.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }

    expected_frame_id_ = frame_id;
}

// ============================================================
// Frame Timestamp Lookup
// ============================================================
rclcpp::Time CameraDisplayNode::getFrameTimestamp(uint16_t frame_id) {
    std::lock_guard<std::mutex> lock(trigger_map_mutex_);
    auto it = trigger_map_.find(frame_id);
    if (it == trigger_map_.end()) {
        return rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    rclcpp::Time candidate = it->second;

    // Layer 2: latency validation (only when buf.timestamp is unavailable)
    if (!v4l2_ts_available_ && latency_baseline_valid_) {
        double latency_ns = static_cast<double>(
            (this->now() - candidate).nanoseconds());
        double deviation = latency_ns - latency_ema_ns_;

        if (deviation > LATENCY_JUMP_THRESHOLD_NS) {
            // Latency anomalously high — try adjacent frame_ids
            for (int shift = 1; shift <= 3; ++shift) {
                uint16_t trial_id = static_cast<uint16_t>(
                    (frame_id + shift) & 0xFFFF);
                auto trial_it = trigger_map_.find(trial_id);
                if (trial_it != trigger_map_.end()) {
                    double trial_latency = static_cast<double>(
                        (this->now() - trial_it->second).nanoseconds());
                    double trial_dev = std::abs(trial_latency - latency_ema_ns_);
                    if (trial_dev < std::abs(deviation) * 0.5) {
                        // Better match — correct offset and use it
                        rclcpp::Time corrected = trial_it->second;
                        trigger_map_.erase(it);        // stale entry
                        trigger_map_.erase(trial_it);  // consumed entry
                        sequence_to_frame_id_offset_ += shift;
                        latency_corrections_++;
                        latency_baseline_valid_ = false;
                        latency_warmup_count_ = 0;
                        latency_ema_ns_ = 0.0;
                        RCLCPP_WARN(this->get_logger(),
                            "Latency correction: shift=+%d, new offset=%d",
                            shift, sequence_to_frame_id_offset_);
                        return corrected;
                    }
                }
            }
        }
    }

    trigger_map_.erase(it);
    return candidate;
}

// ============================================================
// Synchronization Statistics
// ============================================================
void CameraDisplayNode::logSyncStats() {
    RCLCPP_INFO(this->get_logger(), "=== Synchronization Statistics ===");
    RCLCPP_INFO(this->get_logger(), "Total frames received: %u", frames_received_.load());
    RCLCPP_INFO(this->get_logger(), "Frames matched to trigger: %u", frames_matched_.load());
    RCLCPP_INFO(this->get_logger(), "Frame drops detected (seq): %u, (interval): %u",
               frame_drops_.load(), interval_drops_detected_.load());
    RCLCPP_INFO(this->get_logger(), "Latency corrections: %u", latency_corrections_.load());
    RCLCPP_INFO(this->get_logger(), "Frames skipped (publisher busy): %u",
               frames_skipped_mono_.load());
    RCLCPP_INFO(this->get_logger(), "Slow callbacks (>5ms): %u", slow_callbacks_.load());
    RCLCPP_INFO(this->get_logger(), "Publishing mode: Mono@20Hz (event-driven V4L2)");
    RCLCPP_INFO(this->get_logger(), "Last callback: %.1f µs [convert: %.1f]",
               callback_time_us_, convert_time_us_);
    RCLCPP_INFO(this->get_logger(), "Last publish: Mono %.1f µs", publish_time_mono_us_);

    uint32_t total = frames_received_.load();
    uint32_t published_mono = total - frames_skipped_mono_.load();
    if (total > 0) {
        double match_rate = (100.0 * frames_matched_.load()) / total;
        double mono_rate = (100.0 * published_mono) / total;
        RCLCPP_INFO(this->get_logger(), "Match rate: %.1f%%", match_rate);
        RCLCPP_INFO(this->get_logger(), "Mono publish rate: %.1f%% (%u/%u frames)",
                   mono_rate, published_mono, total);
    }
}

// ============================================================
// Mono Publisher Thread Loop (20 Hz, Event-Driven)
// ============================================================
void CameraDisplayNode::publisherThreadLoopMono() {
    // Local buffer avoids holding the mutex during publish (~700µs of DDS serialization)
    sensor_msgs::msg::Image local_msg;
    local_msg.header.frame_id = "camera_link";
    local_msg.encoding = "mono8";
    local_msg.is_bigendian = false;
    local_msg.width = width_;
    local_msg.height = height_;
    local_msg.step = width_;
    local_msg.data.resize(static_cast<size_t>(width_) * static_cast<size_t>(height_));

    while (publisher_running_mono_) {
        std::unique_lock<std::mutex> lock(publish_mutex_mono_);

        publish_cv_mono_.wait(lock, [this] {
            return frame_ready_to_publish_mono_ || !publisher_running_mono_;
        });

        if (!publisher_running_mono_) {
            break;
        }

        if (frame_ready_to_publish_mono_) {
            std::swap(local_msg, pending_msg_mono_);
            frame_ready_to_publish_mono_ = false;
            lock.unlock();

            auto publish_start = std::chrono::steady_clock::now();
            image_pub_mono_->publish(local_msg);
            publish_time_mono_us_ = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - publish_start).count() * 1e6;
        }
    }
}

// ============================================================
// Auto-Exposure Config Loader
// ============================================================
bool CameraDisplayNode::loadAEConfig(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        RCLCPP_WARN(this->get_logger(), "AE config not found: %s — auto-exposure disabled", path.c_str());
        ae_enabled_ = false;
        return false;
    }

    auto trim = [](std::string& s) {
        s.erase(0, s.find_first_not_of(" \t\r\n"));
        s.erase(s.find_last_not_of(" \t\r\n") + 1);
    };

    std::string line;
    while (std::getline(file, line)) {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#') continue;

        // Support both YAML "key: value" and legacy "key=value"
        auto sep = line.find(':');
        if (sep == std::string::npos) {
            sep = line.find('=');
            if (sep == std::string::npos) continue;
        }

        std::string key = line.substr(0, sep);
        std::string val = line.substr(sep + 1);
        trim(key);
        trim(val);

        if (key == "enabled") ae_enabled_ = (val == "true" || val == "1");
        else if (key == "target_mean") ae_target_mean_ = std::stod(val);
        else if (key == "kp") ae_kp_ = std::stod(val);
        else if (key == "deadband") ae_deadband_ = std::stod(val);
        else if (key == "min_exposure") ae_min_exposure_ = std::stoi(val);
        else if (key == "max_exposure") ae_max_exposure_ = std::stoi(val);
        else if (key == "afps_enabled") afps_enabled_ = (val == "true" || val == "1");
        else if (key.size() > 7 && key.substr(0, 7) == "bracket") {
            // Parse bracket_N_max and bracket_N_fps
            auto sep2 = key.rfind('_');
            if (sep2 != std::string::npos && sep2 > 7) {
                std::string field = key.substr(sep2 + 1);        // "max" or "fps"
                std::string mid   = key.substr(7, sep2 - 7);     // "_N"
                if (mid.size() >= 2 && mid[0] == '_') {
                    int idx = std::stoi(mid.substr(1));
                    if (idx >= static_cast<int>(afps_brackets_.size()))
                        afps_brackets_.resize(idx + 1, {0, 0});
                    if (field == "max") afps_brackets_[idx].max_exp = std::stoi(val);
                    else if (field == "fps") afps_brackets_[idx].fps = std::stoi(val);
                }
            }
        }
    }

    // Sort brackets ascending by max_exp (safety guard against file ordering)
    std::sort(afps_brackets_.begin(), afps_brackets_.end(),
              [](const AfpsBracket & a, const AfpsBracket & b) { return a.max_exp < b.max_exp; });

    RCLCPP_INFO(this->get_logger(), "AE config loaded from %s", path.c_str());
    if (!afps_brackets_.empty())
        RCLCPP_INFO(this->get_logger(), "AFPS: %zu brackets loaded, enabled=%s",
                    afps_brackets_.size(), afps_enabled_ ? "true" : "false");
    return true;
}

// ============================================================
// Adaptive Frame Rate: Apply new frame rate via Arducam ioctl
// ============================================================
void CameraDisplayNode::setFrameRate(int fps) {
    struct v4l2_control ctrl{};
    ctrl.id    = 0x00981906;
    ctrl.value = fps;
    if (ioctl(v4l2_fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "AFPS: set frame_rate=%d failed: %s", fps, strerror(errno));
    } else {
        RCLCPP_INFO(this->get_logger(), "AFPS: frame_rate → %d fps (exposure=%d)",
                    fps, current_exposure_.load());
    }
    afps_active_fps_ = fps;
}

// ============================================================
// Auto-Exposure Thread Loop (P-Controller)
// ============================================================
void CameraDisplayNode::autoExposureThreadLoop() {
    // Seed accumulator from the initial exposure so we don't start from 0
    ae_exposure_acc_ = static_cast<double>(current_exposure_.load());

    while (ae_running_) {
        std::unique_lock<std::mutex> lock(ae_mutex_);
        ae_cv_.wait(lock, [this] { return ae_frame_ready_ || !ae_running_; });
        if (!ae_running_) break;

        double mean = ae_current_mean_;
        ae_frame_ready_ = false;
        lock.unlock();

        double error = ae_target_mean_ - mean;

        // Deadband: ignore small errors to prevent chatter around setpoint
        if (std::abs(error) <= ae_deadband_) continue;

        ae_exposure_acc_ += ae_kp_ * error;
        ae_exposure_acc_ = std::clamp(ae_exposure_acc_,
                                      static_cast<double>(ae_min_exposure_),
                                      static_cast<double>(ae_max_exposure_));
        int clamped = static_cast<int>(std::round(ae_exposure_acc_));
        current_exposure_.store(clamped);

        struct v4l2_control ctrl{};
        ctrl.id = V4L2_CID_EXPOSURE;
        ctrl.value = clamped;
        if (ioctl(v4l2_fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "AE: ioctl set exposure failed: %s", strerror(errno));
        }

        // Adaptive frame rate: pick first bracket whose max_exp >= new exposure
        if (afps_enabled_ && !afps_brackets_.empty()) {
            int target_fps = afps_brackets_.back().fps;
            for (const auto & b : afps_brackets_) {
                if (clamped <= b.max_exp) { target_fps = b.fps; break; }
            }
            if (target_fps != afps_active_fps_) {
                setFrameRate(target_fps);
            }
        }
    }
}

// ============================================================
// Main Entry Point
// ============================================================
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraDisplayNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
