// lccv_publisher_node.cpp
// ROS2 node that captures video from LCCV and publishes frames
// Optimized with async publishing via condition variable (event-driven)

#include "lccv_camera_test/lccv_publisher_node.h"

// ============================================================
// Constructor
// ============================================================
LccvPublisherNode::LccvPublisherNode() : Node("lccv_publisher_node") {
    // Declare ROS2 parameters
    camera_index_ = this->declare_parameter<int>("camera_index", 0);
    width_ = this->declare_parameter<int>("width", 728);
    height_ = this->declare_parameter<int>("height", 544);
    framerate_ = this->declare_parameter<int>("framerate", 20);

    RCLCPP_INFO(this->get_logger(), "Initializing LCCV Camera Publisher");
    RCLCPP_INFO(this->get_logger(), "  Camera Index: %d", camera_index_);
    RCLCPP_INFO(this->get_logger(), "  Resolution: %dx%d", width_, height_);
    RCLCPP_INFO(this->get_logger(), "  Framerate: %d Hz", framerate_);

    // Initialize camera object
    camera_ = std::make_unique<lccv::PiCamera>();

    // Configure camera options
    camera_->options->camera = camera_index_;
    camera_->options->video_width = width_;
    camera_->options->video_height = height_;
    camera_->options->framerate = framerate_;
    camera_->options->verbose = false;  // Reduce noise in logs

    // Manual exposure and gain (disable auto)
    camera_->options->shutter = 3000;  // 3ms exposure in microseconds
    camera_->options->gain = 2.0;

    // Start video capture
    RCLCPP_INFO(this->get_logger(), "Starting video capture...");
    if (!camera_->startVideo()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start video capture!");
        throw std::runtime_error("Failed to start LCCV video capture");
    }

    // Create publisher
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/image_display", rclcpp::SensorDataQoS());

    // Pre-allocate message buffers for double buffering
    const size_t frame_size = static_cast<size_t>(width_) * static_cast<size_t>(height_) * 3;
    reusable_msg_.data.reserve(frame_size);
    pending_msg_.data.reserve(frame_size);
    frame_ready_to_publish_ = false;

    // Initialize FPS tracking
    last_frame_time_ = std::chrono::steady_clock::now();
    last_log_time_ = last_frame_time_;
    smoothed_fps_ = 0.0;
    frames_since_log_ = 0;

    // Start event-driven publisher thread
    // Uses condition variable for instant notification (zero CPU when idle)
    // This removes publishing overhead from the camera callback thread
    publisher_running_ = true;
    publisher_thread_ = std::thread(&LccvPublisherNode::publisherThreadLoop, this);

    // Calculate timer period: 1 second / framerate, in milliseconds
    // Add small margin to ensure we don't miss frames
    int timer_period_ms = static_cast<int>((1000.0 / framerate_) * 0.95);
    if (timer_period_ms < 10) timer_period_ms = 10;  // Minimum 10ms

    RCLCPP_INFO(this->get_logger(), "Timer period: %d ms (event-driven async publish enabled)",
                timer_period_ms);

    // Create timer for frame capture callback
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        std::bind(&LccvPublisherNode::frameCaptureCb, this));

    RCLCPP_INFO(this->get_logger(), "LCCV publisher node ready");
}

// ============================================================
// Destructor
// ============================================================
LccvPublisherNode::~LccvPublisherNode() {
    // Stop publisher thread
    if (publisher_running_) {
        publisher_running_ = false;
        publish_cv_.notify_one();  // Wake up thread so it can exit
        if (publisher_thread_.joinable()) {
            publisher_thread_.join();
        }
    }

    if (camera_) {
        RCLCPP_INFO(this->get_logger(), "Stopping video capture...");
        camera_->stopVideo();
    }

    logFinalStats();
}

// ============================================================
// Publisher Thread Loop (Event-Driven)
// ============================================================
// This thread waits on a condition variable and publishes frames instantly
// when notified by the camera callback. Uses ZERO CPU when idle.
void LccvPublisherNode::publisherThreadLoop() {
    while (publisher_running_) {
        std::unique_lock<std::mutex> lock(publish_mutex_);

        // Wait for notification (blocks with zero CPU usage)
        publish_cv_.wait(lock, [this] {
            return frame_ready_to_publish_ || !publisher_running_;
        });

        // Exit if shutting down
        if (!publisher_running_) {
            break;
        }

        // Publish frame
        if (frame_ready_to_publish_) {
            image_pub_->publish(pending_msg_);
            frame_ready_to_publish_ = false;
        }
    }
}

// ============================================================
// Frame Capture Timer Callback
// ============================================================
void LccvPublisherNode::frameCaptureCb() {
    auto callback_start = std::chrono::steady_clock::now();

    // Attempt to get frame from LCCV (with 1 second timeout)
    cv::Mat frame;
    if (!camera_->getVideoFrame(frame, 1000)) {
        frame_timeouts_++;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Frame timeout (total timeouts: %u)", frame_timeouts_.load());
        return;
    }

    // Track frame timing
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_frame_time_).count();
    last_frame_time_ = now;

    // Update smoothed FPS
    if (dt > 0.0) {
        double inst_fps = 1.0 / dt;
        double alpha = 0.1;  // Smoothing factor
        smoothed_fps_ = (smoothed_fps_ == 0.0) ? inst_fps :
                       (alpha * inst_fps + (1.0 - alpha) * smoothed_fps_);
    }

    frames_since_log_++;
    total_frames_++;

    // Log FPS every 5 seconds
    double time_since_log = std::chrono::duration<double>(now - last_log_time_).count();
    if (time_since_log >= 5.0) {
        uint32_t total = total_frames_.load();
        uint32_t skipped = frames_skipped_.load();
        uint32_t timeouts = frame_timeouts_.load();
        double publish_rate = (total > 0) ? (100.0 * (total - skipped) / total) : 0.0;

        RCLCPP_INFO(this->get_logger(),
                   "FPS: %.1f | Pub Rate: %.0f%% | Skipped: %u | Timeouts: %u",
                   smoothed_fps_, publish_rate, skipped, timeouts);
        frames_since_log_ = 0;
        last_log_time_ = now;
    }

    // ============================================================
    // Prepare Message for Async Publish
    // ============================================================
    // This section prepares the ROS message but does NOT publish it.
    // Publishing happens in publisherThreadLoop() to avoid blocking camera.
    // Key optimization: Reduces callback overhead significantly
    // ============================================================

    // Convert frame to message
    matToImageMsg(frame, reusable_msg_);
    reusable_msg_.header.stamp = this->now();
    reusable_msg_.header.frame_id = "camera_link";

    // Notify publisher thread (event-driven, instant wakeup)
    {
        std::lock_guard<std::mutex> lock(publish_mutex_);
        if (!frame_ready_to_publish_) {
            // Publisher is ready, swap and notify
            std::swap(pending_msg_, reusable_msg_);
            frame_ready_to_publish_ = true;
            publish_cv_.notify_one();
        } else {
            // Publisher still busy with previous frame, skip this one
            frames_skipped_++;
        }
    }

    // Measure callback execution time
    auto callback_end = std::chrono::steady_clock::now();
    double callback_time_us = std::chrono::duration<double>(
        callback_end - callback_start).count() * 1e6;

    // Warn if callback is slow (>10ms)
    if (callback_time_us > 10000.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Slow callback: %.1f µs", callback_time_us);
    }
}

// ============================================================
// Convert OpenCV Mat to ROS2 Image Message (Optimized)
// ============================================================
// Direct memcpy approach without intermediate allocations
// Pre-allocated buffer allows reuse across frames
void LccvPublisherNode::matToImageMsg(const cv::Mat& frame,
                                     sensor_msgs::msg::Image& msg) {
    // Set image metadata
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = frame.cols * 3;  // Assuming 3-channel BGR

    // Resize data buffer if needed
    const size_t frame_size = msg.step * frame.rows;
    if (msg.data.size() != frame_size) {
        msg.data.resize(frame_size);
    }

    // Fast path: single memcpy if stride matches
    if (frame.isContinuous() && frame.step == msg.step) {
        std::memcpy(msg.data.data(), frame.data, frame_size);
    } else {
        // Slow path: row-by-row copy (for non-standard stride)
        uint8_t* dst = msg.data.data();
        const uint8_t* src = frame.data;
        for (int r = 0; r < frame.rows; ++r) {
            std::memcpy(dst + r * msg.step, src + r * frame.step, msg.step);
        }
    }
}

// ============================================================
// Log Final Statistics
// ============================================================
void LccvPublisherNode::logFinalStats() {
    RCLCPP_INFO(this->get_logger(), "=== LCCV Publisher Statistics ===");
    uint32_t total = total_frames_.load();
    uint32_t skipped = frames_skipped_.load();
    uint32_t timeouts = frame_timeouts_.load();
    uint32_t published = total - skipped;

    RCLCPP_INFO(this->get_logger(), "Total frames captured: %u", total);
    RCLCPP_INFO(this->get_logger(), "Frames published: %u", published);
    RCLCPP_INFO(this->get_logger(), "Frames skipped (publisher busy): %u", skipped);
    RCLCPP_INFO(this->get_logger(), "Frame timeouts: %u", timeouts);
    RCLCPP_INFO(this->get_logger(), "Publishing mode: Event-driven (condition variable, zero CPU idle)");
    RCLCPP_INFO(this->get_logger(), "Final smoothed FPS: %.1f", smoothed_fps_);

    if (total > 0) {
        double publish_rate = (100.0 * published) / total;
        RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f%% (%u/%u frames)",
                   publish_rate, published, total);
    }
    if ((total + timeouts) > 0) {
        double timeout_rate = (100.0 * timeouts) / (total + timeouts);
        RCLCPP_INFO(this->get_logger(), "Timeout rate: %.2f%%", timeout_rate);
    }
}

// ============================================================
// Main Entry Point
// ============================================================
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LccvPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
