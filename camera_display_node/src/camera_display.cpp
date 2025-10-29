// camera_display.cpp
// ROS2 node that captures video from libcamera, synced with Pico triggers.
// Integrates with SerialReader to match frames with hardware trigger events.
// Minimal overhead: request completion callback handles everything.

#include "camera_display_node/camera_display_node.h"

#include <libcamera/formats.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <rclcpp/executors/multi_threaded_executor.hpp>

// ============================================================
// Constructor
// ============================================================
CameraDisplayNode::CameraDisplayNode() : Node("camera_display_node"),
                                         enable_pico_sync_(true),
                                         frames_received_(0),
                                         frames_matched_(0),
                                         frame_drops_(0),
                                         expected_frame_id_(0),
                                         trigger_map_max_size_(20)
{
    // Declare parameters
    camera_index_ = this->declare_parameter<int>("camera_index", 0);
    width_ = this->declare_parameter<int>("width", 640);
    height_ = this->declare_parameter<int>("height", 480);
    std::string serial_port = this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
    enable_pico_sync_ = this->declare_parameter<bool>("enable_pico_sync", true);


    rclcpp::QoS qos(
    rclcpp::QoSInitialization(
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        1 // keep only the newest frame
    )
);
qos.best_effort();
qos.durability_volatile();
    // Create publishers
    image_pub_mono_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/image_mono", qos);
    image_pub_color_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/image_color", qos);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/imu/data_raw", rclcpp::SensorDataQoS());

    if (!initCamera()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
        throw std::runtime_error("Failed to initialize camera");
    }

    // Pre-allocate message buffers to avoid per-frame allocations
    const size_t frame_size_color = static_cast<size_t>(width_) * static_cast<size_t>(height_) * 3;
    const size_t frame_size_mono = static_cast<size_t>(width_) * static_cast<size_t>(height_);

    reusable_msg_color_.data.reserve(frame_size_color);
    pending_msg_color_.data.reserve(frame_size_color);
    frame_ready_to_publish_color_ = false;

    reusable_msg_mono_.data.reserve(frame_size_mono);
    pending_msg_mono_.data.reserve(frame_size_mono);
    pending_bgr_for_mono_.data.reserve(frame_size_color);  // BGR source for conversion
    frame_ready_to_publish_mono_ = false;

    // Start event-driven publisher threads
    // Mono @ 20 Hz for VIO, Color @ 2 Hz for visualization
    publisher_running_mono_ = true;
    publisher_running_color_ = true;
    publisher_thread_mono_ = std::thread(&CameraDisplayNode::publisherThreadLoopMono, this);
    publisher_thread_color_ = std::thread(&CameraDisplayNode::publisherThreadLoopColor, this);

    RCLCPP_INFO(this->get_logger(), "Camera ready: %dx%d | Mono@20Hz + Color@2Hz (event-driven)",
               width_, height_);

    // Initialize FPS tracking
    last_frame_time_ = std::chrono::steady_clock::now();
    last_log_time_ = last_frame_time_;
    smoothed_fps_ = 0.0;
    frames_since_log_ = 0;

    // Initialize performance metrics
    callback_time_us_ = 0.0;
    memcpy_time_us_ = 0.0;
    convert_time_us_ = 0.0;
    alloc_time_us_ = 0.0;
    publish_time_mono_us_ = 0.0;
    publish_time_color_us_ = 0.0;
    slow_callbacks_ = 0;
    frames_skipped_mono_ = 0;
    frames_skipped_color_ = 0;
    frame_counter_ = 0;

    // Initialize Pico serial synchronization if enabled
    if (enable_pico_sync_) {
        initPicoSync(serial_port);
    }
}

// ============================================================
// Destructor
// ============================================================
CameraDisplayNode::~CameraDisplayNode() {
    // Stop publisher threads
    if (publisher_running_mono_) {
        publisher_running_mono_ = false;
        publish_cv_mono_.notify_one();
        if (publisher_thread_mono_.joinable()) {
            publisher_thread_mono_.join();
        }
    }

    if (publisher_running_color_) {
        publisher_running_color_ = false;
        publish_cv_color_.notify_one();
        if (publisher_thread_color_.joinable()) {
            publisher_thread_color_.join();
        }
    }

    if (serial_sync_) {
        serial_sync_->stop();
    }

    if (camera_) {
        camera_->stop();

        for (auto & kv : mappings_) {
            for (auto & pl : kv.second.planes) {
                if (pl.addr && pl.length) {
                    munmap(pl.addr, pl.length);
                }
            }
        }
        mappings_.clear();

        camera_->release();
        camera_.reset();
    }

    if (camera_manager_) {
        camera_manager_->stop();
        camera_manager_.reset();
    }

    // Log final synchronization statistics
    if (enable_pico_sync_) {
        logSyncStats();
    }

    RCLCPP_INFO(this->get_logger(), "Camera display node shut down");
}

// ============================================================
// Camera Initialization
// ============================================================
bool CameraDisplayNode::initCamera() {
    // Initialize camera manager
    camera_manager_ = std::make_unique<libcamera::CameraManager>();
    if (camera_manager_->start()) {
        RCLCPP_ERROR(this->get_logger(), "CameraManager start failed");
        return false;
    }
    if (camera_manager_->cameras().empty()) {
        RCLCPP_ERROR(this->get_logger(), "No cameras found");
        return false;
    }
    if (static_cast<size_t>(camera_index_) >= camera_manager_->cameras().size()) {
        RCLCPP_ERROR(this->get_logger(), "Camera index %d out of range", camera_index_);
        return false;
    }

    camera_ = camera_manager_->cameras()[camera_index_];
    if (camera_->acquire()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to acquire camera");
        return false;
    }

    // Configure as RGB888 viewfinder
    std::unique_ptr<libcamera::CameraConfiguration> config =
        camera_->generateConfiguration({ libcamera::StreamRole::Viewfinder });
    if (!config) {
        RCLCPP_ERROR(this->get_logger(), "generateConfiguration failed");
        return false;
    }

    libcamera::StreamConfiguration & stream_cfg = config->at(0);
    stream_cfg.pixelFormat = libcamera::formats::BGR888;
    stream_cfg.size.width  = width_;
    stream_cfg.size.height = height_;

    if (config->validate() == libcamera::CameraConfiguration::Invalid) {
        RCLCPP_ERROR(this->get_logger(), "Invalid configuration");
        return false;
    }
    if (camera_->configure(config.get()) < 0) {
        RCLCPP_ERROR(this->get_logger(), "camera->configure failed");
        return false;
    }

    width_  = stream_cfg.size.width;
    height_ = stream_cfg.size.height;
    stride_ = stream_cfg.stride;
    stream_ = stream_cfg.stream();

    // Allocate frame buffers
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
    if (allocator_->allocate(stream_) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Buffer allocation failed");
        return false;
    }
    const auto & bufs = allocator_->buffers(stream_);
    if (bufs.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No buffers allocated");
        return false;
    }

    // mmap planes
    mappings_.reserve(bufs.size());
    for (const auto & uptr : bufs) {
        libcamera::FrameBuffer * fb = uptr.get();
        MappedBuffer mb;
        mb.planes.resize(fb->planes().size());
        for (size_t p = 0; p < fb->planes().size(); ++p) {
            const libcamera::FrameBuffer::Plane & pl = fb->planes()[p];
            void * addr = mmap(nullptr, pl.length, PROT_READ | PROT_WRITE,
                              MAP_SHARED, pl.fd.get(), pl.offset);
            if (addr == MAP_FAILED) {
                RCLCPP_ERROR(this->get_logger(), "mmap failed on plane %zu", p);
                return false;
            }
            mb.planes[p] = { addr, pl.length };
        }
        mappings_.emplace(fb, std::move(mb));
    }

    // Create request pool
    requests_.reserve(bufs.size());
    for (const auto & uptr : bufs) {
        libcamera::FrameBuffer * fb = uptr.get();
        auto req = camera_->createRequest();
        if (!req) {
            RCLCPP_ERROR(this->get_logger(), "createRequest failed");
            return false;
        }
        if (req->addBuffer(stream_, fb) < 0) {
            RCLCPP_ERROR(this->get_logger(), "addBuffer failed");
            return false;
        }

        requests_.push_back(std::move(req));
    }

    // Connect completion signal to callback
    camera_->requestCompleted.connect(
        camera_.get(),
        std::bind(&CameraDisplayNode::onRequestCompleted, this, std::placeholders::_1)
    );

    // Optional: disable AE/AWB for consistent exposure
    libcamera::ControlList controls(camera_->controls());
    controls.set(libcamera::controls::AeEnable, false);
    controls.set(libcamera::controls::AwbEnable, false);

    if (camera_->start(&controls) < 0) {
        RCLCPP_ERROR(this->get_logger(), "camera->start failed");
        return false;
    }

    // Queue all requests to start streaming
    for (auto & req : requests_) {
        if (camera_->queueRequest(req.get()) < 0) {
            RCLCPP_ERROR(this->get_logger(), "queueRequest failed");
            return false;
        }
    }

    return true;
}

// ============================================================
// Pico Synchronization Initialization
// ============================================================
void CameraDisplayNode::initPicoSync(const std::string& serial_port) {
    // Create serial sync with IMU and trigger callbacks
    serial_sync_ = std::make_unique<camera_display_node::SerialSync>(
        this,
        serial_port,
        std::bind(&CameraDisplayNode::onImuPacket, this,
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                 std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
                 std::placeholders::_7),
        std::bind(&CameraDisplayNode::onTriggerPacket, this,
                 std::placeholders::_1, std::placeholders::_2)
    );

    // Start serial reader
    serial_sync_->start();

    // Wait for time calibration
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
void CameraDisplayNode::onImuPacket(uint32_t timestamp_us, float ax, float ay, float az,
                                     float gx, float gy, float gz) {
    if (!serial_sync_ || !imu_pub_) return;

    // Create and publish IMU message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = serial_sync_->pico_to_ros_time(timestamp_us);
    imu_msg.header.frame_id = "imu_link";

    // Linear acceleration
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    // Angular velocity
    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;

    // Covariances
    imu_msg.orientation_covariance[0] = -1.0;  // No orientation
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[0] = 0.01;

    imu_pub_->publish(imu_msg);
}

// ============================================================
// Trigger Packet Callback
// ============================================================
void CameraDisplayNode::onTriggerPacket(uint32_t timestamp_us, uint16_t frame_id) {
    if (!serial_sync_) return;

    // Convert Pico time to ROS time
    rclcpp::Time trigger_time;
    if (serial_sync_->is_calibrated()) {
        // Use the conversion function (time offset already applied)
        trigger_time = serial_sync_->pico_to_ros_time(timestamp_us);
    } else {
        trigger_time = this->now();
    }

    // Store trigger in map and update latest
    {
        std::lock_guard<std::mutex> lock(trigger_map_mutex_);
        trigger_map_[frame_id] = trigger_time;
        latest_trigger_time_ = trigger_time;

        // Prune old entries if map is too large
        if (trigger_map_.size() > trigger_map_max_size_) {
            // Remove oldest entries (lower frame_ids)
            auto it = trigger_map_.begin();
            for (size_t i = 0; i < 5 && it != trigger_map_.end(); ++i) {
                it = trigger_map_.erase(it);
            }
        }
    }

    // Update expected frame counter
    expected_frame_id_ = frame_id;
}

// ============================================================
// Frame Timestamp Lookup
// ============================================================
rclcpp::Time CameraDisplayNode::getFrameTimestamp(uint16_t frame_id) {
    std::lock_guard<std::mutex> lock(trigger_map_mutex_);
    auto it = trigger_map_.find(frame_id);
    if (it != trigger_map_.end()) {
        return it->second;
    }
    // Fallback: use latest trigger or current time
    return this->now();
}

// ============================================================
// Synchronization Statistics
// ============================================================
void CameraDisplayNode::logSyncStats() {
    RCLCPP_INFO(this->get_logger(), "=== Synchronization Statistics ===");
    RCLCPP_INFO(this->get_logger(), "Total frames received: %u", frames_received_.load());
    RCLCPP_INFO(this->get_logger(), "Frames matched to trigger: %u", frames_matched_.load());
    RCLCPP_INFO(this->get_logger(), "Frame drops detected: %u", frame_drops_.load());
    RCLCPP_INFO(this->get_logger(), "Frames skipped - Mono: %u, Color: %u",
               frames_skipped_mono_.load(), frames_skipped_color_.load());
    RCLCPP_INFO(this->get_logger(), "Slow callbacks (>5ms): %u", slow_callbacks_.load());
    RCLCPP_INFO(this->get_logger(), "Publishing mode: Dual stream (Mono@20Hz + Color@2Hz, event-driven)");
    RCLCPP_INFO(this->get_logger(), "Last callback: %.1f µs [alloc: %.1f, convert: %.1f, memcpy: %.1f]",
               callback_time_us_, alloc_time_us_, convert_time_us_, memcpy_time_us_);
    RCLCPP_INFO(this->get_logger(), "Last publish: Mono %.1f µs, Color %.1f µs",
               publish_time_mono_us_, publish_time_color_us_);

    uint32_t total = frames_received_.load();
    uint32_t published_mono = total - frames_skipped_mono_.load();
    uint32_t published_color = (total / 10) - frames_skipped_color_.load();
    if (total > 0) {
        double match_rate = (100.0 * frames_matched_.load()) / total;
        double mono_rate = (100.0 * published_mono) / total;
        double color_rate = (100.0 * published_color * 10) / total;  // Normalize to 100%
        RCLCPP_INFO(this->get_logger(), "Match rate: %.1f%%", match_rate);
        RCLCPP_INFO(this->get_logger(), "Mono publish rate: %.1f%% (%u/%u frames)",
                   mono_rate, published_mono, total);
        RCLCPP_INFO(this->get_logger(), "Color publish rate: %.1f%% (%u/%u expected)",
                   color_rate, published_color, total / 10);
    }
}

// ============================================================
// Mono Publisher Thread Loop (20 Hz, Event-Driven)
// ============================================================
void CameraDisplayNode::publisherThreadLoopMono() {
    while (publisher_running_mono_) {
        std::unique_lock<std::mutex> lock(publish_mutex_mono_);

        // Wait for notification (blocks with zero CPU usage)
        publish_cv_mono_.wait(lock, [this] {
            return frame_ready_to_publish_mono_ || !publisher_running_mono_;
        });

        // Exit if shutting down
        if (!publisher_running_mono_) {
            break;
        }

        // Publish mono frame
        if (frame_ready_to_publish_mono_) {
            auto publish_start = std::chrono::steady_clock::now();
            image_pub_mono_->publish(pending_msg_mono_);
            auto publish_end = std::chrono::steady_clock::now();

            publish_time_mono_us_ = std::chrono::duration<double>(publish_end - publish_start).count() * 1e6;
            frame_ready_to_publish_mono_ = false;
        }
    }
}

// ============================================================
// Color Publisher Thread Loop (2 Hz, Event-Driven)
// ============================================================
void CameraDisplayNode::publisherThreadLoopColor() {
    while (publisher_running_color_) {
        std::unique_lock<std::mutex> lock(publish_mutex_color_);

        // Wait for notification (blocks with zero CPU usage)
        publish_cv_color_.wait(lock, [this] {
            return frame_ready_to_publish_color_ || !publisher_running_color_;
        });

        // Exit if shutting down
        if (!publisher_running_color_) {
            break;
        }

        // Publish color frame
        if (frame_ready_to_publish_color_) {
            auto publish_start = std::chrono::steady_clock::now();
            image_pub_color_->publish(pending_msg_color_);
            auto publish_end = std::chrono::steady_clock::now();

            publish_time_color_us_ = std::chrono::duration<double>(publish_end - publish_start).count() * 1e6;
            frame_ready_to_publish_color_ = false;
        }
    }
}

// ============================================================
// Camera Frame Completion Callback
// ============================================================
void CameraDisplayNode::onRequestCompleted(libcamera::Request * req) {
    auto callback_start = std::chrono::steady_clock::now();

    if (req->status() == libcamera::Request::RequestCancelled) {
        return;
    }

    frames_received_++;

    // Get frame sequence number
    uint64_t frame_sequence = req->sequence();
    uint16_t frame_id = static_cast<uint16_t>(frame_sequence & 0xFFFF);

    // Detect frame drops (if sync enabled)
    if (enable_pico_sync_ && expected_frame_id_ > 0) {
        uint16_t expected = expected_frame_id_ + 1;
        if (frame_id != expected) {
            // Check if it's a drop or just out-of-order
            if ((frame_id > expected) || (frame_id == 0 && expected == 0xFFFF)) {
                uint16_t drop_count = (frame_id > expected) ?
                                     (frame_id - expected) :
                                     (0xFFFF - expected + frame_id + 1);
                frame_drops_ += drop_count;
                RCLCPP_DEBUG(this->get_logger(),
                           "Frame drop detected: expected %u, got %u (dropped %u frames)",
                           expected, frame_id, drop_count);
            }
        }
    }

    // Find buffer for this request
    const auto & buffers_map = req->buffers();
    auto it = buffers_map.find(stream_);
    if (it == buffers_map.end()) {
        RCLCPP_WARN(this->get_logger(), "Request without stream buffer");
        goto requeue;
    }

    {
        libcamera::FrameBuffer * fb = it->second;
        auto mit = mappings_.find(fb);
        if (mit == mappings_.end()) {
            RCLCPP_WARN(this->get_logger(), "Missing mapping for buffer");
            goto requeue;
        }

        const uint8_t * src = static_cast<const uint8_t*>(mit->second.planes[0].addr);

        // --- FPS Calculation ---
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_frame_time_).count();
        last_frame_time_ = now;

        if (dt > 0.0) {
            double inst_fps = 1.0 / dt;
            double alpha = 0.1;  // Smoothing factor
            smoothed_fps_ = (smoothed_fps_ == 0.0) ? inst_fps :
                           (alpha * inst_fps + (1.0 - alpha) * smoothed_fps_);
        }
        frames_since_log_++;

        // Log average FPS and sync stats every 5 seconds
        if (std::chrono::duration<double>(now - last_log_time_).count() >= 5.0) {
            if (enable_pico_sync_) {
                uint32_t matched = frames_matched_.load();
                uint32_t received = frames_received_.load();
                double match_rate = (received > 0) ? (100.0 * matched / received) : 0.0;
                RCLCPP_INFO(this->get_logger(),
                           "FPS: %.1f | Sync: %.0f%% | CB: %.0fµs [Conv:%.0f Mem:%.0f] | Mono: %.0fµs | Color: %.0fµs",
                           smoothed_fps_, match_rate, callback_time_us_,
                           convert_time_us_, memcpy_time_us_,
                           publish_time_mono_us_, publish_time_color_us_);
            } else {
                RCLCPP_INFO(this->get_logger(),
                           "FPS: %.1f | CB: %.0fµs [Conv:%.0f Mem:%.0f] | Mono: %.0fµs | Color: %.0fµs",
                           smoothed_fps_, callback_time_us_,
                           convert_time_us_, memcpy_time_us_,
                           publish_time_mono_us_, publish_time_color_us_);
            }

            frames_since_log_ = 0;
            last_log_time_ = now;
        }

        // ============================================================
        // Prepare Messages for Async Dual Publish
        // ============================================================
        // Mono @ 20 Hz (every frame) + Color @ 2 Hz (every 10th frame)
        // ============================================================

        const size_t dst_step_color = static_cast<size_t>(width_) * 3;
        const size_t dst_step_mono = static_cast<size_t>(width_);
        const size_t frame_size_color = dst_step_color * static_cast<size_t>(height_);
        const size_t frame_size_mono = dst_step_mono * static_cast<size_t>(height_);

        // Determine timestamp from trigger synchronization
        rclcpp::Time frame_timestamp;
        if (enable_pico_sync_) {
            rclcpp::Time frame_time = getFrameTimestamp(frame_id);
            if (frame_time.nanoseconds() > 0) {
                frame_timestamp = frame_time;
                frames_matched_++;
            } else {
                frame_timestamp = this->now();
            }
        } else {
            frame_timestamp = this->now();
        }

        // Frame decimation: publish color every 10th frame (2 Hz @ 20 FPS)
        frame_counter_++;
        bool publish_color_frame = (frame_counter_ % 10 == 0);

        auto alloc_start = std::chrono::steady_clock::now();

        // === MONO MESSAGE (every frame) ===
        reusable_msg_mono_.header.stamp = frame_timestamp;
        reusable_msg_mono_.header.frame_id = "camera_link";
        reusable_msg_mono_.width = width_;
        reusable_msg_mono_.height = height_;
        reusable_msg_mono_.encoding = "mono8";
        reusable_msg_mono_.is_bigendian = false;
        reusable_msg_mono_.step = width_;
        reusable_msg_mono_.data.resize(frame_size_mono);

        // === COLOR MESSAGE (every 10th frame) ===
        if (publish_color_frame) {
            reusable_msg_color_.header.stamp = frame_timestamp;
            reusable_msg_color_.header.frame_id = "camera_link";
            reusable_msg_color_.width = width_;
            reusable_msg_color_.height = height_;
            reusable_msg_color_.encoding = "bgr8";
            reusable_msg_color_.is_bigendian = false;
            reusable_msg_color_.step = width_ * 3;
            reusable_msg_color_.data.resize(frame_size_color);
        }

        auto alloc_end = std::chrono::steady_clock::now();
        alloc_time_us_ = std::chrono::duration<double>(alloc_end - alloc_start).count() * 1e6;

        // === COPY BGR DATA (for color, if needed) ===
        auto memcpy_start = std::chrono::steady_clock::now();
        if (publish_color_frame) {
            uint8_t * dst_color = reusable_msg_color_.data.data();
            if (stride_ == dst_step_color) {
                std::memcpy(dst_color, src, frame_size_color);
            } else {
                for (size_t r = 0; r < static_cast<size_t>(height_); ++r) {
                    std::memcpy(dst_color + r * dst_step_color, src + r * stride_, dst_step_color);
                }
            }
        }
        auto memcpy_end = std::chrono::steady_clock::now();
        memcpy_time_us_ = std::chrono::duration<double>(memcpy_end - memcpy_start).count() * 1e6;

        // === CONVERT BGR → MONO IN CALLBACK (fast fixed-point conversion) ===
        auto convert_start = std::chrono::steady_clock::now();
        uint8_t* dst_mono = reusable_msg_mono_.data.data();

        for (size_t r = 0; r < static_cast<size_t>(height_); ++r) {
            const uint8_t* src_row = src + r * stride_;
            uint8_t* dst_row = dst_mono + r * static_cast<size_t>(width_);

            for (size_t c = 0; c < static_cast<size_t>(width_); ++c) {
                const uint8_t b = src_row[c * 3 + 0];
                const uint8_t g = src_row[c * 3 + 1];
                const uint8_t r_val = src_row[c * 3 + 2];
                // Fixed-point: Y = (77*R + 150*G + 29*B) >> 8
                dst_row[c] = static_cast<uint8_t>((77 * r_val + 150 * g + 29 * b) >> 8);
            }
        }
        auto convert_end = std::chrono::steady_clock::now();
        convert_time_us_ = std::chrono::duration<double>(convert_end - convert_start).count() * 1e6;

        // === NOTIFY MONO PUBLISHER (every frame) ===
        {
            std::lock_guard<std::mutex> lock(publish_mutex_mono_);
            if (!frame_ready_to_publish_mono_) {
                std::swap(pending_msg_mono_, reusable_msg_mono_);
                frame_ready_to_publish_mono_ = true;
                publish_cv_mono_.notify_one();
            } else {
                frames_skipped_mono_++;
            }
        }

        // === NOTIFY COLOR PUBLISHER (every 10th frame) ===
        if (publish_color_frame) {
            std::lock_guard<std::mutex> lock(publish_mutex_color_);
            if (!frame_ready_to_publish_color_) {
                std::swap(pending_msg_color_, reusable_msg_color_);
                frame_ready_to_publish_color_ = true;
                publish_cv_color_.notify_one();
            } else {
                frames_skipped_color_++;
            }
        }
    }

requeue:
    // Requeue request for next frame
    req->reuse(libcamera::Request::ReuseFlag::ReuseBuffers);
    if (camera_->queueRequest(req) < 0) {
        RCLCPP_ERROR(this->get_logger(), "re-queueRequest failed");
    }

    // Measure callback execution time
    auto callback_end = std::chrono::steady_clock::now();
    callback_time_us_ = std::chrono::duration<double>(callback_end - callback_start).count() * 1e6;

    // Track slow callbacks (>5ms)
    if (callback_time_us_ > 5000.0) {
        slow_callbacks_++;
        RCLCPP_DEBUG(this->get_logger(),
                    "Slow callback: %.1f µs (memcpy: %.1f µs)",
                    callback_time_us_, memcpy_time_us_);
    }
}

// ============================================================
// Main Entry Point
// ============================================================
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraDisplayNode>();

    // Use multi-threaded executor to avoid blocking on GUI
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
