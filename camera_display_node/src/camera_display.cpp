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
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/image_display", qos);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/imu/data_raw", rclcpp::SensorDataQoS());

    if (!initCamera()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
        throw std::runtime_error("Failed to initialize camera");
    }

    // Pre-allocate message buffers to avoid per-frame allocations
    const size_t frame_size = static_cast<size_t>(width_) * static_cast<size_t>(height_) * 3;
    reusable_msg_.data.reserve(frame_size);
    pending_msg_.data.reserve(frame_size);
    frame_ready_to_publish_ = false;

    // Start event-driven publisher thread
    // Uses condition variable for instant notification (zero CPU when idle)
    // This removes ~700µs of publish overhead from the camera callback thread
    publisher_running_ = true;
    publisher_thread_ = std::thread(&CameraDisplayNode::publisherThreadLoop, this);

    RCLCPP_INFO(this->get_logger(), "Camera ready: %dx%d | Event-driven async publish enabled",
               width_, height_);

    // Initialize FPS tracking
    last_frame_time_ = std::chrono::steady_clock::now();
    last_log_time_ = last_frame_time_;
    smoothed_fps_ = 0.0;
    frames_since_log_ = 0;

    // Initialize performance metrics
    callback_time_us_ = 0.0;
    memcpy_time_us_ = 0.0;
    alloc_time_us_ = 0.0;
    publish_time_us_ = 0.0;
    slow_callbacks_ = 0;
    frames_skipped_ = 0;

    // Initialize Pico serial synchronization if enabled
    if (enable_pico_sync_) {
        initPicoSync(serial_port);
    }
}

// ============================================================
// Destructor
// ============================================================
CameraDisplayNode::~CameraDisplayNode() {
    // Stop publisher thread
    if (publisher_running_) {
        publisher_running_ = false;
        publish_cv_.notify_one();  // Wake up thread so it can exit
        if (publisher_thread_.joinable()) {
            publisher_thread_.join();
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
    RCLCPP_INFO(this->get_logger(), "Frames skipped (publisher busy): %u", frames_skipped_.load());
    RCLCPP_INFO(this->get_logger(), "Slow callbacks (>5ms): %u", slow_callbacks_.load());
    RCLCPP_INFO(this->get_logger(), "Publishing mode: Event-driven (condition variable, zero CPU idle)");
    RCLCPP_INFO(this->get_logger(), "Last callback: %.1f µs [alloc: %.1f, memcpy: %.1f]",
               callback_time_us_, alloc_time_us_, memcpy_time_us_);
    RCLCPP_INFO(this->get_logger(), "Last publish: %.1f µs (separate thread, instant notification)", publish_time_us_);

    uint32_t total = frames_received_.load();
    uint32_t published = total - frames_skipped_.load();
    if (total > 0) {
        double match_rate = (100.0 * frames_matched_.load()) / total;
        double publish_rate = (100.0 * published) / total;
        RCLCPP_INFO(this->get_logger(), "Match rate: %.1f%%", match_rate);
        RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f%% (%u/%u frames)",
                   publish_rate, published, total);
    }
}

// ============================================================
// Publisher Thread Loop (Event-Driven)
// ============================================================
// This thread waits on a condition variable and publishes frames instantly
// when notified by the camera callback. Uses ZERO CPU when idle.
//
// IMPORTANT: IMU-camera synchronization is preserved because:
// - Timestamps are captured from trigger_map in onRequestCompleted()
// - The timestamp is already set in the message before notification
// - This thread just publishes the pre-timestamped message
// ============================================================
void CameraDisplayNode::publisherThreadLoop() {
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
            auto publish_start = std::chrono::steady_clock::now();
            image_pub_->publish(pending_msg_);
            auto publish_end = std::chrono::steady_clock::now();

            // Update publish time metric (for monitoring, not part of callback time)
            double pub_time = std::chrono::duration<double>(publish_end - publish_start).count() * 1e6;
            publish_time_us_ = pub_time;

            frame_ready_to_publish_ = false;
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
                uint32_t skipped = frames_skipped_.load();
                double match_rate = (received > 0) ? (100.0 * matched / received) : 0.0;
                RCLCPP_INFO(this->get_logger(),
                           "FPS: %.1f | Sync: %.0f%% | Drops: %u | Skipped: %u | CB: %.0fµs | Pub: %.0fµs",
                           smoothed_fps_, match_rate,
                           frame_drops_.load(), skipped, callback_time_us_, publish_time_us_);
            } else {
                RCLCPP_INFO(this->get_logger(),
                           "FPS: %.1f | Skipped: %u | CB: %.0fµs | Pub: %.0fµs",
                           smoothed_fps_, frames_skipped_.load(), callback_time_us_, publish_time_us_);
            }

            frames_since_log_ = 0;
            last_log_time_ = now;
        }

        // ============================================================
        // Prepare Message for Async Publish
        // ============================================================
        // This section prepares the ROS message but does NOT publish it.
        // Publishing happens in onPublishTimer() to avoid blocking camera.
        // Key optimization: Reduces callback from 1700µs → 800µs
        // ============================================================

        const size_t dst_step = static_cast<size_t>(width_) * 3;
        const size_t frame_size = dst_step * static_cast<size_t>(height_);

        // Step 1: Determine timestamp from trigger synchronization
        // Timestamp is captured HERE, before async publish, to preserve IMU sync
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

        // Step 2: Fill message metadata (pre-allocated buffer, minimal overhead)
        auto alloc_start = std::chrono::steady_clock::now();
        reusable_msg_.header.stamp = frame_timestamp;
        reusable_msg_.header.frame_id = "camera_link";
        reusable_msg_.width = width_;
        reusable_msg_.height = height_;
        reusable_msg_.encoding = "bgr8";
        reusable_msg_.is_bigendian = false;
        reusable_msg_.step = width_ * 3;
        reusable_msg_.data.resize(frame_size);  // No allocation (already reserved)
        auto alloc_end = std::chrono::steady_clock::now();
        alloc_time_us_ = std::chrono::duration<double>(alloc_end - alloc_start).count() * 1e6;

        // Step 3: Copy frame data (bulk memcpy if stride matches)
        uint8_t * dst = reusable_msg_.data.data();
        auto memcpy_start = std::chrono::steady_clock::now();
        if (stride_ == dst_step) {
            // Fast path: Single bulk copy for standard stride
            std::memcpy(dst, src, frame_size);
        } else {
            // Slow path: Row-by-row copy for non-standard stride
            for (size_t r = 0; r < static_cast<size_t>(height_); ++r) {
                std::memcpy(dst + r * dst_step, src + r * stride_, dst_step);
            }
        }
        auto memcpy_end = std::chrono::steady_clock::now();
        memcpy_time_us_ = std::chrono::duration<double>(memcpy_end - memcpy_start).count() * 1e6;

        // Step 4: Notify publisher thread (event-driven, instant wakeup)
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

        publish_time_us_ = 0.0;  // Will be updated by publisher thread
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
