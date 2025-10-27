// camera_display.cpp
// ROS2 node that captures video from libcamera, synced with Pico triggers.
// Integrates with SerialReader to match frames with hardware trigger events.
// Minimal overhead: request completion callback handles everything.

#include "camera_display_node/camera_display_node.h"

#include <libcamera/formats.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <rclcpp/executors/multi_threaded_executor.hpp>

namespace {

size_t planeHeightForFormat(const libcamera::PixelFormat & format,
                            size_t plane_index,
                            size_t full_height) {
    if (format == libcamera::formats::YUV420) {
        return plane_index == 0 ? full_height : full_height / 2;
    }
    return full_height;
}

uint8_t clampToByte(int value) {
    return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

bool convertYuv420ToRgb(const MappedBuffer & buffer,
                        size_t width,
                        size_t height,
                        uint8_t * dst,
                        size_t dst_step,
                        const rclcpp::Logger & logger) {
    if (buffer.planes.size() < 3) {
        RCLCPP_ERROR(logger, "YUV420 buffer missing planes (%zu)", buffer.planes.size());
        return false;
    }

    const auto * y_plane = static_cast<const uint8_t *>(buffer.planes[0].addr);
    const auto * u_plane = static_cast<const uint8_t *>(buffer.planes[1].addr);
    const auto * v_plane = static_cast<const uint8_t *>(buffer.planes[2].addr);

    const size_t stride_y = buffer.planes[0].stride;
    const size_t stride_u = buffer.planes[1].stride;
    const size_t stride_v = buffer.planes[2].stride;

    if (stride_y == 0 || stride_u == 0 || stride_v == 0) {
        RCLCPP_ERROR(logger, "Invalid plane stride (Y=%zu, U=%zu, V=%zu)",
                     stride_y, stride_u, stride_v);
        return false;
    }

    for (size_t row = 0; row < height; ++row) {
        const uint8_t * y_row = y_plane + row * stride_y;
        const uint8_t * u_row = u_plane + (row / 2) * stride_u;
        const uint8_t * v_row = v_plane + (row / 2) * stride_v;
        uint8_t * dst_row = dst + row * dst_step;

        for (size_t col = 0; col < width; ++col) {
            const size_t uv_col = col / 2;

            int Y = static_cast<int>(y_row[col]) - 16;
            int U = static_cast<int>(u_row[uv_col]) - 128;
            int V = static_cast<int>(v_row[uv_col]) - 128;

            if (Y < 0) {
                Y = 0;
            }

            int R = (298 * Y + 409 * V + 128) >> 8;
            int G = (298 * Y - 100 * U - 208 * V + 128) >> 8;
            int B = (298 * Y + 516 * U + 128) >> 8;

            dst_row[3 * col + 0] = clampToByte(R);
            dst_row[3 * col + 1] = clampToByte(G);
            dst_row[3 * col + 2] = clampToByte(B);
        }
    }

    return true;
}

}  // namespace

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
    std::string serial_port = this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    enable_pico_sync_ = this->declare_parameter<bool>("enable_pico_sync", true);

    // Create publishers
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/image_display", rclcpp::SensorDataQoS());
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/imu/data_raw", rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "Initializing camera display node...");
    RCLCPP_INFO(this->get_logger(), "Pico sync: ENABLED");

    if (!initCamera()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
        throw std::runtime_error("Failed to initialize camera");
    }

    RCLCPP_INFO(this->get_logger(), "Camera initialized successfully: %dx%d",
               width_, height_);

    // Initialize FPS tracking
    last_frame_time_ = std::chrono::steady_clock::now();
    last_log_time_ = last_frame_time_;
    smoothed_fps_ = 0.0;
    frames_since_log_ = 0;

    // Initialize Pico serial synchronization if enabled
    if (enable_pico_sync_) {
        initPicoSync(serial_port);
    }
}

// ============================================================
// Destructor
// ============================================================
CameraDisplayNode::~CameraDisplayNode() {
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
    RCLCPP_INFO(this->get_logger(), "Using camera: %s", camera_->id().c_str());

    // Configure as RGB888 viewfinder
    std::unique_ptr<libcamera::CameraConfiguration> config =
        camera_->generateConfiguration({ libcamera::StreamRole::Viewfinder });
    if (!config) {
        RCLCPP_ERROR(this->get_logger(), "generateConfiguration failed");
        return false;
    }

    libcamera::StreamConfiguration & stream_cfg = config->at(0);
    stream_cfg.pixelFormat = libcamera::formats::RGB888;
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
    pixel_format_ = stream_cfg.pixelFormat;

    RCLCPP_INFO(this->get_logger(), "Configured: %dx%d stride=%d format=%s",
                width_, height_, stride_, pixel_format_.toString().c_str());

    if (pixel_format_ != libcamera::formats::RGB888) {
        RCLCPP_WARN(this->get_logger(),
                    "Camera provided pixel format %s (requested RGB888)",
                    pixel_format_.toString().c_str());
    }

    if (pixel_format_ == libcamera::formats::RGB888) {
        requires_conversion_ = false;
        image_encoding_ = "rgb8";
        bytes_per_pixel_ = 3;
    } else if (pixel_format_ == libcamera::formats::BGR888) {
        requires_conversion_ = false;
        image_encoding_ = "bgr8";
        bytes_per_pixel_ = 3;
    } else if (pixel_format_ == libcamera::formats::R8) {
        requires_conversion_ = false;
        image_encoding_ = "mono8";
        bytes_per_pixel_ = 1;
    } else if (pixel_format_ == libcamera::formats::YUV420) {
        requires_conversion_ = true;
        image_encoding_ = "rgb8";
        bytes_per_pixel_ = 3;
        RCLCPP_INFO(this->get_logger(),
                    "Pixel format %s requires conversion to %s",
                    pixel_format_.toString().c_str(), image_encoding_.c_str());
    } else {
        RCLCPP_FATAL(this->get_logger(),
                     "Unsupported pixel format negotiated: %s",
                     pixel_format_.toString().c_str());
        return false;
    }

    if (bytes_per_pixel_ == 0) {
        RCLCPP_FATAL(this->get_logger(), "Invalid bytes-per-pixel negotiated");
        return false;
    }

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
            MappedPlane plane;
            plane.addr = addr;
            plane.length = pl.length;

            const size_t plane_height = planeHeightForFormat(pixel_format_, p, height_);
            if (plane_height == 0) {
                RCLCPP_ERROR(this->get_logger(),
                             "Plane %zu has zero height for format %s",
                             p, pixel_format_.toString().c_str());
                return false;
            }

            if (p == 0 && stride_ != 0) {
                plane.stride = stride_;
            } else {
                plane.stride = plane.length / plane_height;
            }

            if (plane.stride == 0) {
                RCLCPP_ERROR(this->get_logger(),
                             "Computed zero stride for plane %zu (len=%zu, height=%zu)",
                             p, plane.length, plane_height);
                return false;
            }

            const size_t expected_length = plane.stride * plane_height;
            if (expected_length != plane.length) {
                RCLCPP_WARN(this->get_logger(),
                            "Plane %zu length mismatch: expected %zu, got %zu",
                            p, expected_length, plane.length);
            }

            mb.planes[p] = plane;
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

    RCLCPP_INFO(this->get_logger(), "Camera streaming started");
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

    if (serial_sync_->is_calibrated()) {
        RCLCPP_INFO(this->get_logger(), "Pico time synchronization complete");
    } else {
        RCLCPP_WARN(this->get_logger(), "Pico time synchronization timeout");
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

    uint32_t total = frames_received_.load();
    if (total > 0) {
        double match_rate = (100.0 * frames_matched_.load()) / total;
        RCLCPP_INFO(this->get_logger(), "Match rate: %.1f%%", match_rate);
    }
}

// ============================================================
// Camera Frame Completion Callback
// ============================================================
void CameraDisplayNode::onRequestCompleted(libcamera::Request * req) {
    if (req->status() == libcamera::Request::RequestCancelled) {
        return;
    }

    frames_received_++;

    // Get frame sequence number
    uint64_t frame_sequence = req->sequence();
    uint16_t frame_id = static_cast<uint16_t>(frame_sequence & 0xFFFF);

    // Log for debugging (first 10 frames)
    static uint32_t debug_count = 0;
    if (debug_count < 10) {
        RCLCPP_INFO(this->get_logger(),
                   "Frame seq=%lu, frame_id=%u, expected=%u, map_size=%zu",
                   frame_sequence, frame_id, expected_frame_id_.load(),
                   trigger_map_.size());
        debug_count++;
    }

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

        // Log average FPS and sync stats every second
        if (std::chrono::duration<double>(now - last_log_time_).count() >= 1.0) {
            double avg_fps = frames_since_log_ /
                std::chrono::duration<double>(now - last_log_time_).count();

            if (enable_pico_sync_) {
                uint32_t matched = frames_matched_.load();
                uint32_t received = frames_received_.load();
                double match_rate = (received > 0) ? (100.0 * matched / received) : 0.0;
                RCLCPP_INFO(this->get_logger(),
                           "FPS: %.1f | Matched: %u/%u (%.1f%%) | Drops: %u",
                           smoothed_fps_, matched, received, match_rate,
                           frame_drops_.load());
            } else {
                RCLCPP_INFO(this->get_logger(), "FPS: %.1f (avg: %.1f)",
                           smoothed_fps_, avg_fps);
            }

            frames_since_log_ = 0;
            last_log_time_ = now;
        }

        // --- Publish to ROS2 Topic ---
        auto img_msg = std::make_unique<sensor_msgs::msg::Image>();

        // Use trigger timestamp if Pico sync enabled, otherwise use current time
        if (enable_pico_sync_) {
            // Use latest trigger timestamp (Pico frames come at 20Hz, camera at ~20Hz)
            // They should be approximately synchronized
            {
                std::lock_guard<std::mutex> lock(trigger_map_mutex_);
                if (latest_trigger_time_.nanoseconds() > 0) {
                    img_msg->header.stamp = latest_trigger_time_;
                    frames_matched_++;
                } else {
                    img_msg->header.stamp = this->now();
                }
            }
        } else {
            img_msg->header.stamp = this->now();
        }

        img_msg->header.frame_id = "camera_link";
        img_msg->width  = width_;
        img_msg->height = height_;
        img_msg->encoding = image_encoding_;
        img_msg->is_bigendian = false;
        img_msg->step = static_cast<uint32_t>(width_) * static_cast<uint32_t>(bytes_per_pixel_);

        if (img_msg->step == 0) {
            RCLCPP_ERROR(this->get_logger(), "Image step resolved to zero for format %s",
                         pixel_format_.toString().c_str());
            goto requeue;
        }

        const size_t dst_step = static_cast<size_t>(img_msg->step);
        img_msg->data.resize(dst_step * static_cast<size_t>(height_));

        uint8_t * dst = img_msg->data.data();

        const MappedBuffer & mapped = mit->second;
        if (requires_conversion_) {
            bool converted = false;
            if (pixel_format_ == libcamera::formats::YUV420) {
                converted = convertYuv420ToRgb(mapped, width_, height_, dst, dst_step,
                                               this->get_logger());
            }

            if (!converted) {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to convert frame from pixel format %s",
                             pixel_format_.toString().c_str());
                goto requeue;
            }
        } else {
            if (mapped.planes.empty() || mapped.planes[0].addr == nullptr) {
                RCLCPP_ERROR(this->get_logger(), "Primary plane mapping missing");
                goto requeue;
            }

            const uint8_t * src = static_cast<const uint8_t *>(mapped.planes[0].addr);
            const size_t src_stride = mapped.planes[0].stride != 0 ?
                                      mapped.planes[0].stride : stride_;

            if (src_stride == 0) {
                RCLCPP_ERROR(this->get_logger(),
                             "Source stride resolved to zero for format %s",
                             pixel_format_.toString().c_str());
                goto requeue;
            }

            for (size_t r = 0; r < static_cast<size_t>(height_); ++r) {
                std::memcpy(dst + r * dst_step, src + r * src_stride, dst_step);
            }
        }

        image_pub_->publish(std::move(img_msg));
    }

requeue:
    // Requeue request for next frame
    req->reuse(libcamera::Request::ReuseFlag::ReuseBuffers);
    if (camera_->queueRequest(req) < 0) {
        RCLCPP_ERROR(this->get_logger(), "re-queueRequest failed");
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
