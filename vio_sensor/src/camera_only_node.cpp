/**
 * @file camera_only_node.cpp
 * @brief Lightweight camera capture node - publishes raw images ONLY
 *
 * This node does ONE thing: capture camera frames and publish them as fast as possible.
 * No IMU, no sync, no heavy processing - just pure camera capture.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/formats.h>

#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <memory>
#include <unordered_map>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace vio_sensor {

struct MappedPlane {
    void* addr = nullptr;
    size_t length = 0;
};

struct MappedBuffer {
    std::vector<MappedPlane> planes;
};

class CameraOnlyNode : public rclcpp::Node {
public:
    CameraOnlyNode() : Node("camera_only_node"), running_(false), publish_queue_size_(0)
    {
        // Parameters
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);

        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();

        // Publisher with SensorDataQoS (BEST_EFFORT, depth=1)
        auto qos = rclcpp::SensorDataQoS().keep_last(1);
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/image_raw", qos);

        RCLCPP_INFO(this->get_logger(), "Camera Only Node starting...");
        RCLCPP_INFO(this->get_logger(), "  Resolution: %dx%d", width_, height_);
        RCLCPP_INFO(this->get_logger(), "  QoS: SensorDataQoS (BEST_EFFORT, depth=1)");
        RCLCPP_INFO(this->get_logger(), "  Async publish: Using background thread");

        // Start background publish thread
        publish_thread_running_ = true;
        publish_thread_ = std::thread(&CameraOnlyNode::publish_worker, this);

        // Initialize camera
        if (!init_camera()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
            rclcpp::shutdown();
            return;
        }

        // Start camera
        start_camera();

        RCLCPP_INFO(this->get_logger(), "Camera Only Node ready - publishing to /camera/image_raw");
    }

    ~CameraOnlyNode()
    {
        stop_camera();

        // Stop publish thread
        publish_thread_running_ = false;
        publish_cv_.notify_all();
        if (publish_thread_.joinable()) {
            publish_thread_.join();
        }

        // Cleanup mmaps
        for (auto& kv : mappings_) {
            for (auto& pl : kv.second.planes) {
                if (pl.addr && pl.length) {
                    munmap(pl.addr, pl.length);
                }
            }
        }

        if (camera_) {
            camera_->release();
        }

        if (camera_manager_) {
            camera_manager_->stop();
        }
    }

private:
    bool init_camera()
    {
        // Create camera manager
        camera_manager_ = std::make_unique<libcamera::CameraManager>();
        if (camera_manager_->start()) {
            RCLCPP_ERROR(this->get_logger(), "CameraManager start failed");
            return false;
        }

        if (camera_manager_->cameras().empty()) {
            RCLCPP_ERROR(this->get_logger(), "No cameras found");
            return false;
        }

        // Get first camera
        camera_ = camera_manager_->cameras()[0];
        if (camera_->acquire()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire camera");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Using camera: %s", camera_->id().c_str());

        // Configure as RGB888 viewfinder
        config_ = camera_->generateConfiguration({ libcamera::StreamRole::Viewfinder });
        if (!config_) {
            RCLCPP_ERROR(this->get_logger(), "generateConfiguration failed");
            return false;
        }

        libcamera::StreamConfiguration& stream_cfg = config_->at(0);
        stream_cfg.pixelFormat = libcamera::formats::RGB888;
        stream_cfg.size.width = width_;
        stream_cfg.size.height = height_;

        if (config_->validate() == libcamera::CameraConfiguration::Invalid) {
            RCLCPP_ERROR(this->get_logger(), "Invalid camera configuration");
            return false;
        }

        if (camera_->configure(config_.get()) < 0) {
            RCLCPP_ERROR(this->get_logger(), "camera->configure failed");
            return false;
        }

        width_ = stream_cfg.size.width;
        height_ = stream_cfg.size.height;
        stream_ = stream_cfg.stream();

        RCLCPP_INFO(this->get_logger(), "Camera configured: %dx%d, format=%s",
                    width_, height_, stream_cfg.pixelFormat.toString().c_str());

        // Allocate frame buffers
        allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
        if (allocator_->allocate(stream_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Buffer allocation failed");
            return false;
        }

        const auto& bufs = allocator_->buffers(stream_);
        if (bufs.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No buffers allocated");
            return false;
        }

        // mmap each buffer
        for (const auto& uptr : bufs) {
            libcamera::FrameBuffer* fb = uptr.get();
            MappedBuffer mb;
            mb.planes.resize(fb->planes().size());

            for (size_t p = 0; p < fb->planes().size(); ++p) {
                const libcamera::FrameBuffer::Plane& pl = fb->planes()[p];
                int fd = pl.fd.get();
                size_t len = pl.length;
                off_t off = pl.offset;

                void* addr = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, off);
                if (addr == MAP_FAILED) {
                    RCLCPP_ERROR(this->get_logger(), "mmap failed on plane %zu", p);
                    return false;
                }
                mb.planes[p] = { addr, len };
            }
            mappings_.emplace(fb, std::move(mb));
        }

        // Create request pool
        requests_.reserve(bufs.size());
        for (const auto& uptr : bufs) {
            libcamera::FrameBuffer* fb = uptr.get();
            std::unique_ptr<libcamera::Request> req = camera_->createRequest();
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

        // Connect completion signal
        camera_->requestCompleted.connect(camera_.get(),
            [this](libcamera::Request* req) {
                if (req->status() != libcamera::Request::RequestCancelled) {
                    request_completed(req);
                }
            });

        return true;
    }

    void start_camera()
    {
        // Set fixed camera controls for VIO
        libcamera::ControlList controls(camera_->controls());
        controls.set(libcamera::controls::AeEnable, false);
        controls.set(libcamera::controls::AwbEnable, false);
        controls.set(libcamera::controls::ExposureTime, 3000);  // 3ms
        controls.set(libcamera::controls::AnalogueGain, 2.0f);

        if (camera_->start(&controls) < 0) {
            RCLCPP_ERROR(this->get_logger(), "camera->start failed");
            return;
        }

        // Queue all requests
        for (auto& req : requests_) {
            if (camera_->queueRequest(req.get()) < 0) {
                RCLCPP_ERROR(this->get_logger(), "queueRequest failed");
                return;
            }
        }

        running_ = true;
        last_stats_time_ = this->now();
        frame_count_ = 0;

        RCLCPP_INFO(this->get_logger(), "Camera started (AE/AWB disabled for VIO)");
    }

    void stop_camera()
    {
        if (running_ && camera_) {
            camera_->stop();
            running_ = false;
        }
    }

    void request_completed(libcamera::Request* req)
    {
        // CRITICAL: Re-queue IMMEDIATELY to ensure buffer available for next trigger!

        // Find buffer for this stream
        const auto& buffers_map = req->buffers();
        auto it = buffers_map.find(stream_);
        if (it == buffers_map.end()) {
            req->reuse(libcamera::Request::ReuseFlag::ReuseBuffers);
            camera_->queueRequest(req);
            return;
        }

        libcamera::FrameBuffer* fb = it->second;

        // Get mapped memory
        auto mit = mappings_.find(fb);
        if (mit == mappings_.end()) {
            req->reuse(libcamera::Request::ReuseFlag::ReuseBuffers);
            camera_->queueRequest(req);
            return;
        }

        // RGB888 single plane
        void* data = mit->second.planes[0].addr;
        size_t data_size = width_ * height_ * 3;

        // **CRITICAL: Re-queue BEFORE doing memcpy!**
        req->reuse(libcamera::Request::ReuseFlag::ReuseBuffers);
        if (camera_->queueRequest(req) < 0) {
            RCLCPP_ERROR(this->get_logger(), "re-queueRequest failed");
            running_ = false;
            return;
        }

        // Create message
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera";
        msg->width = width_;
        msg->height = height_;
        msg->encoding = "rgb8";
        msg->step = width_ * 3;
        msg->is_bigendian = false;
        msg->data.resize(data_size);

        // Fast memcpy
        std::memcpy(msg->data.data(), data, data_size);

        // Queue for async publish (non-blocking)
        {
            std::unique_lock<std::mutex> lock(publish_mutex_);
            if (publish_queue_size_.load() < 2) {  // Limit queue depth
                publish_queue_.push(std::move(msg));
                publish_queue_size_.fetch_add(1);
                publish_cv_.notify_one();
            }
            // else drop frame to avoid backlog
        }

        // Stats
        frame_count_++;
        auto now = this->now();
        if ((now - last_stats_time_).seconds() >= 1.0) {
            double fps = frame_count_ / (now - last_stats_time_).seconds();
            RCLCPP_INFO(this->get_logger(), "Camera: %.1f FPS | Queue: %d",
                       fps, publish_queue_size_.load());
            frame_count_ = 0;
            last_stats_time_ = now;
        }
    }

    // Background publish worker thread
    void publish_worker()
    {
        while (publish_thread_running_) {
            std::unique_ptr<sensor_msgs::msg::Image> msg;

            {
                std::unique_lock<std::mutex> lock(publish_mutex_);
                publish_cv_.wait(lock, [this] {
                    return !publish_queue_.empty() || !publish_thread_running_;
                });

                if (!publish_thread_running_) {
                    break;
                }

                if (!publish_queue_.empty()) {
                    msg = std::move(publish_queue_.front());
                    publish_queue_.pop();
                    publish_queue_size_.fetch_sub(1);
                }
            }

            if (msg) {
                // Publish happens in background thread - doesn't block camera callback
                image_pub_->publish(std::move(msg));
            }
        }
    }

    // ROS2
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // libcamera
    std::unique_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream* stream_;

    // Memory mapped buffers
    std::unordered_map<libcamera::FrameBuffer*, MappedBuffer> mappings_;

    // Request pool
    std::vector<std::unique_ptr<libcamera::Request>> requests_;

    // Config
    int width_;
    int height_;
    bool running_;

    // Async publish thread
    std::thread publish_thread_;
    std::atomic<bool> publish_thread_running_;
    std::queue<std::unique_ptr<sensor_msgs::msg::Image>> publish_queue_;
    std::mutex publish_mutex_;
    std::condition_variable publish_cv_;
    std::atomic<int> publish_queue_size_;

    // Stats
    rclcpp::Time last_stats_time_;
    int frame_count_;
};

}  // namespace vio_sensor

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vio_sensor::CameraOnlyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
