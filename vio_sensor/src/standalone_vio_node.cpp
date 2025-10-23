// standalone_vio_node.cpp
// Single-node VIO sensor: libcamera + IMU with async publish thread
// Based on the working standalone_camera_test.cpp

#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/formats.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "vio_sensor/serial_reader.h"

#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;

struct MappedPlane {
    void *addr = nullptr;
    size_t length = 0;
};

struct MappedBuffer {
    std::vector<MappedPlane> planes;
};

class StandaloneVioNode : public rclcpp::Node {
public:
    StandaloneVioNode() : Node("standalone_vio_node") {
        // Parameters
        int width = this->declare_parameter("width", 640);
        int height = this->declare_parameter("height", 480);
        std::string serial_port = this->declare_parameter("serial_port", "/dev/ttyACM0");
        int serial_baud = this->declare_parameter("serial_baud", 921600);

        RCLCPP_INFO(this->get_logger(), "Standalone VIO Node starting...");
        RCLCPP_INFO(this->get_logger(), "  Resolution: %dx%d", width, height);
        RCLCPP_INFO(this->get_logger(), "  Serial: %s @ %d baud", serial_port.c_str(), serial_baud);

        // Create publishers with BEST_EFFORT QoS
        auto qos = rclcpp::QoS(1).best_effort();
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vio/camera/image_synced", qos);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/vio/imu/data_raw", qos);

        // Start serial reader
        serial_reader_ = std::make_unique<vio_sensor::SerialReader>(
            this, serial_port, serial_baud,
            std::bind(&StandaloneVioNode::imu_callback, this, std::placeholders::_1,
                     std::placeholders::_2, std::placeholders::_3,
                     std::placeholders::_4, std::placeholders::_5, std::placeholders::_6),
            std::bind(&StandaloneVioNode::trigger_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
        serial_reader_->start();

        // Start async publish thread
        publish_thread_ = std::thread(&StandaloneVioNode::publish_worker, this);

        // Initialize camera
        if (!init_camera(width, height)) {
            RCLCPP_ERROR(this->get_logger(), "Camera initialization failed");
            throw std::runtime_error("Camera init failed");
        }

        RCLCPP_INFO(this->get_logger(), "Standalone VIO Node ready");
    }

    ~StandaloneVioNode() {
        running_.store(false);

        if (camera_) {
            camera_->stop();
        }

        // Wake up publish thread
        publish_cv_.notify_all();
        if (publish_thread_.joinable()) {
            publish_thread_.join();
        }

        // Cleanup mmaps
        for (auto &kv : mappings_) {
            for (auto &pl : kv.second.planes) {
                if (pl.addr && pl.length) munmap(pl.addr, pl.length);
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
    bool init_camera(int req_width, int req_height) {
        // Camera manager
        camera_manager_ = std::make_unique<libcamera::CameraManager>();
        if (camera_manager_->start()) {
            RCLCPP_ERROR(this->get_logger(), "CameraManager start failed");
            return false;
        }
        if (camera_manager_->cameras().empty()) {
            RCLCPP_ERROR(this->get_logger(), "No cameras found");
            return false;
        }

        camera_ = camera_manager_->cameras()[0];
        if (camera_->acquire()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire camera");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Using camera: %s", camera_->id().c_str());

        // Configure as RGB888
        std::unique_ptr<libcamera::CameraConfiguration> config =
            camera_->generateConfiguration({ libcamera::StreamRole::Viewfinder });
        if (!config) {
            RCLCPP_ERROR(this->get_logger(), "generateConfiguration failed");
            return false;
        }

        libcamera::StreamConfiguration &stream_cfg = config->at(0);
        stream_cfg.pixelFormat = libcamera::formats::RGB888;
        stream_cfg.size.width = req_width;
        stream_cfg.size.height = req_height;

        if (config->validate() == libcamera::CameraConfiguration::Invalid) {
            RCLCPP_ERROR(this->get_logger(), "Invalid configuration");
            return false;
        }
        if (camera_->configure(config.get()) < 0) {
            RCLCPP_ERROR(this->get_logger(), "camera->configure failed");
            return false;
        }

        width_ = stream_cfg.size.width;
        height_ = stream_cfg.size.height;
        stream_ = stream_cfg.stream();

        RCLCPP_INFO(this->get_logger(), "Camera configured: %dx%d, format=%s",
                   width_, height_, stream_cfg.pixelFormat.toString().c_str());

        // Allocate frame buffers
        libcamera::FrameBufferAllocator allocator(camera_);
        if (allocator.allocate(stream_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Buffer allocation failed");
            return false;
        }
        const auto &bufs = allocator.buffers(stream_);
        if (bufs.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No buffers allocated");
            return false;
        }

        // mmap each buffer
        mappings_.reserve(bufs.size());
        for (const auto &uptr : bufs) {
            libcamera::FrameBuffer *fb = uptr.get();
            MappedBuffer mb;
            mb.planes.resize(fb->planes().size());
            for (size_t p = 0; p < fb->planes().size(); ++p) {
                const libcamera::FrameBuffer::Plane &pl = fb->planes()[p];
                int fd = pl.fd.get();
                size_t len = pl.length;
                off_t off = pl.offset;
                void *addr = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, off);
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
        for (const auto &uptr : bufs) {
            libcamera::FrameBuffer *fb = uptr.get();
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
        camera_->requestCompleted.connect(camera_.get(), [this](libcamera::Request *req) {
            if (req->status() == libcamera::Request::RequestCancelled)
                return;

            // CRITICAL: Re-queue IMMEDIATELY to avoid frame drops
            req->reuse(libcamera::Request::ReuseFlag::ReuseBuffers);
            camera_->queueRequest(req);

            // Get buffer and copy to ROS message asynchronously
            const auto &buffers_map = req->buffers();
            auto it = buffers_map.find(stream_);
            if (it == buffers_map.end()) return;

            libcamera::FrameBuffer *fb = it->second;
            auto mit = mappings_.find(fb);
            if (mit == mappings_.end()) return;

            void *data = mit->second.planes[0].addr;
            size_t data_size = width_ * height_ * 3;

            // Get trigger timestamp
            rclcpp::Time trigger_time = serial_reader_->get_latest_trigger_time();

            // Create ROS message and copy data
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = trigger_time;
            msg->header.frame_id = "camera";
            msg->width = width_;
            msg->height = height_;
            msg->encoding = "rgb8";
            msg->step = width_ * 3;
            msg->data.resize(data_size);
            std::memcpy(msg->data.data(), data, data_size);

            // Queue for async publish
            {
                std::lock_guard<std::mutex> lock(publish_mutex_);
                publish_queue_.push(std::move(msg));
            }
            publish_cv_.notify_one();

            frame_count_++;
        });

        // Disable AE/AWB for VIO
        libcamera::ControlList controls(camera_->controls());
        controls.set(libcamera::controls::AeEnable, false);
        controls.set(libcamera::controls::AwbEnable, false);

        if (camera_->start(&controls) < 0) {
            RCLCPP_ERROR(this->get_logger(), "camera->start failed");
            return false;
        }

        // Queue all requests to start streaming
        for (auto &req : requests_) {
            if (camera_->queueRequest(req.get()) < 0) {
                RCLCPP_ERROR(this->get_logger(), "queueRequest failed");
                return false;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Camera started (AE/AWB disabled for VIO)");

        // Start stats timer
        stats_timer_ = this->create_wall_timer(1s, std::bind(&StandaloneVioNode::stats_callback, this));

        return true;
    }

    void publish_worker() {
        while (running_.load()) {
            std::unique_ptr<sensor_msgs::msg::Image> msg;
            {
                std::unique_lock<std::mutex> lock(publish_mutex_);
                publish_cv_.wait(lock, [this] {
                    return !publish_queue_.empty() || !running_.load();
                });

                if (!running_.load()) break;

                if (!publish_queue_.empty()) {
                    msg = std::move(publish_queue_.front());
                    publish_queue_.pop();
                }
            }

            if (msg) {
                image_pub_->publish(std::move(msg));
            }
        }
    }

    void imu_callback(uint64_t timestamp_us, float ax, float ay, float az,
                     float gx, float gy, float gz) {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = serial_reader_->pico_to_ros_time(timestamp_us);
        msg.header.frame_id = "imu";

        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;

        msg.angular_velocity.x = gx;
        msg.angular_velocity.y = gy;
        msg.angular_velocity.z = gz;

        imu_pub_->publish(msg);
        imu_count_++;
    }

    void trigger_callback(uint64_t timestamp_us, uint32_t frame_id) {
        (void)timestamp_us;
        (void)frame_id;
        trigger_count_++;
    }

    void stats_callback() {
        RCLCPP_INFO(this->get_logger(), "Camera: %d FPS | IMU: %d Hz | Triggers: %d Hz | Queue: %zu",
                   frame_count_, imu_count_, trigger_count_, publish_queue_.size());
        frame_count_ = 0;
        imu_count_ = 0;
        trigger_count_ = 0;
    }

    // ROS2
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    // Serial reader
    std::unique_ptr<vio_sensor::SerialReader> serial_reader_;

    // libcamera objects
    std::unique_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::unordered_map<libcamera::FrameBuffer*, MappedBuffer> mappings_;
    libcamera::Stream *stream_ = nullptr;

    // Camera config
    unsigned width_ = 0;
    unsigned height_ = 0;

    // Async publish thread
    std::thread publish_thread_;
    std::queue<std::unique_ptr<sensor_msgs::msg::Image>> publish_queue_;
    std::mutex publish_mutex_;
    std::condition_variable publish_cv_;

    // Running state
    std::atomic<bool> running_{true};

    // Stats
    int frame_count_ = 0;
    int imu_count_ = 0;
    int trigger_count_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StandaloneVioNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
