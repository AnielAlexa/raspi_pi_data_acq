/**
 * @file camera_capture.cpp
 * @brief Implementation of CameraCapture class
 */

#include "vio_sensor/camera_capture.h"

#include <libcamera/formats.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>

namespace vio_sensor {

CameraCapture::CameraCapture(rclcpp::Node* node, FrameCallback callback)
    : node_(node),
      frame_callback_(callback),
      stream_(nullptr),
      width_(640),
      height_(480),
      camera_frame_id_("camera"),
      running_(false)
{
}

CameraCapture::~CameraCapture()
{
    stop();

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

bool CameraCapture::init(int width, int height)
{
    width_ = width;
    height_ = height;

    // Create camera manager
    camera_manager_ = std::make_unique<libcamera::CameraManager>();
    if (camera_manager_->start()) {
        RCLCPP_ERROR(node_->get_logger(), "CameraManager start failed");
        return false;
    }

    if (camera_manager_->cameras().empty()) {
        RCLCPP_ERROR(node_->get_logger(), "No cameras found");
        return false;
    }

    // Get first camera
    camera_ = camera_manager_->cameras()[0];
    if (camera_->acquire()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to acquire camera");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Using camera: %s", camera_->id().c_str());

    // Configure as RGB888 viewfinder
    config_ = camera_->generateConfiguration({ libcamera::StreamRole::Viewfinder });
    if (!config_) {
        RCLCPP_ERROR(node_->get_logger(), "generateConfiguration failed");
        return false;
    }

    libcamera::StreamConfiguration& stream_cfg = config_->at(0);
    stream_cfg.pixelFormat = libcamera::formats::RGB888;
    stream_cfg.size.width = width_;
    stream_cfg.size.height = height_;

    if (config_->validate() == libcamera::CameraConfiguration::Invalid) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid camera configuration");
        return false;
    }

    if (camera_->configure(config_.get()) < 0) {
        RCLCPP_ERROR(node_->get_logger(), "camera->configure failed");
        return false;
    }

    width_ = stream_cfg.size.width;
    height_ = stream_cfg.size.height;
    stream_ = stream_cfg.stream();

    RCLCPP_INFO(node_->get_logger(), "Camera configured: %dx%d, format=%s",
                width_, height_, stream_cfg.pixelFormat.toString().c_str());

    // Allocate frame buffers
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
    if (allocator_->allocate(stream_) < 0) {
        RCLCPP_ERROR(node_->get_logger(), "Buffer allocation failed");
        return false;
    }

    const auto& bufs = allocator_->buffers(stream_);
    if (bufs.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "No buffers allocated");
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
                RCLCPP_ERROR(node_->get_logger(), "mmap failed on plane %zu", p);
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
            RCLCPP_ERROR(node_->get_logger(), "createRequest failed");
            return false;
        }
        if (req->addBuffer(stream_, fb) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "addBuffer failed");
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

void CameraCapture::start()
{
    // Set fixed camera controls for VIO
    libcamera::ControlList controls(camera_->controls());
    controls.set(libcamera::controls::AeEnable, false);
    controls.set(libcamera::controls::AwbEnable, false);
    controls.set(libcamera::controls::ExposureTime, 3000);  // 3ms
    controls.set(libcamera::controls::AnalogueGain, 2.0f);

    if (camera_->start(&controls) < 0) {
        RCLCPP_ERROR(node_->get_logger(), "camera->start failed");
        return;
    }

    // Queue all requests
    for (auto& req : requests_) {
        if (camera_->queueRequest(req.get()) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "queueRequest failed");
            return;
        }
    }

    running_ = true;
    RCLCPP_INFO(node_->get_logger(), "Camera started (auto-exposure/AWB disabled for VIO)");
}

void CameraCapture::stop()
{
    if (running_ && camera_) {
        camera_->stop();
        running_ = false;
    }
}

void CameraCapture::process_events()
{
    // libcamera event processing is handled by callbacks
    // This function can be used for periodic checks if needed
}

void CameraCapture::request_completed(libcamera::Request* req)
{
    // Find buffer for this stream
    const auto& buffers_map = req->buffers();
    auto it = buffers_map.find(stream_);
    if (it == buffers_map.end()) {
        RCLCPP_WARN(node_->get_logger(), "Request without stream buffer");
        return;
    }

    libcamera::FrameBuffer* fb = it->second;

    // Get mapped memory
    auto mit = mappings_.find(fb);
    if (mit == mappings_.end()) {
        RCLCPP_WARN(node_->get_logger(), "Missing mapping for buffer");
        return;
    }

    // RGB888 single plane tightly packed
    void* data = mit->second.planes[0].addr;
    size_t data_size = width_ * height_ * 3;

    // Create ROS Image message (will be filled by node with trigger timestamp)
    sensor_msgs::msg::Image img_msg;
    img_msg.header.frame_id = camera_frame_id_;
    img_msg.width = width_;
    img_msg.height = height_;
    img_msg.encoding = "rgb8";
    img_msg.step = width_ * 3;
    img_msg.is_bigendian = false;

    // Copy image data (unfortunately ROS2 msg doesn't support shared_ptr to external data)
    img_msg.data.resize(data_size);
    std::memcpy(img_msg.data.data(), data, data_size);

    // Create CameraInfo message
    sensor_msgs::msg::CameraInfo info_msg;
    info_msg.header.frame_id = camera_frame_id_;
    info_msg.width = width_;
    info_msg.height = height_;
    // TODO: Add calibration parameters if available

    // Publish via callback
    frame_callback_(img_msg, info_msg);

    // Re-queue request
    req->reuse(libcamera::Request::ReuseFlag::ReuseBuffers);
    if (camera_->queueRequest(req) < 0) {
        RCLCPP_ERROR(node_->get_logger(), "re-queueRequest failed");
        running_ = false;
    }
}

}  // namespace vio_sensor
