/**
 * @file camera_capture.cpp
 * @brief Implementation of CameraCapture (reused messages, no per-frame allocations)
 */

#include "vio_sensor/camera_capture.h"

#include <libcamera/formats.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>

namespace vio_sensor {

CameraCapture::CameraCapture(rclcpp::Node* node, FrameCallback cb)
  : node_(node), frame_callback_(std::move(cb)),
    stream_(nullptr),
    width_(640),
    height_(480),
    camera_frame_id_("camera"),
    running_(false) {}

CameraCapture::~CameraCapture() {
  stop();

  // Cleanup mmaps
  for (auto& kv : mappings_) {
    for (auto& pl : kv.second.planes) {
      if (pl.addr && pl.length) {
        munmap(pl.addr, pl.length);
      }
    }
  }
  mappings_.clear();

  if (camera_) {
    camera_->release();
    camera_.reset();
  }
  if (camera_manager_) {
    camera_manager_->stop();
    camera_manager_.reset();
  }
}

bool CameraCapture::init(int width, int height) {
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
  stream_cfg.size.width  = width_;
  stream_cfg.size.height = height_;

  if (config_->validate() == libcamera::CameraConfiguration::Invalid) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid camera configuration");
    return false;
  }

  if (camera_->configure(config_.get()) < 0) {
    RCLCPP_ERROR(node_->get_logger(), "camera->configure failed");
    return false;
  }

  width_  = stream_cfg.size.width;
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
      void* addr = mmap(nullptr, pl.length, PROT_READ | PROT_WRITE, MAP_SHARED, pl.fd.get(), pl.offset);
      if (addr == MAP_FAILED) {
        RCLCPP_ERROR(node_->get_logger(), "mmap failed on plane %zu", p);
        return false;
      }
      mb.planes[p] = { addr, pl.length };
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

  // ---- Prepare reusable ROS messages (preallocate once) ----
  const size_t data_size = static_cast<size_t>(width_) * static_cast<size_t>(height_) * 3;

  img_msg_.header.frame_id = camera_frame_id_;
  img_msg_.width  = width_;
  img_msg_.height = height_;
  img_msg_.encoding = "rgb8";
  img_msg_.step = width_ * 3;
  img_msg_.is_bigendian = false;
  img_msg_.data.resize(data_size);  // fixed capacity; reused every frame

  info_msg_.header.frame_id = camera_frame_id_;
  info_msg_.width  = width_;
  info_msg_.height = height_;
  info_msg_.distortion_model = "plumb_bob";
  info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info_msg_.k = {
      static_cast<double>(width_), 0.0, static_cast<double>(width_) / 2.0,
      0.0, static_cast<double>(width_), static_cast<double>(height_) / 2.0,
      0.0, 0.0, 1.0
  };
  info_msg_.r = {1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0};
  info_msg_.p = {
      static_cast<double>(width_), 0.0, static_cast<double>(width_) / 2.0, 0.0,
      0.0, static_cast<double>(width_), static_cast<double>(height_) / 2.0, 0.0,
      0.0, 0.0, 1.0, 0.0
  };

  // Connect completion signal
  camera_->requestCompleted.connect(camera_.get(),
      [this](libcamera::Request* req) {
        if (req->status() != libcamera::Request::RequestCancelled) {
          request_completed(req);
        }
      });

  return true;
}

void CameraCapture::start() {
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

void CameraCapture::stop() {
  if (running_ && camera_) {
    camera_->stop();
    running_ = false;
  }
}

void CameraCapture::process_events() {
  // libcamera event processing is handled by callbacks
}

void CameraCapture::request_completed(libcamera::Request* req) {
  // We’ll always requeue at the end, regardless of success/failure in this cycle.
  bool ok = true;
  const void* data_ptr = nullptr;

  // Locate the buffer for our stream
  const auto& buffers_map = req->buffers();
  auto it = buffers_map.find(stream_);
  if (it == buffers_map.end()) {
    RCLCPP_WARN(node_->get_logger(), "Request without stream buffer");
    ok = false;
  }

  libcamera::FrameBuffer* fb = nullptr;
  if (ok) {
    fb = it->second;
    auto mit = mappings_.find(fb);
    if (mit == mappings_.end()) {
      RCLCPP_WARN(node_->get_logger(), "Missing mapping for buffer");
      ok = false;
    } else {
      // RGB888 single plane tightly packed
      data_ptr = mit->second.planes[0].addr;
    }
  }

  if (ok && data_ptr != nullptr) {
    const size_t data_size = static_cast<size_t>(width_) * static_cast<size_t>(height_) * 3;
    // memcpy only (no allocations)
    std::memcpy(img_msg_.data.data(), data_ptr, data_size);

    // Hand off by reference (publisher can stamp + publish)
    frame_callback_(img_msg_, info_msg_);
  }

  // Re-queue request (even if this frame failed to process)
  req->reuse(libcamera::Request::ReuseFlag::ReuseBuffers);
  if (camera_->queueRequest(req) < 0) {
    RCLCPP_ERROR(node_->get_logger(), "re-queueRequest failed");
    running_ = false;
  }
}

}  // namespace vio_sensor
