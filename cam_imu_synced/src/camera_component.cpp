#include "cam_imu_synced/camera_component.hpp"

#include <libcamera/formats.h>
#include <sys/mman.h>

#include <algorithm>
#include <functional>
#include <cstring>

namespace cam_imu_synced
{

CameraComponent::CameraComponent(const rclcpp::NodeOptions & options)
: Node("camera_component", options),
  stream_(nullptr),
  width_(0),
  height_(0),
  stride_(0),
  max_trigger_map_size_(20),
  latest_frame_id_(0)
{
  // --- Parameters ---
  camera_index_   = this->declare_parameter<int>("camera_index", 0);
  width_          = this->declare_parameter<int>("width", 640);
  height_         = this->declare_parameter<int>("height", 480);
  camera_frame_id_= this->declare_parameter<std::string>("camera_frame_id", "camera_link");
  disable_ae_     = this->declare_parameter<bool>("disable_ae", false);
  disable_awb_    = this->declare_parameter<bool>("disable_awb", false);

  // --- Publishers (SensorDataQoS for non-blocking best-effort) ---
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_synced", rclcpp::SensorDataQoS());
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", rclcpp::QoS(10));

  // --- Subscribers for synchronized trigger events ---
  trigger_sub_ = this->create_subscription<std_msgs::msg::Header>(
      "/sync/trigger_time_ros", rclcpp::QoS(10),
      std::bind(&CameraComponent::triggerTimeCallback, this, std::placeholders::_1));
  frame_id_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
      "/sync/trigger_frame_id", rclcpp::QoS(10),
      std::bind(&CameraComponent::triggerFrameIdCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to /sync/trigger_* for frame synchronization");

  if (!initCamera()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
    throw std::runtime_error("Failed to initialize camera");
  }

  // --- Prepare CameraInfo (static intrinsics; only stamp changes) ---
  camera_info_msg_.header.frame_id = camera_frame_id_;
  camera_info_msg_.width  = width_;
  camera_info_msg_.height = height_;
  camera_info_msg_.distortion_model = "plumb_bob";
  camera_info_msg_.d.assign(5, 0.0);
  camera_info_msg_.k = {
    static_cast<double>(width_), 0.0, static_cast<double>(width_) / 2.0,
    0.0, static_cast<double>(width_), static_cast<double>(height_) / 2.0,
    0.0, 0.0, 1.0
  };
  camera_info_msg_.r = {1,0,0, 0,1,0, 0,0,1};
  camera_info_msg_.p = {
    static_cast<double>(width_), 0.0, static_cast<double>(width_) / 2.0, 0.0,
    0.0, static_cast<double>(width_), static_cast<double>(height_) / 2.0, 0.0,
    0.0, 0.0, 1.0, 0.0
  };

  // --- Preallocate a reusable Image message (compact, no padding) ---
  img_msg_template_.header.frame_id = camera_frame_id_;
  img_msg_template_.width  = width_;
  img_msg_template_.height = height_;
  img_msg_template_.encoding = "rgb8";
  img_msg_template_.is_bigendian = false;
  img_msg_template_.step = width_ * 3; // compact row (no padding)
  img_msg_template_.data.resize(static_cast<size_t>(img_msg_template_.step) * height_);

  RCLCPP_INFO(this->get_logger(), "CameraComponent initialized: %dx%d", width_, height_);
}

CameraComponent::~CameraComponent()
{
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

  RCLCPP_INFO(this->get_logger(), "CameraComponent shut down");
}

bool CameraComponent::initCamera()
{
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

  // Configure RGB888
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
  stride_ = stream_cfg.stride;       // may be > width_*3
  stream_ = stream_cfg.stream();

  RCLCPP_INFO(this->get_logger(), "Configured: %dx%d stride=%d format=%s",
              width_, height_, stride_, stream_cfg.pixelFormat.toString().c_str());

  // Allocate buffers
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
      void * addr = mmap(nullptr, pl.length, PROT_READ | PROT_WRITE, MAP_SHARED, pl.fd.get(), pl.offset);
      if (addr == MAP_FAILED) {
        RCLCPP_ERROR(this->get_logger(), "mmap failed on plane %zu", p);
        return false;
      }
      mb.planes[p] = { addr, pl.length };
    }
    mappings_.emplace(fb, std::move(mb));
  }

  // Request pool
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

  // Completion callback
  camera_->requestCompleted.connect(
    camera_.get(),
    std::bind(&CameraComponent::onRequestCompleted, this, std::placeholders::_1)
  );

  // Manual controls (optional)
  libcamera::ControlList controls(camera_->controls());
  if (disable_ae_)  controls.set(libcamera::controls::AeEnable, false);
  if (disable_awb_) controls.set(libcamera::controls::AwbEnable, false);

  if (camera_->start(&controls) < 0) {
    RCLCPP_ERROR(this->get_logger(), "camera->start failed");
    return false;
  }

  for (auto & req : requests_) {
    if (camera_->queueRequest(req.get()) < 0) {
      RCLCPP_ERROR(this->get_logger(), "queueRequest failed");
      return false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Camera streaming started");
  return true;
}

void CameraComponent::triggerTimeCallback(const std_msgs::msg::Header::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(trigger_mutex_);
  latest_trigger_time_ = rclcpp::Time(msg->stamp);
  RCLCPP_DEBUG(this->get_logger(), "Trigger time received: %.3f", latest_trigger_time_.seconds());
}

void CameraComponent::triggerFrameIdCallback(const std_msgs::msg::UInt16::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(trigger_mutex_);
  latest_frame_id_ = msg->data;

  // Store mapping of frame_id to latest trigger time
  trigger_map_[latest_frame_id_] = latest_trigger_time_;
  RCLCPP_DEBUG(this->get_logger(), "Trigger frame_id received: %u, stored with time %.3f",
               latest_frame_id_, latest_trigger_time_.seconds());

  // Clean up old entries to prevent unbounded growth
  if (trigger_map_.size() > max_trigger_map_size_) {
    auto it = trigger_map_.begin();
    for (size_t i = 0; i < 5 && it != trigger_map_.end(); ++i) {
      it = trigger_map_.erase(it);
    }
  }
}

void CameraComponent::onRequestCompleted(libcamera::Request * req)
{
  if (req->status() == libcamera::Request::RequestCancelled) return;

  // Get frame sequence number from libcamera (should match Pico's frame_id)
  uint64_t frame_sequence = req->sequence();

  // Match completed frame to trigger using frame_id
  rclcpp::Time stamp;
  {
    std::lock_guard<std::mutex> lock(trigger_mutex_);

    // Cast sequence to uint16_t to match against trigger map
    uint16_t frame_id = static_cast<uint16_t>(frame_sequence & 0xFFFF);

    // Try to find matching trigger by frame sequence
    auto it = trigger_map_.find(frame_id);
    if (it != trigger_map_.end()) {
      stamp = it->second;
      RCLCPP_DEBUG(this->get_logger(),
                   "Frame seq=%lu (id=%u) matched to trigger with timestamp",
                   frame_sequence, frame_id);
    } else {
      // Fallback to latest trigger if no exact match found
      stamp = latest_trigger_time_;
      RCLCPP_WARN(this->get_logger(),
                  "No trigger match for frame seq=%lu (id=%u), using latest timestamp",
                  frame_sequence, frame_id);
    }
  }

  // Final fallback to current time if still no timestamp
  if (stamp.nanoseconds() == 0) {
    RCLCPP_WARN(this->get_logger(), "No synchronized timestamp available for frame %lu, using wall clock", frame_sequence);
    stamp = this->now();
  }

  // Locate buffer
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

    auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
    img_msg->header.stamp   = stamp;
    img_msg->header.frame_id= camera_frame_id_;
    img_msg->width          = width_;
    img_msg->height         = height_;
    img_msg->encoding       = "rgb8";
    img_msg->is_bigendian   = false;
    img_msg->step           = static_cast<uint32_t>(width_ * 3);

    // Allocate exact compact size once per message
    const size_t dst_step = static_cast<size_t>(width_) * 3;
    img_msg->data.resize(dst_step * static_cast<size_t>(height_));

    // Row-by-row copy from libcamera stride_ → compact ROS buffer
    const uint8_t *src   = static_cast<const uint8_t*>(mit->second.planes[0].addr);
    const size_t   src_step = static_cast<size_t>(stride_);
    uint8_t *dst     = img_msg->data.data();
    for (size_t r = 0; r < static_cast<size_t>(height_); ++r) {
      std::memcpy(dst + r * dst_step, src + r * src_step, dst_step);
    }


       // only stamp changes
    image_pub_->publish(std::move(img_msg));

    // CameraInfo with same stamp
    auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(camera_info_msg_);
    info_msg->header.stamp = stamp;
    camera_info_pub_->publish(std::move(info_msg));
  }

requeue:
  req->reuse(libcamera::Request::ReuseFlag::ReuseBuffers);
  if (camera_->queueRequest(req) < 0) {
    RCLCPP_ERROR(this->get_logger(), "re-queueRequest failed");
  }
}

}  // namespace cam_imu_synced

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cam_imu_synced::CameraComponent)
