#ifndef CAM_IMU_SYNCED__CAMERA_COMPONENT_HPP_
#define CAM_IMU_SYNCED__CAMERA_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace cam_imu_synced
{

struct MappedPlane {
  void *addr = nullptr;
  size_t length = 0;
};

struct MappedBuffer {
  std::vector<MappedPlane> planes;
};

class CameraComponent : public rclcpp::Node
{
public:
  explicit CameraComponent(const rclcpp::NodeOptions & options);
  virtual ~CameraComponent();

private:
  // Initialize libcamera
  bool initCamera();

  // Trigger subscription callbacks
  void triggerTimeCallback(const std_msgs::msg::Header::SharedPtr msg);
  void triggerFrameIdCallback(const std_msgs::msg::UInt16::SharedPtr msg);

  // libcamera request completion callback
  void onRequestCompleted(libcamera::Request * req);

  // Publish camera info
  void publishCameraInfo(const rclcpp::Time & stamp);

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // Subscribers for trigger timestamps and frame IDs
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr frame_id_sub_;

  // libcamera objects
  std::unique_ptr<libcamera::CameraManager> camera_manager_;
  std::shared_ptr<libcamera::Camera> camera_;
  std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
  std::vector<std::unique_ptr<libcamera::Request>> requests_;
  std::unordered_map<libcamera::FrameBuffer *, MappedBuffer> mappings_;

  libcamera::Stream * stream_;
  unsigned int width_;
  unsigned int height_;
  unsigned int stride_;

  // Frame ID matching: store trigger data
  struct TriggerData {
    uint16_t frame_id;
    rclcpp::Time timestamp;
  };

  std::mutex trigger_mutex_;
  std::unordered_map<uint16_t, rclcpp::Time> trigger_map_;  // frame_id -> timestamp
  size_t max_trigger_map_size_;
  rclcpp::Time latest_trigger_time_;  // Latest trigger timestamp for fallback
  uint16_t latest_frame_id_;          // Latest trigger frame_id for matching

  // Camera parameters
  int camera_index_;
  std::string camera_frame_id_;
  bool disable_ae_;
  bool disable_awb_;

  // Camera info
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  sensor_msgs::msg::Image      img_msg_template_;
};

}  // namespace cam_imu_synced

#endif  // CAM_IMU_SYNCED__CAMERA_COMPONENT_HPP_
