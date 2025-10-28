// lccv_publisher_node.h
// Header file for LCCV ROS2 Publisher Node

#ifndef LCCV_PUBLISHER_NODE_H_
#define LCCV_PUBLISHER_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>
#include <lccv.hpp>

#include <chrono>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <cstring>

class LccvPublisherNode : public rclcpp::Node {
public:
    explicit LccvPublisherNode();
    ~LccvPublisherNode();

private:
    // ============================================================
    // Frame Capture Timer Callback (prepares message)
    // ============================================================
    // Captures frame and prepares message, signals publisher thread
    void frameCaptureCb();

    // ============================================================
    // Publisher Thread Loop (publishes message)
    // ============================================================
    // Event-driven: wakes on condition variable, publishes message
    // Uses ZERO CPU when idle (blocked on condition variable)
    void publisherThreadLoop();

    // ============================================================
    // Convert OpenCV Mat to ROS2 Image Message (optimized)
    // ============================================================
    // Uses pre-allocated message with direct data copy
    // Much faster than creating new message each time
    void matToImageMsg(const cv::Mat& frame,
                      sensor_msgs::msg::Image& msg);

    // ============================================================
    // Log Final Statistics
    // ============================================================
    void logFinalStats();

    // Member variables - Camera and Publisher
    std::unique_ptr<lccv::PiCamera> camera_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Member variables - Parameters
    int camera_index_;
    int width_;
    int height_;
    int framerate_;

    // Member variables - Async Publishing (condition variable + thread)
    std::thread publisher_thread_;
    std::mutex publish_mutex_;
    std::condition_variable publish_cv_;
    std::atomic<bool> publisher_running_{false};
    bool frame_ready_to_publish_{false};

    // Pre-allocated message buffers (double buffering)
    sensor_msgs::msg::Image reusable_msg_;    // Callback fills this
    sensor_msgs::msg::Image pending_msg_;     // Publisher publishes this

    // FPS tracking
    std::chrono::steady_clock::time_point last_frame_time_;
    std::chrono::steady_clock::time_point last_log_time_;
    double smoothed_fps_;
    uint32_t frames_since_log_;
    std::atomic<uint32_t> total_frames_{0};
    std::atomic<uint32_t> frame_timeouts_{0};
    std::atomic<uint32_t> frames_skipped_{0};
};

#endif  // LCCV_PUBLISHER_NODE_H_
