// camera_fps_subscriber.cpp
// ROS2 subscriber that measures FPS of incoming camera frames

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <deque>
#include <mutex>
#include <cmath>

class CameraFpsSubscriber : public rclcpp::Node {
public:
    CameraFpsSubscriber() : Node("camera_fps_subscriber") {
        // Subscribe to camera image topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_display",
            rclcpp::SensorDataQoS(),
            std::bind(&CameraFpsSubscriber::imageCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Camera FPS Subscriber started, listening to /camera/image_display");

        // Create a timer to log FPS stats every 2 seconds
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&CameraFpsSubscriber::logFpsStats, this)
        );

        frame_count_ = 0;
        last_log_time_ = std::chrono::steady_clock::now();
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto now = std::chrono::steady_clock::now();

        // Calculate time since last frame
        if (frame_times_.size() > 0) {
            double dt = std::chrono::duration<double>(now - frame_times_.back()).count();
            if (dt > 0.0) {
                double inst_fps = 1.0 / dt;

                // Exponential moving average for smoothed FPS
                double alpha = 0.1;
                smoothed_fps_ = (smoothed_fps_ == 0.0) ? inst_fps :
                               (alpha * inst_fps + (1.0 - alpha) * smoothed_fps_);
            }
        }

        // Keep last 60 frame times for statistics
        frame_times_.push_back(now);
        if (frame_times_.size() > 60) {
            frame_times_.pop_front();
        }

        frame_count_++;

        // Log every 30 frames
        if (frame_count_ % 30 == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Frame #%lu received at %.3f, smoothed FPS: %.1f",
                        frame_count_, msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9, smoothed_fps_);
        }
    }

    void logFpsStats() {
        if (frame_times_.size() < 2) {
            return;
        }

        // Calculate average FPS over last N frames
        double time_span = std::chrono::duration<double>(
            frame_times_.back() - frame_times_.front()
        ).count();

        if (time_span > 0.0) {
            double avg_fps = (frame_times_.size() - 1) / time_span;

            RCLCPP_INFO(this->get_logger(),
                       "Total frames: %lu | Smoothed FPS: %.1f | Average FPS (last 60): %.1f | Time span: %.2fs",
                       frame_count_, smoothed_fps_, avg_fps, time_span);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    uint64_t frame_count_;
    double smoothed_fps_ = 0.0;
    std::deque<std::chrono::steady_clock::time_point> frame_times_;
    std::chrono::steady_clock::time_point last_log_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraFpsSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
