// lccv_test_node.cpp
// Simple test node to evaluate LCCV library for camera capture
// Tests performance, stability, and flickering issues

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <lccv.hpp>

int main(int argc, char** argv) {
    (void)argc;  // Suppress unused warning
    (void)argv;

    std::cout << "=== LCCV Camera Test Node ===" << std::endl;
    std::cout << "Testing LCCV library for camera capture performance" << std::endl;

    // Create LCCV camera object
    lccv::PiCamera cam;

    // Configure camera options
    // Match the settings from camera_display_node
    cam.options->camera = 0;  // Camera index
    cam.options->video_width = 1456;   // Native IMX296 resolution
    cam.options->video_height = 1088;
    cam.options->framerate = 20;  // 20 Hz to match Pico triggers
    cam.options->verbose = true;  // Enable verbose output for debugging

    // Set exposure and gain (manual control like camera_display_node)
    // Note: LCCV uses shutter in microseconds, gain as float
    cam.options->shutter = 3000;  // 3000 microseconds = 3ms exposure
    cam.options->gain = 2.0;      // Analog gain

    std::cout << "Camera Configuration:" << std::endl;
    std::cout << "  Resolution: " << cam.options->video_width << "x" << cam.options->video_height << std::endl;
    std::cout << "  Frame Rate: " << cam.options->framerate << " Hz" << std::endl;
    std::cout << "  Exposure: " << cam.options->shutter << " us" << std::endl;
    std::cout << "  Gain: " << cam.options->gain << std::endl;

    // Start capture (LCCV initializes internally)
    std::cout << "\nStarting camera capture..." << std::endl;
    if (!cam.startVideo()) {
        std::cerr << "ERROR: Failed to start video capture!" << std::endl;
        return -1;
    }

    std::cout << "Camera started successfully!" << std::endl;
    std::cout << "Press 'q' to quit, 's' to save screenshot" << std::endl;

    // Create OpenCV window
    cv::namedWindow("LCCV Camera Test", cv::WINDOW_NORMAL);
    cv::resizeWindow("LCCV Camera Test", 1456, 1088);

    // FPS tracking variables
    auto start_time = std::chrono::steady_clock::now();
    auto last_fps_time = start_time;
    int frame_count = 0;
    int total_frames = 0;
    double smoothed_fps = 0.0;
    const double fps_alpha = 0.1;  // Smoothing factor

    // Frame capture and display loop
    cv::Mat frame;
    int screenshot_count = 0;

    while (true) {
        // Capture frame (this should be much faster than manual buffer handling)
        if (!cam.getVideoFrame(frame, 1000)) {  // 1000ms timeout
            std::cerr << "WARNING: Failed to get frame (timeout)" << std::endl;
            continue;
        }

        // Calculate FPS
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - start_time).count();
        start_time = now;

        if (dt > 0.0) {
            double instant_fps = 1.0 / dt;
            smoothed_fps = (smoothed_fps == 0.0) ? instant_fps :
                          (fps_alpha * instant_fps + (1.0 - fps_alpha) * smoothed_fps);
        }

        frame_count++;
        total_frames++;

        // Log FPS every second
        double time_since_log = std::chrono::duration<double>(now - last_fps_time).count();
        if (time_since_log >= 1.0) {
            double avg_fps = frame_count / time_since_log;
            std::cout << "FPS: " << std::fixed << std::setprecision(1)
                     << smoothed_fps << " (avg: " << avg_fps
                     << ") | Total frames: " << total_frames << std::endl;

            frame_count = 0;
            last_fps_time = now;
        }

        // Overlay FPS on frame
        std::string fps_text = "FPS: " + std::to_string(static_cast<int>(smoothed_fps));
        cv::putText(frame, fps_text, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        // Display frame
        cv::imshow("LCCV Camera Test", frame);

        // Handle keyboard input
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) {  // q or ESC to quit
            std::cout << "Quit requested by user" << std::endl;
            break;
        } else if (key == 's' || key == 'S') {  // s to save screenshot
            std::string filename = "lccv_screenshot_" + std::to_string(screenshot_count++) + ".png";
            cv::imwrite(filename, frame);
            std::cout << "Screenshot saved: " << filename << std::endl;
        }
    }

    // Cleanup
    std::cout << "\nStopping camera..." << std::endl;
    cam.stopVideo();
    cv::destroyAllWindows();

    // Print final statistics
    std::cout << "\n=== Test Complete ===" << std::endl;
    std::cout << "Total frames captured: " << total_frames << std::endl;
    std::cout << "Final smoothed FPS: " << std::fixed << std::setprecision(2)
             << smoothed_fps << std::endl;

    return 0;
}
