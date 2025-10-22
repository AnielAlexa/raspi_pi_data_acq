// standalone_camera_test.cpp
// Minimal libcamera + OpenCV preview for Raspberry Pi (C++) with FPS display.
// Opens the first camera, configures RGB888, displays frames via cv::imshow().
// Press 'q' or ESC to quit.

#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/formats.h>

#include <opencv2/opencv.hpp>

#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
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

int main(int argc, char **argv) {
    int req_width  = 640;
    int req_height = 480;
    if (argc >= 3) {
        req_width  = std::stoi(argv[1]);
        req_height = std::stoi(argv[2]);
    }

    // --- Camera manager ---
    auto cm = std::make_unique<libcamera::CameraManager>();
    if (cm->start()) {
        std::cerr << "ERROR: CameraManager start failed\n";
        return 1;
    }
    if (cm->cameras().empty()) {
        std::cerr << "ERROR: No cameras found\n";
        return 1;
    }

    std::shared_ptr<libcamera::Camera> camera = cm->cameras()[0];
    if (camera->acquire()) {
        std::cerr << "ERROR: Failed to acquire camera\n";
        return 1;
    }
    std::cout << "Using camera: " << camera->id() << "\n";

    // --- Configure as RGB888 viewfinder ---
    std::unique_ptr<libcamera::CameraConfiguration> config =
        camera->generateConfiguration({ libcamera::StreamRole::Viewfinder });
    if (!config) {
        std::cerr << "ERROR: generateConfiguration failed\n";
        return 1;
    }

    libcamera::StreamConfiguration &stream_cfg = config->at(0);
    stream_cfg.pixelFormat = libcamera::formats::RGB888;  // tight-packed RGB
    stream_cfg.size.width  = req_width;
    stream_cfg.size.height = req_height;

    if (config->validate() == libcamera::CameraConfiguration::Invalid) {
        std::cerr << "ERROR: Invalid configuration\n";
        return 1;
    }
    if (camera->configure(config.get()) < 0) {
        std::cerr << "ERROR: camera->configure failed\n";
        return 1;
    }

    const unsigned width  = stream_cfg.size.width;
    const unsigned height = stream_cfg.size.height;
    libcamera::Stream *stream = stream_cfg.stream();

    std::cout << "Configured: " << width << "x" << height
              << " format=" << stream_cfg.pixelFormat.toString() << "\n";

    // --- Allocate frame buffers (allocator owns them) ---
    libcamera::FrameBufferAllocator allocator(camera);
    if (allocator.allocate(stream) < 0) {
        std::cerr << "ERROR: Buffer allocation failed\n";
        return 1;
    }
    const auto &bufs = allocator.buffers(stream);
    if (bufs.empty()) {
        std::cerr << "ERROR: No buffers allocated\n";
        return 1;
    }

    // mmap each plane, keep mapping keyed by FrameBuffer*
    std::unordered_map<libcamera::FrameBuffer *, MappedBuffer> mappings;
    mappings.reserve(bufs.size());
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
                std::cerr << "ERROR: mmap failed on plane " << p << "\n";
                return 1;
            }
            mb.planes[p] = { addr, len };
        }
        mappings.emplace(fb, std::move(mb));
    }

    // --- Create request pool (does not take ownership of buffers) ---
    std::vector<std::unique_ptr<libcamera::Request>> requests;
    requests.reserve(bufs.size());
    for (const auto &uptr : bufs) {
        libcamera::FrameBuffer *fb = uptr.get();
        std::unique_ptr<libcamera::Request> req = camera->createRequest();
        if (!req) {
            std::cerr << "ERROR: createRequest failed\n";
            return 1;
        }
        if (req->addBuffer(stream, fb) < 0) {
            std::cerr << "ERROR: addBuffer failed\n";
            return 1;
        }
        requests.push_back(std::move(req));
    }

    // --- Completed-request queue ---
    std::mutex mtx;
    std::condition_variable cv;
    std::vector<libcamera::Request *> ready;
    std::atomic<bool> running{true};

    // Connect completion signal
    camera->requestCompleted.connect(camera.get(), [&](libcamera::Request *req) {
        if (req->status() == libcamera::Request::RequestCancelled)
            return;
        {
            std::lock_guard<std::mutex> lk(mtx);
            ready.push_back(req);
        }
        cv.notify_one();
    });

    // Optional: disable AE/AWB (best-effort; some sensors ignore)
    libcamera::ControlList controls(camera->controls());
    controls.set(libcamera::controls::AeEnable, false);
    controls.set(libcamera::controls::AwbEnable, false);
    // controls.set(libcamera::controls::ExposureTime, 3000);   // us (if supported)
    // controls.set(libcamera::controls::AnalogueGain, 2.0f);   // (if supported)

    if (camera->start(&controls) < 0) {
        std::cerr << "ERROR: camera->start failed\n";
        return 1;
    }

    // Queue all requests to start streaming
    for (auto &req : requests) {
        if (camera->queueRequest(req.get()) < 0) {
            std::cerr << "ERROR: queueRequest failed\n";
            return 1;
        }
    }

    const std::string win_name = "libcamera preview";
    cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);

    // --- FPS measurement state ---
    auto t_last_log   = std::chrono::steady_clock::now();
    auto t_last_frame = t_last_log;
    int frames_since_log = 0;
    double smoothed_fps = 0.0; // simple EMA

    // --- Main loop ---
    while (running.load()) {
        libcamera::Request *req = nullptr;
        {
            std::unique_lock<std::mutex> lk(mtx);
            cv.wait(lk, [&]{ return !ready.empty() || !running.load(); });
            if (!running.load()) break;
            req = ready.back();
            ready.pop_back();
        }

        // Find our buffer for this stream
        const auto &buffers_map = req->buffers();
        auto it = buffers_map.find(stream);
        if (it == buffers_map.end()) {
            std::cerr << "WARN: request without stream buffer\n";
            continue;
        }
        libcamera::FrameBuffer *fb = it->second;

        // Mapped memory for this buffer
        auto mit = mappings.find(fb);
        if (mit == mappings.end()) {
            std::cerr << "WARN: missing mapping for buffer\n";
            continue;
        }
        // For RGB888 we expect single plane tightly packed
        unsigned step = width * 3;
        void *data = mit->second.planes[0].addr;

        // FPS measure
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - t_last_frame).count();
        t_last_frame = now;
        if (dt > 0.0) {
            double inst_fps = 1.0 / dt;
            // Exponential moving average for stable on-screen number
            double alpha = 0.1; // smoothing factor
            smoothed_fps = (smoothed_fps == 0.0) ? inst_fps : (alpha * inst_fps + (1.0 - alpha) * smoothed_fps);
        }
        frames_since_log++;

        // Wrap in OpenCV Mat and draw FPS overlay
        cv::Mat img(height, width, CV_8UC3, data, step);
        char text[64];
        std::snprintf(text, sizeof(text), "FPS: %.1f", smoothed_fps);
        cv::putText(img, text, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.9, {0, 255, 0}, 2, cv::LINE_AA);

        // Show & update window title occasionally
        cv::imshow(win_name, img);
        if (std::chrono::duration<double>(now - t_last_log).count() >= 1.0) {
            double avg_fps = frames_since_log / std::chrono::duration<double>(now - t_last_log).count();
            std::cout << "Average FPS: " << avg_fps << "\n";
            cv::setWindowTitle(win_name, win_name + "  |  FPS: " + std::to_string(static_cast<int>(smoothed_fps + 0.5)));
            frames_since_log = 0;
            t_last_log = now;
        }

        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            running.store(false);
        }

        // Re-queue
        req->reuse(libcamera::Request::ReuseFlag::ReuseBuffers);
        if (camera->queueRequest(req) < 0) {
            std::cerr << "ERROR: re-queueRequest failed\n";
            break;
        }
    }

    camera->stop();

    // Cleanup mmaps
    for (auto &kv : mappings) {
        for (auto &pl : kv.second.planes) {
            if (pl.addr && pl.length) munmap(pl.addr, pl.length);
        }
    }

    camera->release();
    cm->stop();

    return 0;
}
