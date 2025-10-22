// libcamera_opencv_preview.cpp
// Minimal libcamera + OpenCV preview for Raspberry Pi (C++)
// Press 'q' to quit.

#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/formats.h>

#include <opencv2/opencv.hpp>

#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
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
    if (argc == 3) {
        req_width  = std::stoi(argv[1]);
        req_height = std::stoi(argv[2]);
    }

    // --- Camera manager ---
    std::unique_ptr<libcamera::CameraManager> cm = std::make_unique<libcamera::CameraManager>();
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
    stream_cfg.pixelFormat = libcamera::formats::RGB888;          // request RGB888
    stream_cfg.size.width  = req_width;
    stream_cfg.size.height = req_height;

    libcamera::CameraConfiguration::Status st = config->validate();
    if (st == libcamera::CameraConfiguration::Invalid) {
        std::cerr << "ERROR: Config invalid\n";
        return 1;
    }
    if (camera->configure(config.get()) < 0) {
        std::cerr << "ERROR: camera->configure failed\n";
        return 1;
    }

    const unsigned width  = stream_cfg.size.width;
    const unsigned height = stream_cfg.size.height;
    std::cout << "Configured size: " << width << "x" << height
              << " format: " << stream_cfg.pixelFormat.toString() << "\n";

    libcamera::Stream *stream = stream_cfg.stream();

    // --- Allocate frame buffers ---
    libcamera::FrameBufferAllocator allocator(camera);
    if (allocator.allocate(stream) < 0) {
        std::cerr << "ERROR: Buffer allocation failed\n";
        return 1;
    }
    const auto &buffers = allocator.buffers(stream);
    if (buffers.empty()) {
        std::cerr << "ERROR: No buffers allocated\n";
        return 1;
    }

    // mmap planes once per buffer
    std::vector<std::unique_ptr<libcamera::FrameBuffer>> framebuffers;
    std::vector<MappedBuffer> mapped;
    framebuffers.reserve(buffers.size());
    mapped.resize(buffers.size());

    for (size_t i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<libcamera::FrameBuffer> fb = std::move(buffers[i]);
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
        mapped[i] = std::move(mb);
        framebuffers.push_back(std::move(fb));
    }

    // --- Request pool ---
    std::vector<std::unique_ptr<libcamera::Request>> requests;
    requests.reserve(framebuffers.size());
    for (auto &fb : framebuffers) {
        std::unique_ptr<libcamera::Request> req = camera->createRequest();
        if (!req) {
            std::cerr << "ERROR: createRequest failed\n";
            return 1;
        }
        if (req->addBuffer(stream, fb.get()) < 0) {
            std::cerr << "ERROR: addBuffer failed\n";
            return 1;
        }
        requests.push_back(std::move(req));
    }

    // --- Simple completed-request queue ---
    std::mutex mtx;
    std::condition_variable cv;
    std::vector<libcamera::Request *> ready;
    std::atomic<bool> running{true};

    camera->requestCompleted.connect(&*camera, [&](libcamera::Request *req) {
        if (req->status() == libcamera::Request::RequestCancelled)
            return;
        {
            std::lock_guard<std::mutex> lk(mtx);
            ready.push_back(req);
        }
        cv.notify_one();
    });

    // Optional: disable AE/AWB and set manual exposure/gain (best-effort)
    libcamera::ControlList controls = camera->controls();
    controls.set(libcamera::controls::AeEnable, false);
    controls.set(libcamera::controls::AwbEnable, false);
    // You can set ExposureTime (in microseconds) and AnalogueGain if supported:
    // controls.set(libcamera::controls::ExposureTime, 3000);
    // controls.set(libcamera::controls::AnalogueGain, 2.0f);

    if (camera->start(&controls) < 0) {
        std::cerr << "ERROR: camera->start failed\n";
        return 1;
    }

    // Queue all requests to begin streaming
    for (auto &req : requests) {
        if (camera->queueRequest(req.get()) < 0) {
            std::cerr << "ERROR: queueRequest failed\n";
            return 1;
        }
    }

    cv::namedWindow("libcamera preview", cv::WINDOW_AUTOSIZE);

    // --- Main loop: wait for a completed request, display, then requeue ---
    while (running) {
        libcamera::Request *req = nullptr;
        {
            std::unique_lock<std::mutex> lk(mtx);
            cv.wait(lk, [&]{ return !ready.empty(); });
            req = ready.back();
            ready.pop_back();
        }

        const auto &buffers_map = req->buffers();
        auto it = buffers_map.find(stream);
        if (it == buffers_map.end()) {
            std::cerr << "ERROR: request without our stream buffer\n";
            continue;
        }
        libcamera::FrameBuffer *fb = it->second;

        // Find index of fb in our framebuffers vector to access mapped memory
        size_t idx = 0;
        for (; idx < framebuffers.size(); ++idx)
            if (framebuffers[idx].get() == fb)
                break;

        // For RGB888 we expect a single plane tightly packed (step = width*3).
        unsigned step = width * 3;
        void *data = mapped[idx].planes[0].addr;

        // Wrap in OpenCV Mat and show
        cv::Mat img(height, width, CV_8UC3, data, step);
        cv::imshow("libcamera preview", img);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) { // 'q' or ESC
            running = false;
        }

        // Re-queue the same request to keep streaming
        req->reuse(libcamera::Request::ReuseFlag::ReuseBuffers);
        if (camera->queueRequest(req) < 0) {
            std::cerr << "ERROR: re-queueRequest failed\n";
            break;
        }
    }

    camera->stop();

    // Cleanup mmaps
    for (auto &mb : mapped) {
        for (auto &pl : mb.planes) {
            if (pl.addr && pl.length)
                munmap(pl.addr, pl.length);
        }
    }

    camera->release();
    cm->stop();

    return 0;
}
