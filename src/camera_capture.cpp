#include "rescue/camera_capture.hpp"

#include <chrono>

namespace rescue {

namespace {
uint64_t nowUs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
} // namespace

CameraCapture::CameraCapture(int index, int width, int height, int fps)
    : index_(index), width_(width), height_(height), fps_(fps) {}

CameraCapture::~CameraCapture() { stop(); }

bool CameraCapture::start() {
    if (running_.load()) return true;
    if (!capture_.open(index_)) return false;
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    capture_.set(cv::CAP_PROP_FPS, fps_);
    running_.store(true);
    thread_ = std::thread(&CameraCapture::captureLoop, this);
    return true;
}

void CameraCapture::stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    if (capture_.isOpened()) capture_.release();
}

bool CameraCapture::latest(CameraFrame &out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_.image.empty()) return false;
    out = latest_;
    out.image = latest_.image.clone();
    return true;
}

void CameraCapture::captureLoop() {
    cv::Mat frame;
    while (running_.load()) {
        if (!capture_.read(frame) || frame.empty()) {
            dropped_frames_.fetch_add(1);
            continue;
        }
        CameraFrame next;
        next.image = frame.clone();
        next.timestamp_us = nowUs();
        next.frame_number = ++captured_frames_;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!latest_.image.empty() && next.frame_number > latest_.frame_number + 1) {
            dropped_frames_.fetch_add(next.frame_number - latest_.frame_number - 1);
        }
        latest_ = std::move(next);
    }
}

} // namespace rescue
