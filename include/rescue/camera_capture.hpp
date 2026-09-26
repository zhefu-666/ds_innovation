#pragma once

#include <opencv2/videoio.hpp>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace rescue {

struct CameraFrame {
    cv::Mat image;
    uint64_t timestamp_us = 0;
    uint64_t frame_number = 0;
};

class CameraCapture {
public:
    CameraCapture(int index, int width, int height, int fps);
    ~CameraCapture();
    CameraCapture(const CameraCapture &) = delete;
    CameraCapture &operator=(const CameraCapture &) = delete;

    bool start();
    void stop();
    bool latest(CameraFrame &out) const;
    bool isRunning() const { return running_.load(); }
    uint64_t droppedFrames() const { return dropped_frames_.load(); }

private:
    void captureLoop();

    int index_;
    int width_;
    int height_;
    int fps_;
    mutable std::mutex mutex_;
    cv::VideoCapture capture_;
    CameraFrame latest_;
    std::atomic_bool running_{false};
    std::atomic<uint64_t> dropped_frames_{0};
    uint64_t captured_frames_ = 0;
    std::thread thread_;
};

} // namespace rescue
