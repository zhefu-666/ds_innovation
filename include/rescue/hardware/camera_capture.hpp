#pragma once

#include <opencv2/videoio.hpp>
#include "rescue/perception/model_io.hpp"

#include <atomic>
#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace rescue {

struct CameraFrame {
    cv::Mat image;
    ModelImage prepared;       // Same decoded frame, immutable until inference completes.
    uint64_t timestamp_us = 0; // Host read completion: before decode in raw mode, after decode in
                               // legacy mode; not exposure.
    uint64_t epoch_ns = 0;
    uint64_t frame_number = 0;
};

class CameraCapture {
  public:
    CameraCapture(int index, int width, int height, int fps, int model_size = 0);
    ~CameraCapture();
    CameraCapture(const CameraCapture&) = delete;
    CameraCapture& operator=(const CameraCapture&) = delete;

    bool start();
    void stop();
    bool latest(CameraFrame& out) const;
    bool waitNext(uint64_t after, CameraFrame& out, int timeout_ms);
    bool isRunning() const {
        return running_.load();
    }
    uint64_t droppedFrames() const {
        return dropped_frames_.load();
    }

  private:
    void captureLoop();
    void decodeLoop();
    struct CompressedFrame {
        cv::Mat jpeg;
        uint64_t timestamp_us = 0, epoch_ns = 0, number = 0;
    };

    int index_;
    int width_;
    int height_;
    int fps_;
    int model_size_;
    bool raw_mjpeg_ = false;
    int decode_fps_ = 40;
    mutable std::mutex compressed_mutex_;
    mutable std::condition_variable compressed_ready_;
    CompressedFrame compressed_;
    uint64_t compressed_read_ = 0;
    mutable bool decode_requested_ = true;
    uint64_t last_consume_us_ = 0;
    double consumer_period_ms_ = 38.0, decode_cost_ms_ = 12.0;
    std::chrono::steady_clock::time_point decode_target_;
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    uint64_t delivered_frame_ = 0;
    cv::VideoCapture capture_;
    CameraFrame latest_;
    std::atomic_bool running_{false};
    std::atomic<uint64_t> dropped_frames_{0};
    uint64_t captured_frames_ = 0;
    std::thread thread_, decode_thread_;
};

} // namespace rescue
