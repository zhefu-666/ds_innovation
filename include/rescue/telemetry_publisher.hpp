#pragma once
#include "rescue/config.hpp"
#include "rescue/push_task.hpp"
#include <opencv2/core.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

namespace rescue {
// A bounded, optional observation path. The worker never opens the camera or UART.
// One atomic snapshot in tmpfs is consumed by the separate Foxglove bridge.
class TelemetryPublisher {
public:
    explicit TelemetryPublisher(const Config& config);
    ~TelemetryPublisher();
    TelemetryPublisher(const TelemetryPublisher&) = delete;
    TelemetryPublisher& operator=(const TelemetryPublisher&) = delete;
    void submit(const cv::Mat& annotated, const std::vector<SegDetection>& detections,
                const PushObservation& observation, const PushOutput& output,
                uint64_t epoch_ns, uint64_t sequence, double loop_fps,
                double inference_ms, double capture_ms) noexcept;
private:
    struct Snapshot {
        cv::Mat frame;
        std::vector<SegDetection> detections;
        PushObservation observation;
        PushOutput output;
        uint64_t epoch_ns = 0, sequence = 0;
        double loop_fps = 0, inference_ms = 0, capture_ms = 0;
    };
    void run() noexcept;
    void writeSnapshot(const Snapshot& snapshot);
    Config config_;
    std::mutex mutex_;
    std::condition_variable ready_;
    Snapshot pending_;
    bool pending_valid_ = false, stopped_ = false;
    std::atomic<uint64_t> dropped_{0}, errors_{0};
    std::chrono::steady_clock::time_point next_submit_{};
    int lock_fd_ = -1;
    std::thread worker_;
};
}
