#include "rescue/app/perception_pipeline.hpp"
#include "rescue/utils.hpp"
#include <future>
#include <stdexcept>

namespace rescue {
PerceptionPipeline::PerceptionPipeline(const Config &config)
    : config_(config), detector_(makeDetector(config)) {
    rknn_ = dynamic_cast<YoloRknnDetector *>(detector_.get());
    if (!config.pose_model_path.empty()) {
        if (!rknn_) throw std::runtime_error("--pose-model needs an .rknn detector (shared letterbox)");
        pose_ = std::make_unique<SafeZoneKeypointRknn>(config);
    }
}

PerceptionResult PerceptionPipeline::infer(const cv::Mat &frame, uint64_t timestamp_us) {
    const auto ms = [](Clock::time_point a, Clock::time_point b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    PerceptionResult r;
    const auto started = Clock::now();
    if (!rknn_) {
        r.detections = detector_->infer(frame);
        r.detect_ms = r.total_ms = ms(started, Clock::now());
        return r;
    }
    const auto image = prepareModelImage(frame, config_.input_size);
    const auto runPose = [&] {
        const auto t = Clock::now();
        auto halves = pose_->infer(image, frame.size());
        return std::make_pair(halves, ms(t, Clock::now()));
    };
    std::future<std::pair<ZoneHalves, double>> pending;
    if (pose_ && config_.parallel_inference) pending = std::async(std::launch::async, runPose);
    const auto detect_started = Clock::now();
    r.detections = rknn_->infer(image, frame.size(), timestamp_us);
    r.detect_ms = ms(detect_started, Clock::now());
    if (pose_) {
        auto pose = pending.valid() ? pending.get() : runPose();
        r.halves = pose.first; r.pose_ms = pose.second; r.pose_ran = true;
    }
    r.total_ms = ms(started, Clock::now());
    return r;
}
} // namespace rescue
