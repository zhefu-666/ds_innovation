#include "rescue/app/perception_pipeline.hpp"
#include "rescue/core/stage_audit.hpp"
#include "rescue/core/utils.hpp"
#include <exception>
#include <thread>
#include <stdexcept>

namespace rescue {
namespace {
struct JoinedThread {
    std::thread worker;
    ~JoinedThread() {
        if (worker.joinable())
            worker.join();
    }
};
} // namespace
PerceptionPipeline::PerceptionPipeline(const Config& config)
    : config_(config), detector_(makeDetector(config)) {
    rknn_ = dynamic_cast<YoloRknnDetector*>(detector_.get());
    if (!config.pose_model_path.empty()) {
        if (!rknn_)
            throw std::runtime_error("--pose-model needs an .rknn detector (shared letterbox)");
        pose_ = std::make_unique<SafeZoneKeypointRknn>(config);
    }
}

PerceptionResult PerceptionPipeline::infer(const cv::Mat& frame,
                                           uint64_t timestamp_us,
                                           const ModelImage* prepared,
                                           PoseFrameIdentity identity) {
    const auto ms = [](Clock::time_point a, Clock::time_point b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    AuditScope audit("vision.total");
    PerceptionResult r;
    const auto started = Clock::now();
    if (!rknn_) {
        r.detections = detector_->infer(frame);
        r.detect_ms = r.total_ms = ms(started, Clock::now());
        return r;
    }
    const ModelImage image = prepared && !prepared->rgb.empty()
                                 ? *prepared
                                 : prepareModelImage(frame, config_.input_size);
    const ModelImage pose_image =
        pose_ && pose_->dualExpert() ? prepareExpertPoseImage(frame) : image;
    const auto runPose = [&] {
        const auto t = Clock::now();
        auto halves = pose_->infer(pose_image, frame.size(), &r.pose_metadata, &r.pose_route);
        return std::make_pair(halves, ms(t, Clock::now()));
    };
    std::pair<ZoneHalves, double> pose_result;
    std::exception_ptr pose_error;
    // Join before the shared image and result storage leave scope, including
    // detector errors. Avoid the std::async result path that overruns the stack
    // in the board's optimized, SDK-linked application.
    JoinedThread pending;
    if (pose_ && config_.parallel_inference)
        pending.worker = std::thread([&] {
            try {
                pose_result = runPose();
            } catch (...) {
                pose_error = std::current_exception();
            }
        });
    const auto detect_started = Clock::now();
    r.detections = rknn_->infer(image, frame.size(), timestamp_us);
    r.detect_ms = ms(detect_started, Clock::now());
    if (pose_) {
        if (pending.worker.joinable()) {
            pending.worker.join();
            if (pose_error)
                std::rethrow_exception(pose_error);
        } else {
            pose_result = runPose();
        }
        r.halves = pose_result.first;
        r.pose_ms = pose_result.second;
        r.pose_ran = true;
        if (config_.pose_smoothing_preview != "off") {
            const auto preview_started = Clock::now();
            r.preview_enabled = true;
            identity.model = config_.pose_model_path;
            if (config_.pose_smoothing_preview == "raw")
                r.display = rawPosePreview(r.halves,
                                           frame.size(),
                                           identity.frame_id ? identity.frame_id : timestamp_us,
                                           timestamp_us);
            else
                r.display = smoother_.update(
                    r.halves, r.pose_metadata, pose_image, frame.size(), timestamp_us, identity);
            r.preview_ms = ms(preview_started, Clock::now());
        }
    }
    r.total_ms = ms(started, Clock::now());
    return r;
}
} // namespace rescue
