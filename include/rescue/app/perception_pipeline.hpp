#pragma once

#include "rescue/config.hpp"
#include "rescue/detector.hpp"
#include "rescue/model_io.hpp"
#include "rescue/zone_keypoints.hpp"
#include <cstdint>
#include <memory>
#include <vector>

namespace rescue {
struct PerceptionResult {
    std::vector<SegDetection> detections;
    ZoneHalves halves;
    bool pose_ran = false;
    double detect_ms = 0, pose_ms = 0, total_ms = 0;
};

// Detector plus optional safe-zone pose model on one shared 640 letterbox. Each
// RKNN context is only ever driven by one caller at a time; with --parallel-infer
// the pose context runs on an async worker while the detector runs here.
class PerceptionPipeline {
public:
    explicit PerceptionPipeline(const Config &config);
    bool hasPose() const { return pose_ != nullptr; }
    PerceptionResult infer(const cv::Mat &frame, uint64_t timestamp_us, const ModelImage *prepared = nullptr);

private:
    Config config_;
    std::unique_ptr<IDetector> detector_;
    YoloRknnDetector *rknn_ = nullptr;
    std::unique_ptr<SafeZoneKeypointRknn> pose_;
};
} // namespace rescue
