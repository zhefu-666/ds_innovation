#pragma once

#include "rescue/config/config.hpp"
#include "rescue/perception/detector.hpp"
#include "rescue/perception/model_io.hpp"
#include "rescue/perception/zone_keypoints.hpp"
#include "rescue/perception/pose_temporal_smoother.hpp"
#include <cstdint>
#include <memory>
#include <vector>

namespace rescue {
struct PerceptionResult {
    std::vector<SegDetection> detections;
    ZoneHalves halves; // Raw same-frame measurements; all task/geometry consumers use this.
    ZonePoseMetadata pose_metadata{};
    ZonePoseRoute pose_route{};
    PosePreview display;
    bool preview_enabled = false;
    double preview_ms = 0;
    bool pose_ran = false;
    double detect_ms = 0, pose_ms = 0, total_ms = 0;
};

// Detector plus optional safe-zone pose model on one shared 640 letterbox. Each
// RKNN context is only ever driven by one caller at a time; with --parallel-infer
// the pose context runs on a joined worker while the detector runs here. The
// worker finishes before infer returns or propagates an exception.
class PerceptionPipeline {
  public:
    explicit PerceptionPipeline(const Config& config);
    void resetForNewRun() { smoother_.reset(); }
    bool hasPose() const {
        return pose_ != nullptr;
    }
    PerceptionResult infer(const cv::Mat& frame,
                           uint64_t timestamp_us,
                           const ModelImage* prepared = nullptr,
                           PoseFrameIdentity identity = {});

  private:
    PoseTemporalSmoother smoother_;
    Config config_;
    std::unique_ptr<IDetector> detector_;
    YoloRknnDetector* rknn_ = nullptr;
    std::unique_ptr<SafeZoneKeypointRknn> pose_;
};
} // namespace rescue
