#pragma once
#include "rescue/perception/zone_keypoints.hpp"
#include <cstdint>
#include <string>

namespace rescue {
// Fixed reference parameters; coordinates/velocity are in model-input pixels.
struct PoseSmoothingParameters {
    static constexpr double min_cutoff_hz = 1.2, max_cutoff_hz = 8.0;
    static constexpr double beta = .016, derivative_cutoff_hz = 1.0;
    static constexpr double max_gap_seconds = .2, box_reset_iou = .2;
    static constexpr float visibility = .5f, detection = .5f;
};
struct PoseFrameIdentity {
    uint64_t frame_id = 0; // zero: use the capture timestamp as identity
    std::string source, model;
    bool pitch_valid = false;
    int pitch_cdeg = 0;
};
struct PosePreview {
    ZoneHalves halves{};
    std::array<std::array<bool, 4>, 2> visible{};
    std::array<std::array<std::string, 4>, 2> status{};
    std::array<std::string, 2> reset_reason{};
    uint64_t frame_id = 0, measurement_us = 0;
    bool ok = true;
    std::string error;
};
bool posePreviewVisible(const ZoneHalf& half, int slot, cv::Size size);
PosePreview
rawPosePreview(const ZoneHalves& raw, cv::Size size, uint64_t frame_id, uint64_t timestamp_us);
class PoseTemporalSmoother {
  public:
    PosePreview update(const ZoneHalves& raw,
                       const ZonePoseMetadata& metadata,
                       const ModelImage& image,
                       cv::Size size,
                       uint64_t timestamp_us,
                       PoseFrameIdentity identity);
    void reset();

  private:
    struct PointState {
        bool valid = false;
        cv::Point2d raw{}, filtered{}, velocity{};
        uint64_t timestamp_us = 0;
    };
    std::array<std::array<PointState, 4>, 2> points_{};
    std::array<cv::Rect2f, 2> boxes_{};
    std::array<bool, 2> box_valid_{};
    bool have_previous_ = false;
    uint64_t last_time_ = 0, last_id_ = 0;
    std::string context_, signature_;
    PosePreview cached_;
};
} // namespace rescue
