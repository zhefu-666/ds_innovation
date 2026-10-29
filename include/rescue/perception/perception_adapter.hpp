#pragma once
#include "rescue/task/push_task.hpp"
#include <vector>
namespace rescue {
// Search mode considers all known classes; cargo mode enforces first-green and black priority.
// Detection-only evidence. Metric geometry, route/sensor safety and delivery
// fields deliberately remain invalid until their independent adapters exist.
PushObservation makePushObservation(const std::vector<SegDetection>& detections,
                                    uint64_t now_us,
                                    bool first_delivered,
                                    float min_confidence,
                                    int locked_id = -1,
                                    const std::vector<int>& rejected = {},
                                    bool allow_search_cues = false,
                                    bool locked_is_search_cue = false,
                                    bool require_geometry = false,
                                    const std::string& prefer_label = "");
// Raw ordinary detection for local split retreat. Prefer the original target, otherwise the
// ordinary nearest the image centre after the view handoff/re-ID. No ground projection needed.
void applySplitRetreatBox(PushObservation&,
                          const std::vector<SegDetection>&,
                          cv::Size image_size,
                          uint64_t now_us,
                          float min_confidence,
                          int preferred_id);
// Exclude ordinary/core/injured targets whose box overlaps or lies within 3% of the
// image's shorter side of an own-team zone box. Image evidence only; no pose required.
inline constexpr double kTargetZoneGapImageFrac = .03;
std::vector<int> targetsNearOwnZone(const std::vector<SegDetection>& detections,
                                    const std::string& team,
                                    cv::Size image_size,
                                    uint64_t now_us,
                                    float min_confidence);
// A fresh safe-zone box supplies a search bearing only, never metric pose or delivery proof.
// Prefer own-team boxes when both colors are visible; enemy-only boxes mark avoidance.
bool applyZoneDetectionSight(PushObservation& in,
                             const std::vector<SegDetection>& detections,
                             const std::string& team, cv::Size image_size,
                             const cv::Mat& camera_matrix, int16_t pitch_cdeg,
                             bool pitch_stable, uint64_t now_us, float min_confidence);
} // namespace rescue
