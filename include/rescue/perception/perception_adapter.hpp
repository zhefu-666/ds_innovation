#pragma once
#include "rescue/task/push_task.hpp"
#include <vector>
#include "rescue/geometry/zone_geometry.hpp"
#include "rescue/perception/zone_keypoints.hpp"
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
                                    const std::string& prefer_label = "",
                                    float ui140_core_confidence = 1.f);
// Startup observes all four cargo types, independently of capture eligibility/first-trip rules.
// Counts all fresh cargo boxes. Retains preferred_id exclusively when set; otherwise
// selects the greatest-centre-Y box; ordinary_only restricts candidates during Startup ordinary search/approach.
// A unique same-class overlapping re-ID within the visual hold window retains the physical target.
// Rejected IDs still contribute to scene counts but cannot be selected.
// Supplies image-only steering and the same lower-quarter view handoff used by search.
void applyStartupSight(PushObservation&, const std::vector<SegDetection>&, cv::Size,
                       uint64_t now_us, float min_confidence, int preferred_id,
                       bool ordinary_only = false, int rejected_id = -1,
                       const StartupSightReference& reference = {});
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
// Full rectangle containment of ordinary cargo in an own-team zone, from one fresh image.
// Independent of ground projection and target selection.
void applyDeliveryBoxEvidence(PushObservation&, const std::vector<SegDetection>&,
                              const std::string& team, cv::Size image_size,
                              bool pitch_stable, uint64_t now_us, float min_confidence);
// A fresh safe-zone box supplies a search bearing only, never metric pose or delivery proof.
// Prefer own-team boxes when both colors are visible; enemy-only boxes report identity only.
bool applyZoneDetectionSight(PushObservation& in,
                             const std::vector<SegDetection>& detections,
                             const std::string& team, cv::Size image_size,
                             const cv::Mat& camera_matrix, int16_t pitch_cdeg,
                             bool pitch_stable, uint64_t now_us, float min_confidence);
// Raw image landmarks align the vertical image centreline with the selected half.
// No ground-camera calibration is required for this image-space measurement.
void applySubzoneView(PushObservation&, const ZoneHalves&,
                      const ZoneGeometry&, cv::Size, bool stable, bool own_identity,
                      float supply_x, float injured_x, float min_confidence, uint64_t sample_us);
} // namespace rescue
