#pragma once
#include "rescue/task/push_task.hpp"
#include <vector>
#include "rescue/geometry/zone_geometry.hpp"
#include "rescue/perception/zone_keypoints.hpp"
namespace rescue {
// Search cues consider all known classes. Cargo selection enforces the first
// ordinary delivery, then ranks feasible local loads through chooseTrip.
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
// pile_approach uses the union centre of visible cargo at UI120. An optional,
// caller-owned guide retains steering members for 150 ms and filters centre X
// over three fresh images. Coherent member motion is retained; membership-only
// centre changes are limited to 10 px per fresh image. Scene/safety evidence
// and target_box remain current.
// Ordinary selection and other views retain box targeting.
class StartupPileGuide {
  public:
    cv::Point2f update(const std::vector<const SegDetection*>& cargo, cv::Size size,
                       int16_t pitch, uint64_t now_us);
    void reset() { *this = {}; }

  private:
    struct Member {
        int id;
        std::string label;
        cv::Rect box;
        uint64_t sample_us;
        unsigned sequence;
        int hits;
        bool confirmed;
    };
    std::vector<Member> members_;
    std::vector<float> centers_;
    cv::Size size_;
    int16_t pitch_ = kCameraPitchInvalid;
    uint64_t sample_us_ = 0;
    unsigned sequence_ = 0;
    cv::Point2f center_;
    float tracked_center_x_ = 0;
};
void applyStartupSight(PushObservation&, const std::vector<SegDetection>&, cv::Size,
                       uint64_t now_us, float min_confidence, int preferred_id,
                       bool ordinary_only = false, int rejected_id = -1,
                       const StartupSightReference& reference = {}, bool pile_approach = false,
                       StartupPileGuide* pile_guide = nullptr);
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
// Evaluate only a confirmed search lock, using the current image. No visible own zone
// means outside for search; this does not change measured geometry or delivery evidence.
void applySearchTargetRegion(PushObservation&, const std::vector<SegDetection>&,
                              const std::string& team, cv::Size image_size,
                              bool stable, uint64_t capture_us, uint64_t now_us,
                              float min_confidence, bool own_pose_visible = false);
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
