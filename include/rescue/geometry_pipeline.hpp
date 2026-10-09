#pragma once
#include <deque>
#include <limits>
#include <map>
#include "rescue/ground_pose_fitter.hpp"
#include "rescue/safe_zone_pose.hpp"
#include "rescue/zone_layout.hpp"
#include "rescue/push_task.hpp"
#include "rescue/silhouette.hpp"
namespace rescue {
struct ExpectedStop {
    bool valid=false;
    int target_id=-1;
    uint64_t frame_id=0,capture_us=0;
    cv::Point2f body_m;
    float radius_m=0;
};
struct GeometryResult {
    bool mapping_valid=false;
    bool identity_verified=false;
    std::string reason="not_configured";
    ZoneEstimate zone;
    std::vector<SegDetection> detections;
};
// Ground contact producer (stage HW2). A detection's floor contact is accepted only when
// the box is consistent with the class solid (2027 rules p.38) resting on the floor at the
// projected box-bottom point: valid frame (pitch/IMU synchronized, calibrated pitch, tilt),
// validated extrinsic model, known class, confident, box clear of every image edge, within
// range, and observed box width AND height inside the range predicted by projecting the
// solid at that point over all yaw angles. Merged neighbours, stacked blocks, truncated or
// heavily occluded boxes fail the size check; small elevations (<~15% of camera height)
// and occlusion below the tolerance cannot be seen from one box and are not claimed.
// Dangerous objects get contact too (for avoidance only; targetSelectable never picks them).
struct GroundContactConfig {
    float min_confidence=.5f;
    int edge_margin_px=4;
    float max_range_m=3.f;
    float size_tolerance=.35f; // relative slack for loose detector boxes
    bool accept_size_mismatch=false; // TEMP_ASSUMPTION only: 20/40 deg view size model is not field-validated; keep the bottom-edge position
    bool injured_lying_only=false; // injured_person is only grabbed lying (sideways); on-end (upright) is rejected
    float upright_margin=.05f; // upright is declared only when its fit error is this much smaller than the lying fit
    int upright_vote_frames=7; // per-track window of recent frames; the lower median of (lying err - upright err) decides
    int upright_vote_min=3; // fewer evaluated frames than this: the single-frame verdict is used
    uint64_t upright_vote_window_us=1500000; // older votes of the same track are dropped
    float upright_vote_jump_m=.30f; // the track jumped this far between frames: treated as a new object, votes reset
    float upright_release_ratio=.4f; // a track already judged upright is released only when its median falls below margin * ratio
    // Real-robot data: lying blocks fit the lying pose clearly better (median diff -0.09..-0.6) while on-end blocks sit near 0 (-0.06..+0.09).
    // An injured block is a candidate only when its voted median is below this; +inf keeps the old "reject only when clearly upright".
    float lying_confirm_diff=std::numeric_limits<float>::infinity();
    bool use_silhouette=false; // injured_lying_only: the orange silhouette aspect (SegDetection::silhouette_hw) overrides the box-fit diff when clearly wide or tall
    SilhouetteRule silhouette;
    float lying_confirm_hysteresis=.02f; // a track judged unconfirmed regains candidacy only below lying_confirm_diff - this
};
// Resting poses of the class solid: vertices in metres, footprint centred at the origin,
// z up. False for labels whose physical shape is unknown.
bool classSolids(const std::string& label,std::vector<std::vector<cv::Point3f>>& poses);
class GeometryPipeline {
public:
    GeometryPipeline(CameraCalibration calibration,ZoneGeometry geometry,GroundContactConfig contact={});
    GeometryResult process(const GeometryFrame&,const KeypointFrame&,const std::vector<SegDetection>&);
    void apply(PushObservation&,const GeometryResult&,const ExpectedStop&,const std::string& team,uint64_t now_us) const;
private:
    std::string groundContact(SegDetection&,const GeometryFrame&);
    bool voteUpright(const SegDetection&,float single_diff,int& n,float& median);
    struct PoseVote{uint64_t t;float diff;cv::Point2f xy;};
    struct TrackVotes{std::deque<PoseVote> q;bool upright=false,unconfirmed=false;};
    std::map<int,TrackVotes> pose_votes_;
    CameraCalibration calibration_;
    ZoneGeometry geometry_;
    KeypointFilter filter_;
    GroundPoseFitter fitter_;
    SafeZonePoseEstimator pnp_;
    GroundContactConfig contact_;
};
// External keypoints are accepted only with an exact frame/capture/schema match.
// No boxes are expanded into invented landmarks. Used by offline annotations and
// the future stage-3 decoder; the geometry pipeline is identical in both cases.
KeypointFrame readKeypointFrame(const cv::FileNode& node);
ExpectedStop readExpectedStop(const cv::FileNode& node,const KeypointFrame& frame);
uint64_t readExactTime(const cv::FileNode& node);
int runGeometryReplay(const std::string& path,const std::string& calibration,
                      const std::string& geometry,const std::string& team);
} // namespace rescue
