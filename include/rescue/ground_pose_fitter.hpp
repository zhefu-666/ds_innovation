#pragma once
#include "rescue/keypoint_filter.hpp"
namespace rescue {
struct GroundPoseConfig {
    float inlier_m=.025f,max_inlier_m=.06f,min_baseline_m=.10f;
    float max_two_point_turn_rad=.15f,max_two_point_jump_m=.12f;
    float reacquire_jump_m=.25f,reacquire_turn_rad=.35f;
    uint64_t prior_age_us=350000;
    int reacquire_frames=3;
    ZoneGate gate;
};
class GroundPoseFitter {
public:
    explicit GroundPoseFitter(GroundPoseConfig config={}) : config_(config) {}
    ZoneEstimate fit(const FilteredKeypoints&,const KeypointFrame&,const GeometryFrame&);
    void reset();
private:
    GroundPoseConfig config_;
    ZoneEstimate anchor_, pending_;
    float anchor_imu_yaw_=0;
    int pending_count_=0;
    uint64_t last_us_=0;
};
} // namespace rescue
