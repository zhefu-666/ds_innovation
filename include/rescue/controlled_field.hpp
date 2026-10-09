#pragma once
#include "rescue/push_task.hpp"
#include "rescue/zone_keypoints.hpp"
namespace rescue {
// This adapter is only for an operator-cleared, own-zone-only test enclosure.
// It must never be enabled automatically or described as autonomous free-space sensing.
class ControlledField {
public:
    explicit ControlledField(ZoneGeometry geometry,float body_radius,cv::Vec<float,6> envelope={}):envelope_(envelope),geometry_(std::move(geometry)),body_radius_(body_radius){}
    void update(PushObservation&,const std::vector<SegDetection>&,const ZoneHalves&,cv::Size,
                const PushOutput&,bool mapping_valid,bool ignore_clearance=false);
    const std::string& reason() const {return reason_;}
private:
    cv::Vec<float,6> envelope_;
    ZoneGeometry geometry_;
    float body_radius_;
    std::vector<PlannerObstacle> last_objects_;
    int last_supply_=-1,last_injured_=-1,stable_frames_=0;
    uint64_t stable_since_=0,last_frame_=0;
    std::string reason_="zone_not_observed";
    void invalidate(const std::string& reason);
};
}
