#include "rescue/perception_adapter.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
namespace rescue {
PushObservation makePushObservation(const std::vector<SegDetection>& detections,
        uint64_t now, bool first, float threshold, int locked_id,
        const std::vector<int>& rejected, bool search, bool locked_is_search_cue, bool require_geometry) {
    PushObservation in; in.now_us=now;
    const SegDetection* best=nullptr;
    const auto rank=[&](const SegDetection& d) {
        if (!first) return d.label=="ordinary_supply"?0:1;
        return d.label=="core_supply"?0:d.label=="ordinary_supply"?1:d.label=="injured_person"?2:3;
    };
    const auto distance=[](const SegDetection& d) {
        return d.ground_position_valid ? cv::norm(d.body_xy_m) : std::numeric_limits<double>::infinity();
    };
    const bool locked_green=std::any_of(detections.begin(),detections.end(),[&](const auto& d){
        return d.track_id==locked_id && d.label=="ordinary_supply";
    });
    const bool allow_green_preemption=search && locked_is_search_cue && !first && !locked_green;
    for(const auto& d:detections) {
        if((locked_id>=0 && d.track_id!=locked_id && !(allow_green_preemption && d.label=="ordinary_supply")) ||
           std::find(rejected.begin(),rejected.end(),d.track_id)!=rejected.end())continue;
        if((search||require_geometry) && (!d.ground_position_valid || !d.ground_contact_valid ||
           !std::isfinite(d.body_xy_m.x)||!std::isfinite(d.body_xy_m.y)||d.body_xy_m.y<=0))continue;
        if(search ? targetKind(d.label)==TargetKind::UNKNOWN : !targetSelectable(d.label,first))continue;
        if(d.track_id<0 || !std::isfinite(d.confidence)||d.confidence<threshold || !d.timestamp_us ||
           now<d.timestamp_us||now-d.timestamp_us>200000||d.box.width<=0||d.box.height<=0)continue;
        if(!best || rank(d)<rank(*best) || (rank(d)==rank(*best) &&
           (distance(d)<distance(*best) || (distance(d)==distance(*best)&&d.confidence>best->confidence))))best=&d;
    }
    if(best){in.target_valid=true;in.target_id=best->track_id;in.label=best->label;in.target_is_search_cue=search;}
    return in;
}
}
