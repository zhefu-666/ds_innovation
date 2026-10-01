#include "rescue/perception_adapter.hpp"
#include <cmath>
#include <algorithm>
namespace rescue {
PushObservation makePushObservation(const std::vector<SegDetection> &detections,
        uint64_t now, bool first, float threshold, int locked_id, const std::vector<int>& rejected) {
    PushObservation in;in.now_us=now;
    const SegDetection *best=nullptr;
    for(const auto &d:detections) {
        if((locked_id>=0 && d.track_id!=locked_id) ||
           std::find(rejected.begin(),rejected.end(),d.track_id)!=rejected.end())continue;
        const bool allowed=targetSelectable(d.label,first);
        if(!allowed||d.track_id<0||!std::isfinite(d.confidence)||d.confidence<threshold||
           d.timestamp_us==0||now<d.timestamp_us||now-d.timestamp_us>200000||d.box.width<=0||d.box.height<=0)continue;
        // Nearest by image bottom when metric geometry is unavailable at this stage.
        if(!best||d.box.br().y>best->box.br().y||(d.box.br().y==best->box.br().y&&d.confidence>best->confidence))best=&d;
    }
    if(best){in.target_valid=true;in.target_id=best->track_id;in.label=best->label;}
    return in;
}
}
