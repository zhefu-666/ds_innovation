#include "rescue/perception_adapter.hpp"
#include <cmath>
namespace rescue {
PushObservation makePushObservation(const std::vector<SegDetection> &detections,
        uint64_t now, bool first, float threshold) {
    PushObservation in;in.now_us=now;
    const SegDetection *best=nullptr;
    for(const auto &d:detections) {
        const bool allowed=d.label=="ordinary_supply" ||
            (first && (d.label=="core_supply" || d.label=="injured_person"));
        if(!allowed||d.track_id<0||!std::isfinite(d.confidence)||d.confidence<threshold||
           d.timestamp_us==0||now<d.timestamp_us||now-d.timestamp_us>200000||d.box.width<=0||d.box.height<=0)continue;
        if(!best||d.confidence>best->confidence)best=&d;
    }
    if(best){in.target_valid=true;in.target_id=best->track_id;in.label=best->label;}
    return in;
}
}
