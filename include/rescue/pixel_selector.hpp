#pragma once
#include "rescue/perception_adapter.hpp"
#include <map>
#include <set>
namespace rescue {
struct PixelSelection {
    int id=-1; double raw=0, smoothed=0;
    std::string mode="none", reason="no_candidate";
    bool locked=false;
};
// Masks must already be mapped to the original image. Never resize an unknown mask.
class PixelSelector {
    struct Track { double value=0; std::string label; };
    std::map<int,Track> history_;
    cv::Size size_; int16_t pitch_=kCameraPitchInvalid;
    std::string mode_; int locked_=-1, challenger_=-1, count_=0;
    uint64_t last_=0,last_capture_=0;
public:
    void reset(){history_.clear();locked_=-1;challenger_=-1;count_=0;last_=0;last_capture_=0;}
    PixelSelection update(const std::vector<SegDetection>& ds, cv::Size size,
        int16_t pitch, bool stable, uint64_t now, bool first, float confidence,
        const std::vector<int>& rejected={}, const std::string& prefer_label="") {
        PixelSelection out;
        if(size.width<=0||size.height<=0||!stable||pitch==kCameraPitchInvalid){reset();out.reason="invalid_image_or_pitch";return out;}
        if(last_ && now>last_ && now-last_>200000)reset();
        uint64_t capture_stamp=0;
        bool masks=false, boxes=false;
        std::map<int,std::pair<double,std::string>> values;
        for(const auto& d:ds){
            if(d.track_id<0||!targetSelectable(d.label,first)||!std::isfinite(d.confidence)||d.confidence<confidence||
                !d.timestamp_us||now<d.timestamp_us||now-d.timestamp_us>200000||d.box.width<=0||d.box.height<=0||
                std::find(rejected.begin(),rejected.end(),d.track_id)!=rejected.end())continue;
            const auto clipped=d.box & cv::Rect(0,0,size.width,size.height);
            if(clipped.empty())continue;
            double pixels=clipped.area();
            if(!d.mask.empty()){
                if(d.mask.size()!=size||d.mask.type()!=CV_8UC1)continue;
                pixels=cv::countNonZero(d.mask);masks=true;
            }else boxes=true;
            capture_stamp=std::max(capture_stamp,d.timestamp_us);
            if(pixels>0)values[d.track_id]={pixels/(double(size.width)*size.height),d.label};
        }
        if(!prefer_label.empty()&&std::any_of(values.begin(),values.end(),[&](const auto& kv){return kv.second.second==prefer_label;}))
            for(auto it=values.begin();it!=values.end();)if(it->second.second!=prefer_label)it=values.erase(it);else ++it;
        const std::string mode=masks?(boxes?"mixed_invalid":"mask_pixels"):"box_area";
        if(size!=size_||pitch!=pitch_||mode!=mode_){reset();out.reason="comparison_context_changed";}
        size_=size;pitch_=pitch;mode_=mode;out.mode=mode;
        if(masks&&boxes){reset();out.reason="mixed_pixel_modes";return out;}
        if(!now||now<=last_||(capture_stamp&&capture_stamp<=last_capture_)){out.reason="duplicate_or_old_frame";return out;}
        last_=now;last_capture_=capture_stamp;
        for(auto it=history_.begin();it!=history_.end();)if(!values.count(it->first))it=history_.erase(it);else ++it;
        if(locked_>=0 && values.count(locked_) && history_.count(locked_) && values[locked_].second!=history_[locked_].label){locked_=-1;challenger_=-1;count_=0;}
        int best=-1;
        for(const auto& [id,v]:values){
            auto it=history_.find(id);
            if(it==history_.end()||it->second.label!=v.second)history_[id]={v.first,v.second};
            else it->second.value=.5*v.first+.5*it->second.value;
            if(best<0||history_[id].value>history_[best].value)best=id;
        }
        if(locked_>=0 && !values.count(locked_)){locked_=-1;count_=0;challenger_=-1;}
        if(best<0){out.reason="no_legal_candidate";return out;}
        const bool challenge=locked_<0||(best!=locked_&&history_[best].value>1.2*history_[locked_].value);
        if(challenge){if(challenger_!=best){challenger_=best;count_=0;}if(++count_>=3){locked_=best;count_=0;challenger_=-1;}}
        else{count_=0;challenger_=-1;}
        out.locked=locked_>=0;out.id=locked_;out.reason=out.locked?"pixel_ratio_locked":"confirming_three_frames";
        const int shown=out.locked?locked_:best;out.raw=values[shown].first;out.smoothed=history_[shown].value;
        return out;
    }
};
}
