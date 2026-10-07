#pragma once
#include "rescue/types.hpp"
#include "rescue/transport_rules.hpp"
#include <opencv2/imgproc.hpp>
#include <vector>
namespace rescue {
enum class CaptureVerdict { Uncertain, Enclosed, Empty };
inline const char* captureVerdictName(CaptureVerdict v){return v==CaptureVerdict::Enclosed?"enclosed":v==CaptureVerdict::Empty?"not_enclosed":"uncertain";}
struct FrameView {
    int16_t pitch_cdeg=kCameraPitchInvalid;
    std::vector<cv::Point2f> polygon;
    bool calibrated=false, fully_observable=false;
};
struct MultiViewResult {
    CaptureVerdict verdict=CaptureVerdict::Uncertain;
    Inventory inventory;
    int view=0, frames=0;
    uint64_t round=0;
    bool finished=false;
    std::string reason="not_started";
};
// Each observation round belongs to one close transaction and one immutable configuration.
// Matching uses stationary body coordinates + class, never image coordinates or track ids.
class MultiViewCapture {
    struct Object { std::string label; cv::Point2f position; };
    struct Evidence { bool valid=false; std::vector<Object> objects; };
    std::vector<FrameView> views_;
    std::vector<Evidence> evidence_;
    cv::Size size_;
    MultiViewResult result_;
    Evidence current_;
    uint64_t started_=0, view_started_=0, last_frame_=0;
    uint8_t action_=0;
    static bool match(const Evidence& a,const Evidence& b){
        if(!a.valid||!b.valid||a.objects.size()!=b.objects.size())return false;
        std::vector<bool> used(b.objects.size());
        for(const auto& x:a.objects){int candidate=-1;
            for(size_t j=0;j<b.objects.size();++j)if(x.label==b.objects[j].label&&cv::norm(x.position-b.objects[j].position)<.025){if(candidate>=0)return false;candidate=int(j);}
            if(candidate<0||used[candidate])return false;
            used[candidate]=true;
        }return true;
    }
    void advance(uint64_t now){
        evidence_.push_back(current_);current_={};result_.frames=0;
        // Require two consistent independent views. A contradictory completed view vetoes success.
        if(evidence_.size()>=2){
            bool all=true;for(const auto& e:evidence_)all=all&&match(evidence_[0],e);
            if(all){result_.finished=true;result_.inventory={};for(const auto& o:evidence_[0].objects)result_.inventory.add(o.label);
                result_.verdict=result_.inventory.total()?CaptureVerdict::Enclosed:CaptureVerdict::Empty;
                result_.reason="consistent_independent_views";return;}
        }
        if(++result_.view>=int(views_.size())){result_.finished=true;result_.reason="view_conflict_or_unobservable";return;}
        view_started_=now;
    }
public:
    MultiViewCapture(std::vector<FrameView> views={},cv::Size size={}):views_(std::move(views)),size_(size){}
    void reset(){started_=0;action_=0;evidence_.clear();current_={};result_={};last_frame_=0;}
    void begin(uint64_t now,uint8_t action,uint64_t round){reset();started_=view_started_=now;action_=action;result_.round=round;
        if(action==0||views_.size()<2||views_.size()>3){result_.finished=true;result_.reason="missing_calibrated_views_or_action";}}
    int16_t desiredPitch()const{return result_.view<int(views_.size())?views_[result_.view].pitch_cdeg:kCameraPitchInvalid;}
    const MultiViewResult& result()const{return result_;}
    void update(uint64_t now,uint64_t frame_stamp,uint8_t action,bool closed,bool stationary,
        int16_t pitch,bool stable,const std::vector<SegDetection>& ds){
        if(!started_||result_.finished)return;
        if(action!=action_||!closed||!stationary||now<started_){result_.finished=true;result_.reason="round_invalidated";return;}
        if(now-started_>8000000){result_.finished=true;result_.reason="observation_budget_exhausted";return;}
        if(now-view_started_>2000000){current_={};advance(now);return;}
        if(!stable||std::abs(int(pitch)-int(desiredPitch()))>100){current_={};result_.frames=0;return;}
        if(!frame_stamp||frame_stamp<=last_frame_||frame_stamp<view_started_||now<frame_stamp||now-frame_stamp>200000)return;
        last_frame_=frame_stamp;
        const auto& v=views_[result_.view];Evidence sample;sample.valid=v.calibrated&&v.fully_observable&&v.polygon.size()>=3;
        if(!sample.valid||size_.width<=0||size_.height<=0){current_={};result_.frames=0;result_.reason="uncalibrated_or_unobservable_view";return;}
        std::vector<cv::Rect> occupied;
        for(const auto& d:ds){
            if(!std::isfinite(d.confidence)||d.confidence<.1f)continue;
            if(!d.timestamp_us||now<d.timestamp_us||now-d.timestamp_us>200000){sample.valid=false;continue;}
            const cv::Rect b=d.box; if(b.width<=0||b.height<=0){sample.valid=false;continue;}
            const std::vector<cv::Point2f> corners={{float(b.x),float(b.y)},{float(b.x+b.width),float(b.y)},
                {float(b.x+b.width),float(b.y+b.height)},{float(b.x),float(b.y+b.height)}};
            bool inside=true;for(auto p:corners)inside=inside&&cv::pointPolygonTest(v.polygon,p,true)>6;
            const bool touches=!(b & cv::boundingRect(v.polygon)).empty();
            if(!touches)continue;
            if(!inside||b.x<=0||b.y<=0||b.br().x>=size_.width||b.br().y>=size_.height||d.confidence<.25f||
               !d.ground_position_valid||!d.ground_contact_valid||!std::isfinite(d.body_xy_m.x)||!std::isfinite(d.body_xy_m.y)||
               targetKind(d.label)==TargetKind::DANGEROUS||targetKind(d.label)==TargetKind::UNKNOWN){sample.valid=false;continue;}
            for(const auto& other:occupied)if(!(b&other).empty())sample.valid=false;
            occupied.push_back(b);sample.objects.push_back({d.label,d.body_xy_m});
        }
        if(!sample.valid){current_={};result_.frames=0;result_.reason="occlusion_boundary_or_association_unknown";return;}
        if(!match(current_,sample)){current_=sample;result_.frames=1;}else ++result_.frames;
        result_.reason="collecting_view";
        if(result_.frames>=5)advance(now);
    }
};
}
