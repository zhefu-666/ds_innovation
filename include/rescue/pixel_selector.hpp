#pragma once
#include "rescue/perception_adapter.hpp"
#include "rescue/transport_rules.hpp"
#include "rescue/row_selector.hpp"
#include <map>
#include <set>
namespace rescue {
struct PixelSelection {
    int id=-1; double raw=0, smoothed=0;
    std::string mode="none", reason="no_candidate";
    bool locked=false;
    bool row_split=false; // row mode: the locked ordinary block has an injured/dangerous/core piece in its frame footprint (to be separated, not skipped)
    std::string row; // row-mode note: seq / count / chosen index / reason; empty when row mode is off
};
// Masks must already be mapped to the original image. Never resize an unknown mask.
class PixelSelector {
    struct Track { double value=0; std::string label; };
    std::map<int,Track> history_;
    cv::Size size_; int16_t pitch_=kCameraPitchInvalid;
    std::string mode_; int locked_=-1, challenger_=-1, count_=0;
    uint64_t last_=0,last_capture_=0;
    int miss_=0; bool have_center_=false; cv::Point2f last_center_,challenger_center_; uint64_t center_us_=0;
    static constexpr int kLockGrace=5;
    uint64_t relock_window_us_=1500000; double grace_gate_frac_=.05;
    // Neighbour window around a candidate: footprint half-width .07 + block half-size .02, footprint depth .081/2 + .02.
    static constexpr float kCrowdHalfX=.09f, kCrowdHalfY=.06f;
    // Pile-edge preference: a candidate with injured/dangerous pieces inside its footprint window (x10) or on
    // the robot->target line in front of it (x3, the frame would have to sweep them aside first) ranks last.
    // 0 = clean, 1 = one ordinary neighbour (still legal), >=10 = the trip rules would veto it.
    // relax 1 drops the in-front term, relax 2 ignores crowding altogether (no delivery for a long time).
    static constexpr float kCrowdMinConfidence=.25f, kLaneHalfWidth=.09f, kLaneBehindM=.07f;
    static int crowd(const std::vector<SegDetection>& ds,int id,float confidence,uint64_t now,int relax){
        if(relax>=2)return 0;
        const SegDetection* t=nullptr;
        for(const auto& d:ds)if(d.track_id==id){t=&d;break;}
        const auto usable=[&](const SegDetection& d){return d.ground_position_valid&&std::isfinite(d.body_xy_m.x)&&std::isfinite(d.body_xy_m.y);};
        if(!t||!usable(*t))return 5;
        const bool ordinary=targetKind(t->label)==TargetKind::ORDINARY;
        const float tx=t->body_xy_m.x,ty=t->body_xy_m.y,dist=std::hypot(tx,ty);
        const float ux=dist>1e-3f?tx/dist:0.f,uy=dist>1e-3f?ty/dist:1.f;
        const float floor=std::min(confidence,kCrowdMinConfidence);
        int other=0,fatal=0,front_fatal=0,front_other=0;
        for(const auto& d:ds){
            if(&d==t||d.track_id==id||d.track_id<0||!usable(d)||!std::isfinite(d.confidence)||d.confidence<floor||
                !d.timestamp_us||now<d.timestamp_us||now-d.timestamp_us>200000)continue;
            const bool same_kind=ordinary&&targetKind(d.label)==TargetKind::ORDINARY;
            if(std::abs(d.body_xy_m.x-tx)<=kCrowdHalfX&&std::abs(d.body_xy_m.y-ty)<=kCrowdHalfY){
                if(same_kind)++other;else ++fatal;
                continue;
            }
            if(relax>=1)continue;
            const float along=d.body_xy_m.x*ux+d.body_xy_m.y*uy,lateral=std::abs(d.body_xy_m.x*uy-d.body_xy_m.y*ux);
            if(along<0.f||along>dist+kLaneBehindM||lateral>kLaneHalfWidth)continue;
            if(same_kind)++front_other;else ++front_fatal;
        }
        // Exactly one more ordinary supply beside/in front (and nothing dangerous) is a legal pair: rank it first.
        // Three supplies would break the trip limit, so two or more neighbours stay vetoed.
        const bool pair=ordinary&&other+front_other==1&&fatal==0&&front_fatal==0;
        return fatal*10+(other>=kMaxSuppliesPerTrip?10:0)+front_fatal*3-(pair?1:0);
    }
    int relax_=0;
    bool row_on_=false; RowParams row_;
public:
    // Row mode: with a left-right row of >= row.min_n blocks in view the lock goes to the middle block whose frame footprint is clean, not the largest box.
    void setRow(bool on,const RowParams& p={}){row_on_=on;row_=p;}
    void setRelax(int level){relax_=std::clamp(level,0,2);}
    // How long after the last tracked frame a re-lock prefers the candidate nearest to where the lock vanished, and how far (fraction of the image width) a re-ID'd block may sit from it while the lock is only absent.
    void setRelock(uint64_t window_us,double gate_frac){relock_window_us_=window_us;grace_gate_frac_=std::clamp(gate_frac,.0,1.);}
    void forgetCenter(){have_center_=false;center_us_=0;}
    void reset(){history_.clear();locked_=-1;challenger_=-1;count_=0;last_=0;last_capture_=0;miss_=0;have_center_=false;center_us_=0;}
    PixelSelection update(const std::vector<SegDetection>& ds, cv::Size size,
        int16_t pitch, bool stable, uint64_t now, bool first, float confidence,
        const std::vector<int>& rejected={}, const std::string& prefer_label="") {
        PixelSelection out;
        if(size.width<=0||size.height<=0||!stable||pitch==kCameraPitchInvalid){reset();out.reason="invalid_image_or_pitch";return out;}
        if(last_ && now>last_ && now-last_>200000)reset();
        uint64_t capture_stamp=0;
        bool masks=false, boxes=false;
        std::map<int,std::pair<double,std::string>> values;
        std::set<int> geo_ok;
        std::map<int,cv::Point2f> centers;
        std::set<int> vetoed; // explicitly rejected ids release the lock at once; only mere absence gets grace
        for(int id:rejected)vetoed.insert(id);
        for(const auto& d:ds){
            if(d.injured_upright_rejected){vetoed.insert(d.track_id);continue;} // an on-end injured block never competes for the lock
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
            if(pixels>0){
                values[d.track_id]={pixels/(double(size.width)*size.height),d.label};
                centers[d.track_id]=cv::Point2f(clipped.x+clipped.width*.5f,clipped.y+clipped.height*.5f);
                if(d.ground_position_valid&&d.ground_contact_valid&&std::isfinite(d.body_xy_m.x)&&std::isfinite(d.body_xy_m.y)&&d.body_xy_m.y>0)geo_ok.insert(d.track_id);
            }
        }
        if(!prefer_label.empty()&&std::any_of(values.begin(),values.end(),[&](const auto& kv){return kv.second.second==prefer_label;}))
            for(auto it=values.begin();it!=values.end();)if(it->second.second!=prefer_label)it=values.erase(it);else ++it;
        // A larger box without usable ground contact (e.g. clipped at the image edge after a pitch change)
        // must not out-rank a measurable one: the task could never approach it.
        if(std::any_of(values.begin(),values.end(),[&](const auto& kv){return geo_ok.count(kv.first)>0;}))
            for(auto it=values.begin();it!=values.end();)if(!geo_ok.count(it->first))it=values.erase(it);else ++it;
        const std::string mode=masks?(boxes?"mixed_invalid":"mask_pixels"):"box_area";
        if(size!=size_||pitch!=pitch_||mode!=mode_){reset();out.reason="comparison_context_changed";}
        size_=size;pitch_=pitch;mode_=mode;out.mode=mode;
        if(masks&&boxes){reset();out.reason="mixed_pixel_modes";return out;}
        if(!now||now<=last_||(capture_stamp&&capture_stamp<=last_capture_)){out.reason="duplicate_or_old_frame";return out;}
        last_=now;last_capture_=capture_stamp;
        // A tracker dropout of a few frames must not discard the lock or its smoothed history.
        bool lock_absent=false;
        if(locked_>=0 && vetoed.count(locked_)){locked_=-1;count_=0;challenger_=-1;miss_=0;have_center_=false;}
        else if(locked_>=0 && !values.count(locked_)){
            if(++miss_>kLockGrace){locked_=-1;count_=0;challenger_=-1;miss_=0;}
            else lock_absent=true;
        }else miss_=0;
        for(auto it=history_.begin();it!=history_.end();)if(!values.count(it->first)&&!(lock_absent&&it->first==locked_))it=history_.erase(it);else ++it;
        if(locked_>=0 && values.count(locked_) && history_.count(locked_) && values[locked_].second!=history_[locked_].label){locked_=-1;challenger_=-1;count_=0;}
        int best=-1;
        for(const auto& [id,v]:values){
            auto it=history_.find(id);
            if(it==history_.end()||it->second.label!=v.second)history_[id]={v.first,v.second};
            else it->second.value=.5*v.first+.5*it->second.value;
            if(best<0||history_[id].value>history_[best].value)best=id;
        }
        if(best<0){
            out.reason=lock_absent?"lock_grace_waiting":"no_legal_candidate";
            return out;
        }
        // A fresh acquisition prefers the least crowded comparable candidate: a block whose footprint would
        // swallow an injured/dangerous piece or a third supply is vetoed later anyway, so do not drive at it first.
        int row_pick=-1;
        std::vector<int> row_split_ids;
        if(row_on_){
            const auto rp=RowSelector::pick(ds,now,first,confidence,locked_,row_);
            out.row="seq="+rp.seq+" n="+std::to_string(rp.n)+" idx="+std::to_string(rp.index)+" id="+std::to_string(rp.id)+" "+rp.reason;
            if(rp.id>=0&&values.count(rp.id))row_pick=rp.id;
            row_split_ids=rp.split_ids;
        }
        const bool recent_lock=have_center_&&now>=center_us_&&now-center_us_<=relock_window_us_;
        if(row_pick<0&&(locked_<0&&!recent_lock)&&values.size()>1){
            const double floor=.3*history_[best].value;
            int pick=best,pen=crowd(ds,best,confidence,now,relax_);
            for(const auto& [id,v]:values){
                if(history_[id].value<floor)continue;
                const int p=crowd(ds,id,confidence,now,relax_);
                if(p<pen||(p==pen&&history_[id].value>history_[pick].value)){pick=id;pen=p;}
            }
            if(challenger_>=0&&challenger_!=pick&&values.count(challenger_)&&history_[challenger_].value>=floor&&
                (crowd(ds,challenger_,confidence,now,relax_)<=pen||crowd(ds,challenger_,confidence,now,relax_)<10))pick=challenger_;
            best=pick;
        }
        // When (re)locking, prefer the comparable candidate nearest to where the previous lock last was,
        // so a neighbouring cube in a row is not picked just because it is slightly larger.
        if(row_pick>=0)best=row_pick;
        else if((locked_<0||lock_absent)&&have_center_&&now>=center_us_&&now-center_us_<=relock_window_us_){
            const double floor=.5*history_[best].value;double nearest=1e18;
            const double gate=lock_absent?grace_gate_frac_*size.width:1e18; // during grace only the same physical object (re-IDed) may take over
            int pick=-1;
            for(const auto& [id,v]:values){
                const auto c=centers.find(id);
                if(c==centers.end()||history_[id].value<floor)continue;
                const double dd=cv::norm(c->second-last_center_);
                if(dd<=gate&&dd<nearest){nearest=dd;pick=id;}
            }
            if(pick>=0)best=pick;
            else if(lock_absent){out.reason="lock_grace_waiting";count_=0;challenger_=-1;return out;}
        }
        const bool challenge=locked_<0||lock_absent||(best!=locked_&&(row_pick>=0||history_[best].value>1.2*history_[locked_].value));
        if(challenge){
            if(challenger_!=best){ // a re-ID of the same physical block (centre within 4% of the width) keeps the confirmation count
                const bool same=challenger_>=0&&centers.count(best)&&cv::norm(centers[best]-challenger_center_)<=.04*size.width;
                challenger_=best;if(!same)count_=0;
            }
            if(centers.count(best))challenger_center_=centers[best];
            if(++count_>=3){locked_=best;count_=0;challenger_=-1;miss_=0;}
        }
        else{count_=0;challenger_=-1;}
        const bool held=locked_>=0&&values.count(locked_);
        out.locked=held;out.id=held?locked_:-1;
        out.row_split=held&&std::find(row_split_ids.begin(),row_split_ids.end(),locked_)!=row_split_ids.end();
        out.reason=held?"pixel_ratio_locked":(locked_>=0?"lock_grace_waiting":"confirming_three_frames");
        const int shown=held?locked_:best;out.raw=values[shown].first;out.smoothed=history_[shown].value;
        if(held&&centers.count(locked_)){last_center_=centers[locked_];center_us_=now;have_center_=true;}
        return out;
    }
};
}
