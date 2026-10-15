#include "rescue/zone_color.hpp"
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <cmath>
#include <algorithm>
namespace rescue {
ZoneColorResult previewBlueZoneDefault(ZoneColorResult result, bool dry_run, const std::string& team,
                                       bool blue_model_detected_both_halves) {
    if (dry_run && team == "blue" && blue_model_detected_both_halves &&
        !result.verified && result.reason == "color_unmeasured") {
        result.color = "blue";
        result.reason = "blue_pose_model_assumption_preview_only";
        result.assumed = true;
    }
    return result;
}
ZoneColorConfig ZoneColorConfig::load(const std::string& path){
    cv::FileStorage f(path,cv::FileStorage::READ);ZoneColorConfig c;
    if(!f.isOpened()||int(f["schema_version"])!=1||int(f["measured"])!=1)
        throw std::runtime_error("Zone color calibration must be measured, schema_version=1");
    for(const char* key:{"saturation_min","value_min","min_pixels","min_fraction","max_other_fraction"})
        if(f[key].empty() || (!f[key].isInt()&&!f[key].isReal()))throw std::runtime_error(std::string("Missing zone color threshold: ")+key);
    c.saturation_min=int(f["saturation_min"]);c.value_min=int(f["value_min"]);c.min_pixels=int(f["min_pixels"]);
    c.min_fraction=float(f["min_fraction"]);c.max_other_fraction=float(f["max_other_fraction"]);
    if(!f["min_coloured_fraction"].empty())c.min_coloured_fraction=float(f["min_coloured_fraction"]);
    if(!f["expand_x"].empty())c.expand_x=float(f["expand_x"]);
    if(!f["expand_near"].empty())c.expand_near=float(f["expand_near"]);
    if(c.saturation_min<1||c.saturation_min>255||c.value_min<1||c.value_min>255||c.min_pixels<50||
       !std::isfinite(c.min_fraction)||c.min_fraction<.5||c.min_fraction>1||
       !std::isfinite(c.max_other_fraction)||c.max_other_fraction<0||c.max_other_fraction>=.5||
       !std::isfinite(c.min_coloured_fraction)||c.min_coloured_fraction<0||c.min_coloured_fraction>=1||
       !std::isfinite(c.expand_x)||c.expand_x<0||c.expand_x>1||!std::isfinite(c.expand_near)||c.expand_near<0||c.expand_near>1.5)
        throw std::runtime_error("Invalid zone color thresholds");
    c.measured=true;return c;
}
ZoneColorResult resolveZoneColor(ZoneColorResult measured,const std::string& team,bool assume_team){
    if(measured.verified){
        measured.assumed=false;
        if(measured.color!=team)measured.reason="enemy_zone_color";
        return measured;
    }
    if(assume_team){
        measured.color=team;measured.verified=true;measured.assumed=true;measured.reason="assumed_team_colour";
    }
    return measured;
}
ZoneColorResult ZoneColorClassifier::classify(const cv::Mat& frame,const ZoneHalves& halves,
                                              const std::vector<SegDetection>& objects,float threshold) const {
    ZoneColorResult result;
    if(!config_.measured)return result;
    if(frame.empty()||frame.type()!=CV_8UC3){result.reason="invalid_color_frame";return result;}
    cv::Mat hsv;cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
    std::string colors[2];std::string skip_reason="no_zone_half";
    for(int side=0;side<2;++side) {
        const auto& half=halves[side];std::vector<cv::Point> polygon;
        if(!half.valid){skip_reason="no_zone_half";continue;}
        bool usable=true;
        for(int index:{0,2,3,1}) {
            const auto& p=half.keypoints[index];
            if(!std::isfinite(p.x)||!std::isfinite(p.y)||p.x<0||p.y<0||p.x>=frame.cols||p.y>=frame.rows||
               !std::isfinite(half.confidence[index])||half.confidence[index]<threshold) {
                skip_reason="color_keypoints_invalid";usable=false;break;
            }
            polygon.emplace_back(cvRound(p.x),cvRound(p.y));
        }
        if(!usable)continue;
        if(!cv::isContourConvex(polygon)){skip_reason="color_polygon_invalid";continue;}
        {   // polygon order is far-left, far-right, near-right, near-left
            float min_x=polygon[0].x,max_x=min_x,min_y=polygon[0].y,max_y=min_y;
            for(const auto& q:polygon){min_x=std::min(min_x,float(q.x));max_x=std::max(max_x,float(q.x));
                                       min_y=std::min(min_y,float(q.y));max_y=std::max(max_y,float(q.y));}
            const float cx=(min_x+max_x)*.5f,half_w=(max_x-min_x)*.5f*config_.expand_x,drop=(max_y-min_y)*config_.expand_near;
            for(size_t i=0;i<polygon.size();++i){
                polygon[i].x=std::clamp(int(std::lround(polygon[i].x+(polygon[i].x<cx?-half_w:half_w))),0,frame.cols-1);
                if(i>=2)polygon[i].y=std::clamp(int(std::lround(polygon[i].y+drop)),0,frame.rows-1);
            }
        }
        cv::Mat mask=cv::Mat::zeros(frame.size(),CV_8U);cv::fillConvexPoly(mask,polygon,255);
        const int initial=cv::countNonZero(mask);
        for(const auto& object:objects) {
            auto box=object.box;box.x-=5;box.y-=5;box.width+=10;box.height+=10;
            box&=cv::Rect(0,0,frame.cols,frame.rows);if(!box.empty())mask(box).setTo(0);
        }
        int total=0,red=0,blue=0;
        for(int y=0;y<frame.rows;++y)for(int x=0;x<frame.cols;++x)if(mask.at<uint8_t>(y,x)) {
            ++total;const auto p=hsv.at<cv::Vec3b>(y,x);
            if(p[1]<config_.saturation_min||p[2]<config_.value_min)continue;
            if(p[0]<=12||p[0]>=168)++red;
            if(p[0]>=95&&p[0]<=135)++blue;
        }
        if(total<config_.min_pixels || total<initial*.3){skip_reason="color_occluded";continue;}
        const int coloured=red+blue;
        if(coloured<config_.min_pixels || coloured<total*config_.min_coloured_fraction){result.reason="color_too_faint";return result;}
        const float r=float(red)/coloured,b=float(blue)/coloured;
        if(r>=config_.min_fraction&&b<=config_.max_other_fraction)colors[side]="red";
        else if(b>=config_.min_fraction&&r<=config_.max_other_fraction)colors[side]="blue";
        else {result.reason="color_ambiguous";return result;}
        ++result.halves_used;
    }
    if(result.halves_used==0){result.reason=skip_reason;return result;}
    if(result.halves_used==2&&colors[0]!=colors[1]) {result.reason="color_halves_disagree";return result;}
    result.color=colors[0].empty()?colors[1]:colors[0];result.verified=true;result.reason="ok";return result;
}
}
