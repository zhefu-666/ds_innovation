#include "rescue/zone_color.hpp"
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <cmath>
namespace rescue {
ZoneColorConfig ZoneColorConfig::load(const std::string& path){
    cv::FileStorage f(path,cv::FileStorage::READ);ZoneColorConfig c;
    if(!f.isOpened()||int(f["schema_version"])!=1||int(f["measured"])!=1)
        throw std::runtime_error("Zone color calibration must be measured, schema_version=1");
    for(const char* key:{"saturation_min","value_min","min_pixels","min_fraction","max_other_fraction"})
        if(f[key].empty() || (!f[key].isInt()&&!f[key].isReal()))throw std::runtime_error(std::string("Missing zone color threshold: ")+key);
    c.saturation_min=int(f["saturation_min"]);c.value_min=int(f["value_min"]);c.min_pixels=int(f["min_pixels"]);
    c.min_fraction=float(f["min_fraction"]);c.max_other_fraction=float(f["max_other_fraction"]);
    if(c.saturation_min<1||c.saturation_min>255||c.value_min<1||c.value_min>255||c.min_pixels<50||
       !std::isfinite(c.min_fraction)||c.min_fraction<.5||c.min_fraction>1||
       !std::isfinite(c.max_other_fraction)||c.max_other_fraction<0||c.max_other_fraction>=.5)
        throw std::runtime_error("Invalid zone color thresholds");
    c.measured=true;return c;
}
ZoneColorResult ZoneColorClassifier::classify(const cv::Mat& frame,const ZoneHalves& halves,
                                              const std::vector<SegDetection>& objects,float threshold) const {
    ZoneColorResult result;
    if(!config_.measured)return result;
    if(frame.empty()||frame.type()!=CV_8UC3){result.reason="invalid_color_frame";return result;}
    cv::Mat hsv;cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
    std::string colors[2];
    for(int side=0;side<2;++side) {
        const auto& half=halves[side];std::vector<cv::Point> polygon;
        if(!half.valid){result.reason="both_halves_required";return result;}
        for(int index:{0,2,3,1}) {
            const auto& p=half.keypoints[index];
            if(!std::isfinite(p.x)||!std::isfinite(p.y)||p.x<0||p.y<0||p.x>=frame.cols||p.y>=frame.rows||
               !std::isfinite(half.confidence[index])||half.confidence[index]<threshold) {
                result.reason="color_keypoints_invalid";return result;
            }
            polygon.emplace_back(cvRound(p.x),cvRound(p.y));
        }
        if(!cv::isContourConvex(polygon)){result.reason="color_polygon_invalid";return result;}
        cv::Mat mask=cv::Mat::zeros(frame.size(),CV_8U);cv::fillConvexPoly(mask,polygon,255);
        cv::erode(mask,mask,cv::getStructuringElement(cv::MORPH_RECT,{11,11}));
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
        if(total<config_.min_pixels || total<initial*.3){result.reason="color_occluded";return result;}
        const float r=float(red)/total,b=float(blue)/total;
        if(r>=config_.min_fraction&&b<=config_.max_other_fraction)colors[side]="red";
        else if(b>=config_.min_fraction&&r<=config_.max_other_fraction)colors[side]="blue";
        else {result.reason="color_ambiguous";return result;}
    }
    if(colors[0]!=colors[1]) {result.reason="color_halves_disagree";return result;}
    result.color=colors[0];result.verified=true;result.reason="ok";return result;
}
}
