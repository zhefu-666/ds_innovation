#include "rescue/task_calibration.hpp"
#include <opencv2/core.hpp>
#include <cmath>
#include <stdexcept>
namespace rescue {
TaskCalibration loadTaskCalibration(const std::string& path,TaskTuning tuning,int width,int height) {
    cv::FileStorage f(path,cv::FileStorage::READ);
    if(!f.isOpened() || int(f["schema_version"])!=1 || int(f["measured"])!=1)
        throw std::runtime_error("Task calibration must be schema_version=1 and measured=1");
    const auto number=[&](const char* key){
        const auto n=f[key];
        if(n.empty() || (!n.isInt()&&!n.isReal()))throw std::runtime_error(std::string("Missing task calibration: ")+key);
        const float v=float(n);
        if(!std::isfinite(v) || v<=0 || v>2)throw std::runtime_error(std::string("Invalid measured length: ")+key);
        return v;
    };
    if(int(f["image_width"])!=width || int(f["image_height"])!=height || width<=0 || height<=0)
        throw std::runtime_error("Task calibration image size mismatch");
    TaskCalibration result;result.task=tuning;result.capture.holding.clear();result.capture.image_height_px=height;
    result.task.hold_center_y_m=number("hold_center_y_m");result.task.mouth_y_m=number("mouth_y_m");
    if(!f["grasp_trigger_y_m"].empty())result.task.grasp_trigger_y_m=number("grasp_trigger_y_m");
    if(!f["injured_hold_center_y_m"].empty())result.task.injured_hold_center_y_m=number("injured_hold_center_y_m");
    else result.task.injured_hold_center_y_m=result.task.hold_center_y_m;
    if(result.task.mouth_y_m<=result.task.hold_center_y_m ||
       result.task.mouth_y_m<=result.task.injured_hold_center_y_m ||
       result.task.grasp_trigger_y_m>result.task.mouth_y_m)
        throw std::runtime_error("Gripper geometry order is invalid");
    result.capture.corridor_half_width_m=number("corridor_half_width_m");
    result.load_radius_m=number("load_radius_m");
    const auto views=f["holding_views"];
    if(!views.isSeq() || views.empty())throw std::runtime_error("Missing measured holding_views");
    bool near=false;
    for(const auto& v:views) {
        if(!v["pitch_cdeg"].isInt() || !v["area"].isSeq() || v["area"].size()!=4)
            throw std::runtime_error("Invalid holding view");
        const int pitch=int(v["pitch_cdeg"]);
        if(pitch < -4000 || pitch>4000)throw std::runtime_error("Holding pitch outside hardware range");
        RectArea r{float(v["area"][0]),float(v["area"][1]),float(v["area"][2]),float(v["area"][3])};
        if(!std::isfinite(r.x1)||!std::isfinite(r.x2)||!std::isfinite(r.y1)||!std::isfinite(r.y2)||
           r.x1<0||r.y1<0||r.x2>width||r.y2>height||r.x2<=r.x1||r.y2<=r.y1)
            throw std::runtime_error("Holding area outside calibrated image");
        for(const auto& existing:result.capture.holding)
            if(std::abs(pitch-int(existing.pitch_cdeg))<=2*result.capture.pitch_tolerance_cdeg)
                throw std::runtime_error("Ambiguous overlapping holding pitch ranges");
        float min_bottom=0;
        if(!v["min_visible_bottom_y_px"].empty()) {
            if(!v["min_visible_bottom_y_px"].isInt()&&!v["min_visible_bottom_y_px"].isReal())
                throw std::runtime_error("Invalid holding depth threshold");
            min_bottom=float(v["min_visible_bottom_y_px"]);
            if(!std::isfinite(min_bottom)||min_bottom<r.y1||min_bottom>r.y2)
                throw std::runtime_error("Holding depth threshold outside region");
        }
        result.capture.holding.push_back({int16_t(pitch),r,min_bottom});
        if(pitch==tuning.near_pitch_cdeg)near=true;
    }
    if(!near)throw std::runtime_error("No holding region measured at the selected NEAR pitch");
    return result;
}
}
