#include <algorithm>
#include "rescue/task_calibration.hpp"
#include <opencv2/core.hpp>
#include <cmath>
#include <stdexcept>
namespace rescue {
TaskCalibration loadTaskCalibration(const std::string& path,TaskTuning tuning,int width,int height,bool search_only) {
    cv::FileStorage f(path,cv::FileStorage::READ);
    if(f.isOpened() && int(f["schema_version"])==2) {
        const auto gate=[&](const char* key){if(!f[key].isInt() || int(f[key])!=1)throw std::runtime_error(std::string("Mechanical calibration pending: ")+key);};
        for(const char* key:{"coordinate_transform_verified","action_mapping_verified","body_envelope_verified"})gate(key);
        if(!search_only){gate("capture_stop_verified");gate("visual_acceptance_passed");}
        TaskCalibration r;r.task=tuning;r.task.require_multi_view=true;
        f["version"]>>r.version;if(r.version.empty())throw std::runtime_error("Missing mechanical configuration version");
        if(int(f["image_width"])!=width||int(f["image_height"])!=height||width<=0||height<=0)throw std::runtime_error("Mechanical image size mismatch");
        const auto n=[&](const char* key){auto node=f[key];if(!node.isInt()&&!node.isReal())throw std::runtime_error(std::string("Missing mechanical number: ")+key);float v=float(node);if(!std::isfinite(v))throw std::runtime_error("Nonfinite geometry");return v;};
        std::string semantics;if(!f["a6_semantics"].empty())f["a6_semantics"]>>semantics;
        if(semantics=="done_flag"){
            // TEMP_ASSUMPTION: byte 1 is a completion flag for either target; open/close comes from sent targets.
            r.a6_done_flag=true;r.a6_done_settle_ms=int(n("a6_done_settle_ms"));
            if(!f["a6_done_settle_ms"].isInt()||r.a6_done_settle_ms<100||r.a6_done_settle_ms>3000)throw std::runtime_error("a6_done_settle_ms must be integer 100..3000");
        }else if(!semantics.empty()&&semantics!="state_mapping")throw std::runtime_error("Unknown a6_semantics");
        else{
        if(!f["a6_open_state"].isInt()||!f["a6_close_state"].isInt())throw std::runtime_error("A6 states must be integers");
        r.feedback_open=int(n("a6_open_state"));r.feedback_close=int(n("a6_close_state"));
        if(r.feedback_open<0||r.feedback_open>1||r.feedback_close<0||r.feedback_close>1||r.feedback_open==r.feedback_close)throw std::runtime_error("Invalid A6 mapping");
        }
        if(n("forward_sign")!=1)throw std::runtime_error("Only confirmed positive body-forward transform is supported");
        const float origin=n("rotation_origin_y_m"), lateral=n("rotation_origin_x_m");
        const float near=n("near_inner_m")+origin, far=n("far_inner_m")+origin;
        const float inner_width=n("inner_width_m"),inner_depth=n("inner_depth_m");
        if(inner_width<=0||inner_depth<=0||near<=0||far<=near||std::abs(far-near-inner_depth)>.0001f||std::abs(lateral)>.0001f)throw std::runtime_error("Invalid or unsupported frame transform");
        r.task.mouth_y_m=far;r.task.hold_center_y_m=(near+far)/2;r.task.injured_hold_center_y_m=r.task.hold_center_y_m;
        float margin=0; // no forward capture corridor is authorized in search-only mode
        if(!search_only){
        r.task.grasp_trigger_y_m=n("grasp_trigger_y_m");
        const float block_depth=n("maximum_block_depth_m");margin=n("stopping_margin_m");
        const float block_width=n("maximum_block_width_m");
        if(block_width<=0||block_width+2*margin>=inner_width||block_width+2*margin>=n("entrance_width_m"))throw std::runtime_error("Block width does not fit entrance");
        if(block_depth<=0||margin<=0||r.task.grasp_trigger_y_m-block_depth/2-margin<near||r.task.grasp_trigger_y_m+block_depth/2+margin>far)throw std::runtime_error("Capture stopping footprint does not fit frame");
        }
        r.load_half_width_m=inner_width/2;r.load_half_depth_m=inner_depth/2;r.load_radius_m=std::hypot(r.load_half_width_m,r.load_half_depth_m);
        const char* keys[]={"body_front_m","body_rear_m","body_left_m","body_right_m","open_width_m","open_front_m"};
        for(int i=0;i<6;++i){r.body_envelope[i]=n(keys[i]);if(r.body_envelope[i]<=0||r.body_envelope[i]>2)throw std::runtime_error("Invalid body envelope");}
        r.robot_swept_radius_m=n("robot_swept_radius_m");
        if(r.robot_swept_radius_m<std::hypot(std::max({r.body_envelope[0],r.body_envelope[1],r.body_envelope[5]}),std::max({r.body_envelope[2],r.body_envelope[3],r.body_envelope[4]/2})))throw std::runtime_error("Insufficient swept radius");
        r.task.closed_front_y_m=r.body_envelope[0];r.capture.mouth_y_m=far;r.capture.corridor_half_width_m=r.body_envelope[4]/2+margin;
        r.capture.image_height_px=height;
        if(search_only){r.box_area_accepted=int(f["box_area_accepted"])==1;return r;}
        std::string matching;if(!f["frame_view_matching"].empty())f["frame_view_matching"]>>matching;
        if(matching=="image_polygon")r.frame_view_image_polygon=true;
        else if(!matching.empty()&&matching!="body_ground")throw std::runtime_error("Unknown frame_view_matching");
        const auto views=f["frame_views"];
        if(!views.isSeq()||views.size()<2||views.size()>3)throw std::runtime_error("Need two or three independent frame views");
        for(const auto& v:views){FrameView view;const int pitch=int(v["pitch_cdeg"]);
            if(!v["pitch_cdeg"].isInt()||pitch < -4000||pitch>4000||int(v["calibrated"])!=1||int(v["fully_observable"])!=1)throw std::runtime_error("Uncalibrated frame view");
            view.pitch_cdeg=pitch;view.calibrated=view.fully_observable=true;
            for(const auto& prev:r.frame_views)if(std::abs(pitch-int(prev.pitch_cdeg))<=200)throw std::runtime_error("Overlapping view pitches");
            const auto poly=v["polygon"];if(!poly.isSeq()||poly.size()<3)throw std::runtime_error("Missing frame polygon");
            for(const auto& point:poly){if(!point.isSeq()||point.size()!=2)throw std::runtime_error("Bad polygon point");float x=float(point[0]),y=float(point[1]);
                if(!std::isfinite(x)||!std::isfinite(y)||x<0||x>=width||y<0||y>=height)throw std::runtime_error("Polygon outside image");
                view.polygon.emplace_back(x,y);}
            if(!cv::isContourConvex(view.polygon)||std::abs(cv::contourArea(view.polygon))<1)throw std::runtime_error("Frame polygon must be convex");
            r.frame_views.push_back(view);
            auto b=cv::boundingRect(view.polygon);r.capture.holding.push_back({view.pitch_cdeg,{float(b.x),float(b.y),float(b.br().x),float(b.br().y)},0,view.polygon});
        }
        r.task.near_pitch_cdeg=r.frame_views.front().pitch_cdeg;
        r.task.enable_search_cues=false;r.task.enable_short_push=false;r.task.startup_advance_us=0;
        r.task.attempt_budget_us=60000000;r.task.approach_limit_m=.30f;r.task.retreat_limit_m=.20f;
        r.task.max_speed=.10f;r.task.approach_speed=.10f;r.task.rush_speed=.05f;
        r.box_area_accepted=int(f["box_area_accepted"])==1;
        return r;
    }
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
    result.capture.mouth_y_m=result.task.mouth_y_m; // objects clearly beyond it are not held
    if(!f["closed_front_y_m"].empty())result.task.closed_front_y_m=number("closed_front_y_m");
    result.load_radius_m=number("load_radius_m");
    if(!f["robot_swept_radius_m"].empty()) result.robot_swept_radius_m=number("robot_swept_radius_m");
    if(!f["load_half_width_m"].empty() || !f["load_half_depth_m"].empty()) {
        result.load_half_width_m=number("load_half_width_m");
        result.load_half_depth_m=number("load_half_depth_m");
        if(std::hypot(result.load_half_width_m,result.load_half_depth_m)>result.load_radius_m+.0001f)
            throw std::runtime_error("Load radius must enclose the rectangular footprint");
    }
    if(!f["body_front_m"].empty()) {
        const char* keys[]={"body_front_m","body_rear_m","body_left_m","body_right_m","open_width_m","open_front_m"};
        for(int i=0;i<6;++i)result.body_envelope[i]=number(keys[i]);
        const float front=std::max(result.body_envelope[0],result.body_envelope[5]);
        const float side=std::max({result.body_envelope[2],result.body_envelope[3],result.body_envelope[4]/2});
        if(std::hypot(std::max(front,result.body_envelope[1]),side)>result.robot_swept_radius_m)
            throw std::runtime_error("Body/open jaw exceeds rotation envelope");
    }
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
        result.capture.holding.push_back({int16_t(pitch),r,min_bottom,{}});
        if(pitch==tuning.near_pitch_cdeg)near=true;
    }
    if(!near)throw std::runtime_error("No holding region measured at the selected NEAR pitch");
    return result;
}
}
