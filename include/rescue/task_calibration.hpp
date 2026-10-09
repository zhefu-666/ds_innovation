#pragma once
#include "rescue/capture_monitor.hpp"
#include <cmath>
#include "rescue/multi_view_capture.hpp"
namespace rescue {
struct TaskCalibration {
    std::vector<FrameView> frame_views;
    int feedback_open=-1, feedback_close=-1;
    // a6_semantics="done_flag": byte 1 is a completion flag (TEMP_ASSUMPTION), open/close mapping unused.
    bool a6_done_flag=false; int a6_done_settle_ms=0;
    // frame_view_matching="image_polygon": TEMP_ASSUMPTION, multi-view uses image polygons without ground ranging.
    bool frame_view_image_polygon=false;
    std::string version;
    bool box_area_accepted=false;
    cv::Vec<float,6> body_envelope{}; // front,rear,left,right,open width,open front; zero disables
    TaskTuning task;
    CaptureConfig capture;
    float load_half_width_m = 0, load_half_depth_m = 0;
    float robot_swept_radius_m = 0; // body rotation envelope, independent of deposited objects
    float load_radius_m = 0; // enclosing radius of the maximum validated load footprint
};
// Necessary empty-half fit only; the live planner adds pose uncertainty/occupancy.
inline bool loadFitsHalf(float radius, float zone_width, float zone_depth, float divider, float half_width=0, float half_depth=0) {
    const float mx = (half_width>0?half_width:radius) + .01f;
    const float my = (half_depth>0?half_depth:radius) + .01f;
    return std::isfinite(radius) && radius > 0 && std::isfinite(zone_width) &&
        std::isfinite(zone_depth) && std::isfinite(divider) && divider >= 0 &&
        std::isfinite(mx) && std::isfinite(my) && 2*my < zone_depth && 2*mx + divider < zone_width/2;
}
// Explicit measured gripper geometry + pitch-specific image region. No defaults
// are silently marked calibrated; invalid/incomplete files fail before opening hardware.
TaskCalibration loadTaskCalibration(const std::string& path,TaskTuning tuning,int width,int height,bool search_only=false);
}
