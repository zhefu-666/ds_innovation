#include "rescue/keypoint_filter.hpp"
#include <cmath>
#include <set>
namespace rescue {
std::string KeypointFilter::frameRejection(const GeometryFrame& f,const CameraCalibration& c) const {
    if (!f.frame_id || !f.capture_us || f.now_us<f.capture_us || f.now_us-f.capture_us>config_.max_frame_age_us)
        return "stale_frame";
    if (!c.valid()) return "invalid_calibration";
    if (f.image_size.width<=0 || f.image_size.height<=0 || c.imageSize()!=f.image_size)
        return "calibration_image_size_mismatch";
    const auto& s=f.sensors;
    if (!s.imu.imu_valid || !std::isfinite(s.imu.yaw_rad) ||
        !causalFresh(s.imu.timestamp_us,f.capture_us,config_.max_skew_us)) return "imu_not_synchronized";
    if (!s.actuator.valid || !causalFresh(s.actuator.timestamp_us,f.capture_us,config_.max_skew_us))
        return "pitch_not_synchronized";
    if (!s.pitch_stable) return "pitch_moving";
    if (!c.pitchUsable(s.actuator.camera_pitch_cdeg)) return "pitch_not_calibrated";
    if (!c.tiltTrusted(s.imu)) return "body_tilt_untrusted";
    return {};
}
FilteredKeypoints KeypointFilter::filter(const KeypointFrame& in,const GeometryFrame& f,
                                         const ZoneGeometry& g,const CameraCalibration& c) const {
    FilteredKeypoints out;
    const auto fail=[&](const std::string& why){out.reason=why;out.points.clear();return out;};
    const auto reason=frameRejection(f,c);if (!reason.empty()) return fail(reason);
    if (!g.valid()) return fail("invalid_zone_geometry");
    if (in.frame_id!=f.frame_id || in.capture_us!=f.capture_us || in.image_size!=f.image_size)
        return fail("keypoint_frame_mismatch");
    if (in.points.empty()) return fail("no_keypoints");
    if (in.zone_label!=g.label || in.geometry_id!=g.id) return fail("keypoint_schema_mismatch");
    if (in.points.size()>6) return fail("too_many_keypoints");
    const auto object=g.objectPoints();std::set<int> seen;
    for(const auto& k:in.points) {
        if (k.id<0 || k.id>=6 || !seen.insert(k.id).second) return fail("invalid_or_duplicate_id");
        if (!k.visible || !std::isfinite(k.confidence) || k.confidence<config_.min_confidence || k.confidence>1 ||
            !std::isfinite(k.pixel.x) || !std::isfinite(k.pixel.y) || k.pixel.x<1 || k.pixel.y<1 ||
            k.pixel.x>=f.image_size.width-1 || k.pixel.y>=f.image_size.height-1) continue;
        cv::Point2f b,dx,dy;
        // Landmarks lie on the plane z=landmark_height_m (fence tops for v2), not the floor.
        const auto project=[&](cv::Point2f p,cv::Point2f& q){
            return c.pixelToPlane(p,f.sensors.actuator.camera_pitch_cdeg,g.landmark_height_m,q,&f.sensors.imu);};
        if (!project(k.pixel,b) || !project(k.pixel+cv::Point2f(1,0),dx) || !project(k.pixel+cv::Point2f(0,1),dy)) continue;
        const float sigma=config_.pixel_sigma/k.confidence*std::max(cv::norm(dx-b),cv::norm(dy-b));
        if (!std::isfinite(sigma) || sigma>config_.max_ground_sigma_m || cv::norm(b)>config_.max_range_m || b.y<=0) continue;
        out.points.push_back({k.id,k.pixel,{object[k.id].x,object[k.id].y},b,std::max(.001f,sigma),k.confidence});
    }
    // Keep only points that participate in a physically plausible baseline.
    // A bad point does not veto all good points; rigid-pose consensus follows.
    std::vector<GroundKeypoint> good;
    for(size_t i=0;i<out.points.size();++i) {
        bool supported=false;
        for(size_t j=0;j<out.points.size();++j) {
            if(i==j)continue;
            const float expected=cv::norm(out.points[i].zone_m-out.points[j].zone_m);
            const float measured=cv::norm(out.points[i].body_m-out.points[j].body_m);
            if (expected>=config_.min_baseline_m &&
                std::abs(expected-measured)<=config_.spacing_absolute_m+config_.spacing_relative*expected) supported=true;
        }
        if(supported)good.push_back(out.points[i]);
    }
    out.points=std::move(good);
    if(out.points.size()<2)return fail("insufficient_consistent_points");
    out.valid=true;out.reason="ok";return out;
}
} // namespace rescue
