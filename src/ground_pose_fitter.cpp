#include "rescue/ground_pose_fitter.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
namespace rescue {
namespace {
ZoneEstimate solve(const std::vector<GroundKeypoint>& p,const std::vector<int>& ids) {
    ZoneEstimate out;cv::Point2d za{},ba{};double total=0;
    for(int i:ids) {double w=1.0/(p[i].sigma_m*p[i].sigma_m);total+=w;za+=cv::Point2d(p[i].zone_m)*w;ba+=cv::Point2d(p[i].body_m)*w;}
    za*=1/total;ba*=1/total;double dot=0,cross=0;
    for(int i:ids) {double w=1.0/(p[i].sigma_m*p[i].sigma_m);auto a=cv::Point2d(p[i].zone_m)-za,b=cv::Point2d(p[i].body_m)-ba;
        dot+=w*a.dot(b);cross+=w*(a.x*b.y-a.y*b.x);}
    out.yaw_body_rad=std::atan2(cross,dot);
    const double c=std::cos(out.yaw_body_rad),s=std::sin(out.yaw_body_rad);
    out.origin_body_m={float(ba.x-c*za.x+s*za.y),float(ba.y-s*za.x-c*za.y)};
    return out;
}
}
void GroundPoseFitter::reset() {anchor_={};pending_={};pending_count_=0;last_us_=0;}
ZoneEstimate GroundPoseFitter::fit(const FilteredKeypoints& in,const KeypointFrame& k,const GeometryFrame& f) {
    ZoneEstimate out;out.zone_label=k.zone_label;out.geometry_id=k.geometry_id;
    out.frame_id=f.frame_id;out.timestamp_us=out.observed_us=f.capture_us;
    const auto fail=[&](const std::string& why,bool keep_pending=false){
        if(!keep_pending) pending_count_=0;
        out.valid=false;out.reason=why;return out;
    };
    if (!f.capture_us || (last_us_ && f.capture_us<=last_us_)) return fail("non_monotonic_frame");
    last_us_=f.capture_us;
    if (!in.valid || in.points.size()<2) {pending_count_=0;return fail(in.reason);}
    if (anchor_.zone_label!=k.zone_label || anchor_.geometry_id!=k.geometry_id) {anchor_={};pending_count_=0;}
    const auto& p=in.points;
    if(p.size()>6)return fail("invalid_filtered_points");
    std::set<int> seen;
    for(const auto& v:p)if(v.id<0||v.id>=6||!seen.insert(v.id).second||
        !std::isfinite(v.zone_m.x)||!std::isfinite(v.zone_m.y)||!std::isfinite(v.body_m.x)||!std::isfinite(v.body_m.y)||
        !std::isfinite(v.sigma_m)||v.sigma_m<=0)return fail("invalid_filtered_points");
    std::vector<int> best;
    double best_cost=1e30;bool ambiguous=false;
    ZoneEstimate best_pose;
    // Exhaustive minimal-pair RANSAC: at most six fixed IDs, so every hypothesis
    // is evaluated deterministically. Never fit a free-scale affine transform.
    for(size_t a=0;a<p.size();++a)for(size_t b=a+1;b<p.size();++b) {
        if(cv::norm(p[a].zone_m-p[b].zone_m)<config_.min_baseline_m)continue;
        auto candidate=solve(p,{int(a),int(b)});std::vector<int> ids;double cost=0;
        for(size_t i=0;i<p.size();++i) {
            const double r=cv::norm(candidate.zoneToBody(p[i].zone_m)-p[i].body_m);
            const double limit=std::min(config_.max_inlier_m,std::max(config_.inlier_m,2.5f*p[i].sigma_m));
            if(r<=limit){ids.push_back(int(i));cost+=r*r;}
        }
        const bool distinct=cv::norm(candidate.origin_body_m-best_pose.origin_body_m)>.08 ||
                            std::abs(wrapAngle(candidate.yaw_body_rad-best_pose.yaw_body_rad))>.15;
        if(ids.size()>best.size() || (ids.size()==best.size() && cost<best_cost-1e-8)) {
            ambiguous=ids.size()==best.size() && distinct && std::abs(cost-best_cost)<1e-5;
            best=ids;best_cost=cost;best_pose=candidate;
        } else if(ids.size()==best.size() && distinct && std::abs(cost-best_cost)<1e-5) ambiguous=true;
    }
    if(best.size()<2 || (p.size()>=3 && (best.size()<3 || best.size()*5<p.size()*3))) return fail("no_ransac_consensus");
    if(ambiguous)return fail("ambiguous_consensus");
    auto pose=solve(p,best);out.origin_body_m=pose.origin_body_m;out.yaw_body_rad=pose.yaw_body_rad;
    double sum=0;float sigma=0,baseline=0;
    for(int i:best) {
        const float residual=cv::norm(out.zoneToBody(p[i].zone_m)-p[i].body_m);
        if(residual>config_.max_inlier_m)return fail("unstable_refinement");
        sum+=residual*residual;sigma=std::max(sigma,p[i].sigma_m);out.inlier_ids.push_back(p[i].id);
        for(int j:best)baseline=std::max(baseline,float(cv::norm(p[i].zone_m-p[j].zone_m)));
    }
    out.residual_m=std::sqrt(sum/best.size());
    out.position_sigma_m=sigma+out.residual_m;
    out.yaw_sigma_rad=std::atan2(2*out.position_sigma_m,baseline);
    out.source=best.size()==2?ZoneEstimate::Source::TWO_POINT:ZoneEstimate::Source::MULTI_POINT;
    out.valid=true;out.reason="ok";
    if(!out.trusted(f.now_us,config_.gate))return fail("pose_quality_rejected");
    if(best.size()==2) {
        if(!anchor_.valid || f.capture_us<anchor_.timestamp_us || f.capture_us-anchor_.timestamp_us>config_.prior_age_us)
            return fail("two_points_need_recent_anchor");
        // Zone yaw in body coordinates changes opposite to IMU body yaw.
        const float expected=wrapAngle(anchor_.yaw_body_rad-wrapAngle(f.sensors.imu.yaw_rad-anchor_imu_yaw_));
        if(std::abs(wrapAngle(out.yaw_body_rad-expected))>config_.max_two_point_turn_rad ||
           cv::norm(out.origin_body_m-anchor_.origin_body_m)>config_.max_two_point_jump_m)
            return fail("two_point_prior_disagreement");
        // Two-point frames never renew the reliable multi-point anchor lease.
        pending_count_=0;return out;
    }
    if(anchor_.valid && (cv::norm(out.origin_body_m-anchor_.origin_body_m)>config_.reacquire_jump_m ||
        std::abs(wrapAngle(out.yaw_body_rad-anchor_.yaw_body_rad))>config_.reacquire_turn_rad)) {
        const bool consistent=pending_count_ && f.capture_us>pending_.timestamp_us &&
            f.capture_us-pending_.timestamp_us<=config_.prior_age_us &&
            cv::norm(out.origin_body_m-pending_.origin_body_m)<.04 &&
            std::abs(wrapAngle(out.yaw_body_rad-pending_.yaw_body_rad))<.08;
        pending_count_=consistent?pending_count_+1:1;pending_=out;
        if(pending_count_<config_.reacquire_frames)return fail("reacquiring_pose",true);
    }
    anchor_=out;anchor_imu_yaw_=f.sensors.imu.yaw_rad;pending_count_=0;return out;
}
} // namespace rescue
