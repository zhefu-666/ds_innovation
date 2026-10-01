#include "rescue/zone_estimate.hpp"
#include <cmath>
#include <set>
namespace rescue {
float wrapAngle(float a) {return std::atan2(std::sin(a),std::cos(a));}
bool ZoneEstimate::trusted(uint64_t now, const ZoneGate& g) const {
    if (!valid || source == Source::NONE || !frame_id || zone_label.empty() || geometry_id.empty() ||
        !timestamp_us || !observed_us || now < timestamp_us || timestamp_us < observed_us ||
        now-timestamp_us > g.max_age_us || int(inlier_ids.size()) < g.min_points ||
        std::set<int>(inlier_ids.begin(),inlier_ids.end()).size()!=inlier_ids.size()) return false;
    for(int id:inlier_ids)if(id<0||id>=6)return false;
    if ((source==Source::MULTI_POINT && inlier_ids.size()<3) ||
        (source==Source::TWO_POINT && inlier_ids.size()!=2)) return false;
    if (!std::isfinite(origin_body_m.x) || !std::isfinite(origin_body_m.y) || !std::isfinite(yaw_body_rad) ||
        !std::isfinite(residual_m) || residual_m<0 || residual_m>g.max_residual_m ||
        !std::isfinite(position_sigma_m) || position_sigma_m<0 || position_sigma_m>g.max_position_sigma_m ||
        !std::isfinite(yaw_sigma_rad) || yaw_sigma_rad<0 || yaw_sigma_rad>g.max_yaw_sigma_rad ||
        !std::isfinite(predicted_distance_m) || predicted_distance_m<0 ||
        predicted_distance_m>g.max_prediction_distance_m || (pnp_checked && !pnp_consistent)) return false;
    if (source==Source::PREDICTED) return now-observed_us<=g.max_prediction_age_us;
    return observed_us==timestamp_us && predicted_distance_m==0;
}
cv::Point2f ZoneEstimate::zoneToBody(const cv::Point2f& p) const {
    const float c=std::cos(yaw_body_rad),s=std::sin(yaw_body_rad);
    return {c*p.x-s*p.y+origin_body_m.x,s*p.x+c*p.y+origin_body_m.y};
}
cv::Point2f ZoneEstimate::bodyToZone(const cv::Point2f& p) const {
    const auto d=p-origin_body_m;
    const float c=std::cos(yaw_body_rad),s=std::sin(yaw_body_rad);
    return {c*d.x+s*d.y,-s*d.x+c*d.y};
}
} // namespace rescue
