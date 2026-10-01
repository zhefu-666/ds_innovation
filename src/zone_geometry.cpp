#include "rescue/zone_geometry.hpp"
#include <opencv2/core/persistence.hpp>
#include <cmath>
#include <stdexcept>
namespace rescue {
bool ZoneGeometry::valid() const {
    return confirmed && !id.empty() && (label=="red_safe_zone" || label=="blue_safe_zone") &&
        std::isfinite(width_m) && width_m>.1f && width_m<5 && std::isfinite(depth_m) && depth_m>.1f && depth_m<5 &&
        std::isfinite(divider_exclusion_half_width_m) && divider_exclusion_half_width_m>0 &&
        divider_exclusion_half_width_m<width_m/4 &&
        std::isfinite(landmark_width_m) && landmark_width_m>.1f && landmark_width_m<5 &&
        std::isfinite(landmark_depth_m) && landmark_depth_m>.1f && landmark_depth_m<5 &&
        std::isfinite(landmark_height_m) && landmark_height_m>=0 && landmark_height_m<.2f;
}
std::array<cv::Point3f,6> ZoneGeometry::objectPoints() const {
    const float w=landmark_width_m,d=landmark_depth_m;
    return {{{-w/2,0,0},{0,0,0},{w/2,0,0},{-w/2,d,0},{0,d,0},{w/2,d,0}}};
}
ZoneGeometry ZoneGeometry::load(const std::string& file, const std::string& label) {
    cv::FileStorage f(file,cv::FileStorage::READ);
    if (!f.isOpened() || int(f["schema_version"])!=1 || !f["zones"].isSeq())
        throw std::runtime_error("Invalid zone geometry schema");
    ZoneGeometry out; int matches=0;
    for (const auto& n:f["zones"]) {
        if (std::string(n["label"])!=label) continue;
        ++matches;out.label=label;out.id=std::string(n["geometry_id"]);
        out.width_m=float(n["inner_width_m"]);out.depth_m=float(n["inner_depth_m"]);
        out.divider_exclusion_half_width_m=float(n["divider_exclusion_half_width_m"]);
        // Optional landmark rectangle; absent means legacy inner-floor landmarks.
        const bool any=!n["landmark_width_m"].empty()||!n["landmark_depth_m"].empty()||!n["landmark_height_m"].empty();
        if(any&&(n["landmark_width_m"].empty()||n["landmark_depth_m"].empty()||n["landmark_height_m"].empty()))
            throw std::runtime_error("Zone landmark width, depth and height must be given together");
        out.landmark_width_m=any?float(n["landmark_width_m"]):out.width_m;
        out.landmark_depth_m=any?float(n["landmark_depth_m"]):out.depth_m;
        out.landmark_height_m=any?float(n["landmark_height_m"]):0.f;
        const auto side=std::string(n["supply_side"]);
        if (side!="left" && side!="right") throw std::runtime_error("Unknown supply side");
        out.supply_left=side=="left";out.confirmed=int(n["confirmed"])==1;
    }
    if (matches!=1 || !out.valid()) throw std::runtime_error("Missing, duplicate or unconfirmed zone geometry");
    return out;
}
} // namespace rescue
