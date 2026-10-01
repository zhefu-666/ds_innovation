#include "rescue/zone_layout.hpp"
#include <cmath>
#include <algorithm>

namespace rescue {

SubzoneDecision classifyExpectedStop(const ZoneGeometry& g,const ZoneEstimate& z,
        const cv::Point2f& stop,float radius,uint64_t now) {
    SubzoneDecision out;
    if(!g.valid()||z.zone_label!=g.label||z.geometry_id!=g.id||!z.trusted(now)) {out.reason="zone_untrusted";return out;}
    if(!std::isfinite(stop.x)||!std::isfinite(stop.y)||!std::isfinite(radius)||radius<0)return out;
    out.stop_zone_m=z.bodyToZone(stop);
    const float margin=radius+2*z.position_sigma_m+cv::norm(out.stop_zone_m)*2*z.yaw_sigma_rad;
    if(out.stop_zone_m.y<=margin||out.stop_zone_m.y>=g.depth_m-margin||
       std::abs(out.stop_zone_m.x)>=g.width_m/2-margin){out.reason="outside_inner_boundary";return out;}
    if(std::abs(out.stop_zone_m.x)<=g.divider_exclusion_half_width_m+margin){out.reason="divider_clearance";return out;}
    out.side=out.stop_zone_m.x<0?ZoneLayout::LEFT:ZoneLayout::RIGHT;
    out.zone_class=((out.side==ZoneLayout::LEFT)==g.supply_left)?"supply":"injured";
    out.valid=true;out.reason="ok";return out;
}

DeliveryClass deliveryClassForLabel(const std::string &label) {
    if (label == "dangerous_object") return DeliveryClass::REJECT;
    if (label == "injured_person") return DeliveryClass::INJURED;
    if (label == "ordinary_supply" || label == "core_supply") return DeliveryClass::SUPPLY;
    return DeliveryClass::REJECT;
}

bool isCorrectSubzone(const ZoneLayout &layout, const std::string &target_label,
                      ZoneLayout::Side observed_side) {
    if (!layout.complete()) return false;
    switch (deliveryClassForLabel(target_label)) {
    case DeliveryClass::SUPPLY: return observed_side == layout.supply_side;
    case DeliveryClass::INJURED: return observed_side == layout.injured_side;
    case DeliveryClass::REJECT: return false;
    }
    return false;
}

} // namespace rescue
