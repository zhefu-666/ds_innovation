#include "rescue/navigation_adapter.hpp"
#include <cmath>
namespace rescue {
void CarryNavigator::update(PushObservation& in,const PushOutput& previous) const {
    in.carry_plan_valid=in.drop_plan_valid=false;in.navigation_timestamp_us=0;
    const bool active=previous.state==PushState::CARRY || previous.state==PushState::GATE ||
        previous.state==PushState::OPEN_RELEASE || previous.state==PushState::ENTER;
    if(!active || !in.zone_own || !in.zone_identity_verified || !in.zone_counts_valid ||
       !in.zone_inventory_complete || !in.zone_estimate.trusted(in.now_us))return;
    auto plan=drops_.plan(geometry_,in.zone_estimate,in.zone_identity_verified,in.zone_inventory_complete,
                          in.zone_occupied,previous.cargo_injured,radius_,in.now_us);
    if(previous.drop_locked) {
        // Revalidate the locked spot instead of silently changing the destination halfway.
        const auto q=previous.drop_centre_zone;
        const auto& z=in.zone_estimate;
        const float margin=radius_+.01f+2*z.position_sigma_m+2*cv::norm(q)*z.yaw_sigma_rad;
        const bool left=previous.cargo_injured?!geometry_.supply_left:geometry_.supply_left;
        bool clear=geometry_.valid() && z.geometry_id==geometry_.id && z.zone_label==geometry_.label &&
            (left?q.x<0:q.x>0) && std::isfinite(radius_) && radius_>0 && std::isfinite(q.x)&&std::isfinite(q.y)&&std::abs(q.x)>geometry_.divider_exclusion_half_width_m+margin &&
            std::abs(q.x)<geometry_.width_m/2-margin && q.y>margin && q.y<geometry_.depth_m-margin;
        for(const auto& o:in.zone_occupied)if(!std::isfinite(o.radius_m)||o.radius_m<0||!std::isfinite(o.center_m.x)||
            !std::isfinite(o.center_m.y)||cv::norm(q-o.center_m)<=margin+o.radius_m)clear=false;
        plan.valid=clear;plan.centre_zone_m=q;
    }
    if(!plan.valid)return;
    if(previous.state!=PushState::CARRY) {
        in.drop_plan_valid=true;in.drop_centre_zone=plan.centre_zone_m;
        in.navigation_timestamp_us=in.zone_estimate.timestamp_us;return;
    }
    const auto gate=in.zone_estimate.zoneToBody({plan.centre_zone_m.x,-tuning_.gate_clearance_m-tuning_.hold_center_y_m});
    const auto route=routes_.planCarry(gate,in.navigation_scene,in.now_us);
    if(!route.valid || route.waypoints_m.size()<2)return;
    in.drop_plan_valid=in.carry_plan_valid=true;
    in.navigation_timestamp_us=in.navigation_scene.timestamp_us;
    in.drop_centre_zone=plan.centre_zone_m;
    in.carry_waypoint_body=route.waypoints_m[1];
}
}
