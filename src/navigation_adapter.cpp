#include "rescue/navigation_adapter.hpp"
#include <cmath>
namespace rescue {
void CarryNavigator::update(PushObservation& in,const PushOutput& previous) {
    in.carry_plan_valid=in.drop_plan_valid=false;in.navigation_timestamp_us=0;
    const bool active=previous.state==PushState::CARRY || previous.state==PushState::GATE ||
        previous.state==PushState::RAISE_RELEASE || previous.state==PushState::ENTER;
    if(previous.state==PushState::SCAN || previous.state==PushState::WAIT_START)
        release_background_us_=0;
    if(!active || !in.zone_own || !in.zone_identity_verified || !in.zone_estimate.trusted(in.now_us))return;
    if(controlled_ && previous.state==PushState::GATE && in.zone_counts_valid && in.zone_inventory_complete) {
        release_background_=in.zone_occupied;release_background_us_=in.zone_estimate.timestamp_us;
        release_geometry_id_=in.zone_estimate.geometry_id;
    }
    const bool releasing=controlled_ && (previous.state==PushState::RAISE_RELEASE || previous.state==PushState::ENTER);
    if(releasing && (!release_background_us_ || in.now_us<release_background_us_ ||
       in.now_us-release_background_us_>20000000 || release_geometry_id_!=in.zone_estimate.geometry_id))return;
    if(!releasing && (!in.zone_counts_valid || !in.zone_inventory_complete))return;
    const auto& occupied=releasing?release_background_:in.zone_occupied;
    // The release cache is a planning assumption of the explicitly static test
    // environment, never a source of current counts or proof of delivery.
    auto plan=drops_.plan(geometry_,in.zone_estimate,in.zone_identity_verified,true,
                          occupied,previous.cargo_injured,radius_,in.now_us);
    if(previous.drop_locked) {
        // Revalidate the locked spot instead of silently changing the destination halfway.
        const auto q=previous.drop_centre_zone;
        const auto& z=in.zone_estimate;
        const bool clear=geometry_.valid() && z.geometry_id==geometry_.id && z.zone_label==geometry_.label &&
            drops_.positionClear(geometry_,z,q,occupied,previous.cargo_injured,radius_);
        plan.valid=clear;plan.centre_zone_m=q;
    }
    if(!plan.valid)return;
    if(previous.state!=PushState::CARRY) {
        in.drop_plan_valid=true;in.drop_centre_zone=plan.centre_zone_m;
        in.navigation_timestamp_us=in.zone_estimate.timestamp_us;return;
    }
    const float centre_y=previous.cargo_injured?tuning_.injured_hold_center_y_m:tuning_.hold_center_y_m;
    const auto gate=in.zone_estimate.zoneToBody({plan.centre_zone_m.x,-tuning_.gate_clearance_m-centre_y});
    if (controlled_) {
        // Local visual servoing in the explicitly cleared test area. Do not invent
        // a NavigationScene or claim global localization/free-space perception.
        if (!in.path_safe || !in.opponent_zone_clear || !std::isfinite(gate.x) ||
            !std::isfinite(gate.y) || cv::norm(gate)>3.f) return;
        in.drop_plan_valid=in.carry_plan_valid=true;
        in.navigation_timestamp_us=in.zone_estimate.timestamp_us;
        in.drop_centre_zone=plan.centre_zone_m;in.carry_waypoint_body=gate;return;
    }
    const auto route=routes_.planCarry(gate,in.navigation_scene,in.now_us);
    if(!route.valid || route.waypoints_m.size()<2)return;
    in.drop_plan_valid=in.carry_plan_valid=true;
    in.navigation_timestamp_us=in.navigation_scene.timestamp_us;
    in.drop_centre_zone=plan.centre_zone_m;
    in.carry_waypoint_body=route.waypoints_m[1];
}
}
