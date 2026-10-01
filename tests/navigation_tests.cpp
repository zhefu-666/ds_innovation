#include "rescue/navigation_adapter.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
using namespace rescue;
int main(){
    NavigationScene s;s.timestamp_us=1000000;s.complete=s.opponent_region_known=true;s.swept_radius_m=.1;
    s.known_region={{-2,-2},{2,-2},{2,2},{-2,2}};
    LocalPlanner planner;
    assert(planner.planCarry({0,1},s,1000000).valid);
    s.obstacles.push_back({{0,.5},.15,false});
    auto route=planner.planCarry({0,1},s,1000000);
    assert(route.valid && route.waypoints_m.size()>2);
    for(size_t i=1;i<route.waypoints_m.size();++i)
        assert(planner.sweptSegmentSafe(route.waypoints_m[i-1],route.waypoints_m[i],s,1000000));
    assert(!planner.sweptSegmentSafe({0,0},{0,1},s,1000000));
    assert(!planner.planCarry({0,1},s,1200001).valid);
    s.complete=false;assert(!planner.planCarry({0,1},s,1000000).valid);s.complete=true;
    s.opponent_region_known=false;assert(!planner.planCarry({0,1},s,1000000).valid);s.opponent_region_known=true;
    assert(!planner.planCarry({3,1},s,1000000).valid);
    s.obstacles.push_back({{0,0},.1,true});assert(!planner.planCarry({0,1},s,1000000).valid);
    s.obstacles.clear();
    s.swept_radius_m=0;assert(!planner.planCarry({0,1},s,1000000).valid);s.swept_radius_m=.1;
    ZoneGeometry g;g.label="red_safe_zone";g.id="test";g.width_m=.6;g.depth_m=.3;
    g.landmark_width_m=.66;g.landmark_depth_m=.33;g.landmark_height_m=.02;g.confirmed=true;
    ZoneEstimate z;z.valid=true;z.source=ZoneEstimate::Source::MULTI_POINT;z.zone_label=g.label;z.geometry_id=g.id;
    z.frame_id=1;z.timestamp_us=z.observed_us=1000000;z.inlier_ids={0,2,3,5};
    z.residual_m=.001;z.position_sigma_m=.001;z.yaw_sigma_rad=.001;z.origin_body_m={0,1};
    assert(g.valid()&&z.trusted(1000000));
    DropPlanner drops;
    auto first=drops.plan(g,z,true,true,{},false,.03,1000000);
    assert(first.valid && first.centre_zone_m.x<0);
    auto second=drops.plan(g,z,true,true,{{first.centre_zone_m,.03,false}},false,.03,1000000);
    assert(second.valid && cv::norm(second.centre_zone_m-first.centre_zone_m)>.06);
    assert(!drops.plan(g,z,false,true,{},false,.03,1000000).valid);
    assert(!drops.plan(g,z,true,false,{},false,.03,1000000).valid);
    assert(!drops.plan(g,z,true,true,{},false,.2,1000000).valid);
    auto injured=drops.plan(g,z,true,true,{},true,.03,1000000);
    assert(injured.valid && injured.centre_zone_m.x>0);
    CarryNavigator nav(g,TaskTuning{},.03);PushObservation in;PushOutput previous;
    previous.state=PushState::CARRY;in.now_us=1000000;in.zone_estimate=z;
    in.zone_own=in.zone_identity_verified=in.zone_counts_valid=in.zone_inventory_complete=true;
    in.navigation_scene=s;nav.update(in,previous);
    assert(in.carry_plan_valid&&in.drop_plan_valid&&in.drop_centre_zone.x<0);
    previous.drop_locked=true;previous.drop_centre_zone=in.drop_centre_zone;
    in.zone_occupied={{in.drop_centre_zone,.03,false}};nav.update(in,previous);
    assert(!in.carry_plan_valid&&!in.drop_plan_valid); // don't relocate a locked drop silently
    std::cout<<"Carry detours, unknown-space rejection and multi-trip drop occupancy passed\n";
}
