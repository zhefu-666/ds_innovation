#include "rescue/controlled_field.hpp"
#include <cassert>
#include <iostream>
using namespace rescue;
int main(){
    ZoneGeometry g;g.label="blue_safe_zone";g.id="test";g.confirmed=true;
    g.width_m=.6;g.depth_m=.3;g.landmark_width_m=.66;g.landmark_depth_m=.33;g.landmark_height_m=.02;
    ZoneHalves halves;
    for(int h=0;h<2;++h){
        halves[h].valid=true;float x=50+300*h;
        halves[h].keypoints={cv::Point2f{x,50},cv::Point2f{x,300},cv::Point2f{x+250,50},cv::Point2f{x+250,300}};
        halves[h].confidence={1,1,1,1};
    }
    PushOutput previous;ControlledField field(g,.32f);PushObservation in;
    const auto tick=[&](std::vector<SegDetection> objects={},bool mapping=true){
        in.now_us+=100000;in.zone_own=in.zone_identity_verified=true;
        auto& z=in.zone_estimate;z.valid=true;z.source=ZoneEstimate::Source::MULTI_POINT;
        z.zone_label=g.label;z.geometry_id=g.id;z.timestamp_us=z.observed_us=in.now_us;z.frame_id=in.now_us/100000;
        z.inlier_ids={0,2,3,5};z.residual_m=.001;z.position_sigma_m=.001;z.yaw_sigma_rad=.001;z.origin_body_m={0,.5};
        for(auto& d:objects){d.frame_id=z.frame_id;d.timestamp_us=in.now_us;}
        field.update(in,objects,halves,{1280,720},previous,mapping);
    };
    tick();tick();tick();assert(in.zone_counts_valid&&in.zone_inventory_complete&&in.zone_supply_count==0);
    SegDetection d;d.label="ordinary_supply";d.confidence=.9;d.ground_position_valid=d.ground_contact_valid=true;
    d.body_xy_m={-.16,.65};d.box={110,110,30,30};
    tick({d});assert(!in.zone_counts_valid);tick({d});tick({d});
    assert(in.zone_counts_valid&&in.zone_supply_count==1&&in.zone_occupied.size()==1);
    d.body_xy_m={.16,.65};tick({d});assert(!in.zone_counts_valid&&field.reason()=="wrong_half_or_divider");
    d.body_xy_m={-.16,.65};d.ground_contact_valid=false;tick({d});assert(!in.zone_counts_valid);
    halves[1].valid=false;tick();assert(!in.zone_counts_valid);halves[1].valid=true;
    tick({},false);assert(!in.path_safe&&!in.retreat_safe&&!in.zone_counts_valid);
    d.label="dangerous_object";d.body_xy_m={0,.3};tick({d});assert(!in.path_safe&&!in.retreat_safe);
    // No extra 15 cm margin, but the actual body radius is still protected.
    d.body_xy_m={0,.46f};tick({d});assert(in.path_safe&&in.retreat_safe);
    d.body_xy_m={0,.32f};tick({d});assert(!in.path_safe&&!in.retreat_safe);
    d.body_xy_m={0,.321f};tick({d});assert(in.path_safe&&in.retreat_safe);
    d.ground_position_valid=false;tick({d});assert(!in.path_safe&&!in.retreat_safe);
    d.ground_position_valid=true;d.body_xy_m={0,.3f};
    // Even a legacy cue-contact flag cannot bypass body collision protection.
    d.ground_contact_valid=true;d.track_id=7;
    previous.state=PushState::CUE_APPROACH;previous.target_is_search_cue=true;previous.cue_contact_allowed=true;previous.target_id=7;
    tick({d});assert(!in.path_safe);
    d.label="unknown";tick({d});assert(!in.path_safe);
    d.label="dangerous_object";previous.state=PushState::APPROACH;
    tick({d});assert(!in.path_safe);
    previous.target_is_search_cue=false;
    previous.first_ordinary_delivered=true;in.target_region_valid=false;in.geometry_valid=true;tick();
    assert(!in.target_region_valid); // no re-picking unseen delivered objects
    { // Same near blue: straight travel can clear a side object, rotation cannot.
        ControlledField directional(g,.32f,{.22f,.10f,.15f,.15f,.18f,.22f});
        PushObservation obs;obs.geometry_valid=true;obs.distance_m=.4f;
        SegDetection blue;blue.label="dangerous_object";blue.ground_position_valid=true;blue.body_xy_m={.25f,.1f};
        directional.update(obs,{blue},halves,{1280,720},previous,true);
        assert(obs.directional_clearance_valid&&obs.path_safe&&obs.jaw_open_safe&&!obs.turn_safe&&!obs.arc_safe);
        blue.body_xy_m={.085f,.272f};directional.update(obs,{blue},halves,{1280,720},previous,true);
        assert(!obs.path_safe); // The actual observed front blue really blocks this path.
        blue.body_xy_m={0,-.25f};directional.update(obs,{blue},halves,{1280,720},previous,true);
        assert(obs.path_safe&&!obs.retreat_safe);
        blue.ground_position_valid=false;directional.update(obs,{blue},halves,{1280,720},previous,true);
        assert(!obs.path_safe&&!obs.retreat_safe&&!obs.turn_safe&&!obs.jaw_open_safe);
    }
    { // Only explicit contact-test permission overrides missing and blocked clearance.
        ControlledField field(g,.32f,{.22f,.10f,.15f,.15f,.18f,.22f});
        PushObservation obs;SegDetection blue;blue.label="dangerous_object";
        field.update(obs,{blue},halves,{1280,720},previous,false,true);
        assert(obs.clearance_checks_disabled&&obs.path_safe&&obs.retreat_safe);
        assert(obs.turn_safe&&obs.arc_safe&&obs.jaw_open_safe);
        assert(!obs.zone_counts_valid&&!obs.zone_inventory_complete&&!obs.clear_push_safe);
        field.update(obs,{blue},halves,{1280,720},previous,false);
        assert(!obs.clearance_checks_disabled&&!obs.path_safe&&!obs.retreat_safe);
    }
    { // Contact allowance requires a known non-overlapping zone and fresh ground evidence.
        ControlledField field(g,.32f,{.22f,.10f,.15f,.15f,.18f,.22f});
        PushObservation obs=in;obs.zone_own=obs.zone_identity_verified=true;obs.zone_estimate.origin_body_m={0,1.f};
        SegDetection blue;blue.label="dangerous_object";blue.ground_position_valid=blue.ground_contact_valid=true;
        blue.body_xy_m={0,.24f};blue.timestamp_us=obs.now_us;
        field.update(obs,{blue},halves,{1280,720},previous,true);assert(obs.clear_push_safe);
        obs.zone_identity_verified=false;
        field.update(obs,{blue},halves,{1280,720},previous,true);assert(!obs.clear_push_safe);
        obs.zone_identity_verified=true;obs.zone_estimate.origin_body_m={0,.25f};
        field.update(obs,{blue},halves,{1280,720},previous,true);assert(!obs.clear_push_safe);
    }
    std::cout<<"Controlled-field visible inventory, provenance and hazard veto passed\n";
}
