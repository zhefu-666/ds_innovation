#include "../../support/push_task_sim.hpp"
#include "../../../src/task/task_tick.hpp"
namespace {
TaskTuning tuning() {
    TaskTuning t; t.pixel_grasp_enabled=true; t.startup_advance_us=0;
    t.near_pitch_cdeg=2000; t.track_pitch_cdeg=0; return t;
}
struct Rig {
    PushTask task{tuning()}; PushObservation in; PushOutput out;
    std::vector<PushObservation::CaptureImageObject> objects;
    Rig(float x=650, float y=250) {
        in.now_us=1000000;
        in.run=in.safety_ok=in.path_safe=in.retreat_safe=in.opponent_zone_clear=true;
        in.camera_pitch_stable=in.sent_velocity_valid=in.heading_valid=true;
        in.camera_pitch_cdeg=2000;
        in.target_valid=in.target_center_px_valid=true;
        in.target_id=1; in.label="ordinary_supply";
        in.pixel_grasp_view=in.pixel_grasp_near_view=in.pixel_grasp_target_valid=true;
        in.target_grasp_range_valid=in.target_grasp_range_inside=true;
        in.target_grasp_range_lateral=in.target_grasp_range_lateral_measured=true;
        in.target_grasp_range_applicable=true;
        in.target_center_px={x,y}; in.pixel_grasp_error_px=647.5f-x;
        in.capture_area_px={342,226,689,263};
        in.capture_class_regions_px["ordinary_supply"]=in.capture_area_px;
        in.layout_count_region_px={342,180,689,309};
        in.capture_path_px={342,205,689,284};
        in.capture_scene_valid=in.capture_path_valid=in.capture_path_lateral_measured=true;
        in.corridor_complete=in.corridor_occlusion_free=true; in.corridor.ordinary=1;
        in.gripper_done=true; in.gripper_feedback_open=1;
        task.trip_context_.target_id_=task.trip_context_.capture_target_id_=1;
        task.trip_context_.label_=in.label; task.pitch_cmd_=2000; task.frame_raised_=1;
        task.enter(PushState::APPROACH,in.now_us,"entry_fixture");
        task.search_phase_.approach_pitch_=2000; task.last_us_=in.now_us;
        objects={{1,in.label,{int(x)-30,int(y)-30,60,60},true}}; fresh();
    }
    void fresh() {
        in.pixel_grasp_sample_us=in.capture_scene_us=in.now_us;
        in.capture_image_objects=objects;
    }
    void step(bool fresh_frame=true) {
        in.now_us+=50000; if(fresh_frame) fresh(); out=task.update(in);
    }
    bool locate() {
        in.now_us+=50000; fresh();
        TaskTick tick(in); tick.forward_ok=tick.scan_ok=true;
        return task.locateSearchEntry(tick);
    }
};
void anywhereInsideStopsAndHandsOff() {
    for(float x:{360.f,650.f,1000.f}) {
        Rig r(x);
        for(int i=0;i<40 && r.task.state_!=PushState::RUSH;++i) {
            r.step(); assert(stopped(r.out.motion));
        }
        assert(r.task.state_==PushState::RUSH && r.task.search_entry_.first_id==1);
        assert(r.out.entry_confirmed && r.out.entry_position_ok);
        assert(!r.in.pixel_grasp_ready); // centre Y250, no Y337 requirement
        r.step(); assert(r.task.state_==PushState::LOWER_FRAME && stopped(r.out.motion));
        r.in.target_center_px.x=1100; r.in.target_grasp_range_inside=false;
        r.step(); assert(r.task.state_==PushState::PREPARE && !r.task.search_entry_.handoff);
    }
}
void entryOrder() {
    Rig earlier;
    earlier.in.target_grasp_range_inside=false;
    earlier.in.target_center_px.y=210;
    earlier.objects[0].box.y=180;
    earlier.objects.push_back({2,"ordinary_supply",{800,250,60,60},true});
    earlier.locate();
    assert(earlier.task.search_entry_.first_id==2);
    earlier.in.target_center_px.y=250;
    earlier.in.target_grasp_range_inside=true;
    earlier.objects[0].box.y=220;
    for(int i=0;i<30 && earlier.task.state_!=PushState::ABORT_DROP;++i) earlier.locate();
    assert(earlier.task.state_==PushState::ABORT_DROP && !earlier.task.search_entry_.handoff);
    Rig class_range;
    class_range.in.target_grasp_range_inside=false;
    class_range.in.target_center_px.y=210;
    class_range.objects[0].box.y=180;
    class_range.in.capture_class_regions_px["core_supply"]={268.5f,236,748.5f,319};
    class_range.objects.push_back({3,"core_supply",{270,480,60,60},true});
    class_range.locate();
    assert(class_range.task.search_entry_.first_id==3); // own class range, outside ordinary X
    Rig tie;
    tie.objects.push_back({2,"core_supply",{800,250,60,60},true}); tie.locate();
    assert(tie.task.search_entry_.first_id==-2);
    Rig later;
    later.locate(); assert(later.task.search_entry_.first_id==1);
    later.objects.push_back({2,"ordinary_supply",{800,250,60,60},true});
    bool ready=false;
    for(int i=0;i<30 && !ready;++i) ready=later.locate();
    assert(ready && later.task.search_entry_.first_id==1); // later objects do not change first
    later.task.clearTrip(); assert(later.task.search_entry_.first_id==-1);
}
void freshnessAndReset() {
    Rig r;
    for(int i=0;i<12;++i) r.step(false);
    assert(!r.task.search_entry_.confirmed && r.task.state_!=PushState::RUSH);
    Rig changing;
    changing.locate(); assert(changing.task.search_entry_.first_id==1);
    changing.in.camera_pitch_stable=false; changing.step();
    assert(changing.task.search_entry_.first_id==-1 && !changing.task.search_entry_.confirmed);
    Rig splitting;
    for(int i=0;i<30 && !splitting.task.search_entry_.confirmed;++i) splitting.locate();
    assert(splitting.task.search_entry_.confirmed);
    TaskTick tick(splitting.in); splitting.task.beginPixelSplit(tick);
    assert(splitting.task.search_entry_.first_id==-1 && !splitting.task.search_entry_.confirmed);
}
void removedActionsCannotExecute() {
    for (auto state : {PushState::CUE_APPROACH, PushState::CUE_PREPARE,
                       PushState::CLEAR_PILE, PushState::CLEAR_PAUSE,
                       PushState::SCATTER, PushState::PILE}) {
        Rig r;
        r.task.enter(state, r.in.now_us, "legacy_action");
        r.step();
        assert(r.out.state == PushState::SAFE_STOP && r.out.run_status == RunStatus::Fault);
        assert(r.out.reason == "removed_search_action" && stopped(r.out.motion));
    }
}
void searchAndCaptureUseTheSameSeparation() {
    for (bool capture : {false, true}) {
        Rig r;
        r.objects.push_back({2, "dangerous_object", {800,250,60,60}, true});
        if (capture) {
            r.task.enter(PushState::LOWER_FRAME, r.in.now_us, "capture_review");
            r.task.frame_raised_ = 1; // preclose review, matching the shared caller
        }
        bool reviewed = false;
        for (int i = 0; i < 30 && !r.task.search_phase_.first_trip_split_.pending; ++i) {
            r.in.now_us += 50000;
            r.fresh();
            TaskTick tick(r.in);
            tick.scan_ok = tick.forward_ok = true;
            reviewed = r.task.reviewFirstTripCapture(tick, capture ? "preclose" : "approach", capture);
            assert(stopped(tick.motion));
        }
        const auto& split = r.task.search_phase_.first_trip_split_;
        assert(!reviewed && split.pending && split.capture_retry && split.runs == 1);
        assert(r.task.state_ == PushState::ABORT_DROP);
        assert(split.recovery_stage == SearchPhaseData::FirstTripSplit::RecoveryStage::Retreat);
        assert(!split.upper_corner && split.retreat_objects.size() == 1);
        // Lower targets retain the current Y270 retreat reference: the wanted block.
        assert(split.retreat_objects[0].observation.id == 1);
    }
}

}
int main() {
    anywhereInsideStopsAndHandsOff(); entryOrder(); freshnessAndReset();
    removedActionsCannotExecute(); searchAndCaptureUseTheSameSeparation();
    std::cout<<"PASS UI140 first entry, full class range, order, freshness, split and handoff\n";
}
