#include "rescue/task/transport_rules.hpp"
#include "rescue/core/types.hpp"
#include "rescue/geometry/planner.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#define private public
#include "rescue/task/push_task.hpp"
#undef private
#include "rescue/control/controlled_field.hpp"
#include "rescue/control/motion_readiness.hpp"
#include "rescue/perception/pixel_selector.hpp"
#include "../../src/task/task_tick.hpp"
using namespace rescue;
static SegDetection box(int id,const char* label,float x,float y,int pixels=60) {
    SegDetection d;d.track_id=id;d.label=label;d.confidence=.95f;d.timestamp_us=1000000;
    d.ground_position_valid=d.ground_contact_valid=true;d.body_xy_m={x,y};
    d.box={int(600+500*x),int(300+200*y),pixels,pixels};return d;
}
static void startupMemberChanges() {
    StartupPileGuide guide;auto left=box(1,"ordinary_supply",0,.5f);
    auto right=box(2,"ordinary_supply",0,.5f);auto edge=box(3,"core_supply",0,.5f);
    left.box={450,200,80,100};right.box={650,200,80,100};edge.box={1100,100,80,100};
    cv::Point2f previous;
    for(int n=0;n<10;++n) {
        auto now=uint64_t(1000000+30000*n);
        left.timestamp_us=right.timestamp_us=edge.timestamp_us=now;
        std::vector<const SegDetection*> ds{&left,&right};
        if(n>=3)ds.push_back(&edge);
        const auto center=guide.update(ds,{1280,720},0,now);
        if(n>0)assert(std::abs(center.x-previous.x)<=15.f);
        previous=center;
    }
    // Coherent motion of the retained cohort still changes the guide, rather
    // than freezing old image coordinates.
    left.box.x+=100;right.box.x+=100;edge.box.x+=100;
    left.timestamp_us=right.timestamp_us=edge.timestamp_us=1400000;
    guide.update({&left,&right,&edge},{1280,720},0,1400000);
    left.timestamp_us=right.timestamp_us=edge.timestamp_us=1430000;
    const auto moved=guide.update({&left,&right,&edge},{1280,720},0,1430000);
    assert(moved.x>previous.x);
    std::cout<<"PASS membership changes do not jump the UI120 guide\n";
}
static void startupBudget() {
    TaskTuning t;t.startup_advance_us=1;t.start_zone=3;t.pixel_grasp_enabled=true;
    PushTask task(t);PushObservation in;in.now_us=1000000;
    in.run=in.safety_ok=in.path_safe=in.retreat_safe=in.opponent_zone_clear=true;
    in.camera_pitch_stable=in.gripper_done=in.sent_velocity_valid=true;
    in.camera_pitch_cdeg=t.track_pitch_cdeg;in.gripper_feedback_open=0;task.update(in);
    task.startup_action_=PushTask::StartupAction::Approach;task.startup_pitch_=t.track_pitch_cdeg;
    in.target_valid=in.target_center_px_valid=in.pixel_grasp_view=in.pixel_grasp_target_valid=true;
    in.target_id=kStartupPileTargetId;in.label="startup_pile";in.startup_visible_classes=1;
    PushOutput out;
    for(int n=0;n<160;++n) {
        in.now_us+=50000;in.startup_scene_us=in.pixel_grasp_sample_us=in.now_us;
        in.pixel_grasp_error_px=(n%12<6?110.f:-95.f);in.target_center_px={640-in.pixel_grasp_error_px,300};
        out=task.update(in);assert(out.motion.vx_mps==0);in.sent_wz_rps=out.motion.wz_rps;
        if(out.state==PushState::SAFE_STOP)break;
    }
    assert(out.state==PushState::SAFE_STOP);
    assert(out.reason=="startup_pile_alignment_stalled" || out.reason=="startup_pile_alignment_timeout");
    assert(in.now_us<=7500000);std::cout<<"PASS bounded UI120 virtual pile alignment\n";
}
// Regression for the 14:09 UI120 pile episode: a 100 ms request can expire
// during UART ramp-up, and one closing box sample can brake it even earlier.
static void startupPileSentPulse() {
    for (float sign : {-1.f, 1.f}) {
        TaskTuning t;
        t.pixel_grasp_enabled = true;
        PushTask task(t);
        task.phase_ = TaskPhase::Startup;
        task.state_ = PushState::START_ADVANCE;
        task.startup_action_ = PushTask::StartupAction::Approach;
        PushObservation in;
        in.now_us = 1000000;
        in.target_valid = in.target_center_px_valid = true;
        in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
        in.sent_velocity_valid = in.camera_pitch_stable = true;
        in.target_id = kStartupPileTargetId;
        in.label = "startup_pile";
        const auto step = [&](uint64_t dt, float error, float sent) {
            in.now_us += dt;
            in.pixel_grasp_sample_us = in.now_us;
            in.pixel_grasp_error_px = sign * error;
            in.target_center_px = {640 - in.pixel_grasp_error_px, 350};
            in.sent_wz_rps = sign * sent;
            TaskTick tick(in);
            tick.scan_ok = tick.forward_ok = true;
            task.pixelAlign(tick);
            return tick.motion.wz_rps * sign;
        };
        assert(step(0, 62, 0) >= .06f);
        // A closing sample outside the 30 px band is image noise, not proof
        // that the chassis has already completed the requested correction.
        const float after_jitter = step(30000, 46, 0);
        std::cout << "pile pulse after closing jitter=" << after_jitter << std::endl;
        assert(after_jitter >= .06f);
        assert(step(90000, 46, 0) >= .06f); // request age >100 ms; nothing sent yet
        assert(task.pixel_alignment_.correction_since_us == 0);
        assert(step(30000, 46, .08f) >= .06f); // start the sent-motion budget
        const auto sent_start = in.now_us;
        assert(task.pixel_alignment_.correction_since_us == sent_start);
        assert(step(90000, 46, .08f) >= .06f);
        assert(step(10000, 46, .08f) == 0); // a full 100 ms was actually sent
        assert(task.pixel_alignment_.stage == PushTask::PixelAlignment::Stage::Brake);
        // Crossing the centre / entering tolerance still cancels a pulse at once.
        task.pixel_alignment_ = {};
        assert(step(300000, 62, 0) > 0);
        assert(step(30000, -46, .08f) == 0);
        task.pixel_alignment_ = {};
        assert(step(300000, 62, 0) > 0);
        assert(step(30000, 20, .08f) == 0);
    }
    std::cout << "PASS pile correction survives jitter/ramp-up and keeps immediate safety stops\n";
}

static void startupPileCorrectionConverges() {
    for (float pixels_per_rad : {1000.f, 4000.f}) for (float sign : {-1.f, 1.f}) {
        TaskTuning t;
        t.startup_advance_us = 1;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        t.pixel_align_brake_motion_px = 30;
        PushTask task(t);
        PushObservation in;
        in.now_us = 1000000;
        in.run = in.safety_ok = in.path_safe = in.retreat_safe = in.opponent_zone_clear = true;
        in.camera_pitch_stable = in.gripper_done = in.sent_velocity_valid = true;
        in.camera_pitch_cdeg = t.track_pitch_cdeg;
        in.gripper_feedback_open = 0;
        task.update(in);
        task.startup_action_ = PushTask::StartupAction::Approach;
        task.startup_pitch_ = t.track_pitch_cdeg;
        in.target_valid = in.target_center_px_valid = true;
        in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
        in.target_id = kStartupPileTargetId;
        in.label = "startup_pile";
        in.startup_visible_classes = 1;
        float error = sign * 62.f;
        std::array<float, 3> delayed_wz{}; // 150 ms delayed image response
        bool sustained_forward = false;
        const auto started = in.now_us;
        for (int n = 0; n < 120; ++n) {
            in.now_us += 50000;
            error -= delayed_wz[0] * .05f * pixels_per_rad;
            in.pixel_grasp_error_px = error + sign * (n % 2 ? -10.f : 0.f);
            in.target_center_px = {640 - in.pixel_grasp_error_px, 350};
            in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us;
            const auto out = task.update(in);
            if (out.state == PushState::SAFE_STOP) break;
            if (in.sent_vx_mps > .01f && !task.startup_pile_alignment_.started_us) {
                sustained_forward = true;
                break;
            }
            // Match MotionLink's .8 rad/s^2 angular ramp and immediate stops.
            in.sent_wz_rps = out.immediate_motion_stop ? 0.f : in.sent_wz_rps +
                std::clamp(out.motion.wz_rps - in.sent_wz_rps, -.05f, .04f);
            in.sent_vx_mps = out.immediate_motion_stop ? 0.f : in.sent_vx_mps +
                std::clamp(out.motion.vx_mps - in.sent_vx_mps, -.02f, .015f);
            delayed_wz[0] = delayed_wz[1];
            delayed_wz[1] = delayed_wz[2];
            delayed_wz[2] = in.sent_wz_rps;
        }
        std::cout << "pile convergence gain=" << pixels_per_rad << " sign=" << sign
                  << " elapsed_us=" << in.now_us - started << " forward=" << sustained_forward
                  << " error=" << error << " reason=" << task.reason_ << std::endl;
        assert(sustained_forward);
        assert(in.now_us - started < 6000000);
        assert(task.frame_raised_ == 0 && task.desiredPitch() == t.track_pitch_cdeg);
        in.now_us += 50000;
        in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us;
        in.startup_lower_corners_occupied = true;
        const auto handoff = task.update(in);
        assert(handoff.reason == "startup_pitch_turn_ready");
        assert(handoff.immediate_motion_stop && handoff.motion.vx_mps == 0 &&
               handoff.motion.wz_rps == 0);
    }
}

// Reconstruct the 17:48 run's post-delivery search sample (log lines 6976-6985).
// The larger injured target outside the chosen row must compete with every supply.
static void loggedSearchSelection() {
    std::vector<SegDetection> ds;
    {
        auto d = box(240, "ordinary_supply", 0.346897f, 0.610278f);
        d.confidence = 0.866933f;
        d.box = {1104, 427, 67, 66};
        ds.push_back(d);
    }
    {
        auto d = box(239, "ordinary_supply", 0.29131f, 0.559722f);
        d.confidence = 0.901921f;
        d.box = {1054, 442, 86, 76};
        ds.push_back(d);
    }
    {
        auto d = box(237, "dangerous_object", 0.424761f, 1.09748f);
        d.confidence = 0.676441f;
        d.box = {971, 326, 41, 36};
        ds.push_back(d);
    }
    {
        auto d = box(234, "injured_person", 0.540298f, 1.07758f);
        d.confidence = 0.894146f;
        d.box = {1056, 331, 62, 36};
        ds.push_back(d);
    }
    {
        auto d = box(233, "injured_person", 0.178285f, 0.404471f);
        d.confidence = 0.886371f;
        d.box = {941, 515, 172, 121};
        ds.push_back(d);
    }
    {
        auto d = box(232, "ordinary_supply", 0.617127f, 0.888841f);
        d.confidence = 0.851382f;
        d.box = {1210, 364, 66, 41};
        ds.push_back(d);
    }
    {
        auto d = box(228, "ordinary_supply", 0.154758f, 0.990622f);
        d.confidence = 0.83972f;
        d.box = {780, 333, 41, 42};
        ds.push_back(d);
    }
    {
        auto d = box(226, "injured_person", 0.378924f, 1.14072f);
        d.confidence = 0.905809f;
        d.box = {918, 323, 56, 32};
        ds.push_back(d);
    }
    {
        auto d = box(225, "ordinary_supply", 0.472862f, 1.1026f);
        d.confidence = 0.800844f;
        d.box = {1003, 326, 47, 36};
        ds.push_back(d);
    }
    {
        auto d = box(214, "injured_person", 0.42887f, 0.642749f);
        d.confidence = 0.905809f;
        d.box = {1172, 409, 92, 71};
        ds.push_back(d);
    }
    PixelSelector selector;
    PixelSelection picked;
    for (int n = 0; n < 4; ++n) {
        const auto now = uint64_t(1000000 + n * 33333);
        for (auto& d : ds)
            d.timestamp_us = now;
        picked = selector.update(ds, {1280, 720}, -3000, true, now, true, .6f);
    }
    std::cout << "logged post-delivery selection id=" << picked.id << std::endl;
    assert(picked.locked && picked.id == 233);
    std::cout << "PASS post-delivery injured target outside a supply row\n";
}

static void scoreSelection() {
    loggedSearchSelection();
    for(int s=0;s<5;++s) {
        PixelSelector selector;std::vector<SegDetection> ds;
        if(s==0 || s==3)ds={box(1,"ordinary_supply",-.3f,.6f,140),box(2,"injured_person",0,.6f),box(3,"core_supply",.3f,.6f)};
        if(s==1 || s==2)ds={box(1,"ordinary_supply",-.4f,.6f,140),box(2,"injured_person",-.15f,.6f),box(3,"core_supply",.2f,.6f),box(4,s==1?"core_supply":"ordinary_supply",.26f,.6f)};
        if(s==4)ds={box(1,"ordinary_supply",-.3f,.6f,140),box(2,"ordinary_supply",0,.6f),box(3,"ordinary_supply",.3f,.6f)};
        PixelSelection picked;
        for(int n=0;n<5;++n) {
            auto now=uint64_t(1000000+50000*n);for(auto& d:ds)d.timestamp_us=now;
            picked=selector.update(ds,{1280,720},-3000,true,now,s!=3,.5f);
        }
        assert(picked.locked);
        if(s==0 || s==2)assert(picked.id==2);
        if(s==1)assert(picked.id==3 || picked.id==4);
        if(s==3)assert(picked.id==1);
        selector.syncTaskTarget(picked.id,ds[picked.id-1].label);
        ds.push_back(box(9,"injured_person",.5f,.6f,200));for(auto& d:ds)d.timestamp_us=1300000;
        auto held=selector.update(ds,{1280,720},-3000,true,1300000,true,.5f);
        assert(held.id==picked.id && held.locked);
    }
    std::cout<<"PASS shared feasible-trip scoring and lock continuity\n";
}

static TaskTuning rejectTuning() {
    TaskTuning t;t.pixel_grasp_enabled=t.field_carry=true;
    t.far_pitch_cdeg=-3000;t.track_pitch_cdeg=0;t.near_pitch_cdeg=2000;
    t.carry_budget_us=60000000;t.max_speed=.4f;t.max_wz=t.scan_wz=.6f;
    return t;
}
static void inspectionOnlyBeforePush() {
    const auto t=rejectTuning();PushTask task(t);
    task.phase_=TaskPhase::Transport;task.state_=PushState::CARRY;
    task.transport_phase_.approach_pitch_=t.far_pitch_cdeg;
    PushObservation in;in.now_us=1000000;in.camera_pitch_cdeg=t.far_pitch_cdeg;
    in.transport_checks_enabled=in.transport_external_blocked=true;
    in.sent_velocity_valid=in.heading_valid=in.camera_pitch_stable=true;
    in.path_safe=in.turn_safe=in.arc_safe=in.opponent_zone_clear=true;
    TaskTick tick(in);tick.scan_ok=tick.forward_ok=true;task.updatePixelCarry(tick);
    assert(task.desiredPitch()==t.far_pitch_cdeg);
    assert(task.reason_.find("review")==std::string::npos);
    task.transport_phase_.jaw_sequence_.stage=TransportPhaseData::JawSequence::Stage::ColourAdvance;
    in.now_us+=50000;TaskTick prepush(in);task.updatePixelCarry(prepush);
    assert(task.desiredPitch()==kTransportReviewPitchCdeg);
    assert(prepush.immediate_motion_stop && prepush.motion.vx_mps==0);
    // A fresh UI120 frame must not satisfy the UI130 review.
    in.now_us+=50000;in.capture_scene_us=in.now_us;
    in.capture_scene_valid=in.pixel_grasp_view=in.transport_review_clear=true;
    in.camera_pitch_cdeg=t.track_pitch_cdeg;
    TaskTick wrong_view(in);task.updatePixelCarry(wrong_view);
    assert(task.reason_=="carry_pre_push_image_wait");
    assert(wrong_view.immediate_motion_stop && wrong_view.motion.vx_mps==0);
    std::cout<<"PASS inspection occurs only before final safe-zone push\n";
}
struct AvoidRig {
    TaskTuning tuning=rejectTuning();PushTask task{tuning};PushObservation in;
    ControlledField field{{},.32f,{.22f,.10f,.15f,.15f,.18f,.22f}};
    PushOutput out;float yaw=0,position=0,velocity=0;uint64_t sequence=0;
    bool obstacle=true,visible=true,samples=true,ack=true,follow=true;
    bool injured=false,core=false,sparse_regions=false;int held_id=10;
    std::string extra_label;int reviews=0;bool opened=false,turned=false;
    float turn_position=0;bool reversed=false;
    int obstacle_x=650;float expected_turn=1;
    std::vector<cv::Rect> additional_obstacles;
    explicit AvoidRig(bool injury=false,bool supply_core=false):injured(injury),core(supply_core) {
        in.now_us=1000000;in.run=in.safety_ok=in.gripper_done=true;
        in.heading_valid=in.sent_velocity_valid=in.camera_pitch_stable=true;
        task.phase_=TaskPhase::Transport;task.state_=PushState::CARRY;task.run_status_=RunStatus::Running;
        if(injury)task.trip_context_.trip_.injured=1;else if(core)task.trip_context_.trip_.core=1;else task.trip_context_.trip_.ordinary=1;
        task.trip_context_.transported_objects={{10,injury?"injured_person":core?"core_supply":"ordinary_supply",{500,350,100,100},true}};
        task.trip_context_.transported_view_pitch=tuning.near_pitch_cdeg;
        task.transport_phase_.half_injured_=injury;
        task.transport_phase_.carry_us_=task.last_us_=in.now_us;
        task.frame_raised_=task.wire_last_=0;task.frame_transaction_=task.wire_txn_=41;
        task.pitch_cmd_=task.transport_phase_.approach_pitch_=tuning.track_pitch_cdeg;
        task.pitch_wait_us_=in.now_us;
        task.transport_phase_.jaw_sequence_.stage=TransportPhaseData::JawSequence::Stage::ColourAdvance;
        out.phase=TaskPhase::Transport;out.state=PushState::CARRY;out.run_status=RunStatus::Running;
        out.navigation_approach=true;out.transported_inventory=task.trip_context_.trip_;
        out.transported_objects=task.trip_context_.transported_objects;out.transported_view_pitch=tuning.near_pitch_cdeg;
        if(core)task.first_=out.first_ordinary_delivered=true;
    }
    PushOutput step() {
        const float target=follow?out.motion.vx_mps:0;
        const float accel=(target-velocity)/.05f;
        yaw=wrapAngle(yaw+out.motion.wz_rps*.05f);
        in.imu_translation_samples.clear();
        for(int i=0;i<5;++i) {
            in.now_us+=10000;const float next=velocity+accel*.01f;
            position+=(velocity+next)*.005f;velocity=next;
            if(samples) {
                ImuTranslationSample a;a.sequence=++sequence;a.received_us=in.now_us;
                a.device_time_ms=in.now_us/1000;a.valid=true;
                a.acceleration_mps2={accel+.01f,0,9.8f};
                a.quaternion_wxyz={std::cos(yaw*.5f),0,0,std::sin(yaw*.5f)};
                in.imu_translation_samples.push_back(a);
            }
        }
        in.heading_rad=yaw;in.sent_vx_mps=out.motion.vx_mps;in.sent_wz_rps=out.motion.wz_rps;
        in.camera_pitch_cdeg=task.desiredPitch();in.camera_pitch_stable=true;
        in.gripper_done=ack;in.gripper_feedback_open=task.frame_raised_;
        in.transport_checks_enabled=true;in.capture_center_membership=true;
        in.pixel_grasp_near_view=in.camera_pitch_cdeg==tuning.near_pitch_cdeg;
        in.pixel_grasp_view=in.pixel_grasp_near_view || in.camera_pitch_cdeg==tuning.track_pitch_cdeg;
        in.capture_scene_valid=visible;in.capture_scene_us=visible?in.now_us:0;
        in.capture_class_regions_px={{"ordinary_supply",{209,300,862,300}},
            {"injured_person",{209,300,862,300}},{"core_supply",{209,300,862,300}}};
        in.capture_path_px={209,100,862,500};in.layout_count_region_px={209,300,862,300};
        if(sparse_regions) {in.capture_class_regions_px.clear();in.capture_path_px={};}
        std::vector<SegDetection> ds;
        auto held=box(held_id,injured?"injured_person":core?"core_supply":"ordinary_supply",0,.1f);
        held.box={500,350,100,100};held.ground_position_valid=false;held.timestamp_us=in.now_us;
        ds.push_back(held);
        if(obstacle && visible) {
            auto extra=box(20,extra_label.empty()?(injured?"ordinary_supply":"injured_person"):extra_label.c_str(),0,.2f);
            extra.box={obstacle_x,150+int(std::min(.06f,std::max(0.f,position))*5000),80,80};
            extra.ground_position_valid=false;extra.timestamp_us=in.now_us;ds.push_back(extra);
            for(const auto& rect:additional_obstacles) {
                extra.box=rect;extra.track_id++;ds.push_back(extra);
            }
        }
        in.capture_image_objects.clear();
        if(visible)for(const auto& d:ds)in.capture_image_objects.push_back({d.track_id,d.label,d.box,true});
        field.update(in,visible?ds:std::vector<SegDetection>{},{},{1280,720},out,false);
        in.transport_floor.valid=in.transport_floor.region_valid=true;
        in.transport_floor.sample_us=in.now_us;in.transport_floor.inside=false;
        out=task.update(in);
        if(out.motion.vx_mps!=0 || out.motion.wz_rps!=0)
            assert(!residualMotionBlocked(in,out,out.motion.vx_mps,out.motion.wz_rps));
        if(out.reason=="carry_pre_push_review")++reviews;
        if(out.motion.gripper_offset==0)opened=true;
        if(out.motion.vx_mps<0) {
            assert(!turned && out.motion.wz_rps==0);reversed=true;
        }
        if(out.motion.wz_rps!=0 && out.reason.find("carry_avoid_")==0) {
            assert(reversed && out.motion.vx_mps==0 && position<=-.095f);
            assert(out.motion.wz_rps*expected_turn>0);
            if(!turned)turn_position=position;
            turned=true;
        }
        return out;
    }
};

static void frontCategoryReview() {
    ControlledField field{{},.32f,{.22f,.10f,.15f,.15f,.18f,.22f}};
    PushOutput previous;previous.phase=TaskPhase::Transport;
    previous.state=PushState::CARRY;previous.run_status=RunStatus::Running;
    previous.transported_inventory.ordinary=1;
    previous.transported_view_pitch=2000;
    previous.transported_objects={{10,"ordinary_supply",{500,350,100,100},true}};
    PushObservation in;in.now_us=in.capture_scene_us=1000000;
    in.capture_scene_valid=in.pixel_grasp_view=in.camera_pitch_stable=true;
    in.pixel_grasp_near_view=false;
    in.camera_pitch_cdeg=kTransportReviewPitchCdeg;
    in.pixel_grasp_view=false;in.layout_count_region_px={209,180,862,320};
    const auto review=[&](const char* label,cv::Rect rect) {
        auto d=box(99,label,0,.2f);d.box=rect;
        d.ground_position_valid=false;d.timestamp_us=in.now_us;
        field.update(in,{d},{},{1280,720},previous,false);
    };
    // Matches the failed run: no capture/path regions, changed ID and position.
    review("ordinary_supply",{327,429,252,132});
    assert(in.transport_review_clear && !in.transport_external_blocked);
    assert(in.transport_front_objects.empty());
    previous.first_ordinary_delivered=true;
    review("core_supply",{500,80,80,80});
    assert(in.transport_review_clear);
    previous.first_ordinary_delivered=false;
    review("core_supply",{500,80,80,80});
    assert(in.transport_external_blocked && !in.transport_review_clear);
    for(const char* label:{"dangerous_object","unknown","injured_person"}) {
        review(label,{500,80,80,80});
        assert(in.transport_external_blocked && in.transport_front_objects.size()==1);
        assert(!in.transport_review_clear);
        review(label,{0,300,80,80});
        assert(in.transport_review_clear && !in.transport_external_blocked);
    }
    previous.transported_inventory={};previous.transported_inventory.injured=1;
    review("injured_person",{500,350,80,80});
    assert(in.transport_review_clear);
    for(const char* label:{"ordinary_supply","core_supply","dangerous_object"}) {
        review(label,{500,350,80,80});
        assert(in.transport_external_blocked && !in.transport_review_clear);
    }
    in.capture_scene_us=in.now_us-200001;
    review("injured_person",{500,350,80,80});
    assert(!in.transport_review_clear && !in.transport_external_blocked);
    in.capture_scene_us=in.now_us;in.camera_pitch_stable=false;
    review("injured_person",{500,350,80,80});
    assert(!in.transport_review_clear);
    std::cout<<"PASS front categories, changed cargo identity, empty capture/path, side objects and fresh view"<<std::endl;
}
static void firstTripPrePushQuantityReview() {
    ControlledField field{{},.32f,{.22f,.10f,.15f,.15f,.18f,.22f}};
    PushOutput previous;previous.phase=TaskPhase::Transport;
    previous.state=PushState::CARRY;previous.run_status=RunStatus::Running;
    previous.transported_inventory.ordinary=1;
    PushObservation in;in.now_us=in.capture_scene_us=1000000;
    in.capture_scene_valid=in.pixel_grasp_view=in.camera_pitch_stable=true;
    in.camera_pitch_cdeg=kTransportReviewPitchCdeg;
    in.pixel_grasp_view=false;in.layout_count_region_px={209,0,862,720};
    // Replay both boxes from match-main-20261105-021705-93006.log, line 4280.
    // The bottom box is clipped; neither old IDs nor complete boxes gate this review.
    auto ahead=box(43,"ordinary_supply",0,.2f);
    ahead.box={568,349,192,232};ahead.ground_position_valid=false;
    auto held=box(44,"ordinary_supply",0,.1f);
    held.box={840,610,212,110};held.ground_position_valid=false;
    const auto update=[&](std::vector<SegDetection> ds) {
        field.update(in,ds,{},{1280,720},previous,false);
    };
    update({held,ahead});
    assert(in.transport_external_blocked && !in.transport_review_clear);
    assert(in.transport_clearance_reason=="first_trip_too_many_supplies");
    assert(in.transport_front_objects.size()==2);
    assert(in.transport_front_objects[0].id==44 && in.transport_front_objects[1].id==43);
    update({ahead,held});
    assert(in.transport_external_blocked && in.transport_front_objects.size()==2);
    // A single visible ordinary is not added to the old one-item inventory again.
    update({held});assert(in.transport_review_clear && in.transport_front_objects.empty());
    update({ahead});assert(in.transport_review_clear);
    ahead.box={0,349,80,100};
    update({held,ahead});assert(in.transport_review_clear);
    ahead.box={568,349,192,232};
    previous.first_ordinary_delivered=true;
    update({held,ahead});assert(in.transport_review_clear);
    previous.first_ordinary_delivered=false;
    in.capture_scene_us=in.now_us-200001;
    update({held,ahead});
    assert(!in.transport_review_clear && !in.transport_external_blocked);
    in.capture_scene_us=in.now_us;in.camera_pitch_stable=false;
    update({held,ahead});
    assert(!in.transport_review_clear && !in.transport_external_blocked);

    // The same first-trip over-limit now backs away and resumes this delivery.
    AvoidRig r;r.extra_label="ordinary_supply";r.expected_turn=-1;
    bool pushing=false;
    for(int n=0;n<750;++n) {
        const auto out=r.step();
        assert(!r.opened && out.state==PushState::CARRY);
        assert(out.reason!="carry_pre_push_clear");
        if(out.motion.vx_mps>0) {
            assert(r.reversed && r.turned && r.reviews>=3);
            assert(out.reason=="carry_ui120_region_advance");
            pushing=true;break;
        }
    }
    assert(pushing);
    std::cout<<"PASS first-trip two ordinary boxes avoid before push; one/side/later-trip cases remain allowed\n";
}

static void categoryReviewLifecycle() {
    for(const char* label:{"ordinary_supply","core_supply"}) {
        AvoidRig clean;clean.sparse_regions=true;clean.held_id=99;clean.extra_label=label;
        clean.task.first_=clean.out.first_ordinary_delivered=true;
        bool cleared=false,pushing=false;
        for(int n=0;n<90;++n) {
            const auto out=clean.step();
            assert(out.state!=PushState::SAFE_STOP && !clean.opened && !clean.turned);
            if(out.reason.find("carry_pre_push_")==0)
                assert(clean.task.desiredPitch()==kTransportReviewPitchCdeg);
            if(out.reason=="carry_ui120_region_advance")
                assert(clean.task.desiredPitch()==clean.tuning.track_pitch_cdeg);
            assert(!clean.in.pixel_grasp_near_view);
            cleared=cleared || out.reason=="carry_pre_push_clear";
            if(out.reason=="carry_ui120_region_advance" && out.motion.vx_mps>0) {pushing=true;break;}
        }
        assert(cleared && pushing);
    }
    AvoidRig missing;missing.visible=false;
    for(int n=0;n<90 && missing.out.state!=PushState::SAFE_STOP;++n)missing.step();
    assert(missing.out.state==PushState::SAFE_STOP && missing.out.reason=="carry_pre_push_unobservable");
    std::cout<<"PASS permitted extra cargo resumes push without old matching; missing images still stop"<<std::endl;
}

static void avoidanceLifecycle() {
    for(int kind:{0,1,2}) for(float initial_yaw:{0.f,3.1f,-3.1f})
        for(float side:{-1.f,0.f,1.f}) {
        AvoidRig r(kind==2,kind==1);r.yaw=initial_yaw;
        r.obstacle_x=side<0 ? 550 : side>0 ? 650 : 600;
        r.expected_turn=side!=0 ? side : kind==2 ? 1.f : -1.f;
        bool pushing=false;
        for(int n=0;n<750;++n) {
            auto out=r.step();
            if(r.reversed)r.obstacle_x=r.expected_turn<0 ? 650 : 550;
            assert(out.state==PushState::CARRY && !r.opened);
            assert(r.task.trip_context_.trip_.total()==1 && r.task.total_==0);
            if(out.reason.find("carry_pre_push_")==0)
                assert(r.task.desiredPitch()==kTransportReviewPitchCdeg);
            if(out.motion.vx_mps!=0 || out.motion.wz_rps!=0)
                assert(r.task.desiredPitch()==r.tuning.track_pitch_cdeg);
            if(out.motion.vx_mps>0) {
                assert(r.reversed && r.turned && r.reviews>=3);
                const float wanted=wrapAngle(initial_yaw+r.expected_turn*10.f*float(CV_PI)/180.f);
                assert(std::abs(wrapAngle(r.yaw-wanted))<.02f);
                assert(std::abs(wrapAngle(r.task.transport_phase_.jaw_sequence_.heading-wanted))<.001f);
                assert(r.turn_position>=-.12f && r.turn_position<=-.095f);
                assert(out.reason=="carry_ui120_region_advance");pushing=true;break;
            }
        }
        assert(pushing);
        // Keep the conflicting detections visible: this one-time maneuver must
        // neither loop nor let the push's old heading undo the 10-degree turn.
        const int reviewed=r.reviews;
        for(int n=0;n<12;++n) {
            const auto out=r.step();
            assert(out.reason=="carry_ui120_region_advance" && out.motion.vx_mps>0);
            assert(!r.opened && r.reviews==reviewed);
        }
    }
    std::cout<<"PASS left/right obstacles choose wider side; equal gaps retain supply-right/injured-left, then push at locked heading\n";
}

static void avoidanceUsesAllCurrentBoxes() {
    using Stage=TransportPhaseData::JawSequence::PrePushReview::Stage;
    for(bool injured:{false,true}) for(float side:{-1.f,1.f}) {
        AvoidRig r(injured);
        // A close box on one side must not hide the larger obstruction on the other.
        r.obstacle_x=side<0 ? 650 : 550;
        r.additional_obstacles={{side<0 ? 450 : 750,100,80,80}};
        r.expected_turn=side;
        bool pushing=false;
        for(int n=0;n<750;++n) {
            const auto out=r.step();
            assert(out.state==PushState::CARRY && !r.opened);
            const auto stage=r.task.transport_phase_.jaw_sequence_.prepush.stage;
            if(stage!=Stage::Idle && stage!=Stage::Review) {
                // The review choice survives disappearing evidence during the retreat.
                r.obstacle=false;
            }
            if(out.motion.vx_mps>0) {
                assert(r.reversed && r.turned);
                assert(std::abs(wrapAngle(r.yaw-side*10.f*float(CV_PI)/180.f))<.02f);
                pushing=true;break;
            }
        }
        assert(pushing);
    }
    std::cout<<"PASS all current forbidden boxes determine lateral gap; choice survives missing later boxes\n";
}

static void avoidanceIgnoresHeadingChanges() {
    using Stage=TransportPhaseData::JawSequence::PrePushReview::Stage;
    for(bool injured:{false,true}) for(Stage drift_stage:{Stage::BackReady,Stage::Back,Stage::BackStop}) {
        AvoidRig r(injured);bool drifted=false,pushing=false;
        for(int n=0;n<750;++n) {
            auto& review=r.task.transport_phase_.jaw_sequence_.prepush;
            if(!drifted && review.stage==drift_stage) {
                r.yaw=wrapAngle(r.yaw+r.expected_turn*.12f);
                drifted=true;
            }
            const auto out=r.step();
            if(r.reversed)r.obstacle_x=r.expected_turn<0 ? 650 : 550;
            assert(out.state==PushState::CARRY && !r.opened);
            if(out.motion.vx_mps>0) {
                assert(drifted && r.reversed && r.turned);
                const float wanted=r.expected_turn*(.12f+10.f*float(CV_PI)/180.f);
                assert(std::abs(wrapAngle(r.yaw-wanted))<.02f);
                assert(std::abs(wrapAngle(r.task.transport_phase_.jaw_sequence_.heading-wanted))<.001f);
                assert(r.turn_position>=-.12f && r.turn_position<=-.095f);
                assert(out.reason=="carry_ui120_region_advance");
                pushing=true;break;
            }
        }
        assert(pushing);
    }
    std::cout<<"PASS heading changes during avoidance calibration, reverse and stop retain 10cm retreat and relative 10deg turn\n";
}
static void cleanPushAndMissingEvidence() {
    AvoidRig clean;clean.obstacle=false;bool pushing=false;
    for(int n=0;n<100;++n) {
        auto out=clean.step();assert(!clean.opened && !clean.turned && !clean.reversed);
        if(out.reason=="carry_ui120_region_advance" && out.motion.vx_mps>0) {pushing=true;break;}
    }
    assert(pushing);
    AvoidRig missing;missing.visible=false;
    for(int n=0;n<90 && missing.out.state!=PushState::SAFE_STOP;++n)missing.step();
    assert(missing.out.state==PushState::SAFE_STOP && !missing.opened);
    assert(missing.out.reason=="carry_pre_push_unobservable");
    std::cout<<"PASS clear review pushes directly; missing review images stop\n";
}
static void avoidanceFailuresAndPause() {
    using Stage=TransportPhaseData::JawSequence::PrePushReview::Stage;
    AvoidRig stalled;
    for(int n=0;n<250 && stalled.task.reason_!="carry_avoid_back_010m";++n)stalled.step();
    assert(stalled.task.reason_=="carry_avoid_back_010m");
    stalled.follow=false;
    for(int n=0;n<100;++n) {stalled.step();assert(!stalled.turned && !stalled.opened);}
    assert(std::abs(stalled.task.transport_phase_.jaw_sequence_.prepush.imu.distance())<.02f);
    stalled.samples=false;
    for(int n=0;n<6;++n)stalled.step();
    assert(stalled.out.state==PushState::SAFE_STOP && stalled.out.reason=="carry_avoid_imu_gap");
    for(bool during_turn:{false,true}) {
        AvoidRig paused;bool did_pause=false,pushing=false;
        for(int n=0;n<750;++n) {
            const auto out=paused.step();
            auto& review=paused.task.transport_phase_.jaw_sequence_.prepush;
            if(!did_pause && (during_turn?paused.turned:
                 review.stage==Stage::Back && std::abs(review.imu.distance())>.03f)) {
                paused.in.run=false;paused.step();
                for(int k=0;k<8;++k)paused.step();
                paused.in.run=true;paused.step();did_pause=true;
                assert(paused.task.transport_phase_.jaw_sequence_.prepush.stage==
                       (during_turn?Stage::Turn:Stage::BackReady));
            }
            assert(paused.out.state!=PushState::SAFE_STOP && !paused.opened);
            if(out.motion.vx_mps>0) {
                assert(did_pause && paused.reversed && paused.turned);
                assert(paused.turn_position>=-.13f && paused.turn_position<=-.095f);
                assert(std::abs(paused.yaw-paused.expected_turn*10.f*float(CV_PI)/180.f)<.02f);
                pushing=true;break;
            }
        }
        assert(pushing);
    }
    AvoidRig feedback;feedback.in.sent_velocity_valid=false;
    feedback.step();
    assert(feedback.out.state==PushState::SAFE_STOP && feedback.out.reason=="carry_avoid_feedback_missing");
    std::cout<<"PASS measured retreat, IMU/feedback loss and pause/resume retain remaining distance and turn\n";
}


// 15:23 board trace: ordinary targets remain visible while one lock stalls.
struct OrdinaryReselectRig {
    TaskTuning tuning;
    PushTask task;
    PushObservation in;
    explicit OrdinaryReselectRig(bool open) : tuning([] {
        TaskTuning t; t.startup_advance_us=2000000; t.start_zone=3;
        t.pixel_grasp_enabled=true; return t;
    }()), task(tuning) {
        in.now_us=1000000;
        in.run=in.safety_ok=in.path_safe=in.retreat_safe=in.opponent_zone_clear=true;
        in.camera_pitch_stable=in.gripper_done=in.sent_velocity_valid=true;
        in.camera_pitch_cdeg=tuning.track_pitch_cdeg;
        task.update(in);
        task.startup_action_=open ? PushTask::StartupAction::OpenApproach
                                 : PushTask::StartupAction::SelectOrdinary;
        task.startup_pitch_=tuning.track_pitch_cdeg;
        task.frame_raised_=open ? 1 : 0;
        task.trip_context_.target_id_=103;
        task.trip_context_.label_="ordinary_supply";
    }
    PushOutput step(std::vector<SegDetection> scene) {
        in.now_us+=50000;
        in.pixel_grasp_view=true;
        in.gripper_feedback_open=task.frame_raised_;
        for(auto& d:scene) d.timestamp_us=in.now_us;
        applyStartupSight(in,scene,{1280,720},in.now_us,.5f,
                          task.startupSightTargetId(),task.startupOrdinaryOnly(),
                          task.startupSightRejectedId(in.now_us),task.startupSightReference());
        return task.update(in);
    }
};
static void ordinaryTargetRecovery() {
    auto failed=box(103,"ordinary_supply",0,.5f);
    failed.box={665,449,100,100}; // -75 px, no convergence; lower than the alternative.
    auto alternate=box(101,"ordinary_supply",0,.5f);
    alternate.box={590,250,100,100};
    for(bool open:{false,true}) {
        OrdinaryReselectRig rig(open);
        PushOutput out;
        bool released=false;
        for(int n=0;n<85;++n) {
            out=rig.step({failed,alternate});
            assert(out.run_status==RunStatus::Running);
            if(out.reason=="startup_alignment_reselect") { released=true; break; }
        }
        assert(released && rig.task.startupSightTargetId()==-1);
        assert(rig.task.startupSightReference().id==-1);
        assert(rig.task.startupSightRejectedId(rig.in.now_us)==103);
        assert(rig.task.startupSightRejectedId(rig.in.now_us+1000000)==-1);
        assert(!out.immediate_motion_stop && rig.task.frame_raised_==(open?1:0));
        out=rig.step({failed,alternate});
        assert(out.run_status==RunStatus::Running && rig.task.startupSightTargetId()==101);
        bool advanced=false;
        for(int n=0;n<35;++n) {
            out=rig.step({alternate});
            assert(out.run_status==RunStatus::Running);
            if(out.motion.vx_mps>0) { advanced=true; break; }
        }
        assert(advanced);
        rig.in.safety_ok=false;
        out=rig.step({alternate});
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
    }
    // Only a fresh, eligible ordinary alternative shortens the missing lock.
    for(int kind=0;kind<6;++kind) {
        OrdinaryReselectRig rig(true);
        rig.step({failed});
        if(kind==5) {
            rig.task.startup_rejected_id_=alternate.track_id;
            rig.task.startup_rejected_until_us_=rig.in.now_us+2000000;
        }
        auto other=alternate;
        if(kind==1) other.label="injured_person";
        if(kind==2) other.label="dangerous_object";
        if(kind==3) other.confidence=.1f;
        const auto start=rig.in.now_us+50000;
        PushOutput out;
        uint64_t released_at=0;
        for(int n=0;n<36;++n) {
            if(kind==4) {
                rig.in.now_us+=50000;
                other.timestamp_us=rig.in.now_us-250000;
                applyStartupSight(rig.in,{other},{1280,720},rig.in.now_us,.5f,
                    rig.task.startupSightTargetId(),true,-1,rig.task.startupSightReference());
                out=rig.task.update(rig.in);
            } else out=rig.step({other});
            assert(out.run_status==RunStatus::Running);
            if(out.reason=="startup_target_reselect") { released_at=rig.in.now_us; break; }
            assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        }
        assert(released_at==start+(kind==0?300000:kStartupTargetLossHoldUs));
        assert(rig.task.startupSightReference().id==-1);
        if(kind==0) {
            out=rig.step({alternate});
            assert(out.run_status==RunStatus::Running && rig.task.startupSightTargetId()==101);
        } else {
            out=rig.step({});
            assert(out.run_status==RunStatus::Running && out.reason=="startup_find_ordinary");
        }
    }
    // Same-object re-ID preserves the episode despite another visible ordinary.
    OrdinaryReselectRig rebound(true);
    rebound.step({failed,alternate});
    const auto since=rebound.task.startup_alignment_since_us_;
    failed.track_id=203;
    rebound.step({failed,alternate});
    assert(rebound.task.startupSightTargetId()==203);
    assert(rebound.task.startup_alignment_since_us_==since);
    std::cout<<"PASS ordinary stall reselects; missing lock uses 300ms only with a fresh ordinary alternative; recovery advances\n";
}


// Replay the 15:49 far-guide loss with another fresh cargo still visible.
static void startupFarGuideContinuity() {
    OrdinaryReselectRig rig(false);
    rig.tuning.far_pitch_cdeg=rig.task.t_.far_pitch_cdeg=-3000;
    rig.task.t_.approach_speed=.07f;
    rig.task.startup_action_=PushTask::StartupAction::Approach;
    rig.task.startup_pitch_=rig.tuning.far_pitch_cdeg;
    rig.task.trip_context_.target_id_=-1;
    auto guide=box(10,"ordinary_supply",0,.5f);
    guide.box={1200,300,70,128};
    auto other=box(23,"core_supply",0,.5f);other.box={500,280,80,100};
    const auto step=[&](std::vector<SegDetection> ds) {
        rig.in.now_us+=32000;
        rig.in.pixel_grasp_view=false;
        rig.in.gripper_feedback_open=0;
        rig.in.camera_pitch_cdeg=rig.tuning.far_pitch_cdeg;
        for(auto& d:ds)d.timestamp_us=rig.in.now_us;
        applyStartupSight(rig.in,ds,{1280,720},rig.in.now_us,.5f,
            rig.task.startupSightTargetId(),false,-1,rig.task.startupSightReference());
        return rig.task.update(rig.in);
    };
    auto initial=step({guide,other});
    std::cout<<"far initial reason="<<initial.reason<<" vx="<<initial.motion.vx_mps<<std::endl;
    assert(initial.motion.vx_mps>0);
    auto out=step({other});
    std::cout<<"far visible replacement reason="<<out.reason<<" vx="<<out.motion.vx_mps<<std::endl;
    assert(out.motion.vx_mps>0 && rig.in.target_id==23);
    guide.box={600,400,80,100};
    out=step({guide,other});assert(rig.in.target_id==23);
    out=step({});assert(out.motion.vx_mps==0);
    rig.in.safety_ok=false;out=step({other});
    assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
    std::cout<<"PASS far guide replacement does not insert a 1.5 second stop\n";
}
static void startupPileForwardJitter() {
    for(float sign:{-1.f,1.f}) {
        OrdinaryReselectRig rig(false);
        rig.task.t_.approach_speed=.07f;
        rig.task.startup_action_=PushTask::StartupAction::Approach;
        rig.task.trip_context_.target_id_=kStartupPileTargetId;
        rig.task.trip_context_.label_="startup_pile";
        rig.task.pixel_alignment_.stage=PushTask::PixelAlignment::Stage::Advance;
        rig.task.pixel_alignment_.target_id=kStartupPileTargetId;
        rig.task.pixel_alignment_.label="startup_pile";
        rig.task.pixel_alignment_.pitch=rig.tuning.track_pitch_cdeg;
        rig.in.pixel_grasp_view=rig.in.pixel_grasp_target_valid=true;
        rig.in.target_valid=rig.in.target_center_px_valid=true;
        rig.in.target_id=kStartupPileTargetId;rig.in.label="startup_pile";
        rig.in.startup_visible_classes=1;
        rig.in.gripper_feedback_open=0;
        const auto step=[&](float error) {
            rig.in.now_us+=32000;
            rig.in.pixel_grasp_error_px=sign*error;
            rig.in.target_center_px={640-rig.in.pixel_grasp_error_px,140};
            rig.in.startup_scene_us=rig.in.pixel_grasp_sample_us=rig.in.now_us;
            return rig.task.update(rig.in);
        };
        int stops=0;
        for(float e:{34.f,39.f,46.5f,46.5f,29.f,26.5f}) {
            auto out=step(e);stops+=out.motion.vx_mps==0;
            assert(std::abs(out.motion.wz_rps)<=.08001f);
            assert(out.motion.wz_rps*sign>=0);
        }
        std::cout<<"pile transient-jitter stops="<<stops<<std::endl;
        assert(stops==0);
        bool stopped=false;
        for(int n=0;n<6;++n)stopped|=step(rig.task.t_.pixel_align_approach_restart_px+6.5f).motion.vx_mps==0;
        assert(stopped);
        rig.task.pixel_alignment_.stage=PushTask::PixelAlignment::Stage::Advance;
        assert(step(2.f*rig.task.t_.pixel_align_approach_restart_px+10.f).motion.vx_mps==0);
        rig.task.pixel_alignment_.stage=PushTask::PixelAlignment::Stage::Advance;
        rig.in.startup_lower_corners_occupied=true;
        const auto out=step(20);
        assert(out.reason=="startup_pitch_turn_ready" && out.motion.vx_mps==0 && out.motion.wz_rps==0);
    }
    std::cout<<"PASS closed UI120 guide rolls through brief jitter and retains committed stops\n";
}

int main(int argc,char** argv) {
    const std::string mode=argc>1?argv[1]:"all";
    if(mode=="far-continuity" || mode=="startup" || mode=="all") startupFarGuideContinuity();
    if(mode=="pile-forward-jitter" || mode=="startup" || mode=="all") startupPileForwardJitter();
    if(mode=="ordinary-reselect" || mode=="startup" || mode=="all") ordinaryTargetRecovery();
    if(mode=="pile-pulse") startupPileSentPulse();
    if(mode=="pile-convergence") startupPileCorrectionConverges();
    if(mode=="startup" || mode=="all") { startupMemberChanges();startupPileSentPulse();startupPileCorrectionConverges();startupBudget(); }
    if(mode=="score" || mode=="all")scoreSelection();
    if(mode=="transport" || mode=="all") {firstTripPrePushQuantityReview();frontCategoryReview();categoryReviewLifecycle();inspectionOnlyBeforePush();}
    if(mode=="reject" || mode=="all") {avoidanceLifecycle();avoidanceUsesAllCurrentBoxes();avoidanceIgnoresHeadingChanges();cleanPushAndMissingEvidence();avoidanceFailuresAndPause();}
}
