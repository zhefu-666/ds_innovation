#include "../../support/push_task_sim.hpp"
#include "push_task_cases.hpp"

void rulesTests() {
    assert(targetKind("ordinary_supply") == TargetKind::ORDINARY &&
           targetKind("core_supply") == TargetKind::CORE);
    assert(targetKind("injured_person") == TargetKind::INJURED &&
           targetKind("dangerous_object") == TargetKind::DANGEROUS);
    assert(targetKind("") == TargetKind::UNKNOWN && targetKind("normal") == TargetKind::UNKNOWN);
    assert(targetSelectable("ordinary_supply", false) && !targetSelectable("core_supply", false));
    assert(!targetSelectable("injured_person", false) && targetSelectable("injured_person", true));
    assert(targetSelectable("core_supply", true) && !targetSelectable("dangerous_object", true));
    assert(!targetSelectable("unknown", true));
    Inventory a;
    a.add("ordinary_supply");
    a.add("core_supply");
    a.add("weird");
    assert(a.ordinary == 1 && a.core == 1 && a.unknown == 1 && a.supplies() == 2 && a.total() == 3);
    assert(checkTrip({}, true) == RuleVerdict::EMPTY);
    assert(checkTrip(inv(1), false) == RuleVerdict::OK &&
           checkTrip(inv(2), false) == RuleVerdict::OK);
    assert(checkTrip(inv(3), false) == RuleVerdict::TOO_MANY_SUPPLIES);
    assert(checkTrip(inv(4), true) == RuleVerdict::TOO_MANY_SUPPLIES);
    assert(checkTrip(inv(2, 2), true) == RuleVerdict::TOO_MANY_SUPPLIES);
    assert(checkTrip(inv(1, 1), true) == RuleVerdict::OK);
    assert(checkTrip(inv(2, 1), true) == RuleVerdict::TOO_MANY_SUPPLIES);
    assert(checkTrip(inv(1, 1), false) == RuleVerdict::CORE_BEFORE_FIRST);
    assert(checkTrip(inv(0, 0, 1), false) == RuleVerdict::INJURED_BEFORE_FIRST);
    assert(checkTrip(inv(0, 0, 1), true) == RuleVerdict::OK);
    assert(checkTrip(inv(1, 0, 1), true) == RuleVerdict::INJURED_NOT_ALONE);
    assert(checkTrip(inv(0, 0, 2), true) == RuleVerdict::INJURED_NOT_ALONE);
    assert(checkTrip(inv(1, 0, 0, 1), true) == RuleVerdict::DANGEROUS);
    assert(checkTrip(inv(1, 0, 0, 0, 1), true) == RuleVerdict::UNKNOWN_OBJECT);
    // Corridor: the primary must be inside, and before the first delivery (or for an
    // injured person) nothing may be hidden behind it.
    assert(checkCorridor(inv(1), false, true, "ordinary_supply", false) == RuleVerdict::INCOMPLETE);
    assert(checkCorridor(inv(0, 1), true, true, "ordinary_supply", true) ==
           RuleVerdict::INCOMPLETE);
    assert(checkCorridor(inv(1), true, false, "ordinary_supply", false) == RuleVerdict::OCCLUDED);
    assert(checkCorridor(inv(1, 1), true, false, "core_supply", true) == RuleVerdict::OK);
    assert(checkCorridor(inv(0, 0, 1), true, false, "injured_person", true) ==
           RuleVerdict::OCCLUDED);
    assert(checkCorridor(inv(1, 1), true, true, "ordinary_supply", false) ==
           RuleVerdict::CORE_BEFORE_FIRST);
    assert(std::string(requiredHalf(inv(0, 0, 1))) == "injured" &&
           std::string(requiredHalf(inv(2, 1))) == "supply");
    assert(std::string(verdictName(RuleVerdict::TOO_MANY_SUPPLIES)) == "too_many_supplies");
}

void captureTests() {
    { // Pure observations: no devices. Exercise the approved centering controller.
        TaskTuning t;
        t.pixel_grasp_enabled = true;
        t.require_multi_view = false;
        t.startup_advance_us = 0;
        t.track_pitch_cdeg = t.intermediate_pitch_cdeg = 0;
        t.near_pitch_cdeg = 2000;
        PushTask task(t);
        PushObservation in;
        in.run = in.safety_ok = in.path_safe = in.retreat_safe = in.opponent_zone_clear = true;
        in.target_valid = in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
        in.camera_pitch_stable = true;
        in.camera_pitch_cdeg = 0;
        in.target_id = 7;
        in.label = "ordinary_supply";
        in.sent_velocity_valid = true;
        in.gripper_done = true;
        in.gripper_feedback_open = 1;
        in.now_us = 1000000;
        task.update(in);
        task.enter(PushState::MID_APPROACH, in.now_us, "alignment_regression");
        PushOutput out;
        auto step = [&](float error, bool new_image = true, uint64_t image_age = 0) {
            in.now_us += 50000;
            if (new_image) in.pixel_grasp_sample_us = in.now_us - image_age;
            in.pixel_grasp_error_px = error;
            in.pixel_grasp_steering = error / 640.f;
            in.pixel_grasp_aligned = std::abs(error) <= 12.f;
            out = task.update(in);
            return out;
        };
        auto settle = [&] {
            for (int i = 0; i < 20 && out.motion.vx_mps == 0; ++i) step(0);
            assert(out.motion.vx_mps > 0 && out.pixel_align_state == "advance");
        };
        step(80); assert(std::abs(out.motion.wz_rps - .20f) < 1e-6f);
        step(50); assert(std::abs(out.motion.wz_rps - .20f) < 1e-6f);
        step(30); assert(std::abs(out.motion.wz_rps - .12f) < 1e-6f);
        // Crossing the centre must brake, including the still-old-direction UART command.
        in.sent_wz_rps = .12f;
        step(-50); assert(stopped(out.motion) && out.reason == "pixel_align_brake");
        step(-50); assert(stopped(out.motion));
        in.sent_wz_rps = 0;
        step(-50, false); // first observation of zero sent speed
        step(-50, true, 75000); // new image, but captured before the stop observation
        assert(stopped(out.motion) && out.pixel_align_stable_frames == 0);
        step(-50);
        assert(out.pixel_align_stable_frames == 1);
        for (int i=0; i<3; ++i) {
            step(-50, false);
            assert(stopped(out.motion) && out.pixel_align_stable_frames == 1);
        }
        // Continued image motion resets the brake window even with zero sent speed.
        for (float error : {-56.f,-62.f,-68.f}) {
            step(error);
            assert(stopped(out.motion) && out.pixel_align_stable_frames == 1);
        }
        for (int i=0; i<3; ++i) { step(-68); assert(stopped(out.motion)); }
        step(-68); assert(std::abs(out.motion.wz_rps + .20f) < 1e-6f);
        // Zero image error cannot authorize Advance until both sent axes have stopped.
        in.sent_vx_mps = .04f;
        step(0);
        for (int i=0; i<7; ++i) { step(0); assert(stopped(out.motion)); }
        in.sent_vx_mps = 0; in.sent_wz_rps = .03f;
        for (int i=0; i<7; ++i) { step(0); assert(stopped(out.motion)); }
        in.sent_wz_rps = 0;
        for (int i=0; i<5; ++i) step(0); // stationary brake window, then Settle
        assert(out.pixel_align_state == "settle");
        step(0);
        const auto frames = out.pixel_align_stable_frames;
        assert(frames == 1);
        for (int i=0; i<3; ++i) {
            step(0, false);
            assert(stopped(out.motion) && out.pixel_align_stable_frames == frames);
        }
        step(0); // 200 ms elapsed, but only two distinct stable images
        assert(stopped(out.motion) && out.pixel_align_stable_frames == 2);
        step(0); assert(out.motion.vx_mps > 0);
        // UI120 keeps advancing within its coarse hysteresis, including +/-40 px.
        for (float error : {13.f,-18.f,24.f,-24.f,35.f,-40.f}) {
            step(error); assert(out.motion.vx_mps > 0 && out.motion.wz_rps == 0);
        }
        step(41); assert(stopped(out.motion) && out.reason == "pixel_align_brake");
        step(0); settle();
        // Initial alignment need not reach +/-12. Once inside +/-24, the entire
        // brake/settle sequence tolerates jitter up to +/-40 without chasing it.
        in.target_id = 9;
        step(23); assert(stopped(out.motion));
        step(23);
        for (int i=0; i<4; ++i) step(26);
        assert(out.pixel_align_state == "settle");
        for (float error : {25.f,30.f,39.f,26.f}) {
            step(error); assert(stopped(out.motion) && out.pixel_align_state == "settle");
        }
        step(28); assert(out.motion.vx_mps > 0);
        // Being outside the entry band from the start cannot authorize forward motion.
        in.target_id = 10;
        for (int i=0; i<20; ++i) { step(30); assert(out.motion.vx_mps == 0); }
        step(0); settle();
        // Stale/missing images discard confirmation and require a fresh stable window.
        in.pixel_grasp_target_valid = false;
        step(0); assert(stopped(out.motion));
        in.pixel_grasp_target_valid = true;
        step(0); assert(stopped(out.motion)); settle();
        step(0, false); step(0, false); step(0, false); step(0, false); step(0, false);
        assert(stopped(out.motion));
        step(0); assert(stopped(out.motion)); settle();
        in.target_id = 8;
        step(0); assert(stopped(out.motion)); settle();
        in.label = "core_supply"; task.first_ = true;
        step(0); assert(stopped(out.motion)); settle();
        in.camera_pitch_stable = false;
        step(0); assert(stopped(out.motion));
        in.camera_pitch_stable = true;
        step(0); assert(stopped(out.motion)); settle();
        // A stop/resume must not reuse an old alignment confirmation.
        in.run = false; step(0); assert(stopped(out.motion));
        in.run = true;
        task.enter(PushState::APPROACH, in.now_us, "resume_regression");
        step(0); assert(stopped(out.motion)); settle();
        // Near-view entry resets confirmation and keeps class-specific close checks strict.
        in.camera_pitch_cdeg = 2000;
        in.pixel_grasp_near_view = true;
        in.target_grasp_range_valid = in.target_grasp_range_lateral = true;
        in.target_grasp_range_inside = true;
        in.corridor.core = 1;
        in.corridor_complete = in.corridor_occlusion_free = true;
        step(0); assert(stopped(out.motion));
        task.enter(PushState::RUSH, in.now_us, "near_regression");
        for(int i=0;i<20 && out.motion.vx_mps==0;++i) step(0);
        assert(out.motion.vx_mps>0);
        step(18); assert(out.motion.vx_mps>0); // forward hysteresis only
        in.pixel_grasp_ready = true;
        in.sent_vx_mps = .10f;
        step(18); assert(stopped(out.motion) && out.state==PushState::RUSH);
        in.sent_vx_mps=0;
        step(0);
        for(int i=0;i<20 && out.state!=PushState::LOWER_FRAME;++i) step(0);
        assert(out.state==PushState::LOWER_FRAME);
        auto veto=task;
        auto bad=in; bad.now_us+=50000; bad.pixel_grasp_aligned=false;
        assert(veto.update(bad).state==PushState::CAPTURE_FAIL);
    }

    // The complete image route works at UI120/UI140 with absent metric geometry.
    for (const bool demo : {false, true})
    for (const std::string label : {"ordinary_supply", "injured_person", "core_supply"}) {
        TaskTuning t;
        t.pixel_grasp_enabled = true;
        t.demo_capture_motion = demo;
        t.frame_open_default = true;
        t.require_multi_view = false;
        t.startup_advance_us = 0;
        t.intermediate_pitch_cdeg = t.track_pitch_cdeg = 0;
        t.near_pitch_cdeg = 2000;
        PushTask task(t);
        PushObservation in;
        in.run = in.safety_ok = in.path_safe = in.retreat_safe = in.opponent_zone_clear = true;
        in.target_valid = true;
        in.target_id = 7; in.label = label;
        in.pixel_bearing_valid = true;
        in.pixel_bearing = .2f;
        in.geometry_valid = in.target_region_valid = in.target_center_valid = false;
        in.distance_m = in.heading_error = std::numeric_limits<float>::quiet_NaN();
        in.camera_pitch_cdeg = 0; in.camera_pitch_stable = true;
        in.now_us = 1000000;
        task.update(in);
        task.first_ = label != "ordinary_supply";
        task.enter(PushState::MID_APPROACH, in.now_us, "pixel_test_start");
        CaptureConfig config;
        config.pixel_grasp_enabled = true;
        config.grasp_center_ranges = {
            {"ordinary_supply",2000,1280,720,{0,226,1279,489},{647.5f,337},false},
            {"injured_person",2000,1280,720,{395,236,906,479},{647.5f,337}},
            {"core_supply",2000,1280,720,{268.5f,236,1017,555},{647.5f,337}}};
        CaptureMonitor monitor(config);
        SegDetection d;
        d.track_id=7; d.label=label; d.confidence=.95f;
        d.box={500,200,100,100}; // centre Y250, bottom Y300; still ahead of Y337
        PushOutput out;
        auto step = [&](bool fresh = true) {
            in.now_us += 50000;
            d.timestamp_us = fresh ? in.now_us : in.now_us - 300000;
            monitor.update(in, {d}, in.now_us, label);
            out = task.update(in);
            assert(!in.geometry_valid && !in.target_center_valid && !in.target_region_valid);
            return out;
        };
        step();
        assert(out.reason == "pixel_center_align" && out.motion.wz_rps == .2f && out.motion.vx_mps == 0);
        // Both directions preserve proportional speed, with a zero-command braking tick
        // before reversing; no legacy floor, boost or one-second escalation.
        for (int i = 0; i < 30; ++i) {
            d.box.x = i < 10 ? 500 : (i < 20 ? 620 : 560);
            step();
            assert(out.motion.vx_mps == 0);
            if (out.reason == "pixel_center_align") {
                const float expected = std::clamp(.004f * in.pixel_grasp_error_px, -.2f, .2f);
                assert(std::abs(out.motion.wz_rps - expected) < 1e-6f);
            } else assert(out.reason == "pixel_align_brake" && stopped(out.motion));
        }
        assert(!in.target_grasp_range_applicable && !in.pixel_grasp_ready);
        in.pixel_bearing = 0;
        d.box.x = 598;
        in.gripper_done = true; in.gripper_feedback_open = 1;
        d.box.y = 15; // Recorded failure: aligned but at the top of UI120.
        for (int i=0;i<20;++i) step();
        assert(out.reason == "pixel_center_approach_120" && out.motion.vx_mps > 0 &&
               out.motion.camera_pitch_cdeg == 0 && !in.pixel_view_handoff_ready);
        step(false);
        assert(stopped(out.motion) && out.reason == "pixel_target_wait");
        d.box.y = 450; // Bottom is already below 540, but centre is only 500.
        for (int i=0;i<20;++i) step();
        assert(out.motion.vx_mps > 0 && out.motion.camera_pitch_cdeg == 0);
        d.box.y = 490; // Centre reaches UI120 view handoff Y540.
        step();
        assert(out.reason == "pixel_center_switch_to_140" && stopped(out.motion) &&
               out.motion.camera_pitch_cdeg == 2000);
        in.camera_pitch_stable = false;
        step();
        assert(stopped(out.motion) && out.reason == "camera_pitch_wait");
        in.camera_pitch_cdeg = 2000; in.camera_pitch_stable = true;
        d.box.y = 200; // Same object recovered in the new view; centre Y250.
        for (int i=0;i<20 && out.state!=PushState::PREPARE;++i) step();
        assert(out.state == PushState::PREPARE && out.motion.gripper_offset == 0);
        in.gripper_done = true; in.gripper_feedback_open = 1;
        for (int i=0;i<20 && out.state!=PushState::RUSH;++i) step();
        assert(out.state == PushState::RUSH);
        // Near-view alignment also bypasses the legacy turn floor and boost.
        for (int i = 0; i < 30; ++i) {
            d.box.x = i < 15 ? 560 : 630;
            step();
            assert(out.motion.vx_mps == 0);
            if (out.reason == "pixel_center_align") {
                const float expected = std::clamp(.004f * in.pixel_grasp_error_px, -.2f, .2f);
                assert(std::abs(out.motion.wz_rps - expected) < 1e-6f);
            } else assert(out.reason == "pixel_align_brake" && stopped(out.motion));
        }
        in.directional_clearance_valid = true;
        in.turn_safe = false;
        step();
        assert(stopped(out.motion) && out.reason == "turn_sweep_blocked");
        in.directional_clearance_valid = false;
        d.box.x = 598;
        for (int i=0;i<20;++i) step();
        assert(out.reason == "pixel_center_approach" && out.motion.vx_mps > 0 && out.motion.wz_rps == 0);
        step(false);
        assert(out.state == PushState::RUSH && stopped(out.motion) && !in.pixel_grasp_ready);
        // A bottom below the stop line is not a centre crossing.
        d.box.y = 250; // centre Y300, bottom Y350
        for (int i=0;i<20;++i) step();
        assert(out.state == PushState::RUSH && out.motion.vx_mps > 0);
        d.box.y = 287; // centre Y337, exact stop line
        for (int i=0;i<20 && out.state!=PushState::LOWER_FRAME;++i) step();
        assert(out.state == PushState::LOWER_FRAME && stopped(out.motion));
        // A fresh pre-close range failure must veto issuing the close command.
        auto veto = task;
        auto bad = in; bad.now_us += 50000; bad.pixel_grasp_ready = false;
        const auto rejected = veto.update(bad);
        assert(rejected.state == PushState::CAPTURE_FAIL && rejected.motion.gripper_offset == 0);
        step();
        assert(out.motion.gripper_offset == 20);
        in.gripper_feedback_open = 0; in.gripper_closed_observed = true;
        for (int i=0;i<30 && out.state!=PushState::CARRY;++i) step();
        assert(out.state == PushState::CARRY && out.capture_evidence == CaptureEvidence::Verified);
        // Moving camera and stale boxes cannot count as held or trigger capture.
        in.camera_pitch_stable = false;
        monitor.update(in,{d},in.now_us,label);
        assert(!in.captured && !in.held_complete && !in.pixel_grasp_ready);
        in.camera_pitch_stable = true;
        d.timestamp_us = in.now_us - 300000;
        monitor.update(in,{d},in.now_us,label);
        assert(!in.captured && !in.pixel_grasp_target_valid);
    }
    { // Reproduce the real SCAN lock at UI90 with an unprojectable lower-edge box.
        TaskTuning t;
        t.pixel_grasp_enabled = true;
        t.frame_open_default = true;
        t.far_pitch_cdeg = -3000;
        t.intermediate_pitch_cdeg = t.track_pitch_cdeg = 0;
        t.near_pitch_cdeg = 2000;
        PushTask task(t);
        PushObservation in;
        in.now_us = 1000000;
        in.run = in.safety_ok = in.path_safe = in.retreat_safe = in.opponent_zone_clear = true;
        in.camera_pitch_cdeg = -3000; in.camera_pitch_stable = true;
        in.target_valid = true; in.target_id = 2; in.label = "ordinary_supply";
        in.geometry_valid = in.target_region_valid = false;
        task.update(in);
        task.enter(PushState::SCAN, in.now_us, "recorded_scan");
        CaptureConfig config;
        config.pixel_grasp_enabled = true;
        config.grasp_center_ranges = {
            {"ordinary_supply",2000,1280,720,{0,226,1279,489},{647.5f,337},false}};
        CaptureMonitor monitor(config);
        SegDetection d;
        d.track_id=2; d.label=in.label; d.confidence=.84f; d.box={607,607,122,113};
        in.now_us += 50000; d.timestamp_us=in.now_us;
        monitor.update(in,{d},in.now_us);
        assert(in.target_center_px_valid && in.pixel_view_handoff_ready &&
               in.target_center_px == cv::Point2f(668,663.5f) && !in.pixel_grasp_view);
        auto rejected = [&](PushObservation bad) {
            auto copy=task;
            const auto out=copy.update(bad);
            assert(out.motion.camera_pitch_cdeg == -3000 && out.motion.vx_mps == 0);
        };
        auto bad=in; bad.target_in_zone=true; rejected(bad);
        bad=in; bad.camera_pitch_stable=false; rejected(bad);
        bad=in; bad.label="core_supply"; rejected(bad); // First-trip rule still applies.
        bad=in; bad.target_is_search_cue=true; rejected(bad);
        auto stale=in;
        monitor.update(stale,{d},in.now_us+300000);
        assert(!stale.target_center_px_valid && !stale.pixel_view_handoff_ready);
        rejected(stale);
        const auto out=task.update(in);
        assert(out.state == PushState::APPROACH && stopped(out.motion) &&
               out.reason == "pixel_center_switch_to_120" && out.motion.camera_pitch_cdeg == 0);
        assert(!in.geometry_valid && !in.target_region_valid);
        // A normal far lock retains UI90 and its metric approach until the centre is low.
        task.enter(PushState::SCAN,in.now_us,"reset_far");
        task.pitch_cmd_ = -3000;
        task.enter(PushState::APPROACH,in.now_us,"far_lock");
        task.trip_context_.target_id_=task.trip_context_.capture_target_id_=2;
        task.trip_context_.label_=in.label;
        d.box={590,200,100,100};
        in.now_us+=50000; d.timestamp_us=in.now_us;
        in.geometry_valid=in.target_region_valid=true;
        in.distance_m=1; in.heading_error=0;
        monitor.update(in,{d},in.now_us);
        auto far=task.update(in);
        assert(far.motion.camera_pitch_cdeg == -3000 && far.motion.vx_mps > 0 &&
               far.reason == "pixel_far_approach");
        // A re-ID at the same position must resume without consuming the 15 s budget.
        const auto far_task = task;
        const auto far_input = in;
        in.target_id = d.track_id = 20;
        in.distance_m = .82f; in.heading_error = .04f;
        in.now_us += 50000; d.timestamp_us = in.now_us;
        monitor.update(in,{d},in.now_us);
        far = task.update(in);
        assert(far.state == PushState::APPROACH && stopped(far.motion) &&
               far.reason == "pixel_far_id_rebound_same_object" &&
               far.capture_target_id == 20);
        in.now_us += 50000; d.timestamp_us = in.now_us;
        monitor.update(in,{d},in.now_us);
        far = task.update(in);
        assert(far.reason == "pixel_far_approach" && far.motion.vx_mps > 0);
        // A further re-ID uses the refreshed target history, not the original ID.
        in.target_id = d.track_id = 21; in.now_us += 50000; d.timestamp_us = in.now_us;
        monitor.update(in,{d},in.now_us);
        far = task.update(in);
        assert(stopped(far.motion) && far.capture_target_id == 21 &&
               far.reason == "pixel_far_id_rebound_same_object");
        // Invalid, unrelated or blacklisted detections cannot replace the old target.
        for (int mode = 0; mode < 11; ++mode) {
            auto copy = far_task;
            auto bad = far_input;
            bad.target_id = 30;
            if (mode == 0) bad.target_valid = false;
            if (mode == 1) bad.geometry_valid = false;
            if (mode == 2) bad.target_region_valid = false;
            if (mode == 3) bad.target_in_zone = true;
            if (mode == 4) bad.label = "core_supply";
            if (mode == 5) bad.label = "dangerous_object";
            if (mode == 6) bad.target_is_search_cue = true;
            if (mode == 7) bad.distance_m = 2.f;
            if (mode == 8) bad.heading_error = .5f;
            if (mode == 9) bad.target_center_px_valid = false;
            if (mode == 10) copy.rejected_[30] = bad.now_us + 1000000;
            for (int i = 0; i <= t.hold_grace_frames; ++i) {
                bad.now_us += 50000;
                const auto waiting = copy.update(bad);
                assert(stopped(waiting.motion) && waiting.capture_target_id != 30);
                if (i < t.hold_grace_frames)
                    assert(waiting.state == PushState::APPROACH &&
                           waiting.reason == "pixel_far_target_wait");
                else
                    assert(waiting.state == PushState::SCAN &&
                           waiting.reason == "pixel_far_target_lost");
            }
        }
        // Rebinding does not bypass the existing forward-clearance checks.
        for (bool blocked_path : {false, true}) {
            auto copy = far_task;
            auto blocked = far_input;
            blocked.target_id = 40;
            blocked.path_safe = !blocked_path;
            blocked.opponent_zone_clear = blocked_path;
            for (int i = 0; i < 2; ++i) {
                blocked.now_us += 50000;
                assert(stopped(copy.update(blocked).motion));
            }
        }
        task = far_task; in = far_input; d.track_id = in.target_id;
        in.geometry_valid=false; in.now_us+=50000; d.timestamp_us=in.now_us;
        monitor.update(in,{d},in.now_us);
        far=task.update(in);
        assert(stopped(far.motion) && far.reason == "pixel_far_target_wait");
    }
    // The bottom can cross the close line while the box centre is still ahead.
    // Both normal and demo capture must wait for the centre, including precise rush.
    for (bool demo : {false, true})
        for (bool precise : {false, true}) {
            TaskTuning tuning;
            tuning.field_rush = demo;
            tuning.demo_capture_motion = demo;
            tuning.precise_rush = precise;
            Sim sim(tuning);
            sim.toRush();
            const float trigger = demo ? tuning.assume_rush_trigger_m : tuning.grasp_trigger_y_m;
            sim.in.distance_m = trigger - .03f;
            sim.in.target_center_valid = true;
            sim.in.target_center_body = {0.f, trigger + .05f};
            sim.tick();
            assert(sim.out.state == PushState::RUSH && sim.out.motion.vx_mps > 0);
            // Conversely, a distant bottom-derived range must not postpone centre-based closure.
            sim.in.distance_m = trigger + .08f;
            sim.in.target_center_body.y = trigger;
            sim.tick();
            assert(sim.out.state == PushState::LOWER_FRAME &&
                   sim.out.reason == "frame_enclosure_position" && stopped(sim.out.motion));
        }
    // Class-specific limits use the box centre, including half-pixel boundaries.
    CaptureConfig center_config;
    center_config.grasp_center_ranges = {
        {"injured_person", 2000, 1280, 720, {395, 236, 906, 479}},
        {"core_supply", 2000, 1280, 720, {268.5f, 236, 1017, 555}}};
    const std::vector<std::pair<std::string, cv::Rect>> samples = {
        {"injured_person", {495,104,386,264}}, {"injured_person", {440,357,456,244}},
        {"injured_person", {764,139,284,416}}, {"injured_person", {263,145,264,405}},
        {"core_supply", {597,165,142,142}}, {"core_supply", {182,281,173,143}},
        {"core_supply", {933,354,168,148}}, {"core_supply", {571,509,113,92}}};
    CaptureMonitor center_monitor(center_config);
    PushObservation center_input;
    center_input.target_valid = true;
    center_input.target_id = 9;
    center_input.camera_pitch_cdeg = 2000;
    center_input.camera_pitch_stable = true;
    SegDetection sample;
    sample.track_id = 9;
    sample.confidence = .9f;
    sample.timestamp_us = 1000000;
    for (const auto& entry : samples) {
        center_input.label = sample.label = entry.first;
        sample.box = entry.second;
        center_monitor.update(center_input, {sample}, sample.timestamp_us);
        assert(center_input.target_grasp_range_applicable && center_input.target_grasp_range_valid &&
               center_input.target_grasp_range_inside);
    }
    // Core's left sample shifted one original pixel is outside; its box overlaps the region.
    center_input.label = sample.label = "core_supply";
    sample.box = {181,281,173,143};
    center_monitor.update(center_input, {sample}, sample.timestamp_us);
    assert(center_input.target_grasp_range_valid && !center_input.target_grasp_range_inside &&
           !center_input.target_grasp_range_lateral);
    center_monitor.update(center_input, {sample}, sample.timestamp_us + 300000);
    assert(center_input.target_grasp_range_applicable && !center_input.target_grasp_range_valid);
    center_input.label.clear();
    center_input.target_valid = false;
    center_monitor.update(center_input, {}, sample.timestamp_us, "core_supply");
    assert(center_input.target_grasp_range_applicable && !center_input.target_grasp_range_valid);
    center_input.label = "core_supply";
    center_input.target_valid = true;
    center_input.camera_pitch_stable = false;
    center_monitor.update(center_input, {sample}, sample.timestamp_us);
    assert(center_input.target_grasp_range_applicable && !center_input.target_grasp_range_valid);
    center_input.camera_pitch_stable = true;
    center_input.camera_pitch_cdeg = 0;
    center_monitor.update(center_input, {sample}, sample.timestamp_us);
    assert(!center_input.target_grasp_range_applicable); // no reuse at another camera view
    center_input.camera_pitch_cdeg = 2000;
    center_input.label = sample.label = "ordinary_supply";
    center_monitor.update(center_input, {sample}, sample.timestamp_us);
    assert(!center_input.target_grasp_range_applicable); // ordinary calibration is unchanged
    // Feed both classes through the actual capture state machine and monitor.
    for (const char* label : {"injured_person", "core_supply"}) {
        for (int scenario = 0; scenario < 5; ++scenario) {
            Sim sim;
            sim.toRush();
            sim.task.first_ = true;
            sim.task.trip_context_.label_ = sim.in.label = label;
            sim.in.corridor = {};
            sim.in.corridor.add(label);
            sim.in.target_center_valid = true;
            sim.in.target_center_body = {0, sim.tune.grasp_trigger_y_m};
            sample.track_id = sim.in.target_id;
            sample.label = label;
            sample.box = scenario == 0 ? cv::Rect(600,300,100,100) :
                         scenario == 1 ? cv::Rect(0,300,100,100) :
                         scenario == 2 ? cv::Rect(600,560,100,100) :
                         scenario == 3 ? cv::Rect(600,100,100,100) : cv::Rect(600,300,100,100);
            PushObservation range_input;
            range_input.target_valid = true;
            range_input.target_id = sample.track_id;
            range_input.label = label;
            range_input.camera_pitch_cdeg = 2000;
            range_input.camera_pitch_stable = true;
            center_monitor.update(range_input, {sample}, sample.timestamp_us + (scenario == 4 ? 300000 : 0));
            sim.in.target_grasp_range_applicable = range_input.target_grasp_range_applicable;
            sim.in.target_grasp_range_valid = range_input.target_grasp_range_valid;
            sim.in.target_grasp_range_inside = range_input.target_grasp_range_inside;
            sim.in.target_grasp_range_lateral = range_input.target_grasp_range_lateral;
            sim.in.target_grasp_range_past_rear = range_input.target_grasp_range_past_rear;
            sim.tick();
            if (scenario == 0) {
                assert(sim.out.state == PushState::LOWER_FRAME);
                // Losing the centre after arrival but before command must not close.
                sim.in.target_grasp_range_valid = false;
                for (int i = 0; i < 30 && sim.out.state == PushState::LOWER_FRAME; ++i)
                    sim.tick();
                assert(sim.out.state == PushState::CAPTURE_FAIL && sim.task.frame_raised_ == 1);
            } else if (scenario == 3)
                assert(sim.out.state == PushState::RUSH); // centre has not entered front boundary
            else
                assert(sim.out.state == PushState::CAPTURE_FAIL && stopped(sim.out.motion));
        }
    }
    CaptureConfig old_view;
    old_view.holding = {{3500, {540, 570, 820, 690}}};
    CaptureMonitor monitor(old_view);
    PushObservation in;
    in.camera_pitch_cdeg = 3500;
    in.camera_pitch_stable = true; // NEAR, settled
    const auto box = [](int id, const char* label, cv::Rect r, uint64_t t) {
        SegDetection d;
        d.track_id = id;
        d.label = label;
        d.confidence = .9f;
        d.box = r;
        d.timestamp_us = t;
        return d;
    };
    uint64_t t = 1000000;
    // Held only after the box stays inside the closed frame for follow_frames.
    for (int i = 0; i < 2; ++i) {
        t += 50000;
        monitor.update(in, {box(1, "ordinary_supply", {600, 600, 40, 40}, t)}, t);
        assert(!in.captured);
    }
    t += 50000;
    monitor.update(in, {box(1, "ordinary_supply", {602, 600, 40, 40}, t)}, t);
    assert(in.hold_observable && in.captured && in.held_complete && in.held == inv(1));
    // Zone boxes overlapping the holding view are surfaces, never cargo or occluders.
    t += 50000;
    monitor.update(in,
                   {box(1, "ordinary_supply", {602, 600, 40, 40}, t),
                    box(2, "blue_safe_zone", {500, 500, 400, 200}, t),
                    box(3, "red_safe_zone", {580, 580, 80, 80}, t)},
                   t);
    assert(in.captured && in.held_complete && in.held == inv(1));
    // FAR, or a camera still moving: the region is out of view, nothing is claimed either way.
    auto far = in;
    far.camera_pitch_cdeg = 1200;
    CaptureMonitor other(old_view);
    for (int i = 0; i < 4; ++i) {
        other.update(far, {box(1, "ordinary_supply", {602, 600, 40, 40}, t)}, t);
        assert(!far.hold_observable && !far.captured);
    }
    auto moving = in;
    moving.camera_pitch_stable = false;
    other.update(moving, {box(1, "ordinary_supply", {602, 600, 40, 40}, t)}, t);
    assert(!moving.hold_observable && !moving.captured);
    // A pitch change restarts the follow history: positions at another pitch are not comparable.
    t += 50000;
    far.camera_pitch_cdeg = 3500;
    other.update(far, {box(1, "ordinary_supply", {602, 600, 40, 40}, t)}, t);
    assert(far.hold_observable && !far.captured);
    // A box straddling the frame edge makes the held set incomplete.
    t += 50000;
    monitor.update(in,
                   {box(1, "ordinary_supply", {602, 600, 40, 40}, t),
                    box(2, "core_supply", {800, 600, 60, 40}, t)},
                   t);
    assert(!in.held_complete);
    // Stale or low-confidence boxes are never held as known targets.
    t += 50000;
    monitor.update(in, {box(1, "ordinary_supply", {602, 600, 40, 40}, t - 400000)}, t);
    assert(!in.captured && in.held.total() == 0);
    // Corridor: objects in the swept width are inventoried; one without ground position spoils
    // completeness.
    CaptureMonitor corridor(old_view);
    in = {};
    in.target_valid = in.geometry_valid = true;
    in.distance_m = .3f;
    in.heading_error = 0;
    in.camera_pitch_cdeg = 3500;
    in.camera_pitch_stable = true;
    auto a = box(5, "ordinary_supply", {600, 300, 40, 40}, t);
    a.ground_position_valid = true;
    a.body_xy_m = {0, .3f};
    auto b = box(6, "core_supply", {300, 300, 40, 40}, t);
    b.ground_position_valid = true;
    b.body_xy_m = {-.4f, .3f};
    corridor.update(in, {a, b}, t);
    assert(in.corridor_complete && in.corridor_occlusion_free && in.corridor == inv(1));
    auto hidden = box(7, "core_supply", {610, 310, 40, 40}, t);
    hidden.ground_position_valid = true;
    hidden.body_xy_m = {.02f, .35f};
    corridor.update(in, {a, hidden}, t);
    assert(in.corridor_complete && !in.corridor_occlusion_free && in.corridor == inv(1, 1));
    // An overlapping box whose ground position is clearly beside the corridor is a neighbour, not
    // occlusion.
    auto neighbour = box(9, "core_supply", {610, 310, 40, 40}, t);
    neighbour.ground_position_valid = true;
    neighbour.body_xy_m = {.20f, .3f};
    corridor.update(in, {a, neighbour}, t);
    assert(in.corridor_complete && in.corridor_occlusion_free && in.corridor == inv(1));
    // Two overlapping ordinary supplies in the corridor are a legal pair, not occlusion; a core
    // behind one still is.
    auto pair2 = box(10, "ordinary_supply", {615, 305, 40, 40}, t);
    pair2.ground_position_valid = true;
    pair2.body_xy_m = {.03f, .4f};
    corridor.update(in, {a, pair2}, t);
    assert(in.corridor_complete && in.corridor_occlusion_free && in.corridor == inv(2));
    corridor.update(in, {a, pair2, hidden}, t);
    assert(!in.corridor_occlusion_free);
    // An ordinary block cut by the image bottom no longer spoils the corridor; a core cut by it
    // does.
    auto cut = box(11, "ordinary_supply", {600, 690, 40, 40}, t);
    cut.ground_position_valid = true;
    cut.body_xy_m = {0, .3f};
    corridor.update(in, {cut}, t);
    assert(in.corridor_occlusion_free && in.corridor == inv(1));
    cut.label = "core_supply";
    corridor.update(in, {cut}, t);
    assert(!in.corridor_occlusion_free);
    auto floating = box(8, "ordinary_supply", {900, 300, 40, 40}, t);
    corridor.update(in, {a, floating}, t);
    assert(!in.corridor_complete && !in.corridor_occlusion_free);
    in.camera_pitch_stable = false;
    corridor.update(in, {a}, t);
    assert(!in.corridor_complete &&
           in.corridor.total() == 0); // no inventory while the camera moves
}

void holding40StaticReviewTests() {
    CaptureConfig config;
    config.holding = {{4000, {440, 210, 930, 720}, 440}};
    const auto detection =
        [](int id, const char* label, float confidence, cv::Rect rect, uint64_t when) {
            SegDetection d;
            d.track_id = id;
            d.label = label;
            d.confidence = confidence;
            d.box = rect;
            d.timestamp_us = when;
            return d;
        };
    const auto check = [&](const std::vector<SegDetection>& sample, const Inventory& expected) {
        CaptureMonitor monitor(config);
        PushObservation in;
        in.camera_pitch_cdeg = 4000;
        in.camera_pitch_stable = true;
        for (int frame = 0; frame < 3; ++frame) {
            const uint64_t now = 1000000 + frame * 50000;
            auto boxes = sample;
            for (auto& d : boxes)
                d.timestamp_us = now;
            monitor.update(in, boxes, now);
        }
        assert(in.hold_observable && in.held == expected);
        assert(in.captured == (expected.total() > 0));
        if (expected.total() > 0)
            assert(in.held_complete);
    };
    constexpr uint64_t now = 1000000;
    check({detection(1, "ordinary_supply", .811f, {589, 22, 171, 210}, now)}, {});
    check({detection(1, "ordinary_supply", .875f, {614, 255, 284, 306}, now),
           detection(2, "core_supply", .176f, {383, 110, 55, 73}, now)},
          inv(1));
    check({detection(1, "ordinary_supply", .854f, {463, 399, 312, 304}, now),
           detection(2, "ordinary_supply", .785f, {594, 228, 254, 252}, now)},
          inv(2));
    check({detection(1, "injured_person", .573f, {622, 255, 290, 465}, now)}, inv(0, 0, 1));
    check({detection(1, "ordinary_supply", .711f, {545, 104, 203, 180}, now),
           detection(2, "ordinary_supply", .429f, {591, 274, 150, 82}, now)},
          {});
    check({detection(1, "ordinary_supply", .862f, {597, 141, 218, 258}, now),
           detection(2, "core_supply", .592f, {321, 101, 104, 111}, now)},
          {});
}
// Field 2026-10-05 07:52: one block held at y=0.158 m while blocks at y~0.29-0.31 m crossed the
// top of the 40deg region; those must not make the held set ambiguous.

void holdingOutsideMouthTests() {
    CaptureConfig config;
    config.holding = {{4000, {440, 210, 930, 720}, 440}};
    config.mouth_y_m = .22f;
    const auto det = [](int id, const char* label, cv::Rect r, bool contact, float y) {
        SegDetection d;
        d.track_id = id;
        d.label = label;
        d.confidence = .9f;
        d.box = r;
        d.ground_contact_valid = d.ground_position_valid = contact;
        d.body_xy_m = {0, y};
        return d;
    };
    const auto run = [&](std::vector<SegDetection> boxes, PushObservation& in, CaptureMonitor& m) {
        in.camera_pitch_cdeg = 4000;
        in.camera_pitch_stable = true;
        for (int frame = 0; frame < 3; ++frame) {
            const uint64_t now = 1000000 + frame * 50000;
            for (auto& d : boxes)
                d.timestamp_us = now;
            m.update(in, boxes, now);
        }
    };
    const auto held = det(8, "ordinary_supply", {560, 330, 320, 390}, true, .158f);
    { // In front of the mouth with a measured contact: ignored, the held block is complete.
        CaptureMonitor m(config);
        PushObservation in;
        run({held,
             det(13, "ordinary_supply", {580, 40, 200, 230}, true, .309f),
             det(12, "core_supply", {430, 0, 200, 215}, true, .347f)},
            in,
            m);
        assert(in.hold_observable && in.captured && in.held_complete && in.held == inv(1));
        assert(m.diagnostics().outside.size() == 2 && m.diagnostics().ambiguous.empty());
    }
    { // The same box without a measured contact stays ambiguous.
        CaptureMonitor m(config);
        PushObservation in;
        run({held, det(13, "ordinary_supply", {580, 40, 200, 230}, false, .309f)}, in, m);
        assert(in.captured && !in.held_complete);
        assert(m.diagnostics().ambiguous.size() == 1 && m.diagnostics().ambiguous[0] == 13);
    }
    { // Within the margin beyond the mouth: still treated conservatively.
        CaptureMonitor m(config);
        PushObservation in;
        run({held, det(13, "ordinary_supply", {580, 40, 200, 230}, true, .24f)}, in, m);
        assert(!in.held_complete);
    }
    { // Uncalibrated mouth (default): old behaviour.
        CaptureConfig old = config;
        old.mouth_y_m = std::numeric_limits<float>::infinity();
        CaptureMonitor m(old);
        PushObservation in;
        run({held, det(13, "ordinary_supply", {580, 40, 200, 230}, true, .309f)}, in, m);
        assert(!in.held_complete);
    }
}
// Field 2026-10-12 01:31: a dangerous block (across 9.5 cm) and an injured person (across -9.8 cm)
// sat just outside a 9 cm centre-point corridor and were found only during RUSH. Values below are
// the logged body_xy_m of the two failures (synthetic frames around logged coordinates, not a full
// log replay).

void corridorFootprintTests() {
    const auto make =
        [](int id, const char* label, float conf, float x, float y, uint64_t t, cv::Rect r) {
            SegDetection d;
            d.track_id = id;
            d.label = label;
            d.confidence = conf;
            d.box = r;
            d.timestamp_us = t;
            d.ground_contact_valid = d.ground_position_valid = true;
            d.body_xy_m = {x, y};
            return d;
        };
    const uint64_t t = 2000000;
    const auto observe = [&](CaptureConfig cfg,
                             std::vector<SegDetection> boxes,
                             float dist,
                             float head,
                             PushObservation& in) {
        CaptureMonitor m(cfg);
        in = {};
        in.target_valid = in.geometry_valid = true;
        in.distance_m = dist;
        in.heading_error = head;
        in.camera_pitch_cdeg = 3500;
        in.camera_pitch_stable = true;
        m.update(in, boxes, t);
        return m.corridorEntries();
    };
    CaptureConfig field;
    field.corridor_half_width_m = .09f; // frame_20261008.assume.json: 0.15/2 + 0.015
    CaptureConfig centre_only = field;
    centre_only.footprint_ordinary_m = centre_only.footprint_dangerous_m =
        centre_only.footprint_core_m = centre_only.footprint_injured_m =
            centre_only.footprint_unknown_m = 0.f;
    const auto target1 = make(190, "ordinary_supply", .9f, -.0039f, .2659f, t, {600, 400, 40, 40});
    const auto danger = make(177, "dangerous_object", .85f, .0909f, .2821f, t, {700, 380, 40, 40});
    PushObservation in;
    { // centre-point rule (old behaviour) misses the dangerous block, the footprint rule counts it
        observe(centre_only, {target1, danger}, .2659f, .0146f, in);
        assert(in.corridor == inv(1));
        assert(checkCorridor(in.corridor,
                             in.corridor_complete,
                             in.corridor_occlusion_free,
                             "ordinary_supply",
                             false) == RuleVerdict::OK);
        const auto entries = observe(field, {target1, danger}, .2659f, .0146f, in);
        assert(in.corridor == inv(1, 0, 0, 1));
        assert(checkCorridor(in.corridor,
                             in.corridor_complete,
                             in.corridor_occlusion_free,
                             "ordinary_supply",
                             false) == RuleVerdict::DANGEROUS);
        bool saw = false;
        for (const auto& e : entries)
            if (e.track_id == 177) {
                saw = true;
                assert(e.counted && e.footprint_only && std::string(e.reason) == "counted" &&
                       e.across_m > .09f);
            }
        assert(saw);
    }
    { // injured person left of the target
        const auto target2 =
            make(286, "ordinary_supply", .9f, -.0037f, .2647f, t, {600, 400, 40, 40});
        const auto injured =
            make(298, "injured_person", .785f, -.1019f, .2767f, t, {480, 380, 80, 40});
        observe(centre_only, {target2, injured}, .2647f, .0132f, in);
        assert(in.corridor == inv(1));
        observe(field, {target2, injured}, .2647f, .0132f, in);
        assert(in.corridor == inv(1, 0, 1));
        assert(checkCorridor(in.corridor,
                             in.corridor_complete,
                             in.corridor_occlusion_free,
                             "ordinary_supply",
                             false) == RuleVerdict::INJURED_BEFORE_FIRST);
    }
    { // a legal pair of ordinary supplies, one only reached by its footprint, still passes
        const auto pair2 = make(191, "ordinary_supply", .9f, .0999f, .2759f, t, {700, 380, 40, 40});
        observe(field, {target1, pair2}, .2659f, .0146f, in);
        assert(in.corridor == inv(2));
        assert(checkCorridor(in.corridor,
                             in.corridor_complete,
                             in.corridor_occlusion_free,
                             "ordinary_supply",
                             true) == RuleVerdict::OK);
    }
    { // clearly outside, behind, stale or low-confidence neighbours are not counted (and say why)
        const auto far_side = make(201, "dangerous_object", .9f, .20f, .28f, t, {900, 380, 40, 40});
        const auto behind = make(202, "dangerous_object", .9f, 0.f, -.10f, t, {600, 600, 40, 40});
        const auto stale =
            make(203, "dangerous_object", .9f, .05f, .28f, t - 400000, {650, 380, 40, 40});
        const auto noise = make(204, "dangerous_object", .05f, .05f, .28f, t, {650, 380, 40, 40});
        const auto entries =
            observe(field, {target1, far_side, behind, stale, noise}, .2659f, .0146f, in);
        assert(in.corridor == inv(1));
        std::set<std::string> reasons;
        for (const auto& e : entries) {
            assert(e.track_id != 203 && e.track_id != 204);
            if (e.track_id == 201 || e.track_id == 202)
                reasons.insert(e.reason);
        }
        assert(reasons.count("outside_lateral") && reasons.count("behind"));
    }
    { // a neighbour without ground position still makes the evidence incomplete
        auto blind = make(205, "dangerous_object", .9f, .2f, .3f, t, {900, 380, 40, 40});
        blind.ground_position_valid = false;
        const auto entries = observe(field, {target1, blind}, .2659f, .0146f, in);
        assert(!in.corridor_complete);
        bool saw = false;
        for (const auto& e : entries)
            if (e.track_id == 205) {
                saw = true;
                assert(!e.counted && std::string(e.reason) == "no_ground_position");
            }
        assert(saw);
    }
}
// Release retreat when the zone is not visible at FAR (field 2026-10-05 07:52: retreat_unverified).

void demoCaptureMotionTests() {
    const auto tune = [] {
        TaskTuning t;
        t.field_rush = true;
        t.demo_capture_motion = true;
        t.precise_rush = false;
        t.rush_stall_us = 0;
        t.far_pitch_cdeg = 500;
        t.intermediate_pitch_cdeg = 2000;
        t.track_pitch_cdeg = t.near_pitch_cdeg = 4000;
        t.track_near_m = .4f;
        t.intermediate_to_near_m = .27f;
        t.rush_start_m = .35f;
        t.rush_speed = .32f;
        t.max_wz = .8f;
        t.min_turn_wz = .4f;
        t.inplace_turn_boost = 2.f;
        t.rush_budget_us = 12000000;
        return t;
    };
    const auto prepare = [](Sim& s) {
        s.in.distance_m = .5f;
        assert(s.until(PushState::APPROACH, 80));
        s.in.distance_m = .36f;
        for (int i = 0; i < 30; ++i)
            s.tick();
        assert(s.out.state == PushState::APPROACH && s.out.motion.camera_pitch_cdeg == 2000);
        s.in.distance_m = .32f;
        assert(s.until(PushState::PREPARE, 20));
        assert(s.out.motion.gripper_offset == 0 && s.out.motion.camera_pitch_cdeg == 2000);
    };
    {
        Sim s(tune());
        prepare(s);
        s.toRush();
        assert(!s.tune.assume_all_safe);
        s.in.distance_m = .18f;
        s.in.heading_error = .02f;
        s.tick();
        assert(s.out.state == PushState::RUSH && s.out.motion.vx_mps == .32f &&
               std::abs(s.out.motion.wz_rps) <= .25f);
        assert(s.out.motion.camera_pitch_cdeg == 2000);
        for (int i = 0; i < 50; ++i) {
            s.tick();
            assert(s.out.state == PushState::RUSH && s.out.motion.vx_mps == .32f);
        }
        s.in.target_valid = false;
        s.in.corridor = {};
        s.in.corridor_complete = false;
        bool drove = false;
        for (int i = 0; i < 100 && s.out.state == PushState::RUSH; ++i) {
            s.tick();
            if (s.out.state == PushState::RUSH) {
                assert(s.out.reason == "assumed_blind_rush" && s.out.motion.wz_rps == 0);
                drove |= s.out.motion.vx_mps == .32f;
            }
        }
        assert(drove && s.out.state == PushState::LOWER_FRAME &&
               s.out.reason == "assumed_enclosure_target_below_view");
        assert(s.out.motion.camera_pitch_cdeg == 4000);
    }
    { // 0.25 rad/s does not move the chassis: pure in-place turns in demo APPROACH use the 0.4
      // floor and escalate to max_wz after 1 s.
        Sim s(tune());
        s.in.distance_m = .5f;
        assert(s.until(PushState::APPROACH, 80));
        s.in.distance_m = .30f;
        s.in.heading_error = .15f;
        float first = 0, last = 0;
        bool turned = false;
        for (int i = 0; i < 80; ++i) {
            s.tick();
            if (s.out.state == PushState::APPROACH && s.out.motion.vx_mps == 0 &&
                s.out.motion.wz_rps != 0) {
                if (!turned) {
                    first = std::abs(s.out.motion.wz_rps);
                    turned = true;
                }
                last = std::abs(s.out.motion.wz_rps);
            }
        }
        assert(turned && first >= .4f - 1e-4f && last > first && last <= .8f + 1e-4f);
    }
    { // Recognized danger wins over the blind completion and the bound on the same tick.
        Sim s(tune());
        prepare(s);
        s.toRush();
        s.in.target_valid = false;
        s.in.corridor = inv(1, 0, 0, 1);
        s.task.actionProgress().travel_limit = 0;
        s.tick();
        assert(s.out.state == PushState::CAPTURE_FAIL &&
               s.out.reason == "rush_corridor_dangerous" && stopped(s.out.motion));
    }
    { // Opening earlier does not bypass PREPARE's inventory check.
        Sim s(tune());
        prepare(s);
        s.in.corridor = inv(1, 0, 1);
        assert(s.until(PushState::LOST_HOLD, 80));
        assert(!s.visited.count(PushState::RUSH));
    }
    { // Time budget still bounds a continuously visible stalled target, as in demo.
        auto t = tune();
        t.rush_budget_us = 500000;
        Sim s(t);
        prepare(s);
        s.toRush();
        assert(s.until(PushState::LOWER_FRAME, 40));
        assert(s.out.reason == "assumed_enclosure_rush_bound");
    }
    { // Normal visual enclosure threshold still closes without waiting for the fallback.
        Sim s(tune());
        prepare(s);
        s.toRush();
        s.in.distance_m = .14f;
        s.tick();
        assert(s.out.state == PushState::LOWER_FRAME && s.out.reason == "frame_enclosure_position");
    }
}

void fieldCaptureCases() {
    { // Other objects in the PREPARE corridor: more frames to confirm, then keep the frame up,
      // reverse and search again.
        auto prepareTicks = [](int frames) {
            TaskTuning t;
            t.corridor_precheck = false;
            t.corridor_confirm_frames = frames;
            Sim s(t);
            s.in.corridor = inv(1, 0, 0, 0, 1);
            int n = 0;
            while (s.out.state != PushState::LOST_HOLD && n < 400) {
                s.tick();
                ++n;
            }
            assert(s.out.state == PushState::LOST_HOLD && !s.visited.count(PushState::RUSH));
            return n;
        };
        assert(prepareTicks(6) - prepareTicks(3) == 3);
        TaskTuning ft;
        ft.corridor_precheck = false;
        ft.corridor_confirm_frames = 6;
        Sim s(ft);
        s.in.corridor = inv(1, 0, 0, 0, 1);
        bool reversed = false, lowered = false;
        bool held = false;
        for (int i = 0; i < 600 && !(held && s.out.state == PushState::SCAN); ++i) {
            s.tick();
            if (s.out.state == PushState::LOST_HOLD) {
                held = true;
                if (s.out.motion.vx_mps < 0)
                    reversed = true;
            }
            if (s.out.state == PushState::LOWER_FRAME || s.out.state == PushState::RUSH)
                lowered = true;
        }
        assert(reversed && !lowered && s.out.state == PushState::SCAN);
        assert(
            s.task.rejectedTargets(s.in.now_us).empty()); // first block: search again, no blacklist
    }
    { // Blocked again and again: the target is blacklisted after corridor_retry_max back-offs.
        TaskTuning pt;
        pt.corridor_precheck = false;
        Sim s(pt);
        s.in.corridor = inv(1, 0, 0, 0, 1);
        int backoffs = 0;
        PushState prev = s.out.state;
        for (int i = 0; i < 4000 && s.task.rejectedTargets(s.in.now_us).empty(); ++i) {
            s.tick();
            if (s.out.state == PushState::LOST_HOLD && prev != PushState::LOST_HOLD)
                ++backoffs;
            prev = s.out.state;
        }
        assert(!s.task.rejectedTargets(s.in.now_us).empty() &&
               backoffs == s.tune.corridor_retry_max + 1);
    }
    { // A fatal verdict (dangerous in the corridor) blacklists at once and remembers the world
      // bearing.
        TaskTuning pt;
        pt.corridor_precheck = false;
        Sim s(pt);
        s.in.corridor = inv(1, 0, 0, 1);
        for (int i = 0; i < 4000 && s.task.rejectedTargets(s.in.now_us).empty(); ++i)
            s.tick();
        assert(!s.task.rejectedTargets(s.in.now_us).empty());
        assert(s.task.rejectedBearings(s.in.now_us).size() == 1);
    }
    { // Without zone geometry the back-off is blind: corridor_back_m x motion ratio of commanded
      // reverse; unsafe rear skips it.
        for (bool rear_safe : {true, false}) {
            TaskTuning bt;
            bt.corridor_precheck = false;
            bt.back_speed = .32f; // field speed: the 0.8 commanded fits the retreat budget
            Sim s(bt);
            s.in.corridor = inv(1, 0, 0, 1);
            s.in.retreat_safe = rear_safe;
            assert(s.until(PushState::PREPARE, 80));
            s.zone_visible = false;
            float back = 0;
            bool held = false;
            for (int i = 0; i < 800 && !(held && s.out.state == PushState::SCAN); ++i) {
                s.tick();
                if (s.out.state == PushState::LOST_HOLD) {
                    held = true;
                    back += -std::min(0.f, s.out.motion.vx_mps) * .05f;
                }
            }
            const float want = s.tune.corridor_back_m * s.tune.assume_motion_ratio;
            assert(held && s.out.state == PushState::SCAN);
            if (rear_safe)
                assert(back >= want - .05f && back <= want + .05f);
            else
                assert(back < .05f && s.out.reason == "corridor_back_off_unsafe");
        }
    }
    { // field_rush: ID rebound after the frame opens, scaled rush limit, blind finish below view
      // (strict path unchanged).
        for (int mode = 0; mode < 3;
             ++mode) { // 0 strict, 1 field_rush near, 2 field_rush far (not the same object)
            TaskTuning ft;
            ft.field_rush = mode > 0;
            ft.corridor_confirm_frames = 3;
            Sim s(ft);
            bool switched = false, relost = false;
            for (int i = 0; i < 200 && !relost && !s.visited.count(PushState::RUSH); ++i) {
                s.tick();
                if (!switched && s.out.state == PushState::PREPARE &&
                    s.out.reason == "open_confirmed_return_to_tracking_pitch") {
                    switched = true;
                    s.in.target_id = 9;
                    if (mode == 2)
                        s.in.distance_m += .30f;
                }
                if (switched && s.out.state == PushState::SCAN)
                    relost = true;
            }
            assert(switched);
            if (mode == 1)
                assert(s.visited.count(PushState::RUSH) && !relost);
            else
                assert(relost && !s.visited.count(PushState::RUSH));
        }
        for (bool fr : {false, true}) {
            TaskTuning ft;
            ft.field_rush = fr;
            ft.rush_stall_us = 0; // legacy bound path, watchdog covered separately
            Sim s(ft);
            s.in.distance_m = .30f;
            s.toRush();
            bool failed = false;
            for (int i = 0; i < 400 && s.out.state != PushState::LOWER_FRAME && !failed; ++i) {
                s.tick();
                failed = s.out.state == PushState::CAPTURE_FAIL;
            }
            if (fr)
                assert(s.out.state == PushState::LOWER_FRAME && !failed);
            else
                assert(failed && s.out.reason == "rush_overrun");
        }
        for (bool fr : {false, true}) {
            TaskTuning ft;
            ft.field_rush = fr;
            Sim s(ft);
            s.in.distance_m = .30f;
            s.toRush();
            for (int i = 0; i < 3; ++i)
                s.tick();
            s.in.target_valid = false;
            bool drove = false, failed = false;
            for (int i = 0; i < 400 && s.out.state == PushState::RUSH; ++i) {
                s.tick();
                drove = drove || s.out.motion.vx_mps > 0;
                failed = s.out.state == PushState::CAPTURE_FAIL;
            }
            if (fr)
                assert(s.out.state == PushState::LOWER_FRAME && drove &&
                       s.out.reason == "assumed_enclosure_target_below_view");
            else
                assert(failed && s.out.reason == "rush_target_lost");
        }
    }
    { // RUSH no-progress watchdog (Stage C). Synthetic frames: frozen/closing distance values, not
      // a replay of the field log.
        const auto run = [](const char* mode) {
            TaskTuning ft;
            ft.field_rush = true;
            Sim s(ft);
            s.tune.rush_stall_us = 2000000;
            s.task = PushTask(s.tune);
            s.in.distance_m = .30f;
            s.toRush();
            if (std::string(mode) == "closing")
                s.in.distance_m = .45f;
            if (std::string(mode) == "in_frame")
                s.hold(inv(1));
            bool reobserve = false, lowered = false;
            float moved_in_reobserve = 0;
            for (int i = 0; i < 400 && s.out.state == PushState::RUSH; ++i) {
                if (std::string(mode) == "closing")
                    s.in.distance_m -= .0015f;
                if (std::string(mode) == "flicker" && i % 4 == 0)
                    s.pitch_still = 0;
                s.tick();
                if (s.out.reason == "rush_stall_reobserve") {
                    reobserve = true;
                    moved_in_reobserve +=
                        std::abs(s.out.motion.vx_mps) + std::abs(s.out.motion.wz_rps);
                }
                lowered = lowered || s.out.state == PushState::LOWER_FRAME;
            }
            struct R {
                PushState st;
                std::string reason;
                bool reobserve, lowered;
                float moved;
            } r{s.out.state, s.out.reason, reobserve, lowered, moved_in_reobserve};
            return r;
        };
        {
            const auto r = run("frozen"); // valid stall: stop, re-observe, then recover; the bound
                                          // must not be credited as enclosure
            assert(r.reobserve && r.moved == 0 && !r.lowered && r.st == PushState::CAPTURE_FAIL &&
                   r.reason == "rush_no_progress");
        }
        {
            const auto r =
                run("in_frame"); // reliable in-frame evidence at the stall allows lowering
            assert(r.st == PushState::LOWER_FRAME && r.reason == "rush_stall_in_frame_evidence");
        }
        {
            const auto r = run("closing"); // real closing (3 cm/s) never raises a stall
            assert(!r.reobserve && r.reason != "rush_no_progress");
        }
        {
            const auto r =
                run("flicker"); // camera repeatedly unstable: no valid window, so no stall claim
            assert(!r.reobserve && r.reason != "rush_no_progress");
        }
    }
    { // VERIFY_CAPTURE false-class re-check (Stage D). Synthetic held-region frames; the field #313
      // frame itself is not replayed.
        struct R {
            PushState st;
            std::string reason;
            int bad_frames_seen;
            bool carried;
        };
        // pattern(k) -> bad objects present in verify frame k (core on a track id, with confidence)
        const auto run =
            [](const std::function<std::vector<PushObservation::HeldObject>(int)>& pattern,
               bool recheck_on = true) {
                TaskTuning ft;
                ft.field_rush = true;
                ft.verify_held_contents = true;
                Sim s(ft);
                s.tune.require_multi_view = true;
                s.tune.held_recheck = recheck_on;
                s.task = PushTask(s.tune);
                s.multi_evidence = false;
                s.toRush();
                s.hold(inv(1));
                int k = -1;
                bool carried = false;
                int bad_seen = 0;
                for (int i = 0; i < 400 && s.out.state != PushState::ABORT_DROP &&
                                s.out.state != PushState::CAPTURE_FAIL && !carried;
                     ++i) {
                    if (s.out.state == PushState::VERIFY_CAPTURE) {
                        ++k;
                        auto objs = pattern(k);
                        s.load = inv(1);
                        s.load_objects = {{7, "ordinary_supply", .9f}};
                        for (const auto& o : objs) {
                            s.load_objects.push_back(o);
                            if (o.label == "core_supply")
                                s.load.core++;
                            if (o.label == "dangerous_object")
                                s.load.dangerous++;
                        }
                        if (!objs.empty())
                            ++bad_seen;
                    }
                    s.tick();
                    carried = s.out.state == PushState::CARRY;
                }
                return R{s.out.state, s.out.reason, bad_seen, carried};
            };
        const auto core = [](int id, float c) {
            return PushObservation::HeldObject{id, "core_supply", c};
        };
        {
            const auto r = run(
                [&](int k) {
                    return k >= 2 && k < 4
                               ? std::vector<PushObservation::HeldObject>{core(99, .54f)}
                               : std::vector<PushObservation::HeldObject>{};
                },
                false);
            assert(r.st == PushState::ABORT_DROP && r.reason == "held_core_before_first");
        } // old behaviour: two frames abort
        {
            const auto r = run([&](int k) {
                return k >= 2 && k < 4 ? std::vector<PushObservation::HeldObject>{core(99, .54f)}
                                       : std::vector<PushObservation::HeldObject>{};
            });
            assert(r.carried && r.reason == "assumed_capture" && r.st == PushState::CARRY);
        } // brief weak class: re-check clears, flow continues
        {
            const auto r = run([&](int k) {
                return k >= 2 ? std::vector<PushObservation::HeldObject>{core(99, .54f)}
                              : std::vector<PushObservation::HeldObject>{};
            });
            assert(r.st == PushState::ABORT_DROP && r.reason == "held_core_before_first" &&
                   r.bad_frames_seen >= 4);
        } // stable same identity: confirmed illegal, existing drop
        {
            const auto r = run([&](int k) {
                return k >= 2 ? std::vector<PushObservation::HeldObject>{core(90 + k, .54f)}
                              : std::vector<PushObservation::HeldObject>{};
            });
            assert(!r.carried && r.st == PushState::CAPTURE_FAIL &&
                   r.reason == "held_illegal_unresolved");
        } // identity keeps changing: uncertain is not a capture
        {
            const auto r = run([&](int k) {
                return k >= 2 ? std::vector<PushObservation::HeldObject>{core(99, .90f)}
                              : std::vector<PushObservation::HeldObject>{};
            });
            assert(r.st == PushState::ABORT_DROP && r.bad_frames_seen == 2);
        } // strong class keeps the prompt block
        {
            const auto r = run([&](int k) {
                return k >= 2
                           ? std::vector<PushObservation::HeldObject>{{98, "dangerous_object", .5f}}
                           : std::vector<PushObservation::HeldObject>{};
            });
            assert(r.st == PushState::ABORT_DROP && r.reason == "held_dangerous" &&
                   r.bad_frames_seen == 2);
        } // a dangerous object is never re-checked
    }
    { // field_rush: unconfirmed multi-view credits the capture; a measured inventory conflict stays
      // fatal.
        TaskTuning ft;
        ft.field_rush = true;
        {
            Sim s(ft);
            s.tune.require_multi_view = true;
            s.task = PushTask(s.tune);
            s.multi_evidence = false;
            s.toRush();
            s.hold(inv(1));
            assert(s.until(PushState::CARRY, 300) && s.out.reason == "assumed_capture");
        }
        {
            Sim s(ft);
            s.tune.require_multi_view = true;
            s.task = PushTask(s.tune);
            s.toRush();
            s.hold(inv(2));
            bool credited = false;
            for (int i = 0; i < 300 && s.out.state != PushState::ABORT_DROP; ++i) {
                s.tick();
                credited = credited || s.out.reason == "assumed_capture";
            }
            assert(!credited && s.out.state == PushState::ABORT_DROP &&
                   s.out.reason.rfind("multi_grab_", 0) == 0);
        }
    }
    { // field_rush: after the frame opens the tracker re-IDs the target and the 40deg view clips it
      // (region/geometry invalid): rebind by position, not by validity.
        TaskTuning ft;
        ft.field_rush = true;
        ft.corridor_confirm_frames = 3;
        Sim s(ft);
        bool done = false;
        for (int i = 0; i < 200 && !done; ++i) {
            s.tick();
            if (s.out.state == PushState::PREPARE &&
                s.out.reason == "open_confirmed_return_to_tracking_pitch") {
                s.in.target_id = 9;
                s.in.target_region_valid = false;
                s.in.geometry_valid = false;
                for (int k = 0; k < 40 && s.out.reason != "prepare_id_rebound_after_open"; ++k)
                    s.tick();
                assert(s.out.reason == "prepare_id_rebound_after_open" && s.out.target_id == 9);
                done = true;
            }
        }
        assert(done);
    }
    { // Frame contents seen in VERIFY_CAPTURE: the target must be inside, a dangerous object never,
      // an injured person only after a delivery and alone.
        const auto run = [](bool first, const Inventory& held, PushState want, const char* prefix) {
            TaskTuning ft;
            ft.field_rush = true;
            ft.verify_held_contents = true;
            ft.assume_injured_trip = first;
            Sim s(ft);
            s.tune.require_multi_view = true;
            s.task = PushTask(s.tune);
            s.multi_evidence = false;
            s.toRush();
            s.hold(held);
            for (int i = 0; i < 400 && s.out.state != want; ++i)
                s.tick();
            assert(s.out.state == want && s.out.reason.rfind(prefix, 0) == 0);
        };
        Inventory with_danger = inv(1);
        with_danger.dangerous = 1;
        Inventory with_injured = inv(1);
        with_injured.injured = 1;
        run(false, inv(1), PushState::CARRY, "assumed_capture");
        run(false, with_danger, PushState::ABORT_DROP, "held_");
        run(true, with_danger, PushState::ABORT_DROP, "held_");
        run(false, with_injured, PushState::ABORT_DROP, "held_");
        run(true, with_injured, PushState::ABORT_DROP, "held_");
        Inventory only_injured;
        only_injured.injured = 1;
        run(true, only_injured, PushState::CAPTURE_FAIL, "capture_target_not_in_frame");
        run(true, Inventory{}, PushState::CAPTURE_FAIL, "capture_target_not_in_frame");
    }
}
