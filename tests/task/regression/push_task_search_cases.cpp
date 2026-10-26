#include "../../support/push_task_sim.hpp"
#include "push_task_cases.hpp"

void precisionMotionTests() {
    const auto tune = [] {
        TaskTuning t;
        t.field_rush = true;
        t.precise_rush = true;
        t.min_turn_wz = .4f;
        t.rush_speed = .32f;
        return t;
    };
    {
        Sim s(tune());
        s.in.distance_m = .30f;
        s.toRush();
        const float bound = s.task.travel_limit_;
        s.in.distance_m = .18f;
        s.in.heading_error = .01745f;
        s.tick();
        assert(s.out.state == PushState::RUSH && s.out.motion.vx_mps > 0 &&
               s.out.motion.vx_mps <= .16f);
        assert(s.out.motion.wz_rps == 0 && s.out.reason == "rush_straight_near");
        for (int i = 0; i < 10; ++i)
            s.tick();
        assert(s.task.travel_limit_ ==
               bound); // a frozen distance cannot move the hard bound forward
        s.in.heading_error = .10f;
        s.tick();
        assert(s.out.reason == "rush_align_at_rest" && s.out.motion.vx_mps == 0);
        assert(s.out.motion.wz_rps > 0 && s.out.motion.wz_rps <= .25f);
        s.in.heading_error = -.10f;
        s.tick();
        assert(s.out.motion.vx_mps == 0 && s.out.motion.wz_rps < 0 && s.out.motion.wz_rps >= -.25f);
        s.in.directional_clearance_valid = true;
        s.in.turn_safe = false;
        s.tick();
        assert(stopped(s.out.motion));
    }
    for (int mode = 0; mode < 4; ++mode) {
        Sim s(tune());
        s.in.distance_m = .30f;
        s.toRush();
        if (mode == 0)
            s.task.travel_limit_ = 0; // distance bound
        if (mode == 2)
            s.in.target_valid = false; // lost target cannot authorize blind drive
        if (mode == 3) {
            s.task.travel_limit_ = 0;
            s.hold(inv(1));
        } // positive held-region evidence still permits enclosure
        if (mode == 1) {
            s.in.now_us += s.tune.rush_budget_us;
            s.task.last_us_ = s.in.now_us;
            s.task.phase_us_ = s.in.now_us - s.tune.rush_budget_us - 1;
        }
        s.tick();
        assert(s.out.state == PushState::RUSH && stopped(s.out.motion));
        assert(s.out.reason == (mode == 2 ? "rush_target_lost_reobserve" : "rush_bound_reobserve"));
        bool lowered = false;
        for (int i = 0; i < 100 && s.out.state == PushState::RUSH; ++i) {
            s.tick();
            assert(stopped(s.out.motion));
            lowered |= s.out.state == PushState::LOWER_FRAME;
        }
        if (mode == 3)
            assert(lowered && s.out.reason == "rush_stall_in_frame_evidence");
        else
            assert(!lowered && s.out.state == PushState::CAPTURE_FAIL);
    }
    {
        Sim s(tune());
        s.in.distance_m = .30f;
        s.toRush();
        s.in.distance_m = .14f;
        s.in.heading_error = .08f;
        s.tick();
        assert(s.out.state == PushState::RUSH &&
               s.out.motion.vx_mps == 0); // close enough but not aligned: no drop
        s.in.heading_error = 0;
        s.tick();
        assert(s.out.state == PushState::LOWER_FRAME);
    }
    for (bool white : {true, false}) { // SCAN: a block seen for more than 3 frames at confidence >=
                                       // .5 is driven at before its ground projection works
        Sim s;
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        s.in.target_valid = true;
        s.in.geometry_valid = false;
        s.in.pixel_bearing_valid = true;
        s.in.pixel_bearing = .1f;
        s.in.pixel_confidence = white ? .6f : .4f;
        for (int i = 0; i < 3; ++i) {
            s.tick();
            assert(s.out.state == PushState::SCAN && s.out.motion.vx_mps == 0);
        }
        s.tick();
        if (white) {
            assert(s.out.state == PushState::SCAN && s.out.reason == "visual_chase" &&
                   s.out.motion.vx_mps > 0);
            s.in.geometry_valid = true;
            assert(s.until(PushState::APPROACH, 10)); // once projectable the normal lock takes over
        } else {
            assert(s.out.state == PushState::SCAN && s.out.motion.vx_mps == 0 &&
                   s.out.reason == "searching_target"); // low confidence: keep spinning
        }
    }
    { // SCAN: a pixel candidate stops the spin; the lock then collects the 4 frames and
      // visual_chase starts
        Sim s;
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        s.tick();
        assert(s.out.motion.wz_rps != 0 &&
               s.out.reason == "searching_target"); // nothing in view: spinning
        s.in.pixel_reason = "confirming_three_frames";
        s.tick();
        assert(s.out.state == PushState::SCAN && s.out.reason == "scan_sighting_hold" &&
               stopped(s.out.motion));
        for (int i = 0; i < 2; ++i) {
            s.tick();
            assert(s.out.reason == "scan_sighting_hold" && stopped(s.out.motion));
        }
        s.in.pixel_reason = "pixel_ratio_locked";
        s.in.target_valid = true;
        s.in.geometry_valid = false;
        s.in.pixel_bearing_valid = true;
        s.in.pixel_bearing = .1f;
        s.in.pixel_confidence = .6f;
        for (int i = 0; i < 3; ++i) {
            s.tick();
            assert(s.out.reason == "scan_sighting_hold" && stopped(s.out.motion));
        }
        s.tick();
        assert(s.out.reason == "visual_chase" && s.out.motion.vx_mps > 0);
    }
    { // a candidate that never locks holds the spin for scan_hold_max_us only, then the scan
      // resumes for the cool-down
        Sim s;
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        s.in.pixel_reason = "confirming_three_frames";
        int held = 0;
        for (int i = 0; i < 60; ++i) {
            s.tick();
            if (s.out.reason == "scan_sighting_hold")
                ++held;
            else
                break;
        }
        assert(held >= 25 && held <= 32 && s.out.motion.wz_rps != 0);
        for (int i = 0; i < 20; ++i) {
            s.tick();
            assert(s.out.reason != "scan_sighting_hold" && s.out.motion.wz_rps != 0);
        }
        for (int i = 0; i < 20; ++i)
            s.tick();
        s.tick();
        assert(s.out.reason ==
               "scan_sighting_hold"); // cool-down over: the next sighting holds again
    }
    { // a candidate that disappears frees the spin after scan_hold_lost_us
        Sim s;
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        s.in.pixel_reason = "confirming_three_frames";
        s.tick();
        assert(s.out.reason == "scan_sighting_hold");
        s.in.pixel_reason = "no_legal_candidate";
        int held = 0;
        for (int i = 0; i < 30; ++i) {
            s.tick();
            if (s.out.reason == "scan_sighting_hold")
                ++held;
            else
                break;
        }
        assert(held >= 6 && held <= 10 && s.out.motion.wz_rps != 0);
    }
    for (int phase : {4, 5, 7}) {
        TaskTuning t;
        t.sweep_observation_guard = true;
        t.disperse_swing_vx = 0;
        t.blind_scatter_passes = 1;
        Sim s(t);
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        s.task.enter(PushState::SCATTER, s.in.now_us, "test_sweep");
        s.task.sweep_phase_ = phase;
        s.task.sweep_ledger_.begin(s.task.t_, s.in.now_us);
        s.task.sweep_ledger_.setSwung(phase != 7);
        s.task.sweep_start_us_ = s.in.now_us;
        s.task.disperse_rush_m_ = .5f;
        s.in.clutter_body.clear();
        s.in.clutter_ts_us = 0;
        s.tick();
        assert(s.out.state == PushState::SCAN && stopped(s.out.motion));
        assert(s.out.reason == "blind_scatter_observation_lost");
        const auto spent = s.task.sweepSpentUs(s.in.now_us);
        assert(spent > 0);
        s.in.target_valid = true;
        for (int i = 0; i < 8; ++i) {
            s.tick();
            assert(stopped(s.out.motion) && s.out.reason == "sweep_reobserve");
        }
        assert(s.task.sweepSpentUs(s.in.now_us) == spent);
        assert(
            s.until(PushState::APPROACH, 50)); // observation pause is bounded, not a new deadlock
    }
    {
        TaskTuning t;
        t.sweep_observation_guard = true;
        t.disperse_swing_vx = 0;
        Sim s(t);
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        s.task.enter(PushState::SCATTER, s.in.now_us, "test_swing");
        s.task.sweep_phase_ = 5;
        s.task.sweep_ledger_.begin(s.task.t_, s.in.now_us);
        s.task.sweep_start_us_ = s.in.now_us;
        s.task.sweep_yaw0_ = s.phi;
        s.task.disperse_swing_.configure(s.task.disperse_swing_.passLimit(), 3);
        s.in.clutter_body = {{0, .3f}, {.1f, .3f}, {-.1f, .3f}};
        s.in.clutter_ts_us = s.in.now_us;
        s.tick();
        assert(s.out.motion.vx_mps == 0 &&
               s.out.motion.wz_rps != 0); // pure swing, no hidden forward creep
    }
    // Single turn: camera to the 20deg stage first, then one in-place turn toward the side with
    // fewer blocks in view.
    for (int blocks_left : {1, 0}) {
        TaskTuning t;
        t.sweep_observation_guard = true;
        t.disperse_swing_vx = 0;
        t.disperse_single_turn = true;
        t.far_pitch_cdeg = 500;
        t.intermediate_pitch_cdeg = 2000;
        t.track_pitch_cdeg = 4000;
        Sim s(t);
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        s.task.enter(PushState::SCATTER, s.in.now_us, "test_single_turn");
        s.task.sweep_phase_ = 5;
        s.task.sweep_ledger_.begin(s.task.t_, s.in.now_us);
        s.task.sweep_start_us_ = s.in.now_us;
        s.task.sweep_yaw0_ = s.phi;
        s.task.disperse_turn_.begin(true);
        s.task.disperse_sign_ = blocks_left ? 1.f : -1.f;
        const auto feed = [&] { // blocks on the left (x<0) only when requested; right side empty
            s.in.clutter_body.clear();
            if (blocks_left)
                s.in.clutter_body = {{-.2f, .5f}, {-.25f, .6f}, {-.15f, .45f}};
            else
                s.in.clutter_body = {{.2f, .5f}, {.25f, .6f}, {.15f, .45f}, {.1f, .5f}};
            s.in.clutter_ts_us = s.in.now_us;
        };
        feed();
        const float phi0 = s.phi;
        bool looked = false, turned = false;
        float dir_seen = 0;
        for (int i = 0; i < 400; ++i) {
            feed();
            s.tick();
            if (s.out.reason == "disperse_look") {
                assert(!turned && stopped(s.out.motion));
                looked = true;
            }
            if (s.out.reason == "disperse_turn") {
                assert(looked && s.out.motion.vx_mps == 0 &&
                       s.out.motion.camera_pitch_cdeg == s.tune.intermediate_pitch_cdeg);
                turned = true;
                dir_seen = s.out.motion.wz_rps;
            }
            if (!s.task.disperse_turn_.active() && turned)
                break;
        }
        assert(looked && turned && !s.task.disperse_turn_.active());
        assert(dir_seen * (blocks_left ? -1.f : 1.f) >
               0); // fewer blocks on the right: turn right; fewer on the left: turn left
        const float done = std::abs(s.phi - phi0);
        assert(done >= s.tune.disperse_turn_rad - s.tune.disperse_swing_tol_rad - .2f &&
               done <= s.tune.disperse_turn_rad + .4f); // one turn, not three swings
        assert(s.task.disperse_swing_.swingCount() == 0);
    }
    { // thin view: stops after the minimum turn instead of the full angle
        TaskTuning t;
        t.sweep_observation_guard = true;
        t.disperse_swing_vx = 0;
        t.disperse_single_turn = true;
        t.far_pitch_cdeg = 500;
        t.intermediate_pitch_cdeg = 2000;
        t.track_pitch_cdeg = 4000;
        Sim s(t);
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        s.task.enter(PushState::SCATTER, s.in.now_us, "test_single_turn_thin");
        s.task.sweep_phase_ = 5;
        s.task.sweep_ledger_.begin(s.task.t_, s.in.now_us);
        s.task.sweep_start_us_ = s.in.now_us;
        s.task.sweep_yaw0_ = s.phi;
        s.task.disperse_turn_.begin(true);
        s.task.disperse_sign_ = 1.f;
        const float phi0 = s.phi;
        for (int i = 0; i < 400; ++i) {
            s.in.clutter_body.clear();
            s.in.clutter_ts_us = s.in.now_us;
            s.tick();
            if (!s.task.disperse_turn_.active() && s.out.reason != "disperse_look" &&
                std::abs(s.phi - phi0) > .1f)
                break;
        }
        assert(!s.task.disperse_turn_.active());
        const float done = std::abs(s.phi - phi0);
        assert(done >= s.tune.disperse_turn_min_rad - .1f && done < s.tune.disperse_turn_rad - .3f);
    }
    for (int zone_blocks : {0, 3}) { // frame-zone check: thin view ahead is not enough, the low
                                     // camera must also see <=1 block in the zone
        TaskTuning t;
        t.sweep_observation_guard = true;
        t.disperse_swing_vx = 0;
        t.disperse_single_turn = true;
        t.disperse_turn_frame_check = true;
        t.disperse_turn_rad = 1.8f;
        t.far_pitch_cdeg = 500;
        t.intermediate_pitch_cdeg = 2000;
        t.track_pitch_cdeg = 4000;
        Sim s(t);
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        s.task.enter(PushState::SCATTER, s.in.now_us, "test_frame_zone");
        s.task.sweep_phase_ = 5;
        s.task.sweep_ledger_.begin(s.task.t_, s.in.now_us);
        s.task.sweep_start_us_ = s.in.now_us;
        s.task.sweep_yaw0_ = s.phi;
        s.task.disperse_turn_.begin(true);
        s.task.disperse_sign_ = 1.f;
        const float phi0 = s.phi;
        bool low_turn = false, low_look = false;
        for (int i = 0; i < 600; ++i) {
            s.in.clutter_body.clear();
            if (s.task.disperse_turn_.needsFrameView() && zone_blocks)
                for (int k = 0; k < zone_blocks; ++k)
                    s.in.clutter_body.push_back(
                        {.03f * float(k), .2f}); // only the low camera sees the zone
            s.in.clutter_ts_us = s.in.now_us;
            s.tick();
            if (s.out.reason == "disperse_frame_look") {
                assert(stopped(s.out.motion));
                low_look = true;
            }
            if (s.out.reason == "disperse_turn_frame") {
                assert(s.out.motion.vx_mps == 0 &&
                       s.out.motion.camera_pitch_cdeg == s.tune.track_pitch_cdeg);
                low_turn = true;
            }
            if (!s.task.disperse_turn_.active() && low_look)
                break;
        }
        assert(!s.task.disperse_turn_.active() && low_look);
        const float done = std::abs(s.phi - phi0);
        if (zone_blocks == 0) {
            assert(!low_turn && done < s.tune.disperse_turn_rad - .3f);
        } // zone clear at once: stops at the minimum turn
        else {
            assert(
                low_turn &&
                done >= s.tune.disperse_turn_min_rad + s.tune.disperse_turn_frame_extra_rad - .1f &&
                done < s.tune.disperse_turn_min_rad + s.tune.disperse_turn_frame_extra_rad + .25f);
        } // zone stays crowded: only the extra allowance past the minimum turn, not the cap
    }
    { // push-on after the turn: straight forward ~disperse_turn_push_m along the new heading, then
      // the sweep ends
        TaskTuning t;
        t.sweep_observation_guard = true;
        t.disperse_swing_vx = 0;
        t.disperse_single_turn = true;
        t.disperse_turn_push_m = .2f;
        t.far_pitch_cdeg = 500;
        t.intermediate_pitch_cdeg = 2000;
        t.track_pitch_cdeg = 4000;
        Sim s(t);
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        s.task.enter(PushState::SCATTER, s.in.now_us, "test_single_turn_push");
        s.task.sweep_phase_ = 5;
        s.task.sweep_ledger_.begin(s.task.t_, s.in.now_us);
        s.task.sweep_start_us_ = s.in.now_us;
        s.task.sweep_yaw0_ = s.phi;
        s.task.disperse_turn_.begin(true);
        s.task.disperse_sign_ = 1.f;
        bool pushed = false;
        float phi_push = 0;
        for (int i = 0; i < 600; ++i) {
            s.in.clutter_body.clear();
            s.in.clutter_ts_us = s.in.now_us;
            s.tick();
            if (s.out.reason == "disperse_turn_push") {
                if (!pushed)
                    phi_push = s.phi;
                pushed = true;
                if (s.task.disperse_turn_.phase() == DisperseTurnPhase::Push)
                    assert(s.out.motion.vx_mps > 0 &&
                           s.out.motion.wz_rps ==
                               0); // the last tick finishes the push and ends the sweep
            }
            if (pushed && !s.task.disperse_turn_.active())
                break;
        }
        assert(pushed && !s.task.disperse_turn_.active());
        assert(std::abs(s.phi - phi_push) < .05f); // no turning while pushing
    }
}
// Sweeps with the frame open: open acknowledged before the chassis moves, closed again before it
// moves on, cargo logic untouched.

void searchSelectionCases() {
    { // No eligible cargo: stop to reselect, then rescan; never push the cue.
        TaskTuning t;
        t.enable_search_cues = true;
        Sim s(t);
        s.in.label = "dangerous_object";
        s.in.target_is_search_cue = true;
        s.in.distance_m = .34f;
        assert(s.until(PushState::SELECT_CARGO, 80));
        s.in.target_is_search_cue = false;
        for (int i = 0; i < 90; ++i) {
            s.tick();
            assert(stopped(s.out.motion) && s.out.motion.gripper_offset == 20);
            if (s.out.state == PushState::SCAN)
                break;
        }
        assert(s.out.state == PushState::SCAN && s.out.reason == "no_eligible_cargo");
        assert(!s.visited.count(PushState::CLEAR_PILE) && !s.out.first_ordinary_delivered);
    }
    { // Switch at 0.6m, before the old 0.35m gate; never drive while pitch moves.
        TaskTuning t;
        t.enable_search_cues = true;
        t.track_pitch_cdeg = 4000;
        Sim s(t);
        s.in.target_is_search_cue = true;
        s.in.distance_m = .55f;
        assert(s.until(PushState::CUE_APPROACH, 80));
        s.tick();
        assert(stopped(s.out.motion) && s.out.motion.camera_pitch_cdeg == 4000);
        for (int i = 0; i < 15; ++i) {
            s.tick();
            if (!s.in.camera_pitch_stable)
                assert(stopped(s.out.motion));
        }
        assert(s.pitch == 4000);
        s.in.distance_m = .34f;
        assert(s.out.state == PushState::NEAR_REACQUIRE);
        assert(s.out.motion.camera_pitch_cdeg == 4000);
        s.in.target_is_search_cue = false;
        s.in.target_id = 9;
        s.in.distance_m = .2f;
        s.toCarry();
        assert(s.out.motion.camera_pitch_cdeg == 500);
    }
    { // Re-identify a new green ID at 40 degrees without lifting the camera.
        TaskTuning t;
        t.enable_search_cues = true;
        t.track_pitch_cdeg = 4000;
        Sim s(t);
        s.in.target_is_search_cue = true;
        s.in.label = "core_supply";
        s.in.distance_m = .55f;
        assert(s.until(PushState::NEAR_REACQUIRE, 80));
        s.in.target_valid = false;
        for (int i = 0; i < 20; ++i) {
            s.tick();
            assert(stopped(s.out.motion) && s.out.motion.camera_pitch_cdeg == 4000);
        }
        s.in.target_valid = true;
        s.in.target_is_search_cue = false;
        s.in.target_id = 99;
        s.in.label = "ordinary_supply";
        assert(s.until(PushState::APPROACH, 10));
        assert(s.out.capture_target_id == 99 && s.out.motion.camera_pitch_cdeg == 4000);
        s.in.target_valid = false;
        assert(s.until(PushState::NEAR_REACQUIRE, 20));
        assert(s.out.motion.camera_pitch_cdeg == 4000);
        assert(s.until(PushState::SAFE_STOP, 90));
        assert(s.out.reason == "near_reacquire_failed_stop");
    }
    { // Empty near views get one return to FAR; the second failure latches at NEAR.
        TaskTuning t;
        t.enable_search_cues = true;
        t.track_pitch_cdeg = 4000;
        Sim s(t);
        s.in.target_is_search_cue = true;
        s.in.distance_m = .55f;
        assert(s.until(PushState::NEAR_REACQUIRE, 80));
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 100));
        s.in.target_valid = true;
        s.in.target_id = 100;
        assert(s.until(PushState::NEAR_REACQUIRE, 80));
        s.in.target_valid = false;
        assert(s.until(PushState::SAFE_STOP, 100));
        for (int i = 0; i < 10; ++i) {
            s.tick();
            assert(stopped(s.out.motion) && s.out.motion.camera_pitch_cdeg == 4000);
        }
    }
    { // 无线索的直接APPROACH：远处保持5°，贴近时停车下压到20°，再到40°（只下压不回抬）。
        TaskTuning t;
        t.enable_search_cues = false;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.track_near_m = .4f;
        t.intermediate_to_near_m = .27f;
        t.rush_start_m = .2f;
        Sim s(t);
        s.in.distance_m = .5f;
        assert(s.until(PushState::APPROACH, 80));
        for (int i = 0; i < 5; ++i) {
            s.tick();
            assert(s.out.state == PushState::APPROACH && s.out.motion.camera_pitch_cdeg == 500);
        }
        s.in.distance_m = .39f;
        s.tick();
        assert(s.out.state == PushState::APPROACH && stopped(s.out.motion) &&
               s.out.motion.camera_pitch_cdeg == 2000);
        for (int i = 0; i < 12; ++i) {
            s.in.distance_m = .45f;
            s.tick();
            assert(s.out.motion.camera_pitch_cdeg == 2000);
        } // 到位后远离也不回抬
        s.in.distance_m = .26f;
        for (int i = 0; i < 12 && s.out.motion.camera_pitch_cdeg != 4000; ++i)
            s.tick();
        assert(s.out.state == PushState::APPROACH && s.out.motion.camera_pitch_cdeg == 4000);
    }
    { // 下压后跟踪器改发新ID：改绑继续接近，不进入丢失搜索。
        TaskTuning t;
        t.enable_search_cues = false;
        t.enable_lost_search = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.track_near_m = .4f;
        t.intermediate_to_near_m = .27f;
        t.rush_start_m = .2f;
        Sim s(t);
        s.in.distance_m = .5f;
        assert(s.until(PushState::APPROACH, 80));
        s.in.distance_m = .39f;
        s.tick();
        assert(s.out.motion.camera_pitch_cdeg == 2000);
        s.in.target_id += 1;
        s.in.distance_m = .47f;
        for (int i = 0; i < 40; ++i) {
            s.tick();
            assert(s.out.state == PushState::APPROACH);
        }
        assert(s.out.motion.vx_mps > 0 && s.out.capture_target_id == s.in.target_id);
    }
    { // 下压后图像无效/重新确认期间（远超
      // hold_grace_frames）不判丢失；窗口内恢复并换新ID则改绑，窗口耗尽才进丢失搜索。
        for (int late = 0; late < 2; ++late) {
            TaskTuning t;
            t.enable_search_cues = false;
            t.enable_lost_search = true;
            t.track_pitch_cdeg = 4000;
            t.intermediate_pitch_cdeg = 2000;
            t.track_near_m = .4f;
            t.intermediate_to_near_m = .27f;
            t.rush_start_m = .2f;
            Sim s(t);
            s.in.distance_m = .5f;
            assert(s.until(PushState::APPROACH, 80));
            s.in.distance_m = .39f;
            s.tick();
            assert(s.out.motion.camera_pitch_cdeg == 2000);
            s.in.target_valid = false;
            const int gap = late ? 120 : 25;
            bool lost = false;
            for (int i = 0; i < gap; ++i) {
                s.tick();
                if (s.out.state != PushState::APPROACH) {
                    lost = true;
                    break;
                }
            }
            if (late) {
                assert(lost && s.out.state == PushState::LOST_SEARCH);
                continue;
            }
            assert(!lost);
            s.in.target_valid = true;
            s.in.target_id += 1;
            s.in.distance_m = .47f;
            for (int i = 0; i < 40; ++i) {
                s.tick();
                assert(s.out.state == PushState::APPROACH);
            }
            assert(s.out.motion.vx_mps > 0 && s.out.capture_target_id == s.in.target_id);
        }
    }
    { // 假设模式：方框张开后跟踪器换新ID，PREPARE改绑继续而不是退回SCAN。
        TaskTuning t;
        t.enable_search_cues = false;
        t.assume_all_safe = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.track_near_m = .4f;
        t.intermediate_to_near_m = .27f;
        t.rush_start_m = .2f;
        Sim s(t);
        s.in.distance_m = .5f;
        assert(s.until(PushState::APPROACH, 80));
        s.in.distance_m = .19f;
        assert(s.until(PushState::PREPARE, 80));
        s.in.target_id += 1;
        for (int i = 0; i < 40 && s.out.state == PushState::PREPARE; ++i)
            s.tick();
        assert(s.out.state != PushState::SCAN && s.out.capture_target_id == s.in.target_id);
    }
}

void approachCases() {
    { // 假设模式：20°阶段进入PREPARE/RUSH后保持20°，不为开框切40°。
        TaskTuning t;
        t.enable_search_cues = false;
        t.assume_all_safe = true;
        t.track_pitch_cdeg = 4000;
        t.near_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.track_near_m = .4f;
        t.intermediate_to_near_m = .27f;
        t.rush_start_m = .35f;
        Sim s(t);
        s.in.distance_m = .5f;
        assert(s.until(PushState::APPROACH, 80));
        s.in.distance_m = .33f;
        assert(s.until(PushState::PREPARE, 120));
        for (int i = 0; i < 30 && s.out.state == PushState::PREPARE; ++i) {
            s.tick();
            assert(s.out.motion.camera_pitch_cdeg == 2000);
        }
    }
    { // 真实模式：目标在0.33m（>intermediate_to_near）时不进PREPARE，先在20°下贴近，进入范围后才PREPARE。
        TaskTuning t;
        t.enable_search_cues = false;
        t.track_pitch_cdeg = 4000;
        t.near_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.track_near_m = .4f;
        t.intermediate_to_near_m = .27f;
        t.rush_start_m = .35f;
        Sim s(t);
        s.in.distance_m = .5f;
        assert(s.until(PushState::APPROACH, 80));
        s.in.distance_m = .33f;
        for (int i = 0; i < 40; ++i) {
            s.tick();
            assert(s.out.state == PushState::APPROACH);
        }
        assert(s.out.reason == "approach_to_prepare_range" && s.out.motion.vx_mps > 0 &&
               s.out.motion.camera_pitch_cdeg == 2000);
        s.in.distance_m = .25f;
        assert(s.until(PushState::PREPARE, 120));
    }
    { // 距离卡在0.29 m（>0.27、<=0.32）不再前进：3 s后视为到位进PREPARE，不拖到approach_timeout。
        TaskTuning t;
        t.enable_search_cues = false;
        t.track_pitch_cdeg = 4000;
        t.near_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.track_near_m = .4f;
        t.intermediate_to_near_m = .27f;
        t.rush_start_m = .35f;
        Sim s(t);
        s.in.distance_m = .5f;
        assert(s.until(PushState::APPROACH, 80));
        s.in.distance_m = .29f;
        bool stalled = false;
        for (int i = 0; i < 400 && !stalled; ++i) {
            s.tick();
            stalled = s.out.reason == "approach_range_stalled";
        }
        assert(stalled && s.out.state == PushState::PREPARE);
    }
    { // 5 -> 20 -> 40, reacquiring a new ID at each stable angle before advancing.
        TaskTuning t;
        t.enable_search_cues = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.track_near_m = .4f;
        t.intermediate_to_near_m = .2f;
        Sim s(t);
        s.in.target_is_search_cue = true;
        s.in.distance_m = .39f;
        assert(s.until(PushState::MID_REACQUIRE, 80));
        assert(s.out.motion.camera_pitch_cdeg == 2000);
        s.in.target_is_search_cue = false;
        s.in.target_id = 101;
        for (int i = 0; i < 30 && s.out.state != PushState::MID_APPROACH; ++i) {
            s.tick();
            assert(stopped(s.out.motion));
        }
        assert(s.out.state == PushState::MID_APPROACH && s.pitch == 2000 &&
               s.in.camera_pitch_stable);
        s.tick();
        assert(s.out.motion.vx_mps > 0 && s.out.motion.vx_mps <= .4f);
        s.in.distance_m = .21f;
        s.tick();
        assert(s.out.state == PushState::MID_APPROACH);
        s.in.distance_m = .19f;
        assert(s.until(PushState::NEAR_REACQUIRE, 15));
        assert(stopped(s.out.motion) && s.out.motion.gripper_offset == 0);
        assert(s.out.motion.camera_pitch_cdeg == 4000);
        s.in.target_id = 102;
        assert(s.until(PushState::APPROACH, 40));
        assert(s.out.capture_target_id == 102 && s.pitch == 4000);
        s.in.distance_m = .2f;
        s.toCarry();
        assert(s.out.motion.camera_pitch_cdeg == 500);
    }
    { // Open at 20deg/0.35m, wait for ACK, then retain open across 40deg reassociation.
        TaskTuning t;
        t.enable_search_cues = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.intermediate_to_near_m = .2f;
        Sim s(t);
        s.in.target_is_search_cue = true;
        s.in.distance_m = .39f;
        assert(s.until(PushState::MID_REACQUIRE, 80));
        s.in.target_is_search_cue = false;
        s.in.target_id = 101;
        assert(s.until(PushState::MID_APPROACH, 40));
        s.in.distance_m = .34f;
        s.gripper_responds = false;
        for (int i = 0; i < 5; ++i) {
            s.tick();
            assert(stopped(s.out.motion));
        }
        assert(s.out.motion.gripper_offset == 0 && s.out.motion.camera_pitch_cdeg == 2000);
        assert(s.out.reason == "mid_gripper_feedback_wait");
        s.gripper_responds = true;
        s.tick();
        assert(s.out.motion.vx_mps > 0);
        s.in.distance_m = .19f;
        s.tick();
        assert(s.out.state == PushState::NEAR_REACQUIRE && s.out.motion.gripper_offset == 0);
        s.in.target_id = 102;
        assert(s.until(PushState::APPROACH, 40));
        assert(s.out.motion.gripper_offset == 0 && s.out.motion.camera_pitch_cdeg == 4000);
    }
    for (int mode = 0; mode < 4; ++mode) {
        // Field 20deg setup (near switch 0.27). 0: tracked to 0.26 switches to 40deg.
        // 1: open jaw loses the box at 0.29 (image bottom) -> stopped 40deg handoff, not SAFE_STOP.
        // 2: open jaw lost at 0.34 (beyond the mouth range) -> lost-search guard SAFE_STOP.
        // 3: closed jaw lost at 0.38 -> ordinary lost search, no handoff.
        TaskTuning t;
        t.enable_search_cues = true;
        t.enable_lost_search = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.track_near_m = .4f;
        t.intermediate_to_near_m = .27f;
        Sim s(t);
        s.in.target_is_search_cue = true;
        s.in.distance_m = .39f;
        assert(s.until(PushState::MID_REACQUIRE, 80));
        s.in.target_is_search_cue = false;
        s.in.target_id = 101;
        assert(s.until(PushState::MID_APPROACH, 40));
        if (mode == 3) {
            s.in.distance_m = .38f;
            s.tick();
            assert(s.out.motion.gripper_offset == 20);
            s.in.target_valid = false;
            assert(s.until(PushState::LOST_SEARCH, 20));
            continue;
        }
        s.in.distance_m = .34f;
        for (int i = 0; i < 10 && !(s.out.motion.gripper_offset == 0 && s.out.motion.vx_mps > 0);
             ++i)
            s.tick();
        assert(s.out.motion.gripper_offset == 0 && s.out.motion.vx_mps > 0);
        if (mode == 0) {
            s.in.distance_m = .28f;
            s.tick();
            assert(s.out.state == PushState::MID_APPROACH && s.out.motion.vx_mps > 0);
            s.in.distance_m = .26f;
            s.tick();
            assert(s.out.state == PushState::NEAR_REACQUIRE && stopped(s.out.motion) &&
                   s.out.motion.gripper_offset == 0);
            continue;
        }
        if (mode == 1) {
            s.in.distance_m = .29f;
            s.tick();
            assert(s.out.state == PushState::MID_APPROACH);
        }
        s.in.target_valid = false;
        s.tick();
        if (mode == 1) {
            assert(s.out.state == PushState::NEAR_REACQUIRE && stopped(s.out.motion) &&
                   s.out.motion.gripper_offset == 0);
            assert(s.out.reason == "open_jaw_lost_at_mouth_near_handoff" &&
                   s.out.motion.camera_pitch_cdeg == 4000);
            s.in.target_valid = true;
            s.in.target_id = 102;
            s.in.distance_m = .22f;
            assert(s.until(PushState::APPROACH, 40) && s.out.motion.gripper_offset == 0);
        } else {
            assert(s.until(PushState::SAFE_STOP, 20) &&
                   s.out.reason == "lost_search_jaw_or_load_unsafe");
        }
    }
    { // Aligned approach drives straight: a small bearing is not lifted to min_turn_wz.
        Sim s;
        s.in.distance_m = .8f;
        s.in.heading_error = .03f;
        assert(s.until(PushState::APPROACH, 30));
        s.tick();
        assert(s.out.motion.vx_mps > 0 && s.out.motion.wz_rps == 0);
        s.in.heading_error = -.05f;
        s.tick();
        assert(s.out.motion.vx_mps > 0 && s.out.motion.wz_rps == 0);
        s.in.heading_error = .1f;
        s.tick();
        assert(s.out.motion.vx_mps > 0 && s.out.motion.wz_rps == .4f);
    }
    for (int mode = 0; mode < 3; ++mode) {
        // Open jaw, corridor changes: 0 one-frame flicker continues, 1 confirmed dangerous
        // object stops, 2 confirmed second ordinary on the first trip is carried as a new batch.
        TaskTuning t;
        t.enable_search_cues = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.intermediate_to_near_m = .2f;
        Sim s(t);
        s.in.target_is_search_cue = true;
        s.in.distance_m = .39f;
        assert(s.until(PushState::MID_REACQUIRE, 80));
        s.in.target_is_search_cue = false;
        s.in.target_id = 101;
        assert(s.until(PushState::MID_APPROACH, 40));
        s.in.distance_m = .34f;
        for (int i = 0; i < 10 && !(s.out.motion.gripper_offset == 0 && s.out.motion.vx_mps > 0);
             ++i)
            s.tick();
        assert(s.out.motion.gripper_offset == 0 && s.out.motion.vx_mps > 0);
        s.in.corridor = mode == 1 ? inv(1, 0, 0, 1) : inv(2);
        s.tick();
        assert(s.out.state == PushState::MID_APPROACH && stopped(s.out.motion) &&
               s.out.reason == "open_jaw_corridor_recheck");
        if (mode == 0) {
            s.in.corridor = inv(1);
            s.tick();
            assert(s.out.state == PushState::MID_APPROACH && s.out.motion.vx_mps > 0 &&
                   s.out.motion.gripper_offset == 0);
            continue;
        }
        s.tick();
        assert(stopped(s.out.motion));
        s.tick();
        if (mode == 1) {
            assert(s.out.state == PushState::SAFE_STOP &&
                   s.out.reason == "open_jaw_corridor_changed");
            assert(s.out.verdict == RuleVerdict::DANGEROUS && stopped(s.out.motion));
        } else {
            assert(s.out.state == PushState::MID_APPROACH && s.out.motion.vx_mps > 0 &&
                   s.out.batch_size == 2);
            s.tick();
            assert(s.out.motion.vx_mps > 0); // new batch accepted, no recheck
        }
    }
    { // A blocked closed-jaw approach stops immediately and rejects its cargo ID.
        TaskTuning t;
        t.enable_search_cues = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        Sim s(t);
        s.in.target_is_search_cue = true;
        s.in.distance_m = .39f;
        assert(s.until(PushState::MID_REACQUIRE, 80));
        s.in.target_is_search_cue = false;
        s.in.target_id = 101;
        assert(s.until(PushState::MID_APPROACH, 40));
        s.in.path_safe = false;
        s.tick();
        assert(s.out.state == PushState::MID_REACQUIRE && stopped(s.out.motion));
        const auto rejected = s.task.rejectedTargets(s.in.now_us);
        assert(std::find(rejected.begin(), rejected.end(), 101) != rejected.end());
    }
    { // 张框距离内航向略超限：底盘小wz(<0.5)转不动(实测)，改用0.55rad/s按IMU量角的短脉冲，转到位再张框；原地小wz会一直卡住。
        struct R {
            bool prepared = false, blocked_moved = false;
            int pulse_ticks = 0;
            float first_wz = 0, max_dev = 0;
            bool bad_wz = false;
        };
        const auto run = [&](float e0, bool pulse, int ticks, bool turn_blocked) {
            TaskTuning t;
            t.rush_pulse_turn = pulse;
            Sim s(t);
            s.in.distance_m = .3f;
            s.in.heading_error = e0;
            assert(s.until(PushState::APPROACH, 30));
            if (turn_blocked) {
                s.in.directional_clearance_valid = true;
                s.in.turn_safe = false;
                s.in.arc_safe = false;
            }
            const float p0 = s.phi;
            R r;
            for (int i = 0; i < ticks && !r.prepared; ++i) {
                s.in.heading_error = e0 - (s.phi - p0);
                s.tick();
                if (s.out.state == PushState::PREPARE) {
                    r.prepared = true;
                    break;
                }
                assert(s.out.state == PushState::APPROACH);
                if (s.out.reason == "rush_pulse_turn" && s.out.motion.wz_rps != 0) {
                    ++r.pulse_ticks;
                    if (!r.first_wz)
                        r.first_wz = s.out.motion.wz_rps;
                    if (std::abs(s.out.motion.wz_rps) != t.rush_pulse_wz ||
                        s.out.motion.vx_mps != 0)
                        r.bad_wz = true;
                }
                if (turn_blocked && s.out.motion.wz_rps != 0)
                    r.blocked_moved = true;
                r.max_dev = std::max(r.max_dev, std::abs(s.phi - p0));
                if (std::abs(s.out.motion.wz_rps) < .5f)
                    s.out.motion.wz_rps = 0; // base dead band
            }
            return r;
        };
        auto stuck = run(.10f, false, 200, false);
        assert(!stuck.prepared && stuck.pulse_ticks == 0 &&
               stuck.max_dev < 1e-4f); // old behaviour: commands 0.4, base never turns
        auto left = run(.10f, true, 400, false);
        assert(left.prepared && left.pulse_ticks > 0 && left.first_wz > 0 && !left.bad_wz &&
               left.max_dev > .07f && left.max_dev < .14f);
        auto right = run(-.15f, true, 400, false);
        assert(right.prepared && right.first_wz < 0 && !right.bad_wz && right.max_dev > .11f &&
               right.max_dev < .20f);
        auto big =
            run(.30f,
                true,
                100,
                false); // beyond rush_pulse_max_rad: legacy rotate-in-place command, no pulse
        assert(big.pulse_ticks == 0);
        auto blocked = run(.10f, true, 100, true); // unsafe turn stays blocked even for a pulse
        assert(!blocked.prepared && !blocked.blocked_moved);
    }
    { // Forward permission alone cannot authorize an unsafe turn or curved approach.
        Sim s;
        s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 30));
        s.in.directional_clearance_valid = true;
        s.in.turn_safe = false;
        s.in.arc_safe = false;
        s.in.heading_error = .6f;
        s.tick();
        assert(stopped(s.out.motion) &&
               s.out.reason == "turn_sweep_blocked"); // in-place turn stays blocked
        s.in.heading_error = .1f;
        s.tick(); // blocked arc with a verified straight corridor keeps going straight
        assert(s.out.motion.vx_mps > 0 && s.out.motion.wz_rps == 0 &&
               s.out.reason == "turn_sweep_blocked_straight");
        s.in.path_safe = false;
        s.tick();
        assert(stopped(s.out.motion)); // no straight corridor: stop
        s.in.path_safe = true;
        s.in.heading_error = 0;
        s.tick();
        assert(s.out.motion.vx_mps > 0);
    }
    { // Both signs escape the yaw deadband, zero and safety stops remain zero.
        Sim s;
        s.in.distance_m = .3f;
        s.in.heading_error = .07f;
        assert(s.until(PushState::APPROACH, 30));
        s.tick();
        assert(s.out.motion.vx_mps == 0 && s.out.motion.wz_rps == .4f);
        s.in.heading_error = -.07f;
        s.tick();
        assert(s.out.motion.wz_rps == -.4f);
        s.in.path_safe = false;
        s.tick();
        assert(stopped(s.out.motion));
        s.in.path_safe = true;
        s.in.heading_error = 0;
        s.tick();
        assert(stopped(s.out.motion));
        assert(s.out.state == PushState::PREPARE);
    }
    for (bool lose_feedback : {false, true}) {
        TaskTuning t;
        t.enable_search_cues = true;
        t.enable_short_push = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        Sim s(t);
        s.in.target_is_search_cue = true;
        s.in.distance_m = .39f;
        assert(s.until(PushState::MID_REACQUIRE, 80));
        s.in.target_is_search_cue = false;
        assert(s.until(PushState::MID_APPROACH, 40));
        s.in.distance_m = .34f;
        s.in.corridor = inv(1, 0, 0, 1);
        s.in.clear_push_safe = true;
        assert(s.until(PushState::CLEAR_PILE, 15));
        if (lose_feedback) {
            s.gripper_responds = false;
            s.tick();
            assert(s.out.state == PushState::SAFE_STOP && stopped(s.out.motion));
        } else {
            int moving = 0;
            for (int i = 0; i < 12 && s.out.state != PushState::CLEAR_PAUSE; ++i) {
                s.in.distance_m = std::max(.19f, s.in.distance_m - .04f);
                s.tick();
                assert(s.out.motion.gripper_offset == 20 && s.out.motion.wz_rps == 0 &&
                       s.out.delivered_total == 0);
                if (s.out.motion.vx_mps > 0) {
                    assert(s.out.motion.vx_mps == .4f);
                    ++moving;
                }
            }
            assert(moving > 0 && moving <= 6 && s.out.state == PushState::CLEAR_PAUSE &&
                   s.out.clearing_attempts == 1);
            assert(s.out.reason == "clear_target_distance_reached");
            assert(s.until(PushState::MID_REACQUIRE, 15));
        }
    }
    { // A pushed object maintaining its distance must never cause unlimited forward travel.
        TaskTuning t;
        t.enable_search_cues = true;
        t.enable_short_push = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        Sim s(t);
        s.in.target_is_search_cue = true;
        s.in.distance_m = .39f;
        assert(s.until(PushState::MID_REACQUIRE, 80));
        s.in.target_is_search_cue = false;
        assert(s.until(PushState::MID_APPROACH, 40));
        s.in.distance_m = .34f;
        s.in.corridor = inv(1, 0, 0, 1);
        s.in.clear_push_safe = true;
        assert(s.until(PushState::CLEAR_PILE, 15));
        assert(s.until(PushState::SAFE_STOP, 40));
        assert(s.out.reason == "clear_distance_not_reached" && stopped(s.out.motion));
    }
    { // Repeated blocked paths reselect at rest instead of latching an attempt fault.
        TaskTuning t;
        t.enable_search_cues = true;
        t.enable_lost_search = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        Sim s(t);
        s.in.target_is_search_cue = true;
        s.in.distance_m = .39f;
        assert(s.until(PushState::MID_REACQUIRE, 80));
        s.in.target_is_search_cue = false;
        assert(s.until(PushState::MID_APPROACH, 40));
        for (int attempt = 0; attempt < 5; ++attempt) {
            s.in.path_safe = false;
            s.tick();
            assert(s.out.state == PushState::MID_REACQUIRE && stopped(s.out.motion));
            assert(s.out.reason == "blocked_cargo_reselect");
            s.in.path_safe = true;
            ++s.in.target_id;
            assert(s.until(PushState::MID_APPROACH, 40));
        }
    }
    { // Open-jaw corridor rejected by neighbours: small closed-frame swings first; the old reselect
      // path only after the budget is used.
        TaskTuning t;
        t.enable_search_cues = true;
        t.enable_lost_search = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        Sim s(t);
        s.in.target_is_search_cue = true;
        s.in.distance_m = .39f;
        assert(s.until(PushState::MID_REACQUIRE, 80));
        s.in.target_is_search_cue = false;
        assert(s.until(PushState::MID_APPROACH, 40));
        for (int round = 0; round < 2; ++round) {
            s.in.distance_m = .30f;
            s.in.corridor = inv(1, 0, 1);
            for (int i = 0; i < 10 && s.out.reason != "mid_neighbour_clear"; ++i)
                s.tick();
            assert(s.out.reason == "mid_neighbour_clear" && s.out.motion.gripper_offset == 20);
            bool swung = false;
            float max_dyaw = 0;
            const float yaw0 = s.phi;
            for (int i = 0; i < 400 && s.out.reason != "mid_neighbour_clear_done"; ++i) {
                s.tick();
                assert(s.out.state != PushState::SAFE_STOP && s.out.motion.gripper_offset == 20);
                if (s.out.motion.wz_rps != 0)
                    swung = true;
                max_dyaw = std::max(max_dyaw, std::abs(s.phi - yaw0));
            }
            assert(s.out.reason == "mid_neighbour_clear_done" &&
                   s.out.state == PushState::MID_REACQUIRE && swung);
            assert(max_dyaw > .3f && max_dyaw < .8f);
            s.in.corridor = inv(1);
            assert(s.until(PushState::MID_APPROACH, 40));
        }
        s.in.distance_m = .30f;
        s.in.corridor = inv(1, 0, 1);
        for (int i = 0; i < 10 && s.out.reason != "mid_open_corridor_reselect"; ++i)
            s.tick();
        assert(s.out.reason == "mid_open_corridor_reselect" &&
               s.out.state == PushState::MID_REACQUIRE);
    }
    { // Lost closed-jaw target: zero translation, bounded turn, new-ID confirmation.
        TaskTuning t;
        t.enable_search_cues = true;
        t.enable_lost_search = true;
        Sim s(t);
        s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 30));
        s.in.target_valid = false;
        assert(s.until(PushState::LOST_SEARCH, 20));
        s.tick();
        assert(s.out.motion.vx_mps == 0 && s.out.motion.wz_rps == .4f);
        s.in.directional_clearance_valid = true;
        s.in.turn_safe = false;
        s.tick();
        assert(stopped(s.out.motion));
        s.in.turn_safe = true;
        s.in.target_valid = true;
        s.in.target_id = 99;
        assert(s.until(PushState::CUE_APPROACH, 5));
        assert(stopped(s.out.motion) && s.out.target_id == 99);
    }
    { // Repeated successful reacquisitions do not exhaust a lifetime attempt budget.
        TaskTuning t;
        t.enable_search_cues = true;
        t.enable_lost_search = true;
        Sim s(t);
        s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 30));
        for (int attempt = 0; attempt < 5; ++attempt) {
            s.in.target_valid = false;
            assert(s.until(PushState::LOST_SEARCH, 20));
            s.tick();
            assert(s.out.motion.vx_mps == 0 && s.out.motion.wz_rps == .4f);
            s.in.target_valid = true;
            s.in.target_id = 100 + attempt;
            assert(s.until(PushState::CUE_APPROACH, 5));
            assert(stopped(s.out.motion) && s.out.target_id == 100 + attempt);
        }
    }
    { // Searching never becomes endless when no valid target is seen.
        TaskTuning t;
        t.enable_lost_search = true;
        t.lost_search_rescan = false;
        Sim s(t);
        s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 30));
        s.in.target_valid = false;
        assert(s.until(PushState::LOST_SEARCH, 20));
        assert(s.until(PushState::SAFE_STOP, 430));
        assert(s.out.reason == "lost_search_exhausted" && stopped(s.out.motion));
    }
    { // 默认：两轮丢失搜索都没找到，不再整场停死，而是回到SCAN继续找，之后再看到目标仍能进入APPROACH。
        TaskTuning t;
        t.enable_lost_search = true;
        t.enable_search_cues = false;
        t.blind_scatter = false;
        Sim s(t);
        s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 30));
        s.in.target_valid = false;
        assert(s.until(PushState::LOST_SEARCH, 20));
        bool rescan = false;
        for (int i = 0; i < 1200 && !rescan; ++i) {
            s.tick();
            rescan = s.out.reason == "lost_search_rescan";
            assert(s.out.state != PushState::SAFE_STOP);
        }
        assert(rescan && s.out.state == PushState::SCAN);
        s.in.target_valid = true;
        s.in.target_id = 77;
        s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 40));
    }
    { // FAR sweep finds nothing: stop, tilt to 20deg, sweep once more, then stop for good.
        TaskTuning t;
        t.enable_lost_search = true;
        t.lost_search_rescan = false;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        Sim s(t);
        s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 80));
        s.in.target_valid = false;
        assert(s.until(PushState::LOST_SEARCH, 20));
        bool far_turn = false;
        for (int i = 0; i < 430 && s.out.reason != "lost_search_mid_pitch"; ++i) {
            s.tick();
            far_turn |= s.out.motion.wz_rps != 0 && s.out.motion.camera_pitch_cdeg == 500;
        }
        assert(far_turn && s.out.state == PushState::LOST_SEARCH && stopped(s.out.motion) &&
               s.out.motion.camera_pitch_cdeg == 2000);
        bool mid_turn = false;
        for (int i = 0; i < 430 && s.out.state == PushState::LOST_SEARCH; ++i) {
            s.tick();
            if (s.out.motion.wz_rps != 0) {
                assert(s.pitch == 2000 && s.out.motion.vx_mps == 0);
                mid_turn = true;
            }
        }
        assert(mid_turn && s.out.state == PushState::SAFE_STOP &&
               s.out.reason == "lost_search_exhausted" && stopped(s.out.motion));
    }
    { // Close cargo seen in the 20deg sweep keeps the 20deg view (MID_REACQUIRE), not FAR.
        TaskTuning t;
        t.enable_lost_search = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        Sim s(t);
        s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 80));
        s.in.target_valid = false;
        assert(s.until(PushState::LOST_SEARCH, 20));
        for (int i = 0; i < 430 && s.out.reason != "lost_search_mid_pitch"; ++i)
            s.tick();
        assert(s.out.reason == "lost_search_mid_pitch");
        for (int i = 0; i < 200 && s.out.reason != "rotating_for_lost_target_mid"; ++i)
            s.tick();
        assert(s.out.reason == "rotating_for_lost_target_mid" && s.pitch == 2000);
        s.in.target_valid = true;
        s.in.target_id = 77;
        s.in.distance_m = .3f;
        assert(s.until(PushState::MID_REACQUIRE, 5));
        assert(stopped(s.out.motion) && s.out.motion.camera_pitch_cdeg == 2000 &&
               s.out.reason == "lost_target_reacquired_mid");
        assert(s.until(PushState::MID_APPROACH, 40));
    }
    { // Stop before replacing a blue cue with newly visible green; then confirm its lock.
        TaskTuning t;
        t.enable_search_cues = true;
        Sim s(t);
        s.in.label = "dangerous_object";
        s.in.target_is_search_cue = true;
        s.in.distance_m = .8f;
        assert(s.until(PushState::CUE_APPROACH, 40));
        s.in.label = "ordinary_supply";
        s.in.target_id = 99;
        s.tick();
        assert(s.out.state == PushState::SCAN && stopped(s.out.motion) &&
               s.out.reason == "green_priority_reselect");
        assert(s.until(PushState::CUE_APPROACH, 5));
        assert(s.out.target_id == 99);
    }
    { // A detector cannot activate cue navigation when the feature is disabled.
        Sim s;
        s.in.label = "dangerous_object";
        s.in.target_is_search_cue = true;
        for (int i = 0; i < 40; ++i)
            s.tick();
        assert(s.out.state == PushState::SCAN);
    }
    { // Requested startup duration/speed and angular cap must reach real task output.
        TaskTuning t;
        t.startup_advance_us = 15000000;
        t.startup_advance_speed = 1.f;
        t.max_speed = 1.f;
        t.scan_wz = t.turn_wz = t.max_wz = 3.f;
        PushTask task(t);
        PushObservation in;
        in.run = in.safety_ok = in.path_safe = in.opponent_zone_clear = true;
        in.camera_pitch_cdeg = 500;
        in.camera_pitch_stable = true;
        int forward_frames = 0;
        PushOutput out;
        for (int i = 0; i < 440; ++i) {
            in.now_us += 100000;
            in.path_safe = !(i >= 30 && i < 40);
            out = task.update(in);
            if (out.motion.vx_mps > 0) {
                ++forward_frames;
                assert(out.motion.vx_mps == 1.f && out.motion.wz_rps == 0);
            }
            if (out.state == PushState::SCAN)
                break;
        }
        assert(forward_frames == 150 && out.startup_commanded_us == 15000000);
        in.now_us += 100000;
        out = task.update(in);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 3.f);
        assert(out.reason == "searching_target");
        in.path_safe = false;
        in.now_us += 100000;
        out = task.update(in);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0 &&
               out.reason == "scan_path_blocked");
        in.path_safe = true;
        in.now_us += 100000;
        out = task.update(in);
        assert(out.motion.wz_rps == 3.f);
    }
    { // CLI speed input rejects out-of-contract values and trailing garbage.
        const auto parse = [](std::vector<std::string> values) {
            std::vector<char*> argv;
            for (auto& v : values)
                argv.push_back(v.data());
            return parseArgs(int(argv.size()), argv.data());
        };
        auto c = parse({"test",
                        "--startup-speed",
                        "0.2",
                        "--scan-wz",
                        "3",
                        "--turn-wz",
                        "3",
                        "--startup-advance-ms",
                        "40000"});
        assert(c.startup_speed_mps == .2f && c.scan_wz_rps == 3 && c.turn_wz_rps == 3 &&
               c.startup_advance_ms == 40000);
        for (const auto& args :
             std::vector<std::vector<std::string>>{{"test", "--startup-speed", "1.01"},
                                                   {"test", "--scan-wz", "nan"},
                                                   {"test", "--turn-wz", "1junk"},
                                                   {"test", "--scan-wz", "3.1"},
                                                   {"test", "--startup-speed", "0"}}) {
            bool rejected = false;
            try {
                parse(args);
            } catch (const std::exception&) {
                rejected = true;
            }
            assert(rejected);
        }
    }
}
