#include "../../support/push_task_sim.hpp"
#include "push_task_cases.hpp"

void blindRetreatTests() {
    // Rush `ticks` frames from 0.3 m, then close on nothing; the zone stays out of view.
    const auto failAfterRush = [](Sim& s, int ticks) {
        s.in.distance_m = .3f;
        s.toRush();
        s.zone_visible = false;
        for (int i = 0; i < ticks; ++i) {
            s.tick();
            assert(s.out.state == PushState::RUSH);
        }
        s.in.distance_m = .19f;
        assert(s.until(PushState::CAPTURE_FAIL, 150) && s.out.reason == "capture_unverified");
    };
    for (int ticks :
         {12, 2}) { // reverse straight by min(abort_back_m, forward since the lock), then rescan
        Sim s;
        failAfterRush(s, ticks);
        const float forward = s.py + .75f, y0 = s.py, phi0 = s.phi;
        bool reversed = false;
        for (int i = 0; i < 400 && s.out.state != PushState::SCAN; ++i) {
            s.tick();
            assert(s.out.state != PushState::SAFE_STOP);
            if (s.out.motion.vx_mps < 0) {
                reversed = true;
                assert(s.out.motion.wz_rps == 0 && s.out.motion.gripper_offset == 0);
            }
        }
        assert(reversed && s.out.state == PushState::SCAN &&
               s.out.reason == "blind_retreat_complete");
        assert(std::abs(s.phi - phi0) < 1e-6f);
        const float expected = std::min(s.tune.abort_back_m, forward);
        assert(std::abs((y0 - s.py) - expected) < .01f);
        if (ticks == 2)
            assert(expected < s.tune.abort_back_m);
    }
    { // Nothing driven forward since the lock: nothing to retrace, rescan without reversing.
        Sim s;
        s.toRush();
        s.zone_visible = false;
        assert(s.until(PushState::CAPTURE_FAIL, 150));
        const float y0 = s.py;
        assert(s.until(PushState::SCAN, 400) && s.out.reason == "blind_retreat_complete" &&
               std::abs(s.py - y0) < .005f);
    }
    { // Heading drift while reversing blind stops the robot.
        Sim s;
        failAfterRush(s, 12);
        for (int i = 0; i < 200 && s.out.motion.vx_mps >= 0; ++i)
            s.tick();
        assert(s.out.motion.vx_mps < 0);
        s.phi += .3f;
        s.tick();
        assert(s.out.state == PushState::SAFE_STOP &&
               s.out.reason == "blind_retreat_heading_drift" && stopped(s.out.motion));
    }
    { // No IMU heading: no blind retreat, explicit stop.
        Sim s;
        failAfterRush(s, 12);
        s.in.heading_valid = false;
        for (int i = 0; i < 200 && s.out.state != PushState::SAFE_STOP; ++i) {
            s.tick();
            assert(s.out.motion.vx_mps >= 0);
        }
        assert(s.out.state == PushState::SAFE_STOP &&
               s.out.reason == "retreat_unverified_no_heading");
    }
    { // Zone visible: the measured retreat is unchanged.
        Sim s;
        s.in.distance_m = .3f;
        s.toRush();
        for (int i = 0; i < 12; ++i)
            s.tick();
        s.in.distance_m = .19f;
        assert(s.until(PushState::CAPTURE_FAIL, 150));
        const float y0 = s.py;
        assert(s.until(PushState::SCAN, 400) && s.out.reason != "blind_retreat_complete");
        assert(y0 - s.py >= s.tune.abort_back_m - .01f);
    }
}
// Regressions for the 01:31 run: tiny-error weaving, moving distance bound,
// blind continuation and sweep forward creep. Synthetic observations, no hardware.

void navigationReadinessCases() {
    {
        Sim s;
        s.in.target_region_valid = false;
        for (int i = 0; i < 60; ++i)
            assert(s.tick().state != PushState::APPROACH);
    }
    {
        Sim s;
        s.in.opponent_zone_clear = false;
        s.in.target_valid = false;
        for (int i = 0; i < 20; ++i)
            assert(stopped(s.tick().motion));
    }
    {
        Sim s;
        s.toCarry();
        s.in.zone_identity_verified = false;
        for (int i = 0; i < 20; ++i) {
            auto o = s.tick();
            assert(o.motion.vx_mps == 0 && o.state != PushState::GATE);
        }
    }
    {
        Sim s;
        s.toCarry();
        s.navigation_ready = false;
        for (int i = 0; i < 30; ++i)
            assert(s.tick().motion.vx_mps == 0);
    }
    {
        Sim s;
        s.toCarry();
        assert(s.until(PushState::RAISE_RELEASE, 400));
        s.drop_available = false;
        for (int i = 0; i < 10; ++i) {
            auto o = s.tick();
            assert(stopped(o.motion) && o.motion.gripper_offset == 20);
        }
    }
}

void cameraStartupCases() {
    { // Live-loop guard permits camera work only at zero speed in explicit phases.
        PushOutput out;
        out.state = PushState::LOWER_FRAME;
        out.stationary_pitch_work = true;
        assert(stationaryPitchWork(out, "pitch_moving"));
        assert(stationaryPitchWork(out, "pitch_not_calibrated"));
        assert(!stationaryPitchWork(out, "imu_not_synchronized"));
        assert(!stationaryPitchWork(out, "stale_frame"));
        out.motion.vx_mps = .1f;
        assert(!stationaryPitchWork(out, "pitch_moving"));
        inhibitUnmappedMotion(out.motion, false);
        assert(stopped(out.motion));
        out.state = PushState::SCAN;
        out.stationary_pitch_work = false;
        out.scan_pitch_return = true;
        out.motion.camera_pitch_cdeg = 500;
        assert(stationaryPitchWork(out, "pitch_moving"));
        assert(!stationaryPitchWork(out, "pitch_not_calibrated"));
        out.motion.wz_rps = .3f;
        assert(!stationaryPitchWork(out, "pitch_moving"));
        out.motion.wz_rps = 0;
        out.motion.camera_pitch_cdeg = kCameraPitchInvalid;
        assert(!stationaryPitchWork(out, "pitch_moving"));
    }
    for (bool stalled :
         {false, true}) { // Actual failure: lose a cue at 40deg, return to SCAN at 5deg.
        TaskTuning t;
        t.enable_search_cues = true;
        t.track_pitch_cdeg = 4000;
        Sim s(t);
        s.in.target_is_search_cue = true;
        s.in.distance_m = .55f;
        assert(s.until(PushState::CUE_APPROACH, 80));
        for (int i = 0; i < 20; ++i)
            s.tick();
        assert(s.pitch == 4000);
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 90));
        assert(stopped(s.out.motion) && s.out.motion.camera_pitch_cdeg == 500);
        s.servo_responds = !stalled;
        bool waited = false, rotated = false;
        for (int i = 0; i < 65; ++i) {
            const auto prior = s.out;
            s.tick();
            if (!s.in.camera_pitch_stable) {
                waited = true;
                assert(stationaryPitchWork(prior, "pitch_moving"));
                assert(stopped(s.out.motion));
            }
            if (s.out.motion.wz_rps != 0) {
                assert(s.pitch == 500 && s.in.camera_pitch_stable);
                rotated = true;
                break;
            }
            if (s.out.state == PushState::SAFE_STOP)
                break;
        }
        if (stalled)
            assert(s.out.state == PushState::SAFE_STOP && s.out.reason == "camera_pitch_timeout");
        else
            assert(waited && rotated && s.out.state == PushState::SCAN);
    }
    { // Exercise complete 5 -> 40 -> 5 cycles with live-loop mapping rejection.
        Sim s;
        s.live_camera_guard = true;
        s.pitch = 500;
        s.pitch_still = 3;
        s.out.motion.camera_pitch_cdeg = 500;
        s.toCarry();
        s.deliver();
        assert(s.out.delivered_total == 1);
    }
    { // Full first trip: no target or zone at the start, rotate, approach from
      // farther away, capture, then search for a zone that is still out of view.
        TaskTuning t;
        t.carry_budget_us = 90000000;
        Sim s(t);
        s.live_camera_guard = true;
        s.pitch = 500;
        s.pitch_still = 3;
        s.out.motion.camera_pitch_cdeg = 500;
        s.zone_visible = false;
        s.in.target_valid = false;
        bool scanned = false;
        for (int i = 0; i < 30; ++i) {
            s.tick();
            assert(s.out.motion.vx_mps == 0);
            scanned |= s.out.motion.wz_rps > 0;
        }
        assert(scanned && s.out.state == PushState::SCAN);
        s.in.target_valid = true;
        s.in.distance_m = .8f;
        bool approached = false;
        for (int i = 0; i < 300 && s.out.state != PushState::RUSH; ++i) {
            s.in.distance_m -= std::max(0.f, s.out.motion.vx_mps) * .05f;
            s.tick();
            approached |= s.out.state == PushState::APPROACH && s.out.motion.vx_mps > 0;
        }
        assert(approached && s.out.state == PushState::RUSH);
        s.hold(inv(1));
        for (int i = 0; i < 100 && s.out.state != PushState::CARRY; ++i) {
            s.in.distance_m -= std::max(0.f, s.out.motion.vx_mps) * .05f;
            s.tick();
        }
        assert(s.out.state == PushState::CARRY);
        bool searched = false;
        for (int i = 0; i < 550; ++i) {
            s.tick();
            assert(s.out.state == PushState::CARRY && s.out.motion.vx_mps == 0 &&
                   s.out.motion.gripper_offset == 20);
            searched |= s.out.motion.wz_rps > 0 && s.out.reason == "searching_own_zone";
        }
        assert(searched); // >25s with no zone no longer aborts the requested first-trip search
        s.zone_visible = true;
        assert(s.until(PushState::GATE, 1200));
        s.deliver();
        assert(s.out.delivered_total == 1 && s.out.first_ordinary_delivered);
        assert(s.until(PushState::SCAN, 400));
    }
    { // Missing zone eventually stops with the gripper closed, never a guessed route.
        TaskTuning t;
        t.carry_budget_us = 90000000;
        t.zone_search_budget_us = 2000000;
        Sim s(t);
        s.toCarry();
        s.zone_visible = false;
        assert(s.until(PushState::SAFE_STOP, 100));
        assert(s.out.reason == "own_zone_search_timeout" && stopped(s.out.motion) &&
               s.out.motion.gripper_offset == 20);
    }
    { // Startup is deliberately deferred: old duration settings cannot command motion.
        TaskTuning t;
        t.startup_advance_us = 10000000;
        PushTask task(t);
        PushObservation in;
        in.run = in.safety_ok = in.path_safe = in.opponent_zone_clear = true;
        in.camera_pitch_cdeg = 500;
        in.camera_pitch_stable = true;
        in.now_us = 100000;
        auto out = task.update(in);
        assert(out.phase == TaskPhase::Startup && stopped(out.motion));
        in.now_us += 100000;
        out = task.update(in);
        assert(out.phase == TaskPhase::Search && out.state == PushState::SCAN);
        assert(stopped(out.motion) && out.startup_commanded_us == 0);
        assert(out.reason == "startup_deferred");
        in.run = false;
        in.now_us += 100000;
        task.update(in);
        in.run = true;
        in.now_us += 100000;
        out = task.update(in);
        assert(out.phase == TaskPhase::Search && stopped(out.motion));
    }
    for (const auto* label :
         {"ordinary_supply", "injured_person", "core_supply", "dangerous_object"}) {
        TaskTuning t;
        t.enable_search_cues = true;
        Sim s(t);
        s.in.label = label;
        s.in.target_is_search_cue = true;
        s.in.distance_m = .34f;
        assert(s.until(PushState::SELECT_CARGO, 80));
        assert(stopped(s.out.motion) && s.out.motion.gripper_offset == 20);
        assert(s.out.search_cue_id == 7 && s.out.capture_target_id == -1 &&
               !s.out.cue_contact_allowed);
        s.in.label = "ordinary_supply";
        s.in.target_id = 9;
        s.in.target_is_search_cue = false;
        s.in.distance_m = .7f;
        assert(s.until(PushState::APPROACH, 10));
        assert(s.out.search_cue_id == 7 && s.out.capture_target_id == 9 && s.out.target_id == 9);
        s.tick();
        assert(s.out.state == PushState::APPROACH && s.out.motion.gripper_offset == 20);
        s.in.distance_m = .2f;
        s.toCarry();
        s.deliver();
        assert(s.out.first_ordinary_delivered && s.out.delivered_total == 1);
        assert(!s.visited.count(PushState::CLEAR_PILE));
    }
}

void captureSafetyCases() {
    { // Stale frame or safety veto stops; resuming keeps the verified load.
        Sim s;
        s.toCarry();
        s.tick();
        s.in.safety_ok = false;
        s.tick();
        assert(s.out.state == PushState::WAIT_START && stopped(s.out.motion) &&
               s.out.batch_size == 1);
        assert(s.out.motion.gripper_offset == 20); // a stop never opens the frame
        s.in.safety_ok = true;
        s.tick();
        assert(s.out.state == PushState::CARRY && s.out.batch_size == 1);
        s.in.now_us += 300000;
        s.tick();
        assert(s.out.state == PushState::WAIT_START && stopped(s.out.motion));
        s.tick();
        assert(s.out.state == PushState::CARRY);
        s.deliver();
        assert(s.out.delivered_total == 1 && s.out.first_ordinary_delivered);
    }
    { // Same timestamp twice is not a fresh frame.
        Sim s;
        s.toRush();
        auto out = s.task.update(s.in);
        assert(out.state == PushState::WAIT_START && stopped(out.motion));
    }
    { // A pause during the rush releases rather than guessing what is enclosed.
        Sim s;
        s.toRush();
        s.tick();
        s.in.run = false;
        s.tick();
        s.in.run = true;
        s.tick();
        assert(s.out.state == PushState::ABORT_DROP);
    }
    { // Invalid geometry while approaching: no motion toward a guessed position.
        Sim s;
        s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 10));
        s.in.distance_m = std::numeric_limits<float>::quiet_NaN();
        s.tick();
        assert(s.out.motion.vx_mps == 0);
    }
    { // Reset clears the delivery record and the first-delivery permission.
        Sim s;
        s.toCarry();
        s.deliver();
        assert(s.out.first_ordinary_delivered);
        s.in.reset = true;
        s.tick();
        s.in.reset = false;
        assert(s.out.delivered_total == 0 && !s.out.first_ordinary_delivered &&
               s.out.state == PushState::WAIT_START);
    }
    { // A gripper that never acknowledges is a fault, not a guess.
        Sim s;
        s.gripper_responds = false;
        assert(s.until(PushState::SAFE_STOP, 100) && s.out.reason == "gripper_open_timeout");
        for (int i = 0; i < 5; ++i)
            assert(stopped(s.tick().motion));
        s.in.reset = true;
        s.tick();
        assert(s.out.state == PushState::WAIT_START);
    }
    { // The tracker re-IDs the same cargo (same label, near the last bearing and range): rebind
      // instead of searching.
        Sim s;
        s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 20));
        s.tick();
        s.in.target_id = 8;
        s.tick();
        assert(s.out.state == PushState::APPROACH &&
               s.out.reason == "approach_id_rebound_same_object" && s.out.target_id == 8);
    }
    { // The body turned .4 rad (e.g. a rush pulse) between the last tracked frame and the re-ID:
      // the bearing shifts by the same amount, so the world bearing is unchanged and the new ID is
      // still the same object.
        Sim s;
        s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 20));
        s.tick();
        s.in.target_id = 8;
        s.phi += .4f;
        s.in.heading_error -= .4f;
        s.tick();
        assert(s.out.state == PushState::APPROACH &&
               s.out.reason == "approach_id_rebound_same_object" && s.out.target_id == 8);
    }
    { // Other detections do not authorize motion after the locked ID changes.
        Sim s;
        s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 20));
        s.tick();
        const float h0 = s.in.heading_error;
        s.in.target_id = 8;
        s.in.heading_error = h0 + .8f;
        s.tick();
        assert(stopped(s.out.motion) && s.out.reason == "target_id_mismatch");
        s.in.target_id = 7;
        s.in.heading_error = h0;
        s.in.geometry_valid = false;
        s.tick();
        assert(stopped(s.out.motion) && s.out.reason == "target_geometry_invalid");
        s.in.geometry_valid = true;
        s.in.path_safe = false;
        s.tick();
        assert(stopped(s.out.motion) && s.out.reason == "forward_path_blocked");
    }
    { // Waiting for an open ACK is distinguishable from target or path failure.
        Sim s;
        s.gripper_responds = false;
        assert(s.until(PushState::PREPARE, 30));
        bool saw_wait = false;
        for (int i = 0; i < 45 && s.out.state == PushState::PREPARE; ++i) {
            s.tick();
            saw_wait = saw_wait || s.out.reason == "gripper_feedback_wait";
            assert(stopped(s.out.motion));
        }
        assert(saw_wait);
    }
    { // Entering PREPARE must open in the same stationary output, not one frame later.
        Sim s;
        assert(s.until(PushState::PREPARE, 40));
        assert(stopped(s.out.motion) && s.out.motion.gripper_offset == 0);
    }
    { // Open at NEAR without ground evidence, then reacquire TRACK before moving.
        Sim s;
        assert(s.until(PushState::PREPARE, 80));
        s.live_camera_guard = true;
        assert(s.out.motion.camera_pitch_cdeg == 4000 && s.out.motion.gripper_offset == 0);
        bool returned = false;
        for (int i = 0; i < 60; ++i) {
            s.in.target_valid = s.in.geometry_valid = false;
            s.tick();
            assert(stopped(s.out.motion));
            assert(s.out.state == PushState::PREPARE);
            if (s.out.reason == "open_confirmed_return_to_tracking_pitch") {
                assert(s.pitch == 4000 && s.in.camera_pitch_stable);
                assert(s.out.motion.camera_pitch_cdeg == 500);
                returned = true;
                break;
            }
        }
        assert(returned);
        s.in.target_valid = s.in.geometry_valid = true;
        assert(s.until(PushState::RUSH, 60));
        assert(s.pitch == 500 && s.in.camera_pitch_stable);
    }
    { // Closing uses longitudinal depth, not the longer off-axis radial range.
        Sim s;
        s.toRush();
        s.in.distance_m = .209f;
        s.in.heading_error = .32f;
        s.tick();
        assert(s.out.state == PushState::LOWER_FRAME && stopped(s.out.motion));
    }
    { // Still outside the closing plane: do not close early.
        Sim s;
        s.toRush();
        s.in.distance_m = .21f;
        s.in.heading_error = .2f;
        s.tick();
        assert(s.out.state == PushState::RUSH);
    }
    { // New default: missing multiview evidence holds CLOSE and ends in safe stop.
        Sim s;
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.multi_evidence = false;
        s.toRush();
        s.load = inv(1);
        s.in.distance_m = .19f;
        assert(s.until(PushState::VERIFY_CAPTURE, 100));
        for (int i = 0; i < 200 && s.out.state != PushState::SAFE_STOP; ++i) {
            s.tick();
            assert(stopped(s.out.motion));
            assert(s.out.motion.gripper_offset == 20);
        }
        assert(s.out.state == PushState::SAFE_STOP && s.out.reason == "multi_view_unconfirmed");
        assert(!s.visited.count(PushState::CARRY));
    }
    { // Positive injected multiview summary exercises the actual default path.
        Sim s;
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.toCarry();
        assert(s.out.state == PushState::CARRY && s.out.reason == "multi_view_capture_verified");
    }
}

void demoWatchdogCases() {
    { // Search-only is a terminal decision, never an approach or a pickup.
        TaskTuning t;
        t.demo_search_only = true;
        Sim s(t);
        assert(s.until(PushState::SAFE_STOP, 100));
        assert(s.out.reason == "demo_target_found" && stopped(s.out.motion));
        assert(!s.visited.count(PushState::APPROACH) && !s.visited.count(PushState::RUSH));
    }
    { // One-trip demo stops before the normal post-delivery turn / next search.
        TaskTuning t;
        t.demo_carry_once = true;
        Sim s(t);
        s.toCarry();
        assert(s.until(PushState::ENTER, 400));
        s.in.delivery_observed = true;
        assert(s.until(PushState::BACK_OUT, 400));
        s.in.zone_supply_count += s.out.batch_size;
        s.hold({});
        assert(s.until(PushState::VERIFY_DELIVERY, 400));
        assert(s.until(PushState::SAFE_STOP, 100));
        assert(s.out.reason == "demo_delivery_complete" && s.out.delivered_total == 1 &&
               stopped(s.out.motion));
        assert(!s.visited.count(PushState::TURN_SCAN));
    }
    assert(makeTargetAreas("red") == std::vector<std::string>{"red_safe_zone"});
    assert(makeTargetAreas("blue") == std::vector<std::string>{"blue_safe_zone"});
    assert(makeTargetBalls("red").size() == 3);
    { // MotionWatch 在 ~9Hz
      // 控制环下也能判静止；原地转向指令下陀螺≈0超过阈值计为转圈卡死，正常转动/停止指令立即清零。
        MotionWatch w;
        std::array<float, 3> still{0, 0, 9.8f}, stalled{0, 0, .004f}, turning{0, 0, .2f};
        uint64_t t = 1000000;
        for (int i = 0; i < 40; ++i, t += 105000)
            w.feed(t, "SCAN", 0, .8f, true, true, still, stalled, stalled);
        assert(w.suspect() && w.spinStallUs() >= 3000000);
        assert(w.takeEvents().find("spin_stall_begin state=SCAN") != std::string::npos);
        w.feed(t, "SCAN", 0, .8f, true, true, still, turning, turning);
        t += 105000;
        assert(w.spinStallUs() == 0 && w.takeEvents().find("spin_stall_end") != std::string::npos);
        MotionWatch f;
        for (int i = 0; i < 40; ++i, t += 105000)
            f.feed(t, "SCAN", 0, .8f, true, true, still, turning, turning);
        assert(f.spinStallUs() == 0 && !f.suspect());
        MotionWatch d; // 前进指令不算转圈卡死
        for (int i = 0; i < 40; ++i, t += 105000)
            d.feed(t, "APPROACH", .3f, .8f, true, true, still, stalled, stalled);
        assert(d.spinStallUs() == 0);
    }
    { // SCAN转圈卡死 -> 后退再前进各一段，然后恢复转圈。
        TaskTuning t;
        t.scan_wz = t.turn_wz = t.max_wz = .4f;
        PushTask task(t);
        PushObservation in;
        in.run = in.safety_ok = in.path_safe = in.opponent_zone_clear = in.retreat_safe = true;
        in.camera_pitch_cdeg = 500;
        in.camera_pitch_stable = true;
        PushOutput out;
        for (int i = 0; i < 400; ++i) {
            in.now_us += 100000;
            out = task.update(in);
            if (out.state == PushState::SCAN)
                break;
        }
        assert(out.state == PushState::SCAN);
        in.now_us += 100000;
        out = task.update(in);
        assert(out.motion.wz_rps > 0 && out.motion.vx_mps == 0);
        in.imu_spin_stall_us = 1600000;
        int back = 0, fwd = 0;
        for (int i = 0; i < 40; ++i) {
            in.now_us += 100000;
            out = task.update(in);
            if (out.reason == "spin_stall_back") {
                ++back;
                assert(out.motion.vx_mps < 0 && out.motion.wz_rps == 0);
            }
            if (out.reason == "spin_stall_forward") {
                ++fwd;
                assert(out.motion.vx_mps > 0 && out.motion.wz_rps == 0);
            }
            in.imu_spin_stall_us = 0; // 动作期间 MotionWatch 的计数被清零
        }
        assert(back >= 7 && back <= 9 && fwd >= 7 && fwd <= 9);
        in.now_us += 100000;
        out = task.update(in);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps > 0 && out.reason == "searching_target");
        TaskTuning off = t;
        off.spin_unstick = false;
        PushTask task2(off);
        PushObservation i2 = in;
        i2.now_us = 0;
        i2.imu_spin_stall_us = 0;
        for (int i = 0; i < 400; ++i) {
            i2.now_us += 100000;
            out = task2.update(i2);
            if (out.state == PushState::SCAN)
                break;
        }
        i2.imu_spin_stall_us = 9000000;
        i2.now_us += 100000;
        out = task2.update(i2);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps > 0);
    }
    { // MotionWatch: commanded but vibration-free for >=1.5 s is flagged; real vibration or no
      // command is not.
        MotionWatch w;
        std::array<float, 3> still{0, 0, 9.8f}, zero{0, 0, 0};
        uint64_t t = 1000000;
        for (int i = 0; i < 200; ++i, t += 10000)
            w.feed(t, "CARRY", .3f, 0, true, true, still, zero, zero);
        assert(w.suspect() && w.suspectTotalUs() >= 400000 && w.stateSuspectUs("CARRY") > 0);
        assert(w.takeEvents().find("[MOTION_EVENT] begin state=CARRY") != std::string::npos);
        MotionWatch v;
        for (int i = 0; i < 200; ++i, t += 10000) {
            std::array<float, 3> a{0, 0, 9.8f + ((i & 1) ? .8f : -.8f)};
            v.feed(t, "CARRY", .3f, 0, true, true, a, zero, zero);
        }
        assert(!v.suspect() && v.suspectTotalUs() == 0 && v.stateCmdUs("CARRY") > 0);
        MotionWatch n;
        for (int i = 0; i < 200; ++i, t += 10000)
            n.feed(t, "SCAN", 0, 0, true, true, still, zero, zero);
        assert(!n.suspect() && n.stateCmdUs("SCAN") == 0);
        MotionWatch o;
        for (int i = 0; i < 200; ++i, t += 10000)
            o.feed(t, "CARRY", .3f, 0, false, true, still, zero, zero);
        assert(!o.suspect());
        assert(w.line().find("[MOTION] state=CARRY") != std::string::npos);
    }
    { // MotionWatch blocked:
      // 前进+指令转向，陀螺响应占比<0.16持续一个窗口才算被顶住；正常转向、直行、无指令、换状态都不算。
        std::array<float, 3> acc{0, 0, 9.8f}, turn_slow{0, 0, .05f}, turn_ok{0, 0, .18f},
            rpy{0, 0, 0};
        uint64_t t = 1000000;
        MotionWatch b;
        for (int i = 0; i < 300; ++i, t += 10000)
            b.feed(t, "ENTER", .3f, i & 64 ? .55f : -.55f, true, true, acc, turn_slow, rpy);
        assert(b.blockedUs() >= 600000 && b.blockRatio() >= 0 && b.blockRatio() < .16f);
        assert(b.takeEvents().find("[MOTION_EVENT] blocked_begin state=ENTER") !=
               std::string::npos);
        assert(b.line().find("blocked_ms=") != std::string::npos);
        b.feed(t, "BACK_OUT", -.2f, 0, true, true, acc, turn_slow, rpy);
        assert(b.blockedUs() == 0);
        MotionWatch f;
        for (int i = 0; i < 300; ++i, t += 10000)
            f.feed(t, "ENTER", .3f, .55f, true, true, acc, turn_ok, rpy);
        assert(f.blockedUs() == 0);
        MotionWatch g;
        for (int i = 0; i < 300; ++i, t += 10000)
            g.feed(t, "ENTER", .3f, 0, true, true, acc, turn_slow, rpy);
        assert(g.blockedUs() == 0);
        MotionWatch h;
        for (int i = 0; i < 300; ++i, t += 10000)
            h.feed(t, "ENTER", .3f, .55f, false, true, acc, turn_slow, rpy);
        assert(h.blockedUs() == 0);
        MotionWatch k;
        for (int i = 0; i < 120; ++i, t += 10000)
            k.feed(t, "ENTER", .3f, .55f, true, true, acc, turn_slow, rpy);
        for (int i = 0; i < 120; ++i, t += 10000)
            k.feed(t, "ENTER", .3f, .55f, true, true, acc, turn_ok, rpy);
        assert(k.blockedUs() == 0);
    }
}
