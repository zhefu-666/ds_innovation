#include "rescue/task/sweep_stage.hpp"
#include "rescue/task/push_task.hpp"

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace rescue;

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
    }
}
#define CHECK(expression) check((expression), #expression, __LINE__)

const char* stateName(SearchAction s) {
    switch (s) {
        case SearchAction::Scan:
            return "SCAN";
        case SearchAction::ClearPile:
            return "CLEAR_PILE";
        case SearchAction::ClearPause:
            return "CLEAR_PAUSE";
        case SearchAction::MidReacquire:
            return "MID_REACQUIRE";
        default:
            return "OTHER";
    }
}

struct Fixture {
    SweepStage stage;
    TaskTuning tuning;
    PushObservation in;
    SearchPhaseData data;
    TripContext trip_data;
    MotionCommand motion;
    SweepLedger ledger_state;
    std::string reason_text;
    SweepContext context{data, data.progress, trip_data, motion, ledger_state, reason_text};
    decltype(motion.vx_mps)& motion_vx = motion.vx_mps;
    decltype(motion.wz_rps)& motion_wz = motion.wz_rps;
    decltype(trip_data.approach_forward_)& approach_forward = trip_data.approach_forward_;
    decltype(data.burst_fast_)& burst_fast = data.burst_fast_;
    decltype(data.burst_swing_)& burst_swing = data.burst_swing_;
    decltype(data.burst_swing_armed_)& burst_swing_armed = data.burst_swing_armed_;
    decltype(trip_data.capture_target_id_)& capture_target_id = trip_data.capture_target_id_;
    decltype(data.clear_attempts_)& clear_attempts = data.clear_attempts_;
    decltype(data.clear_commanded_us_)& clear_commanded_us = data.clear_commanded_us_;
    decltype(data.progress.confirmations)& confirmations = data.progress.confirmations;
    decltype(data.cue_mode_)& cue_mode = data.cue_mode_;
    decltype(data.disperse_cool_until_us_)& disperse_cool_until_us = data.disperse_cool_until_us_;
    decltype(data.disperse_rush_m_)& disperse_rush_m = data.disperse_rush_m_;
    decltype(data.disperse_small_)& disperse_small = data.disperse_small_;
    decltype(trip_data.label_)& label = trip_data.label_;
    decltype(data.progress.misses)& misses = data.progress.misses;
    decltype(data.pile_chain_n_)& pile_chain_n = data.pile_chain_n_;
    decltype(data.pile_cool_until_us_)& pile_cool_until_us = data.pile_cool_until_us_;
    decltype(data.pile_phase_)& pile_phase = data.pile_phase_;
    decltype(data.pile_phase_us_)& pile_phase_us = data.pile_phase_us_;
    decltype(data.pile_seen_)& pile_seen = data.pile_seen_;
    decltype(data.pile_yaw0_)& pile_yaw0 = data.pile_yaw0_;
    decltype(reason_text)& reason = reason_text;
    decltype(data.scan_mid_limit_)& scan_mid_limit = data.scan_mid_limit_;
    decltype(data.scan_turn_acc_)& scan_turn_acc = data.scan_turn_acc_;
    decltype(data.scatter_approach_m_)& scatter_approach_m = data.scatter_approach_m_;
    decltype(data.scatter_approach_n_)& scatter_approach_n = data.scatter_approach_n_;
    decltype(data.scatter_commit_)& scatter_commit = data.scatter_commit_;
    decltype(data.scatter_cool_until_us_)& scatter_cool_until_us = data.scatter_cool_until_us_;
    decltype(data.scatter_far_)& scatter_far = data.scatter_far_;
    decltype(data.scatter_idle_since_us_)& scatter_idle_since_us = data.scatter_idle_since_us_;
    decltype(data.scatter_range_)& scatter_range = data.scatter_range_;
    decltype(data.scatter_seen_)& scatter_seen = data.scatter_seen_;
    decltype(data.scatter_yaw_)& scatter_yaw = data.scatter_yaw_;
    decltype(data.search_cue_id_)& search_cue_id = data.search_cue_id_;
    decltype(data.sweep_phase_)& sweep_phase = data.sweep_phase_;
    decltype(data.sweep_pitch_)& sweep_pitch = data.sweep_pitch_;
    decltype(data.sweep_start_us_)& sweep_start_us = data.sweep_start_us_;
    decltype(data.sweep_t0_us_)& sweep_t0_us = data.sweep_t0_us_;
    decltype(data.sweep_travel0_)& sweep_travel0 = data.sweep_travel0_;
    decltype(data.sweep_yaw0_)& sweep_yaw0 = data.sweep_yaw0_;
    decltype(trip_data.target_id_)& target_id = trip_data.target_id_;
    decltype(data.progress.track_close)& track_close = data.progress.track_close;
    decltype(data.progress.travel)& travel = data.progress.travel;
    decltype(ledger_state)& ledger = ledger_state;
    decltype(data.disperse_rush_)& rush = data.disperse_rush_;
    decltype(data.disperse_turn_)& turn = data.disperse_turn_;
    decltype(data.disperse_swing_)& swing = data.disperse_swing_;

    SweepServices services;

    float target_forward_m = 1.f;

    int16_t pitch_cmd = 0;

    std::vector<std::string> enters;
    std::vector<std::string> faults;
    int cleared = 0, frames = 0, rejected = 0;

    bool candidate_now = false, tracking_now = false;

    Fixture() {
        sweep_pitch = 0;
        tuning.confirm_frames = 1;
        tuning.enable_short_push = true;
        tuning.grasp_trigger_y_m = .5f;
        in.now_us = 1000000;
        in.heading_valid = true;
        in.clear_push_safe = true;
        in.gripper_closed_observed = true;

        context.now_us = in.now_us;
        context.pitch_cmd = pitch_cmd;
        context.target_forward_m = target_forward_m;

        services.command_frame = [&](uint8_t, uint64_t) {
            ++frames;
        };
        services.drive = [&](float, float) {
        };
        services.enter = [&](SearchAction s, uint64_t, const char* why) {
            enters.push_back(std::string(stateName(s)) + ":" + why);
        };
        services.fail_stop = [&](const char* why) {
            faults.push_back(why);
        };
        services.scan_after_clear = [&](const char*) {
        };
        services.blacklist = [&](uint64_t) {
        };
        services.clear_trip = [&] {
            ++cleared;
        };
        services.candidate = [&](const PushObservation&, uint64_t) {
            return candidate_now;
        };
        services.weak_candidate = [&](const PushObservation&) {
            return false;
        };
        services.cue_candidate = [&] {
            return false;
        };
        services.target_body = [&] {
            return cv::Point2f(0.f, .4f);
        };
        services.find_best = [&](const std::vector<cv::Point2f>&, float, float) {
            return SweepBest{0, 0.f};
        };
        services.eval_group = [&](const std::vector<cv::Point2f>&, int, int, float) {
            return 0;
        };
        services.add_ordinary_extra = [&](const cv::Point2f&) {
        };
        services.clear_rejected = [&] {
            ++rejected;
        };
        services.clear_rejected_bearings = [&] {
        };
        services.begin_scatter = [&](bool, bool, const char*) {
        };
        services.sweep_frame_ready = [&] {
            return true;
        };
        services.disperse_step = [&] {
            return false;
        };
        services.begin_pile_from_sweep = [&](const char*) {
            return false;
        };
        services.search_lost = [&] {
        };
        services.lost_target = [&] {
            return false;
        };
        services.steer = [&](float, float) {
        };
        services.desired_pitch = [&] {
            return pitch_cmd;
        };
        services.tracking = [&](const PushObservation&) {
            return tracking_now;
        };
        services.mid_pitch = [&] {
            return false;
        };
        services.motion_ratio = [&] {
            return 1.f;
        };
    }

    void tick(SearchAction state, uint64_t now) {
        in.now_us = now;
        context.now_us = now;
        context.elapsed_us = now - pile_phase_us;
        stage.update(state, tuning, in, context, services);
    }
};

void confirmedOrdinaryInterruptsOnlyObservationStages() {
    for (int phase : {0, 7, 4, 5}) {
        for (bool clean : {false, true}) {
            Fixture f;
            f.candidate_now = true;
            f.in.target_valid = f.in.pixel_locked = true;
            f.in.label = "ordinary_supply";
            f.in.distance_m = 1.73f;
            f.in.pixel_ratio_smoothed = .000733507;
            f.in.geometry_valid = f.in.target_region_valid = true;
            f.in.path_safe = f.in.opponent_zone_clear = true;
            f.in.corridor.ordinary = 1;
            f.in.corridor.dangerous = clean ? 0 : 1;
            f.in.corridor_complete = f.in.corridor_occlusion_free = true;
            f.scatter_commit = f.scatter_far = true;
            f.sweep_phase = phase;
            f.scatter_approach_m = 2.f;
            f.scatter_range = 2.f;
            f.pile_phase_us = f.sweep_start_us = f.in.now_us;
            f.context.clutter_fresh = true;
            f.tick(SearchAction::Scatter, f.in.now_us);
            const bool expected = clean && (phase == 0 || phase == 7);
            if (expected) {
                CHECK(f.enters.size() == 1 && f.enters[0] == "SCAN:blind_scatter_target_seen");
                CHECK(!f.scatter_commit && !f.scatter_far);
                CHECK(f.motion_vx == 0.f && f.motion_wz == 0.f);
            } else {
                for (const auto& entry : f.enters)
                    CHECK(entry != "SCAN:blind_scatter_target_seen");
            }
        }
    }
}

void cuePrepareFaultsWhenShortPushIsOff() {
    Fixture f;
    f.tuning.enable_short_push = false;
    f.tick(SearchAction::CuePrepare, f.in.now_us);
    CHECK(f.faults.size() == 1 && f.faults[0] == "clear_attempt_limit");
}

void cuePrepareConfirmsIntoClearPile() {
    Fixture f;
    f.tick(SearchAction::CuePrepare, f.in.now_us);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "CLEAR_PILE:closed_push_010");
    CHECK(f.clear_attempts == 1);
}

void cuePrepareNeedsClearanceEvidence() {
    Fixture f;
    f.in.clear_push_safe = false;
    f.tick(SearchAction::CuePrepare, f.in.now_us);
    CHECK(f.faults.size() == 1 && f.faults[0] == "clear_path_or_boundary_unverified");
}

void clearPileFaultsWhenEvidenceIsLost() {
    Fixture f;
    f.in.clear_push_safe = false;
    f.tick(SearchAction::ClearPile, f.in.now_us);
    CHECK(f.faults.size() == 1 && f.faults[0] == "clear_evidence_lost");
}

void clearPileReachesThePause() {
    Fixture f;
    f.tracking_now = true;
    f.context.target_forward_m = .2f;
    f.tick(SearchAction::ClearPile, f.in.now_us);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "CLEAR_PAUSE:clear_target_distance_reached");
}

void clearPileDrivesUnderTheCap() {
    Fixture f;
    f.tracking_now = true;
    f.context.target_forward_m = 2.f;
    f.tuning.clear_speed = .12f;
    f.tuning.max_speed = .4f;
    f.tick(SearchAction::ClearPile, f.in.now_us);
    CHECK(f.enters.empty());
    CHECK(f.motion_vx == .12f);
    CHECK(f.motion_wz == 0.f);
}

void clearPauseResumesTheSearch() {
    Fixture f;
    f.pile_phase_us = 1000000;
    f.tick(SearchAction::ClearPause, 1000000 + 500000);
    CHECK(f.cleared == 1);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "MID_REACQUIRE:clear_reobserve_cargo");
}

void scatterWithoutHeadingEndsIntoScan() {
    Fixture f;
    f.in.heading_valid = false;
    f.tick(SearchAction::Scatter, f.in.now_us);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "SCAN:blind_scatter_no_heading");
    CHECK(f.frames == 1);
}

void scatterTimeoutEndsIntoScan() {
    Fixture f;
    f.tuning.blind_scatter_budget_us = 100;
    f.tick(SearchAction::Scatter, f.in.now_us);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "SCAN:blind_scatter_timeout");
}

// Exercise actual cluster selection, UART residual commands and IMU drift together.
void prepareFarScatter(Fixture& f) {
    f.pile_phase_us = f.in.now_us;
    f.scatter_far = f.burst_fast = true;
    f.burst_swing_armed = true;
    f.scatter_range = 2.f;
    f.scatter_approach_m = 1.5f;
    f.tuning.first_burst_full_impact = true;
    f.tuning.blind_scatter_max_y_m = .5f;
    f.tuning.blind_scatter_far_max_y_m = 4.f;
    f.in.sent_velocity_valid = true;
    f.context.scatter_ok = true;
    f.services.drive = [&](float vx, float wz) {
        f.motion_vx = vx;
        f.motion_wz = wz;
    };
    f.services.find_best = [&](const std::vector<cv::Point2f>& pts, float max_y, float radius) {
        DisperseClusterPlanner planner;
        const auto best = planner.findBest(f.tuning, f.in, pts, max_y, radius);
        return SweepBest{best.n, best.d, planner.bestCentre()};
    };
}

void observeFarGroup(Fixture& f, float bearing = 0) {
    const float x = -2.f * std::sin(bearing), y = 2.f * std::cos(bearing);
    f.in.clutter_body = {{x, y}, {x - .05f, y}, {x + .05f, y}};
    f.in.clutter_ordinary = f.in.clutter_body;
    f.context.clutter_fresh = true;
}

void farTick(Fixture& f, uint64_t now, bool fresh = true) {
    f.motion_vx = f.motion_wz = 0;
    if (fresh)
        f.in.clutter_ts_us = now;
    f.tick(SearchAction::Scatter, now);
}

void settleFarScatter(Fixture& f, uint64_t start) {
    for (int i = 0; i <= 6; ++i)
        farTick(f, start + i * 50000);
    CHECK(f.reason == "first_burst_approach");
    CHECK(f.motion_vx == f.tuning.first_burst_speed);
    CHECK(f.sweep_phase == 7);
}

void farScatterBrakesAtTransientAlignment() {
    Fixture f;
    prepareFarScatter(f);
    observeFarGroup(f);
    f.in.sent_wz_rps = .18f;
    farTick(f, 1000000);
    farTick(f, 1050000);
    CHECK(f.motion_vx == 0 && f.motion_wz == 0);
    CHECK(f.reason == "blind_scatter_align_brake");
    // A zero command alone is insufficient while the measured heading keeps moving.
    f.in.sent_wz_rps = 0;
    for (int i = 0; i < 5; ++i) {
        f.in.heading_rad += .01f;
        observeFarGroup(f, -f.in.heading_rad);
        farTick(f, 1100000 + i * 50000);
        CHECK(f.motion_vx == 0 && f.motion_wz == 0);
    }
    settleFarScatter(f, 1350000);
}

void farScatterReconfirmsVisualDirection() {
    Fixture f;
    prepareFarScatter(f);
    // The original stored bearing is zero; the current group is 0.4 rad to the left.
    observeFarGroup(f, .4f);
    farTick(f, 1000000);
    farTick(f, 1050000);
    CHECK(f.motion_vx == 0 && f.motion_wz > 0);
    CHECK(f.motion_wz <= f.tuning.pixel_align_max_wz);
    CHECK(std::abs(f.scatter_yaw - .4f) < 1e-4f);
    f.in.heading_rad = .4f;
    observeFarGroup(f);
    settleFarScatter(f, 1100000);
}

void farScatterStopsAndRealignsWithoutResettingTravel() {
    Fixture f;
    prepareFarScatter(f);
    observeFarGroup(f);
    settleFarScatter(f, 1000000);
    f.travel = .3f;
    const auto started = f.sweep_start_us;
    const auto travel0 = f.sweep_travel0;
    f.in.heading_rad = .35f;
    observeFarGroup(f, -.35f);
    f.in.sent_vx_mps = 1.f;
    f.in.sent_wz_rps = .14f; // the old turn is still being sent, as in the incident
    farTick(f, 1350000);
    CHECK(f.motion_vx == 0 && f.motion_wz == 0);
    CHECK(f.reason == "blind_scatter_align_brake");
    f.in.sent_vx_mps = f.in.sent_wz_rps = 0;
    farTick(f, 1400000);
    CHECK(f.motion_vx == 0 && f.motion_wz < 0);
    f.in.heading_rad = 0;
    observeFarGroup(f);
    f.in.sent_wz_rps = -.1f;
    farTick(f, 1450000);
    CHECK(f.motion_vx == 0 && f.motion_wz == 0);
    f.in.sent_wz_rps = 0;
    settleFarScatter(f, 1500000);
    CHECK(f.sweep_start_us == started && f.sweep_travel0 == travel0);
    CHECK(f.travel == .3f);
}

void farScatterLosesVisionAndNeedsFreshConfirmation() {
    Fixture f;
    prepareFarScatter(f);
    observeFarGroup(f);
    settleFarScatter(f, 1000000);
    f.in.clutter_body.clear();
    f.in.clutter_ordinary.clear();
    farTick(f, 1350000);
    CHECK(f.motion_vx == 0 && f.motion_wz == 0);
    CHECK(f.reason == "blind_scatter_reobserve");
    observeFarGroup(f);
    f.in.clutter_ts_us = 1400000;
    for (int i = 0; i < 6; ++i) {
        farTick(f, 1400000 + i * 50000, false);
        CHECK(f.motion_vx == 0); // reusing one image cannot satisfy the stable-frame count
    }
    settleFarScatter(f, 1700000);
}

void farScatterFreshFramesMustSpanTheSettlingWindow() {
    Fixture f;
    prepareFarScatter(f);
    observeFarGroup(f);
    for (int i = 0; i < 4; ++i)
        farTick(f, 1000000 + i * 50000);
    for (int i = 0; i < 4; ++i) {
        farTick(f, 1200000 + i * 50000, false);
        CHECK(f.motion_vx == 0); // three early frames do not prove the rest of the stop window
    }
    farTick(f, 1400000);
    CHECK(f.reason == "first_burst_approach" && f.motion_vx == f.tuning.first_burst_speed);
}

void farScatterCannotUseStaleVisionOrAnUnrelatedGroup() {
    Fixture f;
    prepareFarScatter(f);
    observeFarGroup(f);
    f.in.clutter_ts_us = 1;
    farTick(f, 1000000, false);
    CHECK(f.reason == "blind_scatter_reobserve" && f.motion_vx == 0);
    // A group outside the original bearing's association gate must not redirect the rush.
    f.in.heading_rad = 1.f;
    observeFarGroup(f);
    farTick(f, 1050000);
    CHECK(f.reason == "blind_scatter_reobserve" && f.motion_vx == 0);
    farTick(f, 1000000 + f.tuning.blind_scatter_align_budget_us + 1);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "SCAN:blind_scatter_realign_timeout");
}

void farScatterNewAttemptNeedsToSettleAgain() {
    Fixture f;
    prepareFarScatter(f);
    observeFarGroup(f);
    settleFarScatter(f, 1000000);
    f.stage.resetScatterApproach();
    f.sweep_phase = 0;
    farTick(f, 1350000);
    CHECK(f.motion_vx == 0 && f.motion_wz == 0);
    settleFarScatter(f, 1400000);
}

void pileWithoutHeadingEndsIntoScan() {
    Fixture f;
    f.in.heading_valid = false;
    f.tick(SearchAction::Pile, f.in.now_us);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "SCAN:pile_no_heading");
    CHECK(f.pile_phase == 0);
}

void removedMetricPilePhasesCannotMove() {
    for (int phase = 0; phase <= 6; ++phase) {
        Fixture f;
        f.pile_phase = phase;
        f.pile_phase_us = f.in.now_us;
        f.context.clutter_fresh = true;
        f.in.clutter_body = {{-.04f, .4f}, {.04f, .4f}};
        f.in.clutter_ordinary = f.in.clutter_body;
        int drive_calls = 0;
        f.services.drive = [&](float, float) { ++drive_calls; };
        f.tick(SearchAction::Pile, f.in.now_us);
        CHECK(drive_calls == 0);
        CHECK(f.enters.size() == 1 && f.enters[0] == "SCAN:pile_bad_phase");
        CHECK(f.motion_vx == 0 && f.motion_wz == 0);
    }
}

void pileChainFinishesWithoutCut() {
    for (bool flat : {false, true}) {
        Fixture f;
        f.pile_phase = 7;
        f.pile_phase_us = f.in.now_us;
        f.tuning.pile_measure_us = 0;
        f.tuning.pile_chain_max = 0;
        f.context.clutter_fresh = true;
        f.in.clutter_body = {{-.04f, .4f}, {.04f, flat ? .4f : .65f}};
        f.in.clutter_ordinary = f.in.clutter_body;
        int drive_calls = 0;
        f.services.drive = [&](float, float) { ++drive_calls; };
        f.tick(SearchAction::Pile, f.in.now_us);
        CHECK(drive_calls == 0);
        CHECK(f.enters.size() == 1);
        CHECK(f.enters[0] == (flat ? "SCAN:pile_chain_one_layer" : "SCAN:pile_chain_limit"));
        CHECK(f.motion_vx == 0 && f.motion_wz == 0);
    }
}
} // namespace

int main() {
    const std::pair<const char*, void (*)()> tests[] = {
        {"confirmedOrdinaryInterruptsOnlyObservationStages",
         confirmedOrdinaryInterruptsOnlyObservationStages},
        {"cuePrepareFaultsWhenShortPushIsOff", cuePrepareFaultsWhenShortPushIsOff},
        {"cuePrepareConfirmsIntoClearPile", cuePrepareConfirmsIntoClearPile},
        {"cuePrepareNeedsClearanceEvidence", cuePrepareNeedsClearanceEvidence},
        {"clearPileFaultsWhenEvidenceIsLost", clearPileFaultsWhenEvidenceIsLost},
        {"clearPileReachesThePause", clearPileReachesThePause},
        {"clearPileDrivesUnderTheCap", clearPileDrivesUnderTheCap},
        {"clearPauseResumesTheSearch", clearPauseResumesTheSearch},
        {"scatterWithoutHeadingEndsIntoScan", scatterWithoutHeadingEndsIntoScan},
        {"scatterTimeoutEndsIntoScan", scatterTimeoutEndsIntoScan},
        {"farScatterBrakesAtTransientAlignment", farScatterBrakesAtTransientAlignment},
        {"farScatterReconfirmsVisualDirection", farScatterReconfirmsVisualDirection},
        {"farScatterStopsAndRealignsWithoutResettingTravel",
         farScatterStopsAndRealignsWithoutResettingTravel},
        {"farScatterLosesVisionAndNeedsFreshConfirmation",
         farScatterLosesVisionAndNeedsFreshConfirmation},
        {"farScatterFreshFramesMustSpanTheSettlingWindow",
         farScatterFreshFramesMustSpanTheSettlingWindow},
        {"farScatterCannotUseStaleVisionOrAnUnrelatedGroup",
         farScatterCannotUseStaleVisionOrAnUnrelatedGroup},
        {"farScatterNewAttemptNeedsToSettleAgain", farScatterNewAttemptNeedsToSettleAgain},
        {"removedMetricPilePhasesCannotMove", removedMetricPilePhasesCannotMove},
        {"pileChainFinishesWithoutCut", pileChainFinishesWithoutCut},
        {"pileWithoutHeadingEndsIntoScan", pileWithoutHeadingEndsIntoScan},
    };
    for (const auto& [name, run] : tests) {
        try {
            run();
        } catch (const std::exception& e) {
            std::cout << "FAIL " << name << ": " << e.what() << "\n";
            return 1;
        }
    }
    std::cout << "sweep stage OK\n";
    return 0;
}
