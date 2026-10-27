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
    decltype(data.pile_push0_)& pile_push0 = data.pile_push0_;
    decltype(data.pile_runs_)& pile_runs = data.pile_runs_;
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

void pileWithoutHeadingEndsIntoScan() {
    Fixture f;
    f.in.heading_valid = false;
    f.tick(SearchAction::Pile, f.in.now_us);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "SCAN:pile_no_heading");
    CHECK(f.pile_phase == 0);
}

void pileGivesUpWhenTheClusterIsAlreadyClear() {
    Fixture f;
    f.pile_phase = 4;
    f.pile_phase_us = f.in.now_us - 1000000;
    f.tuning.pile_measure_us = 0;
    f.pile_seen = 7;
    f.context.clutter_fresh = false;
    f.in.target_valid = false;
    f.tick(SearchAction::Pile, f.in.now_us);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "SCAN:pile_cluster_clear");
}
} // namespace

int main() {
    const std::pair<const char*, void (*)()> tests[] = {
        {"cuePrepareFaultsWhenShortPushIsOff", cuePrepareFaultsWhenShortPushIsOff},
        {"cuePrepareConfirmsIntoClearPile", cuePrepareConfirmsIntoClearPile},
        {"cuePrepareNeedsClearanceEvidence", cuePrepareNeedsClearanceEvidence},
        {"clearPileFaultsWhenEvidenceIsLost", clearPileFaultsWhenEvidenceIsLost},
        {"clearPileReachesThePause", clearPileReachesThePause},
        {"clearPileDrivesUnderTheCap", clearPileDrivesUnderTheCap},
        {"clearPauseResumesTheSearch", clearPauseResumesTheSearch},
        {"scatterWithoutHeadingEndsIntoScan", scatterWithoutHeadingEndsIntoScan},
        {"scatterTimeoutEndsIntoScan", scatterTimeoutEndsIntoScan},
        {"pileWithoutHeadingEndsIntoScan", pileWithoutHeadingEndsIntoScan},
        {"pileGivesUpWhenTheClusterIsAlreadyClear", pileGivesUpWhenTheClusterIsAlreadyClear},
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
