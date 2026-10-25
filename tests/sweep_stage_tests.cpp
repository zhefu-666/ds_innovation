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

const char* stateName(PushState s) {
    switch (s) {
        case PushState::SCAN:
            return "SCAN";
        case PushState::CLEAR_PILE:
            return "CLEAR_PILE";
        case PushState::CLEAR_PAUSE:
            return "CLEAR_PAUSE";
        case PushState::MID_REACQUIRE:
            return "MID_REACQUIRE";
        default:
            return "OTHER";
    }
}

struct Fixture {
    SweepStage stage;
    TaskTuning tuning;
    PushObservation in;
    SweepContext context;
    SweepServices services;

    SweepLedger ledger;
    DisperseRushStage rush;
    DisperseTurnStage turn;
    DisperseSwingStage swing;

    float motion_vx = 0, motion_wz = 0;
    float approach_forward = 0, travel = 0, scatter_approach_m = 0, scatter_range = 0;
    float scatter_yaw = 0, scan_turn_acc = 0, scan_mid_limit = 0, disperse_rush_m = 0;
    float sweep_travel0 = 0, sweep_yaw0 = 0, pile_yaw0 = 0, pile_push0 = -1.f;
    float target_forward_m = 1.f;
    int capture_target_id = -1, clear_attempts = 0, confirmations = 0, misses = 0;
    int pile_chain_n = 0, pile_phase = 0, pile_runs = 0, pile_seen = 0;
    int scatter_approach_n = 0, scatter_seen = 0, search_cue_id = -1, sweep_phase = 0;
    int target_id = -1;
    int16_t pitch_cmd = 0, sweep_pitch = 0;
    uint64_t clear_commanded_us = 0, disperse_cool_until_us = 0, pile_cool_until_us = 0;
    uint64_t pile_phase_us = 0, scatter_cool_until_us = 0, scatter_idle_since_us = 0;
    uint64_t sweep_start_us = 0, sweep_t0_us = 0;
    bool burst_fast = false, burst_swing = false, burst_swing_armed = false;
    bool cue_mode = false, disperse_small = false, scatter_commit = false, scatter_far = false;
    bool track_close = false;
    std::string label, reason;

    std::vector<std::string> enters;
    std::vector<std::string> faults;
    int cleared = 0, frames = 0, rejected = 0;

    bool candidate_now = false, tracking_now = false;

    Fixture() {
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
        context.motion_vx = &motion_vx;
        context.motion_wz = &motion_wz;
        context.approach_forward = &approach_forward;
        context.burst_fast = &burst_fast;
        context.burst_swing = &burst_swing;
        context.burst_swing_armed = &burst_swing_armed;
        context.capture_target_id = &capture_target_id;
        context.clear_attempts = &clear_attempts;
        context.clear_commanded_us = &clear_commanded_us;
        context.confirmations = &confirmations;
        context.cue_mode = &cue_mode;
        context.disperse_cool_until_us = &disperse_cool_until_us;
        context.disperse_rush_m = &disperse_rush_m;
        context.disperse_small = &disperse_small;
        context.label = &label;
        context.misses = &misses;
        context.pile_chain_n = &pile_chain_n;
        context.pile_cool_until_us = &pile_cool_until_us;
        context.pile_phase = &pile_phase;
        context.pile_phase_us = &pile_phase_us;
        context.pile_push0 = &pile_push0;
        context.pile_runs = &pile_runs;
        context.pile_seen = &pile_seen;
        context.pile_yaw0 = &pile_yaw0;
        context.reason = &reason;
        context.scan_mid_limit = &scan_mid_limit;
        context.scan_turn_acc = &scan_turn_acc;
        context.scatter_approach_m = &scatter_approach_m;
        context.scatter_approach_n = &scatter_approach_n;
        context.scatter_commit = &scatter_commit;
        context.scatter_cool_until_us = &scatter_cool_until_us;
        context.scatter_far = &scatter_far;
        context.scatter_idle_since_us = &scatter_idle_since_us;
        context.scatter_range = &scatter_range;
        context.scatter_seen = &scatter_seen;
        context.scatter_yaw = &scatter_yaw;
        context.search_cue_id = &search_cue_id;
        context.sweep_phase = &sweep_phase;
        context.sweep_pitch = &sweep_pitch;
        context.sweep_start_us = &sweep_start_us;
        context.sweep_t0_us = &sweep_t0_us;
        context.sweep_travel0 = &sweep_travel0;
        context.sweep_yaw0 = &sweep_yaw0;
        context.target_id = &target_id;
        context.track_close = &track_close;
        context.travel = &travel;
        context.sweep_ledger = &ledger;
        context.disperse_rush = &rush;
        context.disperse_turn = &turn;
        context.disperse_swing = &swing;

        services.command_frame = [&](uint8_t, uint64_t) {
            ++frames;
        };
        services.drive = [&](float, float) {
        };
        services.enter = [&](PushState s, uint64_t, const char* why) {
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

    void tick(PushState state, uint64_t now) {
        in.now_us = now;
        context.now_us = now;
        context.elapsed_us = now - pile_phase_us;
        stage.update(state, tuning, in, context, services);
    }
};

void cuePrepareFaultsWhenShortPushIsOff() {
    Fixture f;
    f.tuning.enable_short_push = false;
    f.tick(PushState::CUE_PREPARE, f.in.now_us);
    CHECK(f.faults.size() == 1 && f.faults[0] == "clear_attempt_limit");
}

void cuePrepareConfirmsIntoClearPile() {
    Fixture f;
    f.tick(PushState::CUE_PREPARE, f.in.now_us);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "CLEAR_PILE:closed_push_010");
    CHECK(f.clear_attempts == 1);
}

void cuePrepareNeedsClearanceEvidence() {
    Fixture f;
    f.in.clear_push_safe = false;
    f.tick(PushState::CUE_PREPARE, f.in.now_us);
    CHECK(f.faults.size() == 1 && f.faults[0] == "clear_path_or_boundary_unverified");
}

void clearPileFaultsWhenEvidenceIsLost() {
    Fixture f;
    f.in.clear_push_safe = false;
    f.tick(PushState::CLEAR_PILE, f.in.now_us);
    CHECK(f.faults.size() == 1 && f.faults[0] == "clear_evidence_lost");
}

void clearPileReachesThePause() {
    Fixture f;
    f.tracking_now = true;
    f.context.target_forward_m = .2f;
    f.tick(PushState::CLEAR_PILE, f.in.now_us);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "CLEAR_PAUSE:clear_target_distance_reached");
}

void clearPileDrivesUnderTheCap() {
    Fixture f;
    f.tracking_now = true;
    f.context.target_forward_m = 2.f;
    f.tuning.clear_speed = .12f;
    f.tuning.max_speed = .4f;
    f.tick(PushState::CLEAR_PILE, f.in.now_us);
    CHECK(f.enters.empty());
    CHECK(f.motion_vx == .12f);
    CHECK(f.motion_wz == 0.f);
}

void clearPauseResumesTheSearch() {
    Fixture f;
    f.pile_phase_us = 1000000;
    f.tick(PushState::CLEAR_PAUSE, 1000000 + 500000);
    CHECK(f.cleared == 1);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "MID_REACQUIRE:clear_reobserve_cargo");
}

void scatterWithoutHeadingEndsIntoScan() {
    Fixture f;
    f.in.heading_valid = false;
    f.tick(PushState::SCATTER, f.in.now_us);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "SCAN:blind_scatter_no_heading");
    CHECK(f.frames == 1);
}

void scatterTimeoutEndsIntoScan() {
    Fixture f;
    f.tuning.blind_scatter_budget_us = 100;
    f.tick(PushState::SCATTER, f.in.now_us);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "SCAN:blind_scatter_timeout");
}

void pileWithoutHeadingEndsIntoScan() {
    Fixture f;
    f.in.heading_valid = false;
    f.tick(PushState::PILE, f.in.now_us);
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
    f.tick(PushState::PILE, f.in.now_us);
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
