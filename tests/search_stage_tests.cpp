#include "rescue/task/search_stage.hpp"
#include "rescue/task/push_task.hpp"

#include <cmath>
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
        case PushState::LOST_SEARCH:
            return "LOST_SEARCH";
        case PushState::MID_REACQUIRE:
            return "MID_REACQUIRE";
        case PushState::NEAR_REACQUIRE:
            return "NEAR_REACQUIRE";
        case PushState::SELECT_CARGO:
            return "SELECT_CARGO";
        case PushState::CUE_APPROACH:
            return "CUE_APPROACH";
        case PushState::MID_APPROACH:
            return "MID_APPROACH";
        case PushState::APPROACH:
            return "APPROACH";
        default:
            return "OTHER";
    }
}

struct Fixture {
    SearchStage stage;
    TaskTuning tuning;
    PushObservation in;
    SearchContext context;
    SearchServices services;

    int target_id = -1, search_cue_id = -1, capture_target_id = -1, step = 0;
    int lost_round = 0, scatter_seen = 0, near_failures = 0, scatter_approach_n = 0;
    int clear_attempts = 0;
    int confirmations = 0, frame_raised = 0, dummy_best_n = 0;
    bool cue_mode = false, heading_seen = false, burst_drive_used = false;
    bool burst_swing_armed = true;
    float turned = 0, last_heading = 0, approach_forward = 0, scan_turn_acc = 0;
    float far_y = 1.f;
    uint64_t phase_us = 0, scatter_idle_since_us = 0;
    std::string label, reason;
    Inventory pending;

    bool candidate_now = false, cue_now = false, weak_now = false, unstick_now = false;
    int eval_group_now = 0;
    BestGroup best_group{0, 0.f};

    std::vector<std::string> enters;
    std::vector<std::string> faults;
    std::vector<std::string> clears;
    std::vector<std::string> scatters;
    int frames = 0, drives = 0, spins = 0, lost_calls = 0, rejections = 0;

    Fixture() {
        tuning.confirm_frames = 2;
        tuning.blind_scatter_min = 2;
        tuning.blind_scatter_far_min = 2;
        tuning.sweep_confirm = 1;
        tuning.far_pitch_cdeg = 100;
        tuning.intermediate_pitch_cdeg = 200;
        tuning.track_pitch_cdeg = 300;

        in.now_us = 1000000;
        in.heading_valid = true;
        in.heading_rad = 0;
        in.gripper_closed_observed = true;
        in.distance_m = 1.f;

        context.now_us = in.now_us;
        context.far_y = far_y;
        context.frame_raised = frame_raised;
        context.pending = &pending;
        context.target_id = &target_id;
        context.label = &label;
        context.confirmations = &confirmations;
        context.cue_mode = &cue_mode;
        context.search_cue_id = &search_cue_id;
        context.capture_target_id = &capture_target_id;
        context.step = &step;
        context.phase_us = &phase_us;
        context.turned = &turned;
        context.heading_seen = &heading_seen;
        context.last_heading = &last_heading;
        context.lost_round = &lost_round;
        context.scatter_seen = &scatter_seen;
        context.near_failures = &near_failures;
        context.scan_turn_acc = &scan_turn_acc;
        context.scatter_count = &dummy_best_n;
        context.scatter_approach_n = &scatter_approach_n;
        context.clear_attempts = &clear_attempts;
        context.scatter_idle_since_us = &scatter_idle_since_us;
        context.burst_drive_used = &burst_drive_used;
        context.burst_swing_armed = &burst_swing_armed;
        context.approach_forward = &approach_forward;
        context.reason = &reason;

        services.candidate = [&](const PushObservation&, uint64_t) {
            return candidate_now;
        };
        services.weak_candidate = [&](const PushObservation&) {
            return weak_now;
        };
        services.cue_candidate = [&] {
            return cue_now;
        };
        services.target_body = [&] {
            return cv::Point2f(0.f, .4f);
        };
        services.find_best = [&](const std::vector<cv::Point2f>&, float, float) {
            return best_group;
        };
        services.eval_group = [&](const std::vector<cv::Point2f>&, int, int, float) {
            return eval_group_now;
        };
        services.add_ordinary_extra = [&](const cv::Point2f&) {
        };
        services.begin_scatter = [&](bool far, bool, const char* why) {
            scatters.push_back(std::string(far ? "far:" : "near:") + why);
        };
        services.drive = [&](float, float) {
            ++drives;
        };
        services.scan_spin = [&] {
            ++spins;
        };
        services.unstick = [&] {
            return unstick_now;
        };
        services.enter = [&](PushState s, uint64_t, const char* why) {
            enters.push_back(std::string(stateName(s)) + ":" + why);
        };
        services.fail_stop = [&](const char* why) {
            faults.push_back(why);
        };
        services.scan_after_clear = [&](const char* why) {
            clears.push_back(why);
        };
        services.command_frame = [&](uint8_t, uint64_t) {
            ++frames;
        };
        services.reject_search_cue = [&](uint64_t) {
            ++rejections;
        };
        services.search_lost = [&] {
            ++lost_calls;
        };
    }

    void tick(PushState state, uint64_t now) {
        in.now_us = now;
        context.now_us = now;
        context.elapsed_us = now - phase_us;
        stage.update(state, tuning, in, context, services);
    }
};

void scanWithoutCandidateSpinsAndClearsTheLock() {
    Fixture f;
    f.target_id = 4;
    f.confirmations = 1;
    f.context.scan_ok = true;
    f.tick(PushState::SCAN, f.in.now_us);
    CHECK(f.reason == "searching_target");
    CHECK(f.target_id == -1 && f.confirmations == 0);
    CHECK(f.spins == 1);
    CHECK(f.frames == 1);
}

void scanConfirmsALockAndStartsTheCueApproach() {
    Fixture f;
    f.candidate_now = true;
    f.cue_now = true;
    f.in.target_id = 7;
    f.in.label = "ordinary_supply";
    f.tick(PushState::SCAN, f.in.now_us);
    CHECK(f.confirmations == 1 && f.enters.empty());
    f.tick(PushState::SCAN, f.in.now_us + 1);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "CUE_APPROACH:search_cue_locked");
    CHECK(f.search_cue_id == 7 && f.capture_target_id == -1 && f.cue_mode);
    CHECK(!f.burst_swing_armed);
}

void scanConfirmsAnOrdinaryLock() {
    Fixture f;
    f.candidate_now = true;
    f.in.target_id = 3;
    f.in.label = "ordinary_supply";
    f.tick(PushState::SCAN, f.in.now_us);
    f.tick(PushState::SCAN, f.in.now_us + 1);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "APPROACH:target_locked");
    CHECK(f.capture_target_id == 3 && f.search_cue_id == -1 && !f.cue_mode);
}

void scanFarGroupStartsAFarScatter() {
    Fixture f;
    f.candidate_now = true;
    f.in.distance_m = 2.f;
    f.context.scatter_ok = true;
    f.context.approach_ok = true;
    f.context.far_y = 3.f;
    f.tuning.blind_scatter_far_lock_min = 2;
    f.tuning.blind_scatter_max_y_m = 1.f;
    f.tuning.blind_scatter_far_chase_m = .5f;
    f.best_group = BestGroup{2, 1.5f};
    f.tick(PushState::SCAN, f.in.now_us);
    CHECK(f.scatters.size() == 1);
    CHECK(f.scatters[0] == "far:blind_scatter_far_group");
}

void scanDenseGroupDefersWithoutAFreshCluster() {
    Fixture f;
    f.candidate_now = true;
    f.in.distance_m = .6f;
    f.context.scatter_ok = true;
    f.context.clutter_fresh = true;
    f.tuning.dense_no_lock = true;
    f.tuning.sweep_confirm = 2;
    f.eval_group_now = 1;
    f.tick(PushState::SCAN, f.in.now_us);
    CHECK(f.scatters.empty());
    CHECK(f.reason == "dense_group_confirm");
    CHECK(f.confirmations == 0);
}

void lostSearchTiltsToTheMidViewOnce() {
    Fixture f;
    f.context.mid_pitch = true;
    f.step = 1;
    f.phase_us = f.in.now_us - 20000000;
    f.tick(PushState::LOST_SEARCH, f.in.now_us);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "LOST_SEARCH:lost_search_mid_pitch");
    CHECK(f.lost_round == 1);
}

void lostSearchRescansWhenNothingIsFound() {
    Fixture f;
    f.step = 1;
    f.phase_us = f.in.now_us - 20000000;
    f.tuning.lost_search_rescan = true;
    f.context.trip_empty = true;
    f.tick(PushState::LOST_SEARCH, f.in.now_us);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "SCAN:lost_search_rescan");
}

void lostSearchExhaustsIntoAFault() {
    Fixture f;
    f.step = 1;
    f.phase_us = f.in.now_us - 20000000;
    f.tick(PushState::LOST_SEARCH, f.in.now_us);
    CHECK(f.faults.size() == 1 && f.faults[0] == "lost_search_exhausted");
}

void lostSearchNeedsClosedFrameFeedback() {
    Fixture f;
    f.in.gripper_closed_observed = false;
    f.context.sweep_open = false;
    f.tick(PushState::LOST_SEARCH, f.in.now_us);
    CHECK(f.faults.size() == 1 && f.faults[0] == "lost_search_feedback_missing");
}

void lostSearchReacquiresAtCloseRange() {
    Fixture f;
    f.step = 1;
    f.lost_round = 1;
    f.candidate_now = true;
    f.in.distance_m = .3f;
    f.in.target_id = 9;
    f.in.label = "ordinary_supply";
    f.tuning.track_near_m = .5f;
    f.tick(PushState::LOST_SEARCH, f.in.now_us);
    f.tick(PushState::LOST_SEARCH, f.in.now_us + 1);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "MID_REACQUIRE:lost_target_reacquired_mid");
}

void lostSearchGroupStartsACommittedScatter() {
    Fixture f;
    f.step = 1;
    f.context.scatter_ok = true;
    f.context.clutter_fresh = true;
    f.context.trip_empty = true;
    f.tuning.blind_scatter_far_min = 1;
    f.eval_group_now = 2;
    f.tick(PushState::LOST_SEARCH, f.in.now_us);
    CHECK(f.scatters.size() == 1);
    CHECK(f.scatters[0] == "far:lost_group_far");
}

void reacquireConfirmsTheNewCameraLock() {
    Fixture f;
    f.context.forward_ok = true;
    f.candidate_now = true;
    f.in.target_id = 5;
    f.in.label = "ordinary_supply";
    f.tick(PushState::MID_REACQUIRE, f.in.now_us);
    f.tick(PushState::MID_REACQUIRE, f.in.now_us + 1);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "MID_APPROACH:mid_cargo_reacquired");
    CHECK(f.target_id == 5);
}

void reacquireTimesOutIntoASearch() {
    Fixture f;
    f.step = 1;
    f.phase_us = f.in.now_us - 3000000;
    f.tuning.enable_lost_search = true;
    f.tick(PushState::NEAR_REACQUIRE, f.in.now_us);
    CHECK(f.lost_calls == 1);
}

void reacquireWithAnOpenJawFaults() {
    Fixture f;
    f.step = 1;
    f.phase_us = f.in.now_us - 3000000;
    f.frame_raised = 1;
    f.context.frame_raised = 1;
    f.tick(PushState::NEAR_REACQUIRE, f.in.now_us);
    CHECK(f.faults.size() == 1 && f.faults[0] == "open_jaw_reacquire_timeout");
}

void reacquireRejectsABlockedPath() {
    Fixture f;
    f.candidate_now = true;
    f.context.forward_ok = false;
    f.tick(PushState::MID_REACQUIRE, f.in.now_us);
    CHECK(f.reason == "mid_cargo_path_blocked");
    CHECK(f.confirmations == 0);
}

void selectCargoLocksTheApproach() {
    Fixture f;
    f.candidate_now = true;
    f.in.target_id = 2;
    f.in.label = "ordinary_supply";
    f.tick(PushState::SELECT_CARGO, f.in.now_us);
    f.tick(PushState::SELECT_CARGO, f.in.now_us + 1);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "APPROACH:cargo_locked");
    CHECK(f.target_id == 2);
}

void selectCargoBudgetRejectsTheCue() {
    Fixture f;
    f.tuning.prepare_budget_us = 100;
    f.context.elapsed_us = 101;
    f.tick(PushState::SELECT_CARGO, f.in.now_us);
    CHECK(f.rejections == 1);
    CHECK(f.clears.size() == 1 && f.clears[0] == "no_eligible_cargo");
}

void unstickOwnsTheScanTick() {
    Fixture f;
    f.unstick_now = true;
    f.context.scan_ok = true;
    f.tick(PushState::SCAN, f.in.now_us);
    CHECK(f.spins == 0);
    CHECK(f.reason == "searching_target");
}
} // namespace

int main() {
    const std::pair<const char*, void (*)()> tests[] = {
        {"scanWithoutCandidateSpinsAndClearsTheLock", scanWithoutCandidateSpinsAndClearsTheLock},
        {"scanConfirmsALockAndStartsTheCueApproach", scanConfirmsALockAndStartsTheCueApproach},
        {"scanConfirmsAnOrdinaryLock", scanConfirmsAnOrdinaryLock},
        {"scanFarGroupStartsAFarScatter", scanFarGroupStartsAFarScatter},
        {"scanDenseGroupDefersWithoutAFreshCluster", scanDenseGroupDefersWithoutAFreshCluster},
        {"lostSearchTiltsToTheMidViewOnce", lostSearchTiltsToTheMidViewOnce},
        {"lostSearchRescansWhenNothingIsFound", lostSearchRescansWhenNothingIsFound},
        {"lostSearchExhaustsIntoAFault", lostSearchExhaustsIntoAFault},
        {"lostSearchNeedsClosedFrameFeedback", lostSearchNeedsClosedFrameFeedback},
        {"lostSearchReacquiresAtCloseRange", lostSearchReacquiresAtCloseRange},
        {"lostSearchGroupStartsACommittedScatter", lostSearchGroupStartsACommittedScatter},
        {"reacquireConfirmsTheNewCameraLock", reacquireConfirmsTheNewCameraLock},
        {"reacquireTimesOutIntoASearch", reacquireTimesOutIntoASearch},
        {"reacquireWithAnOpenJawFaults", reacquireWithAnOpenJawFaults},
        {"reacquireRejectsABlockedPath", reacquireRejectsABlockedPath},
        {"selectCargoLocksTheApproach", selectCargoLocksTheApproach},
        {"selectCargoBudgetRejectsTheCue", selectCargoBudgetRejectsTheCue},
        {"unstickOwnsTheScanTick", unstickOwnsTheScanTick},
    };
    for (const auto& [name, run] : tests) {
        try {
            run();
        } catch (const std::exception& e) {
            std::cout << "FAIL " << name << ": " << e.what() << "\n";
            return 1;
        }
    }
    std::cout << "search stage OK\n";
    return 0;
}
