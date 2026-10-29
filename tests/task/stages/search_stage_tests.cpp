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

const char* stateName(SearchAction s) {
    switch (s) {
        case SearchAction::Scan:
            return "SCAN";
        case SearchAction::LostSearch:
            return "LOST_SEARCH";
        case SearchAction::MidReacquire:
            return "MID_REACQUIRE";
        case SearchAction::NearReacquire:
            return "NEAR_REACQUIRE";
        case SearchAction::SelectCargo:
            return "SELECT_CARGO";
        case SearchAction::CueApproach:
            return "CUE_APPROACH";
        case SearchAction::MidApproach:
            return "MID_APPROACH";
        case SearchAction::Approach:
            return "APPROACH";
        default:
            return "OTHER";
    }
}

struct Fixture {
    SearchStage stage;
    TaskTuning tuning;
    PushObservation in;
    SearchPhaseData data;
    TripContext trip_data;
    MotionCommand motion;
    SweepLedger ledger_state;
    std::string reason_text;
    SearchContext context{data, data.progress, trip_data, motion, ledger_state, reason_text};
    decltype(trip_data.pending_)& pending = trip_data.pending_;
    decltype(trip_data.target_id_)& target_id = trip_data.target_id_;
    decltype(trip_data.label_)& label = trip_data.label_;
    decltype(data.progress.confirmations)& confirmations = data.progress.confirmations;
    decltype(data.cue_mode_)& cue_mode = data.cue_mode_;
    decltype(data.search_cue_id_)& search_cue_id = data.search_cue_id_;
    decltype(trip_data.capture_target_id_)& capture_target_id = trip_data.capture_target_id_;
    decltype(data.progress.step)& step = data.progress.step;
    decltype(data.progress.started_us)& phase_us = data.progress.started_us;
    decltype(data.progress.turned)& turned = data.progress.turned;
    decltype(data.progress.heading_seen)& heading_seen = data.progress.heading_seen;
    decltype(data.progress.last_heading)& last_heading = data.progress.last_heading;
    decltype(data.lost_round_)& lost_round = data.lost_round_;
    decltype(data.scatter_seen_)& scatter_seen = data.scatter_seen_;
    decltype(data.near_failures_)& near_failures = data.near_failures_;
    decltype(data.scan_turn_acc_)& scan_turn_acc = data.scan_turn_acc_;
    decltype(data.scatter_count_)& dummy_best_n = data.scatter_count_;
    decltype(data.scatter_approach_n_)& scatter_approach_n = data.scatter_approach_n_;
    decltype(data.clear_attempts_)& clear_attempts = data.clear_attempts_;
    decltype(data.scatter_idle_since_us_)& scatter_idle_since_us = data.scatter_idle_since_us_;
    decltype(data.burst_drive_used_)& burst_drive_used = data.burst_drive_used_;
    decltype(data.burst_swing_armed_)& burst_swing_armed = data.burst_swing_armed_;
    decltype(trip_data.approach_forward_)& approach_forward = trip_data.approach_forward_;
    decltype(reason_text)& reason = reason_text;

    SearchServices services;

    int frame_raised = 0;

    float far_y = 1.f;

    bool candidate_now = false, cue_now = false, weak_now = false, unstick_now = false;
    int eval_group_now = 0;
    BestGroup best_group{0, 0.f};

    std::vector<std::string> enters;
    std::vector<std::string> faults;
    std::vector<std::string> clears;
    std::vector<std::string> scatters;
    int frames = 0, drives = 0, spins = 0, lost_calls = 0, rejections = 0;

    Fixture() {
        burst_swing_armed = true;
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
        services.enter = [&](SearchAction s, uint64_t, const char* why) {
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

    void tick(SearchAction state, uint64_t now) {
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
    f.tick(SearchAction::Scan, f.in.now_us);
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
    f.tick(SearchAction::Scan, f.in.now_us);
    CHECK(f.confirmations == 1 && f.enters.empty());
    f.tick(SearchAction::Scan, f.in.now_us + 1);
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
    f.tick(SearchAction::Scan, f.in.now_us);
    f.tick(SearchAction::Scan, f.in.now_us + 1);
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
    f.tick(SearchAction::Scan, f.in.now_us);
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
    f.in.label = "ordinary_supply";
    f.in.corridor.ordinary = 1;
    f.in.corridor.dangerous = 1;
    f.in.corridor_complete = true;
    f.in.clutter_body = {{.02f, .4f}};
    f.eval_group_now = 1;
    f.tick(SearchAction::Scan, f.in.now_us);
    CHECK(f.scatters.empty());
    CHECK(f.reason == "dense_group_confirm");
    CHECK(f.confirmations == 0);
}

void scanOnlyOverlappingBoxesEnterMultiVisibleGroup() {
    const auto run = [](bool overlap) {
        Fixture f;
        f.candidate_now = true;
        f.in.distance_m = .6f;
        f.context.scatter_ok = true;
        f.context.scatter_multi_ok = true;
        f.context.clutter_fresh = true;
        f.tuning.dense_no_lock = true;
        f.tuning.lock_max_visible = 1;
        f.in.label = "ordinary_supply";
        f.in.corridor.ordinary = 1;
        f.in.corridor.dangerous = 1;
        f.in.corridor_complete = true;
        f.eval_group_now = 1;
        f.in.clutter_body = {{.02f, .4f}};
        f.in.clutter_boxes = {cv::Rect(10, 10, 20, 20)};
        f.in.target_box = overlap ? cv::Rect(20, 10, 20, 20) : cv::Rect(40, 10, 20, 20);
        f.tick(SearchAction::Scan, f.in.now_us);
        return f.scatters.empty() ? std::string() : f.scatters.front();
    };
    CHECK(run(true) == "near:multi_visible_split");
    CHECK(run(false) == "near:dense_group_split");
}

void ordinaryTargetWinsOverUnrelatedGroups() {
    for (bool complete : {false, true}) {
        Fixture f;
        f.candidate_now = true;
        f.in.target_id = 13;
        f.in.label = "ordinary_supply";
        f.in.distance_m = 1.73f;
        f.in.pixel_ratio_smoothed = .000733507;
        f.in.corridor.ordinary = 1;
        f.in.corridor_complete = complete;
        f.in.corridor_occlusion_free = true;
        f.in.target_box = {600, 280, 26, 26};
        f.in.clutter_body = {{1.2f, 1.8f}, {1.21f, 1.81f}, {-.1f, 1.5f}};
        f.in.clutter_boxes = {{900, 280, 30, 30}, {910, 280, 30, 30}, {500, 280, 30, 30}};
        f.context.scatter_ok = f.context.scatter_multi_ok = f.context.clutter_fresh = true;
        f.context.approach_ok = true;
        f.context.far_y = 4.f;
        f.tuning.dense_no_lock = true;
        f.tuning.lock_max_visible = 2;
        f.best_group = BestGroup{8, 1.8f};
        f.eval_group_now = 2;
        f.tick(SearchAction::Scan, f.in.now_us);
        f.tick(SearchAction::Scan, f.in.now_us + 30000);
        CHECK(f.scatters.empty());
        CHECK(f.enters.size() == 1 && f.enters[0] == "APPROACH:target_locked");
    }
}

void unrelatedOverlapsCannotBecomeTargetGroup() {
    Fixture f;
    f.candidate_now = true;
    f.in.target_id = 169;
    f.in.label = "ordinary_supply";
    f.in.distance_m = .6f;
    f.in.corridor.ordinary = 1;
    f.in.corridor.dangerous = 1;
    f.in.corridor_complete = true;
    f.in.target_box = {10, 10, 20, 20};
    // Even a blocked corridor cannot redirect this target to a separate visible pile.
    f.in.clutter_body = {{1.5f, .4f}, {1.51f, .42f}};
    f.in.clutter_boxes = {{100, 10, 20, 20}, {110, 10, 20, 20}};
    f.context.scatter_ok = f.context.scatter_multi_ok = f.context.clutter_fresh = true;
    f.tuning.dense_no_lock = true;
    f.tuning.lock_max_visible = 1;
    f.eval_group_now = 1;
    f.tick(SearchAction::Scan, f.in.now_us);
    f.tick(SearchAction::Scan, f.in.now_us + 30000);
    CHECK(f.scatters.empty());
    CHECK(f.enters.size() == 1 && f.enters[0] == "APPROACH:target_locked");
}

void lostSearchTiltsToTheMidViewOnce() {
    Fixture f;
    f.context.mid_pitch = true;
    f.step = 1;
    f.phase_us = f.in.now_us - 20000000;
    f.tick(SearchAction::LostSearch, f.in.now_us);
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
    f.tick(SearchAction::LostSearch, f.in.now_us);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "SCAN:lost_search_rescan");
}

void lostSearchExhaustsIntoAFault() {
    Fixture f;
    f.step = 1;
    f.phase_us = f.in.now_us - 20000000;
    f.tick(SearchAction::LostSearch, f.in.now_us);
    CHECK(f.faults.size() == 1 && f.faults[0] == "lost_search_exhausted");
}

void lostSearchNeedsClosedFrameFeedback() {
    Fixture f;
    f.in.gripper_closed_observed = false;
    f.context.sweep_open = false;
    f.tick(SearchAction::LostSearch, f.in.now_us);
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
    f.tick(SearchAction::LostSearch, f.in.now_us);
    f.tick(SearchAction::LostSearch, f.in.now_us + 1);
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
    f.tick(SearchAction::LostSearch, f.in.now_us);
    CHECK(f.scatters.size() == 1);
    CHECK(f.scatters[0] == "far:lost_group_far");
}

void reacquireConfirmsTheNewCameraLock() {
    Fixture f;
    f.context.forward_ok = true;
    f.candidate_now = true;
    f.in.target_id = 5;
    f.in.label = "ordinary_supply";
    f.tick(SearchAction::MidReacquire, f.in.now_us);
    f.tick(SearchAction::MidReacquire, f.in.now_us + 1);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "MID_APPROACH:mid_cargo_reacquired");
    CHECK(f.target_id == 5);
}

void reacquireTimesOutIntoASearch() {
    Fixture f;
    f.step = 1;
    f.phase_us = f.in.now_us - 3000000;
    f.tuning.enable_lost_search = true;
    f.tick(SearchAction::NearReacquire, f.in.now_us);
    CHECK(f.lost_calls == 1);
}

void reacquireWithAnOpenJawFaults() {
    Fixture f;
    f.step = 1;
    f.phase_us = f.in.now_us - 3000000;
    f.frame_raised = 1;
    f.context.frame_raised = 1;
    f.tick(SearchAction::NearReacquire, f.in.now_us);
    CHECK(f.faults.size() == 1 && f.faults[0] == "open_jaw_reacquire_timeout");
}

void reacquireRejectsABlockedPath() {
    Fixture f;
    f.candidate_now = true;
    f.context.forward_ok = false;
    f.tick(SearchAction::MidReacquire, f.in.now_us);
    CHECK(f.reason == "mid_cargo_path_blocked");
    CHECK(f.confirmations == 0);
}

void selectCargoLocksTheApproach() {
    Fixture f;
    f.candidate_now = true;
    f.in.target_id = 2;
    f.in.label = "ordinary_supply";
    f.tick(SearchAction::SelectCargo, f.in.now_us);
    f.tick(SearchAction::SelectCargo, f.in.now_us + 1);
    CHECK(f.enters.size() == 1);
    CHECK(f.enters[0] == "APPROACH:cargo_locked");
    CHECK(f.target_id == 2);
}

void selectCargoBudgetRejectsTheCue() {
    Fixture f;
    f.tuning.prepare_budget_us = 100;
    f.context.elapsed_us = 101;
    f.tick(SearchAction::SelectCargo, f.in.now_us);
    CHECK(f.rejections == 1);
    CHECK(f.clears.size() == 1 && f.clears[0] == "no_eligible_cargo");
}

void unstickOwnsTheScanTick() {
    Fixture f;
    f.unstick_now = true;
    f.context.scan_ok = true;
    f.tick(SearchAction::Scan, f.in.now_us);
    CHECK(f.spins == 0);
    CHECK(f.reason == "searching_target");
}
} // namespace

int main() {
    const std::pair<const char*, void (*)()> tests[] = {
        {"scanWithoutCandidateSpinsAndClearsTheLock", scanWithoutCandidateSpinsAndClearsTheLock},
        {"scanConfirmsALockAndStartsTheCueApproach", scanConfirmsALockAndStartsTheCueApproach},
        {"scanConfirmsAnOrdinaryLock", scanConfirmsAnOrdinaryLock},
        {"ordinaryTargetWinsOverUnrelatedGroups", ordinaryTargetWinsOverUnrelatedGroups},
        {"unrelatedOverlapsCannotBecomeTargetGroup", unrelatedOverlapsCannotBecomeTargetGroup},
        {"scanFarGroupStartsAFarScatter", scanFarGroupStartsAFarScatter},
        {"scanDenseGroupDefersWithoutAFreshCluster", scanDenseGroupDefersWithoutAFreshCluster},
        {"scanOnlyOverlappingBoxesEnterMultiVisibleGroup",
         scanOnlyOverlappingBoxesEnterMultiVisibleGroup},
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
