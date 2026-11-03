#include "rescue/task/search_stage.hpp"
#include "rescue/task/push_task.hpp"
#include <cassert>
#include <iostream>
#include <vector>
using namespace rescue;
namespace {
struct Fixture {
    SearchStage stage;
    TaskTuning tuning;
    PushObservation in;
    SearchPhaseData data;
    TripContext trip;
    MotionCommand motion;
    std::string reason;
    SearchContext context{data, data.progress, trip, motion, reason};
    SearchServices services;
    std::vector<SearchAction> enters;
    std::string failure, cleared;
    bool candidate = false, tracking = true;
    int spins = 0, blacklists = 0, lost = 0;
    Fixture() {
        tuning.confirm_frames = 2;
        // Even explicit old configuration cannot enable removed actions.
        tuning.enable_search_cues = tuning.enable_short_push = true;
        tuning.blind_scatter = tuning.disperse = tuning.pile_clear = tuning.pile_chain = true;
        tuning.sweep_non_supply = tuning.sweep_upright = tuning.mid_neighbour_clear = true;
        tuning.dense_no_lock = true;
        in.now_us = context.now_us = 1000000;
        in.heading_valid = in.gripper_closed_observed = true;
        in.target_id = 7;
        in.label = "ordinary_supply";
        in.distance_m = .6f;
        context.scan_ok = context.forward_ok = context.trip_empty = true;
        services.candidate = [&](const PushObservation& o, uint64_t) {
            return candidate && !o.target_is_search_cue;
        };
        services.drive = [&](float vx, float wz) { motion.vx_mps = vx; motion.wz_rps = wz; };
        services.steer = services.drive;
        services.scan_spin = [&] { ++spins; };
        services.unstick = [] { return false; };
        services.enter = [&](SearchAction a, uint64_t, const char*) { enters.push_back(a); };
        services.fail_stop = [&](const char* why) { failure = why; };
        services.scan_after_clear = [&](const char* why) { cleared = why; };
        services.command_frame = [](uint8_t, uint64_t) {};
        services.search_lost = [&] { ++lost; };
        services.blacklist = [&](uint64_t) { ++blacklists; };
        services.frame_wait = [](const PushObservation&, uint8_t, uint64_t) { return 1; };
        services.lost_target = [] { return true; };
        services.tracking = [&](const PushObservation&) { return tracking; };
        services.tracking_reason = [] { return "target_missing"; };
        services.clear_trip = [&] { trip = {}; };
        services.approach_heading_shift = [](const PushObservation&) { return 0.f; };
        services.desired_pitch = [&] { return context.pitch_cmd; };
        services.mid_pitch = [] { return false; };
    }
    void tick(SearchAction action, uint64_t dt = 50000) {
        in.now_us += dt;
        context.now_us = in.now_us;
        stage.update(action, tuning, in, context, services);
    }
};
void scanAndLock() {
    Fixture f;
    f.trip.target_id_ = 9;
    f.tick(SearchAction::Scan);
    assert(f.spins == 1 && f.trip.target_id_ == -1);
    f.candidate = true;
    f.tick(SearchAction::Scan);
    assert(f.enters.empty());
    f.tick(SearchAction::Scan);
    assert(f.enters == std::vector<SearchAction>{SearchAction::Approach});
    assert(f.trip.target_id_ == 7 && f.trip.capture_target_id_ == 7);
}
void cueAndClutterNeverStartClearing() {
    Fixture cue;
    cue.candidate = true;
    cue.in.target_is_search_cue = true;
    for (int i = 0; i < 100; ++i) cue.tick(SearchAction::Scan);
    assert(cue.enters.empty() && cue.motion.vx_mps == 0 && cue.spins == 100);
    Fixture dense;
    dense.candidate = true;
    dense.in.corridor_complete = true;
    dense.in.corridor.ordinary = 1;
    dense.in.corridor.dangerous = 1;
    dense.in.clutter_body = {{0, .3f}, {.05f, .3f}, {-.05f, .3f}};
    dense.in.clutter_ts_us = dense.in.now_us;
    dense.tick(SearchAction::Scan);
    dense.tick(SearchAction::Scan);
    assert(dense.enters == std::vector<SearchAction>{SearchAction::Approach});
    Fixture empty;
    empty.in.clutter_body = dense.in.clutter_body;
    for (int i = 0; i < 100; ++i) empty.tick(SearchAction::Scan);
    assert(empty.enters.empty() && empty.motion.vx_mps == 0);
}
void lostSearchRetainsReacquisitionAndLimits() {
    Fixture found;
    found.candidate = true;
    found.tick(SearchAction::LostSearch);
    found.tick(SearchAction::LostSearch);
    assert(found.enters == std::vector<SearchAction>{SearchAction::Approach});
    Fixture close;
    close.candidate = true;
    close.data.lost_round_ = 1;
    close.in.distance_m = .1f;
    close.tick(SearchAction::LostSearch);
    close.tick(SearchAction::LostSearch);
    assert(close.enters == std::vector<SearchAction>{SearchAction::MidReacquire});
    Fixture mid;
    mid.context.mid_pitch = true;
    mid.data.progress.step = 1;
    mid.data.progress.turned = 7;
    mid.tick(SearchAction::LostSearch);
    assert(mid.data.lost_round_ == 1 && mid.enters[0] == SearchAction::LostSearch);
    Fixture exhausted;
    exhausted.data.progress.step = 1;
    exhausted.data.progress.turned = 7;
    exhausted.tuning.lost_search_rescan = false;
    exhausted.tick(SearchAction::LostSearch);
    assert(exhausted.failure == "lost_search_exhausted");
    Fixture feedback;
    feedback.in.gripper_closed_observed = false;
    feedback.tick(SearchAction::LostSearch);
    assert(feedback.failure == "lost_search_feedback_missing");
}
void cameraReacquisitionKeepsChecks() {
    for (bool mid : {false, true}) {
        Fixture f;
        f.candidate = true;
        const auto action = mid ? SearchAction::MidReacquire : SearchAction::NearReacquire;
        f.tick(action); f.tick(action);
        assert(f.enters.size() == 1);
        assert(f.enters[0] == (mid ? SearchAction::MidApproach : SearchAction::Approach));
        assert(f.trip.target_id_ == 7);
    }
    Fixture blocked;
    blocked.candidate = true;
    blocked.context.forward_ok = false;
    blocked.tick(SearchAction::MidReacquire);
    blocked.tick(SearchAction::MidReacquire);
    assert(blocked.enters.empty() && blocked.reason == "mid_cargo_path_blocked");
    Fixture open;
    open.context.frame_raised = 1;
    open.tick(SearchAction::NearReacquire);
    open.tick(SearchAction::NearReacquire, 3000000);
    assert(open.failure == "open_jaw_reacquire_timeout");
}
void blockedCargoIsReselectedWithoutClosedPush() {
    Fixture f;
    f.context.forward_ok = false;
    f.in.clear_push_safe = true;
    f.trip.target_id_ = 7;
    f.trip.label_ = f.in.label;
    f.tick(SearchAction::MidApproach);
    assert(f.enters == std::vector<SearchAction>{SearchAction::MidReacquire});
    assert(f.blacklists == 1 && f.motion.vx_mps == 0 && f.motion.wz_rps == 0);
}
void selectCargoLocksOrTimesOut() {
    Fixture f;
    f.candidate = true;
    f.tick(SearchAction::SelectCargo); f.tick(SearchAction::SelectCargo);
    assert(f.enters == std::vector<SearchAction>{SearchAction::Approach});
    Fixture timeout;
    timeout.context.elapsed_us = timeout.tuning.prepare_budget_us + 1;
    timeout.tick(SearchAction::SelectCargo);
    assert(timeout.blacklists == 1 && timeout.cleared == "no_eligible_cargo");
}
}
int main() {
    scanAndLock(); cueAndClutterNeverStartClearing();
    lostSearchRetainsReacquisitionAndLimits(); cameraReacquisitionKeepsChecks();
    blockedCargoIsReselectedWithoutClosedPush(); selectCargoLocksOrTimesOut();
    std::cout << "PASS search discovery, no legacy clearing, reacquisition and blocked paths\n";
}
