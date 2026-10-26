#include "rescue/task/transport_rules.hpp"
#include "rescue/core/types.hpp"
#include "rescue/geometry/zone_estimate.hpp"
#include "rescue/geometry/zone_dead_reckoning.hpp"
#include "rescue/geometry/planner.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#if __has_include("rescue/task/capture_verification_stage.hpp")
#include "rescue/task/capture_verification_stage.hpp"
#endif
#define private public
#include "rescue/task/push_task.hpp"
#undef private

namespace {
using namespace rescue;
constexpr uint64_t kStart = 1000000;
constexpr uint64_t kTick = 50000;

struct Rig {
    PushTask task;
    PushObservation input;
    explicit Rig(TaskTuning tuning = {}) : task(tuning) {
        input.now_us = kStart;
        input.run = input.safety_ok = true;
        input.path_safe = input.retreat_safe = input.opponent_zone_clear = true;
        input.heading_valid = true;
        input.camera_pitch_stable = true;
        input.camera_pitch_cdeg = tuning.far_pitch_cdeg;
        task.last_us_ = kStart;
    }
    PushOutput tick() {
        input.now_us += kTick;
        return task.update(input);
    }
};

void startupIsDeferredAndDoesNotRepeat() {
    TaskTuning tuning;
    tuning.startup_advance_us = 10000000;
    tuning.startup_advance_speed = .4f;
    Rig rig(tuning);
    rig.input.run = false;
    auto out = rig.tick();
    assert(out.phase == TaskPhase::Startup && out.run_status == RunStatus::Waiting);
    rig.input.run = true;
    out = rig.tick();
    assert(out.phase == TaskPhase::Startup && out.run_status == RunStatus::Running);
    const auto frame = out.motion.gripper_offset;
    const auto transaction = out.motion.frame_transaction;
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    out = rig.tick();
    assert(out.phase == TaskPhase::Search && out.search_action == SearchAction::Scan);
    assert(out.reason == "startup_deferred" && out.startup_commanded_us == 0);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    assert(out.motion.gripper_offset == frame && out.motion.frame_transaction == transaction);
    rig.input.run = false;
    out = rig.tick();
    assert(out.phase == TaskPhase::Search && out.run_status == RunStatus::Paused);
    rig.input.run = true;
    out = rig.tick();
    assert(out.phase == TaskPhase::Search && out.run_status == RunStatus::Running);
    rig.input.reset = true;
    out = rig.tick();
    assert(out.phase == TaskPhase::Startup && out.run_status == RunStatus::Waiting);
}

void prepareTransfersInventoryAndRushLimit() {
    TaskTuning tuning;
    tuning.require_multi_view = false;
    tuning.corridor_confirm_frames = 1;
    Rig rig(tuning);
    rig.task.enter(PushState::PREPARE, kStart, "prepare_test");
    rig.task.actionProgress().step = 1;
    rig.task.frame_raised_ = 1;
    rig.task.pitch_cmd_ = tuning.track_pitch_cdeg;
    rig.input.camera_pitch_cdeg = tuning.track_pitch_cdeg;
    rig.input.gripper_done = true;
    rig.input.gripper_feedback_open = 1;
    rig.input.target_valid = rig.input.geometry_valid = rig.input.target_region_valid = true;
    rig.input.target_id = rig.task.trip_context_.target_id_ = 7;
    rig.input.label = rig.task.trip_context_.label_ = "ordinary_supply";
    rig.input.distance_m = .3f;
    rig.input.corridor.ordinary = 2;
    rig.input.corridor_complete = rig.input.corridor_occlusion_free = true;
    rig.task.transport_phase_.progress.travel = .77f;
    auto out = rig.tick();
    assert(out.phase == TaskPhase::Capture && out.capture_action == CaptureAction::Advance);
    assert(out.batch_size == 2 && rig.task.trip_context_.pending_.ordinary == 2);
    assert(out.capture_evidence == CaptureEvidence::None);
    const float limit = .3f - tuning.grasp_trigger_y_m + tuning.rush_extra_m;
    assert(std::abs(rig.task.capture_phase_.progress.travel_limit - limit) < 1e-6f);
    assert(rig.task.capture_phase_.progress.travel == 0);
    assert(rig.task.transport_phase_.progress.travel == .77f);
    assert(out.corridor_observation_requested && !out.navigation_requested);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
}

void pauseKeepsCargoAndRecoveryKeepsItsOwner() {
    TaskTuning tuning;
    tuning.require_multi_view = false;
    Rig rig(tuning);
    rig.task.enter(PushState::GATE, kStart, "gate_test");
    rig.task.trip_context_.trip_.ordinary = 1;
    rig.task.capture_phase_.evidence = CaptureEvidence::Verified;
    rig.input.run = false;
    auto out = rig.tick();
    assert(out.phase == TaskPhase::Transport && out.run_status == RunStatus::Paused);
    assert(out.batch_size == 1 && out.capture_evidence == CaptureEvidence::Verified);
    assert(!out.navigation_requested && out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    rig.input.run = true;
    out = rig.tick();
    assert(out.phase == TaskPhase::Transport && out.transport_action == TransportAction::Carry);
    assert(out.batch_size == 1 && out.capture_evidence == CaptureEvidence::Verified);
    assert(rig.task.transport_phase_.progress.step == 1);

    rig.task.enter(PushState::LOST_HOLD, rig.input.now_us, "hold_lost");
    out = rig.tick();
    assert(out.phase == TaskPhase::Transport && out.recovering);
    assert(out.transport_action == TransportAction::Recover && !out.navigation_requested);
    rig.input.gripper_done = true;
    rig.input.gripper_feedback_open = 1;
    for (int n = 0; n < 20 && out.phase != TaskPhase::Search; ++n)
        out = rig.tick();
    assert(out.phase == TaskPhase::Search && out.search_action == SearchAction::Scan);
    assert(!out.recovering && out.batch_size == 0);
    assert(out.capture_evidence == CaptureEvidence::None);
    rig.task.fail("test_fault");
    out = rig.tick();
    assert(out.phase == TaskPhase::Search && out.run_status == RunStatus::Fault);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
}
} // namespace
int main() {
    startupIsDeferredAndDoesNotRepeat();
    prepareTransfersInventoryAndRushLimit();
    pauseKeepsCargoAndRecoveryKeepsItsOwner();
    std::cout << "four-phase contracts passed\n";
}
