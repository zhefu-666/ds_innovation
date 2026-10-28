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

void prepareTransfersInventoryAndRushLimit(bool stepping = false) {
    TaskTuning tuning;
    tuning.require_multi_view = false;
    tuning.corridor_confirm_frames = 1;
    Rig rig(tuning);
    rig.task.setPhaseStepping(stepping);
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
    if (stepping) {
        assert(out.phase == TaskPhase::Search && out.next_phase == TaskPhase::Capture);
        assert(out.phase_waiting && out.run_status == RunStatus::Paused);
        const auto frame = out.motion.gripper_offset;
        const auto txn = out.motion.frame_transaction;
        const auto pitch = out.motion.camera_pitch_cdeg;
        rig.input.run = rig.input.safety_ok = false;
        rig.input.now_us += 300000000; // human inspection cannot exhaust capture timers
        out = rig.tick();
        assert(out.phase_waiting && out.phase == TaskPhase::Search && out.target_id == 7);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        assert(out.motion.gripper_offset == frame && out.motion.frame_transaction == txn);
        assert(out.motion.camera_pitch_cdeg == pitch && out.batch_size == 2);
        rig.input.run = rig.input.safety_ok = true;
        out = rig.tick();
        assert(!out.phase_waiting && rig.task.actionProgress().started_us == rig.input.now_us);
    }
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
void checkpointStartupCaptureAndTransport() {
    Rig startup;
    startup.task.setPhaseStepping(true);
    startup.tick();
    auto out = startup.tick();
    assert(out.phase_waiting && out.phase == TaskPhase::Startup && out.next_phase == TaskPhase::Search);
    startup.input.run = startup.input.safety_ok = false;
    for (int n = 0; n < 5; ++n) {
        out = startup.tick();
        assert(out.phase_waiting && out.phase == TaskPhase::Startup);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    }
    startup.input.run = startup.input.safety_ok = true;
    out = startup.tick();
    assert(!out.phase_waiting && out.phase == TaskPhase::Search);

    TaskTuning tuning;
    tuning.require_multi_view = true;
    Rig capture(tuning);
    capture.task.setPhaseStepping(true);
    capture.task.enter(PushState::VERIFY_CAPTURE, kStart, "capture_test");
    capture.task.trip_context_.pending_.ordinary = 1;
    capture.task.trip_context_.label_ = "ordinary_supply";
    capture.input.multi_view_finished = true;
    capture.input.multi_view_verdict = 1;
    capture.input.multi_view_inventory.ordinary = 1;
    out = capture.tick();
    assert(out.phase_waiting && out.phase == TaskPhase::Capture && out.next_phase == TaskPhase::Transport);
    assert(out.batch_size == 1 && out.capture_evidence == CaptureEvidence::Verified);
    capture.input.run = capture.input.safety_ok = false;
    capture.input.now_us += 300000000;
    out = capture.tick();
    assert(out.phase_waiting && out.capture_evidence == CaptureEvidence::Verified);
    capture.input.run = capture.input.safety_ok = true;
    out = capture.tick();
    assert(out.phase == TaskPhase::Transport && !out.phase_waiting);
    assert(out.batch_size == 1 && capture.task.transport_phase_.carry_us_ == capture.input.now_us);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);

    TaskTuning delivery_tuning;
    delivery_tuning.require_multi_view = false;
    delivery_tuning.delivery_budget_us = 1;
    Rig delivery(delivery_tuning);
    delivery.task.setPhaseStepping(true);
    delivery.task.enter(PushState::VERIFY_DELIVERY, kStart, "delivery_test");
    delivery.task.trip_context_.trip_.ordinary = 1;
    out = delivery.tick();
    assert(out.phase_waiting && out.phase == TaskPhase::Transport && out.next_phase == TaskPhase::Search);
    assert(out.delivery_expected == 1 && out.delivery_credited == 0);
    delivery.input.run = delivery.input.safety_ok = false;
    for (int n = 0; n < 5; ++n) {
        out = delivery.tick();
        assert(out.phase_waiting && out.delivered_total == 0);
    }
    delivery.input.run = delivery.input.safety_ok = true;
    out = delivery.tick();
    assert(!out.phase_waiting && out.phase == TaskPhase::Search);
    assert(out.search_action == SearchAction::TurnAway);
    delivery.input.reset = true;
    out = delivery.tick();
    assert(!out.phase_waiting && out.phase == TaskPhase::Startup);
}
void directPhasesSkipPrecedingPhases() {
    Rig startup;
    startup.task.setSinglePhase("startup");
    startup.task.setInitialFrameOpen(true);
    auto out = startup.tick();
    assert(out.phase == TaskPhase::Startup && out.motion.gripper_offset == 0);
    out = startup.tick();
    assert(out.single_phase_done && out.phase == TaskPhase::Startup);
    out = startup.tick();
    assert(out.single_phase_done && out.motion.vx_mps == 0 && out.motion.wz_rps == 0);

    Rig search;
    search.task.setSinglePhase("search");
    out = search.tick();
    assert(out.phase == TaskPhase::Search && out.state == PushState::SCAN);
    assert(!out.single_phase_done && out.motion.vx_mps == 0);

    TaskTuning tuning;
    tuning.require_multi_view = false;
    tuning.corridor_confirm_frames = 1;
    tuning.capture_frames = 1;
    Rig capture(tuning);
    capture.task.setSinglePhase("capture");
    out = capture.tick();
    assert(out.phase == TaskPhase::Capture && out.state == PushState::RUSH);
    assert(out.corridor_observation_requested && out.motion.vx_mps == 0);
    capture.input.camera_pitch_cdeg = tuning.near_pitch_cdeg;
    out = capture.tick();
    assert(capture.task.single_phase_initializing_ && out.batch_size == 0);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    capture.input.gripper_done = true;
    capture.input.gripper_feedback_open = 1;
    capture.input.target_valid = capture.input.geometry_valid = capture.input.target_region_valid = true;
    capture.input.target_id = 7;
    capture.input.label = "ordinary_supply";
    capture.input.distance_m = .3f;
    capture.input.corridor.ordinary = 1;
    capture.input.corridor_complete = capture.input.corridor_occlusion_free = true;
    out = capture.tick();
    assert(!capture.task.single_phase_initializing_ && out.target_id == 7 && out.batch_size == 1);
    assert(out.phase == TaskPhase::Capture && out.motion.vx_mps == 0);
    assert(capture.task.actionProgress().travel_limit > 0);
    capture.task.enter(PushState::VERIFY_CAPTURE, capture.input.now_us, "test_enclosure");
    capture.input.hold_observable = capture.input.captured = capture.input.held_complete = true;
    capture.input.held.ordinary = 1;
    out = capture.tick();
    assert(out.single_phase_done && out.phase == TaskPhase::Capture);
    assert(out.capture_evidence == CaptureEvidence::Verified);

    Rig transport(tuning);
    transport.task.setSinglePhase("transport");
    out = transport.tick();
    assert(out.phase == TaskPhase::Transport && out.state == PushState::CARRY);
    assert(out.capture_observation_requested && !out.navigation_requested);
    transport.input.camera_pitch_cdeg = tuning.near_pitch_cdeg;
    transport.input.gripper_done = true;
    transport.input.gripper_feedback_open = 0;
    out = transport.tick();
    assert(transport.task.single_phase_initializing_ && out.batch_size == 0);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    transport.input.hold_observable = transport.input.captured = transport.input.held_complete = true;
    transport.input.held.ordinary = 1;
    out = transport.tick();
    assert(!transport.task.single_phase_initializing_ && out.batch_size == 1);
    assert(out.phase == TaskPhase::Transport && out.capture_evidence == CaptureEvidence::Verified);
    assert(transport.task.transport_phase_.carry_us_ == transport.input.now_us);

    TaskTuning multi_tuning;
    multi_tuning.require_multi_view = true;
    Rig multi(multi_tuning);
    multi.task.setSinglePhase("transport");
    multi.tick();
    multi.input.camera_pitch_cdeg = multi_tuning.near_pitch_cdeg;
    multi.input.gripper_done = true;
    multi.input.gripper_feedback_open = 0;
    multi.input.multi_view_finished = true;
    multi.input.multi_view_verdict = 1;
    multi.input.multi_view_inventory.dangerous = 1;
    out = multi.tick();
    assert(multi.task.single_phase_initializing_ && out.batch_size == 0);
    multi.input.multi_view_inventory = {};
    multi.input.multi_view_inventory.ordinary = 1;
    out = multi.tick();
    assert(!multi.task.single_phase_initializing_ && out.batch_size == 1);
    assert(out.phase == TaskPhase::Transport && out.motion.vx_mps == 0);

    Rig absent(tuning);
    absent.task.setSinglePhase("capture");
    absent.tick();
    absent.input.camera_pitch_cdeg = tuning.near_pitch_cdeg;
    for (int n = 0; n < 165; ++n) out = absent.tick();
    assert(out.run_status == RunStatus::Fault && !out.single_phase_done);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
}
} // namespace
int main() {
    {
        Rig rig;
        rig.input.run = false;
        rig.task.last_vx_ = .4f; // target speed before the UART ramp
        rig.task.last_wz_ = .2f;
        rig.input.sent_velocity_valid = true;
        rig.input.sent_vx_mps = .03f;
        rig.input.sent_wz_rps = .02f;
        rig.tick();
        assert(std::abs(rig.task.odo_ - .03f * .05f) < 1e-6f);
    }
    startupIsDeferredAndDoesNotRepeat();
    prepareTransfersInventoryAndRushLimit();
    prepareTransfersInventoryAndRushLimit(true);
    checkpointStartupCaptureAndTransport();
    directPhasesSkipPrecedingPhases();
    pauseKeepsCargoAndRecoveryKeepsItsOwner();
    std::cout << "four-phase contracts passed\n";
}
