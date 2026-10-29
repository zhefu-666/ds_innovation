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
    assert(out.phase == TaskPhase::Search && out.search_action == SearchStatusAction::Scan);
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
    rig.task.first_ = true; // the two-object handoff is a later trip
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
    assert(out.phase == TaskPhase::Search && out.search_action == SearchStatusAction::Scan);
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
    assert(out.phase_waiting && out.phase == TaskPhase::Startup &&
           out.next_phase == TaskPhase::Search);
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
    assert(out.phase_waiting && out.phase == TaskPhase::Capture &&
           out.next_phase == TaskPhase::Transport);
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
    assert(out.phase_waiting && out.phase == TaskPhase::Transport &&
           out.next_phase == TaskPhase::Search);
    assert(out.delivery_expected == 1 && out.delivery_credited == 0);
    delivery.input.run = delivery.input.safety_ok = false;
    for (int n = 0; n < 5; ++n) {
        out = delivery.tick();
        assert(out.phase_waiting && out.delivered_total == 0);
    }
    delivery.input.run = delivery.input.safety_ok = true;
    out = delivery.tick();
    assert(!out.phase_waiting && out.phase == TaskPhase::Search);
    assert(out.search_action == SearchStatusAction::TurnAway);
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
    capture.input.target_valid = capture.input.geometry_valid = capture.input.target_region_valid =
        true;
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
    transport.input.hold_observable = transport.input.captured = transport.input.held_complete =
        true;
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
    for (int n = 0; n < 165; ++n)
        out = absent.tick();
    assert(out.run_status == RunStatus::Fault && !out.single_phase_done);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
}

void metricOvercaptureDoesNotStartSplit(bool single, int extra_kind,
                                       bool multiview = false, bool raw_held = false) {
    TaskTuning tuning;
    tuning.require_multi_view = multiview;
    tuning.field_rush = tuning.verify_held_contents = raw_held;
    tuning.held_recheck = false;
    tuning.capture_frames = 1;
    Rig rig(tuning);
    if (single)
        rig.task.setSinglePhase("capture");
    rig.task.enter(PushState::VERIFY_CAPTURE, kStart, "metric_overcapture");
    rig.task.trip_context_.label_ = "ordinary_supply";
    rig.task.trip_context_.target_id_ = 7;
    rig.task.trip_context_.pending_.ordinary = 1;
    rig.input.hold_observable = rig.input.captured = rig.input.held_complete = true;
    rig.input.held.ordinary = 1;
    if (extra_kind == 0)
        ++rig.input.held.ordinary;
    if (extra_kind == 1)
        ++rig.input.held.core;
    if (extra_kind == 2)
        ++rig.input.held.injured;
    if (extra_kind == 3)
        ++rig.input.held.dangerous;
    if (extra_kind == 4)
        ++rig.input.held.unknown;
    if (multiview && !raw_held) {
        rig.input.multi_view_finished = true;
        rig.input.multi_view_verdict = 1;
        rig.input.multi_view_inventory = rig.input.held;
    }
    rig.input.camera_pitch_cdeg = rig.task.desiredPitch();
    auto out = rig.tick();
    if (raw_held)
        out = rig.tick();
    assert(!rig.task.search_phase_.first_trip_split_.pending);
    assert(!rig.task.search_phase_.first_trip_split_.capture_retry);
    assert(out.capture_split_runs == 0 && !out.first_ordinary_delivered);
    assert(out.capture_evidence != CaptureEvidence::Verified);
    if (single)
        assert(out.run_status == RunStatus::Fault);
    else
        assert(out.state == PushState::ABORT_DROP && out.recovering);
}

void metricInitialCaptureRejectsIllegalCorridor(int extra_kind) {
    TaskTuning tuning;
    tuning.require_multi_view = false;
    tuning.corridor_confirm_frames = 1;
    Rig rig(tuning);
    rig.task.setSinglePhase("capture");
    rig.tick();
    rig.input.camera_pitch_cdeg = rig.task.desiredPitch();
    rig.input.gripper_done = true;
    rig.input.gripper_feedback_open = 1;
    rig.input.target_valid = rig.input.geometry_valid = rig.input.target_region_valid = true;
    rig.input.target_id = 1;
    rig.input.label = "ordinary_supply";
    rig.input.distance_m = .4f;
    rig.input.corridor.ordinary = 1;
    if (extra_kind == 0)
        ++rig.input.corridor.ordinary;
    if (extra_kind == 1)
        ++rig.input.corridor.core;
    if (extra_kind == 2)
        ++rig.input.corridor.injured;
    if (extra_kind == 3)
        ++rig.input.corridor.dangerous;
    if (extra_kind == 4)
        ++rig.input.corridor.unknown;
    rig.input.corridor_complete = rig.input.corridor_occlusion_free = true;
    for (int n = 0; n < 12; ++n) {
        const auto out = rig.tick();
        assert(rig.task.single_phase_initializing_);
        assert(!out.recovering && !out.single_phase_done);
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        assert(out.capture_split_runs == 0);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    }
}

void splitEvidenceAndLifetime() {
    TaskTuning tuning;
    tuning.require_multi_view = false;
    tuning.capture_frames = 1;
    for (bool later : {false, true}) {
        Rig rig(tuning);
        rig.task.first_ = later;
        rig.task.enter(PushState::VERIFY_CAPTURE, kStart, "split_evidence");
        rig.task.trip_context_.target_id_ = 7;
        rig.task.trip_context_.label_ = "ordinary_supply";
        rig.task.trip_context_.pending_.ordinary = later ? 2 : 1;
        rig.input.camera_pitch_cdeg = rig.task.desiredPitch();
        rig.input.hold_observable = rig.input.captured = true;
        rig.input.held.ordinary = 2;
        // Incomplete held evidence cannot start separation.
        rig.tick();
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        rig.input.held_complete = true;
        auto out = rig.tick();
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        if (later)
            assert(out.phase == TaskPhase::Transport && out.batch_size == 2);
        rig.input.reset = true;
        rig.tick();
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        assert(!rig.task.search_phase_.first_trip_split_.capture_retry);
    }
    Rig absent(tuning);
    absent.task.enter(PushState::VERIFY_CAPTURE, kStart, "no_ordinary");
    absent.task.trip_context_.label_ = "ordinary_supply";
    absent.input.camera_pitch_cdeg = absent.task.desiredPitch();
    absent.input.hold_observable = absent.input.captured = absent.input.held_complete = true;
    absent.input.held.core = 1;
    absent.input.held.dangerous = 1;
    absent.tick();
    assert(!absent.task.search_phase_.first_trip_split_.pending);
}

void pixelSplitDirectionAndMissingBox() {
    TaskTuning tuning;
    tuning.pixel_grasp_enabled = true;
    tuning.require_multi_view = false;
    for (int side : {-1, 1}) {
        Rig rig(tuning);
        auto& split = rig.task.search_phase_.first_trip_split_;
        split.pending = split.capture_retry = true;
        split.started_us = kStart;
        rig.task.enter(PushState::SCAN, kStart, "pixel_split_direction");
        rig.input.camera_pitch_cdeg = 0;
        rig.input.gripper_done = true;
        rig.input.gripper_feedback_open = 1;
        rig.input.split_ordinary_box = {600 + side * 50, 0, 80, 80};
        rig.input.split_ordinary_box_id = 7;
        rig.input.split_image_width_px = 1280;
        rig.input.split_ordinary_box_us = rig.input.now_us + kTick;
        auto out = rig.tick();
        assert(out.state == PushState::PILE && split.pixel_target_x == 640 + side * 20);
        for (int n = 0; n < 12 && out.motion.wz_rps == 0; ++n) {
            rig.input.split_ordinary_box_us = rig.input.now_us + kTick;
            out = rig.tick();
        }
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps * side < 0);
        for (int missing = 0; missing < 3; ++missing) {
            rig.input.split_ordinary_box_us = missing == 2 ? rig.input.now_us + 2 * kTick
                                                         : rig.input.now_us - tuning.frame_timeout_us;
            if (missing == 0)
                rig.input.split_ordinary_box = {};
            else
                rig.input.split_ordinary_box = {600 + side * 20, 0, 80, 80};
            out = rig.tick();
            assert(out.state == PushState::PILE && out.motion.camera_pitch_cdeg == 0);
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0 && out.immediate_motion_stop);
        }
    }
}

// Exercise the actual task entry, including ordinary selector release/rebound events.
struct PixelSplitRig : Rig {
    explicit PixelSplitRig(float x = 700) : Rig(tuning()) {
        auto& split = task.search_phase_.first_trip_split_;
        split.pending = split.capture_retry = true;
        split.started_us = kStart;
        task.enter(PushState::SCAN, kStart, "pixel_split_regression");
        input.camera_pitch_cdeg = 0;
        input.gripper_done = true;
        input.gripper_feedback_open = 1;
        input.split_image_width_px = 1280;
        input.split_ordinary_box_id = 7;
        input.sent_velocity_valid = true;
        center(x);
    }
    static TaskTuning tuning() {
        TaskTuning t;
        t.pixel_grasp_enabled = true;
        t.require_multi_view = false;
        return t;
    }
    void center(float x) { input.split_ordinary_box = {int(std::lround(x)) - 40, 0, 80, 80}; }
    PushOutput step() {
        input.split_ordinary_box_us = input.now_us + kTick;
        auto out = tick();
        input.sent_vx_mps = out.motion.vx_mps;
        input.sent_wz_rps = out.motion.wz_rps;
        return out;
    }
};

void pixelSplitPulseAndReselection() {
    PixelSplitRig rig;
    auto out = rig.step();
    assert(out.state == PushState::PILE && out.split_align_target_x == 660);
    while (out.motion.wz_rps == 0)
        out = rig.step();
    const auto pulse_start = rig.input.now_us;
    assert(out.motion.wz_rps < 0 && std::abs(out.motion.wz_rps) <= .10f);
    // Loss/replacement while moving must stop, retain the old ID and never restart the pulse.
    rig.input.split_ordinary_box_id = 8;
    out = rig.step();
    assert(out.immediate_motion_stop && out.motion.wz_rps == 0);
    assert(out.split_align_target_id == 7);
    const auto stop_at = rig.input.now_us;
    for (int i = 0; i < 4; ++i) {
        out = rig.step();
        assert(out.motion.wz_rps == 0 && out.split_align_target_id == 7);
    }
    while (out.split_align_target_id == 7) {
        out = rig.step();
        assert(out.motion.wz_rps == 0);
        assert(rig.input.now_us - stop_at < 1000000);
    }
    assert(rig.input.now_us - stop_at >= 250000 && out.split_align_target_id == 8);
    assert(out.target_id == 8);
    // Shared selector release/rebound must not clear this independently confirmed split ID.
    rig.input.pixel_selection_managed = true;
    rig.input.pixel_released_id = rig.input.pixel_rebound_from = 8;
    out = rig.step();
    assert(out.target_id == 8 && out.split_align_target_id == 8);
    rig.input.heading_valid = false;
    out = rig.step();
    assert(out.immediate_motion_stop && out.motion.wz_rps == 0);
    assert(pulse_start < rig.input.now_us);
}

void pixelSplitStationaryAndHysteresis() {
    PixelSplitRig rig;
    auto out = rig.step();
    rig.center(652); // within 10 px; hysteresis permits a small subsequent excursion
    for (int i = 0; i < 4; ++i) {
        out = rig.step();
        assert(out.state == PushState::PILE && out.motion.wz_rps == 0);
    }
    rig.center(646); // 14 px error, 6 px frame window: remain accepted until 16 px
    for (int i = 0; i < 10 && out.state == PushState::PILE; ++i)
        out = rig.step();
    assert(out.state == PushState::APPROACH && out.motion.vx_mps == 0);
    assert(out.reason == "first_trip_pixel_split_advance");
    // A zero UART command alone must not permit advance while the IMU still turns.
    PixelSplitRig moving;
    out = moving.step();
    moving.center(660);
    for (int i = 0; i < 20; ++i) {
        moving.input.heading_rad += .02f;
        out = moving.step();
        assert(out.state == PushState::PILE && out.motion.wz_rps == 0);
    }
    // Repeated capture timestamps cannot satisfy the stationary frame requirement.
    const auto old_sample = moving.input.split_ordinary_box_us;
    for (int i = 0; i < 3; ++i) {
        moving.input.split_ordinary_box_us = old_sample;
        out = moving.tick();
        assert(out.state == PushState::PILE);
    }
    for (int i = 0; i < 20 && out.state == PushState::PILE; ++i)
        out = moving.step();
    assert(out.state == PushState::APPROACH);
    // The UI120 safety permission still vetoes each pulse.
    PixelSplitRig blocked;
    blocked.input.directional_clearance_valid = true;
    blocked.input.turn_safe = false;
    for (int i = 0; i < 25; ++i) {
        out = blocked.step();
        assert(out.motion.wz_rps == 0 && out.motion.vx_mps == 0);
    }
}

void pixelSplitDelayedChassisConverges() {
    for (int side : {-1, 1}) {
        PixelSplitRig rig(640 + side * 60);
        float x = 640 + side * 60;
        std::array<float, 3> delay{}; // 150 ms response lag, including motion after zero TX
        PushOutput out;
        uint64_t pulse_start = 0, stop_at = 0;
        bool turning = false, saw_residual = false;
        for (int i = 0; i < 500; ++i) {
            const float actual_wz = delay[0];
            x += actual_wz * .05f * 2000.f;
            rig.input.heading_rad += actual_wz * .05f;
            rig.center(x);
            out = rig.step();
            assert(out.motion.vx_mps == 0 && std::abs(out.motion.wz_rps) <= .10f);
            assert(out.run_status != RunStatus::Fault);
            const bool current = out.motion.wz_rps != 0;
            saw_residual |= !current && actual_wz != 0;
            if (current && !turning) {
                if (stop_at)
                    assert(rig.input.now_us - stop_at >= 250000);
                pulse_start = rig.input.now_us;
            }
            if (current)
                assert(rig.input.now_us - pulse_start < 100000);
            if (!current && turning)
                stop_at = rig.input.now_us;
            turning = current;
            delay[0] = delay[1];
            delay[1] = delay[2];
            delay[2] = out.motion.wz_rps;
            if (out.state == PushState::APPROACH)
                break;
        }
        assert(saw_residual && out.state == PushState::APPROACH);
        assert(std::abs(x - (640 + side * 20)) <= 16.f);
        std::cout << "split_delay side=" << side << " final_x=" << x
                  << " elapsed_us=" << rig.input.now_us - kStart << '\n';
    }
}

void initialCaptureNeedsFreshCompleteEvidence() {
    TaskTuning tuning;
    tuning.pixel_grasp_enabled = true;
    tuning.require_multi_view = false;
    tuning.corridor_confirm_frames = 1;
    for (int missing = 0; missing < 6; ++missing) {
        Rig rig(tuning);
        rig.task.setSinglePhase("capture");
        rig.tick();
        rig.input.camera_pitch_cdeg = tuning.near_pitch_cdeg;
        rig.input.gripper_done = true;
        rig.input.gripper_feedback_open = 1;
        rig.input.target_valid = true;
        rig.input.target_id = 1;
        rig.input.label = "ordinary_supply";
        rig.input.pixel_grasp_view = rig.input.pixel_grasp_near_view = true;
        rig.input.pixel_grasp_target_valid = true;
        rig.input.pixel_bearing_valid = true;
        rig.input.pixel_bearing = .247f;
        rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
        rig.input.corridor.ordinary = 2;
        rig.input.corridor_complete = rig.input.corridor_occlusion_free = true;
        if (missing == 0)
            rig.input.heading_valid = false;
        if (missing == 1)
            rig.input.pixel_bearing_valid = false;
        if (missing == 2)
            rig.input.pixel_grasp_sample_us = rig.input.now_us - tuning.frame_timeout_us;
        if (missing == 3)
            rig.input.camera_pitch_stable = false;
        if (missing == 4)
            rig.input.corridor_complete = false;
        if (missing == 5)
            rig.input.gripper_done = false;
        const auto out = rig.tick();
        assert(rig.task.single_phase_initializing_);
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        assert(!out.recovering && out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    }
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
    // Metric entry points reject illegal cargo without starting the removed cut.
    for (int extra_kind = 0; extra_kind < 5; ++extra_kind)
        metricInitialCaptureRejectsIllegalCorridor(extra_kind);
    pixelSplitDirectionAndMissingBox();
    pixelSplitPulseAndReselection();
    pixelSplitStationaryAndHysteresis();
    pixelSplitDelayedChassisConverges();
    initialCaptureNeedsFreshCompleteEvidence();
    for (bool single : {false, true})
        for (int extra_kind = 0; extra_kind < 5; ++extra_kind)
            metricOvercaptureDoesNotStartSplit(single, extra_kind);
    metricOvercaptureDoesNotStartSplit(true, 1, true);
    for (int extra_kind = 0; extra_kind < 5; ++extra_kind)
        metricOvercaptureDoesNotStartSplit(true, extra_kind, true, true);
    splitEvidenceAndLifetime();
    startupIsDeferredAndDoesNotRepeat();
    prepareTransfersInventoryAndRushLimit();
    prepareTransfersInventoryAndRushLimit(true);
    checkpointStartupCaptureAndTransport();
    directPhasesSkipPrecedingPhases();
    pauseKeepsCargoAndRecoveryKeepsItsOwner();
    std::cout << "four-phase contracts passed\n";
}
