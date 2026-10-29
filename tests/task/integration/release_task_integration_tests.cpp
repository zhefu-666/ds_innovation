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
#if __has_include("rescue/task/delivery_verification_stage.hpp")
#include "rescue/task/delivery_verification_stage.hpp"
#endif
#if __has_include("rescue/task/release_retreat_stage.hpp")
#include "rescue/task/release_retreat_stage.hpp"
#endif
#define private public
#include "rescue/task/push_task.hpp"
#undef private

namespace {
using namespace rescue;
constexpr uint64_t kStart = 1000000;
constexpr uint64_t kTick = 50000;
constexpr uint64_t kTransaction = 41;

Inventory ordinary() {
    Inventory inventory;
    inventory.ordinary = 1;
    return inventory;
}

TaskTuning tuning(bool carry = false) {
    TaskTuning value;
    value.require_multi_view = false;
    value.field_carry = carry;
    value.assume_all_safe = carry;
    return value;
}

ZoneEstimate zone(uint64_t timestamp, uint64_t frame, float origin_y) {
    ZoneEstimate estimate;
    estimate.valid = true;
    estimate.source = ZoneEstimate::Source::MULTI_POINT;
    estimate.zone_label = "red_safe_zone";
    estimate.geometry_id = "release-test";
    estimate.frame_id = frame;
    estimate.timestamp_us = estimate.observed_us = timestamp;
    estimate.origin_body_m = {0, origin_y};
    estimate.inlier_ids = {0, 2, 3};
    estimate.residual_m = .001f;
    estimate.position_sigma_m = .002f;
    estimate.yaw_sigma_rad = .01f;
    return estimate;
}

struct Rig {
    PushTask task;
    PushObservation input;
    PushOutput output;
    float origin_y = 0, origin_x = 0;
    bool vref = false, fresh_cargo = true, fresh_navigation = true;
    ZoneEstimate::Source source = ZoneEstimate::Source::MULTI_POINT;
    uint64_t frame = 0;

    explicit Rig(TaskTuning parameters = tuning(), PushState initial = PushState::RAISE_RELEASE)
        : task(parameters) {
        input.now_us = kStart;
        input.run = input.safety_ok = true;
        input.path_safe = input.retreat_safe = input.opponent_zone_clear = true;
        input.zone_valid = input.zone_own = input.zone_identity_verified = true;
        input.camera_pitch_stable = true;
        input.gripper_done = true;
        input.gripper_feedback_open = 1;
        input.drop_plan_valid = true;
        task.trip_context_.trip_ = ordinary();
        task.transport_phase_.drop_locked_ = true;
        task.transport_phase_.drop_centre_zone_ = {parameters.supply_half_x_m,
                                                   parameters.deposit_y_m};
        input.drop_centre_zone = task.transport_phase_.drop_centre_zone_;
        task.frame_transaction_ = task.wire_txn_ = kTransaction;
        task.wire_last_ = 0;
        task.frame_raised_ = 0;
        task.gripper_us_ = kStart;
        seed(initial);
    }

    void seed(PushState state) {
        task.enter(state, input.now_us, "seed_release");
        task.last_us_ = input.now_us;
        task.last_vx_ = task.last_wz_ = 0;
        task.pitch_cmd_ = task.desiredPitch();
        task.pitch_wait_us_ = input.now_us;
        input.camera_pitch_cdeg = task.pitch_cmd_;
    }

    void openFrame() {
        task.frame_raised_ = 1;
        task.wire_last_ = 1;
    }

    void refresh() {
        input.zone_estimate = zone(input.now_us, ++frame, origin_y);
        input.zone_estimate.source = source;
        input.zone_estimate.origin_body_m.x = origin_x;
        if (fresh_navigation)
            input.navigation_timestamp_us = input.now_us;
        if (!input.cargo_body.empty() && fresh_cargo)
            input.cargo_ts_us = input.now_us;
        input.vref_valid = vref;
        input.vref_points = vref ? 3 : 0;
        input.vref_zone =
            zone(input.cargo_body.empty() ? input.now_us : input.cargo_ts_us, frame, origin_y);
        input.vref_zone.origin_body_m.x = origin_x;
    }

    PushOutput tick() {
        input.now_us += kTick;
        refresh();
        output = task.update(input);
        return output;
    }

    PushOutput elapsed(uint64_t duration) {
        input.now_us = task.actionProgress().started_us + duration;
        task.last_us_ = input.now_us - kTick;
        refresh();
        output = task.update(input);
        return output;
    }

    void cargo(float rear_y, bool clipped = false) {
        input.cargo_body = {{task.halfX(), origin_y + rear_y}};
        input.cargo_img = {PushObservation::CargoImg{}};
        input.cargo_img[0].clipped = clipped;
        vref = true;
    }

    void beginBackOff() {
        tick();
        assert(output.state == PushState::BACK_OUT && output.reason == "assumed_release_back_off");
        assert(output.motion.vx_mps == 0 && output.motion.wz_rps == 0);
        assert(output.motion.gripper_offset == 0 &&
               output.motion.frame_transaction == kTransaction + 1);
    }

    void stationary(bool open) const {
        assert(output.motion.vx_mps == 0 && output.motion.wz_rps == 0);
        assert(output.motion.gripper_offset == (open ? 0 : 20));
    }

    void fault(const char* reason, bool open, uint64_t phase = kStart) const {
        assert(output.state == PushState::SAFE_STOP && output.reason == reason);
        assert(task.fault_ && task.actionProgress().started_us == phase);
        stationary(open);
    }

    void report(const char* test, std::string events = {}) const {
        std::replace(events.begin(), events.end(), '\n', '|');
        std::cout << "case=" << test << " state=" << PushTask::name(output.state)
                  << " reason=" << output.reason << " fault=" << task.fault_
                  << " frame=" << int(output.motion.gripper_offset)
                  << " transaction=" << output.motion.frame_transaction
                  << " pitch=" << output.motion.camera_pitch_cdeg << " vx=" << output.motion.vx_mps
                  << " phase_us=" << task.actionProgress().started_us
                  << " travel=" << task.actionProgress().travel
                  << " cargo_ts=" << task.transport_phase_.cargo_last_ts_
                  << " cargo_ref=" << task.transport_phase_.cargo_ref_us_
                  << " repush=" << task.transport_phase_.repush_n_
                  << " pushed=" << task.transport_phase_.assume_pushed_
                  << " blind=" << task.actionProgress().blind
                  << " align=" << task.transport_phase_.push_in_.align_pending_
                  << " align_start=" << task.transport_phase_.push_in_.align_start_us_
                  << " push_hold=" << task.transport_phase_.push_in_.push_hold_valid_
                  << " events=" << events << '\n';
    }
};


void jawEnterReleaseRetreat() {
    for (int mode : {0, 1, 2}) {
        auto parameters = tuning();
        parameters.pixel_grasp_enabled = true;
        parameters.field_carry = mode == 1;
        parameters.assume_all_safe = mode == 2;
        Rig rig(parameters, PushState::ENTER);
        rig.origin_x = -parameters.supply_half_x_m;
        rig.origin_y = rig.task.holdCenter() - parameters.deposit_y_m + .02f;
        rig.input.gripper_feedback_open = 0;
        rig.tick();
        assert(rig.output.state == PushState::ENTER && rig.output.motion.vx_mps > 0);
        assert(rig.output.motion.gripper_offset == 20);
        rig.origin_y = rig.task.holdCenter() - parameters.deposit_y_m;
        rig.tick();
        assert(rig.output.state == PushState::RAISE_RELEASE);
        rig.stationary(false);
        rig.input.gripper_done = false;
        rig.tick();
        assert(rig.output.state == PushState::RAISE_RELEASE);
        rig.stationary(true);
        rig.input.gripper_done = true;
        rig.tick();
        assert(rig.output.state == PushState::RAISE_RELEASE);
        rig.stationary(true);
        rig.input.gripper_feedback_open = 1;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               rig.output.reason == "gripper_opened_at_drop");
        assert(!rig.task.transport_phase_.release_retreat_.needsCargoTracking());
        rig.stationary(true);
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT && rig.output.motion.vx_mps < 0);
        assert(rig.output.motion.gripper_offset == 0);
        rig.origin_y = .5f;
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY);
        rig.stationary(true);
        assert(rig.task.transport_phase_.repush_n_ == 0);
        rig.report("jaw_enter_open_then_retreat");
    }
    {
        auto parameters = tuning(true);
        parameters.pixel_grasp_enabled = true;
        Rig rig(parameters);
        rig.task.transport_phase_.release_retreat_.beginReleaseBackOff(0);
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               !rig.task.transport_phase_.release_retreat_.needsCargoTracking());
        rig.task.actionProgress().travel = 10;
        rig.tick();
        assert(rig.output.state != PushState::ENTER);
        assert(rig.output.motion.gripper_offset == 0);
        rig.report("jaw_release_clears_legacy_prepush");
    }
    {
        auto parameters = tuning(true);
        parameters.pixel_grasp_enabled = true;
        Rig rig(parameters, PushState::CARRY);
        rig.origin_x = -parameters.supply_half_x_m;
        rig.origin_y = rig.task.holdCenter() - .05f;
        rig.vref = true;
        rig.input.gripper_feedback_open = 0;
        rig.tick();
        assert(rig.output.state == PushState::ENTER);
        rig.stationary(false);
        rig.input.camera_pitch_cdeg = rig.task.desiredPitch();
        rig.input.zone_valid = false;
        rig.tick();
        assert(rig.output.state == PushState::ENTER && rig.output.motion.vx_mps > 0);
        assert(rig.output.motion.gripper_offset == 20);
        rig.report("jaw_visual_carry_enters_before_release");
    }
}

void releaseGates() {
    {
        Rig rig;
        rig.input.drop_plan_valid = false;
        rig.input.opponent_zone_clear = false;
        rig.tick();
        assert(rig.output.state == PushState::RAISE_RELEASE);
        assert(rig.output.reason == "drop_revalidation_missing");
        assert(rig.output.motion.frame_transaction == kTransaction);
        rig.stationary(false);
        rig.report("strict_revalidation_before_clearance");
        rig.elapsed(rig.task.t_.carry_budget_us + 1);
        rig.fault("drop_revalidation_timeout", false);
        rig.report("strict_revalidation_timeout");
    }
    {
        Rig rig(tuning(true));
        rig.input.drop_plan_valid = false;
        rig.input.opponent_zone_clear = false;
        rig.elapsed(rig.task.t_.gripper_timeout_us);
        assert(rig.output.state == PushState::RAISE_RELEASE && !rig.task.fault_);
        assert(rig.output.reason == "release_opponent_clearance_missing");
        rig.stationary(false);
        rig.report("release_clearance_exact_deadline");
        rig.elapsed(rig.task.t_.gripper_timeout_us + 1);
        rig.fault("release_clearance_timeout", false);
        rig.report("release_clearance_timeout");
    }
    {
        Rig rig;
        rig.input.camera_pitch_stable = false;
        rig.input.gripper_done = false;
        rig.tick();
        assert(rig.output.state == PushState::RAISE_RELEASE && !rig.task.fault_);
        assert(rig.output.motion.frame_transaction == kTransaction + 1);
        assert(rig.task.gripper_us_ == rig.input.now_us);
        rig.stationary(true);
        rig.report("release_opens_while_pitch_moves");
    }
    {
        auto parameters = tuning();
        parameters.pitch_timeout_us = 100000;
        parameters.gripper_timeout_us = 100000;
        Rig rig(parameters);
        rig.openFrame();
        rig.input.camera_pitch_stable = false;
        rig.input.gripper_done = false;
        rig.elapsed(100001);
        rig.fault("camera_pitch_timeout", true);
        rig.report("release_pitch_timeout_precedes_gripper");
    }
    {
        auto parameters = tuning();
        parameters.pitch_timeout_us = 500000;
        parameters.gripper_timeout_us = 100000;
        Rig rig(parameters);
        rig.openFrame();
        rig.input.camera_pitch_stable = false;
        rig.input.gripper_done = false;
        rig.elapsed(100000);
        assert(rig.output.state == PushState::RAISE_RELEASE && !rig.task.fault_);
        rig.elapsed(100001);
        rig.fault("gripper_open_timeout", true);
        rig.report("release_gripper_timeout_while_pitch_moves");
    }
    {
        Rig rig;
        rig.input.camera_pitch_stable = false;
        rig.tick();
        assert(rig.output.state == PushState::RAISE_RELEASE && !rig.task.fault_);
        rig.input.camera_pitch_stable = true;
        rig.input.gripper_feedback_open = 0;
        rig.tick();
        assert(rig.output.state == PushState::RAISE_RELEASE && !rig.task.fault_);
        rig.input.gripper_feedback_open = 1;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               rig.output.reason == "frame_raised_at_drop");
        rig.stationary(true);
        assert(rig.output.motion.frame_transaction == kTransaction + 1);
        rig.report("release_needs_both_matching_feedbacks");
    }
    for (bool already_pushed : {false, true}) {
        auto parameters = tuning(true);
        if (!already_pushed)
            parameters.assume_push_m = 0;
        Rig rig(parameters);
        rig.task.transport_phase_.assume_pushed_ = already_pushed;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               rig.output.reason == "frame_raised_at_drop");
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               rig.output.reason == "frame_raised_at_drop");
        assert(rig.output.motion.vx_mps < 0);
        rig.report(already_pushed ? "release_already_pushed_no_prepush"
                                  : "release_zero_push_no_prepush");
    }
}

void backOutGatesAndDeadlines() {
    {
        Rig rig(tuning(true), PushState::BACK_OUT);
        rig.cargo(.12f);
        rig.input.camera_pitch_stable = false;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT && rig.output.reason == "camera_pitch_wait");
        assert(rig.task.transport_phase_.cargo_last_ts_ == 0 &&
               rig.output.motion.frame_transaction == kTransaction);
        rig.stationary(false);
        rig.report("back_pitch_wait_precedes_open_and_tracking");
        rig.elapsed(rig.task.t_.pitch_timeout_us + 1);
        rig.fault("camera_pitch_timeout", false);
        rig.report("back_pitch_timeout_keeps_closed_frame");
    }
    {
        Rig rig(tuning(true));
        rig.beginBackOff();
        const auto phase = rig.task.actionProgress().started_us;
        rig.cargo(.12f);
        rig.input.gripper_feedback_open = 0;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               rig.task.transport_phase_.cargo_last_ts_ == 0);
        rig.stationary(true);
        rig.elapsed(rig.task.t_.gripper_timeout_us + 1);
        rig.fault("gripper_open_timeout", true, phase);
        assert(rig.task.transport_phase_.cargo_last_ts_ == 0);
        rig.report("back_frame_wait_precedes_tracking");
    }
    {
        Rig rig(tuning(), PushState::BACK_OUT);
        rig.openFrame();
        rig.input.zone_valid = false;
        rig.elapsed(rig.task.t_.retreat_budget_us);
        assert(rig.output.state == PushState::BACK_OUT && !rig.task.fault_);
        rig.stationary(true);
        rig.elapsed(rig.task.t_.retreat_budget_us + 1);
        rig.fault("retreat_reference_lost", true);
        rig.report("strict_back_missing_reference_timeout");
    }
    {
        Rig rig(tuning(), PushState::BACK_OUT);
        rig.openFrame();
        rig.source = ZoneEstimate::Source::PREDICTED;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT && !rig.task.fault_);
        rig.stationary(true);
        rig.report("strict_back_rejects_predicted_reference");
    }
    {
        Rig rig(tuning(), PushState::BACK_OUT);
        rig.openFrame();
        rig.elapsed(rig.task.t_.retreat_budget_us);
        assert(rig.output.state == PushState::BACK_OUT && rig.output.motion.vx_mps < 0);
        rig.report("strict_back_exact_deadline_drives");
        rig.elapsed(rig.task.t_.retreat_budget_us + 1);
        rig.fault("retreat_blocked", true);
        rig.report("strict_back_blocked_timeout");
    }
    {
        Rig rig(tuning(), PushState::BACK_OUT);
        rig.openFrame();
        rig.origin_y = .5f;
        rig.elapsed(rig.task.t_.retreat_budget_us + 1);
        assert(rig.output.state == PushState::VERIFY_DELIVERY && rig.output.reason == "backed_out");
        assert(!rig.task.fault_ && rig.task.actionProgress().started_us == rig.input.now_us);
        rig.stationary(true);
        rig.report("strict_back_clear_precedes_timeout");
    }
    {
        Rig rig(tuning(), PushState::BACK_OUT);
        rig.openFrame();
        rig.input.retreat_safe = false;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               rig.output.reason == "retreat_clearance_missing");
        rig.stationary(true);
        rig.report("back_drive_uses_shared_clearance_gate");
    }
    {
        Rig rig(tuning(true), PushState::BACK_OUT);
        rig.openFrame();
        rig.input.zone_valid = false;
        rig.tick();
        assert(rig.output.reason == "assumed_blind_back_out" && rig.task.actionProgress().blind);
        rig.input.zone_valid = true;
        rig.origin_y = .5f;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               rig.output.reason == "assumed_blind_back_out");
        assert(rig.output.motion.vx_mps < 0 && rig.task.actionProgress().blind);
        rig.report("assumed_back_blind_choice_stays_latched");
        rig.task.actionProgress().travel = rig.task.t_.assume_back_m;
        rig.task.last_vx_ = 0;
        rig.elapsed(rig.task.t_.retreat_budget_us + 1);
        assert(rig.output.state == PushState::VERIFY_DELIVERY &&
               rig.output.reason == "assumed_backed_out");
        assert(!rig.task.fault_ && !rig.task.actionProgress().blind);
        rig.stationary(true);
        rig.report("assumed_back_distance_precedes_timeout");
    }
    {
        Rig rig(tuning(true), PushState::BACK_OUT);
        rig.openFrame();
        rig.input.zone_valid = false;
        rig.elapsed(rig.task.t_.retreat_budget_us + 1);
        rig.fault("retreat_blocked", true);
        assert(rig.task.actionProgress().blind);
        rig.report("assumed_back_timeout_keeps_phase");
    }
}

void prepushProgressAndEvidence() {
    {
        Rig rig(tuning(true));
        rig.beginBackOff();
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               rig.output.reason == "assumed_release_back_off_blind");
        rig.task.actionProgress().travel =
            rig.task.t_.assume_prepush_back_m * rig.task.drRatio(rig.input.now_us);
        rig.task.last_vx_ = 0;
        rig.task.transport_phase_.cargo_ref_us_ = 123;
        rig.task.transport_phase_.push_in_.align_start_us_ = 456;
        rig.task.transport_phase_.push_in_.push_hold_valid_ = true;
        rig.task.t_.assume_align_push = true;
        rig.tick();
        assert(rig.output.state == PushState::ENTER &&
               rig.output.reason == "assumed_release_then_push");
        assert(rig.task.transport_phase_.cargo_ref_us_ == 0 &&
               rig.task.transport_phase_.push_in_.align_pending_);
        assert(rig.task.transport_phase_.push_in_.align_start_us_ == 0 &&
               !rig.task.transport_phase_.push_in_.push_hold_valid_);
        assert(rig.task.actionProgress().travel == 0 && !rig.task.fault_);
        rig.stationary(true);
        rig.report("prepush_dead_reckoning_resets_push_alignment");
        rig.input.gripper_feedback_open = 0;
        rig.tick();
        assert(rig.output.motion.gripper_offset == 20 &&
               rig.output.motion.frame_transaction == kTransaction + 2);
        rig.report("prepush_next_tick_lowers_frame");
    }
    {
        Rig rig(tuning(true));
        rig.vref = true;
        rig.beginBackOff();
        rig.task.zone_.advance(-rig.task.t_.assume_prepush_back_m, 0);
        rig.vref = false;
        rig.tick();
        assert(rig.output.state == PushState::ENTER &&
               rig.output.reason == "assumed_release_then_push");
        assert(!rig.task.fault_);
        rig.stationary(true);
        rig.report("prepush_anchored_geometry_completion");
    }
    {
        Rig rig(tuning(true));
        rig.beginBackOff();
        rig.elapsed(rig.task.t_.retreat_budget_us + 1);
        assert(rig.output.state == PushState::ENTER &&
               rig.output.reason == "assumed_release_then_push");
        assert(!rig.task.fault_);
        rig.stationary(true);
        rig.report("prepush_timeout_continues_to_push");
    }
    {
        auto parameters = tuning(true);
        parameters.assume_inside_after_release_obs = 1;
        Rig rig(parameters);
        rig.cargo(.12f);
        rig.beginBackOff();
        rig.elapsed(rig.task.t_.retreat_budget_us + 1);
        assert(rig.output.state == PushState::VERIFY_DELIVERY &&
               rig.output.reason == "assumed_inside_after_release");
        assert(rig.task.transport_phase_.assume_pushed_ && !rig.task.fault_);
        rig.stationary(true);
        rig.report("prepush_inside_precedes_timeout", rig.task.takeDropZoneEvent());
    }
    {
        Rig rig(tuning(true), PushState::ENTER);
        rig.input.gripper_feedback_open = 0;
        rig.input.imu_blocked_us = rig.task.t_.assume_imu_block_us;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               rig.output.reason == "assumed_stuck_repush");
        assert(rig.task.transport_phase_.repush_n_ == 1 &&
               rig.task.transport_phase_.cargo_ref_us_ == 0);
        assert(rig.output.motion.frame_transaction == kTransaction);
        rig.stationary(false);
        const auto events = rig.task.takeDropZoneEvent();
        assert(events.find("event=stuck_repush") != std::string::npos);
        assert(events.find("repush=1 in=0 out=0") != std::string::npos);
        rig.report("enter_stuck_starts_back_off_without_opening", events);
        rig.input.gripper_feedback_open = 1;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT && rig.output.motion.vx_mps < 0);
        rig.task.last_vx_ = 0;
        rig.task.actionProgress().travel =
            rig.task.t_.assume_prepush_back_m * rig.task.drRatio(rig.input.now_us);
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT);
        rig.task.last_vx_ = 0;
        rig.task.actionProgress().travel =
            (rig.task.t_.assume_prepush_back_m + rig.task.t_.assume_repush_back_m) *
            rig.task.drRatio(rig.input.now_us);
        rig.tick();
        assert(rig.output.state == PushState::ENTER &&
               rig.output.reason == "assumed_release_then_push");
        rig.stationary(true);
        assert(rig.output.motion.frame_transaction == kTransaction + 1);
        rig.report("repush_back_off_uses_incremented_distance");
    }
    {
        Rig rig(tuning(true));
        rig.seed(PushState::VERIFY_DELIVERY);
        rig.openFrame();
        rig.input.zone_counts_valid = false;
        rig.cargo(.05f);
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY);
        rig.task.takeDropZoneEvent();
        rig.seed(PushState::RAISE_RELEASE);
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT);
        rig.cargo(.12f);
        rig.task.dropzone_evt_ = "prefix\n";
        for (int index = 0; index < 2; ++index) {
            rig.tick();
            assert(rig.output.state == PushState::BACK_OUT);
        }
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY &&
               rig.output.reason == "assumed_inside_after_release");
        assert(rig.task.transport_phase_.assume_pushed_ && !rig.task.fault_);
        rig.stationary(true);
        rig.task.noteCargo(
            "after_clear",
            {rig.task.transport_phase_.cargo_rear_x_, rig.task.transport_phase_.cargo_rear_y_},
            true);
        const auto events = rig.task.takeDropZoneEvent();
        assert(events.find("prefix\n[CARGO_ZONE] event=inside_after_release") == 0);
        assert(events.find("repush=0 in=1 out=0\n[CARGO_ZONE] event=after_clear") !=
               std::string::npos);
        assert(events.find("event=after_clear") < events.find("repush=0 in=0 out=0"));
        rig.report("inside_event_before_visual_counter_clear", events);
    }
    {
        auto parameters = tuning(true);
        parameters.assume_inside_after_release_obs = 2;
        Rig rig(parameters);
        rig.cargo(.12f);
        rig.beginBackOff();
        rig.tick();
        const auto timestamp = rig.task.transport_phase_.cargo_last_ts_;
        rig.fresh_cargo = false;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT);
        rig.seed(PushState::RAISE_RELEASE);
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT);
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               rig.task.transport_phase_.cargo_last_ts_ == timestamp);
        rig.fresh_cargo = true;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT);
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY);
        rig.report("prepush_new_release_retains_timestamp_only", rig.task.takeDropZoneEvent());
    }
    for (bool clipped : {false, true}) {
        Rig rig(tuning(true));
        rig.cargo(.12f, clipped);
        rig.beginBackOff();
        if (!clipped)
            rig.task.transport_phase_.repush_n_ = 1;
        for (int index = 0; index < 4; ++index)
            rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               !rig.task.transport_phase_.assume_pushed_);
        assert(rig.task.takeDropZoneEvent().empty());
        rig.report(clipped ? "prepush_clipped_cargo_cannot_shortcut"
                           : "prepush_repush_cannot_shortcut");
    }
    {
        auto parameters = tuning(true);
        parameters.assume_inside_after_release_obs = 2;
        Rig rig(parameters);
        rig.cargo(.12f);
        rig.beginBackOff();
        rig.tick();
        rig.cargo(.02f);
        rig.tick();
        rig.cargo(.12f);
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT);
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY);
        rig.report("prepush_failed_inside_resets_consecutive_count", rig.task.takeDropZoneEvent());
    }
}

void lifecycleAndRecovery() {
    {
        Rig rig(tuning(true));
        rig.vref = true;
        rig.beginBackOff();
        rig.vref = false;
        rig.task.zone_.advance(-.1f, 0);
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT);
        rig.input.run = false;
        rig.tick();
        rig.input.run = true;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT && rig.task.actionProgress().travel == 0);
        rig.task.zone_.advance(-.1f, 0);
        rig.tick();
        assert(rig.output.state == PushState::ENTER &&
               rig.output.reason == "assumed_release_then_push");
        assert(!rig.task.fault_);
        rig.report("prepush_pause_preserves_visual_retreat_origin");
    }
    {
        auto parameters = tuning(true);
        parameters.assume_inside_after_release_obs = 2;
        Rig rig(parameters);
        rig.cargo(.12f);
        rig.beginBackOff();
        rig.tick();
        rig.fresh_cargo = false;
        rig.input.run = false;
        rig.tick();
        assert(rig.output.state == PushState::WAIT_START &&
               rig.task.stopped_from_ == PushState::BACK_OUT);
        rig.input.run = true;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT && rig.output.reason == "resume");
        assert(rig.task.actionProgress().started_us == rig.input.now_us &&
               rig.task.actionProgress().travel == 0);
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT);
        rig.fresh_cargo = true;
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY &&
               rig.task.transport_phase_.assume_pushed_);
        rig.report("prepush_pause_retains_count_and_duplicate_marker",
                   rig.task.takeDropZoneEvent());
    }
    for (auto state : {PushState::RAISE_RELEASE, PushState::ENTER, PushState::BACK_OUT}) {
        Rig rig(tuning(true), state);
        rig.input.run = false;
        rig.tick();
        assert(rig.output.state == PushState::WAIT_START && rig.task.stopped_from_ == state);
        rig.stationary(false);
        rig.input.run = true;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT && rig.output.reason == "resume");
        rig.stationary(false);
        assert(rig.output.motion.frame_transaction == kTransaction);
        rig.report(state == PushState::RAISE_RELEASE ? "resume_release_defers_open"
                   : state == PushState::ENTER       ? "resume_enter_defers_open"
                                                     : "resume_back_defers_open");
        rig.tick();
        assert(rig.output.motion.frame_transaction == kTransaction + 1);
        assert(rig.output.motion.gripper_offset == 0);
    }
    {
        auto parameters = tuning(true);
        parameters.assume_inside_after_release_obs = 2;
        Rig rig(parameters);
        rig.cargo(.12f);
        rig.beginBackOff();
        rig.tick();
        rig.input.reset = true;
        rig.tick();
        assert(rig.output.state == PushState::WAIT_START && rig.output.batch_size == 0 &&
               !rig.task.fault_);
        rig.input.reset = false;
        rig.task.trip_context_.trip_ = ordinary();
        rig.task.transport_phase_.drop_locked_ = true;
        rig.task.transport_phase_.drop_centre_zone_ = rig.input.drop_centre_zone;
        rig.seed(PushState::RAISE_RELEASE);
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT);
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT);
        rig.tick();
        assert(rig.output.state == PushState::VERIFY_DELIVERY);
        rig.report("reset_clears_prepush_observation_count", rig.task.takeDropZoneEvent());
    }
    {
        Rig rig(tuning(true));
        rig.beginBackOff();
        rig.task.sweep_close_wait_ = true;
        rig.task.sweep_close_us_ = rig.input.now_us;
        rig.task.frame_raised_ = 0;
        rig.task.wire_last_ = 0;
        rig.input.gripper_done = false;
        rig.cargo(.12f);
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               rig.output.reason == "sweep_frame_close_wait");
        assert(rig.task.transport_phase_.cargo_last_ts_ == 0);
        rig.stationary(false);
        rig.report("shared_close_wait_precedes_back_case");
    }
}
} // namespace

int main() {
    jawEnterReleaseRetreat();
    releaseGates();
    backOutGatesAndDeadlines();
    prepushProgressAndEvidence();
    lifecycleAndRecovery();
    std::cout << "release task integration tests passed\n";
}
