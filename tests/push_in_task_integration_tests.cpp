#include "rescue/transport_rules.hpp"
#include "rescue/types.hpp"
#include "rescue/zone_estimate.hpp"
#include "rescue/zone_dead_reckoning.hpp"
#include "rescue/planner.hpp"
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
#if __has_include("rescue/task/push_in_stage.hpp")
#include "rescue/task/push_in_stage.hpp"
#endif
#define private public
#include "rescue/push_task.hpp"
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

TaskTuning tuning(bool carry = true) {
    TaskTuning value;
    value.require_multi_view = false;
    value.field_carry = carry;
    value.assume_probe = false;
    return value;
}

ZoneEstimate zone(uint64_t timestamp, uint64_t frame, cv::Point2f origin) {
    ZoneEstimate estimate;
    estimate.valid = true;
    estimate.source = ZoneEstimate::Source::MULTI_POINT;
    estimate.zone_label = "red_safe_zone";
    estimate.geometry_id = "push-in-test";
    estimate.frame_id = frame;
    estimate.timestamp_us = estimate.observed_us = timestamp;
    estimate.origin_body_m = origin;
    estimate.inlier_ids = {0, 2, 3};
    estimate.residual_m = .001f;
    estimate.position_sigma_m = .002f;
    estimate.yaw_sigma_rad = .01f;
    return estimate;
}

bool close(float left, float right) {
    return std::abs(left - right) < 1e-5f;
}

struct Rig {
    PushTask task;
    PushObservation input;
    PushOutput output;
    float hold_y = 0, hold_x;
    bool vref = false, fresh_cargo = true, fresh_navigation = true;
    uint64_t frame = 0;

    explicit Rig(TaskTuning parameters = tuning())
        : task(parameters), hold_x(parameters.supply_half_x_m) {
        input.now_us = kStart;
        input.run = input.safety_ok = true;
        input.path_safe = input.retreat_safe = input.opponent_zone_clear = true;
        input.zone_valid = input.zone_own = input.zone_identity_verified = true;
        input.camera_pitch_stable = true;
        input.gripper_done = true;
        input.gripper_feedback_open = 0;
        input.drop_plan_valid = true;
        task.trip_ = ordinary();
        task.drop_locked_ = true;
        task.drop_centre_zone_ = {parameters.supply_half_x_m, parameters.deposit_y_m};
        input.drop_centre_zone = task.drop_centre_zone_;
        task.frame_transaction_ = task.wire_txn_ = kTransaction;
        task.wire_last_ = 0;
        task.frame_raised_ = 0;
        task.gripper_us_ = kStart;
        task.carry_us_ = kStart;
        seed(PushState::ENTER);
    }

    ZoneEstimate estimate(uint64_t timestamp) const {
        return zone(timestamp, frame, {-hold_x, task.holdCenter() - hold_y});
    }

    void seed(PushState state) {
        task.enter(state, input.now_us, "seed_push_in");
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
        ++frame;
        input.zone_estimate = estimate(input.now_us);
        if (fresh_navigation)
            input.navigation_timestamp_us = input.now_us;
        if (!input.cargo_body.empty() && fresh_cargo)
            input.cargo_ts_us = input.now_us;
        input.vref_valid = vref;
        input.vref_points = vref ? 3 : 0;
        input.vref_zone = estimate(input.cargo_body.empty() ? input.now_us : input.cargo_ts_us);
    }

    PushOutput tick() {
        input.now_us += kTick;
        refresh();
        output = task.update(input);
        return output;
    }

    PushOutput elapsed(uint64_t duration) {
        input.now_us = task.phase_us_ + duration;
        task.last_us_ = input.now_us - kTick;
        task.last_vx_ = task.last_wz_ = 0;
        refresh();
        output = task.update(input);
        return output;
    }

    void cargo(float rear_y, float offset_x = 0, bool clipped = false) {
        input.cargo_body = {estimate(input.now_us).zoneToBody({task.halfX() + offset_x, rear_y})};
        assert(input.cargo_body.front().y > 0);
        input.cargo_img = {PushObservation::CargoImg{}};
        input.cargo_img[0].clipped = clipped;
        vref = true;
    }

    void forgetCargo() {
        input.cargo_body.clear();
        input.cargo_img.clear();
        vref = false;
    }

    void alignedEntry() {
        seed(PushState::RAISE_RELEASE);
        input.gripper_feedback_open = 1;
        vref = false;
        input.cargo_body.clear();
        tick();
        assert(output.state == PushState::BACK_OUT && output.reason == "assumed_release_back_off");
        task.zone_.reset();
        task.travel_ = 10;
        tick();
        assert(output.state == PushState::ENTER && output.reason == "assumed_release_then_push");
        input.gripper_feedback_open = 0;
    }

    void closed() const {
        assert(output.motion.gripper_offset == 20);
    }

    void stationary() const {
        assert(output.motion.vx_mps == 0 && output.motion.wz_rps == 0);
    }

    void completed() const {
        assert(output.state == PushState::RAISE_RELEASE && output.reason == "assumed_push_in_done");
        assert(task.assume_pushed_ && !task.fault_ && task.phase_us_ == input.now_us);
        closed();
        stationary();
    }

    void report(const char* test, std::string events = {}) const {
        std::replace(events.begin(), events.end(), '\n', '|');
        std::cout << "case=" << test << " state=" << PushTask::name(output.state)
                  << " reason=" << output.reason << " fault=" << task.fault_
                  << " frame=" << int(output.motion.gripper_offset)
                  << " transaction=" << output.motion.frame_transaction
                  << " pitch=" << output.motion.camera_pitch_cdeg << " vx=" << output.motion.vx_mps
                  << " wz=" << output.motion.wz_rps << " phase_us=" << task.phase_us_
                  << " travel=" << task.travel_ << " cargo_ts=" << task.cargo_last_ts_
                  << " cargo_ref=" << task.cargo_ref_us_ << " repush=" << task.repush_n_
                  << " pushed=" << task.assume_pushed_ << " batch=" << output.batch_size
                  << " events=" << events << '\n';
    }
};

void commonGates() {
    {
        Rig rig;
        rig.openFrame();
        rig.cargo(.02f);
        rig.input.camera_pitch_stable = false;
        rig.tick();
        assert(rig.output.state == PushState::ENTER && rig.output.reason == "camera_pitch_wait");
        assert(rig.output.motion.gripper_offset == 0 &&
               rig.output.motion.frame_transaction == kTransaction);
        assert(rig.task.cargo_last_ts_ == 0);
        rig.stationary();
        rig.report("pitch_wait_precedes_lower_and_tracking");
        rig.elapsed(rig.task.t_.pitch_timeout_us + 1);
        assert(rig.output.state == PushState::SAFE_STOP &&
               rig.output.reason == "camera_pitch_timeout");
        assert(rig.task.fault_ && rig.task.phase_us_ == kStart &&
               rig.output.motion.gripper_offset == 0);
        rig.report("pitch_timeout_preserves_open_frame_and_phase");
    }
    {
        Rig rig;
        rig.openFrame();
        rig.cargo(.02f);
        rig.input.gripper_feedback_open = 1;
        rig.tick();
        assert(rig.output.state == PushState::ENTER && rig.task.cargo_last_ts_ == 0);
        assert(rig.output.motion.frame_transaction == kTransaction + 1);
        rig.closed();
        rig.stationary();
        rig.report("frame_wait_precedes_tracking");
        rig.elapsed(kTick + rig.task.t_.gripper_timeout_us + 1);
        assert(rig.output.state == PushState::SAFE_STOP &&
               rig.output.reason == "frame_lower_timeout");
        assert(rig.task.fault_ && rig.task.phase_us_ == kStart && rig.task.cargo_last_ts_ == 0);
        rig.report("lower_timeout_preserves_phase");
    }
    {
        Rig rig(tuning(false));
        rig.openFrame();
        rig.input.drop_plan_valid = false;
        rig.tick();
        assert(rig.output.state == PushState::ENTER &&
               rig.output.reason == "drop_revalidation_missing");
        assert(rig.output.motion.gripper_offset == 0 &&
               rig.output.motion.frame_transaction == kTransaction);
        rig.report("strict_drop_permission_precedes_lower");
        rig.elapsed(rig.task.t_.carry_budget_us + 1);
        assert(rig.output.state == PushState::SAFE_STOP &&
               rig.output.reason == "drop_revalidation_timeout");
        assert(rig.task.phase_us_ == kStart);
        rig.report("strict_drop_permission_timeout");
    }
    {
        Rig rig;
        rig.input.drop_plan_valid = false;
        rig.tick();
        assert(rig.output.state == PushState::ENTER &&
               rig.output.reason == "assumed_push_in_blind");
        assert(rig.output.motion.vx_mps > 0 && !rig.task.fault_);
        rig.report("field_carry_uses_experimental_push_path");
    }
    {
        Rig rig;
        rig.cargo(.02f);
        rig.input.path_safe = false;
        rig.tick();
        assert(rig.output.state == PushState::ENTER && rig.output.reason == "forward_path_blocked");
        assert(rig.task.cargo_last_ts_ == rig.input.now_us);
        rig.stationary();
        rig.report("shared_forward_gate_still_tracks_cargo");
    }
    {
        Rig rig;
        rig.cargo(.02f, .08f);
        rig.input.directional_clearance_valid = true;
        rig.input.arc_safe = false;
        rig.tick();
        assert(rig.output.reason == "turn_sweep_blocked_straight");
        assert(rig.output.motion.vx_mps > 0 && rig.output.motion.wz_rps == 0);
        rig.report("shared_arc_gate_falls_back_to_verified_straight");
    }
    {
        Rig rig;
        rig.input.opponent_zone_clear = false;
        rig.tick();
        assert(rig.output.state == PushState::ENTER &&
               rig.output.reason == "opponent_clearance_missing");
        rig.stationary();
        rig.report("shared_opponent_gate_blocks_blind_push");
    }
    {
        Rig rig(tuning(false));
        rig.elapsed(rig.task.t_.enter_budget_us + 1);
        assert(rig.output.state == PushState::ABORT_DROP && rig.output.reason == "enter_timeout");
        assert(!rig.task.fault_ && !rig.task.assume_pushed_);
        rig.closed();
        rig.report("strict_enter_keeps_its_original_timeout");
    }
}

void completionPriority() {
    {
        auto parameters = tuning();
        parameters.assume_enter_confirm = true;
        Rig rig(parameters);
        rig.hold_y = .12f;
        rig.cargo(.10f);
        rig.input.imu_blocked_us = parameters.assume_imu_block_us;
        rig.task.dropzone_evt_ = "prefix\n";
        rig.tick();
        rig.completed();
        assert(rig.task.repush_n_ == 0);
        const auto events = rig.task.takeDropZoneEvent();
        assert(events.find("prefix\n[CARGO_ZONE] event=push_inside") == 0);
        assert(events.find("event=push_inside") < events.find("event=push_done"));
        assert(events.find("stuck") == std::string::npos &&
               events.find("unconfirmed") == std::string::npos);
        rig.report("inside_precedes_blocked_stuck_and_confirmation", events);
        rig.input.gripper_feedback_open = 1;
        rig.tick();
        assert(rig.output.motion.gripper_offset == 0 &&
               rig.output.motion.frame_transaction == kTransaction + 1);
        rig.report("complete_opens_frame_on_following_tick");
    }
    for (bool clipped : {false, true}) {
        Rig rig;
        rig.hold_y = .12f;
        rig.vref = true;
        if (clipped)
            rig.cargo(.12f, 0, true);
        rig.input.imu_blocked_us = rig.task.t_.assume_imu_block_us;
        rig.tick();
        rig.completed();
        assert(rig.task.repush_n_ == 0);
        const auto events = rig.task.takeDropZoneEvent();
        assert(events.find("event=blocked_deep") < events.find("event=push_done"));
        assert(events.find(clipped ? "fix=1" : "fix=0") != std::string::npos);
        assert(events.find("stuck") == std::string::npos);
        rig.report(clipped ? "clipped_deep_blockage_completes" : "hidden_deep_blockage_completes",
                   events);
    }
    {
        auto parameters = tuning();
        parameters.assume_align_push = true;
        Rig rig(parameters);
        rig.alignedEntry();
        rig.hold_y = .12f;
        rig.vref = true;
        rig.input.imu_blocked_us = parameters.assume_imu_block_us;
        rig.tick();
        rig.completed();
        const auto events = rig.task.takeDropZoneEvent();
        assert(events.find("event=blocked_deep") != std::string::npos);
        assert(events.find("push_align") == std::string::npos);
        rig.report("blocked_deep_precedes_pending_alignment", events);
    }
    {
        Rig rig;
        rig.hold_y = .12f;
        rig.cargo(.02f);
        rig.input.imu_blocked_us = rig.task.t_.assume_imu_block_us;
        rig.task.dropzone_evt_ = "prefix\n";
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               rig.output.reason == "assumed_stuck_repush");
        assert(rig.task.repush_n_ == 1 && rig.task.cargo_ref_us_ == 0 && !rig.task.assume_pushed_);
        assert(rig.task.phase_us_ == rig.input.now_us &&
               rig.output.motion.frame_transaction == kTransaction);
        rig.closed();
        rig.stationary();
        const auto events = rig.task.takeDropZoneEvent();
        assert(events.find("prefix\n[CARGO_ZONE] event=stuck_repush") == 0);
        assert(events.find("repush=1 in=0 out=0") != std::string::npos);
        assert(events.find("blocked_deep") == std::string::npos);
        rig.report("placed_outside_cargo_repushes_before_opening", events);
        rig.input.gripper_feedback_open = 1;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT &&
               rig.output.motion.frame_transaction == kTransaction + 1);
        assert(rig.output.motion.gripper_offset == 0);
        rig.report("repush_opens_frame_on_following_tick");
    }
    {
        Rig rig;
        rig.cargo(.02f);
        rig.tick();
        const auto reference = rig.task.cargo_ref_us_;
        rig.elapsed(kTick + rig.task.t_.assume_stuck_us - 1);
        assert(rig.output.state == PushState::ENTER && rig.task.cargo_ref_us_ == reference);
        rig.elapsed(kTick + rig.task.t_.assume_stuck_us);
        assert(rig.output.state == PushState::BACK_OUT && rig.task.repush_n_ == 1);
        rig.report("cargo_static_reference_uses_inclusive_deadline", rig.task.takeDropZoneEvent());
    }
    {
        auto parameters = tuning();
        parameters.assume_enter_confirm = true;
        Rig rig(parameters);
        rig.cargo(.02f);
        rig.task.repush_n_ = parameters.assume_repush_max;
        rig.input.imu_blocked_us = parameters.assume_imu_block_us;
        rig.tick();
        rig.completed();
        const auto events = rig.task.takeDropZoneEvent();
        assert(events.find("event=stuck_giveup") < events.find("event=push_done"));
        assert(events.find("unconfirmed") == std::string::npos);
        assert(rig.task.repush_n_ == parameters.assume_repush_max);
        rig.report("exhausted_repush_skips_confirmation_extension", events);
    }
    {
        Rig rig;
        rig.cargo(.12f);
        rig.tick();
        assert(rig.output.state == PushState::ENTER);
        rig.task.takeDropZoneEvent();
        rig.task.zone_.reset();
        rig.forgetCargo();
        rig.tick();
        rig.completed();
        const auto events = rig.task.takeDropZoneEvent();
        assert(events.find("event=push_inside") != std::string::npos);
        assert(events.find("event=push_done") == std::string::npos);
        rig.report("historic_fixed_cargo_inside_without_anchor", events);
    }
}

void alignment() {
    {
        auto parameters = tuning();
        parameters.assume_align_push = true;
        Rig rig(parameters);
        rig.alignedEntry();
        rig.cargo(.02f, .15f);
        rig.input.imu_blocked_us = parameters.assume_imu_block_us;
        rig.tick();
        assert(rig.output.state == PushState::ENTER &&
               rig.output.reason == "assumed_push_align_turn");
        assert(rig.output.motion.vx_mps == 0 && rig.output.motion.wz_rps < 0);
        assert(rig.task.repush_n_ == 0 && rig.task.cargo_ref_us_ == 0);
        assert(rig.task.takeDropZoneEvent().empty());
        rig.report("alignment_turn_suppresses_stuck_and_resets_reference");
        rig.elapsed(kTick + parameters.assume_align_budget_us);
        assert(rig.output.state == PushState::ENTER &&
               rig.output.reason == "assumed_push_in_pursuit");
        assert(rig.output.motion.vx_mps > 0 && rig.task.repush_n_ == 0);
        const auto events = rig.task.takeDropZoneEvent();
        assert(events.find("event=push_align_giveup") != std::string::npos);
        assert(events.find("waited_ms=3000") != std::string::npos);
        rig.report("alignment_exact_deadline_gives_up_then_pushes", events);
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT && rig.task.repush_n_ == 1);
        rig.report("stuck_resumes_after_alignment_has_finished", rig.task.takeDropZoneEvent());
    }
    for (bool seen : {false, true}) {
        auto parameters = tuning();
        parameters.assume_align_push = true;
        Rig rig(parameters);
        rig.alignedEntry();
        rig.vref = true;
        if (seen)
            rig.cargo(.02f);
        rig.tick();
        assert(rig.output.state == PushState::ENTER && rig.output.motion.vx_mps > 0);
        const auto events = rig.task.takeDropZoneEvent();
        assert(events.find(seen ? "event=push_aligned" : "event=push_align_unseen") !=
               std::string::npos);
        assert(events.find("waited_ms=0") != std::string::npos);
        rig.report(seen ? "aligned_cargo_pushes_immediately"
                        : "unseen_alignment_pushes_immediately",
                   events);
    }
    {
        auto parameters = tuning();
        parameters.assume_align_push = true;
        Rig rig(parameters);
        rig.alignedEntry();
        rig.cargo(.02f, .15f);
        rig.input.directional_clearance_valid = true;
        rig.input.turn_safe = false;
        rig.tick();
        assert(rig.output.state == PushState::ENTER && rig.output.reason == "turn_sweep_blocked");
        assert(rig.task.repush_n_ == 0);
        rig.stationary();
        rig.report("alignment_turn_uses_shared_clearance_gate");
    }
}

void confirmation() {
    {
        auto parameters = tuning();
        parameters.assume_enter_confirm = true;
        parameters.enter_budget_us = 200000;
        parameters.assume_enter_extra_us = 300000;
        Rig rig(parameters);
        rig.cargo(.02f);
        rig.elapsed(parameters.enter_budget_us);
        assert(rig.output.state == PushState::ENTER && rig.task.takeDropZoneEvent().empty());
        rig.elapsed(parameters.enter_budget_us + 1);
        assert(rig.output.state == PushState::ENTER && rig.output.motion.vx_mps > 0);
        auto events = rig.task.takeDropZoneEvent();
        assert(events.find("event=push_unconfirmed_visible") <
               events.find("event=push_unconfirmed half="));
        rig.report("visible_confirmation_extends_after_budget", events);
        rig.elapsed(parameters.enter_budget_us + 50000);
        assert(rig.task.takeDropZoneEvent().empty());
        rig.seed(PushState::ENTER);
        rig.elapsed(parameters.enter_budget_us + 1);
        assert(rig.output.state == PushState::ENTER && rig.task.takeDropZoneEvent().empty());
        rig.report("same_state_reentry_retains_unconfirmed_history");
        rig.cargo(.08f);
        rig.elapsed(parameters.enter_budget_us + 2);
        rig.completed();
        events = rig.task.takeDropZoneEvent();
        assert(events.find("event=push_confirmed_late") < events.find("event=push_done"));
        rig.report("visible_late_confirmation_completes", events);
    }
    {
        auto parameters = tuning();
        parameters.assume_enter_confirm = true;
        parameters.enter_budget_us = 200000;
        parameters.assume_enter_extra_us = 300000;
        Rig rig(parameters);
        rig.vref = true;
        rig.elapsed(parameters.enter_budget_us + 1);
        assert(rig.output.state == PushState::ENTER && rig.output.motion.vx_mps > 0);
        auto events = rig.task.takeDropZoneEvent();
        assert(events.find("event=push_unconfirmed_hidden") <
               events.find("event=push_unconfirmed half="));
        rig.report("hidden_confirmation_extends_after_budget", events);
        rig.elapsed(parameters.enter_budget_us + parameters.assume_enter_extra_us);
        rig.completed();
        events = rig.task.takeDropZoneEvent();
        assert(events.find("event=push_unconfirmed_giveup") < events.find("event=push_done"));
        rig.report("confirmation_hard_deadline_is_exclusive", events);
    }
    {
        auto parameters = tuning();
        parameters.assume_enter_confirm = true;
        parameters.assume_enter_hold_y_m = 1;
        Rig rig(parameters);
        rig.hold_y = parameters.assume_enter_max_hold_y_m + .01f;
        rig.vref = true;
        rig.tick();
        rig.completed();
        const auto events = rig.task.takeDropZoneEvent();
        assert(events.find("event=push_unconfirmed_giveup") < events.find("event=push_done"));
        rig.report("confirmation_depth_cap_precedes_extension", events);
    }
    {
        auto parameters = tuning();
        parameters.assume_enter_confirm = true;
        Rig rig(parameters);
        rig.cargo(.12f, 0, true);
        rig.elapsed(parameters.enter_budget_us + 1);
        assert(rig.output.state == PushState::ENTER && !rig.task.assume_pushed_);
        const auto events = rig.task.takeDropZoneEvent();
        assert(events.find("event=push_unconfirmed_visible") != std::string::npos);
        rig.report("clipped_visible_cargo_cannot_confirm", events);
    }
}

void steeringAndProgress() {
    {
        Rig rig;
        rig.tick();
        assert(rig.output.state == PushState::ENTER &&
               rig.output.reason == "assumed_push_in_blind");
        rig.task.travel_ = rig.task.t_.assume_push_m;
        rig.task.last_vx_ = 0;
        rig.tick();
        rig.completed();
        assert(rig.task.takeDropZoneEvent().empty());
        rig.report("blind_push_uses_unscaled_commanded_distance");
    }
    {
        Rig rig;
        rig.elapsed(rig.task.t_.enter_budget_us);
        assert(rig.output.state == PushState::ENTER && rig.output.motion.vx_mps > 0);
        rig.elapsed(rig.task.t_.enter_budget_us + 1);
        rig.completed();
        rig.report("blind_push_timeout_uses_strict_greater_than");
    }
    {
        Rig rig;
        rig.vref = true;
        rig.tick();
        assert(rig.output.reason == "assumed_push_in_visual" && rig.output.motion.wz_rps == 0);
        rig.vref = false;
        rig.elapsed(kTick + 400001);
        assert(rig.output.state == PushState::ENTER &&
               rig.output.reason == "assumed_push_in_dead_reckoning");
        assert(rig.output.motion.vx_mps > 0);
        rig.report("anchored_push_labels_visual_reference_age");
    }
    {
        auto parameters = tuning();
        parameters.min_turn_wz = .01f;
        parameters.max_wz = 2;
        parameters.heading_gain = .8f;
        parameters.lateral_gain = .4f;
        parameters.heading_deadband_rad = .001f;
        Rig rig(parameters);
        rig.cargo(.02f, .07f);
        rig.task.repush_n_ = 2;
        rig.tick();
        assert(rig.output.state == PushState::ENTER &&
               rig.output.reason == "assumed_push_in_visual");
        assert(close(rig.output.motion.vx_mps, .06f) && rig.output.motion.wz_rps > 0);
        rig.report("visible_line_push_uses_repush_speed_and_slant");
    }
    {
        auto parameters = tuning();
        parameters.assume_align_push = true;
        parameters.assume_slant_max_rad = .1f;
        parameters.min_turn_wz = .01f;
        parameters.max_wz = 2;
        parameters.heading_gain = .8f;
        parameters.lateral_gain = .4f;
        parameters.heading_deadband_rad = .001f;
        Rig rig(parameters);
        rig.cargo(.02f, .07f);
        rig.tick();
        assert(rig.output.reason == "assumed_push_in_pursuit" && rig.output.motion.wz_rps < 0);
        const auto pursuit_wz = rig.output.motion.wz_rps;
        rig.report("pursuit_uses_cargo_bearing_and_capped_slant");
        rig.forgetCargo();
        rig.task.zone_.advance(0, .1f);
        rig.elapsed(kTick + 400001);
        assert(rig.output.state == PushState::ENTER &&
               rig.output.reason == "assumed_push_in_hold_heading");
        assert(close(rig.output.motion.wz_rps, pursuit_wz - parameters.heading_gain * .1f));
        rig.report("lost_cargo_holds_last_pursuit_heading");
        rig.seed(PushState::ENTER);
        rig.tick();
        assert(rig.output.reason == "assumed_push_in_hold_heading");
        rig.report("same_state_reentry_retains_pursuit_heading");
    }
    {
        Rig rig;
        rig.hold_y = rig.task.t_.assume_push_depth_m + .01f;
        rig.vref = true;
        rig.tick();
        rig.completed();
        rig.report("hidden_cargo_uses_base_depth", rig.task.takeDropZoneEvent());
    }
    {
        Rig rig;
        rig.hold_y = rig.task.t_.assume_push_depth_m + .01f;
        rig.cargo(.08f);
        rig.tick();
        assert(rig.output.state == PushState::ENTER && !rig.task.assume_pushed_);
        assert(rig.output.motion.vx_mps > 0);
        rig.report("visible_cargo_requires_overshoot_depth");
    }
}

void probesAndLifecycle() {
    {
        auto parameters = tuning();
        parameters.assume_probe = true;
        parameters.assume_probe_first_us = 100000;
        parameters.assume_probe_len_us = 150000;
        parameters.assume_probe_gap_us = 200000;
        Rig rig(parameters);
        rig.vref = true;
        rig.input.heading_valid = true;
        rig.elapsed(99999);
        assert(rig.output.motion.wz_rps == 0);
        rig.elapsed(100000);
        assert(rig.output.reason == "assumed_push_imu_probe" && rig.output.motion.wz_rps < 0);
        rig.report("probe_starts_at_first_boundary");
        rig.elapsed(249999);
        assert(rig.output.motion.wz_rps < 0);
        rig.elapsed(250000);
        assert(rig.output.motion.wz_rps == 0 && rig.output.reason == "assumed_push_in_visual");
        rig.report("probe_stops_at_pulse_end_boundary");
        rig.elapsed(449999);
        assert(rig.output.motion.wz_rps == 0);
        rig.elapsed(450000);
        assert(rig.output.reason == "assumed_push_imu_probe" && rig.output.motion.wz_rps > 0);
        rig.report("probe_alternates_at_next_start_boundary");
        rig.vref = false;
        rig.task.zone_.advance(0, .1f);
        rig.elapsed(500000);
        assert(rig.output.reason != "assumed_push_imu_probe" && rig.output.motion.wz_rps < 0);
        rig.task.zone_.reset();
        rig.vref = true;
        rig.elapsed(550000);
        assert(rig.output.motion.wz_rps == 0 && rig.output.reason == "assumed_push_in_visual");
        rig.report("steering_cancels_active_probe_without_rescheduling");
        rig.input.reset = true;
        rig.tick();
        rig.input.reset = false;
        rig.task.trip_ = ordinary();
        rig.task.drop_locked_ = true;
        rig.task.drop_centre_zone_ = rig.input.drop_centre_zone;
        rig.seed(PushState::ENTER);
        rig.elapsed(parameters.assume_probe_first_us);
        assert(rig.output.reason == "assumed_push_imu_probe" && rig.output.motion.wz_rps < 0);
        rig.report("reset_restarts_probe_with_original_sign");
    }
    {
        auto parameters = tuning();
        parameters.assume_probe = true;
        parameters.assume_probe_first_us = 100000;
        parameters.assume_probe_len_us = 150000;
        parameters.assume_probe_gap_us = 200000;
        Rig rig(parameters);
        rig.vref = true;
        rig.elapsed(100000);
        assert(rig.output.motion.wz_rps < 0);
        rig.seed(PushState::ENTER);
        rig.elapsed(150000);
        assert(rig.output.motion.wz_rps == 0 && rig.output.reason == "assumed_push_in_visual");
        rig.report("same_state_reentry_retains_probe_schedule");
    }
    {
        Rig rig;
        rig.cargo(.02f);
        rig.tick();
        const auto cargo_timestamp = rig.task.cargo_last_ts_;
        rig.input.run = false;
        rig.tick();
        assert(rig.output.state == PushState::WAIT_START &&
               rig.task.stopped_from_ == PushState::ENTER);
        assert(rig.task.cargo_last_ts_ == cargo_timestamp && rig.output.batch_size == 1);
        rig.closed();
        rig.stationary();
        rig.input.run = true;
        rig.tick();
        assert(rig.output.state == PushState::BACK_OUT && rig.output.reason == "resume");
        assert(rig.task.phase_us_ == rig.input.now_us && rig.output.batch_size == 1);
        rig.closed();
        rig.stationary();
        rig.report("pause_resumes_to_back_out_with_load_and_closed_frame");
    }
    {
        auto parameters = tuning();
        parameters.assume_align_push = true;
        Rig rig(parameters);
        rig.cargo(.02f, .07f);
        rig.tick();
        assert(rig.output.reason == "assumed_push_in_pursuit");
        rig.input.reset = true;
        rig.forgetCargo();
        rig.tick();
        assert(rig.output.state == PushState::WAIT_START && rig.output.batch_size == 0 &&
               !rig.task.fault_);
        rig.input.reset = false;
        rig.task.trip_ = ordinary();
        rig.task.drop_locked_ = true;
        rig.task.drop_centre_zone_ = rig.input.drop_centre_zone;
        rig.seed(PushState::ENTER);
        rig.vref = true;
        rig.tick();
        assert(rig.output.reason == "assumed_push_in_visual" && rig.output.motion.wz_rps == 0);
        rig.report("reset_clears_last_pursuit_heading");
    }
}
} // namespace

int main() {
    commonGates();
    completionPriority();
    alignment();
    confirmation();
    steeringAndProgress();
    probesAndLifecycle();
    std::cout << "push-in task integration tests passed\n";
}
