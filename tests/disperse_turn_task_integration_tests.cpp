#include "rescue/transport_rules.hpp"
#include "rescue/types.hpp"
#include "rescue/zone_estimate.hpp"
#include "rescue/zone_dead_reckoning.hpp"
#include "rescue/planner.hpp"
#include "rescue/task/capture_verification_stage.hpp"
#include "rescue/task/delivery_verification_stage.hpp"
#include "rescue/task/release_retreat_stage.hpp"
#include "rescue/task/push_in_stage.hpp"
#include "rescue/task/sweep_ledger.hpp"
#if __has_include("rescue/task/disperse_turn_stage.hpp")
#include "rescue/task/disperse_turn_stage.hpp"
#endif
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#define private public
#if __has_include("rescue/task/disperse_rush_stage.hpp")
#define RESCUE_HAS_RUSH_STAGE 1
#endif
#include "rescue/push_task.hpp"
#undef private

namespace {
using namespace rescue;
constexpr uint64_t kStart = 1000000;
constexpr uint64_t kTick = 50000;

TaskTuning tuning() {
    TaskTuning value;
    value.require_multi_view = false;
    value.disperse_single_turn = value.sweep_observation_guard = true;
    value.far_pitch_cdeg = 500;
    value.intermediate_pitch_cdeg = 2000;
    value.track_pitch_cdeg = value.near_pitch_cdeg = 4000;
    value.sweep_view_handoff = value.post_clear_mid_view = false;
    value.blind_scatter_return = value.spin_unstick = value.visual_chase = false;
    value.scan_sighting_hold = false;
    value.disperse_turn_rad = 1.8f;
    value.sweep_time_budget_us = value.sweep_op_budget_us = 30000000;
    value.sweep_match_budget_us = 100000000;
    value.sweep_total_max = 10;
    value.sweep_escalate_max = 0;
    return value;
}

struct Rig {
    PushTask task;
    PushObservation input;
    PushOutput output;
    bool fresh = true, auto_camera = true;
    uint64_t trace = 1469598103934665603ULL;

    explicit Rig(TaskTuning parameters = tuning(), PushState state = PushState::SCATTER) : task(parameters) {
        input.now_us = kStart;
        input.run = input.safety_ok = input.path_safe = input.retreat_safe = true;
        input.opponent_zone_clear = input.heading_valid = input.camera_pitch_stable = true;
        input.gripper_done = true;
        input.gripper_feedback_open = 0;
        arm(state);
    }

    void arm(PushState state) {
        task.enter(state, input.now_us, "seed_turn_rush");
        task.last_us_ = input.now_us;
        task.last_vx_ = task.last_wz_ = 0;
        if (state == PushState::APPROACH) task.approach_pitch_ = task.t_.track_pitch_cdeg;
        if (state == PushState::PILE) {
            task.pile_phase_ = 9;
            task.pile_phase_us_ = input.now_us;
            task.pile_travel0_ = 0;
        }
        task.sweep_phase_ = 4;
        task.sweep_start_us_ = task.sweep_t0_us_ = input.now_us;
        task.sweep_travel0_ = 0;
        task.disperse_rush_m_ = .1f;
        task.travel_ = .11f * task.motionRatio();
        task.disperse_sign_ = 1;
        task.scatter_commit_ = true;
        task.sweep_ledger_.begin(task.t_, input.now_us);
        task.pitch_cmd_ = task.desiredPitch();
        task.pitch_wait_us_ = input.now_us;
        input.camera_pitch_cdeg = task.pitch_cmd_;
    }

    void at(uint64_t now) {
        input.now_us = now;
        task.last_us_ = now - kTick;
        task.last_vx_ = task.last_wz_ = 0;
        if (fresh) input.clutter_ts_us = now;
        if (auto_camera) input.camera_pitch_cdeg = task.desiredPitch();
        output = task.update(input);
        std::ostringstream sample;
        sample << std::setprecision(std::numeric_limits<float>::max_digits10)
               << int(output.state) << ':' << output.reason << ':' << output.motion.vx_mps << ':'
               << output.motion.wz_rps << ':' << output.motion.camera_pitch_cdeg << ':'
               << int(output.motion.gripper_offset) << ':' << output.motion.frame_transaction << ':'
               << task.sweep_phase_ << ':' << task.sweep_start_us_ << ':' << task.sweep_yaw0_ << ':'
               << task.sweep_travel0_ << ':' << task.travel_ << ':' << task.phase_us_ << ':'
               << task.sweepDone() << ':' << task.sweepSpentUs(now) << ':' << task.sweepChainOpen();
        for (const unsigned char c : sample.str()) trace = (trace ^ c) * 1099511628211ULL;
    }

    void tick() { at(input.now_us + kTick); }
    void stationary() const { assert(output.motion.vx_mps == 0 && output.motion.wz_rps == 0); }

    void start() {
        tick();
        assert(task.sweep_phase_ == 5);
        assert(output.reason == "disperse_look" || (output.state == PushState::PILE && output.reason == "disperse_turn"));
        if (output.reason == "disperse_look") stationary();
        assert(output.motion.camera_pitch_cdeg == task.t_.intermediate_pitch_cdeg);
    }

    void turn() {
        start();
        if (output.reason == "disperse_look") tick();
        assert(output.reason == "disperse_turn" && output.motion.vx_mps == 0 && output.motion.wz_rps != 0);
    }

    void thinToLow() {
        turn();
        input.heading_rad = task.t_.disperse_turn_min_rad + .01f;
        tick();
        assert(output.reason == "disperse_frame_look");
        stationary();
        assert(output.motion.camera_pitch_cdeg == task.t_.track_pitch_cdeg);
    }

    void finishTurn(float sign = 1) {
        input.heading_rad = task.sweep_yaw0_ + sign * task.t_.disperse_turn_rad;
        tick();
    }

    void report(const char* name) const {
        std::cout << std::setprecision(9) << "case=" << name << " trace=" << trace
                  << " state=" << PushTask::name(output.state) << " reason=" << output.reason
                  << " phase=" << task.sweep_phase_ << " phase_us=" << task.phase_us_
                  << " start=" << task.sweep_start_us_ << " yaw0=" << task.sweep_yaw0_
                  << " travel0=" << task.sweep_travel0_ << " travel=" << task.travel_
                  << " vx=" << output.motion.vx_mps << " wz=" << output.motion.wz_rps
                  << " pitch=" << output.motion.camera_pitch_cdeg
                  << " frame=" << int(output.motion.gripper_offset)
                  << " done=" << task.sweepDone() << " spent=" << task.sweepSpentUs(input.now_us)
                  << " open=" << task.sweepChainOpen() << " pile=" << task.pile_phase_
                  << " pile_mid=" << task.pile_chain_yaw_mid_ << '\n';
    }
};

void directionAndView() {
    for (bool left : {false, true}) {
        Rig rig;
        const float x = left ? -.2f : .2f;
        rig.input.clutter_body = {{x, .5f}, {x, .6f}, {x, .7f}};
        rig.turn();
        assert(rig.output.motion.wz_rps * (left ? -1.f : 1.f) > 0);
        rig.report(left ? "left_crowded_turns_right" : "right_crowded_turns_left");
        rig.finishTurn(left ? -1.f : 1.f);
        assert(rig.output.state == PushState::SCAN && rig.output.reason == "blind_scatter_done");
        assert(rig.task.sweepDone() == 1 && !rig.task.sweepChainOpen());
        rig.report(left ? "right_turn_counts_once" : "left_turn_counts_once");
    }
    {
        Rig rig;
        rig.task.disperse_sign_ = -1;
        rig.input.clutter_body = {{-.2f, .5f}, {.2f, .5f}};
        rig.turn();
        assert(rig.output.motion.wz_rps < 0);
        rig.report("equal_view_keeps_planned_side");
    }
    {
        Rig rig;
        rig.task.disperse_sign_ = -1;
        rig.start();
        rig.fresh = false;
        rig.input.clutter_ts_us = 0;
        rig.tick();
        assert(rig.output.reason == "disperse_look");
        const auto ready = rig.input.now_us;
        rig.at(ready + rig.task.t_.disperse_turn_look_us - 1);
        assert(rig.output.reason == "disperse_look");
        rig.report("stale_initial_view_waits_until_deadline");
        rig.at(ready + rig.task.t_.disperse_turn_look_us);
        assert(rig.output.reason == "disperse_turn" && rig.output.motion.wz_rps < 0);
        rig.report("missing_initial_view_uses_planned_side_at_deadline");
    }
    {
        Rig rig;
        rig.turn();
        rig.input.heading_rad = rig.task.t_.disperse_turn_min_rad + .01f;
        rig.fresh = false;
        rig.tick();
        assert(rig.output.state == PushState::SCAN && rig.output.reason == "blind_scatter_done");
        rig.report("thin_view_reused_fresh_timestamp_keeps_legacy_counting");
    }
    {
        Rig rig;
        rig.start();
        rig.input.camera_pitch_stable = false;
        rig.tick();
        assert(rig.output.reason == "camera_pitch_wait");
        rig.stationary();
        rig.report("camera_motion_gates_initial_look");
        rig.input.camera_pitch_stable = true;
        rig.tick();
        assert(rig.output.reason == "disperse_turn");
        rig.report("stable_camera_releases_initial_look");
    }
}

void turnBoundsAndMotion() {
    for (bool small : {false, true}) {
        Rig rig;
        rig.task.t_.disperse_single_turn = small;
        rig.task.disperse_small_ = small;
        rig.task.t_.disperse_swing_vx = 0;
        rig.tick();
        assert(rig.output.reason == "disperse_swing" && rig.output.motion.wz_rps != 0);
        assert(rig.output.motion.camera_pitch_cdeg == rig.task.t_.far_pitch_cdeg);
        rig.report(small ? "small_neighbour_clear_keeps_alternating_swing" : "single_turn_disabled_keeps_alternating_swing");
    }
    {
        auto parameters = tuning();
        parameters.intermediate_pitch_cdeg = kCameraPitchInvalid;
        Rig rig(parameters);
        rig.tick();
        assert(rig.output.reason == "disperse_turn" && rig.output.motion.wz_rps != 0);
        assert(rig.output.motion.camera_pitch_cdeg == parameters.far_pitch_cdeg);
        rig.finishTurn();
        assert(rig.output.reason == "blind_scatter_done");
        rig.report("single_turn_without_mid_pitch_uses_existing_camera");
    }
    {
        Rig rig;
        rig.input.clutter_body = {{-.2f, .5f}, {.2f, .5f}, {0, .6f}};
        rig.turn();
        const auto start = rig.task.sweep_start_us_;
        rig.at(start + rig.task.t_.disperse_swing_budget_us);
        assert(rig.output.reason == "disperse_turn");
        rig.report("turn_budget_equality_still_spins");
        rig.at(start + rig.task.t_.disperse_swing_budget_us + 1);
        assert(rig.output.state == PushState::SCAN && rig.output.reason == "blind_scatter_done");
        rig.report("turn_budget_strictly_exceeded_finishes");
    }
    {
        Rig rig;
        rig.input.clutter_body = {{-.2f, .5f}, {.2f, .5f}, {0, .6f}};
#ifdef RESCUE_HAS_RUSH_STAGE
        rig.task.disperse_rush_.active_ = true;
        rig.task.disperse_rush_.cap_positive_ = .45f;
#else
        rig.task.swing_plan_active_ = true;
        rig.task.swing_cap_pos_ = .45f;
#endif
        rig.turn();
        rig.input.heading_rad = .45f;
        rig.tick();
        assert(rig.output.reason == "blind_scatter_done");
        rig.report("parent_swing_cap_limits_single_turn");
    }
    {
        Rig rig;
        rig.turn();
        rig.input.directional_clearance_valid = true;
        rig.input.turn_safe = false;
        rig.tick();
        assert(rig.output.reason == "turn_sweep_blocked");
        rig.stationary();
        rig.report("turn_clearance_veto_preserved");
        rig.input.turn_safe = true;
        rig.tick();
        assert(rig.output.reason == "disperse_turn" && rig.output.motion.wz_rps != 0);
        rig.report("turn_clearance_restored_continues");
    }
    for (PushState state : {PushState::SCATTER, PushState::APPROACH}) {
        Rig rig(tuning(), state);
        if (state == PushState::SCATTER) rig.task.burst_swing_ = true;
        else rig.task.t_.approach_dense_burst_swing = true;
        rig.turn();
        assert(rig.output.motion.wz_rps == rig.task.t_.first_burst_swing_wz);
        rig.report(state == PushState::SCATTER ? "scatter_burst_turn_speed" : "approach_burst_turn_speed");
    }
}

void frameZone() {
    auto parameters = tuning();
    parameters.disperse_turn_frame_check = true;
    {
        Rig rig(parameters);
        rig.thinToLow();
        rig.tick();
        assert(rig.output.reason == "disperse_frame_confirm");
        rig.stationary();
        rig.report("low_view_first_empty_frame_waits_for_confirmation");
        rig.fresh = false;
        rig.tick();
        assert(rig.output.reason == "disperse_frame_confirm");
        rig.report("low_view_duplicate_frame_never_confirms_twice");
        rig.fresh = true;
        rig.tick();
        assert(rig.output.reason == "blind_scatter_done");
        rig.report("low_view_second_independent_empty_frame_finishes");
    }
    {
        Rig rig(parameters);
        rig.thinToLow();
        rig.input.clutter_body = {{0, .2f}, {.03f, .2f}, {-.03f, .2f}};
        rig.tick();
        assert(rig.output.reason == "disperse_turn_frame" && rig.output.motion.wz_rps > 0);
        rig.report("crowded_frame_zone_keeps_turning");
        rig.input.heading_rad += parameters.disperse_turn_frame_extra_rad + .001f;
        rig.tick();
        assert(rig.output.reason == "blind_scatter_done");
        rig.report("low_view_extra_angle_caps_crowded_turn");
    }
    {
        Rig rig(parameters);
        rig.thinToLow();
        rig.fresh = false;
        rig.input.clutter_ts_us = 0;
        rig.tick();
        const auto ready = rig.input.now_us;
        assert(rig.output.reason == "disperse_frame_look");
        rig.stationary();
        rig.at(ready + parameters.disperse_turn_look_us - 1);
        assert(rig.output.reason == "disperse_frame_look");
        rig.report("missing_low_view_waits_before_deadline");
        rig.at(ready + parameters.disperse_turn_look_us);
        assert(rig.output.reason == "blind_scatter_done");
        rig.report("missing_low_view_stops_at_deadline");
    }
    {
        Rig rig(parameters);
        rig.thinToLow();
        rig.input.camera_pitch_stable = false;
        rig.tick();
        assert(rig.output.reason == "camera_pitch_wait");
        rig.stationary();
        rig.report("low_camera_motion_gates_frame_counting");
        rig.input.camera_pitch_stable = true;
        rig.tick();
        assert(rig.output.reason == "disperse_frame_confirm");
        rig.tick();
        assert(rig.output.reason == "blind_scatter_done");
        rig.report("stable_low_camera_resumes_frame_confirmation");
    }
}

void pushAndParentLifecycle() {
    auto parameters = tuning();
    parameters.disperse_turn_push_m = .2f;
    for (PushState state : {PushState::SCATTER, PushState::APPROACH}) {
        auto chain = tuning();
        chain.pile_clear = chain.pile_chain = true;
        Rig rig(chain, state);
        rig.turn();
        rig.finishTurn();
        assert(rig.output.state == PushState::PILE && rig.task.pile_phase_ == 7);
        assert(rig.task.sweepDone() == 1 && !rig.task.sweepChainOpen());
        rig.report(state == PushState::SCATTER ? "scatter_turn_hands_completed_chain_to_pile" : "approach_turn_hands_completed_chain_to_pile");
    }
    for (PushState state : {PushState::SCATTER, PushState::APPROACH, PushState::PILE}) {
        Rig rig(parameters, state);
        rig.turn();
        rig.finishTurn();
        const auto start = rig.task.sweep_start_us_;
        const auto travel0 = rig.task.sweep_travel0_;
        rig.tick();
        assert(rig.output.reason == "disperse_turn_push" && rig.output.motion.vx_mps > 0 && rig.output.motion.wz_rps == 0);
        rig.report(state == PushState::SCATTER ? "scatter_turn_starts_straight_push" :
                   state == PushState::APPROACH ? "approach_turn_starts_straight_push" : "pile_turn_starts_straight_push");
        rig.task.travel_ = travel0 + parameters.disperse_turn_push_m * rig.task.motionRatio() + .001f;
        rig.tick();
        assert(rig.task.sweep_start_us_ == start);
        if (state == PushState::PILE) {
            assert(rig.output.state == PushState::PILE && rig.task.pile_phase_ == 10 && rig.task.sweep_phase_ == 0);
            assert(std::abs(rig.task.pile_chain_yaw_mid_ - .9f) < .0001f);
        } else assert(rig.task.sweepDone() == 1 && !rig.task.sweepChainOpen());
        rig.report(state == PushState::SCATTER ? "scatter_push_completion_closes_parent_chain" :
                   state == PushState::APPROACH ? "approach_push_completion_restores_parent_budget" : "pile_completion_keeps_parent_midpoint");
    }
    {
        Rig rig(parameters);
        rig.turn();
        rig.finishTurn();
        const auto start = rig.task.sweep_start_us_;
        rig.input.path_safe = false;
        rig.tick();
        assert(rig.output.reason == "forward_path_blocked");
        rig.stationary();
        rig.report("straight_push_respects_forward_clearance");
        rig.input.path_safe = true;
        rig.at(start + parameters.disperse_turn_push_budget_us);
        assert(rig.output.reason == "disperse_turn_push" && rig.output.motion.vx_mps > 0);
        rig.report("straight_push_budget_equality_still_drives");
        rig.at(start + parameters.disperse_turn_push_budget_us + 1);
        assert(rig.output.reason == "blind_scatter_done");
        rig.report("straight_push_budget_exceeded_finishes");
    }
    {
        auto budget = parameters;
        budget.sweep_op_budget_us = 300000;
        Rig rig(budget);
        rig.turn();
        rig.at(kStart + budget.sweep_op_budget_us + 1);
        assert(rig.output.reason == "sweep_op_timeout" && rig.task.sweepDone() == 1);
        rig.stationary();
        rig.report("parent_operation_budget_interrupts_turn");
    }
    for (PushState state : {PushState::SCATTER, PushState::APPROACH}) {
        Rig rig(tuning(), state);
        rig.turn();
        rig.fresh = false;
        rig.input.clutter_ts_us = 0;
        rig.tick();
        assert(rig.output.reason == "disperse_turn" && rig.output.state == state);
        rig.report(state == PushState::SCATTER ? "scatter_active_turn_survives_missing_clutter" : "approach_active_turn_survives_missing_clutter");
        rig.input.heading_valid = false;
        rig.tick();
        assert(rig.output.reason == (state == PushState::SCATTER ? "blind_scatter_no_heading" : "sweep_upright_no_heading"));
        rig.stationary();
        rig.report(state == PushState::SCATTER ? "scatter_parent_heading_loss_closes_chain" : "approach_parent_heading_loss_closes_chain");
    }
    {
        auto framed = tuning();
        framed.sweep_frame_open = true;
        Rig rig(framed);
        rig.tick();
        assert(rig.output.reason == "sweep_frame_open_wait" && rig.task.sweep_phase_ == 4);
        assert(rig.output.motion.gripper_offset == 0);
        rig.stationary();
        rig.report("frame_open_ack_gates_turn_initialization");
        rig.input.gripper_feedback_open = 1;
        rig.start();
        rig.tick();
        assert(rig.output.reason == "disperse_turn");
        rig.finishTurn();
        assert(rig.output.reason == "blind_scatter_done" && rig.output.motion.gripper_offset == 20);
        rig.tick();
        assert(rig.output.reason == "sweep_frame_close_wait");
        rig.stationary();
        rig.report("completed_turn_waits_for_frame_close_ack");
    }
    {
        Rig rig;
        rig.turn();
        rig.input.run = false;
        rig.tick();
        assert(rig.output.state == PushState::WAIT_START && rig.output.reason == "not_running");
        rig.stationary();
        rig.report("pause_cancels_turn_through_parent_enter");
        rig.input.run = true;
        rig.tick();
        assert(rig.output.state == PushState::SCAN);
        rig.tick();
        assert(!rig.task.sweepChainOpen());
        rig.report("resume_scan_settles_original_sweep_once");
    }
    {
        Rig rig(parameters);
        rig.turn();
        rig.finishTurn();
        rig.input.reset = true;
        rig.tick();
        assert(rig.output.reason == "reset" && rig.task.sweepDone() == 0 && !rig.task.sweepChainOpen());
        rig.report("explicit_reset_discards_pending_push_and_sweep_usage");
        rig.input.reset = false;
        rig.input.heading_rad = 0;
        rig.arm(PushState::SCATTER);
        rig.turn();
        rig.report("new_sweep_after_reset_starts_with_initial_look");
    }
}
} // namespace

int main() {
    directionAndView();
    turnBoundsAndMotion();
    frameZone();
    pushAndParentLifecycle();
    std::cout << "disperse turn task integration tests passed\n";
}
