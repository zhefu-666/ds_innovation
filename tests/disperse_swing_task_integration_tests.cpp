#include "rescue/task/transport_rules.hpp"
#include "rescue/core/types.hpp"
#include "rescue/geometry/zone_estimate.hpp"
#include "rescue/geometry/zone_dead_reckoning.hpp"
#include "rescue/geometry/planner.hpp"
#include "rescue/task/capture_verification_stage.hpp"
#include "rescue/task/delivery_verification_stage.hpp"
#include "rescue/task/release_retreat_stage.hpp"
#include "rescue/task/push_in_stage.hpp"
#include "rescue/task/sweep_ledger.hpp"
#include "rescue/task/disperse_turn_stage.hpp"
#if __has_include("rescue/task/disperse_swing_stage.hpp")
#include "rescue/task/disperse_swing_stage.hpp"
#define RESCUE_HAS_SWING_STAGE 1
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
#include "rescue/task/push_task.hpp"
#undef private

namespace {
using namespace rescue;
constexpr uint64_t kStart = 1000000;
constexpr uint64_t kTick = 50000;

void configure(PushTask& task, int passes, int swings) {
#ifdef RESCUE_HAS_SWING_STAGE
    task.disperse_swing_.configure(passes, swings);
#else
    task.disperse_pass_ = 0;
    task.disperse_passes_ = passes;
    task.disperse_swings_ = swings;
#endif
}
int swingCount(const PushTask& task) {
#ifdef RESCUE_HAS_SWING_STAGE
    return task.disperse_swing_.swingCount();
#else
    return task.disperse_swing_n_;
#endif
}
int pass(const PushTask& task) {
#ifdef RESCUE_HAS_SWING_STAGE
    return task.disperse_swing_.pass();
#else
    return task.disperse_pass_;
#endif
}
int passLimit(const PushTask& task) {
#ifdef RESCUE_HAS_SWING_STAGE
    return task.disperse_swing_.passLimit();
#else
    return task.disperse_passes_;
#endif
}
int swingLimit(const PushTask& task) {
#ifdef RESCUE_HAS_SWING_STAGE
    return task.disperse_swing_.swingLimit();
#else
    return task.disperse_swings_;
#endif
}

TaskTuning tuning() {
    TaskTuning value;
    value.require_multi_view = false;
    value.disperse_single_turn = false;
    value.sweep_observation_guard = true;
    value.far_pitch_cdeg = 500;
    value.intermediate_pitch_cdeg = 2000;
    value.track_pitch_cdeg = value.near_pitch_cdeg = 4000;
    value.sweep_view_handoff = value.post_clear_mid_view = false;
    value.blind_scatter_return = value.spin_unstick = value.visual_chase = false;
    value.scan_sighting_hold = false;
    value.sweep_time_budget_us = value.sweep_op_budget_us = 30000000;
    value.sweep_match_budget_us = 100000000;
    value.sweep_total_max = 10;
    value.sweep_escalate_max = 0;
    value.assume_motion_ratio = 1;
    return value;
}

struct Rig {
    PushTask task;
    PushObservation input;
    PushOutput output;
    bool fresh = true, auto_camera = true;
    uint64_t trace = 1469598103934665603ULL;

    explicit Rig(TaskTuning parameters = tuning(), PushState state = PushState::SCATTER)
        : task(parameters) {
        input.now_us = kStart;
        input.run = input.safety_ok = input.path_safe = input.retreat_safe = true;
        input.opponent_zone_clear = input.heading_valid = input.camera_pitch_stable = true;
        input.gripper_done = true;
        configure(task, 1, 2);
        arm(state);
    }

    void arm(PushState state) {
        task.enter(state, input.now_us, "seed_swing_rush");
        task.last_us_ = input.now_us;
        task.last_vx_ = task.last_wz_ = 0;
        if (state == PushState::APPROACH)
            task.approach_pitch_ = task.t_.track_pitch_cdeg;
        if (state == PushState::PILE) {
            task.pile_phase_ = 9;
            task.pile_phase_us_ = input.now_us;
            task.sweep_.pile_travel0_ = 0;
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
        if (fresh)
            input.clutter_ts_us = now;
        if (auto_camera)
            input.camera_pitch_cdeg = task.desiredPitch();
        output = task.update(input);
        std::ostringstream sample;
        sample << std::setprecision(std::numeric_limits<float>::max_digits10) << int(output.state)
               << ':' << output.reason << ':' << output.motion.vx_mps << ':' << output.motion.wz_rps
               << ':' << output.motion.camera_pitch_cdeg << ':' << int(output.motion.gripper_offset)
               << ':' << output.motion.frame_transaction << ':' << task.sweep_phase_ << ':'
               << task.sweep_start_us_ << ':' << task.sweep_yaw0_ << ':' << task.sweep_travel0_
               << ':' << task.travel_ << ':' << task.phase_us_ << ':' << swingCount(task) << ':'
               << pass(task) << ':' << passLimit(task) << ':' << swingLimit(task) << ':'
               << task.disperse_sign_ << ':' << task.disperse_rush_m_ << ':' << task.disperse_small_
               << ':' << task.sweepDone() << ':' << task.sweepSpentUs(now) << ':'
               << task.sweepChainOpen() << ':' << task.pile_phase_ << ':'
               << task.sweep_.pile_chain_yaw_mid_ << ':' << task.sweep_.pile_chain_in_m_;
        for (const unsigned char c : sample.str())
            trace = (trace ^ c) * 1099511628211ULL;
    }

    void tick() {
        at(input.now_us + kTick);
    }
    void stationary() const {
        assert(output.motion.vx_mps == 0 && output.motion.wz_rps == 0);
    }
    void start() {
        tick();
        assert(task.sweep_phase_ == 5 && swingCount(task) == 0);
        assert(output.reason == "disperse_swing" || output.reason == "first_burst_swing");
    }
    void reachSwing() {
        const float side = (swingCount(task) % 2 == 0 ? 1.f : -1.f) * task.disperse_sign_;
        float angle =
            task.disperse_small_ ? task.t_.mid_neighbour_clear_turn_rad : task.t_.disperse_turn_rad;
#ifdef RESCUE_HAS_RUSH_STAGE
        if (task.disperse_rush_.active_)
            angle = std::min(angle,
                             side > 0 ? task.disperse_rush_.cap_positive_
                                      : task.disperse_rush_.cap_negative_);
#else
        if (task.swing_plan_active_)
            angle = std::min(angle, side > 0 ? task.swing_cap_pos_ : task.swing_cap_neg_);
#endif
        input.heading_rad = wrapAngle(task.sweep_yaw0_ + side * angle);
        const int previous = swingCount(task);
        tick();
        assert(swingCount(task) == previous + 1);
        stationary();
    }
    void finishPass() {
        while (swingCount(task) < swingLimit(task))
            reachSwing();
        input.heading_rad = task.sweep_yaw0_;
        tick();
        stationary();
    }
    void report(const char* name) const {
        std::cout << std::setprecision(9) << "case=" << name << " trace=" << trace
                  << " state=" << PushTask::name(output.state) << " reason=" << output.reason
                  << " phase=" << task.sweep_phase_ << " phase_us=" << task.phase_us_
                  << " start=" << task.sweep_start_us_ << " yaw0=" << task.sweep_yaw0_
                  << " travel0=" << task.sweep_travel0_ << " travel=" << task.travel_
                  << " swing=" << swingCount(task) << '/' << swingLimit(task)
                  << " pass=" << pass(task) << '/' << passLimit(task)
                  << " sign=" << task.disperse_sign_ << " rush=" << task.disperse_rush_m_
                  << " vx=" << output.motion.vx_mps << " wz=" << output.motion.wz_rps
                  << " pitch=" << output.motion.camera_pitch_cdeg
                  << " frame=" << int(output.motion.gripper_offset) << " done=" << task.sweepDone()
                  << " spent=" << task.sweepSpentUs(input.now_us)
                  << " open=" << task.sweepChainOpen() << " pile=" << task.pile_phase_
                  << " pile_mid=" << task.sweep_.pile_chain_yaw_mid_
                  << " pile_in=" << task.sweep_.pile_chain_in_m_ << '\n';
    }
};

void callersAndPasses() {
    for (PushState state :
         {PushState::SCATTER, PushState::APPROACH, PushState::MID_APPROACH, PushState::PILE}) {
        Rig rig(tuning(), state);
        rig.start();
        assert(rig.output.motion.vx_mps > 0 && rig.output.motion.wz_rps == rig.task.t_.disperse_wz);
        rig.report(state == PushState::SCATTER        ? "scatter_rush_falls_through_to_swing"
                   : state == PushState::APPROACH     ? "approach_rush_falls_through_to_swing"
                   : state == PushState::MID_APPROACH ? "mid_rush_falls_through_to_swing"
                                                      : "pile_rush_falls_through_to_swing");
        rig.reachSwing();
        rig.tick();
        assert(rig.output.motion.wz_rps == -rig.task.t_.disperse_wz);
        rig.reachSwing();
        rig.tick();
        assert(rig.output.reason == "disperse_centre" && rig.output.motion.vx_mps == 0 &&
               rig.output.motion.wz_rps > 0);
        rig.input.heading_rad = rig.task.sweep_yaw0_;
        rig.tick();
        rig.stationary();
        assert(pass(rig.task) == 1 && swingCount(rig.task) == 2);
        if (state == PushState::SCATTER)
            assert(rig.output.state == PushState::SCAN &&
                   rig.output.reason == "blind_scatter_done");
        if (state == PushState::APPROACH)
            assert(rig.output.state == state && rig.output.reason == "disperse_done");
        if (state == PushState::MID_APPROACH)
            assert(rig.output.state == PushState::MID_REACQUIRE &&
                   rig.output.reason == "mid_neighbour_clear_done");
        if (state == PushState::PILE)
            assert(rig.task.pile_phase_ == 10 && rig.task.sweep_phase_ == 0);
        rig.report(state == PushState::SCATTER        ? "scatter_centred_completion"
                   : state == PushState::APPROACH     ? "approach_centred_completion"
                   : state == PushState::MID_APPROACH ? "mid_centred_completion"
                                                      : "pile_centred_completion");
    }
    {
        Rig rig;
        configure(rig.task, 2, 3);
        rig.start();
        rig.finishPass();
        assert(pass(rig.task) == 1 && rig.task.sweep_phase_ == 4 && rig.task.disperse_sign_ == -1);
        assert(rig.task.disperse_rush_m_ == rig.task.t_.disperse_pass_extra_m &&
               swingCount(rig.task) == 3);
        rig.report("next_pass_flips_direction_without_same_tick_rush");
        rig.tick();
        assert(rig.output.reason == "disperse_rush" && rig.output.motion.vx_mps > 0);
        rig.report("next_pass_starts_extra_rush_next_tick");
        rig.task.travel_ =
            rig.task.sweep_travel0_ + rig.task.disperse_rush_m_ * rig.task.motionRatio() + .001f;
        rig.start();
        assert(pass(rig.task) == 1 && rig.output.motion.wz_rps < 0);
        rig.report("new_rush_resets_swing_count_and_keeps_pass");
        rig.finishPass();
        assert(pass(rig.task) == 2 && rig.output.reason == "blind_scatter_done");
        rig.report("second_pass_finishes_once");
    }
    {
        Rig rig;
        configure(rig.task, 0, 0);
        rig.tick();
        assert(pass(rig.task) == 1 && swingCount(rig.task) == 0 &&
               rig.output.reason == "blind_scatter_done");
        rig.report("zero_limits_keep_legacy_immediate_centred_completion");
    }
}

void smallPlanAndSpeeds() {
    {
        Rig rig(tuning(), PushState::MID_APPROACH);
        rig.task.disperse_small_ = true;
        rig.task.t_.disperse_single_turn = true;
        rig.start();
        rig.reachSwing();
        assert(!rig.task.disperse_turn_.active() && swingCount(rig.task) == 1);
        rig.report("small_neighbour_uses_small_angle_even_with_single_turn_enabled");
        rig.task.travel_ = rig.task.sweep_travel0_ +
                           rig.task.t_.mid_neighbour_clear_creep_m * rig.task.motionRatio();
        rig.tick();
        assert(rig.output.motion.vx_mps == 0 && rig.output.motion.wz_rps < 0);
        rig.report("small_creep_distance_equality_stops_translation");
    }
    {
        Rig rig;
#ifdef RESCUE_HAS_RUSH_STAGE
        rig.task.disperse_rush_.active_ = true;
        rig.task.disperse_rush_.cap_positive_ = .25f;
        rig.task.disperse_rush_.cap_negative_ = .5f;
#else
        rig.task.swing_plan_active_ = true;
        rig.task.swing_cap_pos_ = .25f;
        rig.task.swing_cap_neg_ = .5f;
#endif
        rig.start();
        rig.reachSwing();
        rig.report("positive_swing_respects_outside_plan_cap");
        rig.reachSwing();
        rig.tick();
        assert(rig.output.reason == "disperse_centre" && rig.output.motion.vx_mps == 0);
        rig.report("negative_plan_cap_keeps_centring_unrestricted");
    }
    {
        Rig rig;
#ifdef RESCUE_HAS_RUSH_STAGE
        rig.task.disperse_rush_.active_ = true;
        rig.task.disperse_rush_.cap_positive_ = 0;
#else
        rig.task.swing_plan_active_ = true;
        rig.task.swing_cap_pos_ = 0;
#endif
        rig.tick();
        assert(swingCount(rig.task) == 1);
        rig.stationary();
        rig.report("zero_plan_cap_advances_swing_without_motion");
    }
    for (PushState state : {PushState::SCATTER, PushState::APPROACH, PushState::MID_APPROACH}) {
        Rig rig(tuning(), state);
        rig.task.burst_swing_ = true;
        rig.task.t_.approach_dense_burst_swing = true;
        rig.start();
        const float speed = state == PushState::MID_APPROACH ? rig.task.t_.disperse_wz
                                                             : rig.task.t_.first_burst_swing_wz;
        assert(rig.output.motion.wz_rps == speed);
        rig.report(state == PushState::SCATTER    ? "scatter_burst_swing_speed"
                   : state == PushState::APPROACH ? "approach_dense_swing_speed"
                                                  : "mid_keeps_ordinary_swing_speed");
        rig.reachSwing();
        rig.reachSwing();
        rig.tick();
        assert(std::abs(rig.output.motion.wz_rps) == speed && rig.output.motion.vx_mps == 0);
        rig.report(state == PushState::SCATTER    ? "scatter_burst_centring_speed"
                   : state == PushState::APPROACH ? "approach_dense_centring_speed"
                                                  : "mid_ordinary_centring_speed");
    }
    for (bool small : {false, true}) {
        Rig rig(tuning(), PushState::APPROACH);
        rig.task.t_.approach_dense_burst_swing = true;
        rig.task.sweep_non_supply_active_ = !small;
        rig.task.disperse_small_ = small;
        rig.start();
        assert(rig.output.motion.wz_rps == rig.task.t_.disperse_wz);
        rig.report(small ? "small_neighbour_disables_approach_fast_swing"
                         : "non_supply_disables_approach_fast_swing");
    }
    {
        Rig rig;
        rig.input.heading_rad = 3.1f;
        rig.start();
        rig.reachSwing();
        assert(rig.input.heading_rad < 0 && swingCount(rig.task) == 1);
        rig.report("swing_heading_wrap_preserves_goal");
    }
}

void permissionsAndWaits() {
    {
        Rig rig;
        rig.task.burst_swing_ = true;
        rig.input.directional_clearance_valid = true;
        rig.input.turn_safe = rig.input.arc_safe = false;
        rig.tick();
        assert(rig.output.reason == "turn_sweep_blocked_straight" && rig.output.motion.vx_mps > 0 &&
               rig.output.motion.wz_rps == 0);
        rig.report("arc_veto_keeps_verified_straight_creep_and_no_fast_override");
        rig.task.travel_ =
            rig.task.sweep_travel0_ + rig.task.t_.disperse_swing_creep_m * rig.task.motionRatio();
        rig.tick();
        assert(rig.output.reason == "turn_sweep_blocked");
        rig.stationary();
        rig.report("turn_veto_stops_swing_at_creep_boundary");
        rig.reachSwing();
        assert(swingCount(rig.task) == 1);
        rig.report("reached_angle_updates_history_even_when_motion_denied");
        rig.input.turn_safe = rig.input.arc_safe = true;
        rig.tick();
        assert(rig.output.motion.wz_rps == -rig.task.t_.first_burst_swing_wz);
        rig.report("restored_permission_resumes_next_burst_swing");
    }
    {
        Rig rig;
        rig.input.path_safe = false;
        rig.tick();
        assert(rig.output.reason == "forward_path_blocked");
        rig.stationary();
        assert(rig.task.sweep_phase_ == 5 && swingCount(rig.task) == 0);
        rig.report("forward_veto_preserves_rush_to_swing_transition");
    }
    {
        Rig rig;
        rig.input.camera_pitch_stable = false;
        rig.tick();
        assert(rig.output.reason == "camera_pitch_wait" && rig.task.sweep_phase_ == 4);
        rig.stationary();
        rig.report("camera_wait_gates_swing_initialization");
        rig.input.camera_pitch_stable = true;
        rig.start();
        rig.report("camera_stability_releases_swing");
    }
    {
        auto parameters = tuning();
        parameters.sweep_frame_open = true;
        Rig rig(parameters);
        rig.tick();
        assert(rig.output.reason == "sweep_frame_open_wait" && rig.task.sweep_phase_ == 4);
        rig.stationary();
        rig.report("frame_open_ack_gates_swing_initialization");
        rig.input.gripper_feedback_open = 1;
        rig.start();
        rig.finishPass();
        assert(rig.output.reason == "blind_scatter_done");
        rig.tick();
        assert(rig.output.reason == "sweep_frame_close_wait");
        rig.stationary();
        rig.report("completed_swing_waits_for_frame_close");
    }
    for (PushState state : {PushState::SCATTER, PushState::APPROACH, PushState::MID_APPROACH}) {
        Rig rig(tuning(), state);
        rig.start();
        rig.fresh = false;
        rig.input.clutter_ts_us = 0;
        rig.tick();
        assert(rig.output.reason == (state == PushState::SCATTER ? "blind_scatter_observation_lost"
                                                                 : "sweep_observation_lost"));
        rig.stationary();
        rig.report(state == PushState::SCATTER    ? "scatter_missing_observation_closes_swing"
                   : state == PushState::APPROACH ? "approach_missing_observation_closes_swing"
                                                  : "mid_missing_observation_closes_swing");
    }
    for (PushState state :
         {PushState::SCATTER, PushState::APPROACH, PushState::MID_APPROACH, PushState::PILE}) {
        Rig rig(tuning(), state);
        rig.start();
        rig.input.heading_valid = false;
        rig.tick();
        rig.stationary();
        assert(swingCount(rig.task) == 0 && pass(rig.task) == 0);
        assert(rig.output.reason == (state == PushState::SCATTER        ? "blind_scatter_no_heading"
                                     : state == PushState::APPROACH     ? "sweep_upright_no_heading"
                                     : state == PushState::MID_APPROACH ? "mid_clear_no_heading"
                                                                        : "pile_no_heading"));
        rig.report(state == PushState::SCATTER        ? "scatter_heading_loss_closes_swing"
                   : state == PushState::APPROACH     ? "approach_heading_loss_closes_swing"
                   : state == PushState::MID_APPROACH ? "mid_heading_loss_closes_swing"
                                                      : "pile_heading_loss_closes_swing");
    }
}

void triggerConfiguration() {
    for (PushState state : {PushState::APPROACH, PushState::MID_APPROACH}) {
        for (bool non_supply : {false, true}) {
            Rig rig(tuning(), state);
            configure(rig.task, 3, 7);
            rig.task.enter(state, rig.input.now_us, "seed_sweep_trigger");
            rig.task.sweep_ledger_.reset();
            rig.task.t_.disperse_passes = 4;
            rig.task.t_.disperse_swings = 5;
            rig.input.target_valid = rig.input.geometry_valid = rig.input.target_region_valid =
                true;
            rig.input.target_id = rig.task.target_id_ = 19;
            rig.input.label = rig.task.label_ = non_supply ? "dangerous_object" : "ordinary_supply";
            rig.input.distance_m = .3f;
            rig.input.corridor_complete = rig.input.corridor_occlusion_free = true;
            rig.input.corridor.ordinary = 1;
            rig.input.corridor.dangerous = non_supply ? 0 : 1;
            rig.input.clutter_body = {{-.08f, .3f}, {.08f, .3f}, {.12f, .35f}};
            if (state == PushState::APPROACH)
                rig.task.approach_pitch_ = rig.task.t_.track_pitch_cdeg;
            rig.task.pitch_cmd_ = rig.task.desiredPitch();
            for (int i = 0; i < 8 && rig.task.sweep_phase_ < 4; ++i)
                rig.tick();
            assert(rig.task.sweep_phase_ == 4 && pass(rig.task) == 0);
            const bool small = state == PushState::MID_APPROACH && !non_supply;
            assert(rig.task.disperse_small_ == small);
            assert(passLimit(rig.task) == (small ? 1 : 4));
            assert(swingLimit(rig.task) == (small ? rig.task.t_.mid_neighbour_clear_swings : 5));
            rig.report(state == PushState::APPROACH
                           ? (non_supply ? "approach_non_supply_trigger_configures_limits"
                                         : "approach_dense_trigger_configures_limits")
                           : (non_supply ? "mid_non_supply_trigger_configures_limits"
                                         : "mid_neighbour_trigger_configures_small_limits"));
        }
    }
}

void budgetsAndLifecycle() {
    {
        Rig rig;
        rig.start();
        const auto start = rig.task.sweep_start_us_;
        rig.at(start + rig.task.t_.disperse_swing_budget_us);
        assert(swingCount(rig.task) == 0 && rig.output.motion.wz_rps > 0);
        rig.report("swing_budget_equality_still_moves");
        rig.at(start + rig.task.t_.disperse_swing_budget_us + 1);
        assert(swingCount(rig.task) == 1);
        rig.stationary();
        rig.report("swing_budget_strictly_exceeded_advances_with_stop_tick");
        rig.reachSwing();
        rig.tick();
        const auto centre = rig.task.sweep_start_us_;
        rig.at(centre + rig.task.t_.disperse_swing_budget_us);
        assert(pass(rig.task) == 0 && rig.output.reason == "disperse_centre");
        rig.report("centring_budget_equality_still_moves");
        rig.at(centre + rig.task.t_.disperse_swing_budget_us + 1);
        assert(pass(rig.task) == 1 && rig.output.reason == "blind_scatter_done");
        rig.report("centring_timeout_finishes_pass");
    }
    {
        Rig rig;
        rig.task.travel_ = 0;
        const auto start = rig.task.sweep_start_us_;
        rig.at(start + rig.task.t_.disperse_rush_budget_us);
        assert(rig.task.sweep_phase_ == 4 && rig.output.reason == "disperse_rush");
        rig.report("rush_budget_equality_keeps_rush");
        rig.at(start + rig.task.t_.disperse_rush_budget_us + 1);
        assert(rig.task.sweep_phase_ == 5 && swingCount(rig.task) == 0 &&
               rig.output.reason == "disperse_swing");
        rig.report("rush_timeout_falls_through_to_swing_same_tick");
    }
    for (PushState state : {PushState::SCATTER, PushState::APPROACH, PushState::MID_APPROACH}) {
        auto parameters = tuning();
        parameters.sweep_op_budget_us = 300000;
        Rig rig(parameters, state);
        rig.start();
        rig.at(kStart + parameters.sweep_op_budget_us);
        assert(rig.task.sweep_phase_ == 5 && swingCount(rig.task) == 0);
        rig.report(state == PushState::SCATTER    ? "scatter_operation_budget_equality"
                   : state == PushState::APPROACH ? "approach_operation_budget_equality"
                                                  : "mid_operation_budget_equality");
        rig.at(kStart + parameters.sweep_op_budget_us + 1);
        assert(rig.task.sweep_phase_ == 0 && pass(rig.task) == 0 && swingCount(rig.task) == 0);
        rig.stationary();
        rig.report(state == PushState::SCATTER    ? "scatter_operation_budget_cuts_swing"
                   : state == PushState::APPROACH ? "approach_operation_budget_cuts_swing"
                                                  : "mid_operation_budget_cuts_swing");
    }
    {
        Rig rig;
        configure(rig.task, 3, 4);
        rig.start();
        rig.reachSwing();
        rig.input.run = false;
        rig.tick();
        assert(rig.output.state == PushState::WAIT_START && swingCount(rig.task) == 1 &&
               passLimit(rig.task) == 3);
        rig.stationary();
        rig.report("pause_cancels_parent_phase_and_keeps_swing_history");
        rig.input.run = true;
        rig.tick();
        assert(rig.output.state == PushState::SCAN && swingCount(rig.task) == 1);
        rig.tick();
        assert(!rig.task.sweepChainOpen());
        rig.report("resume_settles_sweep_without_resetting_private_history");
        rig.input.reset = true;
        rig.tick();
        assert(rig.output.reason == "reset" && swingCount(rig.task) == 1 &&
               passLimit(rig.task) == 3 && swingLimit(rig.task) == 4);
        assert(!rig.task.sweepChainOpen() && rig.task.sweepDone() == 0);
        rig.report("explicit_reset_keeps_inactive_swing_history_and_clears_ledger");
        rig.input.reset = false;
        rig.arm(PushState::SCATTER);
        rig.start();
        assert(swingCount(rig.task) == 0 && passLimit(rig.task) == 3);
        rig.report("new_rush_after_reset_resets_only_swing_count");
    }
    {
        auto parameters = tuning();
        parameters.disperse_single_turn = true;
        Rig rig(parameters);
        configure(rig.task, 3, 4);
        rig.tick();
        rig.tick();
        assert(rig.output.reason == "disperse_turn");
        rig.input.heading_rad = parameters.disperse_turn_rad;
        rig.tick();
        assert(rig.output.reason == "blind_scatter_done" && pass(rig.task) == 0 &&
               swingCount(rig.task) == 0);
        rig.report("single_turn_completes_without_consuming_swing_pass");
    }
    {
        Rig rig;
        configure(rig.task, 3, 2);
        rig.start();
        rig.finishPass();
        assert(pass(rig.task) == 1);
        rig.task.clearTrip();
        assert(pass(rig.task) == 1 && swingCount(rig.task) == 2 && passLimit(rig.task) == 3);
        rig.tick();
        rig.report("clear_trip_preserves_inactive_swing_progress_and_limits");
    }
    {
        Rig rig(tuning(), PushState::PILE);
        configure(rig.task, 3, 4);
        rig.start();
        rig.finishPass();
        assert(pass(rig.task) == 1);
        rig.task.enter(PushState::PILE, rig.input.now_us, "seed_pile_inherited_limits");
        rig.task.pile_phase_ = 8;
        rig.task.pile_phase_us_ = rig.input.now_us;
        rig.task.pile_yaw0_ = rig.input.heading_rad;
        rig.tick();
        assert(rig.task.pile_phase_ == 9 && pass(rig.task) == 0 && swingCount(rig.task) == 0);
        assert(passLimit(rig.task) == 3 && swingLimit(rig.task) == 4);
        rig.report("pile_reentry_resets_progress_and_inherits_previous_limits");
        rig.task.travel_ =
            rig.task.sweep_travel0_ + rig.task.disperse_rush_m_ * rig.task.motionRatio() + .001f;
        rig.start();
        rig.report("pile_inherited_limits_survive_new_rush");
    }
}
} // namespace

int main() {
    callersAndPasses();
    smallPlanAndSpeeds();
    permissionsAndWaits();
    triggerConfiguration();
    budgetsAndLifecycle();
    std::cout << "disperse swing task integration tests passed\n";
}
