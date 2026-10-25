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
#if __has_include("rescue/task/push_in_stage.hpp")
#include "rescue/task/push_in_stage.hpp"
#endif
#if __has_include("rescue/task/sweep_ledger.hpp")
#include "rescue/task/sweep_ledger.hpp"
#endif
#define private public
#include "rescue/task/push_task.hpp"
#undef private

namespace {
using namespace rescue;
constexpr uint64_t kStart = 1000000;
constexpr uint64_t kTick = 50000;

TaskTuning tuning() {
    TaskTuning value;
    value.require_multi_view = false;
    value.confirm_frames = value.sweep_confirm = 1;
    value.blind_scatter_first_turn_rad = 0;
    value.blind_scatter_wait_us = 0;
    value.blind_scatter_need_ordinary = false;
    value.blind_scatter_max_y_m = .5f;
    value.blind_scatter_far_max_y_m = 2;
    value.blind_scatter_half_x_m = 1;
    value.blind_scatter_return = false;
    value.blind_scatter_swings = value.disperse_swings = value.mid_neighbour_clear_swings = 1;
    value.blind_scatter_cooldown_us = value.disperse_cooldown_us = value.sweep_gap_us = 0;
    value.sweep_escalate_max = 0;
    value.sweep_total_max = 5;
    value.sweep_view_handoff = value.post_clear_mid_view = false;
    value.disperse_swing_vx = 0;
    value.intermediate_pitch_cdeg = 2000;
    value.track_pitch_cdeg = value.near_pitch_cdeg = 4000;
    value.spin_unstick = value.visual_chase = value.scan_sighting_hold = false;
    value.sweep_time_budget_us = value.sweep_op_budget_us = 10000000;
    value.sweep_match_budget_us = 100000000;
    return value;
}

struct Rig {
    PushTask task;
    PushObservation input;
    PushOutput output;
    bool fresh_clutter = true, auto_camera = true;

    explicit Rig(TaskTuning parameters = tuning()) : task(parameters) {
        input.now_us = kStart;
        input.run = input.safety_ok = true;
        input.path_safe = input.retreat_safe = input.opponent_zone_clear = true;
        input.heading_valid = true;
        input.camera_pitch_stable = true;
        input.gripper_done = true;
        input.gripper_feedback_open = 0;
        seed(PushState::SCAN);
    }

    void seed(PushState state) {
        task.enter(state, input.now_us, "seed_sweep");
        if (state == PushState::APPROACH)
            task.approach_pitch_ = task.t_.track_pitch_cdeg;
        task.last_us_ = input.now_us;
        task.last_vx_ = task.last_wz_ = 0;
        const auto pitch = task.desiredPitch();
        if (pitch != kCameraPitchInvalid)
            task.pitch_cmd_ = pitch;
        task.pitch_wait_us_ = input.now_us;
        input.camera_pitch_cdeg = task.pitch_cmd_;
    }

    void refresh() {
        if (fresh_clutter)
            input.clutter_ts_us = input.now_us;
        if (!input.upright_body.empty())
            input.upright_ts_us = input.now_us;
        if (auto_camera) {
            const auto pitch = task.desiredPitch();
            if (pitch != kCameraPitchInvalid)
                input.camera_pitch_cdeg = pitch;
        }
    }

    PushOutput tick() {
        input.now_us += kTick;
        refresh();
        output = task.update(input);
        return output;
    }

    PushOutput at(uint64_t now) {
        input.now_us = now;
        task.last_us_ = now - kTick;
        task.last_vx_ = task.last_wz_ = 0;
        refresh();
        output = task.update(input);
        return output;
    }

    void group(float y, int count = 3) {
        input.clutter_body.clear();
        for (int index = 0; index < count; ++index)
            input.clutter_body.push_back({(float(index) - .5f * float(count - 1)) * .08f, y});
    }

    void target(const char* label = "ordinary_supply", float distance = .25f) {
        input.target_valid = input.geometry_valid = input.target_region_valid = true;
        input.target_id = task.target_id_ = task.capture_target_id_ = 7;
        input.label = task.label_ = label;
        input.distance_m = distance;
        input.heading_error = 0;
    }

    void noTarget() {
        input.target_valid = false;
    }

    uint64_t nearStart() {
        group(.3f);
        tick();
        assert(output.state == PushState::SCATTER && output.reason == "blind_scatter_start");
        assert(task.sweepChainOpen());
        return input.now_us;
    }

    uint64_t farStart() {
        group(1.2f);
        tick();
        assert(output.state == PushState::SCATTER && output.reason == "blind_scatter_far_start");
        assert(task.sweepChainOpen());
        return input.now_us;
    }

    uint64_t farPause() {
        farStart();
        tick();
        assert(output.state == PushState::SCATTER && output.reason == "blind_scatter_approach");
        task.travel_ = task.sweep_travel0_ + task.scatter_approach_m_ * task.motionRatio();
        task.last_vx_ = 0;
        tick();
        assert(output.state == PushState::SCAN && output.reason == "blind_scatter_approached");
        assert(task.sweepChainOpen() && task.sweepDone() == 0);
        input.clutter_body.clear();
        return input.now_us;
    }

    void toSwing() {
        if (task.sweep_phase_ == 0)
            tick();
        assert(task.sweep_phase_ == 4);
        task.travel_ = task.sweep_travel0_ + task.disperse_rush_m_ * task.motionRatio() + .001f;
        task.last_vx_ = 0;
        tick();
        assert(task.sweep_phase_ == 5 && task.sweepChainOpen());
        assert(output.motion.wz_rps != 0);
    }

    void finishSwing() {
        assert(task.sweep_phase_ == 5);
        const auto origin = task.sweep_yaw0_;
        const auto magnitude =
            task.disperse_small_ ? task.t_.mid_neighbour_clear_turn_rad : task.t_.disperse_turn_rad;
        input.heading_rad = origin + task.disperse_sign_ * magnitude;
        tick();
        input.heading_rad = origin;
        tick();
        assert(!task.sweepChainOpen());
    }

    uint64_t completeNear() {
        const auto start = nearStart();
        toSwing();
        finishSwing();
        assert(output.state == PushState::SCAN && output.reason == "blind_scatter_done");
        assert(task.sweepDone() == 1 && task.sweepSpentUs(input.now_us) == input.now_us - start);
        return input.now_us;
    }

    void newTrip() {
        input.clutter_body.clear();
        seed(PushState::TURN_SCAN);
        input.heading_rad = 0;
        tick();
        input.heading_rad = task.t_.turn_min_rad + .01f;
        tick();
        assert(output.state == PushState::SCAN && !task.fault_);
        assert(task.sweepDone() == 0 && task.sweepSpentUs(input.now_us) == 0 &&
               !task.sweepChainOpen());
    }

    void stationary() const {
        assert(output.motion.vx_mps == 0 && output.motion.wz_rps == 0);
    }

    void report(const char* test) const {
        std::cout << "case=" << test << " state=" << PushTask::name(output.state)
                  << " reason=" << output.reason << " fault=" << task.fault_
                  << " done=" << task.sweepDone() << " spent=" << task.sweepSpentUs(input.now_us)
                  << " open=" << task.sweepChainOpen() << " phase=" << task.sweep_phase_
                  << " phase_us=" << task.phase_us_ << " now_us=" << input.now_us
                  << " vx=" << output.motion.vx_mps << " wz=" << output.motion.wz_rps
                  << " frame=" << int(output.motion.gripper_offset)
                  << " pitch=" << output.motion.camera_pitch_cdeg
                  << " scatter_count=" << task.scatter_count_ << " far=" << task.scatter_far_
                  << '\n';
    }
};

void chainAccounting() {
    {
        Rig rig;
        const auto end = rig.completeNear();
        const auto spent = rig.task.sweepSpentUs(end);
        rig.report("near_swing_closes_and_counts_once");
        rig.input.clutter_body.clear();
        for (int index = 0; index < 3; ++index)
            rig.tick();
        assert(rig.task.sweepDone() == 1 && rig.task.sweepSpentUs(rig.input.now_us) == spent);
        assert(!rig.task.sweepChainOpen());
        rig.report("closed_chain_never_recounts");
    }
    {
        Rig rig;
        const auto start = rig.farStart();
        rig.tick();
        rig.group(.3f);
        rig.tick();
        assert(rig.output.state == PushState::SCATTER &&
               rig.output.reason == "blind_scatter_near_group");
        assert(rig.task.sweepSpentUs(rig.input.now_us) == rig.input.now_us - start);
        rig.report("far_to_near_reentry_keeps_active_start");
        rig.toSwing();
        rig.finishSwing();
        assert(rig.task.sweepDone() == 1 &&
               rig.task.sweepSpentUs(rig.input.now_us) == rig.input.now_us - start);
        rig.report("far_and_near_form_one_counted_chain");
    }
    {
        Rig rig;
        const auto paused = rig.farPause();
        const auto spent = rig.task.sweepSpentUs(paused);
        rig.at(paused + rig.task.t_.sweep_chain_link_us);
        assert(rig.task.sweepChainOpen() && rig.task.sweepSpentUs(rig.input.now_us) == spent);
        rig.report("paused_chain_stays_open_at_link_deadline");
        rig.at(paused + rig.task.t_.sweep_chain_link_us + 1);
        assert(!rig.task.sweepChainOpen() && rig.task.sweepDone() == 0);
        assert(rig.task.sweepSpentUs(rig.input.now_us) == spent);
        rig.report("paused_unswung_chain_settles_without_count");
    }
    {
        auto parameters = tuning();
        parameters.sweep_chain_link_us = 300000;
        parameters.sweep_op_budget_us = 500000;
        Rig rig(parameters);
        const auto paused = rig.farPause();
        const auto original_start = kStart + kTick;
        const auto active_far = rig.task.sweepSpentUs(paused);
        rig.group(.3f);
        const auto near_start = paused + parameters.sweep_chain_link_us;
        rig.at(near_start);
        assert(rig.output.state == PushState::SCATTER &&
               rig.task.sweepSpentUs(rig.input.now_us) == active_far);
        rig.at(original_start + parameters.sweep_op_budget_us);
        assert(rig.output.state == PushState::SCATTER && rig.task.sweepChainOpen());
        rig.report("linked_chain_retains_operation_deadline_at_equality");
        rig.at(original_start + parameters.sweep_op_budget_us + 1);
        assert(rig.output.state == PushState::SCAN && rig.output.reason == "sweep_op_timeout");
        assert(rig.task.sweepDone() == 0 && !rig.task.sweepChainOpen());
        assert(rig.task.sweepSpentUs(rig.input.now_us) ==
               active_far + rig.input.now_us - near_start);
        rig.report("linked_chain_excludes_pause_but_keeps_wall_deadline");
    }
    {
        auto parameters = tuning();
        parameters.sweep_gap_us = 9000000;
        Rig rig(parameters);
        const auto start = rig.nearStart();
        rig.target();
        rig.tick();
        assert(rig.output.state == PushState::SCAN &&
               rig.output.reason == "blind_scatter_target_seen");
        assert(rig.task.sweepDone() == 0 && !rig.task.sweepChainOpen());
        assert(rig.task.sweepSpentUs(rig.input.now_us) == rig.input.now_us - start);
        rig.noTarget();
        rig.tick();
        assert(rig.output.state == PushState::SCATTER &&
               rig.output.reason == "blind_scatter_start");
        rig.report("unswung_target_exit_leaves_group_gap_open");
    }
    {
        auto parameters = tuning();
        parameters.sweep_gap_us = 9000000;
        Rig rig(parameters);
        rig.nearStart();
        rig.input.heading_valid = false;
        rig.tick();
        assert(rig.output.state == PushState::SCAN &&
               rig.output.reason == "blind_scatter_no_heading");
        assert(rig.task.sweepDone() == 0 && !rig.task.sweepChainOpen());
        rig.input.heading_valid = true;
        rig.tick();
        assert(rig.output.state == PushState::SCAN && !rig.task.sweepChainOpen());
        rig.report("unswung_failure_still_arms_group_gap");
    }
}

void budgetsAndGaps() {
    for (bool match : {false, true}) {
        auto parameters = tuning();
        if (match)
            parameters.sweep_match_budget_us = 200000;
        else
            parameters.sweep_time_budget_us = 200000;
        Rig rig(parameters);
        const auto start = rig.nearStart();
        rig.at(start + 199999);
        assert(rig.output.state == PushState::SCATTER && rig.task.sweepChainOpen());
        rig.at(start + 200000);
        assert(rig.output.state == PushState::SCAN &&
               rig.output.reason == "sweep_budget_exhausted");
        assert(rig.task.sweepSpentUs(rig.input.now_us) == 200000 && rig.task.sweepDone() == 0);
        rig.stationary();
        rig.report(match ? "match_budget_cuts_at_equality" : "trip_time_budget_cuts_at_equality");
    }
    {
        auto parameters = tuning();
        parameters.sweep_op_budget_us = 200000;
        Rig rig(parameters);
        const auto start = rig.nearStart();
        rig.at(start + parameters.sweep_op_budget_us);
        assert(rig.output.state == PushState::SCATTER && rig.task.sweepChainOpen());
        rig.at(start + parameters.sweep_op_budget_us + 1);
        assert(rig.output.state == PushState::SCAN && rig.output.reason == "sweep_op_timeout");
        assert(rig.task.sweepSpentUs(rig.input.now_us) == parameters.sweep_op_budget_us + 1);
        rig.report("operation_deadline_uses_strict_greater_than");
    }
    {
        auto parameters = tuning();
        parameters.sweep_time_budget_us = 200000;
        Rig rig(parameters);
        const auto start = rig.nearStart();
        rig.input.camera_pitch_stable = false;
        rig.at(start + 200000);
        assert(rig.output.state == PushState::SCATTER && rig.output.reason == "camera_pitch_wait");
        assert(rig.task.sweepChainOpen() && rig.task.sweepSpentUs(rig.input.now_us) == 200000);
        rig.stationary();
        rig.input.camera_pitch_stable = true;
        rig.at(start + 250000);
        assert(rig.output.state == PushState::SCAN &&
               rig.output.reason == "sweep_budget_exhausted");
        assert(rig.task.sweepSpentUs(rig.input.now_us) == 250000 && !rig.task.sweepChainOpen());
        rig.report("pitch_wait_delays_budget_exit_without_pausing_time");
    }
    {
        auto parameters = tuning();
        parameters.sweep_total_max = 1;
        Rig rig(parameters);
        rig.completeNear();
        const auto spent = rig.task.sweepSpentUs(rig.input.now_us);
        rig.task.clearTrip();
        rig.tick();
        assert(rig.output.state == PushState::SCAN && !rig.task.sweepChainOpen());
        assert(rig.task.sweepDone() == 1 && rig.task.sweepSpentUs(rig.input.now_us) == spent);
        rig.report("local_clear_trip_retains_count_and_time_budget");
    }
    {
        auto parameters = tuning();
        parameters.sweep_gap_us = 300000;
        Rig rig(parameters);
        const auto end = rig.completeNear();
        rig.at(end + parameters.sweep_gap_us - 1);
        assert(rig.output.state == PushState::SCAN && !rig.task.sweepChainOpen());
        rig.at(end + parameters.sweep_gap_us);
        assert(rig.output.state == PushState::SCATTER && rig.task.sweepChainOpen());
        rig.report("group_gap_opens_at_exact_deadline");
    }
    {
        auto parameters = tuning();
        parameters.sweep_escalate_max = 2;
        Rig rig(parameters);
        rig.completeNear();
        rig.tick();
        assert(rig.output.state == PushState::SCAN && !rig.task.sweepChainOpen());
        rig.group(.3f, 4);
        rig.tick();
        assert(rig.output.state == PushState::SCATTER && rig.task.sweepChainOpen());
        rig.report("completed_count_increases_next_group_density");
    }
}

void approachAndMidSweeps() {
    {
        auto parameters = tuning();
        parameters.disperse = false;
        parameters.sweep_creep = false;
        Rig rig(parameters);
        rig.seed(PushState::APPROACH);
        rig.target();
        rig.input.upright_body = {{.1f, .25f}};
        rig.tick();
        const auto start = rig.input.now_us;
        assert(rig.output.reason == "sweep_upright_out" && rig.task.sweepChainOpen());
        rig.input.heading_rad = -parameters.sweep_turn_rad;
        rig.tick();
        assert(rig.output.reason == "sweep_upright_back");
        rig.input.heading_rad = 0;
        rig.tick();
        assert(rig.output.state == PushState::APPROACH &&
               rig.output.reason == "sweep_upright_done");
        assert(rig.task.sweepDone() == 1 && !rig.task.sweepChainOpen());
        assert(rig.task.sweepSpentUs(rig.input.now_us) == rig.input.now_us - start);
        rig.report("approach_upright_swing_counts_and_compensates_phase");
        const auto spent = rig.task.sweepSpentUs(rig.input.now_us);
        rig.input.upright_body.clear();
        rig.input.distance_m = .6f;
        rig.tick();
        assert(rig.task.sweepSpentUs(rig.input.now_us) == spent && rig.task.sweepDone() == 1);
    }
    {
        auto parameters = tuning();
        parameters.disperse = false;
        Rig rig(parameters);
        rig.seed(PushState::APPROACH);
        rig.target();
        rig.input.upright_body = {{.1f, .35f}};
        rig.tick();
        assert(rig.output.reason == "sweep_creep" && rig.task.sweepChainOpen());
        rig.input.heading_valid = false;
        rig.tick();
        assert(rig.output.reason == "sweep_upright_no_heading");
        assert(rig.task.sweepDone() == 0 && rig.task.sweepSpentUs(rig.input.now_us) == kTick);
        assert(!rig.task.sweepChainOpen());
        rig.report("approach_creep_without_swing_charges_time_only");
    }
    {
        Rig rig;
        rig.seed(PushState::MID_APPROACH);
        rig.target();
        rig.input.corridor_complete = rig.input.corridor_occlusion_free = true;
        rig.input.corridor.ordinary = rig.input.corridor.dangerous = 1;
        rig.group(.3f);
        rig.tick();
        const auto start = rig.input.now_us;
        assert(rig.output.reason == "mid_neighbour_clear" && rig.task.sweepChainOpen());
        rig.toSwing();
        rig.finishSwing();
        assert(rig.output.state == PushState::MID_REACQUIRE &&
               rig.output.reason == "mid_neighbour_clear_done");
        assert(rig.task.sweepDone() == 1 &&
               rig.task.sweepSpentUs(rig.input.now_us) == rig.input.now_us - start);
        rig.report("mid_neighbour_swing_uses_same_ledger");
    }
    {
        auto parameters = tuning();
        parameters.sweep_total_max = 1;
        parameters.sweep_gap_us = 9000000;
        Rig rig(parameters);
        rig.completeNear();
        rig.seed(PushState::MID_APPROACH);
        rig.target("dangerous");
        rig.tick();
        assert(rig.output.reason == "non_supply_rush" && rig.task.sweepChainOpen());
        rig.toSwing();
        rig.finishSwing();
        assert(rig.output.state == PushState::SCAN && rig.output.reason == "non_supply_swept");
        assert(rig.task.sweepDone() == 2);
        rig.report("non_supply_bypasses_count_and_gap_limits");
        rig.task.t_.sweep_time_budget_us = rig.task.sweepSpentUs(rig.input.now_us);
        rig.seed(PushState::MID_APPROACH);
        rig.target("dangerous");
        rig.tick();
        assert(!rig.task.sweepChainOpen() && rig.output.reason != "non_supply_rush");
        rig.report("non_supply_still_obeys_shared_time_budget");
    }
    {
        auto parameters = tuning();
        parameters.sweep_observation_guard = true;
        Rig rig(parameters);
        rig.seed(PushState::MID_APPROACH);
        rig.target();
        rig.input.corridor_complete = rig.input.corridor_occlusion_free = true;
        rig.input.corridor.ordinary = rig.input.corridor.dangerous = 1;
        rig.group(.3f);
        rig.tick();
        rig.toSwing();
        rig.noTarget();
        rig.fresh_clutter = false;
        rig.input.clutter_ts_us = 0;
        rig.tick();
        assert(rig.output.state == PushState::MID_REACQUIRE &&
               rig.output.reason == "sweep_observation_lost");
        assert(rig.task.sweepDone() == 1 && !rig.task.sweepChainOpen());
        const auto spent = rig.task.sweepSpentUs(rig.input.now_us);
        rig.tick();
        assert(rig.output.reason == "sweep_reobserve" &&
               rig.task.sweepSpentUs(rig.input.now_us) == spent);
        rig.stationary();
        rig.report("mid_observation_loss_closes_before_local_clear");
    }
}

void observationAndLifecycle() {
    {
        auto parameters = tuning();
        parameters.sweep_observation_guard = true;
        parameters.sweep_reobserve_us = 200000;
        Rig rig(parameters);
        const auto end = rig.completeNear();
        const auto spent = rig.task.sweepSpentUs(end);
        rig.at(end + 199999);
        assert(rig.output.reason == "sweep_reobserve" && !rig.task.sweepChainOpen());
        assert(rig.task.sweepSpentUs(rig.input.now_us) == spent);
        rig.stationary();
        rig.at(end + 200000);
        assert(rig.output.state == PushState::SCATTER && rig.task.sweepChainOpen());
        rig.report("observation_wait_ends_at_exact_deadline");
    }
    {
        auto parameters = tuning();
        parameters.sweep_observation_guard = true;
        parameters.sweep_reobserve_us = 200000;
        Rig rig(parameters);
        const auto end = rig.completeNear();
        rig.input.camera_pitch_stable = false;
        rig.at(end + 100000);
        assert(rig.output.reason == "camera_pitch_wait");
        rig.at(end + 200000);
        assert(rig.output.reason == "camera_pitch_wait");
        rig.input.camera_pitch_stable = true;
        rig.at(end + 300000);
        assert(rig.output.reason == "sweep_reobserve" && !rig.task.sweepChainOpen());
        rig.stationary();
        rig.at(end + 400000);
        assert(rig.output.state == PushState::SCATTER && rig.task.sweepChainOpen());
        rig.report("moving_camera_extends_existing_observation_wait");
    }
    {
        Rig rig;
        const auto start = rig.nearStart();
        rig.toSwing();
        rig.input.run = false;
        rig.tick();
        assert(rig.output.state == PushState::WAIT_START && rig.task.sweepChainOpen());
        rig.at(rig.input.now_us + 500000);
        assert(rig.output.state == PushState::WAIT_START && rig.task.sweepChainOpen());
        assert(rig.task.sweepSpentUs(rig.input.now_us) == rig.input.now_us - start);
        rig.input.run = true;
        rig.input.clutter_body.clear();
        rig.tick();
        assert(rig.output.state == PushState::SCAN && rig.output.reason == "resume" &&
               rig.task.sweepChainOpen());
        rig.tick();
        assert(!rig.task.sweepChainOpen() && rig.task.sweepDone() == 1);
        assert(rig.task.sweepSpentUs(rig.input.now_us) == rig.input.now_us - start);
        rig.report("operator_pause_time_is_charged_on_resumed_route_exit");
    }
    {
        Rig rig;
        const auto start = rig.nearStart();
        rig.seed(PushState::SCAN);
        rig.input.clutter_body.clear();
        rig.tick();
        assert(!rig.task.sweepChainOpen() && rig.task.sweepDone() == 0);
        assert(rig.task.sweepSpentUs(rig.input.now_us) == rig.input.now_us - start);
        const auto spent = rig.task.sweepSpentUs(rig.input.now_us);
        rig.tick();
        assert(rig.task.sweepSpentUs(rig.input.now_us) == spent);
        rig.report("unexpected_route_exit_closes_unswung_chain_once");
    }
    {
        auto parameters = tuning();
        parameters.sweep_total_max = 1;
        parameters.sweep_gap_us = 9000000;
        Rig rig(parameters);
        rig.completeNear();
        const auto match_spent = rig.task.sweepSpentUs(rig.input.now_us);
        rig.task.t_.sweep_match_budget_us = match_spent;
        rig.newTrip();
        rig.group(.3f);
        rig.tick();
        assert(rig.output.state == PushState::SCAN && !rig.task.sweepChainOpen());
        assert(rig.task.sweepDone() == 0 && rig.task.sweepSpentUs(rig.input.now_us) == 0);
        rig.report("new_trip_resets_trip_ledger_but_retains_match_budget");
        rig.task.t_.sweep_match_budget_us = match_spent + 1;
        rig.tick();
        assert(rig.output.state == PushState::SCATTER && rig.task.sweepChainOpen());
        rig.report("new_trip_also_clears_count_and_group_gap");
    }
    {
        Rig rig;
        rig.completeNear();
        rig.task.t_.sweep_match_budget_us = rig.task.sweepSpentUs(rig.input.now_us);
        rig.input.reset = true;
        rig.tick();
        assert(rig.output.state == PushState::WAIT_START && !rig.task.sweepChainOpen());
        assert(rig.task.sweepDone() == 0 && rig.task.sweepSpentUs(rig.input.now_us) == 0 &&
               !rig.task.fault_);
        rig.input.reset = false;
        rig.seed(PushState::SCAN);
        rig.tick();
        assert(rig.output.state == PushState::SCATTER && rig.task.sweepChainOpen());
        rig.report("explicit_reset_clears_match_budget_and_chain_history");
    }
}
} // namespace

int main() {
    chainAccounting();
    budgetsAndGaps();
    approachAndMidSweeps();
    observationAndLifecycle();
    std::cout << "sweep task integration tests passed\n";
}
