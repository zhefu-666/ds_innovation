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
#include "rescue/task/disperse_turn_stage.hpp"
#include "rescue/task/disperse_swing_stage.hpp"
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

int planActive(const PushTask& task) {
#ifdef RESCUE_HAS_RUSH_STAGE
    return task.disperse_rush_.planActive();
#else
    return task.swing_plan_active_;
#endif
}
int planPending(const PushTask& task) {
#ifdef RESCUE_HAS_RUSH_STAGE
    return task.disperse_rush_.planPending();
#else
    return task.swing_plan_pending_;
#endif
}
float capPositive(const PushTask& task) {
#ifdef RESCUE_HAS_RUSH_STAGE
    return task.disperse_rush_.capPositive();
#else
    return task.swing_cap_pos_;
#endif
}
float capNegative(const PushTask& task) {
#ifdef RESCUE_HAS_RUSH_STAGE
    return task.disperse_rush_.capNegative();
#else
    return task.swing_cap_neg_;
#endif
}
void seedPlanPending(PushTask& task) {
#ifdef RESCUE_HAS_RUSH_STAGE
    task.disperse_rush_.beginSweep(true);
#else
    task.swing_plan_pending_ = true;
    task.swing_plan_active_ = false;
    task.swing_cap_pos_ = task.swing_cap_neg_ = 99.f;
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

    explicit Rig(TaskTuning parameters = tuning(), PushState state = PushState::SCATTER) : task(parameters) {
        input.now_us = kStart;
        input.run = input.safety_ok = input.path_safe = input.retreat_safe = true;
        input.opponent_zone_clear = input.heading_valid = input.camera_pitch_stable = true;
        input.gripper_done = true;
        arm(state);
    }

    void arm(PushState state) {
        task.enter(state, input.now_us, "seed_rush");
        task.last_us_ = input.now_us;
        task.last_vx_ = task.last_wz_ = 0;
        if (state == PushState::APPROACH) task.approach_pitch_ = task.t_.track_pitch_cdeg;
        if (state == PushState::PILE) {
            task.pile_phase_ = 9;
            task.pile_phase_us_ = input.now_us;
            task.pile_travel0_ = 0;
            task.pile_chain_yaw_mid_ = 0;
            task.pile_chain_in_m_ = 0;
        }
        task.sweep_phase_ = 4;
        task.sweep_start_us_ = task.sweep_t0_us_ = input.now_us;
        task.sweep_travel0_ = 0;
        task.sweep_yaw0_ = input.heading_rad;
        task.disperse_rush_m_ = .5f;
        task.scatter_range_ = 1.2f;
        task.scatter_commit_ = true;
        task.disperse_sign_ = 1;
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
               << task.disperse_sign_ << ':' << task.disperse_rush_m_ << ':'
               << planActive(task) << ':' << planPending(task) << ':'
               << capPositive(task) << ':' << capNegative(task) << ':'
               << task.pile_phase_ << ':' << task.pile_chain_yaw_mid_ << ':' << task.pile_chain_in_m_;
        for (const unsigned char c : sample.str()) trace = (trace ^ c) * 1099511628211ULL;
    }

    void tick() { at(input.now_us + kTick); }
    void report(const char* name) const {
        std::cout << std::setprecision(9) << "case=" << name << " trace=" << trace
                  << " state=" << PushTask::name(output.state) << " reason=" << output.reason
                  << " phase=" << task.sweep_phase_ << " phase_us=" << task.phase_us_
                  << " start=" << task.sweep_start_us_ << " yaw0=" << task.sweep_yaw0_
                  << " travel0=" << task.sweep_travel0_ << " travel=" << task.travel_
                  << " sign=" << task.disperse_sign_ << " rush=" << task.disperse_rush_m_
                  << " plan=" << planActive(task) << '/' << planPending(task)
                  << " cap=" << capPositive(task) << ',' << capNegative(task)
                  << " vx=" << output.motion.vx_mps << " wz=" << output.motion.wz_rps
                  << " pitch=" << output.motion.camera_pitch_cdeg
                  << " frame=" << int(output.motion.gripper_offset)
                  << " pile=" << task.pile_phase_
                  << " pile_mid=" << task.pile_chain_yaw_mid_ << " pile_in=" << task.pile_chain_in_m_ << '\n';
    }
};

void rushAndCompletion() {
    for (PushState state : {PushState::SCATTER, PushState::APPROACH, PushState::MID_APPROACH, PushState::PILE}) {
        Rig rig(tuning(), state);
        rig.tick();
        assert(rig.task.sweep_phase_ == 4 && rig.output.motion.vx_mps > 0 && rig.output.motion.wz_rps == 0);
        assert(std::string(rig.output.reason) == "disperse_rush");
        rig.report(state == PushState::SCATTER ? "scatter_rush_holds_phase_four" :
                   state == PushState::APPROACH ? "approach_rush_holds_phase_four" :
                   state == PushState::MID_APPROACH ? "mid_rush_holds_phase_four" : "pile_rush_holds_phase_four");
        rig.task.travel_ = rig.task.sweep_travel0_ + rig.task.disperse_rush_m_ * rig.task.motionRatio();
        rig.tick();
        assert(rig.task.sweep_phase_ != 4);
        rig.report(state == PushState::SCATTER ? "scatter_rush_travel_equality_swings" :
                   state == PushState::APPROACH ? "approach_rush_travel_equality_swings" :
                   state == PushState::MID_APPROACH ? "mid_rush_travel_equality_swings" : "pile_rush_travel_equality_swings");
    }
    {
        Rig rig;
        rig.tick();
        rig.task.travel_ = rig.task.sweep_travel0_ + rig.task.disperse_rush_m_ * rig.task.motionRatio() - .001f;
        rig.at(rig.task.sweep_start_us_ + rig.task.t_.disperse_rush_budget_us);
        assert(rig.task.sweep_phase_ == 4 && std::string(rig.output.reason) == "disperse_rush");
        rig.report("rush_budget_equality_keeps_rushing");
        rig.at(rig.task.sweep_start_us_ + rig.task.t_.disperse_rush_budget_us + 1);
        assert(rig.task.sweep_phase_ != 4);
        rig.report("rush_budget_strictly_exceeded_swings");
    }
}

void speedsAndPermissions() {
    {
        TaskTuning t = tuning();
        t.max_speed = .2f;
        t.disperse_speed = .6f;
        Rig rig(t);
        rig.tick();
        assert(rig.output.motion.vx_mps == t.disperse_speed); // the rush bypasses the ordinary speed cap
        rig.report("rush_speed_bypasses_max_speed");
    }
    {
        TaskTuning t = tuning();
        t.first_burst_full_impact = true;
        Rig rig(t);
        rig.task.burst_swing_ = true;
        rig.tick();
        assert(std::string(rig.output.reason) == "first_burst_impact" && rig.output.motion.vx_mps == t.first_burst_speed);
        rig.report("scatter_opening_impact_speed");
    }
    {
        TaskTuning t = tuning();
        t.first_burst_full_impact = true;
        Rig rig(t, PushState::APPROACH);
        rig.task.burst_swing_ = true;
        rig.tick();
        assert(std::string(rig.output.reason) == "disperse_rush" && rig.output.motion.vx_mps == t.disperse_speed);
        rig.report("approach_keeps_ordinary_rush_speed");
    }
    {
        Rig rig;
        rig.input.path_safe = false;
        rig.tick();
        assert(rig.task.sweep_phase_ == 4 && rig.output.motion.vx_mps == 0);
        assert(std::string(rig.output.reason) == "forward_path_blocked");
        rig.report("blocked_path_keeps_position_in_rush");
    }
}

void outsideSwingPlan() {
    {
        Rig rig;
        rig.task.t_.swing_avoid_outside = true;
        seedPlanPending(rig.task);
        rig.input.clutter_body = {{-.1918f, .8510f}}; // left of the rush end, outside the group
        rig.tick();
        assert(planActive(rig.task) && !planPending(rig.task));
        assert(rig.task.disperse_sign_ == -1); // the blocked left side sends the first swing right
        assert(std::abs(capPositive(rig.task) - .35f) < 1e-3f && capNegative(rig.task) == 99.f);
        rig.report("outside_block_caps_the_first_swing");
        rig.task.travel_ = rig.task.sweep_travel0_ + rig.task.disperse_rush_m_ * rig.task.motionRatio();
        rig.tick();
        assert(planActive(rig.task) && std::abs(capPositive(rig.task) - .35f) < 1e-3f);
        rig.report("plan_survives_the_swing_transition");
    }
    {
        Rig rig;
        rig.task.t_.swing_avoid_outside = true;
        rig.task.disperse_sign_ = 1;
        seedPlanPending(rig.task);
        rig.input.clutter_body = {{-.1918f, .8510f}};
        rig.fresh = false;
        rig.input.clutter_ts_us = rig.input.now_us - rig.task.t_.sweep_fresh_us - 1;
        rig.tick();
        assert(!planActive(rig.task) && !planPending(rig.task) && rig.task.disperse_sign_ == 1);
        rig.report("stale_clutter_skips_the_outside_plan");
    }
    {
        Rig rig;
        rig.task.t_.swing_avoid_outside = true;
        seedPlanPending(rig.task);
        rig.input.clutter_body = {{-.1918f, .8510f}, {.1918f, .8510f}}; // symmetric: both caps equal, direction unchanged
        rig.tick();
        assert(planActive(rig.task) && rig.task.disperse_sign_ == 1);
        assert(std::abs(capPositive(rig.task) - .35f) < 1e-3f && std::abs(capNegative(rig.task) - .35f) < 1e-3f);
        rig.report("equal_caps_keep_the_existing_direction");
    }
}

void planScopedToScatter() {
    for (PushState state : {PushState::APPROACH, PushState::MID_APPROACH, PushState::PILE}) {
        Rig rig(tuning(), state);
        rig.task.t_.swing_avoid_outside = true;
        seedPlanPending(rig.task);
        rig.input.clutter_body = {{-.1918f, .8510f}};
        rig.tick();
        assert(!planActive(rig.task) && !planPending(rig.task));
        rig.report(state == PushState::APPROACH ? "approach_never_plans_swing_caps" :
                   state == PushState::MID_APPROACH ? "mid_never_plans_swing_caps" : "pile_never_plans_swing_caps");
    }
}
} // namespace

int main() {
    rushAndCompletion();
    speedsAndPermissions();
    outsideSwingPlan();
    planScopedToScatter();
    std::cout.flush();
    return 0;
}
