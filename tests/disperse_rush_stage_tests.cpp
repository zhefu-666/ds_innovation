#include "rescue/task/disperse_rush_stage.hpp"
#include "rescue/task/push_task.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace rescue;

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition)
        throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
}
#define CHECK(expression) check((expression), #expression, __LINE__)

void expectRush(const DisperseRushResult& result, float vx, const char* reason) {
    CHECK(result.outcome == DisperseRushOutcome::Continue);
    CHECK(result.motion_requested && result.vx == vx);
    CHECK(result.reason && std::string(result.reason) == reason);
}

void expectComplete(const DisperseRushResult& result) {
    CHECK(result.outcome == DisperseRushOutcome::Complete);
    CHECK(!result.motion_requested && result.vx == 0 && result.reason == nullptr);
}

struct Fixture {
    DisperseRushStage stage;
    TaskTuning tuning;
    PushObservation obs;
    DisperseRushContext context;

    Fixture() {
        tuning.swing_avoid_outside = true;
        tuning.disperse_radius_m = .3f;
        tuning.swing_avoid_radius_m = 1.f;
        tuning.swing_avoid_margin_rad = .1f;
        tuning.disperse_rush_budget_us = 1000;
        tuning.disperse_speed = .5f;
        tuning.first_burst_full_impact = true;
        tuning.first_burst_speed = .9f;
        context.now_us = context.sweep_start_us = 10000;
        context.travel_m = context.sweep_travel_m = 0;
        context.motion_ratio = 1;
        context.rush_distance_m = 1;
        context.cluster_range_m = 1.2f;
        context.scatter_state = true;
        context.clutter_fresh = true;
        context.pitch_ready = true;
    }

    // A point at body bearing `bearing` (left positive) and `range` from the rush end.
    static cv::Point2f at(float bearing, float range, float end_y) {
        return cv::Point2f(-std::sin(bearing) * range, end_y + std::cos(bearing) * range);
    }

    DisperseRushResult update() {
        return stage.update(tuning, obs, context);
    }
};

void defaultStateAndLifecycle() {
    Fixture f;
    CHECK(!f.stage.planActive() && f.stage.capPositive() == 99.f && f.stage.capNegative() == 99.f);
    f.stage.beginSweep(false);
    expectRush(f.update(), f.tuning.disperse_speed, "disperse_rush");
    CHECK(!f.stage.planActive());

    f.stage.beginSweep(true);
    f.obs.clutter_body = {Fixture::at(.5f, .5f, 1.f)};
    const auto plan = f.update();
    CHECK(plan.plan_event && plan.outside_n == 1 && std::abs(plan.cap_positive - .4f) < 1e-5f);
    CHECK(f.stage.planActive());

    f.stage.clear();
    CHECK(!f.stage.planActive() && f.stage.capPositive() == 99.f && f.stage.capNegative() == 99.f);

    f.stage.beginSweep(true);
    f.update();
    CHECK(f.stage.planActive());
    f.stage.clearFlags();
    CHECK(!f.stage.planActive() &&
          f.stage.capPositive() == .4f); // flags only: caps stay as planned
}

void planNeedsSwitchFreshClutterAndPitch() {
    { // avoid switch off: the pending request is consumed without a plan
        Fixture f;
        f.tuning.swing_avoid_outside = false;
        f.stage.beginSweep(true);
        const auto result = f.update();
        CHECK(!result.plan_event && !f.stage.planActive());
    }
    { // stale clutter: no plan
        Fixture f;
        f.context.clutter_fresh = false;
        f.stage.beginSweep(true);
        CHECK(!f.update().plan_event);
    }
    { // a plan never belongs to the approach or pile sweeps
        Fixture f;
        f.context.scatter_state = false;
        f.stage.beginSweep(true);
        CHECK(!f.update().plan_event);
    }
    { // far judge defers the plan until the far pitch is in place
        Fixture f;
        f.context.far_judge = true;
        f.context.pitch_ready = false;
        f.obs.clutter_body = {Fixture::at(.5f, .5f, 1.f)};
        f.stage.beginSweep(true);
        CHECK(!f.update().plan_event && f.stage.planPending());
        f.context.pitch_ready = true;
        CHECK(f.update().plan_event && !f.stage.planPending());
    }
}

void planDefersToTheCompletionFrame() {
    Fixture f;
    f.context.far_judge = true;
    f.context.pitch_ready = false;
    f.obs.clutter_body = {Fixture::at(.5f, .5f, 0.f)}; // measured from the completion-time rush end
    f.stage.beginSweep(true);
    expectRush(f.update(), f.tuning.disperse_speed, "disperse_rush");
    CHECK(!f.stage.planActive() && f.stage.planPending());
    f.context.travel_m =
        f.context.sweep_travel_m + f.context.rush_distance_m; // travel equality completes
    const auto result = f.update();
    CHECK(result.outcome == DisperseRushOutcome::Complete);
    CHECK(result.plan_event && result.outside_n == 1);
    CHECK(f.stage.planActive() && !f.stage.planPending());
}

void planSkipsClusterMembersAndFarBlocks() {
    Fixture f;
    f.stage.beginSweep(true);
    f.obs.clutter_body = {
        {0.f, 1.3f},                 // inside the dispersal cluster
        {0.f, 3.f},                  // beyond the swing-avoid radius of the rush end
        Fixture::at(.5f, .5f, 1.f),  // counted
        Fixture::at(-.5f, .5f, 1.f), // counted
    };
    const auto result = f.update();
    CHECK(result.plan_event && result.outside_n == 2);
    CHECK(std::abs(result.cap_positive - .4f) < 1e-5f);
    CHECK(std::abs(result.cap_negative - .4f) < 1e-5f);
}

void planClampsCapsAndPicksTheFreerSide() {
    { // nearer blocks on the left shrink the left cap to zero
        Fixture f;
        f.stage.beginSweep(true);
        f.obs.clutter_body = {Fixture::at(.05f, .5f, 1.f), Fixture::at(-.6f, .5f, 1.f)};
        const auto result = f.update();
        CHECK(result.cap_positive == 0);
        CHECK(std::abs(result.cap_negative - .5f) < 1e-5f);
        CHECK(result.first_sign == -1.f);
    }
    { // equal caps keep the existing direction
        Fixture f;
        f.stage.beginSweep(true);
        f.obs.clutter_body = {Fixture::at(.5f, .5f, 1.f), Fixture::at(-.5f, .5f, 1.f)};
        CHECK(f.update().first_sign == 0.f);
    }
    { // the locked target joins the outside blocks
        Fixture f;
        f.stage.beginSweep(true);
        f.context.has_target = true;
        f.context.target_body = Fixture::at(-.5f, .5f, 1.f);
        const auto result = f.update();
        CHECK(result.outside_n == 1);
        CHECK(std::abs(result.cap_negative - .4f) < 1e-5f);
        CHECK(result.first_sign == 1.f); // left (99) is freer
    }
}

void rushSpeedAndCompletionBoundaries() {
    { // ordinary rush
        Fixture f;
        f.stage.beginSweep(false);
        expectRush(f.update(), .5f, "disperse_rush");
    }
    { // the opening impact speed applies to a SCATTER sweep only
        Fixture f;
        f.stage.beginSweep(false);
        f.context.burst_swing = true;
        expectRush(f.update(), .9f, "first_burst_impact");
        f.context.scatter_state = false;
        expectRush(f.update(), .5f, "disperse_rush");
    }
    { // travel equality already completes
        Fixture f;
        f.stage.beginSweep(false);
        f.context.rush_distance_m = .25f;
        f.context.travel_m = f.context.sweep_travel_m + .25f;
        expectComplete(f.update());
    }
    { // the budget is exceeded strictly
        Fixture f;
        f.stage.beginSweep(false);
        f.context.now_us = f.context.sweep_start_us + f.tuning.disperse_rush_budget_us;
        expectRush(f.update(), .5f, "disperse_rush");
        f.context.now_us += 1;
        expectComplete(f.update());
    }
    { // the motion ratio scales the required travel
        Fixture f;
        f.stage.beginSweep(false);
        f.context.rush_distance_m = .25f;
        f.context.motion_ratio = .5f;
        f.context.travel_m = f.context.sweep_travel_m + .1f;
        expectRush(f.update(), .5f, "disperse_rush");
        f.context.travel_m += .025f; // rush_distance_m * motion_ratio
        expectComplete(f.update());
    }
}
} // namespace

int main() {
    try {
        defaultStateAndLifecycle();
        planNeedsSwitchFreshClutterAndPitch();
        planDefersToTheCompletionFrame();
        planSkipsClusterMembersAndFarBlocks();
        planClampsCapsAndPicksTheFreerSide();
        rushSpeedAndCompletionBoundaries();
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        return 1;
    }
    return 0;
}
