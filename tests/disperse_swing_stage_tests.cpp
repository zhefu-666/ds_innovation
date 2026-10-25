#include "rescue/task/disperse_swing_stage.hpp"
#include "rescue/push_task.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace rescue;

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
}
#define CHECK(expression) check((expression), #expression, __LINE__)

void expectEvent(const DisperseSwingResult& result, DisperseSwingOutcome outcome, bool restart = false) {
    CHECK(result.outcome == outcome);
    CHECK(result.restart_clock == restart);
    CHECK(!result.motion_requested && result.vx == 0 && result.direction == 0 && !result.fast);
    CHECK(result.reason == nullptr);
}

void expectMotion(const DisperseSwingResult& result, float vx, float direction,
                  const char* reason, bool fast = false) {
    CHECK(result.outcome == DisperseSwingOutcome::Continue && !result.restart_clock);
    CHECK(result.motion_requested && result.vx == vx && result.direction == direction);
    CHECK(result.fast == fast);
    CHECK(result.reason && std::string(result.reason) == reason);
}

struct Fixture {
    DisperseSwingStage stage;
    TaskTuning tuning;
    DisperseSwingContext context;

    Fixture() {
        context.now_us = context.sweep_start_us = 10000;
        tuning.disperse_turn_rad = 1;
        tuning.mid_neighbour_clear_turn_rad = .5f;
        tuning.disperse_swing_tol_rad = .125f;
        tuning.sweep_return_tol_rad = .0625f;
        tuning.disperse_swing_creep_m = .25f;
        tuning.mid_neighbour_clear_creep_m = .125f;
        tuning.disperse_swing_vx = .14f;
        tuning.disperse_swing_budget_us = 1000;
    }

    DisperseSwingResult update() {
        const auto result = stage.update(tuning, context);
        if (result.restart_clock) context.sweep_start_us = context.now_us;
        return result;
    }

    void finishSwing() {
        context.now_us = context.sweep_start_us + tuning.disperse_swing_budget_us + 1;
        expectEvent(update(), DisperseSwingOutcome::Continue, true);
    }

    void centre() {
        context.heading_rad = context.sweep_yaw;
    }
};

void defaultStateAndSelectiveLifecycleResets() {
    Fixture f;
    CHECK(f.stage.swingCount() == 0 && f.stage.pass() == 0);
    CHECK(f.stage.passLimit() == 1 && f.stage.swingLimit() == 6);
    f.finishSwing();
    f.stage.configure(3, 0);
    CHECK(f.stage.swingCount() == 1 && f.stage.pass() == 0);
    CHECK(f.stage.passLimit() == 3 && f.stage.swingLimit() == 0);
    expectEvent(f.update(), DisperseSwingOutcome::NextPass);
    CHECK(f.stage.pass() == 1);
    f.stage.beginSwing();
    CHECK(f.stage.swingCount() == 0 && f.stage.pass() == 1);
    CHECK(f.stage.passLimit() == 3 && f.stage.swingLimit() == 0);
    expectEvent(f.update(), DisperseSwingOutcome::NextPass);
    f.stage.restartPass();
    CHECK(f.stage.swingCount() == 0 && f.stage.pass() == 0);
    CHECK(f.stage.passLimit() == 3 && f.stage.swingLimit() == 0);
    f.stage.configure(2, 2);
    f.finishSwing();
    f.stage.restartPass();
    CHECK(f.stage.swingCount() == 1 && f.stage.pass() == 0);
    f.stage.configure(4, 3);
    CHECK(f.stage.swingCount() == 1 && f.stage.pass() == 0);
    CHECK(f.stage.passLimit() == 4 && f.stage.swingLimit() == 3);
}

void alternatingSwingsNeedASeparateCentreTick() {
    for (const float direction : {-1.f, 1.f}) {
        Fixture f;
        f.context.direction = direction;
        f.stage.configure(1, 2);
        expectMotion(f.update(), f.tuning.disperse_swing_vx, direction, "disperse_swing");
        f.context.heading_rad = direction;
        expectEvent(f.update(), DisperseSwingOutcome::Continue, true);
        CHECK(f.stage.swingCount() == 1 && f.stage.pass() == 0);
        expectMotion(f.update(), f.tuning.disperse_swing_vx, -direction, "disperse_swing");
        f.context.heading_rad = -direction;
        expectEvent(f.update(), DisperseSwingOutcome::Continue, true);
        CHECK(f.stage.swingCount() == 2 && f.stage.pass() == 0);
        expectMotion(f.update(), 0, direction, "disperse_centre");
        f.centre();
        expectEvent(f.update(), DisperseSwingOutcome::Complete);
        CHECK(f.stage.swingCount() == 2 && f.stage.pass() == 1);
    }
}

void swingAndCentreTolerancesAreInclusive() {
    for (const float direction : {-1.f, 1.f}) {
        Fixture f;
        f.context.direction = direction;
        f.stage.configure(1, 1);
        f.context.heading_rad = direction * .874f;
        expectMotion(f.update(), f.tuning.disperse_swing_vx, direction, "disperse_swing");
        f.context.heading_rad = direction * .875f;
        expectEvent(f.update(), DisperseSwingOutcome::Continue, true);
        f.context.heading_rad = direction * .0626f;
        expectMotion(f.update(), 0, -direction, "disperse_centre");
        f.context.heading_rad = direction * .0625f;
        expectEvent(f.update(), DisperseSwingOutcome::Complete);
    }
}

void overshootingTheGoalPreservesTheReturnDirection() {
    for (const float direction : {-1.f, 1.f}) {
        Fixture f;
        f.context.direction = direction;
        f.context.heading_rad = direction * 1.25f;
        expectMotion(f.update(), f.tuning.disperse_swing_vx, -direction, "disperse_swing");
        CHECK(f.stage.swingCount() == 0);
    }
}

void swingAndCentreBudgetsAreExclusive() {
    Fixture f;
    f.stage.configure(1, 1);
    f.context.now_us += f.tuning.disperse_swing_budget_us;
    expectMotion(f.update(), f.tuning.disperse_swing_vx, 1, "disperse_swing");
    ++f.context.now_us;
    expectEvent(f.update(), DisperseSwingOutcome::Continue, true);
    CHECK(f.context.sweep_start_us == f.context.now_us);
    f.context.heading_rad = .5f;
    f.context.now_us += f.tuning.disperse_swing_budget_us;
    expectMotion(f.update(), 0, -1, "disperse_centre");
    ++f.context.now_us;
    expectEvent(f.update(), DisperseSwingOutcome::Complete);
}

void creepingStopsAtTheScaledDistanceAndNeverOccursWhileCentring() {
    for (const bool small : {false, true}) {
        Fixture f;
        f.stage.configure(1, 1);
        f.context.small = small;
        f.context.motion_ratio = 2;
        f.context.sweep_travel_m = 4;
        const float limit = (small ? f.tuning.mid_neighbour_clear_creep_m : f.tuning.disperse_swing_creep_m) * 2;
        f.context.travel_m = 4 + limit - .001f;
        expectMotion(f.update(), f.tuning.disperse_swing_vx, 1, "disperse_swing");
        f.context.travel_m = 4 + limit;
        expectMotion(f.update(), 0, 1, "disperse_swing");
        f.context.travel_m += .001f;
        expectMotion(f.update(), 0, 1, "disperse_swing");
        f.finishSwing();
        f.context.travel_m = 0;
        f.context.heading_rad = .25f;
        expectMotion(f.update(), 0, -1, "disperse_centre");
    }
}

void smallUsesItsOwnAngleAndPlannedCapsFollowTheActualSide() {
    for (const float direction : {-1.f, 1.f}) {
        Fixture small;
        small.context.direction = direction;
        small.context.small = true;
        small.context.heading_rad = direction * .374f;
        expectMotion(small.update(), small.tuning.disperse_swing_vx, direction, "disperse_swing");
        small.context.heading_rad = direction * .375f;
        expectEvent(small.update(), DisperseSwingOutcome::Continue, true);

        Fixture plan;
        plan.context.direction = direction;
        plan.context.swing_plan_active = true;
        plan.context.cap_positive = direction > 0 ? .5f : .75f;
        plan.context.cap_negative = direction < 0 ? .5f : .75f;
        plan.context.heading_rad = direction * .374f;
        expectMotion(plan.update(), plan.tuning.disperse_swing_vx, direction, "disperse_swing");
        plan.context.heading_rad = direction * .375f;
        expectEvent(plan.update(), DisperseSwingOutcome::Continue, true);
        plan.context.heading_rad = -direction * .625f;
        expectEvent(plan.update(), DisperseSwingOutcome::Continue, true);
        CHECK(plan.stage.swingCount() == 2);
    }
}

void inactivePlansAndWideCapsKeepTheConfiguredAngle() {
    Fixture inactive;
    inactive.context.cap_positive = 0;
    expectMotion(inactive.update(), inactive.tuning.disperse_swing_vx, 1, "disperse_swing");
    Fixture wide;
    wide.context.swing_plan_active = true;
    wide.context.cap_positive = 2;
    wide.context.heading_rad = .875f;
    expectEvent(wide.update(), DisperseSwingOutcome::Continue, true);
    Fixture small;
    small.context.small = true;
    small.context.swing_plan_active = true;
    small.context.cap_positive = .25f;
    small.context.heading_rad = .125f;
    expectEvent(small.update(), DisperseSwingOutcome::Continue, true);
}

void centringIgnoresPlannedCaps() {
    Fixture f;
    f.stage.configure(1, 0);
    f.context.swing_plan_active = true;
    f.context.cap_positive = f.context.cap_negative = 0;
    f.context.heading_rad = .25f;
    expectMotion(f.update(), 0, -1, "disperse_centre");
    f.centre();
    expectEvent(f.update(), DisperseSwingOutcome::Complete);
}

void burstAndApproachFastPreserveReasonsAndSmallException() {
    for (const bool burst : {false, true}) {
        for (const bool approach : {false, true}) {
            for (const bool small : {false, true}) {
                Fixture f;
                f.stage.configure(1, 1);
                f.context.burst = burst;
                f.context.approach_fast = approach;
                f.context.small = small;
                const bool fast = burst || (approach && !small);
                expectMotion(f.update(), f.tuning.disperse_swing_vx, 1,
                             burst ? "first_burst_swing" : "disperse_swing", fast);
                f.finishSwing();
                f.context.heading_rad = .25f;
                expectMotion(f.update(), 0, -1,
                             burst ? "first_burst_centre" : "disperse_centre", fast);
            }
        }
    }
}

void nextPassIsAnEventUntilTheParentBeginsTheNextSwing() {
    Fixture f;
    f.stage.configure(2, 1);
    f.finishSwing();
    f.centre();
    expectEvent(f.update(), DisperseSwingOutcome::NextPass);
    CHECK(f.stage.pass() == 1 && f.stage.swingCount() == 1);
    f.context.direction = -f.context.direction;
    f.context.sweep_start_us = f.context.now_us;
    f.context.sweep_travel_m = f.context.travel_m;
    f.stage.beginSwing();
    CHECK(f.stage.pass() == 1 && f.stage.swingCount() == 0);
    expectMotion(f.update(), f.tuning.disperse_swing_vx, -1, "disperse_swing");
    f.finishSwing();
    f.centre();
    expectEvent(f.update(), DisperseSwingOutcome::Complete);
    CHECK(f.stage.pass() == 2 && f.stage.swingCount() == 1);
}

void zeroAndNegativeLimitsAreNotClamped() {
    for (const int passes : {-1, 0, 1}) {
        for (const int swings : {-1, 0}) {
            Fixture f;
            f.stage.configure(passes, swings);
            CHECK(f.stage.passLimit() == passes && f.stage.swingLimit() == swings);
            expectEvent(f.update(), DisperseSwingOutcome::Complete);
            CHECK(f.stage.pass() == 1 && f.stage.swingCount() == 0);
        }
    }
}

void wrappedHeadingUsesTheShortAngleInBothDirections() {
    for (const float direction : {-1.f, 1.f}) {
        Fixture f;
        f.context.direction = direction;
        f.context.sweep_yaw = direction * (float(CV_PI) - .25f);
        f.context.heading_rad = direction * (-float(CV_PI) + .25f);
        expectMotion(f.update(), f.tuning.disperse_swing_vx, direction, "disperse_swing");
        f.context.heading_rad = direction * (-float(CV_PI) + .75f);
        expectEvent(f.update(), DisperseSwingOutcome::Continue, true);
        f.stage.configure(1, 0);
        f.context.heading_rad = direction * (-float(CV_PI) + .25f);
        expectMotion(f.update(), 0, -direction, "disperse_centre");
        f.context.heading_rad = f.context.sweep_yaw;
        expectEvent(f.update(), DisperseSwingOutcome::Complete);
    }
}

void zeroAndUnsignedRollbackClocksRetainTheirOriginalBehavior() {
    Fixture zero;
    zero.context.now_us = zero.context.sweep_start_us = 0;
    expectMotion(zero.update(), zero.tuning.disperse_swing_vx, 1, "disperse_swing");
    zero.context.now_us = zero.tuning.disperse_swing_budget_us;
    expectMotion(zero.update(), zero.tuning.disperse_swing_vx, 1, "disperse_swing");
    ++zero.context.now_us;
    expectEvent(zero.update(), DisperseSwingOutcome::Continue, true);

    Fixture rollback;
    --rollback.context.now_us;
    expectEvent(rollback.update(), DisperseSwingOutcome::Continue, true);
    CHECK(rollback.stage.swingCount() == 1);
    rollback.stage.configure(1, 0);
    rollback.context.heading_rad = .5f;
    --rollback.context.now_us;
    expectEvent(rollback.update(), DisperseSwingOutcome::Complete);
}
} // namespace

int main() {
    try {
        defaultStateAndSelectiveLifecycleResets();
        alternatingSwingsNeedASeparateCentreTick();
        swingAndCentreTolerancesAreInclusive();
        overshootingTheGoalPreservesTheReturnDirection();
        swingAndCentreBudgetsAreExclusive();
        creepingStopsAtTheScaledDistanceAndNeverOccursWhileCentring();
        smallUsesItsOwnAngleAndPlannedCapsFollowTheActualSide();
        inactivePlansAndWideCapsKeepTheConfiguredAngle();
        centringIgnoresPlannedCaps();
        burstAndApproachFastPreserveReasonsAndSmallException();
        nextPassIsAnEventUntilTheParentBeginsTheNextSwing();
        zeroAndNegativeLimitsAreNotClamped();
        wrappedHeadingUsesTheShortAngleInBothDirections();
        zeroAndUnsignedRollbackClocksRetainTheirOriginalBehavior();
        std::cout << "disperse swing stage: 14 test groups passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
