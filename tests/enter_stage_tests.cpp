#include "rescue/task/enter_stage.hpp"
#include "rescue/push_task.hpp"
#include "rescue/zone_estimate.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

using namespace rescue;

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
    }
}
#define CHECK(expression) check((expression), #expression, __LINE__)

struct Fixture {
    EnterStage stage;
    TaskTuning tuning;
    EnterContext context;

    Fixture() {
        context.zone_ok = true;
        context.hold = {0, 0};
        context.half_x = 0;
    }

    EnterResult update() {
        return stage.update(tuning, context);
    }
};

void expect(const EnterResult& result, EnterOutcome outcome, const char* reason) {
    CHECK(result.outcome == outcome);
    if (reason == nullptr)
        CHECK(result.reason == nullptr);
    else
        CHECK(result.reason != nullptr && std::string(result.reason) == reason);
}

void drivesStraightWhenAligned() {
    Fixture f;
    const auto result = f.update();
    expect(result, EnterOutcome::Continue, nullptr);
    CHECK(result.drive);
    CHECK(result.vx == f.tuning.enter_speed);
    CHECK(result.wz == 0);
    CHECK(result.reset_misses);
}

void steerCombinesHeadingAndLateralGain() {
    Fixture f;
    f.context.zone_yaw = .1f;
    f.context.hold = {.02f, 0};
    const auto result = f.update();
    expect(result, EnterOutcome::Continue, nullptr);
    CHECK(result.drive);
    CHECK(std::abs(result.wz - (f.tuning.heading_gain * .1f + f.tuning.lateral_gain * .02f)) <
          1e-6f);
}

void wrapsHeadingError() {
    Fixture f;
    f.context.zone_yaw = 6.2f; // just short of a full turn: the shortest arc turns slightly
    const auto result = f.update();
    CHECK(result.drive);
    CHECK(std::abs(result.wz - f.tuning.heading_gain * wrapAngle(6.2f)) < 1e-6f);
    CHECK(std::abs(result.wz) < f.tuning.heading_gain);
}

void depositLineUsesTheTuningTargetWhenUnlocked() {
    Fixture f;
    f.context.hold = {0, f.tuning.deposit_y_m - .001f};
    expect(f.update(), EnterOutcome::Continue, nullptr);
    f.context.hold = {0, f.tuning.deposit_y_m}; // the comparison is an inclusive >=
    expect(f.update(), EnterOutcome::RaiseRelease, "enclosed_load_at_drop");
}

void lockedDropCentreOverridesTheTuningTarget() {
    Fixture f;
    f.context.drop_locked = true;
    f.context.drop_centre_y = .4f; // further in than the tuning target
    f.context.hold = {0, f.tuning.deposit_y_m + .01f};
    expect(f.update(), EnterOutcome::Continue, nullptr);
    f.context.hold = {0, .4f};
    expect(f.update(), EnterOutcome::RaiseRelease, "enclosed_load_at_drop");
}

void lateralLimitAdvancesTheHalfX() {
    Fixture f;
    f.context.half_x = .25f;
    f.context.hold = {.25f + f.tuning.enter_lateral_m - .001f, 0};
    expect(f.update(), EnterOutcome::Continue, nullptr);
    f.context.hold = {.25f + f.tuning.enter_lateral_m, 0}; // strict > on the absolute lateral
    expect(f.update(), EnterOutcome::Continue, nullptr);
    f.context.hold = {.25f + f.tuning.enter_lateral_m + .001f, 0};
    expect(f.update(), EnterOutcome::AbortDrop, "enter_lateral");
}

void negativeLateralUsesTheAbsoluteValue() {
    Fixture f;
    f.context.half_x = .25f;
    f.context.hold = {.25f - f.tuning.enter_lateral_m - .001f, 0};
    expect(f.update(), EnterOutcome::AbortDrop, "enter_lateral");
}

void budgetBoundaryIsStrict() {
    Fixture f;
    f.context.elapsed_us = f.tuning.enter_budget_us;
    expect(f.update(), EnterOutcome::Continue, nullptr);
    f.context.elapsed_us = f.tuning.enter_budget_us + 1;
    expect(f.update(), EnterOutcome::AbortDrop, "enter_timeout");
}

void lostZoneWaitsForTheMissedFrameCount() {
    Fixture f;
    f.context.zone_ok = false;
    f.context.lost_target = false;
    const auto result = f.update();
    expect(result, EnterOutcome::Continue, nullptr);
    CHECK(!result.drive);
    CHECK(!result.reset_misses); // an unusable zone never restarts the miss count
}

void lostZoneEndsTheTrip() {
    Fixture f;
    f.context.zone_ok = false;
    f.context.lost_target = true;
    expect(f.update(), EnterOutcome::AbortDrop, "zone_lost");
    f.context.carry = true;
    expect(f.update(), EnterOutcome::RaiseRelease, "assumed_drop_point_zone_lost");
}

void depositCheckedBeforeLateralAndBudget() {
    Fixture entrance;
    entrance.context.hold = {0, entrance.tuning.deposit_y_m};
    entrance.context.half_x = 0;
    entrance.context.hold.x = entrance.tuning.enter_lateral_m * 2; // also sideways
    entrance.context.elapsed_us = entrance.tuning.enter_budget_us * 2;
    expect(entrance.update(), EnterOutcome::RaiseRelease, "enclosed_load_at_drop");

    Fixture sideways;
    sideways.context.hold = {0, 0};
    sideways.context.half_x = -sideways.tuning.enter_lateral_m * 2;
    sideways.context.elapsed_us = sideways.tuning.enter_budget_us * 2;
    expect(sideways.update(), EnterOutcome::AbortDrop, "enter_lateral");
}

void noMotionWhenAnOutcomeEndsTheState() {
    Fixture f;
    f.context.hold = {0, f.tuning.deposit_y_m};
    const auto result = f.update();
    expect(result, EnterOutcome::RaiseRelease, "enclosed_load_at_drop");
    CHECK(!result.drive);
    CHECK(result.vx == 0 && result.wz == 0);
}
} // namespace

int main() {
    const std::pair<const char*, void (*)()> tests[] = {
        {"drivesStraightWhenAligned", drivesStraightWhenAligned},
        {"steerCombinesHeadingAndLateralGain", steerCombinesHeadingAndLateralGain},
        {"wrapsHeadingError", wrapsHeadingError},
        {"depositLineUsesTheTuningTargetWhenUnlocked", depositLineUsesTheTuningTargetWhenUnlocked},
        {"lockedDropCentreOverridesTheTuningTarget", lockedDropCentreOverridesTheTuningTarget},
        {"lateralLimitAdvancesTheHalfX", lateralLimitAdvancesTheHalfX},
        {"negativeLateralUsesTheAbsoluteValue", negativeLateralUsesTheAbsoluteValue},
        {"budgetBoundaryIsStrict", budgetBoundaryIsStrict},
        {"lostZoneWaitsForTheMissedFrameCount", lostZoneWaitsForTheMissedFrameCount},
        {"lostZoneEndsTheTrip", lostZoneEndsTheTrip},
        {"depositCheckedBeforeLateralAndBudget", depositCheckedBeforeLateralAndBudget},
        {"noMotionWhenAnOutcomeEndsTheState", noMotionWhenAnOutcomeEndsTheState},
    };
    for (const auto& [name, run] : tests) {
        try {
            run();
        } catch (const std::exception& e) {
            std::cout << "FAIL " << name << ": " << e.what() << "\n";
            return 1;
        }
    }
    std::cout << "enter stage OK\n";
    return 0;
}
