#include "rescue/task/recovery_stage.hpp"
#include "rescue/push_task.hpp"

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
    RecoveryStage stage;
    TaskTuning tuning;
    RecoveryContext context;

    Fixture() {
        context.now_us = 1000000;
        context.motion_ratio = 1;
        context.heading_valid = true;
        context.approach_forward_m = 1;
        context.retreat_safe = true;
    }

    RecoveryResult update() {
        return stage.update(tuning, context);
    }

    void observe(float origin_y = 0, const char* geometry = "g") {
        context.observed_zone = true;
        context.zone_origin = {0, origin_y};
        context.zone_forward = {0, 1};
        context.zone_geometry_id = geometry;
    }
};

void expect(const RecoveryResult& result,
            RecoveryOutcome outcome,
            const std::string& reason = "",
            bool reverse = false) {
    CHECK(result.outcome == outcome);
    CHECK(result.reason == reason);
    CHECK((result.outcome == RecoveryOutcome::Reverse) == reverse);
}

void measuredRetreatStopsAtAbortDistance() {
    Fixture f;
    f.observe();
    expect(f.update(), RecoveryOutcome::Reverse, "", true);
    f.context.now_us += 1000;
    f.context.observed_zone = false;
    expect(f.update(), RecoveryOutcome::Continue);

    f.observe(-f.tuning.abort_back_m);
    const auto result = f.update();
    expect(result, RecoveryOutcome::Scan, "");
    CHECK(result.blacklist);
    CHECK(result.vx == 0);
}

void measuredRetreatCorridorUsesCorridorDistance() {
    Fixture corridor;
    corridor.context.corridor_retreat = true;
    corridor.tuning.corridor_back_m = .10f;
    corridor.observe();
    corridor.update();
    corridor.observe(-corridor.tuning.corridor_back_m);
    const auto short_of_abort = corridor.update();
    expect(short_of_abort, RecoveryOutcome::Scan, "");
    CHECK(short_of_abort.blacklist);

    Fixture plain;
    plain.observe();
    plain.update();
    plain.observe(-.10f);
    expect(plain.update(), RecoveryOutcome::Reverse, "", true);
}

void lostHoldKeepsTheTargetUnblacklisted() {
    Fixture f;
    f.context.lost_hold = true;
    f.observe();
    f.update();
    f.observe(-f.tuning.abort_back_m);
    const auto result = f.update();
    expect(result, RecoveryOutcome::Scan, "");
    CHECK(!result.blacklist);
}

void measuredReverseKeepsTheExistingReason() {
    Fixture f;
    f.observe();
    f.update();
    f.observe(-f.tuning.abort_back_m * .5f);
    const auto result = f.update();
    expect(result, RecoveryOutcome::Reverse, "", true);
    CHECK(result.vx == -f.tuning.back_speed);
}

void geometryChangeDiscardsTheMeasuredReference() {
    Fixture f;
    f.observe();
    f.update();
    f.observe(-f.tuning.abort_back_m, "other");
    expect(f.update(), RecoveryOutcome::Continue);
}

void blindFallbackNeedsAHeading() {
    Fixture f;
    f.context.heading_valid = false;
    f.context.since_step_us = f.tuning.blind_retreat_wait_us;
    expect(f.update(), RecoveryOutcome::Fault, "retreat_unverified_no_heading");
}

void blindFallbackWaitsForTheBlindWindow() {
    Fixture f;
    f.context.approach_forward_m = 1;
    f.context.since_step_us = f.tuning.blind_retreat_wait_us - 1;
    expect(f.update(), RecoveryOutcome::Continue);
    f.context.since_step_us = f.tuning.blind_retreat_wait_us;
    const auto result = f.update();
    expect(result, RecoveryOutcome::Reverse, "blind_retreat", true);
    CHECK(result.reset_travel);
}

void corridorBlindWindowIsCapped() {
    Fixture f;
    f.context.corridor_retreat = true;
    f.context.since_step_us = 200000;
    const auto result = f.update();
    expect(result, RecoveryOutcome::Reverse, "blind_retreat", true);
    CHECK(result.reset_travel);

    Fixture early;
    early.context.corridor_retreat = true;
    early.context.since_step_us = 199999;
    expect(early.update(), RecoveryOutcome::Continue);
}

void blindRetreatStopsAtTheApproachBound() {
    Fixture f;
    f.context.since_step_us = f.tuning.blind_retreat_wait_us;
    f.update();
    f.context.travel_m = f.tuning.abort_back_m;
    const auto result = f.update();
    expect(result, RecoveryOutcome::Scan, "blind_retreat_complete");
    CHECK(result.blacklist);
}

void corridorBlindRetreatNeedsSafety() {
    Fixture f;
    f.context.corridor_retreat = true;
    f.context.since_step_us = 200000;
    f.update();
    f.context.retreat_safe = false;
    const auto result = f.update();
    expect(result, RecoveryOutcome::Scan, "corridor_back_off_unsafe");
    CHECK(!result.blacklist);
}

void blindRetreatChecksHeadingAndBudget() {
    Fixture drift;
    drift.context.since_step_us = drift.tuning.blind_retreat_wait_us;
    drift.update();
    drift.context.heading_rad = drift.tuning.blind_retreat_yaw_rad + .01f;
    expect(drift.update(), RecoveryOutcome::Fault, "blind_retreat_heading_drift");

    Fixture budget;
    budget.context.since_step_us = budget.tuning.blind_retreat_wait_us;
    budget.update();
    budget.context.elapsed_us = budget.tuning.retreat_budget_us + 1;
    expect(budget.update(), RecoveryOutcome::Fault, "retreat_unverified");

    Fixture unverified;
    unverified.context.since_step_us = unverified.tuning.blind_retreat_wait_us;
    unverified.update();
    unverified.context.heading_valid = false;
    expect(unverified.update(), RecoveryOutcome::Fault, "blind_retreat_heading_missing");
}

void resetRestartsTheDecision() {
    Fixture f;
    f.observe();
    f.update();
    f.observe(-f.tuning.abort_back_m);
    expect(f.update(), RecoveryOutcome::Scan, "");
    f.stage.reset();
    f.observe();
    expect(f.update(), RecoveryOutcome::Reverse, "", true);
}
} // namespace

int main() {
    const std::pair<const char*, void (*)()> tests[] = {
        {"measuredRetreatStopsAtAbortDistance", measuredRetreatStopsAtAbortDistance},
        {"measuredRetreatCorridorUsesCorridorDistance",
         measuredRetreatCorridorUsesCorridorDistance},
        {"lostHoldKeepsTheTargetUnblacklisted", lostHoldKeepsTheTargetUnblacklisted},
        {"measuredReverseKeepsTheExistingReason", measuredReverseKeepsTheExistingReason},
        {"geometryChangeDiscardsTheMeasuredReference", geometryChangeDiscardsTheMeasuredReference},
        {"blindFallbackNeedsAHeading", blindFallbackNeedsAHeading},
        {"blindFallbackWaitsForTheBlindWindow", blindFallbackWaitsForTheBlindWindow},
        {"corridorBlindWindowIsCapped", corridorBlindWindowIsCapped},
        {"blindRetreatStopsAtTheApproachBound", blindRetreatStopsAtTheApproachBound},
        {"corridorBlindRetreatNeedsSafety", corridorBlindRetreatNeedsSafety},
        {"blindRetreatChecksHeadingAndBudget", blindRetreatChecksHeadingAndBudget},
        {"resetRestartsTheDecision", resetRestartsTheDecision},
    };
    for (const auto& [name, run] : tests) {
        try {
            run();
        } catch (const std::exception& e) {
            std::cout << "FAIL " << name << ": " << e.what() << "\n";
            return 1;
        }
    }
    std::cout << "recovery stage OK\n";
    return 0;
}
