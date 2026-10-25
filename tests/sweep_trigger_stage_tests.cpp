#include "rescue/task/sweep_trigger_stage.hpp"
#include "rescue/task/push_task.hpp"

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
    SweepTriggerPlanner planner;
    TaskTuning tuning;
    SweepTriggerContext context;
    uint64_t clutter_ts_us = 0;

    Fixture() {
        tuning.blind_scatter = true;
        tuning.disperse = true;
        tuning.blind_scatter_max = 4;
        tuning.blind_scatter_approach_max = 3;
        tuning.blind_scatter_max_y_m = 2.f;
        tuning.blind_scatter_far_max_y_m = 4.f;
        tuning.sweep_fresh_us = 500000;
        tuning.dense_multi_ignore_gap = false;
        context.now_us = 1000000;
        context.scan_ok = true;
        context.forward_ok = true;
        context.heading_valid = true;
        context.heading_rad = 0;
        context.ledger_ready = true;
        context.budget_ok = true;
        clutter_ts_us = 1000000;
    }

    SweepTriggerGate eval() {
        return planner.evaluate(tuning, context, clutter_ts_us);
    }
};

void allowsAScatterWhenEverythingIsReady() {
    Fixture f;
    CHECK(f.eval().scatter_ok);
    CHECK(f.eval().scatter_multi_ok);
}

void refusesWithoutBlindScatter() {
    Fixture f;
    f.tuning.blind_scatter = false;
    CHECK(!f.eval().scatter_ok);
    CHECK(!f.eval().scatter_multi_ok);
}

void refusesWithoutMotionPermission() {
    Fixture f;
    f.context.scan_ok = false;
    CHECK(!f.eval().scatter_ok);
    f.context.scan_ok = true;
    f.context.forward_ok = false;
    CHECK(!f.eval().scatter_ok);
}

void refusesWithoutAHeading() {
    Fixture f;
    f.context.heading_valid = false;
    CHECK(!f.eval().scatter_ok);
    f.context.heading_valid = true;
    f.context.heading_rad = std::nanf("");
    CHECK(!f.eval().scatter_ok);
}

void countLimitIsStrict() {
    Fixture f;
    f.context.scatter_count = f.tuning.blind_scatter_max - 1;
    CHECK(f.eval().scatter_ok);
    f.context.scatter_count = f.tuning.blind_scatter_max;
    CHECK(!f.eval().scatter_ok);
}

void coolingBlocksTheTrigger() {
    Fixture f;
    f.context.scatter_cool_until_us = f.context.now_us + 1;
    CHECK(!f.eval().scatter_ok);
    f.context.scatter_cool_until_us = f.context.now_us; // the check is an inclusive >=
    CHECK(f.eval().scatter_ok);
}

void ledgerReadinessGatesThePlainTrigger() {
    Fixture f;
    f.context.ledger_ready = false;
    CHECK(!f.eval().scatter_ok);
    // The multi trigger falls back to the ledger unless dense_multi_ignore_gap is set.
    CHECK(!f.eval().scatter_multi_ok);
    f.tuning.dense_multi_ignore_gap = true;
    CHECK(f.eval().scatter_multi_ok);
}

void farWindowShrinksWhenApproachesAreExhausted() {
    Fixture f;
    f.context.scatter_approach_n = 0;
    CHECK(f.eval().approach_ok);
    CHECK(f.eval().far_y ==
          std::max(f.tuning.blind_scatter_max_y_m, f.tuning.blind_scatter_far_max_y_m));
    f.context.scatter_approach_n = f.tuning.blind_scatter_approach_max;
    CHECK(!f.eval().approach_ok);
    CHECK(f.eval().far_y == f.tuning.blind_scatter_max_y_m);
}

void clutterFreshnessNeedsATimestamp() {
    Fixture f;
    CHECK(f.eval().clutter_fresh);
    f.clutter_ts_us = 0;
    CHECK(!f.eval().clutter_fresh);
}

void clutterFreshnessExpires() {
    Fixture f;
    f.clutter_ts_us = f.context.now_us;
    CHECK(f.eval().clutter_fresh);
    f.clutter_ts_us = f.context.now_us - f.tuning.sweep_fresh_us; // inclusive window
    CHECK(f.eval().clutter_fresh);
    f.clutter_ts_us = f.context.now_us - f.tuning.sweep_fresh_us - 1;
    CHECK(!f.eval().clutter_fresh);
}

void futureClutterTimestampIsNotFresh() {
    Fixture f;
    f.clutter_ts_us = f.context.now_us + 1;
    CHECK(!f.eval().clutter_fresh);
}
} // namespace

int main() {
    const std::pair<const char*, void (*)()> tests[] = {
        {"allowsAScatterWhenEverythingIsReady", allowsAScatterWhenEverythingIsReady},
        {"refusesWithoutBlindScatter", refusesWithoutBlindScatter},
        {"refusesWithoutMotionPermission", refusesWithoutMotionPermission},
        {"refusesWithoutAHeading", refusesWithoutAHeading},
        {"countLimitIsStrict", countLimitIsStrict},
        {"coolingBlocksTheTrigger", coolingBlocksTheTrigger},
        {"ledgerReadinessGatesThePlainTrigger", ledgerReadinessGatesThePlainTrigger},
        {"farWindowShrinksWhenApproachesAreExhausted", farWindowShrinksWhenApproachesAreExhausted},
        {"clutterFreshnessNeedsATimestamp", clutterFreshnessNeedsATimestamp},
        {"clutterFreshnessExpires", clutterFreshnessExpires},
        {"futureClutterTimestampIsNotFresh", futureClutterTimestampIsNotFresh},
    };
    for (const auto& [name, run] : tests) {
        try {
            run();
        } catch (const std::exception& e) {
            std::cout << "FAIL " << name << ": " << e.what() << "\n";
            return 1;
        }
    }
    std::cout << "sweep trigger planner OK\n";
    return 0;
}
