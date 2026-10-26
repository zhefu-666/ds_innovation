#include "rescue/task/release_retreat_stage.hpp"
#include "rescue/task/push_task.hpp"

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace rescue;

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
    }
}
#define CHECK(expression) check((expression), #expression, __LINE__)

struct Fixture {
    ReleaseRetreatStage stage;
    TaskTuning tuning;
    ReleaseRetreatContext context{0, 0, true, false, true, .5f, .25f, 4, false, 0, 0};
    ReleasedCargoEvidence cargo;

    ReleaseRetreatResult update(uint64_t elapsed_us = 1) {
        context.elapsed_us = elapsed_us;
        return stage.update(tuning, context, cargo);
    }

    void begin(float hold_y = .5f) {
        stage.beginReleaseBackOff(hold_y);
        context.hold_y = hold_y;
    }

    void inside(uint64_t timestamp) {
        cargo.visible = cargo.fixed = true;
        cargo.clipped = false;
        cargo.rear_x = context.half_x;
        cargo.rear_y = tuning.assume_cargo_done_y_m;
        cargo.timestamp_us = timestamp;
    }
};

void expect(const ReleaseRetreatResult& result,
            ReleaseRetreatOutcome outcome,
            const std::string& reason = "",
            bool reverse = false) {
    CHECK(result.outcome == outcome);
    CHECK(result.reason == reason);
    CHECK(result.reverse == reverse);
}

void strictRetreatUsesObservedMouthClearance() {
    Fixture missing;
    missing.context.observed_zone = false;
    expect(missing.update(missing.tuning.retreat_budget_us), ReleaseRetreatOutcome::Continue);
    CHECK(!missing.update(missing.tuning.retreat_budget_us).blind_back_out);
    expect(missing.update(missing.tuning.retreat_budget_us + 1),
           ReleaseRetreatOutcome::Fault,
           "retreat_reference_lost");

    Fixture blocked;
    expect(blocked.update(blocked.tuning.retreat_budget_us),
           ReleaseRetreatOutcome::Continue,
           "",
           true);
    expect(blocked.update(blocked.tuning.retreat_budget_us + 1),
           ReleaseRetreatOutcome::Fault,
           "retreat_blocked");

    Fixture clear;
    clear.context.mouth_clear = true;
    expect(clear.update(clear.tuning.retreat_budget_us + 1),
           ReleaseRetreatOutcome::VerifyDelivery,
           "backed_out");

    Fixture predicted;
    predicted.context.observed_zone = false;
    predicted.context.anchored_zone = true;
    predicted.context.mouth_clear = true;
    predicted.context.travel_m = 100;
    expect(predicted.update(), ReleaseRetreatOutcome::Continue);
    CHECK(!predicted.update().blind_back_out);
}

void carryModesAllowBoundedBlindRetreat() {
    for (const bool assume_all_safe : {false, true}) {
        Fixture fixture;
        fixture.tuning.field_carry = !assume_all_safe;
        fixture.tuning.assume_all_safe = assume_all_safe;
        fixture.context.observed_zone = false;
        fixture.context.travel_m = fixture.tuning.assume_back_m - .001f;
        const auto moving = fixture.update(fixture.tuning.retreat_budget_us);
        expect(moving, ReleaseRetreatOutcome::Continue, "assumed_blind_back_out", true);
        CHECK(moving.blind_back_out);
        fixture.context.travel_m = fixture.tuning.assume_back_m;
        expect(fixture.update(fixture.tuning.retreat_budget_us + 1),
               ReleaseRetreatOutcome::VerifyDelivery,
               "assumed_backed_out");

        Fixture timeout;
        timeout.tuning.field_carry = !assume_all_safe;
        timeout.tuning.assume_all_safe = assume_all_safe;
        timeout.context.observed_zone = false;
        expect(timeout.update(timeout.tuning.retreat_budget_us + 1),
               ReleaseRetreatOutcome::Fault,
               "retreat_blocked");
    }

    Fixture rush_only;
    rush_only.tuning.field_rush = true;
    rush_only.context.observed_zone = false;
    expect(rush_only.update(), ReleaseRetreatOutcome::Continue);
    CHECK(!rush_only.update().blind_back_out);

    Fixture no_scaling;
    no_scaling.tuning.field_carry = true;
    no_scaling.context.observed_zone = false;
    no_scaling.context.motion_ratio = 100;
    no_scaling.context.travel_m = no_scaling.tuning.assume_back_m;
    expect(no_scaling.update(), ReleaseRetreatOutcome::VerifyDelivery, "assumed_backed_out");
}

void blindRetreatRemainsBlindAfterAnObservationReturns() {
    Fixture fixture;
    fixture.tuning.field_carry = true;
    fixture.context.observed_zone = true;
    fixture.context.mouth_clear = true;
    fixture.context.assume_blind = true;
    const auto moving = fixture.update();
    expect(moving, ReleaseRetreatOutcome::Continue, "assumed_blind_back_out", true);
    CHECK(moving.blind_back_out);
    fixture.context.travel_m = fixture.tuning.assume_back_m;
    expect(fixture.update(), ReleaseRetreatOutcome::VerifyDelivery, "assumed_backed_out");

    Fixture observed;
    observed.tuning.field_carry = true;
    observed.context.mouth_clear = true;
    expect(observed.update(), ReleaseRetreatOutcome::VerifyDelivery, "backed_out");
    CHECK(!observed.update().blind_back_out);

    Fixture strict;
    strict.context.assume_blind = true;
    strict.context.mouth_clear = true;
    expect(strict.update(), ReleaseRetreatOutcome::VerifyDelivery, "backed_out");
}

void prepushUsesAnchoredDisplacement() {
    Fixture fixture;
    fixture.begin();
    CHECK(fixture.stage.needsCargoTracking());
    fixture.context.hold_y = .2501f;
    fixture.context.travel_m = 100;
    expect(fixture.update(), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
    fixture.context.hold_y = .25f;
    expect(fixture.update(), ReleaseRetreatOutcome::PushIn, "assumed_release_then_push");
    CHECK(!fixture.stage.needsCargoTracking());

    Fixture predicted_anchor;
    predicted_anchor.begin();
    predicted_anchor.context.observed_zone = false;
    predicted_anchor.context.hold_y = .25f;
    expect(predicted_anchor.update(), ReleaseRetreatOutcome::PushIn, "assumed_release_then_push");

    Fixture not_backing_away;
    not_backing_away.begin();
    not_backing_away.context.hold_y = .75f;
    expect(not_backing_away.update(),
           ReleaseRetreatOutcome::Continue,
           "assumed_release_back_off",
           true);
}

void prepushBlindDistanceUsesMotionRatio() {
    Fixture fixture;
    fixture.begin();
    fixture.context.anchored_zone = false;
    fixture.context.hold_y = -100;
    fixture.context.travel_m = .999f;
    const auto moving = fixture.update();
    expect(moving, ReleaseRetreatOutcome::Continue, "assumed_release_back_off_blind", true);
    CHECK(!moving.blind_back_out);
    fixture.context.travel_m = 1;
    expect(fixture.update(), ReleaseRetreatOutcome::PushIn, "assumed_release_then_push");

    Fixture custom_ratio;
    custom_ratio.begin();
    custom_ratio.context.anchored_zone = false;
    custom_ratio.context.back_target_m = .5f;
    custom_ratio.context.motion_ratio = 2;
    custom_ratio.context.travel_m = 1;
    expect(custom_ratio.update(), ReleaseRetreatOutcome::PushIn, "assumed_release_then_push");
}

void prepushTimeoutProceedsToPushIn() {
    for (const bool anchored : {false, true}) {
        Fixture fixture;
        fixture.begin();
        fixture.context.anchored_zone = anchored;
        expect(fixture.update(fixture.tuning.retreat_budget_us),
               ReleaseRetreatOutcome::Continue,
               anchored ? "assumed_release_back_off" : "assumed_release_back_off_blind",
               true);
        expect(fixture.update(fixture.tuning.retreat_budget_us + 1),
               ReleaseRetreatOutcome::PushIn,
               "assumed_release_then_push");
        CHECK(!fixture.stage.needsCargoTracking());
    }
}

void insideShortcutRequiresIndependentTimestamps() {
    Fixture fixture;
    fixture.begin();
    fixture.inside(10);
    expect(fixture.update(), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
    expect(fixture.update(), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
    fixture.inside(11);
    expect(fixture.update(), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
    fixture.inside(12);
    expect(fixture.update(),
           ReleaseRetreatOutcome::InsideAfterRelease,
           "assumed_inside_after_release");
    CHECK(!fixture.stage.needsCargoTracking());

    Fixture timestamp_zero;
    timestamp_zero.begin();
    timestamp_zero.inside(0);
    expect(
        timestamp_zero.update(), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
    for (const uint64_t timestamp : {1ULL, 2ULL}) {
        timestamp_zero.inside(timestamp);
        expect(timestamp_zero.update(),
               ReleaseRetreatOutcome::Continue,
               "assumed_release_back_off",
               true);
    }
    timestamp_zero.inside(3);
    expect(timestamp_zero.update(),
           ReleaseRetreatOutcome::InsideAfterRelease,
           "assumed_inside_after_release");

    Fixture different_not_monotonic;
    different_not_monotonic.begin();
    different_not_monotonic.inside(10);
    expect(different_not_monotonic.update(),
           ReleaseRetreatOutcome::Continue,
           "assumed_release_back_off",
           true);
    different_not_monotonic.inside(11);
    expect(different_not_monotonic.update(),
           ReleaseRetreatOutcome::Continue,
           "assumed_release_back_off",
           true);
    different_not_monotonic.inside(10);
    expect(different_not_monotonic.update(),
           ReleaseRetreatOutcome::InsideAfterRelease,
           "assumed_inside_after_release");
}

void insideConditionsAndBrokenEvidenceResetTheCount() {
    for (int invalid_gate = 0; invalid_gate < 6; ++invalid_gate) {
        Fixture fixture;
        fixture.begin();
        for (const uint64_t timestamp : {10ULL, 11ULL}) {
            fixture.inside(timestamp);
            expect(fixture.update(),
                   ReleaseRetreatOutcome::Continue,
                   "assumed_release_back_off",
                   true);
        }
        fixture.inside(12);
        switch (invalid_gate) {
            case 0:
                fixture.cargo.visible = false;
                break;
            case 1:
                fixture.cargo.fixed = false;
                break;
            case 2:
                fixture.cargo.clipped = true;
                break;
            case 3:
                fixture.cargo.rear_y = fixture.tuning.assume_cargo_done_y_m - .001f;
                break;
            case 4:
                fixture.cargo.rear_x = fixture.tuning.assume_cargo_x_tol_m + .001f;
                break;
            case 5:
                fixture.context.repush_count = 1;
                break;
        }
        expect(fixture.update(), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
        fixture.context.repush_count = 0;
        // Invalid evidence clears the count but retains the last accepted timestamp.
        for (const uint64_t timestamp : {11ULL, 13ULL, 14ULL}) {
            fixture.inside(timestamp);
            expect(fixture.update(),
                   ReleaseRetreatOutcome::Continue,
                   "assumed_release_back_off",
                   true);
        }
        fixture.inside(15);
        expect(fixture.update(),
               ReleaseRetreatOutcome::InsideAfterRelease,
               "assumed_inside_after_release");
    }

    Fixture inclusive;
    inclusive.begin();
    inclusive.tuning.assume_inside_after_release_obs = 1;
    inclusive.inside(1);
    inclusive.cargo.rear_x = inclusive.tuning.assume_cargo_x_tol_m;
    expect(inclusive.update(),
           ReleaseRetreatOutcome::InsideAfterRelease,
           "assumed_inside_after_release");
}

void insideShortcutPrecedesDistanceAndTimeout() {
    Fixture fixture;
    fixture.begin();
    for (const uint64_t timestamp : {1ULL, 2ULL}) {
        fixture.inside(timestamp);
        expect(fixture.update(), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
    }
    fixture.inside(3);
    fixture.context.hold_y = .25f;
    expect(fixture.update(fixture.tuning.retreat_budget_us + 1),
           ReleaseRetreatOutcome::InsideAfterRelease,
           "assumed_inside_after_release");

    Fixture retry;
    retry.begin();
    retry.context.repush_count = 1;
    for (uint64_t timestamp = 1; timestamp <= 4; ++timestamp) {
        retry.inside(timestamp);
        expect(retry.update(), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
    }
    retry.context.hold_y = .25f;
    expect(retry.update(), ReleaseRetreatOutcome::PushIn, "assumed_release_then_push");
}

void releaseBackOffResetsCountButRetainsTimestamp() {
    Fixture fixture;
    fixture.begin();
    for (const uint64_t timestamp : {10ULL, 11ULL}) {
        fixture.inside(timestamp);
        expect(fixture.update(), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
    }
    fixture.begin(.75f);
    for (const uint64_t timestamp : {11ULL, 12ULL, 13ULL}) {
        fixture.inside(timestamp);
        expect(fixture.update(), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
    }
    fixture.inside(14);
    expect(fixture.update(),
           ReleaseRetreatOutcome::InsideAfterRelease,
           "assumed_inside_after_release");
}

void repushBackOffRetainsHistoryAndUpdatesTheOrigin() {
    Fixture fixture;
    fixture.begin();
    for (const uint64_t timestamp : {10ULL, 11ULL}) {
        fixture.inside(timestamp);
        expect(fixture.update(), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
    }
    fixture.stage.beginRepushBackOff(.75f);
    fixture.context.hold_y = .75f;
    fixture.inside(11);
    expect(fixture.update(), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
    fixture.inside(12);
    expect(fixture.update(),
           ReleaseRetreatOutcome::InsideAfterRelease,
           "assumed_inside_after_release");

    Fixture new_origin;
    new_origin.begin();
    new_origin.stage.beginRepushBackOff(.75f);
    new_origin.context.hold_y = .5f;
    expect(new_origin.update(), ReleaseRetreatOutcome::PushIn, "assumed_release_then_push");
}

void clearTripDiscardsPrepushHistory() {
    Fixture fixture;
    fixture.begin();
    for (const uint64_t timestamp : {10ULL, 11ULL}) {
        fixture.inside(timestamp);
        expect(fixture.update(), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
    }
    fixture.stage.clearTrip();
    CHECK(!fixture.stage.needsCargoTracking());
    fixture.context.mouth_clear = true;
    expect(fixture.update(), ReleaseRetreatOutcome::VerifyDelivery, "backed_out");
    fixture.context.mouth_clear = false;
    fixture.stage.beginRepushBackOff(.75f);
    fixture.context.hold_y = .75f;
    for (const uint64_t timestamp : {11ULL, 12ULL}) {
        fixture.inside(timestamp);
        expect(fixture.update(), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
    }
    fixture.inside(13);
    expect(fixture.update(),
           ReleaseRetreatOutcome::InsideAfterRelease,
           "assumed_inside_after_release");
}

void pauseKeepsPrepushHistoryWhileCallerResetsElapsedAndTravel() {
    Fixture fixture;
    fixture.begin(.75f);
    for (const uint64_t timestamp : {10ULL, 11ULL}) {
        fixture.inside(timestamp);
        expect(fixture.update(5000000),
               ReleaseRetreatOutcome::Continue,
               "assumed_release_back_off",
               true);
    }
    fixture.context.travel_m = 0;
    fixture.inside(11);
    expect(fixture.update(1), ReleaseRetreatOutcome::Continue, "assumed_release_back_off", true);
    fixture.inside(12);
    expect(fixture.update(2),
           ReleaseRetreatOutcome::InsideAfterRelease,
           "assumed_inside_after_release");
}
} // namespace

int main() {
    try {
        strictRetreatUsesObservedMouthClearance();
        carryModesAllowBoundedBlindRetreat();
        blindRetreatRemainsBlindAfterAnObservationReturns();
        prepushUsesAnchoredDisplacement();
        prepushBlindDistanceUsesMotionRatio();
        prepushTimeoutProceedsToPushIn();
        insideShortcutRequiresIndependentTimestamps();
        insideConditionsAndBrokenEvidenceResetTheCount();
        insideShortcutPrecedesDistanceAndTimeout();
        releaseBackOffResetsCountButRetainsTimestamp();
        repushBackOffRetainsHistoryAndUpdatesTheOrigin();
        clearTripDiscardsPrepushHistory();
        pauseKeepsPrepushHistoryWhileCallerResetsElapsedAndTravel();
        std::cout << "release stage tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "release stage tests failed: " << error.what() << '\n';
        return 1;
    }
}
