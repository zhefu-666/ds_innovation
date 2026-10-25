#include "rescue/task/delivery_verification_stage.hpp"
#include "rescue/push_task.hpp"

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

Inventory inventory(int ordinary, int core = 0, int injured = 0) {
    Inventory value;
    value.ordinary = ordinary;
    value.core = core;
    value.injured = injured;
    return value;
}

struct Fixture {
    DeliveryVerificationStage stage;
    TaskTuning tuning;
    PushObservation observation;
    DeliveryCargoEvidence cargo;
    Inventory trip = inventory(1);
    int baseline = 3, baseline_other = 1, repush_count = 0;
    std::string geometry_id = "delivery-test-zone";
    bool observed_zone = true, half_injured = false, tracked_cargo_has_fix = true;
    float half_x = -.15f;
    uint64_t clock_base = 10000000;

    Fixture() {
        tuning.delivery_frames = 3;
        observation.zone_estimate.geometry_id = geometry_id;
        observation.zone_estimate.origin_body_m = {0, .5f};
        observation.zone_inventory_complete = observation.zone_counts_valid = true;
        observation.zone_supply_count = baseline + 1;
        observation.zone_injured_count = baseline_other;
        observation.gripper_done = true;
        observation.gripper_feedback_open = 1;
    }

    DeliveryVerificationResult
    update(uint64_t elapsed_us, uint64_t frame_id = 1, uint64_t cargo_timestamp = 0) {
        observation.now_us = clock_base + elapsed_us;
        observation.zone_estimate.frame_id = frame_id;
        if (cargo_timestamp)
            observation.cargo_ts_us = cargo_timestamp;
        return stage.update(observation,
                            tuning,
                            {elapsed_us,
                             trip,
                             baseline,
                             baseline_other,
                             geometry_id,
                             observed_zone,
                             half_x,
                             half_injured,
                             tracked_cargo_has_fix,
                             repush_count},
                            cargo);
    }

    void field(bool assume = false) {
        tuning.field_carry = !assume;
        tuning.assume_all_safe = assume;
        observed_zone = false;
    }

    void fixedCargo(float y = .02f) {
        cargo.available = true;
        cargo.rear = {half_x, y};
        cargo.fixed = true;
    }

    void imageCargo(float fraction = .5f) {
        cargo.available = true;
        cargo.rear = {half_x, .02f};
        cargo.fixed = false;
        cargo.front_valid = true;
        cargo.height_px = 100;
        cargo.bottom_px = 400;
        cargo.front_px = cargo.bottom_px + fraction * cargo.height_px;
    }

    void fix(int points = 3) {
        observation.vref_valid = observation.vref_zone.valid = true;
        observation.vref_points = points;
    }
};

void expect(const DeliveryVerificationResult& result,
            DeliveryVerificationOutcome outcome,
            int credited = 0,
            bool visual = false,
            const std::string& reason = "") {
    CHECK(result.outcome == outcome);
    CHECK(result.credited == credited);
    CHECK(result.visual == visual);
    CHECK(result.reason == reason);
}

void independentInventoryFramesAndStableWindow() {
    Fixture fixture;
    expect(fixture.update(100, 10), DeliveryVerificationOutcome::Continue);
    expect(fixture.update(100100, 10), DeliveryVerificationOutcome::Continue);
    expect(fixture.update(200100, 11), DeliveryVerificationOutcome::Continue);
    expect(fixture.update(300099, 12), DeliveryVerificationOutcome::Continue);
    expect(fixture.update(300100, 12), DeliveryVerificationOutcome::Complete, 1, true, "delivered");

    Fixture duplicates;
    for (const uint64_t elapsed : {100ULL, 150000ULL, 300100ULL, 500000ULL}) {
        expect(duplicates.update(elapsed, 10), DeliveryVerificationOutcome::Continue);
    }

    Fixture nonmonotonic;
    expect(nonmonotonic.update(100, 1), DeliveryVerificationOutcome::Continue);
    expect(nonmonotonic.update(150000, 2), DeliveryVerificationOutcome::Continue);
    expect(nonmonotonic.update(300100, 1),
           DeliveryVerificationOutcome::Complete,
           1,
           true,
           "delivered");

    Fixture initial_zero;
    initial_zero.tuning.delivery_frames = 1;
    expect(initial_zero.update(100, 0), DeliveryVerificationOutcome::Continue);
    expect(initial_zero.update(300100, 0), DeliveryVerificationOutcome::Continue);
    expect(initial_zero.update(400000, 1), DeliveryVerificationOutcome::Continue);
    expect(initial_zero.update(700000, 1),
           DeliveryVerificationOutcome::Complete,
           1,
           true,
           "delivered");
}

void changedDeltaRestartsTheStableWindow() {
    Fixture fixture;
    fixture.trip = inventory(2);
    fixture.tuning.delivery_frames = 2;
    expect(fixture.update(100, 1), DeliveryVerificationOutcome::Continue);
    expect(fixture.update(150000, 2), DeliveryVerificationOutcome::Continue);
    fixture.observation.zone_supply_count = fixture.baseline + 2;
    expect(fixture.update(200000, 3), DeliveryVerificationOutcome::Continue);
    expect(fixture.update(499999, 4), DeliveryVerificationOutcome::Continue);
    expect(fixture.update(500000, 4), DeliveryVerificationOutcome::Complete, 2, true, "delivered");
}

void inventoryEvidenceGatesResetConfirmation() {
    for (int invalid_gate = 0; invalid_gate < 12; ++invalid_gate) {
        Fixture fixture;
        fixture.tuning.delivery_frames = 2;
        expect(fixture.update(100, 1), DeliveryVerificationOutcome::Continue);
        expect(fixture.update(150000, 2), DeliveryVerificationOutcome::Continue);
        const auto valid = fixture.observation;
        switch (invalid_gate) {
            case 0:
                fixture.observed_zone = false;
                break;
            case 1:
                fixture.observation.zone_estimate.geometry_id = "other-geometry";
                break;
            case 2:
                fixture.observation.zone_inventory_complete = false;
                break;
            case 3:
                fixture.observation.zone_counts_valid = false;
                break;
            case 4:
                fixture.observation.zone_supply_count = -1;
                break;
            case 5:
                fixture.observation.zone_injured_count = -1;
                break;
            case 6:
                ++fixture.observation.zone_injured_count;
                break;
            case 7:
                fixture.observation.zone_supply_count = fixture.baseline;
                break;
            case 8:
                fixture.observation.zone_supply_count = fixture.baseline + 2;
                break;
            case 9:
                fixture.observation.gripper_done = false;
                break;
            case 10:
                fixture.observation.gripper_feedback_open = 0;
                break;
            case 11:
                fixture.observation.zone_estimate.origin_body_m = {};
                break;
        }
        expect(fixture.update(300100, 3), DeliveryVerificationOutcome::Continue);
        fixture.observation = valid;
        fixture.observed_zone = true;
        expect(fixture.update(350000, 4), DeliveryVerificationOutcome::Continue);
        expect(fixture.update(649999, 5), DeliveryVerificationOutcome::Continue);
        expect(
            fixture.update(650000, 5), DeliveryVerificationOutcome::Complete, 1, true, "delivered");
    }
}

void partialDeliveryAndTheCorrectHalf() {
    for (const bool assumed_mode : {false, true}) {
        Fixture partial;
        partial.trip = inventory(2);
        partial.tuning.assume_all_safe = assumed_mode;
        expect(partial.update(100, 1), DeliveryVerificationOutcome::Continue);
        expect(partial.update(150000, 2), DeliveryVerificationOutcome::Continue);
        expect(partial.update(300100, 3),
               DeliveryVerificationOutcome::Complete,
               1,
               true,
               "partial_delivery");
    }

    Fixture injured;
    injured.trip = inventory(0, 0, 1);
    injured.half_injured = true;
    injured.half_x = .15f;
    injured.observation.zone_supply_count = injured.baseline_other;
    injured.observation.zone_injured_count = injured.baseline + 1;
    expect(injured.update(100, 1), DeliveryVerificationOutcome::Continue);
    expect(injured.update(150000, 2), DeliveryVerificationOutcome::Continue);
    expect(injured.update(300100, 3), DeliveryVerificationOutcome::Complete, 1, true, "delivered");

    Fixture fleeting;
    fleeting.observation.zone_supply_count = fleeting.baseline;
    fleeting.observation.delivery_observed = true;
    expect(fleeting.update(fleeting.tuning.delivery_budget_us + 1),
           DeliveryVerificationOutcome::Complete,
           0,
           false,
           "delivery_unverified");
}

void strictTimeoutBoundaries() {
    for (const uint64_t elapsed : {2499999ULL, 2500000ULL, 2500001ULL}) {
        Fixture fixture;
        fixture.observed_zone = false;
        if (elapsed <= fixture.tuning.delivery_budget_us) {
            expect(fixture.update(elapsed), DeliveryVerificationOutcome::Continue);
        } else {
            expect(fixture.update(elapsed),
                   DeliveryVerificationOutcome::Complete,
                   0,
                   false,
                   "delivery_unverified");
        }
    }

    Fixture stable_late;
    expect(stable_late.update(100, 1), DeliveryVerificationOutcome::Continue);
    expect(stable_late.update(150000, 2), DeliveryVerificationOutcome::Continue);
    expect(stable_late.update(stable_late.tuning.delivery_budget_us + 1, 3),
           DeliveryVerificationOutcome::Complete,
           1,
           true,
           "delivered");

    Fixture rush_only;
    rush_only.tuning.field_rush = true;
    rush_only.observed_zone = false;
    rush_only.fixedCargo();
    expect(rush_only.update(rush_only.tuning.delivery_budget_us + 1, 1, 100),
           DeliveryVerificationOutcome::Complete,
           0,
           false,
           "delivery_unverified");
    CHECK(rush_only.stage.insideCount() == 0);

    Fixture empty;
    empty.trip = {};
    empty.observed_zone = false;
    expect(empty.update(empty.tuning.delivery_budget_us + 1),
           DeliveryVerificationOutcome::Complete,
           0,
           false,
           "delivered");
}

void noFixFarViewAndTimeouts() {
    Fixture field;
    field.field();
    expect(field.update(1500000), DeliveryVerificationOutcome::Continue);
    CHECK(!field.stage.farView());
    expect(field.update(1500001), DeliveryVerificationOutcome::Continue);
    CHECK(field.stage.farView());
    expect(field.update(7000000), DeliveryVerificationOutcome::Continue);
    CHECK(field.stage.farView());
    expect(field.update(7000001),
           DeliveryVerificationOutcome::Complete,
           0,
           false,
           "delivery_unverified");
    CHECK(!field.stage.farView());

    Fixture assumed;
    assumed.field(true);
    assumed.trip = inventory(2);
    expect(assumed.update(7000001),
           DeliveryVerificationOutcome::Complete,
           2,
           false,
           "delivered_unverified");

    Fixture longer_delivery_budget;
    longer_delivery_budget.field();
    longer_delivery_budget.tuning.assume_verify_fix_wait_us = 100;
    longer_delivery_budget.tuning.delivery_budget_us = 200;
    expect(longer_delivery_budget.update(200), DeliveryVerificationOutcome::Continue);
    expect(longer_delivery_budget.update(201),
           DeliveryVerificationOutcome::Complete,
           0,
           false,
           "delivery_unverified");

    Fixture insufficient_fix;
    insufficient_fix.field();
    insufficient_fix.fix(2);
    expect(insufficient_fix.update(1), DeliveryVerificationOutcome::Continue);
    expect(insufficient_fix.update(7000000), DeliveryVerificationOutcome::Continue);
    expect(insufficient_fix.update(7000001),
           DeliveryVerificationOutcome::Complete,
           0,
           false,
           "delivery_unverified");
}

void lateFixStartsOneStickyBudget() {
    Fixture fixture;
    fixture.field();
    expect(fixture.update(2000000), DeliveryVerificationOutcome::Continue);
    CHECK(fixture.stage.farView());
    fixture.fix();
    expect(fixture.update(6000000), DeliveryVerificationOutcome::Continue);
    CHECK(fixture.stage.farView());
    fixture.observation.vref_valid = false;
    expect(fixture.update(8500000), DeliveryVerificationOutcome::Continue);
    expect(fixture.update(8500001),
           DeliveryVerificationOutcome::Complete,
           0,
           false,
           "delivery_unverified");

    Fixture subsequent_fix;
    subsequent_fix.field();
    subsequent_fix.fix();
    expect(subsequent_fix.update(100), DeliveryVerificationOutcome::Continue);
    // The original vfix gate deliberately does not check the reference timestamp.
    subsequent_fix.observation.vref_zone.timestamp_us = 0;
    expect(subsequent_fix.update(2500100), DeliveryVerificationOutcome::Continue);
    expect(subsequent_fix.update(2500101),
           DeliveryVerificationOutcome::Complete,
           0,
           false,
           "delivery_unverified");
}

void visualEvidenceIsIndependentOfCredit() {
    for (const bool assumed : {false, true}) {
        Fixture fixture;
        fixture.field(assumed);
        fixture.trip = inventory(2);
        fixture.fixedCargo();
        expect(fixture.update(100, 1, 100), DeliveryVerificationOutcome::Continue);
        expect(fixture.update(200, 2, 101), DeliveryVerificationOutcome::Continue);
        const auto result = fixture.update(300, 3, 102);
        expect(result,
               DeliveryVerificationOutcome::Complete,
               assumed ? 2 : 0,
               true,
               assumed ? "delivered_visual" : "delivery_unverified");
        CHECK(result.events == "[CARGO_ZONE] event=verify half=supply rear_x=-0.150 rear_y=0.020 "
                               "fix=1 repush=0 in=3 out=0\n");
    }

    Fixture timestamp;
    timestamp.field();
    timestamp.fixedCargo();
    expect(timestamp.update(100, 1, 100), DeliveryVerificationOutcome::Continue);
    CHECK(timestamp.stage.insideCount() == 1);
    const auto duplicate = timestamp.update(200, 2, 100);
    expect(duplicate, DeliveryVerificationOutcome::Continue);
    CHECK(duplicate.events.empty());
    CHECK(timestamp.stage.insideCount() == 1);
    expect(timestamp.update(300, 3, 101), DeliveryVerificationOutcome::Continue);
    expect(timestamp.update(400, 4, 102),
           DeliveryVerificationOutcome::Complete,
           0,
           true,
           "delivery_unverified");
}

void visualThresholdUsesTrackedCargoFix() {
    Fixture historical_no_fix;
    historical_no_fix.field();
    historical_no_fix.tracked_cargo_has_fix = false;
    historical_no_fix.tuning.assume_img_verify_obs = 5;
    historical_no_fix.fixedCargo();
    for (uint64_t frame = 1; frame < 5; ++frame) {
        expect(historical_no_fix.update(frame, frame, frame),
               DeliveryVerificationOutcome::Continue);
    }
    expect(historical_no_fix.update(5, 5, 5),
           DeliveryVerificationOutcome::Complete,
           0,
           true,
           "delivery_unverified");

    Fixture historical_fix;
    historical_fix.field();
    historical_fix.tuning.assume_img_verify_obs = 5;
    historical_fix.imageCargo();
    expect(historical_fix.update(1, 1, 1), DeliveryVerificationOutcome::Continue);
    expect(historical_fix.update(2, 2, 2), DeliveryVerificationOutcome::Continue);
    const auto result = historical_fix.update(3, 3, 3);
    expect(result, DeliveryVerificationOutcome::Complete, 0, true, "delivery_unverified");
    CHECK(result.events == "[CARGO_ZONE] event=verify_img frac=0.50 clipped=0 in=3 out=0\n"
                           "[CARGO_ZONE] event=verify half=supply rear_x=-0.150 rear_y=0.020 fix=0 "
                           "repush=0 in=3 out=0\n");
}

void visualThresholdsAndMissingSamples() {
    Fixture fixed;
    fixed.field();
    fixed.tuning.assume_repush_max = 0;
    fixed.fixedCargo(fixed.tuning.assume_cargo_verify_in_y_m);
    expect(fixed.update(1, 1, 1), DeliveryVerificationOutcome::Continue);
    CHECK(fixed.stage.insideCount() == 1);
    fixed.cargo.rear.y = fixed.tuning.assume_cargo_verify_out_y_m;
    expect(fixed.update(2, 2, 2), DeliveryVerificationOutcome::Continue);
    CHECK(fixed.stage.insideCount() == 1 && fixed.stage.outsideCount() == 0);
    fixed.cargo.available = false;
    expect(fixed.update(3, 3, 3), DeliveryVerificationOutcome::Continue);
    CHECK(fixed.stage.insideCount() == 1);
    fixed.fixedCargo();
    fixed.cargo.clipped = true;
    expect(fixed.update(4, 4, 4), DeliveryVerificationOutcome::Continue);
    CHECK(fixed.stage.insideCount() == 0 && fixed.stage.outsideCount() == 1);
    fixed.cargo.clipped = false;
    fixed.cargo.rear.x = fixed.half_x + fixed.tuning.assume_cargo_x_tol_m + .01f;
    expect(fixed.update(5, 5, 5), DeliveryVerificationOutcome::Continue);
    CHECK(fixed.stage.outsideCount() == 2);

    Fixture image;
    image.field();
    image.tuning.assume_repush_max = 0;
    image.imageCargo(image.tuning.assume_img_inside_frac);
    expect(image.update(1, 1, 1), DeliveryVerificationOutcome::Continue);
    CHECK(image.stage.insideCount() == 1);
    image.imageCargo(image.tuning.assume_img_outside_frac);
    expect(image.update(2, 2, 2), DeliveryVerificationOutcome::Continue);
    CHECK(image.stage.insideCount() == 1 && image.stage.outsideCount() == 0);
    image.cargo.front_valid = false;
    expect(image.update(3, 3, 3), DeliveryVerificationOutcome::Continue);
    CHECK(image.stage.insideCount() == 1);
    image.cargo.front_valid = true;
    image.cargo.height_px = 1;
    expect(image.update(4, 4, 4), DeliveryVerificationOutcome::Continue);
    CHECK(image.stage.insideCount() == 1);
    image.imageCargo(.09f);
    expect(image.update(5, 5, 5), DeliveryVerificationOutcome::Continue);
    CHECK(image.stage.insideCount() == 0 && image.stage.outsideCount() == 1);
}

void outsideEvidenceAndRepushBudget() {
    Fixture repush;
    repush.field();
    repush.tuning.assume_verify_out_obs = 2;
    repush.repush_count = 1;
    repush.fixedCargo(-.02f);
    expect(repush.update(100, 1, 1), DeliveryVerificationOutcome::Continue);
    const auto retry = repush.update(8000000, 2, 2);
    expect(retry, DeliveryVerificationOutcome::Repush, 0, false, "assumed_verify_repush");
    CHECK(repush.stage.insideCount() == 0 && repush.stage.outsideCount() == 0);
    CHECK(!repush.stage.farView());
    CHECK(retry.events == "[CARGO_ZONE] event=verify half=supply rear_x=-0.150 rear_y=-0.020 fix=1 "
                          "repush=1 in=0 out=2\n"
                          "[CARGO_ZONE] event=verify_outside_repush half=supply rear_x=-0.150 "
                          "rear_y=-0.020 fix=1 repush=2 in=0 out=0\n");

    Fixture exhausted;
    exhausted.field();
    exhausted.tuning.assume_verify_out_obs = 2;
    exhausted.repush_count = exhausted.tuning.assume_repush_max;
    exhausted.fixedCargo(-.02f);
    expect(exhausted.update(100, 1, 1), DeliveryVerificationOutcome::Continue);
    expect(exhausted.update(200, 2, 2), DeliveryVerificationOutcome::Continue);
    CHECK(exhausted.stage.outsideCount() == 2);
    exhausted.cargo.available = false;
    expect(exhausted.update(8000000),
           DeliveryVerificationOutcome::Complete,
           0,
           false,
           "delivery_unverified");

    Fixture other_inside;
    other_inside.field();
    other_inside.tuning.assume_verify_out_obs = 2;
    other_inside.fixedCargo(-.02f);
    expect(other_inside.update(100, 1, 1), DeliveryVerificationOutcome::Continue);
    other_inside.cargo.other_inside = true;
    const auto done = other_inside.update(200, 2, 2);
    expect(done, DeliveryVerificationOutcome::Complete, 0, true, "delivery_unverified");
    CHECK(done.events == "[CARGO_ZONE] event=verify half=supply rear_x=-0.150 rear_y=-0.020 fix=1 "
                         "repush=0 in=0 out=2\n"
                         "[CARGO_ZONE] event=verify_other_inside half=supply rear_x=-0.150 "
                         "rear_y=-0.020 fix=1 repush=0 in=0 out=2\n");
}

void localResetMethodsPreserveOtherEvidence() {
    Fixture resumed;
    resumed.tuning.delivery_frames = 2;
    expect(resumed.update(100, 1), DeliveryVerificationOutcome::Continue);
    expect(resumed.update(150000, 2), DeliveryVerificationOutcome::Continue);
    resumed.stage.resetConfirmations();
    resumed.clock_base += 400000;
    expect(resumed.update(1, 2), DeliveryVerificationOutcome::Continue);
    expect(resumed.update(2, 3), DeliveryVerificationOutcome::Continue);
    expect(resumed.update(3, 4), DeliveryVerificationOutcome::Complete, 1, true, "delivered");

    Fixture new_baseline;
    new_baseline.tuning.delivery_frames = 2;
    expect(new_baseline.update(100, 1), DeliveryVerificationOutcome::Continue);
    expect(new_baseline.update(150000, 2), DeliveryVerificationOutcome::Continue);
    new_baseline.stage.resetInventoryEvidence();
    expect(new_baseline.update(300100, 2), DeliveryVerificationOutcome::Continue);
    expect(new_baseline.update(600099, 3), DeliveryVerificationOutcome::Continue);
    expect(new_baseline.update(600100, 3),
           DeliveryVerificationOutcome::Complete,
           1,
           true,
           "delivered");

    Fixture visual;
    visual.field();
    visual.fixedCargo();
    expect(visual.update(1500001, 1, 1), DeliveryVerificationOutcome::Continue);
    CHECK(visual.stage.farView() && visual.stage.insideCount() == 1);
    visual.stage.resetConfirmations();
    visual.stage.resetInventoryEvidence();
    CHECK(visual.stage.farView() && visual.stage.insideCount() == 1);
    visual.stage.clearVisualCounts();
    CHECK(visual.stage.farView() && visual.stage.insideCount() == 0);
    expect(visual.update(1500002, 2, 1), DeliveryVerificationOutcome::Continue);
    CHECK(visual.stage.insideCount() == 0);
    expect(visual.update(1500003, 3, 2), DeliveryVerificationOutcome::Continue);
    CHECK(visual.stage.insideCount() == 1);
}

void clearTripPreservesInventoryAndCargoTimestamp() {
    Fixture inventory_history;
    expect(inventory_history.update(100, 1), DeliveryVerificationOutcome::Continue);
    expect(inventory_history.update(150000, 2), DeliveryVerificationOutcome::Continue);
    expect(inventory_history.update(200000, 3), DeliveryVerificationOutcome::Continue);
    inventory_history.stage.clearTrip();
    expect(inventory_history.update(300100, 3),
           DeliveryVerificationOutcome::Complete,
           1,
           true,
           "delivered");

    Fixture visual_history;
    visual_history.field();
    visual_history.fixedCargo();
    expect(visual_history.update(1500001, 1, 1), DeliveryVerificationOutcome::Continue);
    CHECK(visual_history.stage.farView() && visual_history.stage.insideCount() == 1);
    visual_history.stage.clearTrip();
    CHECK(!visual_history.stage.farView());
    CHECK(visual_history.stage.insideCount() == 0 && visual_history.stage.outsideCount() == 0);
    expect(visual_history.update(1, 2, 1), DeliveryVerificationOutcome::Continue);
    CHECK(visual_history.stage.insideCount() == 0);

    Fixture fix_history;
    fix_history.field();
    fix_history.fix();
    expect(fix_history.update(100), DeliveryVerificationOutcome::Continue);
    fix_history.stage.clearTrip();
    fix_history.observation.vref_valid = false;
    expect(fix_history.update(3000000), DeliveryVerificationOutcome::Continue);
    CHECK(fix_history.stage.farView());
}
} // namespace

int main() {
    try {
        independentInventoryFramesAndStableWindow();
        changedDeltaRestartsTheStableWindow();
        inventoryEvidenceGatesResetConfirmation();
        partialDeliveryAndTheCorrectHalf();
        strictTimeoutBoundaries();
        noFixFarViewAndTimeouts();
        lateFixStartsOneStickyBudget();
        visualEvidenceIsIndependentOfCredit();
        visualThresholdUsesTrackedCargoFix();
        visualThresholdsAndMissingSamples();
        outsideEvidenceAndRepushBudget();
        localResetMethodsPreserveOtherEvidence();
        clearTripPreservesInventoryAndCargoTimestamp();
        std::cout << "delivery verification tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "delivery verification tests failed: " << error.what() << '\n';
        return 1;
    }
}
