#include "rescue/task/push_in_stage.hpp"
#include "rescue/push_task.hpp"

#include <cmath>
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

bool near(float actual, float expected) {
    return std::abs(actual - expected) < .00001f;
}

struct Fixture {
    PushInStage stage;
    TaskTuning tuning;
    PushInContext context;
    PushInCargoEvidence cargo;

    Fixture() {
        tuning.assume_probe = false;
        context.now_us = 10000000;
        context.elapsed_us = 1;
        context.anchored_zone = true;
        context.visual_reference_seen_us = context.now_us;
        context.hold_center = .12f;
    }

    PushInResult update() {
        return stage.update(tuning, context, cargo);
    }

    PushInResult at(uint64_t now_us, uint64_t elapsed_us) {
        context.now_us = now_us;
        context.elapsed_us = elapsed_us;
        return update();
    }

    void visible(float rear_y = 0) {
        cargo.visible = cargo.fixed = true;
        cargo.clipped = false;
        cargo.rear = {context.half_x, rear_y};
        cargo.body = {0, .32f};
        cargo.seen_us = context.now_us;
    }
};

void expect(const PushInResult& result,
            PushInOutcome outcome,
            const std::string& reason,
            bool drive = false) {
    CHECK(result.outcome == outcome);
    CHECK(result.reason == reason);
    CHECK(result.drive == drive);
    if (!drive)
        CHECK(result.vx == 0 && result.wz == 0);
}

void event(const PushInResult& result,
           size_t index,
           PushInEventKind kind,
           const std::string& text,
           const cv::Point2f& position,
           bool visible) {
    CHECK(index < result.events.size());
    const auto& actual = result.events[index];
    CHECK(actual.kind == kind);
    CHECK(actual.text == text);
    CHECK(near(actual.position.x, position.x));
    CHECK(near(actual.position.y, position.y));
    CHECK(actual.visible == visible);
}

void insideEvidencePrecedesBlockedStuckAndTimeout() {
    Fixture fixture;
    fixture.stage.prepare(true);
    fixture.context.hold.y = fixture.tuning.assume_inside_min_hold_y_m;
    fixture.context.imu_blocked_us = fixture.tuning.assume_imu_block_us;
    fixture.context.elapsed_us = fixture.tuning.enter_budget_us + 1;
    fixture.visible(fixture.tuning.assume_cargo_done_y_m);
    fixture.cargo.rear.x = fixture.tuning.assume_cargo_x_tol_m;
    fixture.cargo.static_since_us = fixture.context.now_us - fixture.tuning.assume_stuck_us;
    const auto result = fixture.update();
    expect(result, PushInOutcome::Complete, "assumed_push_in_done");
    CHECK(result.reset_cargo_reference);
    CHECK(result.events.size() == 2);
    event(result, 0, PushInEventKind::Cargo, "push_inside", fixture.cargo.rear, true);
    event(result, 1, PushInEventKind::DropZone, "push_done", fixture.context.hold, false);
}

void blockedDeepRequiresUnplacedCargoAndEnoughDepth() {
    for (const bool visible : {false, true}) {
        Fixture fixture;
        fixture.context.hold.y = fixture.tuning.assume_blocked_deep_hold_y_m;
        fixture.context.imu_blocked_us = fixture.tuning.assume_imu_block_us;
        fixture.tuning.assume_enter_confirm = true;
        if (visible) {
            fixture.visible();
            fixture.cargo.clipped = true;
        }
        const auto result = fixture.update();
        expect(result, PushInOutcome::Complete, "assumed_push_in_done");
        CHECK(result.events.size() == 2);
        event(result, 0, PushInEventKind::Cargo, "blocked_deep", fixture.cargo.rear, visible);
        event(result, 1, PushInEventKind::DropZone, "push_done", fixture.context.hold, false);
    }

    Fixture placed_outside;
    placed_outside.context.hold.y = .2f;
    placed_outside.context.imu_blocked_us = placed_outside.tuning.assume_imu_block_us;
    placed_outside.visible(-.02f);
    const auto retry = placed_outside.update();
    expect(retry, PushInOutcome::Repush, "assumed_stuck_repush");
    CHECK(retry.reset_cargo_reference && retry.events.size() == 1);
    event(retry, 0, PushInEventKind::Cargo, "stuck_repush", placed_outside.cargo.rear, true);

    Fixture too_shallow;
    too_shallow.context.hold.y = too_shallow.tuning.assume_blocked_deep_hold_y_m - .001f;
    too_shallow.context.imu_blocked_us = too_shallow.tuning.assume_imu_block_us;
    expect(too_shallow.update(), PushInOutcome::Repush, "assumed_stuck_repush");
}

void stuckCanRetryAfterTheEnterTimeout() {
    Fixture fixture;
    fixture.context.elapsed_us = fixture.tuning.enter_budget_us + 1;
    fixture.context.imu_blocked_us = fixture.tuning.assume_imu_block_us;
    const auto retry = fixture.update();
    expect(retry, PushInOutcome::Repush, "assumed_stuck_repush");
    CHECK(retry.reset_cargo_reference && retry.events.size() == 1);

    Fixture exhausted;
    exhausted.tuning.assume_enter_confirm = true;
    exhausted.tuning.assume_enter_hold_y_m = 1;
    exhausted.context.imu_blocked_us = exhausted.tuning.assume_imu_block_us;
    exhausted.context.repush_count = exhausted.tuning.assume_repush_max;
    const auto giveup = exhausted.update();
    expect(giveup, PushInOutcome::Complete, "assumed_push_in_done");
    CHECK(giveup.events.size() == 2);
    event(giveup, 0, PushInEventKind::Cargo, "stuck_giveup", exhausted.cargo.rear, true);
    event(giveup, 1, PushInEventKind::DropZone, "push_done", exhausted.context.hold, false);
}

void staticCargoUsesAnInclusiveBudgetAndIgnoresClippedEvidence() {
    Fixture fixture;
    fixture.visible(-.02f);
    fixture.cargo.static_since_us = fixture.context.now_us - fixture.tuning.assume_stuck_us + 1;
    expect(fixture.update(), PushInOutcome::Continue, "assumed_push_in_visual", true);
    --fixture.cargo.static_since_us;
    expect(fixture.update(), PushInOutcome::Repush, "assumed_stuck_repush");

    Fixture clipped;
    clipped.visible();
    clipped.cargo.clipped = true;
    clipped.cargo.static_since_us = clipped.context.now_us - clipped.tuning.assume_stuck_us;
    expect(clipped.update(), PushInOutcome::Continue, "assumed_push_in_visual", true);
    clipped.cargo.visible = false;
    expect(clipped.update(), PushInOutcome::Continue, "assumed_push_in_visual", true);
}

void alignmentTurnsBeforeDepthAndSuppressesTheStuckWatchdog() {
    Fixture fixture;
    fixture.tuning.min_turn_wz = .1f;
    fixture.stage.prepare(true);
    fixture.context.hold.y = fixture.tuning.assume_push_depth_m + fixture.tuning.assume_overshoot_m;
    fixture.context.imu_blocked_us = fixture.tuning.assume_imu_block_us;
    fixture.visible();
    fixture.cargo.body.x = .2f;
    fixture.cargo.static_since_us = 1;
    const auto turning = fixture.update();
    expect(turning, PushInOutcome::Continue, "assumed_push_align_turn", true);
    CHECK(turning.vx == 0 && near(turning.wz, -.3f));
    CHECK(turning.reset_cargo_reference && turning.events.empty());

    const uint64_t started = fixture.context.now_us;
    fixture.cargo.seen_us = started + fixture.tuning.assume_align_budget_us - 1;
    const auto before = fixture.at(started + fixture.tuning.assume_align_budget_us - 1, 1);
    expect(before, PushInOutcome::Continue, "assumed_push_align_turn", true);
    const auto expired = fixture.at(started + fixture.tuning.assume_align_budget_us, 2);
    expect(expired, PushInOutcome::Complete, "assumed_push_in_done");
    CHECK(expired.reset_cargo_reference && expired.events.size() == 2);
    CHECK(expired.events[0].kind == PushInEventKind::Text);
    CHECK(expired.events[0].text ==
          "[CARGO_ZONE] event=push_align_giveup half=supply bearing_deg=-45.0 body_x=0.200 "
          "body_y=0.320 seen=1 waited_ms=3000\n");
    event(expired, 1, PushInEventKind::DropZone, "push_done", fixture.context.hold, false);
}

void alignmentFreshnessAndPrepareControlItsLifecycle() {
    Fixture fixture;
    fixture.stage.prepare(true);
    fixture.visible();
    fixture.cargo.body.x = .2f;
    fixture.cargo.seen_us = fixture.context.now_us - 400000;
    const auto fresh = fixture.update();
    expect(fresh, PushInOutcome::Continue, "assumed_push_align_turn", true);
    CHECK(fresh.reset_cargo_reference);
    const auto stale = fixture.at(fixture.context.now_us + 1, 2);
    expect(stale, PushInOutcome::Continue, "assumed_push_in_visual", true);
    CHECK(stale.reset_cargo_reference && stale.events.size() == 1);
    CHECK(stale.events[0].text.find("event=push_align_unseen") != std::string::npos);
    CHECK(stale.events[0].text.find("seen=0 waited_ms=0\n") != std::string::npos);
    CHECK(!fixture.update().reset_cargo_reference);

    Fixture cancelled;
    cancelled.stage.prepare(true);
    cancelled.visible();
    cancelled.cargo.body.x = .2f;
    cancelled.stage.prepare(false);
    const auto moving = cancelled.update();
    expect(moving, PushInOutcome::Continue, "assumed_push_in_visual", true);
    CHECK(!moving.reset_cargo_reference && moving.events.empty());

    Fixture aligned;
    aligned.context.half_injured = true;
    aligned.stage.prepare(true);
    aligned.visible();
    const auto straight = aligned.update();
    expect(straight, PushInOutcome::Continue, "assumed_push_in_visual", true);
    CHECK(straight.reset_cargo_reference && straight.events.size() == 1);
    CHECK(straight.events[0].text ==
          "[CARGO_ZONE] event=push_aligned half=injured bearing_deg=-0.0 body_x=0.000 body_y=0.320 "
          "seen=1 waited_ms=0\n");
}

void confirmationCanExtendAndThenCompleteWithoutACargoFix() {
    Fixture fixture;
    fixture.tuning.assume_enter_confirm = true;
    fixture.tuning.assume_enter_hold_y_m = .25f;
    fixture.tuning.assume_enter_max_hold_y_m = .5f;
    fixture.context.hold.y = fixture.tuning.assume_push_depth_m;
    const auto waiting = fixture.update();
    expect(waiting, PushInOutcome::Continue, "assumed_push_in_visual", true);
    CHECK(waiting.events.size() == 2);
    event(waiting, 0, PushInEventKind::Cargo, "push_unconfirmed_hidden", fixture.cargo.rear, false);
    event(waiting, 1, PushInEventKind::DropZone, "push_unconfirmed", fixture.context.hold, false);
    CHECK(fixture.update().events.empty());
    fixture.stage.prepare(false);
    CHECK(fixture.update().events.empty());
    fixture.context.hold.y = .25f;
    const auto late = fixture.update();
    expect(late, PushInOutcome::Complete, "assumed_push_in_done");
    CHECK(late.events.size() == 2);
    event(late, 0, PushInEventKind::Cargo, "push_confirmed_late", fixture.cargo.rear, false);
    event(late, 1, PushInEventKind::DropZone, "push_done", fixture.context.hold, false);

    Fixture visible_without_fix;
    visible_without_fix.tuning.assume_enter_confirm = true;
    visible_without_fix.context.hold.y = visible_without_fix.tuning.assume_push_depth_m +
                                         visible_without_fix.tuning.assume_overshoot_m;
    visible_without_fix.visible(visible_without_fix.tuning.assume_enter_true_y_m);
    visible_without_fix.cargo.fixed = false;
    const auto real = visible_without_fix.update();
    expect(real, PushInOutcome::Complete, "assumed_push_in_done");
    CHECK(real.events.size() == 1);
    event(real, 0, PushInEventKind::DropZone, "push_done", visible_without_fix.context.hold, false);
}

void confirmationStopsAtMaximumDepthOrExtraBudget() {
    for (const bool maximum_depth : {false, true}) {
        Fixture fixture;
        fixture.tuning.assume_enter_confirm = true;
        fixture.tuning.assume_enter_hold_y_m = 1;
        fixture.context.hold.y = fixture.tuning.assume_enter_max_hold_y_m - .001f;
        fixture.context.elapsed_us =
            fixture.tuning.enter_budget_us + fixture.tuning.assume_enter_extra_us - 1;
        expect(fixture.update(), PushInOutcome::Continue, "assumed_push_in_visual", true);
        if (maximum_depth)
            fixture.context.hold.y = fixture.tuning.assume_enter_max_hold_y_m;
        else
            ++fixture.context.elapsed_us;
        const auto stopped = fixture.update();
        expect(stopped, PushInOutcome::Complete, "assumed_push_in_done");
        CHECK(stopped.events.size() == 2);
        event(stopped,
              0,
              PushInEventKind::Cargo,
              "push_unconfirmed_giveup",
              fixture.cargo.rear,
              false);
        event(stopped, 1, PushInEventKind::DropZone, "push_done", fixture.context.hold, false);
    }

    Fixture cleared;
    cleared.tuning.assume_enter_confirm = true;
    cleared.tuning.assume_enter_hold_y_m = 1;
    cleared.context.hold.y = cleared.tuning.assume_push_depth_m;
    CHECK(cleared.update().events.size() == 2);
    cleared.stage.clearTrip();
    const auto fresh = cleared.update();
    CHECK(fresh.events.size() == 2);
    event(fresh, 0, PushInEventKind::Cargo, "push_unconfirmed_hidden", cleared.cargo.rear, false);
}

void pursuitUsesTheLastCargoHeadingUntilPrepare() {
    Fixture fixture;
    fixture.tuning.assume_align_push = true;
    fixture.tuning.heading_gain = 1;
    fixture.tuning.lateral_gain = 0;
    fixture.tuning.enter_speed = .1f;
    fixture.tuning.max_speed = .15f;
    fixture.context.repush_count = 3;
    fixture.context.zone_yaw = .3f;
    fixture.visible();
    fixture.cargo.body.x = .2f;
    const auto pursuit = fixture.update();
    expect(pursuit, PushInOutcome::Continue, "assumed_push_in_pursuit", true);
    CHECK(near(pursuit.vx, .15f));
    CHECK(near(pursuit.wz, -.78539816f));
    fixture.cargo.visible = false;
    fixture.context.zone_yaw = .5f;
    const auto held = fixture.update();
    expect(held, PushInOutcome::Continue, "assumed_push_in_hold_heading", true);
    CHECK(near(held.wz, -.58539816f));
    fixture.stage.prepare(false);
    const auto reset = fixture.update();
    expect(reset, PushInOutcome::Continue, "assumed_push_in_visual", true);
    CHECK(near(reset.wz, .5f));

    Fixture slant;
    slant.tuning.assume_align_push = true;
    slant.tuning.heading_gain = 1;
    slant.tuning.assume_slant_max_rad = .1f;
    slant.visible();
    slant.cargo.rear.x = .2f;
    const auto limited = slant.update();
    expect(limited, PushInOutcome::Continue, "assumed_push_in_pursuit", true);
    CHECK(near(limited.wz, .1f));
}

void visualReferenceFreshnessAndPursuitHaveInclusiveBoundaries() {
    Fixture fixture;
    fixture.tuning.assume_align_push = true;
    fixture.visible();
    fixture.cargo.body.x = .2f;
    fixture.cargo.seen_us = fixture.context.now_us - 400000;
    expect(fixture.update(), PushInOutcome::Continue, "assumed_push_in_pursuit", true);
    expect(fixture.at(fixture.context.now_us + 1, 2),
           PushInOutcome::Continue,
           "assumed_push_in_hold_heading",
           true);

    Fixture reference;
    reference.context.visual_reference_seen_us = reference.context.now_us - 400000;
    expect(reference.update(), PushInOutcome::Continue, "assumed_push_in_visual", true);
    expect(reference.at(reference.context.now_us + 1, 2),
           PushInOutcome::Continue,
           "assumed_push_in_dead_reckoning",
           true);
}

void probePulseBoundariesAndPrepareHistory() {
    Fixture fixture;
    fixture.tuning.assume_probe = true;
    fixture.tuning.assume_probe_first_us = 100;
    fixture.tuning.assume_probe_len_us = 50;
    fixture.tuning.assume_probe_gap_us = 70;
    const uint64_t start = fixture.context.now_us;
    const auto before = fixture.at(start, 99);
    expect(before, PushInOutcome::Continue, "assumed_push_in_visual", true);
    CHECK(before.wz == 0);
    const auto first = fixture.at(start + 1, 100);
    expect(first, PushInOutcome::Continue, "assumed_push_imu_probe", true);
    CHECK(near(first.wz, -fixture.tuning.min_turn_wz));
    CHECK(near(fixture.at(start + 50, 149).wz, -fixture.tuning.min_turn_wz));
    const auto end = fixture.at(start + 51, 150);
    expect(end, PushInOutcome::Continue, "assumed_push_in_visual", true);
    CHECK(end.wz == 0);
    CHECK(fixture.at(start + 120, 219).wz == 0);
    const auto second = fixture.at(start + 121, 220);
    expect(second, PushInOutcome::Continue, "assumed_push_imu_probe", true);
    CHECK(near(second.wz, fixture.tuning.min_turn_wz));
    fixture.stage.prepare(false);
    CHECK(near(fixture.at(start + 122, 100).wz, fixture.tuning.min_turn_wz));
    fixture.stage.clearTrip();
    CHECK(near(fixture.at(start + 123, 100).wz, -fixture.tuning.min_turn_wz));
}

void steeringCancelsAnActiveProbeWithoutRearmingItsGap() {
    Fixture fixture;
    fixture.tuning.assume_probe = true;
    fixture.tuning.assume_probe_first_us = 100;
    fixture.tuning.assume_probe_len_us = 50;
    fixture.tuning.assume_probe_gap_us = 70;
    const uint64_t start = fixture.context.now_us;
    CHECK(near(fixture.at(start, 100).wz, -fixture.tuning.min_turn_wz));
    fixture.context.zone_yaw = .2f;
    const auto steering = fixture.at(start + 1, 101);
    expect(steering, PushInOutcome::Continue, "assumed_push_in_visual", true);
    CHECK(near(steering.wz, .3f));
    fixture.context.zone_yaw = 0;
    const auto cancelled = fixture.at(start + 2, 102);
    expect(cancelled, PushInOutcome::Continue, "assumed_push_in_visual", true);
    CHECK(cancelled.wz == 0);
    CHECK(near(fixture.at(start + 120, 220).wz, fixture.tuning.min_turn_wz));
}

void noAnchorUsesBlindDistanceAndTheLegacyInsideSentinel() {
    Fixture inside;
    inside.context.anchored_zone = false;
    inside.context.hold.y = -100;
    inside.visible(inside.tuning.assume_cargo_done_y_m);
    const auto seen = inside.update();
    expect(seen, PushInOutcome::Complete, "assumed_push_in_done");
    CHECK(seen.events.size() == 1);
    event(seen, 0, PushInEventKind::Cargo, "push_inside", inside.cargo.rear, true);

    Fixture blind;
    blind.context.anchored_zone = false;
    blind.tuning.assume_enter_confirm = true;
    blind.tuning.enter_speed = .8f;
    blind.tuning.max_speed = .1f;
    blind.context.repush_count = 3;
    blind.context.travel_m = blind.tuning.assume_push_m - .001f;
    blind.context.elapsed_us = blind.tuning.enter_budget_us;
    const auto moving = blind.update();
    expect(moving, PushInOutcome::Continue, "assumed_push_in_blind", true);
    CHECK(near(moving.vx, .8f) && moving.wz == 0);
    blind.context.travel_m = blind.tuning.assume_push_m;
    const auto reached = blind.update();
    expect(reached, PushInOutcome::Complete, "assumed_push_in_done");
    CHECK(reached.events.empty());

    Fixture timeout;
    timeout.context.anchored_zone = false;
    timeout.tuning.assume_enter_confirm = true;
    timeout.context.elapsed_us = timeout.tuning.enter_budget_us + 1;
    const auto expired = timeout.update();
    expect(expired, PushInOutcome::Complete, "assumed_push_in_done");
    CHECK(expired.events.empty());
}
} // namespace

int main() {
    try {
        insideEvidencePrecedesBlockedStuckAndTimeout();
        blockedDeepRequiresUnplacedCargoAndEnoughDepth();
        stuckCanRetryAfterTheEnterTimeout();
        staticCargoUsesAnInclusiveBudgetAndIgnoresClippedEvidence();
        alignmentTurnsBeforeDepthAndSuppressesTheStuckWatchdog();
        alignmentFreshnessAndPrepareControlItsLifecycle();
        confirmationCanExtendAndThenCompleteWithoutACargoFix();
        confirmationStopsAtMaximumDepthOrExtraBudget();
        pursuitUsesTheLastCargoHeadingUntilPrepare();
        visualReferenceFreshnessAndPursuitHaveInclusiveBoundaries();
        probePulseBoundariesAndPrepareHistory();
        steeringCancelsAnActiveProbeWithoutRearmingItsGap();
        noAnchorUsesBlindDistanceAndTheLegacyInsideSentinel();
        std::cout << "push-in stage tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "push-in stage tests failed: " << error.what() << '\n';
        return 1;
    }
}
