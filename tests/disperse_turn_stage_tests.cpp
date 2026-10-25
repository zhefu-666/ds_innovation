#include "rescue/task/disperse_turn_stage.hpp"
#include "rescue/push_task.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace rescue;

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
}
#define CHECK(expression) check((expression), #expression, __LINE__)

void expect(const DisperseTurnResult& result, DisperseTurnOutcome outcome,
            DisperseTurnMotion motion, const char* reason = nullptr) {
    CHECK(result.outcome == outcome);
    CHECK(result.motion == motion);
    CHECK(reason ? result.reason && std::string(result.reason) == reason : result.reason == nullptr);
    if (motion != DisperseTurnMotion::Turn) CHECK(result.direction == 0);
}

struct Fixture {
    DisperseTurnStage stage;
    TaskTuning tuning;
    DisperseTurnContext context;
    PushObservation observation;

    Fixture() {
        context.now_us = context.sweep_start_us = 10000;
        context.camera_ready = context.mid_pitch = true;
        tuning.sweep_frame_front_m = .1f;
        tuning.sweep_fresh_us = 100;
        tuning.disperse_turn_view_range_m = 1;
        tuning.disperse_turn_view_half_rad = 1;
        tuning.disperse_turn_rad = 2;
        tuning.disperse_turn_min_rad = .5f;
        tuning.disperse_swing_tol_rad = .125f;
        tuning.disperse_swing_budget_us = 10000;
        tuning.disperse_turn_clear_max = 1;
        tuning.disperse_turn_clear_frames = 2;
        tuning.disperse_turn_look_us = 1000;
        tuning.disperse_turn_push_m = 0;
        tuning.disperse_turn_push_budget_us = 500;
        tuning.disperse_turn_frame_check = false;
        tuning.disperse_turn_frame_max = 1;
        tuning.disperse_turn_frame_range_m = .3f;
        tuning.disperse_turn_frame_half_m = .14f;
        tuning.disperse_turn_frame_extra_rad = .25f;
        observation.heading_rad = 0;
        crowded();
        stage.begin(true);
    }

    void crowded() { observation.clutter_body = {{-.1f, .5f}, {.1f, .5f}}; }
    void fresh() { observation.clutter_ts_us = context.now_us; }

    DisperseTurnResult update() {
        const auto result = stage.update(observation, tuning, context);
        if (result.turn_started) {
            context.sweep_start_us = context.now_us;
            context.sweep_yaw = observation.heading_rad;
        }
        if (result.push_started) {
            context.sweep_start_us = context.now_us;
            context.sweep_travel_m = context.travel_m;
        }
        return result;
    }

    DisperseTurnResult start() {
        fresh();
        const auto result = update();
        CHECK(result.turn_started);
        return result;
    }

    void frameCheck() {
        tuning.disperse_turn_frame_check = true;
        observation.clutter_body.clear();
        expect(start(), DisperseTurnOutcome::Continue, DisperseTurnMotion::Turn, "disperse_turn");
        observation.heading_rad = tuning.disperse_turn_min_rad;
        expect(update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::None, "disperse_frame_look");
        CHECK(stage.phase() == DisperseTurnPhase::FrameCheck && stage.needsFrameView());
    }
};

void lifecycleAndFreshLookDeadline() {
    Fixture fixture;
    fixture.stage.cancel();
    CHECK(!fixture.stage.active() && !fixture.stage.needsFrameView());
    const auto idle = fixture.update();
    expect(idle, DisperseTurnOutcome::Continue, DisperseTurnMotion::None);
    CHECK(!idle.turn_started && !idle.push_started && !idle.frame_event);
    fixture.stage.begin(false);
    CHECK(fixture.stage.phase() == DisperseTurnPhase::Idle);
    fixture.stage.begin(true);
    CHECK(fixture.stage.phase() == DisperseTurnPhase::Look);
    fixture.context.camera_ready = false;
    fixture.fresh();
    expect(fixture.update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::None, "disperse_look");
    fixture.context.camera_ready = true;
    fixture.observation.clutter_ts_us = 0;
    expect(fixture.update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::None, "disperse_look");
    fixture.context.now_us += fixture.tuning.disperse_turn_look_us - 1;
    expect(fixture.update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::None, "disperse_look");
    ++fixture.context.now_us;
    fixture.context.fallback_direction = -1;
    const auto timeout = fixture.update();
    expect(timeout, DisperseTurnOutcome::Continue, DisperseTurnMotion::Turn, "disperse_turn");
    CHECK(timeout.turn_started && timeout.direction == -1);
    fixture.stage.begin(true);
    fixture.context.now_us += 1000;
    expect(fixture.update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::None, "disperse_look");
    CHECK(fixture.stage.phase() == DisperseTurnPhase::Look);
}

void beginRestartsConfirmationHistory() {
    Fixture fixture;
    fixture.tuning.disperse_turn_clear_frames = 3;
    fixture.observation.clutter_body.clear();
    fixture.start();
    fixture.update();
    fixture.stage.begin(true);
    CHECK(fixture.stage.phase() == DisperseTurnPhase::Look);
    const auto restarted = fixture.start();
    CHECK(restarted.motion == DisperseTurnMotion::Turn);
    fixture.observation.heading_rad = .5f;
    expect(fixture.update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::Turn, "disperse_turn");
    expect(fixture.update(), DisperseTurnOutcome::Complete, DisperseTurnMotion::None);
    CHECK(!fixture.stage.active());
}

void directionUsesEmptierSideThenTheFallback() {
    struct Case { std::vector<cv::Point2f> points; float fallback, expected; };
    const Case cases[] = {
        {{{-.1f, .5f}, {-.2f, .5f}}, 1, -1},
        {{{.1f, .5f}, {.2f, .5f}}, -1, 1},
        {{{-.1f, .5f}, {.1f, .5f}}, -1, -1},
        {{}, 1, 1},
        {{{0, .5f}}, -1, 1}
    };
    for (const auto& item : cases) {
        Fixture fixture;
        fixture.observation.clutter_body = item.points;
        fixture.context.fallback_direction = item.fallback;
        fixture.observation.heading_rad = .25f;
        const auto result = fixture.start();
        expect(result, DisperseTurnOutcome::Continue, DisperseTurnMotion::Turn, "disperse_turn");
        CHECK(result.direction == item.expected && fixture.context.sweep_yaw == .25f);
        CHECK(fixture.context.sweep_start_us == fixture.context.now_us);
    }
}

void lookRequiresAnAtOrAfterReadyFreshNonfutureFrame() {
    for (const int age : {100, 101, -1}) {
        Fixture fixture;
        fixture.observation.clutter_ts_us = 0;
        fixture.update();
        fixture.context.now_us += 100;
        fixture.observation.clutter_ts_us = static_cast<uint64_t>(static_cast<int64_t>(fixture.context.now_us) - age);
        const auto result = fixture.update();
        CHECK(result.turn_started == (age == 100));
        CHECK(fixture.stage.phase() == (age == 100 ? DisperseTurnPhase::Turn : DisperseTurnPhase::Look));
    }
    Fixture fixture;
    fixture.observation.clutter_ts_us = fixture.context.now_us - 1;
    CHECK(!fixture.update().turn_started);
    fixture.fresh();
    CHECK(fixture.update().turn_started);
}

void viewGeometryKeepsInclusiveOuterBoundsAndSkipsInvalidPoints() {
    Fixture fixture;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    const float front = fixture.tuning.sweep_frame_front_m + .03f;
    fixture.observation.clutter_body = {
        {.1f, .5f}, {nan, .5f}, {-.1f, nan}, {-infinity, .5f}, {-.1f, infinity},
        {-2, .5f}, {-1, .2f}, {-.01f, front}, {-.1f, 0}, {-.1f, -.5f}
    };
    fixture.context.fallback_direction = -1;
    CHECK(fixture.start().direction == 1);

    Fixture range;
    range.tuning.disperse_turn_view_range_m = .5f;
    range.observation.clutter_body = {{0, .5f}};
    range.context.fallback_direction = -1;
    CHECK(range.start().direction == 1);

    Fixture bearing;
    bearing.tuning.disperse_turn_view_half_rad = std::atan2(.25f, .25f);
    bearing.observation.clutter_body = {{-.25f, .25f}};
    CHECK(bearing.start().direction == -1);
}

void repeatedFreshFramesCountDuringTurnAndStaleFramesPreserveHistory() {
    Fixture duplicate;
    duplicate.tuning.disperse_turn_clear_frames = 3;
    duplicate.observation.clutter_body.clear();
    duplicate.start();
    duplicate.observation.heading_rad = .5f;
    expect(duplicate.update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::Turn, "disperse_turn");
    expect(duplicate.update(), DisperseTurnOutcome::Complete, DisperseTurnMotion::None);

    for (const bool future : {false, true}) {
        Fixture fixture;
        fixture.observation.clutter_body.clear();
        fixture.start();
        fixture.observation.heading_rad = .5f;
        fixture.context.now_us += fixture.tuning.sweep_fresh_us + 1;
        if (future) fixture.observation.clutter_ts_us = fixture.context.now_us + 1;
        expect(fixture.update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::Turn, "disperse_turn");
        fixture.fresh();
        expect(fixture.update(), DisperseTurnOutcome::Complete, DisperseTurnMotion::None);
    }
}

void crowdedFreshFramesResetTurnConfirmation() {
    Fixture fixture;
    fixture.observation.clutter_body.clear();
    fixture.start();
    fixture.observation.heading_rad = .5f;
    fixture.crowded();
    fixture.update();
    fixture.observation.clutter_body.clear();
    expect(fixture.update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::Turn, "disperse_turn");
    expect(fixture.update(), DisperseTurnOutcome::Complete, DisperseTurnMotion::None);
}

void minimumAngleAndPlannedCapsUseTheirOriginalBoundaries() {
    Fixture minimum;
    minimum.observation.clutter_body.clear();
    minimum.start();
    minimum.observation.heading_rad = .499f;
    expect(minimum.update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::Turn, "disperse_turn");
    minimum.observation.heading_rad = .5f;
    expect(minimum.update(), DisperseTurnOutcome::Complete, DisperseTurnMotion::None);

    for (const float direction : {-1.f, 1.f}) {
        Fixture fixture;
        fixture.context.fallback_direction = direction;
        fixture.context.swing_plan_active = true;
        fixture.context.cap_positive = direction > 0 ? .5f : .25f;
        fixture.context.cap_negative = direction < 0 ? .5f : .25f;
        fixture.start();
        fixture.context.camera_ready = false;
        fixture.observation.heading_rad = direction * .374f;
        expect(fixture.update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::Turn, "disperse_turn");
        fixture.observation.heading_rad = direction * .375f;
        expect(fixture.update(), DisperseTurnOutcome::Complete, DisperseTurnMotion::None);
    }
}

void turnTimeoutIsExclusiveAndLeavesTheCompletionReasonEmpty() {
    Fixture fixture;
    fixture.start();
    fixture.context.now_us += fixture.tuning.disperse_swing_budget_us;
    fixture.context.camera_ready = false;
    expect(fixture.update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::Turn, "disperse_turn");
    ++fixture.context.now_us;
    const auto result = fixture.update();
    expect(result, DisperseTurnOutcome::Complete, DisperseTurnMotion::None);
    CHECK(!result.turn_started && !result.push_started && !result.frame_event);
}

void frameChecksRequireMidPitchAndDeduplicateTimestamps() {
    Fixture no_mid;
    no_mid.tuning.disperse_turn_frame_check = true;
    no_mid.context.mid_pitch = false;
    no_mid.observation.clutter_body.clear();
    no_mid.start();
    no_mid.observation.heading_rad = .5f;
    expect(no_mid.update(), DisperseTurnOutcome::Complete, DisperseTurnMotion::None);

    Fixture fixture;
    fixture.frameCheck();
    const auto first = fixture.update();
    expect(first, DisperseTurnOutcome::Continue, DisperseTurnMotion::None, "disperse_frame_confirm");
    CHECK(first.frame_event && first.frame_count == 0 && first.confirmations == 1 && first.turned == .5f);
    const auto duplicate = fixture.update();
    expect(duplicate, DisperseTurnOutcome::Continue, DisperseTurnMotion::None, "disperse_frame_confirm");
    CHECK(!duplicate.frame_event);
    ++fixture.context.now_us;
    fixture.fresh();
    const auto confirmed = fixture.update();
    expect(confirmed, DisperseTurnOutcome::Complete, DisperseTurnMotion::None);
    CHECK(confirmed.frame_event && confirmed.confirmations == 2 && !fixture.stage.needsFrameView());
}

void lowFrameGeometryHasInclusiveSidesAndFrontButExcludesTheOrigin() {
    Fixture fixture;
    fixture.frameCheck();
    const float half = fixture.tuning.disperse_turn_frame_half_m;
    const float range = fixture.tuning.disperse_turn_frame_range_m;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    fixture.observation.clutter_body = {
        {half, range}, {-half, range}, {0, .01f},
        {half + .001f, .1f}, {-half - .001f, .1f}, {0, range + .001f},
        {0, 0}, {0, -.01f}, {nan, .1f}, {0, nan}, {infinity, .1f}, {0, infinity}
    };
    fixture.tuning.disperse_turn_frame_max = 3;
    const auto boundary = fixture.update();
    expect(boundary, DisperseTurnOutcome::Continue, DisperseTurnMotion::None, "disperse_frame_confirm");
    CHECK(boundary.frame_event && boundary.frame_count == 3 && boundary.confirmations == 1);
    fixture.tuning.disperse_turn_frame_max = 2;
    ++fixture.context.now_us;
    fixture.fresh();
    const auto crowded = fixture.update();
    expect(crowded, DisperseTurnOutcome::Continue, DisperseTurnMotion::Turn, "disperse_turn_frame");
    CHECK(crowded.frame_event && crowded.frame_count == 3 && crowded.confirmations == 0);
}

void lowFramesMustBeAtOrAfterReadinessFreshAndNotFromTheFuture() {
    for (const int age : {100, 101, -1}) {
        Fixture fixture;
        fixture.frameCheck();
        fixture.observation.clutter_ts_us = 0;
        fixture.context.camera_ready = false;
        expect(fixture.update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::None, "disperse_frame_look");
        fixture.context.now_us += 10;
        fixture.context.camera_ready = true;
        fixture.observation.clutter_ts_us = fixture.context.now_us - 1;
        CHECK(!fixture.update().frame_event);
        fixture.context.now_us += 100;
        fixture.observation.clutter_ts_us = static_cast<uint64_t>(static_cast<int64_t>(fixture.context.now_us) - age);
        const auto result = fixture.update();
        CHECK(result.frame_event == (age == 100));
        CHECK(fixture.stage.phase() == DisperseTurnPhase::FrameCheck);
        if (age == 100) CHECK(result.confirmations == 1);
    }
}

void lowFrameFirstObservationTimeoutIsInclusive() {
    Fixture fixture;
    fixture.frameCheck();
    fixture.observation.clutter_ts_us = 0;
    expect(fixture.update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::None, "disperse_frame_look");
    fixture.context.now_us += fixture.tuning.disperse_turn_look_us - 1;
    expect(fixture.update(), DisperseTurnOutcome::Continue, DisperseTurnMotion::None, "disperse_frame_look");
    ++fixture.context.now_us;
    expect(fixture.update(), DisperseTurnOutcome::Complete, DisperseTurnMotion::None);
}

void lowFrameCapsAndTimeoutPrecedeCameraReadiness() {
    for (const int cause : {0, 1, 2}) {
        Fixture fixture;
        fixture.frameCheck();
        fixture.context.camera_ready = false;
        if (cause == 0) fixture.observation.heading_rad += fixture.tuning.disperse_turn_frame_extra_rad;
        if (cause == 1) {
            fixture.context.swing_plan_active = true;
            fixture.context.cap_positive = .75f;
            fixture.observation.heading_rad = .625f;
        }
        if (cause == 2) fixture.context.now_us += fixture.tuning.disperse_swing_budget_us + 1;
        expect(fixture.update(), DisperseTurnOutcome::Complete, DisperseTurnMotion::None);
    }
}

void clearConfirmationPreservesPhaseDirectionAndLowFrameDeduplication() {
    Fixture fixture;
    fixture.context.fallback_direction = -1;
    fixture.tuning.disperse_turn_frame_check = true;
    fixture.observation.clutter_body.clear();
    fixture.start();
    fixture.observation.heading_rad = -.5f;
    fixture.update();
    CHECK(fixture.stage.needsFrameView());
    const auto first = fixture.update();
    CHECK(first.frame_event && first.confirmations == 1);
    fixture.stage.clearConfirmation();
    CHECK(fixture.stage.needsFrameView());
    const auto duplicate = fixture.update();
    expect(duplicate, DisperseTurnOutcome::Continue, DisperseTurnMotion::Turn, "disperse_turn_frame");
    CHECK(!duplicate.frame_event && duplicate.direction == -1);
    ++fixture.context.now_us;
    fixture.fresh();
    const auto first_after_clear = fixture.update();
    CHECK(first_after_clear.frame_event && first_after_clear.confirmations == 1);
    expect(first_after_clear, DisperseTurnOutcome::Continue, DisperseTurnMotion::None, "disperse_frame_confirm");
    ++fixture.context.now_us;
    fixture.fresh();
    expect(fixture.update(), DisperseTurnOutcome::Complete, DisperseTurnMotion::None);
}

void oneTickCanStartBothTurningAndPushingWithoutGivingAMotionOrReason() {
    Fixture fixture;
    fixture.tuning.disperse_turn_push_m = .25f;
    fixture.context.swing_plan_active = true;
    fixture.context.cap_positive = 0;
    fixture.context.travel_m = 10;
    fixture.observation.heading_rad = 3;
    const auto result = fixture.start();
    expect(result, DisperseTurnOutcome::Continue, DisperseTurnMotion::None);
    CHECK(result.turn_started && result.push_started && fixture.stage.phase() == DisperseTurnPhase::Push);
    CHECK(fixture.context.sweep_yaw == 3 && fixture.context.sweep_travel_m == 10);
    CHECK(fixture.context.sweep_start_us == fixture.context.now_us);
}

void pushDistanceIsInclusiveAndTheBudgetIsExclusive() {
    for (const bool distance : {false, true}) {
        Fixture fixture;
        fixture.tuning.disperse_turn_push_m = .25f;
        fixture.context.motion_ratio = 2;
        fixture.context.swing_plan_active = true;
        fixture.context.cap_positive = 0;
        fixture.context.travel_m = 10;
        const auto armed = fixture.start();
        CHECK(armed.push_started);
        fixture.context.camera_ready = false;
        fixture.observation.heading_rad = std::numeric_limits<float>::quiet_NaN();
        if (distance) fixture.context.travel_m += .499f;
        else fixture.context.now_us += fixture.tuning.disperse_turn_push_budget_us;
        const auto pushing = fixture.update();
        expect(pushing, DisperseTurnOutcome::Continue, DisperseTurnMotion::Push, "disperse_turn_push");
        CHECK(!pushing.turn_started && !pushing.push_started);
        if (distance) fixture.context.travel_m = 10.5f;
        else ++fixture.context.now_us;
        expect(fixture.update(), DisperseTurnOutcome::Complete, DisperseTurnMotion::None);
        CHECK(!fixture.stage.active());
    }
}

void wrappedHeadingUsesTheShortAngleForBothDirections() {
    for (const float direction : {-1.f, 1.f}) {
        Fixture fixture;
        fixture.context.fallback_direction = direction;
        fixture.tuning.disperse_turn_clear_frames = 1;
        fixture.observation.heading_rad = direction * (float(CV_PI) - .1f);
        fixture.start();
        fixture.observation.heading_rad = direction * (-float(CV_PI) + .45f);
        fixture.observation.clutter_body.clear();
        expect(fixture.update(), DisperseTurnOutcome::Complete, DisperseTurnMotion::None);
    }
}

void zeroSentinelAndUnsignedClockRollbackKeepTheirExistingBehavior() {
    Fixture zero;
    zero.context.now_us = 0;
    zero.observation.clutter_ts_us = 0;
    CHECK(!zero.update().turn_started);
    zero.context.now_us = 500;
    CHECK(!zero.update().turn_started);
    zero.context.now_us = 1499;
    CHECK(!zero.update().turn_started);
    zero.context.now_us = 1500;
    CHECK(zero.update().turn_started);

    Fixture look_rollback;
    look_rollback.observation.clutter_ts_us = 0;
    look_rollback.update();
    --look_rollback.context.now_us;
    CHECK(look_rollback.update().turn_started);

    Fixture turn_rollback;
    turn_rollback.start();
    --turn_rollback.context.now_us;
    expect(turn_rollback.update(), DisperseTurnOutcome::Complete, DisperseTurnMotion::None);
}
} // namespace

int main() {
    try {
        lifecycleAndFreshLookDeadline();
        beginRestartsConfirmationHistory();
        directionUsesEmptierSideThenTheFallback();
        lookRequiresAnAtOrAfterReadyFreshNonfutureFrame();
        viewGeometryKeepsInclusiveOuterBoundsAndSkipsInvalidPoints();
        repeatedFreshFramesCountDuringTurnAndStaleFramesPreserveHistory();
        crowdedFreshFramesResetTurnConfirmation();
        minimumAngleAndPlannedCapsUseTheirOriginalBoundaries();
        turnTimeoutIsExclusiveAndLeavesTheCompletionReasonEmpty();
        frameChecksRequireMidPitchAndDeduplicateTimestamps();
        lowFrameGeometryHasInclusiveSidesAndFrontButExcludesTheOrigin();
        lowFramesMustBeAtOrAfterReadinessFreshAndNotFromTheFuture();
        lowFrameFirstObservationTimeoutIsInclusive();
        lowFrameCapsAndTimeoutPrecedeCameraReadiness();
        clearConfirmationPreservesPhaseDirectionAndLowFrameDeduplication();
        oneTickCanStartBothTurningAndPushingWithoutGivingAMotionOrReason();
        pushDistanceIsInclusiveAndTheBudgetIsExclusive();
        wrappedHeadingUsesTheShortAngleForBothDirections();
        zeroSentinelAndUnsignedClockRollbackKeepTheirExistingBehavior();
        std::cout << "disperse turn stage: 19 test groups passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
