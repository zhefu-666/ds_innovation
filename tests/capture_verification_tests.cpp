#include "rescue/task/capture_verification_stage.hpp"
#include "rescue/push_task.hpp"

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace rescue;

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
    }
}
#define CHECK(expression) check((expression), #expression, __LINE__)

Inventory inventory(int ordinary, int core = 0, int injured = 0,
                    int dangerous = 0, int unknown = 0) {
    Inventory value;
    value.ordinary = ordinary;
    value.core = core;
    value.injured = injured;
    value.dangerous = dangerous;
    value.unknown = unknown;
    return value;
}

struct Fixture {
    CaptureVerificationStage stage;
    TaskTuning tuning;
    PushObservation observation;
    std::string label = "ordinary_supply";
    Inventory pending = inventory(1);
    bool first = false;

    CaptureVerificationResult update(uint64_t elapsed_us) {
        observation.now_us = 10000000 + elapsed_us;
        return stage.update(observation, tuning, {elapsed_us, label, pending, first});
    }

    void field() {
        tuning.field_rush = true;
        tuning.verify_held_contents = true;
    }

    void held(const Inventory& value,
              std::vector<PushObservation::HeldObject> objects = {}) {
        observation.hold_observable = true;
        observation.captured = true;
        observation.held_complete = true;
        observation.held = value;
        observation.held_objects = std::move(objects);
    }

    void multiview(const Inventory& value, int verdict = 1) {
        observation.multi_view_finished = true;
        observation.multi_view_verdict = verdict;
        observation.multi_view_inventory = value;
    }

    void weakCore(int track_id = 99, float confidence = .54f) {
        held(inventory(1, 1), {{7, "ordinary_supply", .9f},
                               {track_id, "core_supply", confidence}});
    }
};

void expect(const CaptureVerificationResult& result, CaptureVerificationOutcome outcome,
            const std::string& reason = "", bool blacklist = false) {
    CHECK(result.outcome == outcome);
    CHECK(result.reason == reason);
    CHECK(result.blacklist_target == blacklist);
}

void strictMultiviewBoundaries() {
    for (const uint64_t elapsed : {7999999ULL, 8000000ULL, 8000001ULL}) {
        Fixture missing;
        const auto absent = missing.update(elapsed);
        if (elapsed <= 8000000) {
            expect(absent, CaptureVerificationOutcome::Continue);
            CHECK(!absent.verdict.has_value());
        } else {
            expect(absent, CaptureVerificationOutcome::SafeStop, "multi_view_unconfirmed");
        }

        Fixture finished;
        finished.multiview(finished.pending);
        const auto result = finished.update(elapsed);
        if (elapsed <= 8000000) {
            expect(result, CaptureVerificationOutcome::Verified, "multi_view_capture_verified");
            CHECK(result.inventory == finished.pending);
            CHECK(result.verdict == RuleVerdict::OK);
        } else {
            expect(result, CaptureVerificationOutcome::SafeStop, "multi_view_unconfirmed");
        }
    }

    for (const int verdict : {0, 2}) {
        Fixture negative;
        negative.multiview(negative.pending, verdict);
        expect(negative.update(1), CaptureVerificationOutcome::SafeStop,
               "multi_view_unconfirmed");
    }
}

void multiviewInventoryMustMatchExactly() {
    Fixture mixed;
    mixed.first = true;
    mixed.pending = inventory(1, 1);
    mixed.multiview(mixed.pending);
    const auto valid = mixed.update(1);
    expect(valid, CaptureVerificationOutcome::Verified, "multi_view_capture_verified");
    CHECK(valid.inventory == mixed.pending);

    Fixture mismatch;
    mismatch.multiview(inventory(2));
    const auto wrong_count = mismatch.update(1);
    expect(wrong_count, CaptureVerificationOutcome::AbortDrop,
           "multi_grab_inventory_mismatch", true);
    CHECK(wrong_count.verdict == RuleVerdict::OK);

    Fixture different_kind;
    different_kind.first = true;
    different_kind.pending = inventory(1, 1);
    different_kind.multiview(inventory(2));
    expect(different_kind.update(1), CaptureVerificationOutcome::AbortDrop,
           "multi_grab_inventory_mismatch", true);

    Fixture core_before_first;
    core_before_first.pending = inventory(0, 1);
    core_before_first.multiview(core_before_first.pending);
    const auto illegal = core_before_first.update(1);
    expect(illegal, CaptureVerificationOutcome::AbortDrop,
           "multi_grab_core_before_first", true);
    CHECK(illegal.verdict == RuleVerdict::CORE_BEFORE_FIRST);

    Fixture empty;
    empty.pending = {};
    empty.multiview({});
    const auto empty_result = empty.update(1);
    expect(empty_result, CaptureVerificationOutcome::AbortDrop, "multi_grab_empty", true);
    CHECK(empty_result.verdict == RuleVerdict::EMPTY);
}

void assumptionModesAndBoundaries() {
    for (const bool all_safe : {false, true}) {
        for (const uint64_t elapsed : {7999999ULL, 8000000ULL, 8000001ULL}) {
            Fixture fixture;
            fixture.tuning.field_rush = !all_safe;
            fixture.tuning.assume_all_safe = all_safe;
            const auto result = fixture.update(elapsed);
            if (elapsed <= 8000000) {
                expect(result, CaptureVerificationOutcome::Continue);
            } else {
                expect(result, CaptureVerificationOutcome::Assumed, "assumed_capture");
                CHECK(result.inventory == fixture.pending);
                CHECK(!result.verdict.has_value());
            }
        }

        for (const int verdict : {0, 2}) {
            Fixture negative;
            negative.tuning.field_rush = !all_safe;
            negative.tuning.assume_all_safe = all_safe;
            negative.multiview({}, verdict);
            const auto result = negative.update(1);
            expect(result, CaptureVerificationOutcome::Assumed, "assumed_capture");
            CHECK(result.inventory == negative.pending);
        }

        Fixture conflict;
        conflict.tuning.field_rush = !all_safe;
        conflict.tuning.assume_all_safe = all_safe;
        conflict.multiview(inventory(2));
        const auto result = conflict.update(1);
        if (all_safe) {
            expect(result, CaptureVerificationOutcome::Assumed, "assumed_capture");
            CHECK(result.inventory == conflict.pending);
        } else {
            expect(result, CaptureVerificationOutcome::AbortDrop,
                   "multi_grab_inventory_mismatch", true);
        }
    }

    Fixture carry_only;
    carry_only.tuning.field_carry = true;
    expect(carry_only.update(8000001), CaptureVerificationOutcome::SafeStop,
           "multi_view_unconfirmed");

    Fixture valid;
    valid.field();
    valid.multiview(valid.pending);
    expect(valid.update(1), CaptureVerificationOutcome::Verified,
           "multi_view_capture_verified");

    for (const std::string label : {"ordinary_supply", "core_supply", "injured_person"}) {
        Fixture fallback;
        fallback.tuning.field_rush = true;
        fallback.label = label;
        fallback.pending = {};
        fallback.first = true;
        Inventory expected;
        expected.add(label);
        const auto result = fallback.update(8000001);
        expect(result, CaptureVerificationOutcome::Assumed, "assumed_capture");
        CHECK(result.inventory == expected);
    }
}

void assumptionUsesAccumulatedHeldEvidence() {
    Fixture unseen;
    unseen.field();
    expect(unseen.update(8000001), CaptureVerificationOutcome::Assumed, "assumed_capture");

    Fixture observed_empty;
    observed_empty.field();
    observed_empty.held({});
    expect(observed_empty.update(8000001), CaptureVerificationOutcome::CaptureFail,
           "capture_target_not_in_frame", true);

    Fixture disabled;
    disabled.tuning.field_rush = true;
    disabled.held({});
    expect(disabled.update(8000001), CaptureVerificationOutcome::Assumed, "assumed_capture");

    Fixture once_seen;
    once_seen.field();
    once_seen.held(inventory(1));
    expect(once_seen.update(1), CaptureVerificationOutcome::Continue);
    once_seen.held({});
    expect(once_seen.update(8000001), CaptureVerificationOutcome::Assumed, "assumed_capture");

    Fixture unknown;
    unknown.field();
    unknown.held(inventory(0, 0, 0, 0, 1));
    expect(unknown.update(8000001), CaptureVerificationOutcome::Assumed, "assumed_capture");

    Fixture other_kind;
    other_kind.field();
    other_kind.first = true;
    other_kind.held(inventory(0, 1));
    expect(other_kind.update(8000001), CaptureVerificationOutcome::CaptureFail,
           "capture_target_not_in_frame", true);
}

void promptIllegalContents() {
    struct Case {
        Inventory held;
        bool first;
        RuleVerdict verdict;
    };
    const std::vector<Case> cases = {
        {inventory(1, 0, 0, 1), false, RuleVerdict::DANGEROUS},
        {inventory(1, 1), false, RuleVerdict::CORE_BEFORE_FIRST},
        {inventory(1, 0, 1), false, RuleVerdict::INJURED_BEFORE_FIRST},
        {inventory(1, 0, 1), true, RuleVerdict::INJURED_NOT_ALONE},
        {inventory(3), true, RuleVerdict::TOO_MANY_SUPPLIES},
    };
    for (const auto& test : cases) {
        Fixture fixture;
        fixture.field();
        fixture.first = test.first;
        fixture.held(test.held);
        expect(fixture.update(1), CaptureVerificationOutcome::Continue);
        const auto result = fixture.update(2);
        expect(result, CaptureVerificationOutcome::AbortDrop,
               std::string("held_") + verdictName(test.verdict), true);
        CHECK(result.verdict == test.verdict);
    }

    Fixture disabled_recheck;
    disabled_recheck.field();
    disabled_recheck.tuning.held_recheck = false;
    disabled_recheck.weakCore();
    expect(disabled_recheck.update(1), CaptureVerificationOutcome::Continue);
    expect(disabled_recheck.update(2), CaptureVerificationOutcome::AbortDrop,
           "held_core_before_first", true);

    Fixture untracked;
    untracked.field();
    untracked.weakCore(-1, .99f);
    expect(untracked.update(1), CaptureVerificationOutcome::Continue);
    expect(untracked.update(2), CaptureVerificationOutcome::AbortDrop,
           "held_core_before_first", true);

    Fixture interrupted;
    interrupted.field();
    interrupted.held(inventory(1, 0, 0, 1));
    expect(interrupted.update(1), CaptureVerificationOutcome::Continue);
    interrupted.held(inventory(1));
    expect(interrupted.update(2), CaptureVerificationOutcome::Continue);
    interrupted.held(inventory(1, 0, 0, 1));
    expect(interrupted.update(3), CaptureVerificationOutcome::Continue);
    expect(interrupted.update(4), CaptureVerificationOutcome::AbortDrop, "held_dangerous", true);
}

void recheckConfirmationThresholds() {
    Fixture weak;
    weak.field();
    weak.weakCore();
    for (uint64_t frame = 1; frame < 4; ++frame) {
        expect(weak.update(frame), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    }
    const auto weak_result = weak.update(4);
    expect(weak_result, CaptureVerificationOutcome::AbortDrop, "held_core_before_first", true);
    CHECK(weak_result.verdict == RuleVerdict::CORE_BEFORE_FIRST);

    Fixture strong;
    strong.field();
    strong.weakCore(99, .75f);
    expect(strong.update(1), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    expect(strong.update(2), CaptureVerificationOutcome::AbortDrop,
           "held_core_before_first", true);

    Fixture below_threshold;
    below_threshold.field();
    below_threshold.weakCore(99, .749f);
    for (uint64_t frame = 1; frame < 4; ++frame) {
        expect(below_threshold.update(frame), CaptureVerificationOutcome::Continue,
               "held_illegal_recheck");
    }
    expect(below_threshold.update(4), CaptureVerificationOutcome::AbortDrop,
           "held_core_before_first", true);

    Fixture strong_interrupted;
    strong_interrupted.field();
    strong_interrupted.tuning.held_recheck_confirm_frames = 10;
    strong_interrupted.weakCore(99, .9f);
    expect(strong_interrupted.update(1), CaptureVerificationOutcome::Continue,
           "held_illegal_recheck");
    strong_interrupted.weakCore(99, .54f);
    expect(strong_interrupted.update(2), CaptureVerificationOutcome::Continue,
           "held_illegal_recheck");
    strong_interrupted.weakCore(99, .9f);
    expect(strong_interrupted.update(3), CaptureVerificationOutcome::Continue,
           "held_illegal_recheck");
    expect(strong_interrupted.update(4), CaptureVerificationOutcome::AbortDrop,
           "held_core_before_first", true);

    Fixture highest_confidence;
    highest_confidence.field();
    highest_confidence.held(inventory(0, 2), {{90, "core_supply", .54f},
                                            {99, "core_supply", .9f}});
    expect(highest_confidence.update(1), CaptureVerificationOutcome::Continue,
           "held_illegal_recheck");
    expect(highest_confidence.update(2), CaptureVerificationOutcome::AbortDrop,
           "held_core_before_first", true);

    for (const bool first : {false, true}) {
        Fixture injured;
        injured.field();
        injured.first = first;
        injured.held(inventory(1, 0, 1), {{99, "injured_person", .9f}});
        expect(injured.update(1), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
        expect(injured.update(2), CaptureVerificationOutcome::AbortDrop,
               first ? "held_injured_not_alone" : "held_injured_before_first", true);
    }
}

void recheckIdentityAndTimeBoundaries() {
    Fixture identity;
    identity.field();
    identity.weakCore(90);
    for (uint64_t frame = 1; frame <= 3; ++frame) {
        expect(identity.update(frame), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    }
    identity.weakCore(91);
    for (uint64_t frame = 4; frame <= 6; ++frame) {
        expect(identity.update(frame), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    }
    expect(identity.update(7), CaptureVerificationOutcome::AbortDrop,
           "held_core_before_first", true);

    Fixture strong_identity;
    strong_identity.field();
    strong_identity.weakCore(90, .9f);
    expect(strong_identity.update(1), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    strong_identity.weakCore(91, .9f);
    expect(strong_identity.update(2), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    expect(strong_identity.update(3), CaptureVerificationOutcome::AbortDrop,
           "held_core_before_first", true);

    Fixture timer;
    timer.field();
    timer.weakCore(90);
    expect(timer.update(500000), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    timer.weakCore(91);
    expect(timer.update(1499999), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    timer.weakCore(92);
    const auto expired = timer.update(1500000);
    expect(expired, CaptureVerificationOutcome::CaptureFail, "held_illegal_unresolved", true);
    CHECK(!expired.verdict.has_value());
}

void recheckClearsOnlyAfterConsecutiveClearFrames() {
    Fixture fixture;
    fixture.field();
    fixture.weakCore();
    expect(fixture.update(1), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    fixture.held(inventory(1));
    fixture.multiview({}, 2);
    for (uint64_t frame = 2; frame <= 3; ++frame) {
        CHECK(fixture.update(frame).outcome == CaptureVerificationOutcome::Continue);
    }
    expect(fixture.update(4), CaptureVerificationOutcome::Assumed, "assumed_capture");

    Fixture interrupted;
    interrupted.field();
    interrupted.weakCore(90);
    expect(interrupted.update(1), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    interrupted.held(inventory(1));
    interrupted.multiview({}, 2);
    CHECK(interrupted.update(2).outcome == CaptureVerificationOutcome::Continue);
    interrupted.weakCore(91);
    expect(interrupted.update(3), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    interrupted.held(inventory(1));
    CHECK(interrupted.update(4).outcome == CaptureVerificationOutcome::Continue);
    CHECK(interrupted.update(5).outcome == CaptureVerificationOutcome::Continue);
    expect(interrupted.update(6), CaptureVerificationOutcome::Assumed, "assumed_capture");
}

void recheckPreservesExistingBranchOrder() {
    Fixture unseen;
    unseen.field();
    unseen.weakCore();
    expect(unseen.update(1), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    unseen.observation.hold_observable = false;
    CHECK(unseen.update(8000001).outcome == CaptureVerificationOutcome::Continue);
    unseen.held(inventory(1));
    expect(unseen.update(8000002), CaptureVerificationOutcome::CaptureFail,
           "held_illegal_unresolved", true);

    Fixture verified;
    verified.field();
    verified.weakCore();
    expect(verified.update(1), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    verified.observation.hold_observable = false;
    verified.multiview(verified.pending);
    expect(verified.update(2), CaptureVerificationOutcome::Verified, "multi_view_capture_verified");

    Fixture dangerous;
    dangerous.field();
    dangerous.weakCore();
    expect(dangerous.update(1), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    dangerous.held(inventory(1, 0, 0, 1));
    expect(dangerous.update(2), CaptureVerificationOutcome::AbortDrop, "held_dangerous", true);
}

void singleViewBoundariesAndIllegalEvidence() {
    for (const uint64_t elapsed : {1499999ULL, 1500000ULL, 1500001ULL}) {
        Fixture missing;
        missing.tuning.require_multi_view = false;
        const auto absent = missing.update(elapsed);
        if (elapsed <= missing.tuning.verify_budget_us) {
            expect(absent, CaptureVerificationOutcome::Continue);
        } else {
            expect(absent, CaptureVerificationOutcome::CaptureFail, "capture_unverified");
        }

        Fixture finished;
        finished.tuning.require_multi_view = false;
        finished.tuning.capture_frames = 1;
        finished.held(finished.pending);
        const auto result = finished.update(elapsed);
        if (elapsed <= finished.tuning.verify_budget_us) {
            expect(result, CaptureVerificationOutcome::Verified, "capture_verified");
            CHECK(result.inventory == finished.pending);
            CHECK(result.verdict == RuleVerdict::OK);
        } else {
            expect(result, CaptureVerificationOutcome::CaptureFail, "capture_unverified");
        }
    }

    Fixture illegal;
    illegal.tuning.require_multi_view = false;
    illegal.held(inventory(1, 0, 0, 1));
    const auto result = illegal.update(1);
    expect(result, CaptureVerificationOutcome::AbortDrop, "held_dangerous", true);
    CHECK(result.verdict == RuleVerdict::DANGEROUS);

    Fixture timed_out;
    timed_out.tuning.require_multi_view = false;
    timed_out.held(inventory(1, 0, 0, 1));
    expect(timed_out.update(timed_out.tuning.verify_budget_us + 1),
           CaptureVerificationOutcome::CaptureFail, "capture_unverified");
}

void singleViewNeedsConsecutiveCompleteEvidence() {
    for (int interrupted_gate = 0; interrupted_gate < 5; ++interrupted_gate) {
        Fixture fixture;
        fixture.tuning.require_multi_view = false;
        fixture.tuning.capture_frames = 3;
        fixture.held(fixture.pending);
        expect(fixture.update(1), CaptureVerificationOutcome::Continue);
        switch (interrupted_gate) {
        case 0: fixture.observation.hold_observable = false; break;
        case 1: fixture.observation.captured = false; break;
        case 2: fixture.observation.held_complete = false; break;
        case 3: fixture.observation.held = {}; break;
        case 4: fixture.observation.held = inventory(2); break;
        }
        expect(fixture.update(2), CaptureVerificationOutcome::Continue);
        fixture.held(fixture.pending);
        expect(fixture.update(3), CaptureVerificationOutcome::Continue);
        expect(fixture.update(4), CaptureVerificationOutcome::Continue);
        expect(fixture.update(5), CaptureVerificationOutcome::Verified, "capture_verified");
    }

    for (int missing_gate = 0; missing_gate < 3; ++missing_gate) {
        Fixture fixture;
        fixture.tuning.require_multi_view = false;
        fixture.held(inventory(1, 0, 0, 1));
        switch (missing_gate) {
        case 0: fixture.observation.hold_observable = false; break;
        case 1: fixture.observation.captured = false; break;
        case 2: fixture.observation.held_complete = false; break;
        }
        expect(fixture.update(1), CaptureVerificationOutcome::Continue);
    }

    Fixture empty_pending;
    empty_pending.tuning.require_multi_view = false;
    empty_pending.tuning.capture_frames = 1;
    empty_pending.pending = {};
    empty_pending.held({});
    expect(empty_pending.update(1), CaptureVerificationOutcome::Continue);
}

void resetClearsStageHistory() {
    Fixture confirmations;
    confirmations.tuning.require_multi_view = false;
    confirmations.tuning.capture_frames = 3;
    confirmations.held(confirmations.pending);
    expect(confirmations.update(1), CaptureVerificationOutcome::Continue);
    expect(confirmations.update(2), CaptureVerificationOutcome::Continue);
    confirmations.stage.reset();
    expect(confirmations.update(3), CaptureVerificationOutcome::Continue);
    expect(confirmations.update(4), CaptureVerificationOutcome::Continue);
    expect(confirmations.update(5), CaptureVerificationOutcome::Verified, "capture_verified");

    Fixture recheck;
    recheck.field();
    recheck.weakCore(99, .9f);
    expect(recheck.update(1), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    recheck.stage.reset();
    expect(recheck.update(2000000), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    expect(recheck.update(2000001), CaptureVerificationOutcome::AbortDrop,
           "held_core_before_first", true);

    Fixture bad_frames;
    bad_frames.field();
    bad_frames.held(inventory(1, 0, 0, 1));
    expect(bad_frames.update(1), CaptureVerificationOutcome::Continue);
    bad_frames.stage.reset();
    expect(bad_frames.update(2), CaptureVerificationOutcome::Continue);
    expect(bad_frames.update(3), CaptureVerificationOutcome::AbortDrop, "held_dangerous", true);

    Fixture observed_empty;
    observed_empty.field();
    observed_empty.held({});
    expect(observed_empty.update(1), CaptureVerificationOutcome::Continue);
    observed_empty.stage.reset();
    observed_empty.observation.hold_observable = false;
    expect(observed_empty.update(8000001), CaptureVerificationOutcome::Assumed, "assumed_capture");

    Fixture label_history;
    label_history.field();
    label_history.held(label_history.pending);
    expect(label_history.update(1), CaptureVerificationOutcome::Continue);
    label_history.stage.reset();
    label_history.held({});
    expect(label_history.update(8000001), CaptureVerificationOutcome::CaptureFail,
           "capture_target_not_in_frame", true);

    Fixture active_recheck;
    active_recheck.field();
    active_recheck.weakCore();
    expect(active_recheck.update(1), CaptureVerificationOutcome::Continue, "held_illegal_recheck");
    active_recheck.stage.reset();
    active_recheck.observation.hold_observable = false;
    expect(active_recheck.update(8000001), CaptureVerificationOutcome::Assumed, "assumed_capture");
}
} // namespace

int main() {
    try {
        strictMultiviewBoundaries();
        multiviewInventoryMustMatchExactly();
        assumptionModesAndBoundaries();
        assumptionUsesAccumulatedHeldEvidence();
        promptIllegalContents();
        recheckConfirmationThresholds();
        recheckIdentityAndTimeBoundaries();
        recheckClearsOnlyAfterConsecutiveClearFrames();
        recheckPreservesExistingBranchOrder();
        singleViewBoundariesAndIllegalEvidence();
        singleViewNeedsConsecutiveCompleteEvidence();
        resetClearsStageHistory();
        std::cout << "capture verification tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "capture verification tests failed: " << error.what() << '\n';
        return 1;
    }
}
