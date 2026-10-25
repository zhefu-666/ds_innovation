#include "rescue/task/release_retreat_stage.hpp"
#include "rescue/task/push_task.hpp"
#include <cmath>

namespace rescue {
void ReleaseRetreatStage::beginReleaseBackOff(float hold_y) {
    beginRepushBackOff(hold_y);
    inside_count_ = 0;
}
void ReleaseRetreatStage::beginRepushBackOff(float hold_y) {
    prepush_ = true;
    start_hold_y_ = hold_y;
}
void ReleaseRetreatStage::clearTrip() {
    prepush_ = false;
    inside_count_ = 0;
    inside_timestamp_us_ = 0;
}

ReleaseRetreatResult ReleaseRetreatStage::update(const TaskTuning& t,
                                                 const ReleaseRetreatContext& context,
                                                 const ReleasedCargoEvidence& cargo) {
    ReleaseRetreatResult result;
    const auto finish = [&](ReleaseRetreatOutcome outcome, const char* reason) {
        result.outcome = outcome;
        result.reason = reason;
        return result;
    };
    if (prepush_) {
        const bool inside = context.repush_count == 0 && cargo.visible && cargo.fixed &&
                            !cargo.clipped && cargo.rear_y >= t.assume_cargo_done_y_m &&
                            std::abs(cargo.rear_x - context.half_x) <= t.assume_cargo_x_tol_m;
        if (!inside)
            inside_count_ = 0;
        else if (cargo.timestamp_us != inside_timestamp_us_) {
            ++inside_count_;
            inside_timestamp_us_ = cargo.timestamp_us;
        }
        if (inside && inside_count_ >= t.assume_inside_after_release_obs) {
            prepush_ = false;
            return finish(ReleaseRetreatOutcome::InsideAfterRelease,
                          "assumed_inside_after_release");
        }
        const bool done = context.anchored_zone
                              ? start_hold_y_ - context.hold_y >= context.back_target_m
                              : context.travel_m >= context.back_target_m * context.motion_ratio;
        // Legacy pre-push timeout proceeds to the push, rather than faulting the retreat.
        if (done || context.elapsed_us > t.retreat_budget_us) {
            prepush_ = false;
            return finish(ReleaseRetreatOutcome::PushIn, "assumed_release_then_push");
        }
        result.reverse = true;
        result.reason =
            context.anchored_zone ? "assumed_release_back_off" : "assumed_release_back_off_blind";
        return result;
    }
    if ((t.assume_all_safe || t.field_carry) && (context.assume_blind || !context.observed_zone)) {
        // TEMP_ASSUMPTION: bounded straight reverse leaves the released load behind.
        result.blind_back_out = true;
        if (context.travel_m >= t.assume_back_m)
            return finish(ReleaseRetreatOutcome::VerifyDelivery, "assumed_backed_out");
        if (context.elapsed_us > t.retreat_budget_us)
            return finish(ReleaseRetreatOutcome::Fault, "retreat_blocked");
        result.reverse = true;
        result.reason = "assumed_blind_back_out";
        return result;
    }
    if (!context.observed_zone) {
        if (context.elapsed_us > t.retreat_budget_us)
            return finish(ReleaseRetreatOutcome::Fault, "retreat_reference_lost");
        return result;
    }
    if (context.mouth_clear)
        return finish(ReleaseRetreatOutcome::VerifyDelivery, "backed_out");
    if (context.elapsed_us > t.retreat_budget_us)
        return finish(ReleaseRetreatOutcome::Fault, "retreat_blocked");
    result.reverse = true;
    return result;
}
} // namespace rescue
