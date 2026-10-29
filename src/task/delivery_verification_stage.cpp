#include "rescue/task/delivery_verification_stage.hpp"
#include "rescue/task/push_task.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace rescue {
void DeliveryVerificationStage::resetConfirmations() {
    confirmations_ = 0;
}
void DeliveryVerificationStage::resetInventoryEvidence() {
    delta_ = 0;
    frame_ = stable_since_us_ = 0;
}
void DeliveryVerificationStage::clearVisualCounts() {
    inside_ = outside_ = 0;
}
void DeliveryVerificationStage::clearTrip() {
    clearVisualCounts();
    far_view_ = false;
    fix_us_ = 0;
}

DeliveryVerificationResult
DeliveryVerificationStage::update(const PushObservation& in,
                                  const TaskTuning& t,
                                  const DeliveryVerificationContext& context,
                                  const DeliveryCargoEvidence& cargo) {
    DeliveryVerificationResult result;
    const auto noteCargo = [&](const char* event, int repush) {
        char buffer[200];
        std::snprintf(
            buffer,
            sizeof buffer,
            "[CARGO_ZONE] event=%s half=%s rear_x=%.3f rear_y=%.3f fix=%d repush=%d in=%d out=%d\n",
            event,
            context.half_injured ? "injured" : "supply",
            cargo.rear.x,
            cargo.rear.y,
            int(cargo.fixed),
            repush,
            inside_,
            outside_);
        result.events += buffer;
    };
    const uint64_t now = in.now_us;
    const int expected = context.trip.total();
    const bool carry = t.assume_all_safe || t.field_carry;
    const auto& z = in.zone_estimate;
    const int count = context.trip.injured ? in.zone_injured_count : in.zone_supply_count;
    const int other = context.trip.injured ? in.zone_supply_count : in.zone_injured_count;
    const int delta = count - context.baseline;
    // Credit requires a stable increase in the released load's half, with the other half unchanged.
    const bool evidence = context.observed_zone && z.geometry_id == context.geometry_id &&
                          in.zone_inventory_complete && in.zone_counts_valid && count >= 0 &&
                          other >= 0 && other == context.baseline_other && delta > 0 &&
                          delta <= expected && in.gripper_done && in.gripper_feedback_open == 1 &&
                          z.bodyToZone({0, t.mouth_y_m}).y <= t.mouth_clear_y_m;
    if (!evidence) {
        confirmations_ = 0;
        delta_ = 0;
        stable_since_us_ = 0;
    } else if (z.frame_id != frame_) {
        if (delta_ != delta || !stable_since_us_) {
            confirmations_ = 0;
            stable_since_us_ = now;
            delta_ = delta;
        }
        ++confirmations_;
        frame_ = z.frame_id;
    }
    const bool stable = evidence && confirmations_ >= t.delivery_frames && stable_since_us_ &&
                        now - stable_since_us_ >= 300000;
    const bool vfix = in.vref_valid && in.vref_zone.valid && in.vref_points >= 3;
    if (carry && vfix && !fix_us_)
        fix_us_ = now;
    if (carry && !fix_us_ && context.elapsed_us > t.assume_verify_far_after_us)
        far_view_ = true;
    const bool expired = carry
                             ? (fix_us_ ? now - fix_us_ > t.delivery_budget_us
                                        : context.elapsed_us > std::max(t.assume_verify_fix_wait_us,
                                                                        t.delivery_budget_us))
                             : context.elapsed_us > t.delivery_budget_us;
    bool visual_ok = false;
    // TEMP_ASSUMPTION: judge released cargo against a zone fix or the uncalibrated image front
    // edge.
    if (carry && !stable) {
        if (cargo.available && in.cargo_ts_us != cargo_ts_us_) {
            cargo_ts_us_ = in.cargo_ts_us;
            if (cargo.fixed) {
                const bool x_ok = std::abs(cargo.rear.x - context.half_x) <= t.assume_cargo_x_tol_m;
                if (!cargo.clipped && cargo.rear.y >= t.assume_cargo_verify_in_y_m && x_ok) {
                    ++inside_;
                    outside_ = 0;
                } else if (cargo.clipped || !x_ok || cargo.rear.y < t.assume_cargo_verify_out_y_m) {
                    ++outside_;
                    inside_ = 0;
                }
            } else if (cargo.front_valid && cargo.height_px > 1.f) {
                const float fraction = (cargo.front_px - cargo.bottom_px) / cargo.height_px;
                if (!cargo.clipped && fraction >= t.assume_img_inside_frac) {
                    ++inside_;
                    outside_ = 0;
                } else if (cargo.clipped || fraction < t.assume_img_outside_frac) {
                    ++outside_;
                    inside_ = 0;
                }
                char buffer[160];
                std::snprintf(buffer,
                              sizeof buffer,
                              "[CARGO_ZONE] event=verify_img frac=%.2f clipped=%d in=%d out=%d\n",
                              fraction,
                              int(cargo.clipped),
                              inside_,
                              outside_);
                result.events += buffer;
            }
            noteCargo("verify", context.repush_count);
        }
        // The acceptance threshold uses the tracked cargo's prior fix, not this frame's fix.
        visual_ok = inside_ >= (context.tracked_cargo_has_fix ? 3 : t.assume_img_verify_obs);
        if (!visual_ok && outside_ >= t.assume_verify_out_obs && cargo.other_inside) {
            noteCargo("verify_other_inside", context.repush_count);
            visual_ok = true;
        }
        // Jaw delivery ends with open-gripper retreat; never re-close to push released cargo.
        if (!t.pixel_grasp_enabled && !visual_ok && outside_ >= t.assume_verify_out_obs &&
            context.repush_count < t.assume_repush_max) {
            clearTrip();
            noteCargo("verify_outside_repush", context.repush_count + 1);
            result.outcome = DeliveryVerificationOutcome::Repush;
            result.reason = "assumed_verify_repush";
            return result;
        }
    }
    if (!stable && !expired && !visual_ok)
        return result;
    // TEMP_ASSUMPTION: assume_all_safe credits the whole unconfirmed trip.
    const bool assumed = !stable && t.assume_all_safe && expected > 0;
    result.credited = stable ? delta_ : assumed ? expected : 0;
    far_view_ = false;
    fix_us_ = 0;
    result.visual = stable || visual_ok;
    result.reason = assumed ? (visual_ok ? "delivered_visual" : "delivered_unverified")
                    : result.credited == expected ? "delivered"
                    : result.credited             ? "partial_delivery"
                                                  : "delivery_unverified";
    result.outcome = DeliveryVerificationOutcome::Complete;
    return result;
}
} // namespace rescue
