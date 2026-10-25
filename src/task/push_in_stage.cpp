#include "rescue/task/push_in_stage.hpp"
#include "rescue/push_task.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace rescue {
void PushInStage::prepare(bool align) {
    align_pending_ = align;
    align_start_us_ = 0;
    push_hold_valid_ = false;
}
void PushInStage::clearTrip() {
    prepare(false);
    enter_unconfirmed_ = false;
    probe_until_us_ = probe_next_us_ = 0;
    probe_sign_ = 1.f;
}

PushInResult PushInStage::update(const TaskTuning& t,
                                 const PushInContext& context,
                                 const PushInCargoEvidence& cargo) {
    PushInResult result;
    const uint64_t now = context.now_us, elapsed = context.elapsed_us;
    const bool cvis = cargo.visible;
    const auto cargoEvent = [&](const char* event, bool visible) {
        result.events.push_back({PushInEventKind::Cargo, event, cargo.rear, visible});
    };
    const auto dropEvent = [&](const char* event) {
        result.events.push_back({PushInEventKind::DropZone, event, context.hold, false});
    };
    const auto motion = [&](float vx, float wz, const char* reason) {
        result.drive = true;
        result.vx = vx;
        result.wz = wz;
        result.reason = reason;
        return result;
    };
    bool done = elapsed > t.enter_budget_us;
    // Missing anchored depth historically permits an independent visual inside fix.
    const float hy0 = context.anchored_zone ? context.hold.y : 1e9f;
    const bool cinside = cvis && cargo.fixed && !cargo.clipped &&
                         cargo.rear.y >= t.assume_cargo_done_y_m &&
                         std::abs(cargo.rear.x - context.half_x) <= t.assume_cargo_x_tol_m &&
                         hy0 >= t.assume_inside_min_hold_y_m;
    const bool placed = cvis && cargo.fixed && !cargo.clipped;
    const bool blocked_deep =
        !cinside && !placed && context.imu_blocked_us >= t.assume_imu_block_us &&
        context.anchored_zone && context.hold.y >= t.assume_blocked_deep_hold_y_m;
    const bool aligning = align_pending_;
    result.reset_cargo_reference = aligning;
    const bool stuck = !cinside && !blocked_deep && !aligning &&
                       (context.imu_blocked_us >= t.assume_imu_block_us ||
                        (cvis && !cargo.clipped && cargo.static_since_us &&
                         now - cargo.static_since_us >= t.assume_stuck_us));
    if (cinside) {
        cargoEvent("push_inside", true);
        done = true;
    } else if (blocked_deep) {
        cargoEvent("blocked_deep", cvis);
        done = true;
    } else if (stuck) {
        if (context.repush_count < t.assume_repush_max) {
            cargoEvent("stuck_repush", true);
            result.reset_cargo_reference = true;
            result.outcome = PushInOutcome::Repush;
            result.reason = "assumed_stuck_repush";
            return result;
        }
        cargoEvent("stuck_giveup", true);
        done = true;
    }
    if (align_pending_ && !done) {
        if (!align_start_us_)
            align_start_us_ = now;
        const bool seen = cvis && now - cargo.seen_us <= 400000;
        const float brg =
            seen ? -std::atan2(cargo.body.x, std::max(.08f, cargo.body.y - context.hold_center))
                 : 0.f;
        if (seen && std::abs(brg) > t.assume_align_tol_rad &&
            now - align_start_us_ < t.assume_align_budget_us)
            return motion(
                0,
                std::copysign(std::max(t.min_turn_wz,
                                       std::min(t.assume_align_wz, t.heading_gain * std::abs(brg))),
                              brg),
                "assumed_push_align_turn");
        char b[200];
        std::snprintf(b,
                      sizeof b,
                      "[CARGO_ZONE] event=%s half=%s bearing_deg=%.1f body_x=%.3f body_y=%.3f "
                      "seen=%d waited_ms=%llu\n",
                      !seen                                     ? "push_align_unseen"
                      : std::abs(brg) <= t.assume_align_tol_rad ? "push_aligned"
                                                                : "push_align_giveup",
                      context.half_injured ? "injured" : "supply",
                      brg * 57.2958f,
                      cargo.body.x,
                      cargo.body.y,
                      seen ? 1 : 0,
                      (unsigned long long)((now - align_start_us_) / 1000));
        result.events.push_back({PushInEventKind::Text, b, {}, false});
        align_pending_ = false;
    }
    if (context.anchored_zone) {
        const auto& hold = context.hold;
        done = done || hold.y >= t.assume_push_depth_m + (cvis ? t.assume_overshoot_m : 0.f);
        if (done && !cinside && !blocked_deep &&
            !(stuck && !(context.repush_count < t.assume_repush_max)) && t.assume_enter_confirm) {
            const bool real =
                cvis ? (!cargo.clipped && cargo.rear.y >= t.assume_enter_true_y_m &&
                        std::abs(cargo.rear.x - context.half_x) <= t.assume_cargo_x_tol_m)
                     : hold.y >= t.assume_enter_hold_y_m;
            if (!real && elapsed < t.enter_budget_us + t.assume_enter_extra_us &&
                hold.y < t.assume_enter_max_hold_y_m) {
                done = false;
                if (!enter_unconfirmed_) {
                    enter_unconfirmed_ = true;
                    cargoEvent(cvis ? "push_unconfirmed_visible" : "push_unconfirmed_hidden", cvis);
                    dropEvent("push_unconfirmed");
                }
            } else if (!real)
                cargoEvent("push_unconfirmed_giveup", cvis);
            else if (enter_unconfirmed_)
                cargoEvent("push_confirmed_late", cvis);
        }
        if (!done) {
            const char* reason = now - context.visual_reference_seen_us <= 400000
                                     ? "assumed_push_in_visual"
                                     : "assumed_push_in_dead_reckoning";
            float aim = context.half_x, phid = 0.f;
            if (cvis) {
                const float dx = cargo.rear.x - context.half_x;
                const float dxe =
                    std::copysign(std::max(0.f, std::abs(dx) - t.assume_push_dx_deadband_m), dx);
                const float dy = std::max(.08f, t.assume_goal_depth_m - cargo.rear.y);
                phid = std::clamp(
                    std::atan2(dxe, dy), -t.assume_push_angle_max_rad, t.assume_push_angle_max_rad);
                aim = std::clamp(cargo.rear.x + std::tan(phid) * (cargo.rear.y - hold.y),
                                 context.half_x - .25f,
                                 context.half_x + .25f);
            }
            const float speed =
                std::min(t.max_speed,
                         t.enter_speed * (1.f + t.assume_repush_speed_gain * context.repush_count));
            float err = t.heading_gain * wrapAngle(context.zone_yaw + phid) +
                        t.lateral_gain * (hold.x - aim);
            if (t.assume_align_push) {
                if (cvis && now - cargo.seen_us <= 400000) {
                    const float brg = -std::atan2(
                        cargo.body.x, std::max(.08f, cargo.body.y - context.hold_center));
                    const float slant =
                        std::clamp(phid, -t.assume_slant_max_rad, t.assume_slant_max_rad);
                    push_hold_phi_ = wrapAngle(-context.zone_yaw + brg + slant);
                    push_hold_valid_ = true;
                    err = t.heading_gain * (brg + slant);
                    reason = "assumed_push_in_pursuit";
                } else if (push_hold_valid_) {
                    err = t.heading_gain * wrapAngle(push_hold_phi_ + context.zone_yaw);
                    reason = "assumed_push_in_hold_heading";
                }
            }
            float wz = std::abs(err) <= t.heading_deadband_rad ? 0.f : err;
            if (wz != 0.f)
                probe_until_us_ = 0;
            else if (t.assume_probe && !align_pending_ && elapsed >= t.assume_probe_first_us) {
                // The first pulse flips the sign; ordinary turns retain the next pulse deadline.
                if (now < probe_until_us_)
                    wz = probe_sign_ * t.min_turn_wz;
                else if (now >= probe_next_us_) {
                    probe_sign_ = -probe_sign_;
                    probe_until_us_ = now + t.assume_probe_len_us;
                    probe_next_us_ = probe_until_us_ + t.assume_probe_gap_us;
                    wz = probe_sign_ * t.min_turn_wz;
                }
                if (wz != 0.f)
                    reason = "assumed_push_imu_probe";
            }
            return motion(speed, wz, reason);
        }
    } else {
        done = done || context.travel_m >= t.assume_push_m;
        if (!done)
            return motion(t.enter_speed, 0, "assumed_push_in_blind");
    }
    if (done) {
        if (context.anchored_zone)
            dropEvent("push_done");
        result.outcome = PushInOutcome::Complete;
        result.reason = "assumed_push_in_done";
        return result;
    }
    return motion(t.enter_speed, 0, "assumed_push_in_frame_down");
}
} // namespace rescue
