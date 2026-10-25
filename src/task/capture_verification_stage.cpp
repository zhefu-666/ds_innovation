#include "rescue/task/capture_verification_stage.hpp"
#include "rescue/task/push_task.hpp"

namespace rescue {
void CaptureVerificationStage::reset() {
    *this = CaptureVerificationStage{};
}

CaptureVerificationResult CaptureVerificationStage::update(
    const PushObservation& in, const TaskTuning& t, const CaptureVerificationContext& context) {
    CaptureVerificationResult result;
    const auto finish = [&](CaptureVerificationOutcome outcome,
                            const std::string& reason = "",
                            bool blacklist = false) {
        result.outcome = outcome;
        if (!reason.empty())
            result.reason = reason;
        result.blacklist_target = blacklist;
        return result;
    };
    const auto reject = [&](RuleVerdict verdict, const char* prefix) {
        result.verdict = verdict;
        return finish(CaptureVerificationOutcome::AbortDrop,
                      std::string(prefix) + verdictName(verdict),
                      true);
    };
    const bool rush = t.assume_all_safe || t.field_rush;
    const uint64_t now = in.now_us;
    const auto& pending = context.pending;
    const bool first = context.first_ordinary_delivered;
    if (t.require_multi_view) {
        if (rush && t.verify_held_contents && in.hold_observable) {
            ++observed_frames_;
            if (in.held.count(targetKind(context.label)) > 0 || in.held.unknown > 0)
                ++label_frames_;
            const auto verdict = checkTrip(in.held, first);
            const bool bad = verdict == RuleVerdict::DANGEROUS ||
                             verdict == RuleVerdict::INJURED_BEFORE_FIRST ||
                             verdict == RuleVerdict::INJURED_NOT_ALONE ||
                             verdict == RuleVerdict::CORE_BEFORE_FIRST ||
                             verdict == RuleVerdict::TOO_MANY_SUPPLIES;
            bad_frames_ = bad ? bad_frames_ + 1 : 0;
            if (bad)
                bad_verdict_ = verdict;
            // Only core/injured class conflicts may wait for identity and class stability.
            const TargetKind bad_kind = verdict == RuleVerdict::CORE_BEFORE_FIRST ? TargetKind::CORE
                                        : (verdict == RuleVerdict::INJURED_BEFORE_FIRST ||
                                           verdict == RuleVerdict::INJURED_NOT_ALONE)
                                            ? TargetKind::INJURED
                                            : TargetKind::UNKNOWN;
            const PushObservation::HeldObject* bad_object = nullptr;
            if (bad && bad_kind != TargetKind::UNKNOWN)
                for (const auto& object : in.held_objects)
                    if (targetKind(object.label) == bad_kind &&
                        (!bad_object || object.confidence > bad_object->confidence))
                        bad_object = &object;
            const bool recheck = t.held_recheck && bad_object && bad_object->track_id >= 0;
            if (recheck) {
                recheck_clear_ = 0;
                if (!recheck_active_) {
                    recheck_active_ = true;
                    recheck_started_us_ = now;
                    recheck_track_ = -1;
                    recheck_frames_ = recheck_strong_ = 0;
                }
                if (bad_object->track_id == recheck_track_) {
                    ++recheck_frames_;
                    recheck_strong_ = bad_object->confidence >= t.held_recheck_strong_conf
                                          ? recheck_strong_ + 1
                                          : 0;
                } else {
                    recheck_track_ = bad_object->track_id;
                    recheck_frames_ = 1;
                    recheck_strong_ = bad_object->confidence >= t.held_recheck_strong_conf ? 1 : 0;
                }
                result.reason = "held_illegal_recheck";
                if (recheck_frames_ >= t.held_recheck_confirm_frames ||
                    recheck_strong_ >= t.held_recheck_strong_frames)
                    return reject(verdict, "held_");
            } else if (recheck_active_ && !bad) {
                if (++recheck_clear_ >= t.held_recheck_clear_frames) {
                    recheck_active_ = false;
                    recheck_track_ = -1;
                    recheck_frames_ = recheck_strong_ = 0;
                }
            }
            if (recheck_active_ && now - recheck_started_us_ >= t.held_recheck_us)
                return finish(
                    CaptureVerificationOutcome::CaptureFail, "held_illegal_unresolved", true);
            if (!recheck && bad_frames_ >= 2)
                return reject(bad_verdict_, "held_");
        }
        const bool verified = in.multi_view_finished && in.multi_view_verdict == 1 &&
                              checkTrip(in.multi_view_inventory, first) == RuleVerdict::OK &&
                              in.multi_view_inventory == pending;
        const bool conflict = in.multi_view_finished && in.multi_view_verdict == 1 &&
                              (checkTrip(in.multi_view_inventory, first) != RuleVerdict::OK ||
                               in.multi_view_inventory != pending);
        if (rush && !verified && (t.assume_all_safe || !conflict)) {
            if (recheck_active_)
                return result;
            // Preserve the legacy fallback timing and target-presence evidence gate.
            if (context.elapsed_us <= 8000000 && !in.multi_view_finished)
                return result;
            if (observed_frames_ > 0 && label_frames_ == 0)
                return finish(
                    CaptureVerificationOutcome::CaptureFail, "capture_target_not_in_frame", true);
            result.inventory = pending;
            if (!result.inventory.total())
                result.inventory.add(context.label);
            return finish(CaptureVerificationOutcome::Assumed, "assumed_capture");
        }
        if (context.elapsed_us > 8000000 || (in.multi_view_finished && in.multi_view_verdict != 1))
            return finish(CaptureVerificationOutcome::SafeStop, "multi_view_unconfirmed");
        if (!in.multi_view_finished)
            return result;
        result.verdict = checkTrip(in.multi_view_inventory, first);
        if (*result.verdict != RuleVerdict::OK || in.multi_view_inventory != pending) {
            const auto reason = *result.verdict != RuleVerdict::OK
                                    ? std::string("multi_grab_") + verdictName(*result.verdict)
                                    : "multi_grab_inventory_mismatch";
            return finish(CaptureVerificationOutcome::AbortDrop, reason, true);
        }
        result.inventory = in.multi_view_inventory;
        return finish(CaptureVerificationOutcome::Verified, "multi_view_capture_verified");
    }

    if (context.elapsed_us > t.verify_budget_us)
        return finish(CaptureVerificationOutcome::CaptureFail, "capture_unverified");
    if (in.hold_observable && in.captured && in.held_complete) {
        const auto verdict = checkTrip(in.held, first);
        if (verdict != RuleVerdict::OK && verdict != RuleVerdict::EMPTY)
            return reject(verdict, "held_");
    }
    if (!in.hold_observable || !in.captured || !in.held_complete || in.held.total() == 0 ||
        in.held != pending) {
        confirmations_ = 0;
        return result;
    }
    if (confirmations_ == 0 || in.held != seen_) {
        seen_ = in.held;
        confirmations_ = 0;
    }
    if (++confirmations_ < t.capture_frames)
        return result;
    result.verdict = checkTrip(in.held, first);
    if (*result.verdict != RuleVerdict::OK)
        return reject(*result.verdict, "held_");
    result.inventory = in.held;
    return finish(CaptureVerificationOutcome::Verified, "capture_verified");
}
} // namespace rescue
