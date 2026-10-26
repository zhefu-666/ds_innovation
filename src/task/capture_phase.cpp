#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
namespace rescue {
void PushTask::updateCapturePhase(TaskTick& tick) {
    switch (capture_phase_.action) {
        case CaptureAction::Advance: {
            // Restoring demo motion does not restore its assumed inventory. Even on a blind
            // completion/budget tick, an observed illegal neighbour must veto enclosure.
            if (t_.demo_capture_motion && !t_.assume_all_safe && tick.in.corridor.total() > 0) {
                const auto v = checkTrip(tick.in.corridor, first_);
                if (v != RuleVerdict::OK) {
                    trip_context_.verdict_ = v;
                    blacklist(tick.now);
                    const std::string why = std::string("rush_corridor_") + verdictName(v);
                    enter(PushState::CAPTURE_FAIL, tick.now, why.c_str());
                    break;
                }
            }
            const auto finishRush = [&](const char* why) {
                if (t_.demo_capture_motion)
                    enter(PushState::LOWER_FRAME, tick.now, why);
                else
                    preLower(tick, why);
            };
            const uint64_t stall_prev_us = capture_phase_.rush_stall_last_us_;
            capture_phase_.rush_stall_last_us_ =
                0; // any tick that does not reach the sample point breaks continuity
            if (capture_phase_.rush_stall_phase_) { // stalled: stationary near-view re-check; the
                                                    // rush bound or a
                                                    // lost target must not credit enclosure here
                reason_ = "rush_stall_reobserve";
                const bool seen = tick.in.camera_pitch_stable && tick.in.hold_observable;
                if (seen)
                    ++capture_phase_.rush_stall_obs_frames_;
                const bool in_frame = seen && tick.in.captured && tick.in.held_complete &&
                                      tick.in.held.count(targetKind(trip_context_.label_)) >= 1 &&
                                      checkTrip(tick.in.held, first_) == RuleVerdict::OK;
                if (in_frame) {
                    preLower(tick, "rush_stall_in_frame_evidence");
                    break;
                }
                const uint64_t waited = tick.now - capture_phase_.rush_stall_obs_us_;
                if ((capture_phase_.rush_stall_obs_frames_ >= t_.rush_stall_reobserve_frames &&
                     waited >= t_.rush_stall_reobserve_us) ||
                    waited >= t_.rush_stall_reobserve_max_us) {
                    blacklist(tick.now);
                    enter(PushState::CAPTURE_FAIL, tick.now, "rush_no_progress");
                }
                break;
            }
            const auto reobserveRush = [&](const char* why) {
                capture_phase_.rush_stall_phase_ = true;
                capture_phase_.rush_stall_obs_us_ = tick.now;
                capture_phase_.rush_stall_obs_frames_ = 0;
                tick.motion.vx_mps = tick.motion.wz_rps = 0;
                reason_ = why;
            };
            if (actionProgress().travel >= actionProgress().travel_limit ||
                tick.elapsed > t_.rush_budget_us) {
                if (t_.precise_rush) {
                    reobserveRush("rush_bound_reobserve");
                    break;
                }
                if (rushLogic()) {
                    finishRush("assumed_enclosure_rush_bound");
                    break;
                }
                enter(PushState::CAPTURE_FAIL, tick.now, "rush_overrun");
                break;
            }
            if (!tracking(tick.in)) {
                if (t_.precise_rush) {
                    reobserveRush("rush_target_lost_reobserve");
                    break;
                }
                if (rushLogic()) {
                    // TEMP_ASSUMPTION: the target leaves the 5deg view below ~0.31 m; finish the
                    // last estimated distance straight, then lower the frame.
                    if (actionProgress().travel >= assume_rush_end_) {
                        finishRush("assumed_enclosure_target_below_view");
                        break;
                    }
                    reason_ = "assumed_blind_rush";
                    drive(tick, t_.rush_speed, t_.demo_capture_motion ? 0.f : yawHoldWz(tick));
                    break;
                }
                if (lostTarget(tick))
                    enter(PushState::CAPTURE_FAIL, tick.now, "rush_target_lost");
                break;
            }
            actionProgress().misses = 0;
            if (std::abs(tick.in.heading_error) > t_.rush_abort_heading_rad) {
                enter(PushState::CAPTURE_FAIL, tick.now, "rush_misaligned");
                break;
            }
            // Recheck before every forward/close decision: a blue object may enter
            // the corridor after PREPARE. Never continue on incomplete evidence.
            if (rushLogic() && !t_.precise_rush) {
                // Demo capture/replay follows the last visible remaining distance; precision mode
                // retains its initial hard bound.
                // 实测指令行程约为实际位移的4~6倍：用视觉剩余距离乘以比例得到所需指令行程。
                assume_rush_end_ = actionProgress().travel +
                                   std::max(0.f, tick.target_forward_m - t_.assume_rush_trigger_m) *
                                       t_.assume_motion_ratio;
                actionProgress().travel_limit =
                    assume_rush_end_ + t_.rush_extra_m * t_.assume_motion_ratio;
            }
            trip_context_.verdict_ = t_.assume_all_safe
                                         ? RuleVerdict::OK
                                         : checkCorridor(tick.in.corridor,
                                                         tick.in.corridor_complete,
                                                         tick.in.corridor_occlusion_free,
                                                         trip_context_.label_,
                                                         first_);
            if (trip_context_.verdict_ != RuleVerdict::OK) {
                reason_ = std::string("rush_corridor_") + verdictName(trip_context_.verdict_);
                blacklist(tick.now);
                enter(PushState::CAPTURE_FAIL, tick.now, "");
                break;
            }
            if (t_.precise_rush && std::abs(tick.in.heading_error) > t_.rush_align_deadband_rad) {
                reason_ = "rush_align_at_rest";
                drive(tick, 0, t_.heading_gain * tick.in.heading_error);
                // Preserve clearance vetoes, but bypass drive's coarse minimum turn command.
                if (tick.motion.wz_rps != 0)
                    tick.motion.wz_rps = std::clamp(t_.heading_gain * tick.in.heading_error,
                                                    -t_.rush_align_wz,
                                                    t_.rush_align_wz);
                break;
            }
            if (tick.target_forward_m <=
                (rushLogic() ? t_.assume_rush_trigger_m : t_.grasp_trigger_y_m)) {
                enter(PushState::LOWER_FRAME, tick.now, "frame_enclosure_position");
                break;
            }
            if (t_.precise_rush) {
                reason_ = tick.target_forward_m <= t_.rush_slow_range_m ? "rush_straight_near"
                                                                        : "rush_straight";
                drive(tick,
                      tick.target_forward_m <= t_.rush_slow_range_m
                          ? std::min(t_.rush_speed, t_.rush_near_speed)
                          : t_.rush_speed,
                      0);
            } else
                drive(tick, t_.rush_speed, t_.heading_gain * tick.in.heading_error);
            if (t_.rush_stall_us > 0) {
                const bool pitch_ok =
                    tick.in.camera_pitch_stable && tick.in.camera_pitch_cdeg != kCameraPitchInvalid;
                const bool continuous =
                    stall_prev_us && tick.now >= stall_prev_us &&
                    tick.now - stall_prev_us <= t_.rush_stall_gap_us &&
                    tick.in.target_id == capture_phase_.rush_stall_id_ &&
                    capture_phase_.rush_stall_pitch_ != kCameraPitchInvalid &&
                    std::abs(int(tick.in.camera_pitch_cdeg) -
                             int(capture_phase_.rush_stall_pitch_)) <= t_.pitch_tolerance_cdeg;
                if (!pitch_ok || tick.motion.vx_mps <= 0 || !continuous ||
                    tick.in.distance_m <
                        capture_phase_.rush_stall_ref_m_ - t_.rush_stall_progress_m) {
                    // Not a valid no-progress sample (stale/other object/camera moving/no forward
                    // command) or real closing: restart the window.
                    capture_phase_.rush_stall_ref_m_ = tick.in.distance_m;
                    capture_phase_.rush_stall_ref_us_ = tick.now;
                    capture_phase_.rush_stall_samples_ = 0;
                } else
                    ++capture_phase_.rush_stall_samples_;
                capture_phase_.rush_stall_id_ = tick.in.target_id;
                capture_phase_.rush_stall_pitch_ =
                    pitch_ok ? tick.in.camera_pitch_cdeg : kCameraPitchInvalid;
                capture_phase_.rush_stall_last_us_ = pitch_ok ? tick.now : 0;
                if (capture_phase_.rush_stall_samples_ >= t_.rush_stall_min_samples &&
                    tick.now - capture_phase_.rush_stall_ref_us_ >= t_.rush_stall_us) {
                    capture_phase_.rush_stall_phase_ = true;
                    capture_phase_.rush_stall_obs_us_ = tick.now;
                    capture_phase_.rush_stall_obs_frames_ = 0;
                    tick.motion.vx_mps = tick.motion.wz_rps = 0;
                    reason_ = "rush_no_progress_stop";
                }
            }
            break;
        }
        case CaptureAction::LowerFrame: {
            commandFrame(0, tick.now);
            const int g = frameWait(tick.in, 0, tick.now);
            if (g < 0) {
                failStop(tick, "frame_close_timeout");
            } else if (g > 0)
                enter(PushState::VERIFY_CAPTURE, tick.now, "frame_lowered");
            break;
        }
        case CaptureAction::Verify: {
            const auto verification = capture_phase_.capture_verification_.update(
                tick.in, t_, {tick.elapsed, trip_context_.label_, trip_context_.pending_, first_});
            if (verification.verdict)
                trip_context_.verdict_ = *verification.verdict;
            if (!verification.reason.empty())
                reason_ = verification.reason;
            if (verification.blacklist_target)
                blacklist(tick.now);
            switch (verification.outcome) {
                case CaptureVerificationOutcome::Continue:
                    break;
                case CaptureVerificationOutcome::Verified:
                case CaptureVerificationOutcome::Assumed:
                    capture_phase_.evidence =
                        verification.outcome == CaptureVerificationOutcome::Verified
                            ? CaptureEvidence::Verified
                            : CaptureEvidence::Assumed;
                    trip_context_.trip_ = verification.inventory;
                    transport_phase_.carry_us_ = tick.now;
                    transport_phase_.zone_search_started_us_ = 0;
                    if (verification.outcome == CaptureVerificationOutcome::Assumed)
                        transport_phase_.carry_approach_.reset();
                    requestPhase(PhaseOutcome::TransportReady, "");
                    transport_phase_.hold_seen_us_ = tick.now;
                    transport_phase_.carried_ = 0;
                    break;
                case CaptureVerificationOutcome::CaptureFail:
                    enter(PushState::CAPTURE_FAIL, tick.now, "");
                    break;
                case CaptureVerificationOutcome::AbortDrop:
                    enter(PushState::ABORT_DROP, tick.now, "");
                    break;
                case CaptureVerificationOutcome::SafeStop:
                    failStop(tick, verification.reason.c_str());
                    break;
            }
            break;
        }
        default:
            break;
    }
}
} // namespace rescue
