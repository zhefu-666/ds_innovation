#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
namespace rescue {
bool PushTask::updateCaptureViewHandoff(TaskTick& tick) {
    auto& handoff = capture_phase_.view_handoff_;
    const auto& in = tick.in;
    tick.immediate_motion_stop = true;
    const int pitch = pitchWait(in, tick.now);
    if (pitch < 0) {
        failStop(tick, "camera_pitch_timeout");
        return true;
    }
    if (pitch > 0 && !handoff.confirmation_started_us)
        handoff.confirmation_started_us = tick.now;
    const bool scene_fresh = in.capture_scene_valid && in.capture_scene_us &&
        in.capture_scene_us <= tick.now && tick.now-in.capture_scene_us <= t_.frame_timeout_us;
    const auto object = std::find_if(in.capture_image_objects.begin(), in.capture_image_objects.end(),
        [&](const auto& item) {
            return item.id == in.target_id && item.label == trip_context_.label_ &&
                   !item.box.empty();
        });
    const bool near_target = pitch > 0 && in.pixel_grasp_near_view && candidate(in,tick.now) &&
        (!in.pixel_selection_managed || in.pixel_locked) && in.target_id == trip_context_.target_id_ &&
        object != in.capture_image_objects.end() && scene_fresh &&
        in.capture_scene_us > handoff.started_us && in.pixel_grasp_sample_us > handoff.started_us &&
        in.pixel_grasp_sample_us <= tick.now &&
        tick.now-in.pixel_grasp_sample_us <= t_.frame_timeout_us;
    if (handoff.confirmation_started_us &&
        tick.now-handoff.confirmation_started_us >= 1500000 &&
        !(near_target && captureReviewStopped(in) && in.target_grasp_range_valid &&
          in.target_grasp_range_inside && !in.target_grasp_range_past_rear)) {
        // Return to the coarse view within Capture. The class, lock episode and
        // total approach deadline survive; a fresh selector may replace its ID.
        capture_phase_.pixel_handoff_retry_height_ = handoff.retry_height;
        handoff = {};
        capture_phase_.prepare_pitch_ = t_.track_pitch_cdeg;
        pixel_alignment_ = {};
        search_entry_ = {};
        capture_scene_ = {};
        reason_ = "capture_view_retry_120";
        return true;
    }
    if (!pitch || !scene_fresh || !captureReviewStopped(in) || !near_target) {
        handoff.frames = 0;
        handoff.sample_us = handoff.stable_us = 0;
        handoff.near_box = {};
        handoff.near_label.clear();
        reason_ = !pitch ? "camera_pitch_wait" : "pixel_capture_handoff_confirm";
        return true;
    }
    if (in.capture_scene_us <= handoff.sample_us)
        return true;
    const double overlap = (handoff.near_box & object->box).area();
    const double smaller = std::min(handoff.near_box.area(), object->box.area());
    if (handoff.near_label != in.label || smaller <= 0 || overlap < .5 * smaller ||
        (handoff.sample_us && in.capture_scene_us - handoff.sample_us > t_.frame_timeout_us))
    {
        handoff.frames = 0;
        handoff.sample_us = handoff.stable_us = 0;
    }
    handoff.near_box = object->box;
    handoff.near_label = in.label;
    if (!handoff.frames)
        handoff.stable_us = in.capture_scene_us;
    handoff.sample_us = in.capture_scene_us;
    if (++handoff.frames < t_.pixel_align_stable_frames ||
        in.capture_scene_us - handoff.stable_us < t_.pixel_align_stable_us)
        return true;
    handoff = {};
    capture_phase_.pixel_handoff_retry_height_ = 0;
    actionProgress().started_us = tick.now; // entry budget begins after view acquisition
    reason_ = "pixel_capture_handoff_ready";
    return true; // Capture performs entry positioning and inventory review next tick
}

void PushTask::updateCapturePhase(TaskTick& tick) {
    switch (capture_phase_.action) {
        case CaptureAction::Prepare: {
            if (search_phase_.pixel_approach_started_us_ &&
                tick.now-search_phase_.pixel_approach_started_us_ > t_.approach_budget_us) {
                enter(PushState::CAPTURE_FAIL,tick.now,"pixel_approach_timeout");
                break;
            }
            if (capture_phase_.prepare_pitch_ == t_.track_pitch_cdeg) {
                if (capture_phase_.view_handoff_.started_us) {
                    updateSearchViewHandoff(tick);
                    break;
                }
                if (!tracking(tick.in)) {
                    tick.immediate_motion_stop = true;
                    reason_ = "capture_retry_target_wait";
                    break;
                }
                if (updateSearchViewHandoff(tick))
                    break;
                if (!captureReviewStopped(tick.in) && !pixel_alignment_.last_sample_us) {
                    tick.immediate_motion_stop = true;
                    reason_ = "capture_retry_stop_wait";
                    break;
                }
                if (!pixelAlign(tick)) break;
                commandFrame(1,tick.now);
                const int frame = frameWait(tick.in,1,tick.now);
                if (frame < 0) {
                    failStop(tick,"gripper_open_timeout");
                    break;
                }
                if (!frame) {
                    reason_ = "gripper_feedback_wait";
                    break;
                }
                reason_ = "capture_retry_approach_120";
                drive(tick,.10f,0);
                break;
            }
            const int p = pitchWait(tick.in, tick.now);
            if (p < 0) {
                failStop(tick, "camera_pitch_timeout");
                break;
            }
            if (p == 0) {
                reason_ = "camera_pitch_wait";
                break;
            }
            commandFrame(1, tick.now);
            const int g = frameWait(tick.in, 1, tick.now);
            if (g < 0) {
                failStop(tick, "gripper_open_timeout");
                break;
            }
            if (g == 0) {
                reason_ = "gripper_feedback_wait";
                break;
            }
            if (tick.elapsed > t_.prepare_budget_us) {
                enter(PushState::CAPTURE_FAIL, tick.now, "pixel_corridor_unresolved");
                break;
            }
            if (!tracking(tick.in)) {
                reason_ = "pixel_target_wait";
                actionProgress().confirmations = 0;
                break;
            }
            if (search_entry_.review_before_entry) {
                if (reviewFirstTripCapture(tick, "startup_reselected") ||
                    search_phase_.first_trip_split_.pending)
                    search_entry_.review_before_entry = false;
                break;
            }
            if (!locateSearchEntry(tick))
                break;
            // Inventory review starts only AFTER the selected target entered first; mixed boxes may enter.
            if (!reviewFirstTripCapture(tick, "search_positioned"))
                break;
            trip_context_.verdict_ = checkCorridor(tick.in.corridor,
                                                   tick.in.corridor_complete,
                                                   tick.in.corridor_occlusion_free,
                                                   trip_context_.label_,
                                                   first_);
            if (trip_context_.verdict_ == RuleVerdict::INCOMPLETE) {
                actionProgress().confirmations = 0;
                reason_ = "pixel_corridor_incomplete";
                break;
            }
            if (trip_context_.verdict_ != RuleVerdict::OK) {
                beginPixelSplit(tick);
                break;
            }
            if (trip_context_.seen_ != tick.in.corridor) {
                trip_context_.seen_ = tick.in.corridor;
                actionProgress().confirmations = 0;
            }
            if (++actionProgress().confirmations < t_.corridor_confirm_frames)
                break;
            trip_context_.pending_ = tick.in.corridor;
            search_entry_.handoff = true;
            search_entry_.accepted_us = tick.in.pixel_grasp_sample_us;
            enter(PushState::RUSH, tick.now, "pixel_first_entry_inventory_ok");
            break;
        }

        case CaptureAction::Advance: {
            if (t_.pixel_grasp_enabled && search_entry_.handoff) {
                tick.immediate_motion_stop = true;
                if (!entryPositionValid(tick.in)) {
                    invalidateEntryPosition();
                    enter(PushState::PREPARE, tick.now, "entry_handoff_position_invalid");
                    break;
                }
                if (tick.in.pixel_grasp_sample_us <= search_entry_.accepted_us) {
                    reason_ = "entry_handoff_new_image_wait";
                    break;
                }
                trip_context_.verdict_ = checkCorridor(tick.in.corridor,
                    tick.in.corridor_complete, tick.in.corridor_occlusion_free,
                    trip_context_.label_, first_);
                if (trip_context_.verdict_ != RuleVerdict::OK) {
                    invalidateEntryPosition();
                    enter(PushState::PREPARE, tick.now, "entry_handoff_inventory_recheck");
                    break;
                }
                if (!pixelAlign(tick, true, true))
                    break;
                enter(PushState::LOWER_FRAME, tick.now, "entry_handoff_enclosure_position");
                break;
            }
            if (t_.pixel_grasp_enabled) {
                if (tick.elapsed > t_.rush_budget_us) {
                    enter(PushState::CAPTURE_FAIL, tick.now, "pixel_capture_timeout");
                    break;
                }
                if (!tick.in.pixel_grasp_near_view || !tracking(tick.in) ||
                    !tick.in.target_grasp_range_valid) {
                    reason_ = "pixel_target_wait";
                    break;
                }
                if (tick.in.target_grasp_range_past_rear) {
                    enter(PushState::CAPTURE_FAIL, tick.now, "target_outside_grasp_center_range");
                    break;
                }
                trip_context_.verdict_ = checkCorridor(tick.in.corridor,
                                                       tick.in.corridor_complete,
                                                       tick.in.corridor_occlusion_free,
                                                       trip_context_.label_,
                                                       first_);
                if (trip_context_.verdict_ != RuleVerdict::OK) {
                    enter(PushState::CAPTURE_FAIL, tick.now, "pixel_corridor_rejected");
                    break;
                }
                const bool accepted_lateral = pixelLateralAccepted(tick.in);
                const bool capture_ready = pixelCaptureReady(tick.in);
                if (!pixelAlign(tick,
                                capture_ready,
                                accepted_lateral && (tick.in.target_grasp_range_lateral_measured ||
                                                     capture_ready)))
                    break;
                if (capture_ready && pixelLateralAccepted(tick.in)) {
                    enter(PushState::LOWER_FRAME, tick.now,
                          tick.in.pixel_grasp_ready ? "pixel_center_enclosure_position"
                                                   : "first_trip_split_enclosure_position");
                    break;
                }
                reason_ = "pixel_center_approach";
                drive(tick, std::min(t_.rush_speed, t_.rush_near_speed), 0);
                if (tick.motion.vx_mps > 0) {
                    capture_phase_.pixel_forward_us_ = tick.in.pixel_grasp_sample_us;
                    capture_phase_.pixel_forward_pitch_ = tick.in.camera_pitch_cdeg;
                }
                break;
            }
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
            // At the sampled view, a visible target outside the side/rear bounds
            // must not be advanced or enclosed by a distance/budget fallback.
            if (tick.in.target_grasp_range_applicable &&
                (!tick.in.target_grasp_range_valid || !tick.in.target_grasp_range_lateral ||
                 tick.in.target_grasp_range_past_rear)) {
                enter(PushState::CAPTURE_FAIL, tick.now, "target_outside_grasp_center_range");
                break;
            }
            const auto finishRush = [&](const char* why) {
                if (tick.in.target_grasp_range_applicable && !tick.in.target_grasp_range_inside) {
                    enter(PushState::CAPTURE_FAIL, tick.now, "target_outside_grasp_center_range");
                    return;
                }
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
                if (in_frame &&
                    (!tick.in.target_grasp_range_applicable || tick.in.target_grasp_range_inside)) {
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
                    (rushLogic() ? t_.assume_rush_trigger_m : t_.grasp_trigger_y_m) &&
                (!tick.in.target_grasp_range_applicable || tick.in.target_grasp_range_inside)) {
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
            auto preclose_inventory = tick.in.corridor;
            if (startup_first_capture_)
                preclose_inventory.ordinary = std::min(preclose_inventory.ordinary, 1);
            if (frame_raised_ != 0 && search_entry_.handoff && !entryPositionValid(tick.in)) {
                tick.immediate_motion_stop = true;
                invalidateEntryPosition();
                enter(PushState::PREPARE, tick.now, "entry_preclose_position_invalid");
                break;
            }
            if (t_.pixel_grasp_enabled && frame_raised_ != 0 &&
                (!tick.in.pixel_grasp_near_view || !tracking(tick.in) ||
                 (search_entry_.handoff ? !entryPositionValid(tick.in)
                                        : (!pixelCaptureReady(tick.in) || !pixelLateralAccepted(tick.in))) ||
                 checkCorridor(preclose_inventory,
                               tick.in.corridor_complete,
                               tick.in.corridor_occlusion_free,
                               trip_context_.label_,
                               first_) != RuleVerdict::OK)) {
                enter(PushState::CAPTURE_FAIL, tick.now, "pixel_preclose_recheck_failed");
                break;
            }
            if (frame_raised_ != 0 && tick.in.target_grasp_range_applicable &&
                (!tick.in.target_grasp_range_valid || !tick.in.target_grasp_range_inside)) {
                enter(PushState::CAPTURE_FAIL, tick.now, "target_outside_grasp_center_range");
                break;
            }
            if (t_.pixel_grasp_enabled && frame_raised_ != 0 && !pixelAlign(tick, true, true))
                break;
            commandFrame(0, tick.now);
            const int g = frameWait(tick.in, 0, tick.now);
            if (g < 0) {
                failStop(tick, "frame_close_timeout");
            } else if (g > 0)
                enter(PushState::VERIFY_CAPTURE, tick.now, "frame_lowered");
            break;
        }
        case CaptureAction::Verify: {
            tick.immediate_motion_stop = true;
            capture_scene_.stage = "postclose";
            final_validation_ = "running";
            const bool pixel_review =
                t_.pixel_grasp_enabled && targetSelectable(trip_context_.label_, first_);
            if (pixel_review) {
                // Closure has finished. Verify the actual jaw contents using fresh
                // post-close images; approach-path neighbours no longer veto transport.
                const auto sample = tick.in.capture_scene_us;
                const bool fresh_image =
                    tick.in.camera_pitch_stable && tick.in.pixel_grasp_near_view &&
                    tick.in.capture_scene_valid && sample > tick.now - tick.elapsed &&
                    sample > capture_scene_.confirm_us && sample <= tick.now &&
                    tick.now - sample <= t_.frame_timeout_us;
                if (!fresh_image) {
                    reason_ = "first_trip_postclose_held_image_wait";
                    if (tick.elapsed > t_.verify_budget_us) {
                        final_validation_ = "failed";
                        enter(PushState::CAPTURE_FAIL, tick.now, reason_.c_str());
                    }
                    break;
                }
                capture_scene_.confirm_us = capture_scene_.sample_us = sample;
                capture_scene_.blocker_id = -1;
                capture_scene_.blocker_box = {};
                capture_scene_.association.clear();
                if (capture_load_.active && tick.in.hold_observable && tick.in.held_complete &&
                    tick.in.held != capture_load_.choice.inventory) {
                    final_validation_ = "failed";
                    enter(PushState::ABORT_DROP, tick.now, "capture_score_inventory_mismatch");
                    search_phase_.first_trip_split_.capture_retry = true;
                    break;
                }
            }
            auto verification = capture_phase_.capture_verification_.update(
                tick.in, t_, {tick.elapsed, trip_context_.label_, trip_context_.pending_, first_});
            if (capture_load_.active && verification.outcome == CaptureVerificationOutcome::Verified &&
                verification.inventory != capture_load_.choice.inventory) {
                verification.outcome = CaptureVerificationOutcome::AbortDrop;
                verification.reason = "capture_score_inventory_mismatch";
            }
            if (verification.verdict)
                trip_context_.verdict_ = *verification.verdict;
            if (!verification.reason.empty())
                reason_ = verification.reason;
            auto& split = search_phase_.first_trip_split_;
            // Post-close counts never grant pixel reverse permission. They report a failed
            // capture; any retry must return through stationary scene review.
            if (pixel_review && verification.outcome == CaptureVerificationOutcome::Assumed) {
                verification.outcome = tick.elapsed > t_.verify_budget_us
                    ? CaptureVerificationOutcome::CaptureFail : CaptureVerificationOutcome::Continue;
                verification.reason = "first_trip_actual_count_unverified";
                reason_ = verification.reason;
            }
            if (pixel_review) {
                capture_scene_.inventory = tick.in.held;
                const auto held_verdict = checkTrip(tick.in.held, first_);
                const bool illegal_held =
                    held_verdict != RuleVerdict::OK && held_verdict != RuleVerdict::EMPTY;
                capture_scene_.status =
                    verification.outcome == CaptureVerificationOutcome::Verified ? "clear"
                    : illegal_held || verification.outcome == CaptureVerificationOutcome::AbortDrop
                        ? "risk"
                        : "incomplete";
                capture_scene_.detail = !verification.reason.empty() ? verification.reason
                                        : illegal_held
                                            ? std::string("held_") + verdictName(held_verdict)
                                            : "held_inventory_unverified";
            }
            if (verification.outcome == CaptureVerificationOutcome::Verified)
                final_validation_ = "passed";
            else if (verification.outcome != CaptureVerificationOutcome::Continue)
                final_validation_ = "failed";
            if (verification.blacklist_target &&
                !(pixel_review && verification.outcome == CaptureVerificationOutcome::AbortDrop))
                blacklist(tick.now);
            switch (verification.outcome) {
                case CaptureVerificationOutcome::Continue:
                    break;
                case CaptureVerificationOutcome::Verified:
                case CaptureVerificationOutcome::Assumed: {
                    capture_phase_.evidence =
                        verification.outcome == CaptureVerificationOutcome::Verified
                            ? CaptureEvidence::Verified
                            : CaptureEvidence::Assumed;
                    const int split_runs = split.runs, corner_runs = split.corner_runs;
                    split = {}; // capture never credits first ordinary delivery
                    split.runs = split_runs;
                    split.corner_runs = corner_runs;
                    trip_context_.trip_ = verification.inventory;
                    trip_context_.transported_objects.clear();
                    trip_context_.transported_view_pitch = tick.in.camera_pitch_cdeg;
                    for (const auto& object : tick.in.capture_image_objects)
                        if (object.complete && tick.in.captureInventoryOverlaps(object.box, object.label) &&
                            verification.inventory.count(targetKind(object.label)) > 0)
                            trip_context_.transported_objects.push_back(object);
                    capture_load_.active = false;
                    transport_phase_.carry_us_ = tick.now;
                    transport_phase_.zone_search_started_us_ = 0;
                    if (verification.outcome == CaptureVerificationOutcome::Assumed)
                        transport_phase_.carry_approach_.reset();
                    requestPhase(PhaseOutcome::TransportReady, "");
                    transport_phase_.hold_seen_us_ = tick.now;
                    transport_phase_.carried_ = 0;
                    break;
                }
                case CaptureVerificationOutcome::CaptureFail:
                    enter(PushState::CAPTURE_FAIL, tick.now, "");
                    break;
                case CaptureVerificationOutcome::AbortDrop:
                    enter(PushState::ABORT_DROP, tick.now, "");
                    if (pixel_review)
                        split.capture_retry = true;
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
