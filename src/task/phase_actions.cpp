#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
namespace rescue {
void PushTask::failStop(TaskTick&, const char* why) {
    fail(why);
    state_ = PushState::SAFE_STOP;
}

void PushTask::scanAfterClear(TaskTick& tick, const char* why) {
    clearTrip();
    if (phase_ == TaskPhase::Search)
        enterSearch(SearchAction::Scan, tick.now, why);
    else
        requestPhase(PhaseOutcome::SearchReady, why);
}

cv::Point2f PushTask::targetBody(TaskTick& tick) {
    if (tick.in.target_center_valid)
        return tick.in.target_center_body;
    return cv::Point2f(-tick.in.distance_m * std::sin(tick.in.heading_error),
                       tick.target_forward_m);
}

void PushTask::drive(TaskTick& tick, float vx, float wz, bool pixel_align) {
    const bool split_reverse = state_ == PushState::ABORT_DROP &&
                               search_phase_.first_trip_split_.pending &&
                               t_.pixel_grasp_enabled && vx < 0;
    const bool turn_blocked = tick.in.directional_clearance_valid && wz != 0 &&
                              !(vx == 0 ? tick.in.turn_safe
                                        : ((vx > 0 || split_reverse) && tick.in.arc_safe));
    if (turn_blocked) {
        // A blocked arc keeps going straight when the straight corridor is verified clear.
        if (!(vx > 0 && tick.forward_ok)) {
            reason_ = "turn_sweep_blocked";
            return;
        }
        wz = 0;
        reason_ = "turn_sweep_blocked_straight";
    }
    if ((vx > 0 && !tick.forward_ok) || (vx < 0 && !tick.in.retreat_safe) ||
        (vx == 0 && wz != 0 && !tick.scan_ok)) {
        if (state_ != PushState::SCAN && state_ != PushState::START_ADVANCE)
            reason_ = vx < 0               ? "retreat_clearance_missing"
                      : !tick.in.path_safe ? "forward_path_blocked"
                                           : "opponent_clearance_missing";
        return; // missing clearance stops the robot; it never substitutes another motion
    }
    const float vx_cap = state_ == PushState::START_ADVANCE
                             ? std::max(t_.max_speed, t_.startup_advance_speed)
                             : t_.max_speed; // only the start-up run may exceed the normal cap
    tick.motion.vx_mps = std::clamp(vx, -vx_cap, vx_cap);
    // Carrying forward uses continuous curvature steering. A minimum turn rate would turn
    // small bearing corrections into alternating pulses after the sender's deceleration ramp.
    // Keep the existing floor for stationary alignment and every other task state.
    if (state_ == PushState::CARRY && !transport_phase_.placing_ && carryLogic() && vx > 0) {
        tick.motion.wz_rps = std::clamp(wz, -t_.max_wz, t_.max_wz);
        return;
    }
    if (pixel_align) {
        // Grasp pixel steering stays proportional. Stationary carry corrections
        // need the effective turn floor; IMU endpoints stop them at the target.
        // Forward carry arcs above remain proportional without a minimum rate.
        const bool carry_turn = state_ == PushState::CARRY && !transport_phase_.placing_ &&
                                carryLogic() && vx == 0;
        const float cap = carry_turn ? t_.transportTurnCap()
                                     : std::min(t_.pixel_align_max_wz, t_.max_wz);
        if (carry_turn && wz!=0)
            wz=std::copysign(std::max(std::abs(wz),std::min(t_.transportTurnFloor(),cap)),wz);
        tick.motion.wz_rps = std::clamp(wz, -cap, cap);
        return;
    }
    // Preserve zero and direction; never let the floor exceed the configured cap.
    const bool demo_capture_turn =
        t_.demo_capture_motion && (state_ == PushState::APPROACH || state_ == PushState::RUSH);
    // 0.25 rad/s does not move the chassis (field-measured): pure in-place turns in demo
    // APPROACH/RUSH use the effective floor, and escalate to max_wz if they persist.
    const bool demo_inplace = demo_capture_turn && vx == 0 && wz != 0;
    float demo_cap = .25f;
    bool escalated = false;
    if (demo_inplace) {
        if (tick.now - inplace_last_us_ > 300000)
            inplace_since_us_ = tick.now;
        inplace_last_us_ = tick.now;
        escalated = tick.now - inplace_since_us_ >= 1000000;
        demo_cap = escalated ? t_.max_wz : std::max(.25f, t_.min_turn_wz);
    }
    const bool startup_pitch_turn = phase_ == TaskPhase::Startup &&
                                    startup_action_ == StartupAction::TurnBeforePitch;
    const float wz_cap = startup_pitch_turn ? 1.f
                         : demo_capture_turn
                             ? std::min(t_.max_wz, demo_cap)
                             : (tick.scan_spin ? std::max(t_.max_wz, t_.scan_wz) : t_.max_wz);
    if (wz != 0)
        wz = std::copysign(
            std::max(std::abs(wz), escalated ? wz_cap : std::min(t_.min_turn_wz, wz_cap)), wz);
    tick.motion.wz_rps = std::clamp(wz, -wz_cap, wz_cap);
    if (vx == 0 && wz != 0 && !demo_capture_turn && t_.inplace_turn_boost > 1.f &&
        (state_ == PushState::APPROACH || state_ == PushState::MID_APPROACH ||
         state_ == PushState::SELECT_CARGO))
        tick.motion.wz_rps *= t_.inplace_turn_boost;
}

bool PushTask::pixelLateralAccepted(const PushObservation& in) const {
    return in.target_grasp_range_valid && in.target_grasp_range_lateral &&
           (in.target_grasp_range_lateral_measured || in.pixel_grasp_aligned);
}

void PushTask::invalidateEntryPosition() {
    search_entry_.first_id = -1;
    search_entry_.confirmed = search_entry_.handoff = false;
    search_entry_.sample_us = search_entry_.accepted_us = 0;
    search_entry_.stage = "observation_wait";
    pixel_alignment_ = {};
}

void PushTask::selectSearchEntry(const PushObservation& in) {
    if (search_entry_.target_id == trip_context_.target_id_ &&
        search_entry_.label == trip_context_.label_)
        return;
    search_entry_ = {};
    search_entry_.target_id = trip_context_.target_id_;
    search_entry_.label = trip_context_.label_;
    (void)in;
}

bool PushTask::entryPositionValid(const PushObservation& in) const {
    return tracking(in) && in.target_id == search_entry_.target_id &&
           in.label == search_entry_.label && search_entry_.first_id == in.target_id &&
           in.pixel_grasp_near_view && in.camera_pitch_stable &&
           in.target_center_px_valid && in.target_grasp_range_valid &&
           in.target_grasp_range_inside && pixelLateralAccepted(in) &&
           in.pixel_grasp_sample_us && in.pixel_grasp_sample_us <= in.now_us &&
           in.now_us - in.pixel_grasp_sample_us <= t_.frame_timeout_us;
}

bool PushTask::locateSearchEntry(TaskTick& tick) {
    selectSearchEntry(tick.in);
    auto& entry = search_entry_;
    const auto& in = tick.in;
    tick.immediate_motion_stop = true;
    if (!tracking(in) || !in.camera_pitch_stable || !in.pixel_grasp_near_view ||
        !in.target_center_px_valid || !in.target_grasp_range_valid ||
        !in.capture_scene_valid || !in.capture_scene_us || in.capture_scene_us > tick.now ||
        tick.now - in.capture_scene_us > t_.frame_timeout_us || in.capture_area_px.empty() ||
        !in.pixel_grasp_sample_us || in.pixel_grasp_sample_us > tick.now ||
        tick.now - in.pixel_grasp_sample_us > t_.frame_timeout_us) {
        invalidateEntryPosition();
        reason_ = "first_entry_observation_wait";
        return false;
    }
    if (in.target_grasp_range_past_rear) {
        enter(PushState::CAPTURE_FAIL, tick.now, "first_entry_past_rear_recovery");
        return false;
    }
    // Centres establish entry order. Whole boxes remain the inventory/path evidence.
    // Latch only the first fresh UI140 scene containing a centre inside its class range.
    if (entry.first_id == -1 && in.capture_scene_us > entry.sample_us) {
        entry.sample_us = in.capture_scene_us;
        int first = -1;
        bool target_seen = false, all_needed = true;
        for (const auto& object : in.capture_image_objects) {
            const auto region = in.capture_class_regions_px.find(object.label);
            const auto area = region == in.capture_class_regions_px.end()
                                  ? in.capture_area_px : region->second;
            const cv::Point2f center{object.box.x + .5f * object.box.width,
                                     object.box.y + .5f * object.box.height};
            const bool inside = center.x >= area.x && center.x <= area.x + area.width &&
                                center.y >= area.y && center.y <= area.y + area.height;
            target_seen |= object.id == in.target_id && object.label == in.label;
            if (!object.complete && (inside || object.id == in.target_id)) {
                reason_ = "first_entry_incomplete_scene";
                return false;
            }
            if (inside) {
                first = first == -1 ? object.id : -2;
                all_needed &= targetSelectable(object.label, first_);
            }
        }
        if (!target_seen) {
            reason_ = "first_entry_target_missing";
            return false;
        }
        // If all entered blocks are selectable this trip, the first is needed too.
        // Existing inventory review still decides whether this batch can close.
        entry.first_id = first == -2 && all_needed && in.target_grasp_range_inside
                             ? in.target_id : first;
    }
    if (entry.first_id != -1 && entry.first_id != in.target_id) {
        entry.stage = "review_first_object";
        reason_ = entry.first_id == -2 ? "first_entry_order_ambiguous" : "first_entry_other_object";
        // Do not push a preceding neighbour farther in. Reuse the existing stopped split.
        if (pixelAlign(tick, true, true)) {
            if (evaluateCaptureScene(in) == CaptureSceneVerdict::Incomplete) {
                reason_ = "first_entry_inventory_incomplete";
                return false;
            }
            beginPixelSplit(tick);
        }
        return false;
    }
    if (!in.target_grasp_range_inside || !pixelLateralAccepted(in)) {
        entry.confirmed = false;
        entry.stage = "approach";
        if (!pixelAlign(tick, false, pixelLateralAccepted(in)))
            return false;
        if (pixelLateralAccepted(in) && in.target_center_px.y < in.capture_area_px.y) {
            reason_ = "first_entry_approach";
            drive(tick, std::min(t_.approach_speed, t_.rush_near_speed), 0);
            tick.immediate_motion_stop = tick.motion.vx_mps == 0;
        }
        return false;
    }
    entry.stage = "stop_confirm";
    entry.confirmed = entryPositionValid(in) && pixelAlign(tick, true, true);
    entry.stage = entry.confirmed ? "target_entered_first" : "stop_confirm";
    reason_ = entry.confirmed ? "first_entry_target_ready" : "first_entry_stop_confirm";
    return entry.confirmed;
}

bool PushTask::pixelAlign(TaskTick& tick,
                          bool strict,
                          bool hold_position,
                          float tolerance_override_px) {
    return pixelAlignTarget(tick, strict, hold_position, tolerance_override_px, pixel_alignment_);
}

bool PushTask::pixelAlignTarget(TaskTick& tick, bool strict, bool hold_position,
                               float tolerance_override_px, PixelAlignment& a) {
    const auto& in = tick.in;
    tick.immediate_motion_stop = true;
    // A handoff or accepted position needs rest, not another turn.
    if (a.holding_position != hold_position) {
        a.stage = PixelAlignment::Stage::Correct;
        a.direction = 0;
        a.holding_position = hold_position;
    }
    const bool far_handoff = !in.pixel_grasp_view && in.target_center_px_valid &&
        (in.pixel_view_handoff_ready ||
         (phase_ == TaskPhase::Startup && startup_action_ == StartupAction::FarHandoff));
    if ((!in.pixel_grasp_target_valid && !far_handoff) || !in.pixel_grasp_sample_us ||
        in.pixel_grasp_sample_us > tick.now ||
        tick.now - in.pixel_grasp_sample_us > t_.frame_timeout_us ||
        !std::isfinite(in.pixel_grasp_error_px)) {
        a = {};
        reason_ = "pixel_target_wait";
        return false;
    }
    const float error = in.pixel_grasp_error_px;
    const float magnitude = std::abs(error);
    const bool approach = !in.pixel_grasp_near_view;
    const float tolerance = tolerance_override_px > 0 ? tolerance_override_px
                            : far_handoff             ? 40.f
                            : approach                ? t_.pixel_align_approach_tolerance_px
                                                      : t_.pixel_align_tolerance_px;
    const float restart = tolerance_override_px > 0 ? 2.f * tolerance_override_px
                          : far_handoff             ? 40.f
                          : approach                ? t_.pixel_align_approach_restart_px
                                                    : t_.pixel_align_restart_px;
    const float hold_limit = approach ? restart : tolerance;
    // Transport UI120 uses its own alignment state; cargo, UI140 and view handoffs
    // keep their rules. A stopped coarse approach may accept the existing hold band.
    const bool carry_approach = &a == &carry_alignment_ &&
        phase_ == TaskPhase::Transport && !transport_phase_.placing_ &&
        approach && in.pixel_grasp_view && !hold_position;
    const float acceptance = carry_approach ? hold_limit : tolerance;
    a.tolerance_px = tolerance;
    a.acceptance_px = acceptance;
    const float sent_vx = in.sent_velocity_valid ? in.sent_vx_mps : last_vx_;
    const float sent_wz = in.sent_velocity_valid ? in.sent_wz_rps : last_wz_;
    // Sent commands are not measured wheel velocities. Fresh stationary images are also
    // required before Advance. Replays use the last task command when no UART exists.
    const bool stopped = std::isfinite(sent_vx) && std::isfinite(sent_wz) &&
                         std::abs(sent_vx) <= t_.pixel_align_stopped_vx &&
                         std::abs(sent_wz) <= t_.pixel_align_stopped_wz;
    const bool fresh = in.pixel_grasp_sample_us > a.last_sample_us;
    const uint64_t sample_delta = fresh && a.last_sample_us
                                     ? in.pixel_grasp_sample_us - a.last_sample_us : 0;
    const bool resumed = a.resume_pending;
    if (fresh && resumed &&
        (sample_delta > 150000 ||
         std::abs(error - a.last_error_px) > t_.pixel_align_brake_motion_px ||
         std::abs(in.target_center_px.y - a.last_center_y_px) > t_.pixel_align_brake_motion_px)) {
        a.stage = PixelAlignment::Stage::Brake;
        a.stopped_since_us = a.stable_since_us = a.correction_since_us = 0;
        a.stable_frames = a.resume_frames = 0;
        a.within_tolerance = false;
        a.brake_reason = "sample_discontinuity";
    }
    // Reuse startup's bounded prediction/pulses for transport UI120 only.
    // Measured cargo bounds and UI140 transport acceptance stay unchanged.
    const bool near_startup = phase_ == TaskPhase::Startup &&
        startup_action_ == StartupAction::OpenApproach && in.pixel_grasp_near_view;
    float closing_px_per_s = 0;
    const bool predictive = near_startup || carry_approach;
    if (predictive && fresh && !resumed && sample_delta >= 10000 && sample_delta <= 150000) {
        const float rate = (error - a.last_error_px) * 1000000.f / float(sample_delta);
        if (rate * error < 0)
            closing_px_per_s = std::abs(rate);
    }
    if (fresh) {
        a.last_sample_us = in.pixel_grasp_sample_us;
        a.last_error_px = error;
        a.last_center_y_px = in.target_center_px.y;
        if (a.resume_pending && ++a.resume_frames >= t_.pixel_align_stable_frames)
            a.resume_pending = false;
    }
    const int direction = error > 0 ? 1 : (error < 0 ? -1 : 0);
    const auto brake = [&](const char* cause) {
        a.brake_reason = cause;
        a.stage = PixelAlignment::Stage::Brake;
        a.stopped_since_us = 0;
        a.correction_since_us = 0;
        a.within_tolerance = hold_position || magnitude <= acceptance;
        a.stable_frames = 0;
        a.stable_since_us = 0;
        reason_ = "pixel_align_brake";
    };
    if (a.stage == PixelAlignment::Stage::Advance) {
        if ((hold_position || magnitude <= (strict ? tolerance : restart)) &&
            (!strict || stopped) && !a.resume_pending)
            return true;
        brake("advance_limit");
        return false;
    }
    if (a.stage == PixelAlignment::Stage::Correct) {
        const bool fine = predictive && magnitude <= 2.f * restart;
        const bool predicted_stop = predictive && closing_px_per_s > 0 &&
            magnitude <= tolerance + std::min(80.f, .15f * closing_px_per_s);
        // A fast chassis may cross the whole fine band before the first delayed
        // response arrives. Bound transport's coarse bursts too, then observe at rest.
        const bool pulse_done = (fine || carry_approach) && a.correction_since_us &&
                                tick.now - a.correction_since_us >= 100000;
        if (hold_position || magnitude <= tolerance || predicted_stop || pulse_done ||
            (a.direction && direction != a.direction) ||
            !std::isfinite(sent_vx) || !std::isfinite(sent_wz) ||
            std::abs(sent_vx) > t_.pixel_align_stopped_vx ||
            (sent_wz * error < 0 && std::abs(sent_wz) > t_.pixel_align_stopped_wz)) {
            brake(hold_position ? "handoff" : magnitude <= tolerance ? "tolerance" :
                  predicted_stop ? "prediction" : pulse_done ? "pulse_end" :
                  (a.direction && direction != a.direction) ? "crossed_center" : "motion_wait");
            return false;
        }
        a.brake_reason = "none";
        reason_ = "pixel_center_align";
        const float wz = t_.pixel_align_gain_rps_per_px * error;
        drive(tick, 0, fine ? std::clamp(.5f * wz, -.1f, .1f) : wz, true);
        if (tick.motion.wz_rps != 0) {
            a.direction = direction;
            if (!a.correction_since_us)
                a.correction_since_us = tick.now;
        }
        return false;
    }
    if (a.stage == PixelAlignment::Stage::Brake) {
        reason_ = "pixel_align_brake";
        if (!stopped) {
            a.stopped_since_us = 0;
            a.stable_frames = 0;
            a.stable_since_us = 0;
            return false;
        }
        if (!a.stopped_since_us)
            a.stopped_since_us = tick.now;
        // A zero UART command is not a stationary chassis. Ignore pre-stop images and
        // require a fresh, low-drift image window before permitting either direction.
        if (!fresh || in.pixel_grasp_sample_us <= a.stopped_since_us)
            return false;
        if (hold_position || magnitude <= acceptance)
            a.within_tolerance = true;
        else if (magnitude > hold_limit)
            a.within_tolerance = false;
        if (!a.stable_frames) {
            a.brake_min_px = a.brake_max_px = error;
            a.brake_min_y_px = a.brake_max_y_px = in.target_center_px.y;
            a.stable_since_us = in.pixel_grasp_sample_us;
        }
        a.brake_min_px = std::min(a.brake_min_px, error);
        a.brake_max_px = std::max(a.brake_max_px, error);
        a.brake_min_y_px = std::min(a.brake_min_y_px, in.target_center_px.y);
        a.brake_max_y_px = std::max(a.brake_max_y_px, in.target_center_px.y);
        if (a.brake_max_px - a.brake_min_px > t_.pixel_align_brake_motion_px ||
            (hold_position && a.brake_max_y_px - a.brake_min_y_px > t_.pixel_align_brake_motion_px)) {
            a.brake_min_px = a.brake_max_px = error;
            a.brake_min_y_px = a.brake_max_y_px = in.target_center_px.y;
            a.stable_since_us = in.pixel_grasp_sample_us;
            a.stable_frames = 0;
        }
        ++a.stable_frames;
        if (a.stable_frames < t_.pixel_align_stable_frames ||
            in.pixel_grasp_sample_us - a.stable_since_us < t_.pixel_align_brake_us)
            return false;
        a.direction = 0;
        a.stable_frames = 0;
        a.stable_since_us = 0;
        a.stage = a.within_tolerance ? PixelAlignment::Stage::Settle
                                     : PixelAlignment::Stage::Correct;
        return false; // retain zero on the decision tick before a new direction
    }
    reason_ = "pixel_align_settle";
    if (!stopped || (!hold_position && magnitude > hold_limit)) {
        brake("settle_limit");
        return false;
    }
    if (fresh) {
        if (hold_position || carry_approach) {
            a.brake_min_px = std::min(a.brake_min_px, error);
            a.brake_max_px = std::max(a.brake_max_px, error);
            a.brake_min_y_px = std::min(a.brake_min_y_px, in.target_center_px.y);
            a.brake_max_y_px = std::max(a.brake_max_y_px, in.target_center_px.y);
            if (a.brake_max_px - a.brake_min_px > t_.pixel_align_brake_motion_px ||
                a.brake_max_y_px - a.brake_min_y_px > t_.pixel_align_brake_motion_px) {
                brake("image_drift");
                return false;
            }
        }
        if (!a.stable_frames)
            a.stable_since_us = in.pixel_grasp_sample_us;
        ++a.stable_frames;
        if (a.stable_frames >= t_.pixel_align_stable_frames &&
            in.pixel_grasp_sample_us - a.stable_since_us >= t_.pixel_align_stable_us &&
            !a.resume_pending) {
            a.stage = PixelAlignment::Stage::Advance;
            return true;
        }
    }
    return false;
}

void PushTask::scanSpin(TaskTick& tick) {
    tick.scan_spin = true;
    if (phase_ == TaskPhase::Transport && state_ == PushState::CARRY &&
        carryLogic() && t_.transport_scan_wz > 0) {
        drive(tick, 0, t_.transport_scan_wz, true);
    } else {
        drive(tick, 0, t_.scan_wz);
    }
    tick.scan_spin = false;
}

CarrySearchContext PushTask::carrySearchContext(TaskTick& tick) {
    CarrySearchContext context;
    context.now_us = tick.now;
    context.heading_valid = tick.in.heading_valid;
    context.heading_rad = tick.in.heading_rad;
    context.zone_mem = zone_mem_;
    context.zone_mem_us = zone_mem_us_;
    context.zone_mem_yaw = zone_mem_yaw_;
    context.run_yaw0_set = run_yaw0_set_;
    context.run_yaw0 = run_yaw0_;
    return context;
}

bool PushTask::unstick(TaskTick& tick) { // true: the wiggle owns this tick's motion
    if (!t_.spin_unstick)
        return false;
    if (!search_phase_.unstick_step_) {
        if (tick.in.imu_spin_stall_us < t_.spin_stall_trigger_us)
            return false;
        search_phase_.unstick_step_ = 1;
        search_phase_.unstick_us_ = tick.now;
    }
    const uint64_t d = tick.now - search_phase_.unstick_us_;
    if (d >= 2 * t_.spin_unstick_leg_us) {
        search_phase_.unstick_step_ = 0;
        return false;
    }
    const bool back = d < t_.spin_unstick_leg_us;
    reason_ = back ? "spin_stall_back" : "spin_stall_forward";
    drive(tick, back ? -t_.spin_unstick_speed : t_.spin_unstick_speed, 0);
    return true;
}

float PushTask::yawHoldWz(TaskTick& tick) { // IMU hold on the last seen target bearing; 0 when
                                            // unavailable or inside the deadband
    if (!t_.rush_yaw_hold || !rush_yaw_ref_set_ || !tick.in.heading_valid ||
        !std::isfinite(tick.in.heading_rad))
        return 0.f;
    const float err = wrapAngle(rush_yaw_ref_ - tick.in.heading_rad);
    return std::abs(err) > t_.startup_hold_deadband_rad
               ? std::clamp(t_.heading_gain * err, -t_.max_wz, t_.max_wz)
               : 0.f;
}

bool PushTask::preLower(TaskTick& tick, const char* why) { // true: lowering may go ahead
    if (!t_.rush_yaw_hold || !rush_yaw_ref_set_ || !tick.in.heading_valid ||
        !std::isfinite(tick.in.heading_rad)) {
        enter(PushState::LOWER_FRAME, tick.now, why);
        return true;
    }
    const float err = wrapAngle(rush_yaw_ref_ - tick.in.heading_rad);
    if (std::abs(err) <= t_.rush_lower_yaw_tol_rad) {
        enter(PushState::LOWER_FRAME, tick.now, why);
        return true;
    }
    if (!capture_phase_.prelower_us_)
        capture_phase_.prelower_us_ = tick.now;
    if (tick.now - capture_phase_.prelower_us_ > t_.prelower_realign_us) {
        enter(PushState::CAPTURE_FAIL, tick.now, "prelower_heading_drift");
        return false;
    }
    reason_ = "prelower_realign";
    drive(tick, 0, std::clamp(t_.heading_gain * err, -t_.rush_realign_wz, t_.rush_realign_wz));
    return false;
}

void PushTask::steer(TaskTick& tick, float speed, float heading, bool blend, float forward_speed) {
    if (std::abs(heading) > t_.rotate_in_place_rad && !blend) {
        drive(tick, 0, t_.heading_gain * heading);
        return;
    }
    float wz = 0;
    if (std::abs(heading) > t_.heading_deadband_rad) {
        if (state_ == PushState::CARRY && !transport_phase_.placing_ && carryLogic()) {
            // Pure-pursuit curvature: slower forward travel needs a smaller yaw rate, not
            // the same high-gain turn used to rotate in place. Bound the gain by the legacy
            // value so a short configured lookahead cannot make steering more aggressive.
            const float gain = std::min(
                t_.heading_gain, 2.f * speed / std::max(.02f, t_.assume_lookahead_m));
            wz = gain * std::sin(heading);
        } else
            wz = t_.heading_gain * heading;
    }
    // An explicit forward speed leaves the steering gain based on the original speed.
    drive(tick, forward_speed >= 0.f ? forward_speed : speed, wz);
}

bool PushTask::carrySearch(TaskTick& tick) {
    const auto carry = transport_phase_.carry_search_.update(t_, carrySearchContext(tick));
    if (carry.reason)
        reason_ = carry.reason;
    switch (carry.action) {
        case CarrySearchAction::Scan:
            scanSpin(tick);
            return false;
        case CarrySearchAction::Steer:
            steer(tick, carry.speed, carry.heading);
            return false;
        case CarrySearchAction::None:
            break;
    }
    return false;
}

const char* PushTask::trackingReason(TaskTick& tick) {
    if (!tick.in.target_valid)
        return "target_missing";
    if (tick.in.target_id != trip_context_.target_id_)
        return "target_id_mismatch";
    if (tick.in.label != trip_context_.label_)
        return "target_label_mismatch";
    if (!tick.in.geometry_valid)
        return "target_geometry_invalid";
    if (!tick.in.target_region_valid)
        return "target_region_unknown";
    if (tick.in.target_in_zone)
        return "target_already_in_zone";
    return "target_distance_or_heading_invalid";
}

bool PushTask::lostTarget(TaskTick&) {
    return ++actionProgress().misses > t_.hold_grace_frames;
}

void PushTask::searchLost(TaskTick& tick) {
    if (frame_raised_ != 0 || trip_context_.trip_.total() > 0) {
        failStop(tick, "lost_search_jaw_or_load_unsafe");
        return;
    }
    clearTrip();
    search_phase_.lost_round_ = 0;
    search_phase_.post_disperse_mid_ = false;
    enter(PushState::LOST_SEARCH, tick.now, "lost_target_search");
}

int PushTask::holdCheck(TaskTick& tick) {
    if (holdSeen(tick.in) == 1) {
        actionProgress().hold_misses = 0;
        return ++actionProgress().confirmations >= t_.hold_check_frames ? 1 : 0;
    }
    actionProgress().confirmations = 0;
    return ++actionProgress().hold_misses > t_.hold_grace_frames ? -1 : 0;
}

} // namespace rescue
