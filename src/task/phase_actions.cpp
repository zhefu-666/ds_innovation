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
    if (pixel_align) {
        // Preserve proportional pixel steering; no legacy floor, boost or escalation.
        const float cap = std::min(t_.pixel_align_max_wz, t_.max_wz);
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
    const float wz_cap = demo_capture_turn
                             ? std::min(t_.max_wz, demo_cap)
                             : (tick.scan_spin ? std::max(t_.max_wz, t_.scan_wz) : t_.max_wz);
    if (wz != 0)
        wz = std::copysign(
            std::max(std::abs(wz), escalated ? wz_cap : std::min(t_.min_turn_wz, wz_cap)), wz);
    tick.motion.wz_rps = std::clamp(wz, -wz_cap, wz_cap);
    if (vx == 0 && wz != 0 && !demo_capture_turn && t_.inplace_turn_boost > 1.f &&
        (state_ == PushState::APPROACH || state_ == PushState::MID_APPROACH ||
         state_ == PushState::CUE_APPROACH || state_ == PushState::SELECT_CARGO))
        tick.motion.wz_rps *= t_.inplace_turn_boost;
}

bool PushTask::pixelLateralAccepted(const PushObservation& in) const {
    return in.target_grasp_range_valid && in.target_grasp_range_lateral &&
           (in.target_grasp_range_lateral_measured || in.pixel_grasp_aligned);
}

bool PushTask::pixelAlign(TaskTick& tick,
                          bool strict,
                          bool hold_position,
                          float tolerance_override_px) {
    auto& a = pixel_alignment_;
    const auto& in = tick.in;
    tick.immediate_motion_stop = true;
    // A handoff or accepted position needs rest, not another turn.
    if (a.holding_position != hold_position) {
        a.stage = PixelAlignment::Stage::Correct;
        a.direction = 0;
        a.holding_position = hold_position;
    }
    const bool far_handoff = !in.pixel_grasp_view && in.target_center_px_valid &&
                             in.pixel_view_handoff_ready;
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
    const float sent_vx = in.sent_velocity_valid ? in.sent_vx_mps : last_vx_;
    const float sent_wz = in.sent_velocity_valid ? in.sent_wz_rps : last_wz_;
    // Sent commands are not measured wheel velocities. Fresh stationary images are also
    // required before Advance. Replays use the last task command when no UART exists.
    const bool stopped = std::isfinite(sent_vx) && std::isfinite(sent_wz) &&
                         std::abs(sent_vx) <= t_.pixel_align_stopped_vx &&
                         std::abs(sent_wz) <= t_.pixel_align_stopped_wz;
    const bool fresh = in.pixel_grasp_sample_us > a.last_sample_us;
    if (fresh)
        a.last_sample_us = in.pixel_grasp_sample_us;
    const int direction = error > 0 ? 1 : (error < 0 ? -1 : 0);
    const auto brake = [&] {
        a.stage = PixelAlignment::Stage::Brake;
        a.stopped_since_us = 0;
        a.within_tolerance = hold_position || magnitude <= tolerance;
        a.stable_frames = 0;
        a.stable_since_us = 0;
        reason_ = "pixel_align_brake";
    };
    if (a.stage == PixelAlignment::Stage::Advance) {
        if ((hold_position || magnitude <= (strict ? tolerance : restart)) &&
            (!strict || stopped))
            return true;
        brake();
        return false;
    }
    if (a.stage == PixelAlignment::Stage::Correct) {
        if (hold_position || magnitude <= tolerance ||
            (a.direction && direction != a.direction) ||
            !std::isfinite(sent_vx) || !std::isfinite(sent_wz) ||
            std::abs(sent_vx) > t_.pixel_align_stopped_vx ||
            (sent_wz * error < 0 && std::abs(sent_wz) > t_.pixel_align_stopped_wz)) {
            brake();
            return false;
        }
        reason_ = "pixel_center_align";
        drive(tick, 0, t_.pixel_align_gain_rps_per_px * error, true);
        if (tick.motion.wz_rps != 0)
            a.direction = direction;
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
        if (hold_position || magnitude <= tolerance)
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
        brake();
        return false;
    }
    if (fresh) {
        if (hold_position) {
            a.brake_min_px = std::min(a.brake_min_px, error);
            a.brake_max_px = std::max(a.brake_max_px, error);
            a.brake_min_y_px = std::min(a.brake_min_y_px, in.target_center_px.y);
            a.brake_max_y_px = std::max(a.brake_max_y_px, in.target_center_px.y);
            if (a.brake_max_px - a.brake_min_px > t_.pixel_align_brake_motion_px ||
                a.brake_max_y_px - a.brake_min_y_px > t_.pixel_align_brake_motion_px) {
                brake();
                return false;
            }
        }
        if (!a.stable_frames)
            a.stable_since_us = in.pixel_grasp_sample_us;
        ++a.stable_frames;
        if (a.stable_frames >= t_.pixel_align_stable_frames &&
            in.pixel_grasp_sample_us - a.stable_since_us >= t_.pixel_align_stable_us) {
            a.stage = PixelAlignment::Stage::Advance;
            return true;
        }
    }
    return false;
}

void PushTask::scanSpin(TaskTick& tick) {
    tick.scan_spin = true;
    drive(tick, 0, t_.scan_wz);
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
    context.enemy_zone_seen = tick.in.enemy_zone_seen;
    context.travel_m = actionProgress().travel;
    context.motion_ratio = motionRatio();
    return context;
}

bool PushTask::evadeEnemy(TaskTick& tick) {
    const auto evade = transport_phase_.carry_search_.evade(t_, carrySearchContext(tick));
    if (evade.reset_zone_search)
        transport_phase_.zone_search_started_us_ = evade.zone_search_us;
    if (evade.reset_travel)
        actionProgress().travel = 0;
    if (evade.reason)
        reason_ = evade.reason;
    if (evade.action == CarrySearchAction::TurnAway) {
        scanSpin(tick);
        return true;
    }
    if (evade.action == CarrySearchAction::DriveAway) {
        drive(tick, evade.speed, evade.heading);
        return true;
    }
    return false;
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

void PushTask::steer(TaskTick& tick, float speed, float heading) {
    if (std::abs(heading) > t_.rotate_in_place_rad)
        drive(tick, 0, t_.heading_gain * heading);
    else
        drive(tick,
              speed,
              std::abs(heading) <= t_.heading_deadband_rad ? 0.f : t_.heading_gain * heading);
}

bool PushTask::carrySearch(TaskTick& tick) {
    const auto carry = transport_phase_.carry_search_.update(t_, carrySearchContext(tick));
    if (carry.reset_zone_search)
        transport_phase_.zone_search_started_us_ = carry.zone_search_us;
    if (carry.reset_travel)
        actionProgress().travel = 0;
    if (carry.reason)
        reason_ = carry.reason;
    switch (carry.action) {
        case CarrySearchAction::TurnAway:
            scanSpin(tick);
            return true;
        case CarrySearchAction::DriveAway:
            drive(tick, carry.speed, carry.heading);
            return true;
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

ClusterGroup
PushTask::findBest(TaskTick& tick, const std::vector<cv::Point2f>& pts, float max_y, float radius) {
    return tick.cluster.findBest(t_, tick.in, pts, max_y, radius);
}

int PushTask::evalGroup(TaskTick& tick,
                        const std::vector<cv::Point2f>& pts,
                        int need_n,
                        int far_need,
                        float short_radius) {
    return tick.cluster.evalGroup(
        t_, tick.in, pts, need_n, far_need, tick.far_y, tick.sweep_esc, short_radius);
}

void PushTask::beginScatter(TaskTick& tick, bool far, bool commit, const char* why) {
    search_phase_.sweep_.resetScatterApproach();
    trip_context_.target_id_ = -1;
    actionProgress().confirmations = 0;
    search_phase_.cue_mode_ = false;
    const cv::Point2f best_c = tick.cluster.bestCentre();
    const float best_d = tick.cluster.bestDistance();
    search_phase_.scatter_yaw_ = wrapAngle(tick.in.heading_rad + std::atan2(-best_c.x, best_c.y));
    search_phase_.scatter_range_ = best_d;
    search_phase_.scatter_far_ = far;
    search_phase_.scatter_commit_ = commit;
    sweep_ledger_.begin(t_, tick.now);
    search_phase_.burst_fast_ = search_phase_.burst_swing_ = false;
    if (far) {
        search_phase_.scatter_approach_m_ =
            std::max(0.f, best_d - t_.blind_scatter_approach_stop_m);
        if (t_.first_burst && !commit && !search_phase_.burst_drive_used_ &&
            search_phase_.scatter_approach_m_ >= t_.first_burst_min_drive_m) {
            search_phase_.burst_fast_ = search_phase_.burst_drive_used_ =
                search_phase_.burst_swing_armed_ = true;
        }
    } else {
        search_phase_.burst_swing_ = search_phase_.burst_swing_armed_;
        search_phase_.burst_swing_armed_ = false;
        search_phase_.disperse_sign_ = (search_phase_.scatter_count_ % 2 == 0) ? 1.f : -1.f;
        search_phase_.disperse_swing_.configure(t_.blind_scatter_passes, t_.blind_scatter_swings);
        search_phase_.disperse_rush_.beginSweep(t_.swing_avoid_outside);
        ++search_phase_.scatter_count_;
    }
    enter(PushState::SCATTER, tick.now, why);
}

bool PushTask::beginPileFromSweep(
    TaskTick& tick,
    const char* why) { // a disperse sweep that really swung continues in PILE (layer loop)
                       // instead of the plain re-search
    if (!t_.pile_chain || !t_.pile_clear || search_phase_.disperse_small_ || frame_raised_ != 0 ||
        trip_context_.trip_.total() != 0 || !tick.in.heading_valid ||
        !std::isfinite(tick.in.heading_rad))
        return false;
    trip_context_.target_id_ = -1;
    actionProgress().confirmations = 0;
    search_phase_.cue_mode_ = false;
    search_phase_.pile_yaw0_ = tick.in.heading_rad;
    search_phase_.pile_phase_ = 7;
    search_phase_.pile_seen_ = 0;
    search_phase_.pile_chain_n_ = 0;
    search_phase_.sweep_phase_ = 0;
    enter(PushState::PILE, tick.now, why);
    search_phase_.pile_phase_us_ = tick.now;
    return true;
}

bool PushTask::lostTarget(TaskTick&) {
    return ++actionProgress().misses > t_.hold_grace_frames;
}

void PushTask::searchLost(TaskTick& tick) {
    if (frame_raised_ == 0 && trip_context_.trip_.total() == 0 && tick.scatter_ok &&
        tick.clutter_fresh) { // the group is still in view from the approach camera: sweep it
                              // instead of spinning
        const int g =
            evalGroup(tick, tick.in.clutter_body, t_.blind_scatter_min, t_.blind_scatter_far_min);
        if (g) {
            beginScatter(tick, g == 2, true, g == 2 ? "lost_group_far" : "lost_group_sweep");
            return;
        }
    }
    if (frame_raised_ != 0 || trip_context_.trip_.total() > 0) {
        failStop(tick, "lost_search_jaw_or_load_unsafe");
        return;
    }
    clearTrip();
    search_phase_.lost_round_ = search_phase_.post_disperse_mid_ && midPitch() ? 1 : 0;
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

bool PushTask::cueCandidate(TaskTick& tick) {
    return t_.enable_search_cues && tick.in.target_is_search_cue && tick.in.target_valid &&
           tick.in.target_id >= 0 && targetKind(tick.in.label) != TargetKind::UNKNOWN &&
           tick.in.geometry_valid && tick.in.target_region_valid && !tick.in.target_in_zone &&
           std::isfinite(tick.in.distance_m) && tick.in.distance_m >= 0 &&
           std::isfinite(tick.in.heading_error);
}

bool PushTask::sweepFrameReady(TaskTick& tick) {
    if ((!t_.sweep_frame_open && !t_.frame_open_default) || sweep_open_disabled_ ||
        frame_raised_ != 0)
        return true;
    if (!sweep_open_) {
        if (!(t_.sweep_frame_open || defaultOpenState()))
            return true;
        search_phase_.sweep_start_us_ = tick.now;
        reason_ = "sweep_frame_open_wait";
        return false;
    }
    if (sweep_open_done_)
        return true;
    if (tick.in.gripper_done && tick.in.gripper_feedback_open == 1) {
        sweep_open_done_ = true;
        std::cout << "[SWEEP_FRAME] open confirmed after " << (tick.now - sweep_open_us_) / 1000
                  << " ms state=" << name(state_) << "\n";
        return true;
    }
    if (tick.now - sweep_open_us_ > t_.gripper_timeout_us) {
        sweep_open_disabled_ = true;
        std::cout << "[SWEEP_FRAME] open not confirmed within " << t_.gripper_timeout_us / 1000
                  << " ms: sweeps stay closed from now on\n";
    }
    search_phase_.sweep_start_us_ = tick.now;
    reason_ = "sweep_frame_open_wait";
    return false;
}

bool PushTask::disperseStep(TaskTick& tick) {
    if (!sweepFrameReady(tick))
        return false;
    if (search_phase_.sweep_phase_ == 4) {
        const bool plan_target = candidate(tick.in, tick.now) &&
                                 std::isfinite(tick.in.distance_m) &&
                                 std::isfinite(tick.in.heading_error);
        DisperseRushContext rush_context;
        rush_context.now_us = tick.now;
        rush_context.sweep_start_us = search_phase_.sweep_start_us_;
        rush_context.travel_m = actionProgress().travel;
        rush_context.sweep_travel_m = search_phase_.sweep_travel0_;
        rush_context.motion_ratio = motionRatio();
        rush_context.rush_distance_m = search_phase_.disperse_rush_m_;
        rush_context.cluster_range_m = search_phase_.scatter_range_;
        rush_context.scatter_state = state_ == PushState::SCATTER;
        rush_context.burst_swing = search_phase_.burst_swing_;
        rush_context.far_judge = t_.sweep_far_judge;
        rush_context.pitch_ready = pitch_cmd_ == t_.far_pitch_cdeg;
        rush_context.clutter_fresh = tick.clutter_fresh;
        rush_context.has_target = plan_target;
        if (plan_target)
            rush_context.target_body = targetBody(tick);
        const auto rush = search_phase_.disperse_rush_.update(t_, tick.in, rush_context);
        if (rush.plan_event) {
            if (rush.first_sign != 0.f)
                search_phase_.disperse_sign_ = rush.first_sign;
            std::cout << "[SWING_PLAN] outside=" << rush.outside_n
                      << " cap_left=" << rush.cap_positive << " cap_right=" << rush.cap_negative
                      << " first=" << (search_phase_.disperse_sign_ > 0 ? "left" : "right") << "\n";
        }
        if (rush.outcome == DisperseRushOutcome::Complete) {
            search_phase_.sweep_phase_ = 5;
            search_phase_.sweep_start_us_ = tick.now;
            search_phase_.sweep_yaw0_ = tick.in.heading_rad;
            search_phase_.disperse_swing_.beginSwing();
            search_phase_.sweep_travel0_ = actionProgress().travel;
            sweep_ledger_.setSwung(true);
            search_phase_.disperse_turn_.begin(t_.disperse_single_turn &&
                                               !search_phase_.disperse_small_);
        } else {
            if (rush.reason)
                reason_ = rush.reason;
            drive(tick, rush.vx, 0);
            if (tick.motion.vx_mps > 0)
                tick.motion.vx_mps = rush.vx; // preserve every clearance veto before bypassing the
                                              // ordinary speed cap
            return false;
        }
    }
    if (search_phase_.disperse_turn_.active()) {
        const DisperseTurnContext context{tick.now,
                                          search_phase_.sweep_start_us_,
                                          actionProgress().travel,
                                          search_phase_.sweep_travel0_,
                                          search_phase_.sweep_yaw0_,
                                          motionRatio(),
                                          search_phase_.disperse_sign_,
                                          desiredPitch() == pitch_cmd_ &&
                                              tick.in.camera_pitch_stable,
                                          midPitch(),
                                          search_phase_.disperse_rush_.planActive(),
                                          search_phase_.disperse_rush_.capPositive(),
                                          search_phase_.disperse_rush_.capNegative()};
        const auto turn = search_phase_.disperse_turn_.update(tick.in, t_, context);
        if (turn.turn_started) {
            search_phase_.sweep_yaw0_ = tick.in.heading_rad;
            search_phase_.sweep_start_us_ = tick.now;
        }
        if (turn.push_started) {
            search_phase_.sweep_start_us_ = tick.now;
            search_phase_.sweep_travel0_ = actionProgress().travel;
        }
        if (turn.frame_event)
            std::cout << "[TURN] frame_zone n=" << turn.frame_count
                      << " max=" << t_.disperse_turn_frame_max << " turned=" << turn.turned
                      << " ok_frames=" << turn.confirmations << "\n";
        if (turn.reason)
            reason_ = turn.reason;
        if (turn.motion == DisperseTurnMotion::Push)
            drive(tick, t_.disperse_speed, 0);
        else if (turn.motion == DisperseTurnMotion::Turn) {
            const bool fast = (search_phase_.burst_swing_ && state_ == PushState::SCATTER) ||
                              (t_.approach_dense_burst_swing && state_ == PushState::APPROACH &&
                               !search_phase_.sweep_non_supply_active_);
            drive(tick, 0.f, turn.direction);
            if (tick.motion.wz_rps != 0)
                tick.motion.wz_rps =
                    std::copysign(fast ? t_.first_burst_swing_wz : t_.disperse_wz, turn.direction);
        }
        return turn.outcome == DisperseTurnOutcome::Complete;
    }
    const bool burst = search_phase_.burst_swing_ && state_ == PushState::SCATTER;
    const DisperseSwingContext context{tick.now,
                                       search_phase_.sweep_start_us_,
                                       actionProgress().travel,
                                       search_phase_.sweep_travel0_,
                                       search_phase_.sweep_yaw0_,
                                       tick.in.heading_rad,
                                       motionRatio(),
                                       search_phase_.disperse_sign_,
                                       search_phase_.disperse_small_,
                                       search_phase_.disperse_rush_.planActive(),
                                       search_phase_.disperse_rush_.capPositive(),
                                       search_phase_.disperse_rush_.capNegative(),
                                       burst,
                                       t_.approach_dense_burst_swing &&
                                           state_ == PushState::APPROACH &&
                                           !search_phase_.sweep_non_supply_active_};
    const auto swing = search_phase_.disperse_swing_.update(t_, context);
    if (swing.restart_clock)
        search_phase_.sweep_start_us_ = tick.now;
    if (swing.outcome == DisperseSwingOutcome::NextPass) {
        search_phase_.disperse_rush_m_ = t_.disperse_pass_extra_m;
        search_phase_.disperse_sign_ = -search_phase_.disperse_sign_;
        search_phase_.sweep_phase_ = 4;
        search_phase_.sweep_start_us_ = tick.now;
        search_phase_.sweep_travel0_ = actionProgress().travel;
    }
    if (swing.reason)
        reason_ = swing.reason;
    if (swing.motion_requested) {
        drive(tick, swing.vx, swing.direction);
        if (tick.motion.wz_rps != 0)
            tick.motion.wz_rps = std::copysign(
                swing.fast ? t_.first_burst_swing_wz : t_.disperse_wz, swing.direction);
    }
    return swing.outcome == DisperseSwingOutcome::Complete;
}

} // namespace rescue
