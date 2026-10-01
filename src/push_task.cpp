#include "rescue/push_task.hpp"
#include <algorithm>
#include <cmath>

namespace rescue {
const char *PushTask::name(PushState s) {
    switch (s) {
    case PushState::WAIT_START: return "WAIT_START";
    case PushState::SCAN: return "SCAN";
    case PushState::APPROACH: return "APPROACH";
    case PushState::PREPARE: return "PREPARE";
    case PushState::RUSH: return "RUSH";
    case PushState::CLOSE: return "CLOSE";
    case PushState::VERIFY_CAPTURE: return "VERIFY_CAPTURE";
    case PushState::CARRY: return "CARRY";
    case PushState::GATE: return "GATE";
    case PushState::OPEN_RELEASE: return "OPEN_RELEASE";
    case PushState::ENTER: return "ENTER";
    case PushState::BACK_OUT: return "BACK_OUT";
    case PushState::VERIFY_DELIVERY: return "VERIFY_DELIVERY";
    case PushState::TURN_SCAN: return "TURN_SCAN";
    case PushState::CAPTURE_FAIL: return "CAPTURE_FAIL";
    case PushState::LOST_HOLD: return "LOST_HOLD";
    case PushState::ABORT_DROP: return "ABORT_DROP";
    case PushState::SAFE_STOP: return "SAFE_STOP";
    }
    return "UNKNOWN";
}
void PushTask::enter(PushState s, uint64_t now, const char *why) {
    state_ = s; phase_us_ = now; travel_ = 0; travel_limit_ = 0;
    confirmations_ = 0; misses_ = 0; hold_misses_ = 0; step_ = 0; turned_ = 0; heading_seen_ = false;
    track_close_ = false;
    if (why && *why) reason_ = why;
}
void PushTask::clearTrip() {
    drop_locked_=false;drop_centre_zone_={};
    target_id_ = -1; label_.clear(); trip_ = {}; pending_ = {}; seen_ = {};
    verdict_ = RuleVerdict::OK;
}
void PushTask::blacklist(uint64_t now) {
    if (target_id_ >= 0) rejected_[target_id_] = now + t_.blacklist_us;
}
// Unrecoverable without an operator: stop, keep the gripper as commanded.
void PushTask::fail(const char *why) { fault_ = true; reason_ = why; }
void PushTask::commandGripper(uint8_t open, uint64_t now) {
    if (gripper_cmd_ != open) { gripper_cmd_ = open; gripper_us_ = now; }
}
int PushTask::gripperWait(const PushObservation &in, uint8_t open, uint64_t now) const {
    // A done flag counts only for the action matching the current target state.
    if (gripper_cmd_ == open && in.gripper_done && in.gripper_feedback_open == open) return 1;
    // Timeout counts from the later of the command and the phase start, so an action
    // commanded earlier still gets a full window when a new phase depends on it.
    return gripper_cmd_ == open && now - std::max(gripper_us_, phase_us_) > t_.gripper_timeout_us ? -1 : 0;
}
bool PushTask::candidate(const PushObservation &in, uint64_t now) const {
    if (!in.target_valid || in.target_id < 0 || !in.geometry_valid || !in.target_region_valid || in.target_in_zone) return false;
    if (!targetSelectable(in.label, first_)) return false;
    if (!std::isfinite(in.distance_m) || in.distance_m < 0 || !std::isfinite(in.heading_error) ||
        std::abs(in.heading_error) > 1.2f) return false;
    const auto it = rejected_.find(in.target_id);
    return it == rejected_.end() || now >= it->second;
}
bool PushTask::tracking(const PushObservation &in) const {
    return in.target_valid && in.target_id == target_id_ && in.label == label_ && in.geometry_valid &&
        in.target_region_valid && !in.target_in_zone && std::isfinite(in.distance_m) && in.distance_m >= 0 &&
        std::isfinite(in.heading_error) && std::abs(in.heading_error) <= 1.2f;
}
bool PushTask::zoneOk(const PushObservation &in) const {
    return in.zone_identity_verified && in.zone_valid && in.zone_own && in.zone_estimate.trusted(in.now_us);
}
// The held set must still be the verified trip; a changed or partial view is not a hold.
bool PushTask::holding(const PushObservation &in) const {
    return in.hold_observable && in.captured && in.held_complete && in.held == trip_;
}
int PushTask::holdSeen(const PushObservation &in) const {
    if (!in.hold_observable) return -1;
    return holding(in) ? 1 : 0;
}
// Preset the current phase looks with; kCameraPitchInvalid keeps the last command (stops and
// drops do not move the camera).
int16_t PushTask::desiredPitch() const {
    switch (state_) {
    case PushState::SCAN: case PushState::TURN_SCAN: case PushState::VERIFY_DELIVERY: return t_.far_pitch_cdeg;
    case PushState::APPROACH: return track_close_ ? t_.track_pitch_cdeg : t_.far_pitch_cdeg;
    case PushState::PREPARE: case PushState::RUSH: case PushState::CLOSE:
    case PushState::VERIFY_CAPTURE: case PushState::OPEN_RELEASE: return t_.near_pitch_cdeg;
    case PushState::CARRY: case PushState::GATE: // step 1: stopped NEAR hold check
        return step_ ? t_.near_pitch_cdeg : t_.far_pitch_cdeg;
    case PushState::ENTER: case PushState::BACK_OUT: return t_.track_pitch_cdeg; // fence and mouth
    default: return kCameraPitchInvalid;
    }
}
int PushTask::pitchWait(const PushObservation &in, uint64_t now) {
    const int16_t want = desiredPitch();
    if (want == kCameraPitchInvalid) return 1;
    if (want != pitch_cmd_) { pitch_cmd_ = want; pitch_wait_us_ = now; }
    if (in.camera_pitch_cdeg != kCameraPitchInvalid && in.camera_pitch_stable &&
        std::abs(int(in.camera_pitch_cdeg) - int(pitch_cmd_)) <= t_.pitch_tolerance_cdeg) {
        pitch_wait_us_ = now; return 1;
    }
    // Counted from the later of the last command/ready frame and the phase start (resume).
    return now - std::max(pitch_wait_us_, phase_us_) > t_.pitch_timeout_us ? -1 : 0;
}
float PushTask::halfX() const {
    return drop_locked_ ? drop_centre_zone_.x : (trip_.injured > 0 ? t_.injured_half_x_m : t_.supply_half_x_m);
}
// After a stop the gripper state decides where to resume: a verified load is never
// silently forgotten, and anything uncertain is released before scanning again.
PushState PushTask::resumeState() const {
    switch (stopped_from_) {
    case PushState::CARRY: case PushState::GATE: return PushState::CARRY;
    case PushState::WAIT_START: case PushState::SCAN: case PushState::APPROACH:
    case PushState::TURN_SCAN: return PushState::SCAN;
    case PushState::PREPARE: return PushState::APPROACH;
    case PushState::OPEN_RELEASE: case PushState::ENTER: case PushState::BACK_OUT:
        return PushState::BACK_OUT; // the load is at our zone; finish and verify
    case PushState::VERIFY_DELIVERY: return PushState::VERIFY_DELIVERY;
    default: return PushState::ABORT_DROP;
    }
}

PushOutput PushTask::update(const PushObservation &in) {
    MotionCommand motion; // zero velocity by default
    const uint64_t now = in.now_us;
    const auto result = [&] {
        motion.gripper_open = gripper_cmd_; // stops hold the gripper, never toggle it
        const int16_t want = desiredPitch(); // a phase change commands its preset at once
        if (want != kCameraPitchInvalid && want != pitch_cmd_) { pitch_cmd_ = want; pitch_wait_us_ = now; }
        motion.camera_pitch_cdeg = pitch_cmd_;
        last_vx_ = motion.vx_mps; last_wz_ = motion.wz_rps;
        PushOutput out;
        out.state = state_; out.motion = motion; out.target_id = target_id_;
        out.batch_size = trip_.total() ? trip_.total() : pending_.total();
        out.delivered_total = total_; out.first_ordinary_delivered = first_;
        out.cargo_injured=trip_.injured>0;out.drop_locked=drop_locked_;out.drop_centre_zone=drop_centre_zone_;
        out.verdict = verdict_; out.reason = reason_;
        return out;
    };
    if (in.reset) {
        clearTrip(); rejected_.clear(); total_ = 0; first_ = false; fault_ = false;
        gripper_cmd_ = 0; gripper_us_ = now; stopped_from_ = PushState::WAIT_START;
        pitch_cmd_ = t_.far_pitch_cdeg; pitch_wait_us_ = now;
        enter(PushState::WAIT_START, now, "reset"); last_us_ = now;
        return result();
    }
    const bool timely = now > 0 && (last_us_ == 0 || (now > last_us_ && now - last_us_ <= t_.frame_timeout_us));
    const float dt = timely && last_us_ ? float(now - last_us_) * 1e-6f : 0.f;
    last_us_ = now;
    travel_ += std::abs(last_vx_) * dt; carried_ += std::abs(last_vx_) * dt;
    if (fault_) { state_ = PushState::SAFE_STOP; return result(); }
    if (!in.run || !in.safety_ok || !timely) {
        if (state_ != PushState::WAIT_START) stopped_from_ = state_;
        enter(PushState::WAIT_START, now, !in.run ? "not_running" : !in.safety_ok ? "safety_veto" : "stale_frame");
        return result();
    }
    if (state_ == PushState::WAIT_START) {
        enter(resumeState(), now, "resume");
        if (state_ == PushState::CARRY) { carry_us_ = now; step_ = 1; } // a pause is not carry time; re-check the hold
        stopped_from_ = PushState::WAIT_START;
        return result();
    }
    for (auto it = rejected_.begin(); it != rejected_.end();)
        it = now >= it->second ? rejected_.erase(it) : std::next(it);

    const bool forward_ok = in.path_safe && in.opponent_zone_clear;
    const uint64_t elapsed = now - phase_us_;
    const auto drive = [&](float vx, float wz) {
        if ((vx > 0 && !forward_ok) || (vx < 0 && !in.retreat_safe) || (vx == 0 && wz != 0 && !forward_ok))
            return; // missing clearance stops the robot; it never substitutes another motion
        motion.vx_mps = std::clamp(vx, -t_.max_speed, t_.max_speed);
        motion.wz_rps = std::clamp(wz, -t_.max_wz, t_.max_wz);
    };
    // Steer to a body-frame bearing; turn in place when the error is large.
    const auto steer = [&](float speed, float heading) {
        if (std::abs(heading) > t_.rotate_in_place_rad) drive(0, t_.heading_gain * heading);
        else drive(speed, t_.heading_gain * heading);
    };
    const auto lostTarget = [&] { return ++misses_ > t_.hold_grace_frames; };
    // Stopped NEAR check: the load must be seen as the verified trip on consecutive frames.
    // Out of view at NEAR counts as missing too: there it should be visible.
    const auto holdCheck = [&] {
        if (holdSeen(in) == 1) { hold_misses_ = 0; return ++confirmations_ >= t_.hold_check_frames ? 1 : 0; }
        confirmations_ = 0;
        return ++hold_misses_ > t_.hold_grace_frames ? -1 : 0;
    };
    // Gripper commands above stay concurrent with the camera; anything that moves, tracks or
    // counts waits for the preset (frames during a move carry no reliable geometry or hold).
    if (state_ != PushState::PREPARE && state_ != PushState::OPEN_RELEASE) {
        const int p = pitchWait(in, now);
        if (p < 0) { fail("camera_pitch_timeout"); state_ = PushState::SAFE_STOP; return result(); }
        if (p == 0) return result();
    }
    const bool drop_fresh=in.navigation_timestamp_us && now>=in.navigation_timestamp_us &&
        now-in.navigation_timestamp_us<=200000 && in.drop_plan_valid && drop_locked_ &&
        std::isfinite(in.drop_centre_zone.x) && std::isfinite(in.drop_centre_zone.y) &&
        cv::norm(in.drop_centre_zone-drop_centre_zone_)<=.005f;
    if((state_==PushState::GATE || state_==PushState::OPEN_RELEASE || state_==PushState::ENTER) && !drop_fresh) {
        reason_="drop_revalidation_missing";
        if(now-carry_us_>t_.carry_budget_us) {fail("drop_revalidation_timeout");state_=PushState::SAFE_STOP;}
        return result();
    }
    switch (state_) {
    case PushState::SCAN: {
        commandGripper(0, now);
        if (!candidate(in, now)) { confirmations_ = 0; target_id_ = -1; drive(0, t_.scan_wz); break; }
        if (target_id_ != in.target_id || label_ != in.label) {
            target_id_ = in.target_id; label_ = in.label; confirmations_ = 0;
        }
        if (++confirmations_ >= t_.confirm_frames) enter(PushState::APPROACH, now, "target_locked");
        break;
    }
    case PushState::APPROACH:
        commandGripper(0, now); // closed while approaching keeps the swept width small
        if (!tracking(in)) { if (lostTarget()) { clearTrip(); enter(PushState::SCAN, now, "target_lost"); } break; }
        misses_ = 0;
        if (elapsed > t_.approach_budget_us) { blacklist(now); clearTrip(); enter(PushState::SCAN, now, "approach_timeout"); break; }
        // Tilt down to TRACK when close (hysteresis keeps it from toggling), stopped while it moves.
        if (in.distance_m <= t_.track_near_m) track_close_ = true;
        else if (in.distance_m > t_.track_near_m + .1f) track_close_ = false;
        if (desiredPitch() != pitch_cmd_) break;
        if (in.distance_m <= t_.rush_start_m) {
            if (std::abs(in.heading_error) <= t_.rush_heading_rad) enter(PushState::PREPARE, now, "rush_aligned");
            else drive(0, t_.heading_gain * in.heading_error);
        } else steer(t_.approach_speed, in.heading_error);
        break;
    case PushState::PREPARE: {
        // Stopped: open the frame, then inventory everything the rush would sweep.
        commandGripper(1, now);
        const int p = pitchWait(in, now);
        if (p < 0) { fail("camera_pitch_timeout"); break; }
        if (p == 0) break;
        if (!tracking(in)) { if (lostTarget()) { clearTrip(); enter(PushState::SCAN, now, "target_lost"); } break; }
        misses_ = 0;
        const int g = gripperWait(in, 1, now);
        if (g < 0) { fail("gripper_open_timeout"); break; }
        if (elapsed > t_.prepare_budget_us) {
            blacklist(now); clearTrip(); enter(PushState::SCAN, now, "corridor_unresolved"); break;
        }
        if (g == 0) break;
        const auto verdict = checkCorridor(in.corridor, in.corridor_complete, in.corridor_occlusion_free, label_, first_);
        verdict_ = verdict;
        if (verdict == RuleVerdict::INCOMPLETE) { confirmations_ = 0; break; }
        if (confirmations_ == 0 || in.corridor != seen_) { seen_ = in.corridor; confirmations_ = 0; }
        if (++confirmations_ < t_.confirm_frames) break;
        if (verdict != RuleVerdict::OK) {
            reason_ = std::string("corridor_") + verdictName(verdict);
            blacklist(now); clearTrip(); verdict_ = verdict; enter(PushState::SCAN, now, ""); break;
        }
        pending_ = in.corridor;
        enter(PushState::RUSH, now, "corridor_ok");
        travel_limit_ = std::max(0.f, in.distance_m - t_.hold_center_y_m) + t_.rush_extra_m;
        break;
    }
    case PushState::RUSH: {
        const bool complete = in.hold_observable && in.captured && in.held_complete;
        if (complete) {
            const auto verdict = checkTrip(in.held, first_);
            if (verdict != RuleVerdict::OK && verdict != RuleVerdict::EMPTY) {
                verdict_ = verdict; reason_ = std::string("held_") + verdictName(verdict);
                blacklist(now); enter(PushState::ABORT_DROP, now, ""); break;
            }
        }
        if (complete && in.held.count(targetKind(label_)) > 0) {
            if (++confirmations_ >= t_.enclose_frames) { enter(PushState::CLOSE, now, "enclosed"); break; }
        } else confirmations_ = 0;
        if (travel_ >= travel_limit_ || elapsed > t_.rush_budget_us) { enter(PushState::CAPTURE_FAIL, now, "rush_overrun"); break; }
        if (tracking(in)) {
            if (std::abs(in.heading_error) > t_.rush_abort_heading_rad) { enter(PushState::CAPTURE_FAIL, now, "rush_misaligned"); break; }
            drive(t_.rush_speed, t_.heading_gain * in.heading_error);
        } else drive(t_.rush_speed, 0); // target passes below the view before reaching the holding area
        break;
    }
    case PushState::CLOSE: {
        commandGripper(0, now);
        const int g = gripperWait(in, 0, now);
        if (g < 0) { reason_ = "gripper_close_timeout"; enter(PushState::ABORT_DROP, now, ""); }
        else if (g > 0) enter(PushState::VERIFY_CAPTURE, now, "closed");
        break;
    }
    case PushState::VERIFY_CAPTURE: {
        if (elapsed > t_.verify_budget_us) { enter(PushState::CAPTURE_FAIL, now, "capture_unverified"); break; }
        if (!in.hold_observable || !in.captured || !in.held_complete || in.held.total() == 0) { confirmations_ = 0; break; }
        if (confirmations_ == 0 || in.held != seen_) { seen_ = in.held; confirmations_ = 0; }
        if (++confirmations_ < t_.capture_frames) break;
        verdict_ = checkTrip(in.held, first_);
        if (verdict_ != RuleVerdict::OK) {
            reason_ = std::string("held_") + verdictName(verdict_);
            blacklist(now); enter(PushState::ABORT_DROP, now, ""); break;
        }
        trip_ = in.held; carry_us_ = now;
        enter(PushState::CARRY, now, "capture_verified");
        hold_seen_us_ = now; carried_ = 0;
        break;
    }
    case PushState::CARRY: {
        if (now - carry_us_ > t_.carry_budget_us) { enter(PushState::ABORT_DROP, now, "carry_timeout"); break; }
        if (step_) { // stopped at NEAR
            const int h = holdCheck();
            if (h < 0) { enter(PushState::LOST_HOLD, now, "hold_lost"); break; }
            if (h > 0) { step_ = 0; confirmations_ = 0; hold_seen_us_ = now; carried_ = 0; }
            break;
        }
        // At FAR the holding region is normally out of view: unknown, not lost.
        const int h = holdSeen(in);
        if (h == 0 && ++hold_misses_ > t_.hold_grace_frames) { enter(PushState::LOST_HOLD, now, "hold_lost"); break; }
        if (h == 1) { hold_misses_ = 0; hold_seen_us_ = now; carried_ = 0; }
        if (carried_ >= t_.hold_check_m || now - hold_seen_us_ > t_.hold_check_interval_us) {
            step_ = 1; confirmations_ = 0; hold_misses_ = 0; break;
        }
        if (!zoneOk(in)) { drive(0, t_.scan_wz); break; } // look for our zone without moving the load
        const bool fresh_plan=in.navigation_timestamp_us && now>=in.navigation_timestamp_us && now-in.navigation_timestamp_us<=200000;
        if(!fresh_plan || !in.drop_plan_valid || !in.carry_plan_valid ||
           !std::isfinite(in.drop_centre_zone.x)||!std::isfinite(in.drop_centre_zone.y)||
           !std::isfinite(in.carry_waypoint_body.x)||!std::isfinite(in.carry_waypoint_body.y)) {
            reason_="carry_plan_missing";break;
        }
        if(!drop_locked_) {drop_centre_zone_=in.drop_centre_zone;drop_locked_=true;}
        if(cv::norm(in.drop_centre_zone-drop_centre_zone_)>.005f) {reason_="drop_plan_changed";break;}
        // Park the rotation centre so that, once facing in, the held centre sits gate_clearance
        // before the correct half: turning in place there does not sweep the load sideways.
        const auto &z = in.zone_estimate;
        const cv::Point2f gate = z.zoneToBody({halfX(), -t_.gate_clearance_m - t_.hold_center_y_m});
        const float dist = std::hypot(gate.x, gate.y);
        if (dist <= t_.gate_tolerance_m) {
            enter(PushState::GATE, now, "gate_reached"); break;
        }
        const float bearing = std::atan2(-in.carry_waypoint_body.x, in.carry_waypoint_body.y);
        steer(std::max(.03f, std::min(t_.carry_speed, float(cv::norm(in.carry_waypoint_body)))), bearing);
        break;
    }
    case PushState::GATE: {
        if (now - carry_us_ > t_.carry_budget_us) { enter(PushState::ABORT_DROP, now, "carry_timeout"); break; }
        if (step_) { // aligned and counted; stopped at NEAR: open only on a confirmed hold
            const int h = holdCheck();
            if (h < 0) enter(PushState::LOST_HOLD, now, "hold_lost");
            else if (h > 0) enter(PushState::OPEN_RELEASE, now, "gate_aligned");
            break;
        }
        if (holdSeen(in) == 0 && ++hold_misses_ > t_.hold_grace_frames) { enter(PushState::LOST_HOLD, now, "hold_lost"); break; }
        if (!zoneOk(in)) { if (lostTarget()) enter(PushState::CARRY, now, "zone_lost"); break; }
        misses_ = 0;
        const auto &z = in.zone_estimate;
        const float yaw = wrapAngle(z.yaw_body_rad); // heading error to face into the zone
        if (std::abs(yaw) > t_.gate_yaw_rad) { confirmations_ = 0; drive(0, t_.heading_gain * yaw); break; }
        const cv::Point2f hold = z.bodyToZone({0, t_.hold_center_y_m});
        if (std::abs(hold.x - halfX()) > t_.gate_lateral_m ||
            std::abs(hold.y + t_.gate_clearance_m) > 2 * t_.gate_tolerance_m) {
            enter(PushState::CARRY, now, "gate_offset"); break;
        }
        // Baseline counts make the delivery check differential: objects already inside never count.
        if (!in.zone_counts_valid) { confirmations_ = 0; break; }
        if (++confirmations_ < t_.confirm_frames) break;
        baseline_ = trip_.injured ? in.zone_injured_count : in.zone_supply_count;
        baseline_other_ = trip_.injured ? in.zone_supply_count : in.zone_injured_count;
        step_ = 1; confirmations_ = 0; hold_misses_ = 0; // the robot stays put: the baseline holds
        break;
    }
    case PushState::OPEN_RELEASE: {
        if(!in.opponent_zone_clear) {
            reason_="release_opponent_clearance_missing";
            if(elapsed>t_.gripper_timeout_us)fail("release_clearance_timeout");
            break;
        }
        commandGripper(1, now);
        const int p = pitchWait(in, now);
        if (p < 0) { fail("camera_pitch_timeout"); break; }
        const int g = gripperWait(in, 1, now);
        if (g < 0) fail("gripper_open_timeout");
        else if (g > 0) enter(PushState::ENTER, now, "released_at_gate");
        break;
    }
    case PushState::ENTER: {
        // Push the released load with the open frame into the centre of the correct half.
        if (!zoneOk(in)) { if (lostTarget()) enter(PushState::BACK_OUT, now, "zone_lost"); break; }
        misses_ = 0;
        const auto &z = in.zone_estimate;
        const cv::Point2f hold = z.bodyToZone({0, t_.hold_center_y_m});
        const float lateral = hold.x - halfX();
        if (hold.y >= (drop_locked_?drop_centre_zone_.y:t_.deposit_y_m)) { enter(PushState::BACK_OUT, now, "deposited"); break; }
        if (std::abs(lateral) > t_.enter_lateral_m) { enter(PushState::BACK_OUT, now, "enter_lateral"); break; }
        if (elapsed > t_.enter_budget_us) { enter(PushState::BACK_OUT, now, "enter_timeout"); break; }
        drive(t_.enter_speed, t_.heading_gain * wrapAngle(z.yaw_body_rad) + t_.lateral_gain * lateral);
        break;
    }
    case PushState::BACK_OUT: {
        // Keep the frame open so reversing leaves the load behind.
        commandGripper(1, now);
        const int g = gripperWait(in, 1, now);
        if (g <= 0) { if (g < 0) fail("gripper_open_timeout"); break; } // never drag the load out closed
        if (step_ == 0) { // distance bound used if the zone drops out of view
            const float mouth = zoneOk(in) ? in.zone_estimate.bodyToZone({0, t_.mouth_y_m}).y
                                           : (drop_locked_?drop_centre_zone_.y:t_.deposit_y_m) + t_.mouth_y_m;
            travel_limit_ = std::max(0.f, mouth - t_.mouth_clear_y_m) + .05f;
            travel_ = 0; step_ = 1;
        }
        const bool clear = zoneOk(in) ?
            in.zone_estimate.bodyToZone({0, t_.mouth_y_m}).y <= t_.mouth_clear_y_m : travel_ >= travel_limit_;
        if (clear) { enter(PushState::VERIFY_DELIVERY, now, "backed_out"); break; }
        if (elapsed > t_.retreat_budget_us) { fail("retreat_blocked"); break; }
        drive(-t_.back_speed, 0);
        break;
    }
    case PushState::VERIFY_DELIVERY: {
        commandGripper(1, now);
        const int expected = trip_.total();
        if (zoneOk(in) && in.zone_counts_valid) {
            const int got = (trip_.injured ? in.zone_injured_count : in.zone_supply_count) - baseline_;
            const int other = trip_.injured ? in.zone_supply_count : in.zone_injured_count;
            if (confirmations_ == 0 || got != step_) { step_ = got; confirmations_ = 0; }
            if (other != baseline_other_) confirmations_ = 0;
            else ++confirmations_;
        } else confirmations_ = 0;
        const bool stable = confirmations_ >= t_.delivery_frames;
        const bool expired = elapsed > t_.delivery_budget_us;
        if (!(stable && step_ == expected) && !expired) break;
        // Only stable, differential counts are credited; a timed push is not a delivery.
        const int credited = stable && step_ > 0 ? std::min(step_, expected) : 0;
        total_ += credited;
        if (credited > 0 && trip_.ordinary == trip_.total()) first_ = true;
        reason_ = credited == expected ? "delivered" : credited ? "partial_delivery" : "delivery_unverified";
        enter(PushState::TURN_SCAN, now, "");
        break;
    }
    case PushState::TURN_SCAN: {
        commandGripper(0, now);
        // Measure the turn with the IMU; dead-reckon the command only when it is unavailable.
        if (in.heading_valid && std::isfinite(in.heading_rad)) {
            if (heading_seen_) turned_ += std::abs(wrapAngle(in.heading_rad - last_heading_));
            last_heading_ = in.heading_rad; heading_seen_ = true;
        } else { turned_ += std::abs(last_wz_) * dt; heading_seen_ = false; }
        if (turned_ >= t_.turn_min_rad || elapsed > t_.turn_budget_us) {
            clearTrip(); enter(PushState::SCAN, now, ""); break;
        }
        drive(0, t_.turn_wz);
        break;
    }
    case PushState::CAPTURE_FAIL: case PushState::LOST_HOLD: case PushState::ABORT_DROP: {
        // Release where the opponent zone is not involved, then reverse off the objects.
        if (step_ == 0) {
            if (!in.opponent_zone_clear) { if (elapsed > t_.gripper_timeout_us) fail("drop_blocked_opponent_zone"); break; }
            commandGripper(1, now);
            const int g = gripperWait(in, 1, now);
            if (g < 0) { fail("gripper_open_timeout"); break; }
            if (g == 0) break;
            step_ = 1; travel_ = 0; phase_us_ = now;
        }
        if (travel_ >= t_.abort_back_m) {
            if (state_ != PushState::LOST_HOLD) blacklist(now);
            clearTrip(); enter(PushState::SCAN, now, ""); break;
        }
        if (elapsed > t_.retreat_budget_us) { fail("retreat_blocked"); break; }
        drive(-t_.back_speed, 0);
        break;
    }
    case PushState::WAIT_START: case PushState::SAFE_STOP: break;
    }
    if (fault_) { state_ = PushState::SAFE_STOP; motion.vx_mps = motion.wz_rps = 0; }
    return result();
}
std::vector<int> PushTask::rejectedTargets(uint64_t now) const {
    std::vector<int> ids;
    for(const auto& item:rejected_)if(item.second>now)ids.push_back(item.first);
    return ids;
}
} // namespace rescue
