#include "rescue/push_task.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace rescue {
const char *PushTask::name(PushState s) {
    switch (s) {
    case PushState::WAIT_START: return "WAIT_START";
    case PushState::START_ADVANCE: return "START_ADVANCE";
    case PushState::SCAN: return "SCAN";
    case PushState::LOST_SEARCH: return "LOST_SEARCH";
    case PushState::MID_REACQUIRE: return "MID_REACQUIRE";
    case PushState::MID_APPROACH: return "MID_APPROACH";
    case PushState::NEAR_REACQUIRE: return "NEAR_REACQUIRE";
    case PushState::SELECT_CARGO: return "SELECT_CARGO";
    case PushState::CUE_APPROACH: return "CUE_APPROACH";
    case PushState::CUE_PREPARE: return "CUE_PREPARE";
    case PushState::CLEAR_PILE: return "CLEAR_PILE";
    case PushState::CLEAR_PAUSE: return "CLEAR_PAUSE";
    case PushState::APPROACH: return "APPROACH";
    case PushState::PREPARE: return "PREPARE";
    case PushState::RUSH: return "RUSH";
    case PushState::LOWER_FRAME: return "LOWER_FRAME";
    case PushState::VERIFY_CAPTURE: return "VERIFY_CAPTURE";
    case PushState::CARRY: return "CARRY";
    case PushState::GATE: return "GATE";
    case PushState::RAISE_RELEASE: return "RAISE_RELEASE";
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
    track_close_ = false; last_range_ = std::numeric_limits<float>::infinity();
    retreat_origin_valid_ = false; retreat_mode_ = 0;
    if (s == PushState::START_ADVANCE) delivery_seen_ = false;
    if (why && *why) reason_ = why;
}
void PushTask::clearTrip() {
    cue_mode_=false;clear_commanded_us_=clear_planned_us_=0;
    search_cue_id_=capture_target_id_=-1;
    drop_locked_=false;drop_centre_zone_={};
    delivery_seen_=false; zone_search_started_us_=0;
    approach_forward_=0;
    target_id_ = -1; label_.clear(); trip_ = {}; pending_ = {}; seen_ = {};
    verdict_ = RuleVerdict::OK;
}
void PushTask::blacklist(uint64_t now) {
    if (target_id_ >= 0) rejected_[target_id_] = now + t_.blacklist_us;
}
// Unrecoverable without an operator: stop, keep the gripper as commanded.
void PushTask::fail(const char *why) { fault_ = true; reason_ = why; }
void PushTask::commandFrame(uint8_t open, uint64_t now) {
    if (frame_raised_ != open) { frame_raised_ = open; gripper_us_ = now; }
}
int PushTask::frameWait(const PushObservation &in, uint8_t open, uint64_t now) const {
    // A done flag counts only for the action matching the current target state.
    if (frame_raised_ == open && in.gripper_done && in.gripper_feedback_open == open) return 1;
    // Timeout counts from the later of the command and the phase start, so an action
    // commanded earlier still gets a full window when a new phase depends on it.
    return frame_raised_ == open && now - std::max(gripper_us_, phase_us_) > t_.gripper_timeout_us ? -1 : 0;
}
bool PushTask::candidate(const PushObservation &in, uint64_t now) const {
    if (in.target_is_search_cue || !in.target_valid || in.target_id < 0 || !in.geometry_valid || !in.target_region_valid || in.target_in_zone) return false;
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
    case PushState::MID_REACQUIRE: case PushState::MID_APPROACH: return t_.intermediate_pitch_cdeg;
    case PushState::CUE_APPROACH: return track_close_ ? t_.track_pitch_cdeg : t_.far_pitch_cdeg;
    case PushState::CUE_PREPARE: case PushState::CLEAR_PILE: case PushState::CLEAR_PAUSE:
        return t_.intermediate_pitch_cdeg!=kCameraPitchInvalid?t_.intermediate_pitch_cdeg:t_.track_pitch_cdeg;
    case PushState::NEAR_REACQUIRE: case PushState::SELECT_CARGO: return t_.track_pitch_cdeg;
    case PushState::LOST_SEARCH: return lost_round_ && midPitch() ? t_.intermediate_pitch_cdeg : t_.far_pitch_cdeg;
    case PushState::SCAN: case PushState::TURN_SCAN: case PushState::VERIFY_DELIVERY: return t_.far_pitch_cdeg;
    case PushState::APPROACH: return t_.track_pitch_cdeg;
    case PushState::PREPARE: return step_ == 0 ? t_.near_pitch_cdeg : t_.track_pitch_cdeg;
    case PushState::RUSH: case PushState::RAISE_RELEASE: return t_.track_pitch_cdeg;
    case PushState::LOWER_FRAME: case PushState::VERIFY_CAPTURE: return t_.near_pitch_cdeg;
    case PushState::CARRY: case PushState::GATE: // step 1: stopped NEAR hold check
        return step_ ? t_.near_pitch_cdeg : t_.far_pitch_cdeg;
    case PushState::CAPTURE_FAIL: case PushState::LOST_HOLD: case PushState::ABORT_DROP: return t_.far_pitch_cdeg;
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
float PushTask::holdCenter() const {
    return trip_.injured > 0 ? t_.injured_hold_center_y_m : t_.hold_center_y_m;
}
// After a stop the gripper state decides where to resume: a verified load is never
// silently forgotten, and anything uncertain is released before scanning again.
PushState PushTask::resumeState() const {
    switch (stopped_from_) {
    case PushState::CARRY: case PushState::GATE: return PushState::CARRY;
    case PushState::WAIT_START: return PushState::START_ADVANCE;
    case PushState::START_ADVANCE: return PushState::START_ADVANCE;
    case PushState::MID_REACQUIRE: case PushState::MID_APPROACH:
    case PushState::NEAR_REACQUIRE: case PushState::SELECT_CARGO: case PushState::LOST_SEARCH: case PushState::SCAN: case PushState::APPROACH:
    case PushState::TURN_SCAN: return PushState::SCAN;
    case PushState::PREPARE: return PushState::APPROACH;
    case PushState::CUE_APPROACH: case PushState::CUE_PREPARE:
    case PushState::CLEAR_PILE: case PushState::CLEAR_PAUSE: return PushState::SCAN;
    case PushState::RAISE_RELEASE: case PushState::ENTER: case PushState::BACK_OUT:
        return PushState::BACK_OUT; // the load is at our zone; finish and verify
    case PushState::VERIFY_DELIVERY: return PushState::VERIFY_DELIVERY;
    default: return PushState::ABORT_DROP;
    }
}

PushOutput PushTask::update(const PushObservation &in) {
    MotionCommand motion; // zero velocity by default
    const uint64_t now = in.now_us;
    const auto result = [&] {
        motion.gripper_offset = frame_raised_ ? 20 : 0; // stops hold the gripper, never toggle it
        const int16_t want = desiredPitch(); // a phase change commands its preset at once
        if (want != kCameraPitchInvalid && want != pitch_cmd_) { pitch_cmd_ = want; pitch_wait_us_ = now; }
        motion.camera_pitch_cdeg = pitch_cmd_;
        last_vx_ = motion.vx_mps; last_wz_ = motion.wz_rps;
        PushOutput out;
        out.state = state_; out.motion = motion; out.target_id = target_id_;
        out.target_is_search_cue = cue_mode_;
        out.search_cue_id=search_cue_id_; out.capture_target_id=capture_target_id_;
        out.cue_contact_allowed=false; // searching never grants contact permission
        out.clearing_attempts=clear_attempts_;
        out.startup_commanded_us=startup_commanded_us_;out.clear_commanded_us=clear_commanded_us_;
        out.batch_size = trip_.total() ? trip_.total() : pending_.total();
        out.delivered_total = total_; out.first_ordinary_delivered = first_;
        out.cargo_injured=trip_.injured>0;out.drop_locked=drop_locked_;out.drop_centre_zone=drop_centre_zone_;
        out.verdict = verdict_; out.reason = reason_;
        return out;
    };
    if (in.reset) {
        clearTrip(); rejected_.clear(); total_ = 0; first_ = false; fault_ = false;clear_attempts_=0;near_failures_=0;lost_round_=0;startup_commanded_us_=0;
        frame_raised_ = 0; gripper_us_ = now; stopped_from_ = PushState::WAIT_START;
        pitch_cmd_ = t_.far_pitch_cdeg; pitch_wait_us_ = now;
        enter(PushState::WAIT_START, now, "reset"); last_us_ = now;
        return result();
    }
    const bool timely = now > 0 && (last_us_ == 0 || (now > last_us_ && now - last_us_ <= t_.frame_timeout_us));
    const float dt = timely && last_us_ ? float(now - last_us_) * 1e-6f : 0.f;
    last_us_ = now;
    travel_ += std::abs(last_vx_) * dt; carried_ += std::abs(last_vx_) * dt;
    if (last_vx_ > 0) approach_forward_ += last_vx_ * dt;
    if(state_==PushState::START_ADVANCE && last_vx_>0 && timely)
        startup_commanded_us_ += static_cast<uint64_t>(std::llround(double(dt)*1000000));
    if(state_==PushState::CLEAR_PILE && last_vx_>0 && timely)
        clear_commanded_us_ += static_cast<uint64_t>(std::llround(double(dt)*1000000));
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
    const bool scan_ok = in.opponent_zone_clear && (in.directional_clearance_valid?in.turn_safe:in.path_safe);
    const bool observed_zone=zoneOk(in) && in.zone_estimate.source!=ZoneEstimate::Source::PREDICTED;
    const uint64_t elapsed = now - phase_us_;
    // distance_m is radial; gripper planes are longitudinal robot-frame y.
    const float target_forward_m = in.distance_m * std::cos(in.heading_error);
    const auto drive = [&](float vx, float wz) {
        const bool turn_blocked=in.directional_clearance_valid && wz!=0 &&
            !(vx==0?in.turn_safe:(vx>0 && in.arc_safe));
        if (turn_blocked) {reason_="turn_sweep_blocked";return;}
        if ((vx > 0 && !forward_ok) || (vx < 0 && !in.retreat_safe) || (vx == 0 && wz != 0 && !scan_ok))
        {
            if (state_ != PushState::SCAN && state_ != PushState::START_ADVANCE)
                reason_ = vx < 0 ? "retreat_clearance_missing" :
                !in.path_safe ? "forward_path_blocked" : "opponent_clearance_missing";
            return; // missing clearance stops the robot; it never substitutes another motion
        }
        motion.vx_mps = std::clamp(vx, -t_.max_speed, t_.max_speed);
        // Preserve zero and direction; never let the floor exceed the configured cap.
        if(wz!=0) wz=std::copysign(std::max(std::abs(wz),std::min(t_.min_turn_wz,t_.max_wz)),wz);
        motion.wz_rps = std::clamp(wz, -t_.max_wz, t_.max_wz);
    };
    // Steer to a body-frame bearing; turn in place when the error is large.
    const auto steer = [&](float speed, float heading) {
        if (std::abs(heading) > t_.rotate_in_place_rad) drive(0, t_.heading_gain * heading);
        else drive(speed, std::abs(heading) <= t_.heading_deadband_rad ? 0.f : t_.heading_gain * heading);
    };
    const auto trackingReason = [&]() -> const char* {
        if (!in.target_valid) return "target_missing";
        if (in.target_id != target_id_) return "target_id_mismatch";
        if (in.label != label_) return "target_label_mismatch";
        if (!in.geometry_valid) return "target_geometry_invalid";
        if (!in.target_region_valid) return "target_region_unknown";
        if (in.target_in_zone) return "target_already_in_zone";
        return "target_distance_or_heading_invalid";
    };
    const auto lostTarget = [&] { return ++misses_ > t_.hold_grace_frames; };
    const auto searchLost = [&] {
        if(frame_raised_!=0 || trip_.total()>0) {fail("lost_search_jaw_or_load_unsafe");state_=PushState::SAFE_STOP;return;}
        clearTrip();lost_round_=0;enter(PushState::LOST_SEARCH,now,"lost_target_search");
    };
    // Stopped NEAR check: the load must be seen as the verified trip on consecutive frames.
    // Out of view at NEAR counts as missing too: there it should be visible.
    const auto holdCheck = [&] {
        if (holdSeen(in) == 1) { hold_misses_ = 0; return ++confirmations_ >= t_.hold_check_frames ? 1 : 0; }
        confirmations_ = 0;
        return ++hold_misses_ > t_.hold_grace_frames ? -1 : 0;
    };
    // Gripper commands above stay concurrent with the camera; anything that moves, tracks or
    // counts waits for the preset (frames during a move carry no reliable geometry or hold).
    if ((state_ == PushState::CAPTURE_FAIL || state_ == PushState::LOST_HOLD ||
         state_ == PushState::ABORT_DROP) && in.opponent_zone_clear)
        commandFrame(1, now); // release at rest while recovering the FAR view
    if (state_ != PushState::PREPARE && state_ != PushState::RAISE_RELEASE) {
        const int p = pitchWait(in, now);
        if (p < 0) { fail("camera_pitch_timeout"); state_ = PushState::SAFE_STOP; return result(); }
        if (p == 0) { reason_="camera_pitch_wait"; return result(); }
    }
    const bool drop_fresh=in.navigation_timestamp_us && now>=in.navigation_timestamp_us &&
        now-in.navigation_timestamp_us<=200000 && in.drop_plan_valid && drop_locked_ &&
        std::isfinite(in.drop_centre_zone.x) && std::isfinite(in.drop_centre_zone.y) &&
        cv::norm(in.drop_centre_zone-drop_centre_zone_)<=.005f;
    if(((state_==PushState::GATE && !step_) || state_==PushState::RAISE_RELEASE || state_==PushState::ENTER) && !drop_fresh) {
        reason_="drop_revalidation_missing";
        if(now-carry_us_>t_.carry_budget_us) {fail("drop_revalidation_timeout");state_=PushState::SAFE_STOP;}
        return result();
    }
    const auto cueCandidate = [&] {
        return t_.enable_search_cues &&
            in.target_is_search_cue && in.target_valid && in.target_id>=0 &&
            targetKind(in.label)!=TargetKind::UNKNOWN &&
            in.geometry_valid && in.target_region_valid && !in.target_in_zone &&
            std::isfinite(in.distance_m) && in.distance_m>=0 && std::isfinite(in.heading_error);
    };
    switch (state_) {
    case PushState::START_ADVANCE:
        commandFrame(0, now);
        if (startup_commanded_us_ >= t_.startup_advance_us) {
            enter(PushState::SCAN, now, "startup_advance_complete");
            break;
        }
        if(elapsed > t_.startup_advance_us+5000000) {fail("startup_forward_blocked");break;}
        reason_=forward_ok?"startup_forward":"startup_path_blocked";
        drive(t_.startup_advance_speed, 0);
        break;
    case PushState::SCAN: {
        commandFrame(0, now);
        approach_forward_ = 0; // scans turn in place: the retreat bound restarts at the lock
        const bool cue=cueCandidate();
        if (!candidate(in, now) && !cue) {
            confirmations_ = 0; target_id_ = -1;cue_mode_=false;
            reason_=scan_ok?"searching_target":"scan_path_blocked";
            if(scan_ok && in.target_is_search_cue && clear_attempts_>=t_.max_clear_attempts)reason_="cue_attempt_limit";
            drive(0, t_.scan_wz); break;
        }
        if (target_id_ != in.target_id || label_ != in.label || cue_mode_!=cue) {
            target_id_ = in.target_id; label_ = in.label; confirmations_ = 0;cue_mode_=cue;
            if(cue) {search_cue_id_=target_id_;capture_target_id_=-1;}
            else {capture_target_id_=target_id_;search_cue_id_=-1;}
        }
        if (++confirmations_ >= t_.confirm_frames)
            enter(cue?PushState::CUE_APPROACH:PushState::APPROACH, now, cue?"search_cue_locked":"target_locked");
        break;
    }
    case PushState::LOST_SEARCH: {
        approach_forward_ = 0;
        // pitchWait above finishes FAR transition before wheels can turn.
        if(step_==0){step_=1;phase_us_=now;turned_=0;heading_seen_=false;}
        if(!in.gripper_closed_observed || !in.heading_valid || !std::isfinite(in.heading_rad)) {
            fail("lost_search_feedback_missing");state_=PushState::SAFE_STOP;break;
        }
        if(now-phase_us_>=20000000 || turned_>=2*float(CV_PI)) {
            // A FAR sweep misses cargo right in front of the jaw; stop, tilt to 20deg, sweep again.
            if(!lost_round_ && midPitch()) {lost_round_=1;enter(PushState::LOST_SEARCH,now,"lost_search_mid_pitch");break;}
            fail("lost_search_exhausted");state_=PushState::SAFE_STOP;break;
        }
        if(heading_seen_)turned_+=std::abs(wrapAngle(in.heading_rad-last_heading_));
        last_heading_=in.heading_rad;heading_seen_=true;
        if(candidate(in,now)) {
            if(target_id_!=in.target_id || label_!=in.label){target_id_=in.target_id;label_=in.label;confirmations_=0;}
            if(++confirmations_>=t_.confirm_frames) {
                if(lost_round_ && in.distance_m<=t_.track_near_m) {
                    // Close cargo found at 20deg: keep this view instead of tilting back to FAR.
                    target_id_=capture_target_id_=-1;cue_mode_=false;
                    enter(PushState::MID_REACQUIRE,now,"lost_target_reacquired_mid");break;
                }
                search_cue_id_=target_id_;capture_target_id_=-1;cue_mode_=true;
                enter(PushState::CUE_APPROACH,now,"lost_target_reacquired");
            }
            break; // stop while confirming a newly seen cargo candidate
        }
        target_id_=-1;confirmations_=0;
        reason_=scan_ok?(lost_round_?"rotating_for_lost_target_mid":"rotating_for_lost_target"):"lost_search_turn_blocked";
        drive(0,t_.scan_wz);break;
    }
    case PushState::CUE_APPROACH: {
        commandFrame(0,now);
        // First-trip green preempts only a search cue, never an active cargo batch.
        if(!first_ && label_!="ordinary_supply" && in.label=="ordinary_supply" && cueCandidate()) {
            clearTrip();enter(PushState::SCAN,now,"green_priority_reselect");break;
        }
        if(!cueCandidate() || in.target_id!=search_cue_id_ || in.label!=label_) {
            if(lostTarget()){if(t_.enable_lost_search)searchLost();else {blacklist(now);clearTrip();enter(PushState::SCAN,now,"search_cue_lost");}}
            break;
        }
        misses_=0;
        if(elapsed>t_.cue_approach_budget_us){blacklist(now);clearTrip();enter(PushState::SCAN,now,"cue_approach_timeout");break;}
        if(in.distance_m<=t_.track_near_m && t_.track_pitch_cdeg!=t_.far_pitch_cdeg) {
            target_id_=capture_target_id_=-1;cue_mode_=false;
            const bool mid=t_.intermediate_pitch_cdeg>t_.far_pitch_cdeg && t_.intermediate_pitch_cdeg<t_.track_pitch_cdeg;
            enter(mid?PushState::MID_REACQUIRE:PushState::NEAR_REACQUIRE,now,
                  mid?"mid_reacquire_pitch_wait":"near_reacquire_pitch_wait");break;
        }
        if(in.distance_m<=t_.track_near_m) track_close_=true;
        if(desiredPitch()!=pitch_cmd_) {reason_="near_camera_pitch_wait";break;}
        if(in.distance_m<=t_.rush_start_m) {
            target_id_=-1;cue_mode_=false;
            enter(PushState::SELECT_CARGO,now,"select_cargo_at_near_range");break;
        }
        steer(t_.cue_approach_speed,in.heading_error);break;
    }
    case PushState::MID_REACQUIRE: case PushState::NEAR_REACQUIRE: {
        const bool mid=state_==PushState::MID_REACQUIRE;
        // Preserve an already open jaw across a camera change; never close blindly.
        // Global pitchWait has completed. Give re-identification a full 3s at stable TRACK.
        if(step_==0){step_=1;phase_us_=now;confirmations_=0;}
        if(now-phase_us_>=3000000) {
            if(frame_raised_==1){fail("open_jaw_reacquire_timeout");state_=PushState::SAFE_STOP;break;}
            if(t_.enable_lost_search){searchLost();break;}
            ++near_failures_;
            if(near_failures_>=2){fail("near_reacquire_failed_stop");state_=PushState::SAFE_STOP;break;}
            clearTrip();enter(PushState::SCAN,now,"near_reacquire_retry_search");break;
        }
        if(!candidate(in,now)) {confirmations_=0;reason_=mid?"mid_no_valid_cargo":"near_no_valid_cargo";break;}
        if(frame_raised_==1 && (!in.corridor_complete || in.corridor!=pending_ ||
           checkTrip(in.corridor,first_)!=RuleVerdict::OK)) {
            confirmations_=0;reason_="near_open_jaw_batch_unconfirmed";break;
        }
        if(!forward_ok) {confirmations_=0;reason_=mid?"mid_cargo_path_blocked":"near_cargo_path_blocked";break;}
        if(capture_target_id_!=in.target_id || label_!=in.label) {
            capture_target_id_=in.target_id;label_=in.label;confirmations_=0;
        }
        if(++confirmations_>=t_.confirm_frames) {
            target_id_=capture_target_id_;enter(mid?PushState::MID_APPROACH:PushState::APPROACH,now,mid?"mid_cargo_reacquired":"near_cargo_reacquired");
        }
        break;
    }
    case PushState::MID_APPROACH: {
        if(!tracking(in)) {
            reason_=trackingReason();
            if(frame_raised_==1 && last_range_<=t_.open_jaw_handoff_m) {
                target_id_=capture_target_id_=-1;
                enter(PushState::NEAR_REACQUIRE,now,"open_jaw_lost_at_mouth_near_handoff");break;
            }
            if(lostTarget()) {
                if(t_.enable_lost_search){searchLost();break;}
                if(frame_raised_==1){fail("open_jaw_target_lost");state_=PushState::SAFE_STOP;}
                else if(++near_failures_>=2){fail("mid_tracking_failed_stop");state_=PushState::SAFE_STOP;}
                else {target_id_=capture_target_id_=-1;enter(PushState::MID_REACQUIRE,now,"mid_target_lost_reacquire");}
            }
            break;
        }
        misses_=0;last_range_=in.distance_m;
        if(!forward_ok && frame_raised_==0 && t_.enable_short_push && clear_attempts_<2 &&
           in.clear_push_safe && std::abs(in.heading_error)<=t_.rush_heading_rad) {
            enter(PushState::CUE_PREPARE,now,"closed_push_prepare");break;
        }
        if(!forward_ok) {
            if(frame_raised_==1){fail("open_jaw_path_blocked");state_=PushState::SAFE_STOP;break;}
            blacklist(now);target_id_=capture_target_id_=-1;
            enter(PushState::MID_REACQUIRE,now,"blocked_cargo_reselect");
            break;
        }
        if(elapsed>t_.approach_budget_us){fail("mid_approach_timeout");state_=PushState::SAFE_STOP;break;}
        if(frame_raised_==0 && in.distance_m<=t_.rush_start_m) {
            if(std::abs(in.heading_error)>t_.rush_heading_rad) {drive(0,t_.heading_gain*in.heading_error);break;}
            verdict_=checkCorridor(in.corridor,in.corridor_complete,in.corridor_occlusion_free,label_,first_);
            if(verdict_!=RuleVerdict::OK) {
                if(t_.enable_short_push && clear_attempts_<2 && in.clear_push_safe &&
                   in.corridor_complete && in.corridor.unknown==0 &&
                   (in.corridor.dangerous>0 || in.corridor.injured>0 || in.corridor.core>0)) {
                    enter(PushState::CUE_PREPARE,now,"closed_push_prepare");break;
                }
                blacklist(now);target_id_=capture_target_id_=-1;
                if(++near_failures_>=2){fail("mid_open_corridor_rejected");state_=PushState::SAFE_STOP;}
                else enter(PushState::MID_REACQUIRE,now,"mid_open_corridor_reselect");
                break;
            }
            if(confirmations_==0 || seen_!=in.corridor){seen_=in.corridor;confirmations_=0;}
            if(++confirmations_<t_.confirm_frames){reason_="mid_open_corridor_confirm";break;}
            if(in.directional_clearance_valid&&!in.jaw_open_safe){fail("jaw_open_sweep_blocked");state_=PushState::SAFE_STOP;break;}
            pending_=in.corridor;commandFrame(1,now);reason_="mid_open_command";break;
        }
        if(frame_raised_==1) {
            const int g=frameWait(in,1,now);
            if(g<0){fail("mid_gripper_open_timeout");state_=PushState::SAFE_STOP;break;}
            if(g==0){reason_="mid_gripper_feedback_wait";break;}
            verdict_=checkCorridor(in.corridor,in.corridor_complete,in.corridor_occlusion_free,label_,first_);
            // Corridor differs from the batch the jaw opened for: stop and confirm the change for confirm_frames.
            // A confirmed legal batch replaces it; a confirmed illegal or incomplete one stops the task.
            if(verdict_!=RuleVerdict::OK || in.corridor!=pending_) {
                if(confirmations_==0 || seen_!=in.corridor || seen_verdict_!=verdict_)
                    {seen_=in.corridor;seen_verdict_=verdict_;confirmations_=0;}
                if(++confirmations_<t_.confirm_frames){reason_="open_jaw_corridor_recheck";break;}
                if(verdict_!=RuleVerdict::OK){fail("open_jaw_corridor_changed");state_=PushState::SAFE_STOP;break;}
                pending_=in.corridor;
            }
            confirmations_=0;
            if(in.distance_m<=t_.intermediate_to_near_m) {
                target_id_=capture_target_id_=-1;
                enter(PushState::NEAR_REACQUIRE,now,"near_reacquire_pitch_wait");break;
            }
        }
        steer(t_.approach_speed,in.heading_error);break;
    }
    case PushState::SELECT_CARGO: {
        commandFrame(0,now);
        if(elapsed>t_.prepare_budget_us) {
            if(search_cue_id_>=0)rejected_[search_cue_id_]=now+t_.blacklist_us;
            clearTrip();enter(PushState::SCAN,now,"no_eligible_cargo");break;
        }
        if(!candidate(in,now)) {confirmations_=0;reason_="waiting_eligible_cargo";break;}
        if(capture_target_id_!=in.target_id || label_!=in.label) {
            capture_target_id_=in.target_id;label_=in.label;confirmations_=0;
        }
        if(++confirmations_>=t_.confirm_frames) {
            target_id_=capture_target_id_;
            enter(PushState::APPROACH,now,"cargo_locked");
        }
        break;
    }
    case PushState::CUE_PREPARE: {
        commandFrame(0,now);
        if(!t_.enable_short_push || clear_attempts_>=2){fail("clear_attempt_limit");state_=PushState::SAFE_STOP;break;}
        if(!in.clear_push_safe){fail("clear_path_or_boundary_unverified");state_=PushState::SAFE_STOP;break;}
        if(!in.gripper_closed_observed || !in.heading_valid) {
            if(elapsed>t_.gripper_timeout_us){fail("clear_closed_feedback_missing");state_=PushState::SAFE_STOP;}
            break;
        }
        if(++confirmations_<t_.confirm_frames)break;
        clear_heading_rad_=in.heading_rad;clear_commanded_us_=0;
        ++clear_attempts_;enter(PushState::CLEAR_PILE,now,"closed_push_010");break;
    }
    case PushState::CLEAR_PILE: {
        commandFrame(0,now);
        if(!t_.enable_short_push || !in.clear_push_safe || !in.gripper_closed_observed ||
           !in.heading_valid || std::abs(wrapAngle(in.heading_rad-clear_heading_rad_))>.10f) {
            fail("clear_evidence_lost");state_=PushState::SAFE_STOP;break;
        }
        if(!tracking(in)) {fail("clear_target_unverified");state_=PushState::SAFE_STOP;break;}
        if(target_forward_m<=t_.grasp_trigger_y_m) {
            enter(PushState::CLEAR_PAUSE,now,"clear_target_distance_reached");break;
        }
        // Time/integrated command only bounds an attempt; it never proves arrival.
        if(elapsed>=t_.clear_wall_budget_us || travel_>=t_.clear_max_commanded_m) {
            fail("clear_distance_not_reached");state_=PushState::SAFE_STOP;break;
        }
        motion.vx_mps=std::min(t_.clear_speed,t_.max_speed);motion.wz_rps=0;break;
    }
    case PushState::CLEAR_PAUSE:
        commandFrame(0,now);
        if(elapsed>=500000) {
            clearTrip();enter(PushState::MID_REACQUIRE,now,"clear_reobserve_cargo");
        }
        break;
    case PushState::APPROACH:
        // Keep jaw state from MID_APPROACH; a camera change must not close on unseen cargo.
        if (!tracking(in)) {
            reason_=trackingReason();
            if(lostTarget()) {
                if(t_.enable_lost_search){searchLost();break;}
                if(t_.track_pitch_cdeg!=t_.far_pitch_cdeg) {
                    if(++near_failures_>=2){fail("near_tracking_failed_stop");state_=PushState::SAFE_STOP;}
                    else {target_id_=capture_target_id_=-1;cue_mode_=false;enter(PushState::NEAR_REACQUIRE,now,"near_target_lost_reacquire");}
                } else {clearTrip();enter(PushState::SCAN,now,"target_lost");}
            }
            break;
        }
        misses_ = 0;
        if (elapsed > t_.approach_budget_us) { blacklist(now); clearTrip(); enter(PushState::SCAN, now, "approach_timeout"); break; }
        // Tilt down to TRACK when close (hysteresis keeps it from toggling), stopped while it moves.
        if (in.distance_m <= t_.track_near_m) track_close_ = true;
        else if (in.distance_m > t_.track_near_m + .1f) track_close_ = false;
        if (desiredPitch() != pitch_cmd_) { reason_="camera_pitch_wait"; break; }
        reason_ = "approach_target";
        if (in.distance_m <= t_.rush_start_m) {
            if (std::abs(in.heading_error) <= t_.rush_heading_rad) {
                if(in.directional_clearance_valid&&!in.jaw_open_safe){fail("jaw_open_sweep_blocked");state_=PushState::SAFE_STOP;break;}
                enter(PushState::PREPARE, now, "rush_aligned");
                commandFrame(1, now); // PREPARE output is stationary and opens immediately.
            }
            else drive(0, t_.heading_gain * in.heading_error);
        } else steer(t_.approach_speed, in.heading_error);
        break;
    case PushState::PREPARE: {
        // Stopped: open the frame, then inventory everything the rush would sweep.
        commandFrame(1, now);
        const int p = pitchWait(in, now);
        if (p < 0) { fail("camera_pitch_timeout"); break; }
        if (p == 0) { reason_ = "camera_pitch_wait"; break; }
        const int g = frameWait(in, 1, now);
        // NEAR is a stationary opening check, not a calibrated driving view.
        // Do not require ground geometry while observing at 40 degrees.
        if (step_ == 0) {
            if (g < 0) { fail("gripper_open_timeout"); break; }
            if (g == 0) { reason_ = "gripper_feedback_wait"; break; }
            step_ = 1; phase_us_ = now; confirmations_ = 0; misses_ = 0;
            reason_ = "open_confirmed_return_to_tracking_pitch";
            break;
        }
        if (!tracking(in)) { reason_ = trackingReason(); if (lostTarget()) { clearTrip(); enter(PushState::SCAN, now, "target_lost"); } break; }
        misses_ = 0;
        if (g < 0) { fail("gripper_open_timeout"); break; }
        if (elapsed > t_.prepare_budget_us) {
            blacklist(now); clearTrip(); enter(PushState::SCAN, now, "corridor_unresolved"); break;
        }
        if (g == 0) { reason_ = "gripper_feedback_wait"; break; }
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
        travel_limit_ = std::max(0.f, target_forward_m - t_.grasp_trigger_y_m) + t_.rush_extra_m;
        break;
    }
    case PushState::RUSH: {
        if (travel_ >= travel_limit_ || elapsed > t_.rush_budget_us) { enter(PushState::CAPTURE_FAIL, now, "rush_overrun"); break; }
        if (!tracking(in)) {
            if (lostTarget()) enter(PushState::CAPTURE_FAIL, now, "rush_target_lost");
            break;
        }
        misses_ = 0;
        if (std::abs(in.heading_error) > t_.rush_abort_heading_rad) {
            enter(PushState::CAPTURE_FAIL, now, "rush_misaligned"); break;
        }
        // Recheck before every forward/close decision: a blue object may enter
        // the corridor after PREPARE. Never continue on incomplete evidence.
        verdict_ = checkCorridor(in.corridor, in.corridor_complete,
                                in.corridor_occlusion_free, label_, first_);
        if (verdict_ != RuleVerdict::OK) {
            reason_ = std::string("rush_corridor_") + verdictName(verdict_);
            blacklist(now); enter(PushState::CAPTURE_FAIL, now, ""); break;
        }
        if (target_forward_m <= t_.grasp_trigger_y_m) { enter(PushState::LOWER_FRAME, now, "frame_enclosure_position"); break; }
        drive(t_.rush_speed, t_.heading_gain * in.heading_error);
        break;
    }
    case PushState::LOWER_FRAME: {
        commandFrame(0, now);
        const int g = frameWait(in, 0, now);
        if (g < 0) { reason_ = "frame_lower_timeout"; enter(PushState::ABORT_DROP, now, ""); }
        else if (g > 0) enter(PushState::VERIFY_CAPTURE, now, "frame_lowered");
        break;
    }
    case PushState::VERIFY_CAPTURE: {
        if (elapsed > t_.verify_budget_us) { enter(PushState::CAPTURE_FAIL, now, "capture_unverified"); break; }
        if (in.hold_observable && in.captured && in.held_complete) {
            const auto verdict=checkTrip(in.held,first_);
            if(verdict!=RuleVerdict::OK && verdict!=RuleVerdict::EMPTY) {
                verdict_=verdict;reason_=std::string("held_")+verdictName(verdict);
                blacklist(now);enter(PushState::ABORT_DROP,now,"");break;
            }
        }
        if (!in.hold_observable || !in.captured || !in.held_complete || in.held.total() == 0 || in.held != pending_) {
            confirmations_ = 0; break;
        }
        if (confirmations_ == 0 || in.held != seen_) { seen_ = in.held; confirmations_ = 0; }
        if (++confirmations_ < t_.capture_frames) break;
        verdict_ = checkTrip(in.held, first_);
        if (verdict_ != RuleVerdict::OK) {
            reason_ = std::string("held_") + verdictName(verdict_);
            blacklist(now); enter(PushState::ABORT_DROP, now, ""); break;
        }
        trip_ = in.held; carry_us_ = now; zone_search_started_us_ = 0;
        enter(PushState::CARRY, now, "capture_verified");
        hold_seen_us_ = now; carried_ = 0;
        break;
    }
    case PushState::CARRY: {
        if (now - carry_us_ > t_.carry_budget_us) { enter(PushState::ABORT_DROP, now, "carry_timeout"); break; }
        if (!zoneOk(in)) {
            if (!zone_search_started_us_) zone_search_started_us_ = now;
            if (now-zone_search_started_us_ > t_.zone_search_budget_us) {
                fail("own_zone_search_timeout"); break; // hold the load and stop; no blind forward travel
            }
        } else zone_search_started_us_ = 0;
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
        if (!zoneOk(in)) {
            reason_="searching_own_zone";
            drive(0, t_.scan_wz); break; // rotate with closed gripper; never drive blind toward a guessed zone
        }
        reason_="carrying_to_own_zone";
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
        const cv::Point2f gate = z.zoneToBody({halfX(), -t_.gate_clearance_m - holdCenter()});
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
        if (step_) { // aligned and counted; stopped at NEAR: enter only on a confirmed enclosure
            const int h = holdCheck();
            if (h < 0) enter(PushState::LOST_HOLD, now, "hold_lost");
            else if (h > 0) enter(PushState::ENTER, now, "gate_aligned_frame_down");
            break;
        }
        if (holdSeen(in) == 0 && ++hold_misses_ > t_.hold_grace_frames) { enter(PushState::LOST_HOLD, now, "hold_lost"); break; }
        if (!zoneOk(in)) { if (lostTarget()) enter(PushState::CARRY, now, "zone_lost"); break; }
        misses_ = 0;
        const auto &z = in.zone_estimate;
        const float yaw = wrapAngle(z.yaw_body_rad); // heading error to face into the zone
        if (std::abs(yaw) > t_.gate_yaw_rad) { confirmations_ = 0; drive(0, t_.heading_gain * yaw); break; }
        const cv::Point2f hold = z.bodyToZone({0, holdCenter()});
        if (std::abs(hold.x - halfX()) > t_.gate_lateral_m ||
            std::abs(hold.y + t_.gate_clearance_m) > 2 * t_.gate_tolerance_m) {
            enter(PushState::CARRY, now, "gate_offset"); break;
        }
        // Baseline counts make the delivery check differential: objects already inside never count.
        if (!in.zone_counts_valid || !in.zone_inventory_complete ||
            in.zone_supply_count < 0 || in.zone_injured_count < 0) { confirmations_ = 0; break; }
        if (++confirmations_ < t_.confirm_frames) break;
        baseline_ = trip_.injured ? in.zone_injured_count : in.zone_supply_count;
        baseline_other_ = trip_.injured ? in.zone_supply_count : in.zone_injured_count;
        delivery_geometry_id_ = in.zone_estimate.geometry_id;
        delivery_delta_ = 0; delivery_frame_ = delivery_stable_since_ = 0;
        step_ = 1; confirmations_ = 0; hold_misses_ = 0; // the robot stays put: the baseline holds
        break;
    }
    case PushState::RAISE_RELEASE: {
        if(!in.opponent_zone_clear) {
            reason_="release_opponent_clearance_missing";
            if(elapsed>t_.gripper_timeout_us)fail("release_clearance_timeout");
            break;
        }
        commandFrame(1, now);
        const int p = pitchWait(in, now);
        if (p < 0) { fail("camera_pitch_timeout"); break; }
        const int g = frameWait(in, 1, now);
        if (g < 0) fail("gripper_open_timeout");
        else if (g > 0 && p > 0) enter(PushState::BACK_OUT, now, "frame_raised_at_drop");
        break;
    }
    case PushState::ENTER: {
        // Carry the enclosed load with frame DOWN into the correct half.
        commandFrame(0, now);
        const int lowered = frameWait(in, 0, now);
        if (lowered <= 0) {
            if (lowered < 0) fail("frame_lower_timeout");
            break;
        }
        if (!zoneOk(in)) { if (lostTarget()) enter(PushState::ABORT_DROP, now, "zone_lost"); break; }
        misses_ = 0;
        const auto &z = in.zone_estimate;
        const cv::Point2f hold = z.bodyToZone({0, holdCenter()});
        const float lateral = hold.x - halfX();
        if (in.delivery_observed) delivery_seen_ = true;
        // One leading object crossing the line must not strand the second object.
        // Advance the conservative load envelope to the locked drop position;
        // actual credited inventory is checked only after backing out.
        if (hold.y >= (drop_locked_?drop_centre_zone_.y:t_.deposit_y_m)) { enter(PushState::RAISE_RELEASE, now, "enclosed_load_at_drop"); break; }
        if (std::abs(lateral) > t_.enter_lateral_m) { enter(PushState::ABORT_DROP, now, "enter_lateral"); break; }
        if (elapsed > t_.enter_budget_us) { enter(PushState::ABORT_DROP, now, "enter_timeout"); break; }
        drive(t_.enter_speed, t_.heading_gain * wrapAngle(z.yaw_body_rad) + t_.lateral_gain * lateral);
        break;
    }
    case PushState::BACK_OUT: {
        // Keep the frame raised so reversing leaves the load behind.
        commandFrame(1, now);
        const int g = frameWait(in, 1, now);
        if (g <= 0) { if (g < 0) fail("gripper_open_timeout"); break; } // never drag the load out closed
        if (!observed_zone) {
            if (elapsed > t_.retreat_budget_us) fail("retreat_reference_lost");
            break;
        }
        const bool clear = observed_zone &&
            in.zone_estimate.bodyToZone({0, t_.mouth_y_m}).y <= t_.mouth_clear_y_m;
        if (clear) { enter(PushState::VERIFY_DELIVERY, now, "backed_out"); break; }
        if (elapsed > t_.retreat_budget_us) { fail("retreat_blocked"); break; }
        drive(-t_.back_speed, 0);
        break;
    }
    case PushState::VERIFY_DELIVERY: {
        commandFrame(1, now);
        const int expected = trip_.total();
        // A fleeting target crossing cannot credit the whole batch. Require a fresh,
        // complete inventory after the open gripper has cleared the entrance, and
        // a stable increase in the correct half with the other half unchanged.
        const auto& z = in.zone_estimate;
        const int count = trip_.injured ? in.zone_injured_count : in.zone_supply_count;
        const int other = trip_.injured ? in.zone_supply_count : in.zone_injured_count;
        const int delta = count - baseline_;
        const bool evidence = observed_zone && z.geometry_id == delivery_geometry_id_ &&
            in.zone_inventory_complete && in.zone_counts_valid && count >= 0 && other >= 0 &&
            other == baseline_other_ && delta > 0 && delta <= expected &&
            in.gripper_done && in.gripper_feedback_open == 1 &&
            z.bodyToZone({0, t_.mouth_y_m}).y <= t_.mouth_clear_y_m;
        if (!evidence) {
            confirmations_ = 0; delivery_delta_ = 0; delivery_stable_since_ = 0;
        } else if (z.frame_id != delivery_frame_) {
            if (delivery_delta_ != delta || !delivery_stable_since_) {
                confirmations_ = 0; delivery_stable_since_ = now; delivery_delta_ = delta;
            }
            ++confirmations_;
            delivery_frame_ = z.frame_id;
        }
        const bool stable = evidence && confirmations_ >= t_.delivery_frames &&
            delivery_stable_since_ && now - delivery_stable_since_ >= 300000;
        const bool expired = elapsed > t_.delivery_budget_us;
        if (!stable && !expired) break;
        const int credited = stable ? delivery_delta_ : 0;
        total_ += credited;
        if(credited==expected && credited>0){near_failures_=0;}
        if (credited > 0 && credited == expected && trip_.ordinary == trip_.total()) first_ = true;
        reason_ = credited == expected ? "delivered" : credited ? "partial_delivery" : "delivery_unverified";
        enter(PushState::TURN_SCAN, now, "");
        break;
    }
    case PushState::TURN_SCAN: {
        commandFrame(0, now);
        // A turn is complete only with measured IMU heading changes.
        if (in.heading_valid && std::isfinite(in.heading_rad)) {
            if (heading_seen_) turned_ += std::abs(wrapAngle(in.heading_rad - last_heading_));
            last_heading_ = in.heading_rad; heading_seen_ = true;
        } else heading_seen_ = false;
        if (turned_ >= t_.turn_min_rad) {
            clearTrip(); enter(PushState::SCAN, now, ""); break;
        }
        if (elapsed > t_.turn_budget_us) { fail("turn_heading_unverified"); break; }
        drive(0, t_.turn_wz);
        break;
    }
    case PushState::CAPTURE_FAIL: case PushState::LOST_HOLD: case PushState::ABORT_DROP: {
        // Release where the opponent zone is not involved, then reverse off the objects.
        if (step_ == 0) {
            if (!in.opponent_zone_clear) { if (elapsed > t_.gripper_timeout_us) fail("drop_blocked_opponent_zone"); break; }
            commandFrame(1, now);
            const int g = frameWait(in, 1, now);
            if (g < 0) { fail("gripper_open_timeout"); break; }
            if (g == 0) { reason_ = "gripper_feedback_wait"; break; }
            step_ = 1; phase_us_ = now;
        }
        if (retreat_mode_ < 2 && observed_zone) { retreat_mode_ = 1; retreat_zone_us_ = now; }
        if (retreat_mode_ == 0 ? now - phase_us_ >= t_.blind_retreat_wait_us
                               : retreat_mode_ == 1 && now - retreat_zone_us_ >= t_.blind_retreat_wait_us) {
            // No zone geometry at FAR: back off blind instead of standing on the objects.
            // travel_ keeps any measured reversing already done in this phase.
            if (!in.heading_valid || !std::isfinite(in.heading_rad)) { fail("retreat_unverified_no_heading"); break; }
            if (retreat_mode_ == 0) travel_ = 0;
            retreat_mode_ = 2; retreat_heading_ = in.heading_rad;
        }
        if (retreat_mode_ == 2) {
            const float limit = std::min(t_.abort_back_m, approach_forward_);
            if (travel_ >= limit) {
                if (state_ != PushState::LOST_HOLD) blacklist(now);
                clearTrip(); enter(PushState::SCAN, now, "blind_retreat_complete"); break;
            }
            if (!in.heading_valid || !std::isfinite(in.heading_rad)) { fail("blind_retreat_heading_missing"); break; }
            if (std::abs(wrapAngle(in.heading_rad - retreat_heading_)) > t_.blind_retreat_yaw_rad) {
                fail("blind_retreat_heading_drift"); break;
            }
            if (elapsed > t_.retreat_budget_us) { fail("retreat_unverified"); break; }
            reason_ = "blind_retreat";
            drive(-t_.back_speed, 0);
            break;
        }
        if (observed_zone && !retreat_origin_valid_) {
            retreat_origin_zone_=in.zone_estimate.bodyToZone({0,0});
            retreat_forward_zone_=in.zone_estimate.bodyToZone({0,1})-retreat_origin_zone_;
            retreat_geometry_id_=in.zone_estimate.geometry_id;
            retreat_origin_valid_=true;
        }
        const bool measured=retreat_origin_valid_ && observed_zone &&
            in.zone_estimate.geometry_id==retreat_geometry_id_;
        const float back=measured ? -(in.zone_estimate.bodyToZone({0,0})-retreat_origin_zone_).dot(retreat_forward_zone_) : 0;
        if (measured && back >= t_.abort_back_m) {
            if (state_ != PushState::LOST_HOLD) blacklist(now);
            clearTrip(); enter(PushState::SCAN, now, ""); break;
        }
        if (elapsed > t_.retreat_budget_us) { fail("retreat_unverified"); break; }
        if (measured) drive(-t_.back_speed, 0);
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
