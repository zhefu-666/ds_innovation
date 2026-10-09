#include "rescue/push_task.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
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
    state_ = s; phase_us_ = now; travel_ = 0; travel_limit_ = 0; sweep_phase_ = 0; rush_pulse_phase_ = 0; rush_pulse_count_ = 0;
    confirmations_ = 0; misses_ = 0; hold_misses_ = 0; step_ = 0; turned_ = 0; heading_seen_ = false;
    track_close_ = false; last_range_ = std::numeric_limits<float>::infinity();
    retreat_origin_valid_ = false; retreat_mode_ = 0; assume_blind_ = false;
    if (s == PushState::START_ADVANCE) delivery_seen_ = false;
    // 有20°中间角时，APPROACH沿用当前相机角（远处不提前下压），由接近距离分级切换；否则保持直接用TRACK角。
    if (s == PushState::APPROACH) approach_pitch_ = midPitch() ? pitch_cmd_ : t_.track_pitch_cdeg;
    approach_rebind_ = false;
    if (s != PushState::LOST_HOLD) corridor_retreat_ = false;
    if (why && *why) reason_ = why;
}
bool PushTask::uprightInCorridor(const PushObservation &in, uint64_t now, float &lateral) const {
    if (!in.upright_ts_us || now < in.upright_ts_us || now - in.upright_ts_us > t_.sweep_fresh_us) return false;
    const float d = in.distance_m;
    if (!std::isfinite(d) || d < .05f || !std::isfinite(in.heading_error)) return false;
    const cv::Point2f t(-d * std::sin(in.heading_error), d * std::cos(in.heading_error)), a(t.x / d, t.y / d);
    bool hit = false; float best = 0;
    for (const auto &u : in.upright_body) {
        if (!std::isfinite(u.x) || !std::isfinite(u.y) || cv::norm(u - t) < t_.sweep_min_sep_m) continue; // the target itself is not an obstacle
        const float along = u.x * a.x + u.y * a.y, side = u.x * a.y - u.y * a.x; // side > 0: right of the approach line
        if (along < 0 || along > d + t_.sweep_extra_m || std::abs(side) > t_.sweep_corridor_half_m) continue;
        if (!hit || std::abs(side) < best) { best = std::abs(side); lateral = side; hit = true; }
    }
    return hit;
}
void PushTask::clearTrip() {
    vref_used_ = false;carry_progress_us_=0;
    cue_mode_=false;clear_commanded_us_=clear_planned_us_=0;
    search_cue_id_=capture_target_id_=-1;
    drop_locked_=false;drop_centre_zone_={};assume_pushed_=false;enter_unconfirmed_=false;prepush_back_=false;align_pending_=push_hold_valid_=false;align_start_us_=0;repush_n_=verify_in_=verify_out_=0;verify_far_view_=false;verify_fix_us_=0;inside_obs_=0;inside_obs_ts_=0;probe_until_us_=probe_next_us_=0;probe_sign_=1.f;sweep_phase_=sweep_attempts_=sweep_seen_=0;delivery_visual_=false;cargo_clipped_=false;cargo_ref_us_=cargo_seen_us_=0;half_locked_=false;half_injured_=false;half_x_=0;
    delivery_seen_=false; zone_search_started_us_=0;
    approach_forward_=0; last_target_valid_=false;
    target_id_ = -1; label_.clear(); trip_ = {}; pending_ = {}; seen_ = {};
    verdict_ = RuleVerdict::OK;
}
void PushTask::blacklist(uint64_t now) {
    if (target_id_ >= 0) rejected_[target_id_] = now + t_.blacklist_us;
}
// Unrecoverable without an operator: stop, keep the gripper as commanded.
void PushTask::fail(const char *why) { fault_ = true; reason_ = why; }
void PushTask::commandFrame(uint8_t open, uint64_t now) {
    if (frame_raised_ != open) { frame_raised_ = open; gripper_us_ = now; ++frame_transaction_; }
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
    case PushState::VERIFY_DELIVERY: return assumeView() && !verify_far_view_ ? t_.assume_view_pitch_cdeg : t_.far_pitch_cdeg;
    case PushState::SCAN: case PushState::TURN_SCAN: return t_.far_pitch_cdeg;
    case PushState::APPROACH: return approach_pitch_!=kCameraPitchInvalid?approach_pitch_:t_.track_pitch_cdeg;
    // TEMP_ASSUMPTION：开框由MCU完成标志确认，不需要为此下压到40°；货物贴近时在40°画面顶边会被截断（box_at_image_edge），
    // 所以PREPARE/RUSH沿用接近阶段已经到位的角度，不再单独切NEAR/TRACK。
    case PushState::PREPARE: case PushState::RUSH:
        if (t_.assume_all_safe && approach_pitch_ != kCameraPitchInvalid) return approach_pitch_;
        return state_ == PushState::PREPARE && step_ == 0 ? t_.near_pitch_cdeg : t_.track_pitch_cdeg;
    case PushState::RAISE_RELEASE: return assumeView() ? t_.assume_view_pitch_cdeg : t_.track_pitch_cdeg;
    case PushState::LOWER_FRAME: return t_.near_pitch_cdeg;
    case PushState::VERIFY_CAPTURE: return multi_view_pitch_==kCameraPitchInvalid?t_.near_pitch_cdeg:multi_view_pitch_;
    case PushState::CARRY: case PushState::GATE: // step 1: stopped NEAR hold check
        return step_ ? t_.near_pitch_cdeg : t_.far_pitch_cdeg;
    case PushState::CAPTURE_FAIL: case PushState::LOST_HOLD: case PushState::ABORT_DROP: return t_.far_pitch_cdeg;
    case PushState::ENTER: case PushState::BACK_OUT: return assumeView() ? t_.assume_view_pitch_cdeg : t_.track_pitch_cdeg; // fence and mouth
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
    if (drop_locked_) return drop_centre_zone_.x;
    if (half_locked_) return half_x_;
    return trip_.injured > 0 ? t_.injured_half_x_m : t_.supply_half_x_m;
}
void PushTask::lockDropHalf(const char *source) {
    if (half_locked_) return;
    half_locked_ = true;
    half_injured_ = trip_.injured > 0;
    half_x_ = half_injured_ ? t_.injured_half_x_m : t_.supply_half_x_m;
    char b[320];
    std::snprintf(b, sizeof b,
        "[DROP_ZONE] event=lock half=%s target_x=%.3f hold_centre_y=%.3f cargo=o%d/c%d/i%d/d%d/u%d rule=%s source=%s layout=left_supply_right_injured\n",
        half_injured_ ? "injured" : "supply", half_x_, holdCenter(), trip_.ordinary, trip_.core, trip_.injured,
        trip_.dangerous, trip_.unknown, trip_.injured > 0 ? "any_injured->injured" : "no_injured->supply", source);
    dropzone_evt_ += b;
}
void PushTask::noteDropZone(const char *event, const cv::Point2f &hold) {
    char b[240];
    std::snprintf(b, sizeof b, "[DROP_ZONE] event=%s half=%s target_x=%.3f hold_x=%.3f hold_y=%.3f err_x=%.3f\n",
        event, half_injured_ ? "injured" : "supply", halfX(), hold.x, hold.y, hold.x - halfX());
    dropzone_evt_ += b;
}
void PushTask::noteCargo(const char *event, const cv::Point2f &rear, bool fix) {
    char b[200];
    std::snprintf(b, sizeof b, "[CARGO_ZONE] event=%s half=%s rear_x=%.3f rear_y=%.3f fix=%d repush=%d in=%d out=%d\n",
        event, half_injured_ ? "injured" : "supply", rear.x, rear.y, fix ? 1 : 0, repush_n_, verify_in_, verify_out_);
    dropzone_evt_ += b;
}
// Nearest visible cargo around our half, as the ground contact (near edge) in zone coordinates.
void PushTask::trackCargo(const PushObservation &in, uint64_t now) {
    cv::Point2f cb, cr; bool cfix = false; int ci = -1;
    if (!cargoZone(in, now, cb, cr, cfix, &ci) || in.cargo_ts_us == cargo_last_ts_) return;
    cargo_last_ts_ = in.cargo_ts_us; cargo_seen_us_ = now; cargo_body_ = cb; cargo_rear_x_ = cr.x; cargo_rear_y_ = cr.y; cargo_fix_ = cfix;
    cargo_clipped_ = imgAt(in, ci).clipped;
    if (!cargo_ref_us_ || cv::norm(cb - cargo_ref_body_) >= t_.assume_stuck_move_m || cv::norm(cr - cargo_ref_zone_) >= t_.assume_stuck_move_m) {
        cargo_ref_body_ = cb; cargo_ref_zone_ = cr; cargo_ref_us_ = now;
    }
}

float PushTask::backTarget(uint64_t now) const {
    float b = t_.assume_prepush_back_m + t_.assume_repush_back_m * repush_n_;
    if (cargoVisible(now))
        b += std::min(t_.assume_lateral_back_max_m, t_.assume_lateral_back_gain * std::max(0.f, std::abs(cargo_rear_x_ - halfX()) - t_.assume_push_dx_deadband_m));
    return b;
}

bool PushTask::cargoZone(const PushObservation &in, uint64_t now, cv::Point2f &body, cv::Point2f &rear, bool &fix, int *idx) const {
    fix = false;
    if (in.cargo_body.empty() || !(now >= in.cargo_ts_us && now - in.cargo_ts_us <= 300000)) return false;
    ZoneEstimate z;
    if (in.vref_valid && in.vref_zone.valid && in.vref_points >= 3 && in.vref_zone.timestamp_us == in.cargo_ts_us) { z = in.vref_zone; fix = true; }
    else if (zone_.valid(now)) z = zone_.predicted(now);
    else return false;
    // Stay on the block already being tracked (two adjacent blocks must not swap the reference every frame).
    const bool lock = cargo_seen_us_ && now - cargo_seen_us_ <= 600000;
    float best = 1e9f, best_lock = .10f; bool found = false, locked = false;
    for (size_t i = 0; i < in.cargo_body.size(); ++i) {
        const cv::Point2f &p = in.cargo_body[i];
        const cv::Point2f q = z.bodyToZone(p);
        if (p.y <= 0 || std::abs(q.x - halfX()) > t_.assume_cargo_x_tol_m + .10f || q.y < -.40f || q.y > .50f) continue;
        const float d = float(cv::norm(p)), dl = lock ? float(cv::norm(p - cargo_body_)) : 1e9f;
        if (dl < best_lock) { best_lock = dl; body = p; rear = q; found = locked = true; if (idx) *idx = int(i); }
        else if (!locked && d < best) { best = d; body = p; rear = q; found = true; if (idx) *idx = int(i); }
    }
    return found;
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
    multi_view_pitch_=in.multi_view_pitch;
    MotionCommand motion; // zero velocity by default
    const uint64_t now = in.now_us;
    const auto result = [&] {
        motion.gripper_offset = frameAngle(frame_raised_ ? FrameAction::Open : FrameAction::Close);
        motion.frame_transaction = frame_transaction_; // stops hold the gripper, never toggle it
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
        attempt_started_=0;bounded_approach_=bounded_retreat_=0;
        clearTrip(); rejected_.clear(); total_ = 0; first_ = t_.assume_injured_trip; fault_ = false;clear_attempts_=0;near_failures_=0;lost_round_=0;startup_commanded_us_=0;
        odo_ = ratio_est_ = 0; vref_used_ = false; zone_.reset(); ratio_snap_ = imu_seen_ = false; vref_fix_us_ = vref_seen_us_ = 0;
        frame_raised_ = 0; gripper_us_ = now; stopped_from_ = PushState::WAIT_START;
        pitch_cmd_ = t_.far_pitch_cdeg; pitch_wait_us_ = now;
        enter(PushState::WAIT_START, now, "reset"); last_us_ = now;
        return result();
    }
    const bool timely = now > 0 && (last_us_ == 0 || (now > last_us_ && now - last_us_ <= t_.frame_timeout_us));
    const float dt = timely && last_us_ ? float(now - last_us_) * 1e-6f : 0.f;
    last_us_ = now;
    odo_ += last_vx_ * dt;
    {   // TEMP_ASSUMPTION：里程系锚定的粗略区域位姿（IMU航向 + 按比例缩放的指令位移），视野丢失后继续可用
        float dyaw = last_wz_ * dt;
        if (in.heading_valid) {
            if (imu_seen_) dyaw = wrapAngle(in.heading_rad - last_imu_yaw_);
            last_imu_yaw_ = in.heading_rad; imu_seen_ = true;
        } else imu_seen_ = false;
        zone_.advance(last_vx_ * dt / drRatio(now), dyaw);
        const auto &fix = in.vref_zone;
        const char *why = "none";
        if (!in.vref_valid || !fix.valid) why = "no_fix";
        else if (in.vref_points < 3) why = "points<3";
        else if (!(now >= fix.timestamp_us && now - fix.timestamp_us <= 300000)) why = "stale_ts";
        else if (fix.timestamp_us == vref_fix_us_) why = "dup_ts";
        else {
            vref_fix_us_ = fix.timestamp_us;
            why = "rejected";
            if (zone_.update(fix, now)) {
                why = "accepted";
                vref_seen_us_ = now;
                const cv::Point2f hv = fix.bodyToZone({0, holdCenter()});
                if (!ratio_snap_) { ratio_hv_ = hv; ratio_odo_ = odo_; ratio_snap_ = true; }
                else {
                    const float vis = float(std::hypot(hv.x - ratio_hv_.x, hv.y - ratio_hv_.y)), cmd = odo_ - ratio_odo_;
                    if (vis >= .15f && cmd >= .30f) {
                        const float r = std::clamp(cmd / vis, 1.f, t_.assume_ratio_max);
                        ratio_est_ = ratio_est_ > 0 ? .5f * (ratio_est_ + r) : r;
                        ratio_hv_ = hv; ratio_odo_ = odo_;
                    } else if (cmd < -.05f) ratio_snap_ = false;
                }
            }
        }
        if (zone_.valid(now)) {
            const ZoneEstimate pz = zone_.predicted(now);
            const cv::Point2f ph = pz.bodyToZone({0, holdCenter()});
            char b[300];
            std::snprintf(b, sizeof b, "feed=%s n=%d fix_age_ms=%.0f seen_age_ms=%.0f acc=%d rej=%d repl=%d dist=%.3f dyaw=%.3f ratio=%.1f odo=%.3f pred_ox=%.3f pred_oy=%.3f pred_yaw=%.3f hold_x=%.3f hold_y=%.3f",
                why, in.vref_points, in.vref_valid ? double(now - fix.timestamp_us) / 1000. : -1., vref_seen_us_ ? double(now - vref_seen_us_) / 1000. : -1.,
                zone_.accepted, zone_.rejected, zone_.replaced, zone_.last_dist, zone_.last_dyaw, motionRatio(), odo_,
                pz.origin_body_m.x, pz.origin_body_m.y, pz.yaw_body_rad, ph.x, ph.y);
            anchor_dbg_ = b;
        } else anchor_dbg_ = std::string("feed=") + why + " zone=none";
    }
    travel_ += std::abs(last_vx_) * dt; carried_ += std::abs(last_vx_) * dt;
    if (last_vx_ > 0) approach_forward_ += last_vx_ * dt;
    if(state_==PushState::START_ADVANCE && last_vx_>0 && timely)
        startup_commanded_us_ += static_cast<uint64_t>(std::llround(double(dt)*1000000));
    if(state_==PushState::CLEAR_PILE && last_vx_>0 && timely)
        clear_commanded_us_ += static_cast<uint64_t>(std::llround(double(dt)*1000000));
    if(t_.attempt_budget_us && in.run){
        if(!attempt_started_)attempt_started_=now;
        if(last_vx_<0)bounded_retreat_-=last_vx_*dt;
        if(last_vx_>0 && trip_.total()==0)bounded_approach_+=last_vx_*dt;
        if(now<attempt_started_ || now-attempt_started_>t_.attempt_budget_us ||
            (t_.approach_limit_m>0&&bounded_approach_>=t_.approach_limit_m) ||
            (t_.retreat_limit_m>0&&bounded_retreat_>=t_.retreat_limit_m))fail("bounded_attempt_exhausted");
    }
    if (fault_) { state_ = PushState::SAFE_STOP; return result(); }
    if(t_.require_multi_view && state_!=PushState::WAIT_START && (!in.safety_ok||!timely)){
        fail(!timely?"stale_frame":"safety_veto");state_=PushState::SAFE_STOP;return result();
    }
    if (!in.run || !in.safety_ok || !timely) {
        if (state_ != PushState::WAIT_START) stopped_from_ = state_;
        enter(PushState::WAIT_START, now, !in.run ? "not_running" : !in.safety_ok ? "safety_veto" : "stale_frame");
        return result();
    }
    if (state_ == PushState::WAIT_START) {
        enter(resumeState(), now, "resume");
        if (state_ == PushState::CARRY) { carry_us_ = now; carry_progress_us_ = 0; step_ = 1; } // a pause is not carry time; re-check the hold
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
    if (tracking(in)) { last_target_xy_ = {-in.distance_m * std::sin(in.heading_error), target_forward_m}; last_target_valid_ = true; }
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
    if (state_ != PushState::PREPARE && state_ != PushState::RAISE_RELEASE && state_ != PushState::VERIFY_CAPTURE) {
        const int p = pitchWait(in, now);
        if (p < 0) { fail("camera_pitch_timeout"); state_ = PushState::SAFE_STOP; return result(); }
        if (p == 0) { reason_="camera_pitch_wait"; return result(); }
    }
    const bool drop_fresh=in.navigation_timestamp_us && now>=in.navigation_timestamp_us &&
        now-in.navigation_timestamp_us<=200000 && in.drop_plan_valid && drop_locked_ &&
        std::isfinite(in.drop_centre_zone.x) && std::isfinite(in.drop_centre_zone.y) &&
        cv::norm(in.drop_centre_zone-drop_centre_zone_)<=.005f;
    // TEMP_ASSUMPTION: assume mode treats the drop point as valid without navigation revalidation.
    if(!carryLogic() && ((state_==PushState::GATE && !step_) || state_==PushState::RAISE_RELEASE || state_==PushState::ENTER) && !drop_fresh) {
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
        if (++confirmations_ >= t_.confirm_frames) {
            if(t_.demo_search_only){fail("demo_target_found");state_=PushState::SAFE_STOP;}
            else enter(cue?PushState::CUE_APPROACH:PushState::APPROACH, now, cue?"search_cue_locked":"target_locked");
        }
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
                if(!t_.enable_search_cues) {
                    // 搜索线索关闭（演示模式）：CUE_APPROACH永远不会成立，重新找回后直接回到APPROACH。
                    capture_target_id_=target_id_;search_cue_id_=-1;cue_mode_=false;
                    enter(PushState::APPROACH,now,"lost_target_reacquired");break;
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
        if (sweep_phase_) {
            // In-place sweep of an on-end block beside the target: turn out by IMU angle, then back. Tracking may drop meanwhile.
            // The base turns at roughly a quarter of the command, so sweep_wz bypasses max_wz and the sweep time is not charged to the approach budget.
            const auto sweepDrive = [&](float wz) { drive(0, wz); if (motion.wz_rps != 0) motion.wz_rps = std::copysign(t_.sweep_wz, wz); };
            const auto sweepEnd = [&](const char *why) { phase_us_ += now - sweep_t0_us_; sweep_phase_ = 0; ++sweep_attempts_; sweep_seen_ = 0; reason_ = why; };
            if (!in.heading_valid || !std::isfinite(in.heading_rad)) { sweepEnd("sweep_upright_no_heading"); break; }
            const float dyaw = wrapAngle(in.heading_rad - sweep_yaw0_);
            const uint64_t se = now - sweep_start_us_;
            if (sweep_phase_ == 1 && (std::abs(dyaw) >= t_.sweep_turn_rad || se > t_.sweep_budget_us)) { sweep_phase_ = 2; sweep_start_us_ = now; }
            if (sweep_phase_ == 1) { reason_ = "sweep_upright_out"; sweepDrive(sweep_sign_ * t_.sweep_wz); break; }
            if (std::abs(dyaw) <= t_.sweep_return_tol_rad || now - sweep_start_us_ > t_.sweep_budget_us) { sweepEnd("sweep_upright_done"); break; }
            reason_ = "sweep_upright_back"; sweepDrive(dyaw > 0 ? -t_.sweep_wz : t_.sweep_wz); break;
        }
        if (rush_pulse_phase_) {
            // IMU-angle pulse then a short stop; vision re-checks the bearing afterwards. Pulse time is not charged to the approach budget.
            const auto pulseEnd = [&](const char *why) { phase_us_ += now - rush_pulse_t0_us_; rush_pulse_phase_ = 0; reason_ = why; };
            if (!in.heading_valid || !std::isfinite(in.heading_rad)) { rush_pulse_count_ = t_.rush_pulse_max; pulseEnd("rush_pulse_no_heading"); break; }
            const uint64_t pe = now - rush_pulse_start_us_;
            if (rush_pulse_phase_ == 1) {
                const float done = wrapAngle(in.heading_rad - rush_pulse_yaw0_) * (rush_pulse_delta_ > 0 ? 1.f : -1.f);
                if (done >= t_.rush_pulse_done_frac * std::abs(rush_pulse_delta_) || pe > t_.rush_pulse_budget_us) { rush_pulse_phase_ = 2; rush_pulse_start_us_ = now; reason_ = "rush_pulse_settle"; break; }
                reason_ = "rush_pulse_turn"; drive(0, std::copysign(t_.rush_pulse_wz, rush_pulse_delta_));
                if (motion.wz_rps != 0) motion.wz_rps = std::copysign(t_.rush_pulse_wz, rush_pulse_delta_);
                break;
            }
            if (pe < t_.rush_pulse_settle_us) { reason_ = "rush_pulse_settle"; break; }
            pulseEnd("rush_pulse_done");
        }
        if (!tracking(in)) {
            reason_=trackingReason();
            // 下压相机后跟踪器会给同一货物分配新ID：到位后若看到同标签的有效货物，改绑新ID继续接近。
            if(approach_rebind_ && now-rebind_since_us_>t_.approach_rebind_window_us) approach_rebind_=false;
            if(approach_rebind_ && desiredPitch()==pitch_cmd_ && in.camera_pitch_stable && in.target_valid &&
               in.target_id!=target_id_ && in.label==label_ && candidate(in,now)) {
                target_id_=capture_target_id_=in.target_id;misses_=0;
                reason_="approach_id_rebound_after_pitch";break;
            }
            // 分级下压后相机转动/重新选目标期间图像无效、跟踪器要重新确认：窗口内停车等待，不算丢目标。
            if(approach_rebind_) {misses_=0;break;}
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
        // 目标贴近（像素接近画面底边）时分级下压相机继续跟踪：只下压不回抬，停车等待到位。
        if (midPitch()) {
            const int16_t before = approach_pitch_;
            if (in.distance_m <= t_.intermediate_to_near_m) approach_pitch_ = t_.track_pitch_cdeg;
            else if (in.distance_m <= t_.track_near_m && approach_pitch_ == t_.far_pitch_cdeg) approach_pitch_ = t_.intermediate_pitch_cdeg;
            if (approach_pitch_ != before) {approach_rebind_ = true; rebind_since_us_ = now;}
        }
        if (desiredPitch() != pitch_cmd_) { reason_="camera_pitch_wait"; break; }
        reason_ = "approach_target";
        if (in.distance_m <= t_.rush_start_m) {
            if (std::abs(in.heading_error) <= t_.rush_heading_rad) {
                float lat = 0;
                if (t_.sweep_upright && sweep_attempts_ < t_.sweep_max_attempts && in.heading_valid && std::isfinite(in.heading_rad) &&
                    uprightInCorridor(in, now, lat)) {
                    if (++sweep_seen_ < t_.sweep_confirm) { reason_ = "sweep_upright_confirm"; break; }
                    sweep_phase_ = 1; sweep_yaw0_ = in.heading_rad; sweep_sign_ = lat > 0 ? -1.f : 1.f; sweep_start_us_ = sweep_t0_us_ = now; sweep_seen_ = 0;
                    reason_ = "sweep_upright_out"; drive(0, sweep_sign_ * t_.sweep_wz); if (motion.wz_rps != 0) motion.wz_rps = sweep_sign_ * t_.sweep_wz; break;
                }
                if (sweep_seen_ > 0) --sweep_seen_;
                if(in.directional_clearance_valid&&!in.jaw_open_safe){fail("jaw_open_sweep_blocked");state_=PushState::SAFE_STOP;break;}
                enter(PushState::PREPARE, now, "rush_aligned");
                commandFrame(1, now); // PREPARE output is stationary and opens immediately.
            }
            else if (t_.rush_pulse_turn && rush_pulse_count_ < t_.rush_pulse_max && in.heading_valid && std::isfinite(in.heading_rad) &&
                     std::abs(in.heading_error) <= t_.rush_pulse_max_rad) {
                ++rush_pulse_count_; rush_pulse_phase_ = 1; rush_pulse_yaw0_ = in.heading_rad; rush_pulse_delta_ = in.heading_error;
                rush_pulse_start_us_ = rush_pulse_t0_us_ = now; reason_ = "rush_pulse_turn";
                drive(0, std::copysign(t_.rush_pulse_wz, in.heading_error));
                if (motion.wz_rps != 0) motion.wz_rps = std::copysign(t_.rush_pulse_wz, in.heading_error);
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
        if (!tracking(in)) {
            reason_ = trackingReason();
            // TEMP_ASSUMPTION：方框张开后跟踪器会给同一货物发新ID；同标签有效货物直接改绑，不退回SCAN重来。
            const bool near_last = t_.assume_all_safe || (last_target_valid_ && std::isfinite(in.distance_m) && std::isfinite(in.heading_error) &&
                cv::norm(cv::Point2f(-in.distance_m * std::sin(in.heading_error), in.distance_m * std::cos(in.heading_error)) - last_target_xy_) <= t_.rebind_max_shift_m);
            if (rushLogic() && in.target_valid && in.target_id != target_id_ && in.label == label_ && candidate(in, now) && near_last) {
                target_id_ = capture_target_id_ = in.target_id; misses_ = 0;
                reason_ = "prepare_id_rebound_after_open"; break;
            }
            if (lostTarget()) { clearTrip(); enter(PushState::SCAN, now, "target_lost"); }
            break;
        }
        misses_ = 0;
        if (g < 0) { fail("gripper_open_timeout"); break; }
        if (elapsed > t_.prepare_budget_us) {
            blacklist(now); clearTrip(); enter(PushState::SCAN, now, "corridor_unresolved"); break;
        }
        if (g == 0) { reason_ = "gripper_feedback_wait"; break; }
        if (t_.assume_all_safe) {
            // TEMP_ASSUMPTION: an unresolved corridor counts as holding only the locked target.
            if (++confirmations_ < t_.confirm_frames) { reason_ = "assumed_corridor_confirm"; break; }
            Inventory only; only.add(label_);
            const bool seen_ok = checkCorridor(in.corridor, in.corridor_complete, in.corridor_occlusion_free, label_, first_) == RuleVerdict::OK;
            pending_ = seen_ok ? in.corridor : only; verdict_ = RuleVerdict::OK;
            enter(PushState::RUSH, now, seen_ok ? "corridor_ok" : "assumed_corridor_ok");
            assume_rush_end_ = std::max(0.f, target_forward_m - t_.assume_rush_trigger_m) * t_.assume_motion_ratio;
            travel_limit_ = assume_rush_end_ + t_.rush_extra_m * t_.assume_motion_ratio;
            break;
        }
        const auto verdict = checkCorridor(in.corridor, in.corridor_complete, in.corridor_occlusion_free, label_, first_);
        verdict_ = verdict;
        if (verdict == RuleVerdict::INCOMPLETE) { confirmations_ = 0; break; }
        if (confirmations_ == 0 || in.corridor != seen_) { seen_ = in.corridor; confirmations_ = 0; }
        if (++confirmations_ < t_.corridor_confirm_frames) break;
        if (verdict != RuleVerdict::OK) {
            const std::string why = std::string("corridor_") + verdictName(verdict) + "_back_off";
            if (++corridor_retries_ > t_.corridor_retry_max) { blacklist(now); corridor_retries_ = 0; }
            verdict_ = verdict; enter(PushState::LOST_HOLD, now, why.c_str()); corridor_retreat_ = true; break;
        }
        corridor_retries_ = 0;
        pending_ = in.corridor;
        enter(PushState::RUSH, now, "corridor_ok");
        if (rushLogic()) {
            assume_rush_end_ = std::max(0.f, target_forward_m - t_.assume_rush_trigger_m) * t_.assume_motion_ratio;
            travel_limit_ = assume_rush_end_ + t_.rush_extra_m * t_.assume_motion_ratio;
        } else travel_limit_ = std::max(0.f, target_forward_m - t_.grasp_trigger_y_m) + t_.rush_extra_m;
        break;
    }
    case PushState::RUSH: {
        if (travel_ >= travel_limit_ || elapsed > t_.rush_budget_us) {
            if (rushLogic()) { enter(PushState::LOWER_FRAME, now, "assumed_enclosure_rush_bound"); break; }
            enter(PushState::CAPTURE_FAIL, now, "rush_overrun"); break;
        }
        if (!tracking(in)) {
            if (rushLogic()) {
                // TEMP_ASSUMPTION: the target leaves the 5deg view below ~0.31 m; finish the last
                // estimated distance straight, then lower the frame.
                if (travel_ >= assume_rush_end_) { enter(PushState::LOWER_FRAME, now, "assumed_enclosure_target_below_view"); break; }
                reason_ = "assumed_blind_rush"; drive(t_.rush_speed, 0); break;
            }
            if (lostTarget()) enter(PushState::CAPTURE_FAIL, now, "rush_target_lost");
            break;
        }
        misses_ = 0;
        if (std::abs(in.heading_error) > t_.rush_abort_heading_rad) {
            enter(PushState::CAPTURE_FAIL, now, "rush_misaligned"); break;
        }
        // Recheck before every forward/close decision: a blue object may enter
        // the corridor after PREPARE. Never continue on incomplete evidence.
        if (rushLogic()) {
            // 实测指令行程约为实际位移的4~6倍：用视觉剩余距离乘以比例得到所需指令行程。
            assume_rush_end_ = travel_ + std::max(0.f, target_forward_m - t_.assume_rush_trigger_m) * t_.assume_motion_ratio;
            travel_limit_ = assume_rush_end_ + t_.rush_extra_m * t_.assume_motion_ratio;
        }
        verdict_ = t_.assume_all_safe ? RuleVerdict::OK : checkCorridor(in.corridor, in.corridor_complete,
                                in.corridor_occlusion_free, label_, first_);
        if (verdict_ != RuleVerdict::OK) {
            reason_ = std::string("rush_corridor_") + verdictName(verdict_);
            blacklist(now); enter(PushState::CAPTURE_FAIL, now, ""); break;
        }
        if (target_forward_m <= (rushLogic() ? t_.assume_rush_trigger_m : t_.grasp_trigger_y_m)) { enter(PushState::LOWER_FRAME, now, "frame_enclosure_position"); break; }
        drive(t_.rush_speed, t_.heading_gain * in.heading_error);
        break;
    }
    case PushState::LOWER_FRAME: {
        commandFrame(0, now);
        const int g = frameWait(in, 0, now);
        if (g < 0) { fail("frame_close_timeout"); state_=PushState::SAFE_STOP; }
        else if (g > 0) enter(PushState::VERIFY_CAPTURE, now, "frame_lowered");
        break;
    }
    case PushState::VERIFY_CAPTURE: {
        if(t_.require_multi_view) {
            const bool verified=in.multi_view_finished && in.multi_view_verdict==1 &&
                checkTrip(in.multi_view_inventory,first_)==RuleVerdict::OK && in.multi_view_inventory==pending_;
            const bool mv_conflict=in.multi_view_finished && in.multi_view_verdict==1 &&
                (checkTrip(in.multi_view_inventory,first_)!=RuleVerdict::OK || in.multi_view_inventory!=pending_);
            if(rushLogic() && !verified && (t_.assume_all_safe || !mv_conflict)) {
                // TEMP_ASSUMPTION: an unconfirmed enclosure counts as the batch the frame closed on (main path keeps a measured rule/inventory conflict fatal).
                if(elapsed<=8000000 && !in.multi_view_finished)break;
                Inventory assumed=pending_;if(!assumed.total())assumed.add(label_);
                trip_=assumed;carry_us_=now;carry_progress_us_=0;zone_search_started_us_=0;
                enter(PushState::CARRY,now,"assumed_capture");hold_seen_us_=now;carried_=0;break;
            }
            if(elapsed>8000000 || (in.multi_view_finished && in.multi_view_verdict!=1)) {
                fail("multi_view_unconfirmed");state_=PushState::SAFE_STOP;break;
            }
            if(!in.multi_view_finished)break;
            verdict_=checkTrip(in.multi_view_inventory,first_);
            if(verdict_!=RuleVerdict::OK || in.multi_view_inventory!=pending_){fail("multi_view_inventory_conflict");state_=PushState::SAFE_STOP;break;}
            trip_=in.multi_view_inventory;carry_us_=now;zone_search_started_us_=0;
            enter(PushState::CARRY,now,"multi_view_capture_verified");hold_seen_us_=now;carried_=0;break;
        }
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
        if (carryLogic()) {
            // TEMP_ASSUMPTION: the hold is assumed (no NEAR checks). With a fresh zone plan drive the
            // normal gate approach; otherwise search briefly, then a bounded straight leg to an assumed drop point.
            step_ = 0;
            const bool plan = zoneOk(in) && in.navigation_timestamp_us && now >= in.navigation_timestamp_us &&
                now - in.navigation_timestamp_us <= 200000 && in.drop_plan_valid && in.carry_plan_valid &&
                std::isfinite(in.drop_centre_zone.x) && std::isfinite(in.drop_centre_zone.y) &&
                std::isfinite(in.carry_waypoint_body.x) && std::isfinite(in.carry_waypoint_body.y);
            if (plan && !assume_blind_) {
                zone_search_started_us_ = 0;
                lockDropHalf("drop_plan");
                if (!drop_locked_) { drop_centre_zone_ = in.drop_centre_zone; drop_locked_ = true; }
                const cv::Point2f gate = in.zone_estimate.zoneToBody({halfX(), -t_.gate_clearance_m - holdCenter()});
                if (std::hypot(gate.x, gate.y) <= t_.gate_tolerance_m) { enter(PushState::GATE, now, "gate_reached"); break; }
                reason_ = "carrying_to_own_zone";
                steer(std::max(.03f, std::min(t_.carry_speed, float(cv::norm(in.carry_waypoint_body)))),
                      std::atan2(-in.carry_waypoint_body.x, in.carry_waypoint_body.y));
                break;
            }
            // 视觉闭环：YOLO-pose区域位姿锚定在里程系；沿区域中轴“追踪前视点”接近，丢失视野时用IMU航向+缩放位移继续推算。
            if (zone_.valid(now)) {
                const ZoneEstimate z = zone_.predicted(now);
                const bool fresh = now - vref_seen_us_ <= 400000;
                const cv::Point2f hold = z.bodyToZone({0, holdCenter()});
                vref_used_ = true; zone_search_started_us_ = 0;
                lockDropHalf("assumed_trip");
                if(!carry_progress_us_ || hold.y > carry_best_y_ + t_.assume_stall_progress_m) { carry_best_y_ = hold.y; carry_progress_us_ = now; }
            // Blocked by the zone rim: no forward progress near the front edge counts as arrived.
            // The IMU decides "pressed against the rim" (dead reckoning keeps advancing while the wheels slip); a vision-only no-progress check needs a fresh fix.
            const bool imu_blocked = in.imu_blocked_us >= t_.assume_imu_block_us && hold.y >= -t_.assume_imu_block_near_m;
            if (imu_blocked || (fresh && hold.y >= -t_.assume_stall_near_m && now - carry_progress_us_ >= t_.assume_stall_us)) {
                noteDropZone(imu_blocked ? "release_stall_imu" : "release_stall", hold);
                enter(PushState::RAISE_RELEASE, now, imu_blocked ? "assumed_drop_point_imu_blocked" : "assumed_drop_point_stalled"); break;
            }
            if (hold.y >= -t_.assume_release_gap_m) { noteDropZone("release", hold); enter(PushState::RAISE_RELEASE, now, fresh ? "assumed_drop_point_visual" : "assumed_drop_point_visual_dr"); break; }
                const cv::Point2f carrot = z.zoneToBody({halfX(), std::min(hold.y + t_.assume_lookahead_m, -t_.assume_release_gap_m)});
                reason_ = fresh ? "assumed_visual_carry" : "assumed_visual_dead_reckoning";
                const float h = std::atan2(-carrot.x, std::max(.02f, carrot.y - holdCenter()));
                if (std::abs(h) > t_.rotate_in_place_rad && hold.y < -t_.assume_release_gap_m - t_.assume_align_zone_m) {
                    // 离区前沿还远：边走边转，不原地等转向（原地转实测很慢，会卡几秒）；临近前沿再原地对正。
                    drive(t_.carry_speed * std::max(.4f, std::cos(std::min(std::abs(h), 1.f))), t_.heading_gain * h);
                } else steer(t_.carry_speed, h);
                break;
            }
            if (!assume_blind_) {
                if (!zone_search_started_us_) zone_search_started_us_ = now;
                if (now - zone_search_started_us_ < t_.assume_zone_search_us) { reason_ = "assumed_zone_search"; drive(0, t_.scan_wz); break; }
                assume_blind_ = true; travel_ = 0;
            }
            if (travel_ >= t_.assume_carry_m) { enter(PushState::RAISE_RELEASE, now, "assumed_drop_point"); break; }
            reason_ = "assumed_blind_carry"; drive(t_.carry_speed, 0); break;
        }
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
        if (carryLogic() && step_) { enter(PushState::ENTER, now, "assumed_gate_hold"); break; }
        if (step_) { // aligned and counted; stopped at NEAR: enter only on a confirmed enclosure
            const int h = holdCheck();
            if (h < 0) enter(PushState::LOST_HOLD, now, "hold_lost");
            else if (h > 0) enter(PushState::ENTER, now, "gate_aligned_frame_down");
            break;
        }
        if (!carryLogic() && holdSeen(in) == 0 && ++hold_misses_ > t_.hold_grace_frames) { enter(PushState::LOST_HOLD, now, "hold_lost"); break; }
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
            in.zone_supply_count < 0 || in.zone_injured_count < 0) {
            // TEMP_ASSUMPTION: without zone counts the delivery is credited by assumption later.
            if (carryLogic()) { baseline_ = baseline_other_ = -1; step_ = 1; break; }
            confirmations_ = 0; break;
        }
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
        else if (g > 0 && p > 0) {
            // TEMP_ASSUMPTION：快到区域先松开，再用下压(+20)姿态把块向区内推一段，之后再次抬框退出。
            if (carryLogic() && !assume_pushed_ && t_.assume_push_m > 0) {
                // 先退一小段让框离开块，再放下框从块后方推入；否则框会重新罩住块一起往里带。
                prepush_back_ = true; inside_obs_ = 0;
                prepush_hold_y_ = zone_.valid(now) ? zone_.predicted(now).bodyToZone({0, holdCenter()}).y : 0.f;
                enter(PushState::BACK_OUT, now, "assumed_release_back_off");
            }
            else enter(PushState::BACK_OUT, now, "frame_raised_at_drop");
        }
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
        if (carryLogic() && !assume_pushed_) {
            bool done = elapsed > t_.enter_budget_us;
            trackCargo(in, now);
            const bool cvis = cargoVisible(now);
            const float hy0 = zone_.valid(now) ? zone_.predicted(now).bodyToZone({0, holdCenter()}).y : 1e9f;
            // A box clipped by the image bottom puts the contact too deep, and an off-half block is not the delivery: neither counts as inside.
            const bool cinside = cvis && cargo_fix_ && !cargo_clipped_ && cargo_rear_y_ >= t_.assume_cargo_done_y_m
                && std::abs(cargo_rear_x_ - halfX()) <= t_.assume_cargo_x_tol_m && hy0 >= t_.assume_inside_min_hold_y_m;
            const bool stuck = !cinside && (in.imu_blocked_us >= t_.assume_imu_block_us || (cvis && cargo_ref_us_ && now - cargo_ref_us_ >= t_.assume_stuck_us));
            const float hy = zone_.valid(now) ? zone_.predicted(now).bodyToZone({0, holdCenter()}).y : 0.f;
            if (cinside) { noteCargo("push_inside", {cargo_rear_x_, cargo_rear_y_}, true); done = true; }
            else if (stuck) {
                if (repush_n_ < t_.assume_repush_max) {
                    ++repush_n_; noteCargo("stuck_repush", {cargo_rear_x_, cargo_rear_y_}, true);
                    prepush_back_ = true; prepush_hold_y_ = hy; cargo_ref_us_ = 0;
                    enter(PushState::BACK_OUT, now, "assumed_stuck_repush"); break;
                }
                noteCargo("stuck_giveup", {cargo_rear_x_, cargo_rear_y_}, true); done = true;
            }
            if (align_pending_ && !done) {
                // 后退后、前推前的视觉对准：货物可见则原地转到其方位在容差内；看不到则不拦（沿用推算）。
                if (!align_start_us_) align_start_us_ = now;
                const bool seen = cargoVisible(now) && now - cargo_seen_us_ <= 400000;
                const float brg = seen ? -std::atan2(cargo_body_.x, std::max(.08f, cargo_body_.y - holdCenter())) : 0.f;
                if (seen && std::abs(brg) > t_.assume_align_tol_rad && now - align_start_us_ < t_.assume_align_budget_us) {
                    reason_ = "assumed_push_align_turn";
                    drive(0, std::copysign(std::max(t_.min_turn_wz, std::min(t_.assume_align_wz, t_.heading_gain * std::abs(brg))), brg));
                    break;
                }
                char b[200];
                std::snprintf(b, sizeof b, "[CARGO_ZONE] event=%s half=%s bearing_deg=%.1f body_x=%.3f body_y=%.3f seen=%d waited_ms=%llu\n",
                    !seen ? "push_align_unseen" : std::abs(brg) <= t_.assume_align_tol_rad ? "push_aligned" : "push_align_giveup",
                    half_injured_ ? "injured" : "supply", brg * 57.2958f, cargo_body_.x, cargo_body_.y, seen ? 1 : 0,
                    (unsigned long long)((now - align_start_us_) / 1000));
                dropzone_evt_ += b;
                align_pending_ = false;
            }
            if (zone_.valid(now)) {
                const ZoneEstimate z = zone_.predicted(now);
                const cv::Point2f hold = z.bodyToZone({0, holdCenter()});
                // Fresh block sighting overrides dead-reckoned depth: keep pushing until it is really inside.
                done = done || hold.y >= t_.assume_push_depth_m + (cvis ? t_.assume_overshoot_m : 0.f);
                // Do not raise the frame on a timeout / dead-reckoned depth unless the load is confirmed inside.
                if (done && !cinside && !(stuck && !(repush_n_ < t_.assume_repush_max)) && t_.assume_enter_confirm) {
                    const bool real = cvis ? (!cargo_clipped_ && cargo_rear_y_ >= t_.assume_enter_true_y_m && std::abs(cargo_rear_x_ - halfX()) <= t_.assume_cargo_x_tol_m)
                                           : hold.y >= t_.assume_enter_hold_y_m;
                    if (!real && elapsed < t_.enter_budget_us + t_.assume_enter_extra_us && hold.y < t_.assume_enter_max_hold_y_m) {
                        done = false;
                        if (!enter_unconfirmed_) { enter_unconfirmed_ = true; noteCargo(cvis ? "push_unconfirmed_visible" : "push_unconfirmed_hidden", {cargo_rear_x_, cargo_rear_y_}, cvis); noteDropZone("push_unconfirmed", hold); }
                    } else if (!real) { noteCargo("push_unconfirmed_giveup", {cargo_rear_x_, cargo_rear_y_}, cvis); }
                    else if (enter_unconfirmed_) { noteCargo("push_confirmed_late", {cargo_rear_x_, cargo_rear_y_}, cvis); }
                }
                if (!done) {
                    reason_ = now - vref_seen_us_ <= 400000 ? "assumed_push_in_visual" : "assumed_push_in_dead_reckoning";
                    float aim = halfX(), phid = 0.f;
                    if (cvis) {
                        // Push the visible block along the line towards the half centre; the robot lines up behind it on that line.
                        const float dx = cargo_rear_x_ - halfX();
                        const float dxe = std::copysign(std::max(0.f, std::abs(dx) - t_.assume_push_dx_deadband_m), dx);
                        const float dy = std::max(.08f, t_.assume_goal_depth_m - cargo_rear_y_);
                        phid = std::clamp(std::atan2(dxe, dy), -t_.assume_push_angle_max_rad, t_.assume_push_angle_max_rad);
                        aim = std::clamp(cargo_rear_x_ + std::tan(phid) * (cargo_rear_y_ - hold.y), halfX() - .25f, halfX() + .25f);
                    }
                    const float speed = std::min(t_.max_speed, t_.enter_speed * (1.f + t_.assume_repush_speed_gain * repush_n_));
                    float err = t_.heading_gain * wrapAngle(z.yaw_body_rad + phid) + t_.lateral_gain * (hold.x - aim);
                    if (t_.assume_align_push) {
                        // 对准后按货物方位纯追踪（斜推角收小，防止框从货物旁滑过）；货物丢失后保持最后航向，不再向区中线拉回。
                        if (cvis && now - cargo_seen_us_ <= 400000) {
                            const float brg = -std::atan2(cargo_body_.x, std::max(.08f, cargo_body_.y - holdCenter()));
                            const float slant = std::clamp(phid, -t_.assume_slant_max_rad, t_.assume_slant_max_rad);
                            push_hold_phi_ = wrapAngle(-z.yaw_body_rad + brg + slant); push_hold_valid_ = true;
                            err = t_.heading_gain * (brg + slant);
                            reason_ = "assumed_push_in_pursuit";
                        } else if (push_hold_valid_) {
                            err = t_.heading_gain * wrapAngle(push_hold_phi_ + z.yaw_body_rad);
                            reason_ = "assumed_push_in_hold_heading";
                        }
                    }
                    float wz = std::abs(err) <= t_.heading_deadband_rad ? 0.f : err;
                    if (wz != 0.f) probe_until_us_ = 0;
                    else if (t_.assume_probe && !align_pending_ && elapsed >= t_.assume_probe_first_us) {
                        // Straight push commands no turn, and a stalled chassis cannot be told from a moving one by the IMU.
                        // A short alternating yaw pulse gives the gyro something to answer (MotionWatch blocked ratio).
                        if (now < probe_until_us_) wz = probe_sign_ * t_.min_turn_wz;
                        else if (now >= probe_next_us_) {
                            probe_sign_ = -probe_sign_; probe_until_us_ = now + t_.assume_probe_len_us;
                            probe_next_us_ = probe_until_us_ + t_.assume_probe_gap_us; wz = probe_sign_ * t_.min_turn_wz;
                        }
                        if (wz != 0.f) reason_ = "assumed_push_imu_probe";
                    }
                    drive(speed, wz);
                    break;
                }
            } else {
                done = done || travel_ >= t_.assume_push_m;
                if (!done) { reason_ = "assumed_push_in_blind"; drive(t_.enter_speed, 0); break; }
            }
            if (done) {
                if (zone_.valid(now)) noteDropZone("push_done", zone_.predicted(now).bodyToZone({0, holdCenter()}));
                assume_pushed_ = true; enter(PushState::RAISE_RELEASE, now, "assumed_push_in_done"); break;
            }
            reason_ = "assumed_push_in_frame_down"; drive(t_.enter_speed, 0); break;
        }
        if (!zoneOk(in)) {
            if (lostTarget()) {
                if (carryLogic()) enter(PushState::RAISE_RELEASE, now, "assumed_drop_point_zone_lost");
                else enter(PushState::ABORT_DROP, now, "zone_lost");
            }
            break;
        }
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
        if (prepush_back_) {
            trackCargo(in, now);
            const bool ins = repush_n_ == 0 && cargoVisible(now) && cargo_fix_ && !cargo_clipped_ && cargo_rear_y_ >= t_.assume_cargo_done_y_m
                && std::abs(cargo_rear_x_ - halfX()) <= t_.assume_cargo_x_tol_m;
            if (!ins) inside_obs_ = 0;
            else if (cargo_last_ts_ != inside_obs_ts_) { ++inside_obs_; inside_obs_ts_ = cargo_last_ts_; }
            if (ins && inside_obs_ >= t_.assume_inside_after_release_obs) { // several independent fixes, not one biased frame
                // Released deep enough: the block is already inside, no push needed.
                noteCargo("inside_after_release", {cargo_rear_x_, cargo_rear_y_}, true);
                prepush_back_ = false; assume_pushed_ = true; verify_in_ = verify_out_ = 0;
                enter(PushState::VERIFY_DELIVERY, now, "assumed_inside_after_release"); break;
            }
            const bool vis = zone_.valid(now);
            const float tgt = backTarget(now);
            const float back_done = vis ? prepush_hold_y_ - zone_.predicted(now).bodyToZone({0, holdCenter()}).y : 0.f;
            const bool done = vis ? back_done >= tgt : travel_ >= tgt * drRatio(now);
            if (done || elapsed > t_.retreat_budget_us) { prepush_back_ = false; cargo_ref_us_ = 0; align_pending_ = t_.assume_align_push; align_start_us_ = 0; push_hold_valid_ = false; enter(PushState::ENTER, now, "assumed_release_then_push"); break; }
            reason_ = vis ? "assumed_release_back_off" : "assumed_release_back_off_blind";
            drive(-t_.back_speed, 0); break;
        }
        if (carryLogic() && (assume_blind_ || !observed_zone)) {
            // TEMP_ASSUMPTION: bounded straight reverse leaves the released load behind.
            assume_blind_ = true;
            if (travel_ >= t_.assume_back_m) { enter(PushState::VERIFY_DELIVERY, now, "assumed_backed_out"); break; }
            if (elapsed > t_.retreat_budget_us) { fail("retreat_blocked"); break; }
            reason_ = "assumed_blind_back_out"; drive(-t_.back_speed, 0); break;
        }
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
        const bool vfix = in.vref_valid && in.vref_zone.valid && in.vref_points >= 3;
        if (carryLogic() && vfix && !verify_fix_us_) verify_fix_us_ = now;
        if (carryLogic() && !verify_fix_us_ && elapsed > t_.assume_verify_far_after_us) verify_far_view_ = true;
        // Dead reckoning alone cannot confirm delivery: without a visual zone fix keep looking (longer budget, then far view).
        const bool expired = carryLogic()
            ? (verify_fix_us_ ? now - verify_fix_us_ > t_.delivery_budget_us : elapsed > std::max(t_.assume_verify_fix_wait_us, t_.delivery_budget_us))
            : elapsed > t_.delivery_budget_us;
        // TEMP_ASSUMPTION: visual judgement of the released block against the zone (needs the zone references in view).
        bool visual_ok = false;
        if (carryLogic() && !stable) {
            cv::Point2f cb, cr; bool cfix = false; int ci = -1;
            if (cargoZone(in, now, cb, cr, cfix, &ci) && in.cargo_ts_us != verify_ts_) {
                verify_ts_ = in.cargo_ts_us;
                const auto img = imgAt(in, ci);
                if (cfix) {
                    if (!img.clipped && cr.y >= t_.assume_cargo_in_y_m && std::abs(cr.x - halfX()) <= t_.assume_cargo_x_tol_m) { ++verify_in_; verify_out_ = 0; }
                    else { ++verify_out_; verify_in_ = 0; }
                } else if (img.front_valid && img.height_px > 1.f) {
                    // No zone fix: compare the box bottom with the zone's inner front edge in the image (conservative, uncalibrated).
                    const float frac = (img.front_px - img.bottom_px) / img.height_px;
                    if (!img.clipped && frac >= t_.assume_img_inside_frac) { ++verify_in_; verify_out_ = 0; }
                    else if (img.clipped || frac < t_.assume_img_outside_frac) { ++verify_out_; verify_in_ = 0; }
                    char vb[160];
                    std::snprintf(vb, sizeof vb, "[CARGO_ZONE] event=verify_img frac=%.2f clipped=%d in=%d out=%d\n", frac, int(img.clipped), verify_in_, verify_out_);
                    dropzone_evt_ += vb;
                }
                noteCargo("verify", cr, cfix);
            }
            visual_ok = verify_in_ >= (cargo_fix_ ? 3 : t_.assume_img_verify_obs);
            if (!visual_ok && verify_out_ >= 3 && repush_n_ < t_.assume_repush_max) {
                ++repush_n_; verify_in_ = verify_out_ = 0; cargo_ref_us_ = 0; verify_far_view_ = false; verify_fix_us_ = 0;
                noteCargo("verify_outside_repush", cr, cfix);
                assume_pushed_ = false; align_pending_ = t_.assume_align_push; align_start_us_ = 0; push_hold_valid_ = false; enter(PushState::ENTER, now, "assumed_verify_repush"); break;
            }
        }
        if (!stable && !expired && !visual_ok) break;
        // TEMP_ASSUMPTION: an unconfirmed delivery is credited as the whole trip.
        const bool assumed = !stable && t_.assume_all_safe && expected > 0;
        const int credited = stable ? delivery_delta_ : assumed ? expected : 0;
        total_ += credited;
        if(credited>0){attempt_started_=now;bounded_approach_=bounded_retreat_=0;}
        if(credited==expected && credited>0){near_failures_=0;}
        if (credited > 0 && credited == expected && trip_.ordinary == trip_.total()) first_ = true;
        verify_far_view_ = false; verify_fix_us_ = 0;
        delivery_visual_ = stable || visual_ok;
        reason_ = assumed ? (visual_ok ? "delivered_visual" : "delivered_unverified") : credited == expected ? "delivered" : credited ? "partial_delivery" : "delivery_unverified";
        if(t_.demo_carry_once && t_.assume_all_safe) enter(PushState::TURN_SCAN, now, ""); // show the next-trip turn
        else if(t_.demo_carry_once){
            fail(credited==expected && credited>0?"demo_delivery_complete":"demo_delivery_unconfirmed");
            state_=PushState::SAFE_STOP;
        } else enter(PushState::TURN_SCAN, now, "");
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
            if (t_.demo_carry_once) { fail(delivery_visual_ ? "demo_next_trip_ready" : "demo_next_trip_unverified"); state_ = PushState::SAFE_STOP; break; }
            clearTrip(); enter(PushState::SCAN, now, ""); break;
        }
        if (elapsed > t_.turn_budget_us) { fail("turn_heading_unverified"); break; }
        drive(0, t_.turn_wz);
        break;
    }
    case PushState::CAPTURE_FAIL: case PushState::LOST_HOLD: case PushState::ABORT_DROP: {
        // Release where the opponent zone is not involved, then reverse off the objects.
        if (step_ == 0 && corridor_retreat_) { commandFrame(1, now); step_ = 1; phase_us_ = now; } // nothing held: frame stays up
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
            const float limit = corridor_retreat_ ? std::min(t_.corridor_back_m * motionRatio(), .85f * t_.back_speed * float(t_.retreat_budget_us) / 1e6f)
                                                  : std::min(t_.abort_back_m, approach_forward_);
            if (corridor_retreat_ && !in.retreat_safe && travel_ < limit) {
                clearTrip(); enter(PushState::SCAN, now, "corridor_back_off_unsafe"); break;
            }
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
        if (measured && back >= (corridor_retreat_ ? t_.corridor_back_m : t_.abort_back_m)) {
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
