#include "rescue/push_task.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>

namespace rescue {
const char *PushTask::name(PushState s) {
    switch (s) {
    case PushState::WAIT_START: return "WAIT_START";
    case PushState::START_ADVANCE: return "START_ADVANCE";
    case PushState::SCAN: return "SCAN";
    case PushState::SCATTER: return "SCATTER";
    case PushState::PILE: return "PILE";
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
    state_ = s; phase_us_ = now; scan_last_valid_ = false; travel_ = 0; travel_limit_ = 0; sweep_phase_ = 0; rush_pulse_phase_ = 0; rush_pulse_count_ = 0;
    confirmations_ = 0; misses_ = 0; hold_misses_ = 0; step_ = 0; turned_ = 0; heading_seen_ = false;
    track_close_ = false; last_range_ = std::numeric_limits<float>::infinity();
    retreat_origin_valid_ = false; retreat_mode_ = 0; assume_blind_ = false;
    unstick_step_ = 0; prelower_us_ = 0;
    enemy_evade_phase_ = 0; enemy_seen_frames_ = 0; enemy_evade_cool_until_us_ = 0;
    if (s != PushState::SCAN) { scatter_idle_since_us_ = 0; scatter_seen_ = 0; confirm_since_us_ = 0; vis_hits_ = vis_miss_ = 0; vis_chase_t0_ = 0; scan_hold_t0_ = scan_hold_cool_until_us_ = 0; }
    if (s != PushState::RUSH) rush_yaw_ref_set_ = false; // PREPARE (tracked) -> RUSH keeps the last seen target bearing
    if (s == PushState::START_ADVANCE) delivery_seen_ = false;
    if (s == PushState::APPROACH) precheck_seen_ = 0;
    if (s == PushState::RUSH) { rush_stall_phase_ = false; rush_stall_id_ = -1; rush_stall_samples_ = 0; rush_stall_last_us_ = 0; }
    if (s == PushState::VERIFY_CAPTURE) { vc_obs_frames_ = vc_label_frames_ = vc_bad_frames_ = 0; vc_bad_verdict_ = RuleVerdict::OK; vc_re_active_ = false; vc_re_track_ = -1; vc_re_frames_ = vc_re_strong_ = vc_re_clear_ = 0; }
    // 有20°中间角时，APPROACH沿用当前相机角（远处不提前下压），由接近距离分级切换；否则保持直接用TRACK角。
    if (s == PushState::APPROACH) approach_pitch_ = midPitch() ? pitch_cmd_ : t_.track_pitch_cdeg;
    if (s == PushState::SCATTER) sweep_pitch_ = pitch_cmd_;
    else { swing_plan_pending_ = swing_plan_active_ = false; swing_cap_pos_ = swing_cap_neg_ = 99.f; }
    disperse_turn_phase_ = 0;
    approach_rebind_ = false;
    approach_last_id_ = -1; approach_last_world_valid_ = false;
    if (s != PushState::LOST_HOLD) corridor_retreat_ = false;
    if (why && *why) reason_ = why;
}
bool PushTask::uprightInCorridor(const PushObservation &in, uint64_t now, float &lateral, float *along_out) const {
    if (!in.upright_ts_us || now < in.upright_ts_us || now - in.upright_ts_us > t_.sweep_fresh_us) return false;
    const float d = in.distance_m;
    if (!std::isfinite(d) || d < .05f || !std::isfinite(in.heading_error)) return false;
    const cv::Point2f t(-d * std::sin(in.heading_error), d * std::cos(in.heading_error)), a(t.x / d, t.y / d);
    bool hit = false; float best = 0;
    for (const auto &u : in.upright_body) {
        if (!std::isfinite(u.x) || !std::isfinite(u.y) || cv::norm(u - t) < t_.sweep_min_sep_m) continue; // the target itself is not an obstacle
        const float along = u.x * a.x + u.y * a.y, side = u.x * a.y - u.y * a.x; // side > 0: right of the approach line
        if (along < 0 || along > d + t_.sweep_extra_m || std::abs(side) > t_.sweep_corridor_half_m) continue;
        if (along_out) *along_out = hit ? std::min(*along_out, u.y) : u.y;
        if (!hit || std::abs(side) < best) { best = std::abs(side); lateral = side; hit = true; }
    }
    return hit;
}
bool PushTask::neighbourInCorridor(const PushObservation &in, uint64_t now, float &lateral, float *along_out) const {
    if (!t_.clear_neighbours || !in.clutter_ts_us || now < in.clutter_ts_us || now - in.clutter_ts_us > t_.sweep_fresh_us) return false;
    switch (checkCorridor(in.corridor, in.corridor_complete, in.corridor_occlusion_free, label_, first_)) {
    case RuleVerdict::TOO_MANY_SUPPLIES: case RuleVerdict::DANGEROUS: case RuleVerdict::UNKNOWN_OBJECT: case RuleVerdict::CORE_BEFORE_FIRST:
    case RuleVerdict::INJURED_BEFORE_FIRST: case RuleVerdict::INJURED_NOT_ALONE: case RuleVerdict::OCCLUDED: break;
    default: return false;
    }
    const float d = in.distance_m;
    if (!std::isfinite(d) || d < .05f || !std::isfinite(in.heading_error)) return false;
    const cv::Point2f t(-d * std::sin(in.heading_error), d * std::cos(in.heading_error)), a(t.x / d, t.y / d);
    bool hit = false; float best = 0;
    for (const auto &u : in.clutter_body) {
        if (!std::isfinite(u.x) || !std::isfinite(u.y) || cv::norm(u - t) < t_.sweep_min_sep_m) continue;
        const float along = u.x * a.x + u.y * a.y, side = u.x * a.y - u.y * a.x;
        if (along < 0 || along > d - t_.clear_front_gap_m || std::abs(side) > t_.clear_half_m) continue;
        if (along_out) *along_out = hit ? std::min(*along_out, u.y) : u.y;
        if (!hit || std::abs(side) < best) { best = std::abs(side); lateral = side; hit = true; }
    }
    return hit;
}
void PushTask::clearTrip() {
    vref_used_ = false;carry_progress_us_=0;sweep_non_supply_active_=false;
    cue_mode_=false;clear_commanded_us_=clear_planned_us_=0;
    search_cue_id_=capture_target_id_=-1;
    drop_locked_=false;drop_centre_zone_={};assume_pushed_=false;enter_unconfirmed_=false;prepush_back_=false;align_pending_=push_hold_valid_=false;align_start_us_=0;repush_n_=verify_in_=verify_out_=0;verify_far_view_=false;verify_fix_us_=0;inside_obs_=0;inside_obs_ts_=0;probe_until_us_=probe_next_us_=0;probe_sign_=1.f;sweep_phase_=sweep_attempts_=sweep_seen_=0;delivery_visual_=false;cargo_clipped_=false;cargo_ref_us_=cargo_seen_us_=0;half_locked_=false;half_injured_=false;half_x_=0;
    delivery_seen_=false; zone_search_started_us_=0;
    approach_forward_=0; last_target_valid_=false;
    target_id_ = -1; label_.clear(); trip_ = {}; pending_ = {}; seen_ = {};
    verdict_ = RuleVerdict::OK;
}
void PushTask::sweepChainBegin(uint64_t now) {
    sweepChainSettle(now);
    if (!sweep_chain_t0_) { sweep_chain_t0_ = now; sweep_chain_acc_ = 0; sweep_chain_swung_ = false; }
    if (!sweep_chain_active_) sweep_chain_seg_ = now;
    sweep_chain_active_ = true;
}
void PushTask::sweepChainPause(uint64_t now) {
    if (!sweep_chain_t0_ || !sweep_chain_active_) return;
    sweep_chain_acc_ += now > sweep_chain_seg_ ? now - sweep_chain_seg_ : 0;
    sweep_chain_active_ = false; sweep_chain_end_ = now;
}
bool PushTask::sweepChainClose(uint64_t now) {
    if (!sweep_chain_t0_) return false;
    sweepChainPause(now);
    const bool swung = sweep_chain_swung_;
    sweep_spent_us_ += sweep_chain_acc_; sweep_match_spent_us_ += sweep_chain_acc_;
    if (swung) ++sweep_done_;
    sweep_chain_t0_ = sweep_chain_acc_ = 0; sweep_chain_active_ = sweep_chain_swung_ = false;
    if (t_.sweep_observation_guard) sweep_observe_until_us_ = now + t_.sweep_reobserve_us;
    return swung;
}
void PushTask::sweepChainSettle(uint64_t now) { // a paused chain (far drive done) that was not continued by a near sweep in time is a finished sweep
    if (sweep_chain_t0_ && !sweep_chain_active_ && now > sweep_chain_end_ + t_.sweep_chain_link_us) sweepChainClose(now);
}
uint64_t PushTask::sweepSpentNow(uint64_t now) const {
    uint64_t v = sweep_spent_us_;
    if (sweep_chain_t0_) v += sweep_chain_acc_ + (sweep_chain_active_ && now > sweep_chain_seg_ ? now - sweep_chain_seg_ : 0);
    return v;
}
uint64_t PushTask::sweepMatchSpentNow(uint64_t now) const {
    uint64_t v = sweep_match_spent_us_;
    if (sweep_chain_t0_) v += sweep_chain_acc_ + (sweep_chain_active_ && now > sweep_chain_seg_ ? now - sweep_chain_seg_ : 0);
    return v;
}
void PushTask::blacklist(uint64_t now) {
    if (target_id_ < 0) return;
    rejected_[target_id_] = now + t_.blacklist_us;
    // Track ids churn across pitch changes: remember where the vetoed block sits in the world as well.
    bool have = false; float yaw = 0.f;
    if (rush_yaw_ref_set_) { yaw = rush_yaw_ref_; have = true; }
    else if (approach_last_world_valid_) { yaw = approach_last_world_yaw_; have = true; }
    if (have && std::isfinite(yaw)) {
        if (rejected_bearings_.size() >= 8) rejected_bearings_.erase(rejected_bearings_.begin());
        rejected_bearings_.push_back({yaw, last_target_valid_ ? std::hypot(last_target_xy_.x, last_target_xy_.y) : std::max(approach_last_distance_, 0.f), now + t_.blacklist_us});
    }
}
// Unrecoverable without an operator: stop, keep the gripper as commanded.
void PushTask::fail(const char *why) { fault_ = true; reason_ = why; }
void PushTask::commandFrame(uint8_t open, uint64_t now) {
    if (frame_raised_ != open) { frame_raised_ = open; gripper_us_ = now; ++frame_transaction_; }
}
bool PushTask::sweepFramePhase() const {
    if (state_ == PushState::SCATTER) return sweep_phase_ >= 4 && sweep_phase_ <= 6;
    if (state_ == PushState::MID_APPROACH) return sweep_phase_ >= 4;
    return state_ == PushState::APPROACH && sweep_phase_ != 0;
}
static bool searchLikeState(PushState st) {
    switch (st) {
    case PushState::SCAN: case PushState::SCATTER: case PushState::PILE: case PushState::MID_REACQUIRE: case PushState::MID_APPROACH:
    case PushState::NEAR_REACQUIRE: case PushState::SELECT_CARGO: case PushState::CUE_APPROACH: case PushState::APPROACH:
    case PushState::TURN_SCAN: case PushState::LOST_SEARCH: return true;
    default: return false;
    }
}
bool PushTask::defaultOpenState() const { return t_.frame_open_default && trip_.total() == 0 && searchLikeState(state_); }
// Wire-level open overlay: the logical frame (cargo logic) stays as the state machine set it.
void PushTask::syncFrameOverlay(uint64_t now) {
    if (state_ == PushState::SAFE_STOP || fault_) return; // a stop holds the frame as commanded
    const bool want = !sweep_open_disabled_ && frame_raised_ == 0 && ((t_.sweep_frame_open && sweepFramePhase()) || defaultOpenState());
    if (want && !sweep_open_) { sweep_open_ = true; sweep_open_done_ = false; sweep_open_us_ = now; }
    else if (!want && sweep_open_) {
        sweep_open_ = false;
        // Only states that run on a closed frame need the wire confirmed closed; states that open or lower the frame themselves must not be held here.
        const bool needs_closed = searchLikeState(state_) || state_ == PushState::LOST_SEARCH || state_ == PushState::CUE_PREPARE ||
                                  state_ == PushState::CLEAR_PILE || state_ == PushState::CLEAR_PAUSE;
        if (frame_raised_ == 0 && needs_closed) { sweep_close_wait_ = true; sweep_close_us_ = now; }
    }
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
bool PushTask::weakCandidate(const PushObservation &in) const {
    return t_.weak_target_scatter && in.pixel_mode != "none" && std::isfinite(in.pixel_ratio_smoothed) &&
        in.pixel_ratio_smoothed > 0 && in.pixel_ratio_smoothed < t_.weak_pixel_ratio;
}
float PushTask::approachHeadingShift(const PushObservation &in) const {
    if (approach_last_world_valid_ && in.heading_valid && std::isfinite(in.heading_rad))
        return wrapAngle(in.heading_rad + in.heading_error - approach_last_world_yaw_);
    return wrapAngle(in.heading_error - approach_last_heading_);
}
bool PushTask::sameObject(const PushObservation &in, uint64_t now) const {
    if (!last_target_valid_ || !in.target_valid || in.target_id < 0 || in.target_id == target_id_ || in.label != label_ ||
        in.target_is_search_cue || in.target_in_zone) return false;
    if (!std::isfinite(in.distance_m) || in.distance_m <= 0 || !std::isfinite(in.heading_error) || std::abs(in.heading_error) > 1.2f) return false;
    const auto it = rejected_.find(in.target_id);
    if (it != rejected_.end() && now < it->second) return false;
    const cv::Point2f p(-in.distance_m * std::sin(in.heading_error), in.distance_m * std::cos(in.heading_error));
    return cv::norm(p - last_target_xy_) <= t_.rebind_max_shift_m;
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
    // The in-place turn of a sweep is steered by what the 20deg stage sees (SCATTER and APPROACH sweeps alike).
    if (disperse_turn_phase_ && sweep_phase_ == 5 && (state_ == PushState::SCATTER || state_ == PushState::APPROACH || state_ == PushState::PILE) && midPitch())
        return disperse_turn_phase_ == 4 ? t_.track_pitch_cdeg : t_.intermediate_pitch_cdeg;
    switch (state_) {
    case PushState::MID_REACQUIRE: case PushState::MID_APPROACH: return t_.intermediate_pitch_cdeg;
    case PushState::CUE_APPROACH: return track_close_ ? t_.track_pitch_cdeg : t_.far_pitch_cdeg;
    case PushState::CUE_PREPARE: case PushState::CLEAR_PILE: case PushState::CLEAR_PAUSE:
        return t_.intermediate_pitch_cdeg!=kCameraPitchInvalid?t_.intermediate_pitch_cdeg:t_.track_pitch_cdeg;
    case PushState::NEAR_REACQUIRE: case PushState::SELECT_CARGO: return t_.track_pitch_cdeg;
    case PushState::LOST_SEARCH: return lost_round_ && midPitch() ? t_.intermediate_pitch_cdeg : t_.far_pitch_cdeg;
    case PushState::VERIFY_DELIVERY: return assumeView() && !verify_far_view_ ? t_.assume_view_pitch_cdeg : t_.far_pitch_cdeg;
    case PushState::SCAN:
        return t_.post_clear_mid_view && midPitch() && scan_turn_acc_ < scan_mid_limit_ ? t_.intermediate_pitch_cdeg : t_.far_pitch_cdeg;
    case PushState::SCATTER:
        if (t_.sweep_far_judge && sweep_phase_ >= 4 && sweep_phase_ <= 6) return t_.far_pitch_cdeg;
        return t_.sweep_view_handoff && midPitch() && sweep_pitch_ != kCameraPitchInvalid ? sweep_pitch_ : t_.far_pitch_cdeg;
    case PushState::TURN_SCAN: return t_.far_pitch_cdeg;
    case PushState::PILE: // the close cluster is looked at from the mid (or tracking) view throughout
        if (pile_low_) return t_.near_pitch_cdeg; // capture look: lowest preset
        return midPitch() ? t_.intermediate_pitch_cdeg : t_.track_pitch_cdeg;
    case PushState::APPROACH: return approach_pitch_!=kCameraPitchInvalid?approach_pitch_:t_.track_pitch_cdeg;
    // TEMP_ASSUMPTION：开框由MCU完成标志确认，不需要为此下压到40°；货物贴近时在40°画面顶边会被截断（box_at_image_edge），
    // 所以PREPARE/RUSH沿用接近阶段已经到位的角度，不再单独切NEAR/TRACK。
    case PushState::PREPARE: case PushState::RUSH:
        if (state_ == PushState::RUSH && rush_stall_phase_) return t_.near_pitch_cdeg;
        if ((t_.assume_all_safe || t_.demo_capture_motion) && approach_pitch_ != kCameraPitchInvalid) return approach_pitch_;
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
// Any visible block already past the zone's front edge on our half (fresh visual zone fix only).
bool PushTask::cargoInZone(const PushObservation &in, uint64_t now) const {
    if (in.cargo_body.empty() || !(now >= in.cargo_ts_us && now - in.cargo_ts_us <= 300000)) return false;
    if (!(in.vref_valid && in.vref_zone.valid && in.vref_points >= 3 && in.vref_zone.timestamp_us == in.cargo_ts_us)) return false;
    for (size_t i = 0; i < in.cargo_body.size(); ++i) {
        const cv::Point2f q = in.vref_zone.bodyToZone(in.cargo_body[i]);
        if (in.cargo_body[i].y > 0 && !imgAt(in, int(i)).clipped && q.y >= t_.assume_cargo_verify_in_y_m && std::abs(q.x - halfX()) <= t_.assume_cargo_x_tol_m) return true;
    }
    return false;
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
    case PushState::CLEAR_PILE: case PushState::CLEAR_PAUSE: case PushState::SCATTER: case PushState::PILE: return PushState::SCAN;
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
        syncFrameOverlay(now);
        motion.gripper_offset = frameAngle((frame_raised_ || sweep_open_) ? FrameAction::Open : FrameAction::Close);
        // The MCU never acknowledges an action that repeats the state it is already in, so a logical change hidden by the open overlay must not look like a new action.
        const int wire = (frame_raised_ || sweep_open_) ? 1 : 0;
        if (wire_last_ < 0) { wire_last_ = wire; wire_txn_ = frame_transaction_; }
        else if (wire != wire_last_) { wire_last_ = wire; ++wire_txn_; }
        motion.frame_transaction = wire_txn_; // stops hold the gripper, never toggle it
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
        clearTrip(); rejected_.clear(); rejected_bearings_.clear(); disperse_cool_until_us_ = 0; scatter_cool_until_us_ = 0; sweep_gap_until_us_ = 0; sweep_spent_us_ = 0; sweep_done_ = 0; sweep_match_spent_us_ = 0; sweep_chain_t0_ = sweep_chain_acc_ = 0; sweep_chain_active_ = sweep_chain_swung_ = false; confirm_since_us_ = 0; sweep_observe_until_us_ = 0; zone_mem_ = zone_enemy_ = cs_active_ = run_yaw0_set_ = false; scatter_count_ = 0; scatter_approach_n_ = 0; scatter_far_ = false; burst_drive_used_ = burst_fast_ = burst_swing_armed_ = burst_swing_ = false; scan_turn_acc_ = 0; scan_mid_limit_ = 0; post_disperse_mid_ = false; disperse_small_ = false; mid_clear_n_ = 0; precheck_seen_ = 0; precheck_rejects_ = 0; total_ = 0; first_ = t_.assume_injured_trip; fault_ = false;clear_attempts_=0;near_failures_=0;lost_round_=0;startup_commanded_us_=0;startup_yaw0_set_=false;
        odo_ = ratio_est_ = 0; vref_used_ = false; zone_.reset(); ratio_snap_ = imu_seen_ = false; vref_fix_us_ = vref_seen_us_ = 0;
        frame_raised_ = 0; gripper_us_ = now; stopped_from_ = PushState::WAIT_START;
        sweep_open_ = sweep_open_done_ = sweep_open_disabled_ = sweep_close_wait_ = false; sweep_open_us_ = sweep_close_us_ = 0;
        pitch_cmd_ = t_.far_pitch_cdeg; pitch_wait_us_ = now;
        enter(PushState::WAIT_START, now, "reset"); last_us_ = now;
        return result();
    }
    if (state_ != PushState::CARRY) cs_active_ = false;
    if (!run_yaw0_set_ && in.heading_valid && std::isfinite(in.heading_rad)) { run_yaw0_ = in.heading_rad; run_yaw0_set_ = true; }
    if (in.zone_sight_valid && in.heading_valid && std::isfinite(in.heading_rad) && std::isfinite(in.zone_sight_bearing)) {
        const float yaw = wrapAngle(in.heading_rad + in.zone_sight_bearing); // world yaw of the sighted zone (yaw grows on left turns)
        if (in.zone_sight_enemy) {
            zone_enemy_ = true; zone_enemy_yaw_ = yaw;
            if (zone_mem_ && std::abs(wrapAngle(yaw - zone_mem_yaw_)) < .5f) zone_mem_ = false;
        } else if (!(zone_enemy_ && std::abs(wrapAngle(yaw - zone_enemy_yaw_)) < .5f)) {
            const float d = wrapAngle(yaw - zone_mem_yaw_);
            zone_mem_yaw_ = (zone_mem_ && now - zone_mem_us_ < 3000000 && std::abs(d) < .6f) ? wrapAngle(zone_mem_yaw_ + .5f * d) : yaw;
            zone_mem_ = true; zone_mem_us_ = now;
        }
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
    rejected_bearings_.erase(std::remove_if(rejected_bearings_.begin(), rejected_bearings_.end(),
        [now](const RejectedBearing &b) { return now >= b.until_us; }), rejected_bearings_.end());

    const bool forward_ok = in.path_safe && in.opponent_zone_clear;
    const bool scan_ok = in.opponent_zone_clear && (in.directional_clearance_valid?in.turn_safe:in.path_safe);
    const bool observed_zone=zoneOk(in) && in.zone_estimate.source!=ZoneEstimate::Source::PREDICTED;
    const uint64_t elapsed = now - phase_us_;
    // distance_m is radial; gripper planes are longitudinal robot-frame y.
    const float target_forward_m = in.distance_m * std::cos(in.heading_error);
    if (tracking(in)) { last_target_xy_ = {-in.distance_m * std::sin(in.heading_error), target_forward_m}; last_target_valid_ = true; }
    if (tracking(in) && (state_ == PushState::PREPARE || state_ == PushState::RUSH) && in.heading_valid && std::isfinite(in.heading_rad) && std::isfinite(in.heading_error))
        { rush_yaw_ref_ = wrapAngle(in.heading_rad + in.heading_error); rush_yaw_ref_set_ = true; }
    bool scan_spin = false;
    const auto drive = [&](float vx, float wz) {
        const bool turn_blocked=in.directional_clearance_valid && wz!=0 &&
            !(vx==0?in.turn_safe:(vx>0 && in.arc_safe));
        if (turn_blocked) {
            // A blocked arc keeps going straight when the straight corridor is verified clear.
            if (!(vx>0 && forward_ok)) {reason_="turn_sweep_blocked";return;}
            wz=0;reason_="turn_sweep_blocked_straight";
        }
        if ((vx > 0 && !forward_ok) || (vx < 0 && !in.retreat_safe) || (vx == 0 && wz != 0 && !scan_ok))
        {
            if (state_ != PushState::SCAN && state_ != PushState::START_ADVANCE)
                reason_ = vx < 0 ? "retreat_clearance_missing" :
                !in.path_safe ? "forward_path_blocked" : "opponent_clearance_missing";
            return; // missing clearance stops the robot; it never substitutes another motion
        }
        const float vx_cap = state_ == PushState::START_ADVANCE ? std::max(t_.max_speed, t_.startup_advance_speed) : t_.max_speed; // only the start-up run may exceed the normal cap
        motion.vx_mps = std::clamp(vx, -vx_cap, vx_cap);
        // Preserve zero and direction; never let the floor exceed the configured cap.
        const bool demo_capture_turn = t_.demo_capture_motion && (state_ == PushState::APPROACH || state_ == PushState::RUSH);
        // 0.25 rad/s does not move the chassis (field-measured): pure in-place turns in demo APPROACH/RUSH use the effective floor, and escalate to max_wz if they persist.
        const bool demo_inplace = demo_capture_turn && vx == 0 && wz != 0;
        float demo_cap = .25f; bool escalated = false;
        if (demo_inplace) {
            if (now - inplace_last_us_ > 300000) inplace_since_us_ = now;
            inplace_last_us_ = now;
            escalated = now - inplace_since_us_ >= 1000000;
            demo_cap = escalated ? t_.max_wz : std::max(.25f, t_.min_turn_wz);
        }
        const float wz_cap = demo_capture_turn ? std::min(t_.max_wz, demo_cap) : (scan_spin ? std::max(t_.max_wz, t_.scan_wz) : t_.max_wz);
        if(wz!=0) wz=std::copysign(std::max(std::abs(wz),escalated ? wz_cap : std::min(t_.min_turn_wz,wz_cap)),wz);
        motion.wz_rps = std::clamp(wz, -wz_cap, wz_cap);
        if (vx == 0 && wz != 0 && !demo_capture_turn && t_.inplace_turn_boost > 1.f &&
            (state_ == PushState::APPROACH || state_ == PushState::MID_APPROACH || state_ == PushState::CUE_APPROACH || state_ == PushState::SELECT_CARGO))
            motion.wz_rps *= t_.inplace_turn_boost;
    };
    const auto scanSpin = [&] { scan_spin = true; drive(0, -t_.scan_wz); scan_spin = false; }; // 搜索扫描改为顺时针(原逆时针): wz<0 即 CW
    // Searching our zone with a load: an enemy-colour zone in view means turn about and drive a long leg away. true: this tick's motion is owned.
    const auto evadeEnemy = [&]() -> bool {
        const bool heading_ok = in.heading_valid && std::isfinite(in.heading_rad);
        if (enemy_evade_phase_ == 0) {
            if (!t_.enemy_evade || now < enemy_evade_cool_until_us_ || !in.enemy_zone_seen) { enemy_seen_frames_ = 0; return false; }
            if (++enemy_seen_frames_ < t_.enemy_evade_frames || !heading_ok) return false;
            enemy_evade_phase_ = 1; enemy_evade_turned_ = 0; enemy_evade_last_yaw_ = in.heading_rad; enemy_evade_start_us_ = now;
            ++enemy_evade_count_; enemy_seen_frames_ = 0;
        }
        if (enemy_evade_phase_ == 1) {
            if (heading_ok) { enemy_evade_turned_ += std::abs(wrapAngle(in.heading_rad - enemy_evade_last_yaw_)); enemy_evade_last_yaw_ = in.heading_rad; }
            if (!heading_ok || now - enemy_evade_start_us_ > t_.enemy_evade_turn_budget_us) {
                enemy_evade_phase_ = 0; enemy_evade_cool_until_us_ = now + t_.enemy_evade_cool_us; zone_search_started_us_ = now; return false;
            }
            if (enemy_evade_turned_ < t_.enemy_evade_turn_rad) { reason_ = "enemy_zone_turn_away"; scanSpin(); return true; }
            enemy_evade_phase_ = 2; enemy_evade_yaw_ = in.heading_rad; enemy_evade_start_us_ = now; travel_ = 0;
        }
        if (travel_ >= t_.enemy_evade_m * motionRatio() || now - enemy_evade_start_us_ > t_.enemy_evade_drive_budget_us) {
            enemy_evade_phase_ = 0; enemy_evade_cool_until_us_ = now + t_.enemy_evade_cool_us; zone_search_started_us_ = now; return false;
        }
        const float err = heading_ok ? wrapAngle(enemy_evade_yaw_ - in.heading_rad) : 0.f;
        reason_ = "enemy_zone_drive_away";
        drive(t_.enemy_evade_speed, std::clamp(t_.heading_gain * err, -t_.max_wz, t_.max_wz));
        return true;
    };
    const auto unstick = [&]() -> bool { // true: the wiggle owns this tick's motion
        if (!t_.spin_unstick) return false;
        if (!unstick_step_) {
            if (in.imu_spin_stall_us < t_.spin_stall_trigger_us) return false;
            unstick_step_ = 1; unstick_us_ = now;
        }
        const uint64_t d = now - unstick_us_;
        if (d >= 2 * t_.spin_unstick_leg_us) { unstick_step_ = 0; return false; }
        const bool back = d < t_.spin_unstick_leg_us;
        reason_ = back ? "spin_stall_back" : "spin_stall_forward";
        drive(back ? -t_.spin_unstick_speed : t_.spin_unstick_speed, 0);
        return true;
    };
    const auto yawHoldWz = [&]() -> float { // IMU hold on the last seen target bearing; 0 when unavailable or inside the deadband
        if (!t_.rush_yaw_hold || !rush_yaw_ref_set_ || !in.heading_valid || !std::isfinite(in.heading_rad)) return 0.f;
        const float err = wrapAngle(rush_yaw_ref_ - in.heading_rad);
        return std::abs(err) > t_.startup_hold_deadband_rad ? std::clamp(t_.heading_gain * err, -t_.max_wz, t_.max_wz) : 0.f;
    };
    const auto preLower = [&](const char *why) -> bool { // true: lowering may go ahead
        if (!t_.rush_yaw_hold || !rush_yaw_ref_set_ || !in.heading_valid || !std::isfinite(in.heading_rad)) { enter(PushState::LOWER_FRAME, now, why); return true; }
        const float err = wrapAngle(rush_yaw_ref_ - in.heading_rad);
        if (std::abs(err) <= t_.rush_lower_yaw_tol_rad) { enter(PushState::LOWER_FRAME, now, why); return true; }
        if (!prelower_us_) prelower_us_ = now;
        if (now - prelower_us_ > t_.prelower_realign_us) { enter(PushState::CAPTURE_FAIL, now, "prelower_heading_drift"); return false; }
        reason_ = "prelower_realign";
        drive(0, std::clamp(t_.heading_gain * err, -t_.rush_realign_wz, t_.rush_realign_wz));
        return false;
    };
    // Steer to a body-frame bearing; turn in place when the error is large.
    const auto steer = [&](float speed, float heading) {
        if (std::abs(heading) > t_.rotate_in_place_rad) drive(0, t_.heading_gain * heading);
        else drive(speed, std::abs(heading) <= t_.heading_deadband_rad ? 0.f : t_.heading_gain * heading);
    };
    // Own zone not verified while carrying: spin, and when nothing (or only a far sighting) is found, drive toward it and spin again.
    const auto carrySearch = [&] {
        const bool heading_ok = in.heading_valid && std::isfinite(in.heading_rad);
        if (!cs_active_) { cs_active_ = true; cs_phase_ = 0; cs_turned_ = 0; cs_last_set_ = false; cs_t0_ = now; cs_goal_ = t_.carry_search_turns * 2.f * float(CV_PI); }
        const bool fresh = zone_mem_ && now - zone_mem_us_ <= t_.carry_sight_fresh_us;
        const bool known = zone_mem_ && now - zone_mem_us_ <= t_.carry_sight_mem_us;
        if (cs_phase_ == 0) {
            if (heading_ok) { if (cs_last_set_) cs_turned_ += std::abs(wrapAngle(in.heading_rad - cs_last_yaw_)); cs_last_yaw_ = in.heading_rad; cs_last_set_ = true; }
            const bool spun = cs_turned_ >= cs_goal_ || now - cs_t0_ > t_.carry_spin_cap_us;
            if (!fresh && !spun) { reason_ = "searching_own_zone"; scanSpin(); return; }
            if (!heading_ok || (!known && !run_yaw0_set_)) { reason_ = "searching_own_zone"; scanSpin(); return; }
            cs_phase_ = 1; cs_t0_ = cs_leg_t0_ = now;
        }
        if (!heading_ok) { cs_phase_ = 0; cs_t0_ = now; cs_turned_ = 0; cs_last_set_ = false; cs_goal_ = t_.carry_search_respin_turns * 2.f * float(CV_PI); return; }
        const float target = known ? zone_mem_yaw_ : wrapAngle(run_yaw0_ + float(CV_PI));
        const float err = wrapAngle(target - in.heading_rad);
        if (std::abs(err) > t_.rotate_in_place_rad) cs_leg_t0_ = now; // the leg clock runs only while driving
        if (now - cs_leg_t0_ >= t_.carry_leg_us || now - cs_t0_ > t_.carry_leg_turn_cap_us) {
            cs_phase_ = 0; cs_t0_ = now; cs_turned_ = 0; cs_last_set_ = false; cs_goal_ = t_.carry_search_respin_turns * 2.f * float(CV_PI);
            reason_ = "searching_own_zone"; scanSpin(); return;
        }
        reason_ = fresh ? "carry_drive_to_sighted_zone" : known ? "carry_drive_to_remembered_zone" : "carry_drive_start_line";
        steer(t_.carry_speed, err);
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
    // Sweep ledger: one gap / budget / escalation for all group-sweep triggers.
    if (sweep_chain_active_ && state_ != PushState::SCATTER && (sweep_phase_ == 0 || (state_ != PushState::APPROACH && state_ != PushState::MID_APPROACH))) sweepChainClose(now); // the sweep was left by some other route: still account it once
    sweepChainSettle(now);
    const bool sweepTimeOk = sweepSpentNow(now) < t_.sweep_time_budget_us && sweepMatchSpentNow(now) < t_.sweep_match_budget_us;
    const bool sweepBudgetOk = sweep_done_ < t_.sweep_total_max && sweepTimeOk;
    const bool sweepLedgerOk = now >= sweep_gap_until_us_ && sweepBudgetOk;
    // A running sweep is cut when the chain deadline or a time budget runs out.
    const bool sweepOver = sweep_chain_active_ && (now - sweep_chain_t0_ > t_.sweep_op_budget_us || !sweepTimeOk);
    const int sweep_esc = std::min(sweep_done_, t_.sweep_escalate_max);
    // Blind-scatter group search shared by SCAN, the far drive, and the lost-target paths.
    const bool scatter_ok = t_.blind_scatter && t_.disperse && scan_ok && forward_ok && in.heading_valid && std::isfinite(in.heading_rad) &&
        scatter_count_ < t_.blind_scatter_max && now >= scatter_cool_until_us_ && sweepLedgerOk;
    const bool scatter_multi_ok = t_.blind_scatter && t_.disperse && scan_ok && forward_ok && in.heading_valid && std::isfinite(in.heading_rad) &&
        scatter_count_ < t_.blind_scatter_max && now >= scatter_cool_until_us_ && (t_.dense_multi_ignore_gap ? sweepBudgetOk : sweepLedgerOk);
    const bool approach_ok = scatter_approach_n_ < t_.blind_scatter_approach_max;
    const float far_y = approach_ok ? std::max(t_.blind_scatter_max_y_m, t_.blind_scatter_far_max_y_m) : t_.blind_scatter_max_y_m;
    const bool clutter_fresh = in.clutter_ts_us && now >= in.clutter_ts_us && now - in.clutter_ts_us <= t_.sweep_fresh_us;
    int best_n = 0; cv::Point2f best_c(0.f, 0.f); float best_d = 0.f;
    std::vector<cv::Point2f> ord_extra; // lock / weak points appended to a cluster list: they count as ordinary supplies
    const auto isOrdinary = [&](const cv::Point2f &q) {
        for (const auto &o : in.clutter_ordinary) if (o.x == q.x && o.y == q.y) return true;
        for (const auto &o : ord_extra) if (o.x == q.x && o.y == q.y) return true;
        return false;
    };
    const auto findBest = [&](const std::vector<cv::Point2f> &pts, float max_y, float radius) {
        best_n = 0; best_c = cv::Point2f(0.f, 0.f); best_d = 0.f; float best_score = -1e9f;
        for (const auto &p : pts) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.y < t_.blind_scatter_min_y_m || p.y > max_y ||
                std::abs(p.x) > t_.blind_scatter_half_x_m || std::abs(std::atan2(-p.x, p.y)) > t_.blind_scatter_max_bearing_rad) continue;
            int n = 0, n_ord = 0; cv::Point2f sum(0.f, 0.f);
            for (const auto &q : pts) {
                if (!std::isfinite(q.x) || !std::isfinite(q.y) || cv::norm(q - p) > radius) continue;
                ++n; sum += q; if (isOrdinary(q)) ++n_ord;
            }
            if (t_.blind_scatter_need_ordinary && n_ord == 0) continue; // injured / dangerous only: nothing worth ploughing
            const cv::Point2f c = sum * (1.f / float(n));
            const float d = float(cv::norm(c));
            const float score = float(n) - 2.f * std::abs(std::atan2(-c.x, c.y)); // dense, but the one nearest straight ahead wins close calls
            if (score > best_score || (score == best_score && d < best_d)) { best_n = n; best_c = c; best_d = d; best_score = score; }
        }
    };
    // 0: none, 1: group inside the short window (sweep now), 2: group farther out (drive closer first)
    const auto evalGroup = [&](const std::vector<cv::Point2f> &pts, int need_n, int far_need, float short_radius = -1.f) {
        if (need_n > 1) need_n += sweep_esc;
        if (far_need > 1) far_need += sweep_esc;
        findBest(pts, t_.blind_scatter_max_y_m, short_radius > 0.f ? short_radius : t_.disperse_radius_m);
        if (best_n >= need_n) return 1;
        if (far_y > t_.blind_scatter_max_y_m) { // long range: positions are coarse, a wider radius still counts as one group
            findBest(pts, far_y, t_.blind_scatter_far_radius_m);
            if (best_n >= far_need && best_d > t_.blind_scatter_max_y_m) return 2; // a wide-radius group inside the short window is left to the normal trigger
        }
        best_n = 0; return 0;
    };
    // commit: the sweep ignores a locked ordinary supply (it sits inside the group, or the lock was just lost)
    const auto beginScatter = [&](bool far, bool commit, const char *why) {
        target_id_ = -1; confirmations_ = 0; cue_mode_ = false;
        scatter_yaw_ = wrapAngle(in.heading_rad + std::atan2(-best_c.x, best_c.y));
        scatter_range_ = best_d; scatter_far_ = far; scatter_commit_ = commit; sweepChainBegin(now);
        burst_fast_ = burst_swing_ = false;
        if (far) {
            scatter_approach_m_ = std::max(0.f, best_d - t_.blind_scatter_approach_stop_m);
            if (t_.first_burst && !commit && !burst_drive_used_ && scatter_approach_m_ >= t_.first_burst_min_drive_m) {
                burst_fast_ = burst_drive_used_ = burst_swing_armed_ = true;
            }
        } else {
            burst_swing_ = burst_swing_armed_; burst_swing_armed_ = false;
            disperse_sign_ = (scatter_count_ % 2 == 0) ? 1.f : -1.f;
            disperse_pass_ = 0; disperse_passes_ = t_.blind_scatter_passes; disperse_swings_ = t_.blind_scatter_swings;
            swing_plan_pending_ = t_.swing_avoid_outside; swing_plan_active_ = false; swing_cap_pos_ = swing_cap_neg_ = 99.f;
            ++scatter_count_;
        }
        enter(PushState::SCATTER, now, why);
    };
    const auto beginPile = [&](const char *why) {
        target_id_ = -1; confirmations_ = 0; cue_mode_ = false;
        pile_yaw0_ = wrapAngle(in.heading_rad + in.heading_error); // world bearing of the pile (the target sits in it)
        pile_phase_ = 0; pile_seen_ = 0; pile_push0_ = -1.f; ++pile_runs_; pile_chain_n_ = 0; pile_low_ = false; pile_enclosed_ = false; pile_look_n_ = 0;
        enter(PushState::PILE, now, why);
        pile_phase_us_ = now;
    };
    const auto beginPileFromSweep = [&](const char *why) { // a disperse sweep that really swung continues in PILE (layer loop) instead of the plain re-search
        if (!t_.pile_chain || !t_.pile_clear || disperse_small_ || frame_raised_ != 0 || trip_.total() != 0 || !in.heading_valid || !std::isfinite(in.heading_rad)) return false;
        target_id_ = -1; confirmations_ = 0; cue_mode_ = false;
        pile_yaw0_ = in.heading_rad; pile_phase_ = 7; pile_seen_ = 0; pile_push0_ = -1.f; pile_chain_n_ = 0; pile_low_ = false; pile_enclosed_ = false; pile_look_n_ = 0; sweep_phase_ = 0;
        enter(PushState::PILE, now, why);
        pile_phase_us_ = now;
        return true;
    };
    const auto lostTarget = [&] { return ++misses_ > t_.hold_grace_frames; };
    const auto searchLost = [&] {
        if(frame_raised_==0 && trip_.total()==0 && scatter_ok && clutter_fresh) { // the group is still in view from the approach camera: sweep it instead of spinning
            const int g = evalGroup(in.clutter_body, t_.blind_scatter_min, t_.blind_scatter_far_min);
            if (g) { beginScatter(g == 2, true, g == 2 ? "lost_group_far" : "lost_group_sweep"); return; }
        }
        if(frame_raised_!=0 || trip_.total()>0) {fail("lost_search_jaw_or_load_unsafe");state_=PushState::SAFE_STOP;return;}
        clearTrip();lost_round_=post_disperse_mid_&&midPitch()?1:0;post_disperse_mid_=false;enter(PushState::LOST_SEARCH,now,"lost_target_search");
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
        if (p == 0) {
            rush_stall_last_us_ = 0;
            if (t_.sweep_observation_guard && sweep_observe_until_us_) sweep_observe_until_us_ = now + t_.sweep_reobserve_us;
            reason_="camera_pitch_wait"; return result();
        } // moving-camera time cannot count as stationary re-observation
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
    // Sweeping with the frame open: wait (standing still) until the open is acknowledged. A frame that never opens ends the experiment
    // for the rest of the match and the sweep goes on closed, as before.
    const auto sweepFrameReady = [&]() -> bool {
        if ((!t_.sweep_frame_open && !t_.frame_open_default) || sweep_open_disabled_ || frame_raised_ != 0) return true;
        if (!sweep_open_) {
            if (!(t_.sweep_frame_open || defaultOpenState())) return true;
            sweep_start_us_ = now; reason_ = "sweep_frame_open_wait"; return false;
        }
        if (sweep_open_done_) return true;
        if (in.gripper_done && in.gripper_feedback_open == 1) {
            sweep_open_done_ = true;
            std::cout << "[SWEEP_FRAME] open confirmed after " << (now - sweep_open_us_) / 1000 << " ms state=" << name(state_) << "\n";
            return true;
        }
        if (now - sweep_open_us_ > t_.gripper_timeout_us) {
            sweep_open_disabled_ = true;
            std::cout << "[SWEEP_FRAME] open not confirmed within " << t_.gripper_timeout_us / 1000 << " ms: sweeps stay closed from now on\n";
        }
        sweep_start_us_ = now; reason_ = "sweep_frame_open_wait";
        return false;
    };
    // Swing sides stop short of blocks that sit outside the group but inside the frame's reach at the end of the rush (judged from the far view when sweep_far_judge is on).
    const auto planSwing = [&] {
        swing_plan_pending_ = false;
        if (!t_.swing_avoid_outside || state_ != PushState::SCATTER || !clutter_fresh) return;
        const float done = (travel_ - sweep_travel0_) / motionRatio();
        const cv::Point2f c(0.f, scatter_range_ - done), e(0.f, std::max(0.f, disperse_rush_m_ - done));
        std::vector<cv::Point2f> pts = in.clutter_body;
        if (candidate(in, now) && std::isfinite(in.distance_m) && std::isfinite(in.heading_error))
            pts.emplace_back(-in.distance_m * std::sin(in.heading_error), in.distance_m * std::cos(in.heading_error));
        float cap_pos = 99.f, cap_neg = 99.f; int n_out = 0;
        for (const auto &q : pts) {
            if (!std::isfinite(q.x) || !std::isfinite(q.y) || cv::norm(q - c) <= t_.disperse_radius_m) continue;
            const cv::Point2f r = q - e;
            if (cv::norm(r) > t_.swing_avoid_radius_m) continue;
            const float b = std::atan2(-r.x, r.y); // left (positive yaw) is positive
            ++n_out;
            if (b > 0.f) cap_pos = std::min(cap_pos, std::max(0.f, b - t_.swing_avoid_margin_rad));
            else cap_neg = std::min(cap_neg, std::max(0.f, -b - t_.swing_avoid_margin_rad));
        }
        swing_cap_pos_ = cap_pos; swing_cap_neg_ = cap_neg; swing_plan_active_ = true;
        if (cap_pos != cap_neg) disperse_sign_ = cap_pos > cap_neg ? 1.f : -1.f; // the first swing (and the extra one) goes to the freer side
        std::cout << "[SWING_PLAN] outside=" << n_out << " cap_left=" << cap_pos << " cap_right=" << cap_neg << " first=" << (disperse_sign_ > 0 ? "left" : "right") << "\n";
    };
    // One tick of a pile scatter (sweep_phase_ 4 = rush in, 5 = swing the low frame). true once the last pass is centred again.
    const auto disperseStep = [&]() -> bool {
        if (!sweepFrameReady()) return false;
        if (sweep_phase_ == 4) {
            if (swing_plan_pending_ && (!t_.sweep_far_judge || pitch_cmd_ == t_.far_pitch_cdeg)) planSwing();
            if (travel_ - sweep_travel0_ >= disperse_rush_m_ * motionRatio() || now - sweep_start_us_ > t_.disperse_rush_budget_us) {
                if (swing_plan_pending_) planSwing();
                sweep_phase_ = 5; sweep_start_us_ = now; sweep_yaw0_ = in.heading_rad; disperse_swing_n_ = 0; sweep_travel0_ = travel_; sweep_chain_swung_ = true;
                disperse_turn_phase_ = t_.disperse_single_turn && !disperse_small_ ? 1 : 0; disperse_look_ready_us_ = 0; disperse_clear_n_ = 0;
            } else {
                const bool opening_impact = t_.first_burst_full_impact && burst_swing_ && state_ == PushState::SCATTER;
                const float speed = opening_impact ? t_.first_burst_speed : t_.disperse_speed;
                reason_ = opening_impact ? "first_burst_impact" : "disperse_rush"; drive(speed, 0);
                if (motion.vx_mps > 0) motion.vx_mps = speed; // preserve every clearance veto before bypassing the ordinary speed cap
                return false;
            }
        }
        if (disperse_turn_phase_) {
            // Blocks in the camera view beyond the frame mouth, per side (bearing > 0 is left); false while the observation is stale.
            const auto viewCount = [&](int &left, int &right) {
                if (!(in.clutter_ts_us && now >= in.clutter_ts_us && now - in.clutter_ts_us <= t_.sweep_fresh_us)) return false;
                left = right = 0;
                for (const auto &q : in.clutter_body) {
                    if (!std::isfinite(q.x) || !std::isfinite(q.y) || q.y <= t_.sweep_frame_front_m + .03f || cv::norm(q) > t_.disperse_turn_view_range_m) continue;
                    const float b = std::atan2(-q.x, q.y);
                    if (std::abs(b) > t_.disperse_turn_view_half_rad) continue;
                    ++(b > 0.f ? left : right);
                }
                return true;
            };
            if (disperse_turn_phase_ == 1) { // stand still until the camera sits at the 20deg stage and a frame taken there has arrived
                if (desiredPitch() != pitch_cmd_ || !in.camera_pitch_stable) { reason_ = "disperse_look"; return false; }
                if (!disperse_look_ready_us_) disperse_look_ready_us_ = now;
                int l = 0, r = 0;
                const bool seen = in.clutter_ts_us >= disperse_look_ready_us_ && viewCount(l, r);
                if (!seen && now - disperse_look_ready_us_ < t_.disperse_turn_look_us) { reason_ = "disperse_look"; return false; }
                disperse_turn_dir_ = seen && l != r ? (l < r ? 1.f : -1.f) : disperse_sign_; // toward the emptier side; unseen or level: the planned side
                disperse_turn_phase_ = 2; sweep_yaw0_ = in.heading_rad; sweep_start_us_ = now; disperse_clear_n_ = 0;
            }
            if (disperse_turn_phase_ == 3) { // push straight on along the new heading
                if (travel_ - sweep_travel0_ >= t_.disperse_turn_push_m * motionRatio() || now - sweep_start_us_ > t_.disperse_turn_push_budget_us) {
                    disperse_turn_phase_ = 0; return true;
                }
                reason_ = "disperse_turn_push"; drive(t_.disperse_speed, 0);
                return false;
            }
            const float dir = disperse_turn_dir_;
            const float turned = wrapAngle(in.heading_rad - sweep_yaw0_) * dir;
            float cap = t_.disperse_turn_rad;
            if (swing_plan_active_) cap = std::min(cap, dir > 0 ? swing_cap_pos_ : swing_cap_neg_);
            const bool out_of_room = turned >= cap - t_.disperse_swing_tol_rad || now - sweep_start_us_ > t_.disperse_swing_budget_us;
            const auto endTurn = [&]() -> bool {
                if (t_.disperse_turn_push_m > 0.f) { disperse_turn_phase_ = 3; sweep_start_us_ = now; sweep_travel0_ = travel_; return false; }
                disperse_turn_phase_ = 0; return true;
            };
            const auto spin = [&](const char *why) {
                const bool fast = (burst_swing_ && state_ == PushState::SCATTER) || (t_.approach_dense_burst_swing && state_ == PushState::APPROACH && !sweep_non_supply_active_);
                reason_ = why;
                drive(0.f, dir);
                if (motion.wz_rps != 0) motion.wz_rps = std::copysign(fast ? t_.first_burst_swing_wz : t_.disperse_wz, dir);
            };
            if (disperse_turn_phase_ == 4) { // camera at the lowest angle: keep turning until few blocks sit in the frame zone
                if (out_of_room || turned - disperse_frame_turned0_ >= t_.disperse_turn_frame_extra_rad) return endTurn();
                if (desiredPitch() != pitch_cmd_ || !in.camera_pitch_stable) { reason_ = "disperse_frame_look"; return false; }
                if (!disperse_look_ready_us_) disperse_look_ready_us_ = now;
                if (in.clutter_ts_us >= disperse_look_ready_us_ && in.clutter_ts_us != disperse_frame_ts_us_ && now >= in.clutter_ts_us && now - in.clutter_ts_us <= t_.sweep_fresh_us) {
                    int n = 0;
                    for (const auto &q : in.clutter_body)
                        if (std::isfinite(q.x) && std::isfinite(q.y) && q.y > 0.f && q.y <= t_.disperse_turn_frame_range_m && std::abs(q.x) <= t_.disperse_turn_frame_half_m) ++n;
                    disperse_frame_ts_us_ = in.clutter_ts_us;
                    disperse_clear_n_ = n <= t_.disperse_turn_frame_max ? disperse_clear_n_ + 1 : 0;
                    std::cout << "[TURN] frame_zone n=" << n << " max=" << t_.disperse_turn_frame_max << " turned=" << turned << " ok_frames=" << disperse_clear_n_ << "\n";
                }
                if (disperse_clear_n_ >= t_.disperse_turn_clear_frames) return endTurn();
                if (!disperse_frame_ts_us_) { // no frame from the low camera yet: wait a bounded time, then stop turning rather than spin blind
                    if (now - disperse_look_ready_us_ >= t_.disperse_turn_look_us) return endTurn();
                    reason_ = "disperse_frame_look"; return false;
                }
                if (disperse_clear_n_) { reason_ = "disperse_frame_confirm"; return false; }
                spin("disperse_turn_frame");
                return false;
            }
            int l = 0, r = 0;
            if (viewCount(l, r)) disperse_clear_n_ = l + r <= t_.disperse_turn_clear_max ? disperse_clear_n_ + 1 : 0;
            const bool thin = disperse_clear_n_ >= t_.disperse_turn_clear_frames;
            if (out_of_room) return endTurn();
            if (thin && turned >= std::min(t_.disperse_turn_min_rad, cap)) {
                if (!(t_.disperse_turn_frame_check && midPitch())) return endTurn();
                disperse_turn_phase_ = 4; disperse_frame_turned0_ = turned; disperse_look_ready_us_ = 0; disperse_frame_ts_us_ = 0; disperse_clear_n_ = 0;
                reason_ = "disperse_frame_look"; return false;
            }
            spin("disperse_turn");
            return false;
        }
        const float dyaw5 = wrapAngle(in.heading_rad - sweep_yaw0_);
        const bool centring = disperse_swing_n_ >= disperse_swings_;
        const float side = ((disperse_swing_n_ % 2 == 0) ? 1.f : -1.f) * disperse_sign_;
        float swing_mag = disperse_small_ ? t_.mid_neighbour_clear_turn_rad : t_.disperse_turn_rad;
        if (!centring && swing_plan_active_) swing_mag = std::min(swing_mag, side > 0 ? swing_cap_pos_ : swing_cap_neg_);
        const float goal = centring ? 0.f : side * swing_mag;
        const float dir = goal > dyaw5 ? 1.f : -1.f;
        const bool reached = std::abs(goal - dyaw5) <= (centring ? t_.sweep_return_tol_rad : t_.disperse_swing_tol_rad) ||
            (dir > 0 ? dyaw5 >= goal : dyaw5 <= goal);
        if (reached || now - sweep_start_us_ > t_.disperse_swing_budget_us) {
            if (!centring) { ++disperse_swing_n_; sweep_start_us_ = now; return false; }
            if (++disperse_pass_ < disperse_passes_) { // go in again, deeper and starting the swings on the other side
                disperse_rush_m_ = t_.disperse_pass_extra_m; disperse_sign_ = -disperse_sign_;
                sweep_phase_ = 4; sweep_start_us_ = now; sweep_travel0_ = travel_;
                return false;
            }
            return true;
        }
        const bool creeping = !centring && travel_ - sweep_travel0_ < (disperse_small_ ? t_.mid_neighbour_clear_creep_m : t_.disperse_swing_creep_m) * motionRatio();
        const bool burst = burst_swing_ && state_ == PushState::SCATTER;
        const bool fast = burst || (t_.approach_dense_burst_swing && state_ == PushState::APPROACH && !sweep_non_supply_active_ && !disperse_small_);
        reason_ = burst ? (centring ? "first_burst_centre" : "first_burst_swing") : (centring ? "disperse_centre" : "disperse_swing");
        drive(creeping ? t_.disperse_swing_vx : 0.f, dir);
        if (motion.wz_rps != 0) motion.wz_rps = std::copysign(fast ? t_.first_burst_swing_wz : t_.disperse_wz, dir);
        return false;
    };
    syncFrameOverlay(now); // a state change made last tick takes effect before this tick's state logic runs
    if (sweep_open_ && !sweep_open_done_ && in.gripper_done && in.gripper_feedback_open == 1 && now > sweep_open_us_) sweep_open_done_ = true;
    if (sweep_close_wait_) { // the frame opened for a sweep must be back down before the chassis moves on
        if (frame_raised_ != 0 || (in.gripper_done && in.gripper_feedback_open == 0)) sweep_close_wait_ = false;
        else if (now - sweep_close_us_ > t_.gripper_timeout_us) {
            sweep_close_wait_ = false; sweep_open_disabled_ = true;
            std::cout << "[SWEEP_FRAME] close not confirmed within " << t_.gripper_timeout_us / 1000 << " ms: sweep frame-open disabled\n";
        } else { reason_ = "sweep_frame_close_wait"; return result(); }
    }
    // After any sweep, hold the chassis still for fresh observations. SCAN/APPROACH then
    // re-evaluate targets and corridors; a completed angle is not proof of a cleared lane.
    if (t_.sweep_observation_guard && sweep_observe_until_us_ &&
        (state_ == PushState::SCAN || state_ == PushState::APPROACH || state_ == PushState::MID_APPROACH || state_ == PushState::MID_REACQUIRE)) {
        if (now < sweep_observe_until_us_) { reason_ = "sweep_reobserve"; return result(); }
        sweep_observe_until_us_ = 0;
    }
    switch (state_) {
    case PushState::START_ADVANCE:
        commandFrame(1, now); // frame stays open to cross the bumpy section
        if (startup_commanded_us_ >= t_.startup_advance_us) {
            enter(PushState::SCAN, now, "startup_advance_complete");
            break;
        }
        if(elapsed > t_.startup_advance_us+5000000) {fail("startup_forward_blocked");break;}
        {
            float hold_wz=0;
            if (in.heading_valid) {
                if (!startup_yaw0_set_) {startup_yaw0_=in.heading_rad;startup_yaw0_set_=true;}
                const float err=wrapAngle(in.heading_rad-startup_yaw0_); // body yaw grows on left turns
                if (std::abs(err)>t_.startup_hold_deadband_rad) hold_wz=-t_.startup_hold_gain*err;
            }
            reason_=!forward_ok?"startup_path_blocked":hold_wz!=0?"startup_forward_hold":"startup_forward";
            drive(t_.startup_advance_speed, hold_wz);
        }
        break;
    case PushState::SCAN: {
        commandFrame(0, now);
        approach_forward_ = 0; // scans turn in place: the retreat bound restarts at the lock
        const bool cue=cueCandidate();
        const bool cand = candidate(in, now);
        if (in.heading_valid && std::isfinite(in.heading_rad)) {
            if (scan_last_valid_) scan_turn_acc_ += std::abs(wrapAngle(in.heading_rad - scan_last_heading_));
            scan_last_heading_ = in.heading_rad; scan_last_valid_ = true;
        } else scan_last_valid_ = false;
        cv::Point2f weak_pt(0.f, 0.f);
        bool weak_hold = false;
        if (cand && !cue && scatter_ok && weakCandidate(in)) {
            weak_pt = cv::Point2f(-in.distance_m * std::sin(in.heading_error), in.distance_m * std::cos(in.heading_error));
            weak_hold = weak_pt.y >= t_.blind_scatter_min_y_m && weak_pt.y <= far_y && std::abs(weak_pt.x) <= t_.blind_scatter_half_x_m &&
                std::abs(std::atan2(-weak_pt.x, weak_pt.y)) <= t_.blind_scatter_max_bearing_rad;
        }
        // A lock this far out is not trusted (class / position are off at this range): a group of several objects is enough to drive toward; once it is within blind_scatter_far_chase_m the normal decision takes over.
        if (cand && !cue && !weak_hold && scatter_ok && approach_ok && std::isfinite(in.distance_m) && in.distance_m > t_.blind_scatter_far_chase_m) {
            std::vector<cv::Point2f> pts = in.clutter_body;
            pts.emplace_back(-in.distance_m * std::sin(in.heading_error), in.distance_m * std::cos(in.heading_error));
            ord_extra.push_back(pts.back());
            if (!clutter_fresh) { const cv::Point2f lock_pt = pts.back(); pts.assign(1, lock_pt); }
            findBest(pts, far_y, t_.blind_scatter_far_radius_m);
            // An ordinary supply that is being confirmed (confirm_frames) is not pre-empted by the shorter cluster count. The hold starts at the first held tick and ends after sweep_confirm_wait_us
            // whatever the lock does (an id that flips every frame cannot hold the cluster back for ever); the opening burst drive is never held back.
            const bool pending = t_.sweep_confirm_wait_us > 0 && confirmations_ > 0 && confirmations_ < t_.confirm_frames &&
                targetKind(in.label) == TargetKind::ORDINARY && !(t_.first_burst && !burst_drive_used_);
            if (best_n >= t_.blind_scatter_far_lock_min && best_d > t_.blind_scatter_max_y_m) {
                bool hold = false;
                if (pending) { if (!confirm_since_us_) confirm_since_us_ = now; hold = now - confirm_since_us_ < t_.sweep_confirm_wait_us; }
                if (hold) { far_seen_ = 0; reason_ = "sweep_defer_target_confirm"; }
                else if (++far_seen_ >= t_.sweep_confirm) {
                    far_seen_ = 0; confirm_since_us_ = 0;
                    beginScatter(true, false, "blind_scatter_far_group");
                    break;
                }
            } else { far_seen_ = 0; confirm_since_us_ = 0; }
        } else if (t_.dense_no_lock && cand && !cue && !weak_hold && (scatter_ok || scatter_multi_ok) && clutter_fresh && std::isfinite(in.distance_m)) {
            // Several objects together: do not lock one of them, push on and split the group (the committed sweep ignores the lock).
            far_seen_ = 0; confirm_since_us_ = 0;
            std::vector<cv::Point2f> pts = in.clutter_body;
            pts.emplace_back(-in.distance_m * std::sin(in.heading_error), in.distance_m * std::cos(in.heading_error));
            ord_extra.push_back(pts.back());
            // Lock only while few targets are in view: beyond lock_max_visible any pair that sits together is swept open-frame instead.
            int visible_n = 0;
            for (const auto &q : pts) if (std::isfinite(q.x) && std::isfinite(q.y) && q.y >= t_.blind_scatter_min_y_m && q.y <= far_y) ++visible_n;
            const bool multi = t_.lock_max_visible > 0 && visible_n > t_.lock_max_visible && scatter_multi_ok;
            int g = 0;
            if (multi) g = evalGroup(pts, t_.lock_multi_group_min, t_.lock_multi_group_min, t_.lock_multi_radius_m);
            const bool multi_hit = g != 0;
            if (!g && scatter_ok) g = evalGroup(pts, t_.blind_scatter_min, t_.blind_scatter_far_min);
            if (g) {
                if (++dense_seen_ < t_.sweep_confirm) { reason_ = "dense_group_confirm"; break; }
                dense_seen_ = 0;
                beginScatter(g == 2, true, multi_hit ? (g == 2 ? "multi_visible_far" : "multi_visible_split") : (g == 2 ? "dense_group_far" : "dense_group_split"));
                break;
            }
            dense_seen_ = 0;
        } else { far_seen_ = 0; confirm_since_us_ = 0; dense_seen_ = 0; }
        if ((!cand && !cue) || weak_hold) {
            confirmations_ = 0; target_id_ = -1;cue_mode_=false;
            reason_=scan_ok?"searching_target":"scan_path_blocked";
            if(scan_ok && in.target_is_search_cue && clear_attempts_>=t_.max_clear_attempts)reason_="cue_attempt_limit";
            if(weak_hold)reason_="weak_target_scatter_wait";
            if(unstick())break;
            if (t_.visual_chase && !cue) {
                const bool seen = in.target_valid && in.target_id >= 0 && in.pixel_bearing_valid && std::isfinite(in.pixel_bearing) &&
                    in.pixel_confidence >= t_.visual_chase_min_conf && targetSelectable(in.label, first_) && !in.target_in_zone;
                if (seen) { ++vis_hits_; vis_miss_ = 0; vis_bearing_ = in.pixel_bearing; vis_last_seen_us_ = now; }
                else if (++vis_miss_ > t_.visual_chase_miss_frames) { vis_hits_ = 0; vis_chase_t0_ = 0; }
                if (vis_hits_ >= t_.visual_chase_frames && now - vis_last_seen_us_ <= t_.visual_chase_hold_us && now >= vis_cool_until_us_ && scan_ok) {
                    if (!vis_chase_t0_) vis_chase_t0_ = now;
                    if (now - vis_chase_t0_ > t_.visual_chase_budget_us) { vis_hits_ = 0; vis_chase_t0_ = 0; vis_cool_until_us_ = now + t_.visual_chase_cool_us; }
                    else if (std::abs(vis_bearing_) > t_.rotate_in_place_rad) { reason_ = "visual_chase_turn"; drive(0, t_.heading_gain * vis_bearing_); break; }
                    else if (forward_ok) {
                        reason_ = "visual_chase";
                        drive(t_.visual_chase_speed, std::abs(vis_bearing_) > t_.heading_deadband_rad ? t_.heading_gain * vis_bearing_ : 0.f);
                        break;
                    }
                }
            }
            // Nothing pickable in view but other objects are: they may be wrapping the ordinary supplies. Plough through the densest group.
            // A far / few-pixel ordinary sighting is not worth chasing either: it joins the group and is ploughed through.
            if (scatter_ok) {
                if (!scatter_idle_since_us_) scatter_idle_since_us_ = now;
                const bool looked = weak_hold || scatter_count_ > 0 || scan_turn_acc_ >= t_.blind_scatter_first_turn_rad;
                const bool fresh = weak_hold || (looked && in.clutter_ts_us && now >= in.clutter_ts_us && now - in.clutter_ts_us <= t_.sweep_fresh_us);
                if (fresh && now - scatter_idle_since_us_ >= (weak_hold ? t_.weak_scatter_wait_us : t_.blind_scatter_wait_us)) {
                    std::vector<cv::Point2f> pts = in.clutter_body;
                    if (weak_hold) { pts.push_back(weak_pt); ord_extra.push_back(weak_pt); }
                    const int need_n = weak_hold ? 1 : t_.blind_scatter_min;
                    const int g = evalGroup(pts, need_n, weak_hold ? 1 : t_.blind_scatter_far_min);
                    if (g) {
                        if (++scatter_seen_ < t_.sweep_confirm) { reason_ = "blind_scatter_confirm"; break; }
                        // far: too far for the rush budget, drive closer first, then the normal trigger fires from the short range
                        beginScatter(g == 2, false, g == 2 ? "blind_scatter_far_start" : "blind_scatter_start");
                        break;
                    }
                    scatter_seen_ = 0;
                }
            }
            if (t_.scan_sighting_hold && !cue && now >= vis_cool_until_us_) {
                const bool sighted = in.pixel_reason == "confirming_three_frames" || in.pixel_reason == "pixel_ratio_locked" || in.pixel_reason == "lock_grace_waiting" ||
                    (vis_hits_ > 0 && now - vis_last_seen_us_ <= t_.visual_chase_hold_us);
                if (sighted) scan_hold_last_sight_us_ = now;
                if (!scan_hold_t0_ && sighted && now >= scan_hold_cool_until_us_) scan_hold_t0_ = now;
                if (scan_hold_t0_) {
                    if (now - scan_hold_t0_ <= t_.scan_hold_max_us && now - scan_hold_last_sight_us_ <= t_.scan_hold_lost_us) {
                        reason_ = "scan_sighting_hold"; drive(0, 0); break;
                    }
                    scan_hold_t0_ = 0; scan_hold_cool_until_us_ = now + t_.scan_hold_cool_us;
                }
            }
            scanSpin(); break;
        }
        scatter_idle_since_us_ = 0; scatter_seen_ = 0;
        if (confirmations_ + 1 >= t_.confirm_frames) scatter_approach_n_ = 0;
        if (target_id_ != in.target_id || label_ != in.label || cue_mode_!=cue) {
            target_id_ = in.target_id; label_ = in.label; confirmations_ = 0;cue_mode_=cue;
            if(cue) {search_cue_id_=target_id_;capture_target_id_=-1;}
            else {capture_target_id_=target_id_;search_cue_id_=-1;}
        }
        if (++confirmations_ >= t_.confirm_frames) {
            burst_swing_armed_ = false; // a normal lock ends the opening: no fast swing later in the match
            if(t_.demo_search_only){fail("demo_target_found");state_=PushState::SAFE_STOP;}
            else enter(cue?PushState::CUE_APPROACH:PushState::APPROACH, now, cue?"search_cue_locked":"target_locked");
        }
        break;
    }
    case PushState::SCATTER: {
        commandFrame(0, now);
        approach_forward_ = 0;
        const auto scatterEnd = [&](const char *why) {
            burst_fast_ = burst_swing_ = burst_swing_armed_ = false;
            scatter_cool_until_us_ = now + t_.blind_scatter_cooldown_us;
            const bool swung = sweepChainClose(now);
            if (swung || std::strcmp(why, "blind_scatter_target_seen") != 0) sweep_gap_until_us_ = now + t_.sweep_gap_us; // a target seen before any swing leaves the dense-pile check open for APPROACH
            scan_mid_limit_ = scan_turn_acc_ + 2.f * float(CV_PI); // first full turn after the scatter looks at the close range
            rejected_.clear(); rejected_bearings_.clear(); // the group has moved: earlier vetoes no longer apply
            if (swung && std::strcmp(why, "blind_scatter_done") == 0 && beginPileFromSweep("pile_chain_scatter")) return;
            enter(PushState::SCAN, now, why);
        };
        const auto approachEnd = [&](const char *why) { // no sweep happened: keep the scatter budget, retrigger from the short range at once
            scatter_far_ = false; ++scatter_approach_n_; burst_fast_ = false; sweepChainPause(now);
            enter(PushState::SCAN, now, why);
            scatter_idle_since_us_ = now > t_.blind_scatter_wait_us ? now - t_.blind_scatter_wait_us : 1;
        };
        const auto sightedTarget = [&] { // driving to a far group: a lock does not stop the drive until it is inside the short window (the group sweep itself checks first); a committed sweep ignores locks
            return !scatter_commit_ && candidate(in, now) && !weakCandidate(in) && !(scatter_far_ && in.distance_m > t_.blind_scatter_max_y_m);
        };
        if (!in.heading_valid || !std::isfinite(in.heading_rad)) { scatter_far_ = false; scatterEnd("blind_scatter_no_heading"); break; }
        if (elapsed > t_.blind_scatter_budget_us) { scatter_far_ = false; scatterEnd("blind_scatter_timeout"); break; }
        if (sweepOver) { scatter_far_ = false; scatterEnd(now - sweep_chain_t0_ > t_.sweep_op_budget_us ? "sweep_op_timeout" : "sweep_budget_exhausted"); break; }
        if (t_.sweep_view_handoff && midPitch() && sweep_phase_ != 6) { // carry's camera ladder toward the group: down only, the base waits while it tilts
            const float left = scatter_range_ - travel_ / motionRatio();
            if (t_.sweep_view_cap_mid) {
                if (sweep_phase_ < 4 && (!scatter_far_ || left <= t_.track_near_m) && sweep_pitch_ < t_.intermediate_pitch_cdeg) sweep_pitch_ = t_.intermediate_pitch_cdeg;
            } else if (left <= t_.intermediate_to_near_m) sweep_pitch_ = t_.track_pitch_cdeg;
            else if (left <= t_.track_near_m && sweep_pitch_ == t_.far_pitch_cdeg) sweep_pitch_ = t_.intermediate_pitch_cdeg;
        }
        if (sweep_phase_ == 6) { // back out the distance driven in
            if (sightedTarget()) { scatterEnd("blind_scatter_target_seen"); break; }
            if (travel_ - sweep_travel0_ >= scatter_back_ || now - sweep_start_us_ > t_.blind_scatter_back_budget_us) { scatterEnd("blind_scatter_done"); break; }
            if (!in.retreat_safe) { scatterEnd("blind_scatter_back_unsafe"); break; } // the rear is blind: no reversing without the adapter's say-so, the sweep ends where it is
            reason_ = "blind_scatter_back"; drive(-t_.blind_scatter_back_speed, 0);
            if (motion.vx_mps < 0) motion.vx_mps = -t_.blind_scatter_back_speed;
            break;
        }
        if (t_.sweep_observation_guard && !clutter_fresh && !candidate(in, now) && !(t_.sweep_far_judge && sweep_phase_ >= 4) && !(disperse_turn_phase_ && sweep_phase_ == 5)) {
            scatterEnd("blind_scatter_observation_lost"); break;
        }
        if (sweep_phase_ == 0) {
            if (sightedTarget()) { scatter_far_ = false; scatterEnd("blind_scatter_target_seen"); break; }
            if (elapsed > t_.blind_scatter_align_budget_us) { scatter_far_ = false; scatterEnd("blind_scatter_align_timeout"); break; }
            const float err = wrapAngle(scatter_yaw_ - in.heading_rad); // body yaw grows on left turns
            if (std::abs(err) > t_.blind_scatter_align_tol_rad) {
                reason_ = "blind_scatter_align"; drive(0, err);
                if (motion.wz_rps != 0) motion.wz_rps = std::copysign(std::abs(err) > .3f ? t_.blind_scatter_align_wz : .5f * t_.blind_scatter_align_wz, err);
                break;
            }
            if (scatter_far_) { sweep_phase_ = 7; sweep_start_us_ = now; sweep_travel0_ = travel_; }
            else {
                disperse_rush_m_ = std::clamp(scatter_range_ - t_.sweep_frame_front_m + t_.blind_scatter_depth_m, 0.f, t_.blind_scatter_max_m);
                sweep_phase_ = 4; sweep_start_us_ = sweep_t0_us_ = now; sweep_travel0_ = travel_;
            }
        }
        if (sweep_phase_ == 7) { // drive toward the far group at normal speed with IMU yaw hold
            const bool ordinary_lock = t_.sweep_confirm_wait_us > 0 && sightedTarget() && targetKind(in.label) == TargetKind::ORDINARY && (!burst_swing_armed_ || (t_.sweep_observation_guard && !t_.first_burst_full_impact)); // first opening pile keeps its impact; a lone target still exits through sightedTarget below (SCAN, then APPROACH decides on the pile)
            if (scatter_ok && clutter_fresh && !ordinary_lock) { // the group reached the short window: sweep it now, whatever single lock is in view
                std::vector<cv::Point2f> pts = in.clutter_body;
                if (candidate(in, now) && std::isfinite(in.distance_m)) { pts.emplace_back(-in.distance_m * std::sin(in.heading_error), in.distance_m * std::cos(in.heading_error)); ord_extra.push_back(pts.back()); }
                findBest(pts, t_.blind_scatter_max_y_m, t_.disperse_radius_m);
                if (best_n >= t_.blind_scatter_min) {
                    if (++scatter_seen_ < t_.sweep_confirm) { reason_ = "blind_scatter_near_confirm"; break; }
                    scatter_seen_ = 0;
                    beginScatter(false, true, "blind_scatter_near_group");
                    break;
                }
            }
            scatter_seen_ = 0;
            if (sightedTarget()) { scatter_far_ = false; scatterEnd("blind_scatter_target_seen"); break; }
            if (travel_ - sweep_travel0_ >= scatter_approach_m_ * motionRatio()) { approachEnd("blind_scatter_approached"); break; }
            if (now - sweep_start_us_ > t_.blind_scatter_approach_budget_us) { approachEnd("blind_scatter_approach_timeout"); break; }
            const float err = wrapAngle(scatter_yaw_ - in.heading_rad);
            const float hold_wz = std::abs(err) > t_.startup_hold_deadband_rad ? t_.startup_hold_gain * err : 0.f;
            const bool fast = burst_fast_ && (t_.first_burst_full_impact || scatter_approach_m_ - (travel_ - sweep_travel0_) / motionRatio() > t_.first_burst_slow_m);
            reason_ = fast ? "first_burst_approach" : "blind_scatter_approach";
            drive(fast ? t_.first_burst_speed : t_.approach_speed, hold_wz);
            if (fast && motion.vx_mps > 0) motion.vx_mps = t_.first_burst_speed; // drive() caps at max_speed; the chassis packs up to 1.0 m/s
            break;
        }
        if (disperseStep()) {
            if (t_.blind_scatter_return && travel_ > .05f * motionRatio()) {
                scatter_back_ = travel_ * t_.blind_scatter_back_frac; sweep_phase_ = 6; sweep_start_us_ = now; sweep_travel0_ = travel_;
            } else scatterEnd("blind_scatter_done");
        }
        break;
    }
    case PushState::PILE: {
        commandFrame(0, now);
        approach_forward_ = 0;
        const auto pileEnd = [&](const char *why) {
            const uint64_t until = now + t_.pile_cool_us;
            pile_cool_until_us_ = until;
            scatter_cool_until_us_ = std::max(scatter_cool_until_us_, until);
            disperse_cool_until_us_ = std::max(disperse_cool_until_us_, until);
            sweep_gap_until_us_ = std::max(sweep_gap_until_us_, until);
            target_id_ = -1; confirmations_ = 0; cue_mode_ = false; sweep_phase_ = 0; disperse_small_ = false;
            rejected_.clear(); rejected_bearings_.clear();
            scan_mid_limit_ = scan_turn_acc_ + 2.f * float(CV_PI);
            std::cout << "[PILE] end why=" << why << " runs=" << pile_runs_ << "\n";
            enter(PushState::SCAN, now, why);
        };
        const auto setPhase = [&](int ph) { pile_phase_ = ph; pile_phase_us_ = now; pile_seen_ = 0; pile_travel0_ = travel_; pile_push0_ = -1.f; };
        if (!in.heading_valid || !std::isfinite(in.heading_rad)) { pileEnd("pile_no_heading"); break; }
        if (elapsed > t_.pile_budget_us) { pileEnd("pile_timeout"); break; }
        const float yaw = in.heading_rad;
        struct Blk { cv::Point2f p; bool ordinary = false; };
        // Visible blocks in body coordinates; the locked target can be missing from the clutter list.
        const auto pileBlocks = [&] {
            std::vector<Blk> v;
            if (clutter_fresh) {
                const bool centres = in.clutter_center.size() == in.clutter_body.size(); // box centres sit on the block, the ground-contact points sit on its near face
                for (size_t i = 0; i < in.clutter_body.size(); ++i) {
                    const auto &u = in.clutter_body[i];
                    if (!std::isfinite(u.x) || !std::isfinite(u.y)) continue;
                    Blk b; b.p = u;
                    if (centres && std::isfinite(in.clutter_center[i].x) && std::isfinite(in.clutter_center[i].y)) b.p = in.clutter_center[i];
                    for (const auto &o : in.clutter_ordinary) if (std::abs(o.x - u.x) < 1e-4f && std::abs(o.y - u.y) < 1e-4f) { b.ordinary = true; break; }
                    v.push_back(b);
                }
            }
            if (in.target_valid && in.geometry_valid && !in.target_in_zone && std::isfinite(in.distance_m) && std::isfinite(in.heading_error)) {
                cv::Point2f tp(-in.distance_m * std::sin(in.heading_error), in.distance_m * std::cos(in.heading_error));
                if (in.target_center_valid && std::isfinite(in.target_center_body.x) && std::isfinite(in.target_center_body.y)) tp = in.target_center_body;
                const bool tord = targetKind(in.label) == TargetKind::ORDINARY;
                bool dup = false;
                for (const auto &u : v) if (u.ordinary == tord && cv::norm(u.p - tp) < .03) { dup = true; break; } // a touching block of another type is a different block
                if (!dup) { Blk b; b.p = tp; b.ordinary = tord; v.push_back(b); }
            }
            return v;
        };
        // The cluster straight ahead: the block nearest the heading and everything within pile_cluster_radius_m of it.
        const auto pileCluster = [&] {
            std::vector<Blk> c; const auto v = pileBlocks();
            int seed = -1; float bb = 1e9f;
            for (size_t i = 0; i < v.size(); ++i) {
                if (v[i].p.y < .08f || v[i].p.y > 1.2f || std::abs(v[i].p.x) > t_.pile_cluster_half_x_m) continue;
                const float b = std::abs(std::atan2(-v[i].p.x, v[i].p.y));
                if (b < bb) { bb = b; seed = int(i); }
            }
            if (seed < 0) return c;
            for (const auto &u : v)
                if (std::abs(u.p.x) <= t_.pile_cluster_half_x_m && cv::norm(u.p - v[seed].p) <= t_.pile_cluster_radius_m) c.push_back(u);
            return c;
        };
        const auto faceYaw = [&](float target_yaw, float tol, const char *why) { // in-place turn to a world yaw; true when aligned
            const float err = wrapAngle(target_yaw - yaw);
            if (std::abs(err) <= tol) return true;
            reason_ = why;
            const float wz = std::copysign(std::abs(err) > .5f ? t_.pile_turn_wz : .5f * t_.pile_turn_wz, err);
            drive(0, wz);
            if (motion.wz_rps != 0) motion.wz_rps = wz;
            return false;
        };
        const auto backTo = [&](float metres, uint64_t budget) { // reverse a measured distance; true when done
            if (!in.retreat_safe) { reason_ = "pile_back_unsafe"; return true; }
            if (travel_ - pile_travel0_ >= metres * motionRatio() || now - pile_phase_us_ > budget) return true;
            drive(-t_.blind_scatter_back_speed, 0);
            if (motion.vx_mps < 0) motion.vx_mps = -t_.blind_scatter_back_speed;
            return false;
        };
        switch (pile_phase_) {
        case 0: { // frame open (the rest of the flow carries it open), stand still for a moment
            if (!sweepFrameReady()) break;
            if (now - pile_phase_us_ < 400000) { reason_ = "pile_settle"; break; }
            std::cout << "[PILE] start yaw0=" << pile_yaw0_ << " corridor=" << in.corridor.total() << " distance=" << in.distance_m << "\n";
            setPhase(1);
            break;
        }
        case 1: { // face the pile
            if (now - pile_phase_us_ > t_.pile_advance_budget_us) { pileEnd("pile_face_pile_timeout"); break; }
            if (faceYaw(pile_yaw0_, .6f * t_.pile_align_tol_rad, "pile_face_pile")) setPhase(2);
            break;
        }
        case 2: { // straight into the pile with the frame open: pile_advance_m at pile_advance_speed
            if ((travel_ - pile_travel0_) >= t_.pile_advance_m * motionRatio() || now - pile_phase_us_ > t_.pile_advance_budget_us) { setPhase(3); break; }
            const float err = wrapAngle(pile_yaw0_ - yaw);
            reason_ = "pile_advance";
            drive(t_.pile_advance_speed, std::abs(err) > .04f ? t_.heading_gain * err : 0.f);
            break;
        }
        case 3: { // short back-off so the camera sees the remaining cluster
            if (backTo(t_.pile_view_back_m, t_.pile_back_budget_us)) { setPhase(4); break; }
            reason_ = "pile_view_back";
            break;
        }
        case 4: { // measure the cluster, turn by one block's angular size to the better side and cut in
            if (now - pile_phase_us_ < t_.pile_measure_us) { reason_ = "pile_measure"; break; }
            const auto c = pileCluster();
            if (c.size() < 2) {
                if (++pile_seen_ < 8) { reason_ = "pile_measure_wait"; break; }
                pileEnd("pile_cluster_clear"); break;
            }
            float cx = 0, cy = 0, ymin = 1e9f;
            for (const auto &b : c) { cx += b.p.x / float(c.size()); cy += b.p.y / float(c.size()); ymin = std::min(ymin, b.p.y); }
            const float range = std::max(.10f, float(std::hypot(cx, cy)));
            const float theta = 2.f * std::atan(t_.pile_block_half_m / range); // one block as the camera sees it at this range
            struct Eval { int ord_in = 0, other_in = 0, ord_shed = 0, other_shed = 0; float along_min = 1e9f; float need = 0, cost = 0; };
            const auto eval = [&](float beta) { // frame edge line through the origin at body bearing beta (left positive)
                Eval e; const float un = -std::sin(beta), uy = std::cos(beta), nx = std::cos(beta), ny = std::sin(beta);
                for (const auto &b : c) {
                    const float along = b.p.x * un + b.p.y * uy, lat = b.p.x * nx + b.p.y * ny;
                    const bool in_frame = along > 0 && std::abs(lat) <= t_.pile_frame_half_m + t_.pile_cut_margin_m;
                    if (in_frame) { (b.ordinary ? e.ord_in : e.other_in)++; e.along_min = std::min(e.along_min, along); }
                    else (b.ordinary ? e.ord_shed : e.other_shed)++;
                }
                // Keep one ordinary supply in the frame (the capture look carries exactly one) and drop everything else the frame would enclose.
                e.need = 100.f * float(e.other_in) + 20.f * float(std::max(0, e.ord_in - 1));
                e.cost = e.need + 3.f * float(e.ord_shed);
                return e;
            };
            const Eval e0 = eval(0.f), el = eval(theta), er = eval(-theta);
            std::cout << "[PILE] measure n=" << c.size() << " range=" << range << " block_deg=" << theta * 57.2958f
                      << " straight(cost=" << e0.cost << " in=" << e0.ord_in << "+" << e0.other_in << ") left(cost=" << el.cost << " in=" << el.ord_in << "+" << el.other_in
                      << " shed=" << el.ord_shed << "+" << el.other_shed << ") right(cost=" << er.cost << " in=" << er.ord_in << "+" << er.other_in
                      << " shed=" << er.ord_shed << "+" << er.other_shed << ")\n";
            // Same-type first: a mixed frame is the dominant cost (other_in), so widen the cut one block at a time until the enclosed set is a single ordinary block; ties keep the smaller turn, then the left side.
            bool left = el.cost <= er.cost; int best_k = 1; Eval best = left ? el : er;
            for (int k = 2; k <= t_.pile_cut_max_steps && best.need > 0; ++k) {
                const Eval wl = eval(k * theta), wr = eval(-k * theta);
                for (const bool wleft : {true, false}) { // a wide turn can sweep the frame clear of every block: only cuts that still enclose an ordinary supply count
                    const Eval &wb = wleft ? wl : wr;
                    if (wb.ord_in > 0 && wb.cost < best.cost) { best = wb; left = wleft; best_k = k; }
                }
            }
            if (e0.need <= 0) { pileEnd("pile_frame_already_ok"); break; } // nothing the corridor rules would reject is enclosed any more
            if (!(best.need < e0.need)) { pileEnd("pile_cut_no_gain"); break; }
            pile_cut_yaw_ = wrapAngle(yaw + (left ? 1.f : -1.f) * float(best_k) * theta);
            pile_cut_m_ = std::clamp(best.along_min - t_.sweep_frame_front_m + t_.pile_cut_depth_m, 0.f, .5f);
            std::cout << "[PILE] cut side=" << (left ? "left" : "right") << " steps=" << best_k << " turn_deg=" << best_k * theta * 57.2958f << " in=" << best.ord_in << "+" << best.other_in << " travel=" << pile_cut_m_ << "\n";
            setPhase(5);
            break;
        }
        case 5: { // turn, then drive the edge in
            const auto afterPush = [&] { // the push is in: look at the frame zone from the lowest camera before backing out
                if (t_.pile_cut_capture && rushLogic() && pile_push0_ >= 0) {
                    pile_low_ = true; pile_look_n_ = pile_look_ok_ = pile_look_bad_ = 0; pile_look_us_ = pile_look_ts_us_ = 0;
                    setPhase(12);
                } else setPhase(6);
            };
            if (now - pile_phase_us_ > t_.pile_cut_budget_us) { pile_cut_m_ = pile_push0_ >= 0 ? (travel_ - pile_push0_) / motionRatio() : 0.f; afterPush(); break; }
            if (pile_push0_ >= 0) {
                if (travel_ - pile_push0_ >= pile_cut_m_ * motionRatio()) { afterPush(); break; }
                const float err = wrapAngle(pile_cut_yaw_ - yaw);
                reason_ = "pile_cut_drive";
                drive(t_.pile_cut_speed, std::abs(err) > .04f ? t_.heading_gain * err : 0.f);
                break;
            }
            if (faceYaw(pile_cut_yaw_, .1f, "pile_cut_align")) pile_push0_ = travel_;
            break;
        }
        case 6: { // back out to the viewing position; SCAN picks the target from what is left
            if (backTo(pile_cut_m_, t_.pile_back_budget_us)) { pileEnd("pile_split_done"); break; }
            reason_ = "pile_cut_back";
            break;
        }
        case 7: { // chain: is the pushed-out cluster still more than one layer deep?
            if (!sweepFrameReady()) break;
            if (now - pile_phase_us_ < t_.pile_measure_us) { reason_ = "pile_chain_measure"; break; }
            const auto c = pileCluster();
            if (c.size() < 2) {
                if (++pile_seen_ < 8) { reason_ = "pile_chain_wait"; break; }
                if (pile_chain_n_ > 0 && pile_chain_look_ < 2) { // the turn took the blocks out of view: look at the middle of the turn, then at the heading it rushed in on
                    pile_yaw0_ = pile_chain_look_ == 0 ? pile_chain_yaw_mid_ : pile_chain_yaw_pre_;
                    ++pile_chain_look_;
                    std::cout << "[PILE] chain relook " << pile_chain_look_ << " yaw=" << pile_yaw0_ << "\n";
                    setPhase(11); break;
                }
                pileEnd("pile_chain_cluster_clear"); break;
            }
            pile_chain_look_ = 0;
            float cx = 0, cy = 0, ymin = 1e9f, ymax = -1e9f;
            for (const auto &b : c) { cx += b.p.x / float(c.size()); cy += b.p.y / float(c.size()); ymin = std::min(ymin, b.p.y); ymax = std::max(ymax, b.p.y); }
            const float depth = ymax - ymin;
            const bool one_layer = depth <= t_.pile_layer_depth_m;
            std::cout << "[PILE] chain n=" << c.size() << " depth=" << depth << " one_layer=" << one_layer << " round=" << pile_chain_n_ << "/" << t_.pile_chain_max << "\n";
            if (one_layer || pile_chain_n_ >= t_.pile_chain_max) { setPhase(4); break; }
            ++pile_chain_n_;
            pile_yaw0_ = wrapAngle(yaw + std::atan2(-cx, cy));
            pile_chain_yaw_pre_ = pile_yaw0_;
            setPhase(8);
            break;
        }
        case 8: { // chain: face the cluster, then arm the disperse step (rush in + big-angle turn)
            if (now - pile_phase_us_ > t_.pile_advance_budget_us) { pileEnd("pile_chain_face_timeout"); break; }
            if (!faceYaw(pile_yaw0_, .6f * t_.pile_align_tol_rad, "pile_chain_face")) break;
            sweep_phase_ = 4; sweep_start_us_ = now; sweep_travel0_ = travel_; disperse_rush_m_ = t_.pile_chain_rush_m; disperse_small_ = false;
            swing_plan_pending_ = swing_plan_active_ = false; burst_swing_ = false; disperse_swing_n_ = 0; disperse_clear_n_ = 0; disperse_pass_ = 0;
            setPhase(9);
            break;
        }
        case 9: { // chain: one disperse step
            if (disperseStep()) {
                sweep_phase_ = 0; sweep_chain_swung_ = false;
                pile_chain_yaw_mid_ = wrapAngle(sweep_yaw0_ + .5f * wrapAngle(yaw - sweep_yaw0_));
                pile_chain_look_ = 0;
                pile_chain_in_m_ = std::max(0.f, travel_ - pile_travel0_) / motionRatio();
                setPhase(10);
            }
            break;
        }
        case 10: { // chain: back off part of the way in so the camera sees what is left
            if (backTo(std::max(pile_chain_in_m_ * t_.pile_chain_back_frac, t_.pile_chain_min_back_m), t_.pile_back_budget_us)) { pile_yaw0_ = pile_chain_yaw_mid_; setPhase(11); break; }
            reason_ = "pile_chain_back";
            break;
        }
        case 11: { // chain: turn to where the pushed-out blocks went, then measure
            if (now - pile_phase_us_ > t_.pile_advance_budget_us) { pileEnd("pile_chain_look_timeout"); break; }
            if (faceYaw(pile_yaw0_, .6f * t_.pile_align_tol_rad, "pile_chain_look")) setPhase(7);
            break;
        }
        case 12: { // capture look: camera at the lowest preset, exactly one ordinary block (nothing else) in the frame zone -> close the frame and carry
            const auto giveUp = [&](const char *why) { std::cout << "[PILE] capture look " << why << " -> back out\n"; pile_low_ = false; setPhase(6); };
            if (desiredPitch() != pitch_cmd_ || !in.camera_pitch_stable) { reason_ = "pile_capture_pitch"; break; }
            if (!pile_look_us_) pile_look_us_ = now;
            if (in.look_ts_us >= pile_look_us_ && in.look_ts_us != pile_look_ts_us_ && now >= in.look_ts_us && now - in.look_ts_us <= t_.sweep_fresh_us) {
                int n = 0, ord = 0;
                for (const auto &q : in.look_body) {
                    if (!std::isfinite(q.x) || !std::isfinite(q.y) || q.y <= 0.f || q.y > t_.disperse_turn_frame_range_m || std::abs(q.x) > t_.disperse_turn_frame_half_m) continue;
                    ++n;
                    for (const auto &o : in.look_ordinary) if (std::abs(o.x - q.x) < 1e-4f && std::abs(o.y - q.y) < 1e-4f) { ++ord; break; }
                }
                n += in.look_edge_ordinary + in.look_edge_other; ord += in.look_edge_ordinary;
                const bool ok = n == 1 && ord == 1;
                pile_look_ts_us_ = in.look_ts_us;
                pile_look_ok_ = ok ? pile_look_ok_ + 1 : 0; pile_look_bad_ = ok ? 0 : pile_look_bad_ + 1; if (ok) pile_look_n_ = n;
                std::cout << "[PILE] capture look n=" << n << " ordinary=" << ord << " edge=" << in.look_edge_ordinary << "+" << in.look_edge_other << " ok_frames=" << pile_look_ok_ << "/" << t_.pile_look_frames << "\n";
            }
            if (pile_look_ok_ >= t_.pile_look_frames) {
                pending_ = Inventory{}; pending_.ordinary = pile_look_n_;
                label_ = "ordinary_supply"; target_id_ = capture_target_id_ = -1;
                std::cout << "[PILE] frame enclosed ordinary=" << pending_.ordinary << " -> carry\n";
                pile_enclosed_ = true;
                enter(PushState::LOWER_FRAME, now, "pile_frame_enclosed");
                break;
            }
            if (pile_look_bad_ >= t_.pile_look_frames) { giveUp("not_exactly_one_ordinary"); break; }
            if (now - pile_look_us_ >= t_.disperse_turn_look_us) { giveUp("timeout"); break; }
            reason_ = "pile_capture_look";
            break;
        }
        default: pileEnd("pile_bad_phase"); break;
        }
        break;
    }
    case PushState::LOST_SEARCH: {
        approach_forward_ = 0;
        // pitchWait above finishes FAR transition before wheels can turn.
        if(step_==0){step_=1;phase_us_=now;turned_=0;heading_seen_=false;}
        if((!in.gripper_closed_observed && !sweep_open_) || !in.heading_valid || !std::isfinite(in.heading_rad)) { // spinning is allowed with the frame closed (confirmed) or deliberately open
            fail("lost_search_feedback_missing");state_=PushState::SAFE_STOP;break;
        }
        if(now-phase_us_>=20000000 || turned_>=2*float(CV_PI)) {
            // A FAR sweep misses cargo right in front of the jaw; stop, tilt to 20deg, sweep again.
            if(!lost_round_ && midPitch()) {lost_round_=1;enter(PushState::LOST_SEARCH,now,"lost_search_mid_pitch");break;}
            if(t_.lost_search_rescan && frame_raised_==0 && trip_.total()==0) {
                // Nothing found in two sweeps: go back to the normal SCAN (far pitch, blind scatter) instead of idling for the rest of the match.
                lost_round_=0;target_id_=-1;confirmations_=0;
                enter(PushState::SCAN,now,"lost_search_rescan");break;
            }
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
        if(frame_raised_==0 && trip_.total()==0 && scatter_ok && clutter_fresh) { // a group in view needs a sweep, not more spinning
            const int g = evalGroup(in.clutter_body, t_.blind_scatter_min, t_.blind_scatter_far_min);
            if(g) {
                if(++scatter_seen_ < t_.sweep_confirm){reason_="lost_group_confirm";break;}
                scatter_seen_=0;
                beginScatter(g == 2, true, g == 2 ? "lost_group_far" : "lost_group_sweep");break;
            }
        }
        scatter_seen_=0;
        if(unstick())break;
        reason_=scan_ok?(lost_round_?"rotating_for_lost_target_mid":"rotating_for_lost_target"):"lost_search_turn_blocked";
        scanSpin();break;
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
        if(sweep_phase_>=4) {
            if(t_.sweep_observation_guard && !clutter_fresh && !candidate(in, now)) {
                sweepChainClose(now); sweep_phase_=0; disperse_small_=false;
                clearTrip(); enter(PushState::MID_REACQUIRE,now,"sweep_observation_lost"); break;
            }
            if(!in.heading_valid || !std::isfinite(in.heading_rad)) {sweepChainClose(now);sweep_phase_=0;disperse_small_=false;reason_="mid_clear_no_heading";break;}
            if(sweepOver || disperseStep()) {
                const bool swung=sweepChainClose(now);
                if(sweep_non_supply_active_) { // swept aside, not grabbed: veto it and search again at the close view
                    disperse_small_=false;blacklist(now);clearTrip();scan_mid_limit_=scan_turn_acc_+2.f*float(CV_PI);
                    disperse_cool_until_us_=now+t_.disperse_cooldown_us;sweep_gap_until_us_=now+t_.sweep_gap_us;
                    if(!sweepOver&&swung&&beginPileFromSweep("pile_chain_non_supply"))break;
                    enter(PushState::SCAN,now,"non_supply_swept");break;
                }
                sweep_phase_=0;disperse_small_=false;rejected_.clear();rejected_bearings_.clear();
                post_disperse_mid_=t_.post_clear_mid_view;target_id_=capture_target_id_=-1;
                enter(PushState::MID_REACQUIRE,now,"mid_neighbour_clear_done");
            }
            break;
        }
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
            if(t_.sweep_non_supply && t_.disperse && sweepTimeOk && targetKind(label_)!=TargetKind::ORDINARY && in.heading_valid && std::isfinite(in.heading_rad)) {
                if(++confirmations_<t_.confirm_frames){reason_="non_supply_sweep_confirm";break;}
                confirmations_=0;sweep_non_supply_active_=true;disperse_small_=false;disperse_sign_=1.f;
                disperse_rush_m_=std::clamp(target_forward_m-t_.sweep_frame_front_m+t_.disperse_depth_m,0.f,t_.disperse_max_m);
                disperse_pass_=0;disperse_passes_=t_.disperse_passes;disperse_swings_=t_.disperse_swings;
                sweep_phase_=4;sweep_start_us_=sweep_t0_us_=now;sweep_travel0_=travel_;sweepChainBegin(now);
                reason_="non_supply_rush";break;
            }
            verdict_=checkCorridor(in.corridor,in.corridor_complete,in.corridor_occlusion_free,label_,first_);
            if(verdict_!=RuleVerdict::OK) {
                if(t_.enable_short_push && clear_attempts_<2 && in.clear_push_safe &&
                   in.corridor_complete && in.corridor.unknown==0 &&
                   (in.corridor.dangerous>0 || in.corridor.injured>0 || in.corridor.core>0)) {
                    enter(PushState::CUE_PREPARE,now,"closed_push_prepare");break;
                }
                const bool neighbour_verdict=verdict_==RuleVerdict::TOO_MANY_SUPPLIES||verdict_==RuleVerdict::DANGEROUS||verdict_==RuleVerdict::UNKNOWN_OBJECT||
                    verdict_==RuleVerdict::CORE_BEFORE_FIRST||verdict_==RuleVerdict::INJURED_BEFORE_FIRST||verdict_==RuleVerdict::INJURED_NOT_ALONE||verdict_==RuleVerdict::OCCLUDED;
                if(t_.mid_neighbour_clear && t_.disperse && sweepLedgerOk && neighbour_verdict && mid_clear_n_<t_.mid_neighbour_clear_max && in.heading_valid && std::isfinite(in.heading_rad)) {
                    const cv::Point2f tp(-in.distance_m*std::sin(in.heading_error),target_forward_m);
                    float sum_x=0;
                    if(in.clutter_ts_us && now>=in.clutter_ts_us && now-in.clutter_ts_us<=t_.sweep_fresh_us)
                        for(const auto &u:in.clutter_body) {
                            if(!std::isfinite(u.x)||!std::isfinite(u.y)) continue;
                            const float r=float(cv::norm(u-tp));
                            if(r<t_.sweep_min_sep_m||r>t_.disperse_radius_m) continue;
                            sum_x+=u.x-tp.x;
                        }
                    ++mid_clear_n_;disperse_small_=true;disperse_sign_=sum_x>0?-1.f:1.f;
                    disperse_rush_m_=std::clamp(target_forward_m-t_.sweep_frame_front_m-t_.mid_neighbour_clear_gap_m,0.f,t_.mid_neighbour_clear_max_rush_m);
                    disperse_pass_=0;disperse_passes_=1;disperse_swings_=t_.mid_neighbour_clear_swings;
                    sweep_phase_=4;sweep_start_us_=sweep_t0_us_=now;sweep_travel0_=travel_;confirmations_=0;sweepChainBegin(now);
                    reason_="mid_neighbour_clear";break;
                }
                blacklist(now);target_id_=capture_target_id_=-1;
                if(++near_failures_>=2){fail("mid_open_corridor_rejected");state_=PushState::SAFE_STOP;}
                else enter(PushState::MID_REACQUIRE,now,"mid_open_corridor_reselect");
                break;
            }
            if(confirmations_==0 || seen_!=in.corridor){seen_=in.corridor;confirmations_=0;}
            if(++confirmations_<t_.confirm_frames){reason_="mid_open_corridor_confirm";break;}
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
            if (t_.sweep_observation_guard && !clutter_fresh && !candidate(in, now) && !(disperse_turn_phase_ && sweep_phase_ == 5)) {
                sweepChainClose(now); sweep_phase_ = 0;
                clearTrip(); enter(PushState::SCAN, now, "sweep_observation_lost"); break;
            }
            // In-place sweep of an on-end block beside the target: turn out by IMU angle, then back. Tracking may drop meanwhile.
            // The base turns at roughly a quarter of the command, so sweep_wz bypasses max_wz and the sweep time is not charged to the approach budget.
            const auto sweepDrive = [&](float wz) { drive(0, wz); if (motion.wz_rps != 0) motion.wz_rps = std::copysign(t_.sweep_wz, wz); };
            const auto sweepEnd = [&](const char *why) { sweepChainClose(now); phase_us_ += now - sweep_t0_us_; sweep_phase_ = 0; ++sweep_attempts_; sweep_seen_ = 0; if (t_.sweep_observation_guard) sweep_gap_until_us_ = now + t_.sweep_gap_us; reason_ = why; };
            if (!in.heading_valid || !std::isfinite(in.heading_rad)) { sweepEnd("sweep_upright_no_heading"); break; }
            if (sweep_phase_ >= 4) {
                const bool cut = sweepOver;
                if (cut || disperseStep()) {
                    const bool swung = sweepChainClose(now);
                    if (sweep_non_supply_active_) { // swept aside, not grabbed: veto it and search again at the close view
                        blacklist(now); clearTrip(); scan_mid_limit_ = scan_turn_acc_ + 2.f * float(CV_PI);
                        disperse_cool_until_us_ = now + t_.disperse_cooldown_us; sweep_gap_until_us_ = now + t_.sweep_gap_us;
                        if (!cut && swung && beginPileFromSweep("pile_chain_non_supply")) break;
                        enter(PushState::SCAN, now, "non_supply_swept"); break;
                    }
                    phase_us_ += now - sweep_t0_us_; sweep_phase_ = 0; sweep_seen_ = 0; reason_ = cut ? "sweep_budget_exhausted" : "disperse_done";
                    disperse_cool_until_us_ = now + t_.disperse_cooldown_us; sweep_gap_until_us_ = now + t_.sweep_gap_us;
                    rejected_.clear(); rejected_bearings_.clear(); // the pile has moved: earlier vetoes no longer apply
                    post_disperse_mid_ = t_.post_clear_mid_view; // if the target is lost after the sweep, search at 20deg first (cargo is close)
                    if (!cut && swung) beginPileFromSweep("pile_chain_disperse");
                }
                break;
            }
            if (sweep_phase_ < 4 && sweepOver) { sweepEnd("sweep_budget_exhausted"); break; }
            if (!sweepFrameReady()) break;
            if (sweep_phase_ == 3) {
                if (travel_ - sweep_travel0_ >= sweep_creep_m_ * motionRatio() || now - sweep_start_us_ > t_.sweep_creep_budget_us) {
                    sweep_phase_ = 1; sweep_yaw0_ = in.heading_rad; sweep_start_us_ = now; sweep_chain_swung_ = true;
                } else { reason_ = "sweep_creep"; drive(t_.sweep_creep_speed, 0); break; }
            }
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
            // 跟踪器对同一货物重新分配ID：同标签、方位与距离接近上次跟踪值的合法候选，直接改绑而不是进入丢失搜索。
            if(approach_last_id_==target_id_ && target_id_>=0 && in.target_valid && in.target_id!=target_id_ && in.label==label_ &&
               candidate(in,now) && std::abs(approachHeadingShift(in))<=.3f &&
               std::abs(in.distance_m-approach_last_distance_)<=.35f) {
                target_id_=capture_target_id_=in.target_id;misses_=0;approach_last_id_=target_id_;
                reason_="approach_id_rebound_same_object";break;
            }
            // 近处丢失多半是被车框挡住：原地等跟踪器给出新ID（选择器优先锁消失点附近的目标），不立即进入丢失搜索。
            if(t_.occlusion_hold_us>0 && approach_last_id_==target_id_ && target_id_>=0 && approach_last_us_>0 && now>=approach_last_us_ &&
               now-approach_last_us_<=t_.occlusion_hold_us && approach_last_distance_<=t_.occlusion_near_m) {
                misses_=0;reason_="target_occluded_wait";break;
            }
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
        approach_last_id_ = target_id_; approach_last_heading_ = in.heading_error; approach_last_distance_ = in.distance_m; approach_last_us_ = now;
        approach_last_world_valid_ = in.heading_valid && std::isfinite(in.heading_rad) && std::isfinite(in.heading_error);
        if (approach_last_world_valid_) approach_last_world_yaw_ = wrapAngle(in.heading_rad + in.heading_error);
        if (elapsed > t_.approach_budget_us) { blacklist(now); clearTrip(); enter(PushState::SCAN, now, "approach_timeout"); break; }
        // Tilt down to TRACK when close (hysteresis keeps it from toggling), stopped while it moves.
        if (in.distance_m <= t_.track_near_m) track_close_ = true;
        else if (in.distance_m > t_.track_near_m + .1f) track_close_ = false;
        // 目标贴近（像素接近画面底边）时分级下压相机继续跟踪：只下压不回抬，停车等待到位。
        if (midPitch() && !(t_.demo_capture_motion && approach_pitch_ != t_.far_pitch_cdeg && in.distance_m <= t_.rush_start_m && std::abs(in.heading_error) <= t_.rush_heading_rad)) {
            const int16_t before = approach_pitch_;
            if (in.distance_m <= t_.intermediate_to_near_m) approach_pitch_ = t_.track_pitch_cdeg;
            else if (in.distance_m <= t_.track_near_m && approach_pitch_ == t_.far_pitch_cdeg) approach_pitch_ = t_.intermediate_pitch_cdeg;
            if (approach_pitch_ != before) {approach_rebind_ = true; rebind_since_us_ = now;}
        }
        if (desiredPitch() != pitch_cmd_) { reason_="camera_pitch_wait"; break; }
        reason_ = "approach_target";
        // Many blocks in the corridor ahead of the frame mouth: carry the pile away and split the last layer instead of poking at it.
        if (t_.pile_clear && trip_.total() == 0 && frame_raised_ == 0 && now >= pile_cool_until_us_ && pile_runs_ < t_.pile_max_runs && scan_ok &&
            in.heading_valid && std::isfinite(in.heading_rad) && in.corridor_complete && in.corridor.total() >= t_.pile_corridor_min &&
            in.distance_m <= t_.pile_trigger_max_m) {
            if (++pile_seen_ < t_.pile_confirm) { reason_ = "pile_confirm"; break; }
            pile_seen_ = 0;
            beginPile("pile_many_in_frame");
            break;
        }
        pile_seen_ = 0;
        if (in.distance_m <= t_.rush_start_m) {
            if (std::abs(in.heading_error) <= t_.rush_heading_rad) {
                const bool non_supply = t_.sweep_non_supply && t_.disperse && targetKind(label_) != TargetKind::ORDINARY;
                if (t_.disperse && (non_supply ? sweepTimeOk : (now >= disperse_cool_until_us_ && sweepLedgerOk)) && in.heading_valid && std::isfinite(in.heading_rad) && std::isfinite(in.distance_m)) {
                    // Count everything else around the target (any class); a dense pile is scattered, not picked at.
                    const bool use_clutter = in.clutter_ts_us && now >= in.clutter_ts_us && now - in.clutter_ts_us <= t_.sweep_fresh_us;
                    const auto &pts = use_clutter ? in.clutter_body : in.upright_body;
                    const uint64_t pts_ts = use_clutter ? in.clutter_ts_us : in.upright_ts_us;
                    const cv::Point2f tp(-in.distance_m * std::sin(in.heading_error), target_forward_m);
                    int near_n = 0; float sum_x = 0;
                    if (pts_ts && now >= pts_ts && now - pts_ts <= t_.sweep_fresh_us)
                        for (const auto &u : pts) {
                            if (!std::isfinite(u.x) || !std::isfinite(u.y)) continue;
                            const float r = float(cv::norm(u - tp));
                            if (r < t_.sweep_min_sep_m || r > t_.disperse_radius_m) continue;
                            ++near_n; sum_x += u.x - tp.x;
                        }
                    if (non_supply || near_n >= t_.disperse_dense_min + sweep_esc) {
                        if (++sweep_seen_ < t_.sweep_confirm) { reason_ = non_supply ? "non_supply_sweep_confirm" : "disperse_confirm"; break; }
                        sweep_seen_ = 0; sweep_non_supply_active_ = non_supply;
                        disperse_sign_ = sum_x > 0 ? -1.f : 1.f; // first swing toward the heavier side
                        disperse_rush_m_ = std::clamp(target_forward_m - t_.sweep_frame_front_m + t_.disperse_depth_m, 0.f, t_.disperse_max_m);
                        disperse_pass_ = 0; disperse_passes_ = t_.disperse_passes; disperse_swings_ = t_.disperse_swings;
                        sweep_phase_ = 4; sweep_start_us_ = sweep_t0_us_ = now; sweep_travel0_ = travel_; sweepChainBegin(now);
                        reason_ = "disperse_rush"; break;
                    }
                }
                float lat = 0, nearest_y = 0;
                const bool upright_hit = t_.sweep_upright && uprightInCorridor(in, now, lat, &nearest_y);
                const bool neighbour_hit = !upright_hit && neighbourInCorridor(in, now, lat, &nearest_y);
                if ((upright_hit || neighbour_hit) && sweepLedgerOk && sweep_attempts_ < t_.sweep_max_attempts && in.heading_valid && std::isfinite(in.heading_rad)) {
                    if (++sweep_seen_ < t_.sweep_confirm) { reason_ = neighbour_hit ? "sweep_neighbour_confirm" : "sweep_upright_confirm"; break; }
                    sweep_yaw0_ = in.heading_rad; sweep_sign_ = lat > 0 ? -1.f : 1.f; sweep_start_us_ = sweep_t0_us_ = now; sweep_seen_ = 0; sweepChainBegin(now);
                    sweep_creep_m_ = t_.sweep_creep ? std::clamp(nearest_y - (t_.sweep_frame_front_m + t_.sweep_creep_gap_m), 0.f, t_.sweep_creep_max_m) : 0.f;
                    if (sweep_creep_m_ >= .02f) { sweep_phase_ = 3; sweep_travel0_ = travel_; reason_ = "sweep_creep"; break; }
                    sweep_phase_ = 1; sweep_chain_swung_ = true;
                    reason_ = neighbour_hit ? "sweep_neighbour_out" : "sweep_upright_out";
                    if ((t_.sweep_frame_open || t_.frame_open_default) && !sweep_open_disabled_) break; // open the frame first; the next tick turns
                    drive(0, sweep_sign_ * t_.sweep_wz); if (motion.wz_rps != 0) motion.wz_rps = sweep_sign_ * t_.sweep_wz; break;
                }
                if (sweep_seen_ > 0) --sweep_seen_;
                if (t_.corridor_precheck && !t_.assume_all_safe && precheck_rejects_ < t_.corridor_precheck_max && in.corridor_complete) {
                    const auto v = checkCorridor(in.corridor, true, in.corridor_occlusion_free, label_, first_);
                    const bool fatal = v == RuleVerdict::DANGEROUS || v == RuleVerdict::UNKNOWN_OBJECT || v == RuleVerdict::CORE_BEFORE_FIRST ||
                        v == RuleVerdict::INJURED_BEFORE_FIRST || v == RuleVerdict::INJURED_NOT_ALONE || v == RuleVerdict::TOO_MANY_SUPPLIES;
                    if (fatal) {
                        if (++precheck_seen_ < t_.corridor_precheck_confirm) { reason_ = "corridor_precheck_confirm"; break; }
                        precheck_seen_ = 0; ++precheck_rejects_; verdict_ = v;
                        blacklist(now); clearTrip();
                        const std::string why = std::string("corridor_precheck_") + verdictName(v);
                        enter(PushState::SCAN, now, why.c_str()); break;
                    }
                    precheck_seen_ = 0;
                }
                // PREPARE tilts the camera to the tracking pitch; a block farther than this is cut off by the image top (box_at_image_edge), so close in first.
                if (!t_.assume_all_safe && !t_.demo_capture_motion && midPitch() && in.distance_m > t_.intermediate_to_near_m) {
                    if (now - prep_last_us_ > 500000 || in.distance_m < prep_ref_m_ - t_.approach_stall_progress_m) { prep_ref_m_ = in.distance_m; prep_ref_us_ = now; }
                    prep_last_us_ = now;
                    if (now - prep_ref_us_ >= t_.approach_stall_us && in.distance_m <= t_.approach_stall_accept_m) {
                        prep_last_us_ = 0;
                        enter(PushState::PREPARE, now, "approach_range_stalled");
                        commandFrame(1, now);
                        break;
                    }
                    reason_ = "approach_to_prepare_range"; steer(t_.approach_speed, in.heading_error); break;
                }
                prep_last_us_ = 0;
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
            if (rushLogic() && in.target_valid && in.target_id != target_id_ && in.label == label_ && ((candidate(in, now) && near_last) || sameObject(in, now))) {
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
            const bool fatal = verdict == RuleVerdict::DANGEROUS || verdict == RuleVerdict::CORE_BEFORE_FIRST ||
                verdict == RuleVerdict::INJURED_BEFORE_FIRST || verdict == RuleVerdict::INJURED_NOT_ALONE;
            if (fatal || ++corridor_retries_ > t_.corridor_retry_max) { blacklist(now); corridor_retries_ = 0; }
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
        // Restoring demo motion does not restore its assumed inventory. Even on a blind
        // completion/budget tick, an observed illegal neighbour must veto enclosure.
        if (t_.demo_capture_motion && !t_.assume_all_safe && in.corridor.total() > 0) {
            const auto v = checkTrip(in.corridor, first_);
            if (v != RuleVerdict::OK) {
                verdict_ = v; blacklist(now);
                const std::string why = std::string("rush_corridor_") + verdictName(v);
                enter(PushState::CAPTURE_FAIL, now, why.c_str()); break;
            }
        }
        const auto finishRush = [&](const char *why) {
            if (t_.demo_capture_motion) enter(PushState::LOWER_FRAME, now, why);
            else preLower(why);
        };
        const uint64_t stall_prev_us = rush_stall_last_us_; rush_stall_last_us_ = 0; // any tick that does not reach the sample point breaks continuity
        if (rush_stall_phase_) { // stalled: stationary near-view re-check; the rush bound or a lost target must not credit enclosure here
            reason_ = "rush_stall_reobserve";
            const bool seen = in.camera_pitch_stable && in.hold_observable;
            if (seen) ++rush_stall_obs_frames_;
            const bool in_frame = seen && in.captured && in.held_complete && in.held.count(targetKind(label_)) >= 1 &&
                checkTrip(in.held, first_) == RuleVerdict::OK;
            if (in_frame) { preLower("rush_stall_in_frame_evidence"); break; }
            const uint64_t waited = now - rush_stall_obs_us_;
            if ((rush_stall_obs_frames_ >= t_.rush_stall_reobserve_frames && waited >= t_.rush_stall_reobserve_us) || waited >= t_.rush_stall_reobserve_max_us) {
                blacklist(now); enter(PushState::CAPTURE_FAIL, now, "rush_no_progress");
            }
            break;
        }
        const auto reobserveRush = [&](const char *why) {
            rush_stall_phase_ = true; rush_stall_obs_us_ = now; rush_stall_obs_frames_ = 0;
            motion.vx_mps = motion.wz_rps = 0; reason_ = why;
        };
        if (travel_ >= travel_limit_ || elapsed > t_.rush_budget_us) {
            if (t_.precise_rush) { reobserveRush("rush_bound_reobserve"); break; }
            if (rushLogic()) { finishRush("assumed_enclosure_rush_bound"); break; }
            enter(PushState::CAPTURE_FAIL, now, "rush_overrun"); break;
        }
        if (!tracking(in)) {
            if (t_.precise_rush) { reobserveRush("rush_target_lost_reobserve"); break; }
            if (rushLogic()) {
                // TEMP_ASSUMPTION: the target leaves the 5deg view below ~0.31 m; finish the last
                // estimated distance straight, then lower the frame.
                if (travel_ >= assume_rush_end_) { finishRush("assumed_enclosure_target_below_view"); break; }
                reason_ = "assumed_blind_rush"; drive(t_.rush_speed, t_.demo_capture_motion ? 0.f : yawHoldWz()); break;
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
        if (rushLogic() && !t_.precise_rush) {
            // Demo capture/replay follows the last visible remaining distance; precision mode retains its initial hard bound.
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
        if (t_.precise_rush && std::abs(in.heading_error) > t_.rush_align_deadband_rad) {
            reason_ = "rush_align_at_rest";
            drive(0, t_.heading_gain * in.heading_error);
            // Preserve clearance vetoes, but bypass drive's coarse minimum turn command.
            if (motion.wz_rps != 0) motion.wz_rps = std::clamp(t_.heading_gain * in.heading_error, -t_.rush_align_wz, t_.rush_align_wz);
            break;
        }
        if (target_forward_m <= (rushLogic() ? t_.assume_rush_trigger_m : t_.grasp_trigger_y_m)) { enter(PushState::LOWER_FRAME, now, "frame_enclosure_position"); break; }
        if (t_.precise_rush) {
            reason_ = target_forward_m <= t_.rush_slow_range_m ? "rush_straight_near" : "rush_straight";
            drive(target_forward_m <= t_.rush_slow_range_m ? std::min(t_.rush_speed, t_.rush_near_speed) : t_.rush_speed, 0);
        } else drive(t_.rush_speed, t_.heading_gain * in.heading_error);
        if (t_.rush_stall_us > 0) {
            const bool pitch_ok = in.camera_pitch_stable && in.camera_pitch_cdeg != kCameraPitchInvalid;
            const bool continuous = stall_prev_us && now >= stall_prev_us && now - stall_prev_us <= t_.rush_stall_gap_us &&
                in.target_id == rush_stall_id_ && rush_stall_pitch_ != kCameraPitchInvalid &&
                std::abs(int(in.camera_pitch_cdeg) - int(rush_stall_pitch_)) <= t_.pitch_tolerance_cdeg;
            if (!pitch_ok || motion.vx_mps <= 0 || !continuous || in.distance_m < rush_stall_ref_m_ - t_.rush_stall_progress_m) {
                // Not a valid no-progress sample (stale/other object/camera moving/no forward command) or real closing: restart the window.
                rush_stall_ref_m_ = in.distance_m; rush_stall_ref_us_ = now; rush_stall_samples_ = 0;
            } else ++rush_stall_samples_;
            rush_stall_id_ = in.target_id; rush_stall_pitch_ = pitch_ok ? in.camera_pitch_cdeg : kCameraPitchInvalid;
            rush_stall_last_us_ = pitch_ok ? now : 0;
            if (rush_stall_samples_ >= t_.rush_stall_min_samples && now - rush_stall_ref_us_ >= t_.rush_stall_us) {
                rush_stall_phase_ = true; rush_stall_obs_us_ = now; rush_stall_obs_frames_ = 0;
                motion.vx_mps = motion.wz_rps = 0; reason_ = "rush_no_progress_stop";
            }
        }
        break;
    }
    case PushState::LOWER_FRAME: {
        commandFrame(0, now);
        const int g = frameWait(in, 0, now);
        if (g < 0) { fail("frame_close_timeout"); state_=PushState::SAFE_STOP; }
        else if (g > 0 && pile_enclosed_ && rushLogic()) { // the PILE look saw exactly one ordinary supply in the frame: skip the multi-view verify and carry
            pile_enclosed_ = false; pile_low_ = false;
            trip_ = pending_; carry_us_ = now; carry_progress_us_ = 0; zone_search_started_us_ = 0;
            enter(PushState::CARRY, now, "pile_capture"); hold_seen_us_ = now; carried_ = 0;
        }
        else if (g > 0) { pile_enclosed_ = false; pile_low_ = false; enter(PushState::VERIFY_CAPTURE, now, "frame_lowered"); }
        break;
    }
    case PushState::VERIFY_CAPTURE: {
        if(t_.require_multi_view) {
            if(rushLogic() && t_.verify_held_contents && in.hold_observable) {
                // 框内证据在多视角等待期间逐帧累计：危险物永不允许入框；伤员仅在已送达普通物资后且单独入框时允许。
                ++vc_obs_frames_;
                if(in.held.count(targetKind(label_))>0 || in.held.unknown>0) ++vc_label_frames_;
                const auto hv=checkTrip(in.held,first_);
                const bool bad=hv==RuleVerdict::DANGEROUS||hv==RuleVerdict::INJURED_BEFORE_FIRST||hv==RuleVerdict::INJURED_NOT_ALONE||
                    hv==RuleVerdict::CORE_BEFORE_FIRST||hv==RuleVerdict::TOO_MANY_SUPPLIES;
                vc_bad_frames_=bad?vc_bad_frames_+1:0;
                if(bad) vc_bad_verdict_=hv;
                // Possible false class (core/injured only): re-check on identity + class stability instead of aborting on two frames.
                const TargetKind bad_kind = hv==RuleVerdict::CORE_BEFORE_FIRST ? TargetKind::CORE :
                    (hv==RuleVerdict::INJURED_BEFORE_FIRST||hv==RuleVerdict::INJURED_NOT_ALONE) ? TargetKind::INJURED : TargetKind::UNKNOWN;
                const PushObservation::HeldObject *bad_obj = nullptr;
                if(bad && bad_kind!=TargetKind::UNKNOWN)
                    for(const auto &o : in.held_objects)
                        if(targetKind(o.label)==bad_kind && (!bad_obj || o.confidence>bad_obj->confidence)) bad_obj=&o;
                const bool recheck = t_.held_recheck && bad_obj && bad_obj->track_id>=0;
                if(recheck) {
                    vc_re_clear_=0;
                    if(!vc_re_active_) { vc_re_active_=true; vc_re_start_us_=now; vc_re_track_=-1; vc_re_frames_=vc_re_strong_=0; }
                    if(bad_obj->track_id==vc_re_track_) { ++vc_re_frames_; vc_re_strong_ = bad_obj->confidence>=t_.held_recheck_strong_conf ? vc_re_strong_+1 : 0; }
                    else { vc_re_track_=bad_obj->track_id; vc_re_frames_=1; vc_re_strong_ = bad_obj->confidence>=t_.held_recheck_strong_conf ? 1 : 0; }
                    reason_="held_illegal_recheck";
                    if(vc_re_frames_>=t_.held_recheck_confirm_frames || vc_re_strong_>=t_.held_recheck_strong_frames) {
                        verdict_=hv;reason_=std::string("held_")+verdictName(hv);
                        blacklist(now);enter(PushState::ABORT_DROP,now,"");break;
                    }
                } else if(vc_re_active_ && !bad) {
                    if(++vc_re_clear_>=t_.held_recheck_clear_frames) { vc_re_active_=false; vc_re_track_=-1; vc_re_frames_=vc_re_strong_=0; }
                }
                if(vc_re_active_ && now-vc_re_start_us_>=t_.held_recheck_us) { // neither confirmed nor cleared: not a capture, release and retreat
                    blacklist(now);enter(PushState::CAPTURE_FAIL,now,"held_illegal_unresolved");break;
                }
                if(!recheck && vc_bad_frames_>=2) {
                    verdict_=vc_bad_verdict_;reason_=std::string("held_")+verdictName(vc_bad_verdict_);
                    blacklist(now);enter(PushState::ABORT_DROP,now,"");break;
                }
            }
            const bool verified=in.multi_view_finished && in.multi_view_verdict==1 &&
                checkTrip(in.multi_view_inventory,first_)==RuleVerdict::OK && in.multi_view_inventory==pending_;
            const bool mv_conflict=in.multi_view_finished && in.multi_view_verdict==1 &&
                (checkTrip(in.multi_view_inventory,first_)!=RuleVerdict::OK || in.multi_view_inventory!=pending_);
            if(rushLogic() && !verified && (t_.assume_all_safe || !mv_conflict)) {
                if(vc_re_active_) break; // an unresolved illegal class is never credited as an assumed capture
                // TEMP_ASSUMPTION: an unconfirmed enclosure counts as the batch the frame closed on (main path keeps a measured rule/inventory conflict fatal).
                if(elapsed<=8000000 && !in.multi_view_finished)break;
                if(vc_obs_frames_>0 && vc_label_frames_==0) {
                    blacklist(now);enter(PushState::CAPTURE_FAIL,now,"capture_target_not_in_frame");break;
                }
                Inventory assumed=pending_;if(!assumed.total())assumed.add(label_);
                trip_=assumed;carry_us_=now;carry_progress_us_=0;zone_search_started_us_=0;
                enter(PushState::CARRY,now,"assumed_capture");hold_seen_us_=now;carried_=0;break;
            }
            if(elapsed>8000000 || (in.multi_view_finished && in.multi_view_verdict!=1)) {
                fail("multi_view_unconfirmed");state_=PushState::SAFE_STOP;break;
            }
            if(!in.multi_view_finished)break;
            verdict_=checkTrip(in.multi_view_inventory,first_);
            if(verdict_!=RuleVerdict::OK || in.multi_view_inventory!=pending_){
                reason_=verdict_!=RuleVerdict::OK?std::string("multi_grab_")+verdictName(verdict_):std::string("multi_grab_inventory_mismatch");
                blacklist(now);enter(PushState::ABORT_DROP,now,"");break;
            }
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
            if (!t_.assume_all_safe) { if (evadeEnemy()) break; carrySearch(); break; } // no verified own zone: never release blind
            if (!assume_blind_) {
                if (!zone_search_started_us_) zone_search_started_us_ = now;
                if (now - zone_search_started_us_ < t_.assume_zone_search_us) { reason_ = "assumed_zone_search"; scanSpin(); break; }
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
            if (evadeEnemy()) break;
            reason_="searching_own_zone";
            scanSpin(); break; // rotate with closed gripper; never drive blind toward a guessed zone
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
            const float hy = zone_.valid(now) ? zone_.predicted(now).bodyToZone({0, holdCenter()}).y : 0.f;
            const bool placed = cvis && cargo_fix_ && !cargo_clipped_;
            // Vision cannot place the load (hidden / clipped by the frame) and the frame is already deep: a block there means the load reached the far end, not that it is stuck outside.
            const bool blocked_deep = !cinside && !placed && in.imu_blocked_us >= t_.assume_imu_block_us
                && zone_.valid(now) && hy >= t_.assume_blocked_deep_hold_y_m;
            // In-place alignment turns leave the load static by design: not stuck, and the static timer restarts once the forward push begins.
            const bool aligning = align_pending_;
            if (aligning) cargo_ref_us_ = 0;
            const bool stuck = !cinside && !blocked_deep && !aligning
                && (in.imu_blocked_us >= t_.assume_imu_block_us || (cvis && !cargo_clipped_ && cargo_ref_us_ && now - cargo_ref_us_ >= t_.assume_stuck_us));
            if (cinside) { noteCargo("push_inside", {cargo_rear_x_, cargo_rear_y_}, true); done = true; }
            else if (blocked_deep) { noteCargo("blocked_deep", {cargo_rear_x_, cargo_rear_y_}, cvis); done = true; }
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
                if (done && !cinside && !blocked_deep && !(stuck && !(repush_n_ < t_.assume_repush_max)) && t_.assume_enter_confirm) {
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
                    const bool x_ok = std::abs(cr.x - halfX()) <= t_.assume_cargo_x_tol_m;
                    if (!img.clipped && cr.y >= t_.assume_cargo_verify_in_y_m && x_ok) { ++verify_in_; verify_out_ = 0; }
                    else if (img.clipped || !x_ok || cr.y < t_.assume_cargo_verify_out_y_m) { ++verify_out_; verify_in_ = 0; }
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
            if (!visual_ok && verify_out_ >= t_.assume_verify_out_obs && cargoInZone(in, now)) {
                noteCargo("verify_other_inside", cr, cfix);
                visual_ok = true;
            }
            if (!visual_ok && verify_out_ >= t_.assume_verify_out_obs && repush_n_ < t_.assume_repush_max) {
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
        if(credited==expected && credited>0){near_failures_=0;mid_clear_n_=0;}
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
            sweepChainClose(now); sweep_done_ = 0; sweep_spent_us_ = 0; sweep_gap_until_us_ = 0; // a new trip starts with a fresh per-trip sweep ledger (the match-wide time ledger keeps running)
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
        // A corridor veto holds nothing, so it need not wait long for zone geometry before backing off blind.
        const uint64_t blind_wait = corridor_retreat_ ? std::min<uint64_t>(t_.blind_retreat_wait_us, 200000) : t_.blind_retreat_wait_us;
        if (retreat_mode_ == 0 ? now - phase_us_ >= blind_wait
                               : retreat_mode_ == 1 && now - retreat_zone_us_ >= blind_wait) {
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
std::vector<PushTask::RejectedBearing> PushTask::rejectedBearings(uint64_t now) const {
    std::vector<RejectedBearing> out;
    for (const auto &b : rejected_bearings_) if (b.until_us > now) out.push_back(b);
    return out;
}
std::vector<int> PushTask::rejectedTargets(uint64_t now) const {
    std::vector<int> ids;
    for(const auto& item:rejected_)if(item.second>now)ids.push_back(item.first);
    return ids;
}
} // namespace rescue
