#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace rescue {
const char* PushTask::name(PushState s) {
    switch (s) {
        case PushState::WAIT_START:
            return "WAIT_START";
        case PushState::START_ADVANCE:
            return "START_ADVANCE";
        case PushState::SCAN:
            return "SCAN";
        case PushState::SCATTER:
            return "SCATTER";
        case PushState::PILE:
            return "PILE";
        case PushState::LOST_SEARCH:
            return "LOST_SEARCH";
        case PushState::MID_REACQUIRE:
            return "MID_REACQUIRE";
        case PushState::MID_APPROACH:
            return "MID_APPROACH";
        case PushState::NEAR_REACQUIRE:
            return "NEAR_REACQUIRE";
        case PushState::SELECT_CARGO:
            return "SELECT_CARGO";
        case PushState::CUE_APPROACH:
            return "CUE_APPROACH";
        case PushState::CUE_PREPARE:
            return "CUE_PREPARE";
        case PushState::CLEAR_PILE:
            return "CLEAR_PILE";
        case PushState::CLEAR_PAUSE:
            return "CLEAR_PAUSE";
        case PushState::APPROACH:
            return "APPROACH";
        case PushState::PREPARE:
            return "PREPARE";
        case PushState::RUSH:
            return "RUSH";
        case PushState::LOWER_FRAME:
            return "LOWER_FRAME";
        case PushState::VERIFY_CAPTURE:
            return "VERIFY_CAPTURE";
        case PushState::CARRY:
            return "CARRY";
        case PushState::RAISE_RELEASE:
            return "RAISE_RELEASE";
        case PushState::BACK_OUT:
            return "BACK_OUT";
        case PushState::VERIFY_DELIVERY:
            return "VERIFY_DELIVERY";
        case PushState::TURN_SCAN:
            return "TURN_SCAN";
        case PushState::TURN_AROUND:
            return "TURN_AROUND";
        case PushState::CAPTURE_FAIL:
            return "CAPTURE_FAIL";
        case PushState::LOST_HOLD:
            return "LOST_HOLD";
        case PushState::ABORT_DROP:
            return "ABORT_DROP";
        case PushState::SAFE_STOP:
            return "SAFE_STOP";
    }
    return "UNKNOWN";
}
void PushTask::enterSearch(SearchAction action, uint64_t now, const char* why) {
    switch (action) {
        case SearchAction::Scan:
            enter(PushState::SCAN, now, why);
            break;
        case SearchAction::LostSearch:
            enter(PushState::LOST_SEARCH, now, why);
            break;
        case SearchAction::MidReacquire:
            enter(PushState::MID_REACQUIRE, now, why);
            break;
        case SearchAction::MidApproach:
            enter(PushState::MID_APPROACH, now, why);
            break;
        case SearchAction::NearReacquire:
            enter(PushState::NEAR_REACQUIRE, now, why);
            break;
        case SearchAction::SelectCargo:
            enter(PushState::SELECT_CARGO, now, why);
            break;
        case SearchAction::Approach:
            enter(PushState::APPROACH, now, why);
            break;
        case SearchAction::Prepare:
            enter(PushState::PREPARE, now, why);
            break;
        case SearchAction::TurnAway:
            enter(PushState::TURN_SCAN, now, why);
            break;
        case SearchAction::Recover:
            enter(PushState::LOST_HOLD, now, why);
            break;
    }
}
void PushTask::applyPhaseOutcome(uint64_t now) {
    const auto outcome = phase_outcome_;
    phase_outcome_ = PhaseOutcome::Running;
    switch (outcome) {
        case PhaseOutcome::Running:
            return;
        case PhaseOutcome::CaptureReady: {
            const float limit = actionProgress().travel_limit;
            enter(PushState::RUSH, now, phase_outcome_reason_.c_str());
            actionProgress().travel_limit = limit;
            break;
        }
        case PhaseOutcome::TransportReady:
            enter(PushState::CARRY, now, phase_outcome_reason_.c_str());
            break;
        case PhaseOutcome::SearchReady:
            if (phase_ == TaskPhase::Startup && t_.startup_advance_us &&
                t_.pixel_grasp_enabled && startup_action_ == StartupAction::OpenApproach) {
                // Startup hands off the selected ordinary and open frame at UI140.
                // Continue near-view entry review; do not repeat UI90/UI120 search.
                enter(PushState::APPROACH, now, phase_outcome_reason_.c_str());
                search_phase_.approach_pitch_ = t_.near_pitch_cdeg;
            } else {
                enter(phase_ == TaskPhase::Transport && !recovering_ &&
                          state_ != PushState::TURN_AROUND ? PushState::TURN_SCAN
                                                                     : PushState::SCAN,
                      now,
                      phase_outcome_reason_.c_str());
            }
            break;
    }
}
void PushTask::enter(PushState s, uint64_t now, const char* why) {
    if (s == PushState::CAPTURE_FAIL || s == PushState::ABORT_DROP) {
        invalidateEntryPosition();
        if (s == PushState::ABORT_DROP && search_entry_.target_id >= 0)
            search_entry_.stage = "splitting";
    }
    if (s == PushState::SCAN || s == PushState::WAIT_START)
        search_entry_ = {};
    if (s == PushState::CAPTURE_FAIL || s == PushState::ABORT_DROP)
        recordCaptureFailure(why && *why ? why : reason_);
    if (s == PushState::VERIFY_CAPTURE || state_ == PushState::VERIFY_CAPTURE)
        capture_phase_.capture_verification_.reset();
    transport_phase_.delivery_verification_.resetConfirmations();
    if (s != PushState::APPROACH && s != PushState::MID_APPROACH && s != PushState::PREPARE &&
        s != PushState::RUSH && s != PushState::LOWER_FRAME)
        pixel_alignment_ = {};
    if (t_.pixel_grasp_enabled && (s == PushState::APPROACH || s == PushState::MID_APPROACH) &&
        !search_phase_.pixel_approach_started_us_)
        search_phase_.pixel_approach_started_us_ = now;
    // The deadline belongs only to image approach/reacquisition, not later sweep/carry work.
    if (s != PushState::APPROACH && s != PushState::MID_APPROACH && s != PushState::MID_REACQUIRE &&
        s != PushState::NEAR_REACQUIRE && s != PushState::SELECT_CARGO &&
        s != PushState::LOST_SEARCH)
        search_phase_.pixel_approach_started_us_ = 0;
    if ((s == PushState::CAPTURE_FAIL || s == PushState::LOST_HOLD || s == PushState::ABORT_DROP) &&
        !search_phase_.first_trip_split_.pending)
        search_phase_.first_trip_split_.capture_retry = false;
    // Numeric IDs remain readable in historical logs; these actions were removed.
    if (s == PushState::CUE_APPROACH || s == PushState::CUE_PREPARE ||
        s == PushState::CLEAR_PILE || s == PushState::CLEAR_PAUSE ||
        s == PushState::SCATTER || s == PushState::PILE) {
        fail("removed_search_action");
        state_ = PushState::SAFE_STOP;
        return;
    }
    state_ = s;
    if (s == PushState::WAIT_START) {
        run_status_ =
            stopped_from_ == PushState::WAIT_START ? RunStatus::Waiting : RunStatus::Paused;
    } else if (s == PushState::SAFE_STOP) {
        run_status_ = RunStatus::Fault;
    } else {
        run_status_ = RunStatus::Running;
        recovering_ = false;
        switch (s) {
            case PushState::START_ADVANCE:
                phase_ = TaskPhase::Startup;
                break;
            case PushState::SCAN:
                phase_ = TaskPhase::Search;
                search_phase_.action = SearchAction::Scan;
                break;
            case PushState::LOST_SEARCH:
                phase_ = TaskPhase::Search;
                search_phase_.action = SearchAction::LostSearch;
                break;
            case PushState::MID_REACQUIRE:
                phase_ = TaskPhase::Search;
                search_phase_.action = SearchAction::MidReacquire;
                break;
            case PushState::MID_APPROACH:
                phase_ = TaskPhase::Search;
                search_phase_.action = SearchAction::MidApproach;
                break;
            case PushState::NEAR_REACQUIRE:
                phase_ = TaskPhase::Search;
                search_phase_.action = SearchAction::NearReacquire;
                break;
            case PushState::SELECT_CARGO:
                phase_ = TaskPhase::Search;
                search_phase_.action = SearchAction::SelectCargo;
                break;
            case PushState::APPROACH:
                phase_ = TaskPhase::Search;
                search_phase_.action = SearchAction::Approach;
                break;
            case PushState::PREPARE:
                phase_ = TaskPhase::Search;
                search_phase_.action = SearchAction::Prepare;
                break;
            case PushState::TURN_SCAN:
                phase_ = TaskPhase::Search;
                search_phase_.action = SearchAction::TurnAway;
                break;
            case PushState::RUSH:
                phase_ = TaskPhase::Capture;
                capture_phase_.action = CaptureAction::Advance;
                break;
            case PushState::LOWER_FRAME:
                phase_ = TaskPhase::Capture;
                capture_phase_.action = CaptureAction::LowerFrame;
                break;
            case PushState::VERIFY_CAPTURE:
                phase_ = TaskPhase::Capture;
                capture_phase_.action = CaptureAction::Verify;
                break;
            case PushState::CARRY:
                phase_ = TaskPhase::Transport;
                transport_phase_.action = TransportAction::Carry;
                break;
            case PushState::RAISE_RELEASE:
                phase_ = TaskPhase::Transport;
                transport_phase_.action = TransportAction::Release;
                break;
            case PushState::BACK_OUT:
                phase_ = TaskPhase::Transport;
                transport_phase_.action = TransportAction::BackOut;
                break;
            case PushState::TURN_AROUND:
                phase_ = TaskPhase::Transport;
                transport_phase_.action = TransportAction::TurnAround;
                transport_phase_.jaw_sequence_.stopped_us = 0;
                break;
            case PushState::VERIFY_DELIVERY:
                phase_ = TaskPhase::Transport;
                transport_phase_.action = TransportAction::Verify;
                break;
            case PushState::CAPTURE_FAIL:
            case PushState::LOST_HOLD:
            case PushState::ABORT_DROP:
                recovering_ = true;
                if (s == PushState::CAPTURE_FAIL)
                    phase_ = TaskPhase::Capture;
                if (phase_ == TaskPhase::Startup)
                    phase_ = TaskPhase::Capture;
                if (phase_ == TaskPhase::Search)
                    search_phase_.action = SearchAction::Recover;
                if (phase_ == TaskPhase::Capture)
                    capture_phase_.action = CaptureAction::Recover;
                if (phase_ == TaskPhase::Transport)
                    transport_phase_.action = TransportAction::Recover;
                break;
            default:
                break;
        }
    }

    if (s == PushState::LOWER_FRAME && search_phase_.first_trip_split_.capture_retry) {
        auto& split = search_phase_.first_trip_split_;
        split.swing_clear_frames = 0;
        split.swing_sample_us = 0;
        split.swing_stopped_us = now;
    }
    actionProgress().started_us = now;
    search_phase_.search_.resetScanHeading();
    actionProgress().travel = 0;
    actionProgress().travel_limit = 0;
    search_phase_.sweep_phase_ = 0;
    search_phase_.rush_pulse_phase_ = 0;
    search_phase_.rush_pulse_count_ = 0;
    actionProgress().confirmations = 0;
    actionProgress().misses = 0;
    actionProgress().hold_misses = 0;
    actionProgress().step = 0;
    actionProgress().turned = 0;
    actionProgress().heading_seen = false;
    actionProgress().track_close = false;
    actionProgress().last_range = std::numeric_limits<float>::infinity();
    recovery_.reset();
    actionProgress().blind = false;
    search_phase_.unstick_step_ = 0;
    capture_phase_.prelower_us_ = 0;
    if (s != PushState::SCAN) {
        search_phase_.scatter_idle_since_us_ = 0;
        search_phase_.scatter_seen_ = 0;
        search_phase_.search_.resetScanTracking();
    }
    if (s != PushState::RUSH)
        rush_yaw_ref_set_ = false; // PREPARE (tracked) -> RUSH keeps the last seen target bearing
    if (s == PushState::APPROACH)
        search_phase_.precheck_seen_ = 0;
    if (s == PushState::RUSH) {
        capture_phase_.pixel_forward_us_ = 0;
        capture_phase_.pixel_forward_pitch_ = kCameraPitchInvalid;
        capture_phase_.rush_stall_phase_ = false;
        capture_phase_.rush_stall_id_ = -1;
        capture_phase_.rush_stall_samples_ = 0;
        capture_phase_.rush_stall_last_us_ = 0;
    }
    // 有20°中间角时，APPROACH沿用当前相机角（远处不提前下压），由接近距离分级切换；否则保持直接用TRACK角。
    if (s == PushState::APPROACH)
        search_phase_.approach_pitch_ =
            (t_.pixel_grasp_enabled || midPitch()) ? pitch_cmd_ : t_.track_pitch_cdeg;
    search_phase_.approach_rebind_ = false;
    search_phase_.approach_last_id_ = -1;
    search_phase_.approach_last_world_valid_ = false;
    if (s != PushState::LOST_HOLD)
        corridor_retreat_ = false;
    if (s == PushState::CARRY) {
        transport_phase_.placing_ = false;
        transport_phase_.jaw_sequence_ = {};
        transport_phase_.approach_pitch_ = t_.far_pitch_cdeg;
        carry_alignment_ = {};
        if (trip_context_.trip_.total() > 0)
            lockDropHalf("carry_start");
    }
    if (why && *why)
        reason_ = why;
}
void PushTask::clearTrip() {
    search_entry_ = {};
    auto& split = search_phase_.first_trip_split_;
    if (!split.pending && !split.active && !split.reacquire && !split.capture_retry) {
        split = {};
        capture_scene_ = {};
        capture_load_ = {};
    }
    pixel_alignment_ = {};
    search_phase_.far_target_recovery_ = {};
    search_phase_.pixel_approach_started_us_ = 0;
    search_phase_.pixel_reselection_ = {};
    capture_phase_.evidence = CaptureEvidence::None;
    transport_phase_.expected = transport_phase_.credited = 0;
    transport_phase_.visual = false;
    capture_phase_.capture_verification_.reset();
    transport_phase_.delivery_verification_.clearTrip();
    transport_phase_.release_retreat_.clearTrip();
    transport_phase_.push_in_.clearTrip();
    vref_used_ = false;
    transport_phase_.carry_approach_.reset();
    search_phase_.sweep_non_supply_active_ = false;
    search_phase_.cue_mode_ = false;
    search_phase_.clear_commanded_us_ = search_phase_.clear_planned_us_ = 0;
    search_phase_.search_cue_id_ = trip_context_.capture_target_id_ = -1;
    transport_phase_.placing_ = false;
    transport_phase_.jaw_sequence_ = {};
    transport_phase_.drop_locked_ = false;
    transport_phase_.drop_centre_zone_ = {};
    transport_phase_.assume_pushed_ = false;
    transport_phase_.repush_n_ = 0;
    search_phase_.sweep_phase_ = search_phase_.sweep_attempts_ = search_phase_.sweep_seen_ = 0;
    transport_phase_.delivery_visual_ = false;
    transport_phase_.cargo_clipped_ = false;
    transport_phase_.cargo_ref_us_ = transport_phase_.cargo_seen_us_ = 0;
    transport_phase_.half_locked_ = false;
    transport_phase_.half_injured_ = false;
    transport_phase_.half_x_ = 0;
    transport_phase_.zone_search_started_us_ = 0;
    trip_context_.approach_forward_ = 0;
    last_target_valid_ = false;
    trip_context_.target_id_ = -1;
    trip_context_.label_.clear();
    trip_context_.trip_ = {};
    trip_context_.pending_ = {};
    trip_context_.seen_ = {};
    trip_context_.verdict_ = RuleVerdict::OK;
}
void PushTask::blacklist(uint64_t now) {
    if (trip_context_.target_id_ < 0)
        return;
    rejected_[trip_context_.target_id_] = now + t_.blacklist_us;
    // Track ids churn across pitch changes: remember where the vetoed block sits in the world as
    // well.
    bool have = false;
    float yaw = 0.f;
    if (rush_yaw_ref_set_) {
        yaw = rush_yaw_ref_;
        have = true;
    } else if (search_phase_.approach_last_world_valid_) {
        yaw = search_phase_.approach_last_world_yaw_;
        have = true;
    }
    if (have && std::isfinite(yaw)) {
        if (rejected_bearings_.size() >= 8)
            rejected_bearings_.erase(rejected_bearings_.begin());
        rejected_bearings_.push_back({yaw,
                                      last_target_valid_
                                          ? std::hypot(last_target_xy_.x, last_target_xy_.y)
                                          : std::max(search_phase_.approach_last_distance_, 0.f),
                                      now + t_.blacklist_us});
    }
}
// Unrecoverable without an operator: stop, keep the gripper as commanded.
void PushTask::fail(const char* why) {
    if (std::string(why) == "single_phase_recovery_required") {
        if (original_reason_.empty())
            recordCaptureFailure(reason_);
        exit_reason_ = why;
    } else {
        recordCaptureFailure(why);
        exit_reason_ = why;
    }
    fault_ = true;
    run_status_ = RunStatus::Fault;
    reason_ = why;
}
void PushTask::commandFrame(uint8_t open, uint64_t now) {
    if (frame_raised_ != open) {
        frame_raised_ = open;
        gripper_us_ = now;
        ++frame_transaction_;
    }
}
static bool searchLikeState(PushState st) {
    switch (st) {
        case PushState::SCAN:
        case PushState::MID_REACQUIRE:
        case PushState::MID_APPROACH:
        case PushState::NEAR_REACQUIRE:
        case PushState::SELECT_CARGO:
        case PushState::APPROACH:
        case PushState::TURN_SCAN:
        case PushState::LOST_SEARCH:
            return true;
        default:
            return false;
    }
}
bool PushTask::defaultOpenState() const {
    return t_.frame_open_default && trip_context_.trip_.total() == 0 && searchLikeState(state_);
}
// Wire-level open overlay: the logical frame (cargo logic) stays as the state machine set it.
void PushTask::syncFrameOverlay(uint64_t now) {
    if (state_ == PushState::SAFE_STOP || fault_)
        return; // a stop holds the frame as commanded
    const bool want = !sweep_open_disabled_ && frame_raised_ == 0 &&
                      defaultOpenState();
    if (want && !sweep_open_) {
        sweep_open_ = true;
        sweep_open_done_ = false;
        sweep_open_us_ = now;
    } else if (!want && sweep_open_) {
        sweep_open_ = false;
        // Only states that run on a closed frame need the wire confirmed closed; states that open
        // or lower the frame themselves must not be held here.
        const bool needs_closed = searchLikeState(state_);
        if (frame_raised_ == 0 && needs_closed) {
            sweep_close_wait_ = true;
            sweep_close_us_ = now;
        }
    }
}
int PushTask::frameWait(const PushObservation& in, uint8_t open, uint64_t now) const {
    // A done flag counts only for the action matching the current target state.
    if (frame_raised_ == open && in.gripper_done && in.gripper_feedback_open == open)
        return 1;
    // Timeout counts from the later of the command and the phase start, so an action
    // commanded earlier still gets a full window when a new phase depends on it.
    return frame_raised_ == open &&
                   now - std::max(gripper_us_, actionProgress().started_us) > t_.gripper_timeout_us
               ? -1
               : 0;
}
bool PushTask::candidate(const PushObservation& in, uint64_t now) const {
    const bool visual_search_state =
        state_ == PushState::SCAN || state_ == PushState::LOST_SEARCH ||
        state_ == PushState::MID_REACQUIRE || state_ == PushState::NEAR_REACQUIRE ||
        state_ == PushState::SELECT_CARGO;
    // Search only needs a fresh, selectable image detection. Geometry is reserved
    // for approach/capture decisions; a missing ground projection must not suppress discovery.
    if (visual_search_state && in.target_box.width > 0 && in.target_box.height > 0) {
        const auto it = rejected_.find(in.target_id);
        return in.target_valid && in.target_id >= 0 && !in.target_is_search_cue &&
               !in.target_in_zone && targetSelectable(in.label, first_) &&
               (it == rejected_.end() || now >= it->second);
    }
    const bool far_pixel_handoff =
        in.target_center_px_valid && in.pixel_view_handoff_ready &&
        in.camera_pitch_cdeg != kCameraPitchInvalid &&
        std::abs(int(in.camera_pitch_cdeg) - int(t_.far_pitch_cdeg)) <= t_.pitch_tolerance_cdeg;
    if (t_.pixel_grasp_enabled && (in.pixel_grasp_view || far_pixel_handoff)) {
        const auto it = rejected_.find(in.target_id);
        return in.target_valid && in.target_id >= 0 && !in.target_is_search_cue &&
               !in.target_in_zone && targetSelectable(in.label, first_) && in.camera_pitch_stable &&
               (in.pixel_grasp_target_valid || far_pixel_handoff) &&
               std::isfinite(in.pixel_grasp_steering) &&
               (it == rejected_.end() || now >= it->second);
    }
    if (in.target_is_search_cue || !in.target_valid || in.target_id < 0 || !in.geometry_valid ||
        !in.target_region_valid || in.target_in_zone)
        return false;
    if (!targetSelectable(in.label, first_))
        return false;
    if (!std::isfinite(in.distance_m) || in.distance_m < 0 || !std::isfinite(in.heading_error) ||
        std::abs(in.heading_error) > 1.2f)
        return false;
    const auto it = rejected_.find(in.target_id);
    return it == rejected_.end() || now >= it->second;
}
bool PushTask::weakCandidate(const PushObservation& in) const {
    // A confirmed ordinary supply remains a target even when distant/small in the image.
    // Approach and capture keep their own geometry, path and corridor checks.
    return targetKind(in.label) != TargetKind::ORDINARY && t_.weak_target_scatter &&
           in.pixel_mode != "none" && std::isfinite(in.pixel_ratio_smoothed) &&
           in.pixel_ratio_smoothed > 0 && in.pixel_ratio_smoothed < t_.weak_pixel_ratio;
}
float PushTask::approachHeadingShift(const PushObservation& in) const {
    if (search_phase_.approach_last_world_valid_ && in.heading_valid &&
        std::isfinite(in.heading_rad))
        return wrapAngle(in.heading_rad + in.heading_error -
                         search_phase_.approach_last_world_yaw_);
    return wrapAngle(in.heading_error - search_phase_.approach_last_heading_);
}
bool PushTask::sameObject(const PushObservation& in, uint64_t now) const {
    if (!last_target_valid_ || !in.target_valid || in.target_id < 0 ||
        in.target_id == trip_context_.target_id_ || in.label != trip_context_.label_ ||
        in.target_is_search_cue || in.target_in_zone)
        return false;
    if (!std::isfinite(in.distance_m) || in.distance_m <= 0 || !std::isfinite(in.heading_error) ||
        std::abs(in.heading_error) > 1.2f)
        return false;
    const auto it = rejected_.find(in.target_id);
    if (it != rejected_.end() && now < it->second)
        return false;
    const cv::Point2f p(-in.distance_m * std::sin(in.heading_error),
                        in.distance_m * std::cos(in.heading_error));
    return cv::norm(p - last_target_xy_) <= t_.rebind_max_shift_m;
}
bool PushTask::tracking(const PushObservation& in) const {
    if (t_.pixel_grasp_enabled && in.pixel_grasp_view)
        return candidate(in, in.now_us) && in.target_id == trip_context_.target_id_ &&
               in.label == trip_context_.label_;
    return in.target_valid && in.target_id == trip_context_.target_id_ &&
           in.label == trip_context_.label_ && in.geometry_valid && in.target_region_valid &&
           !in.target_in_zone && std::isfinite(in.distance_m) && in.distance_m >= 0 &&
           std::isfinite(in.heading_error) && std::abs(in.heading_error) <= 1.2f;
}
bool PushTask::zoneOk(const PushObservation& in) const {
    return in.zone_identity_verified && in.zone_valid && in.zone_own &&
           in.zone_estimate.trusted(in.now_us);
}
// The held set must still be the verified trip; a changed or partial view is not a hold.
bool PushTask::holding(const PushObservation& in) const {
    return in.hold_observable && in.captured && in.held_complete && in.held == trip_context_.trip_;
}
int PushTask::holdSeen(const PushObservation& in) const {
    if (!in.hold_observable)
        return -1;
    return holding(in) ? 1 : 0;
}
// Preset the current phase looks with; kCameraPitchInvalid keeps the last command (stops and
// drops do not move the camera).
int16_t PushTask::desiredPitch() const {
    // An entry alignment failure holds the last camera angle as well as the
    // cargo; SAFE_STOP must not accidentally request the near-view preset.
    if (state_ == PushState::SAFE_STOP && phase_ == TaskPhase::Transport &&
        transport_phase_.jaw_sequence_.active)
        return pitch_cmd_;
    if (phase_ == TaskPhase::Startup && state_ == PushState::START_ADVANCE) {
        if (!t_.startup_advance_us)
            return kCameraPitchInvalid;
        return startup_pitch_ == kCameraPitchInvalid ? t_.far_pitch_cdeg : startup_pitch_;
    }
    if (state_ == PushState::ABORT_DROP && search_phase_.first_trip_split_.pending)
    {
        using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
        const auto stage = search_phase_.first_trip_split_.recovery_stage;
        return stage == Stage::Turn || stage == Stage::InjuredTurn ||
                       stage == Stage::CornerTurnRight ||
                       stage == Stage::CornerPush || stage == Stage::CornerReturn ||
                       stage == Stage::CornerTurnBack
                   ? t_.track_pitch_cdeg : t_.near_pitch_cdeg;
    }
    if (t_.pixel_grasp_enabled && targetSelectable(trip_context_.label_, first_) &&
        (phase_ != TaskPhase::Transport || !first_) &&
        (state_ == PushState::CAPTURE_FAIL || state_ == PushState::ABORT_DROP ||
         state_ == PushState::LOST_HOLD))
        return t_.near_pitch_cdeg; // Reopen and review before deciding whether to split.
    if (single_phase_initializing_) {
        if (single_phase_ == "transport" && multi_view_pitch_ != kCameraPitchInvalid)
            return multi_view_pitch_;
        return t_.near_pitch_cdeg;
    }
    switch (state_) {
        case PushState::MID_REACQUIRE:
        case PushState::MID_APPROACH:
            return t_.intermediate_pitch_cdeg;
        case PushState::NEAR_REACQUIRE:
        case PushState::SELECT_CARGO:
            return t_.track_pitch_cdeg;
        case PushState::LOST_SEARCH:
            return search_phase_.lost_round_ && midPitch() ? t_.intermediate_pitch_cdeg
                                                           : t_.far_pitch_cdeg;
        case PushState::VERIFY_DELIVERY:
            if (t_.pixel_grasp_enabled && carryLogic())
                return t_.far_pitch_cdeg;
            return assumeView() && !transport_phase_.delivery_verification_.farView()
                       ? t_.assume_view_pitch_cdeg
                       : t_.far_pitch_cdeg;
        case PushState::SCAN:
            return t_.far_pitch_cdeg;
        case PushState::TURN_SCAN:
        case PushState::TURN_AROUND:
            return t_.far_pitch_cdeg;
        case PushState::APPROACH:
            return search_phase_.approach_pitch_ != kCameraPitchInvalid
                       ? search_phase_.approach_pitch_
                       : t_.track_pitch_cdeg;
        // TEMP_ASSUMPTION：开框由MCU完成标志确认，不需要为此下压到40°；货物贴近时在40°画面顶边会被截断（box_at_image_edge），
        // 所以PREPARE/RUSH沿用接近阶段已经到位的角度，不再单独切NEAR/TRACK。
        case PushState::PREPARE:
        case PushState::RUSH:
            if (t_.pixel_grasp_enabled)
                return t_.near_pitch_cdeg;
            if (state_ == PushState::RUSH && capture_phase_.rush_stall_phase_)
                return t_.near_pitch_cdeg;
            if ((t_.assume_all_safe || t_.demo_capture_motion) &&
                search_phase_.approach_pitch_ != kCameraPitchInvalid)
                return search_phase_.approach_pitch_;
            return state_ == PushState::PREPARE && actionProgress().step == 0 ? t_.near_pitch_cdeg
                                                                              : t_.track_pitch_cdeg;
        case PushState::RAISE_RELEASE:
            if (t_.pixel_grasp_enabled && carryLogic())
                return transport_phase_.jaw_sequence_.active ? t_.track_pitch_cdeg : t_.far_pitch_cdeg;
            return assumeView() ? t_.assume_view_pitch_cdeg : t_.track_pitch_cdeg;
        case PushState::LOWER_FRAME:
            return t_.near_pitch_cdeg;
        case PushState::VERIFY_CAPTURE:
            return multi_view_pitch_ == kCameraPitchInvalid ? t_.near_pitch_cdeg
                                                            : multi_view_pitch_;
        case PushState::CARRY:
            if (t_.pushboard_floor_enabled && t_.pixel_grasp_enabled && carryLogic() &&
                transport_phase_.placing_)
                return t_.near_pitch_cdeg;
            // Keep the mapped transport view through legacy IMU advance and release.
            if (t_.pixel_grasp_enabled && carryLogic())
                return transport_phase_.placing_ ? t_.far_pitch_cdeg
                    : transport_phase_.approach_pitch_;
            if (transport_phase_.placing_)
                return assumeView() ? t_.assume_view_pitch_cdeg : t_.track_pitch_cdeg;
            return actionProgress().step ? t_.near_pitch_cdeg : t_.far_pitch_cdeg;
        case PushState::CAPTURE_FAIL:
        case PushState::LOST_HOLD:
        case PushState::ABORT_DROP:
            return t_.far_pitch_cdeg;
        case PushState::BACK_OUT:
            if (t_.pixel_grasp_enabled && carryLogic())
                return pitch_cmd_; // Open-ack retreat keeps the release view; no camera wait.
            return assumeView() ? t_.assume_view_pitch_cdeg
                                : t_.track_pitch_cdeg; // fence and mouth
        default:
            return kCameraPitchInvalid;
    }
}
int PushTask::pitchWait(const PushObservation& in, uint64_t now) {
    const int16_t want = desiredPitch();
    if (want == kCameraPitchInvalid)
        return 1;
    if (want != pitch_cmd_) {
        pitch_cmd_ = want;
        pitch_wait_us_ = now;
    }
    if (in.camera_pitch_cdeg != kCameraPitchInvalid && in.camera_pitch_stable &&
        std::abs(int(in.camera_pitch_cdeg) - int(pitch_cmd_)) <= t_.pitch_tolerance_cdeg) {
        pitch_wait_us_ = now;
        return 1;
    }
    // Counted from the later of the last command/ready frame and the phase start (resume).
    return now - std::max(pitch_wait_us_, actionProgress().started_us) > t_.pitch_timeout_us ? -1
                                                                                             : 0;
}
float PushTask::halfX() const {
    if (transport_phase_.drop_locked_)
        return transport_phase_.drop_centre_zone_.x;
    if (transport_phase_.half_locked_)
        return transport_phase_.half_x_;
    return trip_context_.trip_.injured > 0 ? t_.injured_half_x_m : t_.supply_half_x_m;
}
void PushTask::lockDropHalf(const char* source) {
    if (transport_phase_.half_locked_)
        return;
    transport_phase_.half_locked_ = true;
    transport_phase_.half_injured_ = trip_context_.trip_.injured > 0;
    transport_phase_.half_x_ =
        transport_phase_.half_injured_ ? t_.injured_half_x_m : t_.supply_half_x_m;
    char b[320];
    std::snprintf(b,
                  sizeof b,
                  "[DROP_ZONE] event=lock half=%s target_x=%.3f hold_centre_y=%.3f "
                  "cargo=o%d/c%d/i%d/d%d/u%d rule=%s source=%s layout=left_supply_right_injured\n",
                  transport_phase_.half_injured_ ? "injured" : "supply",
                  transport_phase_.half_x_,
                  holdCenter(),
                  trip_context_.trip_.ordinary,
                  trip_context_.trip_.core,
                  trip_context_.trip_.injured,
                  trip_context_.trip_.dangerous,
                  trip_context_.trip_.unknown,
                  trip_context_.trip_.injured > 0 ? "any_injured->injured" : "no_injured->supply",
                  source);
    dropzone_evt_ += b;
}
void PushTask::noteDropZone(const char* event, const cv::Point2f& hold) {
    char b[240];
    std::snprintf(b,
                  sizeof b,
                  "[DROP_ZONE] event=%s half=%s target_x=%.3f hold_x=%.3f hold_y=%.3f err_x=%.3f\n",
                  event,
                  transport_phase_.half_injured_ ? "injured" : "supply",
                  halfX(),
                  hold.x,
                  hold.y,
                  hold.x - halfX());
    dropzone_evt_ += b;
}
void PushTask::noteCargo(const char* event, const cv::Point2f& rear, bool fix) {
    char b[200];
    std::snprintf(
        b,
        sizeof b,
        "[CARGO_ZONE] event=%s half=%s rear_x=%.3f rear_y=%.3f fix=%d repush=%d in=%d out=%d\n",
        event,
        transport_phase_.half_injured_ ? "injured" : "supply",
        rear.x,
        rear.y,
        fix ? 1 : 0,
        transport_phase_.repush_n_,
        transport_phase_.delivery_verification_.insideCount(),
        transport_phase_.delivery_verification_.outsideCount());
    dropzone_evt_ += b;
}
// Nearest visible cargo around our half, as the ground contact (near edge) in zone coordinates.
void PushTask::trackCargo(const PushObservation& in, uint64_t now) {
    cv::Point2f cb, cr;
    bool cfix = false;
    int ci = -1;
    if (!cargoZone(in, now, cb, cr, cfix, &ci) || in.cargo_ts_us == transport_phase_.cargo_last_ts_)
        return;
    transport_phase_.cargo_last_ts_ = in.cargo_ts_us;
    transport_phase_.cargo_seen_us_ = now;
    transport_phase_.cargo_body_ = cb;
    transport_phase_.cargo_rear_x_ = cr.x;
    transport_phase_.cargo_rear_y_ = cr.y;
    transport_phase_.cargo_fix_ = cfix;
    transport_phase_.cargo_clipped_ = imgAt(in, ci).clipped;
    if (!transport_phase_.cargo_ref_us_ ||
        cv::norm(cb - transport_phase_.cargo_ref_body_) >= t_.assume_stuck_move_m ||
        cv::norm(cr - transport_phase_.cargo_ref_zone_) >= t_.assume_stuck_move_m) {
        transport_phase_.cargo_ref_body_ = cb;
        transport_phase_.cargo_ref_zone_ = cr;
        transport_phase_.cargo_ref_us_ = now;
    }
}

float PushTask::backTarget(uint64_t now) const {
    float b = t_.assume_prepush_back_m + t_.assume_repush_back_m * transport_phase_.repush_n_;
    if (cargoVisible(now))
        b += std::min(t_.assume_lateral_back_max_m,
                      t_.assume_lateral_back_gain *
                          std::max(0.f,
                                   std::abs(transport_phase_.cargo_rear_x_ - halfX()) -
                                       t_.assume_push_dx_deadband_m));
    return b;
}

bool PushTask::cargoZone(const PushObservation& in,
                         uint64_t now,
                         cv::Point2f& body,
                         cv::Point2f& rear,
                         bool& fix,
                         int* idx) const {
    fix = false;
    if (in.cargo_body.empty() || !(now >= in.cargo_ts_us && now - in.cargo_ts_us <= 300000))
        return false;
    ZoneEstimate z;
    if (in.vref_valid && in.vref_zone.valid && in.vref_points >= 3 &&
        in.vref_zone.timestamp_us == in.cargo_ts_us) {
        z = in.vref_zone;
        fix = true;
    } else if (zone_.valid(now))
        z = zone_.predicted(now);
    else
        return false;
    // Stay on the block already being tracked (two adjacent blocks must not swap the reference
    // every frame).
    const bool lock =
        transport_phase_.cargo_seen_us_ && now - transport_phase_.cargo_seen_us_ <= 600000;
    float best = 1e9f, best_lock = .10f;
    bool found = false, locked = false;
    for (size_t i = 0; i < in.cargo_body.size(); ++i) {
        const cv::Point2f& p = in.cargo_body[i];
        const cv::Point2f q = z.bodyToZone(p);
        if (p.y <= 0 || std::abs(q.x - halfX()) > t_.assume_cargo_x_tol_m + .10f || q.y < -.40f ||
            q.y > .50f)
            continue;
        const float d = float(cv::norm(p)),
                    dl = lock ? float(cv::norm(p - transport_phase_.cargo_body_)) : 1e9f;
        if (dl < best_lock) {
            best_lock = dl;
            body = p;
            rear = q;
            found = locked = true;
            if (idx)
                *idx = int(i);
        } else if (!locked && d < best) {
            best = d;
            body = p;
            rear = q;
            found = true;
            if (idx)
                *idx = int(i);
        }
    }
    return found;
}
// Any visible block already past the zone's front edge on our half (fresh visual zone fix only).
bool PushTask::cargoInZone(const PushObservation& in, uint64_t now) const {
    if (in.cargo_body.empty() || !(now >= in.cargo_ts_us && now - in.cargo_ts_us <= 300000))
        return false;
    if (!(in.vref_valid && in.vref_zone.valid && in.vref_points >= 3 &&
          in.vref_zone.timestamp_us == in.cargo_ts_us))
        return false;
    for (size_t i = 0; i < in.cargo_body.size(); ++i) {
        const cv::Point2f q = in.vref_zone.bodyToZone(in.cargo_body[i]);
        if (in.cargo_body[i].y > 0 && !imgAt(in, int(i)).clipped &&
            q.y >= t_.assume_cargo_verify_in_y_m &&
            std::abs(q.x - halfX()) <= t_.assume_cargo_x_tol_m)
            return true;
    }
    return false;
}
float PushTask::holdCenter() const {
    return trip_context_.trip_.injured > 0 ? t_.injured_hold_center_y_m : t_.hold_center_y_m;
}
// After a stop the gripper state decides where to resume: a verified load is never
// silently forgotten, and anything uncertain is released before scanning again.
PushState PushTask::resumeState() const {
    switch (stopped_from_) {
        case PushState::CARRY:
            return transport_phase_.placing_ ? PushState::BACK_OUT : PushState::CARRY;
        case PushState::WAIT_START:
            return PushState::START_ADVANCE;
        case PushState::START_ADVANCE:
            return PushState::START_ADVANCE;
        case PushState::MID_REACQUIRE:
        case PushState::MID_APPROACH:
        case PushState::NEAR_REACQUIRE:
        case PushState::SELECT_CARGO:
        case PushState::LOST_SEARCH:
        case PushState::SCAN:
        case PushState::APPROACH:
        case PushState::TURN_SCAN:
            return PushState::SCAN;
        case PushState::PREPARE:
            return PushState::APPROACH;
        case PushState::CUE_APPROACH:
        case PushState::CUE_PREPARE:
        case PushState::CLEAR_PILE:
        case PushState::CLEAR_PAUSE:
        case PushState::SCATTER:
        case PushState::PILE:
            return PushState::SCAN;
        case PushState::RAISE_RELEASE:
        case PushState::BACK_OUT:
            return PushState::BACK_OUT; // the load is at our zone; finish and verify
        case PushState::TURN_AROUND:
            return PushState::TURN_AROUND;
        case PushState::VERIFY_DELIVERY:
            return PushState::VERIFY_DELIVERY;
        default:
            return PushState::ABORT_DROP;
    }
}

void PushTask::setSinglePhase(const std::string& phase, bool capture_after_first) {
    if (!phase.empty() && phase != "startup" && phase != "search" && phase != "capture" &&
        phase != "transport")
        throw std::invalid_argument("Unknown single phase");
    if (capture_after_first && phase != "capture")
        throw std::invalid_argument("capture-after-first requires capture phase");
    single_phase_ = phase;
    capture_after_first_ = capture_after_first;
    first_ = t_.assume_injured_trip || capture_after_first_;
}

bool PushTask::initializeSinglePhase(TaskTick& tick) {
    const auto& in = tick.in;
    const uint64_t now = tick.now;
    if (now - actionProgress().started_us > 8000000) {
        fail("phase_initial_evidence_timeout");
        return false;
    }
    if (!in.camera_pitch_stable || pitchWait(in, now) <= 0) {
        actionProgress().confirmations = 0;
        capture_scene_.confirmations = 0;
        capture_scene_.stopped_us = now;
        capture_scene_.reobserve_stopped_us = capture_scene_.reobserve_sample_us = 0;
        capture_scene_.reobserve_frames = 0;
        return false;
    }
    if (single_phase_ == "capture") {
        reason_ = "phase_capture_wait_target_corridor";
        if (!in.gripper_done || in.gripper_feedback_open != 1 || !candidate(in, now) ||
            (t_.pixel_grasp_enabled && !in.pixel_grasp_near_view)) {
            actionProgress().confirmations = 0;
            capture_scene_.confirmations = 0;
            capture_scene_.stopped_us = now;
            capture_scene_.reobserve_stopped_us = capture_scene_.reobserve_sample_us = 0;
            capture_scene_.reobserve_frames = 0;
            return false;
        }
        const auto verdict = checkCorridor(
            in.corridor, in.corridor_complete, in.corridor_occlusion_free, in.label, first_);
        const bool pixel_review = t_.pixel_grasp_enabled && targetSelectable(in.label, first_);
        if (pixel_review) {
            // A mixed scene must reach stationary split review before target ranking.
            // This target is only the split reference; it grants no capture permission.
            trip_context_.target_id_ = trip_context_.capture_target_id_ = in.target_id;
            trip_context_.label_ = in.label;
            if (!reviewFirstTripCapture(tick, "initial")) {
                actionProgress().confirmations = 0;
                if (search_phase_.first_trip_split_.pending) {
                    // The next update must execute recovery, not initialize again.
                    single_phase_initializing_ = false;
                    return true;
                }
                return false;
            }
        }
        if (verdict != RuleVerdict::OK && !pixel_review) {
            actionProgress().confirmations = 0;
            return false;
        }
        trip_context_.verdict_ = verdict;
        if (trip_context_.target_id_ != in.target_id || trip_context_.label_ != in.label ||
            trip_context_.seen_ != in.corridor)
            actionProgress().confirmations = 0;
        trip_context_.target_id_ = trip_context_.capture_target_id_ = in.target_id;
        trip_context_.label_ = in.label;
        trip_context_.seen_ = in.corridor;
        if (++actionProgress().confirmations < t_.corridor_confirm_frames)
            return false;
        trip_context_.pending_ = in.corridor;
        if (pixel_review)
            trip_context_.pending_ = capture_scene_.inventory;
        const float forward = in.target_center_valid ? in.target_center_body.y
                                                     : in.distance_m * std::cos(in.heading_error);
        if (!t_.pixel_grasp_enabled && !std::isfinite(forward))
            return false;
        enter(PushState::RUSH, now, "phase_capture_ready");
        if (rushLogic()) {
            assume_rush_end_ =
                std::max(0.f, forward - t_.assume_rush_trigger_m) * t_.assume_motion_ratio;
            actionProgress().travel_limit =
                assume_rush_end_ + t_.rush_extra_m * t_.assume_motion_ratio;
        } else if (!t_.pixel_grasp_enabled)
            actionProgress().travel_limit =
                std::max(0.f, forward - t_.grasp_trigger_y_m) + t_.rush_extra_m;
    } else {
        reason_ = "phase_transport_wait_observed_cargo";
        if (!in.gripper_done || in.gripper_feedback_open != 0) {
            actionProgress().confirmations = 0;
            return false;
        }
        Inventory cargo;
        if (t_.require_multi_view) {
            if (!in.multi_view_finished || in.multi_view_verdict != 1)
                return false;
            cargo = in.multi_view_inventory;
        } else {
            if (!in.hold_observable || !in.captured || !in.held_complete) {
                actionProgress().confirmations = 0;
                return false;
            }
            cargo = in.held;
            if (trip_context_.seen_ != cargo)
                actionProgress().confirmations = 0;
            trip_context_.seen_ = cargo;
            if (++actionProgress().confirmations < t_.capture_frames)
                return false;
        }
        if (checkTrip(cargo, first_) != RuleVerdict::OK)
            return false;
        trip_context_.trip_ = trip_context_.pending_ = cargo;
        trip_context_.label_ = cargo.injured    ? "injured_person"
                               : cargo.ordinary ? "ordinary_supply"
                                                : "core_supply";
        capture_phase_.evidence = CaptureEvidence::Verified;
        enter(PushState::CARRY, now, "phase_transport_ready");
        transport_phase_.carry_us_ = transport_phase_.hold_seen_us_ = now;
        transport_phase_.carried_ = 0;
    }
    single_phase_initializing_ = false;
    return true;
}

PushOutput PushTask::update(const PushObservation& in) {
    if (search_entry_.target_id >= 0 && (!in.run || !in.safety_ok || in.reset ||
        !in.camera_pitch_stable || !in.pixel_grasp_near_view || !tracking(in) ||
        (in.pixel_grasp_sample_us && in.pixel_grasp_sample_us > in.now_us) ||
        !in.pixel_grasp_sample_us || in.now_us - in.pixel_grasp_sample_us > t_.frame_timeout_us)) {
        search_entry_.first_id = -1;
        search_entry_.confirmed = false;
        search_entry_.sample_us = 0;
        search_entry_.stage = "observation_wait";
    }
    multi_view_pitch_ = in.multi_view_pitch;
    TaskTick tick(in);
    auto& motion = tick.motion; // zero velocity by default
    const uint64_t now = in.now_us;
    const bool far_pixel_handoff =
        in.target_center_px_valid &&
        (in.pixel_view_handoff_ready ||
         (phase_ == TaskPhase::Startup && startup_action_ == StartupAction::FarHandoff)) &&
        in.camera_pitch_cdeg != kCameraPitchInvalid &&
        std::abs(int(in.camera_pitch_cdeg) - int(t_.far_pitch_cdeg)) <= t_.pitch_tolerance_cdeg;
    const bool alignment_target_valid =
        (in.pixel_grasp_view && in.pixel_grasp_target_valid) || far_pixel_handoff;
    // Only a brief missing startup detection may retain its identity/rest window.
    // Run, safety, pitch and control-clock changes always invalidate alignment.
    if (phase_ == TaskPhase::Startup &&
        (!in.run || !in.safety_ok || !in.camera_pitch_stable ||
         (last_us_ && (now <= last_us_ || now - last_us_ > t_.frame_timeout_us)))) {
        startup_alignment_since_us_ = now;
        startup_pitch_turn_.heading_valid = false;
        startup_pitch_turn_.stopped_us = 0;
        auto& turn = startup_pitch_turn_;
        if (turn.leg == StartupPitchTurn::Leg::Reverse) {
            turn.reverse_completed_m -= turn.reverse_imu.distance();
            turn.leg = StartupPitchTurn::Leg::ReverseCalibrate;
        }
        if (turn.leg == StartupPitchTurn::Leg::ReverseCalibrate)
            turn.reverse_imu.reset();
    }
    const bool rush_episode = t_.pixel_grasp_enabled && !recovering_ &&
        state_ == PushState::RUSH && !search_entry_.handoff;
    const bool rush_gap = rush_episode && in.pixel_selection_managed && !in.target_valid &&
        in.pixel_released_id < 0 && pixel_alignment_.last_sample_us &&
        now >= pixel_alignment_.last_sample_us &&
        now - pixel_alignment_.last_sample_us <= 200000;
    const bool rush_rebind = rush_episode && in.pixel_selection_managed &&
        trip_context_.target_id_ >= 0 && in.pixel_rebound_from == trip_context_.target_id_ &&
        in.target_valid && in.label == trip_context_.label_ && candidate(in, now);
    if (rush_rebind && pixel_alignment_.target_id == trip_context_.target_id_)
        pixel_alignment_.target_id = in.target_id;
    // Adopt confirmed same-object identity before the generic alignment invalidation.
    // The startup phase updates the trip lock later; an ID-only change must not
    // discard the current correction/rest window in between.
    const int startup_locked_id = startupSightTargetId();
    if (phase_ == TaskPhase::Startup && startup_locked_id >= 0 &&
        in.startup_rebound_from == startup_locked_id && in.target_id >= 0 &&
        in.target_valid && in.target_center_px_valid && in.label == trip_context_.label_ &&
        in.camera_pitch_stable && in.camera_pitch_cdeg == pixel_alignment_.pitch &&
        in.pixel_grasp_sample_us && in.pixel_grasp_sample_us <= now &&
        now - in.pixel_grasp_sample_us <= t_.frame_timeout_us &&
        pixel_alignment_.target_id == startup_locked_id)
        pixel_alignment_.target_id = in.target_id;
    const bool startup_gap = phase_ == TaskPhase::Startup && !in.target_valid &&
        startupSightTargetId() >= 0 &&
        pixel_alignment_.target_id == startupSightTargetId() &&
        in.camera_pitch_cdeg == pixel_alignment_.pitch && pixel_alignment_.last_sample_us &&
        now >= pixel_alignment_.last_sample_us &&
        now - pixel_alignment_.last_sample_us <= 150000;
    if (!in.run || !in.safety_ok || in.reset || !in.camera_pitch_stable ||
         (!(startup_gap || rush_gap) && (!alignment_target_valid || !in.target_valid || !in.pixel_grasp_sample_us ||
         in.pixel_grasp_sample_us > now ||
         (in.pixel_grasp_sample_us <= now &&
          now - in.pixel_grasp_sample_us > t_.frame_timeout_us) ||
         !std::isfinite(in.pixel_grasp_error_px) ||
         in.target_id != pixel_alignment_.target_id || in.label != pixel_alignment_.label ||
         in.pixel_grasp_sample_us < pixel_alignment_.last_sample_us)) ||
         (last_us_ && (now <= last_us_ || now - last_us_ > t_.frame_timeout_us)) ||
         in.camera_pitch_cdeg != pixel_alignment_.pitch) {
        pixel_alignment_ = {};
        pixel_alignment_.target_id = in.target_id;
        pixel_alignment_.label = in.label;
        pixel_alignment_.pitch = in.camera_pitch_cdeg;
    }

    if (!in.run || !in.safety_ok || in.reset || !in.camera_pitch_stable ||
        (last_us_ && (now <= last_us_ || now - last_us_ > t_.frame_timeout_us))) {
        carry_alignment_ = {};
        transport_phase_.jaw_sequence_.stopped_us = 0;

    }

    // Recovery confirmations belong to one uninterrupted far-view approach.
    if (!in.run || !in.safety_ok || in.reset || !in.camera_pitch_stable ||
        phase_ != TaskPhase::Search || search_phase_.action != SearchAction::Approach ||
        in.camera_pitch_cdeg == kCameraPitchInvalid ||
        std::abs(int(in.camera_pitch_cdeg) - int(t_.far_pitch_cdeg)) > t_.pitch_tolerance_cdeg ||
        in.pixel_grasp_view || in.pixel_view_handoff_ready ||
        (last_us_ && (now <= last_us_ || now - last_us_ > t_.frame_timeout_us)))
        search_phase_.far_target_recovery_ = {};

    const auto result = [&] {
        if (!phase_waiting_)
            syncFrameOverlay(now);
        motion.gripper_offset =
            frameAngle((frame_raised_ || sweep_open_) ? FrameAction::Open : FrameAction::Close);
        // The MCU never acknowledges an action that repeats the state it is already in, so a
        // logical change hidden by the open overlay must not look like a new action.
        const int wire = (frame_raised_ || sweep_open_) ? 1 : 0;
        if (wire_last_ < 0) {
            wire_last_ = wire;
            wire_txn_ = frame_transaction_;
        } else if (wire != wire_last_) {
            wire_last_ = wire;
            ++wire_txn_;
        }
        motion.frame_transaction = wire_txn_; // stops hold the gripper, never toggle it
        const int16_t want = phase_waiting_
                                 ? pitch_cmd_
                                 : desiredPitch(); // a phase change commands its preset at once
        if (want != kCameraPitchInvalid && want != pitch_cmd_) {
            pitch_cmd_ = want;
            pitch_wait_us_ = now;
        }
        motion.camera_pitch_cdeg = pitch_cmd_;
        last_vx_ = motion.vx_mps;
        last_wz_ = motion.wz_rps;
        PushOutput out;
        out.state = state_;
        out.phase = phase_;
        out.phase_waiting = phase_waiting_;
        out.single_phase_done = single_phase_done_;
        out.next_phase = phase_outcome_ == PhaseOutcome::CaptureReady     ? TaskPhase::Capture
                         : phase_outcome_ == PhaseOutcome::TransportReady ? TaskPhase::Transport
                         : phase_outcome_ == PhaseOutcome::SearchReady    ? TaskPhase::Search
                                                                          : phase_;
        out.run_status = run_status_;
        out.recovering = recovering_;
        out.search_action = searchStatusAction(search_phase_.action);
        out.capture_action = capture_phase_.action;
        out.transport_action = transport_phase_.action;
        out.capture_evidence = capture_phase_.evidence;
        out.delivery_expected = transport_phase_.expected;
        out.delivery_credited = transport_phase_.credited;
        out.delivery_visual = transport_phase_.visual;
        out.capture_observation_requested = run_status_ == RunStatus::Running && !recovering_ &&
                                            phase_ == TaskPhase::Capture &&
                                            capture_phase_.action == CaptureAction::Verify;
        out.corridor_observation_requested =
            run_status_ == RunStatus::Running && !recovering_ &&
            ((phase_ == TaskPhase::Search && (search_phase_.action == SearchAction::MidApproach ||
                                              search_phase_.action == SearchAction::Approach ||
                                              search_phase_.action == SearchAction::Prepare)) ||
             (phase_ == TaskPhase::Capture && capture_phase_.action == CaptureAction::Advance));
        const bool carrying = run_status_ == RunStatus::Running && !recovering_ &&
                              phase_ == TaskPhase::Transport &&
                              transport_phase_.action == TransportAction::Carry;
        out.pushboard_floor_active = carrying && transport_phase_.placing_ && t_.pushboard_floor_enabled;
        out.navigation_background = carrying && !transport_phase_.placing_;
        out.navigation_release = run_status_ == RunStatus::Running && !recovering_ &&
                                 phase_ == TaskPhase::Transport &&
                                 (transport_phase_.action == TransportAction::Release ||
                                  (carrying && transport_phase_.placing_));
        out.navigation_approach = carrying && !transport_phase_.placing_;
        out.jaw_release_retreat = run_status_ == RunStatus::Running && !recovering_ &&
                                 phase_ == TaskPhase::Transport && state_ == PushState::BACK_OUT &&
                                 t_.pixel_grasp_enabled && carryLogic() &&
                                 motion.vx_mps < 0 && motion.wz_rps == 0;
        out.navigation_requested = out.navigation_release || out.navigation_approach;
        out.navigation_reset =
            run_status_ == RunStatus::Waiting || run_status_ == RunStatus::Paused ||
            (phase_ == TaskPhase::Search && search_phase_.action == SearchAction::Scan);
        out.target_reselecting = run_status_ == RunStatus::Running && !recovering_ &&
                                 phase_ == TaskPhase::Search &&
                                 (search_phase_.action == SearchAction::MidReacquire ||
                                  search_phase_.action == SearchAction::NearReacquire ||
                                  search_phase_.action == SearchAction::SelectCargo);
        out.scan_pitch_return = run_status_ == RunStatus::Running && !recovering_ &&
                                phase_ == TaskPhase::Search &&
                                (search_phase_.action == SearchAction::Scan ||
                                 search_phase_.action == SearchAction::LostSearch);
        out.stationary_pitch_work =
            run_status_ == RunStatus::Running &&
            (recovering_ ||
             (phase_ == TaskPhase::Startup && startup_action_ != StartupAction::Advance &&
              startup_action_ != StartupAction::Observe) ||
             (phase_ == TaskPhase::Capture && capture_phase_.action != CaptureAction::Advance) ||
             (phase_ == TaskPhase::Transport &&
              transport_phase_.action != TransportAction::BackOut &&
              transport_phase_.action != TransportAction::Verify &&
              transport_phase_.action != TransportAction::TurnAround) ||
             (phase_ == TaskPhase::Search &&
              (out.target_reselecting ||
               search_phase_.action == SearchAction::MidApproach ||
               search_phase_.action == SearchAction::Approach ||
               search_phase_.action == SearchAction::Prepare)));
        const bool carry_align = phase_ == TaskPhase::Transport &&
            !transport_phase_.placing_ && carry_alignment_.last_sample_us != 0;
        const auto& displayed_alignment = carry_align ? carry_alignment_ : pixel_alignment_;
        if (displayed_alignment.last_sample_us) {
            switch (displayed_alignment.stage) {
                case PixelAlignment::Stage::Correct:
                    out.pixel_align_state = "correct";
                    break;
                case PixelAlignment::Stage::Brake:
                    out.pixel_align_state = "brake";
                    break;
                case PixelAlignment::Stage::Settle:
                    out.pixel_align_state = "settle";
                    break;
                case PixelAlignment::Stage::Advance:
                    out.pixel_align_state = "advance";
                    break;
            }
        }
        out.pixel_align_stable_frames = displayed_alignment.stable_frames;
        if (carry_align) {
            out.carry_align_active = true;
            out.carry_align_error_px = carry_alignment_.last_error_px;
            out.carry_align_center_x = .5f * in.zone_sight_image_size.width -
                                      carry_alignment_.last_error_px;
            out.carry_align_center_y = carry_alignment_.last_center_y_px;
            out.carry_align_sample_us = carry_alignment_.last_sample_us;
            out.carry_align_tolerance_px = carry_alignment_.tolerance_px;
            out.carry_align_acceptance_px = carry_alignment_.acceptance_px;
            out.carry_align_brake_reason = carry_alignment_.brake_reason;
        }
        out.entry_target_id = search_entry_.target_id;
        out.entry_label = search_entry_.label;
        out.entry_stage = search_entry_.stage;
        out.entry_region_px = in.capture_area_px;
        out.entry_first_id = search_entry_.first_id;
        out.entry_position_ok = entryPositionValid(in);
        out.entry_confirmed = search_entry_.confirmed;
        out.entry_stable_frames = pixel_alignment_.stable_frames;
        const auto& split = search_phase_.first_trip_split_;
        if (t_.pixel_grasp_enabled &&
            (split.pending || split.active || reason_ == "first_trip_pixel_split_advance")) {
            out.split_align_active = true;
            out.split_align_target_id = split.target_id;
            out.split_align_observed_id = in.split_ordinary_box_id;
            out.split_align_center_x = in.split_ordinary_box.x + .5f * in.split_ordinary_box.width;
            out.split_align_target_x = split.pixel_target_x;
            out.split_align_error_px = split.pixel_target_x - out.split_align_center_x;
            out.split_align_stage = split.align_stage;
            out.split_retreat_reference_id = split.retreat_reference_id;
            out.split_retreat_reference_y = split.retreat_reference_y;
            using Margin = SearchPhaseData::FirstTripSplit::MarginStage;
            out.split_margin_stage = split.margin_stage == Margin::Advancing ? "advance"
                : split.margin_stage == Margin::Brake ? "brake"
                : split.margin_stage == Margin::Complete ? "complete" : "idle";
            out.split_margin_used_us = split.margin_used_us;
            out.split_margin_budget_us = split.upper_corner ? 0 : t_.split_margin_budget_us;
            out.split_margin_complete = split.margin_stage == Margin::Complete;
            out.split_imu_ready = split.corner_imu.ready();
            out.split_imu_distance_m = split.corner_imu.distance();
            out.split_imu_velocity_mps = split.corner_imu.velocity();
        }
        if (single_phase_initializing_ && run_status_ == RunStatus::Running) {
            out.capture_observation_requested = single_phase_ == "transport";
            out.corridor_observation_requested = single_phase_ == "capture";
            out.navigation_requested = false;
        }
        // Ordinary startup target gaps/reselection request zero via the sender ramp
        // in every view. Real alignment brakes and revoked clearance remain immediate.
        const bool startup_target_deceleration = reason_ == "startup_locked_target_wait" ||
            reason_ == "startup_target_reselect" || reason_ == "startup_alignment_reselect";
        // A recovered detection must finish the same ramped stop before fresh
        // stationary alignment. Keep this across target release/alignment resets.
        if (phase_ == TaskPhase::Startup && startup_target_deceleration)
            startup_target_decelerating_ = true;
        const float sent_vx = in.sent_velocity_valid ? in.sent_vx_mps : last_vx_;
        const float sent_wz = in.sent_velocity_valid ? in.sent_wz_rps : last_wz_;
        if (phase_ != TaskPhase::Startup || !in.run || !in.safety_ok || in.reset ||
            (std::isfinite(sent_vx) && std::isfinite(sent_wz) &&
             std::abs(sent_vx) <= t_.pixel_align_stopped_vx &&
             std::abs(sent_wz) <= t_.pixel_align_stopped_wz))
            startup_target_decelerating_ = false;
        const bool startup_deceleration_clear =
            phase_ == TaskPhase::Startup && t_.pixel_grasp_enabled &&
            startup_action_ != StartupAction::TurnBeforePitch &&
            startup_action_ != StartupAction::StopBeforePitch &&
            (startup_target_deceleration || startup_target_decelerating_ ||
             (!in.pixel_grasp_near_view && !pixel_alignment_.last_sample_us)) &&
            in.run && in.safety_ok && in.path_safe && in.opponent_zone_clear && in.retreat_safe &&
            (!in.directional_clearance_valid || (in.turn_safe && in.arc_safe));
        out.pixel_selection_paused = run_status_ == RunStatus::Running &&
            phase_ == TaskPhase::Search && search_phase_.pixel_reselection_.observing;
        out.immediate_motion_stop =
            !startup_deceleration_clear && motion.vx_mps == 0 && motion.wz_rps == 0 &&
            (tick.immediate_motion_stop || reason_ == "blind_scatter_target_seen" ||
             out.pixel_selection_paused ||
             search_phase_.first_trip_split_.pending || search_phase_.first_trip_split_.active ||
             search_phase_.first_trip_split_.reacquire ||
             (!tick.controlled_motion_stop && t_.pixel_grasp_enabled &&
              trip_context_.target_id_ >= 0 &&
              ((phase_ == TaskPhase::Search && search_phase_.pixel_approach_started_us_) ||
               (phase_ == TaskPhase::Capture &&
                (capture_phase_.action == CaptureAction::Advance ||
                 capture_phase_.action == CaptureAction::LowerFrame)))));
        out.motion = motion;
        out.split_target_reselecting =
            run_status_ == RunStatus::Running &&
            (search_phase_.first_trip_split_.pending || capture_load_.active ||
             (single_phase_initializing_ && single_phase_ == "capture")) &&
            search_phase_.first_trip_split_.target_reselecting;
        out.split_target_candidate_id = search_phase_.first_trip_split_.target_candidate_id;
        if (capture_load_.active)
            out.capture_keep_inventory = capture_load_.choice.inventory;
        out.capture_supply_score = capture_load_.choice.supply_score;
        out.capture_injured_score = capture_load_.choice.injured_score;
        out.target_id = trip_context_.target_id_;
        out.target_is_search_cue = search_phase_.cue_mode_;
        out.search_cue_id = search_phase_.search_cue_id_;
        out.capture_target_id = trip_context_.capture_target_id_;
        out.cue_contact_allowed = false; // searching never grants contact permission
        out.clearing_attempts = search_phase_.clear_attempts_;
        out.startup_commanded_us = startup_commanded_us_;
        out.clear_commanded_us = search_phase_.clear_commanded_us_;
        out.batch_size = trip_context_.trip_.total() ? trip_context_.trip_.total()
                                                     : trip_context_.pending_.total();
        out.delivered_total = total_;
        out.first_ordinary_delivered = first_;
        out.cargo_injured = transport_phase_.half_locked_
                                ? transport_phase_.half_injured_
                                : trip_context_.trip_.injured > 0;
        out.drop_locked = transport_phase_.drop_locked_;
        out.drop_centre_zone = transport_phase_.drop_centre_zone_;
        out.verdict = trip_context_.verdict_;
        out.reason = reason_;
        out.failure_source = failure_source_;
        out.original_reason = original_reason_;
        out.exit_reason = exit_reason_;
        out.capture_review_stage = capture_scene_.stage;
        out.capture_scene_status = capture_scene_.status;
        out.capture_scene_detail = capture_scene_.detail;
        out.capture_scene_blocker_id = capture_scene_.blocker_id;
        out.capture_scene_blocker_box = capture_scene_.blocker_box;
        out.capture_scene_association = capture_scene_.association;
        out.final_validation = final_validation_;
        out.capture_split_runs = search_phase_.first_trip_split_.runs;
        out.split_before_objects = capture_scene_.before_split;
        return out;
    };
    if (in.reset) {
        capture_scene_ = {};
        capture_load_ = {};
        failure_source_.clear();
        original_reason_.clear();
        exit_reason_.clear();
        final_validation_ = "not_executed";
        single_phase_initializing_ = single_phase_done_ = false;
        phase_waiting_ = false;
        phase_outcome_ = PhaseOutcome::Running;
        phase_wait_started_us_ = 0;
        phase_ = TaskPhase::Startup;
        run_status_ = RunStatus::Waiting;
        recovering_ = false;
        attempt_started_ = 0;
        bounded_approach_ = bounded_retreat_ = 0;
        clearTrip();
        search_phase_.first_trip_split_ = {};
        rejected_.clear();
        rejected_bearings_.clear();
        search_phase_.disperse_cool_until_us_ = 0;
        search_phase_.scatter_cool_until_us_ = 0;
        sweep_ledger_.reset();
            zone_mem_ = zone_enemy_ = run_yaw0_set_ = false;
        transport_phase_.carry_search_.reset();
        search_phase_.scatter_count_ = 0;
        search_phase_.scatter_approach_n_ = 0;
        search_phase_.scatter_far_ = false;
        search_phase_.burst_drive_used_ = search_phase_.burst_fast_ =
            search_phase_.burst_swing_armed_ = search_phase_.burst_swing_ = false;
        search_phase_.scan_turn_acc_ = 0;
        search_phase_.scan_mid_limit_ = 0;
        search_phase_.post_disperse_mid_ = false;
        search_phase_.disperse_small_ = false;
        search_phase_.mid_clear_n_ = 0;
        search_phase_.precheck_seen_ = 0;
        search_phase_.precheck_rejects_ = 0;
        total_ = 0;
        first_ = t_.assume_injured_trip || capture_after_first_;
        fault_ = false;
        search_phase_.clear_attempts_ = 0;
        search_phase_.near_failures_ = 0;
        search_phase_.lost_round_ = 0;
        startup_action_ = StartupAction::Advance;
        startup_pitch_turn_ = {};
        startup_pitch_ = kCameraPitchInvalid;
        startup_target_missing_since_us_ = 0;
        startup_target_decelerating_ = false;
        startup_alignment_since_us_ = startup_rejected_until_us_ = 0;
        startup_alignment_target_id_ = startup_rejected_id_ = -1;
        startup_commanded_us_ = 0;
        startup_yaw0_set_ = false;
        odo_ = ratio_est_ = 0;
        vref_used_ = false;
        zone_.reset();
        ratio_snap_ = imu_seen_ = false;
        vref_fix_us_ = vref_seen_us_ = 0;
        frame_raised_ = 0;
        gripper_us_ = now;
        stopped_from_ = PushState::WAIT_START;
        sweep_open_ = sweep_open_done_ = sweep_open_disabled_ = sweep_close_wait_ = false;
        sweep_open_us_ = sweep_close_us_ = 0;
        pitch_cmd_ = t_.far_pitch_cdeg;
        pitch_wait_us_ = now;
        enter(PushState::WAIT_START, now, "reset");
        last_us_ = now;
        return result();
    }
    // Checkpoints preserve the completed phase and its pending handoff. Ordinary
    // STOP/resume still uses the existing recovery path outside checkpoints.
    if (single_phase_done_) {
        last_us_ = now;
        return result();
    }
    if (phase_waiting_ && !fault_) {
        last_us_ = now;
        if (in.run && in.safety_ok) {
            const auto waited = now >= phase_wait_started_us_ ? now - phase_wait_started_us_ : 0;
            if (attempt_started_)
                attempt_started_ += waited;
            phase_waiting_ = false;
            applyPhaseOutcome(now);
            if (phase_ == TaskPhase::Transport) {
                transport_phase_.carry_us_ = now;
                transport_phase_.carry_approach_.reset();
                actionProgress().step = 1; // re-check the load after an inspection pause
            }
            pitch_wait_us_ = now;
            reason_ = "phase_step_started";
        }
        return result(); // the first tick after NEXT is stationary
    }
    if (state_ != PushState::CARRY)
        transport_phase_.carry_search_.reset();
    if (!run_yaw0_set_ && in.heading_valid && std::isfinite(in.heading_rad)) {
        run_yaw0_ = in.heading_rad;
        run_yaw0_set_ = true;
    }
    if (in.zone_sight_valid && in.heading_valid && std::isfinite(in.heading_rad) &&
        std::isfinite(in.zone_sight_bearing)) {
        const float yaw = wrapAngle(
            in.heading_rad +
            in.zone_sight_bearing); // world yaw of the sighted zone (yaw grows on left turns)
        if (in.zone_sight_enemy) {
            zone_enemy_ = true;
            zone_enemy_yaw_ = yaw;
            // Enemy-only frames must not erase an own-zone heading: overlapping
            // red/blue detections can alternate while carrying toward the same image region.
        } else {
            // Current own-color evidence supersedes an older enemy bearing in the same direction.
            if (zone_enemy_ && std::abs(wrapAngle(yaw - zone_enemy_yaw_)) < .5f)
                zone_enemy_ = false;
            const float d = wrapAngle(yaw - zone_mem_yaw_);
            zone_mem_yaw_ = (zone_mem_ && now - zone_mem_us_ < 3000000 && std::abs(d) < .6f)
                                ? wrapAngle(zone_mem_yaw_ + .5f * d)
                                : yaw;
            zone_mem_ = true;
            zone_mem_us_ = now;
        }
    }
    const bool timely =
        now > 0 && (last_us_ == 0 || (now > last_us_ && now - last_us_ <= t_.frame_timeout_us));
    const float dt = timely && last_us_ ? float(now - last_us_) * 1e-6f : 0.f;
    last_us_ = now;
    // Count issued forward motion time only. Stops, camera waits and pauses do not consume it.
    if (state_ == PushState::START_ADVANCE && timely) {
        const auto elapsed_us = static_cast<uint64_t>(std::llround(double(dt) * 1000000));
        if (startup_action_ == StartupAction::Advance && last_vx_ > 0)
            startup_commanded_us_ += elapsed_us;
    }
    if (in.sent_velocity_valid) {
        last_vx_ = in.sent_vx_mps;
        last_wz_ = in.sent_wz_rps;
    }
    odo_ += last_vx_ * dt;
    { // TEMP_ASSUMPTION：里程系锚定的粗略区域位姿（IMU航向 +
      // 按比例缩放的指令位移），视野丢失后继续可用
        float dyaw = last_wz_ * dt;
        if (in.heading_valid) {
            if (imu_seen_)
                dyaw = wrapAngle(in.heading_rad - last_imu_yaw_);
            last_imu_yaw_ = in.heading_rad;
            imu_seen_ = true;
        } else
            imu_seen_ = false;
        zone_.advance(last_vx_ * dt / drRatio(now), dyaw);
        const auto& fix = in.vref_zone;
        const char* why = "none";
        if (!in.vref_valid || !fix.valid)
            why = "no_fix";
        else if (in.vref_points < 3)
            why = "points<3";
        else if (!(now >= fix.timestamp_us && now - fix.timestamp_us <= 300000))
            why = "stale_ts";
        else if (fix.timestamp_us == vref_fix_us_)
            why = "dup_ts";
        else {
            vref_fix_us_ = fix.timestamp_us;
            why = "rejected";
            if (zone_.update(fix, now)) {
                why = "accepted";
                vref_seen_us_ = now;
                const cv::Point2f hv = fix.bodyToZone({0, holdCenter()});
                if (!ratio_snap_) {
                    ratio_hv_ = hv;
                    ratio_odo_ = odo_;
                    ratio_snap_ = true;
                } else {
                    const float vis = float(std::hypot(hv.x - ratio_hv_.x, hv.y - ratio_hv_.y)),
                                cmd = odo_ - ratio_odo_;
                    if (vis >= .15f && cmd >= .30f) {
                        const float r = std::clamp(cmd / vis, 1.f, t_.assume_ratio_max);
                        ratio_est_ = ratio_est_ > 0 ? .5f * (ratio_est_ + r) : r;
                        ratio_hv_ = hv;
                        ratio_odo_ = odo_;
                    } else if (cmd < -.05f)
                        ratio_snap_ = false;
                }
            }
        }
        if (zone_.valid(now)) {
            const ZoneEstimate pz = zone_.predicted(now);
            const cv::Point2f ph = pz.bodyToZone({0, holdCenter()});
            char b[300];
            std::snprintf(b,
                          sizeof b,
                          "feed=%s n=%d fix_age_ms=%.0f seen_age_ms=%.0f acc=%d rej=%d repl=%d "
                          "dist=%.3f dyaw=%.3f ratio=%.1f odo=%.3f pred_ox=%.3f pred_oy=%.3f "
                          "pred_yaw=%.3f hold_x=%.3f hold_y=%.3f",
                          why,
                          in.vref_points,
                          in.vref_valid ? double(now - fix.timestamp_us) / 1000. : -1.,
                          vref_seen_us_ ? double(now - vref_seen_us_) / 1000. : -1.,
                          zone_.accepted,
                          zone_.rejected,
                          zone_.replaced,
                          zone_.last_dist,
                          zone_.last_dyaw,
                          motionRatio(),
                          odo_,
                          pz.origin_body_m.x,
                          pz.origin_body_m.y,
                          pz.yaw_body_rad,
                          ph.x,
                          ph.y);
            anchor_dbg_ = b;
        } else
            anchor_dbg_ = std::string("feed=") + why + " zone=none";
    }
    actionProgress().travel += std::abs(last_vx_) * dt;
    transport_phase_.carried_ += std::abs(last_vx_) * dt;
    if (last_vx_ > 0)
        trip_context_.approach_forward_ += last_vx_ * dt;
    if (t_.attempt_budget_us && in.run) {
        if (!attempt_started_)
            attempt_started_ = now;
        if (last_vx_ < 0)
            bounded_retreat_ -= last_vx_ * dt;
        if (last_vx_ > 0 && trip_context_.trip_.total() == 0)
            bounded_approach_ += last_vx_ * dt;
        if (now < attempt_started_ || now - attempt_started_ > t_.attempt_budget_us ||
            (t_.approach_limit_m > 0 && bounded_approach_ >= t_.approach_limit_m) ||
            (t_.retreat_limit_m > 0 && bounded_retreat_ >= t_.retreat_limit_m))
            fail("bounded_attempt_exhausted");
    }
    if (fault_) {
        state_ = PushState::SAFE_STOP;
        return result();
    }
    if (t_.require_multi_view && state_ != PushState::WAIT_START && (!in.safety_ok || !timely)) {
        fail(!timely ? "stale_frame" : "safety_veto");
        state_ = PushState::SAFE_STOP;
        return result();
    }
    if (!in.run || !in.safety_ok || !timely) {
        capture_load_.confirmations = 0;
        capture_load_.stopped_us = now;
        auto& selection = search_phase_.pixel_reselection_;
        selection.stopped_us = selection.sample_us = selection.stable_us = 0;
        selection.frames = 0;
        auto& split = search_phase_.first_trip_split_;
        if (state_ != PushState::WAIT_START) {
            stopped_from_ = state_;
            if (split.capture_retry) {
                capture_scene_.reobserve_stopped_us = capture_scene_.reobserve_sample_us = 0;
                capture_scene_.reobserve_frames = 0;
                split.paused_us = now;
                split.margin_tick_us = split.margin_scene_us = 0;
                split.margin_sent_reverse = false;
                split.margin_stage = SearchPhaseData::FirstTripSplit::MarginStage::Idle;
                using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
                if (split.recovery_stage != Stage::InjuredTurn &&
                    split.recovery_stage != Stage::CornerTurnRight &&
                    split.recovery_stage != Stage::CornerPush &&
                    split.recovery_stage != Stage::CornerReturn &&
                    split.recovery_stage != Stage::CornerTurnBack)
                    split.recovery_stage = Stage::Retreat;
                split.recovery_turn_us = split.recovery_view_us = 0;
                split.sample_us = 0;
                split.confirmations = 0;
                if (split.active) {
                    split.active = false;
                    split.pending = true; // resuming a partly executed cut needs a new measurement
                    split.runs = std::max(0, split.runs - 1);
                }
            }
        }
        enter(PushState::WAIT_START,
              now,
              !in.run         ? "not_running"
              : !in.safety_ok ? "safety_veto"
                              : "stale_frame");
        return result();
    }
    if (state_ == PushState::WAIT_START) {
        if (!single_phase_.empty() &&
            (stopped_from_ == PushState::WAIT_START || single_phase_initializing_)) {
            if (single_phase_ == "startup")
                enter(PushState::START_ADVANCE, now, "phase_startup_direct");
            else if (single_phase_ == "search") {
                frame_raised_ = 1;
                enter(PushState::SCAN, now, "phase_search_direct");
            } else {
                frame_raised_ = single_phase_ == "capture" ? 1 : 0;
                enter(single_phase_ == "capture" ? PushState::RUSH : PushState::CARRY,
                      now,
                      "phase_initial_evidence");
                single_phase_initializing_ = true;
            }
            return result();
        }
        auto& split = search_phase_.first_trip_split_;
        if (split.paused_us) {
            split.started_us += now - split.paused_us;
            split.paused_us = 0;
        }
        enter(resumeState(), now, "resume");
        if (state_ == PushState::CARRY) { // a pause is not carry time; re-check the hold
            transport_phase_.carry_us_ = now;
            transport_phase_.carry_approach_.reset();
            actionProgress().step = 1;
        }
        stopped_from_ = PushState::WAIT_START;
        return result();
    }
    if (capture_load_.active && !search_phase_.first_trip_split_.pending &&
        search_phase_.first_trip_split_.target_reselecting) {
        tick.immediate_motion_stop = true;
        auto& split = search_phase_.first_trip_split_;
        reason_ = "capture_score_target_confirm_wait";
        if (now - capture_load_.started_us > t_.pile_budget_us) {
            failStop(tick, "capture_score_review_timeout");
            return result();
        }
        const auto target = std::find_if(in.capture_image_objects.begin(),
            in.capture_image_objects.end(), [&](const auto& object) {
                return object.id == split.target_candidate_id && object.complete &&
                    object.selectable && captureLoadAllows(object.label) && object.label == in.label;
            });
        if (in.target_valid && in.target_id == split.target_candidate_id &&
            (!in.pixel_selection_managed || in.pixel_locked) &&
            target != in.capture_image_objects.end() && captureReviewStopped(in) &&
            in.camera_pitch_stable && in.pixel_grasp_near_view && in.gripper_done &&
            in.gripper_feedback_open == 1 && in.capture_scene_valid && in.capture_scene_us &&
            in.capture_scene_us <= now && now - in.capture_scene_us <= t_.frame_timeout_us) {
            trip_context_.target_id_ = trip_context_.capture_target_id_ = in.target_id;
            trip_context_.label_ = in.label;
            trip_context_.pending_ = {};
            search_entry_ = {};
            pixel_alignment_ = {};
            const auto started = capture_scene_.started_us;
            capture_scene_ = {};
            capture_scene_.started_us = started;
            split.target_reselecting = false;
            split.target_candidate_id = -1;
            reason_ = "capture_score_target_accepted";
        }
        return result();
    }
    if (in.pixel_reselected_from >= 0 && in.pixel_reselected_from == trip_context_.target_id_) {
        tick.immediate_motion_stop = true;
        const auto selected =
            std::find_if(in.capture_image_objects.begin(),
                         in.capture_image_objects.end(),
                         [&](const auto& object) {
                             return object.id == in.target_id && object.label == in.label;
                         });
        const bool allowed =
            t_.pixel_grasp_enabled && in.pixel_selection_managed && !recovering_ &&
            !search_entry_.handoff && !search_phase_.first_trip_split_.active &&
            !search_phase_.first_trip_split_.pending &&
            (state_ == PushState::APPROACH || state_ == PushState::RUSH) &&
            (phase_ == TaskPhase::Search ||
             (phase_ == TaskPhase::Capture && capture_phase_.action == CaptureAction::Advance)) &&
            in.camera_pitch_stable && in.pixel_grasp_near_view && in.gripper_done &&
            in.gripper_feedback_open == 1 && captureReviewStopped(in) && in.capture_scene_valid &&
            in.capture_scene_us && in.capture_scene_us <= now &&
            now - in.capture_scene_us <= t_.frame_timeout_us && candidate(in, now) &&
            selected != in.capture_image_objects.end() && selected->complete;
        reason_ = "near_target_reselection_wait";
        if (allowed) {
            trip_context_.target_id_ = trip_context_.capture_target_id_ = in.target_id;
            trip_context_.label_ = in.label;
            trip_context_.pending_ = trip_context_.seen_ = {};
            search_entry_ = {};
            pixel_alignment_ = {};
            search_phase_.far_target_recovery_ = {};
            const auto started = capture_scene_.started_us;
            const bool waiting = capture_scene_.waiting;
            capture_scene_ = {};
            capture_scene_.started_us = started;
            capture_scene_.waiting = waiting;
            capture_scene_.stopped_us = now;
            capture_scene_.confirm_us = in.capture_scene_us;
            capture_phase_.pixel_forward_us_ = 0;
            actionProgress().confirmations = 0;
            reason_ = "near_target_reselected";
        }
        return result(); // The new object must pass fresh alignment and inventory review.
    }
    if (single_phase_initializing_) {
        initializeSinglePhase(tick);
        if (fault_)
            state_ = PushState::SAFE_STOP;
        return result();
    }
    for (auto it = rejected_.begin(); it != rejected_.end();)
        it = now >= it->second ? rejected_.erase(it) : std::next(it);
    rejected_bearings_.erase(std::remove_if(rejected_bearings_.begin(),
                                            rejected_bearings_.end(),
                                            [now](const RejectedBearing& b) {
                                                return now >= b.until_us;
                                            }),
                             rejected_bearings_.end());

    tick.forward_ok = in.path_safe && in.opponent_zone_clear;
    tick.scan_ok =
        in.opponent_zone_clear && (in.directional_clearance_valid ? in.turn_safe : in.path_safe);
    tick.observed_zone = zoneOk(in) && in.zone_estimate.source != ZoneEstimate::Source::PREDICTED;
    tick.elapsed = now - actionProgress().started_us;
    // One approach deadline precedes pitch/target waits and survives view handoffs.
    if (t_.pixel_grasp_enabled && phase_ == TaskPhase::Search && !recovering_ &&
        search_phase_.action != SearchAction::Prepare && search_phase_.pixel_approach_started_us_ &&
        now - search_phase_.pixel_approach_started_us_ > t_.approach_budget_us) {
        tick.immediate_motion_stop = true;
        if (!first_ && targetKind(trip_context_.label_) == TargetKind::ORDINARY) {
            capture_scene_.stage = "approach";
            enter(PushState::CAPTURE_FAIL, now, "pixel_approach_timeout");
        } else
            scanAfterClear(tick, "pixel_approach_timeout");
        return result();
    }
    // Grasp timing uses the detected box centre projected at block mid-height.
    // Legacy replay observations without a centre retain their supplied range.
    tick.target_forward_m = in.target_center_valid ? in.target_center_body.y
                                                   : in.distance_m * std::cos(in.heading_error);

    if (tracking(in) && in.geometry_valid) {
        last_target_xy_ = targetBody(tick);
        last_target_valid_ = true;
    }
    if (tracking(in) && (state_ == PushState::PREPARE || state_ == PushState::RUSH) &&
        in.heading_valid && std::isfinite(in.heading_rad) && std::isfinite(in.heading_error)) {
        rush_yaw_ref_ = wrapAngle(in.heading_rad + in.heading_error);
        rush_yaw_ref_set_ = true;
    }
    tick.scan_spin = false;

    // Gripper commands above stay concurrent with the camera; anything that moves, tracks or
    // counts waits for the preset (frames during a move carry no reliable geometry or hold).
    if ((state_ == PushState::CAPTURE_FAIL || state_ == PushState::LOST_HOLD ||
         state_ == PushState::ABORT_DROP) &&
        in.opponent_zone_clear)
        commandFrame(1, now); // release at rest while recovering the FAR view
    // The fixed post-release jaw retreat does not use camera observations.
    const bool jaw_back_out = state_ == PushState::BACK_OUT && t_.pixel_grasp_enabled && carryLogic();
    if (state_ != PushState::PREPARE && state_ != PushState::RAISE_RELEASE &&
        state_ != PushState::VERIFY_CAPTURE && !jaw_back_out) {
        const int p = pitchWait(in, now);
        if (p < 0) {
            failStop(tick, "camera_pitch_timeout");
            return result();
        }
        if (p == 0) {
            auto& selection = search_phase_.pixel_reselection_;
            selection.stopped_us = selection.sample_us = selection.stable_us = 0;
            selection.frames = 0;
            if (search_phase_.first_trip_split_.pending) {
                search_phase_.first_trip_split_.recovery_turn_us = 0;
                capture_scene_.reobserve_stopped_us = capture_scene_.reobserve_sample_us = 0;
                capture_scene_.reobserve_frames = 0;
            }
            capture_phase_.rush_stall_last_us_ = 0;
            reason_ = "camera_pitch_wait";
            return result();
        } // moving-camera time cannot count as stationary re-observation
    }
    const bool drop_fresh =
        in.navigation_timestamp_us && now >= in.navigation_timestamp_us &&
        now - in.navigation_timestamp_us <= 200000 && in.drop_plan_valid &&
        transport_phase_.drop_locked_ && std::isfinite(in.drop_centre_zone.x) &&
        std::isfinite(in.drop_centre_zone.y) &&
        cv::norm(in.drop_centre_zone - transport_phase_.drop_centre_zone_) <= .005f;
    // TEMP_ASSUMPTION: assume mode treats the drop point as valid without navigation revalidation.
    if (!carryLogic() &&
        ((state_ == PushState::CARRY && transport_phase_.placing_) ||
         state_ == PushState::RAISE_RELEASE) &&
        !drop_fresh) {
        reason_ = "drop_revalidation_missing";
        if (now - transport_phase_.carry_us_ > t_.carry_budget_us) {
            failStop(tick, "drop_revalidation_timeout");
        }
        return result();
    }

    syncFrameOverlay(
        now); // a state change made last tick takes effect before this tick's state logic runs
    if (sweep_open_ && !sweep_open_done_ && in.gripper_done && in.gripper_feedback_open == 1 &&
        now > sweep_open_us_)
        sweep_open_done_ = true;
    if (sweep_close_wait_) { // the frame opened for a sweep must be back down before the chassis
                             // moves on
        if (frame_raised_ != 0 || (in.gripper_done && in.gripper_feedback_open == 0))
            sweep_close_wait_ = false;
        else if (now - sweep_close_us_ > t_.gripper_timeout_us) {
            sweep_close_wait_ = false;
            sweep_open_disabled_ = true;
            std::cout << "[SWEEP_FRAME] close not confirmed within " << t_.gripper_timeout_us / 1000
                      << " ms: sweep frame-open disabled\n";
        } else {
            reason_ = "sweep_frame_close_wait";
            return result();
        }
    }
    // Before capture closes the frame, identity changes are explicit selector events.
    const bool pixel_identity_phase =
        phase_ == TaskPhase::Search ||
        (phase_ == TaskPhase::Capture &&
         (capture_phase_.action == CaptureAction::Advance ||
          (capture_phase_.action == CaptureAction::LowerFrame && frame_raised_ != 0)));
    if (t_.pixel_grasp_enabled && in.pixel_selection_managed && pixel_identity_phase &&
        !recovering_ && trip_context_.target_id_ >= 0) {
        if (in.pixel_released_id == trip_context_.target_id_ &&
            (state_ == PushState::RUSH ||
             !(t_.pixel_grasp_enabled && targetSelectable(trip_context_.label_, first_) &&
               !capture_scene_.objects.empty()))) {
            tick.immediate_motion_stop = true;
            search_phase_.pixel_approach_started_us_ = 0;
            search_phase_.far_target_recovery_ = {};
            trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
            trip_context_.label_.clear();
            pixel_alignment_ = {};
            actionProgress().confirmations = 0;
            if (phase_ == TaskPhase::Capture || search_phase_.action == SearchAction::Prepare)
                enter(PushState::CAPTURE_FAIL, now, "pixel_target_released");
            reason_ = "pixel_target_released";
            return result();
        }
        if (in.pixel_rebound_from == trip_context_.target_id_ && in.label == trip_context_.label_ &&
            candidate(in, now)) {
            trip_context_.target_id_ = trip_context_.capture_target_id_ = in.target_id;
            search_phase_.far_target_recovery_ = {};
            if (!rush_rebind) {
                pixel_alignment_ = {};
                actionProgress().confirmations = 0;
                reason_ = "pixel_target_rebound";
                return result();
            }
            // Continue this tick with the existing action progress and alignment.
            // Scene review below still verifies the physical association and inventory.
        }
    }
    if (rush_gap && capture_phase_.pixel_forward_us_ &&
        now >= capture_phase_.pixel_forward_us_ &&
        now - capture_phase_.pixel_forward_us_ <= 200000 &&
        in.camera_pitch_stable && in.pixel_grasp_near_view &&
        in.camera_pitch_cdeg == capture_phase_.pixel_forward_pitch_ &&
        capture_scene_.status == "clear" && !capture_scene_.waiting &&
        in.capture_scene_valid && in.capture_path_valid && in.capture_scene_us &&
        in.capture_scene_us <= now && now - in.capture_scene_us <= t_.frame_timeout_us &&
        in.capture_scene_us > capture_scene_.sample_us &&
        !in.capture_path_px.empty() && !in.layout_count_region_px.empty() &&
        tick.elapsed <= t_.rush_budget_us &&
        (actionProgress().travel_limit <= 0 ||
         actionProgress().travel < actionProgress().travel_limit) &&
        checkCorridor(in.corridor, in.corridor_complete, in.corridor_occlusion_free,
                      trip_context_.label_, first_) == RuleVerdict::OK) {
        // Every old neighbour must remain visible, and every current neighbour must
        // remain outside both the inventory and forward path. Missing evidence stops.
        bool clear = !in.capture_dangerous_side_pending;
        for (const auto& old : capture_scene_.objects) {
            if (old.id == capture_scene_.target_id)
                continue;
            int matches = 0;
            for (const auto& object : in.capture_image_objects)
                if (object.id == old.id && object.label == old.label && object.complete)
                    ++matches;
            clear &= matches == 1;
        }
        for (const auto& object : in.capture_image_objects)
            clear &= object.complete &&
                !in.splitInventoryOverlaps(object.box, object.label) &&
                !in.capturePathOverlaps(object.box, object.label);
        if (clear && tick.forward_ok) {
            reason_ = "pixel_rush_gap_hold";
            drive(tick, std::min(t_.rush_speed, t_.rush_near_speed), 0);
            return result();
        }
    }
    const bool first_trip_review_point =
        !recovering_ &&
        phase_ == TaskPhase::Capture &&
        (capture_phase_.action == CaptureAction::Advance ||
         (capture_phase_.action == CaptureAction::LowerFrame && frame_raised_ != 0));
    if (first_trip_review_point && (!search_entry_.handoff || entryPositionValid(in)) &&
        !reviewFirstTripCapture(tick,
                                state_ == PushState::LOWER_FRAME ? "preclose" : "approach",
                                state_ == PushState::LOWER_FRAME))
        return result();
    phase_outcome_ = PhaseOutcome::Running;
    if (recovering_) {
        updateRecoveryPhase(tick);
    } else
        switch (phase_) {
            case TaskPhase::Startup:
                updateStartupPhase(tick);
                break;
            case TaskPhase::Search:
                updateSearchPhase(tick);
                break;
            case TaskPhase::Capture:
                updateCapturePhase(tick);
                break;
            case TaskPhase::Transport:
                updateTransportPhase(tick);
                break;
        }

    const bool capture_split_retry =
        single_phase_ == "capture" && search_phase_.first_trip_split_.capture_retry;
    if (!single_phase_.empty() && recovering_ && !fault_ && !capture_split_retry)
        fail("single_phase_recovery_required");
    const bool internal_capture_retry =
        capture_split_retry && (phase_outcome_ == PhaseOutcome::SearchReady ||
                                phase_outcome_ == PhaseOutcome::CaptureReady);
    if (!fault_ && ((!single_phase_.empty() && !internal_capture_retry) || phase_stepping_) &&
        phase_outcome_ != PhaseOutcome::Running) {
        single_phase_done_ = !single_phase_.empty();
        phase_waiting_ = true;
        phase_wait_started_us_ = now;
        run_status_ = RunStatus::Paused;
        motion.vx_mps = motion.wz_rps = 0;
    } else if (!fault_)
        applyPhaseOutcome(now);

    if (fault_) {
        state_ = PushState::SAFE_STOP;
        motion.vx_mps = motion.wz_rps = 0;
    }
    return result();
}
std::vector<PushTask::RejectedBearing> PushTask::rejectedBearings(uint64_t now) const {
    std::vector<RejectedBearing> out;
    for (const auto& b : rejected_bearings_)
        if (b.until_us > now)
            out.push_back(b);
    return out;
}
std::vector<int> PushTask::rejectedTargets(uint64_t now) const {
    std::vector<int> ids;
    for (const auto& item : rejected_)
        if (item.second > now)
            ids.push_back(item.first);
    return ids;
}
} // namespace rescue
