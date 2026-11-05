#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
namespace rescue {
namespace {
constexpr float kSearchUi120SpeedMps = .04f;
constexpr float kSearchUi120HandoffHeight = .625f; // Y450 at 1280x720; initial field tuning
constexpr uint64_t kSearchViewHandoffTimeoutUs = 1500000;
} // namespace

bool PushTask::updateSearchViewHandoff(TaskTick& tick) {
    auto& handoff = search_phase_.view_handoff_;
    const auto& in = tick.in;
    if (phase_ != TaskPhase::Search || recovering_) {
        handoff = {};
        return false;
    }
    if (!handoff.started_us) {
        const bool early_line =
            in.pixel_grasp_image_height_px > 0 &&
            in.target_center_px.y >= kSearchUi120HandoffHeight * in.pixel_grasp_image_height_px;
        const bool fresh = in.pixel_grasp_sample_us && in.pixel_grasp_sample_us <= tick.now &&
                           tick.now - in.pixel_grasp_sample_us <= t_.frame_timeout_us;
        const bool retry_progress = search_phase_.pixel_handoff_retry_height_ <= 0 ||
            (in.pixel_grasp_image_height_px > 0 && in.target_center_px.y >=
                search_phase_.pixel_handoff_retry_height_ * in.pixel_grasp_image_height_px);
        // Legacy replay observations may supply only the existing handoff flag.
        if (!retry_progress || !t_.pixel_grasp_enabled || !in.pixel_grasp_view ||
            in.pixel_grasp_near_view || !in.target_center_px_valid || !fresh ||
            !candidate(in, tick.now) ||
            in.target_id != trip_context_.target_id_ || in.label != trip_context_.label_ ||
            (!early_line && !in.pixel_view_handoff_ready) ||
            in.camera_pitch_cdeg == kCameraPitchInvalid ||
            std::abs(int(in.camera_pitch_cdeg) - int(t_.track_pitch_cdeg)) >
                t_.pitch_tolerance_cdeg)
            return false;
        handoff.started_us = tick.now;
        if (in.pixel_grasp_image_height_px > 0)
            handoff.retry_height = (in.target_center_px.y +
                std::max(1.f, t_.pixel_align_brake_motion_px)) / in.pixel_grasp_image_height_px;
        pixel_alignment_ = {};
    }
    tick.immediate_motion_stop = true;
    if (tick.now - handoff.started_us >= kSearchViewHandoffTimeoutUs ||
        (search_phase_.pixel_approach_started_us_ &&
         tick.now - search_phase_.pixel_approach_started_us_ > t_.approach_budget_us)) {
        // Repeated IDs at the same UI120 position cannot retry an empty/clipped
        // near view. Resume normal low-speed approach until fresh image progress.
        if (handoff.near && handoff.retry_height > 0)
            search_phase_.pixel_handoff_retry_height_ = handoff.retry_height;
        resumePixelSplitSearch(tick, "pixel_search_handoff_research", true);
        return true;
    }
    const int pitch = pitchWait(in, tick.now);
    const bool scene_fresh = in.capture_scene_valid && in.capture_scene_us &&
                             in.capture_scene_us <= tick.now &&
                             tick.now - in.capture_scene_us <= t_.frame_timeout_us;
    const auto reset_window = [&] {
        handoff.frames = 0;
        handoff.sample_us = handoff.stable_us = 0;
    };
    if (pitch <= 0 || !scene_fresh || !captureReviewStopped(in)) {
        reset_window();
        // Decelerate through the existing sender ramp while the stationary camera still
        // supplies fresh scene evidence. Missing target identity does not restart approach.
        tick.controlled_motion_stop =
            !handoff.near && pitch > 0 && scene_fresh && in.path_safe && in.opponent_zone_clear &&
            (!in.directional_clearance_valid || (in.turn_safe && in.arc_safe));
        tick.immediate_motion_stop = !tick.controlled_motion_stop;
        reason_ = pitch <= 0     ? "camera_pitch_wait"
                  : handoff.near ? "pixel_search_handoff_confirm"
                                 : "pixel_search_handoff_stop";
        return true;
    }
    commandFrame(1, tick.now);
    const int frame = frameWait(in, 1, tick.now);
    if (frame < 0) {
        failStop(tick, "gripper_open_timeout");
        return true;
    }
    if (!frame) {
        reset_window();
        reason_ = "gripper_feedback_wait";
        return true;
    }
    if (!handoff.near) {
        reason_ = "pixel_search_handoff_stop";
        // Fresh whole-scene observations continue even when the old box leaves UI120.
        // As in Startup's handoff, this is a stopped-command window, not wheel odometry.
        if (in.capture_scene_us <= handoff.started_us || in.capture_scene_us <= handoff.sample_us)
            return true;
        if (!handoff.frames)
            handoff.stable_us = in.capture_scene_us;
        handoff.sample_us = in.capture_scene_us;
        if (++handoff.frames < t_.pixel_align_stable_frames ||
            in.capture_scene_us - handoff.stable_us < t_.pixel_align_stable_us)
            return true;
        enter(PushState::APPROACH, tick.now, "pixel_center_switch_to_140");
        search_phase_.approach_pitch_ = t_.near_pitch_cdeg;
        handoff.near = true;
        handoff.started_us = tick.now;
        reset_window();
        pixel_alignment_ = {};
        search_entry_ = {};
        capture_scene_ = {};
        trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
        trip_context_.label_.clear();
        return true;
    }
    // A fresh wanted block in this view is sufficient; no old ID/class linkage.
    const bool selected = candidate(in, tick.now) &&
                          (!in.pixel_selection_managed || in.pixel_locked);
    const bool complete = std::any_of(
        in.capture_image_objects.begin(), in.capture_image_objects.end(), [&](const auto& object) {
            return object.id == in.target_id && object.label == in.label && object.complete;
        });
    if (in.capture_scene_us <= handoff.started_us || !in.pixel_grasp_near_view || !selected ||
        !complete || !candidate(in, tick.now) || (in.pixel_selection_managed && !in.pixel_locked) ||
        !pixelAlign(tick, true, true)) {
        reason_ = "pixel_search_handoff_confirm";
        return true;
    }
    handoff = {};
    search_phase_.pixel_handoff_retry_height_ = 0;
    reason_ = "pixel_search_handoff_ready";
    return true; // retain zero; the normal near-view entry/selector review runs next tick
}

// Reconsider a pre-capture UI120 lock only after sustained lack of forward
// progress and a better complete candidate. A progressing lock remains stable.
bool PushTask::reselectPixelApproach(TaskTick& tick) {
    auto& watch = search_phase_.pixel_reselection_;
    const auto& in = tick.in;
    const bool view =
        in.camera_pitch_stable && in.camera_pitch_cdeg != kCameraPitchInvalid &&
        std::abs(int(in.camera_pitch_cdeg) - int(t_.track_pitch_cdeg)) <= t_.pitch_tolerance_cdeg;
    const bool scene = in.capture_scene_valid && in.capture_scene_us &&
        in.capture_scene_us <= tick.now && tick.now - in.capture_scene_us <= t_.frame_timeout_us;
    if (watch.observing) {
        tick.immediate_motion_stop = true;
        reason_ = "pixel_reselect_observe";
        if (!view || !scene || !captureReviewStopped(in)) {
            watch.stopped_us = watch.sample_us = watch.stable_us = 0;
            watch.frames = 0;
            return true;
        }
        if (!watch.stopped_us)
            watch.stopped_us = tick.now;
        if (in.capture_scene_us <= watch.stopped_us || in.capture_scene_us <= watch.sample_us)
            return true;
        if (watch.sample_us && in.capture_scene_us - watch.sample_us > t_.frame_timeout_us)
            watch.frames = 0;
        watch.sample_us = in.capture_scene_us;
        const bool heading = in.heading_valid && std::isfinite(in.heading_rad);
        if (!watch.frames || (heading && (!std::isfinite(watch.heading) ||
            std::abs(wrapAngle(in.heading_rad - watch.heading)) > .01f))) {
            watch.frames = 0;
            watch.stable_us = in.capture_scene_us;
            watch.heading = heading ? in.heading_rad : std::numeric_limits<float>::quiet_NaN();
        }
        ++watch.frames;
        if (watch.frames >= 3 && in.capture_scene_us - watch.stable_us >= 500000) {
            watch = {};
            reason_ = "pixel_reselect_ready";
        }
        return true; // retain zero while the selector starts fresh confirmation
    }
    const bool approaching = phase_ == TaskPhase::Search &&
        search_phase_.action == SearchAction::Approach && in.pixel_selection_managed &&
        trip_context_.target_id_ >= 0 && frame_raised_ == 1 && view && !in.pixel_grasp_near_view;
    if (!approaching || !scene || !in.sent_velocity_valid ||
        !std::isfinite(in.sent_vx_mps) || in.sent_vx_mps > t_.pixel_align_stopped_vx) {
        watch = {};
        return false;
    }
    if (watch.target_id != trip_context_.target_id_) {
        watch = {};
        watch.target_id = trip_context_.target_id_;
        watch.no_progress_us = tick.now;
    }
    if (in.capture_scene_us <= watch.sample_us)
        return false;
    watch.sample_us = in.capture_scene_us;
    const bool missing = !in.target_valid;
    if (missing && !watch.missing)
        ++watch.gaps;
    watch.missing = missing;
    if (!missing && !in.target_box.empty())
        watch.last_box = in.target_box;
    const auto elapsed = tick.now - watch.no_progress_us;
    if (watch.last_box.empty() || elapsed < 2500000 ||
        (watch.gaps < 3 && elapsed < 4000000))
        return false;
    const bool better = std::any_of(in.capture_image_objects.begin(),
        in.capture_image_objects.end(), [&](const auto& object) {
            const auto rejected = rejected_.find(object.id);
            return object.id >= 0 && object.id != trip_context_.target_id_ && object.complete &&
                targetSelectable(object.label, first_) &&
                object.box.area() >= 2. * watch.last_box.area() &&
                (rejected == rejected_.end() || tick.now >= rejected->second);
        });
    if (!better)
        return false;
    // A short ID cooldown prevents immediately reacquiring the stalled target.
    // Preserve the original approach deadline across this selection change.
    rejected_[trip_context_.target_id_] = tick.now + 4000000;
    trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
    trip_context_.label_.clear();
    trip_context_.pending_ = {};
    search_entry_ = {};
    pixel_alignment_ = {};
    search_phase_.far_target_recovery_ = {};
    capture_scene_ = {};
    watch = {};
    watch.observing = true;
    tick.immediate_motion_stop = true;
    reason_ = "pixel_approach_reselect";
    return true;
}

void PushTask::updateSearchPhase(TaskTick& tick) {
    auto& split = search_phase_.first_trip_split_;
    const bool image_search = search_phase_.action == SearchAction::Scan ||
                              search_phase_.action == SearchAction::LostSearch ||
                              search_phase_.action == SearchAction::MidReacquire ||
                              search_phase_.action == SearchAction::NearReacquire ||
                              search_phase_.action == SearchAction::SelectCargo ||
                              search_phase_.action == SearchAction::MidApproach ||
                              search_phase_.action == SearchAction::Approach;
    // The selector has confirmed this lock. Check membership before any discovery,
    // view handoff or split-search branch can command movement.
    if (t_.pixel_grasp_enabled && image_search && tick.in.pixel_selection_managed &&
        tick.in.pixel_locked && tick.in.target_valid && tick.in.target_id >= 0 &&
        searchTargetRegion(tick.in) == PushObservation::SearchRegion::Inside) {
        tick.immediate_motion_stop = true;
        rejected_[tick.in.target_id] = tick.now + t_.blacklist_us;
        scanAfterClear(tick, "search_target_in_zone");
        return;
    }
    if (t_.pixel_grasp_enabled && image_search && reselectPixelApproach(tick))
        return;
    if (t_.pixel_grasp_enabled && search_phase_.action == SearchAction::Approach &&
        split.search_started_us && !search_phase_.pixel_approach_started_us_ &&
        search_phase_.approach_pitch_ == t_.track_pitch_cdeg) {
        tick.immediate_motion_stop = true;
        if (tick.now - split.search_started_us > t_.pile_budget_us) {
            // An empty discovery window is recoverable: brake, renew the scene
            // and selector, then continue searching the same retained cargo class.
            resumePixelSplitSearch(tick, "pixel_split_search_restart", true);
            return;
        }
        const auto& in = tick.in;
        const bool view = in.camera_pitch_stable &&
            in.camera_pitch_cdeg != kCameraPitchInvalid &&
            std::abs(int(in.camera_pitch_cdeg) - int(t_.track_pitch_cdeg)) <= t_.pitch_tolerance_cdeg;
        const bool fresh = in.capture_scene_valid && in.capture_scene_us &&
            in.capture_scene_us <= tick.now && tick.now - in.capture_scene_us <= t_.frame_timeout_us;
        commandFrame(1, tick.now);
        const int frame = frameWait(in, 1, tick.now);
        if (frame < 0) {
            failStop(tick, "gripper_open_timeout");
            return;
        }
        if (!view || !fresh || frame == 0) {
            reason_ = "pixel_split_search_observation_wait";
            return;
        }
        if (!candidate(in, tick.now) || (in.pixel_selection_managed && !in.pixel_locked)) {
            split.align_stage = "ui120_search";
            reason_ = "pixel_split_search_spin";
            drive(tick, 0, t_.scan_wz, true);
            tick.immediate_motion_stop = tick.motion.wz_rps == 0;
            return;
        }
        // Brake on discovery. Keep this split's original search deadline: selector
        // release clears the approach timer and resumes discovery without renewing it.
        // Normal pixel alignment must settle before translation.
        split.align_stage = "ui120_approach";
        // Adopt the confirmed lock on this brake tick, so even a next-frame loss
        // can match the selector's release event and resume discovery.
        trip_context_.target_id_ = trip_context_.capture_target_id_ = in.target_id;
        trip_context_.label_ = in.label;
        search_phase_.pixel_approach_started_us_ = tick.now;
        reason_ = "pixel_split_search_target_found";
        return;
    }
    auto& recovery = search_phase_.far_target_recovery_;
    const bool far_tracking = t_.pixel_grasp_enabled && tick.in.pixel_selection_managed &&
                              search_phase_.action == SearchAction::Approach &&
                              trip_context_.target_id_ >= 0 && tick.in.camera_pitch_stable &&
                              tick.in.camera_pitch_cdeg != kCameraPitchInvalid &&
                              std::abs(int(tick.in.camera_pitch_cdeg) - int(t_.far_pitch_cdeg)) <=
                                  t_.pitch_tolerance_cdeg &&
                              !tick.in.pixel_grasp_view && !tick.in.pixel_view_handoff_ready;
    const auto& in = tick.in;
    const auto region = searchTargetRegion(in);
    const bool same_lock = search_phase_.action == SearchAction::Approach &&
        trip_context_.target_id_ >= 0 && in.target_valid &&
        in.target_id == trip_context_.target_id_ && in.label == trip_context_.label_;
    if (same_lock && region == PushObservation::SearchRegion::Inside) {
        tick.immediate_motion_stop = true;
        rejected_[in.target_id] = tick.now + t_.blacklist_us;
        scanAfterClear(tick, "search_target_in_zone");
        return;
    }
    // The deadline belongs to this lock. A geometry dropout cannot renew it.
    if (search_phase_.action != SearchAction::Approach ||
        region != PushObservation::SearchRegion::Unknown)
        search_phase_.far_region_wait_started_us_ = 0;
    auto& started = search_phase_.far_region_wait_started_us_;
    if (started && tick.now - started > t_.prepare_budget_us) {
        tick.immediate_motion_stop = true;
        rejected_[trip_context_.target_id_] = tick.now + t_.blacklist_us;
        scanAfterClear(tick, "pixel_far_region_timeout");
        return;
    }
    const bool fresh_region = in.search_region_sample_us &&
        in.search_region_sample_us <= tick.now &&
        tick.now - in.search_region_sample_us <= t_.frame_timeout_us;
    const auto rejected = rejected_.find(in.target_id);
    const bool region_wait = same_lock && in.pixel_locked && in.camera_pitch_stable &&
        region == PushObservation::SearchRegion::Unknown &&
        (fresh_region || (far_tracking && in.target_center_px_valid && in.geometry_valid &&
            std::isfinite(in.distance_m) && in.distance_m >= 0 &&
            std::isfinite(in.heading_error) && std::abs(in.heading_error) <= 1.2f &&
            in.pixel_grasp_sample_us && in.pixel_grasp_sample_us <= tick.now &&
            tick.now - in.pixel_grasp_sample_us <= t_.frame_timeout_us)) &&
        !in.target_is_search_cue && targetSelectable(in.label, first_) &&
        (rejected == rejected_.end() || tick.now >= rejected->second);
    if (region_wait) {
        if (!started) started = tick.now;
        actionProgress().misses = 0;
        recovery = {};
        tick.immediate_motion_stop = true;
        reason_ = "pixel_far_region_wait";
        return;
    }
    const bool deceleration_clear = tick.forward_ok && (!tick.in.directional_clearance_valid ||
                                                        (tick.in.turn_safe && tick.in.arc_safe));
    if (!far_tracking || !deceleration_clear || recovery.target_id != trip_context_.target_id_)
        recovery = {};
    if (t_.pixel_grasp_enabled && image_search && tick.in.pixel_selection_managed) {
        if (trip_context_.target_id_ >= 0 && !tick.in.target_valid) {
            reason_ = "pixel_target_wait";
            if (far_tracking && deceleration_clear &&
                recovery.target_id == trip_context_.target_id_ &&
                tick.in.pixel_reason == "lock_grace_waiting") {
                recovery.waiting = true;
                recovery.frames = 0;
                // Both requests stay zero; dispatch decelerates from the last sent command.
                tick.controlled_motion_stop = true;
                reason_ = "pixel_far_target_decelerate";
            }
            return;
        }
    }
    const auto accept_pixel_target = [&] {
        const bool initial = trip_context_.target_id_ < 0;
        const bool same =
            tick.in.target_id == trip_context_.target_id_ && tick.in.label == trip_context_.label_;
        if (!initial && !same) {
            reason_ = "pixel_target_identity_wait";
            return false;
        }
        if (!same) {
            trip_context_.target_id_ = trip_context_.capture_target_id_ = tick.in.target_id;
            trip_context_.label_ = tick.in.label;
        }
        selectSearchEntry(tick.in);
        if (!search_phase_.pixel_approach_started_us_)
            search_phase_.pixel_approach_started_us_ = tick.now;
        return true;
    };
    if (t_.pixel_grasp_enabled && image_search && tick.in.camera_pitch_stable &&
        tick.in.camera_pitch_cdeg != kCameraPitchInvalid &&
        std::abs(int(tick.in.camera_pitch_cdeg) - int(t_.far_pitch_cdeg)) <=
            t_.pitch_tolerance_cdeg) {
        // A bottom-edge box can have a valid image centre but no ground contact.
        // Stop and change views first; horizontal centering must not trap the FAR view.
        if (tick.in.pixel_view_handoff_ready && candidate(tick.in, tick.now)) {
            if (!accept_pixel_target())
                return;
            if (!pixelAlign(tick, true, true))
                return;
            enter(PushState::APPROACH, tick.now, "pixel_center_switch_to_120");
            search_phase_.approach_pitch_ = t_.track_pitch_cdeg;
            trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
            trip_context_.label_.clear();
            search_entry_ = {};
            pixel_alignment_ = {};
            capture_scene_ = {};
            return;
        }
        if (search_phase_.action == SearchAction::Approach) {
            const auto remember_target = [&] {
                search_phase_.approach_last_id_ = trip_context_.target_id_;
                search_phase_.approach_last_heading_ = tick.in.heading_error;
                search_phase_.approach_last_distance_ = tick.in.distance_m;
                search_phase_.approach_last_world_valid_ =
                    tick.in.heading_valid && std::isfinite(tick.in.heading_rad);
                if (search_phase_.approach_last_world_valid_)
                    search_phase_.approach_last_world_yaw_ =
                        wrapAngle(tick.in.heading_rad + tick.in.heading_error);
            };
            if (!tick.in.target_center_px_valid || !tracking(tick.in)) {
                recovery.frames = 0;
                // This FAR path bypasses SearchStage::approach: retain its same-object
                // re-ID limits instead of waiting for the obsolete ID until timeout.
                if (!tick.in.pixel_selection_managed && tick.in.target_center_px_valid &&
                    search_phase_.approach_last_id_ == trip_context_.target_id_ &&
                    trip_context_.target_id_ >= 0 &&
                    tick.in.target_id != trip_context_.target_id_ &&
                    tick.in.label == trip_context_.label_ && candidate(tick.in, tick.now) &&
                    std::abs(approachHeadingShift(tick.in)) <= .3f &&
                    std::abs(tick.in.distance_m - search_phase_.approach_last_distance_) <= .35f) {
                    trip_context_.target_id_ = trip_context_.capture_target_id_ = tick.in.target_id;
                    actionProgress().misses = 0;
                    remember_target();
                    reason_ = "pixel_far_id_rebound_same_object";
                    return; // keep zero velocity on the rebind tick
                }
                reason_ = "pixel_far_target_wait";
                if (lostTarget(tick))
                    scanAfterClear(tick, "pixel_far_target_lost");
                return;
            }
            if (far_tracking && recovery.waiting) {
                const auto sample = tick.in.pixel_grasp_sample_us;
                const bool fresh =
                    sample && sample <= tick.now && tick.now - sample <= t_.frame_timeout_us;
                if (!fresh) {
                    recovery.frames = 0;
                } else if (sample > recovery.sample_us) {
                    recovery.sample_us = sample;
                    ++recovery.frames;
                }
                if (!fresh || recovery.frames < 2) {
                    tick.controlled_motion_stop = deceleration_clear;
                    reason_ = "pixel_far_target_reconfirm";
                    return;
                }
                recovery.waiting = false;
                recovery.frames = 0;
            }
            actionProgress().misses = 0;
            remember_target();
            reason_ = "pixel_far_approach";
            steer(tick, t_.approach_speed, tick.in.heading_error);
            if (far_tracking && deceleration_clear &&
                (tick.motion.vx_mps != 0 || tick.motion.wz_rps != 0)) {
                recovery.target_id = trip_context_.target_id_;
                recovery.sample_us = tick.in.pixel_grasp_sample_us;
            }
            return;
        }
    }
    if (t_.pixel_grasp_enabled && tick.in.pixel_grasp_view && image_search) {
        if (!candidate(tick.in, tick.now)) {
            reason_ = "pixel_target_wait";
            return;
        }
        if (!accept_pixel_target())
            return;
        if (!tick.in.pixel_grasp_near_view) {
            if (updateSearchViewHandoff(tick))
                return;
            if (!pixelAlign(tick))
                return;
            commandFrame(1, tick.now);
            const int g = frameWait(tick.in, 1, tick.now);
            if (g < 0) {
                failStop(tick, "gripper_open_timeout");
                return;
            }
            if (g == 0) {
                reason_ = "gripper_feedback_wait";
                return;
            }
            reason_ = "pixel_center_approach_120";
            drive(tick, std::min(t_.approach_speed, kSearchUi120SpeedMps), 0);
            return;
        }
        if (!tick.in.target_grasp_range_valid || tick.in.target_grasp_range_past_rear) {
            enter(PushState::CAPTURE_FAIL, tick.now, "target_outside_grasp_center_range");
            return;
        }
        commandFrame(1, tick.now);
        const int entry_frame = frameWait(tick.in, 1, tick.now);
        if (entry_frame < 0) {
            failStop(tick, "gripper_open_timeout");
            return;
        }
        if (entry_frame == 0) {
            invalidateEntryPosition();
            reason_ = "gripper_feedback_wait";
            return;
        }
        if (search_entry_.review_before_entry) {
            tick.immediate_motion_stop = true;
            if (reviewFirstTripCapture(tick, "startup_reselected") ||
                search_phase_.first_trip_split_.pending)
                search_entry_.review_before_entry = false;
            return; // Even a clear review hands motion back on a later tick.
        }
        if (!locateSearchEntry(tick))
            return;
        // Keep an already-open search overlay open across the phase transition.
        commandFrame(1, tick.now);
        enter(PushState::PREPARE, tick.now, "pixel_target_prepare");
        return;
    }
    // Pixel PREPARE keeps UI140 throughout opening and inventories the image corridor.
    if (t_.pixel_grasp_enabled && search_phase_.action == SearchAction::Prepare) {
        const int p = pitchWait(tick.in, tick.now);
        if (p < 0) {
            failStop(tick, "camera_pitch_timeout");
            return;
        }
        if (p == 0) {
            reason_ = "camera_pitch_wait";
            return;
        }
        commandFrame(1, tick.now);
        const int g = frameWait(tick.in, 1, tick.now);
        if (g < 0) {
            failStop(tick, "gripper_open_timeout");
            return;
        }
        if (g == 0) {
            reason_ = "gripper_feedback_wait";
            return;
        }
        if (tick.elapsed > t_.prepare_budget_us) {
            enter(PushState::CAPTURE_FAIL, tick.now, "pixel_corridor_unresolved");
            return;
        }
        if (!tracking(tick.in)) {
            reason_ = "pixel_target_wait";
            actionProgress().confirmations = 0;
            return;
        }
        if (!locateSearchEntry(tick))
            return;
        // Inventory review starts only AFTER the selected target entered first; mixed boxes may enter.
        if (!reviewFirstTripCapture(tick, "search_positioned"))
            return;
        trip_context_.verdict_ = checkCorridor(tick.in.corridor,
                                               tick.in.corridor_complete,
                                               tick.in.corridor_occlusion_free,
                                               trip_context_.label_,
                                               first_);
        if (trip_context_.verdict_ == RuleVerdict::INCOMPLETE) {
            actionProgress().confirmations = 0;
            reason_ = "pixel_corridor_incomplete";
            return;
        }
        if (trip_context_.verdict_ != RuleVerdict::OK) {
            beginPixelSplit(tick);
            return;
        }
        if (trip_context_.seen_ != tick.in.corridor) {
            trip_context_.seen_ = tick.in.corridor;
            actionProgress().confirmations = 0;
        }
        if (++actionProgress().confirmations < t_.corridor_confirm_frames)
            return;
        trip_context_.pending_ = tick.in.corridor;
        search_entry_.handoff = true;
        search_entry_.accepted_us = tick.in.pixel_grasp_sample_us;
        requestPhase(PhaseOutcome::CaptureReady, "pixel_first_entry_inventory_ok");
        return;
    }
    switch (search_phase_.action) {
        case SearchAction::Scan:
        case SearchAction::LostSearch:
        case SearchAction::MidReacquire:
        case SearchAction::NearReacquire:
        case SearchAction::SelectCargo:
        case SearchAction::MidApproach:
        case SearchAction::Approach: {
            SearchContext context{search_phase_,
                                  actionProgress(),
                                  trip_context_,
                                  tick.motion,
                                  reason_};
            context.now_us = tick.now;
            context.elapsed_us = tick.elapsed;
            context.scan_ok = tick.scan_ok;
            context.forward_ok = tick.forward_ok;
            context.mid_pitch = midPitch();
            context.trip_empty = trip_context_.trip_.total() == 0;
            context.sweep_open = sweep_open_;
            context.first_ordinary = first_;
            context.frame_raised = frame_raised_;

            SearchServices services;
            services.candidate = [&](const PushObservation& obs, uint64_t at) {
                return candidate(obs, at);
            };
            services.drive = [&](float vx, float wz) {
                drive(tick, vx, wz);
            };
            services.scan_spin = [&] {
                scanSpin(tick);
            };
            services.unstick = [&] {
                return unstick(tick);
            };
            services.enter = [&](SearchAction next, uint64_t at, const char* why) {
                enterSearch(next, at, why);
            };
            services.fail_stop = [&](const char* why) {
                failStop(tick, why);
            };
            services.scan_after_clear = [&](const char* why) {
                scanAfterClear(tick, why);
            };
            services.command_frame = [&](uint8_t open, uint64_t at) {
                commandFrame(open, at);
            };
            services.search_lost = [&] {
                searchLost(tick);
            };

            context.pitch_cmd = pitch_cmd_;

            services.frame_wait = [&](const PushObservation& o, uint8_t open, uint64_t at) {
                return frameWait(o, open, at);
            };
            services.clear_trip = [&] {
                clearTrip();
            };
            services.blacklist = [&](uint64_t at) {
                blacklist(at);
            };
            services.lost_target = [&] {
                return lostTarget(tick);
            };
            services.tracking = [&](const PushObservation& o) {
                return tracking(o);
            };
            services.tracking_reason = [&] {
                return trackingReason(tick);
            };
            services.approach_heading_shift = [&](const PushObservation& o) {
                return approachHeadingShift(o);
            };
            services.desired_pitch = [&]() -> int16_t {
                return desiredPitch();
            };
            services.mid_pitch = [&] {
                return midPitch();
            };
            services.steer = [&](float speed, float heading) {
                steer(tick, speed, heading);
            };

            search_phase_.search_.update(search_phase_.action, t_, tick.in, context, services);
            break;
        }
        case SearchAction::Prepare: {
            // Stopped: open the frame, then inventory everything the rush would sweep.
            commandFrame(1, tick.now);
            const int p = pitchWait(tick.in, tick.now);
            if (p < 0) {
                fail("camera_pitch_timeout");
                break;
            }
            if (p == 0) {
                reason_ = "camera_pitch_wait";
                break;
            }
            const int g = frameWait(tick.in, 1, tick.now);
            // NEAR is a stationary opening check, not a calibrated driving view.
            // Do not require ground geometry while observing at 40 degrees.
            if (actionProgress().step == 0) {
                if (g < 0) {
                    fail("gripper_open_timeout");
                    break;
                }
                if (g == 0) {
                    reason_ = "gripper_feedback_wait";
                    break;
                }
                actionProgress().step = 1;
                actionProgress().started_us = tick.now;
                actionProgress().confirmations = 0;
                actionProgress().misses = 0;
                reason_ = "open_confirmed_return_to_tracking_pitch";
                break;
            }
            if (!tracking(tick.in)) {
                reason_ = trackingReason(tick);
                // TEMP_ASSUMPTION：方框张开后跟踪器会给同一货物发新ID；同标签有效货物直接改绑，不退回SCAN重来。
                const bool near_last =
                    t_.assume_all_safe ||
                    (last_target_valid_ && std::isfinite(tick.in.distance_m) &&
                     std::isfinite(tick.in.heading_error) &&
                     cv::norm(targetBody(tick) - last_target_xy_) <= t_.rebind_max_shift_m);
                if (rushLogic() && tick.in.target_valid &&
                    tick.in.target_id != trip_context_.target_id_ &&
                    tick.in.label == trip_context_.label_ &&
                    ((candidate(tick.in, tick.now) && near_last) ||
                     sameObject(tick.in, tick.now))) {
                    trip_context_.target_id_ = trip_context_.capture_target_id_ = tick.in.target_id;
                    actionProgress().misses = 0;
                    reason_ = "prepare_id_rebound_after_open";
                    break;
                }
                if (lostTarget(tick)) {
                    scanAfterClear(tick, "target_lost");
                }
                break;
            }
            actionProgress().misses = 0;
            if (g < 0) {
                fail("gripper_open_timeout");
                break;
            }
            if (tick.elapsed > t_.prepare_budget_us) {
                blacklist(tick.now);
                scanAfterClear(tick, "corridor_unresolved");
                break;
            }
            if (g == 0) {
                reason_ = "gripper_feedback_wait";
                break;
            }
            if (t_.assume_all_safe) {
                // TEMP_ASSUMPTION: an unresolved corridor counts as holding only the locked target.
                if (++actionProgress().confirmations < t_.confirm_frames) {
                    reason_ = "assumed_corridor_confirm";
                    break;
                }
                Inventory only;
                only.add(trip_context_.label_);
                const bool seen_ok = checkCorridor(tick.in.corridor,
                                                   tick.in.corridor_complete,
                                                   tick.in.corridor_occlusion_free,
                                                   trip_context_.label_,
                                                   first_) == RuleVerdict::OK;
                trip_context_.pending_ = seen_ok ? tick.in.corridor : only;
                trip_context_.verdict_ = RuleVerdict::OK;
                requestPhase(PhaseOutcome::CaptureReady,
                             seen_ok ? "corridor_ok" : "assumed_corridor_ok");
                assume_rush_end_ = std::max(0.f, tick.target_forward_m - t_.assume_rush_trigger_m) *
                                   t_.assume_motion_ratio;
                actionProgress().travel_limit =
                    assume_rush_end_ + t_.rush_extra_m * t_.assume_motion_ratio;
                break;
            }
            const auto verdict = checkCorridor(tick.in.corridor,
                                               tick.in.corridor_complete,
                                               tick.in.corridor_occlusion_free,
                                               trip_context_.label_,
                                               first_);
            trip_context_.verdict_ = verdict;
            if (verdict == RuleVerdict::INCOMPLETE) {
                actionProgress().confirmations = 0;
                break;
            }
            if (actionProgress().confirmations == 0 || tick.in.corridor != trip_context_.seen_) {
                trip_context_.seen_ = tick.in.corridor;
                actionProgress().confirmations = 0;
            }
            if (++actionProgress().confirmations < t_.corridor_confirm_frames)
                break;
            if (verdict != RuleVerdict::OK) {
                const std::string why =
                    std::string("corridor_") + verdictName(verdict) + "_back_off";
                const bool fatal = verdict == RuleVerdict::DANGEROUS ||
                                   verdict == RuleVerdict::CORE_BEFORE_FIRST ||
                                   verdict == RuleVerdict::INJURED_BEFORE_FIRST ||
                                   verdict == RuleVerdict::INJURED_NOT_ALONE;
                if (fatal || ++search_phase_.corridor_retries_ > t_.corridor_retry_max) {
                    blacklist(tick.now);
                    search_phase_.corridor_retries_ = 0;
                }
                trip_context_.verdict_ = verdict;
                enter(PushState::LOST_HOLD, tick.now, why.c_str());
                corridor_retreat_ = true;
                break;
            }
            search_phase_.corridor_retries_ = 0;
            trip_context_.pending_ = tick.in.corridor;
            requestPhase(PhaseOutcome::CaptureReady, "corridor_ok");
            if (rushLogic()) {
                assume_rush_end_ = std::max(0.f, tick.target_forward_m - t_.assume_rush_trigger_m) *
                                   t_.assume_motion_ratio;
                actionProgress().travel_limit =
                    assume_rush_end_ + t_.rush_extra_m * t_.assume_motion_ratio;
            } else
                actionProgress().travel_limit =
                    std::max(0.f, tick.target_forward_m - t_.grasp_trigger_y_m) + t_.rush_extra_m;
            break;
        }
        case SearchAction::TurnAway: {
            commandFrame(0, tick.now);
            // A turn is complete only with measured IMU heading changes.
            if (tick.in.heading_valid && std::isfinite(tick.in.heading_rad)) {
                if (actionProgress().heading_seen)
                    actionProgress().turned +=
                        std::abs(wrapAngle(tick.in.heading_rad - actionProgress().last_heading));
                actionProgress().last_heading = tick.in.heading_rad;
                actionProgress().heading_seen = true;
            } else
                actionProgress().heading_seen = false;
            if (actionProgress().turned >= t_.turn_min_rad) {
                if (t_.demo_carry_once) {
                    failStop(tick,
                             transport_phase_.delivery_visual_ ? "demo_next_trip_ready"
                                                               : "demo_next_trip_unverified");
                    break;
                }
                sweep_ledger_.newTrip(
                    t_, tick.now); // a new trip starts with a fresh per-trip sweep
                                   // ledger (the match-wide time ledger keeps running)
                scanAfterClear(tick, "");
                break;
            }
            if (tick.elapsed > t_.turn_budget_us) {
                fail("turn_heading_unverified");
                break;
            }
            drive(tick, 0, t_.turn_wz);
            break;
        }
        default:
            break;
    }
}
} // namespace rescue
