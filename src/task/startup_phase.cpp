#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

namespace rescue {
int PushTask::startupSightTargetId() const {
    if (phase_ != TaskPhase::Startup)
        return -1;
    // The far-view object is a guide, not cargo selected for capture. Retain
    // its soft image preference, but allow another visible guide immediately
    // when it leaves the image during a forward arc.
    if (startup_action_ == StartupAction::Approach && startup_pitch_ != kCameraPitchInvalid &&
        std::abs(int(t_.far_pitch_cdeg) - int(t_.track_pitch_cdeg)) > t_.pitch_tolerance_cdeg &&
        std::abs(int(startup_pitch_) - int(t_.far_pitch_cdeg)) <= t_.pitch_tolerance_cdeg)
        return -1;
    if (startup_action_ == StartupAction::Approach ||
        startup_action_ == StartupAction::OpenApproach ||
        (startup_action_ == StartupAction::SelectOrdinary &&
         trip_context_.label_ == "ordinary_supply"))
        return trip_context_.target_id_;
    return -1;
}

bool PushTask::startupPileApproach() const {
    return phase_ == TaskPhase::Startup && startup_action_ == StartupAction::Approach;
}

bool PushTask::startupOrdinaryOnly() const {
    return phase_ == TaskPhase::Startup &&
           (startup_action_ == StartupAction::SelectOrdinary ||
            startup_action_ == StartupAction::OpenApproach);
}

void PushTask::updateStartupPhase(TaskTick& tick) {
    if (!t_.startup_advance_us) {
        requestPhase(PhaseOutcome::SearchReady, "startup_disabled");
        return;
    }
    if (t_.start_zone < 1 || t_.start_zone > 4) {
        failStop(tick, "startup_zone_missing");
        return;
    }
    const float open_tolerance_px = t_.pixel_align_approach_tolerance_px;
    const float direction = t_.start_zone == 2 || t_.start_zone == 3 ? -1.f : 1.f;
    // Gentle zone-side preference while advancing toward the pile. Visual
    // corrections still steer back when needed; grasp alignment stays centered.
    const float forward_bias = direction * .05f;
    const auto& in = tick.in;
    if (startup_action_ != StartupAction::Approach)
        startup_pile_alignment_ = {};
    // Commit the UI120 -> UI140 handoff before the ordinary alignment watchdog.
    // Box jitter around the line cannot restart turning or forward motion.
    const bool fresh = in.target_valid && in.target_center_px_valid &&
                       in.pixel_grasp_sample_us && in.pixel_grasp_sample_us <= tick.now &&
                       tick.now - in.pixel_grasp_sample_us <= t_.frame_timeout_us;
    if (startup_action_ == StartupAction::OpenApproach && !startup_handoff_.pending &&
        fresh && in.pixel_grasp_view && !in.pixel_grasp_near_view &&
        in.pixel_grasp_target_valid && in.pixel_view_handoff_ready &&
        in.camera_pitch_stable && std::isfinite(in.pixel_grasp_error_px) &&
        in.label == "ordinary_supply" &&
        (in.target_id == trip_context_.target_id_ ||
         in.startup_rebound_from == trip_context_.target_id_) &&
        std::abs(int(in.camera_pitch_cdeg) - int(t_.track_pitch_cdeg)) <= t_.pitch_tolerance_cdeg) {
        startup_handoff_ = {};
        startup_handoff_.pending = startup_handoff_.stopping = true;
        startup_handoff_.reference = {in.target_id, in.label, in.target_box,
                                      in.camera_pitch_cdeg, in.pixel_grasp_sample_us};
        startup_handoff_.started_us = tick.now;
        trip_context_.target_id_ = trip_context_.capture_target_id_ = in.target_id;
        startup_sight_ = startup_handoff_.reference;
        startup_alignment_since_us_ = startup_target_missing_since_us_ = 0;
        startup_alignment_target_id_ = -1;
        pixel_alignment_ = {};
    }
    if (startup_handoff_.pending) {
        auto& handoff = startup_handoff_;
        tick.immediate_motion_stop = true;
        reason_ = "startup_handoff_observe";
        if (tick.now - handoff.started_us >= kStartupTargetLossHoldUs) {
            std::cout << "[STARTUP_HANDOFF] mode=region_selection"
                      << " reference_id=" << handoff.reference.id
                      << " candidate_id=" << handoff.candidate_id << " frames=" << handoff.frames
                      << " outcome=timeout\n";
            failStop(tick, handoff.stopping ? "startup_handoff_stop_timeout"
                                            : "startup_handoff_region_timeout");
            return;
        }
        if (handoff.stopping) {
            commandFrame(1, tick.now);
            const int opened = frameWait(in, 1, tick.now);
            if (opened < 0) {
                failStop(tick, "gripper_open_timeout");
                return;
            }
            if (!opened) {
                reason_ = "gripper_feedback_wait";
                return;
            }
            // Confirm the stop using scene freshness and sent motion.
            // Losing the old object must not prevent looking into the jaw region.
            const bool scene_fresh = in.startup_scene_us > handoff.started_us &&
                in.startup_scene_us <= tick.now &&
                tick.now - in.startup_scene_us <= t_.frame_timeout_us;
            if (!scene_fresh || !captureReviewStopped(in) || !in.camera_pitch_stable ||
                !in.pixel_grasp_view || in.pixel_grasp_near_view ||
                std::abs(int(in.camera_pitch_cdeg) - int(t_.track_pitch_cdeg)) >
                    t_.pitch_tolerance_cdeg) {
                handoff.frames = 0;
                handoff.sample_us = handoff.stable_since_us = 0;
                reason_ = "startup_handoff_stop_wait";
                return;
            }
            reason_ = "startup_handoff_stop_wait";
            if (in.startup_scene_us <= handoff.sample_us)
                return;
            if (handoff.sample_us && in.startup_scene_us - handoff.sample_us > t_.frame_timeout_us)
                handoff.frames = 0;
            handoff.sample_us = in.startup_scene_us;
            if (!handoff.frames)
                handoff.stable_since_us = tick.now;
            if (++handoff.frames < t_.pixel_align_stable_frames ||
                tick.now - handoff.stable_since_us < t_.pixel_align_stable_us)
                return;
            trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
            trip_context_.label_.clear();
            startup_sight_ = {};
            handoff.frames = 0;
            handoff.sample_us = handoff.stable_since_us = 0;
            handoff.stopping = false;
            handoff.started_us = tick.now;
            startup_pitch_ = t_.near_pitch_cdeg;
            pixel_alignment_ = {};
            reason_ = "startup_switch_to_140";
            return;
        }
        const bool scene_fresh = in.pixel_grasp_near_view && in.camera_pitch_stable &&
            in.capture_scene_valid && in.capture_scene_us > handoff.started_us &&
            in.capture_scene_us <= tick.now && tick.now - in.capture_scene_us <= t_.frame_timeout_us;
        if (!scene_fresh || !captureReviewStopped(in) || !in.gripper_done ||
            in.gripper_feedback_open != 1) {
            handoff.frames = 0;
            handoff.candidate_id = -1;
            return;
        }
        if (in.capture_scene_us <= handoff.sample_us)
            return;
        if (handoff.sample_us && in.capture_scene_us - handoff.sample_us > t_.frame_timeout_us) {
            handoff.frames = 0;
            handoff.candidate_id = -1;
        }
        handoff.sample_us = in.capture_scene_us;
        const auto area = in.capture_class_regions_px.find("ordinary_supply");
        const auto inside = [&](const auto& object) {
            return area != in.capture_class_regions_px.end() &&
                   PushObservation::imageBoxCenterInRegion(object.box, area->second);
        };
        const PushObservation::CaptureImageObject* candidate = nullptr;
        const PushObservation::CaptureImageObject* pending_candidate = nullptr;
        int candidate_priority = 0, pending_priority = 0;
        int enclosed_count = 0, observation_count = 0;
        std::ostringstream candidates;
        for (const auto& object : in.capture_image_objects) {
            if (object.id < 0 || object.box.empty())
                continue;
            const bool ordinary = object.label == "ordinary_supply";
            const bool in_region = ordinary && inside(object);
            const bool in_observation = ordinary && area != in.capture_class_regions_px.end() &&
                !area->second.empty() &&
                PushObservation::imageBoxCenterInRegion(object.box, in.layout_count_region_px);
            if ((in_region || in_observation) && object.complete && object.selectable) {
                enclosed_count += in_region;
                observation_count += in_observation && !in_region;
                // Prefer cargo already in the calibrated jaw area. The observation
                // strip only acquires a target for the existing UI140 entry approach.
                const int priority = in_region ? 2 : 1;
                if (object.id == handoff.candidate_id) {
                    pending_candidate = &object;
                    pending_priority = priority;
                }
                const float y = object.box.y + .5f * object.box.height;
                const float best_y = candidate ? candidate->box.y + .5f * candidate->box.height : -1.f;
                if (priority > candidate_priority ||
                    (priority == candidate_priority &&
                     (!candidate || y > best_y || (y == best_y && object.id < candidate->id)))) {
                    candidate = &object;
                    candidate_priority = priority;
                }
            }
            candidates << object.id << ':' << object.label << ":inside=" << in_region
                       << ":observation=" << in_observation << ":complete=" << object.complete
                       << ":selectable=" << object.selectable << ';';
        }
        // Retain a candidate within the preferred region. Tracker ID changes
        // do not restart the region-presence confirmation.
        const auto* match = pending_candidate && pending_priority == candidate_priority
                                ? pending_candidate : candidate;
        trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
        trip_context_.label_.clear();
        startup_sight_ = {};
        pixel_alignment_ = {};
        const auto log = [&](const char* outcome) {
            std::cout << "[STARTUP_HANDOFF] scene_us=" << in.capture_scene_us
                      << " mode=region_selection"
                      << " region_candidates=" << enclosed_count
                      << " observation_candidates=" << observation_count
                      << " candidate_id=" << handoff.candidate_id << " frames=" << handoff.frames
                      << " outcome=" << outcome << " candidates=[" << candidates.str() << "]\n";
        };
        if (!match) {
            handoff.frames = 0;
            handoff.candidate_id = -1;
            reason_ = "startup_handoff_reselect_wait";
            log("no_complete_candidate_in_region");
            return;
        }
        handoff.candidate_id = match->id;
        if (++handoff.frames < 3) {
            reason_ = "startup_handoff_reselect_observe";
            log("confirming");
            return;
        }
        log("target_reselected");
        trip_context_.target_id_ = trip_context_.capture_target_id_ = match->id;
        trip_context_.label_ = match->label;
        startup_sight_ = {match->id, match->label, match->box,
                          in.camera_pitch_cdeg, in.capture_scene_us};
        startup_alignment_target_id_ = -1;
        startup_alignment_since_us_ = startup_target_missing_since_us_ = 0;
        pixel_alignment_ = {};
        trip_context_.pending_ = {};
        capture_scene_ = {};
        capture_load_ = {};
        search_entry_ = {};
        startup_first_capture_ = true;
        trip_context_.pending_.ordinary = 1;
        handoff = {};
        // Stay in Startup for UI140 centering and approach. Actual held contents
        // decide ordinary quantity after closure, not the larger observation strip.
        reason_ = "startup_near_target_selected";
        return;
    }
    const auto turn = [&] {
        drive(tick, 0, direction * .3f);
        // Keep clearance/cap decisions from drive, without its legacy .4 rad/s floor.
        tick.motion.wz_rps = std::clamp(tick.motion.wz_rps, -.3f, .3f);
    };
    // Keep UI120 and the closed frame through a measured 0.1 m advance, then
    // two IMU half turns separated by a measured 0.1 m reverse. Once committed,
    // image-target loss cannot interrupt this sequence.
    if (startup_action_ == StartupAction::TurnBeforePitch ||
        startup_action_ == StartupAction::StopBeforePitch) {
        tick.immediate_motion_stop = true;
        commandFrame(0, tick.now);
        const int closed = frameWait(in, 0, tick.now);
        if (closed <= 0) {
            if (closed < 0)
                failStop(tick, "startup_gripper_close_timeout");
            else
                reason_ = "startup_gripper_close_wait";
            startup_pitch_turn_.heading_valid = false;
            startup_pitch_turn_.stopped_us = 0;
            return;
        }
        auto& turn_state = startup_pitch_turn_;
        if (!in.heading_valid || !std::isfinite(in.heading_rad)) {
            turn_state.heading_valid = false;
            turn_state.stopped_us = 0;
            reason_ = "startup_pitch_turn_heading_wait";
            return;
        }
        using Leg = StartupPitchTurn::Leg;
        const float sent_vx = in.sent_velocity_valid ? in.sent_vx_mps : last_vx_;
        const float sent_wz = in.sent_velocity_valid ? in.sent_wz_rps : last_wz_;
        const bool stopped = std::isfinite(sent_vx) && std::isfinite(sent_wz) &&
            std::abs(sent_vx) <= t_.pixel_align_stopped_vx &&
            std::abs(sent_wz) <= t_.pixel_align_stopped_wz;
        if (turn_state.leg == Leg::InitialStop) {
            reason_ = "startup_corner_stop";
            if (!stopped) {
                turn_state.stopped_us = 0;
                return;
            }
            if (!turn_state.stopped_us)
                turn_state.stopped_us = tick.now;
            if (tick.now - turn_state.stopped_us < 250000)
                return;
            turn_state.leg = Leg::ForwardCalibrate;
            turn_state.translation_heading = in.heading_rad;
            turn_state.translation_imu.reset();
            startup_action_ = StartupAction::StopBeforePitch;
            turn_state.heading_valid = false;
        }
        if (turn_state.leg == Leg::Forward || turn_state.leg == Leg::Reverse) {
            const bool forward = turn_state.leg == Leg::Forward;
            const float translation_direction = forward ? 1.f : -1.f;
            if (!in.sent_velocity_valid || !std::isfinite(sent_vx) ||
                !std::isfinite(sent_wz) ||
                translation_direction * sent_vx < -t_.pixel_align_stopped_vx) {
                failStop(tick, forward ? "startup_corner_forward_feedback_missing"
                                       : "startup_pitch_reverse_feedback_missing");
                return;
            }
            const bool valid = std::abs(sent_vx) < .001f
                ? turn_state.translation_imu.holdStationary(in.imu_translation_samples, tick.now)
                : turn_state.translation_imu.advance(in.imu_translation_samples, tick.now);
            if (!valid) {
                failStop(tick, forward ? "startup_corner_forward_imu_gap"
                                       : "startup_pitch_reverse_imu_gap");
                return;
            }
            const float remaining = .1f - turn_state.translation_completed_m -
                                    translation_direction * turn_state.translation_imu.distance();
            if (remaining > 0) {
                reason_ = forward ? "startup_corner_forward_010m" : "startup_pitch_reverse_010m";
                const float speed_cap = forward ? t_.approach_speed : t_.back_speed;
                drive(tick, translation_direction *
                            std::min(speed_cap, std::max(.01f, 2.f * remaining)), 0);
                tick.immediate_motion_stop = tick.motion.vx_mps == 0;
                return;
            }
            turn_state.leg = forward ? Leg::ForwardStop : Leg::ReverseStop;
            turn_state.stopped_us = 0;
        }
        if (startup_action_ == StartupAction::TurnBeforePitch) {
            if (turn_state.heading_valid)
                turn_state.angle += direction * wrapAngle(in.heading_rad - turn_state.last_heading);
            turn_state.last_heading = in.heading_rad;
            turn_state.heading_valid = true;
            if (turn_state.angle < float(CV_PI)) {
                reason_ = turn_state.leg == Leg::SecondTurn
                              ? "startup_pitch_second_turn_180" : "startup_pitch_turn_180";
                drive(tick, 0, direction * 1.f);
                return;
            }
            startup_action_ = StartupAction::StopBeforePitch;
            turn_state.stopped_us = 0;
        }
        reason_ = turn_state.leg == Leg::ForwardStop ? "startup_corner_forward_stop"
                  : turn_state.leg == Leg::ReverseStop ? "startup_pitch_reverse_stop"
                                                       : "startup_pitch_turn_stop";
        if (!stopped) {
            turn_state.stopped_us = 0;
            turn_state.translation_imu.reset();
            return;
        }
        if (!turn_state.stopped_us)
            turn_state.stopped_us = tick.now;
        if (tick.now - turn_state.stopped_us < 250000)
            return;
        if (turn_state.leg == Leg::ForwardStop) {
            turn_state.leg = Leg::FirstTurn;
            turn_state.angle = 0;
            turn_state.heading_valid = false;
            turn_state.stopped_us = 0;
            startup_action_ = StartupAction::TurnBeforePitch;
            reason_ = "startup_pitch_first_turn_ready";
            return;
        }
        if (turn_state.leg == Leg::FirstTurn) {
            turn_state.translation_completed_m = 0;
            turn_state.leg = Leg::ReverseCalibrate;
            turn_state.translation_heading = in.heading_rad;
            turn_state.translation_imu.reset();
        }
        if (turn_state.leg == Leg::ForwardCalibrate || turn_state.leg == Leg::ReverseCalibrate) {
            const bool forward = turn_state.leg == Leg::ForwardCalibrate;
            reason_ = forward ? "startup_corner_forward_calibrate" : "startup_pitch_reverse_calibrate";
            // The 22:54 run saw post-turn gyro noise around 0.10 rad/s.
            // Keep the stopped-command dwell and fresh IMU checks, but do not
            // require capture's much quieter 0.05 rad/s / 0.05 m/s² window.
            // Average 500 ms of samples to reduce the noise in the bias estimate.
            if (turn_state.translation_imu.calibrate(in.imu_translation_samples, tick.now,
                                                     turn_state.translation_heading, .15f, .30f, 500000)) {
                turn_state.leg = forward ? Leg::Forward : Leg::Reverse;
            } else {
                using Status = ImuTranslation::CalibrationStatus;
                switch (turn_state.translation_imu.calibrationStatus()) {
                    case Status::AngularMotion:
                        reason_ = forward ? "startup_corner_forward_calibrate_gyro"
                                          : "startup_pitch_reverse_calibrate_gyro";
                        break;
                    case Status::AccelerationNoise:
                        reason_ = forward ? "startup_corner_forward_calibrate_accel"
                                          : "startup_pitch_reverse_calibrate_accel";
                        break;
                    case Status::InvalidSample:
                        reason_ = forward ? "startup_corner_forward_calibrate_invalid_sample"
                                          : "startup_pitch_reverse_calibrate_invalid_sample";
                        break;
                    case Status::SampleGap:
                        reason_ = forward ? "startup_corner_forward_calibrate_sample_gap"
                                          : "startup_pitch_reverse_calibrate_sample_gap";
                        break;
                    case Status::Collecting:
                    case Status::Ready:
                        break;
                }
            }
            return;
        }
        if (turn_state.leg == Leg::ReverseStop) {
            turn_state.leg = Leg::SecondTurn;
            turn_state.angle = 0;
            turn_state.last_heading = in.heading_rad;
            turn_state.heading_valid = true;
            turn_state.stopped_us = 0;
            startup_action_ = StartupAction::TurnBeforePitch;
            reason_ = "startup_pitch_second_turn_ready";
            return;
        }
        // Search for the ordinary in UI120 after the maneuver; UI140 is only
        // requested later when the open approach reaches the usual handoff line.
        startup_pitch_ = t_.track_pitch_cdeg;
        startup_action_ = StartupAction::SelectOrdinary;
        trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
        trip_context_.label_.clear();
        startup_sight_ = {};
        startup_alignment_since_us_ = startup_target_missing_since_us_ = 0;
        startup_alignment_target_id_ = -1;
        pixel_alignment_ = {};
        reason_ = "startup_track_observe";
        return;
    }
    const bool visible = in.startup_scene_us && in.startup_scene_us <= tick.now &&
                         tick.now - in.startup_scene_us <= t_.frame_timeout_us &&
                         (in.startup_visible_classes & 15u);
    // An existing ordinary lock wins. Without one, a sole enclosed ordinary
    // may start capture; detector iteration order never chooses among neighbours.
    const auto ordinary_region = in.capture_class_regions_px.find("ordinary_supply");
    if (!startup_first_capture_ && in.pixel_grasp_near_view && in.camera_pitch_stable &&
        in.capture_scene_valid &&
        in.capture_scene_us && in.capture_scene_us <= tick.now &&
        tick.now - in.capture_scene_us <= t_.frame_timeout_us &&
        ordinary_region != in.capture_class_regions_px.end() && !ordinary_region->second.empty()) {
        const auto& area = ordinary_region->second;
        const PushObservation::CaptureImageObject* selected = nullptr;
        const PushObservation::CaptureImageObject* retained = nullptr;
        int count = 0;
        for (const auto& object : in.capture_image_objects) {
            if (object.label != "ordinary_supply" || object.id < 0 || object.box.empty()) continue;
            const cv::Point2f center(object.box.x + .5f * object.box.width,
                                     object.box.y + .5f * object.box.height);
            if (center.x < area.x || center.x > area.br().x ||
                center.y < area.y || center.y > area.br().y) continue;
            selected = &object;
            ++count;
            if (trip_context_.label_ == "ordinary_supply" && object.id == trip_context_.target_id_)
                retained = &object;
        }
        if (retained) selected = retained;
        else if (count != 1 || trip_context_.label_ == "ordinary_supply") selected = nullptr;
        if (selected) {
            trip_context_.target_id_ = trip_context_.capture_target_id_ = selected->id;
            trip_context_.label_ = "ordinary_supply";
            startup_action_ = StartupAction::OpenApproach;
            pixel_alignment_ = {};
            commandFrame(1, tick.now);
            tick.immediate_motion_stop = true;
            startup_first_capture_ = true;
            startup_sight_ = {selected->id, selected->label, selected->box,
                              in.camera_pitch_cdeg, in.capture_scene_us};
            capture_scene_ = {};
            search_entry_ = {};
            trip_context_.pending_ = {};
            trip_context_.pending_.ordinary = 1;
            reason_ = "startup_near_target_selected";
            return;
        }
    }
    // The new near view chooses an ordinary independently of the closed guide.
    if (startup_action_ == StartupAction::Approach && in.pixel_grasp_near_view) {
        startup_action_ = StartupAction::SelectOrdinary;
        trip_context_.target_id_ = -1;
        trip_context_.label_.clear();
        pixel_alignment_ = {};
        startup_alignment_since_us_ = startup_target_missing_since_us_ = 0;
        startup_alignment_target_id_ = -1;
        tick.immediate_motion_stop = true;
        reason_ = "startup_near_observe";
        return;
    }
    // UI90 handoff is a committed stop, not another target approach. A clipped
    // box can move back above the line or acquire a new ID while braking.
    if (startup_action_ == StartupAction::Approach && fresh &&
        in.pixel_view_handoff_ready &&
        std::abs(int(in.camera_pitch_cdeg) - int(t_.far_pitch_cdeg)) <= t_.pitch_tolerance_cdeg) {
        startup_action_ = StartupAction::FarHandoff;
        trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
        trip_context_.label_.clear();
        startup_target_missing_since_us_ = startup_alignment_since_us_ = 0;
        startup_alignment_target_id_ = -1;
    }
    // UI120 uses the whole scene's two lower corners, independent of the
    // selected guide's centre/identity. Commit before target-loss/alignment waits.
    if (startup_action_ == StartupAction::Approach && visible &&
        in.startup_lower_corners_occupied && in.camera_pitch_stable &&
        std::abs(int(in.camera_pitch_cdeg) - int(t_.track_pitch_cdeg)) <= t_.pitch_tolerance_cdeg) {
        startup_pitch_turn_ = {};
        startup_pitch_turn_.leg = StartupPitchTurn::Leg::InitialStop;
        startup_action_ = StartupAction::TurnBeforePitch;
        trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
        trip_context_.label_.clear();
        startup_sight_ = {};
        startup_target_missing_since_us_ = startup_alignment_since_us_ = 0;
        startup_alignment_target_id_ = -1;
        pixel_alignment_ = {};
        tick.immediate_motion_stop = true;
        reason_ = "startup_pitch_turn_ready";
        return;
    }
    // One UI120 pile episode survives tracker gaps, brake jitter and short
    // forward pulses. Only sustained aligned forward travel ends the episode.
    const bool pile_view = startup_action_ == StartupAction::Approach &&
        (in.target_id == kStartupPileTargetId ||
         trip_context_.target_id_ == kStartupPileTargetId ||
         startup_pile_alignment_.started_us) &&
        in.camera_pitch_cdeg != kCameraPitchInvalid &&
        std::abs(int(in.camera_pitch_cdeg) - int(t_.track_pitch_cdeg)) <= t_.pitch_tolerance_cdeg;
    if (pile_view) {
        auto& watch = startup_pile_alignment_;
        if (!watch.started_us) {
            watch.started_us = watch.progress_us = tick.now;
        }
        if (fresh && in.pixel_grasp_sample_us > watch.sample_us) {
            watch.sample_us = in.pixel_grasp_sample_us;
            watch.errors[watch.samples++ % watch.errors.size()] = std::abs(in.pixel_grasp_error_px);
            if (watch.samples >= watch.errors.size()) {
                const float worst = *std::max_element(watch.errors.begin(), watch.errors.end());
                if (watch.best_error_px < 0 || worst <= watch.best_error_px - 10.f) {
                    watch.best_error_px = worst;
                    watch.progress_us = tick.now;
                }
            }
        }
        if (tick.now - watch.started_us >= 6000000) {
            tick.immediate_motion_stop = true;
            failStop(tick, "startup_pile_alignment_timeout");
            return;
        }
        if (watch.reobserve_until_us) {
            tick.immediate_motion_stop = true;
            reason_ = "startup_pile_reobserve";
            if (tick.now < watch.reobserve_until_us || !fresh || !captureReviewStopped(in))
                return;
            watch.reobserve_until_us = 0;
            watch.progress_us = tick.now;
            pixel_alignment_ = {};
            return;
        }
        const bool forward = fresh && in.sent_velocity_valid &&
            std::isfinite(in.sent_vx_mps) && in.sent_vx_mps > t_.pixel_align_stopped_vx &&
            std::abs(in.pixel_grasp_error_px) <= t_.pixel_align_approach_restart_px;
        if (forward) {
            if (!watch.forward_us) watch.forward_us = tick.now;
            if (tick.now - watch.forward_us >= 300000)
                watch = {};
        } else
            watch.forward_us = 0;
        if (watch.started_us && tick.now - watch.progress_us >= 3000000) {
            if (watch.reviewed) {
                tick.immediate_motion_stop = true;
                failStop(tick, "startup_pile_alignment_stalled");
                return;
            }
            watch.reviewed = true;
            watch.reobserve_until_us = tick.now + 300000;
            pixel_alignment_ = {};
            tick.immediate_motion_stop = true;
            reason_ = "startup_pile_reobserve";
            return;
        }
    } else
        startup_pile_alignment_ = {};
    int locked_id = startupSightTargetId();
    // The adapter accepts only a unique overlapping same-class box in the same view.
    // Preserve alignment and its timeout across a tracker re-ID of this physical target.
    if (locked_id >= 0 && fresh && in.startup_rebound_from == locked_id &&
        in.label == trip_context_.label_) {
        trip_context_.target_id_ = trip_context_.capture_target_id_ = in.target_id;
        locked_id = startup_alignment_target_id_ = in.target_id;
    }
    const bool new_sight_sample = fresh && in.pixel_grasp_sample_us > startup_sight_.sample_us &&
                                  (locked_id < 0 || in.target_id == locked_id);
    if (fresh && (locked_id < 0 || in.target_id == locked_id))
        startup_sight_ = {in.target_id, in.label, in.target_box,
                          in.camera_pitch_cdeg, in.pixel_grasp_sample_us};
    const auto release = [&](const char* reason, bool stalled) {
        if (stalled) {
            startup_rejected_id_ = locked_id;
            startup_rejected_until_us_ = tick.now + 1000000;
        }
        trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
        trip_context_.label_.clear();
        startup_first_capture_ = false;
        capture_scene_ = {};
        startup_sight_ = {};
        pixel_alignment_ = {};
        startup_alignment_best_error_px_ = -1;
        startup_alignment_error_count_ = 0;
        startup_target_missing_since_us_ = startup_alignment_since_us_ = 0;
        startup_alignment_target_id_ = -1;
        // A lost visual lock releases the target, not the jaw. Once opened,
        // keep it open while selecting and aligning another ordinary cargo.
        if (startup_action_ == StartupAction::OpenApproach)
            startup_action_ = StartupAction::SelectOrdinary;
        tick.immediate_motion_stop = true;
        reason_ = reason;
    };
    const bool locked_sample = fresh && in.target_id == locked_id &&
                               std::isfinite(in.pixel_grasp_error_px);
    const float alignment_error = std::abs(in.pixel_grasp_error_px);
    const bool ordinary_follow = startup_action_ == StartupAction::SelectOrdinary ||
                                 startup_action_ == StartupAction::OpenApproach;
    const auto resetAlignmentProgress = [&] {
        startup_alignment_since_us_ = tick.now;
        startup_alignment_best_error_px_ = !ordinary_follow && locked_sample ? alignment_error : -1;
        startup_alignment_error_count_ = 0;
    };
    if (locked_id != startup_alignment_target_id_) {
        startup_alignment_target_id_ = locked_id;
        resetAlignmentProgress();
    }
    // Jaw acknowledgement has its own timeout. Start alignment evidence only
    // after that acknowledgement, so mechanical waiting spends no correction budget.
    const bool waiting_for_open = startup_action_ == StartupAction::OpenApproach &&
                                  frameWait(in, 1, tick.now) != 1;
    if (waiting_for_open) {
        startup_alignment_since_us_ = 0;
        startup_alignment_best_error_px_ = -1;
        startup_alignment_error_count_ = 0;
    }
    if (locked_id >= 0 && locked_id != kStartupPileTargetId && !waiting_for_open) {
        const float sent_vx = in.sent_velocity_valid ? in.sent_vx_mps : last_vx_;
        if (sent_vx > t_.pixel_align_stopped_vx || !startup_alignment_since_us_)
            resetAlignmentProgress();
        // A new correction after advancing starts at the current error, not the
        // tiny minimum accepted before opening. Brake jitter is not progress.
        if (ordinary_follow && locked_sample && pixel_alignment_.stage == PixelAlignment::Stage::Advance &&
            alignment_error > t_.pixel_align_approach_restart_px)
            resetAlignmentProgress();
        if (new_sight_sample && locked_sample && !ordinary_follow &&
            (startup_alignment_best_error_px_ < 0 || startup_alignment_best_error_px_ -
             alignment_error >= t_.pixel_align_brake_motion_px))
            resetAlignmentProgress();
        if (new_sight_sample && locked_sample && ordinary_follow) {
            const float tolerance = startup_action_ == StartupAction::SelectOrdinary
                                        ? open_tolerance_px
                                        : in.pixel_grasp_near_view ? t_.pixel_align_tolerance_px
                                                                  : t_.pixel_align_approach_tolerance_px;
            // Track only the error outside acceptance. Once inside, progress is
            // the existing stopped-image confirmation, not another 10 px reduction.
            auto& errors = startup_alignment_errors_;
            errors[startup_alignment_error_count_++ % errors.size()] =
                std::max(0.f, alignment_error - tolerance);
            if (startup_alignment_error_count_ >= errors.size()) {
                const float worst = *std::max_element(errors.begin(), errors.end());
                // Establish the correction baseline once, including drift before
                // the first Advance. Brake/Settle/Correct cycles cannot re-arm it.
                const bool first_correction = startup_alignment_best_error_px_ < 0 && worst > 0;
                const bool improved = startup_alignment_best_error_px_ - worst >= 10.f;
                const bool entered_band = startup_alignment_best_error_px_ > 0 && worst == 0;
                if (first_correction || improved || entered_band) {
                    startup_alignment_best_error_px_ = worst;
                    startup_alignment_since_us_ = tick.now;
                }
            }
        }
        const uint64_t alignment_deadline = startup_alignment_since_us_ + 3000000;
        // A short in-band sighting can set the progress floor to zero before
        // post-open drift. Let an already-started stopped confirmation finish
        // across that deadline, using only its existing image/freshness budget.
        // Returning to Brake/Correct or exceeding this bound releases the target.
        const bool finishing_confirmation = ordinary_follow && locked_sample &&
            pixel_alignment_.stage == PixelAlignment::Stage::Settle &&
            pixel_alignment_.stable_since_us &&
            pixel_alignment_.stable_since_us < alignment_deadline &&
            !pixel_alignment_.resume_pending && captureReviewStopped(in) &&
            tick.now < alignment_deadline + t_.pixel_align_stable_us + t_.frame_timeout_us;
        if (tick.now >= alignment_deadline && !finishing_confirmation &&
            (!ordinary_follow || locked_sample)) {
            // Lack of progress abandons this target, not the running match.
            release("startup_alignment_reselect", true);
            return;
        }
    }
    if (locked_id >= 0 && (!fresh || in.target_id != locked_id)) {
        tick.immediate_motion_stop = true;
        auto& a = pixel_alignment_;
        // Short repeated gaps add no evidence, but do not discard fresh stationary
        // samples already collected during this recovery. pixelAlign resets them
        // when the image gap exceeds 150 ms or the target position drifts.
        a.resume_pending = true;
        // Never resume an Advance directly after a blind interval. Retain only
        // already collected stationary evidence; the missing frame adds none.
        if (a.stage == PixelAlignment::Stage::Advance ||
            a.stage == PixelAlignment::Stage::Correct) {
            a.stage = PixelAlignment::Stage::Brake;
            a.resume_frames = 0;
            a.stopped_since_us = a.stable_since_us = 0;
            a.stable_frames = 0;
            a.within_tolerance = false;
        }
        if (!startup_target_missing_since_us_)
            startup_target_missing_since_us_ = tick.now;
        reason_ = "startup_locked_target_wait";
        const bool alternative_fresh = ordinary_follow && in.camera_pitch_stable &&
            in.startup_alternative_ordinary_us &&
            in.startup_alternative_ordinary_us <= tick.now &&
            tick.now - in.startup_alternative_ordinary_us <= 200000;
        const uint64_t loss_hold_us = alternative_fresh ? 300000 : kStartupTargetLossHoldUs;
        if (tick.now - startup_target_missing_since_us_ >= loss_hold_us)
            release("startup_target_reselect", false);
        return;
    }
    startup_target_missing_since_us_ = 0;
    // Close through the guide approach/maneuver. Ordinary re-selection keeps
    // the current jaw target and still waits for its acknowledgement before motion.
    if (startup_action_ != StartupAction::OpenApproach) {
        const uint8_t frame = startup_action_ == StartupAction::SelectOrdinary ? frame_raised_ : 0;
        commandFrame(frame, tick.now);
        const int ready = frameWait(in, frame, tick.now);
        if (ready < 0) {
            failStop(tick, frame ? "gripper_open_timeout" : "startup_gripper_close_timeout");
            return;
        }
        if (!ready) {
            tick.immediate_motion_stop = true;
            reason_ = frame ? "gripper_feedback_wait" : "startup_gripper_close_wait";
            return;
        }
    }
    switch (startup_action_) {
        case StartupAction::Advance:
            if (startup_commanded_us_ < t_.startup_advance_us) {
                reason_ = "startup_advance";
                drive(tick, t_.startup_advance_speed, direction * .08f, true);
                return;
            }
            startup_action_ = StartupAction::Observe;
            tick.immediate_motion_stop = true;
            reason_ = "startup_advance_complete";
            return;
        case StartupAction::Observe: {
            if (visible && in.camera_pitch_stable && in.startup_visible_count >= 2) {
                startup_action_ = StartupAction::Approach;
                pixel_alignment_ = {};
                tick.immediate_motion_stop = true;
                reason_ = "startup_two_objects_seen";
                return;
            }
            reason_ = "startup_find_two_objects";
            turn();
            return;
        }
        case StartupAction::FarHandoff:
            tick.immediate_motion_stop = true;
            if (!fresh) {
                pixel_alignment_ = {};
                reason_ = "startup_handoff_wait";
                return;
            }
            // The adapter may use any fresh cargo box here. Identity changes
            // restart image stability, but never the old target-loss timer.
            if (!pixelAlign(tick, true, true))
                return;
            startup_pitch_ = t_.track_pitch_cdeg;
            startup_action_ = StartupAction::Approach;
            startup_sight_ = {};
            pixel_alignment_ = {};
            reason_ = "startup_switch_to_120";
            return;
        case StartupAction::Approach: {
            if (!fresh) {
                reason_ = "startup_approach_reacquire";
                turn();
                return;
            }
            trip_context_.target_id_ = in.target_id;
            trip_context_.label_ = in.label;
            // Continue the closed guide approach until both lower corners are
            // occupied in UI120; the selected target's Y no longer starts the maneuver.
            if (in.pixel_grasp_view) {
                if (!pixelAlign(tick))
                    return;
                reason_ = "startup_approach_120";
                // A closed pile guide can steer gently while advancing. Outside
                // the restart band, the short jitter grace uses half speed.
                const bool pile = in.target_id == kStartupPileTargetId;
                const float speed = pile && alignment_error > t_.pixel_align_approach_restart_px
                                        ? .5f * t_.approach_speed : t_.approach_speed;
                const float correction = pile
                    ? std::clamp(.5f * t_.pixel_align_gain_rps_per_px * in.pixel_grasp_error_px +
                                     forward_bias, -.08f, .08f)
                    : forward_bias;
                drive(tick, speed, correction, true);
            } else {
                reason_ = "startup_approach_far";
                drive(tick, t_.approach_speed,
                      std::clamp(t_.pixel_align_gain_rps_per_px * in.pixel_grasp_error_px +
                                     forward_bias,
                                 -t_.pixel_align_max_wz, t_.pixel_align_max_wz), true);
            }
            return;
        }
        case StartupAction::TurnBeforePitch:
        case StartupAction::StopBeforePitch:
            return; // handled before target-dependent startup logic
        case StartupAction::SelectOrdinary:
            if (!visible || !fresh || in.label != "ordinary_supply") {
                pixel_alignment_ = {};
                reason_ = "startup_find_ordinary";
                turn();
                return;
            }
            trip_context_.target_id_ = in.target_id;
            trip_context_.label_ = in.label;
            if (in.label == "ordinary_supply") {
                // Opening requires actual centering and a fresh stopped confirmation;
                // hold_position would accept a stable target anywhere in the image.
                if (!pixelAlign(tick, true, false, open_tolerance_px))
                    return;
                // Settle accepts bounded image jitter in the restart band, but
                // the actual opening sample must meet the approach tolerance.
                if (alignment_error > open_tolerance_px) {
                    pixel_alignment_.stage = PixelAlignment::Stage::Correct;
                    pixel_alignment_.within_tolerance = false;
                    return;
                }
                startup_action_ = StartupAction::OpenApproach;
                pixel_alignment_ = {};
                // Opening owns its feedback deadline. Approach will establish
                // its own correction baseline from fresh post-acknowledgement images.
                startup_alignment_target_id_ = in.target_id;
                startup_alignment_since_us_ = 0;
                startup_alignment_best_error_px_ = -1;
                startup_alignment_error_count_ = 0;
                tick.immediate_motion_stop = true;
                commandFrame(1, tick.now);
                reason_ = "startup_nearest_ordinary_open";
                return;
            }
            return;
        case StartupAction::OpenApproach: {
            if (!fresh || in.label != "ordinary_supply") {
                release("startup_nearest_ordinary_lost", false);
                return;
            }
            trip_context_.target_id_ = trip_context_.capture_target_id_ = in.target_id;
            trip_context_.label_ = in.label;
            commandFrame(1, tick.now);
            const int opened = frameWait(in, 1, tick.now);
            if (opened < 0) {
                failStop(tick, "gripper_open_timeout");
                return;
            }
            if (!opened) {
                tick.immediate_motion_stop = true;
                reason_ = "gripper_feedback_wait";
                return;
            }
            if (startup_first_capture_ && in.pixel_grasp_near_view) {
                if (!in.target_grasp_range_valid || in.target_grasp_range_past_rear) {
                    tick.immediate_motion_stop = true;
                    reason_ = "startup_grasp_range_wait";
                    return;
                }
                // Keep non-ordinary hazards and fresh scene checks before motion.
                if (!reviewFirstTripCapture(tick, "startup_approach"))
                    return;
                const bool ready = pixelCaptureReady(in);
                if (!pixelAlign(tick, ready, ready && pixelLateralAccepted(in)))
                    return;
                if (ready && pixelLateralAccepted(in)) {
                    tick.immediate_motion_stop = true;
                    requestPhase(PhaseOutcome::CaptureReady, "startup_complete");
                    return;
                }
            } else if (!pixelAlign(tick))
                return;
            reason_ = in.pixel_grasp_near_view ? "startup_open_approach_140"
                                              : "startup_open_approach_120";
            drive(tick, t_.approach_speed, 0);
            return;
        }
    }
}
} // namespace rescue
