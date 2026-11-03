#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>

namespace rescue {
int PushTask::startupSightTargetId() const {
    if (phase_ != TaskPhase::Startup)
        return -1;
    if (startup_action_ == StartupAction::Approach ||
        startup_action_ == StartupAction::OpenApproach ||
        (startup_action_ == StartupAction::SelectOrdinary &&
         trip_context_.label_ == "ordinary_supply"))
        return trip_context_.target_id_;
    return -1;
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
    const float direction = t_.start_zone == 2 || t_.start_zone == 3 ? -1.f : 1.f;
    const auto& in = tick.in;
    const auto turn = [&] {
        drive(tick, 0, direction * .3f);
        // Keep clearance/cap decisions from drive, without its legacy .4 rad/s floor.
        tick.motion.wz_rps = std::clamp(tick.motion.wz_rps, -.3f, .3f);
    };
    // Keep UI120 and the closed frame throughout two IMU half turns separated
    // by a measured 0.1 m reverse. Image-target loss cannot interrupt this sequence.
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
        if (turn_state.leg == Leg::Reverse) {
            if (!in.sent_velocity_valid || !std::isfinite(sent_vx) ||
                !std::isfinite(sent_wz) || sent_vx > t_.pixel_align_stopped_vx) {
                failStop(tick, "startup_pitch_reverse_feedback_missing");
                return;
            }
            const bool valid = std::abs(sent_vx) < .001f
                ? turn_state.reverse_imu.holdStationary(in.imu_translation_samples, tick.now)
                : turn_state.reverse_imu.advance(in.imu_translation_samples, tick.now);
            if (!valid) {
                failStop(tick, "startup_pitch_reverse_imu_gap");
                return;
            }
            const float remaining = .1f - turn_state.reverse_completed_m +
                                    turn_state.reverse_imu.distance();
            if (remaining > 0) {
                reason_ = "startup_pitch_reverse_010m";
                drive(tick, -std::min(t_.back_speed, std::max(.01f, 2.f * remaining)), 0);
                tick.immediate_motion_stop = tick.motion.vx_mps == 0;
                return;
            }
            turn_state.leg = Leg::ReverseStop;
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
        reason_ = turn_state.leg == Leg::ReverseStop ? "startup_pitch_reverse_stop"
                                                   : "startup_pitch_turn_stop";
        if (!stopped) {
            turn_state.stopped_us = 0;
            turn_state.reverse_imu.reset();
            return;
        }
        if (!turn_state.stopped_us)
            turn_state.stopped_us = tick.now;
        if (tick.now - turn_state.stopped_us < 250000)
            return;
        if (turn_state.leg == Leg::FirstTurn) {
            turn_state.leg = Leg::ReverseCalibrate;
            turn_state.reverse_heading = in.heading_rad;
            turn_state.reverse_imu.reset();
        }
        if (turn_state.leg == Leg::ReverseCalibrate) {
            reason_ = "startup_pitch_reverse_calibrate";
            // The 22:54 run saw post-turn gyro noise around 0.10 rad/s.
            // Keep the stopped-command dwell and fresh IMU checks, but do not
            // require capture's much quieter 0.05 rad/s / 0.05 m/s² window.
            // Average 500 ms of samples to reduce the noise in the bias estimate.
            if (turn_state.reverse_imu.calibrate(in.imu_translation_samples, tick.now,
                                                turn_state.reverse_heading, .15f, .30f, 500000)) {
                turn_state.leg = Leg::Reverse;
            } else {
                using Status = ImuTranslation::CalibrationStatus;
                switch (turn_state.reverse_imu.calibrationStatus()) {
                    case Status::AngularMotion:
                        reason_ = "startup_pitch_reverse_calibrate_gyro";
                        break;
                    case Status::AccelerationNoise:
                        reason_ = "startup_pitch_reverse_calibrate_accel";
                        break;
                    case Status::InvalidSample:
                        reason_ = "startup_pitch_reverse_calibrate_invalid_sample";
                        break;
                    case Status::SampleGap:
                        reason_ = "startup_pitch_reverse_calibrate_sample_gap";
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
    const bool fresh = in.target_valid && in.target_center_px_valid &&
                       in.pixel_grasp_sample_us && in.pixel_grasp_sample_us <= tick.now &&
                       tick.now - in.pixel_grasp_sample_us <= t_.frame_timeout_us;
    // Any ordinary already inside the calibrated frame hands off to Search, even
    // when the old approach target is missing or a different object is selected.
    const auto ordinary_region = in.capture_class_regions_px.find("ordinary_supply");
    if (in.pixel_grasp_near_view && in.camera_pitch_stable && in.capture_scene_valid &&
        in.capture_scene_us && in.capture_scene_us <= tick.now &&
        tick.now - in.capture_scene_us <= t_.frame_timeout_us &&
        ordinary_region != in.capture_class_regions_px.end() && !ordinary_region->second.empty()) {
        const auto& area = ordinary_region->second;
        for (const auto& object : in.capture_image_objects) {
            if (object.label != "ordinary_supply" || object.id < 0 || object.box.empty())
                continue;
            const cv::Point2f center(object.box.x + .5f * object.box.width,
                                     object.box.y + .5f * object.box.height);
            if (center.x < area.x || center.x > area.x + area.width ||
                center.y < area.y || center.y > area.y + area.height)
                continue;
            trip_context_.target_id_ = trip_context_.capture_target_id_ = object.id;
            trip_context_.label_ = "ordinary_supply";
            startup_action_ = StartupAction::OpenApproach;
            pixel_alignment_ = {};
            commandFrame(1, tick.now);
            tick.immediate_motion_stop = true;
            requestPhase(PhaseOutcome::SearchReady, "startup_complete");
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
        pixel_alignment_ = {};
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
    if (locked_id != startup_alignment_target_id_) {
        startup_alignment_target_id_ = locked_id;
        startup_alignment_since_us_ = tick.now;
        startup_alignment_best_error_px_ = locked_sample ? alignment_error : -1;
    }
    // Keep a lock while it is making measurable visual centering progress.
    // Jitter, new tracker IDs and repeated frames cannot restart the stall budget.
    if (locked_id >= 0) {
        if (new_sight_sample && locked_sample &&
            (startup_alignment_best_error_px_ < 0 ||
             startup_alignment_best_error_px_ - alignment_error >= t_.pixel_align_brake_motion_px)) {
            startup_alignment_best_error_px_ = alignment_error;
            startup_alignment_since_us_ = tick.now;
        }
        const float sent_vx = in.sent_velocity_valid ? in.sent_vx_mps : last_vx_;
        if (sent_vx > t_.pixel_align_stopped_vx) {
            startup_alignment_since_us_ = tick.now;
            startup_alignment_best_error_px_ = locked_sample ? alignment_error : -1;
        } else if (!startup_alignment_since_us_) {
            startup_alignment_since_us_ = tick.now;
        }
        if (tick.now - startup_alignment_since_us_ >= 3000000) {
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
        if (tick.now - startup_target_missing_since_us_ >= kStartupTargetLossHoldUs)
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
                drive(tick, t_.startup_advance_speed, 0);
                return;
            }
            startup_action_ = StartupAction::Observe;
            tick.immediate_motion_stop = true;
            reason_ = "startup_advance_complete";
            return;
        case StartupAction::Observe: {
            const unsigned classes = in.startup_visible_classes & 15u;
            if (visible && (classes & (classes - 1u))) {
                startup_action_ = StartupAction::Approach;
                pixel_alignment_ = {};
                tick.immediate_motion_stop = true;
                reason_ = "startup_two_classes_seen";
                return;
            }
            reason_ = "startup_find_two_classes";
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
            // At the lower quarter, confirm fresh rest frames. Switch UI90 directly
            // to UI120; its next handoff turns 180, reverses 0.1 m, then turns 180
            // before searching for ordinary cargo in the same UI120 view.
            if (in.pixel_view_handoff_ready) {
                if (!pixelAlign(tick, true, true))
                    return;
                startup_pitch_turn_ = {};
                startup_action_ = StartupAction::TurnBeforePitch;
                trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
                trip_context_.label_.clear();
                startup_sight_ = {};
                startup_target_missing_since_us_ = startup_alignment_since_us_ = 0;
                startup_alignment_target_id_ = -1;
                pixel_alignment_ = {};
                reason_ = "startup_pitch_turn_ready";
                return;
            }
            if (in.pixel_grasp_view) {
                if (!pixelAlign(tick))
                    return;
                reason_ = "startup_approach_120";
                drive(tick, t_.approach_speed, 0);
            } else {
                reason_ = "startup_approach_far";
                drive(tick, t_.approach_speed,
                      std::clamp(t_.pixel_align_gain_rps_per_px * in.pixel_grasp_error_px,
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
                if (!pixelAlign(tick, true, false, t_.pixel_align_tolerance_px))
                    return;
                startup_action_ = StartupAction::OpenApproach;
                pixel_alignment_ = {};
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
            if (!in.pixel_grasp_near_view && in.pixel_view_handoff_ready) {
                if (!pixelAlign(tick, true, true))
                    return;
                startup_pitch_ = t_.near_pitch_cdeg;
                startup_sight_ = {};
                startup_alignment_since_us_ = startup_target_missing_since_us_ = 0;
                startup_alignment_target_id_ = -1;
                pixel_alignment_ = {};
                tick.immediate_motion_stop = true;
                reason_ = "startup_switch_to_140";
                return;
            }
            if (!pixelAlign(tick))
                return;
            reason_ = in.pixel_grasp_near_view ? "startup_open_approach_140"
                                              : "startup_open_approach_120";
            drive(tick, t_.approach_speed, 0);
            return;
        }
    }
}
} // namespace rescue
