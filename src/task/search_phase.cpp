#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
namespace rescue {
void PushTask::updateSearchPhase(TaskTick& tick) {
    auto& split = search_phase_.first_trip_split_;
    const bool pixel_split =
        t_.pixel_grasp_enabled && ((split.pending && search_phase_.action == SearchAction::Scan) ||
                                   (split.active && search_phase_.action == SearchAction::Pile));
    if (pixel_split) {
        commandFrame(1, tick.now);
        tick.immediate_motion_stop = true;
        if (tick.now - split.started_us > t_.pile_budget_us) {
            failStop(tick, "first_trip_pixel_split_timeout");
            return;
        }
        // Keep the 20 px split bias. Do not use the shared capture controller's 2 px
        // override: detector jitter and chassis stopping delay make it oscillate.
        constexpr float tolerance = 10.f, restart = 16.f, max_motion = 8.f;
        constexpr uint64_t pulse_us = 100000, rest_us = 250000, stable_us = 200000;
        const auto resetWindow = [&] {
            split.stable_us = 0;
            split.stable_frames = 0;
            split.candidate_id = -1;
        };
        const auto stop = [&] {
            split.turning = false;
            split.stopped_us = tick.now;
            split.align_stage = "brake";
            resetWindow();
        };
        const auto& box = tick.in.split_ordinary_box;
        const auto sample = tick.in.split_ordinary_box_us;
        const int id = tick.in.split_ordinary_box_id;
        const bool fresh = !box.empty() && id >= 0 && sample &&
                           sample <= tick.now && tick.now - sample <= t_.frame_timeout_us;
        if (!fresh || !tick.in.camera_pitch_stable || tick.in.camera_pitch_cdeg != 0 ||
            tick.in.split_image_width_px <= 0 || !tick.in.gripper_done ||
            tick.in.gripper_feedback_open != 1 || !tick.in.heading_valid ||
            !std::isfinite(tick.in.heading_rad)) {
            stop();
            split.within_tolerance = false;
            reason_ = "first_trip_pixel_split_box_wait";
            return;
        }
        const float centre_x = box.x + .5f * box.width;
        const float centre_line = .5f * tick.in.split_image_width_px;
        if (split.pending) {
            split.pending = false;
            split.active = true;
            ++split.runs;
            split.target_id = id;
            split.pixel_target_x = centre_line + (centre_x >= centre_line ? 20.f : -20.f);
            split.within_tolerance = false;
            split.last_sample_us = sample;
            stop();
            trip_context_.target_id_ = trip_context_.capture_target_id_ = id;
            trip_context_.label_ = "ordinary_supply";
            enter(PushState::PILE, tick.now, "first_trip_pixel_split");
            return;
        }
        const float error = split.pixel_target_x - centre_x;
        const int direction = error > 0 ? 1 : (error < 0 ? -1 : 0);
        const float sent_vx = tick.in.sent_velocity_valid ? tick.in.sent_vx_mps : last_vx_;
        const float sent_wz = tick.in.sent_velocity_valid ? tick.in.sent_wz_rps : last_wz_;
        const bool stopped = std::isfinite(sent_vx) && std::isfinite(sent_wz) &&
                             std::abs(sent_vx) <= t_.pixel_align_stopped_vx &&
                             std::abs(sent_wz) <= t_.pixel_align_stopped_wz;
        if (split.turning) {
            if (id != split.target_id || tick.now - split.pulse_started_us >= pulse_us ||
                std::abs(error) <= tolerance || direction != split.pulse_direction ||
                std::abs(sent_vx) > t_.pixel_align_stopped_vx) {
                stop();
                reason_ = "first_trip_pixel_split_brake";
                return;
            }
            split.align_stage = "pulse";
            reason_ = "first_trip_pixel_split_pulse";
            drive(tick, 0, std::clamp(.0015f * error, -.10f, .10f), true);
            tick.immediate_motion_stop = tick.motion.wz_rps == 0;
            if (tick.immediate_motion_stop)
                stop();
            return;
        }
        split.align_stage = id == split.target_id ? "settle" : "reselect";
        reason_ = id == split.target_id ? "first_trip_pixel_split_settle"
                                       : "first_trip_pixel_split_reselect";
        if (!stopped) {
            split.stopped_us = tick.now;
            resetWindow();
            return;
        }
        if (!split.stopped_us)
            split.stopped_us = tick.now;
        // Only unique post-stop frames count. A changed ID must also remain stationary
        // before it can replace the preferred target; it can never reset an active turn.
        if (sample <= split.last_sample_us || sample <= split.stopped_us)
            return;
        split.last_sample_us = sample;
        if (split.candidate_id != id || !split.stable_frames ||
            std::max(split.max_x, centre_x) - std::min(split.min_x, centre_x) > max_motion ||
            std::abs(wrapAngle(tick.in.heading_rad - split.heading_ref)) > .01f) {
            split.candidate_id = id;
            split.stable_frames = 0;
            split.stable_us = sample;
            split.min_x = split.max_x = centre_x;
            split.heading_ref = tick.in.heading_rad;
        }
        split.min_x = std::min(split.min_x, centre_x);
        split.max_x = std::max(split.max_x, centre_x);
        ++split.stable_frames;
        if (id != split.target_id)
            split.within_tolerance = false;
        else if (std::abs(error) <= tolerance)
            split.within_tolerance = true;
        else if (std::abs(error) > restart)
            split.within_tolerance = false;
        if (tick.now - split.stopped_us < rest_us || split.stable_frames < 3 ||
            sample - split.stable_us < stable_us)
            return;
        if (id != split.target_id) {
            split.target_id = id;
            trip_context_.target_id_ = trip_context_.capture_target_id_ = id;
            trip_context_.label_ = "ordinary_supply";
            stop();
            reason_ = "first_trip_pixel_split_reselected";
            return;
        }
        if (!split.within_tolerance) {
            split.turning = true;
            split.pulse_started_us = tick.now;
            split.pulse_direction = direction;
            resetWindow();
            split.align_stage = "pulse";
            reason_ = "first_trip_pixel_split_pulse";
            drive(tick, 0, std::clamp(.0015f * error, -.10f, .10f), true);
            tick.immediate_motion_stop = tick.motion.wz_rps == 0;
            if (tick.immediate_motion_stop)
                stop();
            return;
        }
        split.align_stage = "advance";
        split.active = false;
        trip_context_.target_id_ = trip_context_.capture_target_id_ = id;
        trip_context_.label_ = "ordinary_supply";
        enter(PushState::APPROACH, tick.now, "first_trip_pixel_split_advance");
        search_phase_.approach_pitch_ = 0;
        // Existing pixel approach performs the UI120 -> UI140 handoff, stops at the normal
        // grasp position and then closes/verifies. No uncalibrated UI120 stop point is invented.
        return;
    }
    const bool image_search = search_phase_.action == SearchAction::Scan ||
                              search_phase_.action == SearchAction::LostSearch ||
                              search_phase_.action == SearchAction::MidReacquire ||
                              search_phase_.action == SearchAction::NearReacquire ||
                              search_phase_.action == SearchAction::SelectCargo ||
                              search_phase_.action == SearchAction::MidApproach ||
                              search_phase_.action == SearchAction::Approach;
    auto& recovery = search_phase_.far_target_recovery_;
    const bool far_tracking = t_.pixel_grasp_enabled && tick.in.pixel_selection_managed &&
                              search_phase_.action == SearchAction::Approach &&
                              trip_context_.target_id_ >= 0 && tick.in.camera_pitch_stable &&
                              tick.in.camera_pitch_cdeg != kCameraPitchInvalid &&
                              std::abs(int(tick.in.camera_pitch_cdeg) - int(t_.far_pitch_cdeg)) <=
                                  t_.pitch_tolerance_cdeg &&
                              !tick.in.pixel_grasp_view && !tick.in.pixel_view_handoff_ready;
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
            if (tick.in.pixel_view_handoff_ready) {
                if (!pixelAlign(tick, true, true))
                    return;
                enter(PushState::APPROACH, tick.now, "pixel_center_switch_to_140");
                search_phase_.approach_pitch_ = t_.near_pitch_cdeg;
                return;
            }
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
            drive(tick, t_.approach_speed, 0);
            return;
        }
        if (!tick.in.target_grasp_range_valid || tick.in.target_grasp_range_past_rear) {
            enter(PushState::CAPTURE_FAIL, tick.now, "target_outside_grasp_center_range");
            return;
        }
        if (!pixelAlign(tick, true, pixelLateralAccepted(tick.in)))
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
            enter(PushState::CAPTURE_FAIL, tick.now, "pixel_corridor_rejected");
            return;
        }
        if (trip_context_.seen_ != tick.in.corridor) {
            trip_context_.seen_ = tick.in.corridor;
            actionProgress().confirmations = 0;
        }
        if (++actionProgress().confirmations < t_.corridor_confirm_frames)
            return;
        trip_context_.pending_ = tick.in.corridor;
        requestPhase(PhaseOutcome::CaptureReady, "pixel_corridor_ok");
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
                                  sweep_ledger_,
                                  reason_};
            context.now_us = tick.now;
            context.elapsed_us = tick.elapsed;
            context.scan_ok = tick.scan_ok;
            context.forward_ok = tick.forward_ok;
            context.scatter_ok = tick.scatter_ok;
            context.clutter_fresh = tick.clutter_fresh;
            context.scatter_multi_ok = tick.scatter_multi_ok;
            context.approach_ok = tick.approach_ok;
            context.mid_pitch = midPitch();
            context.trip_empty = trip_context_.trip_.total() == 0;
            context.sweep_open = sweep_open_;
            context.first_ordinary = first_;
            context.frame_raised = frame_raised_;
            context.far_y = tick.far_y;

            SearchServices services;
            services.candidate = [&](const PushObservation& obs, uint64_t at) {
                return candidate(obs, at);
            };
            services.weak_candidate = [&](const PushObservation& obs) {
                return weakCandidate(obs);
            };
            services.cue_candidate = [&] {
                return cueCandidate(tick);
            };
            services.target_body = [&] {
                return targetBody(tick);
            };
            services.find_best =
                [&](const std::vector<cv::Point2f>& pts, float max_y, float radius) {
                    const auto best = findBest(tick, pts, max_y, radius);
                    return BestGroup{best.n, best.d};
                };
            services.eval_group =
                [&](const std::vector<cv::Point2f>& pts, int need_n, int far_need, float radius) {
                    return evalGroup(tick, pts, need_n, far_need, radius);
                };
            services.add_ordinary_extra = [&](const cv::Point2f& pt) {
                tick.cluster.addOrdinaryExtra(pt);
            };
            services.begin_scatter = [&](bool far, bool commit, const char* why) {
                beginScatter(tick, far, commit, why);
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
            services.reject_search_cue = [&](uint64_t at) {
                if (search_phase_.search_cue_id_ >= 0)
                    rejected_[search_phase_.search_cue_id_] = at + t_.blacklist_us;
            };
            services.search_lost = [&] {
                searchLost(tick);
            };

            context.pitch_cmd = pitch_cmd_;
            context.sweep_over = tick.sweepOver;
            context.sweep_time_ok = tick.sweepTimeOk;
            context.sweep_ledger_ok = tick.sweepLedgerOk;
            context.sweep_open_disabled = sweep_open_disabled_;
            context.sweep_esc = tick.sweep_esc;
            context.target_forward_m = tick.target_forward_m;

            services.frame_wait = [&](const PushObservation& o, uint8_t open, uint64_t at) {
                return frameWait(o, open, at);
            };
            services.clear_trip = [&] {
                clearTrip();
            };
            services.blacklist = [&](uint64_t at) {
                blacklist(at);
            };
            services.clear_rejected = [&] {
                rejected_.clear();
            };
            services.clear_rejected_bearings = [&] {
                rejected_bearings_.clear();
            };
            services.sweep_frame_ready = [&] {
                return sweepFrameReady(tick);
            };
            services.disperse_step = [&] {
                return disperseStep(tick);
            };
            services.begin_pile_from_sweep = [&](const char* why) {
                return beginPileFromSweep(tick, why);
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
            services.upright_in_corridor =
                [&](const PushObservation& o, uint64_t at, float& lat, float* along) {
                    return uprightInCorridor(o, at, lat, along);
                };
            services.neighbour_in_corridor =
                [&](const PushObservation& o, uint64_t at, float& lat, float* along) {
                    return neighbourInCorridor(o, at, lat, along);
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
            services.motion_ratio = [&] {
                return motionRatio();
            };

            search_phase_.search_.update(search_phase_.action, t_, tick.in, context, services);
            break;
        }
        case SearchAction::Scatter:
        case SearchAction::Pile:
        case SearchAction::CueApproach:
        case SearchAction::CuePrepare:
        case SearchAction::ClearPile:
        case SearchAction::ClearPause: {
            SweepContext context{search_phase_,
                                 actionProgress(),
                                 trip_context_,
                                 tick.motion,
                                 sweep_ledger_,
                                 reason_};
            context.now_us = tick.now;
            context.elapsed_us = tick.elapsed;
            context.clutter_fresh = tick.clutter_fresh;
            context.scatter_ok = tick.scatter_ok;
            context.first_ordinary = first_;
            context.pitch_cmd = pitch_cmd_;
            context.sweep_over = tick.sweepOver;
            context.sweep_operation_timeout = tick.sweepBudget.operation_timeout;
            context.target_forward_m = tick.target_forward_m;

            SweepServices services;
            services.command_frame = [&](uint8_t open, uint64_t at) {
                commandFrame(open, at);
            };
            services.drive = [&](float vx, float wz) {
                drive(tick, vx, wz);
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
            services.blacklist = [&](uint64_t at) {
                blacklist(at);
            };
            services.clear_trip = [&] {
                clearTrip();
            };
            services.candidate = [&](const PushObservation& o, uint64_t at) {
                return candidate(o, at);
            };
            services.weak_candidate = [&](const PushObservation& o) {
                return weakCandidate(o);
            };
            services.cue_candidate = [&] {
                return cueCandidate(tick);
            };
            services.target_body = [&] {
                return targetBody(tick);
            };
            services.find_best =
                [&](const std::vector<cv::Point2f>& pts, float max_y, float radius) {
                    const auto best = findBest(tick, pts, max_y, radius);
                    return SweepBest{best.n, best.d, tick.cluster.bestCentre()};
                };
            services.eval_group =
                [&](const std::vector<cv::Point2f>& pts, int need, int far_need, float radius) {
                    return evalGroup(tick, pts, need, far_need, radius);
                };
            services.add_ordinary_extra = [&](const cv::Point2f& pt) {
                tick.cluster.addOrdinaryExtra(pt);
            };
            services.clear_rejected = [&] {
                rejected_.clear();
            };
            services.clear_rejected_bearings = [&] {
                rejected_bearings_.clear();
            };
            services.begin_scatter = [&](bool far, bool commit, const char* why) {
                beginScatter(tick, far, commit, why);
            };
            services.sweep_frame_ready = [&] {
                return sweepFrameReady(tick);
            };
            services.disperse_step = [&] {
                return disperseStep(tick);
            };
            services.begin_pile_from_sweep = [&](const char* why) {
                return beginPileFromSweep(tick, why);
            };
            services.search_lost = [&] {
                searchLost(tick);
            };
            services.lost_target = [&] {
                return lostTarget(tick);
            };
            services.steer = [&](float speed, float heading) {
                steer(tick, speed, heading);
            };
            services.desired_pitch = [&]() -> int16_t {
                return desiredPitch();
            };
            services.tracking = [&](const PushObservation& o) {
                return tracking(o);
            };
            services.mid_pitch = [&] {
                return midPitch();
            };
            services.motion_ratio = [&] {
                return motionRatio();
            };

            search_phase_.sweep_.update(search_phase_.action, t_, tick.in, context, services);
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
