#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
namespace rescue {
void PushTask::updateRecoveryPhase(TaskTick& tick) {
    switch (state_) {
        case PushState::CAPTURE_FAIL:
        case PushState::LOST_HOLD:
        case PushState::ABORT_DROP: {
            if (t_.pixel_grasp_enabled && targetSelectable(trip_context_.label_, first_) &&
                (phase_ != TaskPhase::Transport || !first_) &&
                !search_phase_.first_trip_split_.pending) {
                tick.immediate_motion_stop = true;
                commandFrame(1, tick.now);
                if (!tick.in.gripper_done || tick.in.gripper_feedback_open != 1) {
                    reason_ = "first_trip_recovery_open_wait";
                    if (tick.elapsed > t_.gripper_timeout_us)
                        failStop(tick, "first_trip_recovery_open_timeout");
                    break;
                }
                if (!nominateCaptureLoad(tick))
                    break;
                if (reviewFirstTripCapture(tick, "approach_recovery")) {
                    trip_context_.pending_ = capture_scene_.inventory;
                    if (search_entry_.target_id >= 0) {
                        invalidateEntryPosition();
                        search_entry_.target_id = trip_context_.target_id_;
                        search_entry_.label = trip_context_.label_;
                        enter(PushState::PREPARE, tick.now, "first_entry_recovery_recheck");
                    } else
                        enter(PushState::RUSH, tick.now, "first_trip_recovery_review_confirmed");
                }
                break;
            }
            // Release where the opponent zone is not involved, then reverse off the objects.
            if (actionProgress().step == 0 && corridor_retreat_) {
                commandFrame(1, tick.now);
                actionProgress().step = 1;
                actionProgress().started_us = tick.now;
            } // nothing held: frame stays up
            if (actionProgress().step == 0) {
                if (!tick.in.opponent_zone_clear) {
                    if (tick.elapsed > t_.gripper_timeout_us)
                        fail("drop_blocked_opponent_zone");
                    break;
                }
                commandFrame(1, tick.now);
                const int g = frameWait(tick.in, 1, tick.now);
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
            }
            if (search_phase_.first_trip_split_.pending) {
                // Injured side targets turn first; ordinary and front-corner splits keep
                // their existing retreat/turn sequence. All finish in stopped UI140 review.
                tick.immediate_motion_stop = true;
                auto& split = search_phase_.first_trip_split_;
                using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
                using Margin = SearchPhaseData::FirstTripSplit::MarginStage;
                const auto interruptMargin = [&] {
                    split.margin_tick_us = 0;
                    split.margin_sent_reverse = false;
                };
                capture_scene_.stage = "postsplit_before_approach";
                const auto resetReobserve = [&] {
                    capture_scene_.reobserve_stopped_us = capture_scene_.reobserve_sample_us = 0;
                    capture_scene_.reobserve_frames = 0;
                };
                const auto resumeApproach = [&](const char* why) {
                    resumePixelSplitSearch(tick, why);
                };
                // Only the balanced return and turn back yield to discovered cargo.
                // UI120 discoveries enter normal approach, which still performs
                // stopped UI140 capture review before closing the gripper.
                const auto& observation = tick.in;
                const bool fresh_candidate = t_.split_capture_preempt &&
                    (split.recovery_stage == Stage::CornerReturn ||
                     split.recovery_stage == Stage::CornerTurnBack) && !split.target_reselecting &&
                    observation.capture_scene_valid && observation.capture_scene_us &&
                    observation.capture_scene_us > std::max(split.started_us, split.recovery_view_us) &&
                    observation.capture_scene_us <= tick.now &&
                    tick.now - observation.capture_scene_us <= t_.frame_timeout_us &&
                    candidate(observation, tick.now) && captureLoadAllows(observation.label) &&
                    observation.pixel_selection_managed && observation.pixel_locked &&
                    observation.pixel_grasp_view && !observation.pixel_grasp_near_view &&
                    std::any_of(observation.capture_image_objects.begin(),
                        observation.capture_image_objects.end(), [&](const auto& object) {
                            return object.id == observation.target_id &&
                                object.label == observation.label && object.complete && object.selectable;
                        });
                if (fresh_candidate) {
                    const int id = observation.target_id;
                    const auto label = observation.label;
                    resumeApproach("pixel_split_target_interrupt");
                    trip_context_.target_id_ = trip_context_.capture_target_id_ = id;
                    trip_context_.label_ = label;
                    search_phase_.pixel_reselection_ = {};
                    search_phase_.pixel_approach_started_us_ = tick.now;
                    break;
                }
                if (tick.now - split.started_us > t_.pile_budget_us) {
                    if (split.observation_wait_us || split.recovery_stage == Stage::Verify) {
                        reason_ = "first_trip_split_observation_wait";
                        break; // retain zero until the observation watchdog restarts search
                    }
                    if (split.upper_corner &&
                        (split.recovery_stage == Stage::CornerTurnRight ||
                         split.recovery_stage == Stage::CornerPush ||
                         split.recovery_stage == Stage::CornerReturn ||
                         split.recovery_stage == Stage::CornerTurnBack))
                        resumePixelSplitSearch(tick, "pixel_split_corner_timeout_research", true);
                    else
                        failStop(tick, "first_trip_vertical_split_timeout");
                    break;
                }
                if (split.recovery_stage == Stage::InjuredTurn) {
                    split.align_stage = "injured_turn_before_retreat";
                    const auto& in = tick.in;
                    if (!in.heading_valid || !std::isfinite(in.heading_rad)) {
                        split.corner_turn_sample_us = split.corner_stopped_us = 0;
                        split.corner_turn_braking = true;
                        reason_ = "injured_split_heading_wait";
                        break;
                    }
                    const float error = wrapAngle(split.injured_turn_goal - in.heading_rad);
                    constexpr float tolerance = .02f, quiet_rate = .03f;
                    const bool sampled = split.corner_turn_sample_us &&
                        tick.now > split.corner_turn_sample_us &&
                        tick.now - split.corner_turn_sample_us <= 200000;
                    const float rate = sampled
                        ? wrapAngle(in.heading_rad - split.corner_turn_heading) *
                            (1000000.f / (tick.now - split.corner_turn_sample_us)) : 0.f;
                    split.turn_rate = rate;
                    split.corner_turn_sample_us = tick.now;
                    split.corner_turn_heading = in.heading_rad;
                    if (!sampled) split.corner_stopped_us = 0;
                    const bool moving = sampled && std::abs(rate) > quiet_rate;
                    split.corner_turn_braking |= std::abs(error) <= tolerance ||
                        (moving && (rate * error < 0 ||
                            std::abs(error) <= tolerance + .15f * std::abs(rate)));
                    if (!split.corner_turn_braking) {
                        reason_ = split.recovery_direction > 0 ? "injured_split_turn_left"
                                                               : "injured_split_turn_right";
                        const float speed = std::min({.15f, t_.max_wz,
                            std::max(.05f, 2.f * std::abs(error))});
                        drive(tick, 0, std::copysign(speed, error), true);
                        tick.immediate_motion_stop = tick.motion.wz_rps == 0;
                        break;
                    }
                    reason_ = "injured_split_turn_brake";
                    split.align_stage = "injured_turn_brake";
                    if (!captureReviewStopped(in) || !sampled || moving) {
                        split.corner_stopped_us = 0;
                        break;
                    }
                    if (!split.corner_stopped_us ||
                        std::abs(wrapAngle(in.heading_rad - split.corner_turn_stop_heading)) > .005f) {
                        split.corner_stopped_us = tick.now;
                        split.corner_turn_stop_heading = in.heading_rad;
                    }
                    if (tick.now - split.corner_stopped_us < 250000) break;
                    split.corner_stopped_us = split.corner_turn_sample_us = 0;
                    split.corner_turn_braking = false;
                    if (std::abs(error) > tolerance) break; // correction only after a measured stop
                    split.recovery_stage = Stage::Retreat;
                    split.recovery_view_us = tick.now;
                    split.recovery_turn_us = 0;
                    split.retreat_objects.clear();
                    capture_scene_.confirmations = 0;
                    capture_scene_.stopped_us = tick.now;
                    reason_ = "injured_split_switch_ui140_retreat";
                    break;
                }
                if (split.recovery_stage == Stage::CornerTurnRight ||
                    split.recovery_stage == Stage::CornerPush ||
                    split.recovery_stage == Stage::CornerReturn ||
                    split.recovery_stage == Stage::CornerTurnBack) {
                    const auto& in = tick.in;
                    if (!in.heading_valid || !std::isfinite(in.heading_rad)) {
                        split.corner_turn_sample_us = split.corner_stopped_us = 0;
                        split.corner_turn_braking = true;
                        reason_ = "pixel_split_corner_heading_wait";
                        break;
                    }
                    const bool stopped = captureReviewStopped(in);
                    if (split.recovery_stage == Stage::CornerTurnRight ||
                        split.recovery_stage == Stage::CornerTurnBack) {
                        const bool right = split.recovery_stage == Stage::CornerTurnRight;
                        split.align_stage = right ? "corner_turn_right" : "corner_turn_back";
                        reason_ = right ? "pixel_split_corner_right90" : "pixel_split_corner_left80";
                        // Right 90 degrees, then return left 80: finish 10 degrees right of entry.
                        const float goal = split.corner_heading - (right ? 1.570796327f : .174532925f);
                        const float error = wrapAngle(goal - in.heading_rad);
                        // Taper the last part of both turns, reserving 150 ms of
                        // measured rotation for command latency and chassis braking.
                        constexpr float tolerance = .02f, quiet_rate = .03f;
                        const bool sampled = split.corner_turn_sample_us &&
                            tick.now > split.corner_turn_sample_us &&
                            tick.now - split.corner_turn_sample_us <= 200000;
                        const float rate = sampled
                            ? wrapAngle(in.heading_rad - split.corner_turn_heading) *
                                (1000000.f / (tick.now - split.corner_turn_sample_us))
                            : 0.f;
                        split.turn_rate = rate;
                        split.corner_turn_sample_us = tick.now;
                        split.corner_turn_heading = in.heading_rad;
                        if (!sampled) split.corner_stopped_us = 0;
                        const bool moving = sampled && std::abs(rate) > quiet_rate;
                        split.corner_turn_braking |= std::abs(error) <= tolerance ||
                            (moving && (rate * error < 0 ||
                                std::abs(error) <= tolerance + .15f * std::abs(rate)));
                        if (split.corner_turn_braking) {
                            // Sent zero is insufficient: also require a quiet heading
                            // spanning 250 ms, with at most 0.005 rad total drift.
                            split.align_stage = right ? "corner_turn_right_brake" : "corner_turn_back_brake";
                            if (!stopped || !sampled || moving) {
                                split.corner_stopped_us = 0;
                                break;
                            }
                            if (!split.corner_stopped_us ||
                                std::abs(wrapAngle(in.heading_rad - split.corner_turn_stop_heading)) > .005f) {
                                split.corner_stopped_us = tick.now;
                                split.corner_turn_stop_heading = in.heading_rad;
                            }
                            if (tick.now - split.corner_stopped_us < 250000)
                                break;
                            split.corner_stopped_us = 0;
                            split.corner_turn_braking = false;
                            split.corner_turn_sample_us = 0;
                            // Any correction starts after braking, never by reversing
                            // a still-rotating chassis at full speed.
                            if (std::abs(error) > tolerance)
                                break;
                            if (right) {
                                // Only heading uses the IMU. Linear travel is the
                                // actual sent-velocity integral for this action.
                                split.corner_distance = .036f;
                                split.corner_command_origin_m = split.corner_command_peak_m = odo_;
                                split.corner_braking = false;
                                split.corner_stopped_us = 0;
                                split.recovery_stage = Stage::CornerPush;
                            } else {
                                resumeApproach("pixel_split_corner_resume_ui120");
                            }
                        } else {
                            if (right) split.corner_imu.reset();
                            const float speed = std::min({.3f + t_.split_wz_increment_rps,
                                t_.max_wz, std::max(.08f, 2.f * std::abs(error))});
                            drive(tick, 0, std::copysign(speed, error), true);
                            if (tick.motion.wz_rps != 0)
                                tick.motion.wz_rps = std::copysign(speed, error);
                            tick.immediate_motion_stop = tick.motion.wz_rps == 0;
                        }
                        break;
                    }
                    const bool push = split.recovery_stage == Stage::CornerPush;
                    split.align_stage = push ? "corner_push" : "corner_return";
                    reason_ = push ? "pixel_split_corner_push036_command"
                                   : "pixel_split_corner_return_command";
                    if (!in.sent_velocity_valid || !std::isfinite(in.sent_vx_mps) ||
                        !std::isfinite(in.sent_wz_rps)) {
                        failStop(tick, "pixel_split_corner_command_feedback_missing");
                        break;
                    }
                    if (push)
                        split.corner_command_peak_m = std::max(split.corner_command_peak_m, odo_);
                    const float command_remaining = odo_ - split.corner_command_origin_m;
                    const float remaining = push ? split.corner_distance - command_remaining
                                                 : command_remaining;
                    // Complete the return when reverse commands balance the actual
                    // forward commands, regardless of translation-IMU drift/gaps.
                    split.corner_braking |= remaining <= 0;
                    if (split.corner_braking) {
                        if (!stopped) {
                            split.corner_stopped_us = 0;
                            break;
                        }
                        if (!split.corner_stopped_us)
                            split.corner_stopped_us = tick.now;
                        if (tick.now - split.corner_stopped_us >= 250000) {
                            split.corner_braking = false;
                            split.corner_stopped_us = 0;
                            split.recovery_stage = push ? Stage::CornerReturn : Stage::CornerTurnBack;
                            split.corner_turn_sample_us = 0;
                            split.corner_turn_braking = false;
                        }
                    } else {
                        const float error = wrapAngle(split.corner_heading - 1.570796327f - in.heading_rad);
                        // Slow the sent commands near each endpoint. This integral
                        // is an action budget, not a measurement of physical travel.
                        float speed = std::min({t_.approach_speed + t_.split_speed_increment_mps,
                                                t_.rush_near_speed + t_.split_speed_increment_mps,
                                                std::max(.01f, 2.f * remaining) + t_.split_speed_increment_mps});
                        drive(tick, push ? speed : -speed, std::clamp(error, -.15f, .15f), true);
                        tick.immediate_motion_stop = tick.motion.vx_mps == 0 && tick.motion.wz_rps == 0;
                    }
                    break;
                }
                if (split.recovery_stage == Stage::Turn) {
                    split.align_stage = "ui120_turn";
                    if (split.recovery_turn_us && tick.now - split.recovery_turn_us >= 500000) {
                        beginPixelSplitReview(tick);
                        reason_ = "first_trip_split_turn_stopped";
                        break;
                    }
                    reason_ = split.recovery_direction > 0 ? "pixel_split_ui120_left"
                                                           : "first_trip_split_ui120_right";
                    drive(tick, 0.f, split.recovery_direction * (.3f + t_.split_wz_increment_rps), true);
                    if (tick.motion.wz_rps != 0) {
                        // This timed split turn has its own speed, beyond the alignment cap.
                        tick.motion.wz_rps = split.recovery_direction * std::min(.3f + t_.split_wz_increment_rps, t_.max_wz);
                        if (!split.recovery_turn_us)
                            split.recovery_turn_us = tick.now;
                        tick.immediate_motion_stop = false;
                    } else {
                        split.recovery_turn_us = 0;
                    }
                    break;
                }
                split.align_stage =
                    split.recovery_stage == Stage::Retreat ? "ui140_retreat" : "ui140_verify";
                const auto& in = tick.in;
                const auto& objects = in.capture_image_objects;
                const auto target =
                    std::find_if(objects.begin(), objects.end(), [&](const auto& object) {
                        return object.id == in.target_id && targetSelectable(object.label, first_);
                    });
                if (pitchWait(in, tick.now) <= 0 || !in.camera_pitch_stable ||
                    !in.pixel_grasp_near_view || !in.capture_scene_valid ||
                    in.layout_count_region_px.empty() || !in.capture_scene_us ||
                    in.capture_scene_us <= std::max(split.started_us, split.recovery_view_us) ||
                    in.capture_scene_us > tick.now ||
                    tick.now - in.capture_scene_us > t_.frame_timeout_us || !in.heading_valid ||
                    !std::isfinite(in.heading_rad)) {
                    capture_scene_.confirmations = 0;
                    capture_scene_.stopped_us = tick.now;
                    reason_ = "first_trip_split_observation_wait";
                    resetReobserve();
                    interruptMargin();
                    break;
                }
                capture_scene_.objects = objects;
                capture_scene_.target_id = in.target_id;
                capture_scene_.pitch = in.camera_pitch_cdeg;
                capture_scene_.sample_us = in.capture_scene_us;
                if (split.recovery_stage == Stage::Retreat) {
                    if (split.keep_injured && !split.upper_corner && split.retreat_objects.empty()) {
                        const PushObservation::CaptureImageObject* injured = nullptr;
                        int candidates = 0;
                        for (const auto& object : objects)
                            if (object.label == "injured_person" && object.selectable) {
                                ++candidates;
                                injured = &object;
                            }
                        if (candidates == 1 && injured->complete) {
                            split.target_id = injured->id;
                            split.retreat_objects.push_back({*injured, splitRetreatCleared(in, *injured)});
                        }
                    }
                    // The completed extra retreat always needs a fresh actual zero
                    // send for its second stop, even if a new pending view reset stage.
                    const bool margin_feedback_missing = !split.upper_corner &&
                        t_.split_margin_budget_us > 0 &&
                        split.margin_used_us >= t_.split_margin_budget_us &&
                        (!in.sent_velocity_us || in.sent_velocity_us > tick.now ||
                         tick.now - in.sent_velocity_us > 100000);
                    // Retained targets can leave the near view during reverse. Only
                    // the branch-specific objects latched at entry control this operation.
                    bool waiting = split.retreat_objects.empty();
                    std::set<int> used, reserved;
                    for (const auto& reference : split.retreat_objects)
                        reserved.insert(reference.observation.id);
                    bool all_clear = !waiting;
                    bool image_quiet = true;
                    const float stop_y = split.upper_corner ? 280.f : 270.f;
                    // The field trace kept moving for about 200 ms after zero; add image age.
                    // Use that horizon to taper reverse speed, not to request an early
                    // stop/restart cycle. Only observed clearance starts the final stop.
                    const float stop_horizon_s = .20f + (tick.now - in.capture_scene_us) * 1e-6f;
                    split.retreat_reference_id = -1;
                    split.retreat_reference_y = 0;
                    for (auto& reference : split.retreat_objects) {
                        auto& previous = reference.observation;
                        const PushObservation::CaptureImageObject* match = nullptr;
                        int matches = 0;
                        for (const auto& object : objects)
                            if (object.id == previous.id && object.label == previous.label) {
                                match = &object;
                                ++matches;
                            }
                        const auto observeMotion = [&](const PushObservation::CaptureImageObject& object) {
                            const float y = object.box.y + .5f * object.box.height;
                            if (split.retreat_braking && std::abs(y - reference.brake_y) > 12.f)
                                image_quiet = false;
                            if (in.capture_scene_us <= reference.motion_sample_us)
                                return; // repeated images cannot create a velocity sample
                            const bool continuous = reference.motion_sample_us &&
                                in.capture_scene_us - reference.motion_sample_us <= 200000 &&
                                object.id == previous.id;
                            if (continuous && !split.retreat_braking && in.sent_velocity_valid &&
                                std::isfinite(in.sent_vx_mps) && in.sent_vx_mps < -.001f) {
                                const float rate = (y - reference.motion_y) * 1000000.f /
                                    (in.capture_scene_us - reference.motion_sample_us);
                                if (y > stop_y && rate < 0) {
                                    // Start tapering one stopping horizon before the
                                    // coast reserve is needed: lower commands also
                                    // take time to reach the chassis and image.
                                    const float speed = -in.sent_vx_mps * (y - stop_y) /
                                        (-rate * 2.f * stop_horizon_s);
                                    // Keep the established 0.01 m/s endpoint speed.
                                    // Latching the cap prevents a flat/noisy next frame
                                    // from accelerating back to 0.05 m/s near the line.
                                    split.retreat_speed_mps = std::min(split.retreat_speed_mps,
                                        std::clamp(speed, .01f, .05f));
                                }
                            }
                            reference.motion_sample_us = in.capture_scene_us;
                            reference.motion_y = y;
                        };
                        if (reference.cleared) {
                            if (matches == 1 && match->complete) {
                                observeMotion(*match);
                                previous = *match;
                            }
                            // A block already observed fully clear may disappear at the
                            // image top while reversing. Do not require it to reappear.
                            const bool known_top_exit = match && !match->complete &&
                                match->box.y <= 0 && !in.splitInventoryOverlaps(match->box, match->label) &&
                                !in.capturePathOverlaps(match->box, match->label);
                            if (split.upper_corner) {
                                if (matches != 1 || !match->complete || splitRetreatCleared(in, *match))
                                    continue;
                            } else if (matches == 0 || (matches == 1 &&
                                (match->complete || known_top_exit) &&
                                (!match->complete || splitRetreatCleared(in, *match)))) {
                                continue;
                            }
                            reference.cleared = false;
                        }
                        if (matches == 0) {
                            // Tracking IDs may change. Reassociate only a unique,
                            // nearby object of the same class as this unwanted block.
                            for (const auto& object : objects) {
                                if (object.id == split.target_id || reserved.count(object.id) ||
                                    object.label != previous.label || previous.box.empty() ||
                                    object.box.empty())
                                    continue;
                                const float scale = float(object.box.area()) / previous.box.area();
                                const float gate = .5f * std::max({previous.box.width,
                                    previous.box.height, object.box.width, object.box.height});
                                const cv::Point2f delta{
                                    object.box.x + .5f * object.box.width - previous.box.x - .5f * previous.box.width,
                                    object.box.y + .5f * object.box.height - previous.box.y - .5f * previous.box.height};
                                if (scale >= .5f && scale <= 2.f && cv::norm(delta) <= gate) {
                                    match = &object;
                                    ++matches;
                                }
                            }
                        }
                        // Count independent camera frames, never repeated control ticks.
                        // A brief miss preserves the last uncleared reference; it cannot
                        // establish clearance or begin the stopped reobserve window.
                        if (in.capture_scene_us > reference.missing_sample_us) {
                            reference.missing_sample_us = in.capture_scene_us;
                            reference.missing_frames = matches == 0
                                ? std::min(reference.missing_frames + 1, 3) : 0;
                        }
                        if (matches == 0 && reference.missing_frames < 3) {
                            all_clear = false;
                            const float y = previous.box.y + .5f * previous.box.height;
                            if (split.retreat_reference_id < 0 || y > split.retreat_reference_y) {
                                split.retreat_reference_id = previous.id;
                                split.retreat_reference_y = y;
                            }
                            continue;
                        }
                        if (matches != 1 || !match || !match->complete ||
                            targetKind(match->label) == TargetKind::UNKNOWN ||
                            !used.insert(match->id).second) {
                            waiting = true;
                            all_clear = false;
                            capture_scene_.blocker_id = previous.id;
                            capture_scene_.blocker_box = match ? match->box : previous.box;
                            capture_scene_.association = matches == 0 ? "missing"
                                : matches > 1 ? "ambiguous"
                                : !match->complete ? "clipped" : "identity_reused";
                            continue;
                        }
                        observeMotion(*match);
                        previous = *match;
                        const float y = match->box.y + .5f * match->box.height;
                        if (split.retreat_reference_id < 0 || y > split.retreat_reference_y) {
                            split.retreat_reference_id = match->id;
                            split.retreat_reference_y = y;
                        }
                        reference.cleared = splitRetreatCleared(in, *match);
                        all_clear &= reference.cleared;
                    }
                    if (waiting) {
                        interruptMargin();
                        split.margin_stage = Margin::Idle;
                        capture_scene_.confirmations = 0;
                        capture_scene_.stopped_us = tick.now;
                        reason_ = "pixel_split_unwanted_observation_wait";
                        // A hidden unwanted block cannot establish Y280 clearance.
                        // Stop and confirm fresh frames, then use the existing gated
                        // reobserve turn rather than waiting on that identity forever.
                        auto& scene = capture_scene_;
                        if (margin_feedback_missing || !in.sent_velocity_valid || !std::isfinite(in.sent_vx_mps) ||
                            !std::isfinite(in.sent_wz_rps) || std::abs(in.sent_vx_mps) > .001f ||
                            std::abs(in.sent_wz_rps) > .001f) {
                            resetReobserve();
                        } else {
                            if (!scene.reobserve_stopped_us)
                                scene.reobserve_stopped_us = tick.now;
                            if (in.capture_scene_us > scene.reobserve_stopped_us &&
                                in.capture_scene_us > scene.reobserve_sample_us) {
                                scene.reobserve_sample_us = in.capture_scene_us;
                                ++scene.reobserve_frames;
                            }
                            if (scene.reobserve_frames >= t_.corridor_confirm_frames &&
                                tick.now - scene.reobserve_stopped_us >= 250000) {
                                split.recovery_stage = Stage::Turn;
                                split.upper_corner = false;
                                split.recovery_turn_us = 0;
                                split.align_stage = "ui120_turn";
                                reason_ = "first_trip_split_unwanted_reobserve";
                                resetReobserve();
                            }
                        }
                        break;
                    }
                    resetReobserve();
                    capture_scene_.blocker_id = -1;
                    capture_scene_.blocker_box = {};
                    capture_scene_.association.clear();
                    const auto resetBrakeWindow = [&] {
                        capture_scene_.confirmations = 0;
                        capture_scene_.stopped_us = tick.now;
                        capture_scene_.confirm_us = in.capture_scene_us;
                        for (auto& reference : split.retreat_objects)
                            reference.brake_y = reference.observation.box.y +
                                .5f * reference.observation.box.height;
                    };
                    if (!split.retreat_braking && all_clear &&
                        split.margin_stage == Margin::Idle) {
                        split.retreat_braking = true;
                        resetBrakeWindow();
                    }
                    if (split.retreat_braking) {
                        interruptMargin();
                        reason_ = "first_trip_split_retreat_brake";
                        split.align_stage = "ui140_retreat_brake";
                        if (!image_quiet)
                            resetBrakeWindow();
                        if (!confirmCaptureReview(in))
                            break;
                        // If the observed clearance recedes after stopping, continue
                        // at the reduced speed; a prediction never authorizes a turn.
                        split.retreat_braking = false;
                        if (!all_clear)
                            for (auto& reference : split.retreat_objects)
                                reference.motion_sample_us = 0;
                    }
                    if (!all_clear) {
                        interruptMargin();
                        split.margin_stage = Margin::Idle;
                        capture_scene_.confirmations = 0;
                        capture_scene_.stopped_us = tick.now;
                        reason_ = "first_trip_split_ui140_retreat";
                        drive(tick, -split.retreat_speed_mps, 0.f, true);
                        tick.immediate_motion_stop = tick.motion.vx_mps == 0;
                        break;
                    }
                    if (!split.upper_corner && split.margin_stage == Margin::Advancing) {
                        split.align_stage = "ui140_margin";
                        const bool feedback = in.sent_velocity_valid && in.sent_velocity_us &&
                            in.sent_velocity_us <= tick.now &&
                            tick.now - in.sent_velocity_us <= 100000 &&
                            std::isfinite(in.sent_vx_mps) && std::isfinite(in.sent_wz_rps) &&
                            std::abs(in.sent_wz_rps) <= .001f && in.sent_vx_mps <= .001f;
                        const bool continuous = !split.margin_tick_us ||
                            (tick.now > split.margin_tick_us && tick.now - split.margin_tick_us <= 100000);
                        const bool fresh = !split.margin_scene_us || in.capture_scene_us > split.margin_scene_us;
                        if (!feedback || !continuous || !fresh || !in.retreat_safe) {
                            reason_ = !in.retreat_safe ? "retreat_clearance_missing"
                                : !fresh ? "pixel_split_margin_fresh_scene_wait"
                                         : "pixel_split_margin_feedback_wait";
                            interruptMargin();
                            break;
                        }
                        const bool reverse_sent = in.sent_vx_mps < -.001f;
                        if (split.margin_tick_us && split.margin_sent_reverse && reverse_sent)
                            split.margin_used_us += std::min(tick.now - split.margin_tick_us,
                                t_.split_margin_budget_us - std::min(split.margin_used_us, t_.split_margin_budget_us));
                        split.margin_tick_us = tick.now;
                        split.margin_scene_us = in.capture_scene_us;
                        split.margin_sent_reverse = reverse_sent;
                        if (split.margin_used_us < t_.split_margin_budget_us) {
                            reason_ = "first_trip_split_margin_retreat";
                            drive(tick, -.05f, 0.f, true);
                            tick.immediate_motion_stop = tick.motion.vx_mps == 0;
                            break;
                        }
                        split.margin_stage = Margin::Brake;
                        interruptMargin();
                        capture_scene_.confirmations = 0;
                        capture_scene_.stopped_us = tick.now;
                        capture_scene_.confirm_us = in.capture_scene_us;
                    }
                    split.align_stage = split.margin_stage == Margin::Brake
                        ? "ui140_margin_brake" : "ui140_retreat";
                    reason_ = split.margin_stage == Margin::Brake
                        ? "first_trip_split_margin_brake" : "first_trip_split_retreat_brake";
                    if (margin_feedback_missing || !in.sent_velocity_valid ||
                        !std::isfinite(in.sent_vx_mps) || !std::isfinite(in.sent_wz_rps) ||
                        std::abs(in.sent_vx_mps) > .001f ||
                        std::abs(in.sent_wz_rps) > .001f) {
                        capture_scene_.confirmations = 0;
                        capture_scene_.stopped_us = tick.now;
                        break;
                    }
                    if (!confirmCaptureReview(in))
                        break;
                    if (split.keep_injured && !split.upper_corner) {
                        beginPixelSplitReview(tick);
                        reason_ = "injured_split_retreat_stopped";
                        break;
                    }
                    if (!split.upper_corner && split.margin_stage != Margin::Brake &&
                        split.margin_used_us < t_.split_margin_budget_us) {
                        split.margin_stage = Margin::Advancing;
                        interruptMargin();
                        capture_scene_.confirmations = 0;
                        reason_ = "first_trip_split_margin_ready";
                        break;
                    }
                    if (!split.upper_corner) split.margin_stage = Margin::Complete;
                    split.recovery_stage = split.upper_corner ? Stage::CornerTurnRight : Stage::Turn;
                    split.corner_heading = in.heading_rad;
                    split.corner_turn_sample_us = split.corner_stopped_us = 0;
                    split.corner_turn_braking = false;
                    split.recovery_turn_us = 0;
                    capture_scene_.confirmations = 0;
                    reason_ = "first_trip_split_switch_ui120";
                    break;
                }
                bool uncertain = in.capture_dangerous_side_pending;
                for (const auto& object : objects) {
                    const bool relevant =
                        in.splitInventoryOverlaps(object.box, object.label) ||
                        in.capturePathOverlaps(object.box, object.label);
                    if (relevant &&
                        (!object.complete || targetKind(object.label) == TargetKind::UNKNOWN))
                        uncertain = true;
                }
                if (uncertain) {
                    capture_scene_.confirmations = 0;
                    capture_scene_.stopped_us = tick.now;
                    reason_ = "pixel_split_incomplete_inventory";
                    break;
                }
                if (!reviewCaptureLoad(tick))
                    break;
                // Keep the scored group when nominating a new tracker ID.
                const PushObservation::CaptureImageObject* deepest = nullptr;
                for (const auto& object : objects) {
                    if (!targetSelectable(object.label, first_) || !object.selectable ||
                        !captureLoadAllows(object.label) ||
                        (!in.splitInventoryOverlaps(object.box, object.label) &&
                         !(split.upper_corner && object.id == in.target_id && object.complete)))
                        continue;
                    if (!deepest || object.box.y + .5f * object.box.height >
                                        deepest->box.y + .5f * deepest->box.height)
                        deepest = &object;
                }
                // Turning can move every retained candidate out of the inventory,
                // including a target clipped at the image edge. The uncertainty
                // check above still blocks incomplete objects in review regions.
                // Confirm stopped fresh frames, then reacquire at UI120; completeness
                // jitter outside the regions must not restart this observation.
                // This only changes view: capture still needs full normal review.
                if (!deepest &&
                    std::any_of(objects.begin(), objects.end(), [&](const auto& object) {
                        return targetSelectable(object.label, first_) && captureLoadAllows(object.label);
                    })) {
                    reason_ = "pixel_split_target_outside_inventory";
                    if (confirmCaptureReview(in))
                        resumeApproach("pixel_split_resume_ui120");
                    break;
                }
                if (!deepest && std::none_of(objects.begin(), objects.end(), [&](const auto& object) {
                        return targetSelectable(object.label, first_) && captureLoadAllows(object.label);
                    })) {
                    reason_ = "pixel_split_no_target_reobserve";
                    if (confirmCaptureReview(in))
                        resumeApproach("pixel_split_search_ui120");
                    break;
                }
                if (split.target_reselecting || !in.target_valid || target == objects.end() ||
                    !target->complete || in.target_id != trip_context_.target_id_) {
                    if (!split.target_reselecting) {
                        split.target_reselecting = true;
                        trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
                        split.target_id = -1;
                        pixel_alignment_ = {};
                    }
                    split.target_candidate_id = deepest && deepest->complete ? deepest->id : -1;
                    if (split.target_candidate_id >= 0)
                        trip_context_.label_ = deepest->label;
                    capture_scene_.confirmations = 0;
                    capture_scene_.stopped_us = tick.now;
                    capture_scene_.confirm_us = in.capture_scene_us;
                    capture_scene_.status = "incomplete";
                    capture_scene_.detail = "split_target_reselection";
                    reason_ = split.target_candidate_id < 0
                                  ? "first_trip_split_candidate_wait"
                                  : "first_trip_split_target_confirm_wait";
                    if (split.target_candidate_id < 0 || !in.target_valid ||
                        in.target_id != split.target_candidate_id || target == objects.end() ||
                        !target->complete || (in.pixel_selection_managed && !in.pixel_locked))
                        break;
                    trip_context_.target_id_ = trip_context_.capture_target_id_ = in.target_id;
                    trip_context_.label_ = target->label;
                    split.target_id = in.target_id;
                    split.target_reselecting = false;
                    split.target_candidate_id = -1;
                    capture_scene_.target_id = in.target_id;
                    reason_ = "first_trip_split_target_accepted";
                    break; // New stopped frames must confirm separation after acceptance.
                }
                split.align_stage = "ui140_verify";
                // A completed turn starts a new scene. The current boxes are not
                // evidence for associating against the pre-turn image.
                capture_scene_.objects.clear();
                const auto verdict = evaluateCaptureScene(in);
                // A clear split returns to PREPARE for normal approach. Requiring
                // the closing range here would wait at zero speed before the target
                // can enter it. Normal positioning and preclose review still gate closure.
                if (verdict == CaptureSceneVerdict::Clear) {
                    reason_ = "first_trip_split_separation_wait";
                    if (confirmCaptureReview(in)) {
                        split.pending = split.vertical_swing = false;
                        capture_scene_.waiting = false;
                        capture_scene_.started_us = capture_scene_.stopped_us = 0;
                        capture_scene_.confirmations = 0;
                        trip_context_.target_id_ = trip_context_.capture_target_id_ = in.target_id;
                        trip_context_.label_ = target->label;
                        trip_context_.pending_ = capture_scene_.inventory;
                        invalidateEntryPosition();
                        search_entry_.target_id = in.target_id;
                        search_entry_.label = in.label;
                        selectSearchEntry(in);
                        enter(PushState::PREPARE, tick.now, "first_trip_split_separation_recheck");
                    }
                    break;
                }
                capture_scene_.confirmations = 0;
                capture_scene_.stopped_us = tick.now;
                // A definite post-split risk requires another separation even when
                // the retained target has moved outside the closing Y range.
                // That range gates closure, not whether the scene still needs splitting.
                if (verdict == CaptureSceneVerdict::Incomplete) {
                    reason_ = "first_trip_split_observation_wait";
                    break;
                }
                selectPixelSplitDirection(in);
                if (t_.pile_max_runs > 0 && split.runsFor(split.upper_corner) >= t_.pile_max_runs) {
                    failStop(tick, "first_trip_split_limit");
                    break;
                }
                ++split.runs;
                if (split.upper_corner) ++split.corner_runs;
                capture_scene_.before_split = capture_scene_.objects;
                reason_ = "first_trip_split_retry_retreat";
                split.recovery_stage = split.keep_injured && !split.upper_corner
                    ? Stage::InjuredTurn : Stage::Retreat;
                split.recovery_turn_us = 0;
                split.align_stage = split.recovery_stage == Stage::InjuredTurn
                    ? "injured_turn_before_retreat" : "ui140_retreat";
                split.recovery_view_us = tick.now;
                break;
            }
            RecoveryContext context;
            context.now_us = tick.now;
            context.elapsed_us = tick.elapsed;
            context.since_step_us = tick.now - actionProgress().started_us;
            context.travel_m = actionProgress().travel;
            context.approach_forward_m = trip_context_.approach_forward_;
            context.motion_ratio = motionRatio();
            context.corridor_retreat = corridor_retreat_;
            context.observed_zone = tick.observed_zone;
            context.retreat_safe = tick.in.retreat_safe;
            context.heading_valid = tick.in.heading_valid;
            context.heading_rad = tick.in.heading_rad;
            context.lost_hold = state_ == PushState::LOST_HOLD;
            if (tick.observed_zone) {
                context.zone_origin = tick.in.zone_estimate.bodyToZone({0, 0});
                context.zone_forward =
                    tick.in.zone_estimate.bodyToZone({0, 1}) - context.zone_origin;
                context.zone_geometry_id = tick.in.zone_estimate.geometry_id;
            }
            const auto recovery = recovery_.update(t_, context);
            if (recovery.outcome == RecoveryOutcome::Fault) {
                fail(recovery.reason.c_str());
                break;
            }
            if (recovery.outcome == RecoveryOutcome::Scan) {
                if (recovery.blacklist && !search_phase_.first_trip_split_.pending)
                    blacklist(tick.now);
                scanAfterClear(tick, recovery.reason.c_str());
                break;
            }
            if (recovery.reset_travel)
                actionProgress().travel = 0;
            if (recovery.outcome == RecoveryOutcome::Reverse) {
                if (!recovery.reason.empty())
                    reason_ = recovery.reason;
                drive(tick, recovery.vx, 0);
            }
            break;
        }
        default:
            break;
    }
}
} // namespace rescue
