#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
namespace rescue {
void PushTask::updateRecoveryPhase(TaskTick& tick) {
    switch (state_) {
        case PushState::CAPTURE_FAIL:
        case PushState::LOST_HOLD:
        case PushState::ABORT_DROP: {
            if (t_.pixel_grasp_enabled && !first_ &&
                targetKind(trip_context_.label_) == TargetKind::ORDINARY &&
                !search_phase_.first_trip_split_.pending) {
                tick.immediate_motion_stop = true;
                commandFrame(1, tick.now);
                if (!tick.in.gripper_done || tick.in.gripper_feedback_open != 1) {
                    reason_ = "first_trip_recovery_open_wait";
                    if (tick.elapsed > t_.gripper_timeout_us)
                        failStop(tick, "first_trip_recovery_open_timeout");
                    break;
                }
                if (reviewFirstTripCapture(tick, "approach_recovery")) {
                    trip_context_.pending_ = {};
                    trip_context_.pending_.ordinary = 1;
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
                // UI140 straight retreat -> UI120 timed turn -> stopped UI140 review.
                tick.immediate_motion_stop = true;
                auto& split = search_phase_.first_trip_split_;
                using Stage = SearchPhaseData::FirstTripSplit::RecoveryStage;
                if (tick.now - split.started_us > t_.pile_budget_us) {
                    failStop(tick, "first_trip_vertical_split_timeout");
                    break;
                }
                capture_scene_.stage = "postsplit_before_approach";
                if (split.recovery_stage == Stage::Turn) {
                    split.align_stage = "ui120_turn";
                    if (split.recovery_turn_us && tick.now - split.recovery_turn_us >= 500000) {
                        split.recovery_stage = Stage::Verify;
                        split.recovery_view_us = tick.now;
                        split.recovery_turn_us = 0;
                        split.align_stage = "ui140_verify";
                        split.target_reselecting = true;
                        split.target_candidate_id = -1;
                        split.target_id = -1;
                        trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
                        pixel_alignment_ = {};
                        capture_scene_.target_id = -1;
                        capture_scene_.status = "incomplete";
                        capture_scene_.detail = "split_target_reselection";
                        capture_scene_.confirmations = 0;
                        capture_scene_.stopped_us = tick.now;
                        reason_ = "first_trip_split_turn_stopped";
                        break;
                    }
                    reason_ = "first_trip_split_ui120_right";
                    drive(tick, 0.f, -.3f, true);
                    if (tick.motion.wz_rps != 0) {
                        // This timed split turn has its own speed, beyond the alignment cap.
                        tick.motion.wz_rps = -std::min(.3f, t_.max_wz);
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
                        return object.id == in.target_id && object.label == "ordinary_supply";
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
                    break;
                }
                // Use the existing UI140 ordinary inventory region, not the full
                // image or the old top-edge retreat condition. Count whole-box overlap.
                const auto ordinary_inside =
                    std::count_if(objects.begin(), objects.end(), [&](const auto& object) {
                        return object.label == "ordinary_supply" &&
                               in.ordinaryInventoryOverlaps(object.box);
                    });
                capture_scene_.objects.clear();
                for (const auto& object : objects)
                    if (object.label == "ordinary_supply")
                        capture_scene_.objects.push_back(object);
                capture_scene_.target_id = in.target_id;
                capture_scene_.pitch = in.camera_pitch_cdeg;
                capture_scene_.sample_us = in.capture_scene_us;
                // Reselect only within the current ordinary inventory, independently of old IDs.
                const PushObservation::CaptureImageObject* deepest = nullptr;
                for (const auto& object : objects) {
                    if (object.label != "ordinary_supply" ||
                        !in.ordinaryInventoryOverlaps(object.box))
                        continue;
                    if (!deepest || object.box.y + .5f * object.box.height >
                                        deepest->box.y + .5f * deepest->box.height)
                        deepest = &object;
                }
                const bool deepest_changed_at_stop =
                    split.recovery_stage == Stage::Retreat && deepest &&
                    deepest->id != in.target_id &&
                    deepest->box.y + .5f * deepest->box.height <= 280.f;
                if (split.target_reselecting || !in.target_valid || target == objects.end() ||
                    !target->complete || in.target_id != trip_context_.target_id_ ||
                    deepest_changed_at_stop) {
                    if (!split.target_reselecting) {
                        split.target_reselecting = true;
                        trip_context_.target_id_ = trip_context_.capture_target_id_ = -1;
                        split.target_id = -1;
                        pixel_alignment_ = {};
                    }
                    split.target_candidate_id = deepest && deepest->complete ? deepest->id : -1;
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
                    split.target_id = in.target_id;
                    split.target_reselecting = false;
                    split.target_candidate_id = -1;
                    capture_scene_.target_id = in.target_id;
                    reason_ = "first_trip_split_target_accepted";
                    break; // New stopped frames must confirm separation after acceptance.
                }
                if (split.recovery_stage == Stage::Retreat) {
                    if (!deepest || !deepest->complete) {
                        reason_ = "first_trip_split_observation_wait";
                        break;
                    }
                    const float y = deepest->box.y + .5f * deepest->box.height;
                    if (y > 280.f) {
                        capture_scene_.confirmations = 0;
                        capture_scene_.stopped_us = tick.now;
                        reason_ = "first_trip_split_ui140_retreat";
                        drive(tick, -.01f, 0.f, true);
                        tick.immediate_motion_stop = tick.motion.vx_mps == 0;
                        break;
                    }
                    if (deepest->id != in.target_id || !in.target_grasp_range_inside) {
                        reason_ = "first_trip_split_grasp_position_wait";
                        break;
                    }
                    reason_ = "first_trip_split_retreat_brake";
                    if (!in.sent_velocity_valid || std::abs(in.sent_vx_mps) > .001f ||
                        std::abs(in.sent_wz_rps) > .001f) {
                        capture_scene_.confirmations = 0;
                        capture_scene_.stopped_us = tick.now;
                        break;
                    }
                    if (!confirmCaptureReview(in))
                        break;
                    split.recovery_stage = Stage::Turn;
                    split.recovery_turn_us = 0;
                    capture_scene_.confirmations = 0;
                    reason_ = "first_trip_split_switch_ui120";
                    break;
                }
                split.align_stage = "ui140_verify";
                if (ordinary_inside == 1 && in.ordinaryInventoryOverlaps(target->box) &&
                    in.target_grasp_range_inside) {
                    if (capture_scene_.status != "clear" ||
                        capture_scene_.detail != "single_ordinary_inside") {
                        capture_scene_.confirmations = 0;
                        capture_scene_.stopped_us = tick.now;
                        capture_scene_.confirm_us = in.capture_scene_us;
                    }
                    capture_scene_.status = "clear";
                    capture_scene_.detail = "single_ordinary_inside";
                    reason_ = "first_trip_split_separation_wait";
                    if (confirmCaptureReview(in)) {
                        split.pending = split.vertical_swing = false;
                        capture_scene_.waiting = false;
                        capture_scene_.started_us = capture_scene_.stopped_us = 0;
                        capture_scene_.confirmations = 0;
                        trip_context_.target_id_ = trip_context_.capture_target_id_ = in.target_id;
                        trip_context_.pending_ = {};
                        trip_context_.pending_.ordinary = 1;
                        enter(PushState::RUSH, tick.now, "first_trip_split_separation_confirmed");
                    }
                    break;
                }
                capture_scene_.status = ordinary_inside > 1 ? "risk" : "incomplete";
                capture_scene_.detail =
                    ordinary_inside > 1 ? "multiple_ordinary_inside" : "ordinary_target_not_inside";
                capture_scene_.confirmations = 0;
                capture_scene_.stopped_us = tick.now;
                if (ordinary_inside == 0) {
                    reason_ = "first_trip_split_observation_wait";
                    break;
                }
                reason_ = "first_trip_split_retry_retreat";
                split.recovery_stage = Stage::Retreat;
                split.align_stage = "ui140_retreat";
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
