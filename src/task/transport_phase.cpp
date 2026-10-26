#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
namespace rescue {
void PushTask::updateTransportPhase(TaskTick& tick) {
    switch (transport_phase_.action) {
        case TransportAction::Carry: {
            if (tick.now - transport_phase_.carry_us_ > t_.carry_budget_us) {
                enter(PushState::ABORT_DROP, tick.now, "carry_timeout");
                break;
            }
            if (carryLogic()) {
                // TEMP_ASSUMPTION: the hold is assumed (no NEAR checks). With a fresh zone plan
                // drive the normal gate approach; otherwise search briefly, then a bounded straight
                // leg to an assumed drop point.
                actionProgress().step = 0;
                const bool plan = zoneOk(tick.in) && tick.in.navigation_timestamp_us &&
                                  tick.now >= tick.in.navigation_timestamp_us &&
                                  tick.now - tick.in.navigation_timestamp_us <= 200000 &&
                                  tick.in.drop_plan_valid && tick.in.carry_plan_valid &&
                                  std::isfinite(tick.in.drop_centre_zone.x) &&
                                  std::isfinite(tick.in.drop_centre_zone.y) &&
                                  std::isfinite(tick.in.carry_waypoint_body.x) &&
                                  std::isfinite(tick.in.carry_waypoint_body.y);
                if (plan && !actionProgress().blind) {
                    transport_phase_.zone_search_started_us_ = 0;
                    lockDropHalf("drop_plan");
                    if (!transport_phase_.drop_locked_) {
                        transport_phase_.drop_centre_zone_ = tick.in.drop_centre_zone;
                        transport_phase_.drop_locked_ = true;
                    }
                    const cv::Point2f gate = tick.in.zone_estimate.zoneToBody(
                        {halfX(), -t_.gate_clearance_m - holdCenter()});
                    if (std::hypot(gate.x, gate.y) <= t_.gate_tolerance_m) {
                        enter(PushState::GATE, tick.now, "gate_reached");
                        break;
                    }
                    reason_ = "carrying_to_own_zone";
                    steer(
                        tick,
                        std::max(
                            .03f,
                            std::min(t_.carry_speed, float(cv::norm(tick.in.carry_waypoint_body)))),
                        std::atan2(-tick.in.carry_waypoint_body.x, tick.in.carry_waypoint_body.y));
                    break;
                }
                // 视觉闭环：YOLO-pose区域位姿锚定在里程系；沿区域中轴“追踪前视点”接近，丢失视野时用IMU航向+缩放位移继续推算。
                if (zone_.valid(tick.now)) {
                    const ZoneEstimate z = zone_.predicted(tick.now);
                    const cv::Point2f hold = z.bodyToZone({0, holdCenter()});
                    CarryApproachContext context;
                    context.now_us = tick.now;
                    context.hold = hold;
                    context.carrot = z.zoneToBody(
                        {halfX(),
                         std::min(hold.y + t_.assume_lookahead_m, -t_.assume_release_gap_m)});
                    context.fresh_fix = tick.now - vref_seen_us_ <= 400000;
                    context.imu_blocked_us = tick.in.imu_blocked_us;
                    context.hold_center = holdCenter();
                    vref_used_ = true;
                    transport_phase_.zone_search_started_us_ = 0;
                    lockDropHalf("assumed_trip");
                    const auto approach = transport_phase_.carry_approach_.update(t_, context);
                    if (approach.outcome == CarryApproachOutcome::RaiseRelease) {
                        noteDropZone(approach.drop_event, hold);
                        enter(PushState::RAISE_RELEASE, tick.now, approach.reason);
                        break;
                    }
                    reason_ = approach.reason;
                    if (approach.blend) {
                        drive(
                            tick,
                            t_.carry_speed *
                                std::max(.4f, std::cos(std::min(std::abs(approach.heading), 1.f))),
                            t_.heading_gain * approach.heading);
                    } else
                        steer(tick, t_.carry_speed, approach.heading);
                    break;
                }
                if (!t_.assume_all_safe) {
                    carrySearch(tick);
                    break;
                } // no verified own zone: never release blind
                if (!actionProgress().blind) {
                    if (!transport_phase_.zone_search_started_us_)
                        transport_phase_.zone_search_started_us_ = tick.now;
                    if (tick.now - transport_phase_.zone_search_started_us_ <
                        t_.assume_zone_search_us) {
                        reason_ = "assumed_zone_search";
                        scanSpin(tick);
                        break;
                    }
                    actionProgress().blind = true;
                    actionProgress().travel = 0;
                }
                if (actionProgress().travel >= t_.assume_carry_m) {
                    enter(PushState::RAISE_RELEASE, tick.now, "assumed_drop_point");
                    break;
                }
                reason_ = "assumed_blind_carry";
                drive(tick, t_.carry_speed, 0);
                break;
            }
            if (!zoneOk(tick.in)) {
                if (!transport_phase_.zone_search_started_us_)
                    transport_phase_.zone_search_started_us_ = tick.now;
                if (tick.now - transport_phase_.zone_search_started_us_ >
                    t_.zone_search_budget_us) {
                    fail("own_zone_search_timeout");
                    break; // hold the load and stop; no blind forward travel
                }
            } else
                transport_phase_.zone_search_started_us_ = 0;
            if (actionProgress().step) { // stopped at NEAR
                const int h = holdCheck(tick);
                if (h < 0) {
                    enter(PushState::LOST_HOLD, tick.now, "hold_lost");
                    break;
                }
                if (h > 0) {
                    actionProgress().step = 0;
                    actionProgress().confirmations = 0;
                    transport_phase_.hold_seen_us_ = tick.now;
                    transport_phase_.carried_ = 0;
                }
                break;
            }
            // At FAR the holding region is normally out of view: unknown, not lost.
            const int h = holdSeen(tick.in);
            if (h == 0 && ++actionProgress().hold_misses > t_.hold_grace_frames) {
                enter(PushState::LOST_HOLD, tick.now, "hold_lost");
                break;
            }
            if (h == 1) {
                actionProgress().hold_misses = 0;
                transport_phase_.hold_seen_us_ = tick.now;
                transport_phase_.carried_ = 0;
            }
            if (transport_phase_.carried_ >= t_.hold_check_m ||
                tick.now - transport_phase_.hold_seen_us_ > t_.hold_check_interval_us) {
                actionProgress().step = 1;
                actionProgress().confirmations = 0;
                actionProgress().hold_misses = 0;
                break;
            }
            if (!zoneOk(tick.in)) {
                if (evadeEnemy(tick))
                    break;
                reason_ = "searching_own_zone";
                scanSpin(tick);
                break; // rotate with closed gripper; never drive blind toward a guessed zone
            }
            reason_ = "carrying_to_own_zone";
            const bool fresh_plan = tick.in.navigation_timestamp_us &&
                                    tick.now >= tick.in.navigation_timestamp_us &&
                                    tick.now - tick.in.navigation_timestamp_us <= 200000;
            if (!fresh_plan || !tick.in.drop_plan_valid || !tick.in.carry_plan_valid ||
                !std::isfinite(tick.in.drop_centre_zone.x) ||
                !std::isfinite(tick.in.drop_centre_zone.y) ||
                !std::isfinite(tick.in.carry_waypoint_body.x) ||
                !std::isfinite(tick.in.carry_waypoint_body.y)) {
                reason_ = "carry_plan_missing";
                break;
            }
            if (!transport_phase_.drop_locked_) {
                transport_phase_.drop_centre_zone_ = tick.in.drop_centre_zone;
                transport_phase_.drop_locked_ = true;
            }
            if (cv::norm(tick.in.drop_centre_zone - transport_phase_.drop_centre_zone_) > .005f) {
                reason_ = "drop_plan_changed";
                break;
            }
            // Park the rotation centre so that, once facing in, the held centre sits gate_clearance
            // before the correct half: turning in place there does not sweep the load sideways.
            const auto& z = tick.in.zone_estimate;
            const cv::Point2f gate = z.zoneToBody({halfX(), -t_.gate_clearance_m - holdCenter()});
            const float dist = std::hypot(gate.x, gate.y);
            if (dist <= t_.gate_tolerance_m) {
                enter(PushState::GATE, tick.now, "gate_reached");
                break;
            }
            const float bearing =
                std::atan2(-tick.in.carry_waypoint_body.x, tick.in.carry_waypoint_body.y);
            steer(tick,
                  std::max(.03f,
                           std::min(t_.carry_speed, float(cv::norm(tick.in.carry_waypoint_body)))),
                  bearing);
            break;
        }
        case TransportAction::Gate: {
            if (tick.now - transport_phase_.carry_us_ > t_.carry_budget_us) {
                enter(PushState::ABORT_DROP, tick.now, "carry_timeout");
                break;
            }
            if (carryLogic() && actionProgress().step) {
                enter(PushState::ENTER, tick.now, "assumed_gate_hold");
                break;
            }
            // Aligned and counted at NEAR: enter only on a confirmed enclosure.
            if (actionProgress().step) {
                const int h = holdCheck(tick);
                if (h < 0)
                    enter(PushState::LOST_HOLD, tick.now, "hold_lost");
                else if (h > 0)
                    enter(PushState::ENTER, tick.now, "gate_aligned_frame_down");
                break;
            }
            if (!carryLogic() && holdSeen(tick.in) == 0 &&
                ++actionProgress().hold_misses > t_.hold_grace_frames) {
                enter(PushState::LOST_HOLD, tick.now, "hold_lost");
                break;
            }
            if (!zoneOk(tick.in)) {
                if (lostTarget(tick))
                    enter(PushState::CARRY, tick.now, "zone_lost");
                break;
            }
            actionProgress().misses = 0;
            const auto& z = tick.in.zone_estimate;
            const float yaw = wrapAngle(z.yaw_body_rad); // heading error to face into the zone
            if (std::abs(yaw) > t_.gate_yaw_rad) {
                actionProgress().confirmations = 0;
                drive(tick, 0, t_.heading_gain * yaw);
                break;
            }
            const cv::Point2f hold = z.bodyToZone({0, holdCenter()});
            if (std::abs(hold.x - halfX()) > t_.gate_lateral_m ||
                std::abs(hold.y + t_.gate_clearance_m) > 2 * t_.gate_tolerance_m) {
                enter(PushState::CARRY, tick.now, "gate_offset");
                break;
            }
            // Baseline counts make the delivery check differential: objects already inside never
            // count.
            if (!tick.in.zone_counts_valid || !tick.in.zone_inventory_complete ||
                tick.in.zone_supply_count < 0 || tick.in.zone_injured_count < 0) {
                // TEMP_ASSUMPTION: without zone counts the delivery is credited by assumption
                // later.
                if (carryLogic()) {
                    transport_phase_.baseline_ = transport_phase_.baseline_other_ = -1;
                    actionProgress().step = 1;
                    break;
                }
                actionProgress().confirmations = 0;
                break;
            }
            if (++actionProgress().confirmations < t_.confirm_frames)
                break;
            transport_phase_.baseline_ = trip_context_.trip_.injured ? tick.in.zone_injured_count
                                                                     : tick.in.zone_supply_count;
            transport_phase_.baseline_other_ = trip_context_.trip_.injured
                                                   ? tick.in.zone_supply_count
                                                   : tick.in.zone_injured_count;
            transport_phase_.delivery_geometry_id_ = tick.in.zone_estimate.geometry_id;
            transport_phase_.delivery_verification_.resetInventoryEvidence();
            actionProgress().step = 1;
            actionProgress().confirmations = 0;
            actionProgress().hold_misses = 0; // the robot stays put: the baseline holds
            break;
        }
        case TransportAction::Release: {
            if (!tick.in.opponent_zone_clear) {
                reason_ = "release_opponent_clearance_missing";
                if (tick.elapsed > t_.gripper_timeout_us)
                    fail("release_clearance_timeout");
                break;
            }
            commandFrame(1, tick.now);
            const int p = pitchWait(tick.in, tick.now);
            if (p < 0) {
                fail("camera_pitch_timeout");
                break;
            }
            const int g = frameWait(tick.in, 1, tick.now);
            if (g < 0)
                fail("gripper_open_timeout");
            else if (g > 0 && p > 0) {
                // TEMP_ASSUMPTION：快到区域先松开，再用下压(+20)姿态把块向区内推一段，之后再次抬框退出。
                if (carryLogic() && !transport_phase_.assume_pushed_ && t_.assume_push_m > 0) {
                    // 先退一小段让框离开块，再放下框从块后方推入；否则框会重新罩住块一起往里带。
                    transport_phase_.release_retreat_.beginReleaseBackOff(
                        zone_.valid(tick.now)
                            ? zone_.predicted(tick.now).bodyToZone({0, holdCenter()}).y
                            : 0.f);
                    enter(PushState::BACK_OUT, tick.now, "assumed_release_back_off");
                } else
                    enter(PushState::BACK_OUT, tick.now, "frame_raised_at_drop");
            }
            break;
        }
        case TransportAction::Enter: {
            // Carry the enclosed load with frame DOWN into the correct half.
            commandFrame(0, tick.now);
            const int lowered = frameWait(tick.in, 0, tick.now);
            if (lowered <= 0) {
                if (lowered < 0)
                    fail("frame_lower_timeout");
                break;
            }
            if (carryLogic() && !transport_phase_.assume_pushed_) {
                trackCargo(tick.in, tick.now);
                PushInContext context;
                context.now_us = tick.now;
                context.elapsed_us = tick.elapsed;
                context.travel_m = actionProgress().travel;
                context.imu_blocked_us = tick.in.imu_blocked_us;
                context.anchored_zone = zone_.valid(tick.now);
                if (context.anchored_zone) {
                    const auto zone = zone_.predicted(tick.now);
                    context.hold = zone.bodyToZone({0, holdCenter()});
                    context.zone_yaw = zone.yaw_body_rad;
                }
                context.half_x = halfX();
                context.hold_center = holdCenter();
                context.half_injured = transport_phase_.half_injured_;
                context.visual_reference_seen_us = vref_seen_us_;
                context.repush_count = transport_phase_.repush_n_;
                const auto push = transport_phase_.push_in_.update(
                    t_,
                    context,
                    {cargoVisible(tick.now),
                     transport_phase_.cargo_fix_,
                     transport_phase_.cargo_clipped_,
                     {transport_phase_.cargo_rear_x_, transport_phase_.cargo_rear_y_},
                     transport_phase_.cargo_body_,
                     transport_phase_.cargo_seen_us_,
                     transport_phase_.cargo_ref_us_});
                if (push.reset_cargo_reference)
                    transport_phase_.cargo_ref_us_ = 0;
                if (push.outcome == PushInOutcome::Repush)
                    ++transport_phase_.repush_n_;
                for (const auto& event : push.events) {
                    switch (event.kind) {
                        case PushInEventKind::Cargo:
                            noteCargo(event.text.c_str(), event.position, event.visible);
                            break;
                        case PushInEventKind::DropZone:
                            noteDropZone(event.text.c_str(), event.position);
                            break;
                        case PushInEventKind::Text:
                            dropzone_evt_ += event.text;
                            break;
                    }
                }
                if (!push.reason.empty())
                    reason_ = push.reason;
                if (push.outcome == PushInOutcome::Repush) {
                    transport_phase_.release_retreat_.beginRepushBackOff(
                        context.anchored_zone ? context.hold.y : 0.f);
                    enter(PushState::BACK_OUT, tick.now, "");
                } else if (push.outcome == PushInOutcome::Complete) {
                    transport_phase_.assume_pushed_ = true;
                    enter(PushState::RAISE_RELEASE, tick.now, "");
                } else if (push.drive)
                    drive(tick, push.vx, push.wz);
                break;
            }
            EnterContext context;
            context.elapsed_us = tick.elapsed;
            context.zone_ok = zoneOk(tick.in);
            // lostTarget(tick) advances the parent's missed-frame counter: it must only run when
            // the zone is unusable, exactly as the inline branch did.
            if (!context.zone_ok)
                context.lost_target = lostTarget(tick);
            context.carry = carryLogic();
            context.drop_locked = transport_phase_.drop_locked_;
            context.drop_centre_y = transport_phase_.drop_centre_zone_.y;
            if (context.zone_ok) {
                const auto& z = tick.in.zone_estimate;
                context.hold = z.bodyToZone({0, holdCenter()});
                context.half_x = halfX();
                context.zone_yaw = z.yaw_body_rad;
            }
            const auto strict = transport_phase_.enter_.update(t_, context);
            if (strict.reset_misses)
                actionProgress().misses = 0;
            switch (strict.outcome) {
                case EnterOutcome::RaiseRelease:
                    enter(PushState::RAISE_RELEASE, tick.now, strict.reason);
                    break;
                case EnterOutcome::AbortDrop:
                    enter(PushState::ABORT_DROP, tick.now, strict.reason);
                    break;
                case EnterOutcome::Continue:
                    if (strict.drive)
                        drive(tick, strict.vx, strict.wz);
                    break;
            }
            break;
        }
        case TransportAction::BackOut: {
            // Keep the frame raised so reversing leaves the load behind.
            commandFrame(1, tick.now);
            const int g = frameWait(tick.in, 1, tick.now);
            if (g <= 0) {
                if (g < 0)
                    fail("gripper_open_timeout");
                break;
            } // never drag the load out closed
            ReleaseRetreatContext context{
                tick.elapsed,
                actionProgress().travel,
                tick.observed_zone,
                tick.observed_zone &&
                    tick.in.zone_estimate.bodyToZone({0, t_.mouth_y_m}).y <= t_.mouth_clear_y_m,
                false,
                0,
                0,
                1,
                actionProgress().blind,
                transport_phase_.repush_n_,
                halfX()};
            ReleasedCargoEvidence cargo;
            if (transport_phase_.release_retreat_.needsCargoTracking()) {
                trackCargo(tick.in, tick.now);
                cargo = {cargoVisible(tick.now),
                         transport_phase_.cargo_fix_,
                         transport_phase_.cargo_clipped_,
                         transport_phase_.cargo_rear_x_,
                         transport_phase_.cargo_rear_y_,
                         transport_phase_.cargo_last_ts_};
                context.anchored_zone = zone_.valid(tick.now);
                context.hold_y = context.anchored_zone
                                     ? zone_.predicted(tick.now).bodyToZone({0, holdCenter()}).y
                                     : 0.f;
                context.back_target_m = backTarget(tick.now);
                context.motion_ratio = drRatio(tick.now);
            }
            const auto retreat = transport_phase_.release_retreat_.update(t_, context, cargo);
            if (retreat.blind_back_out)
                actionProgress().blind = true;
            if (!retreat.reason.empty())
                reason_ = retreat.reason;
            switch (retreat.outcome) {
                case ReleaseRetreatOutcome::Continue:
                    if (retreat.reverse)
                        drive(tick, -t_.back_speed, 0);
                    break;
                case ReleaseRetreatOutcome::PushIn:
                    transport_phase_.cargo_ref_us_ = 0;
                    transport_phase_.push_in_.prepare(t_.assume_align_push);
                    enter(PushState::ENTER, tick.now, "");
                    break;
                case ReleaseRetreatOutcome::InsideAfterRelease:
                    noteCargo("inside_after_release",
                              {transport_phase_.cargo_rear_x_, transport_phase_.cargo_rear_y_},
                              true);
                    transport_phase_.assume_pushed_ = true;
                    transport_phase_.delivery_verification_.clearVisualCounts();
                    enter(PushState::VERIFY_DELIVERY, tick.now, "");
                    break;
                case ReleaseRetreatOutcome::VerifyDelivery:
                    enter(PushState::VERIFY_DELIVERY, tick.now, "");
                    break;
                case ReleaseRetreatOutcome::Fault:
                    fail(retreat.reason.c_str());
                    break;
            }
            break;
        }
        case TransportAction::Verify: {
            commandFrame(1, tick.now);
            const int expected = trip_context_.trip_.total();
            DeliveryCargoEvidence cargo;
            if (carryLogic()) {
                cv::Point2f body;
                int index = -1;
                cargo.available =
                    cargoZone(tick.in, tick.now, body, cargo.rear, cargo.fixed, &index);
                const auto image = imgAt(tick.in, index);
                cargo.clipped = image.clipped;
                cargo.front_valid = image.front_valid;
                cargo.front_px = image.front_px;
                cargo.bottom_px = image.bottom_px;
                cargo.height_px = image.height_px;
                cargo.other_inside = cargoInZone(tick.in, tick.now);
            }
            const auto verification = transport_phase_.delivery_verification_.update(
                tick.in,
                t_,
                {tick.elapsed,
                 trip_context_.trip_,
                 transport_phase_.baseline_,
                 transport_phase_.baseline_other_,
                 transport_phase_.delivery_geometry_id_,
                 tick.observed_zone,
                 halfX(),
                 transport_phase_.half_injured_,
                 transport_phase_.cargo_fix_,
                 transport_phase_.repush_n_},
                cargo);
            dropzone_evt_ += verification.events;
            if (verification.outcome == DeliveryVerificationOutcome::Continue)
                break;
            if (verification.outcome == DeliveryVerificationOutcome::Repush) {
                ++transport_phase_.repush_n_;
                transport_phase_.cargo_ref_us_ = 0;
                transport_phase_.assume_pushed_ = false;
                transport_phase_.push_in_.prepare(t_.assume_align_push);
                enter(PushState::ENTER, tick.now, verification.reason.c_str());
                break;
            }
            transport_phase_.expected = expected;
            transport_phase_.credited = verification.credited;
            transport_phase_.visual = verification.visual;
            const int credited = verification.credited;
            total_ += credited;
            if (credited > 0) {
                attempt_started_ = tick.now;
                bounded_approach_ = bounded_retreat_ = 0;
            }
            if (credited == expected && credited > 0) {
                search_phase_.near_failures_ = 0;
                search_phase_.mid_clear_n_ = 0;
            }
            if (credited > 0 && credited == expected &&
                trip_context_.trip_.ordinary == trip_context_.trip_.total())
                first_ = true;
            transport_phase_.delivery_visual_ = verification.visual;
            reason_ = verification.reason;
            if (t_.demo_carry_once && t_.assume_all_safe)
                requestPhase(PhaseOutcome::SearchReady, ""); // show the next-trip turn
            else if (t_.demo_carry_once) {
                failStop(tick,
                         credited == expected && credited > 0 ? "demo_delivery_complete"
                                                              : "demo_delivery_unconfirmed");
            } else
                requestPhase(PhaseOutcome::SearchReady, "");
            break;
        }
        default:
            break;
    }
}
} // namespace rescue
