#include "rescue/task/push_task.hpp"
#include "task_tick.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
namespace rescue {
void PushTask::beginCarryPlacement(uint64_t now, const char* why) {
    enter(PushState::CARRY, now, why);
    transport_phase_.placing_ = true;
    transport_phase_.deposit_stage_ = TransportPhaseData::DepositStage::Stopping;
    transport_phase_.deposit_imu_.reset();
    transport_phase_.deposit_started_us_ = now;
    transport_phase_.deposit_stopped_us_ = 0;
    transport_phase_.floor_since_us_ = transport_phase_.floor_last_us_ = 0;
    transport_phase_.floor_frames_ = 0;
}

void PushTask::prepareCarryPlacement(TaskTick& tick) {
    const auto& z = tick.in.zone_estimate;
    const float yaw = wrapAngle(z.yaw_body_rad);
    if (std::abs(yaw) > t_.gate_yaw_rad) {
        actionProgress().confirmations = 0;
        drive(tick, 0, t_.heading_gain * yaw);
        return;
    }
    // Save the destination's pre-delivery inventory while still in Carry.
    if (!tick.in.zone_counts_valid || !tick.in.zone_inventory_complete ||
        tick.in.zone_supply_count < 0 || tick.in.zone_injured_count < 0) {
        actionProgress().confirmations = 0;
        if (!carryLogic())
            return;
        transport_phase_.baseline_ = transport_phase_.baseline_other_ = -1;
    } else {
        if (++actionProgress().confirmations < t_.confirm_frames)
            return;
        transport_phase_.baseline_ = transport_phase_.half_injured_
                                        ? tick.in.zone_injured_count : tick.in.zone_supply_count;
        transport_phase_.baseline_other_ = transport_phase_.half_injured_
                                              ? tick.in.zone_supply_count : tick.in.zone_injured_count;
    }
    transport_phase_.delivery_geometry_id_ = z.geometry_id;
    transport_phase_.delivery_verification_.resetInventoryEvidence();
    if (!carryLogic()) {
        actionProgress().step = 2; // final near-view hold confirmation, still in Carry
        actionProgress().confirmations = actionProgress().hold_misses = 0;
        return;
    }
    tick.immediate_motion_stop = true;
    beginCarryPlacement(tick.now, "carry_at_selected_half");
}

void PushTask::updateFloorPlacement(TaskTick& tick) {
    auto& d = transport_phase_;
    const auto& in = tick.in;
    const auto& floor = in.pushboard_floor;
    using Stage = TransportPhaseData::DepositStage;
    tick.immediate_motion_stop = true;
    const auto resetConfirmation = [&] {
        d.floor_since_us_ = 0; d.floor_frames_ = 0;
        d.deposit_stopped_us_ = 0;
    };
    const auto stop = [&](const char* why) {
        tick.motion.vx_mps = tick.motion.wz_rps = 0;
        failStop(tick, why);
    };
    if (tick.now - d.deposit_started_us_ > t_.enter_budget_us) {
        stop("deposit_floor_timeout");
        return;
    }
    if (!in.sent_velocity_valid || !std::isfinite(in.sent_vx_mps) ||
        !std::isfinite(in.sent_wz_rps) || in.sent_vx_mps < -.001f) {
        stop("deposit_floor_feedback_missing");
        return;
    }
    const bool stopped = std::abs(in.sent_vx_mps) < .001f && std::abs(in.sent_wz_rps) < .001f;
    const bool fresh = floor.valid && floor.sample_us > d.deposit_started_us_ &&
        floor.sample_us <= tick.now && tick.now - floor.sample_us <= 200000 &&
        in.camera_pitch_stable && in.camera_pitch_cdeg != kCameraPitchInvalid &&
        std::abs(int(in.camera_pitch_cdeg) - int(t_.near_pitch_cdeg)) <= t_.pitch_tolerance_cdeg;
    if (!fresh) {
        resetConfirmation();
        reason_ = "deposit_floor_observation_wait";
        return;
    }
    if (d.deposit_stage_ == Stage::Stopping) {
        reason_ = "deposit_floor_initial_stop";
        if (!stopped || !in.heading_valid || !std::isfinite(in.heading_rad)) {
            d.deposit_stopped_us_ = 0;
            return;
        }
        if (!d.deposit_stopped_us_)
            d.deposit_stopped_us_ = tick.now;
        if (tick.now - d.deposit_stopped_us_ >= 250000) {
            d.deposit_heading_ = in.heading_rad;
            d.deposit_stopped_us_ = 0;
            d.deposit_stage_ = Stage::Advancing;
        }
        return;
    }
    const bool new_frame = floor.sample_us > d.floor_last_us_;
    if (new_frame) {
        if (d.floor_last_us_ && floor.sample_us - d.floor_last_us_ > 150000)
            resetConfirmation();
        d.floor_last_us_ = floor.sample_us;
        if (floor.inside) {
            if (!d.floor_since_us_)
                d.floor_since_us_ = floor.sample_us;
            ++d.floor_frames_;
        } else {
            resetConfirmation();
            d.deposit_stage_ = Stage::Advancing;
        }
    }
    if (floor.inside) {
        // Stop on the first two-sided hit; confirm against fresh images while stationary.
        reason_ = "deposit_floor_confirm_stop";
        if (!stopped) {
            d.deposit_stopped_us_ = 0;
            return;
        }
        if (!d.deposit_stopped_us_)
            d.deposit_stopped_us_ = tick.now;
        if (new_frame && d.floor_frames_ >= 3 && d.floor_since_us_ &&
            floor.sample_us - d.floor_since_us_ >= t_.pushboard_floor_confirm_us &&
            tick.now - d.deposit_stopped_us_ >= 250000) {
            noteDropZone("deposit_floor_inside", {0, 0});
            enter(PushState::RAISE_RELEASE, tick.now, "deposit_floor_complete");
        }
        return;
    }
    if (!in.heading_valid || !std::isfinite(in.heading_rad)) {
        stop("deposit_floor_heading_missing");
        return;
    }
    reason_ = "deposit_floor_forward";
    const float error = wrapAngle(d.deposit_heading_ - in.heading_rad);
    drive(tick, std::min(t_.carry_speed, t_.pushboard_floor_speed),
          std::clamp(error, -.15f, .15f), true);
    tick.immediate_motion_stop = tick.motion.vx_mps <= 0;
}

void PushTask::updateCarryPlacement(TaskTick& tick) {
    // Keep the gripper closed while carrying the load into the correct half.
    if (t_.pixel_grasp_enabled && carryLogic())
        tick.immediate_motion_stop = true;
    commandFrame(0, tick.now);
    const int lowered = frameWait(tick.in, 0, tick.now);
    if (lowered <= 0) {
        if (lowered < 0)
            fail(t_.pixel_grasp_enabled ? "gripper_close_timeout" : "frame_lower_timeout");
        return;
    }
    if (t_.pixel_grasp_enabled && carryLogic() && t_.pushboard_floor_enabled) {
        updateFloorPlacement(tick);
        return;
    }
    if (t_.pixel_grasp_enabled && carryLogic()) {
        auto& deposit = transport_phase_;
        using Stage = TransportPhaseData::DepositStage;
        const auto stopDeposit = [&](const char* why) {
            tick.motion.vx_mps = tick.motion.wz_rps = 0;
            tick.immediate_motion_stop = true;
            failStop(tick, why);
        };
        tick.immediate_motion_stop = true;
        if (tick.now - deposit.deposit_started_us_ > t_.enter_budget_us) {
            stopDeposit("deposit_imu_timeout");
            return;
        }
        const auto& in = tick.in;
        const bool stopped = in.sent_velocity_valid && std::isfinite(in.sent_vx_mps) &&
            std::isfinite(in.sent_wz_rps) && std::abs(in.sent_vx_mps) < .001f &&
            std::abs(in.sent_wz_rps) < .001f;
        if (deposit.deposit_stage_ == Stage::Stopping) {
            reason_ = "deposit_stop_imu_calibrate";
            if (!stopped || !in.heading_valid || !std::isfinite(in.heading_rad)) {
                deposit.deposit_imu_.reset();
                deposit.deposit_stopped_us_ = 0;
                return;
            }
            if (!deposit.deposit_stopped_us_) {
                deposit.deposit_stopped_us_ = tick.now;
                deposit.deposit_heading_ = in.heading_rad;
            }
            if (deposit.deposit_imu_.calibrate(in.imu_translation_samples, tick.now,
                                              deposit.deposit_heading_)) {
                deposit.deposit_stage_ = Stage::Advancing;
                deposit.deposit_stopped_us_ = 0;
            }
            return;
        }
        if (deposit.deposit_stage_ == Stage::Advancing) {
            reason_ = "deposit_forward_015m_imu";
            if (!in.heading_valid || !std::isfinite(in.heading_rad)) {
                stopDeposit("deposit_heading_missing");
                return;
            }
            const float remaining = .15f - deposit.deposit_imu_.distance();
            const float speed = std::min(t_.carry_speed, std::max(.01f, 2.f * remaining));
            const float error = wrapAngle(deposit.deposit_heading_ - in.heading_rad);
            // Resolve the real drive permissions before consuming any displacement.
            drive(tick, speed, std::clamp(error, -.15f, .15f), true);
            if (tick.motion.vx_mps <= 0) {
                stopDeposit("deposit_forward_blocked");
                return;
            }
            if (!in.sent_velocity_valid || !std::isfinite(in.sent_vx_mps) ||
                !std::isfinite(in.sent_wz_rps) || in.sent_vx_mps < 0) {
                stopDeposit("deposit_forward_feedback_missing");
                return;
            }
            if (in.sent_vx_mps < .001f) {
                // A permitted request is not evidence that the sender moved the chassis.
                // Consume sample times while stationary so drift cannot reach the endpoint.
                if (!deposit.deposit_imu_.holdStationary(in.imu_translation_samples, tick.now))
                    stopDeposit("deposit_imu_gap");
                else
                    reason_ = "deposit_wait_forward_sent";
                return;
            }
        }
        if (!deposit.deposit_imu_.advance(in.imu_translation_samples, tick.now)) {
            stopDeposit("deposit_imu_gap");
            return;
        }
        if (deposit.deposit_imu_.distance() >= .15f)
            deposit.deposit_stage_ = Stage::Braking;
        if (deposit.deposit_stage_ == Stage::Braking) {
            tick.motion.vx_mps = tick.motion.wz_rps = 0;
            reason_ = "deposit_distance_reached_stop";
            if (!stopped) {
                deposit.deposit_stopped_us_ = 0;
                return;
            }
            if (!deposit.deposit_stopped_us_)
                deposit.deposit_stopped_us_ = tick.now;
            if (tick.now - deposit.deposit_stopped_us_ >= 250000) {
                noteDropZone("deposit_imu_015m", {0, deposit.deposit_imu_.distance()});
                enter(PushState::RAISE_RELEASE, tick.now, "deposit_imu_complete");
            }
            return;
        }
        return;
    }
    if (!t_.pixel_grasp_enabled && carryLogic() && !transport_phase_.assume_pushed_) {
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
        return;
    }
    EnterContext context;
    context.elapsed_us = tick.elapsed;
    context.zone_ok = zoneOk(tick.in);
    // lostTarget(tick) advances the parent's missed-frame counter: it must only run when
    // the zone is unusable, exactly as the inline branch did.
    if (!context.zone_ok)
        context.lost_target = lostTarget(tick);
    // A jaw must reach the deposit position before opening; losing the zone is not arrival.
    context.carry = carryLogic() && !t_.pixel_grasp_enabled;
    context.drop_locked = transport_phase_.drop_locked_;
    context.drop_centre_y = transport_phase_.drop_centre_zone_.y;
    if (context.zone_ok) {
        const auto& z = tick.in.zone_estimate;
        context.hold = z.bodyToZone({0, holdCenter()});
        context.half_x = halfX();
        context.zone_yaw = z.yaw_body_rad;
    }
    if (t_.pixel_grasp_enabled && carryLogic() && !context.zone_ok &&
        zone_.valid(tick.now)) {
        const auto z = zone_.predicted(tick.now);
        context.zone_ok = true;
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
            return;
        case EnterOutcome::AbortDrop:
            enter(PushState::ABORT_DROP, tick.now, strict.reason);
            return;
        case EnterOutcome::Continue:
            if (strict.drive)
                drive(tick, strict.vx, strict.wz);
            return;
    }
}

// Both navigation-plan and visual-anchor approaches use the same stationary alignment.
bool PushTask::alignCarryForPlacement(TaskTick& tick, const ZoneEstimate& zone,
                                     bool fresh, bool planned) {
    const auto hold = zone.bodyToZone({0, holdCenter()});
    if (!t_.pixel_grasp_enabled ||
        hold.y < -t_.assume_release_gap_m - t_.assume_align_zone_m)
        return false;
    tick.immediate_motion_stop = true;
    const auto& in = tick.in;
    // Stop incoming forward motion before requesting any in-place turn.
    if (!in.sent_velocity_valid || !std::isfinite(in.sent_vx_mps) ||
        std::abs(in.sent_vx_mps) >= .001f) {
        reason_ = "carry_near_stop";
        return true;
    }
    const auto carrot = zone.zoneToBody(
        {halfX(), std::min(hold.y + t_.assume_lookahead_m, -t_.assume_release_gap_m)});
    const float heading = std::atan2(-carrot.x, std::max(.02f, carrot.y - holdCenter()));
    reason_ = fresh ? "carry_near_align" : "carry_near_align_dr";
    if (std::abs(heading) <= t_.assume_align_tol_rad) {
        if (fresh) {
            if (planned)
                prepareCarryPlacement(tick);
            else
                beginCarryPlacement(tick.now, "carry_aligned_stop");
        }
    } else
        drive(tick, 0, t_.heading_gain * heading);
    return true;
}

// Distance is the signed perpendicular gap from the chassis rotation centre to
// the zone's entrance (zone y=0). Pixels never stand in for the 20 cm measurement.
void PushTask::updatePixelCarry(TaskTick& tick) {
    auto& d = transport_phase_;
    auto& q = d.jaw_sequence_;
    const auto& in = tick.in;
    using Stage = TransportPhaseData::JawSequence::Stage;
    constexpr float alignment_distance=.35f;
    constexpr float measurement_tolerance=.10f+1e-6f;
    q.active = true;
    carry_alignment_ = {};
    const bool colour_leg = q.stage == Stage::ColourAdvance || q.stage == Stage::ColourStop ||
        q.stage == Stage::ColourOverrun;
    d.approach_pitch_ = colour_leg ? t_.track_pitch_cdeg : t_.far_pitch_cdeg;
    const bool view = in.camera_pitch_stable && in.camera_pitch_cdeg != kCameraPitchInvalid &&
        std::abs(int(in.camera_pitch_cdeg) - int(d.approach_pitch_)) <= t_.pitch_tolerance_cdeg;
    const bool stopped = in.sent_velocity_valid && std::isfinite(in.sent_vx_mps) &&
        std::isfinite(in.sent_wz_rps) && std::abs(in.sent_vx_mps) < .001f &&
        std::abs(in.sent_wz_rps) < .001f;
    const auto stop = [&](const char* why) {
        tick.motion.vx_mps = tick.motion.wz_rps = 0;
        tick.immediate_motion_stop = true;
        reason_ = why;
    };
    const auto settled = [&] {
        if (!stopped) { q.stopped_us = 0; return false; }
        if (!q.stopped_us) q.stopped_us = tick.now;
        return tick.now - q.stopped_us >= 250000;
    };
    if (!view) {
        q.stopped_us = 0; q.visual_frames=0; q.visual_last_us=0;
        stop("carry_zone_view_wait");
        return;
    }
    if (colour_leg) {
        // Close-range keypoints no longer select, veto or restart the final
        // push. The already-aligned half's floor contour supplies the endpoint.
        const auto& floor = in.transport_floor;
        const bool fresh = floor.valid && floor.sample_us && floor.sample_us <= tick.now &&
            tick.now - floor.sample_us <= t_.frame_timeout_us;
        if (!fresh) {
            q.stopped_us=0; q.visual_frames=0; q.visual_last_us=0;
            stop("carry_ui120_region_wait");
            return;
        }
        const bool reached = floor.region_valid && floor.inside;
        if (q.stage == Stage::ColourAdvance) {
            if (reached) {
                q.stage=Stage::ColourOverrun;
                q.colour_overrun_us=tick.now;
                q.colour_overrun_wz=std::clamp(t_.heading_gain*wrapAngle(q.heading-in.heading_rad),-.15f,.15f);
                reason_="carry_ui120_region_overrun";
                drive(tick,.04f,q.colour_overrun_wz,true);
                return;
            }
            if (!in.heading_valid || !std::isfinite(in.heading_rad)) {
                stop("carry_ui120_heading_wait");
                return;
            }
            const float error=wrapAngle(q.heading-in.heading_rad);
            if (std::abs(error)>10.f*float(CV_PI)/180.f) {
                stop("carry_ui120_heading_realign");
                if (in.sent_velocity_valid && std::isfinite(in.sent_vx_mps) &&
                    std::abs(in.sent_vx_mps)<.001f)
                    drive(tick,0,std::clamp(t_.heading_gain*error,-.3f,.3f),true);
                return;
            }
            reason_="carry_ui120_region_advance";
            drive(tick,.04f,std::clamp(t_.heading_gain*error,-.15f,.15f),true);
            return;
        }
        if (q.stage == Stage::ColourOverrun) {
            if (tick.now-q.colour_overrun_us < 100000) {
                reason_="carry_ui120_region_overrun";
                drive(tick,.04f,q.colour_overrun_wz,true);
                return;
            }
            stop("carry_ui120_region_stop");
            q.stage=Stage::ColourStop;
            q.stopped_us=0; q.visual_frames=0; q.visual_last_us=0;
            return;
        }
        stop("carry_ui120_region_confirm");
        if (!reached) {
            q.stage=Stage::ColourAdvance;
            q.stopped_us=0; q.visual_frames=0; q.visual_last_us=0;
            return;
        }
        // Confirm only with three distinct captures made after the chassis
        // has been stopped for 250 ms. A stale/failed contour revokes the hit.
        if (!settled()) {
            q.visual_frames=0; q.visual_last_us=0;
            return;
        }
        if (floor.sample_us<=q.stopped_us+250000)
            return;
        if (floor.sample_us>q.visual_last_us) {
            q.visual_last_us=floor.sample_us;
            q.visual_frames=std::min(3,q.visual_frames+1);
        }
        if (q.visual_frames<3)
            return;
        d.baseline_ = in.zone_counts_valid && in.zone_inventory_complete
            ? (d.half_injured_ ? in.zone_injured_count : in.zone_supply_count) : -1;
        d.baseline_other_ = in.zone_counts_valid && in.zone_inventory_complete
            ? (d.half_injured_ ? in.zone_supply_count : in.zone_injured_count) : -1;
        d.delivery_geometry_id_=in.zone_estimate.geometry_id;
        d.delivery_verification_.resetInventoryEvidence();
        enter(PushState::RAISE_RELEASE,tick.now,"carry_ui120_region_stopped");
        return;
    }
    const auto& selected = d.half_injured_ ? in.injured_view : in.supply_view;
    const bool visual = selected.valid && std::isfinite(selected.target_x) &&
        std::abs(selected.target_x-halfX()) < .001f &&
        std::isfinite(selected.error_fraction) &&
        std::isfinite(selected.lateral_m) && std::isfinite(selected.edge_slope) && in.subzone_view_us &&
        in.subzone_view_us <= tick.now &&
        tick.now-in.subzone_view_us <= t_.frame_timeout_us;
    if (visual) {
        q.selected_half_seen_us = in.subzone_view_us;
        q.half_missing_frames = 0;
        if (in.heading_valid && std::isfinite(in.heading_rad))
            q.last_half_heading = wrapAngle(in.heading_rad-selected.error_fraction);
    }
    // The capture-time IMU stamp also identifies images with no usable half.
    const uint64_t half_frame_us = std::max(in.subzone_view_us,in.zone_observed_heading_us);
    if (half_frame_us && half_frame_us<=tick.now &&
        tick.now-half_frame_us<=t_.frame_timeout_us && half_frame_us>q.half_frame_us) {
        q.half_frame_us=half_frame_us;
        if (!visual) q.half_missing_frames=std::min(5,q.half_missing_frames+1);
    }
    // A fresh own-half view carries its own entrance measurement. Whole-zone
    // RANSAC failure or disagreement with the other half cannot veto it.
    const bool half_metric = visual && selected.entrance_valid &&
        std::isfinite(selected.entrance_origin_body_m.x) &&
        std::isfinite(selected.entrance_origin_body_m.y) &&
        std::isfinite(selected.entrance_yaw_rad);
    auto z = in.zone_estimate;
    if (half_metric) {
        z.origin_body_m = selected.entrance_origin_body_m;
        z.yaw_body_rad = selected.entrance_yaw_rad;
        z.observed_us = z.timestamp_us = in.subzone_view_us;
    }
    // A whole-zone fix may supply initial ranging while the selected half is
    // absent. Once visible, alignment uses only that half's entrance and image.
    const bool metric = half_metric || (!visual && in.zone_own && in.zone_identity_verified &&
        z.source != ZoneEstimate::Source::PREDICTED && z.trusted(tick.now));
    const bool own_sight = in.zone_sight_valid && !in.zone_sight_enemy &&
        std::isfinite(in.zone_sight_bearing) && in.zone_sight_sample_us &&
        in.zone_sight_sample_us <= tick.now &&
        tick.now-in.zone_sight_sample_us <= t_.frame_timeout_us;
    // Build a world-referenced entrance normal from three distinct, consistent
    // captures. Current IMU yaw then removes chassis rotation without following
    // each pose frame. Never use this angular memory to extrapolate distance.
    const bool heading_ok = in.heading_valid && std::isfinite(in.heading_rad);
    // Use one measured control interval of yaw as the stopping lead. Commanded
    // wz is not the chassis response and must not stand in for an IMU rate.
    const float heading_increment = heading_ok && q.align_heading_us &&
        tick.now > q.align_heading_us && tick.now-q.align_heading_us <= 200000
            ? wrapAngle(in.heading_rad-q.align_heading_rad) : 0;
    q.align_heading_us = heading_ok ? tick.now : 0;
    if (heading_ok) q.align_heading_rad = in.heading_rad;
    if (q.route_us && q.approach_route.valid(tick.now,t_.frame_timeout_us)) {
        if (heading_ok && tick.now>q.route_us && tick.now-q.route_us<=t_.frame_timeout_us &&
            in.sent_velocity_valid && std::isfinite(in.sent_vx_mps)) {
            const float dt=float(tick.now-q.route_us)*1e-6f;
            q.approach_route.advance(in.sent_vx_mps*dt/drRatio(tick.now),heading_increment);
        } else q.approach_route.reset();
    }
    q.route_us=tick.now;
    constexpr uint64_t turn_hold_us = 600000;
    const bool entry_view = !colour_leg && q.recovery == TransportPhaseData::JawSequence::Recovery::Idle;
    if (q.normal_valid && tick.now >= q.normal_seen_us &&
        tick.now-q.normal_seen_us > 1500000) {
        q.normal_valid=false; q.normal_frames=0;
    }
    if (metric && q.normal_geometry_id != z.geometry_id) {
        q.normal_valid = false; q.normal_frames = 0;
        q.normal_sample_us = q.normal_seen_us = 0;
        q.normal_geometry_id = z.geometry_id;
    }
    // An image-flat entrance cannot support a strongly tilted metric normal;
    // also reject a metric turn direction opposite to the observed edge.
    const bool agrees_with_image = !visual ||
        ((std::abs(selected.edge_slope) > .01f || std::abs(z.yaw_body_rad) <= .035f) &&
         (std::abs(selected.edge_slope) <= .01f || selected.edge_slope*z.yaw_body_rad <= 0));
    const bool normal_capture = half_metric && heading_ok && agrees_with_image &&
        in.zone_observed_heading_valid && std::isfinite(in.zone_observed_heading_rad) &&
        in.zone_observed_heading_us == z.observed_us;
    if (entry_view && !q.normal_rejected && normal_capture &&
        (q.normal_valid || q.stage == Stage::Approach || stopped) &&
        (q.stage == Stage::Approach || std::abs(z.yaw_body_rad) <= .75f) &&
        z.observed_us > q.normal_sample_us) {
        const float measured = wrapAngle(in.zone_observed_heading_rad + z.yaw_body_rad);
        if (!q.normal_frames || z.observed_us-q.normal_sample_us > t_.frame_timeout_us ||
            std::abs(wrapAngle(measured-q.normal_candidate)) > .06f) {
            q.normal_candidate = measured; q.normal_frames = 1;
        } else {
            q.normal_frames = std::min(q.normal_frames+1, 3);
            q.normal_candidate = wrapAngle(q.normal_candidate +
                wrapAngle(measured-q.normal_candidate)/q.normal_frames);
        }
        q.normal_sample_us = z.observed_us;
        if (q.normal_frames >= 3) {
            if (!q.normal_valid) {
                q.normal_heading = q.normal_candidate;
                q.normal_valid = true;
            } else if (std::abs(wrapAngle(q.normal_candidate-q.normal_heading)) <= .12f &&
                       std::isfinite(in.sent_wz_rps) &&
                       (q.stage == Stage::Approach || std::abs(in.sent_wz_rps) < .001f)) {
                // Once turning, retain the target. Refine it only on approach or
                // while stopped, with a bounded correction per new capture.
                q.normal_heading = wrapAngle(q.normal_heading +
                    std::clamp(.2f*wrapAngle(q.normal_candidate-q.normal_heading),-.01f,.01f));
            }
            if (std::abs(wrapAngle(q.normal_candidate-q.normal_heading)) <= .12f)
                q.normal_seen_us = z.observed_us;
        }
    }
    // A previously latched angle can bridge a stop, but must not make an
    // intermittent current projection look like a stable approach measurement.
    if (q.stage == Stage::Approach && !normal_capture)
        q.normal_frames = 0;
    const bool normal_available = entry_view && !q.normal_rejected && heading_ok && q.normal_valid &&
        q.normal_seen_us && tick.now >= q.normal_seen_us &&
        tick.now-q.normal_seen_us <= 1500000;
    const bool image_normal_available=q.image_normal_valid && heading_ok;
    const float normal_error = normal_available ? wrapAngle(q.normal_heading-in.heading_rad)
        : image_normal_available ? wrapAngle(q.heading-in.heading_rad) : z.yaw_body_rad;
    // A single own-colour false positive must not buy another 1.5 s stop.
    // Count distinct consecutive captures; duplicates cannot confirm or renew it.
    if (metric || own_sight) {
        const uint64_t sample = metric
            ? (own_sight ? std::max(z.observed_us,in.zone_sight_sample_us) : z.observed_us)
            : in.zone_sight_sample_us;
        if (sample > q.own_zone_sample_us) {
            if (!q.own_zone_sample_us || sample-q.own_zone_sample_us>t_.frame_timeout_us)
                q.own_zone_frames=0;
            q.own_zone_sample_us=sample;
            q.own_zone_frames=std::min(3,q.own_zone_frames+1);
            if (q.own_zone_frames>=3) q.own_zone_seen_us=sample;
        }
    } else q.own_zone_frames=0;
    const bool enemy_sight = !metric && !own_sight && in.zone_sight_valid &&
        in.zone_sight_enemy && std::isfinite(in.zone_sight_bearing) &&
        in.zone_sight_sample_us && in.zone_sight_sample_us<=tick.now &&
        tick.now-in.zone_sight_sample_us<=t_.frame_timeout_us;
    if (enemy_sight) {
        if (in.zone_sight_sample_us>q.enemy_zone_sample_us) {
            if (!q.enemy_zone_sample_us ||
                in.zone_sight_sample_us-q.enemy_zone_sample_us>t_.frame_timeout_us)
                q.enemy_zone_frames=0;
            q.enemy_zone_sample_us=in.zone_sight_sample_us;
            q.enemy_zone_frames=std::min(3,q.enemy_zone_frames+1);
        }
    } else q.enemy_zone_frames=0;
    const bool enemy_confirmed=q.enemy_zone_frames>=3;
    if (enemy_confirmed) {
        q.own_zone_seen_us=0;
    }
    const bool short_hold = q.own_zone_seen_us && tick.now >= q.own_zone_seen_us &&
        tick.now - q.own_zone_seen_us <= 1500000;
    using Recovery = TransportPhaseData::JawSequence::Recovery;
    // zone_sight_bearing already follows body turn signs: positive left, negative right.
    // Complete the offset leg, then restore its entry heading
    // before checking whether this trip's selected half is visible.
    const bool recoverable_stage = q.stage == Stage::EntryStop || q.stage == Stage::Align || q.stage == Stage::AlignStop;
    // A dropped pose immediately after the 20 cm stop is not a new manoeuvre.
    // Observe while physically stopped before committing to the offset recovery.
    if (recoverable_stage && !normal_available && !visual && stopped) {
        if (!q.entry_missing_us) q.entry_missing_us = tick.now;
    } else {
        q.entry_missing_us = 0;
    }
    if (recoverable_stage && q.entry_missing_us &&
        tick.now-q.entry_missing_us >= 1500000 && own_sight &&
        std::abs(in.zone_sight_bearing)>.035f &&
        q.recovery == Recovery::Idle) {
        if (!in.heading_valid || !std::isfinite(in.heading_rad)) {
            failStop(tick,"carry_pose_recovery_heading_missing"); return;
        }
        q.recovery=Recovery::Turn;
        q.recovery_original_heading=in.heading_rad;
        q.recovery_heading=wrapAngle(in.heading_rad + (in.zone_sight_bearing >= 0 ? float(CV_PI)*.5f : -float(CV_PI)*.25f));
        q.recovery_imu.reset(); q.stopped_us=0;
        q.visual_frames=0; q.visual_last_us=0;
    }
    if (q.recovery != Recovery::Idle) {
        stop("carry_pose_recovery_wait");
        if (!in.heading_valid || !std::isfinite(in.heading_rad)) {
            failStop(tick,"carry_pose_recovery_heading_missing"); return;
        }
        const float e=wrapAngle(q.recovery_heading-in.heading_rad);
        if (q.recovery==Recovery::Search) {
            // The unrelated half may be clipped or absent; only the cargo's
            // selected half is needed to resume entry alignment.
            const bool selected_half = visual && in.zone_own && in.zone_identity_verified;
            if (selected_half) {
                if (!stopped) { q.stopped_us=0; return; }
                if (in.subzone_view_us>q.visual_last_us) {
                    q.visual_last_us=in.subzone_view_us; ++q.visual_frames;
                }
                if (settled() && q.visual_frames>=3) {
                    q.recovery=Recovery::Idle; q.stage=Stage::Approach;
                    q.stopped_us=0; q.visual_frames=0; q.visual_last_us=0;
                    reason_="carry_pose_recovery_reacquired";
                }
            } else {
                q.stopped_us=0; q.visual_frames=0; q.visual_last_us=0;
                if (own_sight) {
                    // Re-evaluate the current box direction for each 15 cm leg.
                    q.recovery=Recovery::Turn;
                    q.recovery_original_heading=in.heading_rad;
                    q.recovery_heading=wrapAngle(in.heading_rad + (in.zone_sight_bearing >= 0 ? float(CV_PI)*.5f : -float(CV_PI)*.25f));
                    q.recovery_imu.reset();
                    reason_="carry_pose_recovery_retry_direction";
                } else if (in.sent_velocity_valid && std::isfinite(in.sent_vx_mps) &&
                    std::isfinite(in.sent_wz_rps) && std::abs(in.sent_vx_mps)<.001f) {
                    reason_="carry_pose_recovery_search"; scanSpin(tick);
                    tick.immediate_motion_stop=false;
                }
            }
            return;
        }
        if (q.recovery==Recovery::Return) {
            reason_="carry_pose_recovery_return";
            const float original_error=wrapAngle(q.recovery_original_heading-in.heading_rad);
            if (std::abs(original_error)>.035f) {
                q.stopped_us=0;
                if (in.sent_velocity_valid && std::isfinite(in.sent_vx_mps) &&
                    std::abs(in.sent_vx_mps)<.001f) {
                    drive(tick,0,t_.heading_gain*original_error,true);
                    tick.immediate_motion_stop=tick.motion.wz_rps==0;
                }
            } else if (settled()) {
                q.recovery=Recovery::Search; q.stopped_us=0;
                q.visual_frames=0; q.visual_last_us=0;
            }
            return;
        }
        // The zone may leave the image during the recovery turn. The latched
        // IMU target remains authoritative until the 15 cm leg is complete.
        if (q.recovery==Recovery::Turn) {
            reason_="carry_pose_recovery_turn";
            if (std::abs(e)>.035f) {
                if (in.sent_velocity_valid && std::isfinite(in.sent_vx_mps) &&
                    std::abs(in.sent_vx_mps)<.001f) {
                    drive(tick,0,t_.heading_gain*e,true);
                    tick.immediate_motion_stop=tick.motion.wz_rps==0;
                }
            } else if (settled()) {
                q.recovery=Recovery::Calibrate; q.recovery_imu.reset();
                q.recovery_heading=in.heading_rad; q.stopped_us=0;
            }
            return;
        }
        if (q.recovery==Recovery::Calibrate) {
            reason_="carry_pose_recovery_calibrate";
            if (!stopped) { q.recovery_imu.reset(); return; }
            if (q.recovery_imu.calibrate(in.imu_translation_samples,tick.now,q.recovery_heading))
                q.recovery=Recovery::Advance;
            return;
        }
        if (q.recovery==Recovery::Advance) {
            const float remaining=.15f-q.recovery_imu.distance();
            reason_="carry_pose_recovery_forward_015m";
            drive(tick,std::min(t_.carry_speed,std::max(.01f,2.f*remaining)),
                  std::clamp(t_.heading_gain*e,-.15f,.15f),true);
            tick.immediate_motion_stop=false;
            if (tick.motion.vx_mps<=0) {
                failStop(tick,"carry_pose_recovery_blocked"); return;
            }
            if (!in.sent_velocity_valid || !std::isfinite(in.sent_vx_mps) ||
                !std::isfinite(in.sent_wz_rps) || in.sent_vx_mps<0) {
                failStop(tick,"carry_pose_recovery_feedback_missing"); return;
            }
            const bool ok=in.sent_vx_mps<.001f
                ? q.recovery_imu.holdStationary(in.imu_translation_samples,tick.now)
                : q.recovery_imu.advance(in.imu_translation_samples,tick.now);
            if (!ok) { failStop(tick,"carry_pose_recovery_imu_gap"); return; }
            if (q.recovery_imu.distance()<.15f) return;
            q.recovery=Recovery::Brake; q.stopped_us=0;
        }
        stop("carry_pose_recovery_015m_stop");
        if (settled()) {
            q.recovery=Recovery::Return; q.stopped_us=0;
            q.visual_frames=0; q.visual_last_us=0;
        }
        return;
    }
    const auto findSelectedHalf = [&] {
        q.approach_resume_pending=true;
        q.approach_resume_frames=0; q.approach_resume_us=0;
        stop("carry_half_reacquire_brake");
        if (!heading_ok || !in.sent_velocity_valid ||
            !std::isfinite(in.sent_vx_mps) || std::abs(in.sent_vx_mps)>=.001f)
            return;
        if (!q.half_search_active) {
            q.half_search_active=true; q.half_search_leg=0;
            q.half_search_origin=in.heading_rad;
            const float direction=own_sight ? in.zone_sight_bearing
                : wrapAngle(q.last_half_heading-in.heading_rad);
            q.half_search_centre=wrapAngle(in.heading_rad+std::clamp(direction,-.45f,.45f));
        }
        if (q.half_search_leg>=3) {
            stop("carry_half_reacquire_wait");
            return;
        }
        // Only the first leg follows a fresh whole-zone bearing. Distance and
        // final entry still require this trip's selected half.
        if (q.half_search_leg==0 && own_sight)
            q.half_search_centre=wrapAngle(q.half_search_origin+
                std::clamp(wrapAngle(in.heading_rad+in.zone_sight_bearing-q.half_search_origin),-.45f,.45f));
        const float sweep=q.half_search_leg==1 ? .30f : q.half_search_leg==2 ? -.30f : 0.f;
        const float target=wrapAngle(q.half_search_centre+sweep);
        const float error=wrapAngle(target-in.heading_rad);
        if (std::abs(error)<=.035f+std::abs(heading_increment)) {
            stop("carry_half_reacquire_brake");
            if (settled()) { ++q.half_search_leg; q.stopped_us=0; }
            return;
        }
        q.stopped_us=0;
        reason_="carry_half_reacquire_turn";
        drive(tick,0,t_.heading_gain*error,true);
        tick.immediate_motion_stop=tick.motion.wz_rps==0;
    };
    if (q.stage==Stage::Approach && q.approach_resume_pending &&
        (half_metric || metric || q.half_search_active || q.half_search_braking)) {
        // A single recovered entrance must not kill the search's velocity ramp.
        // Confirm three distinct consecutive captures before committing to brake.
        if (half_frame_us && half_frame_us<=tick.now &&
            tick.now-half_frame_us<=t_.frame_timeout_us &&
            half_frame_us>q.half_search_sample_us) {
            if (q.half_search_sample_us &&
                half_frame_us-q.half_search_sample_us>t_.frame_timeout_us)
                q.half_search_frames=0;
            q.half_search_sample_us=half_frame_us;
            if (half_metric) {
                q.half_search_seen_us=in.subzone_view_us;
                q.half_search_frames=q.half_search_braking ? 0
                    : std::min(3,q.half_search_frames+1);
            } else {
                q.half_search_frames=q.half_search_braking
                    ? std::min(5,q.half_search_frames+1) : 0;
            }
        }
        // Once braking, bridge brief entrance dropouts without pulsing the turn.
        // Five missing captures or stale evidence return to the bounded search.
        if (q.half_search_braking &&
            (q.half_search_frames>=5 || !q.half_search_seen_us ||
             tick.now-q.half_search_seen_us>t_.frame_timeout_us)) {
            q.half_search_braking=false; q.half_search_frames=0;
            q.approach_resume_frames=0; q.approach_resume_us=0; q.stopped_us=0;
        }
        if (!q.half_search_braking && half_metric && q.half_search_frames>=3) {
            q.half_search_braking=true; q.half_search_frames=0;
            q.approach_resume_frames=0; q.approach_resume_us=0; q.stopped_us=0;
        }
        if (!q.half_search_braking) {
            findSelectedHalf();
            return;
        }
        // Search must finish braking before a fresh entrance can steer a new arc.
        stop("carry_half_reacquire_brake");
        q.approach_route.reset(); q.route_fix_us=0;
        if (!settled() || !half_metric) { q.approach_resume_frames=0; return; }
        if (in.subzone_view_us<=q.stopped_us+250000) return;
        if (in.subzone_view_us>q.approach_resume_us) {
            q.approach_resume_us=in.subzone_view_us;
            ++q.approach_resume_frames;
        }
        if (q.approach_resume_frames<3) return;
        q.approach_resume_pending=q.half_search_active=q.half_search_braking=false;
        q.half_search_frames=0; q.half_search_sample_us=q.half_search_seen_us=0;
        q.approach_resume_frames=0; q.stopped_us=0;
        q.range_valid=q.range_wait=false; q.range_jumps=q.range_frames=0;
        stop("carry_half_reacquired");
        return;
    }
    // Approach and forward reposition share the same short-lived, accepted
    // entrance trajectory. Missing/rejected captures cannot refresh its age.
    bool bridged_entrance=false;
    const auto bridgeEntrance = [&] {
        if (enemy_confirmed || !heading_ok || q.approach_resume_pending ||
            q.half_missing_frames>=5 ||
            !q.approach_route.valid(tick.now,t_.frame_timeout_us)) return false;
        const auto predicted=q.approach_route.predicted(tick.now);
        if (-predicted.bodyToZone({0,0}).y<=alignment_distance) return false;
        // A fresh close entrance can revoke travel; prediction cannot overrule
        // evidence that the parking margin has already been consumed.
        if (half_metric && -z.bodyToZone({0,0}).y<=alignment_distance+.05f) return false;
        z=predicted;
        bridged_entrance=true;
        return true;
    };
    if (q.stage==Stage::Approach && q.reposition && !half_metric && !bridgeEntrance()) {
        q.range_valid=false; q.range_wait=false;
        q.range_jumps=0; q.range_frames=0;
        q.approach_route.reset(); q.route_fix_us=0;
        findSelectedHalf();
        return;
    }
    // A changed entrance position must persist for three distinct captures before braking.
    // Check both axes: the 19:42 trace jumped laterally while forward range stayed stable.
    // The first two outliers cannot replace the last accepted entrance route.
    if (q.stage==Stage::Approach && half_metric) {
        const auto position=z.bodyToZone({0,0});
        const float gap=-position.y, offset=halfX()-position.x;
        if (in.subzone_view_us!=q.range_sample_us) {
            q.range_sample_us=in.subzone_view_us;
            if (q.range_wait) {
                q.range_gaps[q.range_frames]=gap;
                q.range_offsets[q.range_frames++]=offset;
                if (q.range_frames==3) {
                    const auto bounds=std::minmax_element(q.range_gaps.begin(),q.range_gaps.end());
                    const auto lateral=std::minmax_element(q.range_offsets.begin(),q.range_offsets.end());
                    if (*bounds.second-*bounds.first<=measurement_tolerance &&
                        *lateral.second-*lateral.first<=measurement_tolerance) {
                        q.range_view=selected; q.range_valid=true;
                        q.range_wait=false; q.range_jumps=0;
                    }
                    q.range_frames=0;
                }
            } else {
                auto previous=z;
                previous.origin_body_m=q.range_view.entrance_origin_body_m;
                previous.yaw_body_rad=q.range_view.entrance_yaw_rad;
                const auto last_position=previous.bodyToZone({0,0});
                const float last_gap=-last_position.y, last_offset=halfX()-last_position.x;
                if (q.range_valid && (std::abs(gap-last_gap)>measurement_tolerance ||
                                      std::abs(offset-last_offset)>measurement_tolerance)) {
                    if (++q.range_jumps>=3) {
                        q.range_wait=true; q.range_frames=0;
                    }
                } else {
                    q.range_view=selected; q.range_valid=true; q.range_jumps=0;
                }
            }
        }
        if (q.range_wait && !bridgeEntrance()) {
            stop("carry_entry_measurement_unstable");
            return;
        }
        if (q.range_jumps && !bridged_entrance) {
            z.origin_body_m=q.range_view.entrance_origin_body_m;
            z.yaw_body_rad=q.range_view.entrance_yaw_rad;
        }
    } else if (q.stage==Stage::Approach) {
        q.range_jumps=0;
        if (q.range_wait) {
            q.range_frames=0;
            if (!bridgeEntrance()) {
                stop("carry_entry_measurement_unstable");
                return;
            }
        }
    }
    if (q.stage==Stage::Approach && !half_metric && !bridged_entrance)
        bridgeEntrance();
    const auto robot = z.bodyToZone({0,0});
    // Image centring supplies final acceptance. The compensated image lateral
    // estimate below is diagnostic; the joint approach uses one entrance pose.
    // Image lateral displacement belongs to capture time; steering belongs
    // to now. Mixing these yaws turns inference latency into a false side offset.
    const float observed_normal_error = (normal_available || image_normal_available) &&
        in.zone_observed_heading_valid && in.zone_observed_heading_us == in.subzone_view_us
            ? wrapAngle((normal_available ? q.normal_heading : q.heading)-in.zone_observed_heading_rad)
            : z.yaw_body_rad;
    const bool normal_view=metric && std::abs(observed_normal_error)<=.75f && visual;
    // Remove the normal-heading contribution before using image error to
    // steer lateral position. Otherwise a small yaw looks like chassis offset.
    const float lateral=normal_view
        ? selected.lateral_m + std::tan(observed_normal_error)*(-robot.y+t_.deposit_y_m)
        : halfX()-robot.x;
    // The 17:13 field entry remains deliverable at 6.17% centre error and
    // 5.75% edge slope. Confirm that range instead of forcing another turn.
    constexpr float centre_tolerance = .07f;
    constexpr float entry_edge_tolerance=.07f;
    if (visual && in.subzone_view_us != d.jaw_sequence_.diagnostic_us) {
        d.jaw_sequence_.diagnostic_us=in.subzone_view_us;
        std::cout << "[CARRY_SUBZONE] sample_us=" << in.subzone_view_us
                  << " half=" << (d.half_injured_ ? "injured" : "supply")
                  << " stage=" << int(q.stage)
                  << " entry_source=" << (half_metric ? "selected_half" : metric ? "whole_zone" : "none")
                  << " lateral_m=" << lateral
                  << " pose_lateral_m=" << halfX()-robot.x
                  << " parking_remaining_m=" << -robot.y-alignment_distance
                  << " yaw_rad=" << z.yaw_body_rad
                  << " edge_slope=" << selected.edge_slope
                  << " image_error_fraction=" << selected.error_fraction
                  << " centre_inside=" << selected.inside
                  << " acceptance_fraction=" << centre_tolerance
                  << " frames=" << q.visual_frames
                  << " normal_valid=" << normal_available
                  << " normal_frames=" << q.normal_frames
                  << " normal_error_rad=" << normal_error
                  << " normal_rejected=" << q.normal_rejected
                  << " stopped_ms=" << (q.stopped_us ? (tick.now-q.stopped_us)/1000 : 0) << "\n";
    }
    const auto resetVisual = [&] { q.visual_frames=0; q.visual_last_us=0; };
    const auto turnMovesAway = [&](float wz) {
        // Image x grows right; positive body yaw turns left. A turn with
        // the same sign as an off-centre image error increases that error.
        return visual && std::abs(selected.error_fraction)>centre_tolerance &&
            selected.error_fraction*wz>0;
    };
    const auto holdAlignment = [&] {
        // Three stationary captures have proved that the remaining offset
        // cannot be corrected here. Repeating EntryStop changes no geometry.
        stop("carry_entry_reposition_required");
        q.image_turn_heading=0; q.image_turn_wz=0; q.image_turn_centre=false;
        failStop(tick,"carry_entry_reposition_required");
    };
    const auto approachCurveFeasible = [&](float gap, float offset, float yaw) {
        const float span=gap-alignment_distance;
        if (span<=.0001f || std::abs(yaw)>.75f) return false;
        const float slope=std::tan(yaw), ratio=offset/span;
        const float a=3.f*(slope-2.f*ratio), b=2.f*(3.f*ratio-2.f*slope);
        float max_slope=std::abs(slope);
        if (std::abs(a)>.0001f) {
            const float t=-b/(2.f*a);
            if (t>0 && t<1) max_slope=std::max(max_slope,std::abs((a*t+b)*t+slope));
        }
        return max_slope<=std::tan(.75f);
    };
    if (q.stage == Stage::Approach) {
        // Review a short, large-offset curve before it consumes the approach room.
        // Reuse forward reposition; never add an automatic reverse manoeuvre.
        const float gap=-robot.y, offset=halfX()-robot.x;
        if ((!q.reposition || gap-alignment_distance<=.05f) &&
            half_metric && !bridged_entrance && gap>alignment_distance &&
            std::abs(offset)>.03f && !approachCurveFeasible(gap,offset,normal_error)) {
            stop("carry_entry_path_review");
            if (!settled()) { q.approach_review_frames=0; return; }
            if (in.subzone_view_us<=q.stopped_us+250000) return;
            if (in.subzone_view_us>q.approach_review_us) {
                q.approach_review_gaps[q.approach_review_frames]=gap;
                q.approach_review_offsets[q.approach_review_frames++]=offset;
                q.approach_review_us=in.subzone_view_us;
                if (q.approach_review_frames==3) {
                    const auto gaps=std::minmax_element(q.approach_review_gaps.begin(),q.approach_review_gaps.end());
                    const auto offsets=std::minmax_element(q.approach_review_offsets.begin(),q.approach_review_offsets.end());
                    if (*gaps.second-*gaps.first>measurement_tolerance ||
                        *offsets.second-*offsets.first>measurement_tolerance)
                        q.approach_review_frames=0;
                }
            }
            if (q.approach_review_frames<3) return;
            q.approach_review_frames=0; q.stopped_us=0;
            if (gap-alignment_distance<=.05f) {
                failStop(tick,"carry_entry_forward_space_insufficient");
                return;
            }
            q.reposition=true; q.approach_route.reset(); q.route_fix_us=0;
            stop("carry_entry_forward_reposition_prepare");
            return;
        }
        q.approach_review_frames=0;
        // Correct early while clearance remains. The old full-speed 0.2 rad/s
        // arc reached the stop line off-centre and repeated the retreat.
        // Retain proportional steering near centre and the global drive limits.
        const auto approach = [&](float speed, float bearing) {
            const float correction = std::clamp((std::abs(bearing)-.08f)/.42f,0.f,1.f);
            if(in.directional_clearance_valid && !in.arc_safe && std::abs(bearing)>.001f) {
                stop("carry_approach_arc_blocked");
                return;
            }
            drive(tick,speed*(1.f-.65f*correction),
                  std::clamp(t_.heading_gain*bearing,-.4f,.4f),true);
        };
        const auto followRoute = [&](const ZoneEstimate& entrance, float heading_error,
                                     const char* why) {
            const auto position=entrance.bodyToZone({0,0});
            const float gap = -position.y;
            const float remaining = std::max(.05f,gap-alignment_distance);
            const float offset = halfX()-position.x;
            const float bearing = wrapAngle(heading_error-std::atan2(offset,remaining));
            reason_=why;
            if(std::abs(bearing)>.75f) {
                drive(tick,0,t_.heading_gain*bearing,true);
            } else {
                const float slope=std::tan(std::clamp(heading_error,-.75f,.75f));
                const float curvature=(4.f*slope/remaining-6.f*offset/(remaining*remaining)) /
                    std::pow(1.f+slope*slope,1.5f);
                const float cap=std::min(.4f,t_.max_wz);
                const float speed=std::min(
                    std::min(t_.carry_speed,std::max(.01f,.8f*(gap-alignment_distance))) /
                        (1.f+.1f*std::abs(curvature)),
                    cap/std::max(.001f,std::abs(curvature)));
                // If an arc is blocked, drive() can fall back to straight motion.
                // That fallback cannot follow this parking trajectory: stop instead.
                if(in.directional_clearance_valid && !in.arc_safe && std::abs(curvature)>.001f)
                    stop("carry_approach_arc_blocked");
                else
                    drive(tick,speed,speed*curvature,true);
            }
        };
        // Latch accepted captures before either steering branch can return.
        // Outliers, repeated images and predictions never renew this trajectory.
        if (half_metric && !bridged_entrance && !q.range_jumps && !q.range_wait &&
            (q.reposition || normal_available || image_normal_available) &&
            in.subzone_view_us!=q.route_fix_us && heading_ok &&
            in.zone_observed_heading_valid && in.zone_observed_heading_us==in.subzone_view_us) {
            const float latency_yaw=wrapAngle(in.heading_rad-in.zone_observed_heading_rad);
            q.approach_route.reset();
            q.approach_route.update(z,in.subzone_view_us);
            q.route_heading=wrapAngle(in.heading_rad+normal_error);
            q.approach_route.advance(0,latency_yaw);
            q.route_fix_us=in.subzone_view_us;
        }
        if (bridged_entrance && !q.reposition) {
            followRoute(z,wrapAngle(q.route_heading-in.heading_rad),"carry_entry_route_hold");
            return;
        }
        // Begin alignment 15 cm before the required 20 cm entry line.
        // Reaching 20 cm is never the trigger to begin normal alignment.
        // Only this trip's selected entrance can latch final alignment.
        // An unrelated whole-zone fix exposed during a half dropout can report
        // a much shorter gap and otherwise strand an offset chassis in Align.
        if (half_metric && -robot.y <= alignment_distance+.0001f) {
            stop("carry_pre_entry_stop");
            q.stage = Stage::EntryStop;
            q.stopped_us = 0;
            return;
        }
        if (q.reposition) {
            // Use remaining forward room for a lateral approach before final yaw.
            // A short cubic may be impossible even though a turn and forward leg fit.
            const float span=-robot.y-alignment_distance;
            if (span<=.0001f) { stop("carry_pre_entry_stop"); return; }
            const float offset=halfX()-robot.x;
            const float bearing=wrapAngle(z.yaw_body_rad-
                std::atan2(offset,std::max(.025f,.5f*span)));
            if (std::abs(offset)<=.03f) {
                q.reposition=false;
            } else {
                reason_="carry_entry_forward_reposition";
                if (std::abs(bearing)>.15f) {
                    if (!in.sent_velocity_valid || !std::isfinite(in.sent_vx_mps) ||
                        std::abs(in.sent_vx_mps)>=.001f)
                        stop("carry_entry_reposition_brake");
                    else drive(tick,0,t_.heading_gain*bearing,true);
                } else
                    approach(std::min(.05f,std::min(t_.carry_speed,.8f*span)),bearing);
                return;
            }
        }
        // The three-frame gate acquires a world-referenced normal; it must
        // not switch steering formulas on each dropped/noisy capture afterward.
        // Its existing expiry and fresh image disagreement still revoke use.
        const bool normal_agrees_with_view = !visual ||
            ((std::abs(selected.edge_slope)>.01f || std::abs(observed_normal_error)<=.035f) &&
             (std::abs(selected.edge_slope)<=.01f ||
              selected.edge_slope*observed_normal_error<=0));
        const bool approach_normal = image_normal_available ||
            (normal_available && normal_agrees_with_view);
        // A missing half must not replace the parking arc with image-only centring.
        if (!visual && q.selected_half_seen_us) {
            if (q.half_missing_frames>=5) {
                findSelectedHalf();
                return;
            }
            if (!enemy_confirmed && q.approach_route.valid(tick.now,t_.frame_timeout_us)) {
                const auto predicted=q.approach_route.predicted(tick.now);
                // Prediction may bridge the approach, never authorize crossing its stop line.
                if (-predicted.bodyToZone({0,0}).y>alignment_distance) {
                    followRoute(predicted,wrapAngle(q.route_heading-in.heading_rad),"carry_entry_route_hold");
                    return;
                }
            }
            stop("carry_selected_half_wait");
            return;
        }
        if (visual && (!metric || !approach_normal)) {
            // Three consistent capture-time normals are required before adding
            // metric lateral compensation. During projection dropouts or jumps,
            // keep steering from the same half's image, never the whole-zone box.
            reason_ = "carry_approach_selected_half";
            const float bearing = metric
                ? -std::atan2(selected.lateral_m,std::max(.02f,.5f*(-robot.y-alignment_distance)))
                : -selected.error_fraction;
            const float speed = metric
                ? std::min(t_.carry_speed,std::max(.01f,.8f*(-robot.y-alignment_distance)))
                : t_.carry_speed;
            if (metric) approach(speed,bearing);
            else {
                // Without an entrance distance, translation cannot guarantee
                // alignment outside the 20 cm line. Keep only the visual turn.
                stop("carry_selected_half_range_wait");
                if (std::abs(selected.error_fraction)>centre_tolerance)
                    drive(tick,0,-t_.heading_gain*selected.error_fraction,true);
                tick.immediate_motion_stop=tick.motion.wz_rps==0;
            }
            return;
        }
        if (!metric) {
            // Find the fresh own-zone direction even before ranging is
            // available. Translation still requires an entrance measurement.
            const bool sight = in.zone_sight_valid && !in.zone_sight_enemy &&
                std::isfinite(in.zone_sight_bearing) && in.zone_sight_sample_us &&
                in.zone_sight_sample_us <= tick.now &&
                tick.now - in.zone_sight_sample_us <= t_.frame_timeout_us;
            if (sight) {
                reason_ = "carry_drive_to_sighted_zone";
                // A box supplies direction, not entry distance. Turn to find
                // the selected half; do not drive across an unmeasured line.
                stop("carry_zone_metric_wait");
                if (std::abs(in.zone_sight_bearing)>centre_tolerance) {
                    reason_="carry_drive_to_sighted_zone";
                    drive(tick,0,t_.heading_gain*in.zone_sight_bearing,true);
                    tick.immediate_motion_stop=tick.motion.wz_rps==0;
                    q.approach_resume_pending |= tick.motion.wz_rps!=0;
                }
            } else if (short_hold) {
                stop("carry_zone_short_hold");
            } else {
                // Brake any translation before scanning; angular feedback is
                // expected during a scan and must not stop it on the next tick.
                const bool translation_stopped = in.sent_velocity_valid &&
                    std::isfinite(in.sent_vx_mps) && std::isfinite(in.sent_wz_rps) &&
                    std::abs(in.sent_vx_mps) < .001f;
                if (translation_stopped) {
                    reason_ = "searching_own_zone";
                    scanSpin(tick);
                    q.approach_resume_pending |= tick.motion.wz_rps!=0;
                } else {
                    stop("carry_zone_metric_wait");
                }
            }
            return;
        }
        // Preserve the near-zone stop even before this half is first seen.
        // Whole-zone ranging can stop translation, but cannot authorize Align.
        if (!half_metric && metric && -robot.y<=alignment_distance+.0001f) {
            stop("carry_selected_half_range_wait");
            return;
        }
        followRoute(z,normal_error,"carry_entry_020m_approach");
        return;
    }
    if (!in.heading_valid || !std::isfinite(in.heading_rad)) {
        stop("carry_heading_missing");
        failStop(tick, "carry_heading_missing");
        return;
    }
    // Missing entry landmarks without a usable direction remain stationary.
    if ((q.stage == Stage::EntryStop || q.stage == Stage::Align ||
         q.stage == Stage::AlignStop) && !visual && !normal_available &&
        !(q.stage==Stage::Align && q.image_turn_wz!=0 &&
          q.image_turn_sample_us && tick.now>=q.image_turn_sample_us &&
          tick.now-q.image_turn_sample_us<=turn_hold_us)) {
        stop("carry_entry_landmark_wait");
        q.stopped_us=0; resetVisual();
        return;
    }
    if (q.stage == Stage::EntryStop) {
        stop("carry_entry_stop_image_wait");
        const bool stable_stop=settled();
        if (!visual && !normal_available) { resetVisual(); return; }
        if (stable_stop && (normal_available || in.subzone_view_us > q.stopped_us)) {
            // Evaluate alignment conflicts only after the stationary three-
            // capture gate below; one noisy frame cannot fault the transport.
            q.stage=Stage::Align;
            resetVisual();
        }
        return;
    }
    if (q.stage == Stage::Align || q.stage == Stage::AlignStop) {
        stop(reason_=="carry_entry_measurement_unstable"
            ? "carry_entry_measurement_unstable" : "carry_entry_subzone_confirm");
        const auto brakeForObservation = [&] {
            q.stage=Stage::AlignStop;
            q.stopped_us=0;
            q.image_turn_heading=0; q.image_turn_wz=0; q.image_turn_centre=false;
            resetVisual();
        };
        if (q.stage==Stage::Align) {
            // Continue a measured image turn across a clipped half. Fresh
            // same-direction edges extend it without another stop every 0.1 rad.
            // Only the final brake enters the stationary three-frame gate.
            const bool turn_fresh=q.image_turn_sample_us &&
                tick.now>=q.image_turn_sample_us &&
                tick.now-q.image_turn_sample_us<=turn_hold_us;
            if (!q.image_turn_centre && normal_available && !visual && std::abs(normal_error)>.02f &&
                (q.image_turn_wz==0 || normal_error*q.image_turn_wz>0)) {
                q.stopped_us=0; resetVisual();
                reason_="carry_entry_pose_imu_align";
                if (in.sent_velocity_valid && std::isfinite(in.sent_vx_mps) &&
                    std::abs(in.sent_vx_mps)<.001f)
                    drive(tick,0,t_.heading_gain*normal_error,true);
                return;
            }
            if (q.image_turn_wz!=0 && (visual || turn_fresh)) {
                const float edge_limit=entry_edge_tolerance;
                const bool new_turn_view=visual && in.subzone_view_us>q.image_turn_sample_us;
                if (new_turn_view) {
                    q.image_turn_sample_us=in.subzone_view_us;
                    const bool same_direction=selected.edge_slope*q.image_turn_wz<0;
                    if (!q.image_turn_centre && same_direction && std::abs(selected.edge_slope)>edge_limit &&
                        !turnMovesAway(q.image_turn_wz) && in.zone_observed_heading_valid &&
                        std::isfinite(in.zone_observed_heading_rad) &&
                        in.zone_observed_heading_us==in.subzone_view_us) {
                        // The image belongs to capture time; current IMU yaw
                        // already includes rotation during inference latency.
                        q.image_turn_heading=wrapAngle(in.zone_observed_heading_rad+
                            std::clamp(-std::atan(2.f*selected.edge_slope),-.10f,.10f));
                    }
                }
                const float remaining=wrapAngle(q.image_turn_heading-in.heading_rad);
                const bool correction_finished=new_turn_view && (q.image_turn_centre
                    ? (std::abs(selected.error_fraction)<centre_tolerance ||
                       selected.error_fraction*q.image_turn_wz>=0 ||
                       std::abs(selected.edge_slope)>edge_limit)
                    : (std::abs(selected.edge_slope)<=edge_limit ||
                       selected.edge_slope*q.image_turn_wz>=0 || turnMovesAway(q.image_turn_wz)));
                const float brake_lead = heading_increment*q.image_turn_wz>0
                    ? std::min(.10f,std::abs(heading_increment)) : 0;
                if (!correction_finished && remaining*q.image_turn_wz>0 &&
                    std::abs(remaining)>.005f+brake_lead) {
                    q.stopped_us=0;
                    reason_=q.image_turn_centre ? "carry_entry_centre_align" : "carry_entry_image_align";
                    const float cap=t_.transportTurnCap();
                    const float speed=std::min(std::abs(q.image_turn_wz),
                        std::max(std::min(t_.transportTurnFloor(),cap),
                                 t_.heading_gain*std::abs(remaining)));
                    if (in.sent_velocity_valid && std::isfinite(in.sent_vx_mps) &&
                        std::abs(in.sent_vx_mps)<.001f)
                        drive(tick,0,std::copysign(speed,q.image_turn_wz),true);
                    return;
                }
                std::cout << "[CARRY_ENTRY_BRAKE] remaining_rad=" << remaining
                          << " imu_step_rad=" << heading_increment
                          << " lead_rad=" << brake_lead
                          << " edge_finished=" << correction_finished
                          << " goal=" << (q.image_turn_centre ? "centre" : "edge") << "\n";
            }
            brakeForObservation();
            reason_="carry_entry_align_brake";
            return;
        }

        // A new image cannot interrupt braking. All decisions use observations
        // captured after the full 250 ms stationary window.
        if (!settled()) { resetVisual(); return; }
        if (normal_available && !visual && std::abs(normal_error)>.02f) {
            q.stage=Stage::Align; resetVisual(); return;
        }
        if (!visual || in.subzone_view_us<=q.stopped_us+250000) {
            resetVisual();
            reason_="carry_entry_pose_imu_confirm_wait";
            return;
        }
        if (in.subzone_view_us>q.visual_last_us && q.visual_frames<3) {
            q.align_views[q.visual_frames++]=selected;
            q.visual_last_us=in.subzone_view_us;
        }
        if (q.visual_frames<3) return;
        const float align_edge_tolerance=entry_edge_tolerance;
        std::array<float,3> edges;
        bool centred=true, all_displaced=true;
        for (int i=0;i<3;++i) {
            edges[i]=q.align_views[i].edge_slope;
            const bool accepted=q.align_views[i].inside &&
                std::abs(q.align_views[i].error_fraction)<centre_tolerance;
            centred &= accepted;
            all_displaced &= !accepted;
        }
        // A stop from a previous/noisy range fix may leave room for the
        // existing joint approach. Replan only from three stationary selected-
        // half measurements: keep 5 cm beyond the 35 cm parking pose, require
        // a real side offset, and accept at most 10 cm range/offset variation.
        bool can_reapproach=all_displaced && half_metric;
        bool curve_feasible=true, ranges_valid=true;
        float min_gap=std::numeric_limits<float>::infinity(), max_gap=-min_gap;
        float min_offset=min_gap, max_offset=-min_gap;
        for (const auto& v : q.align_views) {
            const bool finite=v.entrance_valid &&
                std::isfinite(v.entrance_origin_body_m.x) &&
                std::isfinite(v.entrance_origin_body_m.y) &&
                std::isfinite(v.entrance_yaw_rad);
            if (!finite) { can_reapproach=false; ranges_valid=false; continue; }
            auto entrance=z;
            entrance.origin_body_m=v.entrance_origin_body_m;
            entrance.yaw_body_rad=v.entrance_yaw_rad;
            const auto position=entrance.bodyToZone({0,0});
            const float gap=-position.y, offset=halfX()-position.x;
            const float span=gap-alignment_distance;
            can_reapproach &= span>.05f && std::abs(offset)>.03f;
            curve_feasible &= approachCurveFeasible(gap,offset,v.entrance_yaw_rad);
            min_gap=std::min(min_gap,gap); max_gap=std::max(max_gap,gap);
            min_offset=std::min(min_offset,offset); max_offset=std::max(max_offset,offset);
        }
        if (ranges_valid && (max_gap-min_gap>measurement_tolerance || max_offset-min_offset>measurement_tolerance)) {
            resetVisual();
            reason_="carry_entry_measurement_unstable";
            return;
        }
        if (can_reapproach) {
            std::cout << "[CARRY_ENTRY_REPLAN] gap_min_m=" << min_gap
                      << " gap_max_m=" << max_gap
                      << " offset_min_m=" << min_offset
                      << " offset_max_m=" << max_offset
                      << " forward_reposition=" << !curve_feasible << "\n";
            stop("carry_entry_resume_approach");
            q.reposition=!curve_feasible;
            q.range_valid=false; q.range_jumps=0; q.range_frames=0;
            q.range_wait=false;
            q.stage=Stage::Approach; q.stopped_us=0;
            q.image_turn_heading=0; q.image_turn_wz=0; q.image_turn_sample_us=0;
            q.image_turn_centre=false;
            q.image_normal_valid=false;
            resetVisual();
            return;
        }
        std::sort(edges.begin(),edges.end());
        const float edge=edges[1]; // ignore one quantized/jittering corner observation
        if (std::abs(edge)>align_edge_tolerance) {
            if (normal_available &&
                (std::abs(normal_error)<=.02f || normal_error*edge>=0)) {
                // Persist until the next CARRY trip: fresh frames from the same
                // biased geometry may not resurrect a rejected heading.
                q.normal_valid=false; q.normal_rejected=true; q.normal_frames=0;
                reason_="carry_entry_pose_image_conflict";
                resetVisual();
                return;
            }
            // Recheck after the stationary three-frame window as well: the
            // half can become displaced during an earlier bounded correction.
            if (!centred && turnMovesAway(-edge)) {
                if (all_displaced) holdAlignment();
                else resetVisual(); // mixed captures need another stationary window
                return;
            }
            const float cap=t_.transportTurnCap();
            const float speed=std::min(cap,std::max(t_.transportTurnFloor(),2.f*std::abs(edge)));
            q.image_turn_centre=false;
            q.image_turn_wz=std::copysign(speed,-edge);
            q.image_turn_sample_us=in.subzone_view_us;
            // Begin with a bounded IMU endpoint. Fresh same-direction images
            // can extend the turn; its final stop still requires 250 ms and
            // three fresh captures before authorizing entry.
            q.image_turn_heading=wrapAngle(in.heading_rad+
                std::clamp(-std::atan(2.f*edge),-.10f,.10f));
            q.stage=Stage::Align;
            resetVisual();
            reason_="carry_entry_image_turn_ready";
            return;
        }
        if (!centred) {
            // A tolerated edge does not require perfect entrance-normal yaw.
            // Turn toward the selected half before declaring it unreachable.
            if (!all_displaced) { resetVisual(); return; }
            std::array<float,3> errors;
            for (int i=0;i<3;++i) errors[i]=q.align_views[i].error_fraction;
            std::sort(errors.begin(),errors.end());
            const float error=errors[1];
            if (std::abs(error)<centre_tolerance) { holdAlignment(); return; }
            if (std::abs(selected.error_fraction)<centre_tolerance ||
                error*selected.error_fraction<=0) { resetVisual(); return; }
            if (!in.zone_observed_heading_valid ||
                !std::isfinite(in.zone_observed_heading_rad) ||
                in.zone_observed_heading_us!=in.subzone_view_us) {
                resetVisual(); reason_="carry_entry_centre_heading_wait"; return;
            }
            q.image_turn_centre=true;
            const float cap=t_.transportTurnCap();
            const float speed=std::min(cap,std::max(t_.transportTurnFloor(),t_.heading_gain*std::abs(error)));
            q.image_turn_wz=std::copysign(speed,-error);
            q.image_turn_sample_us=in.subzone_view_us;
            q.image_turn_heading=wrapAngle(in.zone_observed_heading_rad-
                std::clamp(error,-.10f,.10f));
            q.stage=Stage::Align;
            resetVisual();
            reason_="carry_entry_centre_turn_ready";
            return;
        }
        // Fresh image acceptance is authoritative. A disagreeing metric normal
        // cannot veto the 7% edge/centre gate or restart precision alignment.
        if (normal_available && std::abs(normal_error)>.07f) {
            q.normal_valid=false; q.normal_rejected=true; q.normal_frames=0;
        }
        q.heading=in.heading_rad; q.image_normal_valid=true;
        // One stationary confirmation authorizes the near-view push directly.
        q.stage=Stage::ColourAdvance;
        d.approach_pitch_=t_.track_pitch_cdeg;
        q.stopped_us=0; resetVisual();
        reason_="carry_pre_entry_aligned";
        return;
    }

}

void PushTask::updateTransportPhase(TaskTick& tick) {
    switch (transport_phase_.action) {
        case TransportAction::Carry: {
            if (transport_phase_.placing_) {
                updateCarryPlacement(tick);
                break;
            }
            if (tick.now - transport_phase_.carry_us_ > t_.carry_budget_us) {
                if (reason_=="carry_entry_measurement_unstable")
                    failStop(tick,"carry_entry_measurement_unstable");
                else
                    enter(PushState::ABORT_DROP, tick.now, "carry_timeout");
                break;
            }
            if (t_.pixel_grasp_enabled && carryLogic()) {
                updatePixelCarry(tick);
                break;
            }
            if (!carryLogic() && actionProgress().step == 2) {
                const int held = holdCheck(tick);
                if (held < 0)
                    enter(PushState::LOST_HOLD, tick.now, "hold_lost");
                else if (held > 0)
                    beginCarryPlacement(tick.now, "carry_at_selected_half");
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
                    if (alignCarryForPlacement(tick, tick.in.zone_estimate, true, true))
                        break;
                    const cv::Point2f gate = tick.in.zone_estimate.zoneToBody(
                        {halfX(), -t_.gate_clearance_m - holdCenter()});
                    if (std::hypot(gate.x, gate.y) <= t_.gate_tolerance_m) {
                        prepareCarryPlacement(tick);
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
                // 区域位姿仅用于到达判定和近区对正，不再接管远距离运输方向。
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
                    if (alignCarryForPlacement(tick, z, context.fresh_fix, false))
                        break;
                    const auto approach = transport_phase_.carry_approach_.update(t_, context);
                    if (approach.outcome == CarryApproachOutcome::RaiseRelease) {
                        noteDropZone(approach.drop_event, hold);
                        if (t_.pixel_grasp_enabled)
                            beginCarryPlacement(tick.now, approach.reason);
                        else
                            enter(PushState::RAISE_RELEASE, tick.now, approach.reason);
                        break;
                    }
                    // Far-zone pose axes no longer steer travel, even for one missing
                    // detection frame. Use own-color sight or its existing heading memory;
                    // pose remains responsible for arrival and near-edge alignment only.
                    const bool far_from_edge =
                        hold.y < -t_.assume_release_gap_m - t_.assume_align_zone_m;
                    if (far_from_edge) {
                        if (tick.in.zone_sight_valid && !tick.in.zone_sight_enemy &&
                            std::isfinite(tick.in.zone_sight_bearing)) {
                            transport_phase_.carry_search_.reset();
                            reason_ = "carry_drive_to_sighted_zone";
                            steer(tick, t_.carry_speed, tick.in.zone_sight_bearing);
                        } else
                            carrySearch(tick);
                        break;
                    }
                    reason_ = approach.reason;
                    // Slow the fresh visual near-zone approach without changing yaw steering.
                    steer(tick, t_.carry_speed, approach.heading, false,
                          context.fresh_fix ? .01f : t_.carry_speed);
                    break;
                }
                // Detection-only direction is sufficient to approach; pose remains required
                // by the existing entry/release verification, not by this forward leg.
                if (tick.in.zone_sight_valid && !tick.in.zone_sight_enemy &&
                    std::isfinite(tick.in.zone_sight_bearing)) {
                    transport_phase_.carry_search_.reset();
                    reason_ = "carry_drive_to_sighted_zone";
                    steer(tick, t_.carry_speed, tick.in.zone_sight_bearing);
                    break;
                }
                const bool remembered_own_zone =
                    zone_mem_ && tick.now >= zone_mem_us_ &&
                    tick.now - zone_mem_us_ <= t_.carry_sight_mem_us;
                if (!t_.assume_all_safe || remembered_own_zone) {
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
                    if (t_.pixel_grasp_enabled)
                        beginCarryPlacement(tick.now, "assumed_drop_point");
                    else
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
                if (tick.in.zone_sight_valid && !tick.in.zone_sight_enemy &&
                    std::isfinite(tick.in.zone_sight_bearing)) {
                    transport_phase_.carry_search_.reset();
                    reason_ = "carry_drive_to_sighted_zone";
                    steer(tick, t_.carry_speed, tick.in.zone_sight_bearing);
                    break; // Detection provides approach direction, not release evidence.
                }
                if (zone_mem_ && tick.now >= zone_mem_us_ &&
                    tick.now - zone_mem_us_ <= t_.carry_sight_mem_us)
                    carrySearch(tick);
                else {
                    reason_ = "searching_own_zone";
                    scanSpin(tick);
                }
                break;
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
                prepareCarryPlacement(tick);
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
                if (t_.pixel_grasp_enabled) {
                    // Jaw release: open once, then leave the cargo behind while reversing.
                    transport_phase_.release_retreat_.clearTrip();
                    enter(PushState::BACK_OUT, tick.now, "gripper_opened_at_drop");
                } else if (carryLogic() && !transport_phase_.assume_pushed_ && t_.assume_push_m > 0) {
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
        case TransportAction::BackOut: {
            // Keep the gripper open so reversing leaves the load behind.
            if (t_.pixel_grasp_enabled)
                transport_phase_.release_retreat_.clearTrip();
            commandFrame(1, tick.now);
            const bool jaw_retreat = t_.pixel_grasp_enabled && carryLogic();
            if (!jaw_retreat) {
                const int g = frameWait(tick.in, 1, tick.now);
                if (g <= 0) {
                    if (g < 0)
                        fail("gripper_open_timeout");
                    break;
                }
            } // Jaw release already confirmed open before entering BACK_OUT.
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
                    if (retreat.reverse) {
                        if (jaw_retreat) {
                            // Release retreat is an unconditional straight 0.15 m leg.
                            // Ground mapping and directional clearance do not gate it.
                            tick.motion.vx_mps = std::clamp(-t_.back_speed, -t_.max_speed,
                                                          t_.max_speed);
                            tick.motion.wz_rps = 0;
                        } else {
                            drive(tick, -t_.back_speed, 0);
                        }
                    }
                    break;
                case ReleaseRetreatOutcome::PushIn:
                    transport_phase_.cargo_ref_us_ = 0;
                    transport_phase_.push_in_.prepare(t_.assume_align_push);
                    beginCarryPlacement(tick.now, "");
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
                    if (jaw_retreat) {
                        tick.motion.vx_mps = tick.motion.wz_rps = 0;
                        tick.immediate_motion_stop = true;
                    }
                    enter(PushState::VERIFY_DELIVERY, tick.now, "");
                    break;
                case ReleaseRetreatOutcome::Fault:
                    if (jaw_retreat)
                        failStop(tick, retreat.reason.c_str());
                    else
                        fail(retreat.reason.c_str());
                    break;
            }
            break;
        }
        case TransportAction::TurnAround: {
            auto& q = transport_phase_.jaw_sequence_;
            const auto& in = tick.in;
            commandFrame(1, tick.now);
            tick.immediate_motion_stop = true;
            const bool feedback = in.sent_velocity_valid && std::isfinite(in.sent_vx_mps) &&
                std::isfinite(in.sent_wz_rps);
            if (!feedback || !in.heading_valid || !std::isfinite(in.heading_rad)) {
                failStop(tick, "return_turn_heading_feedback_missing");
                break;
            }
            if (tick.elapsed > std::max<uint64_t>(t_.turn_budget_us, 20000000)) {
                failStop(tick, "return_turn_timeout");
                break;
            }
            const bool stopped = std::abs(in.sent_vx_mps) < .001f && std::abs(in.sent_wz_rps) < .001f;
            const auto settled = [&] {
                if (!stopped) { q.stopped_us = 0; return false; }
                if (!q.stopped_us) q.stopped_us = tick.now;
                return tick.now - q.stopped_us >= 250000;
            };
            if (!q.return_started) {
                reason_ = "return_turn_initial_stop";
                if (settled()) {
                    q.return_started = true;
                    q.return_last_heading = in.heading_rad;
                    q.return_turn = 0;
                    q.stopped_us = 0;
                }
                break;
            }
            // Signed net heading change rejects oscillation; wrap each increment across +/-pi.
            q.return_turn += wrapAngle(in.heading_rad - q.return_last_heading);
            q.return_last_heading = in.heading_rad;
            const float sign = t_.turn_wz >= 0 ? 1.f : -1.f;
            const float remaining = sign * float(CV_PI) - q.return_turn;
            if (std::abs(remaining) <= .035f) {
                reason_ = "return_turn_180_stop";
                if (settled()) {
                    sweep_ledger_.newTrip(t_, tick.now);
                    scanAfterClear(tick, "return_turn_180_complete");
                }
            } else {
                q.stopped_us = 0;
                reason_ = "return_turn_180_imu";
                if (std::abs(in.sent_vx_mps) < .001f)
                    drive(tick, 0, std::clamp(t_.heading_gain * remaining, -.3f, .3f), true);
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
                beginCarryPlacement(tick.now, verification.reason.c_str());
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
            if (t_.pixel_grasp_enabled && carryLogic() && transport_phase_.jaw_sequence_.active)
                enter(PushState::TURN_AROUND, tick.now, "delivery_imu_return_turn");
            else if (t_.demo_carry_once && t_.assume_all_safe)
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
