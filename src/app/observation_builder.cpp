#include "rescue/app/observation_builder.hpp"
#include "rescue/app/runtime_config.hpp"
#include "rescue/controlled_field.hpp"
#include "rescue/imu_adapter.hpp"
#include "rescue/motion_watch.hpp"
#include "rescue/sensor_fusion.hpp"
#include "rescue/silhouette.hpp"
#include "rescue/utils.hpp"
#include "rescue/zone_visual_reference.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

namespace rescue::app {
ObservationBuilder::ObservationBuilder(const ResolvedRuntimeConfig& config,
                                       GeometryPipeline* geometry,
                                       const CameraCalibration& vref_calibration,
                                       bool vref_ready,
                                       const ZoneGeometry& zone_geometry,
                                       const ZoneColorClassifier& zone_colors,
                                       ControlledField& controlled_field,
                                       SensorFusion& fusion)
    : config_(config), geometry_(geometry), vref_calibration_(vref_calibration),
      vref_ready_(vref_ready), zone_geometry_(zone_geometry), zone_colors_(zone_colors),
      controlled_field_(controlled_field), fusion_(fusion), capture_(config.capture),
      multi_view_(config.frame_views,
                  {config.request.frame_width, config.request.frame_height},
                  config.view_image_polygon) {
    const auto& tuning = config.task;
    pixel_selector_.setRelock(tuning.relock_window_us, tuning.relock_gate_frac);
    RowParams rp;
    rp.min_n = tuning.row_min_n;
    rp.resid_m = tuning.row_resid_m;
    rp.max_tilt_rad = tuning.row_max_tilt_rad;
    rp.half_along_m = tuning.row_half_along_m;
    rp.half_across_m = tuning.row_half_across_m;
    pixel_selector_.setRow(tuning.row_select, rp);
}

ObservationResult ObservationBuilder::build(const cv::Mat& frame,
                                            const PerceptionResult& vision_result,
                                            std::vector<SegDetection> detections,
                                            uint64_t frame_sequence,
                                            uint64_t timestamp,
                                            const ObservationContext& context) {
    const auto& config = config_.request;
    auto* geometry = geometry_;
    const auto& vref_calibration = vref_calibration_;
    const bool vref_ready = vref_ready_;
    const auto& zone_geometry = zone_geometry_;
    const auto& zone_colors = zone_colors_;
    auto& controlled_field = controlled_field_;
    auto& fusion = fusion_;
    auto& capture = capture_;
    auto& pixel_selector = pixel_selector_;
    auto& multi_view = multi_view_;
    auto& progress_ref_us = progress_ref_us_;
    auto& progress_total = progress_total_;
    auto& observing = observing_;
    auto& observation_round = observation_round_;
    const auto& task = context.task;
    const auto& previous = context.previous;
    const auto* feedback = context.feedback;
    const auto* imu = context.imu;
    const auto& motion_watch = context.motion_watch;
    const bool link = context.motion_link_enabled;
    const bool a6_done_flag = config_.a6_done_flag;
    const int feedback_close = config_.feedback_close;
    const bool box_area_accepted = config_.box_area_accepted;
    const std::string prefer_label = config.assume_injured_trip ? "injured_person" : "";
    const auto obstacleConfidence = [&](const SegDetection& d) {
        return d.label == "injured_person" ? std::min(config.confidence, config.injured_confidence)
                                           : config.confidence;
    };
    std::vector<ZoneKeypoint> zone_points;
    if (vision_result.pose_ran)
        zone_points = zoneHalvesToKeypoints(vision_result.halves, config.pose_keypoint_confidence);
    const auto observed_at = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch())
            .count());
    const bool blue_model_detected_both_halves =
        !config.pose_model_blue_path.empty() && vision_result.pose_ran &&
        vision_result.halves[0].valid && vision_result.halves[1].valid;
    auto zone_color = previewBlueZoneDefault(
        zone_colors.classify(
            frame, vision_result.halves, detections, config.pose_keypoint_confidence),
        config.dry_run,
        config.team,
        blue_model_detected_both_halves);
    // Identity comes only from the measured zone colour; an enemy-colour zone is never labelled
    // own.
    zone_color = resolveZoneColor(zone_color, config.team, false);
    GeometryResult geometry_result;
    ExpectedStop expected_stop;
    ZoneEstimate vref_estimate;
    bool vref_ok = false;
    // Servo readback at capture time; without the feedback port it stays invalid.
    const FrameSensors frame_sensors = feedback ? feedback->feedbackAt(timestamp) : FrameSensors{};
    for (auto& d : detections)
        if (d.label == "injured_person")
            d.silhouette_hw = orangeSilhouetteAspect(frame, d.box);
    // Close-range injured score below the target threshold: obstacle evidence only, and only with
    // an orange blob (no mat/background ghosts).
    detections.erase(std::remove_if(detections.begin(),
                                    detections.end(),
                                    [&](const SegDetection& d) {
                                        return d.label == "injured_person" &&
                                               d.confidence < config.confidence &&
                                               d.silhouette_hw < 0;
                                    }),
                     detections.end());
    if (geometry) {
        GeometryFrame geometry_frame;
        geometry_frame.frame_id = frame_sequence;
        geometry_frame.capture_us = timestamp;
        geometry_frame.now_us = observed_at;
        geometry_frame.image_size = frame.size();
        geometry_frame.sensors = frame_sensors;
        if (imu)
            geometry_frame.sensors.imu = sensorStateFromImu(imu->snapshotAt(timestamp));
        KeypointFrame points;
        points.frame_id = frame_sequence;
        points.capture_us = timestamp;
        points.image_size = frame.size();
        points.zone_label = config.team + "_safe_zone";
        // Stage 3 supplies measured same-frame points. Missing input is invalid,
        // never reconstructed from a bounding rectangle.
        if (vision_result.pose_ran) {
            // Preview may fit blue geometry from the selected model, but only observed color
            // verifies ownership.
            points.identity_verified = zone_color.verified;
            points.zone_label =
                zone_color.color.empty() ? "unknown" : zone_color.color + "_safe_zone";
            points.geometry_id = zone_geometry.id;
            points.points = zone_points;
        } else if (!config.keypoints_file.empty()) {
            try {
                cv::FileStorage file(config.keypoints_file, cv::FileStorage::READ);
                if (file.isOpened()) {
                    points = readKeypointFrame(file.root());
                    expected_stop = readExpectedStop(file.root(), points);
                }
            } catch (const std::exception&) {
                points = {};
                expected_stop = {};
            }
        }
        geometry_result = geometry->process(geometry_frame, points, detections);
        if (vref_ready && vision_result.pose_ran && zone_color.color == config.team) {
            vref_estimate = ZoneEstimate{};
            vref_ok = estimateVisualZone(vref_calibration,
                                         zone_geometry,
                                         zone_points,
                                         frame_sensors.actuator.camera_pitch_cdeg,
                                         &geometry_frame.sensors.imu,
                                         config.pose_keypoint_confidence,
                                         vref_estimate);
            if (vref_ok) {
                vref_estimate.timestamp_us = observed_at;
                vref_estimate.observed_us = observed_at;
                vref_estimate.zone_label = points.zone_label;
            }
            std::cout << "[VREF] ok=" << vref_ok << " n=" << vref_estimate.inlier_ids.size();
            if (vref_ok)
                std::cout << " ox=" << vref_estimate.origin_body_m.x
                          << " oy=" << vref_estimate.origin_body_m.y
                          << " yaw=" << vref_estimate.yaw_body_rad
                          << " res=" << vref_estimate.residual_m;
            std::cout << "\n";
        }
        detections = geometry_result.detections;
    }
    auto rejected_ids = task.rejectedTargets(observed_at);
    {
        // A vetoed block keeps its world bearing even when the tracker re-issues its id.
        const auto bearings = task.rejectedBearings(observed_at);
        const auto pre_imu = imu ? imu->snapshot() : ImuSnapshot{};
        if (!bearings.empty() && pre_imu.fresh && pre_imu.sample.measurements_valid &&
            std::isfinite(pre_imu.sample.body_rpy_rad[2])) {
            const float heading = pre_imu.sample.body_rpy_rad[2];
            for (const auto& d : detections) {
                if (!d.ground_position_valid || d.body_xy_m.y <= 0.f)
                    continue;
                const float world = heading + std::atan2(-d.body_xy_m.x, d.body_xy_m.y);
                const float range = std::hypot(d.body_xy_m.x, d.body_xy_m.y);
                const float tol = std::clamp(.05f / std::max(range, .15f), .04f, .2f);
                for (const auto& b : bearings) {
                    float diff = world - b.world_yaw;
                    diff = std::atan2(std::sin(diff), std::cos(diff));
                    if (std::abs(diff) <= tol && std::abs(range - b.range_m) <= .6f) {
                        rejected_ids.push_back(d.track_id);
                        break;
                    }
                }
            }
        }
    }
    {
        // No delivery for a long time: stop being picky about crowded blocks (60 s: lane, 120 s:
        // everything).
        if (!progress_ref_us || previous.delivered_total != progress_total) {
            progress_ref_us = observed_at;
            progress_total = previous.delivered_total;
        }
        const uint64_t idle_us = observed_at > progress_ref_us ? observed_at - progress_ref_us : 0;
        pixel_selector.setRelax(idle_us >= 120000000 ? 2 : idle_us >= 60000000 ? 1 : 0);
    }
    if (previous.state == PushState::SCATTER || previous.state == PushState::PILE)
        pixel_selector.forgetCenter(); // the sweep swings the view: the old pixel position no
                                       // longer marks where the lock vanished
    const auto selection =
        pixel_selector.update(detections,
                              frame.size(),
                              frame_sensors.actuator.camera_pitch_cdeg,
                              frame_sensors.actuator.valid && frame_sensors.pitch_stable,
                              observed_at,
                              previous.first_ordinary_delivered,
                              config.confidence,
                              rejected_ids,
                              prefer_label);
    auto input = makePushObservation(detections,
                                     observed_at,
                                     previous.first_ordinary_delivered,
                                     config.confidence,
                                     selection.id,
                                     rejected_ids,
                                     false,
                                     false,
                                     false,
                                     prefer_label);
    if (!selection.locked)
        input.target_valid = false;
    (void)box_area_accepted; // 现场同步：像素比不再要求box_area_accepted标定（沿用demo）
    std::cout << "[PIXEL_SELECTION] timestamp=" << observed_at
              << " pitch=" << frame_sensors.actuator.camera_pitch_cdeg
              << " stable=" << frame_sensors.pitch_stable << " id=" << selection.id
              << " mode=" << selection.mode << " raw=" << selection.raw
              << " smooth=" << selection.smoothed << " locked=" << selection.locked
              << " reason=" << selection.reason
              << (selection.row.empty() ? std::string() : " ROW[" + selection.row + "]") << "\n";
    input.pixel_ratio_raw = selection.raw;
    input.pixel_ratio_smoothed = selection.smoothed;
    input.pixel_mode = selection.mode;
    input.pixel_reason = selection.reason;
    input.pixel_locked = selection.locked;
    if (selection.locked && selection.id >= 0 && vref_ready &&
        vref_calibration.cameraMatrix().type() == CV_64F &&
        vref_calibration.cameraMatrix().rows == 3)
        for (const auto& d : detections)
            if (d.track_id == selection.id) {
                const cv::Mat& K = vref_calibration.cameraMatrix();
                const double xc = (d.box.x + d.box.width * .5 - K.at<double>(0, 2)) /
                                  K.at<double>(0, 0),
                             yc = (d.box.y + d.box.height * .5 - K.at<double>(1, 2)) /
                                  K.at<double>(1, 1);
                const double pitch =
                    double(frame_sensors.actuator.camera_pitch_cdeg) * .01 * CV_PI / 180.;
                const double fwd = std::cos(pitch) - yc * std::sin(pitch);
                if (fwd > .05) {
                    input.pixel_bearing_valid = true;
                    input.pixel_bearing = float(std::atan2(-xc, fwd));
                }
                input.pixel_confidence = d.confidence;
                std::cout << "[VISUAL_SIGHT] id=" << selection.id << " label=" << d.label
                          << " conf=" << d.confidence << " bearing=" << input.pixel_bearing << "\n";
                break;
            }
    input.run = false; // MatchControl supplies the final authorization below.
    if (geometry)
        geometry->apply(input, geometry_result, expected_stop, config.team, imuNowUs());
    input.enemy_zone_seen = vision_result.pose_ran && zone_color.verified && !zone_color.assumed &&
                            zone_color.color != config.team;
    if (vref_ready && vision_result.pose_ran && !zone_points.empty() &&
        vref_calibration.cameraMatrix().type() == CV_64F &&
        vref_calibration.cameraMatrix().rows == 3) {
        cv::Point2f c(0, 0);
        for (const auto& k : zone_points)
            c += k.pixel;
        c *= 1.f / float(zone_points.size());
        const cv::Mat& K = vref_calibration.cameraMatrix();
        const double xc = (c.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                     yc = (c.y - K.at<double>(1, 2)) / K.at<double>(1, 1);
        const double pitch = double(frame_sensors.actuator.camera_pitch_cdeg) * .01 * CV_PI / 180.;
        const double fwd = std::cos(pitch) - yc * std::sin(pitch);
        if (fwd > .05) {
            input.zone_sight_valid = true;
            input.zone_sight_enemy = input.enemy_zone_seen;
            input.zone_sight_bearing = float(std::atan2(-xc, fwd));
        }
    }
    for (const auto& d : detections)
        if (d.label == "injured_person" && d.track_id != selection.id &&
            (d.injured_upright_rejected || !d.ground_contact_valid) && d.ground_position_valid &&
            d.confidence >= obstacleConfidence(d) && std::isfinite(d.body_xy_m.x) &&
            std::isfinite(d.body_xy_m.y))
            input.upright_body.push_back(d.body_xy_m);
    if (!input.upright_body.empty())
        input.upright_ts_us = observed_at;
    for (const auto& d : detections)
        if (d.track_id != selection.id && d.ground_position_valid &&
            d.confidence >= obstacleConfidence(d) && std::isfinite(d.body_xy_m.x) &&
            std::isfinite(d.body_xy_m.y)) {
            input.clutter_body.push_back(d.body_xy_m);
            if (d.label == "ordinary_supply")
                input.clutter_ordinary.push_back(d.body_xy_m);
        }
    if (!input.clutter_body.empty())
        input.clutter_ts_us = observed_at;
    {
        for (const auto& d : detections)
            if (d.ground_position_valid && d.confidence >= config.confidence &&
                std::isfinite(d.body_xy_m.x) && std::isfinite(d.body_xy_m.y) &&
                (d.label == "ordinary_supply" || d.label == "core_supply" ||
                 d.label == "injured_person")) {
                input.cargo_body.push_back(d.body_xy_m);
                PushObservation::CargoImg ci;
                ci.bottom_px = float(d.box.y + d.box.height);
                ci.height_px = float(d.box.height);
                ci.clipped = ci.bottom_px >= float(frame.rows) - 4.f;
                // Front rim edge (keypoints 0/1/2) at the block's column; needs two visible and the
                // block between them.
                std::vector<cv::Point2f> fr;
                for (const auto& k : zone_points)
                    if (k.visible && k.id >= 0 && k.id <= 2)
                        fr.push_back(k.pixel);
                if (fr.size() >= 2) {
                    float xl = fr[0].x, xr = fr[0].x, ys = 0;
                    for (const auto& q : fr) {
                        xl = std::min(xl, q.x);
                        xr = std::max(xr, q.x);
                        ys += q.y;
                    }
                    const float cx = d.box.x + d.box.width * .5f, m = .25f * (xr - xl);
                    if (xr - xl > 20.f && cx >= xl - m && cx <= xr + m) {
                        ci.front_px = ys / float(fr.size());
                        ci.front_valid = true;
                    }
                }
                input.cargo_img.push_back(ci);
            }
        input.cargo_ts_us = observed_at;
    }
    // 严格位姿优先：geometry_pipeline的区域估计可信（非预测、残差/不确定度/时效通过、>=3内点）时作为区域定位喂给PushTask；
    // 否则降级为VREF单帧估计，再不行由PushTask内部航位推算/盲走兜底。
    const bool strict_fix = input.zone_estimate.valid &&
                            input.zone_estimate.source != ZoneEstimate::Source::PREDICTED &&
                            input.zone_estimate.inlier_ids.size() >= 3 &&
                            input.zone_estimate.trusted(observed_at);
    if (strict_fix) {
        input.vref_valid = true;
        input.vref_zone = input.zone_estimate;
        input.vref_zone.timestamp_us = observed_at;
        input.vref_zone.observed_us = observed_at;
        input.vref_points = int(input.zone_estimate.inlier_ids.size());
        std::cout << "[ZONE_FIX] source=strict n=" << input.vref_points << "\n";
    } else if (vref_ok) {
        input.vref_valid = true;
        input.vref_zone = vref_estimate;
        input.vref_points = int(vref_estimate.inlier_ids.size());
        std::cout << "[ZONE_FIX] source=vref n=" << input.vref_points << "\n";
    }
    if (frame_sensors.actuator.valid) {
        // Done-flag A6 cannot show position: closed only after this host's close target completed.
        input.gripper_closed_observed =
            link ? feedback->confirmedFrameOpen(frame_sensors.actuator) == 0
                 : !a6_done_flag && feedback_close >= 0 &&
                       frame_sensors.actuator.gripper_open == feedback_close;
        input.camera_pitch_cdeg = frame_sensors.actuator.camera_pitch_cdeg;
        input.camera_pitch_stable = frame_sensors.pitch_stable;
    }
    // Gripper completion: id echoed and observed open/closed state matches the last target.
    // Read-only feedback never sends, so it never acknowledges anything.
    UARTController::GripperAck gripper_ack;
    if (link) {
        gripper_ack = feedback->gripperAck();
        if (gripper_ack.result == ActionResult::Done) {
            input.gripper_done = true;
            input.gripper_feedback_open = gripper_ack.target;
        }
    }
    // Holding-region and rush-corridor inventory; uncertainty leaves them incomplete.
    capture.update(input, detections, observed_at);
    if (input.target_valid && input.geometry_valid) {
        static uint64_t last_clutter_log_us = 0;
        if (observed_at - last_clutter_log_us >= 1000000) {
            last_clutter_log_us = observed_at;
            std::cout << "[CORRIDOR_NEIGHBOURS] target=" << input.target_id
                      << " dist=" << input.distance_m << " head=" << input.heading_error
                      << " others=";
            for (const auto& d : detections) {
                if (d.track_id == selection.id || !d.ground_position_valid ||
                    d.confidence < obstacleConfidence(d) || !std::isfinite(d.body_xy_m.x) ||
                    !std::isfinite(d.body_xy_m.y))
                    continue;
                std::cout << d.label << "#" << d.track_id << "@(" << d.body_xy_m.x << ","
                          << d.body_xy_m.y << ") ";
            }
            std::cout << "\n";
        }
    }
    if (input.target_valid && input.geometry_valid &&
        (previous.state == PushState::MID_APPROACH || previous.state == PushState::APPROACH ||
         previous.state == PushState::PREPARE || previous.state == PushState::RUSH)) {
        // Event log: only when the set of counted neighbours changes, or once a second as a
        // heartbeat.
        static std::string last_signature;
        static uint64_t last_evidence_log_us = 0;
        std::string signature;
        for (const auto& e : capture.corridorEntries())
            signature += std::to_string(e.track_id) + (e.counted ? "+" : "-") +
                         (e.footprint_only ? "f" : "") + ",";
        if (signature != last_signature || observed_at - last_evidence_log_us >= 1000000) {
            last_signature = signature;
            last_evidence_log_us = observed_at;
            std::cout << "[CORRIDOR_EVIDENCE] state=" << PushTask::name(previous.state)
                      << " target=" << input.target_id << " dist=" << input.distance_m
                      << " head=" << input.heading_error << " complete=" << input.corridor_complete
                      << " clean=" << input.corridor_occlusion_free << " counted=o"
                      << input.corridor.ordinary << "/c" << input.corridor.core << "/i"
                      << input.corridor.injured << "/d" << input.corridor.dangerous << "/u"
                      << input.corridor.unknown << " entries=";
            for (const auto& e : capture.corridorEntries())
                std::cout << e.label << "#" << e.track_id << "{conf=" << e.confidence
                          << ",age_ms=" << e.age_us / 1000 << ",along=" << e.along_m
                          << ",across=" << e.across_m << ",fp=" << e.footprint_m << "," << e.reason
                          << (e.footprint_only ? ",footprint_only" : "") << "} ";
            std::cout << "\n";
        }
    }
    if (previous.state == PushState::VERIFY_CAPTURE) {
        const auto fb = feedback ? feedback->latestActuatorFeedback() : ActuatorFeedback{};
        if (!observing) {
            multi_view.begin(observed_at, fb.gripper_action_id, ++observation_round);
            observing = true;
        }
        multi_view.update(observed_at,
                          timestamp,
                          fb.gripper_action_id,
                          fb.valid && input.gripper_done && input.gripper_feedback_open == 0,
                          previous.motion.vx_mps == 0 && previous.motion.wz_rps == 0,
                          input.camera_pitch_cdeg,
                          input.camera_pitch_stable,
                          detections);
        const auto& evidence = multi_view.result();
        input.multi_view_finished = evidence.finished;
        input.multi_view_verdict = int(evidence.verdict);
        input.multi_view_inventory = evidence.inventory;
        input.multi_view_pitch = multi_view.desiredPitch();
        input.multi_view_reason = evidence.reason;
        input.observation_view = evidence.view;
        input.observation_frames = evidence.frames;
        input.observation_round = evidence.round;
        std::cout << "[MULTI_VIEW] timestamp=" << observed_at << " round=" << evidence.round
                  << " action=" << int(fb.gripper_action_id) << " view=" << evidence.view
                  << " frames=" << evidence.frames
                  << " verdict=" << captureVerdictName(evidence.verdict)
                  << " finished=" << evidence.finished << " reason=" << evidence.reason << "\n";
    } else if (observing) {
        multi_view.reset();
        observing = false;
    }

    if (config.controlled_empty_field) {
        // injured-trip forces first=true for target selection only; the unseen-zone fallback must
        // stay on until a real delivery.
        PushOutput field_previous = previous;
        if (config.assume_injured_trip)
            field_previous.first_ordinary_delivered = previous.delivered_total > 0;
        controlled_field.update(input,
                                detections,
                                vision_result.halves,
                                frame.size(),
                                field_previous,
                                geometry_result.mapping_valid,
                                config.controlled_ignore_clearance);
    }
    // Raw boxes do not establish metric distance, route safety or delivery.
    // Keep unconnected evidence invalid; never synthesize successful observations.
    const auto imu_data = imu ? imu->snapshot() : ImuSnapshot{};
    // IMU absence/fault/over-tilt can veto permission; it never establishes geometry or route
    // safety. SensorState uses body axes (roll-180 mounting calibrated 2026-09-27), same steady
    // clock as observed_at.
    const SensorState sensor = sensorStateFromImu(imu_data);
    if (imu) {
        fusion.update(sensor);
        input.safety_ok = !fusion.emergencyStop(imuNowUs());
        // Relative yaw only measures the post-delivery turn.
        input.heading_valid = imu_data.fresh && imu_data.sample.measurements_valid &&
                              std::isfinite(imu_data.sample.body_rpy_rad[2]);
        input.heading_rad = imu_data.sample.body_rpy_rad[2];
        input.imu_blocked_us = motion_watch.blockedUs();
        input.imu_spin_stall_us = motion_watch.spinStallUs();
    }
    return {std::move(input),
            std::move(detections),
            std::move(zone_points),
            std::move(zone_color),
            std::move(geometry_result),
            selection,
            frame_sensors,
            gripper_ack,
            imu_data,
            sensor,
            observed_at};
}
} // namespace rescue::app
