#include "rescue/config/config.hpp"
#include "rescue/config/demo_policy.hpp"
#include "rescue/core/async_log.hpp"
#include <sstream>
#include <set>
#include <iomanip>
#include <cctype>
#include <limits>
#include "rescue/control/controlled_field.hpp"
#include "rescue/geometry/geometry_pipeline.hpp"
#include "rescue/perception/silhouette.hpp"
#include "rescue/hardware/uart_controller.hpp"
#include "rescue/hardware/motion_link.hpp"
#include "rescue/control/motion_readiness.hpp"
#include "rescue/control/match_control.hpp"
#include "rescue/control/match_server.hpp"
#include "rescue/hardware/hipnuc_imu.hpp"
#include "rescue/control/motion_watch.hpp"
#include "rescue/hardware/imu_adapter.hpp"
#include "rescue/hardware/sensor_fusion.hpp"
#include "rescue/perception/detector.hpp"
#include "rescue/perception/zone_keypoints.hpp"
#include "rescue/geometry/zone_visual_reference.hpp"
#include "rescue/perception/zone_color.hpp"
#include "rescue/task/push_task.hpp"
#include "rescue/perception/capture_monitor.hpp"
#include "rescue/config/task_calibration.hpp"
#include "rescue/geometry/navigation_adapter.hpp"
#include "rescue/perception/perception_adapter.hpp"
#include "rescue/perception/pixel_selector.hpp"
#include "rescue/perception/multi_view_capture.hpp"
#include "rescue/hardware/camera_capture.hpp"
#include "rescue/hardware/camera_settings.hpp"
#include "rescue/core/stage_audit.hpp"
#include "rescue/perception/tracker.hpp"
#include "rescue/core/utils.hpp"
#include "rescue/perception/vision_logic.hpp"
#include <opencv2/opencv.hpp>
#include <csignal>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <memory>
#include <future>
#include <filesystem>
#include <ctime>
#include <map>
#ifdef RESCUE_ENABLE_TELEMETRY
#include "rescue/telemetry/telemetry_publisher.hpp"
#endif

#include "rescue/app/runtime.hpp"
#include "rescue/app/observation_builder.hpp"
#include "rescue/app/runtime_config.hpp"
#include "rescue/app/config_loader.hpp"
#include "rescue/app/offline_tasks.hpp"
#include "rescue/app/perception_pipeline.hpp"
#include "rescue/app/diagnostics.hpp"
#include <fstream>

namespace rescue::app {
namespace {
// Saves a raw + annotated JPEG pair and one [VISION_JUDGE] line so a run can be judged from images
// afterwards.
void saveVisionJudge(const std::string& dir,
                     const char* tag,
                     uint64_t seq,
                     const cv::Mat& frame,
                     const std::vector<rescue::Detection>& dets,
                     const std::string& caption = "",
                     int highlight_id = -1,
                     int input_id = -1,
                     int near_id = -1,
                     bool output_missing = false) {
    try {
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        char stamp[32];
        const std::time_t t = std::time(nullptr);
        std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", std::localtime(&t));
        const std::string base = dir + "/" + stamp + "-" + tag + "-f" + std::to_string(seq);
        cv::Mat annotated = frame.clone();
        std::map<std::string, int> counts;
        std::ostringstream items;
        for (const auto& d : dets) {
            ++counts[d.label];
            const bool danger = d.label.find("danger") != std::string::npos;
            const cv::Scalar color = danger ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 200, 0);
            cv::rectangle(annotated, d.box, color, 2);
            const auto mark = [&](int pad, const cv::Scalar& c, const std::string& text, int row) {
                cv::rectangle(annotated,
                              cv::Rect(d.box.x - pad,
                                       d.box.y - pad,
                                       d.box.width + 2 * pad,
                                       d.box.height + 2 * pad),
                              c,
                              3);
                cv::putText(annotated,
                            text,
                            cv::Point(d.box.x, d.box.y + d.box.height + 16 + 16 * row),
                            cv::FONT_HERSHEY_SIMPLEX,
                            .55,
                            c,
                            2);
            };
            if (highlight_id >= 0 && d.track_id == highlight_id)
                mark(4, cv::Scalar(0, 255, 255), "OUT#" + std::to_string(d.track_id), 0);
            if (input_id >= 0 && d.track_id == input_id)
                mark(9, cv::Scalar(255, 255, 0), "IN#" + std::to_string(d.track_id), 1);
            if (near_id >= 0 && d.track_id == near_id)
                mark(14, cv::Scalar(255, 0, 255), "NEAREST#" + std::to_string(d.track_id), 2);
            char conf[16];
            std::snprintf(conf, sizeof(conf), "%.2f", d.confidence);
            cv::putText(annotated,
                        d.label + " #" + std::to_string(d.track_id) + " " + conf,
                        cv::Point(d.box.x, std::max(d.box.y - 6, 12)),
                        cv::FONT_HERSHEY_SIMPLEX,
                        .5,
                        color,
                        1);
            items << " " << d.label << "#" << d.track_id << "@" << conf;
            if (d.ground_position_valid)
                items << ",xy=" << d.body_xy_m.x << "," << d.body_xy_m.y;
        }
        if (!caption.empty()) {
            cv::rectangle(
                annotated, cv::Rect(0, 0, annotated.cols, 30), cv::Scalar(0, 0, 0), cv::FILLED);
            cv::putText(annotated,
                        caption,
                        cv::Point(8, 21),
                        cv::FONT_HERSHEY_SIMPLEX,
                        .6,
                        cv::Scalar(255, 255, 255),
                        1);
        }
        if (output_missing) {
            cv::rectangle(
                annotated, cv::Rect(0, 30, annotated.cols, 30), cv::Scalar(0, 0, 160), cv::FILLED);
            cv::putText(annotated,
                        "OUTPUT ID NOT IN VIEW (yellow=out cyan=in magenta=nearest same-class)",
                        cv::Point(8, 51),
                        cv::FONT_HERSHEY_SIMPLEX,
                        .6,
                        cv::Scalar(255, 255, 255),
                        1);
        }
        const bool ok = cv::imwrite(base + "-raw.jpg", frame) &&
                        cv::imwrite(base + "-annotated.jpg", annotated);
        std::ostringstream c;
        for (const auto& kv : counts)
            c << (c.tellp() > 0 ? "," : "") << kv.first << ":" << kv.second;
        std::cout << "[VISION_JUDGE] tag=" << tag << " seq=" << seq << " saved=" << ok
                  << " dets=" << dets.size() << " counts=" << (c.tellp() > 0 ? c.str() : "none")
                  << " raw=" << base << "-raw.jpg annotated=" << base << "-annotated.jpg"
                  << (caption.empty() ? "" : " caption=" + caption)
                  << (output_missing ? " note=输出ID不在画面" : "") << " in_id=" << input_id
                  << " out_id=" << highlight_id << " nearest_id=" << near_id
                  << " items=" << (items.tellp() > 0 ? items.str() : " none") << "\n";
    } catch (const std::exception& e) {
        std::cout << "[VISION_JUDGE] tag=" << tag << " saved=0 error=" << e.what() << "\n";
    }
}

int checkConfiguration(const ResolvedRuntimeConfig& resolved, std::ostream& output) {
    const Config& config = resolved.request;
    const auto& tuning = resolved.task;
    const auto& capture_config = resolved.capture;
    const auto& body_envelope = resolved.body_envelope;
    const auto measured_load_radius = resolved.measured_load_radius;
    const auto measured_body_radius = resolved.measured_body_radius;
    const auto load_half_width = resolved.load_half_width;
    const auto load_half_depth = resolved.load_half_depth;
    const auto& frame_views = resolved.frame_views;
    const auto feedback_open = resolved.feedback_open;
    const auto feedback_close = resolved.feedback_close;
    const auto box_area_accepted = resolved.box_area_accepted;
    const auto a6_done_flag = resolved.a6_done_flag;
    const auto a6_done_settle_ms = resolved.a6_done_settle_ms;
    const auto view_image_polygon = resolved.view_image_polygon;
    const auto task_calibrated = resolved.task_calibrated;
    const auto a6_known = resolved.a6_known;
    int missing = 0;
    const auto check = [&](bool ok, const char* reason) {
        output << (ok ? "OK " : "MISSING ") << reason << "\n";
        missing += !ok;
    };
    check(task_calibrated, "task_calibration");
    check(a6_known && (config.demo_mode == "search" || frame_views.size() >= 2),
          "new_frame_mapping_and_required_calibration");
    const auto g = ZoneGeometry::load(config.zone_geometry_file, config.team + "_safe_zone");
    check(loadFitsHalf(measured_load_radius,
                       g.width_m,
                       g.depth_m,
                       g.divider_exclusion_half_width_m,
                       load_half_width,
                       load_half_depth),
          "load_footprint_fits_half");
    CameraCalibration c;
    const bool mapped = !config.calibration_file.empty() &&
                        c.load(config.calibration_file, config.allow_mechanical_pitch_model);
    check(mapped && c.pitchUsable(tuning.far_pitch_cdeg) && c.pitchUsable(tuning.track_pitch_cdeg),
          "far_track_ground_calibration");
    if (tuning.intermediate_pitch_cdeg != kCameraPitchInvalid)
        check(mapped && c.pitchUsable(tuning.intermediate_pitch_cdeg), "intermediate_ground_model");
    check(!config.zone_color_file.empty() && ZoneColorConfig::load(config.zone_color_file).measured,
          "zone_identity_source");
    if (config.controlled_empty_field)
        check(measured_body_radius > 0, "robot_swept_radius");
    check(!config.pose_model_path.empty() || !config.keypoints_file.empty(), "zone_pose_source");
    output << "STARTUP forward_ms=" << config.startup_advance_ms
           << " speed_mps=" << tuning.startup_advance_speed << " scan_wz_rps=" << tuning.scan_wz
           << " turn_wz_rps=" << tuning.turn_wz << " angular_limit_rps=" << tuning.max_wz
           << " clearance_checks_disabled=" << config.controlled_ignore_clearance
           << " search_cues=" << config.search_cues << "\n";
    output << "STATIC_ONLY: live observations, sensor timing and actual motion/holding "
              "still require independent evidence. hardware_output=disabled\n";
    return missing ? 2 : 0;
}
} // namespace

int Runtime::run() {
    const auto& resolved = config_;
    const Config& config = resolved.request;
    const auto& tuning = resolved.task;
    const auto& body_envelope = resolved.body_envelope;
    const auto measured_load_radius = resolved.measured_load_radius;
    const auto measured_body_radius = resolved.measured_body_radius;
    const auto load_half_width = resolved.load_half_width;
    const auto load_half_depth = resolved.load_half_depth;
    const auto feedback_open = resolved.feedback_open;
    const auto feedback_close = resolved.feedback_close;
    const auto a6_done_flag = resolved.a6_done_flag;
    const auto a6_done_settle_ms = resolved.a6_done_settle_ms;
    const auto task_calibrated = resolved.task_calibrated;
    PushTask task(tuning);
    if (!config.dry_run && !config.hardware)
        throw std::runtime_error("Select --dry-run (perception only) or --hardware (MCU output). "
                                 "No serial port was opened.");
    AsyncLog async_log;
    std::string last_stop_signature;
    uint64_t last_stale_count = 0;
    auto last_submit_time = Clock::now();
    std::unique_ptr<HipnucImu> imu;
    if (config.imu) {
        imu = std::make_unique<HipnucImu>(config.imu_port, config.imu_baud, config.imu_timeout_ms);
        std::cout
            << "[IMU] " << config.imu_port << " @ " << config.imu_baud
            << " HI91; body axes via roll-180 mounting (2026-09-27), firmware status unverified\n";
    }
    // 车体安全：IMU过期/数值异常/倾斜超过imu_tilt_limit_deg时否决safety_ok。
    SensorFusion fusion(
        {config.sensor_timeout_ms, config.tof_stop_distance_m, config.imu_tilt_limit_deg});
    std::unique_ptr<UARTController> feedback;
    std::unique_ptr<GeometryPipeline> geometry;
    CameraCalibration vref_calibration;
    bool vref_ready = false;
    ZoneGeometry zone_geometry;
    if (!config.calibration_file.empty()) {
        CameraCalibration calibration;
        if (!calibration.load(config.calibration_file, config.allow_mechanical_pitch_model))
            throw std::runtime_error("Invalid ground calibration file");
        zone_geometry = ZoneGeometry::load(config.zone_geometry_file, config.team + "_safe_zone");
        const auto& contact = resolved.ground_contact;
        geometry = std::make_unique<GeometryPipeline>(calibration, zone_geometry, contact);
        vref_calibration = calibration;
        vref_ready = true;
    }
    CarryNavigator navigator(zone_geometry,
                             tuning,
                             measured_load_radius,
                             config.controlled_empty_field,
                             load_half_width,
                             load_half_depth);
    ControlledField controlled_field(zone_geometry, measured_body_radius, body_envelope);
    // --hardware: one exclusive RDWR port carries motion out and A6 feedback in.
    // The link is declared after the port, so unwinding stops the wheels before closing it.
    const auto& match_config = resolved.match;
    MatchControl match(match_config);
    MatchServer match_server(match, config.match_socket);
    bool auto_start_requested = false;
    std::unique_ptr<MotionLink> link;
    int demo_hold_angle = -1;
    int16_t demo_hold_pitch = kCameraPitchInvalid;
    if (config.hardware) {
        feedback = std::make_unique<UARTController>();
        if (a6_done_flag)
            feedback->configureDoneFlagFeedback(
                config.known_frame_angle, config.known_frame_id, a6_done_settle_ms);
        else
            feedback->configureFrameFeedback(feedback_open, feedback_close);
        feedback->initUART(config.uart_port, config.baudrate, false, config.auto_run);
        // Continue from the MCU's current action id so a restart is never taken as a repeat.
        if (!feedback->syncGripperActionId(Ms(500)))
            throw std::runtime_error(a6_done_flag ? "A6 id/done flag differs from --known-frame-id "
                                                    "or no feedback within 500 ms; nothing was sent"
                                                  : "No A6 feedback from " + config.uart_port +
                                                        " within 500 ms; nothing was sent");
        {
            const auto mcu = feedback->latestActuatorFeedback();
            std::cout << "[MCU] synced gripper_id=" << int(mcu.gripper_action_id)
                      << " open=" << int(mcu.gripper_open)
                      << "; keeping this state reuses the id, each state change uses id+1\n";
        }
        {
            const auto fb = feedback->latestActuatorFeedback();
            const int open = feedback->confirmedFrameOpen(fb);
            if (open < 0)
                throw std::runtime_error("Frame position unknown at startup; nothing was sent");
            demo_hold_angle = open ? 0 : 20;
            demo_hold_pitch = fb.camera_pitch_cdeg;
            std::cout << "[MCU] startup frame angle=" << demo_hold_angle << " source="
                      << (a6_done_flag ? "operator_known_angle(TEMP_ASSUMPTION)"
                                       : "a6_state_mapping")
                      << "\n";
        }
        // 每包注入IMU当前航向：快照过期或数值无效时返回NaN，编码为线上的kImuYawInvalid。
        link = std::make_unique<MotionLink>(
            *feedback,
            resolved.motion_link,
            [&match] {
                return match.status(imuNowUs()).permit;
            },
            [&imu] {
                if (!imu)
                    return std::numeric_limits<float>::quiet_NaN();
                const auto snap = imu->snapshot();
                const float yaw = snap.sample.body_rpy_rad[2];
                return (snap.connected && snap.fresh && snap.sample.measurements_valid &&
                        std::isfinite(yaw))
                           ? yaw
                           : std::numeric_limits<float>::quiet_NaN();
            });
        std::cout << "[MCU] " << config.uart_port
                  << " output enabled; 25 Hz resend, zero velocity after 150 ms stall; imu yaw in "
                     "every packet\n";
    } else if (config.pitch_feedback) {
        feedback = std::make_unique<UARTController>();
        feedback->initFeedbackOnly(config.uart_port, config.baudrate);
    }
    PerceptionPipeline vision_frontend(config);
    ZoneColorClassifier zone_colors(config.zone_color_file.empty()
                                        ? ZoneColorConfig{}
                                        : ZoneColorConfig::load(config.zone_color_file));
    VisionLogic vision(config);
    NearestNeighborTracker tracker;
    ObservationBuilder observations(resolved,
                                    geometry.get(),
                                    vref_calibration,
                                    vref_ready,
                                    zone_geometry,
                                    zone_colors,
                                    controlled_field,
                                    fusion);
    // Use the USB camera's MJPEG V4L2 path; automatic GStreamer negotiation
    // fails when applying the requested 720p/60 FPS settings on this board.
    cv::VideoCapture camera;
    std::unique_ptr<CameraCapture> camera_capture;
    cv::Size capture_size;
    if (config.async_capture) {
        camera_capture = std::make_unique<CameraCapture>(config.camera_index,
                                                         config.frame_width,
                                                         config.frame_height,
                                                         config.fps,
                                                         config.input_size);
        if (!camera_capture->start())
            throw std::runtime_error("Cannot open camera");
        CameraFrame first;
        if (!camera_capture->waitNext(0, first, 2000))
            throw std::runtime_error("Camera startup timed out");
        capture_size = first.image.size();
        std::cout << "[CAMERA] async latest-frame capture; timestamps=host_read_complete, not "
                     "sensor exposure\n";
    } else {
        camera.open(config.camera_index, cv::CAP_V4L2);
        if (!camera.isOpened())
            throw std::runtime_error("Cannot open camera");
        configureCameraCapture(
            camera, config.camera_index, config.frame_width, config.frame_height, config.fps);
        camera.set(cv::CAP_PROP_BUFFERSIZE,
                   1); // backend may ignore; timestamp remains host dequeue time
        capture_size = {int(camera.get(cv::CAP_PROP_FRAME_WIDTH)),
                        int(camera.get(cv::CAP_PROP_FRAME_HEIGHT))};
    }
    cv::VideoWriter writer;
    if (config.save_output) {
        writer.open("output_cpp.mp4",
                    cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                    config.fps,
                    capture_size);
        if (!writer.isOpened())
            throw std::runtime_error("Cannot open recording");
    }
    std::cout << (link ? "Ground-pushing LIVE: motion follows PushTask; still gated by "
                         "safety/geometry evidence.\n"
                       : "Ground-pushing preview: waiting for validated geometry, subzone and "
                         "safety inputs.\n");
#ifdef RESCUE_ENABLE_TELEMETRY
    std::unique_ptr<TelemetryPublisher> telemetry;
    if (config.telemetry) {
        try {
            std::ostringstream effective_config;
            writeEffectiveConfig(resolved, effective_config);
            telemetry = std::make_unique<TelemetryPublisher>(config, effective_config.str());
            std::cout << "[TELEMETRY] snapshot=" << config.telemetry_file << "\n";
        } catch (const std::exception& e) {
            std::cerr << "[TELEMETRY] disabled: " << e.what() << "\n";
        }
    }
#endif
    bool vision_startup_saved = false, vision_match_saved = false, vision_first_lock_saved = false;
    std::string vision_last_state, vision_last_reason;
    std::map<std::string, uint64_t> vision_reason_last_us;
    int vision_event_count = 0;
    uint64_t frame_sequence = 0, rate_frames = 0;
    auto rate_started = Clock::now();
    double loop_fps = 0;
    PushOutput previous;
    previous.first_ordinary_delivered = tuning.assume_injured_trip;
    auto last_report = Clock::now();
    MotionWatch motion_watch;
    const auto demo_started = Clock::now();
    int demo_exit = 0;
    while (!g_should_exit.load()) {
        if (config.demo_mode != "none" &&
            Clock::now() - demo_started >= std::chrono::seconds(config.demo_seconds)) {
            std::cout << "[DEMO_RESULT] wall_budget_exhausted mode=" << config.demo_mode << "\n";
            demo_exit = config.demo_mode == "recognize" ? 0 : 2;
            break;
        }
        AuditScope iteration_audit("main.iteration");
        const auto capture_started = Clock::now();
        cv::Mat frame;
        ModelImage prepared_image;
        uint64_t timestamp = 0, epoch_ns = 0;
        if (camera_capture) {
            CameraFrame captured;
            if (!camera_capture->waitNext(frame_sequence, captured, config.sensor_timeout_ms))
                throw std::runtime_error("Camera frame lost or capture timed out");
            frame = std::move(captured.image);
            prepared_image = std::move(captured.prepared);
            timestamp = captured.timestamp_us;
            epoch_ns = captured.epoch_ns;
            frame_sequence = captured.frame_number;
        } else {
            if (!camera.read(frame) || frame.empty())
                throw std::runtime_error("Camera frame lost");
            const auto read_at = Clock::now();
            epoch_ns =
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::system_clock::now().time_since_epoch())
                                          .count());
            timestamp = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(read_at.time_since_epoch())
                    .count());
            ++frame_sequence;
        }
        const auto now = Clock::now();
        const double capture_ms =
            std::chrono::duration<double, std::milli>(now - capture_started).count();
        if (camera_capture) {
            const auto now_us = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch())
                    .count());
            if (timestamp > now_us ||
                now_us - timestamp > uint64_t(config.sensor_timeout_ms) * 1000)
                throw std::runtime_error("Camera frame is stale");
        }
        // Detect and pose share this frame's id and capture time.
        auto vision_result =
            vision_frontend.infer(frame, timestamp, camera_capture ? &prepared_image : nullptr);
        AuditScope logic_audit("main.logic_and_logs");
        auto& raw = vision_result.detections;
        for (auto& d : raw) {
            d.frame_id = frame_sequence;
            d.timestamp_us = timestamp;
        }
        auto detections = tracker.update(raw, timestamp);
        const double inference_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - now).count();
        auto observation = observations.build(
            frame,
            vision_result,
            std::move(detections),
            frame_sequence,
            timestamp,
            {task, previous, feedback.get(), imu.get(), motion_watch, bool(link)});
        auto& input = observation.input;
        detections = std::move(observation.detections);
        const auto& zone_points = observation.zone_points;
        const auto& zone_color = observation.zone_color;
        const auto& geometry_result = observation.geometry_result;
        const auto& selection = observation.selection;
        const auto& frame_sensors = observation.frame_sensors;
        const auto& gripper_ack = observation.gripper_ack;
        const auto& imu_data = observation.imu_data;
        const auto& sensor = observation.sensor;
        const auto observed_at = observation.observed_at;
        // A dead link (write failures or stale feedback) vetoes motion permission.
        std::string preflight_reason;
        if (!imu || !input.safety_ok)
            preflight_reason = "imu_not_ready";
        else if (!task_calibrated)
            preflight_reason = "gripper_calibration_required";
        else if (!loadFitsHalf(measured_load_radius,
                               zone_geometry.width_m,
                               zone_geometry.depth_m,
                               zone_geometry.divider_exclusion_half_width_m,
                               load_half_width,
                               load_half_depth))
            preflight_reason = "load_footprint_does_not_fit";
        else if (config.zone_color_file.empty())
            preflight_reason = "zone_color_calibration_required";
        else if (config.controlled_empty_field && measured_body_radius <= 0)
            preflight_reason = "robot_swept_radius_required";
        else if (frame.cols != config.frame_width || frame.rows != config.frame_height)
            preflight_reason = "camera_size_mismatch";
        else if (!geometry)
            preflight_reason = "ground_mapping_not_ready";
        else if (config.pose_model_path.empty() && config.keypoints_file.empty())
            preflight_reason = "zone_pose_source_required";
        else if (link && !feedback->latestActuatorFeedback().valid)
            preflight_reason = "mcu_feedback_stale";
        else if (link && link->stats().sent && !link->stats().last_ok)
            preflight_reason = "mcu_write_failed";
        else if (!frame_sensors.actuator.valid || input.camera_pitch_cdeg == kCameraPitchInvalid)
            preflight_reason = "camera_pitch_feedback_invalid";
        // 2026-10-08用户要求：开机静止位RX40（旧0°）也可启动，启动后由任务自行转到旧5°(RX45)再搜索。
        else if (!match.status(imuNowUs()).started_us &&
                 (!frame_sensors.pitch_stable ||
                  (std::abs(int(input.camera_pitch_cdeg) - int(tuning.far_pitch_cdeg)) >
                       tuning.pitch_tolerance_cdeg &&
                   std::abs(int(input.camera_pitch_cdeg) - int(kStartupRestPitchCdeg)) >
                       tuning.pitch_tolerance_cdeg)))
            preflight_reason = "startup_pitch_not_rest_or_5deg";
        // 现场同步（用户要求）：起步预检不再要求ground mapping有效和起步通道/对方区域净空。
        input.now_us = imuNowUs();
        match.health(input.now_us, preflight_reason.empty(), preflight_reason);
        // Only independently identified, observed (not predicted) fixed-zone poses
        // can supply measured translation. Loss of the reference does not count as movement.
        if (input.zone_identity_verified && input.zone_estimate.trusted(input.now_us) &&
            input.zone_estimate.source != ZoneEstimate::Source::PREDICTED) {
            const auto position = input.zone_estimate.bodyToZone({0, 0});
            const double uncertainty = 2 * input.zone_estimate.position_sigma_m +
                                       2 * cv::norm(position) * input.zone_estimate.yaw_sigma_rad;
            match.measuredPosition(input.zone_estimate.timestamp_us,
                                   position.x,
                                   position.y,
                                   input.zone_estimate.geometry_id,
                                   uncertainty);
        }
        if (config.auto_run && !auto_start_requested && preflight_reason.empty()) {
            auto_start_requested = true;
            match.command(MatchCommand::START, input.now_us);
        }
        const auto match_status = match.status(input.now_us);
        if (config.vision_log_dir != "none") {
            // frame 8 lets auto-exposure settle; the second record is the first frame in which the
            // match is permitted
            const bool warm = !vision_startup_saved && frame_sequence >= 8;
            const bool started = !vision_match_saved && match_status.permit;
            if (warm || started) {
                saveVisionJudge(config.vision_log_dir,
                                warm ? "startup" : "match_start",
                                frame_sequence,
                                frame,
                                detections);
                if (warm)
                    vision_startup_saved = true;
                if (started)
                    vision_match_saved = true;
            }
        }
        input.run = match_status.permit;
        input.safety_ok = match_status.permit && preflight_reason.empty();
        navigator.update(input, previous);
        auto out = task.update(input);
        // Remember where the locked target was last seen, so an event can show which detection
        // replaced it.
        static int last_out_id = -1;
        static std::string last_out_label;
        static cv::Point2f last_out_centre;
        if (out.target_id >= 0) {
            last_out_id = out.target_id;
            for (const auto& d : detections)
                if (d.track_id == out.target_id) {
                    last_out_label = d.label;
                    last_out_centre = d.center();
                    break;
                }
        }
        if (config.vision_log_dir != "none" && vision_event_count < 100) {
            const std::string st = PushTask::name(out.state);
            const bool state_changed = st != vision_last_state;
            const bool reason_changed = out.reason != vision_last_reason;
            static const std::set<std::string> key_states = {"PREPARE",
                                                             "RUSH",
                                                             "LOWER_FRAME",
                                                             "VERIFY_CAPTURE",
                                                             "CARRY",
                                                             "RAISE_RELEASE",
                                                             "VERIFY_DELIVERY",
                                                             "CAPTURE_FAIL",
                                                             "LOST_HOLD",
                                                             "ABORT_DROP",
                                                             "SAFE_STOP"};
            static const std::set<std::string> key_reasons = {
                "target_id_mismatch", "target_missing", "lost_target_search", "sweep_upright_out"};
            const bool first_lock =
                !vision_first_lock_saved && st == "APPROACH" && out.target_id >= 0;
            bool key_reason = false;
            if (reason_changed && key_reasons.count(out.reason)) {
                const auto it = vision_reason_last_us.find(out.reason);
                key_reason =
                    it == vision_reason_last_us.end() || input.now_us - it->second > 3000000;
            }
            if ((state_changed && key_states.count(st)) || first_lock || key_reason) {
                std::string tag = "ev-" + st + "-" + out.reason;
                for (auto& ch : tag)
                    if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '-')
                        ch = '_';
                if (tag.size() > 48)
                    tag.resize(48);
                std::ostringstream cap;
                cap << std::fixed << std::setprecision(1) << match_status.remaining_us / 1e6 << "s "
                    << st << " " << out.reason << " | in#" << input.target_id << " "
                    << (input.label.empty() ? "-" : input.label) << " d=" << std::setprecision(2)
                    << input.distance_m << "m h=" << std::setprecision(1)
                    << input.heading_error * 57.2958f << "deg | out#" << out.target_id << " cap#"
                    << out.capture_target_id;
                bool out_present = out.target_id < 0;
                for (const auto& d : detections)
                    if (d.track_id == out.target_id)
                        out_present = true;
                int near_id = -1;
                if (!out_present && out.target_id >= 0 && out.target_id == last_out_id) {
                    float best = 1e9f;
                    for (const auto& d : detections) {
                        if (d.label != last_out_label)
                            continue;
                        const float dist = float(cv::norm(d.center() - last_out_centre));
                        if (dist < best) {
                            best = dist;
                            near_id = d.track_id;
                        }
                    }
                }
                saveVisionJudge(config.vision_log_dir,
                                tag.c_str(),
                                frame_sequence,
                                frame,
                                detections,
                                cap.str(),
                                out.target_id,
                                input.target_id,
                                near_id,
                                !out_present);
                std::cout << "[VISION_EVENT] state=" << st << " reason=" << out.reason
                          << " remaining_s=" << match_status.remaining_us / 1e6
                          << " input_target=" << input.target_id
                          << " label=" << (input.label.empty() ? "-" : input.label)
                          << " dist_m=" << input.distance_m
                          << " heading_err_rad=" << input.heading_error
                          << " out_target=" << out.target_id
                          << " capture_target=" << out.capture_target_id
                          << " pixel_id_locked=" << input.pixel_locked
                          << " frame=" << frame_sequence << "\n";
                ++vision_event_count;
                if (first_lock)
                    vision_first_lock_saved = true;
                if (key_reason)
                    vision_reason_last_us[out.reason] = input.now_us;
            }
            vision_last_state = st;
            vision_last_reason = out.reason;
        }
        std::cout << "[ANCHOR] state=" << int(out.state) << " reason=" << out.reason << " "
                  << task.anchorDebug() << "\n";
        {
            const std::string dz = task.takeDropZoneEvent();
            if (!dz.empty())
                std::cout << dz << std::flush;
        }
        guardDemoMotion(config.demo_mode, out.motion, demo_hold_angle, demo_hold_pitch);
        if (imu) {
            motion_watch.feed(input.now_us,
                              PushTask::name(out.state),
                              out.motion.vx_mps,
                              out.motion.wz_rps,
                              bool(link) && match_status.permit,
                              imu_data.fresh && imu_data.sample.measurements_valid,
                              imu_data.sample.body_acceleration_mps2,
                              imu_data.sample.body_angular_velocity_rps,
                              imu_data.sample.body_rpy_rad);
            const std::string mev = motion_watch.takeEvents();
            if (!mev.empty())
                std::cout << mev << std::flush;
        }
        if (config.demo_mode == "search" && config.dry_run)
            std::cout << "[DEMO_SEARCH_PREVIEW] target=" << selection.id
                      << " locked=" << selection.locked << " pixel_mode=" << selection.mode
                      << " ratio=" << selection.smoothed << " geometry=" << input.geometry_valid
                      << " NO_TX\n";
        // 现场同步（用户要求）：去除最终出口门inhibitUnmappedMotion。
        out.match_state = MatchControl::name(match_status.state);
        out.match_reason = match_status.reason;
        out.match_remaining_us = match_status.remaining_us;
        out.hardware_output_enabled = bool(link) && match_status.permit;
        previous = out;
        const auto submit_time = Clock::now();
        const auto submit_gap_ms =
            std::chrono::duration<double, std::milli>(submit_time - last_submit_time).count();
        last_submit_time = submit_time;
        if (link) {
            const auto stale_count = link->stats().stale;
            link->submit(out.motion);
            if (stale_count != last_stale_count) {
                async_log.submit(
                    "[WATCHDOG] timestamp_us=" + std::to_string(input.now_us) +
                    " stale_zero_delta=" + std::to_string(stale_count - last_stale_count) +
                    " submit_gap_ms=" + std::to_string(submit_gap_ms) + "\n");
                last_stale_count = stale_count;
            }
        }
        // Record every changed stop condition, including one-frame stops.
        // task_reason is historical state-machine context; evidence is current.
        const bool stopped = out.motion.vx_mps == 0 && out.motion.wz_rps == 0;
        std::ostringstream stop_detail;
        stop_detail << "stopped=" << stopped << " state=" << PushTask::name(out.state)
                    << " task_reason=" << out.reason << " vx=" << out.motion.vx_mps
                    << " wz=" << out.motion.wz_rps << " pitch_cmd=" << out.motion.camera_pitch_cdeg
                    << " frame_offset=" << int(out.motion.gripper_offset)
                    << " preflight=" << preflight_reason << " match=" << match_status.reason
                    << " locked_id=" << out.target_id << " observed_id=" << input.target_id
                    << " target_valid=" << input.target_valid
                    << " id_match=" << (out.target_id >= 0 && out.target_id == input.target_id)
                    << " geometry=" << input.geometry_valid
                    << " mapping=" << geometry_result.mapping_valid
                    << " geometry_reason=" << geometry_result.reason
                    << " region=" << input.target_region_valid
                    << " in_zone=" << input.target_in_zone << " path=" << input.path_safe
                    << " retreat=" << input.retreat_safe << " turn_safe=" << input.turn_safe
                    << " arc=" << input.arc_safe << " jaw_open=" << input.jaw_open_safe
                    << " dir_clear_valid=" << input.directional_clearance_valid << " blockers=["
                    << controlled_field.blockers()
                    << "] opponent_clear=" << input.opponent_zone_clear
                    << " pitch_stable=" << input.camera_pitch_stable
                    << " gripper_done=" << input.gripper_done
                    << " corridor_complete=" << input.corridor_complete;
        if (feedback) {
            const auto fb = feedback->latestActuatorFeedback();
            stop_detail << " command_id=" << int(feedback->gripperActionId())
                        << " feedback_id=" << int(fb.gripper_action_id)
                        << " feedback_valid=" << fb.valid;
        }
        // A moving-to-moving reason or command change is a control transition too.
        const auto signature = stop_detail.str();
        if (signature != last_stop_signature) {
            async_log.submit("[CONTROL_CHANGE] timestamp_us=" + std::to_string(input.now_us) + " " +
                             stop_detail.str() + "\n");
            last_stop_signature = signature;
        }

        ++rate_frames;
        const auto rate_now = Clock::now();
        const double rate_seconds = std::chrono::duration<double>(rate_now - rate_started).count();
        if (rate_seconds >= 1.0) {
            loop_fps = rate_frames / rate_seconds;
            rate_frames = 0;
            rate_started = rate_now;
        }
        if (now - last_report >= Ms(500)) {
            std::ostringstream log;
            if (config.controlled_empty_field)
                log << (config.assume_all_safe ? "[TEMP_ASSUMPTION] assume_all_safe=1 " : "")
                    << "[CONTROLLED_TEST] clearance="
                    << (config.controlled_ignore_clearance ? "DISABLED_CONTACT_TEST"
                                                           : "operator_confirmed")
                    << " identity=own_zone_only "
                    << "inventory=" << controlled_field.reason()
                    << " translation_watchdog=disabled phase_budgets=enabled\n";
            if (config.controlled_empty_field)
                log << "[SAFETY] path=" << input.path_safe << " retreat=" << input.retreat_safe
                    << " turn=" << input.turn_safe << " arc=" << input.arc_safe
                    << " jaw=" << input.jaw_open_safe
                    << " dir_valid=" << input.directional_clearance_valid
                    << " state=" << PushTask::name(out.state) << " blockers=["
                    << controlled_field.blockers() << "]\n";
            for (const auto& d : detections)
                log << "[TARGET] id=" << d.track_id << " label=" << d.label
                    << " position=" << d.ground_position_valid
                    << " contact=" << d.ground_contact_valid
                    << " reason=" << d.ground_contact_reason << " xy=" << d.body_xy_m.x << ","
                    << d.body_xy_m.y << " conf=" << d.confidence << " box=" << d.box.x << ","
                    << d.box.y << "," << d.box.width << "," << d.box.height
                    << (d.pose_err_lying >= 0
                            ? " pose_err_lie=" + std::to_string(d.pose_err_lying) +
                                  " pose_err_up=" + std::to_string(d.pose_err_upright) +
                                  " pose_diff_med=" + std::to_string(d.pose_diff_med) +
                                  " votes=" + std::to_string(d.pose_vote_n) +
                                  " upright_rej=" + std::to_string(d.injured_upright_rejected) +
                                  " sil_hw=" + std::to_string(d.silhouette_hw)
                            : std::string())
                    << "\n";
            {
                const auto ids = [&](const char* key, const std::vector<int>& v) {
                    log << " " << key << "=";
                    if (v.empty())
                        log << "-";
                    for (size_t i = 0; i < v.size(); ++i)
                        log << (i ? "," : "") << v[i];
                };
                const auto& hd = observations.holdDiagnostics();
                log << "[HOLD] observable=" << input.hold_observable
                    << " captured=" << input.captured << " complete=" << input.held_complete
                    << " held=o" << input.held.ordinary << "/c" << input.held.core << "/i"
                    << input.held.injured << "/d" << input.held.dangerous << "/u"
                    << input.held.unknown;
                ids("ambiguous", hd.ambiguous);
                ids("outside", hd.outside);
                ids("unfollowed", hd.unfollowed);
                log << "\n";
            }
            report(out, log);
            const bool range_valid = input.target_valid && input.geometry_valid;
            const float target_y = range_valid ? input.distance_m * std::cos(input.heading_error)
                                               : std::numeric_limits<float>::quiet_NaN();
            log << "[GRASP_GEOMETRY] target_valid=" << range_valid << " body_range_m="
                << (range_valid ? input.distance_m : std::numeric_limits<float>::quiet_NaN())
                << " body_forward_m=" << target_y << " front_y_m=" << tuning.closed_front_y_m
                << " front_gap_m=" << target_y - tuning.closed_front_y_m
                << " open_prepare_range_m=" << tuning.rush_start_m
                << " close_trigger_y_m=" << tuning.grasp_trigger_y_m
                << " open_distance_ok=" << (range_valid && input.distance_m <= tuning.rush_start_m)
                << " open_heading_ok="
                << (range_valid && std::abs(input.heading_error) <= tuning.rush_heading_rad)
                << " path=" << input.path_safe
                << " gripper_command=" << int(out.motion.gripper_offset) << "\n";
            log << "[MATCH] state=" << MatchControl::name(match_status.state)
                << " reason=" << match_status.reason
                << " remaining_ms=" << match_status.remaining_us / 1000
                << " preflight=" << preflight_reason << "\n";
            log << "[EVIDENCE] identity=" << input.zone_identity_verified
                << " target_region=" << input.target_region_valid << " color=" << zone_color.color
                << " color_reason=" << zone_color.reason << " path=" << input.path_safe
                << " opponent_clear=" << input.opponent_zone_clear
                << " retreat=" << input.retreat_safe << " zone_counts=" << input.zone_counts_valid
                << "\n";
            if (geometry)
                log << "[GEOMETRY] mapped=" << geometry_result.mapping_valid
                    << " zone_valid=" << input.zone_estimate.trusted(input.now_us)
                    << " reason=" << geometry_result.reason
                    << " inliers=" << input.zone_estimate.inlier_ids.size()
                    << " residual_m=" << input.zone_estimate.residual_m << "\n";
            log << "[PERF] loop_fps=" << loop_fps << " inference_ms=" << inference_ms
                << " detect_ms=" << vision_result.detect_ms << " pose_ms=" << vision_result.pose_ms
                << " zone_points=" << zone_points.size() << " capture_ms=" << capture_ms
                << " detections=" << detections.size() << "\n";
            if (vision_result.pose_ran) {
                static const char* kn[] = {"fl", "nl", "fr", "nr"};
                log << "[ZONEPTS]";
                for (int c = 0; c < 2; ++c) {
                    const auto& h = vision_result.halves[c];
                    log << (c ? " R" : " L") << "=" << h.valid;
                    if (h.valid)
                        for (int k = 0; k < 4; ++k)
                            log << " " << kn[k] << "=" << int(h.keypoints[k].x) << ","
                                << int(h.keypoints[k].y) << "@" << int(h.confidence[k] * 100);
                }
                log << "\n";
            }
            if (imu)
                log << motion_watch.line();
            if (imu) {
                const auto& p = imu_data.sample;
                log << "[IMU] fresh=" << imu_data.fresh << " age_ms=" << imu_data.age_ms
                    << " numeric_ok=" << p.measurements_valid << " seq=" << p.sequence
                    << " body_rpy_rad=" << p.body_rpy_rad[0] << "," << p.body_rpy_rad[1] << ","
                    << p.body_rpy_rad[2] << " status=" << p.status
                    << " crc_errors=" << imu_data.crc_errors << " sensor_valid=" << sensor.imu_valid
                    << " estop=" << fusion.emergencyStop(observed_at) << "\n";
            }
            if (link) {
                static const char* ack_names[] = {"idle", "no_feedback", "not_done", "done"};
                const auto st = link->stats();
                const auto fb = feedback->latestActuatorFeedback();
                log << "[MCU] healthy=" << link->healthy() << " sent=" << st.sent
                    << " failed=" << st.failed << " stale_zero=" << st.stale
                    << " gripper_id=" << int(feedback->gripperActionId())
                    << " target=" << gripper_ack.target
                    << " ack=" << ack_names[int(gripper_ack.result)] << " fb_valid=" << fb.valid
                    << " fb_id=" << int(fb.gripper_action_id) << " fb_open=" << int(fb.gripper_open)
                    << " pitch_rb=" << fb.camera_pitch_cdeg << " camera_protocol_readback_deg="
                    << (fb.camera_pitch_cdeg == kCameraPitchInvalid
                            ? kCameraPitchInvalid
                            : cameraPitchToFeedbackDeg(fb.camera_pitch_cdeg))
                    << "\n";
            }
            log << "[LOG] dropped=" << async_log.dropped() << "\n";
            async_log.submit(log.str());
            last_report = now;
        }
        vision.drawDetections(frame, detections);
        drawZone(frame, zone_points);
        const char* output_label = !link ? "PREVIEW - NO MCU LINK"
                                   : out.hardware_output_enabled
                                       ? "MCU LINK OPEN - MOTION PERMITTED"
                                       : "MCU LINK OPEN - MOTION BLOCKED";
        const cv::Scalar output_color = !link                         ? cv::Scalar(0, 255, 255)
                                        : out.hardware_output_enabled ? cv::Scalar(0, 0, 255)
                                                                      : cv::Scalar(0, 190, 255);
        cv::putText(frame, output_label, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.6, output_color, 2);
#ifdef RESCUE_ENABLE_TELEMETRY
        if (telemetry) {
            DecisionTelemetry decision;
            decision.preflight_reason = preflight_reason;
            decision.zone_color = zone_color.color;
            decision.zone_color_reason = zone_color.reason;
            decision.geometry_reason = geometry_result.reason;
            decision.pose_model_ran = vision_result.pose_ran;
            telemetry->submit(frame,
                              detections,
                              input,
                              out,
                              epoch_ns,
                              frame_sequence,
                              loop_fps,
                              inference_ms,
                              capture_ms,
                              imu_data,
                              decision);
        }
#endif
        if (config.demo_mode != "none" &&
            (out.state == PushState::SAFE_STOP || match_status.state == MatchState::FAULT ||
             match_status.state == MatchState::FINISHED)) {
            const bool success = out.reason == "demo_target_found" ||
                                 out.reason == "demo_delivery_complete" ||
                                 (config.assume_all_safe && out.reason == "demo_next_trip_ready");
            std::cout << "[DEMO_RESULT] " << out.reason << " match=" << match_status.reason << "\n";
            demo_exit = success ? 0 : 2;
            match.command(MatchCommand::FINISH, imuNowUs());
            break;
        }
        if (writer.isOpened())
            writer.write(frame);
        if (config.show) {
            cv::imshow("Ground pushing", frame);
            const int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q')
                break;
        }
    }
    if (link) {
        link->stop();
        std::cout << "[MCU] stopped: final zero-velocity packet sent\n";
    }
    return demo_exit;
}

int runApplication(int argc, char** argv) {
    auto loaded = loadAppConfig(argc, argv);
    auto& diagnostics = loaded.options.dump_config_file == "-" ? std::cerr : std::cout;
    const auto& config = loaded.config;
    diagnostics << "MATCH_TIME_LIMIT="
                << (config.no_match_time_limit ? "disabled_debug" : "enabled") << "\n";
    if (config.allow_mechanical_pitch_model)
        diagnostics << "GEOMETRY_SOURCE=mechanical_assumption NOT_FIELD_VALIDATED\n";
    auto resolved = resolveLegacyRuntimeConfig(config, diagnostics);
    if (!loaded.options.behavior_policy.empty() &&
        loaded.options.behavior_policy != resolved.policy_name)
        throw std::runtime_error(
            "Configured behavior_policy does not match the selected operation");
    for (const auto& source : loaded.options.file_field_sources)
        resolved.field_sources["request." + source.first].push_back(source.second);
    for (const auto& field : loaded.options.explicit_cli_fields)
        resolved.field_sources["request." + field].push_back("explicit_cli");
    for (const auto& source : loaded.options.normalized_field_sources)
        resolved.field_sources["request." + source.first].push_back(source.second);
    auto& identity = resolved.identity;
    identity.config_file = loaded.options.config_file;
    std::error_code cwd_error;
    const auto working_directory = std::filesystem::current_path(cwd_error);
    if (cwd_error)
        identity.audit_error = "working_directory: " + cwd_error.message();
    else
        identity.working_directory = working_directory.string();
    std::error_code executable_error;
    identity.executable =
        std::filesystem::read_symlink("/proc/self/exe", executable_error).string();
    if (executable_error) {
        const std::filesystem::path executable = argv[0];
        identity.executable =
            (executable.is_absolute() || cwd_error ? executable : working_directory / executable)
                .lexically_normal()
                .string();
        identity.audit_error += (identity.audit_error.empty() ? "" : "; ") +
                                std::string("executable: ") + executable_error.message();
    }
    for (int i = 0; i < argc; ++i)
        identity.arguments.emplace_back(argv[i]);
    const auto resource = [&](const char* name, const std::string& path) {
        if (path.empty())
            return;
        const std::filesystem::path reference = path;
        if (!reference.is_absolute() && cwd_error)
            return;
        identity.resource_paths[name] =
            (reference.is_absolute() ? reference : working_directory / reference)
                .lexically_normal()
                .string();
    };
    resource("model_path", config.model_path);
    resource("pose_model_path", config.pose_model_path);
    identity.rknn_library_lookup = config.rknn_library.find('/') == std::string::npos
                                       ? "dynamic_linker_search"
                                       : "explicit_path";
    if (identity.rknn_library_lookup == "explicit_path")
        resource("rknn_library", config.rknn_library);
    resource("calibration_file", config.calibration_file);
    resource("task_calibration_file", config.task_calibration_file);
    resource("zone_geometry_file", config.zone_geometry_file);
    resource("zone_color_file", config.zone_color_file);
    resource("keypoints_file", config.keypoints_file);
    resource("push_replay", config.push_replay);
    resource("geometry_replay", config.geometry_replay);
    resource("detect_image", config.detect_image);
#if RESCUE_HAS_RKNN
    identity.rknn_compiled = true;
#endif
#ifdef RESCUE_ENABLE_TELEMETRY
    identity.telemetry_compiled = true;
#endif
    const auto frozen = std::move(resolved);
    if (!frozen.identity.config_file.empty())
        diagnostics << "[RUNTIME_CONFIG] source=" << frozen.identity.config_file
                    << " policy=" << frozen.policy_name << "\n";
    if (!loaded.options.dump_config_file.empty()) {
        if (loaded.options.dump_config_file == "-") {
            writeEffectiveConfig(frozen, std::cout);
        } else {
            std::ofstream output(loaded.options.dump_config_file);
            if (!output)
                throw std::runtime_error("Cannot open effective configuration output");
            writeEffectiveConfig(frozen, output);
            if (!output)
                throw std::runtime_error("Cannot write effective configuration output");
        }
        return config.check_config ? checkConfiguration(frozen, diagnostics) : 0;
    }
    if (config.check_config)
        return checkConfiguration(frozen, diagnostics);
    printRuntimeTuning(frozen, diagnostics);
#ifndef RESCUE_ENABLE_TELEMETRY
    if (config.telemetry)
        throw std::runtime_error("Rebuild with -DRESCUE_ENABLE_TELEMETRY=ON to enable telemetry");
#endif
    if (const auto result = runOfflineTask(config, frozen.task))
        return *result;
    return Runtime(frozen).run();
}
} // namespace rescue::app
