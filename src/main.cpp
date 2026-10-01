#include "rescue/config.hpp"
#include "rescue/geometry_pipeline.hpp"
#include "rescue/uart_controller.hpp"
#include "rescue/motion_link.hpp"
#include "rescue/match_control.hpp"
#include "rescue/match_server.hpp"
#include "rescue/hipnuc_imu.hpp"
#include "rescue/imu_adapter.hpp"
#include "rescue/sensor_fusion.hpp"
#include "rescue/detector.hpp"
#include "rescue/zone_keypoints.hpp"
#include "rescue/zone_color.hpp"
#include "rescue/push_task.hpp"
#include "rescue/capture_monitor.hpp"
#include "rescue/task_calibration.hpp"
#include "rescue/navigation_adapter.hpp"
#include "rescue/perception_adapter.hpp"
#include "rescue/tracker.hpp"
#include "rescue/utils.hpp"
#include "rescue/vision_logic.hpp"
#include <opencv2/opencv.hpp>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <memory>
#include <future>
#ifdef RESCUE_ENABLE_TELEMETRY
#include "rescue/telemetry_publisher.hpp"
#endif

namespace {
rescue::PushObservation readObservation(const cv::FileNode &n) {
    rescue::PushObservation in;
    double timestamp = 0;
    n["now_us"] >> timestamp;
    if (!std::isfinite(timestamp) || timestamp <= 0 || timestamp > 9007199254740991.0)
        throw std::runtime_error("Replay now_us must be a positive integer <= 2^53-1");
    if (std::floor(timestamp) != timestamp) throw std::runtime_error("Fractional now_us");
    in.now_us = static_cast<uint64_t>(timestamp);
    auto flag = [&](const char *key, bool &value) {
        if (!n[key].empty()) value = static_cast<int>(n[key]) != 0;
    };
    flag("run", in.run); flag("reset", in.reset); flag("safety_ok", in.safety_ok);
    flag("target_valid", in.target_valid); flag("geometry_valid", in.geometry_valid);
    flag("path_safe", in.path_safe); flag("retreat_safe", in.retreat_safe);
    flag("target_region_valid", in.target_region_valid);
    flag("zone_identity_verified", in.zone_identity_verified);
    flag("opponent_zone_clear", in.opponent_zone_clear); flag("target_in_zone", in.target_in_zone);
    flag("corridor_complete", in.corridor_complete); flag("corridor_occlusion_free", in.corridor_occlusion_free);
    flag("hold_observable", in.hold_observable);
    flag("captured", in.captured); flag("held_complete", in.held_complete);
    flag("camera_pitch_stable", in.camera_pitch_stable);
    flag("gripper_done", in.gripper_done);
    flag("zone_valid", in.zone_valid); flag("zone_own", in.zone_own);
    flag("carry_plan_valid",in.carry_plan_valid);flag("drop_plan_valid",in.drop_plan_valid);
    if(!n["navigation_timestamp_us"].empty())in.navigation_timestamp_us=rescue::readExactTime(n["navigation_timestamp_us"]);
    const auto point=[&](const char* key,cv::Point2f& p){const auto v=n[key];if(!v.empty()) {
        if(!v.isSeq()||v.size()!=2)throw std::runtime_error("Invalid replay navigation point");p={float(v[0]),float(v[1])};}};
    point("carry_waypoint_body",in.carry_waypoint_body);point("drop_centre_zone",in.drop_centre_zone);
    flag("zone_counts_valid", in.zone_counts_valid); flag("heading_valid", in.heading_valid);
    auto integer = [&](const char *key, int &value) { if (!n[key].empty()) n[key] >> value; };
    integer("target_id", in.target_id); integer("gripper_feedback_open", in.gripper_feedback_open);
    integer("zone_supply_count", in.zone_supply_count); integer("zone_injured_count", in.zone_injured_count);
    if (!n["camera_pitch_cdeg"].empty()) {
        int pitch = 0; n["camera_pitch_cdeg"] >> pitch;
        if (pitch < -32768 || pitch > 32767) throw std::runtime_error("Replay camera_pitch_cdeg out of int16 range");
        in.camera_pitch_cdeg = static_cast<int16_t>(pitch);
    }
    auto inventory = [&](const char *key, rescue::Inventory &inv) {
        const auto node = n[key];
        if (node.empty()) return;
        if (!node.isMap()) throw std::runtime_error(std::string("Replay ") + key + " must be a map");
        const auto count = [&](const char *name, int &value) {
            if (!node[name].empty()) node[name] >> value;
            if (value < 0) throw std::runtime_error("Negative replay inventory count");
        };
        count("ordinary", inv.ordinary); count("core", inv.core); count("injured", inv.injured);
        count("dangerous", inv.dangerous); count("unknown", inv.unknown);
    };
    inventory("corridor", in.corridor); inventory("held", in.held);
    n["label"] >> in.label; n["zone_class"] >> in.zone_class;
    n["distance_m"] >> in.distance_m; n["heading_error"] >> in.heading_error;
    if (!n["heading_rad"].empty()) n["heading_rad"] >> in.heading_rad;
    const auto zone=n["zone_estimate"];
    if (!zone.empty()) {
        auto& z=in.zone_estimate;
        z.valid=int(zone["valid"])==1;z.zone_label=std::string(zone["zone_label"]);
        z.geometry_id=std::string(zone["geometry_id"]);
        z.frame_id=rescue::readExactTime(zone["frame_id"]);
        z.timestamp_us=rescue::readExactTime(zone["timestamp_us"]);
        z.observed_us=rescue::readExactTime(zone["observed_us"]);
        const auto source=std::string(zone["source"]);
        z.source=source=="multi_point"?rescue::ZoneEstimate::Source::MULTI_POINT:
            source=="two_point"?rescue::ZoneEstimate::Source::TWO_POINT:
            source=="predicted"?rescue::ZoneEstimate::Source::PREDICTED:rescue::ZoneEstimate::Source::NONE;
        if(!zone["origin_body_m"].isSeq()||zone["origin_body_m"].size()!=2||
           zone["residual_m"].empty()||zone["position_sigma_m"].empty()||zone["yaw_sigma_rad"].empty()||
           zone["yaw_body_rad"].empty()||zone["predicted_distance_m"].empty()||!zone["inlier_ids"].isSeq())
            throw std::runtime_error("Incomplete replay ZoneEstimate");
        z.origin_body_m={float(zone["origin_body_m"][0]),float(zone["origin_body_m"][1])};
        z.yaw_body_rad=float(zone["yaw_body_rad"]);z.residual_m=float(zone["residual_m"]);
        z.position_sigma_m=float(zone["position_sigma_m"]);z.yaw_sigma_rad=float(zone["yaw_sigma_rad"]);
        z.predicted_distance_m=float(zone["predicted_distance_m"]);
        for(const auto& id:zone["inlier_ids"])z.inlier_ids.push_back(int(id));
    }
    return in;
}
void report(const rescue::PushOutput &out) {
    std::cout << "state=" << rescue::PushTask::name(out.state) << " batch=" << out.batch_size
              << " delivered=" << out.delivered_total << " first=" << out.first_ordinary_delivered
              << " vx=" << out.motion.vx_mps << " wz=" << out.motion.wz_rps
              << " gripper_open=" << int(out.motion.gripper_open)
              << " pitch_cmd=" << out.motion.camera_pitch_cdeg << " stop="
              << (out.motion.vx_mps == 0.0f && out.motion.wz_rps == 0.0f)
              << " reason=" << (out.reason.empty() ? "-" : out.reason) << "\n";
}
// Detector plus optional safe-zone pose model on one shared 640 letterbox. Each
// RKNN context is only ever driven by one caller at a time; with --parallel-infer
// the pose context runs on an async worker while the detector runs here.
struct VisionResult {
    std::vector<rescue::SegDetection> detections;
    rescue::ZoneHalves halves;
    bool pose_ran = false;
    double detect_ms = 0, pose_ms = 0, total_ms = 0;
};
class VisionFrontend {
public:
    explicit VisionFrontend(const rescue::Config &config)
        : config_(config), detector_(rescue::makeDetector(config)) {
        rknn_ = dynamic_cast<rescue::YoloRknnDetector *>(detector_.get());
        if (!config.pose_model_path.empty()) {
            if (!rknn_) throw std::runtime_error("--pose-model needs an .rknn detector (shared letterbox)");
            pose_ = std::make_unique<rescue::SafeZoneKeypointRknn>(config);
        }
    }
    bool hasPose() const { return pose_ != nullptr; }
    VisionResult infer(const cv::Mat &frame, uint64_t timestamp_us) {
        using namespace rescue;
        const auto ms = [](Clock::time_point a, Clock::time_point b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        VisionResult r;
        const auto started = Clock::now();
        if (!rknn_) {
            r.detections = detector_->infer(frame);
            r.detect_ms = r.total_ms = ms(started, Clock::now());
            return r;
        }
        const auto image = prepareModelImage(frame, config_.input_size);
        const auto runPose = [&] {
            const auto t = Clock::now();
            auto halves = pose_->infer(image, frame.size());
            return std::make_pair(halves, ms(t, Clock::now()));
        };
        std::future<std::pair<ZoneHalves, double>> pending;
        if (pose_ && config_.parallel_inference) pending = std::async(std::launch::async, runPose);
        const auto detect_started = Clock::now();
        r.detections = rknn_->infer(image, frame.size(), timestamp_us);
        r.detect_ms = ms(detect_started, Clock::now());
        if (pose_) {
            auto pose = pending.valid() ? pending.get() : runPose();
            r.halves = pose.first; r.pose_ms = pose.second; r.pose_ran = true;
        }
        r.total_ms = ms(started, Clock::now());
        return r;
    }
private:
    rescue::Config config_;
    std::unique_ptr<rescue::IDetector> detector_;
    rescue::YoloRknnDetector *rknn_ = nullptr;
    std::unique_ptr<rescue::SafeZoneKeypointRknn> pose_;
};
void drawZone(cv::Mat &frame, const std::vector<rescue::ZoneKeypoint> &points) {
    static const char *names[] = {"FL", "FD", "FR", "RL", "RD", "RR"};
    for (const auto &k : points) {
        if (k.id < 0 || k.id > 5) continue;
        cv::circle(frame, k.pixel, 5, {255, 0, 255}, -1);
        cv::putText(frame, names[k.id], k.pixel + cv::Point2f(6, -6), cv::FONT_HERSHEY_SIMPLEX, .45, {255, 0, 255}, 1);
    }
}
void printZone(const rescue::ZoneHalves &halves, const std::vector<rescue::ZoneKeypoint> &points) {
    static const char *half_names[] = {"zone_left", "zone_right"};
    static const char *kpt_names[] = {"far_left", "near_left", "far_right", "near_right"};
    for (int c = 0; c < 2; ++c) {
        const auto &h = halves[c];
        std::cout << "pose " << half_names[c] << " valid=" << h.valid;
        if (h.valid) {
            std::cout << " score=" << h.score << " box=" << h.box.x << "," << h.box.y << "," << h.box.width << "," << h.box.height;
            for (int k = 0; k < 4; ++k)
                std::cout << " " << kpt_names[k] << "=" << h.keypoints[k].x << "," << h.keypoints[k].y << "@" << h.confidence[k];
        }
        std::cout << "\n";
    }
    std::cout << "zone_keypoints=" << points.size();
    for (const auto &k : points) std::cout << " id" << k.id << "=" << k.pixel.x << "," << k.pixel.y << "@" << k.confidence;
    std::cout << "\n";
}
}
int main(int argc, char **argv) {
    using namespace rescue;
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    try {
        const Config config = parseArgs(argc, argv);
        TaskTuning tuning;
        tuning.far_pitch_cdeg = config.pitch_presets_cdeg[0];
        tuning.track_pitch_cdeg = config.pitch_presets_cdeg[1];
        tuning.near_pitch_cdeg = config.pitch_presets_cdeg[2];
        float measured_load_radius=0;
        CaptureConfig capture_config;capture_config.holding.clear(); // uncalibrated is unobservable
        capture_config.image_height_px=config.frame_height;
        const bool task_calibrated=!config.task_calibration_file.empty();
        if(task_calibrated) {
            const auto measured=loadTaskCalibration(config.task_calibration_file,tuning,config.frame_width,config.frame_height);
            tuning=measured.task;capture_config=measured.capture;measured_load_radius=measured.load_radius_m;
        }
        PushTask task(tuning);
#ifndef RESCUE_ENABLE_TELEMETRY
        if (config.telemetry)
            throw std::runtime_error("Rebuild with -DRESCUE_ENABLE_TELEMETRY=ON to enable telemetry");
#endif
        if (!config.geometry_replay.empty())
            return runGeometryReplay(config.geometry_replay,config.calibration_file,config.zone_geometry_file,config.team);
        if (!config.push_replay.empty()) {
            cv::FileStorage replay(config.push_replay, cv::FileStorage::READ);
            if (!replay.isOpened() || !replay["frames"].isSeq())
                throw std::runtime_error("Replay must contain a frames array");
            for (const auto &frame : replay["frames"]) report(task.update(readObservation(frame)));
            return 0;
        }
        if (!config.detect_image.empty()) {
            VisionFrontend vision_frontend(config);
            auto frame = cv::imread(config.detect_image);
            if (frame.empty()) throw std::runtime_error("Cannot read input image");
            // Warm-up run so the reported time excludes first-call allocation.
            vision_frontend.infer(frame, 1);
            const auto result = vision_frontend.infer(frame, 1);
            std::cout << "detections=" << result.detections.size() << "\n";
            for (const auto &d : result.detections)
                std::cout << "id=" << d.class_id << " raw=" << d.model_label << " task=" << d.label
                          << " confidence=" << d.confidence << " box=" << d.box.x << "," << d.box.y
                          << "," << d.box.width << "," << d.box.height << "\n";
            if (result.pose_ran) {
                printZone(result.halves, zoneHalvesToKeypoints(result.halves, config.pose_keypoint_confidence));
                ZoneColorClassifier colors(config.zone_color_file.empty()?ZoneColorConfig{}:ZoneColorConfig::load(config.zone_color_file));
                const auto identity=colors.classify(frame,result.halves,result.detections,config.pose_keypoint_confidence);
                std::cout<<"[ZONE_COLOR] color="<<identity.color<<" verified="<<identity.verified
                         <<" own="<<(identity.verified&&identity.color==config.team)<<" reason="<<identity.reason<<"\n";
            }
            std::cout << "[PERF] detect_ms=" << result.detect_ms << " pose_ms=" << result.pose_ms
                      << " total_ms=" << result.total_ms << " parallel=" << config.parallel_inference << "\n";
            return 0;
        }
        if (!config.dry_run && !config.hardware)
            throw std::runtime_error("Select --dry-run (perception only) or --hardware (MCU output). "
                "No serial port was opened.");
        std::unique_ptr<HipnucImu> imu;
        if (config.imu) {
            imu = std::make_unique<HipnucImu>(config.imu_port, config.imu_baud, config.imu_timeout_ms);
            std::cout << "[IMU] " << config.imu_port << " @ " << config.imu_baud
                      << " HI91; body axes via roll-180 mounting (2026-09-27), firmware status unverified\n";
        }
        // 车体安全：IMU过期/数值异常/倾斜超过imu_tilt_limit_deg时否决safety_ok。
        SensorFusion fusion({config.sensor_timeout_ms, config.tof_stop_distance_m, config.imu_tilt_limit_deg});
        std::unique_ptr<UARTController> feedback;
        std::unique_ptr<GeometryPipeline> geometry;
        ZoneGeometry zone_geometry;
        if (!config.calibration_file.empty()) {
            CameraCalibration calibration;
            if (!calibration.load(config.calibration_file)) throw std::runtime_error("Invalid ground calibration file");
            zone_geometry=ZoneGeometry::load(config.zone_geometry_file,config.team+"_safe_zone");
            GroundContactConfig contact;contact.min_confidence=config.confidence;
            geometry=std::make_unique<GeometryPipeline>(calibration,zone_geometry,contact);
        }
        CarryNavigator navigator(zone_geometry,tuning,measured_load_radius);
        // --hardware: one exclusive RDWR port carries motion out and A6 feedback in.
        // The link is declared after the port, so unwinding stops the wheels before closing it.
        MatchConfig match_config; match_config.duration_us=uint64_t(config.match_seconds)*1000000;
        MatchControl match(match_config);
        MatchServer match_server(match,config.match_socket);
        bool auto_start_requested=false;
        std::unique_ptr<MotionLink> link;
        if(config.hardware) {
            feedback=std::make_unique<UARTController>();
            feedback->initUART(config.uart_port,config.baudrate,false,config.auto_run);
            // Continue from the MCU's current action id so a restart is never taken as a repeat.
            if(!feedback->syncGripperActionId(Ms(500)))
                throw std::runtime_error("No A6 feedback from "+config.uart_port+" within 500 ms; nothing was sent");
            link=std::make_unique<MotionLink>(*feedback,MotionLinkConfig{},[&match]{return match.status(imuNowUs()).permit;});
            std::cout<<"[MCU] "<<config.uart_port<<" output enabled; 25 Hz resend, zero velocity after 150 ms stall\n";
        } else if(config.pitch_feedback) {
            feedback=std::make_unique<UARTController>();
            feedback->initFeedbackOnly(config.uart_port,config.baudrate);
        }
        VisionFrontend vision_frontend(config);
        ZoneColorClassifier zone_colors(config.zone_color_file.empty()?ZoneColorConfig{}:ZoneColorConfig::load(config.zone_color_file));
        VisionLogic vision(config);
        NearestNeighborTracker tracker;
        CaptureMonitor capture(capture_config);
        // Use the USB camera's MJPEG V4L2 path; automatic GStreamer negotiation
        // fails when applying the requested 720p/60 FPS settings on this board.
        cv::VideoCapture camera(config.camera_index, cv::CAP_V4L2);
        if (!camera.isOpened()) throw std::runtime_error("Cannot open camera");
        camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        camera.set(cv::CAP_PROP_FRAME_WIDTH, config.frame_width);
        camera.set(cv::CAP_PROP_FRAME_HEIGHT, config.frame_height);
        camera.set(cv::CAP_PROP_FPS, config.fps);
        camera.set(cv::CAP_PROP_BUFFERSIZE, 1); // backend may ignore; timestamp remains host dequeue time
        cv::VideoWriter writer;
        if (config.save_output) {
            writer.open("output_cpp.mp4", cv::VideoWriter::fourcc('m','p','4','v'), config.fps,
                        cv::Size(camera.get(cv::CAP_PROP_FRAME_WIDTH), camera.get(cv::CAP_PROP_FRAME_HEIGHT)));
            if (!writer.isOpened()) throw std::runtime_error("Cannot open recording");
        }
        std::cout << (link ? "Ground-pushing LIVE: motion follows PushTask; still gated by safety/geometry evidence.\n"
                           : "Ground-pushing preview: waiting for validated geometry, subzone and safety inputs.\n");
#ifdef RESCUE_ENABLE_TELEMETRY
        std::unique_ptr<TelemetryPublisher> telemetry;
        if (config.telemetry) {
            try {
                telemetry = std::make_unique<TelemetryPublisher>(config);
                std::cout << "[TELEMETRY] snapshot=" << config.telemetry_file << "\n";
            } catch (const std::exception& e) {
                std::cerr << "[TELEMETRY] disabled: " << e.what() << "\n";
            }
        }
#endif
        uint64_t frame_sequence = 0, rate_frames = 0;
        auto rate_started = Clock::now();
        double loop_fps = 0;
        PushOutput previous;
        auto last_report = Clock::now();
        while (!g_should_exit.load()) {
            const auto capture_started = Clock::now();
            cv::Mat frame;
            if (!camera.read(frame) || frame.empty()) throw std::runtime_error("Camera frame lost");
            const auto now = Clock::now();
            const auto epoch_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
            const double capture_ms = std::chrono::duration<double, std::milli>(now - capture_started).count();
            const auto timestamp = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());
            ++frame_sequence;
            // Detect and pose share this frame's id and capture time.
            auto vision_result=vision_frontend.infer(frame,timestamp);
            auto& raw=vision_result.detections;
            for(auto& d:raw){d.frame_id=frame_sequence;d.timestamp_us=timestamp;}
            auto detections = tracker.update(raw, timestamp);
            const double inference_ms = std::chrono::duration<double, std::milli>(Clock::now() - now).count();
            std::vector<ZoneKeypoint> zone_points;
            if(vision_result.pose_ran)zone_points=zoneHalvesToKeypoints(vision_result.halves,config.pose_keypoint_confidence);
            const auto observed_at = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                Clock::now().time_since_epoch()).count());
            const auto zone_color=zone_colors.classify(frame,vision_result.halves,detections,config.pose_keypoint_confidence);
            GeometryResult geometry_result;ExpectedStop expected_stop;
            // Servo readback at capture time; without the feedback port it stays invalid.
            const FrameSensors frame_sensors=feedback?feedback->feedbackAt(timestamp):FrameSensors{};
            if(geometry) {
                GeometryFrame geometry_frame;geometry_frame.frame_id=frame_sequence;
                geometry_frame.capture_us=timestamp;geometry_frame.now_us=observed_at;geometry_frame.image_size=frame.size();
                geometry_frame.sensors=frame_sensors;
                if(imu)geometry_frame.sensors.imu=sensorStateFromImu(imu->snapshotAt(timestamp));
                KeypointFrame points;points.frame_id=frame_sequence;points.capture_us=timestamp;
                points.image_size=frame.size();points.zone_label=config.team+"_safe_zone";
                // Stage 3 supplies measured same-frame points. Missing input is invalid,
                // never reconstructed from a bounding rectangle.
                if(vision_result.pose_ran) {
                    // --team only selects ownership; observed red/blue supplies identity.
                    points.identity_verified=zone_color.verified;
                    points.zone_label=zone_color.verified?zone_color.color+"_safe_zone":"unknown";
                    points.geometry_id=zone_geometry.id;points.points=zone_points;
                } else if(!config.keypoints_file.empty()) {
                    try {
                        cv::FileStorage file(config.keypoints_file,cv::FileStorage::READ);
                        if(file.isOpened()){points=readKeypointFrame(file.root());expected_stop=readExpectedStop(file.root(),points);}
                    } catch(const std::exception&) {points={};expected_stop={};}
                }
                geometry_result=geometry->process(geometry_frame,points,detections);
                detections=geometry_result.detections;
            }
            const bool locked=previous.state==PushState::APPROACH || previous.state==PushState::PREPARE || previous.state==PushState::RUSH;
            auto input = makePushObservation(detections, observed_at,
                previous.first_ordinary_delivered, config.confidence,
                locked?previous.target_id:-1,task.rejectedTargets(observed_at));
            input.run = false; // MatchControl supplies the final authorization below.
            if(geometry)geometry->apply(input,geometry_result,expected_stop,config.team,imuNowUs());
            if(frame_sensors.actuator.valid){
                input.camera_pitch_cdeg=frame_sensors.actuator.camera_pitch_cdeg;
                input.camera_pitch_stable=frame_sensors.pitch_stable;
            }
            // Gripper completion: id echoed and done==1 for the last sent target. Read-only
            // feedback never sends, so it never acknowledges anything.
            UARTController::GripperAck gripper_ack;
            if(link) {
                gripper_ack=feedback->gripperAck();
                if(gripper_ack.result==ActionResult::Done){input.gripper_done=true;input.gripper_feedback_open=gripper_ack.target;}
            }
            // Holding-region and rush-corridor inventory; uncertainty leaves them incomplete.
            capture.update(input, detections, observed_at);
            // Raw boxes do not establish metric distance, route safety or delivery.
            // Keep unconnected evidence invalid; never synthesize successful observations.
            const auto imu_data = imu ? imu->snapshot() : ImuSnapshot{};
            // IMU absence/fault/over-tilt can veto permission; it never establishes geometry or route safety.
            // SensorState uses body axes (roll-180 mounting calibrated 2026-09-27), same steady clock as observed_at.
            const SensorState sensor = sensorStateFromImu(imu_data);
            if (imu) {
                fusion.update(sensor);
                input.safety_ok = !fusion.emergencyStop(imuNowUs());
                // Relative yaw only measures the post-delivery turn.
                input.heading_valid = imu_data.fresh && imu_data.sample.measurements_valid &&
                    std::isfinite(imu_data.sample.body_rpy_rad[2]);
                input.heading_rad = imu_data.sample.body_rpy_rad[2];
            }
            // A dead link (write failures or stale feedback) vetoes motion permission.
            std::string preflight_reason;
            if(!imu || !input.safety_ok) preflight_reason="imu_not_ready";
            else if(!task_calibrated) preflight_reason="gripper_calibration_required";
            else if(frame.cols!=config.frame_width || frame.rows!=config.frame_height) preflight_reason="camera_size_mismatch";
            else if(!geometry) preflight_reason="ground_mapping_not_ready";
            else if(config.pose_model_path.empty() && config.keypoints_file.empty()) preflight_reason="zone_pose_source_required";
            else if(config.zone_color_file.empty() && config.keypoints_file.empty()) preflight_reason="zone_identity_source_required";
            else if(link && !feedback->latestActuatorFeedback().valid) preflight_reason="mcu_feedback_stale";
            else if(link && link->stats().sent && !link->stats().last_ok) preflight_reason="mcu_write_failed";
            else if(!frame_sensors.actuator.valid || input.camera_pitch_cdeg==kCameraPitchInvalid) preflight_reason="camera_pitch_feedback_invalid";
            input.now_us=imuNowUs();
            match.health(input.now_us,preflight_reason.empty(),preflight_reason);
            // Only independently identified, observed (not predicted) fixed-zone poses
            // can supply measured translation. Loss of the reference does not count as movement.
            if(input.zone_identity_verified && input.zone_estimate.trusted(input.now_us) &&
               input.zone_estimate.source!=ZoneEstimate::Source::PREDICTED) {
                const auto position=input.zone_estimate.bodyToZone({0,0});
                const double uncertainty=2*input.zone_estimate.position_sigma_m+
                    2*cv::norm(position)*input.zone_estimate.yaw_sigma_rad;
                match.measuredPosition(input.zone_estimate.timestamp_us,position.x,position.y,input.zone_estimate.geometry_id,uncertainty);
            }
            if(config.auto_run && !auto_start_requested && preflight_reason.empty()) {
                auto_start_requested=true;match.command(MatchCommand::START,input.now_us);
            }
            const auto match_status=match.status(input.now_us);
            input.run=match_status.permit;
            input.safety_ok=match_status.permit && preflight_reason.empty();
            navigator.update(input,previous);
            auto out = task.update(input);
            out.match_state=MatchControl::name(match_status.state);out.match_reason=match_status.reason;
            out.match_remaining_us=match_status.remaining_us;
            out.hardware_output_enabled=bool(link)&&match_status.permit;
            previous = out;
            if(link)link->submit(out.motion);
            ++rate_frames;
            const auto rate_now = Clock::now();
            const double rate_seconds = std::chrono::duration<double>(rate_now - rate_started).count();
            if (rate_seconds >= 1.0) {
                loop_fps = rate_frames / rate_seconds;
                rate_frames = 0; rate_started = rate_now;
            }
            if (now - last_report >= Ms(500)) {
                report(out);
                std::cout<<"[MATCH] state="<<MatchControl::name(match_status.state)<<" reason="<<match_status.reason
                         <<" remaining_ms="<<match_status.remaining_us/1000<<" preflight="<<preflight_reason<<"\n";
                std::cout<<"[EVIDENCE] identity="<<input.zone_identity_verified<<" target_region="<<input.target_region_valid
                         <<" color="<<zone_color.color<<" color_reason="<<zone_color.reason<<" path="<<input.path_safe<<" opponent_clear="<<input.opponent_zone_clear
                         <<" retreat="<<input.retreat_safe<<" zone_counts="<<input.zone_counts_valid<<"\n";
                if(geometry)std::cout<<"[GEOMETRY] mapped="<<geometry_result.mapping_valid
                    <<" zone_valid="<<input.zone_estimate.trusted(input.now_us)<<" reason="<<geometry_result.reason
                    <<" inliers="<<input.zone_estimate.inlier_ids.size()<<" residual_m="<<input.zone_estimate.residual_m<<"\n";
                std::cout << "[PERF] loop_fps=" << loop_fps << " inference_ms=" << inference_ms
                          << " detect_ms=" << vision_result.detect_ms << " pose_ms=" << vision_result.pose_ms
                          << " zone_points=" << zone_points.size()
                          << " capture_ms=" << capture_ms << " detections=" << detections.size() << "\n";
                if (imu) {
                    const auto& p = imu_data.sample;
                    std::cout << "[IMU] fresh=" << imu_data.fresh << " age_ms=" << imu_data.age_ms
                              << " numeric_ok=" << p.measurements_valid << " seq=" << p.sequence
                              << " body_rpy_rad=" << p.body_rpy_rad[0] << "," << p.body_rpy_rad[1] << "," << p.body_rpy_rad[2]
                              << " status=" << p.status << " crc_errors=" << imu_data.crc_errors
                              << " sensor_valid=" << sensor.imu_valid << " estop=" << fusion.emergencyStop(observed_at) << "\n";
                }
                if (link) {
                    static const char *ack_names[] = {"idle", "no_feedback", "not_done", "done"};
                    const auto st = link->stats();
                    const auto fb = feedback->latestActuatorFeedback();
                    std::cout << "[MCU] healthy=" << link->healthy() << " sent=" << st.sent << " failed=" << st.failed
                              << " stale_zero=" << st.stale << " gripper_id=" << int(feedback->gripperActionId())
                              << " target=" << gripper_ack.target << " ack=" << ack_names[int(gripper_ack.result)]
                              << " fb_valid=" << fb.valid << " fb_id=" << int(fb.gripper_action_id)
                              << " fb_done=" << int(fb.gripper_done) << " pitch_rb=" << fb.camera_pitch_cdeg << "\n";
                }
                last_report = now;
            }
            vision.drawDetections(frame, detections);
            drawZone(frame, zone_points);
            cv::putText(frame, link ? "PUSH LIVE - MCU OUTPUT ENABLED" : "PUSH PREVIEW - HARDWARE DISABLED", {10,25},
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, link ? cv::Scalar(0,0,255) : cv::Scalar(0,255,255), 2);
#ifdef RESCUE_ENABLE_TELEMETRY
            if (telemetry) telemetry->submit(frame, detections, input, out, epoch_ns,
                                             frame_sequence, loop_fps, inference_ms, capture_ms, imu_data);
#endif
            if (writer.isOpened()) writer.write(frame);
            if (config.show) {
                cv::imshow("Ground pushing", frame);
                const int key = cv::waitKey(1);
                if (key == 27 || key == 'q' || key == 'Q') break;
            }
        }
        if (link) { link->stop(); std::cout << "[MCU] stopped: final zero-velocity packet sent\n"; }
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "[FATAL] " << e.what() << "\n";
        return 1;
    }
}
