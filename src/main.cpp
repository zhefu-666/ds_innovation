#include "rescue/config.hpp"
#include "rescue/demo_policy.hpp"
#include "rescue/async_log.hpp"
#include <sstream>
#include <limits>
#include "rescue/controlled_field.hpp"
#include "rescue/geometry_pipeline.hpp"
#include "rescue/uart_controller.hpp"
#include "rescue/motion_link.hpp"
#include "rescue/motion_readiness.hpp"
#include "rescue/match_control.hpp"
#include "rescue/match_server.hpp"
#include "rescue/hipnuc_imu.hpp"
#include "rescue/imu_adapter.hpp"
#include "rescue/sensor_fusion.hpp"
#include "rescue/detector.hpp"
#include "rescue/zone_keypoints.hpp"
#include "rescue/zone_visual_reference.hpp"
#include "rescue/zone_color.hpp"
#include "rescue/push_task.hpp"
#include "rescue/capture_monitor.hpp"
#include "rescue/task_calibration.hpp"
#include "rescue/navigation_adapter.hpp"
#include "rescue/perception_adapter.hpp"
#include "rescue/pixel_selector.hpp"
#include "rescue/multi_view_capture.hpp"
#include "rescue/tracker.hpp"
#include "rescue/utils.hpp"
#include "rescue/vision_logic.hpp"
#include <opencv2/opencv.hpp>
#include <csignal>
#include <algorithm>
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
    flag("target_is_search_cue",in.target_is_search_cue);
    flag("gripper_closed_observed",in.gripper_closed_observed);
    flag("target_valid", in.target_valid); flag("geometry_valid", in.geometry_valid);
    flag("path_safe", in.path_safe); flag("retreat_safe", in.retreat_safe);
    flag("target_region_valid", in.target_region_valid);
    flag("zone_identity_verified", in.zone_identity_verified);
    flag("opponent_zone_clear", in.opponent_zone_clear); flag("target_in_zone", in.target_in_zone);
    flag("delivery_observed", in.delivery_observed);
    flag("corridor_complete", in.corridor_complete); flag("corridor_occlusion_free", in.corridor_occlusion_free);
    flag("hold_observable", in.hold_observable);
    flag("captured", in.captured); flag("held_complete", in.held_complete);
    flag("camera_pitch_stable", in.camera_pitch_stable);
    flag("multi_view_finished",in.multi_view_finished);
    if(!n["multi_view_verdict"].empty()) n["multi_view_verdict"]>>in.multi_view_verdict;
    flag("gripper_done", in.gripper_done);
    flag("zone_valid", in.zone_valid); flag("zone_own", in.zone_own);
    flag("carry_plan_valid",in.carry_plan_valid);flag("drop_plan_valid",in.drop_plan_valid);
    if(!n["navigation_timestamp_us"].empty())in.navigation_timestamp_us=rescue::readExactTime(n["navigation_timestamp_us"]);
    const auto point=[&](const char* key,cv::Point2f& p){const auto v=n[key];if(!v.empty()) {
        if(!v.isSeq()||v.size()!=2)throw std::runtime_error("Invalid replay navigation point");p={float(v[0]),float(v[1])};}};
    point("carry_waypoint_body",in.carry_waypoint_body);point("drop_centre_zone",in.drop_centre_zone);
    flag("zone_inventory_complete", in.zone_inventory_complete);
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
    inventory("multi_view_inventory",in.multi_view_inventory);
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
void report(const rescue::PushOutput &out, std::ostream& stream = std::cout) {
    stream << "state=" << rescue::PushTask::name(out.state) << " batch=" << out.batch_size
              << " delivered=" << out.delivered_total << " first=" << out.first_ordinary_delivered
              << " vx=" << out.motion.vx_mps << " wz=" << out.motion.wz_rps
              << " frame_offset_deg=" << int(out.motion.gripper_offset)
              << " startup_forward_ms=" << out.startup_commanded_us/1000
              << " search_cue_id=" << out.search_cue_id << " capture_target_id=" << out.capture_target_id
              << " cue=" << out.target_is_search_cue << " clear_attempts=" << out.clearing_attempts
              << " pitch_cmd=" << out.motion.camera_pitch_cdeg
              << " camera_protocol_cmd_deg=" << rescue::cameraPitchToWire(out.motion.camera_pitch_cdeg) << " stop="
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
        std::cout << "MATCH_TIME_LIMIT=" << (config.no_match_time_limit ? "disabled_debug" : "enabled") << "\n";
        if(config.allow_mechanical_pitch_model)
            std::cout << "GEOMETRY_SOURCE=mechanical_assumption NOT_FIELD_VALIDATED\n";
        TaskTuning tuning;
        tuning.far_pitch_cdeg = config.pitch_presets_cdeg[0];
        tuning.track_pitch_cdeg = config.pitch_presets_cdeg[1];
        tuning.near_pitch_cdeg = config.pitch_presets_cdeg[2];
        if(config.allow_mechanical_pitch_model && tuning.track_pitch_cdeg>2000) {
            tuning.intermediate_pitch_cdeg=2000;
            tuning.track_near_m=.4f;
            // 20deg loses the box at the image bottom below ~0.22-0.25 m forward (field logs 2026-10-05).
            tuning.intermediate_to_near_m=.27f;
            std::cout << "PITCH_SWITCH mid20_distance_m=0.4 near40_distance_m=0.27 open_jaw_handoff_m="
                      << tuning.open_jaw_handoff_m << "\n";
        }
        tuning.enable_search_cues=config.search_cues;
        tuning.enable_lost_search=true;
        tuning.enable_short_push=config.controlled_empty_field && config.search_cues;
        tuning.startup_advance_speed=config.startup_speed_mps;
        tuning.max_speed=std::max(tuning.max_speed,tuning.startup_advance_speed);
        tuning.scan_wz=config.scan_wz_rps;tuning.turn_wz=config.turn_wz_rps;
        tuning.max_wz=std::max({tuning.max_wz,tuning.scan_wz,tuning.turn_wz});
        tuning.startup_advance_us = uint64_t(config.startup_advance_ms) * 1000;
        // A full sweep at 0.25 rad/s already takes ~25s, before stopped hold checks.
        // Controlled start-to-zone tests reserve time for both searching and transport.
        if(config.controlled_empty_field) tuning.carry_budget_us = 90000000;
        cv::Vec<float,6> body_envelope{};
        float measured_load_radius=0, measured_body_radius=0, load_half_width=0, load_half_depth=0;
        std::vector<FrameView> frame_views;int feedback_open=-1,feedback_close=-1;bool box_area_accepted=false;
        bool a6_done_flag=false;int a6_done_settle_ms=0;bool view_image_polygon=false;
        CaptureConfig capture_config;capture_config.holding.clear(); // uncalibrated is unobservable
        capture_config.image_height_px=config.frame_height;
        const bool task_calibrated=!config.task_calibration_file.empty();
        if(task_calibrated) {
            const auto measured=loadTaskCalibration(config.task_calibration_file,tuning,config.frame_width,config.frame_height,config.demo_mode=="search");
            frame_views=measured.frame_views;feedback_open=measured.feedback_open;feedback_close=measured.feedback_close;box_area_accepted=measured.box_area_accepted;
            a6_done_flag=measured.a6_done_flag;a6_done_settle_ms=measured.a6_done_settle_ms;view_image_polygon=measured.frame_view_image_polygon;
            body_envelope=measured.body_envelope;
            tuning=measured.task;capture_config=measured.capture;measured_load_radius=measured.load_radius_m;measured_body_radius=measured.robot_swept_radius_m;
            load_half_width=measured.load_half_width_m;load_half_depth=measured.load_half_depth_m;
        }
        configureDemo(config,tuning);
        if(config.assume_all_safe) {
            // TEMP_ASSUMPTION（2026-10-08用户要求“默认全都安全”）：只为跑通决策链，不是验收结果。
            // 实测：指令0.1m/s*1s视觉距离仅降约2.2cm（指令/实际≈5:1），按指令积分的限程需放宽。
            // 实测下一趟转向：指令0.25rad/s，IMU实际约0.022rad/s，15s仅转约0.33rad，达不到1.6rad。
    tuning.turn_min_rad=std::min(tuning.turn_min_rad,.25f);
            // 用户要求加快：实测实际速度仅为指令的1/5~1/10，指令速度整体提高约3倍（仅假设模式）。
            tuning.max_speed=.40f;tuning.approach_speed=.40f;tuning.carry_speed=.34f;tuning.rush_speed=.32f;tuning.rush_budget_us=12000000;
            tuning.enter_speed=.30f;tuning.back_speed=.32f;tuning.turn_wz=.55f;tuning.max_wz=.55f;tuning.min_turn_wz=std::min(tuning.min_turn_wz,.40f);
    tuning.assume_all_safe=true;tuning.assume_injured_trip=config.assume_injured_trip;tuning.approach_limit_m=std::max(tuning.approach_limit_m,8.0f);
            tuning.retreat_limit_m=std::max(tuning.retreat_limit_m,4.0f);
            tuning.assume_carry_m=std::max(tuning.assume_carry_m,.60f);tuning.assume_push_m=.55f;tuning.enter_budget_us=std::max<uint64_t>(tuning.enter_budget_us,20000000);tuning.assume_back_m=std::max(tuning.assume_back_m,std::min(.50f,.85f*tuning.back_speed*float(tuning.retreat_budget_us)/1e6f)); // 须在后退时限内完成
            std::cout<<"[TEMP_ASSUMPTION] assume_all_safe=1 injured_trip="<<config.assume_injured_trip<<" clearance/zone/capture/delivery evidence assumed true;"
                " blind carry_m="<<tuning.assume_carry_m<<" back_m="<<tuning.assume_back_m
                <<" approach_limit_m="<<tuning.approach_limit_m<<" retreat_limit_m="<<tuning.retreat_limit_m
                <<" max_speed="<<tuning.max_speed<<" NOT_ACCEPTANCE\n";
        }
        const bool a6_known=a6_done_flag ? config.known_frame_angle>=0 && config.known_frame_id>0 : feedback_open>=0;
        if(a6_done_flag)std::cout<<"[A6] TEMP_ASSUMPTION a6_semantics=done_flag: byte1==1 with matching id means action finished (open or close);"
            " frame state = last completed target sent by this host; startup state from --known-frame-angle "<<config.known_frame_angle
            <<" --known-frame-id "<<config.known_frame_id<<"; settle_ms="<<a6_done_settle_ms<<"; not a position sensor\n";
        if(view_image_polygon)std::cout<<"[MULTI_VIEW] TEMP_ASSUMPTION frame_view_matching=image_polygon: enclosure judged from per-pitch image polygons;"
            " no ground ranging inside the frame; views compared by class counts only\n";
        if(config.hardware && (!a6_known||(config.demo_mode!="search" && frame_views.size()<2)))throw std::runtime_error(a6_done_flag?
            "Done-flag A6 requires --known-frame-angle and --known-frame-id plus schema 2 views before hardware output":
            "New frame requires accepted schema 2 calibration and A6 mapping before hardware output");
        if (config.check_config) {
            int missing = 0;
            const auto check = [&](bool ok, const char* reason) {
                std::cout << (ok ? "OK " : "MISSING ") << reason << "\n";
                missing += !ok;
            };
            check(task_calibrated, "task_calibration");
            check(a6_known && (config.demo_mode=="search" || frame_views.size()>=2),"new_frame_mapping_and_required_calibration");
            const auto g = ZoneGeometry::load(config.zone_geometry_file, config.team+"_safe_zone");
            check(loadFitsHalf(measured_load_radius,g.width_m,g.depth_m,g.divider_exclusion_half_width_m,load_half_width,load_half_depth),
                  "load_footprint_fits_half");
            CameraCalibration c;
            const bool mapped = !config.calibration_file.empty() && c.load(config.calibration_file,config.allow_mechanical_pitch_model);
            check(mapped && c.pitchUsable(tuning.far_pitch_cdeg) && c.pitchUsable(tuning.track_pitch_cdeg),
                  "far_track_ground_calibration");
            if(tuning.intermediate_pitch_cdeg!=kCameraPitchInvalid)
                check(mapped && c.pitchUsable(tuning.intermediate_pitch_cdeg), "intermediate_ground_model");
            check(config.controlled_empty_field || (!config.zone_color_file.empty() && ZoneColorConfig::load(config.zone_color_file).measured),
                  "zone_identity_source");
            if(config.controlled_empty_field) check(measured_body_radius>0,"robot_swept_radius");
            check(!config.pose_model_path.empty() || !config.keypoints_file.empty(), "zone_pose_source");
            std::cout << "STARTUP forward_ms=" << config.startup_advance_ms
                      << " speed_mps=" << tuning.startup_advance_speed
                      << " scan_wz_rps=" << tuning.scan_wz << " turn_wz_rps=" << tuning.turn_wz
                      << " angular_limit_rps=" << tuning.max_wz
                      << " clearance_checks_disabled=" << config.controlled_ignore_clearance << " search_cues=" << config.search_cues << "\n";
            std::cout << "STATIC_ONLY: live observations, sensor timing and actual motion/holding "
                         "still require independent evidence. hardware_output=disabled\n";
            return missing ? 2 : 0;
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
        AsyncLog async_log;
        std::string last_stop_signature;
        uint64_t last_stale_count = 0;
        auto last_submit_time = Clock::now();
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
        CameraCalibration vref_calibration;bool vref_ready=false;
        ZoneGeometry zone_geometry;
        if (!config.calibration_file.empty()) {
            CameraCalibration calibration;
            if (!calibration.load(config.calibration_file,config.allow_mechanical_pitch_model)) throw std::runtime_error("Invalid ground calibration file");
            zone_geometry=ZoneGeometry::load(config.zone_geometry_file,config.team+"_safe_zone");
            GroundContactConfig contact;contact.min_confidence=config.confidence;contact.accept_size_mismatch=config.assume_all_safe;
            geometry=std::make_unique<GeometryPipeline>(calibration,zone_geometry,contact);
            vref_calibration=calibration;vref_ready=true;
        }
        CarryNavigator navigator(zone_geometry,tuning,measured_load_radius,config.controlled_empty_field,load_half_width,load_half_depth);
        ControlledField controlled_field(zone_geometry,measured_body_radius,body_envelope);
        // --hardware: one exclusive RDWR port carries motion out and A6 feedback in.
        // The link is declared after the port, so unwinding stops the wheels before closing it.
        MatchConfig match_config; match_config.duration_us=uint64_t(config.match_seconds)*1000000;
        match_config.time_limit_enabled=!config.no_match_time_limit;
        match_config.require_measured_progress=!config.controlled_empty_field;
        MatchControl match(match_config);
        MatchServer match_server(match,config.match_socket);
        bool auto_start_requested=false;
        std::unique_ptr<MotionLink> link;
        int demo_hold_angle=-1;int16_t demo_hold_pitch=kCameraPitchInvalid;
        if(config.hardware) {
            feedback=std::make_unique<UARTController>();
            if(a6_done_flag)feedback->configureDoneFlagFeedback(config.known_frame_angle,config.known_frame_id,a6_done_settle_ms);
            else feedback->configureFrameFeedback(feedback_open,feedback_close);
            feedback->initUART(config.uart_port,config.baudrate,false,config.auto_run);
            // Continue from the MCU's current action id so a restart is never taken as a repeat.
            if(!feedback->syncGripperActionId(Ms(500)))
                throw std::runtime_error(a6_done_flag?"A6 id/done flag differs from --known-frame-id or no feedback within 500 ms; nothing was sent":
                    "No A6 feedback from "+config.uart_port+" within 500 ms; nothing was sent");
            {const auto mcu=feedback->latestActuatorFeedback();
             std::cout<<"[MCU] synced gripper_id="<<int(mcu.gripper_action_id)<<" open="<<int(mcu.gripper_open)
                      <<"; keeping this state reuses the id, each state change uses id+1\n";}
            {const auto fb=feedback->latestActuatorFeedback();const int open=feedback->confirmedFrameOpen(fb);
             if(open<0)throw std::runtime_error("Frame position unknown at startup; nothing was sent");
             demo_hold_angle=open?0:20;demo_hold_pitch=fb.camera_pitch_cdeg;
             std::cout<<"[MCU] startup frame angle="<<demo_hold_angle<<" source="<<(a6_done_flag?"operator_known_angle(TEMP_ASSUMPTION)":"a6_state_mapping")<<"\n";}
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
        PixelSelector pixel_selector;MultiViewCapture multi_view(frame_views,{config.frame_width,config.frame_height},view_image_polygon);
        bool observing=false;uint64_t observation_round=0;
        ZoneEstimate observation_pose;
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
        PushOutput previous;previous.first_ordinary_delivered=tuning.assume_injured_trip;
        const std::string prefer_label=config.assume_injured_trip?"injured_person":"";
        auto last_report = Clock::now();
        const auto demo_started=Clock::now();int demo_exit=0;
        while (!g_should_exit.load()) {
            if(config.demo_mode!="none" && Clock::now()-demo_started>=std::chrono::seconds(config.demo_seconds)){
                std::cout<<"[DEMO_RESULT] wall_budget_exhausted mode="<<config.demo_mode<<"\n";
                demo_exit=config.demo_mode=="recognize"?0:2;break;
            }
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
            const bool blue_model_detected_both_halves = !config.pose_model_blue_path.empty() &&
                vision_result.pose_ran && vision_result.halves[0].valid && vision_result.halves[1].valid;
            auto zone_color=previewBlueZoneDefault(
                zone_colors.classify(frame,vision_result.halves,detections,config.pose_keypoint_confidence),
                config.dry_run,config.team,blue_model_detected_both_halves);
            if(config.controlled_empty_field && vision_result.pose_ran &&
               vision_result.halves[0].valid && vision_result.halves[1].valid) {
                zone_color.color=config.team;zone_color.verified=true;zone_color.assumed=false;
                zone_color.reason="controlled_field_operator_identity";
            } else if(config.assume_all_safe) {
                // TEMP_ASSUMPTION: colour fixed to the team at start, independent of any detection.
                zone_color.color=config.team;zone_color.verified=true;zone_color.assumed=true;
                zone_color.reason="assume_all_safe_default_team_colour";
            }
            GeometryResult geometry_result;ExpectedStop expected_stop;ZoneEstimate vref_estimate;bool vref_ok=false;
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
                    // Preview may fit blue geometry from the selected model, but only observed color verifies ownership.
                    points.identity_verified=zone_color.verified;
                    points.zone_label=zone_color.color.empty()?"unknown":zone_color.color+"_safe_zone";
                    points.geometry_id=zone_geometry.id;points.points=zone_points;
                } else if(!config.keypoints_file.empty()) {
                    try {
                        cv::FileStorage file(config.keypoints_file,cv::FileStorage::READ);
                        if(file.isOpened()){points=readKeypointFrame(file.root());expected_stop=readExpectedStop(file.root(),points);}
                    } catch(const std::exception&) {points={};expected_stop={};}
                }
                geometry_result=geometry->process(geometry_frame,points,detections);
                if(config.assume_all_safe&&vref_ready&&vision_result.pose_ran&&zone_color.color==config.team){
                    vref_estimate=ZoneEstimate{};
                    vref_ok=estimateVisualZone(vref_calibration,zone_geometry,zone_points,frame_sensors.actuator.camera_pitch_cdeg,
                        &geometry_frame.sensors.imu,config.pose_keypoint_confidence,vref_estimate);
                    if(vref_ok){vref_estimate.timestamp_us=observed_at;vref_estimate.observed_us=observed_at;vref_estimate.zone_label=points.zone_label;}
                    std::cout<<"[VREF] ok="<<vref_ok<<" n="<<vref_estimate.inlier_ids.size();
                    if(vref_ok)std::cout<<" ox="<<vref_estimate.origin_body_m.x<<" oy="<<vref_estimate.origin_body_m.y
                        <<" yaw="<<vref_estimate.yaw_body_rad<<" res="<<vref_estimate.residual_m;
                    std::cout<<"\n";
                }
                detections=geometry_result.detections;
            }
            const auto selection=pixel_selector.update(detections,frame.size(),frame_sensors.actuator.camera_pitch_cdeg,
                frame_sensors.actuator.valid && frame_sensors.pitch_stable,observed_at,previous.first_ordinary_delivered,
                config.confidence,task.rejectedTargets(observed_at),prefer_label);
            auto input = makePushObservation(detections, observed_at,previous.first_ordinary_delivered,config.confidence,
                selection.id,task.rejectedTargets(observed_at),false,false,false,prefer_label);
            if(!selection.locked) input.target_valid=false;
            if(selection.mode=="box_area" && !box_area_accepted && !config.assume_all_safe && !(config.demo_mode=="search" && config.dry_run))input.target_valid=false;
            std::cout << "[PIXEL_SELECTION] timestamp="<<observed_at<<" pitch="<<frame_sensors.actuator.camera_pitch_cdeg
                <<" stable="<<frame_sensors.pitch_stable<<" id="<<selection.id<<" mode="<<selection.mode
                <<" raw="<<selection.raw<<" smooth="<<selection.smoothed<<" locked="<<selection.locked<<" reason="<<selection.reason<<"\n";
            input.pixel_ratio_raw=selection.raw;input.pixel_ratio_smoothed=selection.smoothed;
            input.pixel_mode=selection.mode;input.pixel_reason=selection.reason;input.pixel_locked=selection.locked;
            input.run = false; // MatchControl supplies the final authorization below.
            if(geometry)geometry->apply(input,geometry_result,expected_stop,config.team,imuNowUs());
            if(config.assume_all_safe){
                for(const auto&d:detections)
                    if(d.ground_position_valid&&d.confidence>=config.confidence&&std::isfinite(d.body_xy_m.x)&&std::isfinite(d.body_xy_m.y)&&
                       (d.label=="ordinary_supply"||d.label=="core_supply"||d.label=="injured_person"))input.cargo_body.push_back(d.body_xy_m);
                input.cargo_ts_us=observed_at;
            }
            if(vref_ok){input.vref_valid=true;input.vref_zone=vref_estimate;input.vref_points=int(vref_estimate.inlier_ids.size());}
            if(frame_sensors.actuator.valid){
                // Done-flag A6 cannot show position: closed only after this host's close target completed.
                input.gripper_closed_observed=link ? feedback->confirmedFrameOpen(frame_sensors.actuator)==0 :
                    !a6_done_flag && feedback_close>=0 && frame_sensors.actuator.gripper_open==feedback_close;
                input.camera_pitch_cdeg=frame_sensors.actuator.camera_pitch_cdeg;
                input.camera_pitch_stable=frame_sensors.pitch_stable;
            }
            // Gripper completion: id echoed and observed open/closed state matches the last target. Read-only
            // feedback never sends, so it never acknowledges anything.
            UARTController::GripperAck gripper_ack;
            if(link) {
                gripper_ack=feedback->gripperAck();
                if(gripper_ack.result==ActionResult::Done){input.gripper_done=true;input.gripper_feedback_open=gripper_ack.target;}
            }
            // Holding-region and rush-corridor inventory; uncertainty leaves them incomplete.
            capture.update(input, detections, observed_at);
            if(previous.state==PushState::VERIFY_CAPTURE){
                const auto fb=feedback?feedback->latestActuatorFeedback():ActuatorFeedback{};
                if(!observing){multi_view.begin(observed_at,fb.gripper_action_id,++observation_round);observing=true;observation_pose=input.zone_estimate;}
                const bool stationary_pose=observation_pose.trusted(observation_pose.timestamp_us) && input.zone_estimate.trusted(observed_at) &&
                    observation_pose.source!=ZoneEstimate::Source::PREDICTED && input.zone_estimate.source!=ZoneEstimate::Source::PREDICTED &&
                    observation_pose.geometry_id==input.zone_estimate.geometry_id &&
                    cv::norm(observation_pose.origin_body_m-input.zone_estimate.origin_body_m)<.01 &&
                    std::abs(wrapAngle(observation_pose.yaw_body_rad-input.zone_estimate.yaw_body_rad))<.02;
                multi_view.update(observed_at,timestamp,fb.gripper_action_id,
                    fb.valid && input.gripper_done && input.gripper_feedback_open==0,
                    (stationary_pose||config.assume_all_safe) && previous.motion.vx_mps==0 && previous.motion.wz_rps==0,
                    input.camera_pitch_cdeg,input.camera_pitch_stable,detections);
                const auto& evidence=multi_view.result();input.multi_view_finished=evidence.finished;
                input.multi_view_verdict=int(evidence.verdict);input.multi_view_inventory=evidence.inventory;
                input.multi_view_pitch=multi_view.desiredPitch();
                input.multi_view_reason=evidence.reason;input.observation_view=evidence.view;input.observation_frames=evidence.frames;input.observation_round=evidence.round;
                std::cout<<"[MULTI_VIEW] timestamp="<<observed_at<<" round="<<evidence.round<<" action="<<int(fb.gripper_action_id)
                    <<" view="<<evidence.view<<" frames="<<evidence.frames<<" verdict="<<captureVerdictName(evidence.verdict)
                    <<" finished="<<evidence.finished<<" reason="<<evidence.reason<<"\n";
            }else if(observing){multi_view.reset();observing=false;}

            if(config.controlled_empty_field) {
                // injured-trip forces first=true for target selection only; the unseen-zone fallback must stay on until a real delivery.
                PushOutput field_previous=previous;
                if(config.assume_injured_trip)field_previous.first_ordinary_delivered=previous.delivered_total>0;
                controlled_field.update(input,detections,vision_result.halves,frame.size(),field_previous,geometry_result.mapping_valid,config.controlled_ignore_clearance);
            }
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
            else if(!loadFitsHalf(measured_load_radius,zone_geometry.width_m,zone_geometry.depth_m,
                                 zone_geometry.divider_exclusion_half_width_m,load_half_width,load_half_depth)) preflight_reason="load_footprint_does_not_fit";
            else if(!config.controlled_empty_field && config.zone_color_file.empty()) preflight_reason="zone_color_calibration_required";
            else if(config.controlled_empty_field && measured_body_radius<=0) preflight_reason="robot_swept_radius_required";
            else if(frame.cols!=config.frame_width || frame.rows!=config.frame_height) preflight_reason="camera_size_mismatch";
            else if(!geometry) preflight_reason="ground_mapping_not_ready";
            else if(config.pose_model_path.empty() && config.keypoints_file.empty()) preflight_reason="zone_pose_source_required";
            else if(link && !feedback->latestActuatorFeedback().valid) preflight_reason="mcu_feedback_stale";
            else if(link && link->stats().sent && !link->stats().last_ok) preflight_reason="mcu_write_failed";
            else if(!frame_sensors.actuator.valid || input.camera_pitch_cdeg==kCameraPitchInvalid) preflight_reason="camera_pitch_feedback_invalid";
            // 2026-10-08用户要求：开机静止位RX40（旧0°）也可启动，启动后由任务自行转到旧5°(RX45)再搜索。
            else if(!match.status(imuNowUs()).started_us &&
                    (!frame_sensors.pitch_stable ||
                     (std::abs(int(input.camera_pitch_cdeg) - int(tuning.far_pitch_cdeg)) > tuning.pitch_tolerance_cdeg &&
                      std::abs(int(input.camera_pitch_cdeg) - int(kStartupRestPitchCdeg)) > tuning.pitch_tolerance_cdeg)))
                preflight_reason="startup_pitch_not_rest_or_5deg";
            else if(!config.assume_all_safe && !geometry_result.mapping_valid &&
                    !stationaryPitchWork(previous, geometry_result.reason))
                preflight_reason=geometry_result.reason.empty()?"ground_mapping_unavailable":geometry_result.reason;
            else if(!config.assume_all_safe && !match.status(imuNowUs()).started_us && (!input.path_safe || !input.opponent_zone_clear))
                preflight_reason="startup_clearance_required";
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
            if(config.assume_all_safe)std::cout<<"[ANCHOR] state="<<int(out.state)<<" reason="<<out.reason<<" "<<task.anchorDebug()<<"\n";
            if(config.assume_all_safe){const std::string dz=task.takeDropZoneEvent();if(!dz.empty())std::cout<<dz<<std::flush;}
            guardDemoMotion(config.demo_mode,out.motion,demo_hold_angle,demo_hold_pitch);
            if(config.demo_mode=="search" && config.dry_run)
                std::cout<<"[DEMO_SEARCH_PREVIEW] target="<<selection.id<<" locked="<<selection.locked<<" pixel_mode="<<selection.mode<<" ratio="<<selection.smoothed<<" geometry="<<input.geometry_valid<<" NO_TX\n";
            // Independent final gate: stationary image checks never authorize wheels.
            if(!config.assume_all_safe) inhibitUnmappedMotion(out.motion, geometry_result.mapping_valid);
            out.match_state=MatchControl::name(match_status.state);out.match_reason=match_status.reason;
            out.match_remaining_us=match_status.remaining_us;
            out.hardware_output_enabled=bool(link)&&match_status.permit;
            previous = out;
            const auto submit_time = Clock::now();
            const auto submit_gap_ms = std::chrono::duration<double, std::milli>(submit_time-last_submit_time).count();
            last_submit_time = submit_time;
            if(link) {
                const auto stale_count = link->stats().stale;
                link->submit(out.motion);
                if (stale_count != last_stale_count) {
                    async_log.submit("[WATCHDOG] timestamp_us=" + std::to_string(input.now_us)
                        + " stale_zero_delta=" + std::to_string(stale_count-last_stale_count)
                        + " submit_gap_ms=" + std::to_string(submit_gap_ms) + "\n");
                    last_stale_count = stale_count;
                }
            }
            // Record every changed stop condition, including one-frame stops.
            // task_reason is historical state-machine context; evidence is current.
            const bool stopped = out.motion.vx_mps == 0 && out.motion.wz_rps == 0;
            std::ostringstream stop_detail;
            stop_detail << "stopped=" << stopped << " state=" << PushTask::name(out.state)
                << " task_reason=" << out.reason << " preflight=" << preflight_reason
                << " match=" << match_status.reason << " locked_id=" << out.target_id
                << " observed_id=" << input.target_id << " target_valid=" << input.target_valid
                << " id_match=" << (out.target_id >= 0 && out.target_id == input.target_id)
                << " geometry=" << input.geometry_valid << " mapping=" << geometry_result.mapping_valid
                << " geometry_reason=" << geometry_result.reason
                << " region=" << input.target_region_valid << " in_zone=" << input.target_in_zone
                << " path=" << input.path_safe << " opponent_clear=" << input.opponent_zone_clear
                << " pitch_stable=" << input.camera_pitch_stable << " gripper_done=" << input.gripper_done
                << " corridor_complete=" << input.corridor_complete;
            if (feedback) {
                const auto fb = feedback->latestActuatorFeedback();
                stop_detail << " command_id=" << int(feedback->gripperActionId())
                    << " feedback_id=" << int(fb.gripper_action_id) << " feedback_valid=" << fb.valid;
            }
            const auto signature = stopped ? stop_detail.str() : std::string("moving");
            if (signature != last_stop_signature) {
                async_log.submit("[CONTROL_CHANGE] timestamp_us=" + std::to_string(input.now_us)
                    + " " + stop_detail.str() + "\n");
                last_stop_signature = signature;
            }

            ++rate_frames;
            const auto rate_now = Clock::now();
            const double rate_seconds = std::chrono::duration<double>(rate_now - rate_started).count();
            if (rate_seconds >= 1.0) {
                loop_fps = rate_frames / rate_seconds;
                rate_frames = 0; rate_started = rate_now;
            }
            if (now - last_report >= Ms(500)) {
                std::ostringstream log;
                if(config.controlled_empty_field) log<<(config.assume_all_safe?"[TEMP_ASSUMPTION] assume_all_safe=1 ":"")<<"[CONTROLLED_TEST] clearance=" << (config.controlled_ignore_clearance?"DISABLED_CONTACT_TEST":"operator_confirmed") << " identity=own_zone_only "
                    <<"inventory="<<controlled_field.reason()<<" translation_watchdog=disabled phase_budgets=enabled\n";
                for(const auto& d:detections) log<<"[TARGET] id="<<d.track_id<<" label="<<d.label
                    <<" position="<<d.ground_position_valid<<" contact="<<d.ground_contact_valid
                    <<" reason="<<d.ground_contact_reason<<" xy="<<d.body_xy_m.x<<","<<d.body_xy_m.y
                    <<" conf="<<d.confidence<<" box="<<d.box.x<<","<<d.box.y<<","<<d.box.width<<","<<d.box.height<<"\n";
                {
                    const auto ids=[&](const char* key,const std::vector<int>& v){
                        log<<" "<<key<<"=";
                        if(v.empty()) log<<"-";
                        for(size_t i=0;i<v.size();++i) log<<(i?",":"")<<v[i];
                    };
                    const auto& hd=capture.diagnostics();
                    log<<"[HOLD] observable="<<input.hold_observable<<" captured="<<input.captured
                       <<" complete="<<input.held_complete<<" held=o"<<input.held.ordinary<<"/c"<<input.held.core
                       <<"/i"<<input.held.injured<<"/d"<<input.held.dangerous<<"/u"<<input.held.unknown;
                    ids("ambiguous",hd.ambiguous);ids("outside",hd.outside);ids("unfollowed",hd.unfollowed);
                    log<<"\n";
                }
                report(out, log);
                const bool range_valid = input.target_valid && input.geometry_valid;
                const float target_y = range_valid ? input.distance_m * std::cos(input.heading_error)
                    : std::numeric_limits<float>::quiet_NaN();
                log << "[GRASP_GEOMETRY] target_valid=" << range_valid
                    << " body_range_m=" << (range_valid ? input.distance_m : std::numeric_limits<float>::quiet_NaN())
                    << " body_forward_m=" << target_y << " front_y_m=" << tuning.closed_front_y_m
                    << " front_gap_m=" << target_y-tuning.closed_front_y_m
                    << " open_prepare_range_m=" << tuning.rush_start_m
                    << " close_trigger_y_m=" << tuning.grasp_trigger_y_m
                    << " open_distance_ok=" << (range_valid && input.distance_m<=tuning.rush_start_m)
                    << " open_heading_ok=" << (range_valid && std::abs(input.heading_error)<=tuning.rush_heading_rad)
                    << " path=" << input.path_safe << " gripper_command=" << int(out.motion.gripper_offset) << "\n";
                log<<"[MATCH] state="<<MatchControl::name(match_status.state)<<" reason="<<match_status.reason
                         <<" remaining_ms="<<match_status.remaining_us/1000<<" preflight="<<preflight_reason<<"\n";
                log<<"[EVIDENCE] identity="<<input.zone_identity_verified<<" target_region="<<input.target_region_valid
                         <<" color="<<zone_color.color<<" color_reason="<<zone_color.reason<<" path="<<input.path_safe<<" opponent_clear="<<input.opponent_zone_clear
                         <<" retreat="<<input.retreat_safe<<" zone_counts="<<input.zone_counts_valid<<"\n";
                if(geometry)log<<"[GEOMETRY] mapped="<<geometry_result.mapping_valid
                    <<" zone_valid="<<input.zone_estimate.trusted(input.now_us)<<" reason="<<geometry_result.reason
                    <<" inliers="<<input.zone_estimate.inlier_ids.size()<<" residual_m="<<input.zone_estimate.residual_m<<"\n";
                log << "[PERF] loop_fps=" << loop_fps << " inference_ms=" << inference_ms
                          << " detect_ms=" << vision_result.detect_ms << " pose_ms=" << vision_result.pose_ms
                          << " zone_points=" << zone_points.size()
                          << " capture_ms=" << capture_ms << " detections=" << detections.size() << "\n";
                if(vision_result.pose_ran) {
                    static const char *kn[]={"fl","nl","fr","nr"};
                    log<<"[ZONEPTS]";
                    for(int c=0;c<2;++c) {
                        const auto &h=vision_result.halves[c];
                        log<<(c?" R":" L")<<"="<<h.valid;
                        if(h.valid)for(int k=0;k<4;++k)log<<" "<<kn[k]<<"="<<int(h.keypoints[k].x)<<","<<int(h.keypoints[k].y)<<"@"<<int(h.confidence[k]*100);
                    }
                    log<<"\n";
                }
                if (imu) {
                    const auto& p = imu_data.sample;
                    log << "[IMU] fresh=" << imu_data.fresh << " age_ms=" << imu_data.age_ms
                              << " numeric_ok=" << p.measurements_valid << " seq=" << p.sequence
                              << " body_rpy_rad=" << p.body_rpy_rad[0] << "," << p.body_rpy_rad[1] << "," << p.body_rpy_rad[2]
                              << " status=" << p.status << " crc_errors=" << imu_data.crc_errors
                              << " sensor_valid=" << sensor.imu_valid << " estop=" << fusion.emergencyStop(observed_at) << "\n";
                }
                if (link) {
                    static const char *ack_names[] = {"idle", "no_feedback", "not_done", "done"};
                    const auto st = link->stats();
                    const auto fb = feedback->latestActuatorFeedback();
                    log << "[MCU] healthy=" << link->healthy() << " sent=" << st.sent << " failed=" << st.failed
                              << " stale_zero=" << st.stale << " gripper_id=" << int(feedback->gripperActionId())
                              << " target=" << gripper_ack.target << " ack=" << ack_names[int(gripper_ack.result)]
                              << " fb_valid=" << fb.valid << " fb_id=" << int(fb.gripper_action_id)
                              << " fb_open=" << int(fb.gripper_open) << " pitch_rb=" << fb.camera_pitch_cdeg
                              << " camera_protocol_readback_deg=" << (fb.camera_pitch_cdeg == kCameraPitchInvalid ? kCameraPitchInvalid : cameraPitchToFeedbackDeg(fb.camera_pitch_cdeg)) << "\n";
                }
                log << "[LOG] dropped=" << async_log.dropped() << "\n";
                async_log.submit(log.str());
                last_report = now;
            }
            vision.drawDetections(frame, detections);
            drawZone(frame, zone_points);
            const char *output_label = !link ? "PREVIEW - NO MCU LINK" :
                out.hardware_output_enabled ? "MCU LINK OPEN - MOTION PERMITTED" :
                "MCU LINK OPEN - MOTION BLOCKED";
            const cv::Scalar output_color = !link ? cv::Scalar(0,255,255) :
                out.hardware_output_enabled ? cv::Scalar(0,0,255) : cv::Scalar(0,190,255);
            cv::putText(frame, output_label, {10,25}, cv::FONT_HERSHEY_SIMPLEX, 0.6, output_color, 2);
#ifdef RESCUE_ENABLE_TELEMETRY
            if (telemetry) {
                DecisionTelemetry decision;
                decision.preflight_reason=preflight_reason;
                decision.zone_color=zone_color.color;
                decision.zone_color_reason=zone_color.reason;
                decision.geometry_reason=geometry_result.reason;
                decision.pose_model_ran=vision_result.pose_ran;
                telemetry->submit(frame, detections, input, out, epoch_ns,
                                  frame_sequence, loop_fps, inference_ms, capture_ms, imu_data, decision);
            }
#endif
            if(config.demo_mode!="none" && (out.state==PushState::SAFE_STOP || match_status.state==MatchState::FAULT || match_status.state==MatchState::FINISHED)) {
                const bool success=out.reason=="demo_target_found"||out.reason=="demo_delivery_complete"||
                    (config.assume_all_safe&&out.reason=="demo_next_trip_ready");
                std::cout<<"[DEMO_RESULT] "<<out.reason<<" match="<<match_status.reason<<"\n";
                demo_exit=success?0:2;match.command(MatchCommand::FINISH,imuNowUs());break;
            }
            if (writer.isOpened()) writer.write(frame);
            if (config.show) {
                cv::imshow("Ground pushing", frame);
                const int key = cv::waitKey(1);
                if (key == 27 || key == 'q' || key == 'Q') break;
            }
        }
        if (link) { link->stop(); std::cout << "[MCU] stopped: final zero-velocity packet sent\n"; }
        return demo_exit;
    } catch (const std::exception &e) {
        std::cerr << "[FATAL] " << e.what() << "\n";
        return 1;
    }
}
