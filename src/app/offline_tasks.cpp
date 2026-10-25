#include "rescue/app/offline_tasks.hpp"
#include "rescue/app/diagnostics.hpp"
#include "rescue/app/perception_pipeline.hpp"
#include "rescue/config.hpp"
#include "rescue/geometry_pipeline.hpp"
#include "rescue/zone_color.hpp"
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace rescue {
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
    flag("zone_valid", in.zone_valid); flag("zone_own", in.zone_own); flag("enemy_zone_seen", in.enemy_zone_seen);
    flag("zone_sight_valid",in.zone_sight_valid);flag("zone_sight_enemy",in.zone_sight_enemy);if(!n["zone_sight_bearing"].empty())n["zone_sight_bearing"]>>in.zone_sight_bearing;
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
} // namespace

std::optional<int> runOfflineTask(const Config &config, const TaskTuning &tuning) {
    if (!config.geometry_replay.empty())
        return runGeometryReplay(config.geometry_replay,config.calibration_file,config.zone_geometry_file,config.team);
    if (!config.push_replay.empty()) {
        PushTask task(tuning);
        cv::FileStorage replay(config.push_replay, cv::FileStorage::READ);
        if (!replay.isOpened() || !replay["frames"].isSeq())
            throw std::runtime_error("Replay must contain a frames array");
        for (const auto &frame : replay["frames"]) report(task.update(readObservation(frame)));
        return 0;
    }
    if (!config.detect_image.empty()) {
        PerceptionPipeline vision_frontend(config);
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
    return std::nullopt;
}
} // namespace rescue
