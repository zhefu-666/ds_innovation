#include "../../support/push_task_sim.hpp"
#include "push_task_cases.hpp"

// Regenerates tests/fixtures/push_delivery.json from the closed-loop simulator, so the
// replay inputs (zone pose, gripper acknowledgements, heading) match the task's own commands.
void writeFixture(const std::string& path) {
    Sim s;
    s.record = true;
    s.toCarry();
    s.deliver();
    s.in.target_valid = false;
    s.in.target_id = -1;
    s.in.corridor = {}; // nothing left in view
    assert(s.until(PushState::SCAN, 400));
    for (int i = 0; i < 3; ++i)
        s.tick();
    assert(s.out.state == PushState::SCAN && s.out.delivered_total == 1 &&
           s.out.first_ordinary_delivered);
    cv::FileStorage f(path, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    assert(f.isOpened());
    const auto inventory = [&](const char* key, const Inventory& i) {
        f << key << "{" << "ordinary" << i.ordinary << "core" << i.core << "injured" << i.injured
          << "dangerous" << i.dangerous << "unknown" << i.unknown << "}";
    };
    f << "frames" << "[";
    for (const auto& in : s.frames) {
        f << "{" << "now_us" << double(in.now_us) << "run" << int(in.run) << "safety_ok"
          << int(in.safety_ok) << "carry_plan_valid" << int(in.carry_plan_valid)
          << "drop_plan_valid" << int(in.drop_plan_valid) << "navigation_timestamp_us"
          << double(in.navigation_timestamp_us) << "carry_waypoint_body" << "["
          << in.carry_waypoint_body.x << in.carry_waypoint_body.y << "]"
          << "drop_centre_zone" << "[" << in.drop_centre_zone.x << in.drop_centre_zone.y << "]"
          << "target_region_valid" << int(in.target_region_valid) << "zone_identity_verified"
          << int(in.zone_identity_verified) << "delivery_observed" << int(in.delivery_observed)
          << "target_valid" << int(in.target_valid) << "geometry_valid" << int(in.geometry_valid)
          << "path_safe" << int(in.path_safe) << "retreat_safe" << int(in.retreat_safe)
          << "opponent_zone_clear" << int(in.opponent_zone_clear) << "target_id" << in.target_id
          << "label" << in.label << "distance_m" << double(in.distance_m) << "heading_error"
          << double(in.heading_error);
        inventory("corridor", in.corridor);
        f << "corridor_complete" << int(in.corridor_complete) << "corridor_occlusion_free"
          << int(in.corridor_occlusion_free) << "hold_observable" << int(in.hold_observable)
          << "captured" << int(in.captured) << "held_complete" << int(in.held_complete)
          << "camera_pitch_cdeg" << int(in.camera_pitch_cdeg) << "camera_pitch_stable"
          << int(in.camera_pitch_stable);
        inventory("held", in.held);
        f << "gripper_done" << int(in.gripper_done) << "gripper_feedback_open"
          << in.gripper_feedback_open << "zone_valid" << int(in.zone_valid) << "zone_own"
          << int(in.zone_own) << "zone_class" << in.zone_class << "zone_inventory_complete"
          << int(in.zone_inventory_complete) << "zone_counts_valid" << int(in.zone_counts_valid)
          << "zone_supply_count" << in.zone_supply_count << "zone_injured_count"
          << in.zone_injured_count << "heading_valid" << int(in.heading_valid) << "heading_rad"
          << double(in.heading_rad);
        const auto& z = in.zone_estimate;
        f << "zone_estimate" << "{" << "valid" << int(z.valid) << "source" << "multi_point"
          << "zone_label" << z.zone_label << "geometry_id" << z.geometry_id << "frame_id"
          << double(z.frame_id) << "timestamp_us" << double(z.timestamp_us) << "observed_us"
          << double(z.observed_us) << "origin_body_m" << "[" << double(z.origin_body_m.x)
          << double(z.origin_body_m.y) << "]"
          << "yaw_body_rad" << double(z.yaw_body_rad) << "inlier_ids" << "[";
        for (int id : z.inlier_ids)
            f << id;
        f << "]" << "residual_m" << double(z.residual_m) << "position_sigma_m"
          << double(z.position_sigma_m) << "yaw_sigma_rad" << double(z.yaw_sigma_rad)
          << "predicted_distance_m" << double(z.predicted_distance_m) << "}" << "}";
    }
    f << "]";
    std::cout << "wrote " << s.frames.size() << " frames to " << path << "\n";
}
