#include "rescue/app/runtime_config.hpp"
#include "rescue/config/file_reference.hpp"
#include "rescue/config/demo_policy.hpp"
#include "rescue/config/task_calibration.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <type_traits>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

namespace rescue {
namespace {

template <class Visit> void visitTask(const TaskTuning& t, Visit visit) {
#define F(name) visit(#name, t.name)
    F(require_multi_view);
    F(pixel_grasp_enabled);
    F(pixel_align_gain_rps_per_px);
    F(pixel_align_max_wz);
    F(pixel_align_tolerance_px);
    F(pixel_align_restart_px);
    F(pixel_align_approach_tolerance_px);
    F(pixel_align_approach_restart_px);
    F(pixel_align_brake_us);
    F(pixel_align_brake_motion_px);
    F(pixel_align_stopped_wz);
    F(pixel_align_stopped_vx);
    F(pixel_align_stable_us);
    F(pixel_align_stable_frames);
    F(demo_search_only);
    F(demo_carry_once);
    F(assume_all_safe);
    F(field_carry);
    F(field_rush);
    F(verify_held_contents);
    F(rebind_max_shift_m);
    F(assume_injured_trip);
    F(assume_push_m);
    F(assume_motion_ratio);
    F(assume_rush_trigger_m);
    F(assume_carry_m);
    F(assume_back_m);
    F(assume_zone_search_us);
    F(assume_prepush_back_m);
    F(assume_align_zone_m);
    F(assume_dr_pessimism);
    F(assume_view_pitch_cdeg);
    F(assume_cargo_in_y_m);
    F(assume_cargo_done_y_m);
    F(assume_cargo_x_tol_m);
    F(assume_overshoot_m);
    F(assume_stuck_move_m);
    F(assume_stuck_us);
    F(assume_repush_max);
    F(approach_rebind_window_us);
    F(assume_repush_back_m);
    F(assume_repush_speed_gain);
    F(assume_lateral_back_gain);
    F(assume_lateral_back_max_m);
    F(assume_goal_depth_m);
    F(assume_push_angle_max_rad);
    F(assume_push_dx_deadband_m);
    F(assume_release_gap_m);
    F(assume_push_depth_m);
    F(assume_ratio_max);
    F(assume_lookahead_m);
    F(assume_stall_near_m);
    F(assume_stall_progress_m);
    F(assume_stall_us);
    F(assume_enter_confirm);
    F(assume_enter_true_y_m);
    F(assume_enter_hold_y_m);
    F(assume_enter_max_hold_y_m);
    F(assume_enter_extra_us);
    F(assume_imu_block_us);
    F(assume_imu_block_near_m);
    F(assume_inside_after_release_obs);
    F(assume_cargo_verify_in_y_m);
    F(assume_cargo_verify_out_y_m);
    F(assume_verify_out_obs);
    F(assume_img_inside_frac);
    F(assume_img_outside_frac);
    F(assume_img_verify_obs);
    F(sweep_upright);
    F(sweep_corridor_half_m);
    F(sweep_extra_m);
    F(sweep_min_sep_m);
    F(sweep_turn_rad);
    F(sweep_return_tol_rad);
    F(sweep_confirm);
    F(sweep_max_attempts);
    F(sweep_wz);
    F(sweep_budget_us);
    F(sweep_fresh_us);
    F(sweep_creep);
    F(sweep_creep_speed);
    F(sweep_creep_max_m);
    F(sweep_frame_front_m);
    F(sweep_creep_gap_m);
    F(sweep_creep_budget_us);
    F(disperse);
    F(disperse_dense_min);
    F(disperse_swings);
    F(disperse_passes);
    F(disperse_radius_m);
    F(disperse_speed);
    F(disperse_depth_m);
    F(disperse_max_m);
    F(disperse_wz);
    F(disperse_turn_rad);
    F(disperse_swing_vx);
    F(disperse_swing_creep_m);
    F(disperse_swing_tol_rad);
    F(disperse_pass_extra_m);
    F(disperse_rush_budget_us);
    F(disperse_swing_budget_us);
    F(disperse_cooldown_us);
    F(sweep_gap_us);
    F(sweep_time_budget_us);
    F(sweep_total_max);
    F(sweep_escalate_max);
    F(sweep_op_budget_us);
    F(sweep_chain_link_us);
    F(sweep_match_budget_us);
    F(sweep_confirm_wait_us);
    F(lost_search_rescan);
    F(blind_scatter);
    F(blind_scatter_need_ordinary);
    F(blind_scatter_min);
    F(blind_scatter_max);
    F(blind_scatter_passes);
    F(blind_scatter_swings);
    F(blind_scatter_min_y_m);
    F(blind_scatter_max_y_m);
    F(blind_scatter_half_x_m);
    F(blind_scatter_depth_m);
    F(blind_scatter_max_m);
    F(blind_scatter_align_tol_rad);
    F(blind_scatter_far_radius_m);
    F(blind_scatter_far_chase_m);
    F(blind_scatter_far_min);
    F(blind_scatter_far_lock_min);
    F(blind_scatter_far_max_y_m);
    F(blind_scatter_approach_stop_m);
    F(blind_scatter_approach_max);
    F(blind_scatter_approach_budget_us);
    F(blind_scatter_wait_us);
    F(blind_scatter_align_budget_us);
    F(blind_scatter_budget_us);
    F(blind_scatter_cooldown_us);
    F(blind_scatter_align_wz);
    F(blind_scatter_max_bearing_rad);
    F(blind_scatter_first_turn_rad);
    F(blind_scatter_return);
    F(blind_scatter_back_frac);
    F(blind_scatter_back_speed);
    F(blind_scatter_back_budget_us);
    F(first_burst_full_impact);
    F(first_burst);
    F(first_burst_speed);
    F(first_burst_slow_m);
    F(first_burst_min_drive_m);
    F(first_burst_swing_wz);
    F(post_clear_mid_view);
    F(sweep_view_handoff);
    F(sweep_view_cap_mid);
    F(sweep_far_judge);
    F(swing_avoid_outside);
    F(swing_avoid_radius_m);
    F(swing_avoid_margin_rad);
    F(approach_dense_burst_swing);
    F(relock_window_us);
    F(relock_gate_frac);
    F(lock_max_visible);
    F(lock_multi_group_min);
    F(lock_multi_radius_m);
    F(dense_multi_ignore_gap);
    F(disperse_single_turn);
    F(disperse_turn_min_rad);
    F(disperse_turn_view_range_m);
    F(disperse_turn_view_half_rad);
    F(disperse_turn_clear_max);
    F(disperse_turn_clear_frames);
    F(disperse_turn_look_us);
    F(row_select);
    F(row_min_n);
    F(row_resid_m);
    F(row_max_tilt_rad);
    F(row_half_along_m);
    F(row_half_across_m);
    F(disperse_turn_push_m);
    F(disperse_turn_push_budget_us);
    F(disperse_turn_frame_check);
    F(disperse_turn_frame_max);
    F(disperse_turn_frame_range_m);
    F(disperse_turn_frame_half_m);
    F(disperse_turn_frame_extra_rad);
    F(occlusion_hold_us);
    F(occlusion_near_m);
    F(pile_clear);
    F(pile_corridor_min);
    F(pile_confirm);
    F(pile_max_runs);
    F(pile_trigger_max_m);
    F(pile_budget_us);
    F(pile_cool_us);
    F(pile_advance_m);
    F(pile_advance_speed);
    F(pile_turn_wz);
    F(pile_align_tol_rad);
    F(pile_block_half_m);
    F(pile_advance_budget_us);
    F(pile_back_budget_us);
    F(pile_cluster_radius_m);
    F(pile_cluster_half_x_m);
    F(pile_view_back_m);
    F(pile_frame_half_m);
    F(pile_cut_depth_m);
    F(pile_cut_speed);
    F(pile_measure_us);
    F(pile_cut_budget_us);
    F(pile_cut_max_steps);
    F(pile_chain);
    F(pile_chain_max);
    F(pile_layer_depth_m);
    F(pile_chain_rush_m);
    F(pile_chain_back_frac);
    F(pile_chain_min_back_m);
    F(dense_no_lock);
    F(sweep_non_supply);
    F(mid_neighbour_clear);
    F(mid_neighbour_clear_max);
    F(mid_neighbour_clear_swings);
    F(rush_stall_us);
    F(rush_stall_gap_us);
    F(rush_stall_reobserve_us);
    F(rush_stall_reobserve_max_us);
    F(rush_stall_progress_m);
    F(rush_stall_min_samples);
    F(rush_stall_reobserve_frames);
    F(precise_rush);
    F(sweep_observation_guard);
    F(demo_capture_motion);
    F(rush_align_deadband_rad);
    F(rush_align_wz);
    F(rush_near_speed);
    F(rush_slow_range_m);
    F(sweep_reobserve_us);
    F(frame_open_default);
    F(sweep_frame_open);
    F(held_recheck);
    F(held_recheck_confirm_frames);
    F(held_recheck_strong_frames);
    F(held_recheck_clear_frames);
    F(held_recheck_strong_conf);
    F(held_recheck_us);
    F(corridor_precheck);
    F(corridor_precheck_confirm);
    F(corridor_precheck_max);
    F(mid_neighbour_clear_turn_rad);
    F(mid_neighbour_clear_gap_m);
    F(mid_neighbour_clear_max_rush_m);
    F(mid_neighbour_clear_creep_m);
    F(weak_target_scatter);
    F(weak_pixel_ratio);
    F(weak_scatter_wait_us);
    F(clear_neighbours);
    F(clear_half_m);
    F(clear_front_gap_m);
    F(rush_pulse_turn);
    F(rush_pulse_wz);
    F(rush_pulse_max_rad);
    F(rush_pulse_done_frac);
    F(rush_pulse_budget_us);
    F(rush_pulse_settle_us);
    F(rush_pulse_max);
    F(assume_probe);
    F(assume_probe_first_us);
    F(assume_probe_len_us);
    F(assume_probe_gap_us);
    F(assume_verify_fix_wait_us);
    F(assume_verify_far_after_us);
    F(assume_inside_min_hold_y_m);
    F(assume_blocked_deep_hold_y_m);
    F(assume_align_push);
    F(assume_align_tol_rad);
    F(assume_align_wz);
    F(assume_slant_max_rad);
    F(assume_align_budget_us);
    F(attempt_budget_us);
    F(approach_limit_m);
    F(retreat_limit_m);
    F(enable_search_cues);
    F(enable_short_push);
    F(enable_lost_search);
    F(cue_approach_speed);
    F(cue_prepare_distance_m);
    F(clear_speed);
    F(clear_extra_m);
    F(clear_max_commanded_m);
    F(closed_front_y_m);
    F(clear_heading_limit_rad);
    F(max_clear_attempts);
    F(cue_approach_budget_us);
    F(clear_wall_budget_us);
    F(clear_pause_us);
    F(approach_speed);
    F(rush_speed);
    F(carry_speed);
    F(enter_speed);
    F(back_speed);
    F(max_speed);
    F(min_turn_wz);
    F(heading_gain);
    F(lateral_gain);
    F(heading_deadband_rad);
    F(scan_wz);
    F(turn_wz);
    F(max_wz);
    F(rotate_in_place_rad);
    F(visual_chase);
    F(visual_chase_frames);
    F(visual_chase_miss_frames);
    F(visual_chase_min_conf);
    F(visual_chase_speed);
    F(visual_chase_budget_us);
    F(visual_chase_cool_us);
    F(visual_chase_hold_us);
    F(scan_sighting_hold);
    F(scan_hold_max_us);
    F(scan_hold_lost_us);
    F(scan_hold_cool_us);
    F(inplace_turn_boost);
    F(startup_advance_speed);
    F(startup_hold_gain);
    F(startup_hold_deadband_rad);
    F(startup_advance_us);
    F(spin_unstick);
    F(spin_stall_trigger_us);
    F(spin_unstick_leg_us);
    F(spin_unstick_speed);
    F(rush_yaw_hold);
    F(rush_lower_yaw_tol_rad);
    F(rush_realign_wz);
    F(prelower_realign_us);
    F(rush_start_m);
    F(rush_heading_rad);
    F(rush_extra_m);
    F(rush_abort_heading_rad);
    F(grasp_trigger_y_m);
    F(hold_center_y_m);
    F(injured_hold_center_y_m);
    F(mouth_y_m);
    F(supply_half_x_m);
    F(injured_half_x_m);
    F(gate_clearance_m);
    F(gate_tolerance_m);
    F(gate_yaw_rad);
    F(gate_lateral_m);
    F(deposit_y_m);
    F(enter_lateral_m);
    F(mouth_clear_y_m);
    F(abort_back_m);
    F(blind_retreat_wait_us);
    F(blind_retreat_yaw_rad);
    F(corridor_confirm_frames);
    F(corridor_retry_max);
    F(corridor_back_m);
    F(turn_min_rad);
    F(confirm_frames);
    F(capture_frames);
    F(delivery_frames);
    F(hold_grace_frames);
    F(frame_timeout_us);
    F(gripper_timeout_us);
    F(approach_budget_us);
    F(prepare_budget_us);
    F(rush_budget_us);
    F(verify_budget_us);
    F(carry_budget_us);
    F(gate_budget_us);
    F(carry_search_turns);
    F(carry_search_respin_turns);
    F(carry_leg_us);
    F(carry_leg_turn_cap_us);
    F(carry_spin_cap_us);
    F(carry_sight_fresh_us);
    F(carry_sight_mem_us);
    F(zone_search_budget_us);
    F(enemy_evade);
    F(enemy_evade_frames);
    F(enemy_evade_turn_rad);
    F(enemy_evade_m);
    F(enemy_evade_speed);
    F(enemy_evade_turn_budget_us);
    F(enemy_evade_drive_budget_us);
    F(enemy_evade_cool_us);
    F(enter_budget_us);
    F(retreat_budget_us);
    F(delivery_budget_us);
    F(turn_budget_us);
    F(blacklist_us);
    F(far_pitch_cdeg);
    F(track_pitch_cdeg);
    F(near_pitch_cdeg);
    F(pitch_tolerance_cdeg);
    F(intermediate_pitch_cdeg);
    F(intermediate_to_near_m);
    F(approach_stall_us);
    F(approach_stall_progress_m);
    F(approach_stall_accept_m);
    F(open_jaw_handoff_m);
    F(track_near_m);
    F(hold_check_m);
    F(hold_check_frames);
    F(hold_check_interval_us);
    F(pitch_timeout_us);
#undef F
}

template <class Visit> void visitCapture(const CaptureConfig& c, Visit visit) {
#define F(name) visit(#name, c.name)
    F(pitch_tolerance_cdeg);
    F(edge_margin_px);
    F(pixel_align_tolerance_px);
    F(follow_tolerance_px);
    F(follow_frames);
    F(mouth_y_m);
    F(outside_margin_m);
    F(max_age_us);
    F(min_confidence);
    F(noise_confidence);
    F(corridor_half_width_m);
    F(corridor_extra_m);
    F(footprint_ordinary_m);
    F(footprint_dangerous_m);
    F(footprint_core_m);
    F(footprint_injured_m);
    F(footprint_unknown_m);
    F(occlusion_side_margin_m);
    F(occlusion_iou);
    F(image_height_px);
#undef F
}

template <class Visit> void visitGround(const GroundContactConfig& c, Visit visit) {
#define F(name) visit(#name, c.name)
    F(min_confidence);
    F(edge_margin_px);
    F(max_range_m);
    F(size_tolerance);
    F(accept_size_mismatch);
    F(injured_lying_only);
    F(upright_margin);
    F(upright_vote_frames);
    F(upright_vote_min);
    F(upright_vote_window_us);
    F(upright_vote_jump_m);
    F(upright_release_ratio);
    F(lying_confirm_diff);
    F(use_silhouette);
    F(lying_confirm_hysteresis);
#undef F
    visit("silhouette_lying_below", c.silhouette.lying_below);
    visit("silhouette_upright_above", c.silhouette.upright_above);
    visit("silhouette_lying_diff", c.silhouette.lying_diff);
    visit("silhouette_upright_diff", c.silhouette.upright_diff);
}

template <class Visit> void visitRequest(const Config& c, Visit visit) {
#define F(name) visit(#name, c.name)
    F(demo_mode);
    F(demo_seconds);
    F(push_replay);
    F(geometry_replay);
    F(calibration_file);
    F(keypoints_file);
    F(zone_geometry_file);
    F(pitch_feedback);
    F(detect_image);
    F(rknn_library);
    F(model_path);
    F(pose_model_path);
    F(pose_model_blue_path);
    F(pose_model_red_path);
    F(pose_confidence);
    F(pose_keypoint_confidence);
    F(detect_core_mask);
    F(pose_core_mask);
    F(parallel_inference);
    F(uart_port);
    F(team);
    F(baudrate);
    F(camera_index);
    F(frame_width);
    F(frame_height);
    F(fps);
    F(detect_interval);
    F(input_size);
    F(confidence);
    F(injured_confidence);
    F(nms);
    F(use_cuda);
    F(show);
    F(save_output);
    F(dry_run);
    F(controlled_ignore_clearance);
    F(clear_neighbours);
    F(search_cues);
    F(controlled_empty_field);
    F(assume_all_safe);
    F(assume_injured_trip);
    F(check_config);
    F(hardware);
    F(imu);
    F(imu_port);
    F(imu_baud);
    F(imu_timeout_ms);
    F(telemetry);
    F(telemetry_fps);
    F(telemetry_file);
    F(auto_run);
    F(step_phases);
    F(single_phase);
    F(allow_mechanical_pitch_model);
    F(known_frame_angle);
    F(known_frame_id);
    F(no_match_time_limit);
    F(match_seconds);
    F(startup_advance_ms);
    F(startup_speed_mps);
    F(scan_wz_rps);
    F(turn_wz_rps);
    F(task_calibration_file);
    F(zone_color_file);
    F(vision_log_dir);
    F(match_socket);
    F(require_instance_masks);
    F(sensor_timeout_ms);
    F(tracker_max_age_ms);
    F(tof_stop_distance_m);
    F(imu_tilt_limit_deg);
    F(pnp_max_reprojection_error_px);
    F(pnp_jump_limit_m);
#undef F
}

void mark(ResolvedRuntimeConfig& r,
          const std::string& source,
          std::initializer_list<const char*> fields) {
    for (const auto* field : fields)
        r.field_sources[field].push_back(source);
}

void readCalibrationAudit(ResolvedRuntimeConfig& r) {
    try {
        const auto reference = splitFileReference(r.request.task_calibration_file);
        cv::FileStorage f(reference.file, cv::FileStorage::READ);
        if (!f.isOpened()) {
            r.calibration_audit.audit_error = "metadata_unavailable";
            return;
        }
        const auto root = selectFileNode(f, reference.fragment);
        auto& a = r.calibration_audit;
        a.schema_version = int(root["schema_version"]);
        for (const auto& node : root)
            a.provided_fields.push_back(node.name());
        const auto text = [&](const char* key, std::string& value) {
            if (root[key].isString())
                root[key] >> value;
        };
        text("version", a.version);
        text("status", a.status);
        text("dimensions_source", a.dimensions_source);
        text("assume_source", a.assume_source);
        text("action_mapping_source", a.action_mapping_source);
        text("a6_done_settle_source", a.a6_done_settle_source);
        text("frame_view_matching_source", a.frame_view_matching_source);
        if (root["pending"].isSeq())
            for (const auto& value : root["pending"])
                if (value.isString())
                    a.pending.push_back(std::string(value));
        for (const char* key : {"measured",
                                "coordinate_transform_verified",
                                "action_mapping_verified",
                                "capture_stop_verified",
                                "body_envelope_verified",
                                "visual_acceptance_passed",
                                "box_area_accepted"})
            if (root[key].isInt())
                a.verification_flags[key] = int(root[key]);
    } catch (const cv::Exception& e) {
        r.calibration_audit.audit_error = e.what();
    }
}

void addAssumptions(ResolvedRuntimeConfig& r) {
    const auto& c = r.request;
    const auto add = [&](const char* id,
                         const char* scope,
                         const char* replacement,
                         const std::string& evidence) {
        r.assumptions.push_back({id, scope, replacement, evidence, "not_field_validated"});
    };
    if (r.task.field_carry)
        add("legacy_field_transport",
            "carry_release_delivery",
            "Use field transport, bounded blind legs and field verification fallback",
            "legacy_field_v1; former main.cpp field overrides");
    if (r.task.field_rush)
        add("legacy_field_rush",
            "approach_capture",
            "Scaled motion limit, blind finish below camera view and target re-identification",
            "legacy_field_v1; former main.cpp field overrides");
    if (c.controlled_empty_field)
        add("operator_cleared_field",
            "route_identity_progress",
            "Own-zone-only cleared field supplies route/identity context; measured-progress "
            "watchdog disabled",
            "--controlled-empty-field operator attestation");
    if (c.controlled_ignore_clearance)
        add("clearance_disabled",
            "travel_turn_jaw",
            "Bypass controlled-field collision and clearance checks",
            "--controlled-ignore-clearance or --assume-all-safe");
    if (c.assume_all_safe)
        add("all_evidence_assumed",
            "capture_zone_clearance_delivery",
            "Treat missing safety, zone, capture and delivery evidence as true",
            "--assume-all-safe; bounded carry_once; NOT_ACCEPTANCE");
    if (c.assume_injured_trip)
        add("first_delivery_assumed",
            "task_history_target_priority",
            "Start with first ordinary supply delivered and prefer injured_person",
            "--assume-injured-trip");
    if (c.allow_mechanical_pitch_model)
        add("mechanical_pitch_model",
            "ground_mapping",
            "Allow unvalidated mechanical pitch model",
            c.calibration_file);
    if (r.a6_done_flag)
        add("a6_done_flag",
            "frame_feedback",
            "Matching action id and done bit imply the last commanded frame target after an "
            "estimated settle period; position is not measured",
            r.calibration_audit.a6_done_settle_source);
    if (r.view_image_polygon)
        add("image_polygon_holding",
            "multi_view_capture",
            "Compare class counts inside per-pitch image polygons without ground ranging inside "
            "the frame",
            r.calibration_audit.frame_view_matching_source);
    if (!c.calibration_file.empty() && r.ground_contact.accept_size_mismatch)
        add("ground_size_mismatch",
            "ground_contact",
            "Accept box-bottom ground contact despite an unvalidated object-size model",
            "legacy main ground-contact configuration");
    const auto& a = r.calibration_audit;
    if (!a.status.empty() || !a.pending.empty() || !a.assume_source.empty())
        add("task_calibration_provenance",
            "mechanical_calibration",
            "Legacy structural verification flags do not establish field acceptance; retain status "
            "and pending evidence",
            a.status + "; " + a.assume_source);
    if (c.no_match_time_limit)
        add("match_time_limit_disabled",
            "match_duration",
            "Disable total match duration while phase and health gates remain",
            "--no-match-time-limit debugging override");
}

template <class T> void scalar(cv::FileStorage& f, const std::string& key, const T& value) {
    f << key;
    if constexpr (std::is_same_v<T, std::string>)
        f << value;
    else if constexpr (std::is_floating_point_v<T>) {
        if (std::isfinite(value))
            f << double(value);
        else
            f << (std::isnan(value) ? "NaN" : value > 0 ? "+infinity" : "-infinity");
    } else if constexpr (std::is_integral_v<T>) {
        if constexpr (std::is_same_v<T, bool>)
            f << int(value);
        else if constexpr (sizeof(T) < sizeof(uint64_t)) {
            if (double(value) <= double(std::numeric_limits<int>::max()) &&
                double(value) >= double(std::numeric_limits<int>::min()))
                f << int(value);
            else
                f << double(value);
        } else {
            if (value <= 9007199254740991ULL)
                f << double(value);
            else
                f << std::to_string(value);
        }
    }
}

void points(cv::FileStorage& f, const std::vector<cv::Point2f>& polygon) {
    f << "[";
    for (const auto& p : polygon)
        f << "[:" << double(p.x) << double(p.y) << "]";
    f << "]";
}

void area(cv::FileStorage& f, const char* key, const RectArea& a) {
    f << key << "[:" << double(a.x1) << double(a.y1) << double(a.x2) << double(a.y2) << "]";
}

} // namespace

ResolvedRuntimeConfig resolveLegacyRuntimeConfig(const Config& config, std::ostream& diagnostics) {
    ResolvedRuntimeConfig r;
    r.request = config;
    r.policy_name = config.push_replay.empty() ? "legacy_field_v1" : "legacy_replay_v1";
    visitTask(r.task, [&](const char* name, auto) {
        r.field_sources[std::string("task.") + name] = {"TaskTuning defaults"};
    });
    visitCapture(r.capture, [&](const char* name, auto) {
        r.field_sources[std::string("capture.") + name] = {"CaptureConfig defaults"};
    });
    visitGround(r.ground_contact, [&](const char* name, auto) {
        r.field_sources[std::string("ground_contact.") + name] = {"GroundContactConfig defaults"};
    });
    visitRequest(config, [&](const char* name, const auto&) {
        r.field_sources[std::string("request.") + name] = {
            "parsed Config: defaults, CLI and demo normalization"};
    });
    mark(r,
         "parsed Config: defaults, CLI and demo normalization",
         {"request.pitch_presets_cdeg",
          "request.class_names",
          "request.catch_area",
          "request.ready_area",
          "request.holding_area",
          "request.center_region",
          "request.cross_center_region",
          "request.catch_angle",
          "request.release_angle"});
    mark(r,
         "legacy unconfigured calibration defaults",
         {"frame_views",
          "feedback_open",
          "feedback_close",
          "box_area_accepted",
          "a6_done_flag",
          "a6_done_settle_ms",
          "view_image_polygon",
          "body_envelope",
          "measured_load_radius",
          "measured_body_radius",
          "load_half_width",
          "load_half_depth"});
    auto& tuning = r.task;
    tuning.far_pitch_cdeg = config.pitch_presets_cdeg[0];
    tuning.track_pitch_cdeg = config.pitch_presets_cdeg[1];
    tuning.near_pitch_cdeg = config.pitch_presets_cdeg[2];
    mark(r,
         "parsed Config pitch presets",
         {"task.far_pitch_cdeg", "task.track_pitch_cdeg", "task.near_pitch_cdeg"});
    if (config.allow_mechanical_pitch_model && tuning.track_pitch_cdeg > 2000) {
        tuning.intermediate_pitch_cdeg = 2000;
        tuning.track_near_m = .4f;
        tuning.intermediate_to_near_m = .27f;
        diagnostics
            << "PITCH_SWITCH mid20_distance_m=0.4 near40_distance_m=0.27 open_jaw_handoff_m="
            << tuning.open_jaw_handoff_m << "\n";
        mark(r,
             "legacy mechanical pitch handoff",
             {"task.intermediate_pitch_cdeg", "task.track_near_m", "task.intermediate_to_near_m"});
    }
    tuning.enable_search_cues = config.search_cues;
    tuning.clear_neighbours = config.clear_neighbours;
    tuning.enable_lost_search = true;
    tuning.enable_short_push = config.controlled_empty_field && config.search_cues;
    tuning.startup_advance_speed = config.startup_speed_mps;
    tuning.max_speed = std::max(tuning.max_speed, tuning.startup_advance_speed);
    tuning.scan_wz = config.scan_wz_rps;
    tuning.turn_wz = config.turn_wz_rps;
    tuning.max_wz = std::max({tuning.max_wz, tuning.scan_wz, tuning.turn_wz});
    tuning.startup_advance_us = uint64_t(config.startup_advance_ms) * 1000;
    mark(r,
         "legacy parsed Config mapping",
         {"task.enable_search_cues",
          "task.clear_neighbours",
          "task.enable_lost_search",
          "task.enable_short_push",
          "task.startup_advance_speed",
          "task.max_speed",
          "task.scan_wz",
          "task.turn_wz",
          "task.max_wz",
          "task.startup_advance_us"});
    if (config.controlled_empty_field) {
        tuning.carry_budget_us = 90000000;
        mark(r, "controlled field carry budget", {"task.carry_budget_us"});
    }
    auto& capture_config = r.capture;
    capture_config.holding.clear();
    capture_config.image_height_px = config.frame_height;
    mark(r,
         "legacy uncalibrated capture initialization",
         {"capture.holding", "capture.image_height_px"});
    r.task_calibrated = !config.task_calibration_file.empty();
    if (r.task_calibrated) {
        const auto measured = loadTaskCalibration(config.task_calibration_file,
                                                  tuning,
                                                  config.frame_width,
                                                  config.frame_height,
                                                  config.demo_mode == "search");
        r.frame_views = measured.frame_views;
        r.feedback_open = measured.feedback_open;
        r.feedback_close = measured.feedback_close;
        r.box_area_accepted = measured.box_area_accepted;
        r.a6_done_flag = measured.a6_done_flag;
        r.a6_done_settle_ms = measured.a6_done_settle_ms;
        r.view_image_polygon = measured.frame_view_image_polygon;
        r.body_envelope = measured.body_envelope;
        const auto cli_startup_us = tuning.startup_advance_us;
        const auto cli_startup_speed = tuning.startup_advance_speed;
        const auto cli_scan_wz = tuning.scan_wz;
        const auto cli_budget_us = tuning.attempt_budget_us;
        tuning = measured.task;
        tuning.startup_advance_us = cli_startup_us;
        tuning.startup_advance_speed = cli_startup_speed;
        tuning.scan_wz = cli_scan_wz;
        tuning.attempt_budget_us = cli_budget_us;
        capture_config = measured.capture;
        const auto task_ref = splitFileReference(config.task_calibration_file);
        cv::FileStorage task_file(task_ref.file, cv::FileStorage::READ);
        const auto task_root = selectFileNode(task_file, task_ref.fragment);
        std::string image_reference;
        task_root["image_trigger_reference"] >> image_reference;
        if (!image_reference.empty()) {
            const auto file = std::filesystem::path(task_ref.file).parent_path().parent_path() /
                              image_reference;
            std::ifstream stream(file);
            nlohmann::json reference;
            stream >> reference;
            if (reference.at("schema_version") != 3 || reference.at("feature") != "bbox_center_xy" ||
                reference.at("image_size") != nlohmann::json::array({config.frame_width, config.frame_height}))
                throw std::runtime_error("Invalid class grasp centre reference or image size");
            const int pitch = reference.at("camera_pitch_cdeg").get<int>();
            if (pitch < -3000 || pitch > 3000)
                throw std::runtime_error("Invalid class grasp reference pitch");
            capture_config.pixel_grasp_enabled = reference.value("runtime_enabled", false);
            if (reference.contains("pixel_alignment")) {
                const auto& alignment = reference.at("pixel_alignment");
                visitTask(tuning, [&](const char* name, auto value) {
                    if (std::string(name).rfind("pixel_align_", 0) != 0 || !alignment.contains(name))
                        return;
                    using T = decltype(value);
                    const auto raw = alignment.at(name).get<double>();
                    if (!std::isfinite(raw) || raw <= 0 ||
                        raw > double(std::numeric_limits<T>::max()))
                        throw std::runtime_error(std::string("Invalid pixel alignment parameter: ") + name);
                    if constexpr (std::is_integral_v<T>)
                        if (std::floor(raw) != raw)
                            throw std::runtime_error(std::string("Noninteger pixel alignment parameter: ") + name);
                    // visitTask supplies values; assign the approved fields explicitly below.
                });
#define ALIGN(name) tuning.name = alignment.value(#name, tuning.name)
                ALIGN(pixel_align_gain_rps_per_px);
                ALIGN(pixel_align_max_wz);
                ALIGN(pixel_align_tolerance_px);
                ALIGN(pixel_align_restart_px);
                ALIGN(pixel_align_approach_tolerance_px);
                ALIGN(pixel_align_approach_restart_px);
                ALIGN(pixel_align_brake_us);
                ALIGN(pixel_align_brake_motion_px);
                ALIGN(pixel_align_stopped_wz);
                ALIGN(pixel_align_stopped_vx);
                ALIGN(pixel_align_stable_us);
                ALIGN(pixel_align_stable_frames);
#undef ALIGN
                mark(r, "image reference pixel_alignment (initial, not field calibrated)",
                     {"task.pixel_align_gain_rps_per_px", "task.pixel_align_max_wz",
                      "task.pixel_align_tolerance_px", "task.pixel_align_restart_px",
                      "task.pixel_align_stopped_wz", "task.pixel_align_stopped_vx",
                      "task.pixel_align_stable_us", "task.pixel_align_stable_frames",
                      "task.pixel_align_approach_tolerance_px",
                      "task.pixel_align_approach_restart_px",
                      "task.pixel_align_brake_us",
                      "task.pixel_align_brake_motion_px"});
            }
            if (tuning.pixel_align_approach_restart_px <= tuning.pixel_align_approach_tolerance_px ||
                tuning.pixel_align_restart_px <= tuning.pixel_align_tolerance_px ||
                tuning.pixel_align_max_wz > tuning.max_wz ||
                tuning.pixel_align_stopped_wz >= tuning.pixel_align_max_wz ||
                tuning.pixel_align_stable_frames < 3)
                throw std::runtime_error("Invalid pixel alignment hysteresis, speed or sample limits");
            capture_config.pixel_align_tolerance_px = tuning.pixel_align_tolerance_px;
            capture_config.pixel_approach_pitch_cdeg =
                reference.value("approach_camera_pitch_cdeg", 0);
            for (const auto& entry : reference.at("classes").items()) {
                auto bounds = entry.value().at("center_bounds_px");
                const bool lateral = !bounds.is_null();
                if (!lateral) {
                    if (!capture_config.pixel_grasp_enabled)
                        continue;
                    const auto& centers = entry.value().at("reference_center_px");
                    bounds = nlohmann::json::array({0, centers.at("front").at(1),
                                                   config.frame_width - 1, centers.at("rear").at(1)});
                }
                if ((entry.key() != "ordinary_supply" && entry.key() != "injured_person" &&
                     entry.key() != "core_supply") || !bounds.is_array() || bounds.size() != 4)
                    throw std::runtime_error("Invalid class grasp centre bounds");
                const RectArea area{bounds.at(0).get<float>(), bounds.at(1).get<float>(),
                                    bounds.at(2).get<float>(), bounds.at(3).get<float>()};
                const auto& target = entry.value().at("target_center_px");
                const cv::Point2f center(target.at(0).get<float>(), target.at(1).get<float>());
                if (!std::isfinite(area.x1) || !std::isfinite(area.x2) ||
                    !std::isfinite(area.y1) || !std::isfinite(area.y2) ||
                    area.x1 < 0 || area.y1 < 0 || area.x1 >= area.x2 || area.y1 >= area.y2 ||
                    area.x2 >= config.frame_width || area.y2 >= config.frame_height ||
                    !std::isfinite(center.x) || !std::isfinite(center.y) ||
                    center.x < area.x1 || center.x > area.x2 ||
                    center.y < area.y1 || center.y > area.y2)
                    throw std::runtime_error("Class grasp bounds outside original image");
                capture_config.grasp_center_ranges.push_back(
                    {entry.key(), int16_t(pitch), config.frame_width, config.frame_height,
                     area, center, lateral});
            }
        }
        r.measured_load_radius = measured.load_radius_m;
        r.measured_body_radius = measured.robot_swept_radius_m;
        r.load_half_width = measured.load_half_width_m;
        r.load_half_depth = measured.load_half_depth_m;
        r.task_calibration_version = measured.version;
        readCalibrationAudit(r);
        const auto source = "task calibration: " + config.task_calibration_file;
        mark(r, source, {"task.hold_center_y_m", "task.mouth_y_m", "task.injured_hold_center_y_m"});
        const auto provided = [&](const char* key) {
            const auto& keys = r.calibration_audit.provided_fields;
            return std::find(keys.begin(), keys.end(), key) != keys.end();
        };
        if (r.calibration_audit.schema_version == 2) {
            mark(r, source, {"task.require_multi_view", "task.closed_front_y_m"});
            if (config.demo_mode != "search")
                mark(r,
                     source,
                     {"task.grasp_trigger_y_m",
                      "task.near_pitch_cdeg",
                      "task.enable_search_cues",
                      "task.enable_short_push",
                      "task.startup_advance_us",
                      "task.attempt_budget_us",
                      "task.approach_limit_m",
                      "task.retreat_limit_m",
                      "task.max_speed",
                      "task.approach_speed",
                      "task.rush_speed"});
        } else {
            if (provided("grasp_trigger_y_m"))
                mark(r, source, {"task.grasp_trigger_y_m"});
            if (provided("closed_front_y_m"))
                mark(r, source, {"task.closed_front_y_m"});
        }
        visitCapture(r.capture, [&](const char* name, auto) {
            r.field_sources[std::string("capture.") + name].push_back(
                "TaskCalibration CaptureConfig defaults");
        });
        mark(r,
             source,
             {"capture.mouth_y_m", "capture.corridor_half_width_m", "capture.image_height_px"});
        mark(r,
             source,
             {"capture.holding",
              "frame_views",
              "feedback_open",
              "feedback_close",
              "box_area_accepted",
              "a6_done_flag",
              "a6_done_settle_ms",
              "view_image_polygon",
              "body_envelope",
              "measured_load_radius",
              "measured_body_radius",
              "load_half_width",
              "load_half_depth"});
        mark(r,
             "legacy restore startup/scan/attempt values after calibration",
             {"task.startup_advance_us",
              "task.startup_advance_speed",
              "task.scan_wz",
              "task.attempt_budget_us"});
    }
    configureDemo(config, tuning);
    if (config.demo_mode != "none")
        mark(r,
             "configureDemo legacy bounds",
             {"task.startup_advance_us",
              "task.enable_search_cues",
              "task.enable_short_push",
              "task.max_speed",
              "task.approach_speed",
              "task.rush_speed",
              "task.scan_wz",
              "task.turn_wz",
              "task.max_wz",
              "task.min_turn_wz",
              "task.attempt_budget_us",
              "task.approach_limit_m",
              "task.retreat_limit_m",
              "task.demo_search_only",
              "task.demo_carry_once"});
    if (config.push_replay.empty()) {
        tuning.max_speed = .40f;
        tuning.approach_speed = .40f;
        tuning.carry_speed = .05f; // 2026-10-28: operator-requested transport speed
        tuning.rush_speed = .32f;
        tuning.rush_budget_us = 12000000;
        tuning.blind_scatter_max_y_m = .50f;
        tuning.blind_scatter_far_max_y_m = 4.0f;
        tuning.blind_scatter_half_x_m = 2.5f;
        tuning.blind_scatter_approach_budget_us = 20000000;
        tuning.blind_scatter_approach_max = 4;
        tuning.blind_scatter_budget_us = 15000000;
        tuning.blind_scatter_max = 8;
        tuning.sweep_total_max = 4;
        tuning.sweep_time_budget_us = 36000000;
        tuning.sweep_op_budget_us = 24000000;
        tuning.sweep_match_budget_us = 60000000;
        tuning.blind_scatter_passes = 1;
        tuning.first_burst = true;
        tuning.first_burst_full_impact = true;
        tuning.first_burst_speed = 1.0f;
        tuning.first_burst_slow_m = .35f;
        tuning.first_burst_min_drive_m = .60f;
        tuning.first_burst_swing_wz = 5.0f;
        if (tuning.startup_advance_us > 0 && tuning.startup_advance_speed > 0.f &&
            tuning.startup_advance_speed < 1.f) {
            tuning.startup_advance_us =
                uint64_t(double(tuning.startup_advance_us) * double(tuning.startup_advance_speed));
            tuning.startup_advance_speed = 1.0f;
            mark(r,
                 "legacy field startup: duration *= requested speed; speed = 1",
                 {"task.startup_advance_us", "task.startup_advance_speed"});
        }
        tuning.demo_capture_motion = true;
        tuning.precise_rush = false;
        tuning.rush_stall_us = 0;
        tuning.rush_pulse_turn = false;
        tuning.sweep_observation_guard = true;
        tuning.frame_open_default = true;
        tuning.disperse_turn_rad = 1.4f + 30.f * float(CV_PI) / 180.f;
        tuning.disperse_turn_push_m = .2f;
        tuning.disperse_turn_frame_check = true;
        tuning.row_select = true;
        tuning.disperse_single_turn = true;
        tuning.sweep_view_cap_mid = true;
        tuning.sweep_far_judge = true;
        tuning.swing_avoid_outside = false;
        tuning.dense_no_lock = true;
        tuning.approach_dense_burst_swing = true;
        tuning.lock_max_visible = 2;
        tuning.lock_multi_group_min = 2;
        tuning.lock_multi_radius_m = .60f;
        tuning.dense_multi_ignore_gap = true;
        tuning.pile_clear = true;
        tuning.pile_cut_max_steps = 3;
        tuning.pile_chain = true;
        tuning.occlusion_hold_us = 2500000;
        tuning.occlusion_near_m = .8f;
        tuning.relock_window_us = 4000000;
        tuning.relock_gate_frac = .10f;
        tuning.sweep_frame_open = true;
        tuning.disperse_swing_vx = 0.f;
        tuning.disperse_passes = 1;
        tuning.blind_scatter_passes = 1;
        tuning.enter_speed = .05f;
        tuning.back_speed = .05f;
        tuning.assume_repush_speed_gain = 0.f; // keep repeated pushes at 0.05 m/s too
        tuning.turn_wz = .55f;
        tuning.max_wz = std::max(.55f, std::min(tuning.scan_wz, .8f));
        tuning.min_turn_wz = std::min(tuning.min_turn_wz, .40f);
        tuning.inplace_turn_boost = 2.f;
        tuning.field_carry = true;
        tuning.field_rush = true;
        tuning.verify_held_contents = true;
        tuning.corridor_confirm_frames = 6;
        tuning.assume_enter_confirm = true;
        tuning.assume_align_push = true;
        tuning.assume_stall_near_m = .12f;
        tuning.assume_slant_max_rad = 0.f;
        tuning.approach_limit_m = std::max(tuning.approach_limit_m, 8.0f);
        tuning.retreat_limit_m = std::max(tuning.retreat_limit_m, 4.0f);
        tuning.assume_carry_m = std::max(tuning.assume_carry_m, .60f);
        tuning.assume_push_m = .55f;
        tuning.enter_budget_us = std::max<uint64_t>(tuning.enter_budget_us, 20000000);
        tuning.assume_back_m = std::max(
            tuning.assume_back_m,
            std::min(.50f, .85f * tuning.back_speed * float(tuning.retreat_budget_us) / 1e6f));
        diagnostics << "[FIELD_CARRY] demo transport logic in main path: blind carry_m="
                    << tuning.assume_carry_m << " back_m=" << tuning.assume_back_m
                    << " approach_limit_m=" << tuning.approach_limit_m
                    << " retreat_limit_m=" << tuning.retreat_limit_m
                    << " max_speed=" << tuning.max_speed << " turn_min_rad=" << tuning.turn_min_rad
                    << "\n";
        mark(r,
             "legacy_field_v1 final field overrides",
             {"task.max_speed",
              "task.approach_speed",
              "task.carry_speed",
              "task.rush_speed",
              "task.rush_budget_us",
              "task.blind_scatter_max_y_m",
              "task.blind_scatter_far_max_y_m",
              "task.blind_scatter_half_x_m",
              "task.blind_scatter_approach_budget_us",
              "task.blind_scatter_approach_max",
              "task.blind_scatter_budget_us",
              "task.blind_scatter_max",
              "task.sweep_total_max",
              "task.sweep_time_budget_us",
              "task.sweep_op_budget_us",
              "task.sweep_match_budget_us",
              "task.blind_scatter_passes",
              "task.first_burst",
              "task.first_burst_full_impact",
              "task.first_burst_speed",
              "task.first_burst_slow_m",
              "task.first_burst_min_drive_m",
              "task.first_burst_swing_wz",
              "task.demo_capture_motion",
              "task.precise_rush",
              "task.rush_stall_us",
              "task.rush_pulse_turn",
              "task.sweep_observation_guard",
              "task.frame_open_default",
              "task.disperse_turn_rad",
              "task.disperse_turn_push_m",
              "task.disperse_turn_frame_check",
              "task.row_select",
              "task.disperse_single_turn",
              "task.sweep_view_cap_mid",
              "task.sweep_far_judge",
              "task.swing_avoid_outside",
              "task.dense_no_lock",
              "task.approach_dense_burst_swing",
              "task.lock_max_visible",
              "task.lock_multi_group_min",
              "task.lock_multi_radius_m",
              "task.dense_multi_ignore_gap",
              "task.pile_clear",
              "task.pile_cut_max_steps",
              "task.pile_chain",
              "task.occlusion_hold_us",
              "task.occlusion_near_m",
              "task.relock_window_us",
              "task.relock_gate_frac",
              "task.sweep_frame_open",
              "task.disperse_swing_vx",
              "task.disperse_passes",
              "task.enter_speed",
              "task.back_speed",
              "task.assume_repush_speed_gain",
              "task.turn_wz",
              "task.max_wz",
              "task.min_turn_wz",
              "task.inplace_turn_boost",
              "task.field_carry",
              "task.field_rush",
              "task.verify_held_contents",
              "task.corridor_confirm_frames",
              "task.assume_enter_confirm",
              "task.assume_align_push",
              "task.assume_stall_near_m",
              "task.assume_slant_max_rad",
              "task.approach_limit_m",
              "task.retreat_limit_m",
              "task.assume_carry_m",
              "task.assume_push_m",
              "task.enter_budget_us",
              "task.assume_back_m"});
    }
    if (config.assume_all_safe) {
        tuning.turn_min_rad = std::min(tuning.turn_min_rad, .25f);
        tuning.assume_all_safe = true;
        tuning.assume_injured_trip = config.assume_injured_trip;
        diagnostics << "[TEMP_ASSUMPTION] assume_all_safe=1 injured_trip="
                    << config.assume_injured_trip
                    << " clearance/zone/capture/delivery evidence assumed true; NOT_ACCEPTANCE\n";
        mark(r,
             "explicit assume_all_safe legacy overrides",
             {"task.turn_min_rad", "task.assume_all_safe", "task.assume_injured_trip"});
    }
    // Explicit lateral-jaw single-view profiles use the measured stop point and
    // the configured three presets after legacy field overrides have finished.
    if (r.task_calibrated && !tuning.require_multi_view && r.frame_views.size() == 1) {
        tuning.assume_rush_trigger_m = tuning.grasp_trigger_y_m;
        tuning.far_pitch_cdeg = config.pitch_presets_cdeg[0];
        tuning.track_pitch_cdeg = config.pitch_presets_cdeg[1];
        tuning.near_pitch_cdeg = r.frame_views.front().pitch_cdeg;
        tuning.intermediate_pitch_cdeg = config.pitch_presets_cdeg[1];
        tuning.assume_view_pitch_cdeg = tuning.near_pitch_cdeg;
        mark(r, "measured lateral-jaw profile", {"task.assume_rush_trigger_m",
             "task.far_pitch_cdeg", "task.track_pitch_cdeg", "task.near_pitch_cdeg",
             "task.intermediate_pitch_cdeg", "task.assume_view_pitch_cdeg"});
    }
    // Pixel control is installed after all legacy pitch overrides.
    if (capture_config.pixel_grasp_enabled) {
        if (capture_config.grasp_center_ranges.size() != 3 ||
            capture_config.pixel_approach_pitch_cdeg != 0 ||
            capture_config.grasp_center_ranges.front().pitch_cdeg != 2000)
            throw std::runtime_error("Pixel grasp requires UI120 approach and UI140 class references");
        tuning.pixel_grasp_enabled = true;
        tuning.require_multi_view = false;
        tuning.track_pitch_cdeg = tuning.intermediate_pitch_cdeg = 0;
        tuning.near_pitch_cdeg = tuning.assume_view_pitch_cdeg = 2000;
        mark(r, "user pixel centre references (UI120/UI140)",
             {"task.pixel_grasp_enabled", "task.require_multi_view", "task.track_pitch_cdeg",
              "task.intermediate_pitch_cdeg", "task.near_pitch_cdeg", "task.assume_view_pitch_cdeg"});
    }
    r.a6_known = r.a6_done_flag ? config.known_frame_angle >= 0 && config.known_frame_id > 0
                                : r.feedback_open >= 0;
    if (r.a6_done_flag)
        diagnostics << "[A6] TEMP_ASSUMPTION a6_semantics=done_flag: byte1==1 with matching id "
                       "means action acknowledged (open or close);"
                       " frame state = last completed target sent by this host; startup state from "
                       "--known-frame-angle "
                    << config.known_frame_angle << " --known-frame-id " << config.known_frame_id
                    << "; settle_ms=" << r.a6_done_settle_ms << "; not a position sensor\n";
    if (r.view_image_polygon)
        diagnostics << "[MULTI_VIEW] TEMP_ASSUMPTION frame_view_matching=image_polygon: enclosure "
                       "judged from per-pitch image polygons;"
                       " no ground ranging inside the frame; views compared by class counts only\n";
    if (config.hardware &&
        (!r.a6_known || (config.demo_mode != "search" &&
                         (!tuning.pixel_grasp_enabled && (tuning.require_multi_view ? r.frame_views.size() < 2
                                                    : r.frame_views.size() != 1)))))
        throw std::runtime_error(r.a6_done_flag
                                     ? "Done-flag A6 requires --known-frame-angle and "
                                       "--known-frame-id plus schema 2 views before hardware output"
                                     : "New frame requires accepted schema 2 calibration and A6 "
                                       "mapping before hardware output");
    r.ground_contact.min_confidence = config.confidence;
    r.ground_contact.accept_size_mismatch = true;
    r.ground_contact.injured_lying_only = true;
    r.ground_contact.lying_confirm_diff = -.06f;
    r.ground_contact.use_silhouette = true;
    mark(r,
         "legacy live ground-contact configuration",
         {"ground_contact.min_confidence",
          "ground_contact.accept_size_mismatch",
          "ground_contact.injured_lying_only",
          "ground_contact.lying_confirm_diff",
          "ground_contact.use_silhouette"});
    r.match.duration_us = uint64_t(config.match_seconds) * 1000000;
    r.match.time_limit_enabled = !config.no_match_time_limit;
    r.match.require_measured_progress = !config.controlled_empty_field;
    mark(r,
         "legacy MatchControl configuration",
         {"match.duration_us", "match.time_limit_enabled", "match.require_measured_progress"});
    mark(r,
         "MatchConfig defaults",
         {"match.evidence_timeout_us", "match.no_movement_us", "match.movement_threshold_m"});
    mark(r,
         "MotionLinkConfig defaults",
         {"motion_link.period_ms", "motion_link.command_timeout_ms",
          "motion_link.linear_accel_mps2", "motion_link.linear_decel_mps2",
          "motion_link.angular_accel_rps2", "motion_link.angular_decel_rps2"});
    addAssumptions(r);
    return r;
}

void printRuntimeTuning(const ResolvedRuntimeConfig& r, std::ostream& diagnostics) {
    const auto& tuning = r.task;
    const auto& capture_config = r.capture;
    diagnostics << "[CONTROL_TUNING] demo_capture_motion=" << tuning.demo_capture_motion
                << " rush_speed=" << tuning.rush_speed << " rush_stall_us=" << tuning.rush_stall_us
                << " precise_rush=" << tuning.precise_rush
                << " sweep_frame_open=" << tuning.sweep_frame_open
                << " frame_open_default=" << tuning.frame_open_default
                << " disperse_turn_rad=" << tuning.disperse_turn_rad
                << " sweep_view_cap_mid=" << tuning.sweep_view_cap_mid
                << " sweep_far_judge=" << tuning.sweep_far_judge
                << " swing_avoid_outside=" << tuning.swing_avoid_outside
                << " dense_no_lock=" << tuning.dense_no_lock
                << " approach_dense_burst_swing=" << tuning.approach_dense_burst_swing
                << " lock_max_visible=" << tuning.lock_max_visible
                << " lock_multi_group_min=" << tuning.lock_multi_group_min
                << " lock_multi_radius_m=" << tuning.lock_multi_radius_m
                << " dense_multi_ignore_gap=" << tuning.dense_multi_ignore_gap
                << " occlusion_hold_us=" << tuning.occlusion_hold_us
                << " relock_window_us=" << tuning.relock_window_us
                << " relock_gate_frac=" << tuning.relock_gate_frac
                << " pile_clear=" << tuning.pile_clear
                << " pile_corridor_min=" << tuning.pile_corridor_min
                << " pile_trigger_max_m=" << tuning.pile_trigger_max_m
                << " pile_advance_m=" << tuning.pile_advance_m
                << " pile_block_half_m=" << tuning.pile_block_half_m
                << " pile_view_back_m=" << tuning.pile_view_back_m
                << " rush_align_deadband_rad=" << tuning.rush_align_deadband_rad
                << " rush_align_wz=" << tuning.rush_align_wz
                << " rush_near_speed=" << tuning.rush_near_speed
                << " rush_slow_range_m=" << tuning.rush_slow_range_m << " rush_trigger_m="
                << (tuning.field_rush || tuning.assume_all_safe ? tuning.assume_rush_trigger_m
                                                                : tuning.grasp_trigger_y_m)
                << " corridor_half_width_m=" << capture_config.corridor_half_width_m
                << " corridor_extra_m=" << capture_config.corridor_extra_m
                << " sweep_observation_guard=" << tuning.sweep_observation_guard
                << " sweep_reobserve_us=" << tuning.sweep_reobserve_us
                << " blind_scatter_passes=" << tuning.blind_scatter_passes
                << " swing_vx=" << tuning.disperse_swing_vx
                << " first_burst_full_impact=" << tuning.first_burst_full_impact
                << " first_burst_speed=" << tuning.first_burst_speed
                << " disperse_turn_push_m=" << tuning.disperse_turn_push_m
                << " disperse_single_turn=" << tuning.disperse_single_turn
                << " disperse_turn_min_rad=" << tuning.disperse_turn_min_rad
                << " disperse_turn_view_range_m=" << tuning.disperse_turn_view_range_m
                << " disperse_turn_view_half_rad=" << tuning.disperse_turn_view_half_rad
                << " disperse_turn_clear_max=" << tuning.disperse_turn_clear_max
                << " disperse_turn_clear_frames=" << tuning.disperse_turn_clear_frames
                << " disperse_turn_look_us=" << tuning.disperse_turn_look_us
                << " disperse_turn_frame_check=" << tuning.disperse_turn_frame_check
                << " frame_max=" << tuning.disperse_turn_frame_max
                << " frame_range_m=" << tuning.disperse_turn_frame_range_m
                << " frame_half_m=" << tuning.disperse_turn_frame_half_m
                << " frame_extra_rad=" << tuning.disperse_turn_frame_extra_rad << "\n";
    if (tuning.pixel_grasp_enabled)
        diagnostics << "[PIXEL_ALIGN_TUNING] enabled=1"
                << " pixel_align_gain_rps_per_px=" << tuning.pixel_align_gain_rps_per_px
                << " pixel_align_max_wz=" << tuning.pixel_align_max_wz
                << " pixel_align_tolerance_px=" << tuning.pixel_align_tolerance_px
                << " pixel_align_restart_px=" << tuning.pixel_align_restart_px
                << " pixel_align_approach_tolerance_px=" << tuning.pixel_align_approach_tolerance_px
                << " pixel_align_approach_restart_px=" << tuning.pixel_align_approach_restart_px
                << " pixel_align_brake_us=" << tuning.pixel_align_brake_us
                << " pixel_align_brake_motion_px=" << tuning.pixel_align_brake_motion_px
                << " pixel_align_stable_us=" << tuning.pixel_align_stable_us
                << " pixel_align_stable_frames=" << tuning.pixel_align_stable_frames
                << " pixel_align_stopped_wz=" << tuning.pixel_align_stopped_wz
                << " pixel_align_stopped_vx=" << tuning.pixel_align_stopped_vx
                << "\n";
}

void writeEffectiveConfig(const ResolvedRuntimeConfig& r, std::ostream& output) {
    cv::FileStorage f("effective_config.json",
                      cv::FileStorage::WRITE | cv::FileStorage::MEMORY |
                          cv::FileStorage::FORMAT_JSON);
    f << "schema_version" << 1 << "resolution" << "legacy_assignment_order"
      << "number_encoding"
      << "finite numbers; nonfinite values use NaN/+infinity/-infinity strings; integers beyond "
         "2^53-1 use decimal strings"
      << "policy" << "{" << "name" << r.policy_name << "version" << r.policy_version << "}";
    const auto& c = r.request;
    const char* operation = !c.push_replay.empty()       ? "push_replay"
                            : !c.geometry_replay.empty() ? "geometry_replay"
                            : !c.detect_image.empty()    ? "detect_image"
                                                         : "live";
    f << "mode" << "{" << "operation" << operation << "mcu_access"
      << (c.hardware         ? "enabled"
          : c.pitch_feedback ? "feedback_only"
                             : "disabled")
      << "task_scope" << c.demo_mode << "static_check" << int(c.check_config) << "}";
    const auto& identity = r.identity;
    f << "identity" << "{" << "config_file" << identity.config_file << "working_directory"
      << identity.working_directory << "executable" << identity.executable << "audit_error"
      << identity.audit_error << "rknn_library_lookup" << identity.rknn_library_lookup
      << "rknn_compiled" << int(identity.rknn_compiled) << "telemetry_compiled"
      << int(identity.telemetry_compiled) << "arguments" << "[";
    for (const auto& arg : identity.arguments)
        f << arg;
    f << "]" << "resource_paths" << "{";
    for (const auto& path : identity.resource_paths)
        scalar(f, path.first, path.second);
    f << "}" << "}";
    f << "request" << "{";
    visitRequest(r.request, [&](const char* name, const auto& value) {
        scalar(f, name, value);
    });
    f << "pitch_presets_cdeg" << "[:";
    for (auto value : r.request.pitch_presets_cdeg)
        f << int(value);
    f << "]" << "class_names" << "[";
    for (const auto& name : r.request.class_names)
        f << name;
    f << "]";
    area(f, "catch_area", r.request.catch_area);
    area(f, "ready_area", r.request.ready_area);
    area(f, "holding_area", r.request.holding_area);
    area(f, "center_region", r.request.center_region);
    area(f, "cross_center_region", r.request.cross_center_region);
    f << "catch_angle" << "[:" << r.request.catch_angle[0] << r.request.catch_angle[1] << "]"
      << "release_angle" << "[:" << r.request.release_angle[0] << r.request.release_angle[1] << "]"
      << "}";
    f << "startup" << "{";
    scalar(f, "requested_advance_ms", r.request.startup_advance_ms);
    scalar(f, "requested_speed_mps", r.request.startup_speed_mps);
    scalar(f, "effective_advance_us", r.task.startup_advance_us);
    scalar(f, "effective_speed_mps", r.task.startup_advance_speed);
    scalar(f, "requested_scan_wz_rps", r.request.scan_wz_rps);
    scalar(f, "effective_scan_wz_rps", r.task.scan_wz);
    scalar(f, "requested_turn_wz_rps", r.request.turn_wz_rps);
    scalar(f, "effective_turn_wz_rps", r.task.turn_wz);
    f << "}" << "task" << "{";
    visitTask(r.task, [&](const char* name, const auto& value) {
        scalar(f, name, value);
    });
    f << "}" << "capture" << "{";
    visitCapture(r.capture, [&](const char* name, const auto& value) {
        scalar(f, name, value);
    });
    scalar(f, "pixel_grasp_enabled", r.capture.pixel_grasp_enabled);
    scalar(f, "pixel_approach_pitch_cdeg", r.capture.pixel_approach_pitch_cdeg);
    if (!r.capture.grasp_center_ranges.empty()) {
        f << "grasp_center_ranges" << "[";
        for (const auto& range : r.capture.grasp_center_ranges) {
            f << "{" << "label" << range.label << "pitch_cdeg" << int(range.pitch_cdeg);
            area(f, "center_bounds_px", range.area);
            f << "target_center_px" << "[" << range.target_center_px.x << range.target_center_px.y << "]";
            scalar(f, "lateral_measured", range.lateral_measured);
            f << "}";
        }
        f << "]";
    }
    f << "holding" << "[";
    for (const auto& view : r.capture.holding) {
        f << "{" << "pitch_cdeg" << int(view.pitch_cdeg);
        area(f, "area", view.area);
        scalar(f, "min_visible_bottom_y_px", view.min_visible_bottom_y_px);
        f << "polygon";
        points(f, view.polygon);
        f << "}";
    }
    f << "]" << "}" << "ground_contact" << "{" << "active"
      << int(!r.request.calibration_file.empty());
    visitGround(r.ground_contact, [&](const char* name, const auto& value) {
        scalar(f, name, value);
    });
    f << "}" << "match" << "{";
    scalar(f, "duration_us", r.match.duration_us);
    scalar(f, "evidence_timeout_us", r.match.evidence_timeout_us);
    scalar(f, "no_movement_us", r.match.no_movement_us);
    scalar(f, "movement_threshold_m", r.match.movement_threshold_m);
    scalar(f, "time_limit_enabled", r.match.time_limit_enabled);
    scalar(f, "require_measured_progress", r.match.require_measured_progress);
    f << "}" << "motion_link" << "{";
    scalar(f, "period_ms", r.motion_link.period_ms);
    scalar(f, "command_timeout_ms", r.motion_link.command_timeout_ms);
    scalar(f, "linear_accel_mps2", r.motion_link.linear_accel_mps2);
    scalar(f, "linear_decel_mps2", r.motion_link.linear_decel_mps2);
    scalar(f, "angular_accel_rps2", r.motion_link.angular_accel_rps2);
    scalar(f, "angular_decel_rps2", r.motion_link.angular_decel_rps2);
    f << "}" << "hardware" << "{";
    scalar(f, "output_enabled", r.request.hardware);
    scalar(f, "feedback_only", r.request.pitch_feedback);
    scalar(f, "mcu_port", r.request.uart_port);
    scalar(f, "mcu_baud", r.request.baudrate);
    scalar(f, "imu_enabled", r.request.imu);
    scalar(f, "imu_port", r.request.imu_port);
    scalar(f, "imu_baud", r.request.imu_baud);
    scalar(f, "imu_timeout_ms", r.request.imu_timeout_ms);
    scalar(f, "camera_index", r.request.camera_index);
    scalar(f, "frame_width", r.request.frame_width);
    scalar(f, "frame_height", r.request.frame_height);
    scalar(f, "fps", r.request.fps);
    f << "}" << "calibration" << "{";
    scalar(f, "task_calibrated", r.task_calibrated);
    scalar(f, "task_calibration_file", r.request.task_calibration_file);
    scalar(f, "task_calibration_version", r.task_calibration_version);
    scalar(f, "a6_known", r.a6_known);
    scalar(f, "feedback_open", r.feedback_open);
    scalar(f, "feedback_close", r.feedback_close);
    scalar(f, "box_area_accepted", r.box_area_accepted);
    scalar(f, "a6_done_flag", r.a6_done_flag);
    scalar(f, "a6_done_settle_ms", r.a6_done_settle_ms);
    scalar(f, "view_image_polygon", r.view_image_polygon);
    scalar(f, "measured_load_radius_m", r.measured_load_radius);
    scalar(f, "measured_body_radius_m", r.measured_body_radius);
    scalar(f, "load_half_width_m", r.load_half_width);
    scalar(f, "load_half_depth_m", r.load_half_depth);
    f << "body_envelope" << "[:";
    for (auto value : r.body_envelope.val)
        f << double(value);
    f << "]" << "frame_views" << "[";
    for (const auto& view : r.frame_views) {
        f << "{" << "pitch_cdeg" << int(view.pitch_cdeg) << "calibrated" << int(view.calibrated)
          << "fully_observable" << int(view.fully_observable) << "polygon";
        points(f, view.polygon);
        f << "}";
    }
    const auto& a = r.calibration_audit;
    f << "]" << "audit" << "{" << "schema_version" << a.schema_version << "version" << a.version
      << "status" << a.status << "dimensions_source" << a.dimensions_source << "assume_source"
      << a.assume_source << "action_mapping_source" << a.action_mapping_source
      << "a6_done_settle_source" << a.a6_done_settle_source << "frame_view_matching_source"
      << a.frame_view_matching_source << "audit_error" << a.audit_error << "pending" << "[";
    for (const auto& value : a.pending)
        f << value;
    f << "]" << "provided_fields" << "[";
    for (const auto& value : a.provided_fields)
        f << value;
    f << "]" << "verification_flags" << "{";
    for (const auto& value : a.verification_flags)
        f << value.first << value.second;
    f << "}" << "}" << "}" << "assumptions" << "[";
    for (const auto& a : r.assumptions)
        f << "{" << "id" << a.id << "scope" << a.scope << "replacement" << a.replacement
          << "evidence" << a.evidence << "status" << a.status << "}";
    f << "]" << "field_sources" << "[";
    for (const auto& field : r.field_sources) {
        f << "{" << "field" << field.first << "history" << "[";
        for (const auto& source : field.second)
            f << source;
        f << "]" << "}";
    }
    f << "]";
    output << f.releaseAndGetString() << '\n';
}

} // namespace rescue
