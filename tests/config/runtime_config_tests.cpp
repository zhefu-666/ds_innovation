#include "rescue/app/runtime_config.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

using namespace rescue;

namespace {
Config parse(std::initializer_list<const char*> values) {
    std::vector<char*> args;
    for (auto value : values)
        args.push_back(const_cast<char*>(value));
    return parseArgs(int(args.size()), args.data());
}

bool close(float a, float b) {
    return std::abs(a - b) < 1e-6f;
}

bool assumption(const ResolvedRuntimeConfig& r, const char* id) {
    return std::any_of(r.assumptions.begin(), r.assumptions.end(), [&](const auto& a) {
        return a.id == id;
    });
}

struct TemporaryCalibration {
    char path[64] = "/tmp/rescue-runtime-calibration-XXXXXX";
    TemporaryCalibration() {
        const int fd = mkstemp(path);
        assert(fd >= 0);
        ::close(fd);
        std::ofstream f(path);
        f << R"JSON({
  "schema_version":2,"version":"synthetic-runtime-test","status":"TEMP_ASSUMPTION_NOT_ACCEPTANCE",
  "dimensions_source":"synthetic_test_only","assume_source":"test; not field evidence",
  "image_width":1280,"image_height":720,
  "coordinate_transform_verified":1,"action_mapping_verified":1,"capture_stop_verified":1,
  "body_envelope_verified":1,"visual_acceptance_passed":1,"box_area_accepted":0,
  "a6_semantics":"done_flag","a6_done_settle_ms":900,"a6_done_settle_source":"synthetic estimate",
  "a6_open_state":-1,"a6_close_state":-1,"forward_sign":1,
  "rotation_origin_x_m":0,"rotation_origin_y_m":0,
  "inner_width_m":0.15,"inner_depth_m":0.08008,"entrance_width_m":0.15,
  "near_inner_m":0.09092,"far_inner_m":0.171,
  "body_left_m":0.145,"body_right_m":0.145,"body_rear_m":0.1,"body_front_m":0.18107,
  "grasp_trigger_y_m":0.13096,"maximum_block_depth_m":0.04,"maximum_block_width_m":0.04,
  "stopping_margin_m":0.005,"open_width_m":0.29,"open_front_m":0.19,"robot_swept_radius_m":0.25,
  "frame_view_matching":"image_polygon","frame_view_matching_source":"synthetic test polygon",
  "frame_views":[
    {"pitch_cdeg":3500,"calibrated":1,"fully_observable":1,"polygon":[[10,10],[300,10],[300,300],[10,300]]},
    {"pitch_cdeg":2000,"calibrated":1,"fully_observable":1,"polygon":[[50,50],[350,50],[350,350],[50,350]]}
  ],"pending":["real measurements pending","synthetic acceptance flags"]
})JSON";
    }
    ~TemporaryCalibration() {
        unlink(path);
    }
};
} // namespace

int main() {
    assert(Config{}.single_phase.empty());
    for (const char* phase : {"startup", "search", "capture", "transport"})
        assert(parse({"test", "--phase", phase}).single_phase == phase);
    for (const auto& args : {std::vector<const char*>{"test", "--phase", "invalid"},
                             std::vector<const char*>{"test", "--phase", "capture", "--step-phases"}}) {
        std::vector<char*> argv;
        for (const char* arg : args) argv.push_back(const_cast<char*>(arg));
        bool rejected = false;
        try { parseArgs(int(argv.size()), argv.data()); }
        catch (const std::exception&) { rejected = true; }
        assert(rejected);
    }
    assert(!Config{}.step_phases);
    assert(parse({"test", "--step-phases"}).step_phases);
    std::ostringstream diagnostics;
    Config request;
    request.push_replay = "recorded.json";
    request.startup_advance_ms = 5000;
    request.startup_speed_mps = .25f;
    request.turn_wz_rps = .7f;
    auto replay = resolveLegacyRuntimeConfig(request, diagnostics);
    assert(replay.policy_name == "legacy_replay_v1");
    assert(!replay.task.field_carry && !replay.task.field_rush && !replay.task.first_burst);
    assert(replay.task.startup_advance_us == 5000000 &&
           close(replay.task.startup_advance_speed, .25f));
    assert(close(replay.task.turn_wz, .7f) &&
           close(replay.task.rush_speed, TaskTuning{}.rush_speed));
    assert(diagnostics.str().empty());
    assert(!assumption(replay, "legacy_field_transport"));

    request.push_replay.clear();
    auto field = resolveLegacyRuntimeConfig(request, diagnostics);
    assert(field.policy_name == "legacy_field_v1");
    assert(field.task.startup_advance_us == 1250000 &&
           close(field.task.startup_advance_speed, 1.f));
    assert(field.request.startup_advance_ms == 5000 &&
           close(field.request.startup_speed_mps, .25f));
    assert(field.task.field_carry && field.task.field_rush && field.task.first_burst);
    assert(close(field.task.carry_speed, .05f) && close(field.task.enter_speed, .05f) &&
           close(field.task.back_speed, .05f));
    assert(close(field.task.assume_repush_speed_gain, 0.f));
    assert(close(field.task.turn_wz, .55f) && close(field.task.rush_speed, .32f));
    assert(field.task.approach_limit_m == 8.f && field.task.retreat_limit_m == 4.f);
    assert(field.ground_contact.accept_size_mismatch && field.ground_contact.use_silhouette);
    assert(field.match.duration_us == 180000000 && field.match.require_measured_progress);
    assert(field.motion_link.period_ms == 40 && field.motion_link.command_timeout_ms == 150);
    assert(close(field.motion_link.linear_accel_mps2, .30f));
    assert(close(field.motion_link.linear_decel_mps2, .40f));
    assert(close(field.motion_link.angular_accel_rps2, .80f));
    assert(close(field.motion_link.angular_decel_rps2, 1.f));
    assert(field.field_sources.at("task.turn_wz").back() ==
           "legacy_field_v1 final field overrides");
    assert(assumption(field, "legacy_field_transport"));
    request.startup_advance_ms = 0;
    auto no_start = resolveLegacyRuntimeConfig(request, diagnostics);
    assert(no_start.task.startup_advance_us == 0 &&
           close(no_start.task.startup_advance_speed, .25f));

    auto demo = resolveLegacyRuntimeConfig(
        parse({"test", "--demo-mode", "carry_once", "--demo-seconds", "13"}), diagnostics);
    assert(demo.task.demo_carry_once && demo.task.startup_advance_us == 0);
    assert(demo.task.attempt_budget_us == 13000000 && demo.match.duration_us == 13000000);
    // Preserve the entire historical chain: field parameters follow the demo bounds.
    assert(close(demo.task.max_speed, .4f) && close(demo.task.rush_speed, .32f) &&
           close(demo.task.max_wz, .55f));
    assert(demo.task.first_burst && demo.task.approach_limit_m == 8.f);
    const auto& rush_sources = demo.field_sources.at("task.rush_speed");
    assert(std::find(rush_sources.begin(), rush_sources.end(), "configureDemo legacy bounds") !=
           rush_sources.end());
    assert(rush_sources.back() == "legacy_field_v1 final field overrides");
    auto assumed = resolveLegacyRuntimeConfig(parse({"test",
                                                     "--demo-mode",
                                                     "carry_once",
                                                     "--controlled-empty-field",
                                                     "--assume-all-safe",
                                                     "--assume-injured-trip",
                                                     "--allow-mechanical-pitch-model",
                                                     "--pitch-presets",
                                                     "500,4000,4000"}),
                                              diagnostics);
    assert(assumed.task.assume_all_safe && assumed.task.assume_injured_trip &&
           close(assumed.task.turn_min_rad, .25f));
    assert(assumed.task.intermediate_pitch_cdeg == 2000 &&
           close(assumed.task.intermediate_to_near_m, .27f));
    assert(!assumed.match.require_measured_progress && assumed.request.controlled_ignore_clearance);
    assert(assumption(assumed, "all_evidence_assumed") &&
           assumption(assumed, "first_delivery_assumed"));

    TemporaryCalibration calibration;
    request = Config{};
    request.task_calibration_file = calibration.path;
    request.startup_advance_ms = 2000;
    request.startup_speed_mps = .25f;
    request.search_cues = true;
    request.controlled_empty_field = true;
    request.hardware = true;
    request.known_frame_angle = 0;
    request.known_frame_id = 17;
    diagnostics.str("");
    diagnostics.clear();
    auto measured = resolveLegacyRuntimeConfig(request, diagnostics);
    assert(measured.task_calibrated && measured.a6_done_flag && measured.a6_known &&
           measured.a6_done_settle_ms == 900);
    assert(measured.frame_views.size() == 2 && measured.capture.holding.size() == 2 &&
           measured.view_image_polygon);
    assert(measured.task.near_pitch_cdeg == 3500 && close(measured.task.mouth_y_m, .171f));
    assert(close(measured.task.hold_center_y_m, .13096f) && close(measured.load_half_width, .075f));
    assert(!measured.task.enable_search_cues && !measured.task.enable_short_push);
    assert(measured.task.attempt_budget_us == 0 && measured.task.startup_advance_us == 500000);
    assert(close(measured.task.max_speed, .4f) && close(measured.task.approach_limit_m, 8.f));
    assert(measured.task_calibration_version == "synthetic-runtime-test");
    assert(measured.calibration_audit.status == "TEMP_ASSUMPTION_NOT_ACCEPTANCE" &&
           measured.calibration_audit.pending.size() == 2);
    assert(assumption(measured, "a6_done_flag") && assumption(measured, "image_polygon_holding") &&
           assumption(measured, "task_calibration_provenance"));
    const auto log = diagnostics.str();
    assert(log.find("[FIELD_CARRY]") < log.find("[A6]"));
    assert(log.find("[A6]") < log.find("[MULTI_VIEW]"));
    request.known_frame_id = 0;
    bool rejected = false;
    try {
        resolveLegacyRuntimeConfig(request, diagnostics);
    } catch (const std::runtime_error& e) {
        rejected = std::string(e.what()) ==
                   "Done-flag A6 requires --known-frame-angle and "
                   "--known-frame-id plus schema 2 views before hardware output";
    }
    assert(rejected);

    std::ostringstream json;
    field.request.model_path = "a\"quoted\\model.rknn";
    field.task.blacklist_us = 9007199254740993ULL;
    writeEffectiveConfig(field, json);
    cv::FileStorage decoded(
        json.str(), cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
    assert(decoded.isOpened());
    assert(std::string(decoded["request"]["model_path"]) == field.request.model_path);
    assert(std::string(decoded["capture"]["mouth_y_m"]) == "+infinity");
    assert(std::string(decoded["task"]["blacklist_us"]) == "9007199254740993");
    assert(double(decoded["startup"]["effective_advance_us"]) == 1250000);
    assert(int(decoded["startup"]["requested_advance_ms"]) == 5000);
    assert(close(float(decoded["task"]["turn_wz"]), .55f));
    assert(decoded["field_sources"].size() > 400 && decoded["task"].size() > 350);
    assert(json.str().find(".Inf") == std::string::npos &&
           json.str().find(".Nan") == std::string::npos);
    std::ostringstream tuning_log;
    printRuntimeTuning(field, tuning_log);
    assert(tuning_log.str().rfind("[CONTROL_TUNING]", 0) == 0 && tuning_log.str().back() == '\n');
    {
        TemporaryCalibration single;
        std::ifstream input(single.path);
        std::string data((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        auto rep=[&](const std::string& a,const std::string& b) {
            auto pos=data.find(a); assert(pos!=std::string::npos); data.replace(pos,a.size(),b);
        };
        rep("\"schema_version\":2", "\"schema_version\":2,\"capture_verification\":\"single_view\"");
        rep("\"pitch_cdeg\":3500", "\"pitch_cdeg\":2000");
        rep(",\n    {\"pitch_cdeg\":2000,\"calibrated\":1,\"fully_observable\":1,\"polygon\":[[50,50],[350,50],[350,350],[50,350]]}", "");
        rep("\"a6_done_settle_ms\":900", "\"a6_done_settle_ms\":100");
        TemporaryCalibration center_reference;
        {
            std::ofstream ref(center_reference.path);
            ref << R"JSON({"schema_version":3,"feature":"bbox_center_xy","image_size":[1280,720],
                "camera_pitch_cdeg":2000,"runtime_enabled":true,"approach_camera_pitch_cdeg":0,"classes":{
                "ordinary_supply":{"center_bounds_px":null,"reference_center_px":{"front":[637.5,226],"rear":[668,489]},"target_center_px":[647.5,337]},
                "injured_person":{"center_bounds_px":[395,236,906,479],"target_center_px":[647.5,337]},
                "core_supply":{"center_bounds_px":[268.5,236,1017,555],"target_center_px":[647.5,337]}}})JSON";
        }
        rep("\"schema_version\":2", "\"image_trigger_reference\":\"" +
            std::string(center_reference.path) + "\",\"schema_version\":2");
        { std::ofstream out(single.path); out<<data; }
        auto input_config=parse({"test","--task-calibration",single.path,
            "--pitch-presets","-3000,0,2000","--hardware","--imu",
            "--known-frame-angle","0","--known-frame-id","7"});
        auto jaw=resolveLegacyRuntimeConfig(input_config,diagnostics);
        assert(jaw.capture.grasp_center_ranges.size() == 3);
        assert(jaw.capture.pixel_grasp_enabled && jaw.task.pixel_grasp_enabled);
        assert(close(jaw.task.pixel_align_gain_rps_per_px, .004f));
        assert(close(jaw.task.pixel_align_max_wz, .20f));
        assert(close(jaw.task.pixel_align_tolerance_px, 12.f));
        assert(close(jaw.capture.pixel_align_tolerance_px, 12.f));
        assert(close(jaw.task.pixel_align_restart_px, 24.f));
        assert(close(jaw.task.pixel_align_approach_tolerance_px, 24.f));
        assert(close(jaw.task.pixel_align_approach_restart_px, 40.f));
        assert(jaw.task.pixel_align_brake_us == 150000);
        assert(close(jaw.task.pixel_align_brake_motion_px, 4.f));
        assert(jaw.task.pixel_align_stable_us == 200000 && jaw.task.pixel_align_stable_frames == 3);
        for (const auto& range : jaw.capture.grasp_center_ranges) {
            assert(range.pitch_cdeg == 2000 && range.image_width_px == 1280 && range.image_height_px == 720);
            assert(range.target_center_px == cv::Point2f(647.5f,337.f));
            if (range.label == "ordinary_supply") {
                assert(!range.lateral_measured && range.area.y1 == 226.f && range.area.y2 == 489.f);
                continue;
            }
            assert(range.area.y1 == 236.f);
            assert(range.area.x1 == (range.label == "injured_person" ? 395.f : 268.5f));
            assert(range.area.y2 == (range.label == "injured_person" ? 479.f : 555.f));
        }
        std::ostringstream center_dump;
        writeEffectiveConfig(jaw, center_dump);
        assert(center_dump.str().find("grasp_center_ranges") != std::string::npos);
        assert(!jaw.task.require_multi_view && jaw.frame_views.size()==1);
        assert(jaw.task.far_pitch_cdeg==-3000 && jaw.task.track_pitch_cdeg==0 &&
               jaw.task.near_pitch_cdeg==2000 && jaw.task.intermediate_pitch_cdeg==0);
        assert(jaw.task.assume_view_pitch_cdeg==2000);
        assert(close(jaw.task.assume_rush_trigger_m,jaw.task.grasp_trigger_y_m));
        assert(jaw.a6_done_flag && jaw.a6_done_settle_ms==100);
        assert(jaw.task.field_carry && jaw.task.assume_push_m>0); // user keeps closed-jaw pushing
        // A real image-reference configuration, not hardcoded controller-only test values.
        std::ifstream original_reference(center_reference.path);
        const std::string original((std::istreambuf_iterator<char>(original_reference)), {});
        const auto write_alignment = [&](const std::string& settings) {
            auto configured = original;
            configured.insert(1, "\"pixel_alignment\":" + settings + ",");
            std::ofstream reference(center_reference.path);
            reference << configured;
        };
        write_alignment(R"({"pixel_align_gain_rps_per_px":0.003,"pixel_align_tolerance_px":10,
                             "pixel_align_restart_px":22,"pixel_align_stable_us":250000,
                             "pixel_align_approach_tolerance_px":26,"pixel_align_approach_restart_px":42,
                             "pixel_align_brake_us":180000,"pixel_align_brake_motion_px":3})");
        auto configured = resolveLegacyRuntimeConfig(input_config, diagnostics);
        assert(close(configured.task.pixel_align_gain_rps_per_px, .003f));
        assert(close(configured.capture.pixel_align_tolerance_px, 10.f));
        assert(close(configured.task.pixel_align_restart_px, 22.f));
        assert(configured.task.pixel_align_stable_us == 250000);
        assert(close(configured.task.pixel_align_approach_tolerance_px, 26.f));
        assert(close(configured.task.pixel_align_approach_restart_px, 42.f));
        assert(configured.task.pixel_align_brake_us == 180000);
        assert(close(configured.task.pixel_align_brake_motion_px, 3.f));
        std::ostringstream alignment_dump;
        writeEffectiveConfig(configured, alignment_dump);
        assert(alignment_dump.str().find("pixel_align_approach_tolerance_px") != std::string::npos);
        assert(alignment_dump.str().find("pixel_align_brake_us") != std::string::npos);
        for (const std::string invalid : {
                 R"({"pixel_align_gain_rps_per_px":0})",
                 R"({"pixel_align_approach_restart_px":24})",
                 R"({"pixel_align_approach_tolerance_px":0})",
                 R"({"pixel_align_brake_us":-1})",
                 R"({"pixel_align_brake_us":1.5})",
                 R"({"pixel_align_brake_motion_px":0})",
                 R"({"pixel_align_restart_px":12})",
                 R"({"pixel_align_stable_frames":2})",
                 R"({"pixel_align_stable_frames":3.5})",
                 R"({"pixel_align_stable_us":-1})"}) {
            write_alignment(invalid);
            bool rejected = false;
            try { resolveLegacyRuntimeConfig(input_config, diagnostics); }
            catch (const std::exception&) { rejected = true; }
            assert(rejected);
        }
    }
    std::cout << "Legacy runtime resolution and effective configuration passed\n";
}
