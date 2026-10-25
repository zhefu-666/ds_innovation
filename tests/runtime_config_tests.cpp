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
    for (auto value : values) args.push_back(const_cast<char*>(value));
    return parseArgs(int(args.size()), args.data());
}

bool close(float a, float b) { return std::abs(a-b)<1e-6f; }

bool assumption(const ResolvedRuntimeConfig& r, const char* id) {
    return std::any_of(r.assumptions.begin(), r.assumptions.end(),
                       [&](const auto& a) { return a.id==id; });
}

struct TemporaryCalibration {
    char path[64] = "/tmp/rescue-runtime-calibration-XXXXXX";
    TemporaryCalibration() {
        const int fd=mkstemp(path);assert(fd>=0);::close(fd);
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
    ~TemporaryCalibration() { unlink(path); }
};
}

int main() {
    std::ostringstream diagnostics;
    Config request;
    request.push_replay="recorded.json";
    request.startup_advance_ms=5000;request.startup_speed_mps=.25f;request.turn_wz_rps=.7f;
    auto replay=resolveLegacyRuntimeConfig(request,diagnostics);
    assert(replay.policy_name=="legacy_replay_v1");
    assert(!replay.task.field_carry&&!replay.task.field_rush&&!replay.task.first_burst);
    assert(replay.task.startup_advance_us==5000000&&close(replay.task.startup_advance_speed,.25f));
    assert(close(replay.task.turn_wz,.7f)&&close(replay.task.rush_speed,TaskTuning{}.rush_speed));
    assert(diagnostics.str().empty());
    assert(!assumption(replay,"legacy_field_transport"));

    request.push_replay.clear();
    auto field=resolveLegacyRuntimeConfig(request,diagnostics);
    assert(field.policy_name=="legacy_field_v1");
    assert(field.task.startup_advance_us==1250000&&close(field.task.startup_advance_speed,1.f));
    assert(field.request.startup_advance_ms==5000&&close(field.request.startup_speed_mps,.25f));
    assert(field.task.field_carry&&field.task.field_rush&&field.task.first_burst);
    assert(close(field.task.turn_wz,.55f)&&close(field.task.rush_speed,.32f));
    assert(field.task.approach_limit_m==8.f&&field.task.retreat_limit_m==4.f);
    assert(field.ground_contact.accept_size_mismatch&&field.ground_contact.use_silhouette);
    assert(field.match.duration_us==180000000&&field.match.require_measured_progress);
    assert(field.motion_link.period_ms==40&&field.motion_link.command_timeout_ms==150);
    assert(field.field_sources.at("task.turn_wz").back()=="legacy_field_v1 final field overrides");
    assert(assumption(field,"legacy_field_transport"));
    request.startup_advance_ms=0;
    auto no_start=resolveLegacyRuntimeConfig(request,diagnostics);
    assert(no_start.task.startup_advance_us==0&&close(no_start.task.startup_advance_speed,.25f));

    auto demo=resolveLegacyRuntimeConfig(parse({"test","--demo-mode","carry_once","--demo-seconds","13"}),diagnostics);
    assert(demo.task.demo_carry_once&&demo.task.startup_advance_us==0);
    assert(demo.task.attempt_budget_us==13000000&&demo.match.duration_us==13000000);
    // Preserve the entire historical chain: field parameters follow the demo bounds.
    assert(close(demo.task.max_speed,.4f)&&close(demo.task.rush_speed,.32f)&&close(demo.task.max_wz,.55f));
    assert(demo.task.first_burst&&demo.task.approach_limit_m==8.f);
    const auto& rush_sources=demo.field_sources.at("task.rush_speed");
    assert(std::find(rush_sources.begin(),rush_sources.end(),"configureDemo legacy bounds")!=rush_sources.end());
    assert(rush_sources.back()=="legacy_field_v1 final field overrides");
    auto assumed=resolveLegacyRuntimeConfig(parse({"test","--demo-mode","carry_once","--controlled-empty-field",
        "--assume-all-safe","--assume-injured-trip","--allow-mechanical-pitch-model","--pitch-presets","500,4000,4000"}),diagnostics);
    assert(assumed.task.assume_all_safe&&assumed.task.assume_injured_trip&&close(assumed.task.turn_min_rad,.25f));
    assert(assumed.task.intermediate_pitch_cdeg==2000&&close(assumed.task.intermediate_to_near_m,.27f));
    assert(!assumed.match.require_measured_progress&&assumed.request.controlled_ignore_clearance);
    assert(assumption(assumed,"all_evidence_assumed")&&assumption(assumed,"first_delivery_assumed"));

    TemporaryCalibration calibration;
    request=Config{};
    request.task_calibration_file=calibration.path;
    request.startup_advance_ms=2000;request.startup_speed_mps=.25f;
    request.search_cues=true;request.controlled_empty_field=true;
    request.hardware=true;request.known_frame_angle=0;request.known_frame_id=17;
    diagnostics.str("");diagnostics.clear();
    auto measured=resolveLegacyRuntimeConfig(request,diagnostics);
    assert(measured.task_calibrated&&measured.a6_done_flag&&measured.a6_known&&measured.a6_done_settle_ms==900);
    assert(measured.frame_views.size()==2&&measured.capture.holding.size()==2&&measured.view_image_polygon);
    assert(measured.task.near_pitch_cdeg==3500&&close(measured.task.mouth_y_m,.171f));
    assert(close(measured.task.hold_center_y_m,.13096f)&&close(measured.load_half_width,.075f));
    assert(!measured.task.enable_search_cues&&!measured.task.enable_short_push);
    assert(measured.task.attempt_budget_us==0&&measured.task.startup_advance_us==500000);
    assert(close(measured.task.max_speed,.4f)&&close(measured.task.approach_limit_m,8.f));
    assert(measured.task_calibration_version=="synthetic-runtime-test");
    assert(measured.calibration_audit.status=="TEMP_ASSUMPTION_NOT_ACCEPTANCE"&&measured.calibration_audit.pending.size()==2);
    assert(assumption(measured,"a6_done_flag")&&assumption(measured,"image_polygon_holding")&&assumption(measured,"task_calibration_provenance"));
    const auto log=diagnostics.str();
    assert(log.find("[FIELD_CARRY]")<log.find("[A6]"));
    assert(log.find("[A6]")<log.find("[MULTI_VIEW]"));
    request.known_frame_id=0;
    bool rejected=false;
    try { resolveLegacyRuntimeConfig(request,diagnostics); }
    catch(const std::runtime_error& e) {
        rejected=std::string(e.what())=="Done-flag A6 requires --known-frame-angle and --known-frame-id plus schema 2 views before hardware output";
    }
    assert(rejected);

    std::ostringstream json;
    field.request.model_path="a\"quoted\\model.rknn";
    field.task.blacklist_us=9007199254740993ULL;
    writeEffectiveConfig(field,json);
    cv::FileStorage decoded(json.str(),cv::FileStorage::READ|cv::FileStorage::MEMORY|cv::FileStorage::FORMAT_JSON);
    assert(decoded.isOpened());
    assert(std::string(decoded["request"]["model_path"])==field.request.model_path);
    assert(std::string(decoded["capture"]["mouth_y_m"])=="+infinity");
    assert(std::string(decoded["task"]["blacklist_us"])=="9007199254740993");
    assert(double(decoded["startup"]["effective_advance_us"])==1250000);
    assert(int(decoded["startup"]["requested_advance_ms"])==5000);
    assert(close(float(decoded["task"]["turn_wz"]),.55f));
    assert(decoded["field_sources"].size()>400&&decoded["task"].size()>350);
    assert(json.str().find(".Inf")==std::string::npos&&json.str().find(".Nan")==std::string::npos);
    std::ostringstream tuning_log;
    printRuntimeTuning(field,tuning_log);
    assert(tuning_log.str().rfind("[CONTROL_TUNING]",0)==0&&tuning_log.str().back()=='\n');
    std::cout << "Legacy runtime resolution and effective configuration passed\n";
}
