#pragma once

#include "rescue/capture_monitor.hpp"
#include "rescue/config.hpp"
#include "rescue/geometry_pipeline.hpp"
#include "rescue/match_control.hpp"
#include "rescue/motion_link.hpp"
#include "rescue/multi_view_capture.hpp"
#include <iosfwd>
#include <map>
#include <string>
#include <vector>

namespace rescue {

struct RuntimeAssumption {
    std::string id;
    std::string scope;
    std::string replacement;
    std::string evidence;
    std::string status = "not_field_validated";
};

struct TaskCalibrationAudit {
    int schema_version = 0;
    std::string version, status, dimensions_source, assume_source;
    std::string action_mapping_source, a6_done_settle_source, frame_view_matching_source;
    std::vector<std::string> pending, provided_fields;
    std::map<std::string, int> verification_flags;
    std::string audit_error;
};

struct RuntimeIdentity {
    std::string config_file, working_directory, executable;
    std::string audit_error;
    std::string rknn_library_lookup = "not_recorded";
    std::vector<std::string> arguments;
    std::map<std::string, std::string> resource_paths;
    bool rknn_compiled = false, telemetry_compiled = false;
};

// The caller freezes this value after adding request provenance, before assembly.
struct ResolvedRuntimeConfig {
    RuntimeIdentity identity;
    Config request;
    TaskTuning task;
    CaptureConfig capture;
    GroundContactConfig ground_contact;
    MatchConfig match;
    MotionLinkConfig motion_link;
    cv::Vec<float, 6> body_envelope{};
    float measured_load_radius = 0, measured_body_radius = 0;
    float load_half_width = 0, load_half_depth = 0;
    std::vector<FrameView> frame_views;
    int feedback_open = -1, feedback_close = -1;
    bool box_area_accepted = false;
    bool a6_done_flag = false;
    int a6_done_settle_ms = 0;
    bool view_image_polygon = false;
    bool task_calibrated = false, a6_known = false;
    std::string task_calibration_version;
    TaskCalibrationAudit calibration_audit;
    std::string policy_name, policy_version = "1";
    std::vector<RuntimeAssumption> assumptions;
    std::map<std::string, std::vector<std::string>> field_sources;
};

// Keeps the legacy assignment order, including field overrides after demo policy.
ResolvedRuntimeConfig resolveLegacyRuntimeConfig(const Config&, std::ostream& diagnostics);
void printRuntimeTuning(const ResolvedRuntimeConfig&, std::ostream& diagnostics);
// JSON uses strings for nonfinite defaults and integers beyond exact JSON-double range.
void writeEffectiveConfig(const ResolvedRuntimeConfig&, std::ostream& output);

} // namespace rescue
