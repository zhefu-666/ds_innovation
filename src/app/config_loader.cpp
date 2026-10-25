#include "rescue/app/config_loader.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace rescue::app {
namespace {

using Json = nlohmann::json;
namespace fs = std::filesystem;

enum class Kind { Text, Path, Boolean, HideWindow, Integer, Number, Classes, PitchPresets };

struct Field {
    const char *group;
    const char *name;
    const char *flag;
    Kind kind;
    double minimum = 0;
    double maximum = 0;
    bool exclusive_minimum = false;
};

const std::vector<Field> &fields() {
    static const std::vector<Field> all{
        {"hardware", "uart_port", "--port", Kind::Path},
        {"hardware", "baudrate", "--baud", Kind::Integer, 1, 4000000},
        {"hardware", "camera_index", "--camera", Kind::Integer, 0, 255},
        {"hardware", "frame_width", "--width", Kind::Integer, 1, 16384},
        {"hardware", "frame_height", "--height", Kind::Integer, 1, 16384},
        {"hardware", "fps", "--fps", Kind::Integer, 1, 1000},
        {"hardware", "imu", "--imu", Kind::Boolean},
        {"hardware", "imu_port", "--imu-port", Kind::Path},
        {"hardware", "imu_baud", "--imu-baud", Kind::Integer, 1, 4000000},
        {"hardware", "imu_timeout_ms", "--imu-timeout-ms", Kind::Integer, 1, 10000},
        {"hardware", "pitch_feedback", "--pitch-feedback", Kind::Boolean},
        {"models", "rknn_library", "--rknn-library", Kind::Path},
        {"models", "model_path", "--model", Kind::Path},
        {"models", "pose_model_path", "--pose-model", Kind::Path},
        {"models", "pose_model_blue_path", "--pose-model-blue", Kind::Path},
        {"models", "pose_model_red_path", "--pose-model-red", Kind::Path},
        {"models", "input_size", "--input-size", Kind::Integer, 1, 8192},
        {"models", "confidence", "--conf", Kind::Number, 0, 1},
        {"models", "injured_confidence", "--injured-conf", Kind::Number, 0, 1},
        {"models", "nms", "--nms", Kind::Number, 0, 1},
        {"models", "pose_confidence", "--pose-conf", Kind::Number, 0, 1},
        {"models", "pose_keypoint_confidence", "--pose-kpt-conf", Kind::Number, 0, 1},
        {"models", "detect_core_mask", "--detect-core", Kind::Integer, -1, 4},
        {"models", "pose_core_mask", "--pose-core", Kind::Integer, -1, 4},
        {"models", "parallel_inference", "--parallel-infer", Kind::Boolean},
        {"models", "use_cuda", "--cuda", Kind::Boolean},
        {"models", "class_names", "--classes", Kind::Classes},
        {"calibration", "calibration_file", "--calibration", Kind::Path},
        {"calibration", "zone_geometry_file", "--zone-geometry", Kind::Path},
        {"calibration", "task_calibration_file", "--task-calibration", Kind::Path},
        {"calibration", "zone_color_file", "--zone-color-calibration", Kind::Path},
        {"calibration", "pitch_presets_cdeg", "--pitch-presets", Kind::PitchPresets},
        {"calibration", "allow_mechanical_pitch_model", "--allow-mechanical-pitch-model", Kind::Boolean},
        {"calibration", "known_frame_angle", "--known-frame-angle", Kind::Integer, 0, 20},
        {"calibration", "known_frame_id", "--known-frame-id", Kind::Integer, 1, 255},
        {"behavior", "sensor_timeout_ms", "--sensor-timeout-ms", Kind::Integer, 1, 60000},
        {"behavior", "tof_stop_distance_m", "--tof-stop-m", Kind::Number, 0, 100, true},
        {"behavior", "imu_tilt_limit_deg", "--imu-tilt-deg", Kind::Number, 0, 180, true},
        {"behavior", "match_seconds", "--match-seconds", Kind::Integer, 1, 86400},
        {"behavior", "startup_advance_ms", "--startup-advance-ms", Kind::Integer, 0, 60000},
        {"behavior", "startup_speed_mps", "--startup-speed", Kind::Number, 0, 1, true},
        {"behavior", "scan_wz_rps", "--scan-wz", Kind::Number, 0, 3, true},
        {"behavior", "turn_wz_rps", "--turn-wz", Kind::Number, 0, 3, true},
        {"behavior", "clear_neighbours", "--clear-neighbours", Kind::Boolean},
        {"behavior", "search_cues", "--search-cues", Kind::Boolean},
        {"behavior", "controlled_ignore_clearance", "--controlled-ignore-clearance", Kind::Boolean},
        {"behavior", "controlled_empty_field", "--controlled-empty-field", Kind::Boolean},
        {"behavior", "assume_all_safe", "--assume-all-safe", Kind::Boolean},
        {"behavior", "assume_injured_trip", "--assume-injured-trip", Kind::Boolean},
        {"runtime", "team", "--team", Kind::Text},
        {"runtime", "demo_mode", "--demo-mode", Kind::Text},
        {"runtime", "demo_seconds", "--demo-seconds", Kind::Integer, 1, 60},
        {"runtime", "push_replay", "--push-replay", Kind::Path},
        {"runtime", "geometry_replay", "--geometry-replay", Kind::Path},
        {"runtime", "detect_image", "--detect-image", Kind::Path},
        {"runtime", "keypoints_file", "--keypoints-file", Kind::Path},
        {"runtime", "show", "--no-show", Kind::HideWindow},
        {"runtime", "save_output", "--save", Kind::Boolean},
        {"runtime", "dry_run", "--dry-run", Kind::Boolean},
        {"runtime", "hardware", "--hardware", Kind::Boolean},
        {"runtime", "check_config", "--check-config", Kind::Boolean},
        {"runtime", "telemetry", "--telemetry", Kind::Boolean},
        {"runtime", "telemetry_fps", "--telemetry-fps", Kind::Integer, 1, 30},
        {"runtime", "telemetry_file", "--telemetry-file", Kind::Path},
        {"runtime", "auto_run", "--auto-run", Kind::Boolean},
        {"runtime", "no_match_time_limit", "--no-match-time-limit", Kind::Boolean},
        {"runtime", "vision_log_dir", "--vision-log-dir", Kind::Path},
        {"runtime", "match_socket", "--match-socket", Kind::Path},
        {"runtime", "require_instance_masks", "--require-masks", Kind::Boolean},
    };
    return all;
}

const Field *findFlag(const std::string &flag) {
    for (const auto &field : fields()) if (flag == field.flag) return &field;
    return nullptr;
}

bool takesValue(const Field &field) {
    return field.kind != Kind::Boolean && field.kind != Kind::HideWindow;
}

std::string absolutePath(const std::string &path, const fs::path &directory) {
    if (path.empty()) return path;
    const fs::path value(path);
    if (value.is_absolute()) return value.lexically_normal().string();
    if (directory.empty())
        throw std::runtime_error("Cannot resolve relative path without an accessible working directory: " + path);
    return (directory / value).lexically_normal().string();
}

[[noreturn]] void invalid(const std::string &key, const std::string &reason) {
    throw std::runtime_error(key + " " + reason);
}

void requireInteger(const Json &value, const std::string &key, double low, double high) {
    if (!value.is_number_integer()) invalid(key, "must be an integer");
    const double number = value.get<double>();
    if (number < low || number > high)
        invalid(key, "must be in [" + std::to_string(low) + "," + std::to_string(high) + "]");
}

std::string fieldValue(const Field &field, const Json &value, const fs::path &directory) {
    const std::string key = std::string(field.group) + "." + field.name;
    if (field.kind == Kind::Text || field.kind == Kind::Path) {
        if (!value.is_string()) invalid(key, "must be a string");
        const auto text = value.get<std::string>();
        if (text.find('\0') != std::string::npos) invalid(key, "must not contain NUL");
        if (key == "runtime.team" && text != "blue" && text != "red")
            invalid(key, "must be blue or red");
        if (key == "runtime.demo_mode" && text != "none" && text != "recognize" &&
            text != "search" && text != "carry_once")
            invalid(key, "must be none, recognize, search or carry_once");
        if (field.kind == Kind::Path && !(key == "runtime.vision_log_dir" && text == "none"))
            return absolutePath(text, directory);
        return text;
    }
    if (field.kind == Kind::Boolean || field.kind == Kind::HideWindow) {
        if (!value.is_boolean()) invalid(key, "must be a boolean");
        return value.get<bool>() ? "true" : "false";
    }
    if (field.kind == Kind::Integer) {
        requireInteger(value, key, field.minimum, field.maximum);
        if ((key == "models.detect_core_mask" || key == "models.pose_core_mask") &&
            value != -1 && value != 0 && value != 1 && value != 2 && value != 4)
            invalid(key, "must be one of -1, 0, 1, 2, 4");
        if (key == "calibration.known_frame_angle" && value != 0 && value != 20)
            invalid(key, "must be 0 or 20");
        if (key == "hardware.imu_baud" && value != 9600 && value != 115200 &&
            value != 230400 && value != 460800 && value != 921600)
            invalid(key, "must be a supported IMU baudrate");
        return value.dump();
    }
    if (field.kind == Kind::Number) {
        if (!value.is_number()) invalid(key, "must be a number");
        const double number = value.get<double>();
        if (!std::isfinite(number) || number < field.minimum || number > field.maximum)
            invalid(key, "is outside the finite supported range");
        if (field.exclusive_minimum && number == field.minimum)
            invalid(key, "must be positive");
        return value.dump();
    }
    if (!value.is_array()) invalid(key, "must be an array");
    if (value.empty()) invalid(key, "must not be empty");
    std::string csv;
    int previous_pitch = -4000;
    if (field.kind == Kind::PitchPresets && value.size() != 3)
        invalid(key, "must contain FAR, TRACK, NEAR");
    for (const auto &item : value) {
        std::string text;
        if (field.kind == Kind::PitchPresets) {
            requireInteger(item, key, -4000, 4000);
            const int pitch = item.get<int>();
            if (pitch < previous_pitch) invalid(key, "requires FAR <= TRACK <= NEAR");
            previous_pitch = pitch;
            text = item.dump();
        } else {
            if (!item.is_string()) invalid(key, "must contain strings");
            text = item.get<std::string>();
            if (text.empty() || text.find(',') != std::string::npos ||
                std::any_of(text.begin(), text.end(), [](unsigned char c) { return c < 0x20; }) ||
                std::isspace(static_cast<unsigned char>(text.front())) ||
                std::isspace(static_cast<unsigned char>(text.back())))
                invalid(key, "must contain nonempty class names without commas or surrounding whitespace");
        }
        if (!csv.empty()) csv += ',';
        csv += text;
    }
    return csv;
}

Json readJson(const std::string &path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open runtime configuration: " + path);
    std::vector<std::set<std::string>> object_keys;
    auto uniqueKeys = [&](int, Json::parse_event_t event, Json &parsed) {
        if (event == Json::parse_event_t::object_start) object_keys.emplace_back();
        else if (event == Json::parse_event_t::object_end) object_keys.pop_back();
        else if (event == Json::parse_event_t::key &&
                 !object_keys.back().insert(parsed.get<std::string>()).second)
            throw std::runtime_error("Duplicate JSON key: " + parsed.get<std::string>());
        return true;
    };
    return Json::parse(input, uniqueKeys);
}

std::vector<std::string> fileArguments(AppConfigOptions &options) {
    try {
        const auto root = readJson(options.config_file);
        if (!root.is_object()) invalid("root", "must be an object");
        const std::set<std::string> root_keys{
            "schema_version", "behavior_policy", "hardware", "models", "calibration", "behavior", "runtime"};
        for (const auto &item : root.items())
            if (!root_keys.count(item.key())) invalid(item.key(), "is an unknown key");
        if (!root.contains("schema_version")) invalid("schema_version", "is required");
        requireInteger(root.at("schema_version"), "schema_version", 1, 1);
        if (!root.contains("behavior_policy") || !root.at("behavior_policy").is_string())
            invalid("behavior_policy", "must be a string");
        options.behavior_policy = root.at("behavior_policy").get<std::string>();
        if (options.behavior_policy != "legacy_field_v1" && options.behavior_policy != "legacy_replay_v1")
            invalid("behavior_policy", "must be legacy_field_v1 or legacy_replay_v1");
        for (const auto &item : root.items()) {
            if (item.key() == "schema_version" || item.key() == "behavior_policy") continue;
            if (!item.value().is_object()) invalid(item.key(), "must be an object");
            for (const auto &entry : item.value().items()) {
                bool known = false;
                for (const auto &field : fields())
                    if (item.key() == field.group && entry.key() == field.name) { known = true; break; }
                if (!known) invalid(item.key() + "." + entry.key(), "is an unknown key");
            }
        }
        std::vector<std::string> arguments;
        const auto directory = fs::path(options.config_file).parent_path();
        for (const auto &field : fields()) {
            if (!root.contains(field.group) || !root.at(field.group).contains(field.name)) continue;
            const auto &value = root.at(field.group).at(field.name);
            const auto text = fieldValue(field, value, directory);
            options.file_field_sources.emplace_back(field.name, "config:" + options.config_file + "#" + field.group + "." + field.name);
            if (field.kind == Kind::Boolean || field.kind == Kind::HideWindow) {
                // All positive switches default false; only show defaults true.
                if (value.get<bool>() != (field.kind == Kind::HideWindow)) arguments.emplace_back(field.flag);
            } else {
                arguments.emplace_back(field.flag);
                arguments.push_back(text);
            }
        }
        return arguments;
    } catch (const std::exception &error) {
        throw std::runtime_error("Runtime configuration " + options.config_file + ": " + error.what());
    }
}

} // namespace

LoadedAppConfig loadAppConfig(int argc, char **argv) {
    LoadedAppConfig result;
    std::vector<std::string> cli;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--config" || argument == "--dump-config") {
            if (i + 1 >= argc || std::string(argv[i + 1]).empty())
                throw std::runtime_error("Missing value for " + argument);
            auto &destination = argument == "--config" ? result.options.config_file : result.options.dump_config_file;
            if (!destination.empty()) throw std::runtime_error("Specify " + argument + " only once");
            const std::string value = argv[++i];
            if (argument == "--dump-config" && value == "-") {
                destination = value;
            } else {
                fs::path directory;
                if (fs::path(value).is_relative()) {
                    std::error_code error;
                    directory = fs::current_path(error);
                    if (error)
                        throw std::runtime_error("Cannot resolve relative " + argument + " path without an accessible working directory: " + value);
                }
                destination = absolutePath(value, directory);
            }
            continue;
        }
        cli.push_back(argument);
        result.options.explicit_cli_flags.push_back(argument);
        if (const auto *field = findFlag(argument)) {
            result.options.explicit_cli_fields.emplace_back(field->name);
            if (takesValue(*field) && i + 1 < argc) cli.emplace_back(argv[++i]);
        }
    }
    std::vector<std::string> arguments{argc > 0 ? argv[0] : "rescue_upper_host"};
    if (!result.options.config_file.empty()) {
        auto from_file = fileArguments(result.options);
        arguments.insert(arguments.end(), from_file.begin(), from_file.end());
    }
    arguments.insert(arguments.end(), cli.begin(), cli.end());
    std::vector<char *> pointers;
    for (auto &argument : arguments) pointers.push_back(argument.data());
    result.config = parseArgs(static_cast<int>(pointers.size()), pointers.data());
    const auto normalized = [&](const char *field, const std::string &reason) {
        result.options.normalized_field_sources.emplace_back(field, "parseArgs: " + reason);
    };
    if (result.config.assume_all_safe)
        normalized("controlled_ignore_clearance", "assume_all_safe sets controlled_ignore_clearance=true");
    if (result.config.demo_mode != "none") {
        const auto reason = "demo_mode=" + result.config.demo_mode;
        if (result.config.demo_mode == "recognize") normalized("dry_run", reason + " sets dry_run=true");
        normalized("startup_advance_ms", reason + " sets startup_advance_ms=0");
        normalized("startup_speed_mps", reason + " sets startup_speed_mps=0.10");
        normalized("scan_wz_rps", reason + " bounds scan_wz_rps to 0.25");
        normalized("turn_wz_rps", reason + " bounds turn_wz_rps to 0.25");
        normalized("match_seconds", reason + " bounds match_seconds by demo_seconds");
    }
    const auto &team_pose = result.config.team == "blue" ? result.config.pose_model_blue_path : result.config.pose_model_red_path;
    if (!team_pose.empty())
        normalized("pose_model_path", "team=" + result.config.team + " selects pose_model_" + result.config.team + "_path");
    if (!result.options.behavior_policy.empty()) {
        const std::string expected = result.config.push_replay.empty() ? "legacy_field_v1" : "legacy_replay_v1";
        if (result.options.behavior_policy != expected)
            throw std::runtime_error("behavior_policy " + result.options.behavior_policy + " does not match the selected operation; expected " + expected);
    }
    return result;
}

} // namespace rescue::app
