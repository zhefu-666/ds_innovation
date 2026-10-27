#include "rescue/app/config_loader.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

namespace fs = std::filesystem;
using rescue::app::LoadedAppConfig;

namespace {

struct TemporaryDirectory {
    fs::path path;
    unsigned sequence = 0;
    TemporaryDirectory() {
        char name[] = "/tmp/rescue-config-loader-XXXXXX";
        const auto* created = mkdtemp(name);
        if (!created)
            throw std::runtime_error("mkdtemp failed");
        path = created;
    }
    ~TemporaryDirectory() {
        fs::remove_all(path);
    }
    fs::path write(const std::string& json) {
        const auto file = path / ("runtime-" + std::to_string(sequence++) + ".json");
        std::ofstream stream(file);
        stream << json;
        if (!stream)
            throw std::runtime_error("Could not write test configuration");
        return file;
    }
};

LoadedAppConfig load(std::vector<std::string> arguments) {
    arguments.insert(arguments.begin(), "config_loader_tests");
    std::vector<char*> pointers;
    for (auto& argument : arguments)
        pointers.push_back(argument.data());
    return rescue::app::loadAppConfig(static_cast<int>(pointers.size()), pointers.data());
}

void expectFailure(const std::vector<std::string>& arguments, const std::string& message) {
    bool failed = false;
    try {
        load(arguments);
    } catch (const std::exception& error) {
        failed = true;
        if (std::string(error.what()).find(message) == std::string::npos) {
            std::cerr << "Expected: " << message << "\nReceived: " << error.what() << '\n';
            assert(false);
        }
    }
    assert(failed);
}

} // namespace

int main() {
    TemporaryDirectory directory;
    {
        const auto loaded =
            load({"--dry-run", "--team", "red", "--startup-speed", ".33", "--no-show"});
        assert(loaded.config.dry_run && !loaded.config.show && loaded.config.team == "red");
        assert(std::abs(loaded.config.startup_speed_mps - .33f) < 1e-6f);
        assert(loaded.options.config_file.empty() && loaded.options.behavior_policy.empty());
        assert(loaded.options.normalized_field_sources.empty());
        assert(loaded.options.explicit_cli_flags.size() == 4);
        // Option values that happen to name wrapper flags retain legacy parsing.
        const auto quoted = load({"--model", "--config", "--dry-run"});
        assert(quoted.config.model_path == "--config" && quoted.config.dry_run);
        assert(load({"--model", "--dump-config"}).config.model_path == "--dump-config");
        expectFailure({"--not-an-option"}, "Unknown argument");
        expectFailure({"--fps"}, "Missing value");
    }
    {
        const auto cli = load({"--detect-core", "4", "--pose-core", "3",
                               "--pose-model", "zone_pose_int8.rknn", "--parallel-infer"});
        assert(cli.config.detect_core_mask == 4 && cli.config.pose_core_mask == 3);
        assert(cli.config.parallel_inference);
        const auto multicore = directory.write(
            R"({"schema_version":1,"behavior_policy":"legacy_field_v1","models":{"detect_core_mask":4,"pose_core_mask":3,"pose_model_path":"zone_pose_int8.rknn","parallel_inference":true}})");
        const auto file = load({"--config", multicore.string()});
        assert(file.config.detect_core_mask == 4 && file.config.pose_core_mask == 3);
        assert(file.config.parallel_inference);
        const auto override = load({"--config", multicore.string(), "--detect-core", "3",
                                    "--pose-core", "4"});
        assert(override.config.detect_core_mask == 3 && override.config.pose_core_mask == 4);
        expectFailure({"--detect-core", "5"}, "Core mask must be");
        expectFailure({"--pose-core", "5"}, "Core mask must be");
    }
    const auto config = directory.write(R"({
        "schema_version": 1,
        "behavior_policy": "legacy_field_v1",
        "hardware": {"fps": 30, "imu": false},
        "models": {"model_path": "../../models/detector.rknn", "confidence": 0.3,
                   "parallel_inference": false, "use_cuda": false,
                   "class_names": ["blue", "orange", "green", "black"]},
        "calibration": {"zone_geometry_file": "landmarks.json", "pitch_presets_cdeg": [500,500,4000]},
        "behavior": {"startup_speed_mps": 0.22, "clear_neighbours": false},
        "runtime": {"show": false, "dry_run": false, "telemetry": false,
                    "hardware": false, "vision_log_dir": "none", "match_socket": "match.sock"}
    })");
    {
        const auto loaded = load({"--config", config.string()});
        assert(loaded.options.config_file == config.string());
        assert(loaded.options.behavior_policy == "legacy_field_v1");
        assert(loaded.config.fps == 30 && !loaded.config.imu);
        assert(!loaded.config.dry_run && !loaded.config.hardware && !loaded.config.telemetry);
        assert(!loaded.config.use_cuda && !loaded.config.parallel_inference &&
               !loaded.config.clear_neighbours);
        assert(!loaded.config.show);
        assert(loaded.config.model_path ==
               (directory.path / "../../models/detector.rknn").lexically_normal());
        assert(loaded.config.zone_geometry_file == directory.path / "landmarks.json");
        assert(loaded.config.match_socket == directory.path / "match.sock");
        assert(loaded.config.vision_log_dir == "none");
        assert(std::abs(loaded.config.startup_speed_mps - .22f) < 1e-6f);
        const auto source = std::find_if(loaded.options.file_field_sources.begin(),
                                         loaded.options.file_field_sources.end(),
                                         [](const auto& entry) {
                                             return entry.first == "telemetry";
                                         });
        assert(source != loaded.options.file_field_sources.end());
        assert(source->second == "config:" + config.string() + "#runtime.telemetry");
        assert(load({"--config", fs::relative(config, fs::current_path()).string()})
                   .options.config_file == config);
    }
    {
        const auto loaded = load({"--startup-speed",
                                  ".35",
                                  "--config",
                                  config.string(),
                                  "--fps",
                                  "42",
                                  "--dry-run",
                                  "--imu",
                                  "--telemetry",
                                  "--model",
                                  "relative-cli.rknn"});
        assert(loaded.config.fps == 42 && loaded.config.dry_run && loaded.config.imu &&
               loaded.config.telemetry);
        assert(loaded.config.model_path == "relative-cli.rknn");
        assert(std::abs(loaded.config.startup_speed_mps - .35f) < 1e-6f);
        assert(std::find(loaded.options.explicit_cli_fields.begin(),
                         loaded.options.explicit_cli_fields.end(),
                         "startup_speed_mps") != loaded.options.explicit_cli_fields.end());
    }
    {
        const auto file =
            directory.write(R"({"schema_version":1,"behavior_policy":"legacy_field_v1",
            "runtime":{"show":true,"dry_run":true},"models":{"use_cuda":true}})");
        const auto loaded = load({"--config", file.string(), "--no-show"});
        assert(!loaded.config.show && loaded.config.dry_run && loaded.config.use_cuda);
        const auto dump = directory.path / "effective.json";
        const auto dumped = load({"--dump-config", dump.string(), "--dry-run"});
        assert(dumped.options.dump_config_file == dump);
        assert(
            !fs::exists(dump)); // Writing the final resolved configuration belongs to the caller.
        assert(load({"--dump-config", "-"}).options.dump_config_file == "-");
    }
    {
        const auto file =
            directory.write(R"({"schema_version":1,"behavior_policy":"legacy_field_v1",
            "models":{"pose_model_blue_path":"blue.rknn","pose_model_red_path":"red.rknn"}})");
        const auto loaded = load({"--config", file.string(), "--team", "red"});
        assert(loaded.config.pose_model_path == directory.path / "red.rknn");
        assert(loaded.options.normalized_field_sources.size() == 1);
        assert(loaded.options.normalized_field_sources[0].first == "pose_model_path");
        assert(loaded.options.normalized_field_sources[0].second.find("team=red") !=
               std::string::npos);
        const auto generic = load({"--pose-model", "generic.rknn"});
        assert(generic.options.normalized_field_sources.empty());
    }
    {
        const auto demo =
            load({"--demo-mode", "recognize", "--startup-speed", ".4", "--match-seconds", "180"});
        assert(demo.config.dry_run && std::abs(demo.config.startup_speed_mps - .1f) < 1e-6f);
        assert(demo.options.normalized_field_sources.size() == 6);
        assert(demo.options.normalized_field_sources[0].first == "dry_run");
        assert(demo.options.normalized_field_sources.back().first == "match_seconds");
        const auto assumed =
            load({"--demo-mode", "carry_once", "--controlled-empty-field", "--assume-all-safe"});
        assert(assumed.options.normalized_field_sources.size() == 6);
        assert(assumed.options.normalized_field_sources[0].first == "controlled_ignore_clearance");
        assert(assumed.options.normalized_field_sources[0].second.find("assume_all_safe") !=
               std::string::npos);
    }
    {
        const auto file =
            directory.write(R"({"schema_version":1,"behavior_policy":"legacy_field_v1",
            "runtime":{"hardware":true}})");
        // Validate combinations after explicit CLI has supplied the remaining dependency.
        assert(load({"--config", file.string(), "--imu"}).config.hardware);
        expectFailure({"--config", file.string()}, "--hardware requires --imu");
    }
    {
        const auto replay =
            directory.write(R"({"schema_version":1,"behavior_policy":"legacy_replay_v1",
            "runtime":{"push_replay":"observations.json"}})");
        assert(load({"--config", replay.string()}).config.push_replay ==
               directory.path / "observations.json");
        const auto wrong =
            directory.write(R"({"schema_version":1,"behavior_policy":"legacy_field_v1",
            "runtime":{"push_replay":"observations.json"}})");
        expectFailure({"--config", wrong.string()}, "does not match the selected operation");
        const auto no_replay =
            directory.write(R"({"schema_version":1,"behavior_policy":"legacy_replay_v1"})");
        expectFailure({"--config", no_replay.string()}, "does not match the selected operation");
        expectFailure({"--config", config.string(), "--push-replay", "replay.json"},
                      "does not match the selected operation");
    }
    const auto reject = [&](const std::string& json, const std::string& message) {
        expectFailure({"--config", directory.write(json).string()}, message);
    };
    reject(R"({"behavior_policy":"legacy_field_v1"})", "schema_version is required");
    reject(R"({"schema_version":true,"behavior_policy":"legacy_field_v1"})",
           "schema_version must be an integer");
    reject(R"({"schema_version":2,"behavior_policy":"legacy_field_v1"})",
           "schema_version must be in");
    reject(R"({"schema_version":1,"behavior_policy":"strict_candidate_v1"})",
           "behavior_policy must be legacy");
    reject(R"({"schema_version":1,"behavior_policy":"legacy_field_v1","extra":1})",
           "extra is an unknown key");
    reject(R"({"schema_version":1,"schema_version":1,"behavior_policy":"legacy_field_v1"})",
           "Duplicate JSON key");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","runtime":{"dry_run":true,"dry_run":false}})",
        "Duplicate JSON key");
    reject(R"({"schema_version":1,"behavior_policy":"legacy_field_v1","runtime":[]})",
           "runtime must be an object");
    reject(R"({"schema_version":1,"behavior_policy":"legacy_field_v1","hardware":{"fps":"30"}})",
           "hardware.fps must be an integer");
    reject(R"({"schema_version":1,"behavior_policy":"legacy_field_v1","hardware":{"fps":30.0}})",
           "hardware.fps must be an integer");
    reject(R"({"schema_version":1,"behavior_policy":"legacy_field_v1","hardware":{"fps":1001}})",
           "hardware.fps must be in");
    reject(R"({"schema_version":1,"behavior_policy":"legacy_field_v1","runtime":{"dry_run":0}})",
           "runtime.dry_run must be a boolean");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","behavior":{"sensor_timeout_ms":-1}})",
        "sensor_timeout_ms must be in");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","behavior":{"sensor_timeout_ms":18446744073709551615}})",
        "sensor_timeout_ms must be in");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","models":{"confidence":true}})",
        "models.confidence must be a number");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","models":{"confidence":1.1}})",
        "confidence is outside");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","models":{"detect_core_mask":5}})",
        "detect_core_mask must be in");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","hardware":{"imu_baud":100000}})",
        "must be a supported IMU baudrate");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","runtime":{"team":"yellow"}})",
        "runtime.team must be blue or red");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","runtime":{"demo_mode":"experimental"}})",
        "runtime.demo_mode must be none");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","models":{"class_names":["blue,red"]}})",
        "class_names must contain");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","behavior":{"tof_stop_distance_m":0}})",
        "tof_stop_distance_m must be positive");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","behavior":{"startup_speed_mps":0}})",
        "startup_speed_mps must be positive");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","runtime":{"detect_image":"bad\u0000path"}})",
        "must not contain NUL");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","calibration":{"pitch_presets_cdeg":[500,4000,0]}})",
        "FAR <= TRACK <= NEAR");
    reject(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","models":{"detect_interval":2}})",
        "detect_interval is an unknown key");
    reject(R"({"schema_version":1,"behavior_policy":"legacy_field_v1"} trailing)", "parse error");
    reject(R"({"schema_version":1,"behavior_policy":"legacy_field_v1", // comment
        "runtime":{}})",
           "parse error");
    expectFailure({"--config", config.string(), "--config", config.string()}, "only once");
    const auto invalid_override = directory.write(
        R"({"schema_version":1,"behavior_policy":"legacy_field_v1","hardware":{"fps":0}})");
    expectFailure({"--config", invalid_override.string(), "--fps", "60"},
                  "hardware.fps must be in");
    expectFailure({"--dump-config"}, "Missing value");
    expectFailure({"--config", (directory.path / "missing.json").string()},
                  "Cannot open runtime configuration");
    {
        const auto deleted = directory.path / "deleted-working-directory";
        fs::create_directory(deleted);
        const int original = open(".", O_RDONLY | O_DIRECTORY);
        assert(original >= 0 && chdir(deleted.c_str()) == 0);
        assert(fs::remove(deleted));
        assert(load({}).config.team == "blue");
        const auto replay = (directory.path / "absolute-replay.json").string();
        assert(load({"--push-replay", replay}).config.push_replay == replay);
        assert(load({"--model", "relative-cli.rknn"}).config.model_path == "relative-cli.rknn");
        assert(load({"--config", config.string()}).config.fps == 30);
        assert(load({"--dump-config", "-"}).options.dump_config_file == "-");
        const auto absolute_dump = (directory.path / "absolute-effective.json").string();
        assert(load({"--dump-config", absolute_dump}).options.dump_config_file == absolute_dump);
        expectFailure({"--config", "relative.json"}, "without an accessible working directory");
        expectFailure({"--dump-config", "relative.json"},
                      "without an accessible working directory");
        assert(fchdir(original) == 0);
        close(original);
    }
    std::cout << "Runtime configuration precedence, provenance, paths, booleans, schema and policy "
                 "validation passed\n";
}
