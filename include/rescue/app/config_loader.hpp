#pragma once

#include "rescue/config.hpp"

#include <string>
#include <utility>
#include <vector>

namespace rescue::app {

struct AppConfigOptions {
    std::string config_file;
    std::string dump_config_file;
    std::string behavior_policy;
    // Config member names, with the file location that supplied each request.
    std::vector<std::pair<std::string, std::string>> file_field_sources;
    std::vector<std::string> explicit_cli_flags;
    std::vector<std::string> explicit_cli_fields;
    std::vector<std::pair<std::string, std::string>> normalized_field_sources;
};

struct LoadedAppConfig {
    Config config;
    AppConfigOptions options;
};

// Loads one schema-v1 JSON file before explicit CLI arguments. The legacy
// resolver still determines final tuning and records its later overrides.
LoadedAppConfig loadAppConfig(int argc, char **argv);

} // namespace rescue::app
