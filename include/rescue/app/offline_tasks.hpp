#pragma once

#include <optional>

namespace rescue {
struct Config;
struct TaskTuning;

std::optional<int> runOfflineTask(const Config &config, const TaskTuning &tuning);
} // namespace rescue
