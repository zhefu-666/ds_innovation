#pragma once
#include "rescue/task/task_phase.hpp"

namespace rescue {
struct ResolvedRuntimeConfig;
namespace app {

// Search and Transport apply the configured yaw scale once after the sender ramp.
// Only Startup and Capture retain their tuning from before compensation.
inline bool preserveAngularTuning(TaskPhase phase) {
    return phase == TaskPhase::Startup || phase == TaskPhase::Capture;
}

class Runtime {
  public:
    explicit Runtime(const ResolvedRuntimeConfig& config) : config_(config) {
    }
    int run();

  private:
    const ResolvedRuntimeConfig& config_;
};

int runApplication(int argc, char** argv);
} // namespace app
} // namespace rescue
