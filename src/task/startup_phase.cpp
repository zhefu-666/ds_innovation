#include "rescue/task/push_task.hpp"

namespace rescue {
void PushTask::updateStartupPhase(TaskTick&) {
    // Reserved for the future launch maneuver. Skip without changing chassis or frame commands.
    requestPhase(PhaseOutcome::SearchReady, "startup_deferred");
}
} // namespace rescue
