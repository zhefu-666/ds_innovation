#pragma once
namespace rescue {
// Business phases; Startup intentionally performs no motion until implemented.
enum class TaskPhase {
    Startup,
    Search,
    Capture,
    Transport
};
enum class RunStatus {
    Waiting,
    Running,
    Paused,
    Fault
};
enum class SearchAction {
    Scan,
    LostSearch,
    MidReacquire,
    MidApproach,
    NearReacquire,
    SelectCargo,
    CueApproach,
    CuePrepare,
    ClearPile,
    ClearPause,
    Approach,
    Prepare,
    TurnAway,
    Scatter,
    Pile,
    Recover
};
enum class CaptureAction {
    Advance,
    LowerFrame,
    Verify,
    Recover
};
enum class TransportAction {
    Carry,
    Gate,
    Enter,
    Release,
    BackOut,
    Verify,
    Recover
};
enum class PhaseOutcome {
    Running,
    SearchReady,
    CaptureReady,
    TransportReady
};
enum class CaptureEvidence {
    None,
    Verified,
    Assumed
};
inline const char* phaseName(TaskPhase phase) {
    switch (phase) {
        case TaskPhase::Startup:
            return "STARTUP";
        case TaskPhase::Search:
            return "SEARCH";
        case TaskPhase::Capture:
            return "CAPTURE";
        case TaskPhase::Transport:
            return "TRANSPORT";
    }
    return "UNKNOWN";
}
inline const char* runStatusName(RunStatus status) {
    switch (status) {
        case RunStatus::Waiting:
            return "WAITING";
        case RunStatus::Running:
            return "RUNNING";
        case RunStatus::Paused:
            return "PAUSED";
        case RunStatus::Fault:
            return "FAULT";
    }
    return "UNKNOWN";
}
} // namespace rescue
