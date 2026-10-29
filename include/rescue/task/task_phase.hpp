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
// Public search status. Detailed motion phases remain internal implementation steps.
enum class SearchStatusAction {
    Scan,
    LostSearch,
    Reacquire,
    Approach,
    Prepare,
    ClearPush,
    TurnAway,
    Scatter,
    Pile,
    Recover
};

inline SearchStatusAction searchStatusAction(SearchAction action) {
    switch (action) {
        case SearchAction::MidReacquire:
        case SearchAction::NearReacquire:
            return SearchStatusAction::Reacquire;
        case SearchAction::MidApproach:
        case SearchAction::SelectCargo:
        case SearchAction::CueApproach:
        case SearchAction::Approach:
            return SearchStatusAction::Approach;
        case SearchAction::CuePrepare:
        case SearchAction::ClearPile:
        case SearchAction::ClearPause:
            return SearchStatusAction::ClearPush;
        case SearchAction::Scan:
            return SearchStatusAction::Scan;
        case SearchAction::LostSearch:
            return SearchStatusAction::LostSearch;
        case SearchAction::Prepare:
            return SearchStatusAction::Prepare;
        case SearchAction::TurnAway:
            return SearchStatusAction::TurnAway;
        case SearchAction::Scatter:
            return SearchStatusAction::Scatter;
        case SearchAction::Pile:
            return SearchStatusAction::Pile;
        case SearchAction::Recover:
            return SearchStatusAction::Recover;
    }
    return SearchStatusAction::Scan;
}
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
