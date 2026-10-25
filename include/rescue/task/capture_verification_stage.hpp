#pragma once

#include "rescue/transport_rules.hpp"
#include <cstdint>
#include <optional>
#include <string>

namespace rescue {
struct PushObservation;
struct TaskTuning;

struct CaptureVerificationContext {
    uint64_t elapsed_us;
    const std::string& label;
    const Inventory& pending;
    bool first_ordinary_delivered;
};

enum class CaptureVerificationOutcome { Continue, Verified, Assumed, CaptureFail, AbortDrop, SafeStop };

struct CaptureVerificationResult {
    CaptureVerificationOutcome outcome = CaptureVerificationOutcome::Continue;
    Inventory inventory;
    // Missing updates preserve the previous task diagnostic, including on Continue.
    std::optional<RuleVerdict> verdict;
    std::string reason;
    bool blacklist_target = false;
};

class CaptureVerificationStage {
public:
    // Start a new verification attempt; cancellation and trip reset discard its evidence.
    void reset();
    CaptureVerificationResult update(const PushObservation&, const TaskTuning&,
                                     const CaptureVerificationContext&);
private:
    int confirmations_ = 0;
    Inventory seen_;
    int observed_frames_ = 0, label_frames_ = 0, bad_frames_ = 0;
    RuleVerdict bad_verdict_ = RuleVerdict::OK;
    bool recheck_active_ = false;
    int recheck_track_ = -1, recheck_frames_ = 0, recheck_strong_ = 0, recheck_clear_ = 0;
    uint64_t recheck_started_us_ = 0;
};
} // namespace rescue
