#pragma once

#include <cstdint>
#include <string>

namespace rescue {
struct TaskTuning;

struct ReleaseRetreatContext {
    uint64_t elapsed_us;
    float travel_m;
    bool observed_zone;
    bool mouth_clear;
    bool anchored_zone;
    float hold_y;
    float back_target_m;
    float motion_ratio;
    bool assume_blind;
    int repush_count;
    float half_x;
};

struct ReleasedCargoEvidence {
    bool visible = false, fixed = false, clipped = false;
    float rear_x = 0, rear_y = 0;
    uint64_t timestamp_us = 0;
};

enum class ReleaseRetreatOutcome { Continue, PushIn, VerifyDelivery, InsideAfterRelease, Fault };

struct ReleaseRetreatResult {
    ReleaseRetreatOutcome outcome = ReleaseRetreatOutcome::Continue;
    std::string reason;
    bool reverse = false;
    bool blind_back_out = false;
};

class ReleaseRetreatStage {
public:
    // Fresh release clears the confirmation count but retains the last cargo timestamp.
    void beginReleaseBackOff(float hold_y);
    void beginRepushBackOff(float hold_y);
    void clearTrip();
    bool needsCargoTracking() const { return prepush_; }
    // Called only after the parent has confirmed the camera and open frame are ready.
    ReleaseRetreatResult update(const TaskTuning&, const ReleaseRetreatContext&, const ReleasedCargoEvidence&);
private:
    bool prepush_ = false;
    float start_hold_y_ = 0;
    int inside_count_ = 0;
    uint64_t inside_timestamp_us_ = 0;
};
} // namespace rescue
