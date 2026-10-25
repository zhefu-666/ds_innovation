#pragma once

#include <cstdint>

namespace rescue {
struct TaskTuning;

struct DisperseSwingContext {
    uint64_t now_us = 0, sweep_start_us = 0;
    float travel_m = 0, sweep_travel_m = 0, sweep_yaw = 0, heading_rad = 0;
    float motion_ratio = 1, direction = 1;
    bool small = false, swing_plan_active = false;
    float cap_positive = 99, cap_negative = 99;
    bool burst = false, approach_fast = false;
};

enum class DisperseSwingOutcome { Continue, NextPass, Complete };

struct DisperseSwingResult {
    DisperseSwingOutcome outcome = DisperseSwingOutcome::Continue;
    bool motion_requested = false;
    float vx = 0, direction = 0;
    bool fast = false;
    const char* reason = nullptr;
    bool restart_clock = false;
};

class DisperseSwingStage {
public:
    void configure(int passes, int swings) { pass_ = 0; pass_limit_ = passes; swing_limit_ = swings; }
    void beginSwing() { swing_count_ = 0; }
    void restartPass() { pass_ = 0; }
    int swingCount() const { return swing_count_; }
    int pass() const { return pass_; }
    int passLimit() const { return pass_limit_; }
    int swingLimit() const { return swing_limit_; }
    // The parent applies progress events and preserves every motion permission check.
    DisperseSwingResult update(const TaskTuning&, const DisperseSwingContext&);
private:
    int swing_count_ = 0, pass_ = 0, pass_limit_ = 1, swing_limit_ = 6;
};
} // namespace rescue
