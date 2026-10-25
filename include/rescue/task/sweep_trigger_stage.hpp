#pragma once

#include <cstdint>

namespace rescue {
struct TaskTuning;
struct SweepBudget;

// Inputs the parent already computed for the sweep trigger gate.
struct SweepTriggerContext {
    uint64_t now_us = 0;
    bool scan_ok = false, forward_ok = false, heading_valid = false;
    float heading_rad = 0;
    int scatter_count = 0;
    int scatter_approach_n = 0;
    uint64_t scatter_cool_until_us = 0;
    bool ledger_ready = false, budget_ok = false;
};

// The derived permissions every blind-scatter trigger shares. Pure: no state, no writes.
struct SweepTriggerGate {
    bool scatter_ok = false;
    bool scatter_multi_ok = false;
    bool approach_ok = false;
    float far_y = 0;
    bool clutter_fresh = false;
};

class SweepTriggerPlanner {
  public:
    SweepTriggerGate
    evaluate(const TaskTuning&, const SweepTriggerContext&, uint64_t clutter_ts_us) const;
};
} // namespace rescue
