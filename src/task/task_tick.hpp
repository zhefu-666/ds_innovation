#pragma once
#include "rescue/task/push_task.hpp"
#include "rescue/task/disperse_cluster_stage.hpp"
namespace rescue {
struct TaskTick {
    explicit TaskTick(const PushObservation& observation)
        : in(observation), now(observation.now_us) {
    }
    const PushObservation& in;
    uint64_t now, elapsed = 0;
    MotionCommand motion;
    bool forward_ok = false, scan_ok = false, observed_zone = false, scan_spin = false;
    float target_forward_m = 0, far_y = 0;
    decltype(SweepLedger{}.budget(TaskTuning{}, uint64_t{})) sweepBudget;
    bool sweepTimeOk = false, sweepBudgetOk = false, sweepLedgerOk = false, sweepOver = false;
    int sweep_esc = 0;
    bool scatter_ok = false, scatter_multi_ok = false, approach_ok = false, clutter_fresh = false;
    DisperseClusterPlanner cluster;
};
} // namespace rescue
