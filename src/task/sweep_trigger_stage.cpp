#include "rescue/task/sweep_trigger_stage.hpp"
#include "rescue/push_task.hpp"
#include <algorithm>
#include <cmath>

namespace rescue {
SweepTriggerGate SweepTriggerPlanner::evaluate(const TaskTuning& t,
                                               const SweepTriggerContext& c,
                                               uint64_t clutter_ts_us) const {
    SweepTriggerGate gate;
    // Blind-scatter group search shared by SCAN, the far drive, and the lost-target paths.
    gate.scatter_ok = t.blind_scatter && t.disperse && c.scan_ok && c.forward_ok &&
                      c.heading_valid && std::isfinite(c.heading_rad) &&
                      c.scatter_count < t.blind_scatter_max &&
                      c.now_us >= c.scatter_cool_until_us && c.ledger_ready;
    gate.scatter_multi_ok = t.blind_scatter && t.disperse && c.scan_ok && c.forward_ok &&
                            c.heading_valid && std::isfinite(c.heading_rad) &&
                            c.scatter_count < t.blind_scatter_max &&
                            c.now_us >= c.scatter_cool_until_us &&
                            (t.dense_multi_ignore_gap ? c.budget_ok : c.ledger_ready);
    gate.approach_ok = c.scatter_approach_n < t.blind_scatter_approach_max;
    gate.far_y = gate.approach_ok ? std::max(t.blind_scatter_max_y_m, t.blind_scatter_far_max_y_m)
                                  : t.blind_scatter_max_y_m;
    gate.clutter_fresh =
        clutter_ts_us && c.now_us >= clutter_ts_us && c.now_us - clutter_ts_us <= t.sweep_fresh_us;
    return gate;
}
} // namespace rescue
