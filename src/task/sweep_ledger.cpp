#include "rescue/task/sweep_ledger.hpp"
#include "rescue/task/push_task.hpp"
#include <algorithm>

namespace rescue {
void SweepLedger::begin(const TaskTuning& t, uint64_t now) {
    settle(t, now);
    if (!chain_start_us_) {
        chain_start_us_ = now;
        accumulated_us_ = 0;
        swung_ = false;
    }
    if (!active_)
        segment_start_us_ = now;
    active_ = true;
}
void SweepLedger::pause(uint64_t now) {
    if (!chain_start_us_ || !active_)
        return;
    accumulated_us_ += now > segment_start_us_ ? now - segment_start_us_ : 0;
    active_ = false;
    paused_us_ = now;
}
bool SweepLedger::close(const TaskTuning& t, uint64_t now) {
    if (!chain_start_us_)
        return false;
    pause(now);
    const bool swung = swung_;
    trip_spent_us_ += accumulated_us_;
    match_spent_us_ += accumulated_us_;
    if (swung)
        ++done_;
    chain_start_us_ = accumulated_us_ = 0;
    active_ = swung_ = false;
    if (t.sweep_observation_guard)
        observe_until_us_ = now + t.sweep_reobserve_us;
    return swung;
}
void SweepLedger::settle(const TaskTuning& t, uint64_t now) {
    if (chain_start_us_ && !active_ && now > paused_us_ + t.sweep_chain_link_us)
        close(t, now);
}
void SweepLedger::newTrip(const TaskTuning& t, uint64_t now) {
    close(t, now);
    done_ = 0;
    trip_spent_us_ = 0;
    gap_until_us_ = 0;
}
void SweepLedger::reset() {
    done_ = 0;
    trip_spent_us_ = match_spent_us_ = gap_until_us_ = observe_until_us_ = 0;
    chain_start_us_ = accumulated_us_ = 0;
    active_ = swung_ = false;
}
uint64_t SweepLedger::activeUs(uint64_t now) const {
    if (!chain_start_us_)
        return 0;
    return accumulated_us_ + (active_ && now > segment_start_us_ ? now - segment_start_us_ : 0);
}
uint64_t SweepLedger::spentUs(uint64_t now) const {
    return trip_spent_us_ + activeUs(now);
}
uint64_t SweepLedger::matchSpentUs(uint64_t now) const {
    return match_spent_us_ + activeUs(now);
}
SweepBudgetState SweepLedger::budget(const TaskTuning& t, uint64_t now) const {
    SweepBudgetState result;
    result.time_ok =
        spentUs(now) < t.sweep_time_budget_us && matchSpentUs(now) < t.sweep_match_budget_us;
    result.budget_ok = done_ < t.sweep_total_max && result.time_ok;
    result.ready = now >= gap_until_us_ && result.budget_ok;
    result.operation_timeout = now - chain_start_us_ > t.sweep_op_budget_us;
    result.over = active_ && (result.operation_timeout || !result.time_ok);
    result.escalation = std::min(done_, t.sweep_escalate_max);
    return result;
}
void SweepLedger::extendGapUntil(uint64_t until) {
    gap_until_us_ = std::max(gap_until_us_, until);
}
void SweepLedger::extendObservationWait(const TaskTuning& t, uint64_t now) {
    if (t.sweep_observation_guard && observe_until_us_)
        observe_until_us_ = now + t.sweep_reobserve_us;
}
bool SweepLedger::waitForObservation(const TaskTuning& t, uint64_t now) {
    if (!t.sweep_observation_guard || !observe_until_us_)
        return false;
    if (now < observe_until_us_)
        return true;
    observe_until_us_ = 0;
    return false;
}
} // namespace rescue
