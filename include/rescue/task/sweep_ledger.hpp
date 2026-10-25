#pragma once

#include <cstdint>

namespace rescue {
struct TaskTuning;

struct SweepBudgetState {
    bool time_ok = false, budget_ok = false, ready = false;
    bool over = false, operation_timeout = false;
    int escalation = 0;
};

class SweepLedger {
  public:
    void begin(const TaskTuning&, uint64_t now);
    void pause(uint64_t now);
    bool close(const TaskTuning&, uint64_t now);
    void settle(const TaskTuning&, uint64_t now);
    void setSwung(bool swung) {
        swung_ = swung;
    }
    // A new trip settles the chain and clears only trip usage, count and gap.
    void newTrip(const TaskTuning&, uint64_t now);
    void reset();

    int done() const {
        return done_;
    }
    bool chainOpen() const {
        return chain_start_us_ != 0;
    }
    bool active() const {
        return active_;
    }
    uint64_t spentUs(uint64_t now) const;
    uint64_t matchSpentUs(uint64_t now) const;
    SweepBudgetState budget(const TaskTuning&, uint64_t now) const;

    void setGapUntil(uint64_t until) {
        gap_until_us_ = until;
    }
    void extendGapUntil(uint64_t until);
    void extendObservationWait(const TaskTuning&, uint64_t now);
    // Called only for the parent's search/approach states after frame feedback.
    bool waitForObservation(const TaskTuning&, uint64_t now);

  private:
    uint64_t activeUs(uint64_t now) const;
    int done_ = 0;
    uint64_t trip_spent_us_ = 0, match_spent_us_ = 0;
    uint64_t chain_start_us_ = 0, segment_start_us_ = 0, accumulated_us_ = 0, paused_us_ = 0;
    bool active_ = false, swung_ = false;
    uint64_t gap_until_us_ = 0, observe_until_us_ = 0;
};
} // namespace rescue
