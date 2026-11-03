#pragma once

#include <cstdint>

namespace rescue {
struct TaskTuning;

// Shared reads from the parent: the last own-zone sighting and the run's start heading.
// Nothing here is written by the stage.
struct CarrySearchContext {
    uint64_t now_us = 0;
    bool heading_valid = false;
    float heading_rad = 0;
    bool zone_mem = false;
    uint64_t zone_mem_us = 0;
    float zone_mem_yaw = 0;
    bool run_yaw0_set = false;
    float run_yaw0 = 0;
};

enum class CarrySearchAction {
    // This tick has no motion of its own; the parent moves on.
    None,
    // Spin in place looking for our own zone.
    Scan,
    // Drive a leg toward the sighted, remembered or start-line zone.
    Steer
};

struct CarrySearchResult {
    CarrySearchAction action = CarrySearchAction::None;
    const char* reason = nullptr;
    float speed = 0, heading = 0;
};

// Carries a load while our own zone is not verified: spin for our zone and drive a leg
// once one is sighted, remembered, or the start line helps. Motion permissions stay parent-owned.
class CarrySearchStage {
  public:
    void reset();
    CarrySearchResult update(const TaskTuning&, const CarrySearchContext&);

  private:
    CarrySearchResult search(const TaskTuning&, const CarrySearchContext&);

    // Zone search chain.
    bool active_ = false;
    int phase_ = 0;
    uint64_t t0_ = 0, leg_t0_ = 0;
    float turned_ = 0, last_yaw_ = 0, goal_ = 0;
    bool last_set_ = false;
};
} // namespace rescue
