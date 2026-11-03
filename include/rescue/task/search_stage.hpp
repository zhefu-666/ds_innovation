#pragma once
#include "rescue/task/task_phase.hpp"

#include "rescue/task/transport_rules.hpp"
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include <opencv2/core/types.hpp>

namespace rescue {
struct SearchPhaseData;
struct ActionProgress;
struct TripContext;
struct MotionCommand;
struct TaskTuning;
struct PushObservation;

// References are scoped to the owning search phase and the current tick.
struct SearchContext {
    SearchContext(SearchPhaseData& search,
                  ActionProgress& progress,
                  TripContext& trip,
                  MotionCommand& motion,
                  std::string& reason)
        : search(search), progress(progress), trip_data(trip), motion(motion),
          reason(reason) {
    }
    SearchPhaseData& search;
    ActionProgress& progress;
    TripContext& trip_data;
    MotionCommand& motion;
    std::string& reason;
    uint64_t now_us = 0, elapsed_us = 0;
    bool scan_ok = false, forward_ok = false;
    bool mid_pitch = false, trip_empty = false, sweep_open = false, first_ordinary = false;
    int frame_raised = 0;
    int16_t pitch_cmd = 0;
};

struct SearchServices {
    std::function<bool(const PushObservation&, uint64_t)> candidate;
    std::function<void(float, float)> drive;
    std::function<void()> scan_spin;
    std::function<bool()> unstick;
    std::function<void(SearchAction, uint64_t, const char*)> enter;
    std::function<void(const char*)> fail_stop;
    std::function<void(const char*)> scan_after_clear;
    std::function<void(uint8_t, uint64_t)> command_frame;
    std::function<void()> search_lost;
    std::function<int(const PushObservation&, uint8_t, uint64_t)> frame_wait;
    std::function<void()> clear_trip;
    std::function<void(uint64_t)> blacklist;
    std::function<bool()> lost_target;
    std::function<bool(const PushObservation&)> tracking;
    std::function<const char*()> tracking_reason;
    std::function<float(const PushObservation&)> approach_heading_shift;
    std::function<int16_t()> desired_pitch;
    std::function<bool()> mid_pitch;
    std::function<void(float, float)> steer;
};

class SearchStage {
  public:
    // enter(): a state change outside SCAN drops the partial sighting history.
    void resetScanTracking();
    // enter(): every transition drops the accumulated scan heading.
    void resetScanHeading();
    // Called for the search states and both approach states.
    void update(SearchAction state,
                const TaskTuning&,
                const PushObservation&,
                const SearchContext&,
                const SearchServices&);

  private:
    void
    scan(const TaskTuning&, const PushObservation&, const SearchContext&, const SearchServices&);
    void lostSearch(const TaskTuning&,
                    const PushObservation&,
                    const SearchContext&,
                    const SearchServices&);
    void reacquire(const TaskTuning&,
                   const PushObservation&,
                   const SearchContext&,
                   const SearchServices&,
                   bool mid);
    void selectCargo(const TaskTuning&,
                     const PushObservation&,
                     const SearchContext&,
                     const SearchServices&);
    void midApproach(const TaskTuning&,
                     const PushObservation&,
                     const SearchContext&,
                     const SearchServices&);
    void approach(const TaskTuning&,
                  const PushObservation&,
                  const SearchContext&,
                  const SearchServices&);

    bool scan_last_valid_ = false;
    float scan_last_heading_ = 0;
    int vis_hits_ = 0, vis_miss_ = 0;
    float vis_bearing_ = 0;
    uint64_t vis_last_seen_us_ = 0, vis_chase_t0_ = 0, vis_cool_until_us_ = 0;
    uint64_t scan_hold_t0_ = 0, scan_hold_last_sight_us_ = 0, scan_hold_cool_until_us_ = 0;
    float rush_pulse_yaw0_ = 0, rush_pulse_delta_ = 0;
    uint64_t rush_pulse_start_us_ = 0, rush_pulse_t0_us_ = 0;
    float prep_ref_m_ = 0;
    uint64_t prep_last_us_ = 0, prep_ref_us_ = 0;
    uint64_t rebind_since_us_ = 0, approach_last_us_ = 0;
    RuleVerdict seen_verdict_ = RuleVerdict::OK;
};
} // namespace rescue
