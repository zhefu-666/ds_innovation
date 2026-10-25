#pragma once

#include "rescue/task/disperse_swing_stage.hpp"
#include "rescue/task/disperse_turn_stage.hpp"
#include "rescue/task/sweep_ledger.hpp"
#include "rescue/transport_rules.hpp"
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include <opencv2/core/types.hpp>

namespace rescue {
struct TaskTuning;
struct PushObservation;
enum class PushState;

struct BestGroup {
    int n = 0;
    float d = 0;
};

// Shared search/approach bookkeeping still owned by PushTask while the carrying states read it.
// The stage borrows it through these pointers; a null pointer is a wiring mistake.
struct SearchContext {
    uint64_t now_us = 0, elapsed_us = 0;
    bool scan_ok = false, forward_ok = false, scatter_ok = false, clutter_fresh = false;
    bool scatter_multi_ok = false, approach_ok = false;
    bool mid_pitch = false, trip_empty = false, sweep_open = false, first_ordinary = false;
    bool sweep_over = false, sweep_time_ok = false, sweep_ledger_ok = false;
    bool sweep_open_disabled = false;
    int frame_raised = 0, sweep_esc = 0;
    int16_t pitch_cmd = 0;
    float far_y = 0, target_forward_m = 0;
    Inventory* pending = nullptr;
    const Inventory* trip = nullptr;

    int* misses = nullptr;
    bool* track_close = nullptr;
    float* travel = nullptr;
    int* target_id = nullptr;
    std::string* label = nullptr;
    int* confirmations = nullptr;
    bool* cue_mode = nullptr;
    int* search_cue_id = nullptr;
    int* capture_target_id = nullptr;
    int* step = nullptr;
    uint64_t* phase_us = nullptr;
    float* turned = nullptr;
    bool* heading_seen = nullptr;
    float* last_heading = nullptr;
    int* lost_round = nullptr;
    int* scatter_seen = nullptr;
    int* near_failures = nullptr;
    float* scan_turn_acc = nullptr;
    int* scatter_count = nullptr;
    int* scatter_approach_n = nullptr;
    int* clear_attempts = nullptr;
    uint64_t* scatter_idle_since_us = nullptr;
    bool* burst_drive_used = nullptr;
    bool* burst_swing_armed = nullptr;
    float* approach_forward = nullptr;
    std::string* reason = nullptr;

    float* motion_vx = nullptr;
    float* motion_wz = nullptr;
    int* sweep_phase = nullptr;
    int* sweep_attempts = nullptr;
    int* sweep_seen = nullptr;
    int* pile_runs = nullptr;
    int* pile_seen = nullptr;
    int* mid_clear_n = nullptr;
    int* approach_last_id = nullptr;
    int* rush_pulse_phase = nullptr;
    int* rush_pulse_count = nullptr;
    int* precheck_seen = nullptr;
    int* precheck_rejects = nullptr;
    float* sweep_creep_m = nullptr;
    float* sweep_travel0 = nullptr;
    float* sweep_yaw0 = nullptr;
    float* sweep_sign = nullptr;
    float* last_range = nullptr;
    float* approach_last_heading = nullptr;
    float* approach_last_distance = nullptr;
    float* approach_last_world_yaw = nullptr;
    float* rush_pulse_yaw0 = nullptr;
    float* rush_pulse_delta = nullptr;
    float* prep_ref_m = nullptr;
    float* disperse_sign = nullptr;
    float* disperse_rush_m = nullptr;
    float* scan_mid_limit = nullptr;
    uint64_t* sweep_start_us = nullptr;
    uint64_t* sweep_t0_us = nullptr;
    uint64_t* rebind_since_us = nullptr;
    uint64_t* approach_last_us = nullptr;
    uint64_t* rush_pulse_start_us = nullptr;
    uint64_t* rush_pulse_t0_us = nullptr;
    uint64_t* pile_cool_until_us = nullptr;
    uint64_t* prep_last_us = nullptr;
    uint64_t* prep_ref_us = nullptr;
    uint64_t* disperse_cool_until_us = nullptr;
    bool* sweep_non_supply_active = nullptr;
    bool* post_disperse_mid = nullptr;
    bool* approach_rebind = nullptr;
    bool* approach_last_world_valid = nullptr;
    bool* disperse_small = nullptr;
    int16_t* approach_pitch = nullptr;
    RuleVerdict* verdict = nullptr;
    RuleVerdict* seen_verdict = nullptr;
    Inventory* seen = nullptr;

    SweepLedger* sweep_ledger = nullptr;
    DisperseTurnStage* disperse_turn = nullptr;
    DisperseSwingStage* disperse_swing = nullptr;
};

struct SearchServices {
    std::function<bool(const PushObservation&, uint64_t)> candidate;
    std::function<bool(const PushObservation&)> weak_candidate;
    std::function<bool()> cue_candidate;
    std::function<cv::Point2f()> target_body;
    std::function<BestGroup(const std::vector<cv::Point2f>&, float, float)> find_best;
    std::function<int(const std::vector<cv::Point2f>&, int, int, float)> eval_group;
    std::function<void(const cv::Point2f&)> add_ordinary_extra;
    std::function<void(bool, bool, const char*)> begin_scatter;
    std::function<void(float, float)> drive;
    std::function<void()> scan_spin;
    std::function<bool()> unstick;
    std::function<void(PushState, uint64_t, const char*)> enter;
    std::function<void(const char*)> fail_stop;
    std::function<void(const char*)> scan_after_clear;
    std::function<void(uint8_t, uint64_t)> command_frame;
    std::function<void(uint64_t)> reject_search_cue;
    std::function<void()> search_lost;
    std::function<int(const PushObservation&, uint8_t, uint64_t)> frame_wait;
    std::function<void()> clear_trip;
    std::function<void(uint64_t)> blacklist;
    std::function<void()> clear_rejected;
    std::function<void()> clear_rejected_bearings;
    std::function<bool()> sweep_frame_ready;
    std::function<bool()> disperse_step;
    std::function<bool(const char*)> begin_pile_from_sweep;
    std::function<void(const char*)> begin_pile;
    std::function<bool()> lost_target;
    std::function<bool(const PushObservation&)> tracking;
    std::function<const char*()> tracking_reason;
    std::function<float(const PushObservation&)> approach_heading_shift;
    std::function<bool(const PushObservation&, uint64_t, float&, float*)> upright_in_corridor;
    std::function<bool(const PushObservation&, uint64_t, float&, float*)> neighbour_in_corridor;
    std::function<int16_t()> desired_pitch;
    std::function<bool()> mid_pitch;
    std::function<void(float, float)> steer;
    std::function<float()> motion_ratio;
};

class SearchStage {
  public:
    // enter(): a state change outside SCAN drops the partial sighting history.
    void resetScanTracking();
    // enter(): every transition drops the accumulated scan heading.
    void resetScanHeading();
    // Match reset drops the pending sweep-confirm timer only.
    void resetConfirmSince();
    // Called for the search states and both approach states.
    void update(PushState state,
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
    int dense_seen_ = 0, far_seen_ = 0;
    uint64_t confirm_since_us_ = 0;
    int vis_hits_ = 0, vis_miss_ = 0;
    float vis_bearing_ = 0;
    uint64_t vis_last_seen_us_ = 0, vis_chase_t0_ = 0, vis_cool_until_us_ = 0;
    uint64_t scan_hold_t0_ = 0, scan_hold_last_sight_us_ = 0, scan_hold_cool_until_us_ = 0;
};
} // namespace rescue
