#pragma once

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

// Shared search bookkeeping still owned by PushTask: the sweep states read some of it, so the
// stage borrows it until those states move too. A null pointer is a wiring mistake.
struct SearchContext {
    uint64_t now_us = 0, elapsed_us = 0;
    bool scan_ok = false, forward_ok = false, scatter_ok = false, clutter_fresh = false;
    bool scatter_multi_ok = false, approach_ok = false;
    bool mid_pitch = false, trip_empty = false, sweep_open = false, first_ordinary = false;
    int frame_raised = 0;
    float far_y = 0;
    const Inventory* pending = nullptr;

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
};

class SearchStage {
  public:
    // enter(): a state change outside SCAN drops the partial sighting history.
    void resetScanTracking();
    // enter(): every transition drops the accumulated scan heading.
    void resetScanHeading();
    // Match reset drops the pending sweep-confirm timer only.
    void resetConfirmSince();
    // Called for SCAN, LOST_SEARCH, MID/NEAR_REACQUIRE and SELECT_CARGO.
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
