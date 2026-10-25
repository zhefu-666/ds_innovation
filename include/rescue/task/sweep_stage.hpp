#pragma once

#include "rescue/task/disperse_rush_stage.hpp"
#include "rescue/task/disperse_swing_stage.hpp"
#include "rescue/task/disperse_turn_stage.hpp"
#include "rescue/task/sweep_ledger.hpp"
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include <opencv2/core/types.hpp>

namespace rescue {
struct TaskTuning;
struct PushObservation;
enum class PushState;

struct SweepBest {
    int n = 0;
    float d = 0;
};

// Sweep bookkeeping the approach states still read while they live in PushTask. The stage
// borrows it through these pointers; a null pointer is a wiring mistake.
struct SweepContext {
    uint64_t now_us = 0, elapsed_us = 0;
    bool clutter_fresh = false, scatter_ok = false, first_ordinary = false;
    bool sweep_over = false, sweep_operation_timeout = false;
    float target_forward_m = 0;
    int16_t pitch_cmd = 0;

    float* motion_vx = nullptr;
    float* motion_wz = nullptr;

    float* approach_forward = nullptr;
    bool* burst_fast = nullptr;
    bool* burst_swing = nullptr;
    bool* burst_swing_armed = nullptr;
    int* capture_target_id = nullptr;
    int* clear_attempts = nullptr;
    uint64_t* clear_commanded_us = nullptr;
    int* confirmations = nullptr;
    bool* cue_mode = nullptr;
    uint64_t* disperse_cool_until_us = nullptr;
    float* disperse_rush_m = nullptr;
    bool* disperse_small = nullptr;
    std::string* label = nullptr;
    int* misses = nullptr;
    int* pile_chain_n = nullptr;
    uint64_t* pile_cool_until_us = nullptr;
    int* pile_phase = nullptr;
    uint64_t* pile_phase_us = nullptr;
    float* pile_push0 = nullptr;
    int* pile_runs = nullptr;
    int* pile_seen = nullptr;
    float* pile_yaw0 = nullptr;
    std::string* reason = nullptr;
    float* scan_mid_limit = nullptr;
    float* scan_turn_acc = nullptr;
    float* scatter_approach_m = nullptr;
    int* scatter_approach_n = nullptr;
    bool* scatter_commit = nullptr;
    uint64_t* scatter_cool_until_us = nullptr;
    bool* scatter_far = nullptr;
    uint64_t* scatter_idle_since_us = nullptr;
    float* scatter_range = nullptr;
    int* scatter_seen = nullptr;
    float* scatter_yaw = nullptr;
    int* search_cue_id = nullptr;
    int* sweep_phase = nullptr;
    int16_t* sweep_pitch = nullptr;
    uint64_t* sweep_start_us = nullptr;
    uint64_t* sweep_t0_us = nullptr;
    float* sweep_travel0 = nullptr;
    float* sweep_yaw0 = nullptr;
    int* target_id = nullptr;
    bool* track_close = nullptr;
    float* travel = nullptr;

    SweepLedger* sweep_ledger = nullptr;
    DisperseRushStage* disperse_rush = nullptr;
    DisperseTurnStage* disperse_turn = nullptr;
    DisperseSwingStage* disperse_swing = nullptr;
};

struct SweepServices {
    std::function<void(uint8_t, uint64_t)> command_frame;
    std::function<void(float, float)> drive;
    std::function<void(PushState, uint64_t, const char*)> enter;
    std::function<void(const char*)> fail_stop;
    std::function<void(const char*)> scan_after_clear;
    std::function<void(uint64_t)> blacklist;
    std::function<void()> clear_trip;
    std::function<bool(const PushObservation&, uint64_t)> candidate;
    std::function<bool(const PushObservation&)> weak_candidate;
    std::function<bool()> cue_candidate;
    std::function<cv::Point2f()> target_body;
    std::function<SweepBest(const std::vector<cv::Point2f>&, float, float)> find_best;
    std::function<int(const std::vector<cv::Point2f>&, int, int, float)> eval_group;
    std::function<void(const cv::Point2f&)> add_ordinary_extra;
    std::function<void()> clear_rejected;
    std::function<void()> clear_rejected_bearings;
    std::function<void(bool, bool, const char*)> begin_scatter;
    std::function<bool()> sweep_frame_ready;
    std::function<bool()> disperse_step;
    std::function<bool(const char*)> begin_pile_from_sweep;
    std::function<void()> search_lost;
    std::function<bool()> lost_target;
    std::function<void(float, float)> steer;
    std::function<int16_t()> desired_pitch;
    std::function<bool(const PushObservation&)> tracking;
    std::function<bool()> mid_pitch;
    std::function<float()> motion_ratio;
};

class SweepStage {
  public:
    // SCATTER, PILE, CUE_APPROACH, CUE_PREPARE, CLEAR_PILE and CLEAR_PAUSE.
    void update(PushState state,
                const TaskTuning&,
                const PushObservation&,
                const SweepContext&,
                const SweepServices&);

  private:
    void
    scatter(const TaskTuning&, const PushObservation&, const SweepContext&, const SweepServices&);
    void pile(const TaskTuning&, const PushObservation&, const SweepContext&, const SweepServices&);
    void cueApproach(const TaskTuning&,
                     const PushObservation&,
                     const SweepContext&,
                     const SweepServices&);
    void cuePrepare(const TaskTuning&,
                    const PushObservation&,
                    const SweepContext&,
                    const SweepServices&);
    void
    clearPile(const TaskTuning&, const PushObservation&, const SweepContext&, const SweepServices&);
    void clearPause(const TaskTuning&,
                    const PushObservation&,
                    const SweepContext&,
                    const SweepServices&);

    float clear_heading_rad_ = 0;
    float pile_chain_in_m_ = 0;
    int pile_chain_look_ = 0;
    float pile_chain_yaw_mid_ = 0, pile_chain_yaw_pre_ = 0;
    float pile_cut_m_ = 0, pile_cut_yaw_ = 0;
    float pile_travel0_ = 0;
    float scatter_back_ = 0;
};
} // namespace rescue
