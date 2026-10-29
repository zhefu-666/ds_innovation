#pragma once
#include "rescue/task/task_phase.hpp"

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
struct SearchPhaseData;
struct ActionProgress;
struct TripContext;
struct MotionCommand;
struct TaskTuning;
struct PushObservation;

struct SweepBest {
    int n = 0;
    float d = 0;
    cv::Point2f centre{0.f, 0.f};
};

// References are scoped to the owning search phase and the current tick.
struct SweepContext {
    SweepContext(SearchPhaseData& search,
                 ActionProgress& progress,
                 TripContext& trip,
                 MotionCommand& motion,
                 SweepLedger& ledger,
                 std::string& reason)
        : search(search), progress(progress), trip_data(trip), motion(motion), ledger(ledger),
          reason(reason) {
    }
    SearchPhaseData& search;
    ActionProgress& progress;
    TripContext& trip_data;
    MotionCommand& motion;
    SweepLedger& ledger;
    std::string& reason;
    uint64_t now_us = 0, elapsed_us = 0;
    bool clutter_fresh = false, scatter_ok = false, first_ordinary = false;
    bool sweep_over = false, sweep_operation_timeout = false;
    float target_forward_m = 0;
    int16_t pitch_cmd = 0;
};

struct SweepServices {
    std::function<void(uint8_t, uint64_t)> command_frame;
    std::function<void(float, float)> drive;
    std::function<void(SearchAction, uint64_t, const char*)> enter;
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
    void resetScatterApproach() {
        scatter_alignment_ = {};
    }
    // SCATTER, PILE, CUE_APPROACH, CUE_PREPARE, CLEAR_PILE and CLEAR_PAUSE.
    void update(SearchAction state,
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

    struct ScatterAlignment {
        bool ready = false, braking = true;
        uint64_t started_us = 0, stable_since_us = 0, visual_us = 0, heading_us = 0;
        float heading = 0;
        int stable_frames = 0;
    } scatter_alignment_;
    float clear_heading_rad_ = 0;
    float pile_chain_in_m_ = 0;
    int pile_chain_look_ = 0;
    float pile_chain_yaw_mid_ = 0, pile_chain_yaw_pre_ = 0;
    float pile_travel0_ = 0;
    float scatter_back_ = 0;
};
} // namespace rescue
