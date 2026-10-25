#pragma once

#include <opencv2/core/types.hpp>
#include <vector>

namespace rescue {
struct TaskTuning;
struct PushObservation;

struct ClusterGroup {
    int n = 0;
    float d = 0.f;
};

// Dense-group planner shared by every blind-scatter trigger. All of its state is per-tick: the
// parent constructs one per update() call, so nothing here outlives a single decision.
class DisperseClusterPlanner {
  public:
    // Points the parent's lock / weak detections append: they count as ordinary supplies.
    void addOrdinaryExtra(const cv::Point2f&);
    // Dense, near-ahead candidate within max_y; the winning group is retained for scatter().
    ClusterGroup findBest(const TaskTuning&,
                          const PushObservation&,
                          const std::vector<cv::Point2f>& points,
                          float max_y,
                          float radius);
    // 0: none, 1: group inside the short window (sweep now), 2: group farther out.
    int evalGroup(const TaskTuning&,
                  const PushObservation&,
                  const std::vector<cv::Point2f>& points,
                  int need_n,
                  int far_need,
                  float far_y,
                  int sweep_esc,
                  float short_radius = -1.f);
    cv::Point2f bestCentre() const {
        return best_c_;
    }
    float bestDistance() const {
        return best_d_;
    }

  private:
    bool isOrdinary(const PushObservation&, const cv::Point2f&) const;

    std::vector<cv::Point2f> ord_extra_;
    int best_n_ = 0;
    cv::Point2f best_c_{0.f, 0.f};
    float best_d_ = 0.f;
};
} // namespace rescue
