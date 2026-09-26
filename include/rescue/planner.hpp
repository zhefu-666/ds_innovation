#pragma once

#include "rescue/types.hpp"

#include <array>
#include <string>
#include <vector>

namespace rescue {

enum class RegionState { START, CENTER, OBSERVE, DROP, UNKNOWN };

struct PlannerObstacle {
    cv::Point2f center_m;
    float radius_m = 0.15f;
    bool dangerous = false;
};

struct PlannerConfig {
    float approach_distance_m = 0.35f;
    float robot_radius_m = 0.25f;
    float target_radius_m = 0.10f;
    float danger_inflation_m = 0.20f;
};

struct PlannedRoute {
    bool valid = false;
    std::string kind;
    std::vector<cv::Point2f> waypoints_m;
    float cost = 0.0f;
};

class LocalPlanner {
public:
    explicit LocalPlanner(PlannerConfig config = {});

    cv::Point2f approachPoint(const cv::Point2f &target, const cv::Point2f &goal) const;
    std::vector<PlannedRoute> plan(const cv::Point2f &robot, const cv::Point2f &target,
                                   const cv::Point2f &goal,
                                   const std::vector<PlannerObstacle> &obstacles) const;
    bool segmentSafe(const cv::Point2f &a, const cv::Point2f &b,
                     const std::vector<PlannerObstacle> &obstacles) const;

private:
    PlannerConfig config_;
};

} // namespace rescue
