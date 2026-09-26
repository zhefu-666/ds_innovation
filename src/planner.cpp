#include "rescue/planner.hpp"

#include <algorithm>
#include <cmath>

namespace rescue {

LocalPlanner::LocalPlanner(PlannerConfig config) : config_(config) {}

cv::Point2f LocalPlanner::approachPoint(const cv::Point2f &target, const cv::Point2f &goal) const {
    const cv::Point2f delta = goal - target;
    const float distance = std::sqrt(delta.dot(delta));
    if (distance < 1e-4f) return target;
    return target - delta * (config_.approach_distance_m / distance);
}

bool LocalPlanner::segmentSafe(const cv::Point2f &a, const cv::Point2f &b,
                               const std::vector<PlannerObstacle> &obstacles) const {
    const cv::Point2f segment = b - a;
    const float length_sq = segment.dot(segment);
    for (const auto &obstacle : obstacles) {
        const float radius = obstacle.radius_m + config_.robot_radius_m +
                             (obstacle.dangerous ? config_.danger_inflation_m : 0.0f);
        const float t = length_sq > 1e-6f ?
            std::clamp((obstacle.center_m - a).dot(segment) / length_sq, 0.0f, 1.0f) : 0.0f;
        const cv::Point2f closest = a + segment * t;
        if (cv::norm(closest - obstacle.center_m) <= radius) return false;
    }
    return true;
}

std::vector<PlannedRoute> LocalPlanner::plan(const cv::Point2f &robot, const cv::Point2f &target,
                                             const cv::Point2f &goal,
                                             const std::vector<PlannerObstacle> &obstacles) const {
    std::vector<PlannedRoute> routes;
    const cv::Point2f approach = approachPoint(target, goal);
    const std::array<std::pair<std::string, std::vector<cv::Point2f>>, 3> candidates{{
        {"direct", {robot, approach}},
        {"left_detour", {robot, robot + cv::Point2f(0.0f, 1.0f), approach}},
        {"right_detour", {robot, robot + cv::Point2f(0.0f, -1.0f), approach}},
    }};
    for (const auto &candidate : candidates) {
        bool safe = true;
        float cost = 0.0f;
        for (size_t i = 1; i < candidate.second.size(); ++i) {
            if (!segmentSafe(candidate.second[i - 1], candidate.second[i], obstacles)) {
                safe = false;
                break;
            }
            cost += cv::norm(candidate.second[i] - candidate.second[i - 1]);
        }
        if (safe) routes.push_back(PlannedRoute{true, candidate.first, candidate.second, cost});
    }
    std::sort(routes.begin(), routes.end(), [](const PlannedRoute &a, const PlannedRoute &b) {
        return a.cost < b.cost;
    });
    return routes;
}

} // namespace rescue
