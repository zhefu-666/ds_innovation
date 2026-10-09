#pragma once

#include "rescue/types.hpp"
#include "rescue/zone_geometry.hpp"

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

// Explicit metric scene. "complete" must come from a validated free-space/obstacle
// producer, never from "the object detector returned no boxes". Convex known_region
// bounds all swept motion. Include opponent zones as conservative enclosing obstacles.
struct NavigationScene {
    uint64_t timestamp_us = 0;
    bool complete = false, opponent_region_known = false;
    std::vector<cv::Point2f> known_region;
    std::vector<PlannerObstacle> obstacles;
    float swept_radius_m = 0; // enclosing radius of body + current gripper + load, measured
};

class LocalPlanner {
public:
    explicit LocalPlanner(PlannerConfig config = {});

    cv::Point2f approachPoint(const cv::Point2f &target, const cv::Point2f &goal) const;
    std::vector<PlannedRoute> plan(const cv::Point2f &robot, const cv::Point2f &target,
                                   const cv::Point2f &goal,
                                   const std::vector<PlannerObstacle> &obstacles) const;
    // Route for a carried load, rather than the legacy target approach planner.
    PlannedRoute planCarry(const cv::Point2f& goal,const NavigationScene& scene,uint64_t now_us) const;
    bool sweptSegmentSafe(const cv::Point2f& a,const cv::Point2f& b,
                          const NavigationScene& scene,uint64_t now_us) const;
    bool segmentSafe(const cv::Point2f &a, const cv::Point2f &b,
                     const std::vector<PlannerObstacle> &obstacles) const;

private:
    PlannerConfig config_;
};

struct DropPlan {
    bool valid = false;
    cv::Point2f centre_zone_m;
    float radius_m = 0;
    std::string reason = "drop_evidence_missing";
};
class DropPlanner {
public:
    explicit DropPlanner(float half_width=0,float half_depth=0):half_width_(half_width),half_depth_(half_depth){}
    bool positionClear(const ZoneGeometry&,const ZoneEstimate&,const cv::Point2f&,
                       const std::vector<PlannerObstacle>&,bool injured,float radius) const;
    // Occupancy is expressed in the fixed zone frame; all objects (not just the
    // intended class) reserve space. Complete inventory is required even if empty.
    DropPlan plan(const ZoneGeometry& geometry,const ZoneEstimate& zone,bool identity_verified,
                  bool inventory_complete,const std::vector<PlannerObstacle>& occupied,
                  bool injured,float load_radius_m,uint64_t now_us) const;
private:
    float half_width_,half_depth_;
};
} // namespace rescue
