#pragma once
#include "rescue/push_task.hpp"
namespace rescue {
class CarryNavigator {
public:
    CarryNavigator(ZoneGeometry geometry,TaskTuning tuning,float load_radius,bool controlled=false,float load_half_width=0,float load_half_depth=0)
        : geometry_(std::move(geometry)), tuning_(tuning), radius_(load_radius), controlled_(controlled),drops_(load_half_width,load_half_depth) {}
    void update(PushObservation& input,const PushOutput& previous);
private:
    ZoneGeometry geometry_;
    TaskTuning tuning_;
    float radius_;
    bool controlled_;
    // Only the controlled static test scene may retain the pre-release background
    // while its known carried load temporarily occludes/crosses the zone boundary.
    std::vector<PlannerObstacle> release_background_;
    std::string release_geometry_id_;
    uint64_t release_background_us_=0;
    LocalPlanner routes_;
    DropPlanner drops_;
};
}
