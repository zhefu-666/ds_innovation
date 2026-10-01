#pragma once
#include "rescue/push_task.hpp"
namespace rescue {
class CarryNavigator {
public:
    CarryNavigator(ZoneGeometry geometry,TaskTuning tuning,float load_radius)
        : geometry_(std::move(geometry)), tuning_(tuning), radius_(load_radius) {}
    void update(PushObservation& input,const PushOutput& previous) const;
private:
    ZoneGeometry geometry_;
    TaskTuning tuning_;
    float radius_;
    LocalPlanner routes_;
    DropPlanner drops_;
};
}
