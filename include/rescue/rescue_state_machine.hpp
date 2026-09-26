#pragma once

#include "rescue/planner.hpp"
#include "rescue/sensor_fusion.hpp"
#include "rescue/types.hpp"
#include "rescue/zone_layout.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace rescue {

enum class RescueState {
    WAIT_START,
    DEPART,
    SEARCH_TARGET,
    SELECT_TARGET,
    APPROACH_TARGET,
    ALIGN_PUSH,
    PUSH_TARGET,
    NAVIGATE_DROP_ZONE,
    IDENTIFY_DROP_SUBZONE,
    ALIGN_DROP,
    VERIFY_DELIVERY,
    RETURN_SEARCH,
    RECOVERY_SEARCH,
    EMERGENCY_STOP,
};

struct RescueInputs {
    uint64_t now_us = 0;
    bool start_requested = false;
    bool reset_requested = false;
    bool target_visible = false;
    SegDetection target;
    // Explicit development escape hatch. Competition/safety deployments
    // must leave this false until instance masks are available.
    bool experimental_box_only = false;
    bool safe_zone_visible = false;
    SafeZonePose safe_zone_pose;
    bool target_in_subzone = false;
    bool robot_disconnected = false;
    bool communication_ok = true;
    bool delivery_stable = false;
    SensorState sensors;
};

class RescueStateMachine {
public:
    explicit RescueStateMachine(SensorFusionConfig safety = {});
    MotionCommand update(const RescueInputs &inputs);

    RescueState state() const { return state_; }
    bool ordinaryDelivered() const { return ordinary_delivered_; }
    void markOrdinaryDelivered(bool delivered = true) { ordinary_delivered_ = delivered; }
    static const char *stateName(RescueState state);

private:
    void transition(RescueState next);
    bool targetAllowed(const SegDetection &target, bool experimental_box_only) const;

    RescueState state_ = RescueState::WAIT_START;
    bool ordinary_delivered_ = false;
    uint32_t verify_frames_ = 0;

    SensorFusion fusion_;
};

} // namespace rescue
