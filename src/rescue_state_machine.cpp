#include "rescue/rescue_state_machine.hpp"

namespace rescue {

RescueStateMachine::RescueStateMachine(SensorFusionConfig safety) : fusion_(safety) {}

const char *RescueStateMachine::stateName(RescueState state) {
    switch (state) {
    case RescueState::WAIT_START: return "WAIT_START";
    case RescueState::DEPART: return "DEPART";
    case RescueState::SEARCH_TARGET: return "SEARCH_TARGET";
    case RescueState::SELECT_TARGET: return "SELECT_TARGET";
    case RescueState::APPROACH_TARGET: return "APPROACH_TARGET";
    case RescueState::ALIGN_PUSH: return "ALIGN_PUSH";
    case RescueState::PUSH_TARGET: return "PUSH_TARGET";
    case RescueState::NAVIGATE_DROP_ZONE: return "NAVIGATE_DROP_ZONE";
    case RescueState::IDENTIFY_DROP_SUBZONE: return "IDENTIFY_DROP_SUBZONE";
    case RescueState::ALIGN_DROP: return "ALIGN_DROP";
    case RescueState::VERIFY_DELIVERY: return "VERIFY_DELIVERY";
    case RescueState::RETURN_SEARCH: return "RETURN_SEARCH";
    case RescueState::RECOVERY_SEARCH: return "RECOVERY_SEARCH";
    case RescueState::EMERGENCY_STOP: return "EMERGENCY_STOP";
    }
    return "UNKNOWN";
}

void RescueStateMachine::transition(RescueState next) {
    state_ = next;
    if (next != RescueState::VERIFY_DELIVERY) verify_frames_ = 0;
}

bool RescueStateMachine::targetAllowed(const SegDetection &target, bool experimental_box_only) const {
    if (target.mask.empty() && !experimental_box_only) return false;
    if (target.label == "dangerous_object") return false;
    if ((target.label == "core_supply" || target.label == "injured_person") && !ordinary_delivered_)
        return false;
    return target.label == "ordinary_supply" || target.label == "core_supply" ||
           target.label == "injured_person";
}

MotionCommand RescueStateMachine::update(const RescueInputs &inputs) {
    fusion_.update(inputs.sensors);
    MotionCommand command;





    if (inputs.reset_requested) {
        transition(RescueState::WAIT_START);
        ordinary_delivered_ = false;
    }
    if (fusion_.emergencyStop(inputs.now_us) || !inputs.communication_ok) {
        transition(RescueState::EMERGENCY_STOP);
    }

    switch (state_) {
    case RescueState::WAIT_START:
        if (inputs.start_requested) transition(RescueState::DEPART);
        break;
    case RescueState::DEPART:

        command.vx_mps = 0.12f;

        transition(RescueState::SEARCH_TARGET);
        break;
    case RescueState::SEARCH_TARGET:
        if (inputs.target_visible) transition(RescueState::SELECT_TARGET);
        break;
    case RescueState::SELECT_TARGET:
        if (inputs.target_visible && targetAllowed(inputs.target, inputs.experimental_box_only)) transition(RescueState::APPROACH_TARGET);
        else transition(RescueState::RECOVERY_SEARCH);
        break;
    case RescueState::APPROACH_TARGET:
        if (!inputs.target_visible) transition(RescueState::RECOVERY_SEARCH);
        else if (inputs.target.body_xy_m.y < 0.45f) transition(RescueState::ALIGN_PUSH);
        else {

            command.vx_mps = 0.08f;
            command.wz_rps = -inputs.target.body_xy_m.x;
        }
        break;
    case RescueState::ALIGN_PUSH:
        transition(RescueState::PUSH_TARGET);
        break;
    case RescueState::PUSH_TARGET:

        command.vx_mps = 0.05f;
        if (inputs.safe_zone_visible) transition(RescueState::NAVIGATE_DROP_ZONE);
        break;
    case RescueState::NAVIGATE_DROP_ZONE:
        if (inputs.safe_zone_pose.valid && inputs.safe_zone_pose.view == SafeZonePose::FRONT)
            transition(RescueState::IDENTIFY_DROP_SUBZONE);
        break;
    case RescueState::IDENTIFY_DROP_SUBZONE:
        if (inputs.target_in_subzone) transition(RescueState::ALIGN_DROP);
        break;
    case RescueState::ALIGN_DROP:
        transition(RescueState::VERIFY_DELIVERY);
        break;
    case RescueState::VERIFY_DELIVERY:
        if (inputs.target_in_subzone && inputs.delivery_stable) ++verify_frames_;
        else verify_frames_ = 0;

        command.vx_mps = -0.04f;
        if (verify_frames_ >= 8 && inputs.robot_disconnected) {
            ordinary_delivered_ = ordinary_delivered_ || inputs.target.label == "ordinary_supply";
            transition(RescueState::RETURN_SEARCH);
        }
        break;
    case RescueState::RETURN_SEARCH:

        command.vx_mps = -0.06f;
        transition(RescueState::SEARCH_TARGET);
        break;
    case RescueState::RECOVERY_SEARCH:

        command.wz_rps = 0.35f;
        if (inputs.target_visible) transition(RescueState::SELECT_TARGET);
        break;
    case RescueState::EMERGENCY_STOP:

        if (inputs.reset_requested && inputs.start_requested) transition(RescueState::DEPART);
        break;
    }
    return fusion_.protect(command, inputs.now_us);
}

} // namespace rescue
