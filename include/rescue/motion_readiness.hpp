#pragma once
#include "rescue/push_task.hpp"
namespace rescue {
// Only planned, stationary camera work may temporarily lack a ground projection.
// Synchronization, IMU health, stale frames and all other faults remain fatal.
inline bool stationaryPitchWork(const PushOutput& previous, const std::string& reason) {
    if (previous.motion.vx_mps != 0 || previous.motion.wz_rps != 0 ||
        (reason != "pitch_moving" && reason != "pitch_not_calibrated")) return false;
    // Returning from a near view commands FAR while already in SCAN.
    // Permit only stationary servo travel; an uncalibrated settled view is a fault.
    if ((previous.state == PushState::SCAN || previous.state == PushState::LOST_SEARCH))
        return reason == "pitch_moving" && previous.motion.camera_pitch_cdeg != kCameraPitchInvalid;
    switch (previous.state) {
    case PushState::CUE_PREPARE: case PushState::CLEAR_PAUSE:
    case PushState::MID_REACQUIRE: case PushState::MID_APPROACH:
    case PushState::NEAR_REACQUIRE: case PushState::CUE_APPROACH: case PushState::SELECT_CARGO: case PushState::APPROACH:
    case PushState::LOWER_FRAME: case PushState::VERIFY_CAPTURE:
    case PushState::CARRY: case PushState::GATE: case PushState::PREPARE:
    case PushState::ENTER: case PushState::RAISE_RELEASE: case PushState::CAPTURE_FAIL:
    case PushState::LOST_HOLD: case PushState::ABORT_DROP: return true;
    default: return false;
    }
}
inline void inhibitUnmappedMotion(MotionCommand& motion, bool mapping_valid) {
    if (!mapping_valid) motion.vx_mps = motion.wz_rps = 0;
}
}
