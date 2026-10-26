#pragma once
#include "rescue/task/push_task.hpp"
namespace rescue {
// Only planned, stationary camera work may temporarily lack a ground projection.
// Synchronization, IMU health, stale frames and all other faults remain fatal.
inline bool stationaryPitchWork(const PushOutput& previous, const std::string& reason) {
    if (previous.motion.vx_mps != 0 || previous.motion.wz_rps != 0 ||
        (reason != "pitch_moving" && reason != "pitch_not_calibrated"))
        return false;
    // Returning from a near view commands FAR while already in SCAN.
    // Permit only stationary servo travel; an uncalibrated settled view is a fault.
    if (previous.scan_pitch_return)
        return reason == "pitch_moving" && previous.motion.camera_pitch_cdeg != kCameraPitchInvalid;
    return previous.stationary_pitch_work;
}

inline void inhibitUnmappedMotion(MotionCommand& motion, bool mapping_valid) {
    if (!mapping_valid) {
        motion.vx_mps = motion.wz_rps = 0;
    }
}
} // namespace rescue
