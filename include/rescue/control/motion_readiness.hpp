#pragma once
#include "rescue/task/push_task.hpp"
namespace rescue {
// Startup is timed/image-guided and does not require IMU readiness. Direct tests of
// other phases retain their IMU gate even before their first task update.
inline bool imuReadinessRequired(TaskPhase phase, const std::string& single_phase) {
    return phase != TaskPhase::Startup || (!single_phase.empty() && single_phase != "startup");
}
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

// Check ramped motion at the same final boundary used by the live sender.
inline bool residualMotionBlocked(const PushObservation& input, const PushOutput& out,
                                  float sent_vx, float sent_wz) {
    // Match the bounded straight jaw retreat permission from the task. Keep
    // forward/angular residual checks and runtime hard-stop conditions intact.
    const bool jaw_retreat = out.jaw_release_retreat && out.motion.vx_mps < 0 &&
                             out.motion.wz_rps == 0;
    return (sent_vx > 0 && !input.path_safe) ||
           (sent_vx < 0 && !input.retreat_safe && !jaw_retreat) ||
           (input.directional_clearance_valid &&
            (sent_wz != 0 || out.motion.wz_rps != 0) &&
            ((!input.turn_safe && sent_vx == 0 && out.motion.vx_mps == 0) ||
             (!input.arc_safe && (sent_vx != 0 || out.motion.vx_mps != 0))));
}

inline void inhibitUnmappedMotion(MotionCommand& motion, bool mapping_valid) {
    if (!mapping_valid) {
        motion.vx_mps = motion.wz_rps = 0;
    }
}
} // namespace rescue
