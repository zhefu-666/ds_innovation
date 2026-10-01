#pragma once
#include "rescue/capture_monitor.hpp"
namespace rescue {
struct TaskCalibration {
    TaskTuning task;
    CaptureConfig capture;
    float load_radius_m = 0; // enclosing radius of the maximum validated load footprint
};
// Explicit measured gripper geometry + pitch-specific image region. No defaults
// are silently marked calibrated; invalid/incomplete files fail before opening hardware.
TaskCalibration loadTaskCalibration(const std::string& path,TaskTuning tuning,int width,int height);
}
