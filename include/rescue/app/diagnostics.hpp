#pragma once

#include "rescue/task/push_task.hpp"
#include "rescue/perception/zone_keypoints.hpp"
#include "rescue/geometry/camera_calibration.hpp"
#include <iostream>

namespace rescue {
void report(const PushOutput& out, std::ostream& stream = std::cout);
void drawZone(cv::Mat& frame, const std::vector<ZoneKeypoint>& points);
// Diagnostic rigid fit only; never supplies navigation or acceptance evidence.
void drawZoneProjection(cv::Mat& frame,
                        const std::vector<ZoneKeypoint>& points,
                        const CameraCalibration& calibration,
                        const ZoneGeometry& geometry,
                        int16_t pitch,
                        const SensorState& sensor,
                        float confidence,
                        bool mapping_valid,
                        const std::string& geometry_reason);
void printZone(const ZoneHalves& halves, const std::vector<ZoneKeypoint>& points);
} // namespace rescue
