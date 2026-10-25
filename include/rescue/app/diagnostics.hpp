#pragma once

#include "rescue/task/push_task.hpp"
#include "rescue/perception/zone_keypoints.hpp"
#include <iostream>

namespace rescue {
void report(const PushOutput& out, std::ostream& stream = std::cout);
void drawZone(cv::Mat& frame, const std::vector<ZoneKeypoint>& points);
void printZone(const ZoneHalves& halves, const std::vector<ZoneKeypoint>& points);
} // namespace rescue
