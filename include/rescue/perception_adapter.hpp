#pragma once
#include "rescue/push_task.hpp"
#include <vector>
namespace rescue {
// Detection-only evidence. Metric geometry, route/sensor safety and delivery
// fields deliberately remain invalid until their independent adapters exist.
PushObservation makePushObservation(const std::vector<SegDetection> &detections,
    uint64_t now_us, bool first_delivered, float min_confidence);
}
