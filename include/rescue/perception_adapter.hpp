#pragma once
#include "rescue/push_task.hpp"
#include <vector>
namespace rescue {
// Search mode considers all known classes; cargo mode enforces first-green and black priority.
// Detection-only evidence. Metric geometry, route/sensor safety and delivery
// fields deliberately remain invalid until their independent adapters exist.
PushObservation makePushObservation(const std::vector<SegDetection> &detections,
    uint64_t now_us, bool first_delivered, float min_confidence, int locked_id = -1,
    const std::vector<int>& rejected = {}, bool allow_search_cues = false, bool locked_is_search_cue = false, bool require_geometry = false,
    const std::string& prefer_label = "");
}
