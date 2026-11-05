#pragma once
#include "rescue/core/types.hpp"
#include "rescue/task/transport_rules.hpp"
#include <algorithm>
#include <cmath>
#include <utility>
#include <set>
#include <vector>
namespace rescue {
// Rank a local attainable load, never the sum of unrelated detections in the
// image. The existing footprint window is only a nomination; capture still
// confirms the complete corridor and legal inventory before closing.
inline std::pair<int, int> candidateTripRank(const SegDetection& target,
    const std::vector<SegDetection>& detections, bool first, float confidence,
    uint64_t now, float core_confidence = 1.f) {
    if (!targetSelectable(target.label, first))
        return {0, 0};
    Inventory available;
    available.add(target.label);
    std::set<int> counted{target.track_id};
    if (target.ground_position_valid && std::isfinite(target.body_xy_m.x) &&
        std::isfinite(target.body_xy_m.y)) {
        for (const auto& d : detections) {
            if (d.track_id == target.track_id || d.track_id < 0 ||
                !targetSelectable(d.label, first) || !d.ground_position_valid ||
                !d.ground_contact_valid || d.body_xy_m.y <= 0 ||
                !std::isfinite(d.body_xy_m.x) || !std::isfinite(d.body_xy_m.y) ||
                !std::isfinite(d.confidence) ||
                d.confidence < (d.label == "core_supply" ? std::min(confidence, core_confidence) : confidence) ||
                !d.timestamp_us || now < d.timestamp_us || now - d.timestamp_us > 200000)
                continue;
            // A pair must fit one existing .18 x .12 m footprint window.
            if (std::abs(d.body_xy_m.x - target.body_xy_m.x) <= .09f &&
                std::abs(d.body_xy_m.y - target.body_xy_m.y) <= .06f &&
                counted.insert(d.track_id).second)
                available.add(d.label);
        }
    }
    // Three supplies in the same footprint are not an immediately carryable
    // pair. Score the primary alone until separation establishes a legal pair.
    if (available.supplies() > (first ? kMaxSuppliesPerTrip : kMaxSuppliesFirstTrip)) {
        available = {};
        available.add(target.label);
    }
    const auto choice = chooseTrip(available, first);
    if (!choice.inventory.count(targetKind(target.label)))
        return {0, 0};
    return {choice.score(), choice.inventory.injured > 0 ? 1 : 0};
}
} // namespace rescue
