#pragma once

#include <string>
#include "rescue/zone_geometry.hpp"

namespace rescue {

struct ZoneLayout {
    enum Side { LEFT, RIGHT };
    std::string zone_label;
    Side supply_side = LEFT;
    Side injured_side = RIGHT;
    float field_heading_deg = 0.0f;

    bool complete() const { return !zone_label.empty() && supply_side != injured_side; }
};

struct SubzoneDecision {
    bool valid=false;
    ZoneLayout::Side side=ZoneLayout::LEFT;
    cv::Point2f stop_zone_m;
    std::string zone_class, reason="invalid_stop";
};
// The point is a planner's predicted final stop, not a detection centre. radius_m
// conservatively bounds the target footprint; rejection near boundaries is explicit.
SubzoneDecision classifyExpectedStop(const ZoneGeometry&, const ZoneEstimate&,
    const cv::Point2f& expected_stop_body_m, float radius_m, uint64_t now_us);

enum class DeliveryClass { SUPPLY, INJURED, REJECT };

DeliveryClass deliveryClassForLabel(const std::string &label);
bool isCorrectSubzone(const ZoneLayout &layout, const std::string &target_label,
                      ZoneLayout::Side observed_side);

} // namespace rescue
