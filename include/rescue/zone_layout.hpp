#pragma once

#include <string>

namespace rescue {

struct ZoneLayout {
    enum Side { LEFT, RIGHT };
    std::string zone_label;
    Side supply_side = LEFT;
    Side injured_side = RIGHT;
    float field_heading_deg = 0.0f;

    bool complete() const { return !zone_label.empty() && supply_side != injured_side; }
};

enum class DeliveryClass { SUPPLY, INJURED, REJECT };

DeliveryClass deliveryClassForLabel(const std::string &label);
bool isCorrectSubzone(const ZoneLayout &layout, const std::string &target_label,
                      ZoneLayout::Side observed_side);

} // namespace rescue
