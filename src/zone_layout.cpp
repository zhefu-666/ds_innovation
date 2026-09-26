#include "rescue/zone_layout.hpp"

namespace rescue {

DeliveryClass deliveryClassForLabel(const std::string &label) {
    if (label == "dangerous_object") return DeliveryClass::REJECT;
    if (label == "injured_person") return DeliveryClass::INJURED;
    if (label == "ordinary_supply" || label == "core_supply") return DeliveryClass::SUPPLY;
    return DeliveryClass::REJECT;
}

bool isCorrectSubzone(const ZoneLayout &layout, const std::string &target_label,
                      ZoneLayout::Side observed_side) {
    if (!layout.complete()) return false;
    switch (deliveryClassForLabel(target_label)) {
    case DeliveryClass::SUPPLY: return observed_side == layout.supply_side;
    case DeliveryClass::INJURED: return observed_side == layout.injured_side;
    case DeliveryClass::REJECT: return false;
    }
    return false;
}

} // namespace rescue
