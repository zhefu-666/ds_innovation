#include "rescue/transport_rules.hpp"

namespace rescue {
TargetKind targetKind(const std::string& label) {
    if (label == "ordinary_supply") return TargetKind::ORDINARY;
    if (label == "core_supply") return TargetKind::CORE;
    if (label == "injured_person") return TargetKind::INJURED;
    if (label == "dangerous_object") return TargetKind::DANGEROUS;
    return TargetKind::UNKNOWN;
}
bool targetSelectable(const std::string& label, bool first) {
    switch (targetKind(label)) {
    case TargetKind::ORDINARY: return true;
    case TargetKind::CORE: case TargetKind::INJURED: return first;
    default: return false;
    }
}
void Inventory::add(const std::string& label) {
    switch (targetKind(label)) {
    case TargetKind::ORDINARY: ++ordinary; break;
    case TargetKind::CORE: ++core; break;
    case TargetKind::INJURED: ++injured; break;
    case TargetKind::DANGEROUS: ++dangerous; break;
    case TargetKind::UNKNOWN: ++unknown; break;
    }
}
int Inventory::count(TargetKind kind) const {
    switch (kind) {
    case TargetKind::ORDINARY: return ordinary;
    case TargetKind::CORE: return core;
    case TargetKind::INJURED: return injured;
    case TargetKind::DANGEROUS: return dangerous;
    case TargetKind::UNKNOWN: return unknown;
    }
    return 0;
}
const char* verdictName(RuleVerdict v) {
    switch (v) {
    case RuleVerdict::OK: return "ok";
    case RuleVerdict::EMPTY: return "empty";
    case RuleVerdict::INCOMPLETE: return "incomplete";
    case RuleVerdict::OCCLUDED: return "occluded";
    case RuleVerdict::DANGEROUS: return "dangerous";
    case RuleVerdict::UNKNOWN_OBJECT: return "unknown_object";
    case RuleVerdict::CORE_BEFORE_FIRST: return "core_before_first";
    case RuleVerdict::INJURED_BEFORE_FIRST: return "injured_before_first";
    case RuleVerdict::INJURED_NOT_ALONE: return "injured_not_alone";
    case RuleVerdict::TOO_MANY_SUPPLIES: return "too_many_supplies";
    }
    return "unknown";
}
RuleVerdict checkTrip(const Inventory& s, bool first) {
    if (s.total() <= 0) return RuleVerdict::EMPTY;
    if (s.dangerous > 0) return RuleVerdict::DANGEROUS;
    if (s.unknown > 0) return RuleVerdict::UNKNOWN_OBJECT;
    if (s.injured > 0) {
        if (!first) return RuleVerdict::INJURED_BEFORE_FIRST;
        if (s.injured != 1 || s.supplies() != 0) return RuleVerdict::INJURED_NOT_ALONE;
        return RuleVerdict::OK;
    }
    if (s.supplies() > kMaxSuppliesPerTrip) return RuleVerdict::TOO_MANY_SUPPLIES;
    if (s.core > 0 && !first) return RuleVerdict::CORE_BEFORE_FIRST;
    return RuleVerdict::OK;
}
RuleVerdict checkCorridor(const Inventory& c, bool complete, bool occlusion_free,
                          const std::string& primary, bool first) {
    if (!complete || c.count(targetKind(primary)) < 1) return RuleVerdict::INCOMPLETE;
    // The corridor contains the primary, so an illegal primary fails checkTrip too.
    const auto verdict = checkTrip(c, first);
    if (verdict != RuleVerdict::OK) return verdict;
    if ((!first || c.injured > 0) && !occlusion_free) return RuleVerdict::OCCLUDED;
    return RuleVerdict::OK;
}
const char* requiredHalf(const Inventory& s) { return s.injured > 0 ? "injured" : "supply"; }
} // namespace rescue
