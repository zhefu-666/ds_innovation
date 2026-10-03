#pragma once
#include <string>

namespace rescue {
// Competition transport rules (stated 2026-10-01). Pure functions; the task
// state machine and the perception adapters both use these, never their own copies.
//  - Before one ordinary supply is verified in our supply half, only ordinary supplies move.
//  - At most kMaxSuppliesPerTrip supplies (ordinary/core) per trip.
//  - An injured person travels alone, exactly one per trip.
//  - Dangerous/unknown objects are never transported.
//  - Anything inside the opponent zone scores for the opponent: never release there.
enum class TargetKind { ORDINARY, CORE, INJURED, DANGEROUS, UNKNOWN };
// The current field strategy carries at most two supply objects per trip.
constexpr int kMaxSuppliesPerTrip = 2;

TargetKind targetKind(const std::string& label);
// May this label be chosen as the primary target of a new trip?
bool targetSelectable(const std::string& label, bool first_ordinary_delivered);

struct Inventory {
    int ordinary = 0, core = 0, injured = 0, dangerous = 0, unknown = 0;
    void add(const std::string& label);
    int count(TargetKind kind) const;
    int supplies() const { return ordinary + core; }
    int total() const { return ordinary + core + injured + dangerous + unknown; }
    bool operator==(const Inventory& o) const {
        return ordinary == o.ordinary && core == o.core && injured == o.injured &&
               dangerous == o.dangerous && unknown == o.unknown;
    }
    bool operator!=(const Inventory& o) const { return !(*this == o); }
};

enum class RuleVerdict {
    OK, EMPTY, INCOMPLETE, OCCLUDED, DANGEROUS, UNKNOWN_OBJECT,
    CORE_BEFORE_FIRST, INJURED_BEFORE_FIRST, INJURED_NOT_ALONE, TOO_MANY_SUPPLIES
};
const char* verdictName(RuleVerdict verdict);

// Is this set legal to carry in one trip (planned corridor set or verified held set)?
RuleVerdict checkTrip(const Inventory& set, bool first_ordinary_delivered);
// Before the rush: everything the gripper frame would sweep must form a legal trip.
// The first trip and injured trips additionally require an occlusion-free corridor,
// because a hidden core/injured object would make the trip illegal.
RuleVerdict checkCorridor(const Inventory& corridor, bool complete, bool occlusion_free,
                          const std::string& primary_label, bool first_ordinary_delivered);
// "injured" or "supply": which half of our own zone this (legal) set belongs in.
const char* requiredHalf(const Inventory& set);
} // namespace rescue
