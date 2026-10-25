#pragma once

#include "rescue/transport_rules.hpp"
#include <opencv2/core/types.hpp>
#include <cstdint>
#include <string>

namespace rescue {
struct PushObservation;
struct TaskTuning;

struct DeliveryVerificationContext {
    uint64_t elapsed_us;
    const Inventory& trip;
    int baseline;
    int baseline_other;
    const std::string& geometry_id;
    bool observed_zone;
    float half_x;
    bool half_injured;
    bool tracked_cargo_has_fix;
    int repush_count;
};

struct DeliveryCargoEvidence {
    bool available = false;
    cv::Point2f rear;
    bool fixed = false, clipped = false, front_valid = false;
    float front_px = 0, bottom_px = 0, height_px = 0;
    bool other_inside = false;
};

enum class DeliveryVerificationOutcome {
    Continue,
    Repush,
    Complete
};

struct DeliveryVerificationResult {
    DeliveryVerificationOutcome outcome = DeliveryVerificationOutcome::Continue;
    int credited = 0;
    bool visual = false;
    std::string reason;
    std::string events;
};

class DeliveryVerificationStage {
  public:
    DeliveryVerificationResult update(const PushObservation&,
                                      const TaskTuning&,
                                      const DeliveryVerificationContext&,
                                      const DeliveryCargoEvidence&);
    // Legacy lifecycle: phase entry clears confirmations, while a new gate baseline
    // clears the inventory window. Pause and repush retain the remaining evidence.
    void resetConfirmations();
    void resetInventoryEvidence();
    void clearVisualCounts();
    void clearTrip();
    bool farView() const {
        return far_view_;
    }
    int insideCount() const {
        return inside_;
    }
    int outsideCount() const {
        return outside_;
    }

  private:
    int confirmations_ = 0, delta_ = 0;
    uint64_t frame_ = 0, stable_since_us_ = 0;
    int inside_ = 0, outside_ = 0;
    bool far_view_ = false;
    uint64_t fix_us_ = 0, cargo_ts_us_ = 0;
};
} // namespace rescue
