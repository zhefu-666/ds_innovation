#include "rescue/push_task.hpp"
#include <algorithm>
#include <cmath>

namespace rescue {
const char *PushTask::name(PushState s) {
    switch (s) {
    case PushState::WAIT_START: return "WAIT_START";
    case PushState::SEARCH_TARGET: return "SEARCH_TARGET";
    case PushState::APPROACH_TARGET: return "APPROACH_TARGET";
    case PushState::PUSH_TARGET: return "PUSH_TARGET";
    case PushState::ALIGN_ZONE: return "ALIGN_ZONE";
    case PushState::VERIFY_DELIVERY: return "VERIFY_DELIVERY";
    case PushState::BACK_OUT: return "BACK_OUT";
    }
    return "UNKNOWN";
}
void PushTask::abandon() {
    state_ = PushState::SEARCH_TARGET;
    target_id_ = -1; batch_ = 0; label_.clear(); confirmations_ = 0; phase_us_ = 0;
}
PushOutput PushTask::update(const PushObservation &in) {
    MotionCommand motion; // zero velocity by default


    const auto result = [&] {
        return PushOutput{state_, motion, batch_, total_, first_};
    };
    if (in.reset) {
        abandon(); state_ = PushState::WAIT_START; total_ = 0; first_ = false;
        last_us_ = in.now_us;
        return result();
    }
    const bool timely = in.now_us > 0 &&
        (last_us_ == 0 || (in.now_us > last_us_ && in.now_us - last_us_ <= 200000));
    last_us_ = in.now_us;
    if (!in.run || !in.safety_ok || !timely) {
        abandon(); state_ = PushState::WAIT_START;
        return result();
    }

    if (state_ == PushState::WAIT_START) {
        state_ = PushState::SEARCH_TARGET;
        return result();
    }
    const bool allowed = in.label == "ordinary_supply" ||
        (first_ && (in.label == "core_supply" || in.label == "injured_person"));
    const bool valid = in.target_valid && in.target_id >= 0 && allowed &&
        in.geometry_valid && in.path_safe && in.available_count > 0 &&
        std::isfinite(in.distance_m) && in.distance_m >= 0 &&
        std::isfinite(in.heading_error) && std::abs(in.heading_error) <= 1.0f;
    const bool zone = in.zone_valid && in.zone_own &&
        in.zone_class == (label_ == "injured_person" ? "injured" : "supply");
    const auto drive = [&](float speed, float turn = 0) {

        motion.vx_mps = speed; motion.wz_rps = std::clamp(turn, -0.3f, 0.3f);
    };
    if (state_ == PushState::SEARCH_TARGET) {
        if (!valid) { confirmations_ = 0; target_id_ = -1; return result(); }
        if (target_id_ != in.target_id || label_ != in.label) {
            target_id_ = in.target_id; label_ = in.label; confirmations_ = 0;
        }
        if (++confirmations_ >= 3) {
            // Available objects are a cap, not permission to change a batch in transit.
            batch_ = !first_ || label_ == "injured_person" ? 1 : std::min(3, in.available_count);
            state_ = PushState::APPROACH_TARGET; phase_us_ = in.now_us;
            confirmations_ = 0;
        }
        return result();
    }
    // Never substitute a cached observation for the locked target, including retreat.
    if (!valid || in.target_id != target_id_ || in.label != label_) {
        abandon(); return result();
    }
    const uint64_t elapsed = in.now_us - phase_us_;
    if (elapsed > 8000000) { abandon(); return result(); }
    switch (state_) {
    case PushState::APPROACH_TARGET:
        if (in.in_push_region && in.distance_m <= 0.25f && std::abs(in.heading_error) <= 0.05f) {
            state_ = PushState::PUSH_TARGET; phase_us_ = in.now_us;
        } else drive(0.08f, in.heading_error);
        break;
    case PushState::PUSH_TARGET:
        // A visible, matching destination and safe route are prerequisites for pushing.
        if (!zone) break;
        if (in.zone_aligned) { state_ = PushState::ALIGN_ZONE; phase_us_ = in.now_us; }
        else drive(label_ == "injured_person" ? 0.03f : 0.05f, in.heading_error);
        break;
    case PushState::ALIGN_ZONE:
        if (!zone || !in.zone_aligned) { confirmations_ = 0; break; }
        if (in.fully_inside && in.off_fence && in.stable && in.delivered_count == batch_) {
            state_ = PushState::VERIFY_DELIVERY; confirmations_ = 0;
        } else if (elapsed <= 1000000) drive(0.03f, in.heading_error);
        else abandon(); // A timed push is not a successful delivery.
        break;
    case PushState::VERIFY_DELIVERY:
        if (!zone || !in.fully_inside || !in.off_fence || !in.stable || in.delivered_count != batch_) {
            abandon(); break;
        }
        if (++confirmations_ >= 8) { state_ = PushState::BACK_OUT; phase_us_ = in.now_us; }
        break;
    case PushState::BACK_OUT:
        if (!zone || !in.fully_inside || !in.off_fence || !in.stable || in.delivered_count != batch_) {
            abandon(); break;
        }
        if (in.separated) {
            total_ += batch_; first_ = first_ || label_ == "ordinary_supply"; abandon();
        } else if (elapsed <= 1000000 && in.retreat_safe) drive(-0.04f);
        else if (elapsed > 1000000) abandon();
        break;
    default: break;
    }
    return result();
}
} // namespace rescue
