#include "rescue/task/disperse_turn_stage.hpp"
#include "rescue/push_task.hpp"
#include <algorithm>
#include <cmath>

namespace rescue {
void DisperseTurnStage::begin(bool enabled) {
    phase_ = enabled ? DisperseTurnPhase::Look : DisperseTurnPhase::Idle;
    look_ready_us_ = 0;
    clear_count_ = 0;
}

DisperseTurnResult DisperseTurnStage::update(const PushObservation& in, const TaskTuning& t,
                                           const DisperseTurnContext& c) {
    DisperseTurnResult result;
    if (!active()) return result;
    const auto now = c.now_us;
    auto started = c.sweep_start_us;
    float yaw0 = c.sweep_yaw;
    const auto viewCount = [&](int& left, int& right) {
        if (!(in.clutter_ts_us && now >= in.clutter_ts_us && now - in.clutter_ts_us <= t.sweep_fresh_us)) return false;
        left = right = 0;
        for (const auto& q : in.clutter_body) {
            if (!std::isfinite(q.x) || !std::isfinite(q.y) || q.y <= t.sweep_frame_front_m + .03f || cv::norm(q) > t.disperse_turn_view_range_m) continue;
            const float bearing = std::atan2(-q.x, q.y);
            if (std::abs(bearing) > t.disperse_turn_view_half_rad) continue;
            ++(bearing > 0.f ? left : right);
        }
        return true;
    };
    if (phase_ == DisperseTurnPhase::Look) {
        if (!c.camera_ready) { result.reason = "disperse_look"; return result; }
        if (!look_ready_us_) look_ready_us_ = now;
        int left = 0, right = 0;
        const bool seen = in.clutter_ts_us >= look_ready_us_ && viewCount(left, right);
        if (!seen && now - look_ready_us_ < t.disperse_turn_look_us) { result.reason = "disperse_look"; return result; }
        direction_ = seen && left != right ? (left < right ? 1.f : -1.f) : c.fallback_direction;
        phase_ = DisperseTurnPhase::Turn;
        yaw0 = in.heading_rad; started = now; clear_count_ = 0;
        result.turn_started = true;
    }
    if (phase_ == DisperseTurnPhase::Push) {
        if (c.travel_m - c.sweep_travel_m >= t.disperse_turn_push_m * c.motion_ratio || now - started > t.disperse_turn_push_budget_us) {
            phase_ = DisperseTurnPhase::Idle; result.outcome = DisperseTurnOutcome::Complete;
        } else {
            result.reason = "disperse_turn_push"; result.motion = DisperseTurnMotion::Push;
        }
        return result;
    }
    const float turned = wrapAngle(in.heading_rad - yaw0) * direction_;
    float cap = t.disperse_turn_rad;
    if (c.swing_plan_active) cap = std::min(cap, direction_ > 0 ? c.cap_positive : c.cap_negative);
    const bool out_of_room = turned >= cap - t.disperse_swing_tol_rad || now - started > t.disperse_swing_budget_us;
    const auto endTurn = [&] {
        if (t.disperse_turn_push_m > 0.f) {
            phase_ = DisperseTurnPhase::Push; result.push_started = true;
        } else {
            phase_ = DisperseTurnPhase::Idle; result.outcome = DisperseTurnOutcome::Complete;
        }
    };
    const auto spin = [&](const char* reason) {
        result.reason = reason; result.motion = DisperseTurnMotion::Turn; result.direction = direction_;
    };
    if (phase_ == DisperseTurnPhase::FrameCheck) {
        if (out_of_room || turned - frame_turned0_ >= t.disperse_turn_frame_extra_rad) { endTurn(); return result; }
        if (!c.camera_ready) { result.reason = "disperse_frame_look"; return result; }
        if (!look_ready_us_) look_ready_us_ = now;
        if (in.clutter_ts_us >= look_ready_us_ && in.clutter_ts_us != frame_timestamp_us_ && now >= in.clutter_ts_us && now - in.clutter_ts_us <= t.sweep_fresh_us) {
            int count = 0;
            for (const auto& q : in.clutter_body)
                if (std::isfinite(q.x) && std::isfinite(q.y) && q.y > 0.f && q.y <= t.disperse_turn_frame_range_m && std::abs(q.x) <= t.disperse_turn_frame_half_m) ++count;
            frame_timestamp_us_ = in.clutter_ts_us;
            clear_count_ = count <= t.disperse_turn_frame_max ? clear_count_ + 1 : 0;
            result.frame_event = true; result.frame_count = count;
            result.turned = turned; result.confirmations = clear_count_;
        }
        if (clear_count_ >= t.disperse_turn_clear_frames) { endTurn(); return result; }
        if (!frame_timestamp_us_) {
            if (now - look_ready_us_ >= t.disperse_turn_look_us) { endTurn(); return result; }
            result.reason = "disperse_frame_look"; return result;
        }
        if (clear_count_) { result.reason = "disperse_frame_confirm"; return result; }
        spin("disperse_turn_frame");
        return result;
    }
    int left = 0, right = 0;
    if (viewCount(left, right)) clear_count_ = left + right <= t.disperse_turn_clear_max ? clear_count_ + 1 : 0;
    const bool thin = clear_count_ >= t.disperse_turn_clear_frames;
    if (out_of_room) { endTurn(); return result; }
    if (thin && turned >= std::min(t.disperse_turn_min_rad, cap)) {
        if (!(t.disperse_turn_frame_check && c.mid_pitch)) { endTurn(); return result; }
        phase_ = DisperseTurnPhase::FrameCheck; frame_turned0_ = turned;
        look_ready_us_ = frame_timestamp_us_ = 0; clear_count_ = 0;
        result.reason = "disperse_frame_look"; return result;
    }
    spin("disperse_turn");
    return result;
}
} // namespace rescue
