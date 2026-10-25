#include "rescue/task/disperse_rush_stage.hpp"
#include "rescue/push_task.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace rescue {
void DisperseRushStage::beginSweep(bool pending) {
    pending_ = pending;
    active_ = false;
    cap_positive_ = cap_negative_ = 99.f;
}
void DisperseRushStage::clear() {
    pending_ = active_ = false;
    cap_positive_ = cap_negative_ = 99.f;
}
DisperseRushResult DisperseRushStage::update(const TaskTuning& t, const PushObservation& in, const DisperseRushContext& c) {
    DisperseRushResult result;
    const auto plan = [&] {
        pending_ = false;
        if (!t.swing_avoid_outside || !c.scatter_state || !c.clutter_fresh) return;
        const float done = (c.travel_m - c.sweep_travel_m) / c.motion_ratio;
        const cv::Point2f cluster(0.f, c.cluster_range_m - done), end(0.f, std::max(0.f, c.rush_distance_m - done));
        std::vector<cv::Point2f> pts = in.clutter_body;
        if (c.has_target) pts.emplace_back(c.target_body);
        float cap_pos = 99.f, cap_neg = 99.f;
        int n_out = 0;
        for (const auto& q : pts) {
            if (!std::isfinite(q.x) || !std::isfinite(q.y) || cv::norm(q - cluster) <= t.disperse_radius_m) continue;
            const cv::Point2f r = q - end;
            if (cv::norm(r) > t.swing_avoid_radius_m) continue;
            const float b = std::atan2(-r.x, r.y); // left (positive yaw) is positive
            ++n_out;
            if (b > 0.f) cap_pos = std::min(cap_pos, std::max(0.f, b - t.swing_avoid_margin_rad));
            else cap_neg = std::min(cap_neg, std::max(0.f, -b - t.swing_avoid_margin_rad));
        }
        cap_positive_ = cap_pos;
        cap_negative_ = cap_neg;
        active_ = true;
        result.plan_event = true;
        result.outside_n = n_out;
        result.cap_positive = cap_pos;
        result.cap_negative = cap_neg;
        result.first_sign = cap_pos != cap_neg ? (cap_pos > cap_neg ? 1.f : -1.f) : 0.f; // the first swing (and the extra one) goes to the freer side
    };
    if (pending_ && (!c.far_judge || c.pitch_ready)) plan();
    if (c.travel_m - c.sweep_travel_m >= c.rush_distance_m * c.motion_ratio || c.now_us - c.sweep_start_us > t.disperse_rush_budget_us) {
        if (pending_) plan();
        result.outcome = DisperseRushOutcome::Complete;
        return result;
    }
    const bool opening_impact = t.first_burst_full_impact && c.burst_swing && c.scatter_state;
    result.reason = opening_impact ? "first_burst_impact" : "disperse_rush";
    result.motion_requested = true;
    result.vx = opening_impact ? t.first_burst_speed : t.disperse_speed;
    return result;
}
} // namespace rescue
