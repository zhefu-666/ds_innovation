#pragma once
// TEMP_ASSUMPTION only: keeps the last coarse YOLO-pose zone fix in an odometry frame, so the zone pose
// in the body frame is still available (IMU yaw + scaled commanded travel) while the zone is out of view.
#include "rescue/zone_estimate.hpp"
#include <algorithm>
#include <cmath>

namespace rescue {
class AnchoredZone {
public:
    void reset() { *this = AnchoredZone{}; }
    // forward_m: real forward travel (commanded travel already divided by the motion ratio); dyaw: left positive.
    void advance(float forward_m, float dyaw) {
        th_ += .5f * dyaw;
        x_ += -forward_m * std::sin(th_); y_ += forward_m * std::cos(th_);
        th_ += .5f * dyaw;
    }
    // Returns true when the fix is accepted. A fix far from the prediction is only taken after 4 in a row.
    bool update(const ZoneEstimate &fix, uint64_t now_us) {
        const float c = std::cos(th_), s = std::sin(th_);
        const cv::Point2f o{c * fix.origin_body_m.x - s * fix.origin_body_m.y + x_,
                            s * fix.origin_body_m.x + c * fix.origin_body_m.y + y_};
        const float yaw = th_ + fix.yaw_body_rad;
        if (!valid_) { set(o, yaw, fix, now_us); ++accepted; return true; }
        last_dist = float(std::hypot(o.x - o_.x, o.y - o_.y)); last_dyaw = wrapAngle(yaw - yaw_);
        if (last_dist <= gate_m && std::abs(last_dyaw) <= .5f) {
            ++accepted;
            o_ = {o_.x + blend * (o.x - o_.x), o_.y + blend * (o.y - o_.y)};
            yaw_ = wrapAngle(yaw_ + blend * wrapAngle(yaw - yaw_));
            bad_ = 0; last_us_ = now_us; fix_ = fix; return true;
        }
        ++rejected;
        if (++bad_ >= 4) { set(o, yaw, fix, now_us); ++replaced; return true; }
        return false;
    }
    bool valid(uint64_t now_us, uint64_t max_age_us = 25000000) const {
        return valid_ && now_us >= last_us_ && now_us - last_us_ <= max_age_us;
    }
    // Zone pose in the current body frame.
    ZoneEstimate predicted(uint64_t now_us) const {
        ZoneEstimate z = fix_;
        const float c = std::cos(th_), s = std::sin(th_);
        const float dx = o_.x - x_, dy = o_.y - y_;
        z.origin_body_m = {c * dx + s * dy, -s * dx + c * dy};
        z.yaw_body_rad = wrapAngle(yaw_ - th_);
        z.valid = valid_; z.timestamp_us = now_us; z.observed_us = last_us_;
        z.source = ZoneEstimate::Source::PREDICTED;
        return z;
    }
    float gate_m = .25f, blend = .6f;
    float last_dist = 0, last_dyaw = 0;
    int accepted = 0, rejected = 0, replaced = 0;

private:
    void set(const cv::Point2f &o, float yaw, const ZoneEstimate &fix, uint64_t now_us) {
        o_ = o; yaw_ = yaw; fix_ = fix; valid_ = true; bad_ = 0; last_us_ = now_us;
    }
    float x_ = 0, y_ = 0, th_ = 0, yaw_ = 0;
    cv::Point2f o_;
    ZoneEstimate fix_;
    bool valid_ = false;
    int bad_ = 0;
    uint64_t last_us_ = 0;
};
} // namespace rescue
