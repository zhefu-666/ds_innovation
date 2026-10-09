#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <sstream>
#include <string>

namespace rescue {

// IMU based "commanded to move but not moving" watchdog.
// suspect (log only): no vibration while commanded. A stalled wheel spins and vibrates MORE, so this misses rim contact.
// blocked (consumed by PushTask): while driving forward with a commanded turn, the yaw rate measured by the gyro is a
// small fraction of the command. Windowed ratio sum(|gyro|*dt)/sum(|cmd_wz|*dt); free travel measured 0.2~0.3, pressed against the rim 0.07~0.2 (run42), hence start <0.16 / release >=0.22.
struct MotionWatchConfig {
    float cmd_min_mps = .05f, cmd_min_wz_rps = .08f;
    float still_acc_std_mps2 = .15f, still_gyro_rps = .04f;
    uint64_t window_us = 400000, suspect_us = 1500000, imu_max_age_us = 200000;
    float block_min_vx_mps = .10f, block_min_turn_rad = .25f, block_ratio = .16f, block_release_ratio = .22f;
    uint64_t block_window_us = 1200000;
};

class MotionWatch {
public:
    explicit MotionWatch(MotionWatchConfig c = {}) : c_(c) {}
    void feed(uint64_t now_us, const std::string &state, float cmd_vx, float cmd_wz, bool output_enabled,
              bool imu_fresh, const std::array<float, 3> &acc, const std::array<float, 3> &gyro,
              const std::array<float, 3> &rpy) {
        const uint64_t dt = last_us_ && now_us > last_us_ ? now_us - last_us_ : 0;
        last_us_ = now_us;
        if (state != state_) { blk_.clear(); blocked_run_us_ = 0; was_blocked_ = false; }
        state_ = state; cmd_vx_ = cmd_vx; cmd_wz_ = cmd_wz; imu_fresh_ = imu_fresh; rpy_ = rpy;
        const bool cmd = output_enabled && (std::abs(cmd_vx) >= c_.cmd_min_mps || std::abs(cmd_wz) >= c_.cmd_min_wz_rps);
        if (imu_fresh) {
            acc_norm_ = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
            gyro_norm_ = std::sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
            win_.emplace_back(now_us, acc_norm_);
            while (!win_.empty() && now_us - win_.front().first > c_.window_us) win_.pop_front();
        } else win_.clear();
        acc_std_ = stddev();
        const bool still = imu_fresh && win_.size() >= 5 && acc_std_ < c_.still_acc_std_mps2 && gyro_norm_ < c_.still_gyro_rps;
        auto &s = per_state_[state];
        if (cmd) {
            cmd_run_us_ += dt; s.cmd_us += dt;
            still_run_us_ = still ? still_run_us_ + dt : 0;
            if (still) s.still_us += dt;
        } else { cmd_run_us_ = still_run_us_ = 0; }
        updateBlocked(now_us, dt, cmd_vx, cmd_wz, output_enabled && imu_fresh);
        const bool was = suspect_;
        suspect_ = cmd && still_run_us_ >= c_.suspect_us;
        if (suspect_) { suspect_total_us_ += dt; s.suspect_us += dt; longest_us_ = std::max(longest_us_, still_run_us_); }
        if (suspect_ && !was) events_ << "[MOTION_EVENT] begin state=" << state << " cmd_vx=" << cmd_vx << " cmd_wz=" << cmd_wz
                                      << " acc_std=" << acc_std_ << " gyro=" << gyro_norm_ << "\n";
        if (!suspect_ && was) events_ << "[MOTION_EVENT] end state=" << state << " still_ms=" << last_still_us_ / 1000 << "\n";
        last_still_us_ = still_run_us_ ? still_run_us_ : last_still_us_;
    }
    bool suspect() const { return suspect_; }
    uint64_t blockedUs() const { return blocked_run_us_; }
    float blockRatio() const { return block_ratio_; }
    uint64_t suspectTotalUs() const { return suspect_total_us_; }
    uint64_t longestUs() const { return longest_us_; }
    uint64_t stateCmdUs(const std::string &s) const { auto i = per_state_.find(s); return i == per_state_.end() ? 0 : i->second.cmd_us; }
    uint64_t stateSuspectUs(const std::string &s) const { auto i = per_state_.find(s); return i == per_state_.end() ? 0 : i->second.suspect_us; }
    std::string takeEvents() { std::string r = events_.str(); events_.str(""); return r; }
    std::string line() const {
        std::ostringstream o;
        o << "[MOTION] state=" << state_ << " imu_fresh=" << imu_fresh_ << " cmd_vx=" << cmd_vx_ << " cmd_wz=" << cmd_wz_
          << " acc_norm=" << acc_norm_ << " acc_std=" << acc_std_ << " gyro=" << gyro_norm_
          << " roll=" << rpy_[0] << " pitch=" << rpy_[1]
          << " cmd_run_ms=" << cmd_run_us_ / 1000 << " still_run_ms=" << still_run_us_ / 1000
          << " blk_ratio=" << block_ratio_ << " blocked_ms=" << blocked_run_us_ / 1000
          << " suspect=" << suspect_ << " suspect_total_ms=" << suspect_total_us_ / 1000 << " longest_ms=" << longest_us_ / 1000
          << " per_state=";
        bool first = true;
        for (const auto &kv : per_state_) {
            if (!kv.second.cmd_us) continue;
            o << (first ? "" : ",") << kv.first << ":cmd" << kv.second.cmd_us / 1000 << "/still" << kv.second.still_us / 1000
              << "/susp" << kv.second.suspect_us / 1000;
            first = false;
        }
        o << "\n";
        return o.str();
    }

private:
    struct BlkSample { uint64_t t, dt; float cmd_turn, gyro_turn; };
    void updateBlocked(uint64_t now_us, uint64_t dt, float cmd_vx, float cmd_wz, bool enabled) {
        if (!enabled || std::abs(cmd_vx) < c_.block_min_vx_mps || dt == 0 || dt > 500000) {
            blk_.clear(); blocked_run_us_ = 0; block_ratio_ = -1.f; was_blocked_ = false; return;
        }
        const float s = float(dt) * 1e-6f;
        blk_.push_back({now_us, dt, std::abs(cmd_wz) * s, gyro_norm_ * s});
        while (!blk_.empty() && now_us - blk_.front().t > c_.block_window_us) blk_.pop_front();
        double cmd = 0, gyro = 0; uint64_t span = 0;
        for (const auto &b : blk_) { cmd += b.cmd_turn; gyro += b.gyro_turn; span += b.dt; }
        block_ratio_ = cmd > 1e-6 ? float(gyro / cmd) : -1.f;
        const bool full = span >= c_.block_window_us * 3 / 4;
        const bool ok = full && cmd >= c_.block_min_turn_rad && block_ratio_ >= 0.f;
        const bool blocked = ok && block_ratio_ < (was_blocked_ ? c_.block_release_ratio : c_.block_ratio);
        if (blocked) {
            blocked_run_us_ += dt;
            if (!was_blocked_) events_ << "[MOTION_EVENT] blocked_begin state=" << state_ << " ratio=" << block_ratio_ << " cmd_turn=" << cmd << "\n";
        } else {
            if (was_blocked_) events_ << "[MOTION_EVENT] blocked_end state=" << state_ << " ms=" << blocked_run_us_ / 1000 << "\n";
            blocked_run_us_ = 0;
        }
        was_blocked_ = blocked;
    }
    struct Acc { uint64_t cmd_us = 0, still_us = 0, suspect_us = 0; };
    float stddev() const {
        if (win_.size() < 2) return 0.f;
        double m = 0, v = 0;
        for (const auto &p : win_) m += p.second;
        m /= win_.size();
        for (const auto &p : win_) v += (p.second - m) * (p.second - m);
        return float(std::sqrt(v / win_.size()));
    }
    MotionWatchConfig c_;
    uint64_t last_us_ = 0, cmd_run_us_ = 0, still_run_us_ = 0, last_still_us_ = 0, suspect_total_us_ = 0, longest_us_ = 0;
    bool suspect_ = false, imu_fresh_ = false;
    std::string state_;
    float cmd_vx_ = 0, cmd_wz_ = 0, acc_norm_ = 0, gyro_norm_ = 0, acc_std_ = 0;
    std::array<float, 3> rpy_{};
    std::deque<std::pair<uint64_t, float>> win_;
    std::deque<BlkSample> blk_;
    uint64_t blocked_run_us_ = 0;
    float block_ratio_ = -1.f;
    bool was_blocked_ = false;
    std::map<std::string, Acc> per_state_;
    std::ostringstream events_;
};

} // namespace rescue
