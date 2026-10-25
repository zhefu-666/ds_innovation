#pragma once
#include <cstdint>
#include <mutex>
#include <string>

namespace rescue {
enum class MatchState { WAITING, RUNNING, PAUSED, FAULT, FINISHED };
enum class MatchCommand { START, STOP, RESUME, FINISH };
struct MatchConfig {
    uint64_t duration_us = 0; // Required explicitly; do not guess the competition duration.
    uint64_t evidence_timeout_us = 200000, no_movement_us = 15000000;
    double movement_threshold_m = .02; // Translation, not motor commands or IMU yaw.
    bool time_limit_enabled = true;
    bool require_measured_progress = true; // only controlled local-vision tests disable this
};
struct MatchStatus {
    MatchState state = MatchState::WAITING;
    uint64_t started_us = 0, elapsed_us = 0, remaining_us = 0;
    bool permit = false;
    std::string reason = "waiting_start";
};
// Thread safe: the serial sender checks the same gate independently of inference.
// STOP/health failures latch. RESUME is explicit and never restarts the match clock.
class MatchControl {
public:
    explicit MatchControl(MatchConfig config);
    void health(uint64_t now, bool ready, const std::string& reason);
    // A measured pose in one fixed reference frame. Changing reference/invalid samples
    // establishes a new baseline, never counts as motion. No command integration here.
    void measuredPosition(uint64_t sample_us, double x, double y, const std::string& reference, double uncertainty_m);
    bool command(MatchCommand command, uint64_t now);
    MatchStatus status(uint64_t now);
    static const char* name(MatchState state);
private:
    void tick(uint64_t now);
    bool ready(uint64_t now) const;
    MatchConfig config_;
    std::mutex mutex_;
    MatchStatus status_;
    uint64_t health_us_ = 0, last_tick_us_ = 0, progress_us_ = 0, pose_us_ = 0;
    bool health_ok_ = false, pose_valid_ = false;
    double anchor_x_ = 0, anchor_y_ = 0, anchor_uncertainty_ = 0;
    std::string health_reason_ = "preflight_pending", reference_;
};
} // namespace rescue
