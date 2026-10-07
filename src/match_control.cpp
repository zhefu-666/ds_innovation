#include "rescue/match_control.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
namespace rescue {
MatchControl::MatchControl(MatchConfig c) : config_(c) {
    if (!c.evidence_timeout_us || !c.no_movement_us || !std::isfinite(c.movement_threshold_m) || c.movement_threshold_m <= 0)
        throw std::invalid_argument("Invalid match timing/movement thresholds");
}
const char* MatchControl::name(MatchState s) {
    switch(s) {
    case MatchState::WAITING:return "WAITING"; case MatchState::RUNNING:return "RUNNING";
    case MatchState::PAUSED:return "PAUSED"; case MatchState::FAULT:return "FAULT";
    case MatchState::FINISHED:return "FINISHED";
    } return "UNKNOWN";
}
bool MatchControl::ready(uint64_t now) const {
    return health_ok_ && health_us_ && now >= health_us_ && now-health_us_ <= config_.evidence_timeout_us;
}
void MatchControl::tick(uint64_t now) {
    if (!now) {
        if(status_.state != MatchState::FINISHED) {status_.state=MatchState::FAULT;status_.reason="clock_invalid";}
        status_.permit=false; return;
    }
    now=std::max(now,last_tick_us_); // callers timestamp before acquiring the shared mutex
    last_tick_us_=now;
    if(status_.started_us) {
        status_.elapsed_us=now-status_.started_us;
        status_.remaining_us=(!config_.time_limit_enabled || status_.elapsed_us>=config_.duration_us)?0:config_.duration_us-status_.elapsed_us;
        if(config_.time_limit_enabled && status_.state!=MatchState::FINISHED && !status_.remaining_us) {
            status_.state=MatchState::FINISHED;status_.reason="match_timeout";
        }
    }
    if(status_.state==MatchState::RUNNING) {
        if(!ready(now)) {status_.state=MatchState::FAULT;status_.reason=health_ok_?"health_stale":health_reason_;}
        else if(config_.require_measured_progress && now-progress_us_>=config_.no_movement_us) {
            status_.state=MatchState::FAULT;status_.reason="no_measured_movement";
        }
    }
    status_.permit=status_.state==MatchState::RUNNING;
}
void MatchControl::health(uint64_t now,bool ok,const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Do not let a late healthy frame conceal a gap or clear a latched fault.
    tick(now);
    if(now>=health_us_) {health_us_=now;health_ok_=ok;health_reason_=reason.empty()?"preflight_not_ready":reason;}
    tick(now);
}
void MatchControl::measuredPosition(uint64_t at,double x,double y,const std::string& reference,double uncertainty) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!at || at<=pose_us_ || !std::isfinite(x) || !std::isfinite(y) || reference.empty() || !std::isfinite(uncertainty) || uncertainty<0 || uncertainty>.1) return;
    if(at>health_us_ || health_us_-at>config_.evidence_timeout_us) return;
    const bool continuous=pose_valid_ && reference==reference_ && at-pose_us_<=config_.evidence_timeout_us;
    pose_us_=at; reference_=reference;
    if(!continuous) {anchor_x_=x;anchor_y_=y;anchor_uncertainty_=uncertainty;pose_valid_=true;return;}
    if(std::hypot(x-anchor_x_,y-anchor_y_)>=config_.movement_threshold_m+anchor_uncertainty_+uncertainty) {
        anchor_x_=x;anchor_y_=y;anchor_uncertainty_=uncertainty;
        if(status_.state==MatchState::RUNNING)progress_us_=at;
    }
}
bool MatchControl::command(MatchCommand c,uint64_t now) {
    std::lock_guard<std::mutex> lock(mutex_);tick(now);
    if(c==MatchCommand::FINISH) {status_.state=MatchState::FINISHED;status_.reason="operator_finish";status_.permit=false;return true;}
    if(status_.state==MatchState::FINISHED)return false;
    if(c==MatchCommand::STOP) {status_.state=MatchState::PAUSED;status_.reason="operator_stop";status_.permit=false;return true;}
    if(config_.time_limit_enabled && !config_.duration_us) {status_.reason="match_duration_required";return false;}
    if(!ready(now)) {status_.reason=health_ok_?"health_stale":health_reason_;return false;}
    if(c==MatchCommand::START && !status_.started_us && (status_.state==MatchState::WAITING || status_.state==MatchState::PAUSED)) {
        status_.started_us=now; progress_us_=now;pose_valid_=false;
        status_.state=MatchState::RUNNING;status_.reason="started";tick(now);return true;
    }
    if(c==MatchCommand::RESUME && status_.started_us &&
       (status_.state==MatchState::PAUSED || status_.state==MatchState::FAULT)) {
        // A movement watchdog expiry cannot be bypassed by repeatedly issuing RESUME.
        if(config_.require_measured_progress && now-progress_us_>=config_.no_movement_us) {status_.reason="no_movement_requires_new_session";return false;}
        status_.state=MatchState::RUNNING;status_.reason="resumed";tick(now);return status_.permit;
    }
    return false;
}
MatchStatus MatchControl::status(uint64_t now) {
    std::lock_guard<std::mutex> lock(mutex_);tick(now);return status_;
}
} // namespace rescue
