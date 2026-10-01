#pragma once
#include "rescue/types.hpp"
#include <deque>
#include <algorithm>
#include <cmath>

namespace rescue {
// All times are host steady-clock microseconds. Selection is causal: never use a
// sample received after the image, even if it is the latest sample at inference end.
struct FrameSensors {
    SensorState imu;
    ActuatorFeedback actuator;
    bool pitch_stable = false;
};
class PitchHistory {
public:
    void clear() { samples_.clear(); }
    void add(const ActuatorFeedback& sample) {
        if (!sample.timestamp_us || (!samples_.empty() && sample.timestamp_us < samples_.back().timestamp_us)) return;
        samples_.push_back(sample);
        while (samples_.size() > 256) samples_.pop_front();
    }
    FrameSensors at(uint64_t capture_us, uint64_t max_skew_us = 50000,
                    uint64_t stable_us = 150000, int tolerance_cdeg = 50) const {
        FrameSensors out;
        auto it = std::find_if(samples_.rbegin(), samples_.rend(),
                              [&](const auto& s){return s.timestamp_us <= capture_us;});
        if (it == samples_.rend() || !it->valid ||
            it->camera_pitch_cdeg == kCameraPitchInvalid ||
            capture_us - it->timestamp_us > max_skew_us) return out;
        out.actuator = *it;
        int low = it->camera_pitch_cdeg, high = low;
        uint64_t last = capture_us;
        for (; it != samples_.rend(); ++it) {
            if (!it->valid || it->camera_pitch_cdeg == kCameraPitchInvalid ||
                last - it->timestamp_us > max_skew_us) break;
            low = std::min(low, int(it->camera_pitch_cdeg));
            high = std::max(high, int(it->camera_pitch_cdeg));
            if (high - low > tolerance_cdeg) break;
            if (capture_us - it->timestamp_us >= stable_us) {out.pitch_stable = true; break;}
            last = it->timestamp_us;
        }
        return out;
    }
private:
    std::deque<ActuatorFeedback> samples_;
};
struct GeometryFrame {
    uint64_t frame_id = 0, capture_us = 0, now_us = 0;
    cv::Size image_size;
    FrameSensors sensors;
};
inline bool causalFresh(uint64_t sample_us, uint64_t frame_us, uint64_t tolerance_us) {
    return sample_us && frame_us >= sample_us && frame_us - sample_us <= tolerance_us;
}
} // namespace rescue
