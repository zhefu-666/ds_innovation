#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace rescue {
struct ImuTranslationSample {
    uint64_t sequence = 0, received_us = 0;
    uint32_t device_time_ms = 0;
    std::array<float, 3> acceleration_mps2{}, angular_velocity_rps{};
    std::array<float, 4> quaternion_wxyz{};
    bool valid = false;
};

// Local, short-duration displacement along a fixed horizontal heading. Use every
// retained IMU sample and device-time deltas; commands never supply velocity/distance.
class ImuTranslation {
  public:
    enum class CalibrationStatus {
        Collecting, InvalidSample, AngularMotion, SampleGap, AccelerationNoise, Ready
    };
    CalibrationStatus calibrationStatus() const { return calibration_status_; }
    void reset() { *this = {}; }
    bool ready() const { return ready_; }
    float distance() const { return distance_; }
    float velocity() const { return velocity_; }
    void zeroVelocity() { velocity_ = previous_accel_ = 0; }

    // Defaults retain the capture/transport calibration limits. Startup can use
    // its post-turn noise limits without weakening sample validity or freshness.
    bool calibrate(const std::vector<ImuTranslationSample>& samples, uint64_t now, float heading,
                   float max_gyro_rps = .05f, float max_accel_std_mps2 = .05f,
                   uint64_t calibration_window_us = 250000) {
        calibration_status_ = CalibrationStatus::Collecting;
        if (!started_us_) {
            started_us_ = now;
            heading_ = heading;
        }
        for (const auto& sample : samples) {
            if (sample.received_us <= started_us_ || sample.sequence <= sequence_)
                continue;
            float acceleration = 0;
            if (!project(sample, now, acceleration))
                return rejectCalibration(CalibrationStatus::InvalidSample);
            for (float w : sample.angular_velocity_rps)
                if (!std::isfinite(w))
                    return rejectCalibration(CalibrationStatus::InvalidSample);
                else if (std::abs(w) > max_gyro_rps)
                    return rejectCalibration(CalibrationStatus::AngularMotion);
            if (sequence_ && uint32_t(sample.device_time_ms - device_ms_) > 200)
                return rejectCalibration(CalibrationStatus::SampleGap);
            sequence_ = sample.sequence;
            device_ms_ = sample.device_time_ms;
            received_us_ = sample.received_us;
            sum_ += acceleration;
            sum2_ += acceleration * acceleration;
            ++count_;
        }
        if (count_ < 10 || received_us_ < started_us_ + calibration_window_us ||
            now < received_us_ || now - received_us_ > 200000)
            return false;
        const float mean = sum_ / count_;
        if (sum2_ / count_ - mean * mean > max_accel_std_mps2 * max_accel_std_mps2)
            return rejectCalibration(CalibrationStatus::AccelerationNoise);
        bias_ = mean;
        previous_accel_ = velocity_ = distance_ = 0;
        ready_ = true;
        calibration_status_ = CalibrationStatus::Ready;
        return true;
    }

    // Consume a stationary interval without turning acceleration bias into travel.
    // Keep already measured displacement; only the velocity/baseline advances.
    bool holdStationary(const std::vector<ImuTranslationSample>& samples, uint64_t now) {
        const float retained_distance = distance_;
        if (!advance(samples, now))
            return false;
        distance_ = retained_distance;
        zeroVelocity();
        return true;
    }

    bool advance(const std::vector<ImuTranslationSample>& samples, uint64_t now) {
        if (!ready_)
            return false;
        for (const auto& sample : samples) {
            if (sample.sequence <= sequence_)
                continue;
            float acceleration = 0;
            const uint32_t delta_ms = sample.device_time_ms - device_ms_;
            if (!project(sample, now, acceleration) || delta_ms == 0 || delta_ms > 200 ||
                sample.received_us < received_us_) {
                ready_ = false;
                return false;
            }
            const float dt = delta_ms * .001f;
            acceleration -= bias_;
            const float next_velocity = velocity_ + .5f * (previous_accel_ + acceleration) * dt;
            distance_ += .5f * (velocity_ + next_velocity) * dt;
            velocity_ = next_velocity;
            previous_accel_ = acceleration;
            sequence_ = sample.sequence;
            device_ms_ = sample.device_time_ms;
            received_us_ = sample.received_us;
        }
        if (now < received_us_ || now - received_us_ > 200000) {
            ready_ = false;
            return false;
        }
        return true;
    }

  private:
    bool rejectCalibration(CalibrationStatus status) {
        reset();
        calibration_status_ = status;
        return false;
    }
    CalibrationStatus calibration_status_ = CalibrationStatus::Collecting;
    bool project(const ImuTranslationSample& s, uint64_t now, float& acceleration) const {
        if (!s.valid || !s.sequence || !s.received_us || s.received_us > now ||
            now - s.received_us > 200000)
            return false;
        float norm2 = 0;
        for (float q : s.quaternion_wxyz) {
            if (!std::isfinite(q)) return false;
            norm2 += q * q;
        }
        for (float a : s.acceleration_mps2)
            if (!std::isfinite(a)) return false;
        if (norm2 < .81f || norm2 > 1.21f) return false;
        const float norm = std::sqrt(norm2);
        const float w = s.quaternion_wxyz[0] / norm, x = s.quaternion_wxyz[1] / norm;
        const float y = s.quaternion_wxyz[2] / norm, z = s.quaternion_wxyz[3] / norm;
        const auto& a = s.acceleration_mps2;
        // Rotate body specific force into the navigation frame. Projecting onto
        // its horizontal plane removes gravity, including when the chassis tilts.
        const float ax = (1 - 2*y*y - 2*z*z)*a[0] + 2*(x*y - w*z)*a[1] + 2*(x*z + w*y)*a[2];
        const float ay = 2*(x*y + w*z)*a[0] + (1 - 2*x*x - 2*z*z)*a[1] + 2*(y*z - w*x)*a[2];
        acceleration = ax * std::cos(heading_) + ay * std::sin(heading_);
        return std::isfinite(acceleration);
    }
    bool ready_ = false;
    uint64_t started_us_ = 0, received_us_ = 0, sequence_ = 0;
    uint32_t device_ms_ = 0;
    int count_ = 0;
    float heading_ = 0, sum_ = 0, sum2_ = 0, bias_ = 0;
    float previous_accel_ = 0, velocity_ = 0, distance_ = 0;
};
} // namespace rescue
