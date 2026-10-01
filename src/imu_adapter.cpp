#include "rescue/imu_adapter.hpp"

#include <cmath>

namespace rescue {

SensorState sensorStateFromImu(const ImuSnapshot &imu) {
    SensorState state;
    const ImuSample &s = imu.sample;
    state.timestamp_us = s.received_us;
    state.roll_rad = s.body_rpy_rad[0];
    state.pitch_rad = s.body_rpy_rad[1];
    state.yaw_rad = s.body_rpy_rad[2];
    const bool finite = std::isfinite(state.roll_rad) && std::isfinite(state.pitch_rad) &&
                        std::isfinite(state.yaw_rad);
    state.imu_valid = imu.connected && imu.fresh && s.measurements_valid && s.received_us != 0 && finite;
    if (!finite) state.roll_rad = state.pitch_rad = state.yaw_rad = 0.0f;
    return state;
}

} // namespace rescue
