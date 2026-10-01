#pragma once

#include "rescue/hipnuc_imu.hpp"
#include "rescue/types.hpp"

namespace rescue {

// HI91接收快照 → 业务SensorState，只填姿态；ToF/编码器保持无效。
// 姿态取车体坐标body_rpy_rad（安装变换已于2026-09-27实车标定），不使用设备坐标rpy_rad。
// imu_valid：串口已连接、数据新鲜、数值检查通过且车体姿态有限；
// 角度是否在可信范围内由使用方判断（SensorFusion::tiltUnsafe、CameraCalibration::tiltTrusted）。
// timestamp_us取收帧时的主机单调时钟，与imuNowUs()、utils.hpp的Clock同为steady_clock。
SensorState sensorStateFromImu(const ImuSnapshot &imu);

} // namespace rescue
