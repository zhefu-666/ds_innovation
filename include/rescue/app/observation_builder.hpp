#pragma once

#include "rescue/app/perception_pipeline.hpp"
#include "rescue/perception/capture_monitor.hpp"
#include "rescue/geometry/geometry_pipeline.hpp"
#include "rescue/hardware/hipnuc_imu.hpp"
#include "rescue/perception/multi_view_capture.hpp"
#include "rescue/perception/pixel_selector.hpp"
#include "rescue/hardware/uart_controller.hpp"
#include "rescue/perception/zone_color.hpp"

namespace rescue {
struct ResolvedRuntimeConfig;
class ControlledField;
class MotionWatch;
class SensorFusion;

namespace app {
struct ObservationContext {
    const PushTask& task;
    const PushOutput& previous;
    const UARTController* feedback;
    const HipnucImu* imu;
    const MotionWatch& motion_watch;
    bool motion_link_enabled;
};

struct ObservationResult {
    PushObservation input;
    std::vector<SegDetection> detections;
    std::vector<ZoneKeypoint> zone_points;
    ZoneColorResult zone_color;
    GeometryResult geometry_result;
    PixelSelection selection;
    FrameSensors frame_sensors;
    UARTController::GripperAck gripper_ack;
    ImuSnapshot imu_data;
    SensorState sensor;
    uint64_t observed_at = 0;
    SensorState geometry_sensor; // Same capture-time IMU used for the geometry overlay.
    uint64_t safety_checked_at_us = 0;
    std::string imu_safety_reason = "imu_missing";
};

class ObservationBuilder {
  public:
    ObservationBuilder(const ResolvedRuntimeConfig& config,
                       GeometryPipeline* geometry,
                       const CameraCalibration& vref_calibration,
                       bool vref_ready,
                       const ZoneGeometry& zone_geometry,
                       const ZoneColorClassifier& zone_colors,
                       ControlledField& controlled_field,
                       SensorFusion& fusion);
    ObservationResult build(const cv::Mat& frame,
                            const PerceptionResult& vision_result,
                            std::vector<SegDetection> detections,
                            uint64_t frame_sequence,
                            uint64_t timestamp,
                            const ObservationContext& context);
    const HoldDiagnostics& holdDiagnostics() const {
        return capture_.diagnostics();
    }

  private:
    const ResolvedRuntimeConfig& config_;
    GeometryPipeline* geometry_;
    const CameraCalibration& vref_calibration_;
    bool vref_ready_;
    const ZoneGeometry& zone_geometry_;
    const ZoneColorClassifier& zone_colors_;
    ControlledField& controlled_field_;
    SensorFusion& fusion_;
    CaptureMonitor capture_;
    PixelSelector pixel_selector_;
    uint64_t progress_ref_us_ = 0;
    int progress_total_ = -1;
    MultiViewCapture multi_view_;
    bool observing_ = false;
    uint64_t observation_round_ = 0;
};
} // namespace app
} // namespace rescue
