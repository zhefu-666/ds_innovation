#pragma once
#include "rescue/geometry/camera_calibration.hpp"
#include "rescue/geometry/zone_geometry.hpp"
#include "rescue/perception/zone_keypoints.hpp"
#include "rescue/task/push_task.hpp"
#include <cmath>

namespace rescue {
// The near edge of one observed half defines its entrance frame. No divider
// fusion, other-half landmarks or whole-zone rigid fit participates here.
inline void measureSubzoneEntrance(PushObservation::SubzoneView& view,
                                   const ZoneHalf& half, const ZoneGeometry& geometry,
                                   const CameraCalibration& camera, int16_t pitch,
                                   const SensorState* sensor) {
    view.entrance_valid = false;
    if (!view.valid || !camera.valid() || !geometry.confirmed ||
        !std::isfinite(geometry.landmark_width_m) || geometry.landmark_width_m <= .1f)
        return;
    cv::Point2f left, right;
    if (!camera.pixelToPlane(half.keypoints[1], pitch, geometry.landmark_height_m, left, sensor) ||
        !camera.pixelToPlane(half.keypoints[3], pitch, geometry.landmark_height_m, right, sensor) ||
        left.y <= 0 || right.y <= 0 || cv::norm(left) > 3.f || cv::norm(right) > 3.f)
        return;
    const cv::Point2f edge = right-left;
    if (cv::norm(edge) < .10f)
        return;
    const float width = geometry.landmark_width_m*.5f;
    const float fraction = view.target_x < 0 ? (view.target_x+width)/width : view.target_x/width;
    if (fraction <= 0 || fraction >= 1)
        return;
    const float yaw = std::atan2(edge.y, edge.x);
    const cv::Point2f target = left+fraction*edge;
    view.entrance_origin_body_m = target - cv::Point2f(std::cos(yaw),std::sin(yaw))*view.target_x;
    view.entrance_yaw_rad = yaw;
    view.entrance_valid = true;
}
} // namespace rescue
