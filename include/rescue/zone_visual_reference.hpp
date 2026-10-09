#pragma once
// TEMP_ASSUMPTION only: coarse zone pose from whatever YOLO-pose points are visible (>=2), used as a
// visual distance reference during carry/push. It is NOT the strict trusted pose and never feeds safety gates.
#include "rescue/camera_calibration.hpp"
#include "rescue/zone_estimate.hpp"
#include "rescue/zone_geometry.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace rescue {
inline bool estimateVisualZone(const CameraCalibration &cal, const ZoneGeometry &geo,
                               const std::vector<ZoneKeypoint> &points, int16_t pitch_cdeg,
                               const SensorState *sensor, float min_confidence, ZoneEstimate &out) {
    struct P { int id; cv::Point2f b, z; };
    std::vector<P> pts;
    const auto obj = geo.objectPoints();
    for (const auto &k : points) {
        if (!k.visible || k.id < 0 || k.id > 5 || k.confidence < min_confidence) continue;
        cv::Point2f b;
        if (!cal.pixelToPlane(k.pixel, pitch_cdeg, geo.landmark_height_m, b, sensor) || b.y <= 0) continue;
        pts.push_back({k.id, b, {obj[k.id].x, obj[k.id].y}});
    }
    if (pts.size() < 2) return false;
    const auto median = [](std::vector<float> v) {
        std::sort(v.begin(), v.end());
        return v.size() % 2 ? v[v.size() / 2] : .5f * (v[v.size() / 2 - 1] + v[v.size() / 2]);
    };
    float yaw = 0, worst = 0;
    // 逐个剔除偏差最大的点（至少保留3个），避免一个远角点的透视误差让整帧作废。
    for (;;) {
        std::vector<float> lateral, depth;
        for (size_t i = 0; i < pts.size(); ++i)
            for (size_t j = i + 1; j < pts.size(); ++j) {
                const cv::Point2f dz = pts[j].z - pts[i].z, db = pts[j].b - pts[i].b;
                if (cv::norm(dz) < .25f || cv::norm(db) < .05f) continue;
                const float a = std::atan2(db.y, db.x) - std::atan2(dz.y, dz.x);
                (std::abs(dz.x) >= std::abs(dz.y) ? lateral : depth).push_back(wrapAngle(a));
            }
        const auto &yaws = lateral.empty() ? depth : lateral;
        yaw = yaws.empty() ? 0.f : median(yaws);
        const float c = std::cos(yaw), s = std::sin(yaw);
        std::vector<float> ox, oy;
        for (const auto &p : pts) {
            ox.push_back(p.b.x - (c * p.z.x - s * p.z.y));
            oy.push_back(p.b.y - (s * p.z.x + c * p.z.y));
        }
        out = {};
        out.origin_body_m = {median(ox), median(oy)};
        out.yaw_body_rad = yaw;
        worst = 0;
        size_t worst_i = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            const float d = float(std::hypot(ox[i] - out.origin_body_m.x, oy[i] - out.origin_body_m.y));
            if (d > worst) { worst = d; worst_i = i; }
        }
        if (worst <= .16f || pts.size() <= 3) break;
        pts.erase(pts.begin() + worst_i);
    }
    for (const auto &p : pts) out.inlier_ids.push_back(p.id);
    if (worst > .16f || std::abs(yaw) > 1.0f || out.origin_body_m.y < -.5f || out.origin_body_m.y > 4.f) return false;
    out.valid = true;
    out.source = ZoneEstimate::Source::MULTI_POINT;
    out.residual_m = worst;
    out.geometry_id = geo.id;
    return true;
}
} // namespace rescue
