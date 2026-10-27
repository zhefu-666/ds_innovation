#include "rescue/app/diagnostics.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <cstdio>

namespace rescue {
void report(const PushOutput& out, std::ostream& stream) {
    stream << "state=" << rescue::PushTask::name(out.state) << " batch=" << out.batch_size
           << " delivered=" << out.delivered_total << " first=" << out.first_ordinary_delivered
           << " vx=" << out.motion.vx_mps << " wz=" << out.motion.wz_rps
           << " frame_offset_deg=" << int(out.motion.gripper_offset)
           << " startup_forward_ms=" << out.startup_commanded_us / 1000
           << " search_cue_id=" << out.search_cue_id
           << " capture_target_id=" << out.capture_target_id << " cue=" << out.target_is_search_cue
           << " clear_attempts=" << out.clearing_attempts
           << " pitch_cmd=" << out.motion.camera_pitch_cdeg
           << " camera_protocol_cmd_deg=" << rescue::cameraPitchToWire(out.motion.camera_pitch_cdeg)
           << " stop=" << (out.motion.vx_mps == 0.0f && out.motion.wz_rps == 0.0f)
           << " reason=" << (out.reason.empty() ? "-" : out.reason) << "\n";
}

void drawZone(cv::Mat& frame, const std::vector<ZoneKeypoint>& points) {
    static const char* names[] = {"FL", "FD", "FR", "RL", "RD", "RR"};
    for (const auto& k : points) {
        if (k.id < 0 || k.id > 5)
            continue;
        cv::circle(frame, k.pixel, 5, {255, 0, 255}, -1);
        cv::putText(frame,
                    names[k.id],
                    k.pixel + cv::Point2f(6, -6),
                    cv::FONT_HERSHEY_SIMPLEX,
                    .45,
                    {255, 0, 255},
                    1);
    }
}

void drawZoneProjection(cv::Mat& frame,
                        const std::vector<ZoneKeypoint>& points,
                        const CameraCalibration& calibration,
                        const ZoneGeometry& geometry,
                        int16_t pitch,
                        const SensorState& sensor,
                        float confidence,
                        bool mapping_valid,
                        const std::string& geometry_reason) {
    const cv::Scalar cyan(255, 255, 0), orange(0, 165, 255);
    const auto status = [&](const std::string& detail) {
        cv::rectangle(
            frame, {0, frame.rows - 66}, {frame.cols - 1, frame.rows - 1}, {15, 15, 15}, -1);
        cv::putText(frame,
                    "MAGENTA: model   CYAN: geometry fit   ORANGE: point error",
                    {10, frame.rows - 43},
                    cv::FONT_HERSHEY_SIMPLEX,
                    .6,
                    cyan,
                    1);
        cv::putText(frame, detail, {10, frame.rows - 17}, cv::FONT_HERSHEY_SIMPLEX, .55, orange, 1);
    };
    cv::Matx34d camera;
    if (!mapping_valid || !geometry.valid() ||
        !calibration.cameraFromGround(pitch, sensor, camera)) {
        status("Projection unavailable | strict: " + geometry_reason);
        return;
    }
    const auto objects = geometry.objectPoints();
    std::vector<cv::Point2d> zone, body;
    std::vector<ZoneKeypoint> observed;
    for (const auto& k : points) {
        if (!k.visible || k.id < 0 || k.id >= 6 || k.confidence < confidence)
            continue;
        cv::Point2f xy;
        if (!calibration.pixelToPlane(k.pixel, pitch, geometry.landmark_height_m, xy, &sensor))
            continue;
        zone.emplace_back(objects[k.id].x, objects[k.id].y);
        body.emplace_back(xy.x, xy.y);
        observed.push_back(k);
    }
    if (zone.size() < 3) {
        status("Projection needs >=3 mapped points | strict: " + geometry_reason);
        return;
    }
    // Fit only ground translation/yaw, retaining measured scale, height and camera extrinsics.
    // Use all visible points without rejecting bad ones, so inconsistencies remain visible.
    cv::Point2d zm{}, bm{};
    for (size_t i = 0; i < zone.size(); ++i) {
        zm += zone[i];
        bm += body[i];
    }
    zm *= 1.0 / zone.size();
    bm *= 1.0 / body.size();
    double dot = 0, cross = 0;
    for (size_t i = 0; i < zone.size(); ++i) {
        const auto z = zone[i] - zm, b = body[i] - bm;
        dot += z.dot(b);
        cross += z.x * b.y - z.y * b.x;
    }
    if (std::hypot(dot, cross) < 1e-10) {
        status("Projection fit degenerate | strict: " + geometry_reason);
        return;
    }
    const double yaw = std::atan2(cross, dot), c = std::cos(yaw), sn = std::sin(yaw);
    const auto transform = [&](double x, double y) {
        return cv::Point2d(bm.x + c * (x - zm.x) - sn * (y - zm.y),
                           bm.y + sn * (x - zm.x) + c * (y - zm.y));
    };
    std::vector<cv::Point3d> camera_points;
    for (const auto& o : objects) {
        const auto xy = transform(o.x, o.y);
        const auto q = camera * cv::Vec4d(xy.x, xy.y, geometry.landmark_height_m, 1);
        if (!std::isfinite(q[2]) || q[2] <= 0) {
            status("Projection behind camera | strict: " + geometry_reason);
            return;
        }
        camera_points.emplace_back(q[0], q[1], q[2]);
    }
    std::vector<cv::Point2d> projected;
    cv::projectPoints(camera_points,
                      cv::Vec3d{},
                      cv::Vec3d{},
                      calibration.cameraMatrix(),
                      calibration.distCoeffs(),
                      projected);
    for (const auto& p : projected) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || std::abs(p.x) > 100000 ||
            std::abs(p.y) > 100000) {
            status("Projection out of range | strict: " + geometry_reason);
            return;
        }
    }
    const int edges[][2] = {{0, 1}, {1, 2}, {3, 4}, {4, 5}, {0, 3}, {1, 4}, {2, 5}};
    for (const auto& edge : edges)
        cv::line(frame, projected[edge[0]], projected[edge[1]], cyan, 1, cv::LINE_AA);
    static const char* names[] = {"FL", "FD", "FR", "RL", "RD", "RR"};
    for (int i = 0; i < 6; ++i) {
        cv::drawMarker(frame, projected[i], cyan, cv::MARKER_CROSS, 16, 2);
        cv::putText(frame,
                    std::string("G-") + names[i],
                    projected[i] + cv::Point2d(6, 18),
                    cv::FONT_HERSHEY_SIMPLEX,
                    .45,
                    cyan,
                    1);
    }
    double sum = 0, maximum = 0;
    for (const auto& k : observed) {
        const double error = cv::norm(projected[k.id] - cv::Point2d(k.pixel));
        sum += error * error;
        maximum = std::max(maximum, error);
        cv::line(frame, k.pixel, projected[k.id], orange, 1, cv::LINE_AA);
    }
    char metrics[150];
    std::snprintf(metrics,
                  sizeof(metrics),
                  "DIAGNOSTIC FIT (not validation) n=%zu RMS=%.1fpx max=%.1fpx | strict: ",
                  observed.size(),
                  std::sqrt(sum / observed.size()),
                  maximum);
    status(std::string(metrics) + geometry_reason);
}

void printZone(const ZoneHalves& halves, const std::vector<ZoneKeypoint>& points) {
    static const char* half_names[] = {"zone_left", "zone_right"};
    static const char* kpt_names[] = {"far_left", "near_left", "far_right", "near_right"};
    for (int c = 0; c < 2; ++c) {
        const auto& h = halves[c];
        std::cout << "pose " << half_names[c] << " valid=" << h.valid;
        if (h.valid) {
            std::cout << " score=" << h.score << " box=" << h.box.x << "," << h.box.y << ","
                      << h.box.width << "," << h.box.height;
            for (int k = 0; k < 4; ++k)
                std::cout << " " << kpt_names[k] << "=" << h.keypoints[k].x << ","
                          << h.keypoints[k].y << "@" << h.confidence[k];
        }
        std::cout << "\n";
    }
    std::cout << "zone_keypoints=" << points.size();
    for (const auto& k : points)
        std::cout << " id" << k.id << "=" << k.pixel.x << "," << k.pixel.y << "@" << k.confidence;
    std::cout << "\n";
}
} // namespace rescue
