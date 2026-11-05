#include "rescue/app/diagnostics.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <algorithm>
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

void drawPosePreview(cv::Mat& frame, const ZoneHalves& raw, const PosePreview& preview,
                     const std::string& mode) {
    const auto draw = [&](const ZoneHalves& halves, cv::Scalar color, int radius) {
        // Model slots: far_left,near_left,far_right,near_right.
        const int edges[][2]={{0,1},{0,2},{1,3},{2,3}};
        for (int c=0;c<2;++c) {
            for (const auto& edge:edges)
                if (posePreviewVisible(halves[c],edge[0],frame.size()) &&
                    posePreviewVisible(halves[c],edge[1],frame.size()))
                    cv::line(frame,halves[c].keypoints[edge[0]],halves[c].keypoints[edge[1]],
                             color,1,cv::LINE_AA);
            for (int k=0;k<4;++k) if (posePreviewVisible(halves[c],k,frame.size())) {
                const auto p=halves[c].keypoints[k];
                cv::circle(frame,p,radius,color,1,cv::LINE_AA);
                cv::putText(frame,std::to_string(c)+":"+std::to_string(k),
                            p+cv::Point2f(5,-5),cv::FONT_HERSHEY_SIMPLEX,.4,color,1);
            }
        }
    };
    if (mode=="raw" || mode=="overlay" || !preview.ok) draw(raw,{255,0,255},5);
    if (mode!="raw" && preview.ok) draw(preview.halves,{0,255,255},3);
    const std::string label=!preview.ok ? "PREVIEW ERROR / DISPLAY RAW / CONTROL RAW: "+preview.error :
        mode=="raw" ? "DISPLAY RAW / CONTROL RAW" :
                      "DISPLAY SMOOTHED / CONTROL RAW"+std::string(mode=="overlay" ? " / MAGENTA RAW" : "");
    cv::putText(frame,label,{10,25},cv::FONT_HERSHEY_SIMPLEX,.55,
                preview.ok ? cv::Scalar(0,255,255) : cv::Scalar(0,0,255),1);
}

void drawZoneProjection(cv::Mat& frame,
                        const std::vector<ZoneKeypoint>& points,
                        const CameraCalibration& calibration,
                        const ZoneGeometry& geometry,
                        int16_t pitch,
                        const SensorState& sensor,
                        float confidence,
                        bool mapping_valid,
                        const std::string& geometry_reason,
                        const ZoneEstimate& accepted,
                        const ZoneHalves& raw_halves,
                        const cv::Mat& pnp_rvec,
                        const cv::Mat& pnp_tvec) {
    const cv::Scalar cyan(255, 255, 0), orange(0, 165, 255);
    const auto status = [&](const std::string& detail) {
        cv::rectangle(
            frame, {0, frame.rows - 66}, {frame.cols - 1, frame.rows - 1}, {15, 15, 15}, -1);
        cv::putText(frame,
                    "MAGENTA: model   CYAN: accepted geometry   ORANGE X: excluded",
                    {10, frame.rows - 43},
                    cv::FONT_HERSHEY_SIMPLEX,
                    .6,
                    cyan,
                    1);
        cv::putText(frame, detail, {10, frame.rows - 17}, cv::FONT_HERSHEY_SIMPLEX, .55, orange, 1);
    };
    const auto inlier = [&](int id) {
        return accepted.valid && accepted.source != ZoneEstimate::Source::PREDICTED &&
            std::find(accepted.inlier_ids.begin(), accepted.inlier_ids.end(), id) !=
                accepted.inlier_ids.end();
    };
    const int ids[2][4] = {{3, 0, 4, 1}, {4, 1, 5, 2}};
    int excluded = 0;
    for (int h = 0; h < 2; ++h) {
        if (!raw_halves[h].valid) continue;
        for (int k = 0; k < 4; ++k) {
            const auto p = raw_halves[h].keypoints[k];
            if (raw_halves[h].confidence[k] < confidence ||
                !std::isfinite(p.x) || !std::isfinite(p.y)) continue;
            if (!inlier(ids[h][k])) {
                ++excluded;
                const cv::Point2f shown(std::clamp(p.x, 4.f, float(frame.cols - 5)),
                                        std::clamp(p.y, 4.f, float(frame.rows - 70)));
                cv::drawMarker(frame, shown, orange, cv::MARKER_TILTED_CROSS, 14, 2);
            }
        }
    }
    if (!accepted.valid || accepted.source == ZoneEstimate::Source::PREDICTED ||
        accepted.observed_us != accepted.timestamp_us) {
        status("No accepted current-frame geometry | " + geometry_reason +
               " | excluded=" + std::to_string(excluded));
        return;
    }
    cv::Matx34d camera;
    if (!mapping_valid || !geometry.valid() ||
        !calibration.cameraFromGround(pitch, sensor, camera)) {
        status("Projection unavailable | strict: " + geometry_reason);
        return;
    }
    const bool pnp = !pnp_rvec.empty() && !pnp_tvec.empty();
    cv::Mat pnp_rotation;
    if (pnp) cv::Rodrigues(pnp_rvec,pnp_rotation);
    const auto objects = geometry.objectPoints();
    std::vector<ZoneKeypoint> observed;
    for (const auto& k : points)
        if (inlier(k.id))
            observed.push_back(k);
    const auto transform = [&](double x, double y) {
        const auto p = accepted.zoneToBody({float(x), float(y)});
        return cv::Point2d(p.x, p.y);
    };
    std::vector<cv::Point3d> camera_points;
    for (const auto& o : objects) {
        const auto xy = transform(o.x, o.y);
        auto q = camera * cv::Vec4d(xy.x, xy.y, geometry.landmark_height_m, 1);
        if (pnp) {
            const cv::Mat v=pnp_rotation*(cv::Mat_<double>(3,1)<<o.x,o.y,o.z)+pnp_tvec;
            q={v.at<double>(0),v.at<double>(1),v.at<double>(2)};
        }
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
                  "ACCEPTED geometry n=%zu RMS=%.1fpx max=%.1fpx | ",
                  observed.size(),
                  observed.empty() ? 0. : std::sqrt(sum / observed.size()),
                  maximum);
    status(std::string(metrics) + (pnp ? "PnP | " : "ground | ") + geometry_reason);
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
