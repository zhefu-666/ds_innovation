#include "rescue/geometry/geometry_pipeline.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <set>
namespace rescue {
namespace {
std::vector<cv::Point3f> cuboid(float x, float y, float z) {
    std::vector<cv::Point3f> v;
    for (float a : {-x / 2, x / 2})
        for (float b : {-y / 2, y / 2})
            for (float c : {0.f, z})
                v.emplace_back(a, b, c);
    return v;
}
} // namespace
bool classSolids(const std::string& label, std::vector<std::vector<cv::Point3f>>& poses) {
    poses.clear();
    if (label == "ordinary_supply" || label == "dangerous_object")
        poses = {cuboid(.04f, .04f, .04f)}; // 40 mm cube
    else if (label == "injured_person")
        poses = {cuboid(.08f, .04f, .04f)}; // field injured blocks always lie on the floor
    else if (label == "core_supply") {
        // Regular tetrahedron, 40 mm edge, resting on a face (all faces are equal).
        const float e = .04f, r = e / std::sqrt(3.f);
        poses = {
            {{-e / 2, -r / 2, 0}, {e / 2, -r / 2, 0}, {0, r, 0}, {0, 0, e * std::sqrt(2.f / 3.f)}}};
    }
    return !poses.empty();
}
GeometryPipeline::GeometryPipeline(CameraCalibration c, ZoneGeometry g, GroundContactConfig contact)
    : calibration_(std::move(c)), geometry_(std::move(g)),
      pnp_(calibration_.cameraMatrix(), calibration_.distCoeffs()), contact_(contact) {
    if (!calibration_.valid() || !geometry_.valid())
        throw std::runtime_error("Geometry requires calibrated camera and confirmed zone");
    if (!std::isfinite(contact_.min_confidence) || !std::isfinite(contact_.max_range_m) ||
        contact_.max_range_m <= 0 || contact_.edge_margin_px < 0 ||
        !std::isfinite(contact_.size_tolerance) || contact_.size_tolerance < 0 ||
        contact_.size_tolerance >= 1)
        throw std::runtime_error("Invalid ground contact configuration");
}
std::string GeometryPipeline::groundContact(SegDetection& d, const GeometryFrame& f) {
    // Zone rectangles are surface sightings, not foreground ground contacts.
    if (d.label == "blue_safe_zone" || d.label == "red_safe_zone")
        return "zone_detection";
    if (d.timestamp_us != f.capture_us || d.frame_id != f.frame_id)
        return "frame_mismatch";
    if (d.box.width <= 0 || d.box.height <= 0 || d.box.x < 0 || d.box.y < 0 ||
        double(d.box.x) + d.box.width > f.image_size.width ||
        double(d.box.y) + d.box.height >= f.image_size.height)
        return "box_outside_image";
    const auto pitch = f.sensors.actuator.camera_pitch_cdeg;
    if (!calibration_.detectionToBody(d, pitch, f.sensors.imu))
        return "not_projected";
    if (d.body_xy_m.y <= 0 || cv::norm(d.body_xy_m) > contact_.max_range_m) {
        d.ground_position_valid = false;
        return "out_of_range";
    }
    // The position is a box-bottom estimate from here on; contact needs the checks below.
    std::vector<std::vector<cv::Point3f>> poses;
    if (!classSolids(d.label, poses))
        return "unknown_label";
    if (!std::isfinite(d.confidence) || d.confidence < (d.label == "core_supply" && f.sensors.actuator.valid && f.sensors.pitch_stable && std::abs(int(pitch) - 2000) <= 100 ? std::min(contact_.min_confidence, contact_.ui140_core_confidence) : contact_.min_confidence))
        return "low_confidence";
    const int m = contact_.edge_margin_px;
    if (d.box.x < m || d.box.y < m || d.box.x + d.box.width > f.image_size.width - m ||
        d.box.y + d.box.height > f.image_size.height - m)
        return "box_at_image_edge"; // truncated silhouette
    cv::Matx34d T;
    if (!calibration_.cameraFromGround(pitch, f.sensors.imu, T))
        return "no_extrinsic_model";
    // Observed box in the undistorted pixel domain of the model.
    const cv::Rect2f b(d.box);
    std::vector<cv::Point2f> edge{b.tl(),
                                  {b.x + b.width, b.y},
                                  {b.x, b.y + b.height},
                                  b.br(),
                                  {b.x + b.width / 2, b.y},
                                  {b.x + b.width / 2, b.y + b.height},
                                  {b.x, b.y + b.height / 2},
                                  {b.x + b.width, b.y + b.height / 2}};
    if (!calibration_.distCoeffs().empty())
        cv::undistortPoints(edge,
                            edge,
                            calibration_.cameraMatrix(),
                            calibration_.distCoeffs(),
                            cv::noArray(),
                            calibration_.cameraMatrix());
    float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f;
    for (const auto& q : edge) {
        x0 = std::min(x0, q.x);
        x1 = std::max(x1, q.x);
        y0 = std::min(y0, q.y);
        y1 = std::max(y1, q.y);
    }
    // The box bottom is the near edge of the footprint; centre the solid ~half a block behind it.
    const cv::Point2f centre = d.body_xy_m + d.body_xy_m * (.02f / float(cv::norm(d.body_xy_m)));
    cv::Mat k64;
    calibration_.cameraMatrix().convertTo(k64, CV_64F);
    const cv::Matx33d K(k64);
    float wmin = 1e9f, wmax = 0, hmin = 1e9f, hmax = 0;
    for (size_t pi = 0; pi < poses.size(); ++pi)
        for (int deg = 0; deg < 180; deg += 10) {
            const auto& pose = poses[pi];
            const float c = std::cos(deg * float(CV_PI) / 180),
                        s = std::sin(deg * float(CV_PI) / 180);
            float u0 = 1e9f, u1 = -1e9f, v0 = 1e9f, v1 = -1e9f;
            for (const auto& v : pose) {
                const cv::Vec3d q =
                    K *
                    (T *
                     cv::Vec4d(centre.x + c * v.x - s * v.y, centre.y + s * v.x + c * v.y, v.z, 1));
                if (!(q[2] > 1e-6))
                    return "footprint_not_projected";
                u0 = std::min(u0, float(q[0] / q[2]));
                u1 = std::max(u1, float(q[0] / q[2]));
                v0 = std::min(v0, float(q[1] / q[2]));
                v1 = std::max(v1, float(q[1] / q[2]));
            }
            wmin = std::min(wmin, u1 - u0);
            wmax = std::max(wmax, u1 - u0);
            hmin = std::min(hmin, v1 - v0);
            hmax = std::max(hmax, v1 - v0);
        }
    const float tol = contact_.size_tolerance, w = x1 - x0, h = y1 - y0;
    if (!(w >= wmin * (1 - tol) && w <= wmax * (1 + tol) && h >= hmin * (1 - tol) &&
          h <= hmax * (1 + tol)))
        return "size_mismatch";
    d.ground_contact_valid = true;
    return {};
}
void GeometryPipeline::estimateZonePnp(GeometryResult& out,
                                       const GeometryFrame& f,
                                       const KeypointFrame& k) {
    out.pnp_primary = true;
    auto& z = out.zone;
    z.frame_id=f.frame_id; z.timestamp_us=z.observed_us=f.capture_us;
    z.zone_label=k.zone_label; z.geometry_id=k.geometry_id;
    const auto fail = [&](const char* reason) {
        z.valid=false; z.reason=out.reason=out.pnp_reason=reason;
    };
    if (k.frame_id!=f.frame_id || k.capture_us!=f.capture_us ||
        k.image_size!=f.image_size) { fail("keypoint_frame_mismatch"); return; }
    if (k.zone_label!=geometry_.label || k.geometry_id!=geometry_.id) {
        fail("keypoint_schema_mismatch"); return;
    }
    cv::Matx34d camera;
    if (!calibration_.cameraFromGround(f.sensors.actuator.camera_pitch_cdeg,
                                       f.sensors.imu,camera)) {
        fail("camera_transform_unavailable"); return;
    }
    const auto objects=geometry_.objectPoints();
    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> img;
    std::vector<int> ids;
    std::set<int> seen;
    for (const auto& p:k.points) {
        if (p.id<0 || p.id>=6 || !seen.insert(p.id).second) {
            fail("invalid_or_duplicate_id"); return;
        }
        if (!p.visible || !std::isfinite(p.confidence) || p.confidence<.60f ||
            p.confidence>1 || !std::isfinite(p.pixel.x) || !std::isfinite(p.pixel.y) ||
            p.pixel.x<=2 || p.pixel.y<=2 || p.pixel.x>=f.image_size.width-2 ||
            p.pixel.y>=f.image_size.height-2) continue;
        obj.push_back(objects[p.id]); img.push_back(p.pixel); ids.push_back(p.id);
    }
    if (obj.size()<4) { fail("pnp_insufficient_points"); return; }
    out.pnp_attempted=true;
    // At most six landmarks: deterministic four-point hypotheses, evaluated in pixels.
    // No ray/plane intersection or metric spacing filter is used by this path.
    const cv::Mat rc(camera.get_minor<3,3>(0,0));
    cv::Mat normal=rc.col(2).clone();
    std::vector<int> best;
    double best_rms=1e30;
    for (size_t a=0;a+3<obj.size();++a)
    for (size_t b=a+1;b+2<obj.size();++b)
    for (size_t c=b+1;c+1<obj.size();++c)
    for (size_t d=c+1;d<obj.size();++d) {
        std::vector<cv::Point3f> sample{obj[a],obj[b],obj[c],obj[d]};
        std::vector<cv::Point2f> pixels{img[a],img[b],img[c],img[d]};
        std::vector<cv::Mat> rv,tv;
        try {
            cv::solvePnPGeneric(sample,pixels,calibration_.cameraMatrix(),
                calibration_.distCoeffs(),rv,tv,false,cv::SOLVEPNP_IPPE);
        } catch (const cv::Exception&) { continue; }
        for (size_t n=0;n<rv.size();++n) {
            if (!cv::checkRange(rv[n]) || !cv::checkRange(tv[n])) continue;
            cv::Mat r; cv::Rodrigues(rv[n],r);
            if (r.col(2).dot(normal)<std::cos(25*CV_PI/180)) continue;
            std::vector<cv::Point2f> projected;
            cv::projectPoints(obj,rv[n],tv[n],calibration_.cameraMatrix(),
                              calibration_.distCoeffs(),projected);
            std::vector<int> inliers;
            double error=0;
            for (size_t i=0;i<img.size();++i) {
                const double depth=r.at<double>(2,0)*obj[i].x+
                    r.at<double>(2,1)*obj[i].y+tv[n].at<double>(2);
                const double e=cv::norm(projected[i]-img[i]);
                if (depth>0 && std::isfinite(e) && e<=4.) {
                    inliers.push_back(int(i)); error+=e*e;
                }
            }
            const double rms=inliers.empty()?1e30:std::sqrt(error/inliers.size());
            if (inliers.size()>best.size() ||
                (inliers.size()==best.size() && rms<best_rms)) {
                best=inliers; best_rms=rms;
            }
        }
    }
    if (best.size()<std::max(size_t(4),obj.size()-1)) {
        fail("pnp_no_consensus"); return;
    }
    SafeZoneObservation observation;
    observation.label=k.zone_label; observation.expected_normal_camera=normal;
    for (int i:best) {
        observation.object_points.push_back(obj[i]);
        observation.image_points.push_back(img[i]);
    }
    PnpDiagnostics diag;
    out.pnp_attempted=true;
    auto pose=pnp_.estimate(observation,f.capture_us,&diag);
    out.pnp_solver_reason=pose.reason;
    out.pnp_reprojection_error_px=diag.reprojection_error_px;
    out.pnp_solution_valid=pose.valid;
    if (!pose.valid) { fail(pose.reason.c_str()); return; }
    cv::Mat r; cv::Rodrigues(pose.rvec,r);
    const cv::Mat tc=(cv::Mat_<double>(3,1)<<camera(0,3),camera(1,3),camera(2,3));
    const cv::Mat body_t=rc.t()*(pose.tvec-tc), body_r=rc.t()*r;
    const double height=body_t.at<double>(2);
    z.origin_body_m={float(body_t.at<double>(0)),float(body_t.at<double>(1))};
    z.yaw_body_rad=std::atan2(body_r.at<double>(1,0),body_r.at<double>(0,0));
    if (!cv::checkRange(body_t) || !cv::checkRange(body_r) ||
        z.origin_body_m.y<=0 || cv::norm(z.origin_body_m)>4 ||
        std::abs(height-geometry_.landmark_height_m)>.10) {
        fail("pnp_body_pose_rejected"); return;
    }
    for (int i:best) z.inlier_ids.push_back(ids[i]);
    // Pixel residual propagated to metres; the existing uncertainty gate still applies.
    const double fx=calibration_.cameraMatrix().at<double>(0,0);
    const double sigma=std::max(1.5,double(pose.reprojection_error_px))*pose.distance_m/fx;
    z.residual_m=pose.reprojection_error_px*pose.distance_m/fx;
    z.position_sigma_m=std::max(.003,double(sigma));
    double span=0;
    for (const auto& a:observation.image_points)
        for (const auto& b:observation.image_points) span=std::max(span,cv::norm(a-b));
    z.yaw_sigma_rad=std::max(.005,2*std::max(1.5,double(pose.reprojection_error_px))/std::max(1.,span));
    z.source=ZoneEstimate::Source::MULTI_POINT;
    z.valid=true; z.reason=out.reason=out.pnp_reason="pnp_primary_ok";
    if (!z.trusted(f.now_us)) {
        fail("pnp_pose_quality_rejected"); return;
    }
    out.zone_camera_rvec=pose.rvec.clone(); out.zone_camera_tvec=pose.tvec.clone();
    out.identity_verified=k.identity_verified;
}
GeometryResult GeometryPipeline::process(const GeometryFrame& f,
                                         const KeypointFrame& k,
                                         const std::vector<SegDetection>& detections) {
    GeometryResult out;
    out.detections = detections;
    out.pnp_primary = calibration_.zonePnpPrimaryEnabled();
    out.reason = filter_.frameRejection(f, calibration_);
    // Inputs never carry contact or position in: both are produced here or stay invalid.
    for (auto& d : out.detections) {
        d.ground_position_valid = d.ground_contact_valid = false;
        d.ground_contact_reason = out.reason;
    }
    if (!out.reason.empty()) {
        if (out.pnp_primary) { pnp_.reset(); out.zone.reason=out.reason; return out; }
        FilteredKeypoints invalid;
        invalid.reason = out.reason;
        out.zone = fitter_.fit(invalid, k, f);
        return out;
    }
    out.mapping_valid = true;
    for (auto& d : out.detections)
        d.ground_contact_reason = groundContact(d, f);
    if (out.pnp_primary) { estimateZonePnp(out,f,k); return out; }
    const auto filtered = filter_.filter(k, f, geometry_, calibration_);
    out.zone = fitter_.fit(filtered, k, f);
    out.reason = out.zone.reason;
    out.identity_verified = k.identity_verified && out.zone.valid && k.frame_id == f.frame_id &&
                            k.capture_us == f.capture_us;
    // IPPE is optional and only cross-checks an already accepted metric fit.
    // Two-point fits and fixed-H files without extrinsics do not require PnP.
    if (out.zone.valid)
        out.pnp_reason = out.zone.inlier_ids.size() < 4 ? "insufficient_inliers"
                                                        : "camera_transform_unavailable";
    if (!calibration_.pnpCrosscheckEnabled()) out.pnp_reason = "disabled_by_calibration";
    cv::Matx34d camera_from_ground;
    if (calibration_.pnpCrosscheckEnabled() && out.zone.valid && out.zone.inlier_ids.size() >= 4 &&
        calibration_.cameraFromGround(
            f.sensors.actuator.camera_pitch_cdeg, f.sensors.imu, camera_from_ground)) {
        const float c = std::cos(out.zone.yaw_body_rad), s = std::sin(out.zone.yaw_body_rad);
        const cv::Matx33d rz(c, -s, 0, s, c, 0, 0, 0, 1),
            rc = camera_from_ground.get_minor<3, 3>(0, 0);
        const cv::Vec3d tc(
            camera_from_ground(0, 3), camera_from_ground(1, 3), camera_from_ground(2, 3));
        SafeZoneObservation o;
        o.label = geometry_.label;
        cv::Rodrigues(cv::Mat(rc * rz), o.reference_rvec);
        // Object points are planar in the landmark plane, lifted landmark_height_m above the floor.
        o.reference_tvec = cv::Mat(rc * cv::Vec3d(out.zone.origin_body_m.x,
                                                  out.zone.origin_body_m.y,
                                                  geometry_.landmark_height_m) +
                                   tc)
                               .clone();
        const auto objects = geometry_.objectPoints();
        for (const auto& point : filtered.points)
            if (std::find(out.zone.inlier_ids.begin(), out.zone.inlier_ids.end(), point.id) !=
                out.zone.inlier_ids.end()) {
                o.object_points.push_back(objects[point.id]);
                o.image_points.push_back(point.pixel);
            }
        PnpDiagnostics diagnostics;
        out.pnp_attempted = true;
        auto check = pnp_.estimate(o, f.capture_us, &diagnostics);
        out.pnp_solution_valid = check.valid;
        out.pnp_solver_reason = out.pnp_reason = check.reason;
        out.pnp_reprojection_error_px = diagnostics.reprojection_error_px;
        // On failure these deltas describe the diagnostic candidate, not an accepted solution.
        if (!diagnostics.rvec.empty() && !diagnostics.tvec.empty()) {
            cv::Mat expected, actual;
            cv::Rodrigues(o.reference_rvec, expected);
            cv::Rodrigues(diagnostics.rvec, actual);
            out.pnp_position_delta_m = cv::norm(diagnostics.tvec - o.reference_tvec);
            out.pnp_angle_delta_deg =
                std::acos(std::clamp((cv::trace(expected.t() * actual)[0] - 1) / 2, -1.0, 1.0)) *
                180 / CV_PI;
            out.pnp_delta_valid = true;
        }
        if (check.valid) {
            cv::Mat expected, actual;
            cv::Rodrigues(o.reference_rvec, expected);
            cv::Rodrigues(check.rvec, actual);
            const double angle =
                std::acos(std::clamp((cv::trace(expected.t() * actual)[0] - 1) / 2, -1.0, 1.0));
            out.zone.pnp_checked = true;
            out.zone.pnp_consistent =
                cv::norm(check.tvec - o.reference_tvec) <= .08 && angle <= .20;
            const bool position_ok = out.pnp_position_delta_m <= .08;
            const bool angle_ok = angle <= .20;
            out.pnp_reason = position_ok && angle_ok
                                 ? "ok"
                                 : (!position_ok && !angle_ok
                                        ? "position_and_angle_mismatch"
                                        : (!position_ok ? "position_mismatch" : "angle_mismatch"));
            if (!out.zone.pnp_consistent) {
                out.zone.valid = false;
                out.zone.reason = out.reason = "pnp_disagreement";
                fitter_.reset();
            }
        } else if (check.reason == "ippe_reprojection_rejected") {
            out.zone.pnp_checked = true;
            out.zone.pnp_consistent = false;
            out.zone.valid = false;
            out.zone.reason = out.reason = "pnp_disagreement";
            fitter_.reset();
        }
    }
    return out;
}
void GeometryPipeline::apply(PushObservation& in,
                             const GeometryResult& r,
                             const ExpectedStop& stop,
                             const std::string& team,
                             uint64_t now) const {
    in.target_region_valid = false;
    in.target_in_zone = false;
    in.delivery_observed = false;
    in.target_center_valid = false;
    in.zone_identity_verified = r.identity_verified;
    in.geometry_valid = false;
    in.zone_valid = false;
    in.zone_own = false;
    in.zone_class.clear();
    in.zone_estimate = r.zone;
    for (const auto& d : r.detections)
        if (in.target_valid && d.track_id == in.target_id && d.ground_position_valid &&
            now >= d.timestamp_us && now - d.timestamp_us <= 200000) {
            in.geometry_valid =
                d.ground_contact_valid ||
                (contact_.accept_size_mismatch && d.ground_contact_reason == "size_mismatch");
            in.distance_m = cv::norm(d.body_xy_m);
            in.heading_error = std::atan2(-d.body_xy_m.x, d.body_xy_m.y); // positive left/CCW
            in.target_center_valid = d.body_center_valid;
            if (d.body_center_valid)
                in.target_center_body = d.body_center_xy_m;
        }
    in.zone_valid = r.zone.trusted(now);
    in.zone_own =
        in.zone_identity_verified && in.zone_valid && r.zone.zone_label == team + "_safe_zone";
    // Only a trusted, independently identified own zone can classify the target.
    // Use conservative footprint + pose uncertainty; touching/ambiguous stays unknown.
    if (in.zone_own)
        for (const auto& d : r.detections)
            if (d.track_id == in.target_id && in.geometry_valid) {
                const auto q = r.zone.bodyToZone(d.body_xy_m);
                const float radius = (d.label == "injured_person" ? .065f : .05f) +
                                     2 * r.zone.position_sigma_m +
                                     2 * cv::norm(q) * r.zone.yaw_sigma_rad;
                const bool inside = std::abs(q.x) < geometry_.width_m / 2 - radius &&
                                    q.y > radius && q.y < geometry_.depth_m - radius;
                const bool outside = std::abs(q.x) > geometry_.width_m / 2 + radius ||
                                     q.y < -radius || q.y > geometry_.depth_m + radius;
                in.target_region_valid = inside || outside;
                in.target_in_zone = inside;
                // Delivery is a direct local observation of the carried target entering our
                // independently identified zone. It does not depend on inventory counts.
                const bool supply = d.label == "ordinary_supply" || d.label == "core_supply";
                const bool injured = d.label == "injured_person";
                const bool left = supply ? geometry_.supply_left : !geometry_.supply_left;
                const bool correct_half =
                    (supply || injured) &&
                    (left ? q.x < -geometry_.divider_exclusion_half_width_m - radius
                          : q.x > geometry_.divider_exclusion_half_width_m + radius);
                if (inside && correct_half && in.zone_own)
                    in.delivery_observed = true;
            }
    if (!in.zone_identity_verified || !stop.valid || !in.target_valid ||
        stop.target_id != in.target_id || stop.frame_id != r.zone.frame_id ||
        stop.capture_us != r.zone.timestamp_us)
        return;
    const auto decision = classifyExpectedStop(geometry_, r.zone, stop.body_m, stop.radius_m, now);
    if (decision.valid)
        in.zone_class = decision.zone_class;
    // path_safe, safety_ok, capture and zone-count evidence are not fabricated.
}
uint64_t readExactTime(const cv::FileNode& n) {
    if (n.empty() || (!n.isReal() && !n.isInt()))
        throw std::runtime_error("Missing numeric frame/time");
    double value = double(n);
    if (!std::isfinite(value) || value <= 0 || value > 9007199254740991.0 ||
        std::floor(value) != value)
        throw std::runtime_error("Frame/time must be a positive exact integer");
    return static_cast<uint64_t>(value);
}
KeypointFrame readKeypointFrame(const cv::FileNode& n) {
    KeypointFrame f;
    f.identity_verified = !n["identity_verified"].empty() && int(n["identity_verified"]) == 1;
    f.frame_id = readExactTime(n["frame_id"]);
    f.capture_us = readExactTime(n["capture_us"]);
    f.zone_label = std::string(n["zone_label"]);
    f.geometry_id = std::string(n["geometry_id"]);
    f.image_size = {int(n["image_width"]), int(n["image_height"])};
    if (!n["points"].isSeq() || n["points"].size() > 6)
        throw std::runtime_error("Expected at most six keypoints");
    for (const auto& p : n["points"]) {
        if (!p["id"].isInt() || !p["pixel"].isSeq() || p["pixel"].size() != 2 ||
            p["confidence"].empty() || p["visible"].empty())
            throw std::runtime_error("Invalid keypoint record");
        f.points.push_back({int(p["id"]),
                            {float(p["pixel"][0]), float(p["pixel"][1])},
                            float(p["confidence"]),
                            int(p["visible"]) == 1});
    }
    return f;
}
ExpectedStop readExpectedStop(const cv::FileNode& node, const KeypointFrame& f) {
    ExpectedStop out;
    const auto n = node["expected_stop"];
    if (n.empty())
        return out;
    if (!n["body_m"].isSeq() || n["body_m"].size() != 2 || n["radius_m"].empty() ||
        !n["target_id"].isInt())
        throw std::runtime_error("Invalid expected stop");
    out.valid = true;
    out.frame_id = f.frame_id;
    out.capture_us = f.capture_us;
    out.target_id = int(n["target_id"]);
    out.body_m = {float(n["body_m"][0]), float(n["body_m"][1])};
    out.radius_m = float(n["radius_m"]);
    return out;
}
} // namespace rescue
