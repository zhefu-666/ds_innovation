#include "rescue/perception/pose_temporal_smoother.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace rescue {
namespace {
double alpha(double f, double dt) {
    return -std::expm1(-2 * CV_PI * f * dt);
}
double iou(const cv::Rect2f& a, const cv::Rect2f& b) {
    const double w = std::max(0.,
                              std::min(double(a.x) + a.width, double(b.x) + b.width) -
                                  std::max(double(a.x), double(b.x)));
    const double h = std::max(0.,
                              std::min(double(a.y) + a.height, double(b.y) + b.height) -
                                  std::max(double(a.y), double(b.y)));
    return w * h / std::max(1e-9, double(a.area()) + b.area() - w * h);
}
bool finite(const cv::Rect2f& b) {
    return std::isfinite(b.x) && std::isfinite(b.y) && std::isfinite(b.width) &&
           std::isfinite(b.height) && b.width > 0 && b.height > 0;
}
} // namespace
bool posePreviewVisible(const ZoneHalf& h, int k, cv::Size s) {
    const auto p = h.keypoints[k];
    return h.valid && std::isfinite(h.score) && h.score >= PoseSmoothingParameters::detection &&
           std::isfinite(h.confidence[k]) &&
           h.confidence[k] >= PoseSmoothingParameters::visibility && std::isfinite(p.x) &&
           std::isfinite(p.y) && p.x >= 0 && p.y >= 0 && p.x < s.width && p.y < s.height;
}
PosePreview rawPosePreview(const ZoneHalves& raw, cv::Size size, uint64_t id, uint64_t t) {
    PosePreview out;
    out.halves = raw;
    out.frame_id = id;
    out.measurement_us = t;
    for (int c = 0; c < 2; ++c)
        for (int k = 0; k < 4; ++k) {
            out.visible[c][k] = posePreviewVisible(raw[c], k, size);
            out.status[c][k] = out.visible[c][k] ? "raw" : "hidden";
        }
    return out;
}
void PoseTemporalSmoother::reset() {
    points_ = {};
    boxes_ = {};
    box_valid_ = {};
    have_previous_ = false;
    last_time_ = last_id_ = 0;
    context_.clear();
    signature_.clear();
    cached_ = {};
}
PosePreview PoseTemporalSmoother::update(const ZoneHalves& raw,
                                         const ZonePoseMetadata& meta,
                                         const ModelImage& image,
                                         cv::Size size,
                                         uint64_t t,
                                         PoseFrameIdentity id) {
    if (!id.frame_id)
        id.frame_id = t;
    auto result = rawPosePreview(raw, size, id.frame_id, t);
    try {
        if (size.width <= 0 || size.height <= 0 || !std::isfinite(image.sx) ||
            !std::isfinite(image.sy) || image.sx <= 0 || image.sy <= 0)
            throw std::runtime_error("invalid preview letterbox/size");
        std::ostringstream ctx, sig;
        ctx << std::quoted(id.source) << ' ' << std::quoted(id.model) << ' ' << size.width << ' '
            << size.height << ' ' << id.pitch_valid << ' ' << (id.pitch_valid ? id.pitch_cdeg : 0);
        sig << std::setprecision(std::numeric_limits<double>::max_digits10) << t << ' ' << image.sx
            << ' ' << image.sy << ' ' << image.pad_x << ' ' << image.pad_y;
        for (int c = 0; c < 2; ++c) {
            const auto& h = raw[c];
            const auto& b = meta[c].unclipped_box;
            sig << ' ' << h.valid << ' ' << h.score << ' ' << h.box.x << ' ' << h.box.y << ' '
                << h.box.width << ' ' << h.box.height << ' ' << meta[c].nms_candidates << ' ' << b.x
                << ' ' << b.y << ' ' << b.width << ' ' << b.height;
            if (h.valid && (!std::isfinite(h.score) || !finite(b) || meta[c].nms_candidates < 1))
                throw std::runtime_error("invalid preview score/box/NMS metadata");
            for (int k = 0; k < 4; ++k) {
                const auto p = h.keypoints[k];
                sig << ' ' << p.x << ' ' << p.y << ' ' << h.confidence[k];
                if (h.valid &&
                    (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(h.confidence[k])))
                    throw std::runtime_error("nonfinite preview point/confidence");
            }
        }
        std::string global;
        if (have_previous_ && context_ != ctx.str()) {
            reset();
            global = "context_changed";
        }
        if (have_previous_ && id.frame_id == last_id_) {
            if (signature_ != sig.str())
                throw std::runtime_error("same preview frame id with different content");
            return cached_;
        }
        if (have_previous_ && (t <= last_time_ || double(t - last_time_) * 1e-6 >
                                                      PoseSmoothingParameters::max_gap_seconds)) {
            points_ = {};
            box_valid_ = {};
            global = "time_discontinuity";
        }
        for (int c = 0; c < 2; ++c) {
            const auto& h = raw[c];
            auto reason = global;
            if (!h.valid || h.score < PoseSmoothingParameters::detection) {
                points_[c] = {};
                box_valid_[c] = false;
                result.reset_reason[c] = "missing";
                continue;
            }
            const auto& box = meta[c].unclipped_box;
            const bool ambiguous = meta[c].nms_candidates > 1;
            if (ambiguous)
                reason = "ambiguous_raw_reset";
            else if (box_valid_[c] && iou(boxes_[c], box) < PoseSmoothingParameters::box_reset_iou)
                reason = "box_discontinuity_raw_reset";
            if (!reason.empty())
                points_[c] = {};
            result.reset_reason[c] = reason;
            boxes_[c] = box;
            box_valid_[c] = true;
            for (int k = 0; k < 4; ++k) {
                auto& previous = points_[c][k];
                if (!result.visible[c][k]) {
                    previous = {};
                    result.status[c][k] = h.confidence[k] < PoseSmoothingParameters::visibility
                                              ? "invalid_invisible"
                                              : "out_of_frame_raw_reset";
                    continue;
                }
                const cv::Point2d current(h.keypoints[k].x * double(image.sx) + image.pad_x,
                                          h.keypoints[k].y * double(image.sy) + image.pad_y);
                cv::Point2d filtered = current, velocity{};
                result.status[c][k] = reason.empty() ? "acquired_raw" : reason;
                if (previous.valid) {
                    const double dt = double(t - previous.timestamp_us) * 1e-6;
                    const auto raw_velocity = (current - previous.raw) / dt;
                    velocity = previous.velocity +
                               alpha(PoseSmoothingParameters::derivative_cutoff_hz, dt) *
                                   (raw_velocity - previous.velocity);
                    const double cutoff =
                        std::min(PoseSmoothingParameters::max_cutoff_hz,
                                 PoseSmoothingParameters::min_cutoff_hz +
                                     PoseSmoothingParameters::beta * cv::norm(velocity));
                    filtered =
                        previous.filtered + alpha(cutoff, dt) * (current - previous.filtered);
                    result.halves[c].keypoints[k] =
                        cv::Point2f(float((filtered.x - image.pad_x) / image.sx),
                                    float((filtered.y - image.pad_y) / image.sy));
                    result.status[c][k] = "measured_smoothed";
                }
                if (ambiguous)
                    previous = {};
                else
                    previous = {true, current, filtered, velocity, t};
            }
        }
        have_previous_ = true;
        last_time_ = t;
        last_id_ = id.frame_id;
        context_ = ctx.str();
        signature_ = sig.str();
        cached_ = result;
    } catch (const std::exception& e) {
        reset();
        result = rawPosePreview(raw, size, id.frame_id, t);
        result.ok = false;
        result.error = e.what();
        result.reset_reason.fill("preview_error_raw");
    }
    return result;
}
} // namespace rescue
