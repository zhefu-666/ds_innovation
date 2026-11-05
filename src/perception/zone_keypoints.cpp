#include "rescue/core/stage_audit.hpp"
#include "rescue/perception/zone_keypoints.hpp"
#include "rescue/perception/rknn_model.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace rescue {
ZoneHalves decodeZonePose(const cv::Mat& output,
                          const ModelImage& image,
                          cv::Size original,
                          float box_confidence,
                          ZonePoseMetadata* metadata) {
    if (output.type() != CV_32F || output.dims != 3 || output.size[0] != 1 ||
        output.size[1] != kZonePoseChannels || !output.isContinuous())
        throw std::runtime_error(
            "Expected float32 [1,18,N] zone pose output (2 classes, 4 keypoints)");
    if (!std::isfinite(box_confidence) || box_confidence < 0 || box_confidence > 1)
        throw std::runtime_error("Invalid pose threshold");
    if (image.sx <= 0 || image.sy <= 0)
        throw std::runtime_error("Invalid letterbox scale");
    AuditScope audit("pose.decode");
    const int count = output.size[2];
    const float* raw = output.ptr<float>();
    const auto at = [&](int row, int i) {
        return raw[row * count + i];
    };
    std::array<int, 2> best{-1, -1};
    std::array<std::vector<int>, 2> candidates;
    if (metadata)
        *metadata = {};
    for (int i = 0; i < count; ++i) {
        bool finite = true;
        for (int k = 0; k < kZonePoseChannels; ++k)
            finite = finite && std::isfinite(at(k, i));
        if (!finite || at(2, i) <= 0 || at(3, i) <= 0)
            continue;
        const int cls = at(5, i) > at(4, i) ? 1 : 0;
        const float score = at(4 + cls, i);
        if (score < box_confidence || score > 1)
            continue;
        if (metadata)
            candidates[cls].push_back(i);
        if (best[cls] < 0 || score > at(4 + cls, best[cls]))
            best[cls] = i;
    }
    const auto unmap = [&](float x, float y) {
        return cv::Point2f((x - image.pad_x) / image.sx, (y - image.pad_y) / image.sy);
    };
    if (metadata) {
        // Side channel only: stable ties preserve the original first-anchor top1.
        for (int cls = 0; cls < 2; ++cls) {
            auto& list = candidates[cls];
            std::stable_sort(list.begin(), list.end(), [&](int a, int b) {
                return at(4 + cls, a) > at(4 + cls, b);
            });
            std::vector<cv::Rect2d> kept;
            for (int i : list) {
                const cv::Rect2d box(double(at(0, i)) - double(at(2, i)) / 2,
                                     double(at(1, i)) - double(at(3, i)) / 2,
                                     at(2, i),
                                     at(3, i));
                bool suppressed = false;
                for (const auto& b : kept) {
                    const double intersection = (box & b).area();
                    if (intersection / std::max(1e-9, box.area() + b.area() - intersection) > .45) {
                        suppressed = true;
                        break;
                    }
                }
                if (!suppressed)
                    kept.push_back(box);
            }
            (*metadata)[cls].nms_candidates = int(kept.size());
        }
    }
    ZoneHalves out;
    for (int cls = 0; cls < 2; ++cls) {
        const int i = best[cls];
        if (i < 0)
            continue;
        auto& h = out[cls];
        h.valid = true;
        h.score = at(4 + cls, i);
        const auto tl = unmap(at(0, i) - at(2, i) / 2, at(1, i) - at(3, i) / 2),
                   br = unmap(at(0, i) + at(2, i) / 2, at(1, i) + at(3, i) / 2);
        if (metadata)
            (*metadata)[cls].unclipped_box = cv::Rect2f(tl, br);
        h.box =
            cv::Rect2f(tl, br) & cv::Rect2f(0, 0, float(original.width), float(original.height));
        for (int k = 0; k < kZonePoseKeypoints; ++k) {
            const int row = 4 + kZonePoseClasses + 3 * k;
            h.keypoints[k] = unmap(at(row, i), at(row + 1, i));
            h.confidence[k] = std::clamp(at(row + 2, i), 0.f, 1.f);
        }
    }
    return out;
}
std::vector<ZoneKeypoint>
zoneHalvesToKeypoints(const ZoneHalves& h, float threshold, cv::Size image_size) {
    enum {
        FAR_LEFT = 0,
        NEAR_LEFT = 1,
        FAR_RIGHT = 2,
        NEAR_RIGHT = 3
    };
    const auto &L = h[0], &R = h[1];
    std::vector<ZoneKeypoint> out;
    const auto seen = [&](const ZoneHalf& half, int k) {
        const auto p = half.keypoints[k];
        const bool inside =
            image_size.empty() ||
            (p.x > 2 && p.y > 2 && p.x < image_size.width - 2 && p.y < image_size.height - 2);
        return half.valid && std::isfinite(half.confidence[k]) && half.confidence[k] >= threshold &&
               std::isfinite(p.x) && std::isfinite(p.y) && inside;
    };
    const auto single = [&](int id, const ZoneHalf& half, int k) {
        if (seen(half, k))
            out.push_back({id, half.keypoints[k], half.confidence[k], true});
    };
    // Divider corners from the two halves mark the same physical divider, so they
    // must agree within 5% of a half width, capped at 12 original-image pixels.
    const auto divider = [&](int id, int left_k, int left_outer, int right_k, int right_outer) {
        if (!seen(L, left_k) || !seen(R, right_k))
            return;
        const auto a = L.keypoints[left_k], b = R.keypoints[right_k];
        float span = 0;
        if (seen(L, left_outer))
            span = std::max(span, float(cv::norm(a - L.keypoints[left_outer])));
        if (seen(R, right_outer))
            span = std::max(span, float(cv::norm(b - R.keypoints[right_outer])));
        if (span <= 0 || cv::norm(a - b) > std::min(12.f, .05f * span))
            return;
        out.push_back(
            {id, (a + b) * .5f, std::min(L.confidence[left_k], R.confidence[right_k]), true});
    };
    single(0, L, NEAR_LEFT);
    divider(1, NEAR_RIGHT, NEAR_LEFT, NEAR_LEFT, NEAR_RIGHT);
    single(2, R, NEAR_RIGHT);
    single(3, L, FAR_LEFT);
    divider(4, FAR_RIGHT, FAR_LEFT, FAR_LEFT, FAR_RIGHT);
    single(5, R, FAR_RIGHT);
    return out;
}
ZoneHalves decodeZonePoseExperts(const cv::Mat& output,
                                 const ModelImage& image,
                                 cv::Size original,
                                 ZonePoseMetadata* metadata,
                                 ZonePoseRoute* route) {
    if (output.type() != CV_32F || output.dims != 3 || output.size[0] != 1 ||
        output.size[1] != 26 || output.size[2] != 8400 || !output.isContinuous())
        throw std::runtime_error("Expected float32 [1,26,8400] dual-expert pose output");
    if (!(image.sx > 0) || !(image.sy > 0) || !std::isfinite(image.sx) ||
        !std::isfinite(image.sy) || original.empty())
        throw std::runtime_error("Invalid letterbox scale");
    const float* data = output.ptr<float>();
    for (size_t i = 0; i < output.total(); ++i)
        if (!std::isfinite(data[i]))
            throw std::runtime_error("Nonfinite dual-expert output");
    const auto at = [&](int row, int i) {
        return data[row * 8400 + i];
    };
    const auto cls = [&](int i) {
        return at(5, i) > at(4, i) ? 1 : 0;
    };
    const auto score = [&](int i) {
        return at(4 + cls(i), i);
    };
    const auto box = [&](int i) {
        return cv::Rect2f(at(0, i) - at(2, i) / 2, at(1, i) - at(3, i) / 2, at(2, i), at(3, i));
    };
    const auto area = [](const cv::Rect2f& b) {
        return std::max(0.f, b.width) * std::max(0.f, b.height);
    };
    const auto unmap = [&](float x, float y) {
        return cv::Point2f(std::clamp((x - image.pad_x) / image.sx, 0.f, float(original.width)),
                           std::clamp((y - image.pad_y) / image.sy, 0.f, float(original.height)));
    };
    const auto originalBox = [&](int i) {
        auto b = box(i);
        return cv::Rect2f(unmap(b.x, b.y), unmap(b.x + b.width, b.y + b.height));
    };
    std::vector<int> order, kept;
    for (int i = 0; i < 8400; ++i)
        if (score(i) > .5f)
            order.push_back(i);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        return score(a) > score(b);
    });
    for (int i : order) {
        bool suppressed = false;
        for (int j : kept)
            if (cls(i) == cls(j)) {
                auto a = box(i), b = box(j);
                const float intersection = area(a & b);
                if (intersection / (area(a) + area(b) - intersection + 1e-7f) > .45f) {
                    suppressed = true;
                    break;
                }
            }
        if (!suppressed)
            kept.push_back(i);
        if (kept.size() == 300)
            break;
    }
    ZonePoseRoute r;
    r.dual_expert = true;
    r.reason = "missing_or_ambiguous";
    if (metadata)
        *metadata = {};
    for (int i : kept)
        ++r.counts[cls(i)];
    if (r.counts == std::array<int, 2>{1, 1}) {
        for (int i : kept)
            r.max_area_fraction =
                std::max(r.max_area_fraction,
                         originalBox(i).area() / (float(original.width) * original.height));
        constexpr double far_max = .04392929565847024, near_min = .14768375983277818;
        r.weight_far =
            float(std::clamp((near_min - r.max_area_fraction) / (near_min - far_max), 0., 1.));
        r.reason = "scale";
    }
    ZoneHalves result{};
    for (int i : kept) {
        const int c = cls(i);
        if (metadata)
            (*metadata)[c].nms_candidates = r.counts[c];
        if (result[c].valid)
            continue;
        auto& h = result[c];
        h.valid = true;
        h.score = score(i);
        h.box = originalBox(i);
        // The reference filter compares clipped original-image boxes.
        if (metadata)
            (*metadata)[c].unclipped_box = h.box;
        for (int k = 0; k < 4; ++k) {
            const float x =
                at(6 + 2 * k, i) * (1 - r.weight_far) + at(14 + 2 * k, i) * r.weight_far;
            const float y =
                at(7 + 2 * k, i) * (1 - r.weight_far) + at(15 + 2 * k, i) * r.weight_far;
            h.keypoints[k] = {(x - image.pad_x) / image.sx, (y - image.pad_y) / image.sy};
            h.confidence[k] = at(22 + k, i);
        }
    }
    if (route)
        *route = r;
    return result;
}
SafeZoneKeypointRknn::SafeZoneKeypointRknn(const Config& c) : box_confidence_(c.pose_confidence) {
    model_ = std::make_unique<RknnModel>(c.rknn_library, c.pose_model_path, c.pose_core_mask);
    if (model_->inputSize() != c.input_size ||
        (model_->outputChannels() != kZonePoseChannels && model_->outputChannels() != 26))
        throw std::runtime_error("Pose model contract mismatch: expected input " +
                                 std::to_string(c.input_size) +
                                 " (shared letterbox with detector) and [1,18|26," +
                                 ",N]; model has input " + std::to_string(model_->inputSize()) +
                                 " and [1," + std::to_string(model_->outputChannels()) + "," +
                                 std::to_string(model_->outputAnchors()) + "]");
}
SafeZoneKeypointRknn::~SafeZoneKeypointRknn() = default;
int SafeZoneKeypointRknn::inputSize() const {
    return model_->inputSize();
}
bool SafeZoneKeypointRknn::dualExpert() const {
    return model_->outputChannels() == 26;
}
ZoneHalves SafeZoneKeypointRknn::infer(const ModelImage& image,
                                       cv::Size original,
                                       ZonePoseMetadata* metadata,
                                       ZonePoseRoute* route) {
    auto output = model_->run(image.rgb);
    if (dualExpert())
        return decodeZonePoseExperts(output, image, original, metadata, route);
    if (route)
        *route = {};
    return decodeZonePose(output, image, original, box_confidence_, metadata);
}
} // namespace rescue
