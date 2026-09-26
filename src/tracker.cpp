#include "rescue/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rescue {

NearestNeighborTracker::NearestNeighborTracker(TrackerConfig config) : config_(config) {}

void NearestNeighborTracker::reset() {
    tracks_.clear();
    quarantined_until_.clear();
    next_id_ = 1;
    frame_index_ = 0;
}

int NearestNeighborTracker::allocateId() {
    for (;;) {
        const int id = next_id_++;
        const auto it = quarantined_until_.find(id);
        if (it == quarantined_until_.end() || it->second <= frame_index_) return id;
    }
}

std::vector<SegDetection> NearestNeighborTracker::update(
    const std::vector<SegDetection> &detections, uint64_t timestamp_us) {
    ++frame_index_;
    std::vector<bool> used(detections.size(), false);
    std::vector<int> matched;
    matched.reserve(detections.size());

    for (auto &entry : tracks_) {
        auto &track = entry.second;
        float best_distance = std::numeric_limits<float>::max();
        size_t best = detections.size();
        for (size_t i = 0; i < detections.size(); ++i) {
            if (used[i] || detections[i].label != track.detection.label) continue;
            const cv::Point2f delta = detections[i].center() - track.detection.center();
            const float distance = std::sqrt(delta.dot(delta));
            if (distance < best_distance && distance <= config_.max_match_distance_px) {
                best_distance = distance;
                best = i;
            }
        }
        if (best < detections.size()) {
            used[best] = true;
            track.detection = detections[best];
            track.detection.track_id = entry.first;
            track.detection.timestamp_us = timestamp_us;
            track.lost_frames = 0;
            track.last_frame = frame_index_;
            matched.push_back(entry.first);
        } else {
            ++track.lost_frames;
        }
    }

    for (size_t i = 0; i < detections.size(); ++i) {
        if (used[i]) continue;
        SegDetection detection = detections[i];
        detection.track_id = allocateId();
        detection.timestamp_us = timestamp_us;
        tracks_.emplace(detection.track_id, Track{detection, 0, frame_index_});
        matched.push_back(detection.track_id);
    }

    for (auto it = tracks_.begin(); it != tracks_.end();) {
        if (it->second.lost_frames > config_.max_lost_frames) {
            quarantined_until_[it->first] = frame_index_ + config_.id_quarantine_frames;
            it = tracks_.erase(it);
        } else {
            ++it;
        }
    }

    std::vector<SegDetection> output;
    output.reserve(matched.size());
    for (const int id : matched) {
        const auto it = tracks_.find(id);
        if (it != tracks_.end() && it->second.lost_frames == 0) output.push_back(it->second.detection);
    }
    return output;
}

} // namespace rescue
