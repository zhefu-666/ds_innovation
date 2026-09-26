#pragma once

#include "rescue/types.hpp"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace rescue {

struct TrackerConfig {
    float max_match_distance_px = 90.0f;
    uint32_t max_lost_frames = 12;
    uint32_t id_quarantine_frames = 30;
};

class NearestNeighborTracker {
public:
    explicit NearestNeighborTracker(TrackerConfig config = {});
    std::vector<SegDetection> update(const std::vector<SegDetection> &detections,
                                     uint64_t timestamp_us);
    void reset();

private:
    struct Track {
        SegDetection detection;
        uint32_t lost_frames = 0;
        uint64_t last_frame = 0;
    };
    int allocateId();

    TrackerConfig config_;
    std::unordered_map<int, Track> tracks_;
    std::unordered_map<int, uint64_t> quarantined_until_;
    int next_id_ = 1;
    uint64_t frame_index_ = 0;
};

} // namespace rescue
