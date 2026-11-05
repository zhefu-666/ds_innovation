#pragma once
#include "rescue/perception/perception_adapter.hpp"
#include "rescue/task/transport_rules.hpp"
#include "rescue/perception/trip_selection.hpp"
#include "rescue/perception/capture_monitor.hpp"
#include <sstream>
#include <map>
#include <limits>
#include <set>
namespace rescue {
struct PixelSelection {
    int id = -1;
    double raw = 0, smoothed = 0;
    std::string mode = "none", reason = "no_candidate";
    bool locked = false;
    int held_id = -1, released_id = -1, rebound_from = -1;
    int reselected_from = -1; // A different object; never a same-object tracker rebind.
    std::string candidates;
    std::string event = "none";
};
// Masks must already be mapped to the original image. Never resize an unknown mask.
class PixelSelector {
    struct Track {
        double value = 0;
        std::string label;
    };
    std::map<int, Track> history_;
    cv::Size size_;
    int16_t pitch_ = kCameraPitchInvalid;
    std::string mode_;
    int locked_ = -1, challenger_ = -1, count_ = 0;
    uint64_t last_ = 0, last_capture_ = 0;
    uint64_t challenger_us_ = 0;
    int task_id_ = -1;
    int near_challenger_ = -1, near_count_ = 0, reselected_from_ = -1;
    bool split_selection_ = false;
    int split_candidate_ = -1;
    Inventory keep_inventory_;
    PushState observation_state_ = PushState::WAIT_START;
    uint64_t missing_since_us_ = 0;
    std::string locked_label_;
    cv::Rect locked_box_;
    bool view_changed_ = false;
    bool searching_ = false;
    std::set<int> neighbours_;
    bool have_center_ = false;
    cv::Point2f last_center_, challenger_center_;
    static constexpr uint64_t kLockGraceUs = 300000;
    // Pending confirmation follows the image position for short tracker re-IDs.
    static constexpr double kPendingCenterGateFrac = .06;
    uint64_t relock_window_us_ = 1500000;
    double grace_gate_frac_ = .05;
    // Neighbour window around a candidate: footprint half-width .07 + block half-size .02,
    // footprint depth .081/2 + .02.
    static constexpr float kCrowdHalfX = .09f, kCrowdHalfY = .06f;
    // Pile-edge preference: a candidate with injured/dangerous pieces inside its footprint window
    // (x10) or on the robot->target line in front of it (x3, the frame would have to sweep them
    // aside first) ranks last. 0 = clean, 1 = one ordinary neighbour (still legal), >=10 = the trip
    // rules would veto it. relax 1 drops the in-front term, relax 2 ignores crowding altogether (no
    // delivery for a long time).
    static constexpr float kCrowdMinConfidence = .25f, kLaneHalfWidth = .09f, kLaneBehindM = .07f;
    static int
    crowd(const std::vector<SegDetection>& ds, int id, float confidence, uint64_t now, int relax) {
        if (relax >= 2)
            return 0;
        const SegDetection* t = nullptr;
        for (const auto& d : ds)
            if (d.track_id == id) {
                t = &d;
                break;
            }
        const auto usable = [&](const SegDetection& d) {
            return d.ground_position_valid && std::isfinite(d.body_xy_m.x) &&
                   std::isfinite(d.body_xy_m.y);
        };
        if (!t || !usable(*t))
            return 5;
        const bool ordinary = targetKind(t->label) == TargetKind::ORDINARY;
        const float tx = t->body_xy_m.x, ty = t->body_xy_m.y, dist = std::hypot(tx, ty);
        const float ux = dist > 1e-3f ? tx / dist : 0.f, uy = dist > 1e-3f ? ty / dist : 1.f;
        const float floor = std::min(confidence, kCrowdMinConfidence);
        int other = 0, fatal = 0, front_fatal = 0, front_other = 0;
        for (const auto& d : ds) {
            if (&d == t || d.track_id == id || d.track_id < 0 || !usable(d) ||
                !std::isfinite(d.confidence) || d.confidence < floor || !d.timestamp_us ||
                now < d.timestamp_us || now - d.timestamp_us > 200000)
                continue;
            const bool same_kind = ordinary && targetKind(d.label) == TargetKind::ORDINARY;
            if (std::abs(d.body_xy_m.x - tx) <= kCrowdHalfX &&
                std::abs(d.body_xy_m.y - ty) <= kCrowdHalfY) {
                if (same_kind)
                    ++other;
                else
                    ++fatal;
                continue;
            }
            if (relax >= 1)
                continue;
            const float along = d.body_xy_m.x * ux + d.body_xy_m.y * uy,
                        lateral = std::abs(d.body_xy_m.x * uy - d.body_xy_m.y * ux);
            if (along < 0.f || along > dist + kLaneBehindM || lateral > kLaneHalfWidth)
                continue;
            if (same_kind)
                ++front_other;
            else
                ++front_fatal;
        }
        // Exactly one more ordinary supply beside/in front (and nothing dangerous) is a legal pair:
        // rank it first. Three supplies would break the trip limit, so two or more neighbours stay
        // vetoed.
        const bool pair = ordinary && other + front_other == 1 && fatal == 0 && front_fatal == 0;
        return fatal * 10 + (other >= kMaxSuppliesPerTrip ? 10 : 0) + front_fatal * 3 -
               (pair ? 1 : 0);
    }
    int relax_ = 0;

  public:
    // Apply only to target selection. All detections still reach inventory/safety checks.
    void setCaptureLoad(const Inventory& inventory) { keep_inventory_ = inventory; }
    void setRelax(int level) {
        relax_ = std::clamp(level, 0, 2);
    }
    // Pending confirmation window and same-view image gate for a new tracker ID.
    // Missing targets are released after kLockGraceUs; this window never extends that grace.
    void setRelock(uint64_t window_us, double gate_frac) {
        relock_window_us_ = window_us;
        grace_gate_frac_ = std::clamp(gate_frac, .0, 1.);
    }
    void forgetCenter() {
        reset(); // A sweep ends the selection episode, including its target identity.
    }
    void syncTaskTarget(int id, const std::string& label) {
        if (id == task_id_) {
            // The task can accept a fresh view choice that reuses the tracker ID.
            if (id == locked_ && !view_changed_)
                reselected_from_ = -1;
            return;
        }
        if (id < 0 || id != locked_) {
            reset();
            locked_ = id;
            locked_label_ = label;
        }
        task_id_ = id;
        if (id == locked_)
            reselected_from_ = -1;
    }
    // End the old selection episode once on entry, then accumulate fresh observations.
    // A task without a cargo lock must not erase the independent visual confirmation.
    void syncTaskState(PushState state, int id, const std::string& label, bool searching = false) {
        if (searching != searching_)
            reset();
        searching_ = searching;
        const bool scattering = state == PushState::SCATTER || state == PushState::PILE;
        if (scattering && state != observation_state_)
            reset();
        observation_state_ = state;
        // Search IDs describe this view only; the task must not revive an old lock.
        syncTaskTarget(scattering || searching ? -1 : id,
                       scattering || searching ? std::string() : label);
    }
    // The split task explicitly ends the old episode and nominates a fresh UI140 block.
    // Repeated requests for the same candidate must preserve three-frame confirmation.
    void syncSplitSelection(bool requested, int candidate) {
        if (requested && (!split_selection_ || candidate != split_candidate_))
            reset();
        split_selection_ = requested;
        split_candidate_ = requested ? candidate : -1;
    }
    void clearComparison() {
        near_challenger_ = -1;
        near_count_ = 0;
        history_.clear();
        challenger_ = -1;
        count_ = 0;
        challenger_us_ = 0;
    }
    void reset() {
        history_.clear();
        locked_ = -1;
        challenger_ = -1;
        count_ = 0;
        last_ = 0;
        last_capture_ = 0;
        challenger_us_ = 0;
        task_id_ = -1;
        near_challenger_ = -1;
        near_count_ = 0;
        reselected_from_ = -1;
        split_selection_ = false;
        split_candidate_ = -1;
        missing_since_us_ = 0;
        locked_label_.clear();
        locked_box_ = {};
        view_changed_ = false;
        neighbours_.clear();
        have_center_ = false;
    }
    PixelSelection update(const std::vector<SegDetection>& ds,
                          cv::Size size,
                          int16_t pitch,
                          bool stable,
                          uint64_t now,
                          bool first,
                          float confidence,
                          const std::vector<int>& rejected = {},
                          const std::string& prefer_label = "",
                          float /* heading */ = std::numeric_limits<float>::quiet_NaN(),
                          float ui140_core_confidence = 1.f,
                          const CaptureConfig* near_config = nullptr,
                          bool allow_near_reselect = false) {
        // Reconfirm wanted cargo independently after every search view change.
        // Capture keeps its existing task-owned identity and association rules.
        if (searching_ && (!stable || pitch == kCameraPitchInvalid ||
                          (pitch_ != kCameraPitchInvalid && (pitch != pitch_ || size != size_))))
            reset();
        PixelSelection out;
        out.held_id = locked_;
        out.reselected_from = reselected_from_;
        if (task_id_ >= 0 && locked_ < 0) {
            out.released_id = task_id_;
            out.reason = "lock_released";
            out.event = "await_task_release";
            return out; // Retry the event until the task acknowledges it, never revive its old ID.
        }
        const auto release = [&](const char* why) {
            out.released_id = locked_;
            out.held_id = -1;
            out.event = why;
            out.reason = "lock_released";
            const int accepted = task_id_;
            reset();
            task_id_ = accepted; // Do not resurrect the previous task output on the next frame.
        };
        if (!now || (last_ && now <= last_)) {
            out.reason = "duplicate_or_old_frame";
            return out;
        }
        if (size.width <= 0 || size.height <= 0 || !stable || pitch == kCameraPitchInvalid) {
            clearComparison();
            if (locked_ >= 0) {
                view_changed_ = true;
                missing_since_us_ = 0; // Camera motion has its own task timeout.
            }
            out.reason = locked_ >= 0 ? "lock_view_wait" : "invalid_image_or_pitch";
            return out;
        }
        if (last_ && now - last_ > 200000) {
            clearComparison();
            if (locked_ >= 0 && !view_changed_ && now - last_ >= kLockGraceUs) {
                release("frame_gap_timeout");
                return out;
            }
        }
        uint64_t capture_stamp = 0;
        bool masks = false, boxes = false;
        std::map<int, std::pair<double, std::string>> values;
        const bool near_view =
            near_config && near_config->pixel_grasp_enabled &&
            near_config->pixel_center_membership &&
            std::any_of(near_config->grasp_center_ranges.begin(),
                        near_config->grasp_center_ranges.end(),
                        [&](const auto& range) {
                            return size == cv::Size(range.image_width_px, range.image_height_px) &&
                                   std::abs(int(pitch) - int(range.pitch_cdeg)) <=
                                       near_config->pitch_tolerance_cdeg;
                        });
        std::map<int, int> priority; // 0: complete inside, 1: complete outside, 2: unselectable.
        std::ostringstream candidates;
        std::set<int> geo_ok;
        std::map<int, cv::Point2f> centers;
        std::set<int> vetoed; // explicitly rejected ids release the lock at once; only mere absence
                              // gets grace
        for (int id : rejected)
            vetoed.insert(id);
        for (const auto& d : ds) {
            if (d.track_id == locked_ &&
                (d.label != locked_label_ || !targetSelectable(d.label, first) ||
                 (keep_inventory_.total() && !keep_inventory_.count(targetKind(d.label)))))
                vetoed.insert(d.track_id);
            if (d.track_id < 0 || !targetSelectable(d.label, first) ||
                (keep_inventory_.total() && !keep_inventory_.count(targetKind(d.label))) ||
                !std::isfinite(d.confidence) || d.confidence < (d.label == "core_supply" ? std::min(confidence, ui140_core_confidence) : confidence) || !d.timestamp_us ||
                now < d.timestamp_us || now - d.timestamp_us > 200000 || d.box.width <= 0 ||
                d.box.height <= 0 ||
                std::find(rejected.begin(), rejected.end(), d.track_id) != rejected.end())
                continue;
            const auto clipped = d.box & cv::Rect(0, 0, size.width, size.height);
            if (clipped.empty())
                continue;
            double pixels = clipped.area();
            if (!d.mask.empty()) {
                if (d.mask.size() != size || d.mask.type() != CV_8UC1)
                    continue;
                pixels = cv::countNonZero(d.mask);
                masks = true;
            } else
                boxes = true;
            capture_stamp = std::max(capture_stamp, d.timestamp_us);
            if (pixels > 0) {
                values[d.track_id] = {pixels / (double(size.width) * size.height), d.label};
                centers[d.track_id] =
                    cv::Point2f(clipped.x + clipped.width * .5f, clipped.y + clipped.height * .5f);
                if (d.ground_position_valid && d.ground_contact_valid &&
                    std::isfinite(d.body_xy_m.x) && std::isfinite(d.body_xy_m.y) &&
                    d.body_xy_m.y > 0)
                    geo_ok.insert(d.track_id);
            }
        }
        if (near_view) {
            for (const auto& d : ds) {
                if (!values.count(d.track_id))
                    continue;
                const auto range =
                    std::find_if(near_config->grasp_center_ranges.begin(),
                                 near_config->grasp_center_ranges.end(),
                                 [&](const auto& r) {
                                     return r.label == d.label &&
                                            size == cv::Size(r.image_width_px, r.image_height_px) &&
                                            std::abs(int(pitch) - int(r.pitch_cdeg)) <=
                                                near_config->pitch_tolerance_cdeg;
                                 });
                const bool calibrated = range != near_config->grasp_center_ranges.end();
                const bool complete = captureBoxComplete(d.box, size);
                const bool inside = calibrated && range->containsCenter(d.box);
                const int rank = calibrated && complete ? (inside ? 0 : 1) : 2;
                priority[d.track_id] = rank;
                candidates << " id=" << d.track_id << ":inside=" << inside
                           << ",complete=" << complete << ",priority=" << rank
                           << ",area=" << d.box.area() << ",reason="
                           << (!calibrated ? "uncalibrated"
                               : !complete ? "image_clipped"
                               : inside    ? "complete_inside"
                                           : "complete_outside");
            }
            out.candidates = candidates.str();
        }
        if (split_selection_) {
            for (auto it = values.begin(); it != values.end();)
                if (it->first != split_candidate_)
                    it = values.erase(it);
                else
                    ++it;
        }
        if (locked_ < 0 && !prefer_label.empty() &&
            std::any_of(
                values.begin(),
                values.end(),
                [&](const auto& kv) {
                    return kv.second.second == prefer_label;
                })) {
            for (auto it = values.begin(); it != values.end();)
                if (it->second.second != prefer_label)
                    it = values.erase(it);
                else
                    ++it;
        }
        if (near_view) {
            if (locked_ < 0) {
                int rank = 2;
                for (const auto& [id, v] : values)
                    rank = std::min(rank, priority[id]);
                for (auto it = values.begin(); it != values.end();)
                    if (rank == 2 || priority[it->first] != rank)
                        it = values.erase(it);
                    else
                        ++it;
            }
        }
        // A larger box without usable ground contact (e.g. clipped at the image edge after a pitch
        // change) must not out-rank a measurable one: the task could never approach it.
        if (!near_view && locked_ < 0 &&
            std::any_of(values.begin(), values.end(), [&](const auto& kv) {
                return geo_ok.count(kv.first) > 0;
            })) {
            for (auto it = values.begin(); it != values.end();)
                if (!geo_ok.count(it->first))
                    it = values.erase(it);
                else
                    ++it;
        }
        const std::string mode = masks ? (boxes ? "mixed_invalid" : "mask_pixels")
                                 : boxes || mode_.empty() ? "box_area"
                                                          : mode_;
        if (size != size_ || pitch != pitch_ || mode != mode_) {
            clearComparison();
            if (locked_ >= 0 && (size != size_ || pitch != pitch_))
                view_changed_ = true;
            out.reason = "comparison_context_changed";
        }
        size_ = size;
        pitch_ = pitch;
        mode_ = mode;
        out.mode = mode;
        if (masks && boxes) {
            clearComparison();
            if (locked_ >= 0) {
                if (!missing_since_us_)
                    missing_since_us_ = now;
                if (now - missing_since_us_ >= kLockGraceUs) {
                    release("invalid_comparison_timeout");
                    return out;
                }
            }
            out.reason = "mixed_pixel_modes";
            return out;
        }
        if (!now || now <= last_ || (capture_stamp && capture_stamp <= last_capture_)) {
            out.reason = "duplicate_or_old_frame";
            return out;
        }
        last_ = now;
        last_capture_ = capture_stamp;
        if (challenger_ >= 0 && challenger_us_ && relock_window_us_ > 0 && now >= challenger_us_ &&
            now - challenger_us_ > relock_window_us_) {
            challenger_ = -1;
            challenger_us_ = 0;
            count_ = 0;
        }
        if (locked_ >= 0 && !view_changed_ && vetoed.count(locked_)) {
            release("target_rejected");
            return out;
        }
        for (auto it = history_.begin(); it != history_.end();)
            if (!values.count(it->first))
                it = history_.erase(it);
            else
                ++it;
        bool lock_absent = locked_ >= 0 && !values.count(locked_);
        if (locked_ >= 0 && view_changed_)
            lock_absent = true; // Even the same tracker ID must be re-observed in a new view.
        int rebound = -1;
        bool continuous_rebind = false;
        if (lock_absent) {
            if (!missing_since_us_)
                missing_since_us_ = now;
            if (now - missing_since_us_ >= kLockGraceUs) {
                release("target_lost_timeout");
                return out;
            }
            int matches = 0;
            if (view_changed_) {
                // A new view starts a new cargo choice, not a physical-identity match.
                // At a stopped open frame, choose wanted cargo inside the measured
                // image region. Neither old ID/class nor ground coordinates constrain it.
                if (allow_near_reselect) {
                    for (const auto& d : ds) {
                        if (!values.count(d.track_id) || !captureBoxComplete(d.box, size) ||
                            (near_view && priority[d.track_id] != 0))
                            continue;
                        if (rebound < 0 || values[d.track_id].first > values[rebound].first)
                            rebound = d.track_id;
                    }
                }
                matches = rebound >= 0 ? 1 : 0;
            } else {
                for (const auto& d : ds) {
                    if (!values.count(d.track_id) || d.label != locked_label_ ||
                        (d.track_id != locked_ && neighbours_.count(d.track_id)))
                        continue;
                    const double scale =
                        locked_box_.area() > 0 ? double(d.box.area()) / locked_box_.area() : 0;
                    const bool image_match = have_center_ &&
                        cv::norm(centers[d.track_id] - last_center_) <= grace_gate_frac_ * size.width &&
                        scale >= .25 && scale <= 4.;
                    if (image_match) {
                        rebound = d.track_id;
                        ++matches;
                    }
                }
            }
            // During RUSH, a unique, strongly overlapping same-view detection is
            // the current physical target under a new tracker ID, not a new selection.
            if (matches == 1 && observation_state_ == PushState::RUSH && !view_changed_) {
                for (const auto& d : ds)
                    if (d.track_id == rebound) {
                        const double overlap = double((d.box & locked_box_).area());
                        const double total = double(d.box.area()) + locked_box_.area() - overlap;
                        continuous_rebind = total > 0 && overlap / total >= .6;
                    }
            }
            if (matches != 1) {
                challenger_ = -1;
                count_ = 0;
                out.reason = "lock_grace_waiting";
                out.event = matches > 1 ? "rebind_ambiguous" : "target_missing";
                return out;
            }
        } else if (locked_ >= 0) {
            missing_since_us_ = 0;
        }
        std::vector<SegDetection> trip_candidates;
        for (const auto& object : ds)
            if (values.count(object.track_id))
                trip_candidates.push_back(object);
        std::map<int, std::pair<int,int>> trip_ranks;
        for (const auto& object : trip_candidates)
            trip_ranks[object.track_id] = candidateTripRank(
                object, trip_candidates, first, confidence, now, ui140_core_confidence);
        const auto tripRank = [&](int id) { return trip_ranks.at(id); };
        int best = -1;
        for (const auto& [id, v] : values) {
            auto it = history_.find(id);
            if (it == history_.end() || it->second.label != v.second)
                history_[id] = {v.first, v.second};
            else
                it->second.value = .5 * v.first + .5 * it->second.value;
            if (best < 0 || tripRank(id) > tripRank(best) ||
                (tripRank(id) == tripRank(best) && history_[id].value > history_[best].value))
                best = id;
        }
        if (best < 0) {
            out.reason = lock_absent ? "lock_grace_waiting" : "no_legal_candidate";
            return out;
        }
        // A fresh acquisition prefers the least crowded comparable candidate: a block whose
        // footprint would swallow an injured/dangerous piece or a third supply is vetoed later
        // anyway, so do not drive at it first.
        // A new inside candidate may replace an outside/clipped lock only at an
        // acknowledged open-frame stop, after three distinct fresh observations.
        int near_pick = -1;
        if (near_view && allow_near_reselect && locked_ >= 0 && !view_changed_ &&
            values.count(locked_)) {
            for (const auto& [id, v] : values) {
                if (priority[id] >= priority[locked_] || priority[id] >= 2)
                    continue;
                if (near_pick < 0 || priority[id] < priority[near_pick] ||
                    (priority[id] == priority[near_pick] &&
                     history_[id].value > history_[near_pick].value))
                    near_pick = id;
            }
        }
        if (near_pick < 0) {
            near_challenger_ = -1;
            near_count_ = 0;
        } else {
            if (near_challenger_ != near_pick) {
                near_challenger_ = near_pick;
                near_count_ = 0;
            }
            if (++near_count_ >= 3) {
                reselected_from_ = locked_;
                out.reselected_from = locked_;
                out.event = "near_target_reselected";
                locked_ = near_pick;
                locked_label_ = values[near_pick].second;
                challenger_ = near_challenger_ = -1;
                count_ = near_count_ = 0;
            }
        }
        if (locked_ < 0 && values.size() > 1) {
            const double floor = .3 * history_[best].value;
            int pick = best, pen = crowd(ds, best, confidence, now, relax_);
            for (const auto& [id, v] : values) {
                if (tripRank(id) != tripRank(best) || history_[id].value < floor)
                    continue;
                const int p = crowd(ds, id, confidence, now, relax_);
                if (p < pen || (p == pen && history_[id].value > history_[pick].value)) {
                    pick = id;
                    pen = p;
                }
            }
            if (challenger_ >= 0 && challenger_ != pick && values.count(challenger_) &&
                tripRank(challenger_) == tripRank(best) &&
                history_[challenger_].value >= floor &&
                (crowd(ds, challenger_, confidence, now, relax_) <= pen ||
                 crowd(ds, challenger_, confidence, now, relax_) < 10))
                pick = challenger_;
            best = pick;
        }
        // Before a lock exists, keep the pending target by image position. Detection IDs can
        // alternate between two frames even when the same physical block stays in place; choosing
        // purely by area would then reset confirmation forever. A different block must move outside
        // this short gate before it can replace the pending target.
        if (locked_ < 0 && challenger_ >= 0 && challenger_us_ &&
            now >= challenger_us_ &&
            (relock_window_us_ == 0 || now - challenger_us_ <= relock_window_us_)) {
            int nearest_id = -1;
            double nearest = 1e18;
            for (const auto& [id, v] : values) {
                const auto c = centers.find(id);
                if (c == centers.end() || tripRank(id) != tripRank(best))
                    continue;
                const double dd = cv::norm(c->second - challenger_center_);
                if (dd <= kPendingCenterGateFrac * size.width && dd < nearest) {
                    nearest = dd;
                    nearest_id = id;
                }
            }
            if (nearest_id >= 0)
                best = nearest_id;
        }
        // For the first ordinary trip, a fore/aft pair starts at the block nearer
        // the chassis. Image layout chooses a target, never permission to split.
        if (!near_view && !first && locked_ < 0 && values.size() == 2 && ds.size() == 2) {
            auto a = values.begin(), b = std::next(a);
            if (a->second.second == "ordinary_supply" && b->second.second == "ordinary_supply") {
                const auto delta = centers[a->first] - centers[b->first];
                if (std::abs(delta.y) > std::abs(delta.x))
                    best = delta.y > 0 ? a->first : b->first;
            }
        }
        if (locked_ >= 0)
            best = lock_absent ? rebound : locked_;
        // Ranking only acquires a target. A visible lock is not an area contest.
        const bool challenge = locked_ < 0 || lock_absent;
        if (challenge) {
            if (challenger_ != best) { // a re-ID of the same physical block (centre within the
                                       // pending gate of the width) keeps the confirmation count
                const bool same = locked_ < 0 && challenger_ >= 0 && centers.count(best) &&
                                  cv::norm(centers[best] - challenger_center_) <=
                                      kPendingCenterGateFrac * size.width;
                challenger_ = best;
                if (!same)
                    count_ = 0;
            }
            if (centers.count(best))
                challenger_center_ = centers[best];
            challenger_us_ = now;
            if (++count_ >= 3 || continuous_rebind) {
                if (view_changed_) {
                    reselected_from_ = locked_;
                    out.reselected_from = locked_;
                    out.event = "view_target_reselected";
                } else {
                    out.rebound_from = locked_;
                    out.event = locked_ < 0 ? "target_acquired" : "same_object_rebound";
                }
                locked_ = best;
                locked_label_ = values[best].second;
                count_ = 0;
                challenger_ = -1;
                missing_since_us_ = 0;
                view_changed_ = false;
            }
        } else {
            count_ = 0;
            challenger_ = -1;
        }
        const bool held = locked_ >= 0 && values.count(locked_) && !view_changed_ &&
                          (!lock_absent || out.rebound_from >= 0 || out.reselected_from >= 0);
        out.locked = held;
        out.held_id = locked_;
        if (held && task_id_ >= 0 && task_id_ != locked_ && reselected_from_ < 0)
            out.rebound_from = task_id_;
        if (held && out.event == "none")
            out.event = reselected_from_ >= 0   ? "await_near_reselection"
                        : out.rebound_from >= 0 ? "await_task_rebind"
                                                : "target_held";
        out.id = held ? locked_ : -1;
        out.reason = held ? "pixel_ratio_locked"
                          : (locked_ >= 0 ? "lock_grace_waiting" : "confirming_three_frames");
        const int shown = held ? locked_ : best;
        out.raw = values[shown].first;
        out.smoothed = history_[shown].value;
        if (held && centers.count(locked_)) {
            last_center_ = centers[locked_];
            have_center_ = true;
            neighbours_.clear();
            for (const auto& d : ds) {
                if (d.track_id != locked_) {
                    if (values.count(d.track_id))
                        neighbours_.insert(d.track_id);
                    continue;
                }
                locked_box_ = d.box;

            }
        }
        return out;
    }
};
} // namespace rescue
