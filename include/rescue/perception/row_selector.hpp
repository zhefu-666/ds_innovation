#pragma once
#include "rescue/perception/perception_adapter.hpp"
#include "rescue/task/transport_rules.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>
namespace rescue {
struct RowParams {
    int min_n = 3;        // fewer collinear blocks than this is not a row
    float resid_m = .03f; // max distance of a block's ground point from the row line
    float max_range_m = 1.5f;
    float max_tilt_rad = .6f;  // the row must run roughly left-right: line angle to the body x axis
    float half_along_m = .09f; // frame footprint around a candidate: along the row axis ...
    float half_across_m = .06f; // ... and across it (depth)
    float neighbour_min_conf = .25f;
    uint64_t fresh_us = 200000;
};
struct RowPick {
    int id = -1, n = 0,
        index = -1; // chosen track id, blocks in the row, its position left to right
    std::string seq, reason = "no_row"; // G ordinary, K core, O injured, B dangerous, ? unknown
    cv::Point2f aim_body{}, aim_px{};
    std::vector<int>
        split_ids; // ordinary row blocks whose frame footprint holds an injured/dangerous/core
                   // piece: kept as candidates, to be separated by PILE
};
// Picks the block in the middle of a left-right row of blocks from the ground positions of the
// detection boxes.
class RowSelector {
    static char letter(const std::string& label) {
        switch (targetKind(label)) {
            case TargetKind::ORDINARY:
                return 'G';
            case TargetKind::CORE:
                return 'K';
            case TargetKind::INJURED:
                return 'O';
            case TargetKind::DANGEROUS:
                return 'B';
            default:
                return '?';
        }
    }

  public:
    // An ordinary block whose frame footprint holds an injured/dangerous/core piece is not skipped:
    // it stays a candidate and is listed in split_ids so the task separates it in PILE. A
    // non-ordinary candidate with any neighbour is still skipped. Ordinary supplies rank before
    // other selectable kinds, then the block nearest the middle of the row wins. prefer_id (the
    // current lock) is kept while it stays a candidate, so the choice does not jump while the row
    // grows or shrinks in view.
    static RowPick pick(const std::vector<SegDetection>& ds,
                        uint64_t now,
                        bool first,
                        float confidence,
                        int prefer_id = -1,
                        const RowParams& P = {}) {
        RowPick out;
        const float floor = std::min(confidence, P.neighbour_min_conf);
        std::vector<const SegDetection*> u;
        for (const auto& d : ds) {
            if (d.track_id < 0 || !d.ground_position_valid || !std::isfinite(d.body_xy_m.x) ||
                !std::isfinite(d.body_xy_m.y) || d.body_xy_m.y <= 0.f ||
                !std::isfinite(d.confidence) || d.confidence < floor || !d.timestamp_us ||
                now < d.timestamp_us || now - d.timestamp_us > P.fresh_us ||
                std::hypot(d.body_xy_m.x, d.body_xy_m.y) > P.max_range_m)
                continue;
            u.push_back(&d);
        }
        const int m = int(u.size());
        std::vector<int> best;
        float best_res = 1e9f;
        for (int i = 0; i < m; ++i)
            for (int j = i + 1; j < m; ++j) {
                const cv::Point2f v = u[j]->body_xy_m - u[i]->body_xy_m;
                const float len = std::hypot(v.x, v.y);
                if (len < 1e-3f || std::atan2(std::abs(v.y), std::abs(v.x)) > P.max_tilt_rad)
                    continue;
                const cv::Point2f dir = v * (1.f / len);
                std::vector<int> in;
                float res = 0.f;
                for (int k = 0; k < m; ++k) {
                    const cv::Point2f w = u[k]->body_xy_m - u[i]->body_xy_m;
                    const float e = std::abs(w.x * dir.y - w.y * dir.x);
                    if (e <= P.resid_m) {
                        in.push_back(k);
                        res += e * e;
                    }
                }
                if (in.size() > best.size() || (in.size() == best.size() && res < best_res)) {
                    best = in;
                    best_res = res;
                }
            }
        out.n = int(best.size());
        if (out.n < P.min_n)
            return out;
        cv::Point2f c(0, 0);
        for (int k : best)
            c += u[k]->body_xy_m;
        c *= 1.f / float(best.size());
        float sxx = 0, syy = 0, sxy = 0;
        for (int k : best) {
            const cv::Point2f w = u[k]->body_xy_m - c;
            sxx += w.x * w.x;
            syy += w.y * w.y;
            sxy += w.x * w.y;
        }
        const float ang = .5f * std::atan2(2.f * sxy, sxx - syy);
        cv::Point2f dir(std::cos(ang), std::sin(ang));
        if (dir.x < 0.f)
            dir = -dir;
        if (std::atan2(std::abs(dir.y), std::abs(dir.x)) > P.max_tilt_rad)
            return out;
        const cv::Point2f nrm(-dir.y, dir.x);
        const auto along = [&](const cv::Point2f& p) {
            return (p - c).dot(dir);
        };
        std::sort(best.begin(), best.end(), [&](int a, int b) {
            return along(u[a]->body_xy_m) < along(u[b]->body_xy_m);
        });
        for (int k : best)
            out.seq += letter(u[k]->label);
        const float s_mid =
            .5f * (along(u[best.front()]->body_xy_m) + along(u[best.back()]->body_xy_m));
        struct Key {
            int rank, pen;
            float mid, range;
        };
        int pick = -1, keep = -1, lpick = -1, lkeep = -1, lkeep_other = 0;
        Key pk{}, lk{};
        for (int r = 0; r < out.n; ++r) {
            const SegDetection& d = *u[best[r]];
            if (d.injured_upright_rejected || d.confidence < confidence ||
                !targetSelectable(d.label, first))
                continue;
            const bool ordinary = targetKind(d.label) == TargetKind::ORDINARY;
            int other = 0, fatal = 0;
            for (const auto* e : u) {
                if (e == &d)
                    continue;
                const cv::Point2f w = e->body_xy_m - d.body_xy_m;
                if (std::abs(w.dot(dir)) > P.half_along_m || std::abs(w.dot(nrm)) > P.half_across_m)
                    continue;
                if (ordinary && targetKind(e->label) == TargetKind::ORDINARY)
                    ++other;
                else
                    ++fatal;
            }
            if (fatal > 0) {
                if (!ordinary)
                    continue;
                out.split_ids.push_back(d.track_id);
            }
            const Key k{ordinary ? 0 : 1,
                        other,
                        std::abs(along(d.body_xy_m) - s_mid),
                        std::hypot(d.body_xy_m.x, d.body_xy_m.y)};
            if (other >= kMaxSuppliesPerTrip) { // too crowded for a clean pick: remember the least
                                                // crowded one (usually a row end)
                if (d.track_id == prefer_id) {
                    lkeep = r;
                    lkeep_other = other;
                }
                if (lpick < 0 || std::tie(k.rank, k.pen, k.mid, k.range) <
                                     std::tie(lk.rank, lk.pen, lk.mid, lk.range)) {
                    lpick = r;
                    lk = k;
                }
                continue;
            }
            if (d.track_id == prefer_id)
                keep = r;
            if (pick < 0 || std::tie(k.rank, k.pen, k.mid, k.range) <
                                std::tie(pk.rank, pk.pen, pk.mid, pk.range)) {
                pick = r;
                pk = k;
            }
        }
        bool crowded = false;
        if (pick < 0 && lpick >= 0) { // a solid run: no clean block, take the end that drags in the
                                      // fewest neighbours
            crowded = true;
            pick = lpick;
            if (lkeep >= 0 && lkeep_other <= lk.pen)
                keep = lkeep;
        }
        if (pick < 0) {
            out.reason = "row_no_clean_candidate";
            return out;
        }
        const bool kept = keep >= 0;
        if (kept)
            pick = keep;
        const SegDetection& d = *u[best[pick]];
        out.id = d.track_id;
        out.index = pick;
        out.reason = kept ? "row_keep_lock" : crowded ? "row_least_crowded" : "row_middle";
        out.aim_body = d.body_xy_m;
        out.aim_px = cv::Point2f(d.box.x + d.box.width * .5f, d.box.y + d.box.height * .5f);
        return out;
    }
};
} // namespace rescue
