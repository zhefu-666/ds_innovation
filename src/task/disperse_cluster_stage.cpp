#include "rescue/task/disperse_cluster_stage.hpp"
#include "rescue/push_task.hpp"
#include <cmath>

namespace rescue {
void DisperseClusterPlanner::addOrdinaryExtra(const cv::Point2f& point) {
    ord_extra_.push_back(point);
}

bool DisperseClusterPlanner::isOrdinary(const PushObservation& in, const cv::Point2f& q) const {
    for (const auto& o : in.clutter_ordinary)
        if (o.x == q.x && o.y == q.y)
            return true;
    for (const auto& o : ord_extra_)
        if (o.x == q.x && o.y == q.y)
            return true;
    return false;
}

ClusterGroup DisperseClusterPlanner::findBest(const TaskTuning& t,
                                              const PushObservation& in,
                                              const std::vector<cv::Point2f>& pts,
                                              float max_y,
                                              float radius) {
    best_n_ = 0;
    best_c_ = cv::Point2f(0.f, 0.f);
    best_d_ = 0.f;
    float best_score = -1e9f;
    for (const auto& p : pts) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.y < t.blind_scatter_min_y_m ||
            p.y > max_y || std::abs(p.x) > t.blind_scatter_half_x_m ||
            std::abs(std::atan2(-p.x, p.y)) > t.blind_scatter_max_bearing_rad)
            continue;
        int n = 0, n_ord = 0;
        cv::Point2f sum(0.f, 0.f);
        for (const auto& q : pts) {
            if (!std::isfinite(q.x) || !std::isfinite(q.y) || cv::norm(q - p) > radius)
                continue;
            ++n;
            sum += q;
            if (isOrdinary(in, q))
                ++n_ord;
        }
        if (t.blind_scatter_need_ordinary && n_ord == 0)
            continue; // injured / dangerous only: nothing worth ploughing
        const cv::Point2f c = sum * (1.f / float(n));
        const float d = float(cv::norm(c));
        // Dense, but the one nearest straight ahead wins close calls.
        const float score = float(n) - 2.f * std::abs(std::atan2(-c.x, c.y));
        if (score > best_score || (score == best_score && d < best_d_)) {
            best_n_ = n;
            best_c_ = c;
            best_d_ = d;
            best_score = score;
        }
    }
    return {best_n_, best_d_};
}

int DisperseClusterPlanner::evalGroup(const TaskTuning& t,
                                      const PushObservation& in,
                                      const std::vector<cv::Point2f>& pts,
                                      int need_n,
                                      int far_need,
                                      float far_y,
                                      int sweep_esc,
                                      float short_radius) {
    if (need_n > 1)
        need_n += sweep_esc;
    if (far_need > 1)
        far_need += sweep_esc;
    findBest(t,
             in,
             pts,
             t.blind_scatter_max_y_m,
             short_radius > 0.f ? short_radius : t.disperse_radius_m);
    if (best_n_ >= need_n)
        return 1;
    if (far_y > t.blind_scatter_max_y_m) { // long range: positions are coarse, a wider radius
                                           // still counts as one group
        findBest(t, in, pts, far_y, t.blind_scatter_far_radius_m);
        if (best_n_ >= far_need && best_d_ > t.blind_scatter_max_y_m)
            return 2; // a wide-radius group inside the short window is left to the normal trigger
    }
    best_n_ = 0;
    return 0;
}
} // namespace rescue
