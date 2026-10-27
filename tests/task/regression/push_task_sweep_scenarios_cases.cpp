#include "../../support/push_task_sim.hpp"
#include "push_task_cases.hpp"

void sweepTriggerCases() {
    { // 密集成堆：冲入后高速摆扫分散，之后清掉黑名单并继续接近；稀疏则不触发；冷却期内不重复触发。
        TaskTuning t;
        t.enable_search_cues = false;
        t.assume_all_safe = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.track_near_m = .4f;
        t.intermediate_to_near_m = .27f;
        t.rush_start_m = .35f;
        struct D {
            bool rush = false, swing = false, done = false, prepared = false;
            float max_phi = 0, max_wz = 0;
        };
        const auto run = [&](int n_clutter, bool disperse) {
            TaskTuning tt = t;
            tt.disperse = disperse;
            Sim s(tt);
            s.in.distance_m = .5f;
            assert(s.until(PushState::APPROACH, 80));
            s.in.distance_m = .25f;
            D d;
            for (int i = 0; i < 400 && !d.prepared; ++i) {
                s.in.clutter_body.clear();
                for (int k = 0; k < n_clutter; ++k)
                    s.in.clutter_body.push_back(
                        {-.12f + .08f * k, .25f + ((k % 2) ? .06f : -.06f)});
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                d.max_phi = std::max(d.max_phi, std::abs(s.phi));
                d.max_wz = std::max(d.max_wz, std::abs(s.out.motion.wz_rps));
                if (s.out.reason == "disperse_rush")
                    d.rush = true;
                if (s.out.reason == "disperse_swing")
                    d.swing = true;
                if (s.out.reason == "disperse_done")
                    d.done = true;
                if (s.out.state == PushState::PREPARE)
                    d.prepared = true;
                if (d.done && s.out.state == PushState::APPROACH)
                    s.in.distance_m = .19f;
                if (s.out.state != PushState::APPROACH && s.out.state != PushState::PREPARE &&
                    s.out.state != PushState::SCAN)
                    break;
            }
            return d;
        };
        auto dense = run(4, true);
        assert(dense.rush && dense.swing && dense.max_wz >= 1.5f && dense.max_phi >= .5f);
        auto sparse = run(1, true);
        assert(!sparse.rush && !sparse.swing);
        auto off = run(4, false);
        assert(!off.rush && !off.swing);

        { // 打散后目标丢失：先在20°视角搜索（近处货物），关闭开关则仍从5°开始。
            const auto after = [&](bool mid) {
                TaskTuning tm = t;
                tm.enable_lost_search = true;
                tm.post_clear_mid_view = mid;
                Sim s(tm);
                s.in.distance_m = .5f;
                assert(s.until(PushState::APPROACH, 80));
                s.in.distance_m = .25f;
                bool done = false;
                for (int i = 0; i < 400 && !done; ++i) {
                    s.in.clutter_body.clear();
                    for (int k = 0; k < 4; ++k)
                        s.in.clutter_body.push_back(
                            {-.12f + .08f * k, .25f + ((k % 2) ? .06f : -.06f)});
                    s.in.clutter_ts_us = s.in.now_us;
                    s.tick();
                    if (s.out.reason == "disperse_done")
                        done = true;
                }
                assert(done);
                s.in.clutter_body.clear();
                s.in.target_valid = false;
                for (int i = 0; i < 60 && s.out.state != PushState::LOST_SEARCH; ++i)
                    s.tick();
                assert(s.out.state == PushState::LOST_SEARCH);
                return int(s.out.motion.camera_pitch_cdeg);
            };
            assert(after(true) == 2000);
            assert(after(false) == 500);
        }
    }
    { // 看不到普通物资但前方有成团的其他物体：转向它、冲入并摆扫（两趟），然后回到搜索；单个也触发；太远/太偏/过期/开关关闭/已有目标都不触发。
        TaskTuning t;
        t.enable_search_cues = false;
        t.blind_scatter_min = 1;
        t.blind_scatter_first_turn_rad = 0.f;
        t.blind_scatter_passes =
            2; // this block exercises the two-pass mechanics; the default is one pass
        struct B {
            bool align = false, swing = false, done = false, budget = false, back = false;
            int starts = 0, rushes = 0, scans_after = 0, pitch_after = -1;
            float min_phi = 0, max_phi = 0;
        };
        const auto run = [&](std::vector<cv::Point2f> body_pts,
                             bool on,
                             bool fresh,
                             bool ordinary,
                             int max_n,
                             int ticks,
                             bool follow = false) {
            TaskTuning tt = t;
            tt.blind_scatter = on;
            tt.blind_scatter_max = max_n;
            Sim s(tt);
            s.in.target_valid = ordinary;
            s.in.distance_m = .9f;
            const float x0 = s.px, y0 = s.py;
            std::vector<cv::Point2f> world;
            for (const auto& b : body_pts)
                world.push_back({x0 + b.x, y0 + b.y});
            B b;
            std::string prev;
            for (int i = 0; i < ticks; ++i) {
                s.in.clutter_body.clear();
                const float c = std::cos(s.phi), sn = std::sin(s.phi);
                if (follow)
                    s.in.clutter_body = body_pts;
                else
                    for (const auto& w : world) {
                        const float dx = w.x - s.px, dy = w.y - s.py;
                        s.in.clutter_body.push_back({dx * c + dy * sn, -dx * sn + dy * c});
                    }
                if (fresh)
                    s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                b.min_phi = std::min(b.min_phi, s.phi);
                b.max_phi = std::max(b.max_phi, s.phi);
                if (s.out.state == PushState::SCATTER) {
                    if (s.out.reason == "blind_scatter_align")
                        b.align = true;
                    if (s.out.reason == "disperse_swing")
                        b.swing = true;
                    if (s.out.reason == "disperse_rush" && prev != "disperse_rush")
                        ++b.rushes;
                    if (s.out.reason == "blind_scatter_back")
                        b.back = true;
                }
                if (s.out.reason == "blind_scatter_start")
                    ++b.starts;
                if (s.out.reason == "blind_scatter_done")
                    b.done = true;
                if (s.out.reason == "blind_scatter_timeout" ||
                    s.out.reason == "blind_scatter_align_timeout")
                    b.budget = true;
                prev = s.out.reason;
                if (b.done && s.out.state == PushState::SCAN && ++b.scans_after == 1)
                    b.pitch_after = s.out.motion.camera_pitch_cdeg;
            }
            return b;
        };
        const std::vector<cv::Point2f> group{{.20f, .50f}, {.32f, .56f}, {.26f, .44f}};
        auto one = run(group, true, true, false, 1, 2400);
        assert(one.starts == 1 && one.align && one.swing && one.done && !one.budget);
        assert(std::max(-one.min_phi, one.max_phi) >= .35f);
        assert(one.rushes >= 2);
        assert(one.scans_after > 0);
        assert(one.pitch_after == 500); // 默认无20°档时保持5°
        {
            TaskTuning tp = t;
            tp.track_pitch_cdeg = 4000;
            tp.intermediate_pitch_cdeg = 2000;
            const auto pa = [&](bool mid) {
                TaskTuning tm = tp;
                tm.post_clear_mid_view = mid;
                tm.blind_scatter_max = 1;
                Sim s(tm);
                s.in.target_valid = false;
                s.in.distance_m = .9f;
                const float x0 = s.px, y0 = s.py;
                int pitch = -1;
                bool done = false;
                const std::vector<cv::Point2f> world{
                    {x0 + .20f, y0 + .50f}, {x0 + .32f, y0 + .56f}, {x0 + .26f, y0 + .44f}};
                for (int i = 0; i < 2400 && pitch < 0; ++i) {
                    s.in.clutter_body.clear();
                    const float c = std::cos(s.phi), sn = std::sin(s.phi);
                    for (const auto& w : world) {
                        const float dx = w.x - s.px, dy = w.y - s.py;
                        s.in.clutter_body.push_back({dx * c + dy * sn, -dx * sn + dy * c});
                    }
                    s.in.clutter_ts_us = s.in.now_us;
                    s.tick();
                    if (s.out.reason == "blind_scatter_done")
                        done = true;
                    if (done && s.out.state == PushState::SCAN)
                        pitch = s.out.motion.camera_pitch_cdeg;
                }
                return pitch;
            };
            assert(pa(true) == 2000);
            assert(pa(false) == 500);
        }
        auto again = run(group, true, true, false, 3, 6000, true);
        assert(again.starts >= 2 && again.starts <= 3);
        auto lone = run({{.20f, .50f}}, true, true, false, 1, 2400);
        assert(lone.starts == 1 && lone.align && lone.swing && lone.done && !lone.budget &&
               lone.rushes >= 2);
        auto wideL = run({{-.70f, 1.05f}}, true, true, false, 1, 2400);
        assert(wideL.starts == 1 && wideL.done && !wideL.budget &&
               std::max(-wideL.min_phi, wideL.max_phi) >= .35f);
        auto far = run({{.0f, 2.2f}, {.1f, 2.25f}}, true, true, false, 3, 600);
        assert(far.starts == 0);
        auto side = run({{1.1f, .8f}}, true, true, false, 3, 600, true);
        assert(side.starts == 0);
        auto stale = run(group, true, false, false, 3, 600);
        assert(stale.starts == 0);
        auto off = run(group, false, true, false, 3, 600);
        assert(off.starts == 0);
        auto have = run(group, true, true, true, 3, 200);
        assert(have.starts == 0);
        assert(one.back); // backs out after the swings
        for (
            const float frac :
            {1.f,
             .35f}) { // 退出距离=前向行程*blind_scatter_back_frac：1.0回到起点附近；默认只退一部分，不把推开的物块甩远。
            TaskTuning tb = t;
            tb.blind_scatter_back_budget_us = 30000000;
            tb.blind_scatter_back_frac = frac;
            Sim s(tb);
            s.in.target_valid = false;
            s.in.distance_m = .9f;
            const float x0 = s.px, y0 = s.py;
            float sx = 0, sy = 0, max_d = 0;
            bool back = false, started = false;
            std::vector<cv::Point2f> world{
                {x0 + .0f, y0 + .60f}, {x0 + .06f, y0 + .64f}, {x0 - .05f, y0 + .56f}};
            for (int i = 0; i < 2400; ++i) {
                s.in.clutter_body.clear();
                const float c = std::cos(s.phi), sn = std::sin(s.phi);
                for (const auto& w : world) {
                    const float dx = w.x - s.px, dy = w.y - s.py;
                    s.in.clutter_body.push_back({dx * c + dy * sn, -dx * sn + dy * c});
                }
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                if (s.out.reason == "blind_scatter_start") {
                    started = true;
                    sx = s.px;
                    sy = s.py;
                }
                if (started)
                    max_d = std::max(max_d, std::hypot(s.px - sx, s.py - sy));
                if (s.out.reason == "blind_scatter_back")
                    back = true;
                if (started && s.out.reason == "blind_scatter_done")
                    break;
            }
            assert(started && back && max_d > .3f);
            const float left = std::hypot(s.px - sx, s.py - sy);
            if (frac >= 1.f)
                assert(left < .4f * max_d);
            else
                assert(left > .45f * max_d);
        }
        { // 默认参数：单个/两个非目标物体不打散；成团(>=3)在转够一圈前也不打散；偏出±40°的团不打散。
            TaskTuning td;
            td.enable_search_cues = false;
            const auto starts = [&](std::vector<cv::Point2f> pts, float first_turn, int ticks) {
                TaskTuning tt = td;
                tt.blind_scatter_first_turn_rad = first_turn;
                Sim s(tt);
                s.in.target_valid = false;
                s.in.distance_m = .9f;
                int n = 0;
                for (int i = 0; i < ticks; ++i) {
                    s.in.clutter_body = pts;
                    s.in.clutter_ts_us = s.in.now_us;
                    s.tick();
                    if (s.out.reason == "blind_scatter_start")
                        ++n;
                }
                return n;
            };
            assert(starts({{.20f, .50f}}, 0.f, 300) == 0);
            assert(starts({{.20f, .50f}, {.26f, .44f}}, 0.f, 300) == 0);
            assert(starts(group, 0.f, 300) == 1);
            assert(starts(group, 1000.f, 300) == 0);
            assert(starts({{.95f * .8f, .40f}, {.78f, .46f}, {.74f, .36f}}, 0.f, 300) == 0);
            const auto startsMix = [&](int n_ord) {
                Sim s(td);
                s.all_ordinary = false;
                s.in.target_valid = false;
                s.in.distance_m = .9f;
                int n = 0;
                for (int i = 0; i < 300; ++i) {
                    s.in.clutter_body = group;
                    s.in.clutter_ordinary.assign(group.begin(), group.begin() + n_ord);
                    s.in.clutter_ts_us = s.in.now_us;
                    s.tick();
                    if (s.out.reason == "blind_scatter_start")
                        ++n;
                }
                return n;
            };
            assert(startsMix(0) == 0); // injured / dangerous only: no sweep
            assert(startsMix(1) == 1);
        } // one ordinary inside the cluster is enough
    }
    { // 普通物资只有很少像素（远/小）：不追，较短时间后直接冲入打散；像素足够大则正常锁定；太远不触发；对准途中看到像素足够的才中止。
        TaskTuning t;
        t.enable_search_cues = false;
        const auto mk = [&](float ratio, float dist, Sim& s) {
            s.in.target_valid = true;
            s.in.distance_m = dist;
            s.in.pixel_mode = "box_area";
            s.in.pixel_ratio_smoothed = s.in.pixel_ratio_raw = ratio;
        };
        const auto first = [&](Sim& s, const char* why, int ticks) {
            for (int i = 0; i < ticks; ++i) {
                s.tick();
                if (s.out.reason == why)
                    return i;
            }
            return -1;
        };
        {
            Sim s(t);
            mk(.002f, .9f, s);
            assert(first(s, "blind_scatter_start", 400) >= 0);
            assert(first(s, "blind_scatter_done", 3000) >= 0);
        }
        {
            Sim s(t);
            mk(.006f, .9f, s);
            assert(first(s, "blind_scatter_start", 300) < 0 && s.out.state == PushState::APPROACH);
        }
        {
            Sim s(t);
            mk(.002f, 1.8f, s);
            assert(first(s, "blind_scatter_start", 300) < 0);
        }
        {
            TaskTuning tw = t;
            tw.weak_target_scatter = false;
            Sim s(tw);
            mk(.002f, .9f, s);
            assert(first(s, "blind_scatter_start", 300) < 0);
        }
        {
            Sim s(t);
            mk(.002f, .9f, s);
            s.in.heading_error = .6f;
            assert(first(s, "blind_scatter_start", 400) >= 0);
            s.tick();
            assert(s.out.state == PushState::SCATTER && s.out.reason == "blind_scatter_align");
            mk(.006f, .9f, s);
            assert(first(s, "blind_scatter_target_seen", 200) >= 0);
        }
    }
    { // 很远(>0.8 m)的锁定不可信：不论群还是单个锁定都直接前驱(不被远处的锁定打断)；<=0.8
      // m恢复原决策。
        TaskTuning t;
        t.enable_search_cues = false;
        t.blind_scatter_max_y_m = .5f;
        t.blind_scatter_far_max_y_m = 4.f;
        t.blind_scatter_half_x_m = 2.5f;
        t.blind_scatter_first_turn_rad = 0.f;
        t.sweep_confirm_wait_us =
            0; // legacy: the cluster decision is not held back by a confirming lock
        struct R {
            bool group = false, far_start = false, approach = false, seen = false, chase = false;
        };
        const auto run = [&](bool locked, float dist, std::vector<cv::Point2f> pts, int ticks) {
            Sim s(t);
            s.in.target_valid = locked;
            s.in.distance_m = dist;
            s.in.pixel_mode = "box_area";
            s.in.pixel_ratio_smoothed = s.in.pixel_ratio_raw = .006f;
            R r;
            for (int i = 0; i < ticks; ++i) {
                s.in.clutter_body = pts;
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                if (s.out.reason == "blind_scatter_far_group")
                    r.group = true;
                if (s.out.reason == "blind_scatter_far_start")
                    r.far_start = true;
                if (s.out.reason == "blind_scatter_approach")
                    r.approach = true;
                if (s.out.reason == "blind_scatter_target_seen")
                    r.seen = true;
                if (s.out.state == PushState::APPROACH)
                    r.chase = true;
            }
            return r;
        };
        const std::vector<cv::Point2f> grp{{.20f, 2.30f}, {-.25f, 2.60f}};
        {
            const R r = run(true, 2.5f, grp, 400);
            assert(r.group && r.approach && !r.seen);
        }
        {
            const R r = run(true, 2.5f, {}, 400);
            assert(r.group && r.approach && !r.seen);
        }
        {
            const R r = run(true, .7f, {{.20f, .60f}, {-.25f, .80f}}, 400);
            assert(!r.group && !r.approach);
        }
        {
            const R r = run(false, 2.5f, {{0.f, 2.5f}, {.20f, 2.30f}, {-.25f, 2.60f}}, 600);
            assert(r.far_start && r.approach);
        }
        {
            const R r = run(false, 2.5f, {{0.f, 2.5f}, {1.5f, 2.30f}, {-1.2f, 2.60f}}, 600);
            assert(!r.far_start && !r.group);
        }
    }
    { // 单个远处锁定(世界固定，杂物视图过期)：直接前驱，锁定进入0.5 m窗口后交给正常APPROACH，不扫。
        TaskTuning t;
        t.enable_search_cues = false;
        t.blind_scatter_max_y_m = .5f;
        t.blind_scatter_far_max_y_m = 4.f;
        t.blind_scatter_half_x_m = 2.5f;
        t.blind_scatter_first_turn_rad = 0.f;
        t.sweep_confirm_wait_us = 0;
        Sim s(t);
        s.in.pixel_mode = "box_area";
        s.in.pixel_ratio_smoothed = s.in.pixel_ratio_raw = .006f;
        const float x0 = s.px, y0 = s.py;
        const cv::Point2f w{x0 + .05f, y0 + 1.90f};
        bool far = false, chase = false, near_group = false;
        for (int i = 0; i < 3000 && !chase; ++i) {
            const float c = std::cos(s.phi), sn = std::sin(s.phi);
            const float dx = w.x - s.px, dy = w.y - s.py;
            const cv::Point2f lk{dx * c + dy * sn, -dx * sn + dy * c};
            s.in.clutter_body.clear();
            s.in.target_valid = lk.y > .1f;
            s.in.distance_m = std::hypot(lk.x, lk.y);
            s.in.heading_error = std::atan2(-lk.x, lk.y);
            s.tick();
            if (s.out.reason == "blind_scatter_far_group")
                far = true;
            if (s.out.reason == "blind_scatter_near_group")
                near_group = true;
            if (s.out.state == PushState::APPROACH)
                chase = true;
        }
        assert(far && chase && !near_group);
    }
    { // 前驱途中群里的普通物资锁定不打断：群进入0.5 m窗口后直接扫(near_group)，不回单目标追踪。
        TaskTuning t;
        t.enable_search_cues = false;
        t.blind_scatter_max_y_m = .5f;
        t.blind_scatter_far_max_y_m = 4.f;
        t.blind_scatter_half_x_m = 2.5f;
        t.blind_scatter_first_turn_rad = 0.f;
        t.sweep_confirm_wait_us = 0;
        Sim s(t);
        s.in.pixel_mode = "box_area";
        s.in.pixel_ratio_smoothed = s.in.pixel_ratio_raw = .006f;
        const float x0 = s.px, y0 = s.py;
        const std::vector<cv::Point2f> world{
            {x0 + .0f, y0 + 1.90f}, {x0 + .08f, y0 + 1.98f}, {x0 - .07f, y0 + 1.84f}};
        bool far = false, near_group = false, seen = false, chase = false, done = false;
        for (int i = 0; i < 3000 && !done; ++i) {
            const float c = std::cos(s.phi), sn = std::sin(s.phi);
            s.in.clutter_body.clear();
            for (const auto& w : world) {
                const float dx = w.x - s.px, dy = w.y - s.py;
                s.in.clutter_body.push_back({dx * c + dy * sn, -dx * sn + dy * c});
            }
            const cv::Point2f lk = s.in.clutter_body[0];
            s.in.target_valid = lk.y > .1f;
            s.in.distance_m = std::hypot(lk.x, lk.y);
            s.in.heading_error = std::atan2(-lk.x, lk.y);
            s.in.clutter_ts_us = s.in.now_us;
            s.tick();
            if (s.out.reason == "blind_scatter_far_group" ||
                s.out.reason == "blind_scatter_far_start")
                far = true;
            if (s.out.reason == "blind_scatter_near_group")
                near_group = true;
            if (!near_group) {
                if (s.out.reason == "blind_scatter_target_seen")
                    seen = true;
                if (s.out.state == PushState::APPROACH)
                    chase = true;
            }
            if (near_group && s.out.reason == "blind_scatter_done")
                done = true;
        }
        assert(far && near_group && !seen && !chase && done);
    }
    { // 开局首次远距离接近(整场一次)：首个远群前驱用高速、剩余0.35
      // m降回常态；随后首次近距打散用高转速摆框；之后的远驱/打散都是常态；总开关关闭则全程常态。
        TaskTuning t;
        t.enable_search_cues = false;
        t.blind_scatter_max_y_m = .5f;
        t.blind_scatter_far_max_y_m = 4.f;
        t.blind_scatter_half_x_m = 2.5f;
        t.blind_scatter_first_turn_rad = 0.f;
        t.first_burst = true;
        t.first_burst_speed = 1.f;
        t.first_burst_swing_wz = 5.f;
        t.assume_motion_ratio = 1.f;
        t.sweep_gap_us = 0;
        t.sweep_escalate_max = 0;
        t.blind_scatter_cooldown_us = 0;
        struct B {
            bool fast = false, slow = false, burst_swing = false, plain_swing = false, done = false,
                 impact = false;
            float fast_vx = 0, slow_vx = 0, burst_wz = 0, plain_wz = 0, impact_min = 10.f,
                  impact_max = 0.f;
        };
        const auto runGroup = [&](Sim& s) {
            const float x0 = s.px, y0 = s.py;
            const std::vector<cv::Point2f> world{
                {x0 + .0f, y0 + 1.90f}, {x0 + .08f, y0 + 1.98f}, {x0 - .07f, y0 + 1.84f}};
            B b;
            bool scattered = false;
            for (int i = 0; i < 3000 && !b.done; ++i) {
                const float c = std::cos(s.phi), sn = std::sin(s.phi);
                s.in.clutter_body.clear();
                for (const auto& w : world) {
                    const float dx = w.x - s.px, dy = w.y - s.py;
                    s.in.clutter_body.push_back({dx * c + dy * sn, -dx * sn + dy * c});
                }
                const cv::Point2f lk = s.in.clutter_body[0];
                s.in.target_valid = lk.y > .1f;
                s.in.distance_m = std::hypot(lk.x, lk.y);
                s.in.heading_error = std::atan2(-lk.x, lk.y);
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                const std::string& r = s.out.reason;
                if (r == "first_burst_approach") {
                    b.fast = true;
                    b.fast_vx = std::max(b.fast_vx, s.out.motion.vx_mps);
                }
                if (r == "first_burst_impact" && s.out.motion.vx_mps > 0) {
                    b.impact = true;
                    b.impact_min = std::min(b.impact_min, s.out.motion.vx_mps);
                    b.impact_max = std::max(b.impact_max, s.out.motion.vx_mps);
                }
                if (r == "blind_scatter_approach") {
                    b.slow = true;
                    b.slow_vx = std::max(b.slow_vx, s.out.motion.vx_mps);
                }
                if (r == "first_burst_swing") {
                    b.burst_swing = true;
                    b.burst_wz = std::max(b.burst_wz, std::abs(s.out.motion.wz_rps));
                }
                if (r == "disperse_swing") {
                    b.plain_swing = true;
                    b.plain_wz = std::max(b.plain_wz, std::abs(s.out.motion.wz_rps));
                }
                if (s.out.state == PushState::SCATTER)
                    scattered = true;
                if (scattered && r == "blind_scatter_done")
                    b.done = true; // SCAN keeps the previous end reason until it overwrites it
            }
            return b;
        };
        const auto newSim = [&](TaskTuning tt) {
            Sim s(tt);
            s.in.pixel_mode = "box_area";
            s.in.pixel_ratio_smoothed = s.in.pixel_ratio_raw = .006f;
            return s;
        };
        {
            Sim s = newSim(t);
            const B a = runGroup(s);
            assert(a.done && a.fast && a.fast_vx == 1.f && a.slow && a.slow_vx > 0 &&
                   a.slow_vx <= .4f + 1e-4f); // 先高速，后段降回常态
            assert(a.burst_swing && a.burst_wz == 5.f && !a.plain_swing); // 首次打散整段高转速
            const B b = runGroup(s); // 第二个群：整场只一次
            assert(b.done && !b.fast && !b.burst_swing && b.plain_swing &&
                   b.plain_wz == t.disperse_wz && b.slow_vx <= .4f + 1e-4f);
        }
        { // Requested opening: keep full speed through near impact, only once; normal sweeps do not
          // inherit it.
            TaskTuning full = t;
            full.first_burst_full_impact = true;
            full.sweep_observation_guard = true;
            Sim s = newSim(full);
            const B a = runGroup(s);
            assert(a.done && a.fast && !a.slow && a.impact && a.impact_min == 1.f &&
                   a.impact_max == 1.f && a.burst_swing);
            const B b = runGroup(s);
            assert(b.done && !b.fast && !b.impact && !b.burst_swing && b.plain_swing);
        }
        { // Speed override must not undo a forward-clearance veto.
            TaskTuning full = t;
            full.first_burst_full_impact = true;
            Sim s = newSim(full);
            s.in.target_valid = false;
            assert(s.until(PushState::SCAN, 30));
            for (int i = 0; i < 20; ++i)
                s.tick();
            s.task.enter(PushState::SCATTER, s.in.now_us, "test_impact");
            s.task.search_phase_.sweep_phase_ = 4;
            s.task.search_phase_.burst_swing_ = true;
            s.task.search_phase_.disperse_rush_m_ = .5f;
            s.task.search_phase_.sweep_start_us_ = s.in.now_us;
            s.in.path_safe = false;
            s.tick();
            assert(stopped(s.out.motion));
        }
        {
            TaskTuning off = t;
            off.first_burst = false;
            Sim s = newSim(off);
            const B a = runGroup(s);
            assert(a.done && !a.fast && !a.burst_swing && a.plain_swing &&
                   a.plain_wz == off.disperse_wz);
        }
        { // 驱动距离不足first_burst_min_drive_m：不算首次远驱，不消耗；打散也用常态。
            TaskTuning tm = t;
            tm.first_burst_min_drive_m = 9.f;
            Sim s = newSim(tm);
            const B a = runGroup(s);
            assert(a.done && !a.fast && !a.burst_swing && a.plain_swing);
        }
        { // 远处少像素锁定反复靠近(approach_max次)：只有第一次靠近是高速。
            const auto mk = [&](float ratio, float dist, Sim& s) {
                s.in.target_valid = true;
                s.in.distance_m = dist;
                s.in.pixel_mode = "box_area";
                s.in.pixel_ratio_smoothed = s.in.pixel_ratio_raw = ratio;
            };
            Sim s(t);
            mk(.002f, 1.1f, s);
            int fast_stints = 0, slow_stints = 0;
            std::string prev;
            float max_slow = 0;
            for (int i = 0; i < 4000; ++i) {
                s.tick();
                const std::string& r = s.out.reason;
                if (r == "first_burst_approach" && prev != r)
                    ++fast_stints;
                if (r == "blind_scatter_approach") {
                    max_slow = std::max(max_slow, s.out.motion.vx_mps);
                    if (prev != r && fast_stints >= 1)
                        ++slow_stints;
                }
                prev = r;
            }
            assert(fast_stints == 1 && max_slow <= .4f + 1e-4f);
        }
        { // 普通锁定先出现(没有远群前驱)：不触发高速接近；之后的首个近距打散也是常态转速。
            Sim s = newSim(t);
            s.in.target_valid = true;
            s.in.distance_m = .7f;
            bool burst = false;
            for (int i = 0; i < 200; ++i) {
                s.tick();
                if (s.out.reason == "first_burst_approach" || s.out.reason == "first_burst_swing")
                    burst = true;
            }
            assert(!burst);
        }
    }
    { // 扫除台账：扫完后20 s间隔内不再扫；每扫完一次所需密度+1(上限+2)；每趟总次数有上限。
        TaskTuning t;
        t.enable_search_cues = false;
        t.blind_scatter_first_turn_rad = 0.f;
        t.blind_scatter_max = 10;
        const auto sweeps = [&](std::vector<cv::Point2f> pts,
                                TaskTuning tt,
                                int ticks,
                                std::vector<int>* at = nullptr) {
            Sim s(tt);
            s.in.target_valid = false;
            s.in.distance_m = .9f;
            int n = 0;
            for (int i = 0; i < ticks; ++i) {
                s.in.clutter_body = pts;
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                if (s.out.reason == "blind_scatter_start") {
                    ++n;
                    if (at)
                        at->push_back(i);
                }
            }
            return n;
        };
        const std::vector<cv::Point2f> g3{{.05f, .30f}, {-.10f, .38f}, {.12f, .42f}};
        const std::vector<cv::Point2f> g5{
            {.05f, .30f}, {-.10f, .38f}, {.12f, .42f}, {.00f, .35f}, {.08f, .36f}};
        {
            TaskTuning off = t;
            off.sweep_gap_us = 0;
            off.sweep_escalate_max = 0;
            off.sweep_total_max = 100;
            off.sweep_time_budget_us = 1ull << 60;
            assert(sweeps(g3, off, 6000) >= 2);
        } // 无台账：同一堆被反复扫
        assert(sweeps(g3, t, 6000) == 1); // 有台账：第二次要求4个，3个的堆不再扫
        {
            std::vector<int> at;
            const int n = sweeps(g5, t, 6000, &at);
            assert(n >= 2);
            assert((at[1] - at[0]) * 100 >= 20000 + 0);
        } // 间隔>=20 s(含扫动本身)
        {
            TaskTuning one = t;
            one.sweep_total_max = 1;
            one.sweep_escalate_max = 0;
            assert(sweeps(g5, one, 6000) == 1);
        }
    }
    { // 追踪丢失时群还在视野里：直接扫(lost_group_sweep)，不进LOST_SEARCH原地转圈；没有群则照常进LOST_SEARCH，转圈中看到群也立刻扫。
        TaskTuning t;
        t.enable_search_cues = false;
        t.enable_lost_search = true;
        t.blind_scatter_max_y_m = .5f;
        t.blind_scatter_first_turn_rad = 0.f;
        const std::vector<cv::Point2f> grp{{.05f, .30f}, {-.10f, .38f}, {.12f, .42f}};
        const auto toApproach = [&](Sim& s) {
            s.in.target_valid = true;
            s.in.distance_m = .7f;
            s.in.clutter_body.clear();
            for (int i = 0; i < 300; ++i) {
                s.tick();
                if (s.out.state == PushState::APPROACH)
                    return true;
            }
            return false;
        };
        {
            Sim s(t);
            assert(toApproach(s));
            s.in.target_valid = false;
            bool sweep = false, lost = false;
            for (int i = 0; i < 80; ++i) {
                s.in.clutter_body = grp;
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                if (s.out.reason == "lost_group_sweep")
                    sweep = true;
                if (s.out.state == PushState::LOST_SEARCH)
                    lost = true;
            }
            assert(sweep && !lost);
        }
        {
            Sim s(t);
            assert(toApproach(s));
            s.in.target_valid = false;
            bool lost = false;
            for (int i = 0; i < 80; ++i) {
                s.in.clutter_body.clear();
                s.tick();
                if (s.out.state == PushState::LOST_SEARCH)
                    lost = true;
            }
            assert(lost);
            bool sweep = false;
            for (int i = 0; i < 40; ++i) {
                s.in.clutter_body = grp;
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                if (s.out.reason == "lost_group_sweep")
                    sweep = true;
            }
            assert(sweep && s.out.state == PushState::SCATTER);
        }
    }
    { // 远处(0.5~1.4
      // m)的少像素普通物资：先低速靠近，再从近距离触发冲入打散；不超过常态速度；靠近次数有上限。
        TaskTuning t;
        t.enable_search_cues = false;
        t.blind_scatter_max_y_m = .5f;
        t.max_speed = .4f;
        t.approach_speed = .4f;
        const auto mk = [&](float ratio, float dist, Sim& s) {
            s.in.target_valid = true;
            s.in.distance_m = dist;
            s.in.pixel_mode = "box_area";
            s.in.pixel_ratio_smoothed = s.in.pixel_ratio_raw = ratio;
        };
        {
            Sim s(t);
            mk(.002f, 1.1f, s);
            bool far_start = false, approach = false, approached = false, start = false;
            float max_vx = 0;
            for (int i = 0; i < 3000 && !approached; ++i) {
                s.tick();
                if (s.out.reason == "blind_scatter_far_start")
                    far_start = true;
                if (s.out.reason == "blind_scatter_approach") {
                    approach = true;
                    max_vx = std::max(max_vx, s.out.motion.vx_mps);
                }
                if (s.out.reason == "blind_scatter_start")
                    start = true;
                if (s.out.reason == "blind_scatter_approached")
                    approached = true;
            }
            assert(far_start && approach && approached && !start && max_vx > 0 &&
                   max_vx <= .4f + 1e-4f && s.out.state == PushState::SCAN);
            mk(.002f, .45f, s);
            for (int i = 0; i < 400 && !start; ++i) {
                s.tick();
                if (s.out.reason == "blind_scatter_start")
                    start = true;
            }
            assert(start);
        }
        {
            TaskTuning tn = t;
            tn.blind_scatter_approach_max = 0;
            Sim s(tn);
            mk(.002f, 1.1f, s);
            for (int i = 0; i < 600; ++i) {
                s.tick();
                assert(s.out.reason != "blind_scatter_far_start" &&
                       s.out.reason != "blind_scatter_approach");
            }
        }
    }
    { // Deferred startup never commands motion even with legacy high-speed settings.
        TaskTuning t;
        t.startup_advance_us = 3000000;
        t.startup_advance_speed = 1.f;
        t.max_speed = .4f;
        PushTask task(t);
        PushObservation in;
        in.run = in.safety_ok = in.path_safe = in.opponent_zone_clear = true;
        in.camera_pitch_cdeg = 500;
        in.camera_pitch_stable = true;
        bool fast = false;
        for (int i = 0; i < 200; ++i) {
            in.now_us += 100000;
            const auto out = task.update(in);
            if (out.state == PushState::START_ADVANCE && out.motion.vx_mps > 0) {
                assert(out.motion.vx_mps == 1.f);
                fast = true;
            } else
                assert(std::abs(out.motion.vx_mps) <= .4f + 1e-4f);
        }
        assert(!fast);
    }
    { // 扫掠通道内有竖放伤员：低框原地转向扫开（IMU量角），转回后再进入PREPARE；不在通道内/过期/就是目标本身则不动。
        TaskTuning t;
        t.enable_search_cues = false;
        t.assume_all_safe = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.track_near_m = .4f;
        t.intermediate_to_near_m = .27f;
        t.rush_start_m = .35f;
        struct R {
            int outs = 0, backs = 0, turn_signs = 0;
            bool prepared = false, left_approach_early = false;
            float first_wz = 0, max_phi = 0;
        };
        const auto run = [&](float ux, float uy, bool fresh, bool persist, int ticks) {
            Sim s(t);
            s.in.distance_m = .5f;
            assert(s.until(PushState::APPROACH, 80));
            s.in.distance_m = .19f;
            R r;
            bool blocker = true;
            for (int i = 0; i < ticks && !r.prepared; ++i) {
                if (blocker) {
                    s.in.upright_body = {{ux, uy}};
                    if (fresh)
                        s.in.upright_ts_us = s.in.now_us;
                }
                s.tick();
                r.max_phi = std::max(r.max_phi, std::abs(s.phi));
                if (s.out.state == PushState::PREPARE) {
                    r.prepared = true;
                    break;
                }
                if (s.out.state != PushState::APPROACH) {
                    r.left_approach_early = true;
                    break;
                }
                if (s.out.reason == "sweep_upright_out" && s.out.motion.wz_rps != 0) {
                    ++r.outs;
                    if (!r.first_wz)
                        r.first_wz = s.out.motion.wz_rps;
                    assert(s.out.motion.vx_mps == 0 && s.out.motion.gripper_offset == 20);
                }
                if (s.out.reason == "sweep_upright_back" && s.out.motion.wz_rps != 0)
                    ++r.backs;
                if (!persist && r.backs > 0)
                    blocker = false;
            }
            r.turn_signs = int(std::abs(s.phi) * 1000);
            return r;
        };
        auto right = run(.10f, .19f, true, false, 200);
        assert(right.outs > 0 && right.backs > 0 && right.first_wz < 0 && right.max_phi >= .8f &&
               right.max_phi <= 1.1f && right.prepared && !right.left_approach_early);
        assert(right.turn_signs < 120); // returned to the original heading before PREPARE
        auto left = run(-.10f, .19f, true, false, 200);
        assert(left.outs > 0 && left.first_wz > 0 && left.prepared);
        auto wide = run(.22f, .19f, true, false, 200); // outer part of the widened corridor
        assert(wide.outs > 0 && wide.first_wz < 0 && wide.prepared);
        auto far = run(.40f, .19f, true, false, 200);
        assert(far.outs == 0 && far.prepared);
        auto behind = run(.10f, .45f, true, false, 200); // beyond target + extra reach
        assert(behind.outs == 0 && behind.prepared);
        auto self = run(.02f, .19f, true, false, 200); // same physical object as the target
        assert(self.outs == 0 && self.prepared);
        auto stale = run(.10f, .19f, false, false, 200);
        assert(stale.outs == 0 && stale.prepared);
        { // sweep time is not charged to the approach budget: no approach_timeout back to SCAN, and
          // the sweep turns at sweep_wz
            TaskTuning b = t;
            b.approach_budget_us = 1500000;
            Sim s(b);
            s.in.distance_m = .5f;
            assert(s.until(PushState::APPROACH, 80));
            s.in.distance_m = .19f;
            bool swept = false, prepared = false;
            float minwz = 9;
            for (int i = 0; i < 200 && !prepared; ++i) {
                if (!swept)
                    s.in.upright_body = {{.10f, .19f}}, s.in.upright_ts_us = s.in.now_us;
                s.tick();
                if (s.out.reason == "sweep_upright_out" || s.out.reason == "sweep_upright_back") {
                    swept = true;
                    minwz = std::min(minwz, std::abs(s.out.motion.wz_rps));
                }
                if (s.out.reason == "sweep_upright_done")
                    s.in.upright_body.clear();
                assert(s.out.state == PushState::APPROACH || s.out.state == PushState::PREPARE);
                prepared = s.out.state == PushState::PREPARE;
            }
            assert(swept && prepared && minwz >= b.sweep_wz - 1e-4f);
        }
        auto stuck =
            run(.10f, .19f, true, true, 600); // block never leaves: two attempts, then carry on
        assert(stuck.outs > 0 && stuck.prepared && stuck.left_approach_early == false);
    }
}

void corridorSweepCases() {
    { // 走廊邻居清理：走廊判定不合法且目标前方走廊内有任意类别邻居 ->
      // 低框原地扫开再回正；开关关闭/走廊合法/邻居在目标后方或带外/数据过期则不动。
        const auto run = [&](bool on, Inventory corridor, float ux, float uy, bool fresh) {
            TaskTuning t;
            t.enable_search_cues = false;
            t.track_pitch_cdeg = 4000;
            t.intermediate_pitch_cdeg = 2000;
            t.track_near_m = .4f;
            t.intermediate_to_near_m = .27f;
            t.rush_start_m = .35f;
            t.clear_neighbours = on;
            Sim s(t);
            s.in.distance_m = .5f;
            assert(s.until(PushState::APPROACH, 80));
            s.in.distance_m = .19f;
            s.in.corridor = corridor;
            int outs = 0, backs = 0;
            float first_wz = 0;
            bool left_sweep = false;
            for (int i = 0; i < 200; ++i) {
                s.in.clutter_body = {{ux, uy}};
                if (fresh)
                    s.in.clutter_ts_us = s.in.now_us;
                if (backs > 0)
                    s.in.corridor = inv(1);
                s.tick();
                if (s.out.state != PushState::APPROACH) {
                    left_sweep = true;
                    break;
                }
                if (s.out.reason == "sweep_neighbour_out" && s.out.motion.wz_rps != 0) {
                    ++outs;
                    if (!first_wz)
                        first_wz = s.out.motion.wz_rps;
                    assert(s.out.motion.vx_mps == 0);
                }
                if (s.out.reason == "sweep_upright_back" && s.out.motion.wz_rps != 0)
                    ++backs;
            }
            struct R {
                int outs, backs;
                float wz;
                bool left;
            };
            return R{outs, backs, first_wz, left_sweep};
        };
        auto many = run(true, inv(3), .08f, .10f, true); // 目标前方偏右的普通物资，走廊内共3个
        assert(many.outs > 0 && many.backs > 0 && many.wz < 0 && many.left);
        auto leftside = run(true, inv(3), -.08f, .10f, true);
        assert(leftside.outs > 0 && leftside.wz > 0);
        auto danger = run(true, inv(1, 0, 0, 1), .06f, .10f, true); // 走廊内有危险物：同样只扫不搬
        assert(danger.outs > 0 && danger.left);
        assert(run(false, inv(3), .08f, .10f, true).outs == 0); // 开关关闭
        assert(run(true, inv(2), .08f, .10f, true).outs == 0);  // 走廊合法
        assert(run(true, inv(3), .08f, .30f, true).outs == 0);  // 在目标后方
        assert(run(true, inv(3), .30f, .10f, true).outs == 0);  // 带外
        assert(run(true, inv(3), .08f, .10f, false).outs == 0); // 数据过期
    }
    { // 远处邻居（框前沿够不到）：先慢速前探到距邻居约.03m，再转出扫开；邻居已贴近则不前探。
        const auto run = [&](float uy) {
            TaskTuning t;
            t.enable_search_cues = false;
            t.track_pitch_cdeg = 4000;
            t.intermediate_pitch_cdeg = 2000;
            t.track_near_m = .4f;
            t.intermediate_to_near_m = .27f;
            t.rush_start_m = .35f;
            t.clear_neighbours = true;
            Sim s(t);
            s.in.distance_m = .5f;
            assert(s.until(PushState::APPROACH, 80));
            s.in.distance_m = .34f;
            s.in.corridor = inv(3);
            int creeps = 0, outs = 0;
            bool order_ok = true;
            for (int i = 0; i < 300; ++i) {
                s.in.clutter_body = {{.08f, uy}};
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                if (s.out.state != PushState::APPROACH)
                    break;
                if (s.out.reason == "sweep_creep" && s.out.motion.vx_mps > 0 &&
                    s.out.motion.wz_rps == 0)
                    ++creeps;
                if ((s.out.reason == "sweep_neighbour_out" ||
                     s.out.reason == "sweep_upright_out") &&
                    s.out.motion.wz_rps != 0) {
                    if (!creeps && uy > .22f)
                        order_ok = false;
                    ++outs;
                }
            }
            return std::make_tuple(creeps, outs, order_ok);
        };
        const auto far = run(.28f);
        assert(std::get<0>(far) > 0 && std::get<1>(far) > 0 && std::get<2>(far));
        assert(std::get<0>(run(.20f)) == 0);
    }
    { // 扫动阶段沿用搬运的视角分级：5° → 20°(剩余≤track_near_m) →
      // 40°(剩余≤intermediate_to_near_m)，只降不升；关闭开关则全程5°。
        TaskTuning t;
        t.enable_search_cues = false;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.track_near_m = .4f;
        t.intermediate_to_near_m = .27f;
        const auto mk = [&](float ratio, float dist, Sim& s) {
            s.in.target_valid = true;
            s.in.distance_m = dist;
            s.in.pixel_mode = "box_area";
            s.in.pixel_ratio_smoothed = s.in.pixel_ratio_raw = ratio;
        };
        const auto ladder = [&](bool handoff) {
            TaskTuning tt = t;
            tt.sweep_view_handoff = handoff;
            Sim s(tt);
            mk(.002f, .9f, s);
            std::vector<int> seq;
            for (int i = 0; i < 3000; ++i) {
                s.tick();
                if (s.out.state == PushState::SCATTER) {
                    const int p = s.out.motion.camera_pitch_cdeg;
                    if (seq.empty() || seq.back() != p)
                        seq.push_back(p);
                }
                if (s.out.reason == "blind_scatter_done")
                    break;
            }
            return seq;
        };
        const auto on = ladder(true);
        assert(on.size() >= 2 && on.front() == 500);
        for (size_t i = 1; i < on.size(); ++i)
            assert(on[i] > on[i - 1]);
        assert(std::find(on.begin(), on.end(), 2000) != on.end());
        const auto off = ladder(false);
        assert(off.size() == 1);
    }
    { // 锁定目标不是普通物资：不抓取，冲入摆扫后拉黑并回到扫描；普通物资单个仍正常抓取；开关关闭则按原流程。
        g_non_supply_sweep = true;
        TaskTuning t;
        t.enable_search_cues = false;
        t.assume_all_safe = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.track_near_m = .4f;
        t.intermediate_to_near_m = .27f;
        t.rush_start_m = .35f;
        const auto runOne = [&](const char* label, bool flag) {
            g_non_supply_sweep = flag;
            Sim s(t);
            s.task.first_ = true;
            s.in.label = label;
            s.in.distance_m = .5f;
            s.in.corridor = inv(0, 1);
            struct R {
                bool rush = false, swept = false, prepared = false;
                float max_wz = 0;
            } r;
            if (!s.until(PushState::APPROACH, 120) && !s.until(PushState::MID_APPROACH, 1)) {
                return r;
            }
            s.in.distance_m = .25f;
            for (int i = 0; i < 600; ++i) {
                s.tick();
                r.max_wz = std::max(r.max_wz, std::abs(s.out.motion.wz_rps));
                if (s.out.reason == "disperse_rush" || s.out.reason == "non_supply_rush")
                    r.rush = true;
                if (s.out.reason == "non_supply_swept") {
                    r.swept = true;
                    assert(s.out.state == PushState::SCAN);
                    break;
                }
                if (s.out.state == PushState::PREPARE || s.out.state == PushState::RUSH) {
                    r.prepared = true;
                    break;
                }
            }
            return r;
        };
        const auto core = runOne("core_supply", true);
        assert(core.rush && core.swept && !core.prepared && core.max_wz >= 1.5f);
        const auto inj = runOne("injured_person", true);
        assert(inj.rush && inj.swept && !inj.prepared);
        const auto ord = runOne("ordinary_supply", true);
        assert(!ord.rush && !ord.swept);
        const auto off = runOne("core_supply", false);
        assert(!off.rush && !off.swept);
        g_non_supply_sweep = false;
    }
    { // 清扫账本(Stage
      // A)：所有退出路径只记一次实际清扫时间；远驱+近扫算同一次；总预算约束执行中的清扫；确认等待有界；扫完不连扫。
        const std::vector<cv::Point2f> grp{{.20f, .50f}, {.32f, .56f}, {.26f, .44f}};
        TaskTuning base;
        base.enable_search_cues = false;
        base.blind_scatter_first_turn_rad = 0.f;
        base.assume_motion_ratio = 1.f;
        struct L {
            int starts = 0;
            bool done = false;
            std::string exit;
            uint64_t t_start = 0, t_end = 0;
            int done_cnt = 0;
            uint64_t spent = 0;
            bool open = true;
        };
        // 近距成团：持续喂同一个团，跑到第一次扫结束(任何退出)后再多跑extra个tick，检查账本不再变化。
        const auto near = [&](TaskTuning tt, int extra, int ticks) {
            Sim s(tt);
            s.in.target_valid = false;
            s.in.distance_m = .9f;
            L l;
            int after = -1;
            uint64_t spent_at_end = 0;
            for (int i = 0; i < ticks; ++i) {
                s.in.clutter_body = grp;
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                if (s.out.reason == "blind_scatter_start") {
                    ++l.starts;
                    if (l.starts == 1)
                        l.t_start = s.in.now_us;
                }
                if (l.starts >= 1 && !l.done && s.out.state != PushState::SCATTER) {
                    l.done = true;
                    l.exit = s.out.reason;
                    l.t_end = s.in.now_us;
                    spent_at_end = s.task.sweepSpentUs(s.in.now_us);
                    after = 0;
                }
                if (after >= 0) {
                    ++after;
                    if (after >= extra)
                        break;
                }
            }
            l.done_cnt = s.task.sweepDone();
            l.spent = s.task.sweepSpentUs(s.in.now_us);
            l.open = s.task.sweepChainOpen();
            assert(!l.done || l.spent == spent_at_end); // 退出后继续跑，账本不变(不重复计时)
            return l;
        };
        { // 完整近扫：计1次，用时≈实际时长，链已关闭。
            const L l = near(base, 200, 2400);
            assert(l.starts >= 1 && l.done && l.exit == "blind_scatter_done" && l.done_cnt == 1 &&
                   !l.open);
            const double dur = double(l.t_end - l.t_start) / 1e6, sp = double(l.spent) / 1e6;
            assert(sp >= dur - .2 && sp <= dur + .2);
        }
        { // 超时退出：用时记入但不计次数；sweep_gap(20
          // s)内不再起扫，间隔过后才允许下一次，且两次用时各记一次。
            TaskTuning t = base;
            t.blind_scatter_budget_us = 600000;
            Sim s(t);
            s.in.target_valid = false;
            s.in.distance_m = .9f;
            int starts = 0, starts_in_gap = 0;
            bool timeout = false;
            for (int i = 0; i < 600; ++i) {
                s.in.clutter_body = grp;
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                if (s.out.reason == "blind_scatter_start") {
                    ++starts;
                    if (i < 380)
                        ++starts_in_gap;
                }
                if (s.out.reason == "blind_scatter_timeout")
                    timeout = true;
                if (i == 379) {
                    assert(timeout && starts == 1 && s.task.sweepDone() == 0 &&
                           !s.task.sweepChainOpen());
                    const double sp = double(s.task.sweepSpentUs(s.in.now_us)) / 1e6;
                    assert(sp >= .5 && sp <= .9);
                }
            }
            assert(starts_in_gap == 1 && starts == 2 && s.task.sweepDone() == 0 &&
                   !s.task.sweepChainOpen());
            const double sp2 = double(s.task.sweepSpentUs(s.in.now_us)) / 1e6;
            assert(sp2 >= 1.0 && sp2 <= 1.8);
        }
        { // 总预算在清扫执行中生效：3
          // s预算切断近扫，用时不超过预算+1个tick，原因sweep_budget_exhausted。
            TaskTuning t = base;
            t.sweep_time_budget_us = 3000000;
            const L l = near(t, 20, 2400);
            assert(l.done && l.exit == "sweep_budget_exhausted" && !l.open);
            const double sp = double(l.spent) / 1e6;
            assert(sp >= 2.9 && sp <= 3.2);
            // 对照：默认预算下同一场景走完整扫且用时大于3 s
            const L d = near(base, 0, 2400);
            assert(d.exit == "blind_scatter_done" && double(d.spent) / 1e6 > 3.3);
        }
        { // 退场不安全：back阶段retreat_safe=false -> 立即结束且不下发倒车。
            Sim s(base);
            s.in.target_valid = false;
            s.in.distance_m = .9f;
            bool back = false, ended = false;
            float min_vx = 0;
            for (int i = 0; i < 2400 && !ended; ++i) {
                s.in.clutter_body = grp;
                s.in.clutter_ts_us = s.in.now_us;
                if (s.out.reason == "blind_scatter_back")
                    back = true;
                s.in.retreat_safe = !back;
                s.tick();
                if (!s.in.retreat_safe)
                    min_vx = std::min(min_vx, s.out.motion.vx_mps);
                if (back && s.out.reason == "blind_scatter_back_unsafe") {
                    ended = true;
                    assert(s.out.state == PushState::SCAN);
                }
            }
            assert(back && ended && min_vx >= -1e-4f && !s.task.sweepChainOpen());
        }
        // 远驱+近扫：世界固定的远群(约1.9 m)，同一条链。
        struct F {
            bool far = false, near_group = false, done = false;
            std::string exit;
            uint64_t t_far = 0, t_near = 0, t_end = 0, spent = 0;
            int done_cnt = 0;
            bool open = true;
        };
        const auto farChain = [&](TaskTuning tt) {
            Sim s(tt);
            s.in.pixel_mode = "box_area";
            s.in.pixel_ratio_smoothed = s.in.pixel_ratio_raw = .006f;
            const float x0 = s.px, y0 = s.py;
            const std::vector<cv::Point2f> world{
                {x0 + .0f, y0 + 1.90f}, {x0 + .08f, y0 + 1.98f}, {x0 - .07f, y0 + 1.84f}};
            F f;
            for (int i = 0; i < 3000 && !f.done; ++i) {
                const float c = std::cos(s.phi), sn = std::sin(s.phi);
                s.in.clutter_body.clear();
                for (const auto& w : world) {
                    const float dx = w.x - s.px, dy = w.y - s.py;
                    s.in.clutter_body.push_back({dx * c + dy * sn, -dx * sn + dy * c});
                }
                const cv::Point2f lk = s.in.clutter_body[0];
                s.in.target_valid = lk.y > .1f;
                s.in.distance_m = std::hypot(lk.x, lk.y);
                s.in.heading_error = std::atan2(-lk.x, lk.y);
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                if (!f.far && (s.out.reason == "blind_scatter_far_group" ||
                               s.out.reason == "blind_scatter_far_start")) {
                    f.far = true;
                    f.t_far = s.in.now_us;
                }
                if (!f.near_group && s.out.reason == "blind_scatter_near_group") {
                    f.near_group = true;
                    f.t_near = s.in.now_us;
                }
                if (f.far && s.out.state != PushState::SCATTER) {
                    f.done = true;
                    f.exit = s.out.reason;
                    f.t_end = s.in.now_us;
                }
            }
            f.spent = s.task.sweepSpentUs(s.in.now_us);
            f.done_cnt = s.task.sweepDone();
            f.open = s.task.sweepChainOpen();
            return f;
        };
        TaskTuning fc = base;
        fc.blind_scatter_max_y_m = .5f;
        fc.blind_scatter_far_max_y_m = 4.f;
        fc.blind_scatter_half_x_m = 2.5f;
        fc.sweep_confirm_wait_us = 0;
        F ref;
        { // 远驱->近扫算同一次：计1次，用时覆盖远驱起点到结束，而不是只有近扫段。
            ref = farChain(fc);
            assert(ref.far && ref.near_group && ref.done && ref.exit == "blind_scatter_done" &&
                   ref.done_cnt == 1 && !ref.open);
            const double whole = double(ref.t_end - ref.t_far) / 1e6,
                         near_only = double(ref.t_end - ref.t_near) / 1e6,
                         sp = double(ref.spent) / 1e6;
            assert(sp >= whole - .2 && sp <= whole + .2 && sp > near_only + .5);
        }
        { // 远->近切换不能重置整次作业期限：作业预算=远驱段+2.5
          // s，近扫被sweep_op_timeout切断，总时长≈预算。
            const double far_part = double(ref.t_near - ref.t_far) / 1e6;
            TaskTuning t = fc;
            t.sweep_op_budget_us = uint64_t((far_part + 2.5) * 1e6);
            const F f = farChain(t);
            assert(f.far && f.near_group && f.done && f.exit == "sweep_op_timeout" && !f.open);
            const double whole = double(f.t_end - f.t_far) / 1e6;
            assert(whole <= double(t.sweep_op_budget_us) / 1e6 + .2 && whole >= far_part + 2.3);
        }
        // 确认等待：普通物资锁定(2.5
        // m)+远群。默认等待内由3帧确认的单目标追踪优先于2帧群决策；等待为0则回到群追。
        struct Cf {
            bool group = false, approach = false, seen = false;
            uint64_t t_group = 0;
        };
        const auto confirm = [&](TaskTuning tt, bool flip, bool burst) {
            tt.first_burst = burst;
            Sim s(tt);
            s.in.pixel_mode = "box_area";
            s.in.pixel_ratio_smoothed = s.in.pixel_ratio_raw = .006f;
            s.in.target_valid = true;
            s.in.distance_m = 2.5f;
            Cf c;
            const std::vector<cv::Point2f> world{{0.f, 2.5f}, {.20f, 2.30f}, {-.25f, 2.60f}};
            for (int i = 0; i < 400 && !c.group && !c.approach; ++i) {
                s.in.clutter_body = world;
                s.in.clutter_ts_us = s.in.now_us;
                s.in.target_id = flip ? (i % 2 ? 7 : 8) : s.in.target_id;
                s.tick();
                if (s.out.reason == "blind_scatter_far_group") {
                    c.group = true;
                    c.t_group = s.in.now_us;
                }
                if (s.out.state == PushState::APPROACH)
                    c.approach = true;
                if (s.out.reason == "blind_scatter_target_seen")
                    c.seen = true;
            }
            return c;
        };
        TaskTuning ct = fc;
        ct.sweep_confirm_wait_us = 1200000;
        {
            const Cf c = confirm(ct, false, false);
            assert(c.approach && !c.group);
        }
        {
            TaskTuning z = ct;
            z.sweep_confirm_wait_us = 0;
            const Cf c = confirm(z, false, false);
            assert(c.group);
        }
        { // 目标ID来回跳(永远凑不够3帧)：等待有界，约1.2 s后仍交给群决策，不无限挂起。
            const Cf c = confirm(ct, true, false);
            assert(c.group && c.t_group >= 900000 && c.t_group <= 2500000);
        }
        { // 开局首次远驱(first_burst且未用过)不被确认等待拦住。
            const Cf c = confirm(ct, true, true);
            assert(c.group && c.t_group <= 500000);
        }
    }
}
