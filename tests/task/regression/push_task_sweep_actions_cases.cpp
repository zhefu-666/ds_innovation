#include "../../support/push_task_sim.hpp"
#include "push_task_cases.hpp"

void sweepFrameOpenTests() {
    for (int phase : {4, 5}) { // SCATTER rush / swing
        TaskTuning t;
        t.sweep_observation_guard = true;
        t.sweep_frame_open = true;
        t.disperse_swing_vx = 0;
        t.blind_scatter_passes = 1;
        Sim s(t);
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        assert(s.out.motion.gripper_offset == 20);
        s.task.enter(PushState::SCATTER, s.in.now_us, "test_sweep");
        s.task.search_phase_.sweep_phase_ = phase;
        s.task.sweep_ledger_.begin(s.task.t_, s.in.now_us);
        s.task.search_phase_.sweep_start_us_ = s.in.now_us;
        s.task.search_phase_.sweep_yaw0_ = s.phi;
        s.task.search_phase_.disperse_rush_m_ = .5f;
        s.task.search_phase_.disperse_swing_.configure(
            s.task.search_phase_.disperse_swing_.passLimit(), 2);
        s.in.clutter_body = {{0, .3f}, {.1f, .3f}, {-.1f, .3f}};
        auto step = [&] {
            s.in.clutter_ts_us = s.in.now_us;
            s.tick();
        };
        step();
        assert(stopped(s.out.motion) &&
               s.out.reason ==
                   "sweep_frame_open_wait"); // nothing moves until the open is acknowledged
        assert(s.out.motion.gripper_offset == 0);
        s.gripper_responds = false;
        step();
        assert(stopped(s.out.motion));
        s.gripper_responds = true; // no ack yet: still waiting
        step();
        assert(!stopped(s.out.motion) && s.out.motion.gripper_offset == 0);
        assert(s.task.frame_raised_ == 0); // cargo logic still sees a closed frame
        s.in.clutter_body.clear();
        s.in.clutter_ts_us = 0;
        s.in.target_valid = false; // the sweep ends by observation loss
        s.tick();
        assert(s.out.state == PushState::SCAN);
        s.in.target_valid = true;
        s.gripper_responds = false;
        s.tick();
        assert(stopped(s.out.motion) && s.out.reason == "sweep_frame_close_wait" &&
               s.out.motion.gripper_offset == 20);
        s.gripper_responds = true;
        s.tick();
        assert(s.out.reason != "sweep_frame_close_wait");
    }
    { // a frame that never acknowledges the open: the sweep continues closed and stays closed
      // afterwards
        TaskTuning t;
        t.sweep_observation_guard = true;
        t.sweep_frame_open = true;
        t.disperse_swing_vx = 0;
        t.gripper_timeout_us = 500000;
        Sim s(t);
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        s.task.enter(PushState::SCATTER, s.in.now_us, "test_sweep");
        s.task.search_phase_.sweep_phase_ = 5;
        s.task.sweep_ledger_.begin(s.task.t_, s.in.now_us);
        s.task.search_phase_.sweep_start_us_ = s.in.now_us;
        s.task.search_phase_.sweep_yaw0_ = s.phi;
        s.task.search_phase_.disperse_swing_.configure(
            s.task.search_phase_.disperse_swing_.passLimit(), 2);
        s.in.clutter_body = {{0, .3f}, {.1f, .3f}, {-.1f, .3f}};
        s.gripper_responds = false;
        for (int i = 0; i < 40; ++i) {
            s.in.clutter_ts_us = s.in.now_us;
            s.tick();
        }
        assert(s.task.sweep_open_disabled_ && s.out.motion.gripper_offset == 20);
        s.gripper_responds = true;
        s.in.clutter_ts_us = s.in.now_us;
        s.tick();
        assert(s.out.motion.wz_rps != 0 && s.out.motion.gripper_offset == 20);
    }
    { // flag off: the closed-frame sweep is unchanged
        TaskTuning t;
        t.sweep_observation_guard = true;
        t.disperse_swing_vx = 0;
        Sim s(t);
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        s.task.enter(PushState::SCATTER, s.in.now_us, "test_sweep");
        s.task.search_phase_.sweep_phase_ = 5;
        s.task.sweep_ledger_.begin(s.task.t_, s.in.now_us);
        s.task.search_phase_.sweep_start_us_ = s.in.now_us;
        s.task.search_phase_.sweep_yaw0_ = s.phi;
        s.task.search_phase_.disperse_swing_.configure(
            s.task.search_phase_.disperse_swing_.passLimit(), 2);
        s.in.clutter_body = {{0, .3f}, {.1f, .3f}, {-.1f, .3f}};
        s.in.clutter_ts_us = s.in.now_us;
        s.tick();
        assert(s.out.motion.wz_rps != 0 && s.out.motion.gripper_offset == 20);
    }
}
// Default-open frame: open whenever nothing is held and the robot searches/approaches/sweeps;
// closed demands wait for the wire.

void defaultOpenFrameTests() {
    {
        TaskTuning t;
        t.frame_open_default = true;
        Sim s(t);
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        assert(s.task.frame_raised_ == 0 &&
               s.out.motion.gripper_offset == 0); // SCAN keeps the wire open
        s.task.enter(PushState::CLEAR_PILE,
                     s.in.now_us,
                     "test_closed_state"); // needs the closed wire: held until it is confirmed
        s.gripper_responds = false;
        s.tick();
        assert(stopped(s.out.motion) && s.out.reason == "sweep_frame_close_wait" &&
               s.out.motion.gripper_offset == 20);
        s.gripper_responds = true;
        s.tick();
        assert(s.out.reason != "sweep_frame_close_wait" && s.out.motion.gripper_offset == 20);
    }
    { // the logical close hidden by the open overlay is not a new MCU action; LOST_SEARCH may spin
      // with the frame open
        TaskTuning t;
        t.frame_open_default = true;
        Sim s(t);
        s.in.target_valid = false;
        uint64_t open_txn = 0;
        bool seen_open = false;
        for (int i = 0; i < 400 && s.out.state != PushState::SCAN; ++i) {
            s.tick();
            if (s.out.motion.gripper_offset == 0) {
                open_txn = s.out.motion.frame_transaction;
                seen_open = true;
            }
        }
        assert(seen_open && s.out.state == PushState::SCAN);
        for (int i = 0; i < 20; ++i)
            s.tick();
        assert(s.out.motion.gripper_offset == 0 && s.out.motion.frame_transaction == open_txn);
        s.task.enter(PushState::LOST_SEARCH, s.in.now_us, "test_lost");
        for (int i = 0; i < 10; ++i)
            s.tick();
        assert(s.out.state == PushState::LOST_SEARCH && s.out.motion.gripper_offset == 0);
    }
    { // flag off: SCAN is closed as before
        TaskTuning t;
        Sim s(t);
        s.in.target_valid = false;
        assert(s.until(PushState::SCAN, 30));
        for (int i = 0; i < 20; ++i)
            s.tick();
        assert(s.out.motion.gripper_offset == 20);
    }
    { // a wider swing angle turns further (default 1.0 rad vs 1.4 rad)
        float turned[2] = {0, 0};
        int k = 0;
        for (float rad : {1.0f, 1.4f}) {
            TaskTuning t;
            t.disperse_swing_vx = 0;
            t.disperse_turn_rad = rad;
            t.sweep_observation_guard = true;
            Sim s(t);
            s.in.target_valid = false;
            assert(s.until(PushState::SCAN, 30));
            for (int i = 0; i < 20; ++i)
                s.tick();
            s.task.enter(PushState::SCATTER, s.in.now_us, "test_sweep");
            s.task.search_phase_.sweep_phase_ = 5;
            s.task.sweep_ledger_.begin(s.task.t_, s.in.now_us);
            s.task.search_phase_.sweep_start_us_ = s.in.now_us;
            s.task.search_phase_.sweep_yaw0_ = s.phi;
            s.task.search_phase_.disperse_swing_.configure(
                s.task.search_phase_.disperse_swing_.passLimit(), 2);
            s.in.clutter_body = {{0, .3f}, {.1f, .3f}, {-.1f, .3f}};
            const float phi0 = s.phi;
            float peak = 0;
            for (int i = 0; i < 400 && s.task.search_phase_.sweep_phase_ == 5; ++i) {
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                peak = std::max(peak, std::fabs(s.phi - phi0));
            }
            turned[k++] = peak;
        }
        assert(turned[1] > turned[0] + .25f);
    }
}
// Requested demo capture timing with real corridor rules still enabled.

void sweepViewTests() {
    const auto base = [] {
        TaskTuning t;
        t.enable_search_cues = false;
        t.blind_scatter_min = 3;
        t.blind_scatter_first_turn_rad = 0.f;
        t.blind_scatter_passes = 1;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        return t;
    };
    struct R {
        int max_pitch = 0, rush_pitch = -1, swing_pitch = -1, align_pitch = -1;
        float max_swing = 0;
        bool done = false, lost = false;
    };
    const auto run = [&](TaskTuning t, std::vector<cv::Point2f> body_pts) {
        Sim s(t);
        s.in.target_valid = false;
        s.in.distance_m = .9f;
        const float x0 = s.px, y0 = s.py;
        std::vector<cv::Point2f> world;
        for (const auto& b : body_pts)
            world.push_back({x0 + b.x, y0 + b.y});
        R r;
        float phi_rush = 0;
        bool rushed = false;
        for (int i = 0; i < 2400 && !r.done; ++i) {
            s.in.clutter_body.clear();
            const float c = std::cos(s.phi), sn = std::sin(s.phi);
            for (const auto& w : world) {
                const float dx = w.x - s.px, dy = w.y - s.py;
                s.in.clutter_body.push_back({dx * c + dy * sn, -dx * sn + dy * c});
            }
            s.in.clutter_ts_us = s.in.now_us;
            s.tick();
            if (s.out.state == PushState::SCATTER) {
                r.max_pitch = std::max(r.max_pitch, int(s.out.motion.camera_pitch_cdeg));
                if (s.out.reason == "blind_scatter_align")
                    r.align_pitch = s.out.motion.camera_pitch_cdeg;
                if (s.out.reason == "disperse_rush") {
                    r.rush_pitch = s.out.motion.camera_pitch_cdeg;
                    if (!rushed) {
                        rushed = true;
                        phi_rush = s.phi;
                    }
                }
                if (s.out.reason == "disperse_swing")
                    r.swing_pitch = s.out.motion.camera_pitch_cdeg;
                if (rushed)
                    r.max_swing = std::max(r.max_swing, s.phi - phi_rush);
            }
            if (s.out.reason == "blind_scatter_done")
                r.done = true;
            if (s.out.reason == "blind_scatter_observation_lost")
                r.lost = true;
        }
        return r;
    };
    const std::vector<cv::Point2f> group{{.20f, .50f}, {.32f, .56f}, {.26f, .44f}};
    { // 扫动前视角封顶20°；扫动动作本身（冲入/摆扫）改用5°远景判断。
        TaskTuning off = base();
        TaskTuning cap = base();
        cap.sweep_view_cap_mid = true;
        TaskTuning far = cap;
        far.sweep_far_judge = true;
        const auto a = run(off, group), b = run(cap, group), c = run(far, group);
        assert(a.done && b.done && c.done);
        assert(b.max_pitch <= 2000);
        assert(c.max_pitch <= 2000 && c.align_pitch == 2000);
        assert(c.rush_pitch == 500 && c.swing_pitch == 500);
        assert(!c.lost);
    }
    { // 摆扫方向避开框外物块：框外块在前方偏左时，向左的摆角被封顶，先摆向空闲的一侧。
        const cv::Point2f gc{.26f, .50f};
        const float len = std::hypot(gc.x, gc.y);
        const cv::Point2f u{gc.x / len, gc.y / len}, xa{u.y, -u.x};
        const float e = len - .171f + .20f;
        const cv::Point2f blk{u.x * (e + .30f) - xa.x * .20f, u.y * (e + .30f) - xa.y * .20f};
        auto pts = group;
        pts.push_back(blk);
        TaskTuning on = base();
        on.sweep_view_cap_mid = true;
        on.sweep_far_judge = true;
        TaskTuning plan = on;
        plan.swing_avoid_outside = true;
        const auto free = run(on, pts), guarded = run(plan, pts);
        assert(free.done && guarded.done);
        assert(free.max_swing > .85f);
        assert(guarded.max_swing < .75f && free.max_swing - guarded.max_swing > .2f);
    }
    { // 摆角上限只属于本次SCATTER扫动：离开SCATTER后不能残留到后面的APPROACH/MID扫动里。
        TaskTuning t = base();
        Sim s(t);
        s.task.enter(PushState::SCATTER, s.in.now_us, "test");
#ifdef RESCUE_HAS_RUSH_STAGE
        s.task.search_phase_.disperse_rush_.active_ = true;
        s.task.search_phase_.disperse_rush_.pending_ = true;
        s.task.search_phase_.disperse_rush_.cap_negative_ = .3f;
        s.task.search_phase_.disperse_rush_.cap_positive_ = .2f;
#else
        s.task.swing_plan_active_ = true;
        s.task.swing_plan_pending_ = true;
        s.task.swing_cap_neg_ = .3f;
        s.task.swing_cap_pos_ = .2f;
#endif
        s.task.enter(PushState::APPROACH, s.in.now_us, "test");
#ifdef RESCUE_HAS_RUSH_STAGE
        assert(!s.task.search_phase_.disperse_rush_.active_ &&
               !s.task.search_phase_.disperse_rush_.pending_ &&
               s.task.search_phase_.disperse_rush_.cap_negative_ >= 99.f &&
               s.task.search_phase_.disperse_rush_.cap_positive_ >= 99.f);
#else
        assert(!s.task.swing_plan_active_ && !s.task.swing_plan_pending_ &&
               s.task.swing_cap_neg_ >= 99.f && s.task.swing_cap_pos_ >= 99.f);
#endif
    }
    { // Only a blocked target corridor lets a local dense group pre-empt approach.
        for (int on = 0; on < 2; ++on) {
            TaskTuning t = base();
            t.sweep_view_cap_mid = true;
            t.dense_no_lock = on;
            Sim s(t);
            s.in.target_valid = true;
            s.in.distance_m = .45f;
            s.in.corridor = inv(3); // three supplies cannot be carried together
            bool split = false, approach = false;
            for (int i = 0; i < 400; ++i) {
                s.in.clutter_body = {{.12f, .50f}, {-.10f, .52f}};
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                if (s.out.state == PushState::SCATTER && s.out.reason.rfind("dense_group", 0) == 0)
                    split = true;
                if (s.out.reason.rfind("dense_group", 0) == 0 || s.out.state == PushState::SCATTER)
                    split = split || s.out.state == PushState::SCATTER;
                if (s.out.state == PushState::APPROACH)
                    approach = true;
                if (split || approach)
                    break;
            }
            if (on) {
                assert(split && !approach);
            } else {
                assert(!split && approach);
            }
        }
    }
    { // Clean target corridors remain selectable regardless of visible count or sweep cooldown.
        const auto run = [&](int max_visible,
                             bool ignore_gap,
                             bool gap_active,
                             std::vector<cv::Point2f> others) {
            TaskTuning t = base();
            t.sweep_view_cap_mid = true;
            t.dense_no_lock = true;
            t.lock_max_visible = max_visible;
            t.dense_multi_ignore_gap = ignore_gap;
            Sim s(t);
            s.in.target_valid = true;
            s.in.distance_m = .45f;
            if (gap_active)
                s.task.sweep_ledger_.setGapUntil(s.in.now_us + 20000000);
            std::string why;
            bool approach = false, split = false;
            for (int i = 0; i < 400; ++i) {
                s.in.clutter_body = others;
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                if (s.out.state == PushState::SCATTER) {
                    split = true;
                    why = s.out.reason;
                }
                if (s.out.state == PushState::APPROACH)
                    approach = true;
                if (split || approach)
                    break;
            }
            return std::make_pair(split, approach);
        };
        const std::vector<cv::Point2f> three{{.30f, .50f}, {-.30f, .52f}}, two{{.30f, .50f}};
        auto r = run(0, false, false, three);
        assert(!r.first && r.second); // 关闭：松散三个照常锁定
        r = run(2, false, false, three);
        assert(!r.first && r.second); // three visible objects do not invalidate this corridor
        r = run(2, false, false, two);
        assert(!r.first && r.second); // 2个可见：照常锁定
        r = run(2, false, true, three);
        assert(!r.first && r.second); // 冷却期内且不无视间隔：回落到锁定
        r = run(2, true, true, three);
        assert(!r.first && r.second); // ignoring the gap does not override a clean corridor
    }
    { // 锁定后近处被车框遮挡丢失：原地等待新ID，不立刻进入丢失搜索；新ID出现在原位置附近则改绑；超时仍走原来的丢失流程。
        const auto run = [&](uint64_t hold, bool newid) {
            TaskTuning t;
            t.enable_search_cues = false;
            t.assume_all_safe = true;
            t.track_pitch_cdeg = 4000;
            t.intermediate_pitch_cdeg = 2000;
            t.occlusion_hold_us = hold;
            Sim s(t);
            s.in.distance_m = .6f;
            assert(s.until(PushState::APPROACH, 80));
            for (int i = 0; i < 10; ++i)
                s.tick();
            s.task.search_phase_.approach_rebind_ =
                false; // isolate the occlusion hold from the pitch-change rebind window
            s.in.target_valid = false;
            bool waited = false, rebound = false;
            int left = -1;
            for (int i = 0; i < 220; ++i) {
                if (newid && i == 20) {
                    s.in.target_valid = true;
                    s.in.target_id = 99;
                    s.in.distance_m = .58f;
                }
                s.tick();
                if (s.out.reason == "target_occluded_wait")
                    waited = true;
                if (s.out.reason == "approach_id_rebound_same_object")
                    rebound = true;
                if (left < 0 && s.out.state != PushState::APPROACH)
                    left = i;
            }
            return std::make_tuple(waited, rebound, left);
        };
        auto [w0, b0, l0] = run(0, false);
        assert(!w0 && l0 >= 0 && l0 < 120);
        auto [w1, b1, l1] = run(2500000, false);
        assert(w1 && l1 >= l0 + 35);
        auto [w2, b2, l2] = run(2500000, true);
        assert(w2 && b2 && (l2 < 0 || l2 > l0 + 35));
    }
    { // APPROACH里的密集扫动：开关打开后摆扫角速度与首发爆发一致(first_burst_swing_wz)，关闭则为disperse_wz；摆幅两侧同为disperse_turn_rad。
        const auto run = [&](bool burst) {
            TaskTuning t;
            t.enable_search_cues = false;
            t.assume_all_safe = true;
            t.track_pitch_cdeg = 4000;
            t.intermediate_pitch_cdeg = 2000;
            t.track_near_m = .4f;
            t.intermediate_to_near_m = .27f;
            t.rush_start_m = .35f;
            t.disperse_swing_vx = 0;
            t.disperse_turn_rad = 1.4f;
            t.first_burst_swing_wz = 5.f;
            t.disperse_wz = 3.2f;
            t.approach_dense_burst_swing = burst;
            Sim s(t);
            s.in.distance_m = .5f;
            assert(s.until(PushState::APPROACH, 80));
            s.in.distance_m = .25f;
            float swing_wz = 0, lo = 0, hi = 0;
            const float phi0 = s.phi;
            bool swung = false;
            for (int i = 0; i < 600; ++i) {
                s.in.clutter_body.clear();
                for (int k = 0; k < 4; ++k)
                    s.in.clutter_body.push_back(
                        {-.12f + .08f * k, .25f + ((k % 2) ? .06f : -.06f)});
                s.in.clutter_ts_us = s.in.now_us;
                s.tick();
                if (s.out.state == PushState::APPROACH && s.out.reason == "disperse_swing") {
                    swung = true;
                    swing_wz = std::max(swing_wz, std::fabs(s.out.motion.wz_rps));
                }
                lo = std::min(lo, s.phi - phi0);
                hi = std::max(hi, s.phi - phi0);
                if (s.out.reason == "disperse_done")
                    break;
            }
            assert(swung);
            return std::array<float, 3>{swing_wz, lo, hi};
        };
        const auto off = run(false), on = run(true);
        assert(on[0] > off[0] + .5f);
        assert(std::fabs(on[1]) > 1.0f && on[2] > 1.0f);
    }
}

void pileTests() {
    const auto base = [] {
        TaskTuning t;
        t.enable_search_cues = false;
        t.assume_all_safe = true;
        t.track_pitch_cdeg = 4000;
        t.intermediate_pitch_cdeg = 2000;
        t.track_near_m = .4f;
        t.assume_motion_ratio = 1.f;
        t.pile_clear = true;
        return t;
    };
    { // The old dense-corridor entry must not start a metric cut.
        Sim s(base());
        s.all_ordinary = false;
        s.in.distance_m = .5f;
        assert(s.until(PushState::APPROACH, 80));
        s.in.corridor = inv(3);
        for (int i = 0; i < 40; ++i) {
            s.tick();
            assert(s.out.state != PushState::PILE);
        }
    }
    { // PILE chain: pushed-out cluster deeper than one layer -> face, rush in, big-angle turn, back
      // off, re-measure (capped), then return to search; one layer also returns to search.
        const auto chain =
            [&](std::vector<cv::Point2f> rel, bool on, int max_rounds, float fov = 0.f) {
                TaskTuning t = base();
                t.pile_chain = on;
                t.pile_chain_max = max_rounds;
                t.disperse_single_turn = true;
                t.disperse_turn_push_m = .2f;
                Sim s(t);
                s.all_ordinary = false;
                s.in.target_valid = false;
                assert(s.until(PushState::SCAN, 30));
                for (int i = 0; i < 20; ++i)
                    s.tick();
                const float x0 = s.px, y0 = s.py, phi0 = s.phi;
                s.task.enter(PushState::PILE, s.in.now_us, "test_pile_chain");
                s.task.search_phase_.pile_yaw0_ = s.phi;
                s.task.search_phase_.pile_phase_ = 7;
                s.task.search_phase_.pile_seen_ = 0;
                s.task.search_phase_.pile_chain_n_ = 0;
                s.task.search_phase_.pile_phase_us_ = s.in.now_us;
                std::set<std::string> reasons;
                std::string end;
                int rounds = 0;
                bool left = false;
                for (int i = 0; i < 6000 && !left; ++i) {
                    s.in.clutter_body.clear();
                    s.in.clutter_ordinary.clear();
                    const float c = std::cos(s.phi), sn = std::sin(s.phi);
                    for (const auto& b : rel) {
                        const float dx = x0 + (b.x * std::cos(phi0) - b.y * std::sin(phi0)) - s.px,
                                    dy = y0 + (b.x * std::sin(phi0) + b.y * std::cos(phi0)) - s.py;
                        const cv::Point2f q{dx * c + dy * sn, -dx * sn + dy * c};
                        if (fov > 0.f && std::fabs(std::atan2(-q.x, q.y)) > fov)
                            continue;
                        s.in.clutter_body.push_back(q);
                        s.in.clutter_ordinary.push_back(q);
                    }
                    s.in.clutter_ts_us = s.in.now_us;
                    s.tick();
                    if (s.out.state == PushState::PILE)
                        reasons.insert(s.out.reason);
                    else {
                        left = true;
                        end = s.out.reason;
                    }
                    rounds = s.task.search_phase_.pile_chain_n_;
                }
                assert(left);
                return std::make_tuple(reasons, end, rounds);
            };
        const std::vector<cv::Point2f> deep{{0.f, .50f}, {.04f, .62f}, {-.04f, .74f}, {0.f, .86f}};
        const std::vector<cv::Point2f> flat{{-.10f, .60f}, {0.f, .60f}, {.10f, .60f}};
        {
            const auto [rs, end, n] = chain(deep, true, 2);
            for (const char* k :
                 {"pile_chain_face", "disperse_rush", "disperse_turn", "pile_chain_back"})
                if (!rs.count(k)) {
                    std::cerr << "missing chain reason " << k << "\n";
                    assert(false);
                }
            assert(n == 2); // blocks never move in the Sim, so the loop runs to its cap
        }
        {
            const auto [rs, end, n] = chain(flat, true, 3);
            assert(end == "pile_chain_one_layer");
            assert(!rs.count("pile_chain_face") && !rs.count("disperse_rush") &&
                   n == 0); // one layer: no more sweeping
        }
        { // the turn takes the cluster out of a narrow view: the car must look back (mid-turn, then
          // pre-turn heading) instead of giving up.
            const auto [rs, end, n] = chain(deep, true, 1, .5f);
            assert(rs.count("pile_chain_look") && end != "pile_chain_cluster_clear");
        }
        {
            const auto [rs, end, n] = chain(deep, true, 0);
            assert(end == "pile_chain_limit");
            assert(!rs.count("pile_chain_face") && n == 0); // cap 0: return to search without cutting
        }
    }
    { // A swung disperse sweep that ends in APPROACH hands over to PILE (phase 7) only when
      // pile_chain is on.
        for (bool on : {true, false}) {
            TaskTuning t = base();
            t.pile_chain = on;
            t.disperse_single_turn = true;
            t.disperse_turn_push_m = .2f;
            Sim s(t);
            s.all_ordinary = false;
            s.in.distance_m = .5f;
            assert(s.until(PushState::APPROACH, 80));
            s.task.sweep_ledger_.begin(s.task.t_, s.in.now_us);
            s.task.sweep_ledger_.setSwung(true);
            s.task.search_phase_.sweep_phase_ = 5;
            s.task.search_phase_.disperse_turn_.phase_ = DisperseTurnPhase::Push;
            s.task.search_phase_.sweep_start_us_ = s.in.now_us;
            s.task.search_phase_.sweep_travel0_ = s.task.actionProgress().travel - 1.f;
            for (int i = 0; i < 60 && s.out.state == PushState::APPROACH; ++i)
                s.tick();
            assert((s.out.state == PushState::PILE) == on);
            if (on)
                assert(s.task.search_phase_.pile_phase_ == 7);
        }
    }
}
