#include "../../support/push_task_sim.hpp"
#include "push_task_cases.hpp"

void assumedDeliveryCases() {
    { // assume_injured_trip starts as if the first ordinary supply was delivered; the default is
      // unchanged.
        for (int on = 0; on < 2; ++on) {
            TaskTuning t;
            t.assume_injured_trip = on;
            PushTask task(t);
            PushObservation in;
            in.run = in.safety_ok = true;
            in.now_us = 100000;
            assert(task.update(in).first_ordinary_delivered == bool(on));
            in.reset = true;
            in.now_us += 100000;
            assert(task.update(in).first_ordinary_delivered == bool(on));
        }
    }
    { // Visual closed loop: release/push distances follow the remembered or visible zone, not fixed
      // blind legs.
        for (int lose = 0; lose < 2; ++lose) {
            TaskTuning t;
            t.assume_all_safe = true;
            t.demo_carry_once = true;
            t.assume_motion_ratio = 1.f;
            Sim s(t);
            s.tune.require_multi_view = true;
            s.task = PushTask(s.tune);
            s.zone_visible = false;
            s.multi_evidence = false;
            s.in.distance_m = .30f;
            s.vref_visible = true;
            s.toRush();
            s.in.target_valid = false;
            assert(s.until(PushState::CARRY, 300) && s.out.reason == "assumed_capture");
            if (lose) { // zone leaves the view mid-leg: remaining distance is dead-reckoned
                for (int i = 0; i < 400 && s.py < -.5f; ++i)
                    s.tick();
                s.vref_visible = false;
            }
            assert(s.until(PushState::RAISE_RELEASE, 600));
            assert(s.out.reason ==
                   (lose ? "assumed_drop_point_visual_dr" : "assumed_drop_point_visual"));
            {
                const float e =
                    s.py + s.tune.hold_center_y_m +
                    s.tune.assume_release_gap_m; // dead-reckoned leg is pessimistic: may run on
                                                 // past the point, never stop short
                if (lose)
                    assert(e > -.06f && e < .32f);
                else
                    assert(std::abs(e) < .06f);
            }
            s.vref_visible = true;
            assert(s.until(PushState::BACK_OUT, 100) && s.out.reason == "assumed_release_back_off");
            const float yb = s.py;
            assert(s.until(PushState::ENTER, 300) && s.out.reason == "assumed_release_then_push");
            assert(std::abs((yb - s.py) - s.tune.assume_prepush_back_m) < .04f);
            assert(s.until(PushState::RAISE_RELEASE, 600) &&
                   s.out.reason == "assumed_push_in_done");
            assert(std::abs(s.py + s.tune.hold_center_y_m - s.tune.assume_push_depth_m) < .06f);
            assert(s.until(PushState::SAFE_STOP, 2000) &&
                   s.out.reason == "demo_next_trip_unverified");
        }
    }
    { // Blocked by the zone rim near the front edge: release by stall instead of ramming until
      // timeout.
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.py_max = -.03f - s.tune.hold_center_y_m;
        s.toRush();
        s.in.target_valid = false;
        assert(s.until(PushState::CARRY, 300) && s.out.reason == "assumed_capture");
        assert(s.until(PushState::RAISE_RELEASE, 600) &&
               s.out.reason == "assumed_drop_point_stalled");
    }
    { // 卡在边框前较远处（第37趟 hold_y≈-0.08）：放宽的卡停窗口内同样按停滞释放，而不是空转到超时。
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        t.assume_stall_near_m = .12f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.py_max = -.09f - s.tune.hold_center_y_m;
        s.toRush();
        s.in.target_valid = false;
        assert(s.until(PushState::CARRY, 300) && s.out.reason == "assumed_capture");
        assert(s.until(PushState::RAISE_RELEASE, 900) &&
               s.out.reason == "assumed_drop_point_stalled");
    }
    { // 视觉判块：块已在区内（近端距前沿≥done_y）则推入提前结束，核验阶段按视觉判进区。
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        assert(s.until(PushState::ENTER, 2000) && s.out.reason == "assumed_release_then_push");
        s.cargo_on = true;
        s.cargo_zone = {s.tune.supply_half_x_m, .12f};
        assert(s.until(PushState::RAISE_RELEASE, 600) && s.out.reason == "assumed_push_in_done");
        assert(s.py + s.tune.hold_center_y_m < s.tune.assume_push_depth_m - .05f);
        assert(s.py + s.tune.hold_center_y_m >=
               s.tune.assume_inside_min_hold_y_m -
                   .02f); // 框自身须已越过前沿一定深度才信“块已进区”
        assert(s.until(PushState::TURN_SCAN, 1500) && s.out.reason == "delivered_visual");
        const std::string ev = s.task.takeDropZoneEvent();
        assert(ev.find("[CARGO_ZONE] event=push_inside") != std::string::npos &&
               ev.find("event=verify") != std::string::npos);
    }
    for (int hard = 0; hard < 2;
         ++hard) { // 翻框前须确认真进区：推入深度不够时继续下压；超过硬上限才放弃确认
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        t.assume_enter_confirm = true;
        t.assume_enter_hold_y_m = hard ? .90f : .22f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        assert(s.until(PushState::ENTER, 2000) && s.out.reason == "assumed_release_then_push");
        assert(s.until(PushState::RAISE_RELEASE, 1500) && s.out.reason == "assumed_push_in_done");
        const float depth = s.py + s.tune.hold_center_y_m;
        const std::string ev = s.task.takeDropZoneEvent();
        assert(ev.find("event=push_unconfirmed") != std::string::npos);
        if (!hard) {
            assert(depth >= .21f && ev.find("event=push_unconfirmed_giveup") == std::string::npos);
        } else {
            assert(depth >= s.tune.assume_enter_max_hold_y_m - .02f &&
                   ev.find("event=push_unconfirmed_giveup") != std::string::npos);
        }
    }
    { // 默认关闭：行为与原先一致（深度到 assume_push_depth_m 即翻框）
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        t.assume_enter_hold_y_m = .90f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        assert(s.until(PushState::ENTER, 2000) && s.out.reason == "assumed_release_then_push");
        assert(s.until(PushState::RAISE_RELEASE, 1500) && s.out.reason == "assumed_push_in_done");
        assert(s.py + s.tune.hold_center_y_m < s.tune.assume_push_depth_m + .06f);
        assert(s.task.takeDropZoneEvent().find("push_unconfirmed") == std::string::npos);
    }
    { // 深松手：松开时块已过前沿，后退中视觉看到块在区内则跳过推入直接核验。
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        s.cargo_on = true;
        s.cargo_zone = {s.tune.supply_half_x_m, .12f};
        assert(s.until(PushState::VERIFY_DELIVERY, 2500) &&
               s.out.reason == "assumed_inside_after_release");
        assert(s.until(PushState::TURN_SCAN, 1500) && s.out.reason == "delivered_visual");
        const std::string ev = s.task.takeDropZoneEvent();
        assert(ev.find("event=inside_after_release") != std::string::npos &&
               ev.find("event=push_done") == std::string::npos);
    }
    { // 核验只认视觉定位：推算认为块在区内但无定位 →
      // 不判视觉通过，先等、再切远俯仰找角点；等不到按“推算”记并标明。
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        s.cargo_on = true;
        s.cargo_zone = {s.tune.supply_half_x_m, .12f};
        assert(s.until(PushState::VERIFY_DELIVERY, 2500) &&
               s.out.reason == "assumed_inside_after_release");
        s.vref_visible = false;
        bool far = false;
        int n = 0;
        for (; n < 400 && s.out.state == PushState::VERIFY_DELIVERY; ++n) {
            s.tick();
            if (s.out.motion.camera_pitch_cdeg == s.tune.far_pitch_cdeg)
                far = true;
        }
        assert(s.out.state == PushState::TURN_SCAN && s.out.reason == "delivered_unverified");
        assert(far && n * 50000ull >= s.tune.assume_verify_fix_wait_us - 100000);
        assert(s.task.takeDropZoneEvent().find("fix=0 repush=0 in=1") == std::string::npos);
    }
    { // 定位晚到：先无定位等待，随后拿到定位且块在区内 → 才判 delivered_visual。
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        s.cargo_on = true;
        s.cargo_zone = {s.tune.supply_half_x_m, .12f};
        assert(s.until(PushState::VERIFY_DELIVERY, 2500));
        s.vref_visible = false;
        for (int i = 0; i < 80; ++i) {
            s.tick();
            assert(s.out.state == PushState::VERIFY_DELIVERY);
        }
        s.vref_visible = true;
        assert(s.until(PushState::TURN_SCAN, 200) && s.out.reason == "delivered_visual");
    }
    { // 定位到了但块在区外（压在边框外）→ 重推，而不是记成功。
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        s.cargo_on = true;
        s.cargo_zone = {s.tune.supply_half_x_m, .12f};
        assert(s.until(PushState::VERIFY_DELIVERY, 2500));
        s.cargo_zone = {s.tune.supply_half_x_m, -.02f};
        assert(s.until(PushState::ENTER, 200) && s.out.reason == "assumed_verify_repush");
        assert(s.task.takeDropZoneEvent().find("event=verify_outside_repush") != std::string::npos);
    }
    { // 块近端刚越过前沿约2cm（日志实测1.4~3cm）→ 算进区，不再重推。
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        s.cargo_on = true;
        s.cargo_zone = {s.tune.supply_half_x_m, .12f};
        assert(s.until(PushState::VERIFY_DELIVERY, 2500));
        s.cargo_zone = {s.tune.supply_half_x_m, .02f};
        assert(s.until(PushState::TURN_SCAN, 200) && s.out.reason == "delivered_visual");
    }
    { // 最近的块在区外，但区内另有块 → 不重推，直接记送达。
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        s.cargo_on = true;
        s.cargo_zone = {s.tune.supply_half_x_m, .12f};
        assert(s.until(PushState::VERIFY_DELIVERY, 2500));
        s.cargo_zone = {s.tune.supply_half_x_m, -.02f};
        s.cargo2_on = true;
        assert(s.until(PushState::TURN_SCAN, 200) && s.out.reason == "delivered_visual");
        assert(s.task.takeDropZoneEvent().find("event=verify_other_inside") != std::string::npos);
    }
    { // 后退后前推前的视觉对准：货物偏在侧前方时先原地转向对准，再按方位追踪，框到达时货物须在框正前方。
        for (int align = 0; align < 2; ++align) {
            TaskTuning t;
            t.assume_all_safe = true;
            t.demo_carry_once = true;
            t.assume_motion_ratio = 1.f;
            t.assume_align_push = align;
            t.assume_slant_max_rad = 0.f;
            Sim s(t);
            s.tune.require_multi_view = true;
            s.task = PushTask(s.tune);
            s.zone_visible = false;
            s.multi_evidence = false;
            s.in.distance_m = .30f;
            s.vref_visible = true;
            s.toRush();
            s.in.target_valid = false;
            assert(s.until(PushState::ENTER, 2000) && s.out.reason == "assumed_release_then_push");
            s.cargo_on = true;
            s.cargo_zone = {s.px + .12f, .03f};
            bool turned = false;
            float lastx = 9.f;
            for (int i = 0; i < 900 && s.out.state == PushState::ENTER; ++i) {
                s.tick();
                if (s.out.reason == "assumed_push_align_turn")
                    turned = true;
                if (!s.in.cargo_body.empty() &&
                    s.in.cargo_body[0].y - s.tune.hold_center_y_m < .15f)
                    lastx = s.in.cargo_body[0].x;
            }
            const std::string ev = s.task.takeDropZoneEvent();
            if (align) {
                assert(turned && ev.find("event=push_aligned") != std::string::npos &&
                       std::abs(lastx) < .05f);
            } else
                assert(!turned);
        }
    }
    { // 卡住重推：块停在前沿、框前进不了；每次重推加长助跑，最多2次后放弃并抬框退出。
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        assert(s.until(PushState::ENTER, 2000) && s.out.reason == "assumed_release_then_push");
        s.cargo_on = true;
        s.cargo_zone = {s.px, .0f};
        s.py_max = -.15f - s.tune.hold_center_y_m;
        float back1 = 0;
        for (int n = 0; n < 2; ++n) {
            assert(s.until(PushState::BACK_OUT, 1500) && s.out.reason == "assumed_stuck_repush");
            const float yb = s.py;
            assert(s.until(PushState::ENTER, 600) && s.out.reason == "assumed_release_then_push");
            const float back = yb - s.py;
            if (n == 0)
                back1 = back;
            else
                assert(back > back1 + .05f);
        }
        assert(s.until(PushState::RAISE_RELEASE, 1500) && s.out.reason == "assumed_push_in_done");
        const std::string ev = s.task.takeDropZoneEvent();
        assert(ev.find("event=stuck_giveup") != std::string::npos &&
               ev.find("event=stuck_repush") != std::string::npos);
    }
    { // IMU判被边框顶住：近区内按IMU释放（早于视觉/推算的+0.04释放点）；远离区域时IMU误报不释放。
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        assert(s.until(PushState::CARRY, 300) && s.out.reason == "assumed_capture");
        s.in.imu_blocked_us = 1000000;
        assert(s.until(PushState::RAISE_RELEASE, 900) &&
               s.out.reason == "assumed_drop_point_imu_blocked");
        const float e = s.py + s.tune.hold_center_y_m;
        assert(e < -.12f && e > -.40f);
        assert(s.task.takeDropZoneEvent().find("event=release_stall_imu") != std::string::npos);
    }
    { // ENTER中IMU判卡住（货物不可见也成立）：重推2次后放弃并抬框，由核验阶段再用视觉判进区。
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        assert(s.until(PushState::ENTER, 2000) && s.out.reason == "assumed_release_then_push");
        s.in.imu_blocked_us = 1000000;
        for (int n = 0; n < 2; ++n) {
            assert(s.until(PushState::BACK_OUT, 200) && s.out.reason == "assumed_stuck_repush");
            s.in.imu_blocked_us = 0;
            assert(s.until(PushState::ENTER, 600) && s.out.reason == "assumed_release_then_push");
            s.in.imu_blocked_us = 1000000;
        }
        assert(s.until(PushState::RAISE_RELEASE, 200));
        const std::string ev = s.task.takeDropZoneEvent();
        assert(ev.find("event=stuck_giveup") != std::string::npos &&
               ev.find("event=stuck_repush") != std::string::npos);
        assert(ev.find("event=push_inside") == std::string::npos);
    }
    { // 视觉定不了货物(截断/看不到)时：IMU堵转且框已深入→视为推到位不重推；框还浅→仍重推；看得清且在区外→仍重推。
        for (int k = 0; k < 4; ++k) {
            TaskTuning t;
            t.assume_all_safe = true;
            t.demo_carry_once = true;
            t.assume_motion_ratio = 1.f;
            Sim s(t);
            s.tune.require_multi_view = true;
            s.task = PushTask(s.tune);
            s.zone_visible = false;
            s.multi_evidence = false;
            s.in.distance_m = .30f;
            s.vref_visible = true;
            s.toRush();
            s.in.target_valid = false;
            assert(s.until(PushState::ENTER, 2000) && s.out.reason == "assumed_release_then_push");
            const bool deep = k == 0 || k == 3; // 0:截断+深 1:截断+浅 2:看不到+浅 3:看不到+深
            s.py_max = (deep ? .13f : .04f) - s.tune.hold_center_y_m;
            if (k < 2) {
                s.cargo_on = true;
                s.cargo_zone = {s.px, .20f};
                s.cargo_clip = true;
            }
            for (int i = 0; i < 400 && s.py < s.py_max - .005f && s.out.state == PushState::ENTER;
                 ++i)
                s.tick();
            s.in.imu_blocked_us = 1000000;
            s.task.takeDropZoneEvent();
            for (int i = 0; i < 60 && s.out.state == PushState::ENTER; ++i)
                s.tick();
            const std::string ev = s.task.takeDropZoneEvent();
            if (k == 1 || k == 2) {
                assert(s.out.state == PushState::BACK_OUT &&
                       s.out.reason == "assumed_stuck_repush");
            } else {
                assert(s.out.reason != "assumed_stuck_repush");
                assert(ev.find("event=stuck_repush") == std::string::npos);
                assert(s.until(PushState::RAISE_RELEASE, 300) &&
                       s.out.reason == "assumed_push_in_done");
                assert(s.until(PushState::VERIFY_DELIVERY, 2500));
            }
        }
    }
    { // 对准转向期间货物必然不动：不能被当成“卡住”触发重推（run58：对准转了2s+就重推）。
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        t.assume_align_push = true;
        t.assume_align_tol_rad = 1e-4f;
        t.assume_stuck_us = 300000;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        assert(s.until(PushState::ENTER, 2000) && s.out.reason == "assumed_release_then_push");
        s.cargo_on = true;
        s.cargo_zone = {s.px + .08f, .30f};
        int aligned = 0;
        for (int i = 0; i < 200 && s.out.state == PushState::ENTER; ++i) {
            s.tick();
            if (s.out.reason == "assumed_push_align_turn") {
                ++aligned;
                continue;
            }
            if (aligned)
                break;
        }
        assert(aligned >= 8 && s.out.reason != "assumed_stuck_repush");
    }
    { // B：框被图像底边截断、或块偏离目标半区，都不能当作“已在区内”（跳过推入）。
        for (int k = 0; k < 2; ++k) {
            TaskTuning t;
            t.assume_all_safe = true;
            t.demo_carry_once = true;
            t.assume_motion_ratio = 1.f;
            Sim s(t);
            s.tune.require_multi_view = true;
            s.task = PushTask(s.tune);
            s.zone_visible = false;
            s.multi_evidence = false;
            s.in.distance_m = .30f;
            s.vref_visible = true;
            s.toRush();
            s.in.target_valid = false;
            s.cargo_on = true;
            s.cargo_zone = {s.tune.supply_half_x_m + (k ? .20f : 0.f), .12f};
            s.cargo_clip = k == 0;
            for (int i = 0; i < 2500 && s.out.state != PushState::VERIFY_DELIVERY &&
                            s.out.state != PushState::SAFE_STOP;
                 ++i) {
                s.tick();
                assert(s.out.reason != "assumed_inside_after_release");
            }
            assert(s.task.takeDropZoneEvent().find("event=inside_after_release") ==
                   std::string::npos);
        }
    }
    { // A：核验无区定位时用图像回退：框底高过区前沿一定比例才算进区；贴边/被截断不算，并触发重推。
        for (int k = 0; k < 3; ++k) {
            TaskTuning t;
            t.assume_all_safe = true;
            t.demo_carry_once = true;
            t.assume_motion_ratio = 1.f;
            Sim s(t);
            s.tune.require_multi_view = true;
            s.task = PushTask(s.tune);
            s.zone_visible = false;
            s.multi_evidence = false;
            s.in.distance_m = .30f;
            s.vref_visible = true;
            s.toRush();
            s.in.target_valid = false;
            s.cargo_on = true;
            s.cargo_zone = {s.tune.supply_half_x_m, .12f};
            assert(s.until(PushState::VERIFY_DELIVERY, 2500) &&
                   s.out.reason == "assumed_inside_after_release");
            s.vref_visible = false;
            s.img_frac = k == 0 ? .5f : k == 1 ? .02f : .5f;
            s.cargo_clip = k == 2;
            if (k == 0) {
                assert(s.until(PushState::TURN_SCAN, 100) && s.out.reason == "delivered_visual");
                assert(s.until(PushState::SAFE_STOP, 400) &&
                       s.out.reason == "demo_next_trip_ready");
            } else {
                assert(s.until(PushState::ENTER, 100) && s.out.reason == "assumed_verify_repush");
            }
        }
    }
    { // C：直线推进也要有主动偏航探测：进 ENTER 满 1.2 s 后才出现，且左右交替；平时(之前)不发偏航。
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        t.assume_probe_gap_us = 300000;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        assert(s.until(PushState::ENTER, 2000) && s.out.reason == "assumed_release_then_push");
        s.py_max = s.py + .01f;
        const uint64_t t0 = s.in.now_us;
        float first = 0, second = 0;
        bool prev = false;
        for (int i = 0; i < 200 && s.out.state == PushState::ENTER; ++i) {
            s.tick();
            const bool probe = s.out.reason == "assumed_push_imu_probe" && s.out.motion.wz_rps != 0;
            if (probe && !prev) {
                assert(s.in.now_us - t0 >= s.tune.assume_probe_first_us);
                if (first == 0)
                    first = s.out.motion.wz_rps;
                else if (second == 0)
                    second = s.out.motion.wz_rps;
            }
            prev = probe;
        }
        assert(first != 0 && second != 0 && first * second < 0);
    }
    { // 未达持续时间的IMU卡住不触发（短暂顿挫不算）。
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        t.assume_motion_ratio = 1.f;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.vref_visible = true;
        s.toRush();
        s.in.target_valid = false;
        assert(s.until(PushState::ENTER, 2000) && s.out.reason == "assumed_release_then_push");
        s.in.imu_blocked_us = s.tune.assume_imu_block_us - 100000;
        for (int i = 0; i < 20; ++i) {
            s.tick();
            assert(s.out.reason != "assumed_stuck_repush");
        }
    }
    { // 目标半区由货物类别决定并在整趟锁定：物资→左(-0.15)，伤员→右(+0.15)，日志可见。
        for (int inj = 0; inj < 2; ++inj) {
            TaskTuning t;
            t.assume_all_safe = true;
            t.demo_carry_once = true;
            t.assume_motion_ratio = 1.f;
            Sim s(t);
            s.tune.require_multi_view = true;
            s.task = PushTask(s.tune);
            s.zone_visible = false;
            s.multi_evidence = false;
            s.in.distance_m = .30f;
            s.vref_visible = true;
            if (inj) {
                s.in.label = "injured_person";
                s.in.corridor = inv(0, 0, 1);
                s.task.first_ = true;
            }
            s.toRush();
            s.in.target_valid = false;
            assert(s.until(PushState::CARRY, 300) && s.out.reason == "assumed_capture");
            assert(s.until(PushState::RAISE_RELEASE, 800));
            const float want = inj ? s.tune.injured_half_x_m : s.tune.supply_half_x_m;
            if (inj)
                assert(s.px > -.15f + .15f);
            else
                assert(std::abs(s.px - want) < .06f); // 右移趋向伤员半区
            assert(s.until(PushState::SAFE_STOP, 3000));
            const std::string ev = s.task.takeDropZoneEvent();
            const char* half = inj ? "half=injured" : "half=supply";
            assert(ev.find("event=lock") != std::string::npos &&
                   ev.find(half) != std::string::npos);
            assert(ev.find("event=lock") == ev.rfind("event=lock"));
            assert(ev.find("event=release") != std::string::npos &&
                   ev.find("event=push_done") != std::string::npos);
            assert(ev.find(inj ? "half=supply" : "half=injured") == std::string::npos);
            assert(ev.find(inj ? "target_x=0.150" : "target_x=-0.150") != std::string::npos);
        }
    }
    { // 盲冲IMU偏航保持 +
      // 落框前偏航门：盲冲中被推偏后先原地对正再落框；卡住不转则放弃本次（不拉黑）。
        for (int stuck = 0; stuck < 2; ++stuck) {
            TaskTuning t;
            t.assume_all_safe = true;
            t.demo_carry_once = true;
            Sim s(t);
            s.tune.require_multi_view = true;
            s.task = PushTask(s.tune);
            s.zone_visible = false;
            s.multi_evidence = false;
            s.in.distance_m = .30f;
            s.toRush();
            s.in.target_valid = false;
            s.phi +=
                .25f; // 盲区里被推偏约14°；stuck=0时盲冲中的IMU保持自行纠回，stuck=1时底盘不转（phi被钉住）
            bool realign = false;
            const float phi0 = s.phi;
            for (int i = 0; i < 200 && s.out.state != PushState::LOWER_FRAME &&
                            s.out.state != PushState::CAPTURE_FAIL;
                 ++i) {
                if (stuck) {
                    s.tick();
                    s.phi = phi0;
                } else
                    s.tick();
                if (s.out.reason == "prelower_realign")
                    realign = true;
                if (s.out.reason == "assumed_blind_rush" && s.out.motion.wz_rps != 0)
                    assert(s.out.motion.wz_rps < 0); // 偏左 -> 向右修正
            }
            if (stuck) {
                assert(realign && s.out.state == PushState::CAPTURE_FAIL &&
                       s.out.reason == "prelower_heading_drift");
            }
            if (!stuck)
                assert(s.out.state == PushState::LOWER_FRAME &&
                       std::abs(s.phi - (phi0 - .25f)) < t.rush_lower_yaw_tol_rad + .02f);
        }
    }
    { // TEMP_ASSUMPTION assume_all_safe: no zone, no multi-view evidence, target lost below the
      // view.
        TaskTuning t;
        t.assume_all_safe = true;
        t.demo_carry_once = true;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.in.distance_m = .30f;
        s.toRush();
        s.in.target_valid = false;
        assert(s.until(PushState::LOWER_FRAME, 80) &&
               s.out.reason == "assumed_enclosure_target_below_view");
        assert(s.until(PushState::CARRY, 300) && s.out.reason == "assumed_capture" &&
               s.out.batch_size == 1);
        const float x0 = s.px, y0 = s.py;
        assert(s.until(PushState::RAISE_RELEASE, 400) && s.out.reason == "assumed_drop_point");
        assert(std::abs(std::hypot(s.px - x0, s.py - y0) - s.tune.assume_carry_m) < .03f);
        assert(s.until(PushState::BACK_OUT, 100) &&
               s.out.reason.rfind("assumed_release_back_off", 0) == 0);
        assert(s.until(PushState::ENTER, 600) && s.out.reason == "assumed_release_then_push");
        const float xp = s.px, yp = s.py;
        assert(s.until(PushState::RAISE_RELEASE, 400) && s.out.reason == "assumed_push_in_done");
        assert(std::abs(std::hypot(s.px - xp, s.py - yp) - s.tune.assume_push_m) < .03f);
        assert(s.until(PushState::BACK_OUT, 100) && s.out.reason == "frame_raised_at_drop");
        const float x1 = s.px, y1 = s.py;
        assert(s.until(PushState::VERIFY_DELIVERY, 400) && s.out.reason == "assumed_backed_out");
        const float back = std::hypot(s.px - x1, s.py - y1);
        assert(back >= s.tune.assume_back_m - .01f && back <= s.tune.assume_back_m + .03f);
        assert(s.until(PushState::TURN_SCAN, 200) && s.out.delivered_total == 1 &&
               s.out.reason == "delivered_unverified");
        assert(s.until(PushState::SAFE_STOP, 400) && s.out.reason == "demo_next_trip_unverified");
        for (auto st : {PushState::CAPTURE_FAIL, PushState::LOST_HOLD, PushState::ABORT_DROP})
            assert(!s.visited.count(st));
    }
    { // Without the assumption the same evidence gap still stops at VERIFY_CAPTURE.
        TaskTuning t;
        t.demo_carry_once = true;
        Sim s(t);
        s.tune.require_multi_view = true;
        s.task = PushTask(s.tune);
        s.zone_visible = false;
        s.multi_evidence = false;
        s.toRush();
        s.hold(inv(1));
        assert(s.until(PushState::SAFE_STOP, 300) && s.out.reason == "multi_view_unconfirmed");
    }
}

void strictDeliveryCases() {
    { // Interim firmware presets (2026-10: only -25/0/+25 deg): FAR 0, TRACK = NEAR = 2500.
        TaskTuning t;
        t.far_pitch_cdeg = 0;
        t.track_pitch_cdeg = t.near_pitch_cdeg = 2500;
        Sim s(t);
        s.toCarry();
        s.deliver();
        assert(s.out.delivered_total == 1 && s.out.first_ordinary_delivered &&
               s.out.reason == "delivered");
        assert(!s.visited.count(PushState::SAFE_STOP) && !s.visited.count(PushState::ABORT_DROP));
        // Every pitch the task commanded is one the MCU has.
        assert((s.commanded == std::set<int>{0, 2500}));
    }
    { // Transport uses a lowered enclosure until the destination, then lifts at rest.
        Sim s;
        s.toCarry();
        assert(s.until(PushState::ENTER, 400));
        assert(!s.visited.count(PushState::RAISE_RELEASE));
        bool advanced = false;
        for (int i = 0; i < 400 && s.out.state == PushState::ENTER; ++i) {
            s.tick();
            assert(s.out.motion.gripper_offset == 20);
            if (s.out.motion.vx_mps > 0)
                advanced = true;
        }
        assert(advanced && s.out.state == PushState::RAISE_RELEASE);
        assert(s.in.zone_estimate.bodyToZone({0, s.tune.hold_center_y_m}).y >=
               s.out.drop_centre_zone.y);
        s.gripper_responds = false;
        for (int i = 0; i < 80 && s.out.state != PushState::SAFE_STOP; ++i) {
            s.tick();
            assert(stopped(s.out.motion));
            assert(s.out.state != PushState::BACK_OUT);
        }
        assert(s.out.state == PushState::SAFE_STOP && s.out.motion.gripper_offset == 0);
    }
    { // Full first ordinary trip: rush, enclose, carry, carry inside, raise at the drop point, back
      // out, turn.
        Sim s;
        s.toCarry();
        assert(s.out.batch_size == 1 && s.out.motion.gripper_offset == 20);
        s.deliver();
        assert(s.out.delivered_total == 1 && s.out.first_ordinary_delivered &&
               s.out.reason == "delivered");
        for (auto st : {PushState::SCAN,
                        PushState::APPROACH,
                        PushState::PREPARE,
                        PushState::RUSH,
                        PushState::LOWER_FRAME,
                        PushState::VERIFY_CAPTURE,
                        PushState::CARRY,
                        PushState::GATE,
                        PushState::RAISE_RELEASE,
                        PushState::ENTER,
                        PushState::BACK_OUT,
                        PushState::VERIFY_DELIVERY,
                        PushState::TURN_SCAN})
            assert(s.visited.count(st));
        assert(!s.visited.count(PushState::ABORT_DROP) && !s.visited.count(PushState::SAFE_STOP));
        assert(s.until(PushState::SCAN, 400));
        assert(s.out.batch_size == 0 && s.out.delivered_total == 1 &&
               s.out.first_ordinary_delivered);
        assert(std::abs(s.phi) >= 1.5f); // turned away from the zone before rescanning
        // After the first delivery a core supply may be taken together with ordinary ones.
        s.in.label = "core_supply";
        s.in.target_id = 8;
        s.in.corridor = inv(1, 1);
        s.in.corridor_occlusion_free = false;
        s.px = -.15f;
        s.py = -.75f;
        s.phi = 0;
        s.toCarry(inv(1, 1));
        assert(s.out.batch_size == 2 && s.out.verdict == RuleVerdict::OK);
        s.deliver();
        assert(s.out.delivered_total == 3);
        assert(s.until(PushState::SCAN, 400));
        // An injured person grabbed together with a supply is dropped, not carried.
        s.in.label = "injured_person";
        s.in.target_id = 9;
        s.in.corridor = inv(0, 0, 1);
        s.in.corridor_occlusion_free = true;
        s.toRush();
        s.hold(inv(1, 0, 1));
        assert(s.until(PushState::ABORT_DROP, 40) &&
               s.out.verdict == RuleVerdict::INJURED_NOT_ALONE);
        s.hold({});
        assert(s.until(PushState::SCAN, 200) && s.out.batch_size == 0 &&
               s.out.delivered_total == 3);
        // Alone, it goes to the injured half.
        s.in.target_id = 10;
        s.px = .15f;
        s.py = -.75f;
        s.phi = 0;
        s.task_injured = true;
        s.toCarry(inv(0, 0, 1));
        s.deliver();
        assert(s.out.delivered_total == 4 && s.in.zone_injured_count == 1);
        assert(s.px > 0); // pushed into the right (injured) half
    }
    { // A held image is only required after the close feedback, never while the frame is open.
        Sim s;
        s.toRush();
        assert(s.until(PushState::LOWER_FRAME, 10));
        assert(s.until(PushState::VERIFY_CAPTURE, 30));
        s.hold(inv(1));
        assert(s.until(PushState::CARRY, 15));
    }
    for (const auto* label : {"core_supply", "injured_person", "dangerous_object", "unknown"}) {
        // Before the first ordinary delivery only ordinary supplies are selectable.
        Sim s;
        s.in.label = label;
        for (int i = 0; i < 20; ++i)
            s.tick();
        assert(s.out.state == PushState::SCAN && s.out.target_id == -1 && s.out.motion.vx_mps == 0);
    }
    { // First trip that also swept a core supply into the frame: release and back off.
        Sim s;
        s.toRush();
        s.hold(inv(1, 1));
        assert(s.until(PushState::ABORT_DROP, 40) &&
               s.out.verdict == RuleVerdict::CORE_BEFORE_FIRST);
        s.tick();
        assert(s.out.motion.gripper_offset == 0);
        s.hold({});
        assert(s.until(PushState::SCAN, 200));
        assert(s.out.delivered_total == 0 && !s.out.first_ordinary_delivered);
        for (int i = 0; i < 10; ++i)
            assert(s.tick().state == PushState::SCAN); // primary blacklisted
    }
    { // A dangerous object enclosed with the target is never carried.
        Sim s;
        s.toRush();
        s.hold(inv(1, 0, 0, 1));
        assert(s.until(PushState::ABORT_DROP, 40) && s.out.verdict == RuleVerdict::DANGEROUS);
    }
    { // Blue in PREPARE must never start a capture rush (pre-check disabled here so PREPARE itself
      // is exercised).
        TaskTuning t;
        t.corridor_precheck = false;
        Sim s(t);
        s.in.corridor = inv(1, 0, 0, 1);
        for (int i = 0; i < 80 && s.out.reason != "corridor_dangerous_back_off"; ++i)
            s.tick();
        assert(s.out.reason == "corridor_dangerous_back_off" && !s.visited.count(PushState::RUSH));
    }
    for (bool at_close : {false, true}) {
        Sim s;
        s.toRush();
        s.in.distance_m = at_close ? .19f : .30f;
        s.in.corridor = inv(1, 0, 0, 1);
        s.tick();
        assert(s.out.state == PushState::CAPTURE_FAIL && stopped(s.out.motion));
        assert(s.out.motion.gripper_offset == 0 && s.out.reason == "rush_corridor_dangerous");
        assert(!s.visited.count(PushState::LOWER_FRAME));
    }
    { // Selection-stage pre-check: a fatal corridor vetoes the target before the frame opens; no
      // RUSH, and a legal pair passes.
        for (const auto& bad :
             {inv(1, 0, 0, 1), inv(1, 0, 1), inv(1, 1), inv(1, 0, 0, 0, 1), inv(4)}) {
            Sim s;
            s.in.corridor = bad;
            for (int i = 0; i < 60 && !(s.out.state == PushState::SCAN &&
                                        s.out.reason.rfind("corridor_precheck_", 0) == 0);
                 ++i)
                s.tick();
            if (!(s.out.reason.rfind("corridor_precheck_", 0) == 0 &&
                  s.out.state == PushState::SCAN))
                std::cerr << "precheck case reason=" << s.out.reason
                          << " state=" << PushTask::name(s.out.state) << " ord=" << bad.ordinary
                          << " core=" << bad.core << " inj=" << bad.injured << "\n";
            assert(s.out.reason.rfind("corridor_precheck_", 0) == 0 &&
                   s.out.state == PushState::SCAN);
            assert(!s.visited.count(PushState::PREPARE) && !s.visited.count(PushState::RUSH));
        }
        Sim pair;
        pair.in.corridor = inv(2);
        pair.toRush(); // two ordinary supplies are a legal trip
        Sim occluded;
        occluded.in.corridor_occlusion_free = false; // not a fatal verdict: left to PREPARE
        for (int i = 0; i < 40 && occluded.out.state != PushState::PREPARE; ++i)
            occluded.tick();
        assert(occluded.out.state == PushState::PREPARE ||
               occluded.out.state == PushState::LOST_HOLD);
    }
    { // The pre-check cannot deadlock the search: after corridor_precheck_max vetoes PREPARE
      // decides alone.
        TaskTuning t;
        t.corridor_precheck_max = 2;
        Sim s(t);
        s.in.corridor = inv(1, 0, 0, 1);
        for (int i = 0; i < 2000 && !s.visited.count(PushState::PREPARE); ++i)
            s.tick();
        assert(s.visited.count(PushState::PREPARE) && !s.visited.count(PushState::RUSH));
        assert(s.out.reason != "corridor_precheck_dangerous" || s.out.state == PushState::SCAN);
    }
    { // Evidence that is not fatal for two frames then clears does not accumulate towards a veto.
        Sim s;
        assert(s.until(PushState::APPROACH, 40));
        s.in.corridor = inv(1, 0, 0, 1);
        s.tick();
        s.in.corridor = inv(1);
        assert(s.until(PushState::RUSH, 80));
    }
    { // Lost corridor evidence also prevents advancing or closing.
        Sim s;
        s.toRush();
        s.in.corridor_complete = false;
        s.tick();
        assert(s.out.state == PushState::CAPTURE_FAIL && stopped(s.out.motion));
        assert(s.out.reason == "rush_corridor_incomplete" && s.out.motion.gripper_offset == 0);
    }
    { // Too many supplies in the corridor: the rush is not started (PREPARE path, pre-check
      // disabled).
        TaskTuning t;
        t.corridor_precheck = false;
        Sim s(t);
        s.in.corridor = inv(4);
        for (int i = 0; i < 40 && s.out.reason != "corridor_too_many_supplies_back_off"; ++i)
            s.tick();
        assert(s.out.reason == "corridor_too_many_supplies_back_off" &&
               !s.visited.count(PushState::RUSH));
    }
    { // ... nor when four end up held after closing.
        Sim s;
        s.in.corridor = inv(2);
        s.toRush();
        s.hold(inv(3));
        assert(s.until(PushState::ABORT_DROP, 20) &&
               s.out.verdict == RuleVerdict::TOO_MANY_SUPPLIES);
    }
    { // Before the first delivery an occluded corridor might hide a core supply.
        Sim s;
        s.in.corridor_occlusion_free = false;
        for (int i = 0; i < 40 && s.out.reason != "corridor_occluded_back_off"; ++i)
            s.tick();
        assert(s.out.reason == "corridor_occluded_back_off" && !s.visited.count(PushState::RUSH));
        assert(s.out.state == PushState::LOST_HOLD);
    }
    { // Incomplete corridor evidence holds the robot stopped until the prepare budget expires.
        Sim s;
        s.in.corridor_complete = false;
        assert(s.until(PushState::PREPARE, 20));
        for (int i = 0; i < 20; ++i) {
            s.tick();
            assert(s.out.state == PushState::PREPARE && stopped(s.out.motion));
        }
        assert(s.until(PushState::SCAN, 100) && s.out.reason == "corridor_unresolved");
    }
    { // Nothing enclosed within the rush distance: release and retry later.
        Sim s;
        s.toRush();
        assert(s.until(PushState::CAPTURE_FAIL, 150) && s.out.reason == "capture_unverified");
        assert(s.until(PushState::SCAN, 200));
    }
    { // The load slips out during the carry: unseen at FAR, found by the next stopped NEAR check.
        Sim s;
        s.toCarry();
        s.tick();
        s.hold({});
        for (int i = 0; i < 20; ++i)
            assert(s.tick().state == PushState::CARRY);
        assert(s.until(PushState::LOST_HOLD, 200));
        assert(std::abs(s.in.camera_pitch_cdeg - 4000) <= 100 && stopped(s.out.motion));
        assert(s.until(PushState::SCAN, 200) && s.out.delivered_total == 0 &&
               s.out.batch_size == 0);
    }
    { // The load changes (another object joined): no longer the verified trip.
        Sim s;
        s.toCarry();
        s.hold(inv(1, 1));
        assert(s.until(PushState::LOST_HOLD, 200));
    }
    { // Looking far ahead is not a loss; the hold is re-checked at NEAR, always stopped.
        Sim s;
        s.toCarry();
        s.zone_visible = false;
        int far = 0, near_checks = 0;
        for (int i = 0; i < 200; ++i) {
            s.tick();
            assert(s.out.state == PushState::CARRY);
            if (s.out.motion.camera_pitch_cdeg == 4000) {
                assert(stopped(s.out.motion));
                ++near_checks;
            }
            if (s.out.motion.camera_pitch_cdeg == 500 && s.in.camera_pitch_stable)
                ++far;
        }
        assert(far > 100 && near_checks > 0);
    }
    { // Nothing moves while the camera is between presets.
        Sim s;
        s.in.distance_m = .8f;
        s.pitch_rate = 50;
        for (int i = 0; i < 10; ++i) {
            s.tick();
            assert(stopped(s.out.motion));
        }
        assert(s.until(PushState::APPROACH, 10));
    }
    { // A camera that never reaches its preset is a fault.
        Sim s;
        s.servo_responds = false;
        assert(s.until(PushState::SAFE_STOP, 60) && s.out.reason == "camera_pitch_timeout");
        for (int i = 0; i < 5; ++i)
            assert(stopped(s.tick().motion));
    }
    { // The gate re-checks the load at NEAR before opening: a load lost on the way is not released
      // as delivered.
        Sim s;
        s.toCarry();
        assert(s.until(PushState::GATE, 800));
        s.hold({});
        for (int i = 0; i < 100 && s.out.state == PushState::GATE; ++i) {
            s.tick();
            assert(s.out.motion.gripper_offset == 20);
        }
        assert(s.out.state == PushState::LOST_HOLD && !s.visited.count(PushState::RAISE_RELEASE));
    }
    { // Pushed, but the zone counts never confirm it: no credit, first stays false.
        Sim s;
        s.toCarry();
        s.deliver(false);
        assert(s.out.delivered_total == 0 && !s.out.first_ordinary_delivered &&
               s.out.reason == "delivery_unverified");
        assert(s.until(PushState::SCAN, 400));
        s.in.label = "core_supply";
        s.in.target_id = 8;
        for (int i = 0; i < 20; ++i)
            assert(s.tick().state == PushState::SCAN);
    }
    { // Seeing one target enter must not credit a two-object batch.
        Sim s;
        s.in.corridor = inv(2);
        s.toCarry(inv(2));
        assert(s.until(PushState::ENTER, 400));
        s.in.delivery_observed = true;
        assert(s.until(PushState::BACK_OUT, 400));
        s.hold({});
        s.in.zone_supply_count = 1;
        assert(s.until(PushState::TURN_SCAN, 400));
        assert(s.out.delivered_total == 1 && s.out.reason == "partial_delivery" &&
               !s.out.first_ordinary_delivered);
    }
    { // A transient target entry, with no inventory increase, gets no credit.
        Sim s;
        s.toCarry();
        assert(s.until(PushState::ENTER, 400));
        s.in.delivery_observed = true;
        assert(s.until(PushState::BACK_OUT, 400));
        s.in.delivery_observed = false;
        s.hold({});
        assert(s.until(PushState::TURN_SCAN, 400));
        assert(s.out.delivered_total == 0);
    }
    { // Incomplete inventory cannot establish delivery, even with a count increase.
        Sim s;
        s.toCarry();
        assert(s.until(PushState::BACK_OUT, 800));
        s.hold({});
        s.in.zone_supply_count = 1;
        s.in.zone_inventory_complete = false;
        assert(s.until(PushState::TURN_SCAN, 400));
        assert(s.out.delivered_total == 0);
    }
    { // A count change in the other half invalidates the check.
        Sim s;
        s.toCarry();
        assert(s.until(PushState::BACK_OUT, 800));
        s.in.zone_supply_count = 1;
        s.in.zone_injured_count = 1;
        s.hold({});
        assert(s.until(PushState::TURN_SCAN, 400));
        assert(s.out.delivered_total == 0 && !s.out.first_ordinary_delivered);
    }
    { // Forward motion needs the opponent zone to stay clear.
        Sim s;
        s.in.distance_m = .8f;
        assert(s.until(PushState::APPROACH, 10));
        s.tick();
        assert(s.out.motion.vx_mps > 0);
        s.in.opponent_zone_clear = false;
        s.tick();
        assert(stopped(s.out.motion));
        s.in.opponent_zone_clear = true;
        s.in.path_safe = false;
        s.tick();
        assert(stopped(s.out.motion));
    }
    { // A load is never released where the opponent zone may receive it.
        Sim s;
        s.toCarry();
        s.in.opponent_zone_clear = false;
        s.hold({});
        assert(s.until(PushState::LOST_HOLD, 200) && s.out.motion.gripper_offset == 20);
        assert(s.until(PushState::SAFE_STOP, 80) && s.out.reason == "drop_blocked_opponent_zone");
        for (int i = 0; i < 5; ++i)
            assert(stopped(s.tick().motion) && s.out.state == PushState::SAFE_STOP);
    }
    { // Reversing needs explicit rear clearance.
        Sim s;
        s.toCarry();
        assert(s.until(PushState::BACK_OUT, 800));
        s.in.retreat_safe = false;
        for (int i = 0; i < 5; ++i)
            assert(stopped(s.tick().motion) && s.out.state == PushState::BACK_OUT);
        s.in.retreat_safe = true;
        s.tick();
        assert(s.out.motion.vx_mps < 0);
    }
    { // A short-term predicted zone pose cannot prove physical retreat or permit more reversing.
        Sim s;
        s.toCarry();
        assert(s.until(PushState::BACK_OUT, 800));
        s.zone_predicted = true;
        for (int i = 0; i < 5; ++i)
            assert(stopped(s.tick().motion) && s.out.state == PushState::BACK_OUT);
    }
    { // No trusted zone estimate: carry only searches in place, never drives blind.
        Sim s;
        s.toCarry();
        s.zone_visible = false;
        for (int i = 0; i < 10; ++i) {
            s.tick();
            assert(s.out.state == PushState::CARRY && s.out.motion.vx_mps == 0);
        }
        s.zone_visible = true;
        bool resumed = false;
        for (int i = 0; i < 40; ++i) {
            s.tick();
            resumed = resumed || s.out.motion.vx_mps > 0;
        }
        assert(resumed);
    }
}

void fieldCarryCases() {
    { // field_carry (main path): transport/release/push logic follows the demo, capture stays
      // strict.
        TaskTuning t;
        t.field_carry = true;
        { // The load vanishing from the NEAR view during the carry is not a LOST_HOLD; without a
          // usable drop plan the trip keeps searching and never releases blind.
            Sim s(t);
            s.toCarry();
            s.tick();
            s.hold({});
            s.drop_available = false;
            bool lost = false;
            for (int i = 0; i < 400 && s.out.state != PushState::RAISE_RELEASE; ++i) {
                s.tick();
                lost = lost || s.out.state == PushState::LOST_HOLD;
            }
            assert(!lost && s.out.state == PushState::CARRY &&
                   s.out.reason == "searching_own_zone");
        }
        { // No verified own zone (enemy colour / unseen): the load is held and the robot searches,
          // never a blind release.
            Sim s(t);
            s.toCarry();
            s.zone_visible = false;
            for (int i = 0; i < 400; ++i) { // 20 s, inside carry_budget_us
                s.tick();
                assert(s.out.state != PushState::RAISE_RELEASE && s.out.state != PushState::ENTER);
                assert(s.out.reason != "assumed_blind_carry" &&
                       s.out.reason != "assumed_drop_point");
            }
            assert(s.out.state == PushState::CARRY && s.out.reason == "searching_own_zone");
        }
        { // Enemy-colour zone seen while searching: turn about (IMU), drive a long leg away, then
          // resume searching our zone.
            Sim s(t);
            s.toCarry();
            s.zone_visible = false;
            for (int i = 0; i < 200 && s.out.reason != "searching_own_zone"; ++i)
                s.tick();
            assert(s.out.reason == "searching_own_zone");
            const float phi0 = s.phi;
            s.in.enemy_zone_seen = true;
            s.tick();
            s.tick(); // below the 3-frame debounce
            s.in.enemy_zone_seen = false;
            s.tick();
            assert(s.out.reason == "searching_own_zone");
            s.in.enemy_zone_seen = true;
            for (int i = 0; i < 3; ++i)
                s.tick();
            s.in.enemy_zone_seen = false;
            assert(s.out.reason == "enemy_zone_turn_away");
            bool turned = false, drove = false;
            float x0 = 0, y0 = 0;
            for (int i = 0; i < 400 && s.out.reason != "searching_own_zone"; ++i) {
                s.tick();
                if (s.out.reason == "enemy_zone_drive_away") {
                    if (!drove) {
                        drove = true;
                        x0 = s.px;
                        y0 = s.py;
                        turned = std::abs(s.phi - phi0) >= s.tune.enemy_evade_turn_rad;
                    }
                    assert(s.out.motion.vx_mps > 0 && s.out.state == PushState::CARRY);
                }
                assert(s.out.state == PushState::CARRY);
            }
            assert(turned && drove && s.out.reason == "searching_own_zone");
            assert(std::abs(std::hypot(s.px - x0, s.py - y0) -
                            s.tune.enemy_evade_m * s.tune.assume_motion_ratio) < .08f);
            // Cooldown: an immediate re-sighting does not restart the manoeuvre.
            s.in.enemy_zone_seen = true;
            for (int i = 0; i < 4; ++i) {
                s.tick();
                assert(s.out.reason == "searching_own_zone");
            }
        }
        { // Without the enemy sighting nothing changes; with it disabled the sighting is ignored.
            TaskTuning off = t;
            off.enemy_evade = false;
            Sim s(off);
            s.toCarry();
            s.zone_visible = false;
            for (int i = 0; i < 200 && s.out.reason != "searching_own_zone"; ++i)
                s.tick();
            s.in.enemy_zone_seen = true;
            for (int i = 0; i < 40; ++i) {
                s.tick();
                assert(s.out.reason == "searching_own_zone");
            }
        }
        { // Zone never found: after carry_search_turns turns the robot drives back along the start
          // line, then spins again.
            TaskTuning q = t;
            q.carry_search_turns = .5f;
            q.carry_leg_us = 2000000;
            q.carry_search_respin_turns = .25f;
            Sim s(q);
            s.toCarry();
            s.zone_visible = false;
            bool drove = false, respun = false;
            for (int i = 0; i < 1200 && !respun; ++i) {
                s.tick();
                assert(s.out.state == PushState::CARRY);
                if (s.out.reason == "carry_drive_start_line") {
                    drove = true;
                } else if (drove && s.out.reason == "searching_own_zone")
                    respun = true;
            }
            assert(drove && respun);
        }
        { // A far sighting (no geometry fix) ends the spin: face it and drive; the memory outlives
          // the sighting; an enemy-colour sighting there erases it.
            Sim s(t);
            s.toCarry();
            s.zone_visible = false;
            for (int i = 0; i < 40 && s.out.reason != "searching_own_zone"; ++i)
                s.tick();
            assert(s.out.reason == "searching_own_zone");
            const float zyaw = s.phi + .5f;
            s.in.zone_sight_valid = true;
            s.in.zone_sight_bearing = .5f;
            s.tick();
            s.in.zone_sight_valid = false;
            bool sighted = false, drove = false;
            for (int i = 0; i < 200; ++i) {
                s.tick();
                assert(s.out.state == PushState::CARRY);
                sighted = sighted || s.out.reason == "carry_drive_to_sighted_zone";
                if (s.out.reason == "carry_drive_to_remembered_zone" && s.out.motion.vx_mps > 0)
                    drove = true;
            }
            assert(sighted && drove);
            s.in.zone_sight_valid = true;
            s.in.zone_sight_enemy = true;
            s.in.zone_sight_bearing = std::remainder(zyaw - s.phi, 2 * float(CV_PI));
            s.tick();
            s.in.zone_sight_valid = false;
            s.in.zone_sight_enemy = false;
            for (int i = 0; i < 5; ++i) {
                s.tick();
                assert(s.out.reason != "carry_drive_to_remembered_zone" &&
                       s.out.reason != "carry_drive_to_sighted_zone");
            }
        }
        { // Strict capture is untouched: without multi-view evidence the capture is not credited.
            Sim s(t);
            s.tune.require_multi_view = true;
            s.task = PushTask(s.tune);
            s.multi_evidence = false;
            s.toRush();
            s.hold(inv(1));
            for (int i = 0; i < 200; ++i) {
                s.tick();
                assert(s.out.state != PushState::CARRY);
                assert(s.out.reason != "assumed_capture");
            }
        }
        { // Without field_carry the same loss still ends the trip in LOST_HOLD (strict path
          // unchanged).
            Sim s;
            s.toCarry();
            s.tick();
            s.hold({});
            assert(s.until(PushState::LOST_HOLD, 200));
        }
    }
}
