#include "rescue/task/transport_rules.hpp"
#include "rescue/hardware/motion_link.hpp"
#include "rescue/core/types.hpp"
#include "rescue/geometry/zone_estimate.hpp"
#include "rescue/geometry/zone_dead_reckoning.hpp"
#include "rescue/geometry/planner.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#if __has_include("rescue/task/capture_verification_stage.hpp")
#include "rescue/task/capture_verification_stage.hpp"
#endif
#define private public
#include "rescue/task/push_task.hpp"
#undef private

namespace {
using namespace rescue;
constexpr uint64_t kStart = 1000000;
constexpr uint64_t kTick = 50000;

struct Rig {
    PushTask task;
    PushObservation input;
    explicit Rig(TaskTuning tuning = {}) : task(tuning) {
        input.now_us = kStart;
        input.run = input.safety_ok = true;
        input.path_safe = input.retreat_safe = input.opponent_zone_clear = true;
        input.heading_valid = true;
        input.camera_pitch_stable = true;
        input.camera_pitch_cdeg = tuning.far_pitch_cdeg;
        task.last_us_ = kStart;
    }
    PushOutput tick() {
        input.now_us += kTick;
        return task.update(input);
    }
    uint64_t imu_sequence = 0;
    float plant_velocity = 0, plant_position = 0;
    float gyro_noise_rps = 0, accel_noise_mps2 = 0;
    void supplyImu(float vx, float wz, bool samples = true) {
        input.imu_translation_samples.clear();
        const float accel = (vx - plant_velocity) / (kTick * 1e-6f);
        for (int i = 1; i <= 5; ++i) {
            const float next = plant_velocity + accel * .01f;
            plant_position += .5f * (plant_velocity + next) * .01f;
            plant_velocity = next;
            if (!samples)
                continue;
            ImuTranslationSample sample;
            sample.sequence = ++imu_sequence;
            sample.received_us = input.now_us + i * 10000;
            sample.device_time_ms = sample.received_us / 1000;
            sample.valid = true;
            const float noise_sign = sample.sequence % 2 ? 1.f : -1.f;
            sample.acceleration_mps2 = {accel + .01f + noise_sign * accel_noise_mps2, 0, 9.8f};
            sample.angular_velocity_rps = {0, 0, wz + noise_sign * gyro_noise_rps};
            sample.quaternion_wxyz = {std::cos(input.heading_rad / 2), 0, 0,
                                      std::sin(input.heading_rad / 2)};
            input.imu_translation_samples.push_back(sample);
        }
    }
    PushOutput motionTick(const PushOutput& previous, bool moving = true, bool samples = true) {
        input.heading_rad = wrapAngle(input.heading_rad +
                                      previous.motion.wz_rps * (kTick * 1e-6f));
        input.sent_velocity_valid = true;
        input.sent_vx_mps = moving ? previous.motion.vx_mps : 0;
        input.sent_wz_rps = previous.motion.wz_rps;
        supplyImu(input.sent_vx_mps, input.sent_wz_rps, samples);
        return tick();
    }
};



void startupOrdinaryLoggedJitterUsesOneStoppedWindow() {
    TaskTuning t;t.startup_advance_us=2000000;t.start_zone=3;t.pixel_grasp_enabled=true;
    t.pixel_align_brake_motion_px=30;
    Rig rig(t);rig.tick();
    auto& in=rig.input;
    rig.task.startup_action_=PushTask::StartupAction::SelectOrdinary;
    rig.task.startup_pitch_=in.camera_pitch_cdeg=t.track_pitch_cdeg;
    in.target_valid=in.target_center_px_valid=in.pixel_grasp_view=in.pixel_grasp_target_valid=true;
    in.target_id=125;in.label="ordinary_supply";in.startup_visible_classes=1;
    in.sent_velocity_valid=in.gripper_done=true;in.gripper_feedback_open=0;
    const auto sample=[&](float error) {
        in.pixel_grasp_error_px=error;in.target_center_px={640-error,420};
        in.startup_scene_us=in.pixel_grasp_sample_us=in.now_us+kTick;
        auto out=rig.tick();in.sent_vx_mps=out.motion.vx_mps;in.sent_wz_rps=out.motion.wz_rps;
        return out;
    };
    const auto start=in.now_us;PushOutput out;
    for(int i=0;i<12;++i) {
        out=sample(26.f+10.f*(i%3));
        assert(out.motion.wz_rps==0);
        if(out.reason=="startup_nearest_ordinary_open")break;
    }
    assert(out.reason=="startup_nearest_ordinary_open");
    assert(in.now_us-start<=600000 && std::abs(in.pixel_grasp_error_px)<=30);
    assert(t.pixel_align_approach_restart_px==60);
    in.gripper_feedback_open=1;
    for(int i=0;i<15;++i) {out=sample(26);if(out.motion.vx_mps>0)break;}
    assert(out.motion.vx_mps>0);
    // A single mild excursion holds the car, retaining alignment; recovery resumes.
    out=sample(66);assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
    out=sample(26);assert(out.motion.vx_mps>0);
    for(int i=0;i<3;++i) out=sample(66);
    assert(out.motion.vx_mps==0);
    assert(rig.task.pixel_alignment_.stage==PushTask::PixelAlignment::Stage::Brake);
    std::cout<<"PASS logged 26/36/46 px jitter, 60 px restart and sustained excursion\n";
}

// 04:23 field failure: open at -4 px, then drift before the first forward tick.
// Keep the real opening transition, feedback wait and alignment state machine.
void startupOpenDriftBeforeFirstAdvance() {
    for (int feedback_ticks : {2, 65}) for (bool repeated : {false, true}) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        t.pixel_align_brake_motion_px = 30;
        t.pixel_align_approach_restart_px = 40; // Historical 04:23 drift replay.
        t.gripper_timeout_us = 6000000;
        Rig rig(t);
        rig.tick();
        rig.task.startup_action_ = PushTask::StartupAction::SelectOrdinary;
        rig.task.startup_pitch_ = t.track_pitch_cdeg;
        auto& in = rig.input;
        in.camera_pitch_cdeg = t.track_pitch_cdeg;
        in.target_valid = in.target_center_px_valid = true;
        in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
        in.target_id = 106;
        in.label = "ordinary_supply";
        in.startup_visible_classes = 1;
        in.sent_velocity_valid = in.gripper_done = true;
        in.gripper_feedback_open = 0;
        const auto sample = [&](float error, bool fresh = true) {
            in.pixel_grasp_error_px = error;
            in.target_center_px = {640 - error, 420};
            if (fresh)
                in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
            const auto out = rig.tick();
            in.sent_vx_mps = out.motion.vx_mps;
            in.sent_wz_rps = out.motion.wz_rps;
            return out;
        };
        PushOutput out;
        for (int n = 0; n < 30; ++n) {
            out = sample(-4);
            if (out.reason == "startup_nearest_ordinary_open") break;
        }
        assert(out.reason == "startup_nearest_ordinary_open");
        const auto transaction = rig.task.frame_transaction_;
        // Waiting for the jaw must use its own deadline, even beyond three seconds.
        for (int n = 0; n < feedback_ticks; ++n) {
            out = sample(16);
            assert(out.reason == "gripper_feedback_wait");
            assert(out.run_status != RunStatus::Fault);
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        }
        in.gripper_feedback_open = 1;
        // Still inside the approach band, but no stable confirmation before drift.
        for (int n = 0; n < 4; ++n) {
            out = sample(16);
            assert(out.motion.vx_mps == 0);
        }
        // Correction starts from this new offset, not the pre-open -4 px.
        for (int n = 0; n < 52; ++n) {
            out = sample(n % 2 ? 56 : 46);
            assert(out.run_status != RunStatus::Fault);
            assert(out.motion.vx_mps == 0);
            assert(rig.task.startupSightTargetId() == 106);
        }
        if (repeated) {
            // Re-reading one image cannot prove that the correction succeeded.
            out = sample(26);
            const auto progress = rig.task.startup_alignment_since_us_;
            const auto samples = rig.task.startup_alignment_error_count_;
            for (int n = 0; n < 4; ++n) {
                out = sample(26, false);
                assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
                assert(rig.task.startup_alignment_since_us_ == progress);
                assert(rig.task.startup_alignment_error_count_ == samples);
            }
        }
        {
            // Bounded detector jitter returns to the approach acceptance band.
            bool advanced = false;
            for (int n = 0; n < 40; ++n) {
                out = sample(n % 2 ? 26 : 16);
                assert(out.run_status != RunStatus::Fault);
                assert(rig.task.startupSightTargetId() == 106);
                assert(rig.task.frame_transaction_ == transaction);
                if (out.motion.vx_mps > 0) { advanced = true; break; }
            }
            assert(advanced && out.reason == "startup_open_approach_120");
        }
    }
    std::cout << "PASS actual opening -> long feedback wait -> pre-forward drift -> fresh stable recovery\n";
}

// Minimized 04:46:37 log: an early in-band sample sets the progress floor
// to zero, then correction returns to Settle just before the 3 s deadline.
void startupOpenFinishesPendingSettle() {
    for (bool drifts_again : {false, true}) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        t.pixel_align_brake_motion_px = 30;
        t.pixel_align_approach_restart_px = 40; // Freeze the historical watchdog replay limits.
        Rig rig(t);
        rig.tick();
        rig.task.startup_action_ = PushTask::StartupAction::SelectOrdinary;
        rig.task.startup_pitch_ = t.track_pitch_cdeg;
        auto& in = rig.input;
        in.camera_pitch_cdeg = t.track_pitch_cdeg;
        in.target_valid = in.target_center_px_valid = true;
        in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
        in.target_id = 88;
        in.label = "ordinary_supply";
        in.startup_visible_classes = 1;
        in.sent_velocity_valid = in.gripper_done = true;
        in.gripper_feedback_open = 0;
        const auto sample = [&](float error) {
            in.pixel_grasp_error_px = error;
            in.target_center_px = {640 - error, 485};
            in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick - 40000;
            const auto out = rig.tick();
            in.sent_vx_mps = out.motion.vx_mps;
            in.sent_wz_rps = out.motion.wz_rps;
            return out;
        };
        PushOutput out;
        for (int n = 0; n < 30; ++n) {
            out = sample(6);
            if (out.reason == "startup_nearest_ordinary_open") break;
        }
        assert(out.reason == "startup_nearest_ordinary_open");
        out = sample(16);
        assert(out.reason == "gripper_feedback_wait");
        in.gripper_feedback_open = 1;
        for (float error : {16.f, 16.f, 36.f, 26.f, 26.f, 26.f})
            out = sample(error);
        assert(rig.task.startup_alignment_best_error_px_ == 0);
        const auto deadline = rig.task.startup_alignment_since_us_ + 3000000;
        while (in.now_us + kTick < deadline - 100000) {
            out = sample(56);
            assert(out.run_status != RunStatus::Fault && out.motion.vx_mps == 0);
        }
        // Seed the valid in-flight confirmation at the watchdog seam. Its
        // combined 250 ms window now finishes after, rather than before, the
        // deadline. Correction-pulse timing is covered by the closed-loop tests.
        auto& align=rig.task.pixel_alignment_;
        align.stage=PushTask::PixelAlignment::Stage::Settle;
        align.stopped_since_us=deadline-400000;
        align.stable_since_us=deadline-200000;
        align.stable_frames=3;
        align.brake_min_px=align.brake_max_px=26;
        align.within_tolerance=true;
        align.resume_pending=false;
        align.advance_outside_since_us=0;align.advance_outside_frames=0;
        in.sent_vx_mps=in.sent_wz_rps=0;
        bool crossed_deadline = false;
        for (int n = 0; n < 20; ++n) {
            const bool expired = in.now_us + kTick >= deadline;
            out = sample(drifts_again && crossed_deadline ? 56 : 26);
            crossed_deadline |= expired;
            assert(out.run_status == RunStatus::Running);
            if (out.reason == "startup_alignment_reselect" || out.motion.vx_mps > 0) break;
            assert(rig.task.startupSightTargetId() == 88);
        }
        assert(crossed_deadline);
        if (drifts_again) {
            assert(out.run_status == RunStatus::Running);
            assert(out.reason == "startup_alignment_reselect");
            assert(rig.task.startupSightTargetId() == -1);
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        } else {
            assert(out.run_status != RunStatus::Fault);
            assert(out.reason == "startup_open_approach_120" && out.motion.vx_mps > 0);
        }
        assert(in.now_us < deadline + t.pixel_align_stable_us + t.frame_timeout_us + kTick);
    }
    std::cout << "PASS pending stopped confirmation finishes at deadline; renewed drift reselects without fault\n";
}

void startupOpeningApproachToleranceBoundary() {
    for (float tolerance : {30.f, 40.f})
    for (bool near : {false, true})
    for (float error : {-tolerance - .5f, -tolerance, -26.f, -16.f,
                        16.f, 26.f, tolerance, tolerance + .5f}) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        t.pixel_align_approach_tolerance_px = tolerance;
        Rig rig(t);
        rig.tick();
        rig.task.startup_action_ = PushTask::StartupAction::SelectOrdinary;
        rig.task.startup_pitch_ = near ? t.near_pitch_cdeg : t.track_pitch_cdeg;
        auto& in = rig.input;
        in.camera_pitch_cdeg = rig.task.startup_pitch_;
        in.pixel_grasp_near_view = near;
        in.target_valid = in.target_center_px_valid = true;
        in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
        in.target_id = 106;
        in.label = "ordinary_supply";
        in.startup_visible_classes = 1;
        in.sent_velocity_valid = in.gripper_done = true;
        in.gripper_feedback_open = 0;
        bool opened = false;
        for (int n = 0; n < 20; ++n) {
            in.pixel_grasp_error_px = error;
            in.target_center_px = {640 - error, 420};
            in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
            const auto out = rig.tick();
            assert(out.motion.vx_mps == 0);
            opened |= out.reason == "startup_nearest_ordinary_open";
            if (opened) break;
        }
        assert(opened == (std::abs(error) <= tolerance));
    }
    std::cout << "PASS opening uses approach tolerance in both views, including 16/26 px offsets\n";
}

void startupOpenJitterCannotRenewProgressForever() {
    for (bool jitter : {false, true}) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        t.pixel_align_brake_motion_px = 30;
        Rig rig(t);
        rig.tick();
        rig.task.startup_action_ = PushTask::StartupAction::OpenApproach;
        rig.task.startup_pitch_ = t.track_pitch_cdeg;
        rig.task.frame_raised_ = 1;
        auto& in = rig.input;
        in.camera_pitch_cdeg = t.track_pitch_cdeg;
        in.target_valid = in.target_center_px_valid = true;
        in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
        in.target_id = 106;
        in.label = "ordinary_supply";
        in.startup_visible_classes = 1;
        in.sent_velocity_valid = in.gripper_done = true;
        in.gripper_feedback_open = 1;
        PushOutput out;
        for (int n = 0; n < 100; ++n) {
            in.pixel_grasp_error_px = jitter ? (n % 2 ? 56 : 16) : 46;
            in.target_center_px = {640 - in.pixel_grasp_error_px, 420};
            in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
            out = rig.tick();
            in.sent_wz_rps = out.motion.wz_rps;
            assert(out.motion.vx_mps == 0);
            assert(out.run_status == RunStatus::Running);
            if (out.reason == "startup_alignment_reselect") break;
            assert(rig.task.startupSightTargetId() == 106);
        }
        assert(out.reason == "startup_alignment_reselect");
        assert(rig.task.startupSightTargetId() == -1);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    }
    std::cout << "PASS fixed offset and non-converging jitter release the target within a bounded correction window\n";
}

// Log 19:24:33: UI120 stays at errors 16/26/36 px through repeated tiny
// corrections. Exercise the real selector with a delayed chassis and detector
// jitter; both the fast existing plant and a slower response must finish.
// 21:20:55 field run: #126 opens at 6 px, then 46/36 px corrections
// must be judged against the new episode, not an unreachable historic minimum.

void startupOpenApproachRetainsFirstTarget() {
    TaskTuning t;
    t.startup_advance_us = 2000000;
    t.start_zone = 3;
    t.pixel_grasp_enabled = true;
    Rig rig(t);
    rig.tick();
    rig.task.startup_action_ = PushTask::StartupAction::OpenApproach;
    rig.task.startup_pitch_ = t.track_pitch_cdeg;
    rig.task.frame_raised_ = 1;
    auto& in = rig.input;
    in.camera_pitch_cdeg = t.track_pitch_cdeg;
    in.gripper_done = in.sent_velocity_valid = true;
    in.gripper_feedback_open = 1;
    in.target_valid = in.target_center_px_valid = true;
    in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
    in.target_id = 126;
    in.label = "ordinary_supply";
    in.startup_visible_classes = 1;
    const auto sample = [&](float error) {
        in.pixel_grasp_error_px = error;
        in.target_center_px = {640 - error, 445};
        in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
        return rig.tick();
    };
    for (int i = 0; i < 20; ++i) {
        if (sample(6).motion.vx_mps > 0) break;
    }
    for (int i = 0; i < 70; ++i) {
        const auto out = sample(i < 30 ? 46 : 36);
        assert(out.run_status != RunStatus::Fault);
        assert(rig.task.startupSightTargetId() == 126);
        assert(out.reason != "startup_alignment_reselect");
    }
    PushOutput out;
    for (int i = 0; i < 70 && out.reason != "startup_alignment_reselect"; ++i)
        out = sample(36);
    assert(out.run_status == RunStatus::Running);
    assert(out.reason == "startup_alignment_reselect");
    assert(rig.task.startupSightTargetId() == -1);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    std::cout << "PASS first ordinary retained through recentering; real stall releases cargo without fault\n";
}

void startupOpenFinePulseWaitsForSentMotion() {
    TaskTuning t;
    t.startup_advance_us = 2000000;
    t.start_zone = 3;
    t.pixel_grasp_enabled = true;
    Rig rig(t);
    rig.tick();
    rig.task.startup_action_ = PushTask::StartupAction::OpenApproach;
    rig.task.startup_pitch_ = t.track_pitch_cdeg;
    rig.task.frame_raised_ = 1;
    auto& in = rig.input;
    in.camera_pitch_cdeg = t.track_pitch_cdeg;
    in.gripper_done = in.sent_velocity_valid = true;
    in.gripper_feedback_open = 1;
    in.target_valid = in.target_center_px_valid = true;
    in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
    in.target_id = 126;
    in.label = "ordinary_supply";
    in.startup_visible_classes = 1;
    in.pixel_grasp_error_px = 26 + 10;
    in.target_center_px = {604, 445};
    for (int i = 0; i < 4; ++i) {
        in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
        const auto out = rig.tick();
        assert(out.motion.wz_rps >= .06f && out.motion.wz_rps <= .1f);
    }
    in.sent_wz_rps = .072f;
    bool stopped = false;
    for (int i = 0; i < 4; ++i) {
        in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
        const auto out = rig.tick();
        if (out.motion.wz_rps == 0) { stopped = true; break; }
    }
    assert(stopped);
    std::cout << "PASS open-approach fine pulse measures sent correction, including ramp delay\n";
}

// Field 2026-11-04 23:32:52: #135 and the surviving neighbour #132
// share #101's old column. Choose a NEW enclosed ordinary, then review core #134.

// 2026-11-05 00:05:50: UI120 handoff flickers around Y540 while the
// old alignment deadline expires. Once crossed, finish a stopped handoff.
void startupOpenHandoffCommitsBeforeAlignmentTimeout() {
    for (int scenario = 0; scenario < 3; ++scenario) {
        const bool expired_alignment = scenario != 0;
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        t.pixel_align_brake_motion_px = 30;
        Rig rig(t);
        rig.task.setPhaseStepping(true);
        rig.tick();
        auto& task = rig.task;
        auto& in = rig.input;
        in.now_us += 3000000;
        task.last_us_ = in.now_us;
        task.startup_action_ = PushTask::StartupAction::OpenApproach;
        task.startup_pitch_ = t.track_pitch_cdeg;
        task.frame_raised_ = 1;
        task.trip_context_.target_id_ = task.trip_context_.capture_target_id_ = 96;
        task.trip_context_.label_ = "ordinary_supply";
        task.startup_alignment_target_id_ = 96;
        task.startup_alignment_best_error_px_ = 26;
        task.startup_alignment_since_us_ = in.now_us - (expired_alignment ? 2980000 : 0);
        in.camera_pitch_cdeg = t.track_pitch_cdeg;
        in.gripper_done = in.sent_velocity_valid = true;
        in.gripper_feedback_open = 1;
        in.target_valid = in.target_center_px_valid = true;
        in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
        in.target_id = 96;
        in.label = "ordinary_supply";
        in.startup_visible_classes = 1;
        const uint64_t started = in.now_us;
        PushOutput out;
        for (int n = 0; n < 25; ++n) {
            const float y = n == 0 ? 545.5f : n % 2 ? 535.5f : 555.5f;
            in.pixel_grasp_error_px = n % 3 == 0 ? 46 : n % 3 == 1 ? 36 : 26;
            in.target_center_px = {640 - in.pixel_grasp_error_px, y};
            in.target_box = {int(in.target_center_px.x) - 96, int(y) - 120, 192, 240};
            in.pixel_view_handoff_ready = y >= 540;
            in.target_valid = in.target_center_px_valid = in.pixel_grasp_target_valid =
                !(scenario == 2 && n == 4); // a short detection gap cannot cancel the stop
            in.sent_wz_rps = n < 3 ? .08f : 0;
            in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
            in.capture_image_objects = {{96, "ordinary_supply", in.target_box, true}};
            out = rig.tick();
            assert(out.run_status != RunStatus::Fault);
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            if (out.reason != "startup_switch_to_140")
                assert(task.startupSightTargetId() == 96);
            if (n < 3) assert(task.startup_pitch_ == t.track_pitch_cdeg);
            if (out.reason == "startup_switch_to_140") break;
        }
        assert(out.reason == "startup_switch_to_140");
        assert(in.now_us - started <= 1250000);
        std::cout << "handoff scenario=" << scenario
                  << " stop_to_switch_us=" << in.now_us - started << '\n';
        in.camera_pitch_cdeg = t.near_pitch_cdeg;
        in.pixel_grasp_near_view = true;
        in.capture_scene_valid = true;
        in.target_valid = in.target_center_px_valid = false;
        in.capture_class_regions_px["ordinary_supply"] = {209, 180, 862, 320};
        in.capture_image_objects = {{96, "ordinary_supply", {498, 220, 192, 240}, true}};
        for (int n = 0; n < 3; ++n) {
            in.capture_scene_us = in.now_us + kTick;
            out = rig.tick();
            assert(out.run_status != RunStatus::Fault);
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        }
        assert(!out.phase_waiting && out.phase == TaskPhase::Startup && task.startup_first_capture_);
        assert(!task.startup_handoff_.pending);
    }
    std::cout << "PASS logged UI120 handoff jitter commits before alignment timeout without prematurely completing Startup\n";
}


void startupOpenHandoffRequiresFreshStoppedImages() {
    for (bool missing_images : {false, true}) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        Rig rig(t);
        rig.tick();
        auto& task = rig.task;
        auto& in = rig.input;
        task.startup_action_ = PushTask::StartupAction::OpenApproach;
        task.startup_pitch_ = t.track_pitch_cdeg;
        task.frame_raised_ = 1;
        task.trip_context_.target_id_ = 96;
        task.trip_context_.label_ = "ordinary_supply";
        in.camera_pitch_cdeg = t.track_pitch_cdeg;
        in.gripper_done = in.sent_velocity_valid = true;
        in.gripper_feedback_open = 1;
        in.target_valid = in.target_center_px_valid = true;
        in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
        in.target_id = 96;
        in.label = "ordinary_supply";
        in.target_box = {498, 444, 192, 243};
        in.target_center_px = {594, 565.5f};
        in.pixel_grasp_error_px = 46;
        in.pixel_view_handoff_ready = true;
        PushOutput out;
        for (int n = 0; n < 35; ++n) {
            in.sent_wz_rps = missing_images ? 0 : .08f;
            if (!missing_images || n == 0)
                in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
            out = rig.tick();
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            assert(task.startup_pitch_ == t.track_pitch_cdeg);
            if (out.run_status == RunStatus::Fault) break;
        }
        assert(out.run_status == RunStatus::Fault);
        assert(out.reason == "startup_handoff_stop_timeout");
    }
    std::cout << "PASS handoff never switches on moving or stale images; bounded stop timeout\n";
}

// Replay 05:31: the UI120 lock #115 disappears before the pitch handoff.
// New IDs in the calibrated region are sufficient; old identity is irrelevant.
void startupHandoffUsesRegionWithoutIdentity() {
    TaskTuning t;
    t.startup_advance_us = 2000000;
    t.start_zone = 3;
    t.pixel_grasp_enabled = true;
    Rig rig(t);
    rig.tick();
    auto& task = rig.task;
    auto& in = rig.input;
    task.startup_action_ = PushTask::StartupAction::OpenApproach;
    task.startup_pitch_ = t.track_pitch_cdeg;
    task.frame_raised_ = 1;
    task.trip_context_.target_id_ = task.trip_context_.capture_target_id_ = 115;
    task.trip_context_.label_ = "ordinary_supply";
    task.startup_handoff_.pending = task.startup_handoff_.stopping = true;
    task.startup_handoff_.started_us = in.now_us;
    task.startup_handoff_.reference = {
        115, "ordinary_supply", {488, 540, 232, 180}, t.track_pitch_cdeg, in.now_us};
    in.camera_pitch_cdeg = t.track_pitch_cdeg;
    in.pixel_grasp_view = true;
    in.target_valid = in.target_center_px_valid = false;
    in.gripper_done = in.sent_velocity_valid = true;
    in.gripper_feedback_open = 1;
    PushOutput out;
    for (int i = 0; i < 20; ++i) {
        in.startup_scene_us = in.now_us + kTick;
        out = rig.tick();
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        assert(out.run_status != RunStatus::Fault);
        if (out.reason == "startup_switch_to_140") break;
    }
    assert(out.reason == "startup_switch_to_140");
    in.camera_pitch_cdeg = t.near_pitch_cdeg;
    in.pixel_grasp_near_view = in.capture_scene_valid = true;
    in.capture_class_regions_px["ordinary_supply"] = {209, 226, 862, 263};
    for (int i = 0; i < 3; ++i) {
        // The old ID is visible outside the region. The wanted object is in a
        // different column and changes ID on every fresh image.
        in.capture_image_objects = {
            {115, "ordinary_supply", {488, 0, 232, 100}, true},
            {143 + i, "ordinary_supply", {800, 270, 100, 100}, true}};
        in.capture_scene_us = in.now_us + kTick;
        out = rig.tick();
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        if (i < 2) assert(out.phase == TaskPhase::Startup);
    }
    assert(out.phase == TaskPhase::Startup);
    assert(task.trip_context_.target_id_ == 145);
    assert(task.startup_first_capture_);
    std::cout << "PASS lost UI120 identity and changing UI140 IDs accept wanted cargo in region\n";
}

void startupNearHandoffReselectsAndReviewsCore() {
    for (bool reverse_order : {false, true})
        for (bool with_core : {false, true}) {
            TaskTuning t;
            t.startup_advance_us = 2000000;
            t.start_zone = 3;
            t.pixel_grasp_enabled = true;
            Rig rig(t);
            rig.tick();
            auto& task = rig.task;
            auto& in = rig.input;
            task.startup_action_ = PushTask::StartupAction::OpenApproach;
            task.startup_pitch_ = t.near_pitch_cdeg;
            task.frame_raised_ = 1;
            task.trip_context_.target_id_ = task.trip_context_.capture_target_id_ = 101;
            task.trip_context_.label_ = "ordinary_supply";
            auto& handoff = task.startup_handoff_;
            handoff.pending = true;
            handoff.started_us = in.now_us;
            handoff.reference = {
                101, "ordinary_supply", {488, 560, 212, 152}, t.track_pitch_cdeg, in.now_us};
            in.camera_pitch_cdeg = t.near_pitch_cdeg;
            in.gripper_done = in.sent_velocity_valid = true;
            in.gripper_feedback_open = 1;
            in.pixel_grasp_view = in.pixel_grasp_near_view = true;
            in.capture_scene_valid = in.capture_path_valid = true;
            in.capture_center_membership = true;
            in.capture_class_regions_px["ordinary_supply"] = {209, 180, 862, 320};
            in.capture_class_regions_px["core_supply"] = {268.5f, 236, 748.5f, 319};
            in.capture_area_px = in.layout_count_region_px = {209, 180, 862, 320};
            in.capture_path_px = {209, 0, 862, 500};
            in.capture_image_objects = {{135, "ordinary_supply", {422, 4, 243, 193}, true},
                                        {134, "core_supply", {397, 479, 172, 132}, true},
                                        {132, "ordinary_supply", {482, 238, 224, 252}, true}};
            if (!with_core)
                in.capture_image_objects.erase(in.capture_image_objects.begin(),
                                               in.capture_image_objects.begin() + 2);
            if (reverse_order)
                std::reverse(in.capture_image_objects.begin(), in.capture_image_objects.end());
            const auto observe = [&] {
                in.capture_scene_us = in.now_us + kTick;
                return rig.tick();
            };
            in.camera_pitch_stable = false;
            auto out = observe();
            assert(out.phase == TaskPhase::Startup &&
                   (out.motion.vx_mps == 0 && out.motion.wz_rps == 0));
            in.camera_pitch_stable = true;
            in.sent_wz_rps = .2f;
            out = observe();
            assert(out.phase == TaskPhase::Startup &&
                   (out.motion.vx_mps == 0 && out.motion.wz_rps == 0));
            in.sent_wz_rps = 0;
            out = observe();
            assert(out.phase == TaskPhase::Startup &&
                   (out.motion.vx_mps == 0 && out.motion.wz_rps == 0));
            for (int i = 0; i < 3; ++i) {
                out = rig.tick(); // The same image must not count as new evidence.
                assert(out.phase == TaskPhase::Startup &&
                       (out.motion.vx_mps == 0 && out.motion.wz_rps == 0));
            }
            in.capture_scene_us = in.now_us - t.frame_timeout_us;
            out = rig.tick();
            assert(out.phase == TaskPhase::Startup && task.startup_handoff_.frames == 0);
            for (int i = 0; i < 3; ++i) {
                out = observe();
                if (i < 2)
                    assert(out.phase == TaskPhase::Startup);
            }
            assert(out.phase == TaskPhase::Startup && out.state == PushState::START_ADVANCE);
            assert(task.trip_context_.target_id_ == 132 && task.frame_raised_ == 1);
            assert(task.startup_first_capture_);
            assert((out.motion.vx_mps == 0 && out.motion.wz_rps == 0) && out.immediate_motion_stop);

            in.target_valid = in.target_center_px_valid = in.pixel_grasp_target_valid = true;
            in.target_id = 132;
            in.label = "ordinary_supply";
            in.target_box = {482, 238, 224, 252};
            in.target_center_px = {594, 364};
            in.pixel_grasp_error_px = 46;
            in.target_grasp_range_valid = in.target_grasp_range_inside = true;
            in.target_grasp_range_lateral_measured = in.target_grasp_range_lateral = true;
            in.corridor = {};
            in.corridor.ordinary = 1;
            in.corridor_complete = in.corridor_occlusion_free = true;
            // Core centre Y545 is outside ordinary Y500, inside core Y555.
            assert(!PushObservation::imageBoxCenterInRegion({397, 479, 172, 132},
                                                            in.layout_count_region_px));
            assert(in.splitInventoryOverlaps({397, 479, 172, 132}, "core_supply"));
            bool reviewed_core = false;
            for (int i = 0; i < 20 && !task.search_phase_.first_trip_split_.pending; ++i) {
                in.pixel_grasp_sample_us = in.now_us + kTick;
                out = observe();
                if (with_core) assert((out.motion.vx_mps == 0 && out.motion.wz_rps == 0));
                assert(task.frame_raised_ == 1 && out.run_status != RunStatus::Fault);
                reviewed_core |= task.capture_scene_.split_inventory.core == 1;
                if (!with_core && out.capture_scene_status == "clear")
                    break;
            }
            if (with_core) {
                assert(reviewed_core && task.search_phase_.first_trip_split_.pending);
                assert(out.capture_split_runs == 1 && out.capture_keep_inventory.core == 0);
            } else {
                assert(!task.search_phase_.first_trip_split_.pending);
                assert(task.startup_first_capture_ && out.capture_scene_status == "clear");
            }
        }
    std::cout << "PASS field handoff reselects enclosed ordinary and reviews class-specific core "
                 "before motion\n";
}

void startupNearHandoffSelectsEligibleOrdinary() {
    // Multiple eligible ordinary supplies are a new acquisition, not an
    // ambiguous identity. No eligible candidate still times out stopped.
    for (int scenario = 0; scenario < 6; ++scenario) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        Rig rig(t);
        rig.tick();
        auto& in = rig.input;
        auto& task = rig.task;
        task.startup_action_ = PushTask::StartupAction::OpenApproach;
        task.startup_pitch_ = t.near_pitch_cdeg;
        task.frame_raised_ = 1;
        task.trip_context_.target_id_ = 101;
        task.trip_context_.label_ = "ordinary_supply";
        auto& handoff = task.startup_handoff_;
        handoff.pending = true;
        handoff.started_us = in.now_us;
        handoff.reference = {
            101, "ordinary_supply", {500, 560, 200, 152}, t.track_pitch_cdeg, in.now_us};
        in.camera_pitch_cdeg = t.near_pitch_cdeg;
        in.pixel_grasp_near_view = in.pixel_grasp_view = true;
        in.gripper_done = in.sent_velocity_valid = in.capture_scene_valid = true;
        in.gripper_feedback_open = 1;
        in.capture_class_regions_px["ordinary_supply"] = {209, 180, 862, 320};
        if (scenario == 1) {
            in.capture_image_objects = {{135, "ordinary_supply", {500, 220, 200, 200}, true},
                                        {132, "ordinary_supply", {510, 240, 200, 200}, true}};
        } else if (scenario == 2) {
            in.capture_image_objects = {{132, "ordinary_supply", {500, 220, 200, 200}, false}};
        } else if (scenario == 3) {
            in.capture_image_objects = {{132, "ordinary_supply", {500, 0, 200, 100}, true}};
        } else if (scenario == 4) {
            in.capture_image_objects = {
                {132, "ordinary_supply", {500, 220, 200, 200}, true},
                {135, "core_supply", {510, 240, 200, 200}, true},
                {136, "dangerous_object", {510, 240, 200, 200}, true}};
            in.capture_image_objects.front().selectable = false;
        } else if (scenario == 5) {
            // Replay the two enclosed candidates from match-main-20261105-013105.
            handoff.reference.id = 123;
            in.capture_image_objects = {
                {139, "ordinary_supply", {427, 1, 253, 203}, true},
                {137, "ordinary_supply", {362, 172, 323, 303}, true},
                {135, "ordinary_supply", {629, 273, 252, 263}, true},
                {142, "dangerous_object", {608, 1, 233, 203}, true}};
        }
        PushOutput out;
        if (scenario == 1 || scenario == 5) {
            const int expected_id = scenario == 1 ? 132 : 135;
            for (int i = 0; i < 3; ++i) {
                std::reverse(in.capture_image_objects.begin(), in.capture_image_objects.end());
                in.capture_scene_us = in.now_us + kTick;
                out = rig.tick();
                assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
                if (i < 2) {
                    assert(out.phase == TaskPhase::Startup);
                    assert(task.startup_handoff_.candidate_id == expected_id);
                    if (i == 0) {
                        if (scenario == 1) {
                            // A different candidate becomes nearer; retain the
                            // eligible acquisition already being confirmed.
                            for (auto& object : in.capture_image_objects)
                                if (object.id == 135) object.box.y = 300;
                        } else {
                            // Old-target association becomes unique again.
                            // Reselection must not switch back to old identity.
                            in.capture_image_objects.erase(std::remove_if(
                                in.capture_image_objects.begin(), in.capture_image_objects.end(),
                                [](const auto& object) { return object.id == 139; }),
                                in.capture_image_objects.end());
                        }
                    }
                    for (int repeat = 0; repeat < 2; ++repeat) {
                        out = rig.tick();
                        assert(out.phase == TaskPhase::Startup);
                    }
                }
            }
            assert(out.phase == TaskPhase::Startup);
            assert(task.trip_context_.target_id_ == expected_id);
            assert(task.startup_first_capture_);
            assert(out.run_status != RunStatus::Fault && task.frame_raised_ == 1);
            continue;
        }
        for (int i = 0; i < 35 && out.run_status != RunStatus::Fault; ++i) {
            in.capture_scene_us = in.now_us + kTick;
            out = rig.tick();
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            assert(task.frame_raised_ == 1);
            assert(out.phase == TaskPhase::Startup);
        }
        assert(out.run_status == RunStatus::Fault);
        assert(out.reason == "startup_handoff_region_timeout");
        assert(task.trip_context_.target_id_ == -1);
    }
    std::cout << "PASS multiple ordinary candidates reselect; ineligible handoffs time out without "
                 "false rebinding\n";
}

// 14:27 field log: ordinary #132 is complete at (584, 203), above
// grasp Y226 but inside the Y180 observation strip. Acquisition must precede entry.
void startupNearHandoffObservesBeforeEntry() {
    for (int scenario = 0; scenario < 10; ++scenario) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        Rig rig(t);
        rig.tick();
        auto& task = rig.task;
        auto& in = rig.input;
        task.startup_action_ = PushTask::StartupAction::OpenApproach;
        task.startup_pitch_ = t.near_pitch_cdeg;
        task.frame_raised_ = 1;
        task.trip_context_.target_id_ = 112;
        task.trip_context_.label_ = "ordinary_supply";
        task.startup_handoff_.pending = true;
        task.startup_handoff_.started_us = in.now_us;
        task.startup_handoff_.reference = {
            112, "ordinary_supply", {508, 394, 192, 242}, t.track_pitch_cdeg, in.now_us};
        in.camera_pitch_cdeg = t.near_pitch_cdeg;
        in.gripper_done = in.sent_velocity_valid = true;
        in.gripper_feedback_open = 1;
        in.pixel_grasp_view = in.pixel_grasp_near_view = true;
        in.capture_scene_valid = in.capture_path_valid = true;
        in.capture_center_membership = in.capture_path_lateral_measured = true;
        in.capture_area_px = {209, 226, 862, 274};
        in.capture_class_regions_px["ordinary_supply"] = in.capture_area_px;
        in.layout_count_region_px = {209, 180, 862, 320};
        in.capture_path_px = {209, 205, 862, 295};
        in.capture_image_objects = {
            {139, "ordinary_supply", {0, 172, 162, 263}, false},
            {138, "dangerous_object", {769, 0, 193, 209}, false},
            {134, "ordinary_supply", {0, 0, 146, 189}, false},
            {132, "ordinary_supply", {478, 77, 212, 252}, true}};
        auto& target = in.capture_image_objects.back();
        if (scenario == 1) target.complete = false;
        if (scenario == 2) target.box.x = 0;
        if (scenario == 3) target.box.y = 40; // centre Y166, above observation
        if (scenario == 4) target.selectable = false;
        if (scenario == 5) target.box.y = 400; // centre Y526, past rear bound
        if (scenario == 6) in.layout_count_region_px = {};
        if (scenario == 7) {
            // An in-grasp candidate wins over an observation-only lock.
            task.startup_handoff_.candidate_id = 132;
            in.capture_image_objects.push_back(
                {150, "ordinary_supply", {700, 250, 100, 100}, true});
        }
        const auto observe = [&] {
            in.pixel_grasp_sample_us = in.capture_scene_us = in.now_us + kTick;
            return rig.tick();
        };
        PushOutput out;
        const bool eligible = scenario == 0 || scenario >= 7;
        if (!eligible) {
            for (int i = 0; i < 35 && out.run_status != RunStatus::Fault; ++i) {
                out = observe();
                assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
                assert(task.frame_raised_ == 1 && out.phase == TaskPhase::Startup);
            }
            assert(out.run_status == RunStatus::Fault);
            assert(out.reason == "startup_handoff_region_timeout");
            continue;
        }
        out = observe();
        assert(out.phase == TaskPhase::Startup);
        assert(task.startup_handoff_.candidate_id == (scenario == 7 ? 150 : 132));
        // Repeated scenes must not count toward the three-frame confirmation.
        for (int i = 0; i < 2; ++i) {
            out = rig.tick();
            assert(out.phase == TaskPhase::Startup && task.startup_handoff_.frames == 1);
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        }
        for (int i = 0; i < 2; ++i) {
            std::reverse(in.capture_image_objects.begin(), in.capture_image_objects.end());
            out = observe();
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            if (i == 0) assert(out.phase == TaskPhase::Startup);
        }
        assert(out.phase == TaskPhase::Startup && out.reason == "startup_near_target_selected");
        assert(task.trip_context_.target_id_ == (scenario == 7 ? 150 : 132));
        assert(task.startup_first_capture_ && task.frame_raised_ == 1);
        if (scenario == 7) continue;
        in.target_valid = in.target_center_px_valid = in.pixel_grasp_target_valid = true;
        in.target_id = 132;
        in.label = "ordinary_supply";
        in.target_box = {478, 77, 212, 252};
        in.target_center_px = {584, 203};
        in.pixel_grasp_error_px = 63.5f;
        in.target_grasp_range_valid = in.target_grasp_range_applicable = true;
        in.target_grasp_range_lateral_measured = in.target_grasp_range_lateral = true;
        in.target_grasp_range_inside = false;
        in.corridor_complete = in.corridor_occlusion_free = true;
        in.corridor.ordinary = 1;
        if (scenario == 8) {
            in.capture_image_objects.push_back(
                {160, "dangerous_object", {700, 250, 100, 100}, true});
        }
        if (scenario == 9) {
            in.target_valid = in.target_center_px_valid = in.pixel_grasp_target_valid = false;
            for (int i = 0; i < 5; ++i) {
                out = observe();
                assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
                assert(task.frame_raised_ == 1 && !task.search_entry_.confirmed);
            }
            continue;
        }
        if (scenario == 8) {
            for (int i = 0; i < 20 && !task.search_phase_.first_trip_split_.pending; ++i) {
                out = observe();
                assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
                assert(task.frame_raised_ == 1);
            }
            assert(task.search_phase_.first_trip_split_.pending);
            continue;
        }
        assert(task.startup_first_capture_);
        // A centred fresh target may approach, but cannot close above Y226.
        in.target_box.x = 542;
        in.target_center_px.x = 648;
        in.pixel_grasp_error_px = -.5f;
        for (auto& object : in.capture_image_objects)
            if (object.id == 132) object.box = in.target_box;
        bool advanced = false;
        for (int i = 0; i < 25 && !advanced; ++i) {
            out = observe();
            assert(task.frame_raised_ == 1 && !task.search_entry_.confirmed);
            assert(out.state != PushState::LOWER_FRAME);
            advanced = out.motion.vx_mps > 0;
        }
        assert(advanced && out.reason == "startup_open_approach_140");
        in.target_box.y = 124;
        in.target_center_px.y = 250;
        in.target_grasp_range_inside = true;
        for (auto& object : in.capture_image_objects)
            if (object.id == 132) object.box = in.target_box;
        for (int i = 0; i < 5; ++i) {
            out = observe();
            assert(out.phase == TaskPhase::Startup && task.frame_raised_ == 1);
        }
        // Startup reaches the existing Y337 close line, then hands directly to Capture.
        in.target_box.y = 211;
        in.target_center_px.y = 337;
        in.pixel_grasp_ready = true;
        for (auto& object : in.capture_image_objects)
            if (object.id == 132) object.box = in.target_box;
        for (int i = 0; i < 25 && out.phase == TaskPhase::Startup; ++i) out = observe();
        assert(out.phase == TaskPhase::Capture && out.state == PushState::LOWER_FRAME);

    }
    std::cout << "PASS field Y203 handoff acquires before entry, continues Startup approach and preserves grasp bounds\n";
}

void startupNearHandoffUsesOrdinaryRegion() {
    for (bool reverse_order : {false, true}) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        Rig rig(t);
        rig.tick();
        rig.task.startup_action_ = PushTask::StartupAction::OpenApproach;
        rig.task.startup_pitch_ = t.track_pitch_cdeg;
        rig.task.frame_raised_ = 1;
        auto& in = rig.input;
        in.camera_pitch_cdeg = t.track_pitch_cdeg;
        in.gripper_done = in.sent_velocity_valid = true;
        in.gripper_feedback_open = 1;
        in.target_valid = in.target_center_px_valid = true;
        in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
        in.target_id = 140;
        in.label = "ordinary_supply";
        in.startup_visible_classes = 1;
        in.target_box = {498, 464, 232, 256};
        in.target_center_px = {614, 592};
        in.pixel_grasp_error_px = 26;
        in.pixel_view_handoff_ready = true;
        in.capture_image_objects = {{140, "ordinary_supply", in.target_box, false},
                                   {149, "ordinary_supply", {774, 308, 163, 193}, true}};
        PushOutput out;
        for (int i = 0; i < 30; ++i) {
            in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
            out = rig.tick();
            if (out.reason == "startup_switch_to_140") break;
        }
        assert(out.reason == "startup_switch_to_140");
        in.camera_pitch_cdeg = t.near_pitch_cdeg;
        in.pixel_grasp_near_view = true;
        in.capture_scene_valid = true;
        in.target_valid = in.target_center_px_valid = false;
        in.capture_class_regions_px["ordinary_supply"] = {209, 226, 862, 274};
        // Both are inside the jaw region. The neighbour is deliberately first.
        in.capture_image_objects = {{149, "ordinary_supply", {639, 263, 292, 303}, true},
                                   {154, "ordinary_supply", {437, 127, 273, 313}, true}};
        if (reverse_order) std::reverse(in.capture_image_objects.begin(), in.capture_image_objects.end());
        const auto observe = [&] {
            in.capture_scene_us = in.now_us + kTick;
            return rig.tick();
        };
        out = observe();
        assert(out.phase == TaskPhase::Startup && out.motion.vx_mps == 0);
        // A repeated frame cannot complete the three-frame handoff.
        for (int i = 0; i < 4; ++i) {
            out = rig.tick();
            assert(out.phase == TaskPhase::Startup);
        }
        // The repeated image has expired; confirm the region with three fresh frames.
        in.capture_image_objects.push_back({160, "ordinary_supply", {480, 250, 200, 200}, true});
        for (int i = 0; i < 3; ++i) {
            std::reverse(in.capture_image_objects.begin(), in.capture_image_objects.end());
            out = observe();
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            if (i < 2) assert(out.phase == TaskPhase::Startup);
        }
        assert(out.reason == "startup_near_target_selected");
        assert(rig.task.trip_context_.target_id_ == 149);
        assert(rig.task.startup_first_capture_);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    }
    std::cout << "PASS UI120 ordinary handoff ignores detector order and requires three fresh UI140 frames with wanted cargo in region\n";
}

void startupOrdinaryFineCorrectionConverges() {
    for (float pixels_per_rad : {1000.f, 4000.f}) for (float sign : {-1.f, 1.f}) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        t.pixel_align_brake_motion_px = 30;
        Rig rig(t);
        rig.tick();
        rig.task.startup_action_ = PushTask::StartupAction::SelectOrdinary;
        rig.task.startup_pitch_ = t.track_pitch_cdeg;
        rig.task.frame_raised_ = 0;
        auto& in = rig.input;
        in.camera_pitch_cdeg = t.track_pitch_cdeg;
        in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
        in.target_valid = in.target_center_px_valid = true;
        in.target_id = 137;
        in.label = "ordinary_supply";
        in.startup_visible_classes = 1;
        in.sent_velocity_valid = in.gripper_done = true;
        in.gripper_feedback_open = 0;
        float error = sign * 36.f;
        std::array<float, 3> delay{};
        bool opened = false;
        for (int n = 0; n < 50; ++n) {
            error -= delay[0] * .05f * pixels_per_rad;
            // 10 px alternating box jitter matches the observed centre steps.
            in.pixel_grasp_error_px = error + sign * (n % 2 ? -10.f : 0.f);
            in.target_center_px = {640 - in.pixel_grasp_error_px, 420};
            in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
            const auto out = rig.tick();
            assert(out.motion.vx_mps == 0);
            assert(out.reason != "startup_alignment_reselect");
            if (out.reason == "startup_nearest_ordinary_open") {
                assert(std::abs(in.pixel_grasp_error_px) <= t.pixel_align_restart_px);
                assert(out.motion.wz_rps == 0 && in.sent_wz_rps == 0);
                opened = true;
                break;
            }
            in.sent_wz_rps = out.motion.wz_rps;
            delay[0] = delay[1]; delay[1] = delay[2]; delay[2] = out.motion.wz_rps;
        }
        std::cout << "ordinary_fine px_per_rad=" << pixels_per_rad << " sign=" << sign
                  << " elapsed_us=" << in.now_us - kStart << " opened=" << opened
                  << " error=" << error << std::endl;
        assert(opened);
    }
}

// Log 19:24:33: centering succeeds near the 3 s progress deadline, then
// opening inherits that deadline and rejects the target 121 ms later.
void startupOpeningStartsNewProgressWindow() {
    TaskTuning t;
    t.startup_advance_us = 2000000;
    t.start_zone = 3;
    t.pixel_grasp_enabled = true;
    t.pixel_align_brake_motion_px = 30;
    Rig rig(t);
    rig.tick();
    rig.task.startup_action_ = PushTask::StartupAction::SelectOrdinary;
    rig.task.startup_pitch_ = t.track_pitch_cdeg;
    rig.task.frame_raised_ = 0;
    auto& in = rig.input;
    in.camera_pitch_cdeg = t.track_pitch_cdeg;
    in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
    in.target_valid = in.target_center_px_valid = true;
    in.target_id = 137;
    in.label = "ordinary_supply";
    in.startup_visible_classes = 1;
    in.sent_velocity_valid = in.gripper_done = true;
    in.gripper_feedback_open = 0;
    const auto sample = [&](float error) {
        in.pixel_grasp_error_px = error;
        in.target_center_px = {640 - error, 420};
        in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
        return rig.tick();
    };
    sample(0);
    rig.task.startup_alignment_target_id_ = in.target_id;
    rig.task.startup_alignment_best_error_px_ = 26;
    rig.task.startup_alignment_since_us_ = in.now_us - 2450000;
    PushOutput out;
    for (int i = 0; i < 10; ++i) {
        out = sample(0);
        assert(out.reason != "startup_alignment_reselect");
        if (out.reason == "startup_nearest_ordinary_open") break;
    }
    assert(out.reason == "startup_nearest_ordinary_open");
    const auto transaction = rig.task.frame_transaction_;
    // Wait for the normal open acknowledgement across the old deadline.
    for (int i = 0; i < 8; ++i) {
        out = sample(16);
        assert(rig.task.startupSightTargetId() == 137);
        assert(out.reason == "gripper_feedback_wait");
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    }
    in.gripper_feedback_open = 1;
    bool advanced = false;
    for (int i = 0; i < 25; ++i) {
        out = sample(16);
        assert(rig.task.startupSightTargetId() == 137);
        assert(out.reason != "startup_alignment_reselect");
        assert(rig.task.frame_transaction_ == transaction);
        if (out.motion.vx_mps > 0) { advanced = true; break; }
    }
    assert(advanced && out.reason == "startup_open_approach_120");
    std::cout << "PASS centered ordinary retains its lock across opening and approach\n";
}

void startupOpensOnlyAfterCentering() {
    for (bool near : {false, true}) for (float error : {-448.f, 561.f}) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        Rig rig(t);
        rig.tick();
        rig.task.startup_action_ = PushTask::StartupAction::SelectOrdinary;
        rig.task.startup_pitch_ = near ? t.near_pitch_cdeg : t.track_pitch_cdeg;
        rig.task.frame_raised_ = 0;
        auto& in = rig.input;
        in.camera_pitch_cdeg = rig.task.startup_pitch_;
        in.pixel_grasp_near_view = near;
        in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
        in.target_valid = in.target_center_px_valid = true;
        in.target_id = 107;
        in.label = "ordinary_supply";
        in.target_center_px = {640 - error, 250};
        in.pixel_grasp_error_px = error;
        in.startup_visible_classes = 1;
        in.sent_velocity_valid = in.gripper_done = true;
        in.gripper_feedback_open = 0;
        const auto transaction = rig.task.frame_transaction_;
        bool corrected = false;
        for (int i = 0; i < 20; ++i) {
            in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
            const auto out = rig.tick();
            assert(rig.task.frame_raised_ == 0); // Logged side targets must not open the jaw.
            assert(out.motion.vx_mps == 0 && rig.task.frame_transaction_ == transaction);
            corrected |= out.motion.wz_rps * error > 0;
        }
        assert(corrected);
        in.pixel_grasp_error_px = 0;
        in.target_center_px.x = 640;
        bool opened = false;
        for (int i = 0; i < 40; ++i) {
            in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
            const auto out = rig.tick();
            assert(out.motion.vx_mps == 0);
            if (rig.task.frame_raised_ == 1) {
                assert(out.reason == "startup_nearest_ordinary_open");
                assert(out.motion.wz_rps == 0 && in.sent_vx_mps == 0 && in.sent_wz_rps == 0);
                opened = true;
                break;
            }
        }
        assert(opened && rig.task.frame_transaction_ == transaction + 1);
    }
    std::cout << "PASS Startup UI120/UI140 side targets stay closed until centered and stopped\n";
}

void startupReselectionKeepsJawOpen() {
    // Repeated observed loss, stationary alignment timeout, and class replacement.
    for (bool near : {false, true}) for (int trigger : {0, 1, 2}) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        Rig rig(t);
        rig.tick();
        rig.task.startup_action_ = PushTask::StartupAction::OpenApproach;
        rig.task.startup_pitch_ = near ? t.near_pitch_cdeg : t.track_pitch_cdeg;
        rig.task.frame_raised_ = 1;
        auto& in = rig.input;
        in.camera_pitch_cdeg = rig.task.startup_pitch_;
        in.pixel_grasp_near_view = near;
        in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
        in.gripper_done = in.sent_velocity_valid = true;
        in.gripper_feedback_open = 1;
        const auto transaction = rig.task.frame_transaction_;
        for (int cycle = 0; cycle < 3; ++cycle) {
            const int id = 107 + cycle;
            rig.task.trip_context_.target_id_ = id;
            rig.task.trip_context_.label_ = "ordinary_supply";
            rig.task.startup_alignment_target_id_ = id;
            rig.task.startup_alignment_since_us_ = in.now_us;
            PushOutput out;
            if (trigger == 1) {
                in.target_valid = in.target_center_px_valid = true;
                in.target_id = id;
                in.label = "ordinary_supply";
                in.target_center_px = {192, 250};
                in.pixel_grasp_error_px = 448;
                for (int i = 0; i < 65; ++i) {
                    in.pixel_grasp_sample_us = in.now_us + kTick;
                    out = rig.tick();
                    assert(out.motion.vx_mps == 0);
                    if (out.reason == "startup_alignment_reselect") break;
                }
                assert(out.reason == "startup_alignment_reselect");
                assert(out.run_status == RunStatus::Running && rig.task.startupSightTargetId() == -1);
                assert(rig.task.frame_raised_ == 1 && rig.task.frame_transaction_ == transaction);
            } else if (trigger == 2) {
                in.target_valid = in.target_center_px_valid = true;
                in.target_id = id;
                in.label = "dangerous_object";
                in.pixel_grasp_sample_us = in.now_us + kTick;
                out = rig.tick();
                assert(out.reason == "startup_nearest_ordinary_lost");
            } else {
                in.target_valid = in.target_center_px_valid = false;
                in.pixel_grasp_sample_us = 0;
                for (int i = 0; i <= int(kStartupTargetLossHoldUs / kTick); ++i) {
                    out = rig.tick();
                    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
                }
                assert(out.reason == "startup_target_reselect");
            }
            assert(rig.task.frame_raised_ == 1 && rig.task.frame_transaction_ == transaction);
            assert(rig.task.startup_action_ == PushTask::StartupAction::SelectOrdinary);
            // Searching with no new candidate must keep the jaw open as well.
            in.target_valid = in.target_center_px_valid = false;
            out = rig.tick();
            assert(out.reason == "startup_find_ordinary");
            assert(out.motion.vx_mps == 0);
            assert(rig.task.frame_raised_ == 1 && rig.task.frame_transaction_ == transaction);
            // Reacquire a centered ordinary, confirm rest and resume without another jaw action.
            in.target_valid = in.target_center_px_valid = true;
            in.target_id = id + 100;
            in.label = "ordinary_supply";
            in.target_center_px = {640, 250};
            in.pixel_grasp_error_px = 0;
            in.startup_visible_classes = 1;
            bool resumed = false;
            for (int i = 0; i < 40; ++i) {
                in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
                out = rig.tick();
                assert(rig.task.frame_raised_ == 1 && rig.task.frame_transaction_ == transaction);
                if (out.motion.vx_mps > 0) { resumed = true; break; }
            }
            assert(resumed && rig.task.startup_action_ == PushTask::StartupAction::OpenApproach);
        }
    }
    std::cout << "PASS Startup repeated loss/timeout/replacement keeps the open jaw and resumes\n";
}


void startupTargetLockSurvivesDetectionGap() {
    TaskTuning t;
    t.startup_advance_us=2000000; t.start_zone=3; t.pixel_grasp_enabled=true;
    Rig rig(t); rig.tick();
    rig.task.startup_action_=PushTask::StartupAction::OpenApproach;
    rig.task.startup_pitch_=t.track_pitch_cdeg;
    rig.task.trip_context_.target_id_=107;
    rig.task.trip_context_.label_="ordinary_supply";
    rig.task.frame_raised_=1;
    auto& in=rig.input;
    in.camera_pitch_cdeg=t.track_pitch_cdeg;
    in.gripper_done=in.sent_velocity_valid=true;in.gripper_feedback_open=1;
    // A visible neighbour must not replace the locked target during a short gap.
    in.target_valid=in.target_center_px_valid=true;
    in.target_id=108;in.label="ordinary_supply";
    in.pixel_grasp_view=in.pixel_grasp_target_valid=true;
    for(int i=0;i<20;++i) {
        in.pixel_grasp_sample_us=in.now_us+kTick;
        auto out=rig.tick();
        assert(rig.task.startupSightTargetId()==107);
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        assert(rig.task.frame_raised_==1);
    }
    in.target_id=107;in.pixel_grasp_error_px=0;in.target_center_px={640,250};
    in.pixel_grasp_sample_us=in.now_us+kTick;
    rig.tick();
    assert(rig.task.startupSightTargetId()==107);
    assert(rig.task.startup_target_missing_since_us_==0);
    std::cout<<"PASS Startup one-second gap retains target despite another visible identity\n";
}

void startupTargetLockTracksAlignmentProgress() {
    TaskTuning t;
    t.startup_advance_us=2000000;t.start_zone=3;t.pixel_grasp_enabled=true;
    Rig rig(t);rig.tick();
    rig.task.startup_action_=PushTask::StartupAction::SelectOrdinary;
    rig.task.startup_pitch_=t.track_pitch_cdeg;rig.task.frame_raised_=0;
    auto& in=rig.input;
    in.camera_pitch_cdeg=t.track_pitch_cdeg;
    in.gripper_done=in.sent_velocity_valid=true;in.gripper_feedback_open=0;
    in.target_valid=in.target_center_px_valid=true;
    in.pixel_grasp_view=in.pixel_grasp_target_valid=true;
    in.target_id=107;in.label="ordinary_supply";in.startup_visible_classes=1;
    // A previously centered target can shift while the robot is advancing.
    // Start the next correction episode from that new observed error.
    in.pixel_grasp_error_px=0;in.target_center_px={640,250};
    for(int i=0;i<2;++i) {
        in.startup_scene_us=in.pixel_grasp_sample_us=in.now_us+kTick;rig.tick();
    }
    in.sent_vx_mps=.07f;in.pixel_grasp_error_px=500;in.target_center_px={140,250};
    in.startup_scene_us=in.pixel_grasp_sample_us=in.now_us+kTick;rig.tick();
    in.sent_vx_mps=0;
    for(int i=0;i<100;++i) {
        in.pixel_grasp_error_px=500-i*3.f; // Five seconds of steady visual centering progress.
        in.target_center_px={640-in.pixel_grasp_error_px,250};
        in.startup_scene_us=in.pixel_grasp_sample_us=in.now_us+kTick;
        auto out=rig.tick();
        assert(rig.task.startupSightTargetId()==107);
        assert(out.reason!="startup_alignment_reselect");
        assert(out.motion.vx_mps==0 && rig.task.frame_raised_==0);
    }
    std::cout<<"PASS Startup steady angular alignment progress retains its target beyond three seconds\n";
}

void startupDisabledDoesNotRepeat() {
    TaskTuning tuning;
    tuning.startup_advance_us = 0;
    tuning.startup_advance_speed = .4f;
    Rig rig(tuning);
    rig.input.run = false;
    auto out = rig.tick();
    assert(out.phase == TaskPhase::Startup && out.run_status == RunStatus::Waiting);
    rig.input.run = true;
    out = rig.tick();
    assert(out.phase == TaskPhase::Startup && out.run_status == RunStatus::Running);
    const auto frame = out.motion.gripper_offset;
    const auto transaction = out.motion.frame_transaction;
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    out = rig.tick();
    assert(out.phase == TaskPhase::Search && out.search_action == SearchStatusAction::Scan);
    assert(out.reason == "startup_disabled" && out.startup_commanded_us == 0);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    assert(out.motion.gripper_offset == frame && out.motion.frame_transaction == transaction);
    rig.input.run = false;
    out = rig.tick();
    assert(out.phase == TaskPhase::Search && out.run_status == RunStatus::Paused);
    rig.input.run = true;
    out = rig.tick();
    assert(out.phase == TaskPhase::Search && out.run_status == RunStatus::Running);
    rig.input.reset = true;
    out = rig.tick();
    assert(out.phase == TaskPhase::Startup && out.run_status == RunStatus::Waiting);
}

void startupLowerCornersTrigger() {
    for (int scenario = 0; scenario < 6; ++scenario) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        t.far_pitch_cdeg = -3000;
        t.track_pitch_cdeg = 0;
        Rig rig(t);
        rig.tick();
        rig.task.startup_action_ = PushTask::StartupAction::Approach;
        rig.task.startup_pitch_ = scenario == 4 ? t.far_pitch_cdeg : t.track_pitch_cdeg;
        rig.task.trip_context_.target_id_ = 7;
        rig.task.trip_context_.label_ = "ordinary_supply";
        auto& in = rig.input;
        in.camera_pitch_cdeg = rig.task.startup_pitch_;
        in.camera_pitch_stable = scenario != 5;
        in.pixel_grasp_view = scenario != 4;
        in.gripper_done = true;
        in.gripper_feedback_open = 0;
        in.sent_velocity_valid = true;
        in.startup_visible_classes = 1;
        in.startup_lower_corners_occupied = scenario != 1;
        // Only scenario 1 supplies a target beyond the old Y540 line.
        in.target_valid = in.target_center_px_valid = scenario == 1;
        in.target_id = 7;
        in.label = "ordinary_supply";
        in.target_center_px = {640, 600};
        in.pixel_view_handoff_ready = scenario == 1;
        for (int frame = 0; frame < 30; ++frame) {
            in.pixel_grasp_sample_us = in.now_us + kTick;
            in.startup_scene_us = scenario == 2 ? in.now_us - t.frame_timeout_us
                : scenario == 3 ? in.now_us + 2 * kTick : in.now_us + kTick;
            const auto out = rig.tick();
            if (scenario == 0) {
                assert(out.reason == "startup_pitch_turn_ready");
                assert(out.immediate_motion_stop && out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
                assert(rig.task.frame_raised_ == 0);
                // Once committed, corners/target may disappear; brake before measured advance.
                in.startup_lower_corners_occupied = false;
                in.startup_scene_us = 0;
                in.sent_vx_mps = .07f;
                for (int i = 0; i < 8; ++i) {
                    const auto braking = rig.tick();
                    assert(braking.motion.vx_mps == 0 && braking.motion.wz_rps == 0);
                    assert(braking.immediate_motion_stop);
                }
                in.sent_vx_mps = 0;
                for (int i = 0; i < 20; ++i) {
                    const auto waiting = rig.tick();
                    assert(waiting.motion.vx_mps == 0 && waiting.motion.wz_rps == 0);
                }
                // The new forward segment cannot start without stationary IMU samples.
                assert(rig.task.startup_pitch_turn_.leg ==
                       PushTask::StartupPitchTurn::Leg::ForwardCalibrate);
                break;
            }
            assert(rig.task.startup_action_ != PushTask::StartupAction::TurnBeforePitch);
        }
    }
    std::cout << "PASS UI120 lower corners trigger without a target; Y threshold alone cannot trigger\n";
}

void startupCornerForwardDistance() {
    using Leg = PushTask::StartupPitchTurn::Leg;
    for (int zone : {1, 2, 3, 4}) for (int scenario = 0; scenario < 6; ++scenario) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = zone;
        t.pixel_grasp_enabled = true;
        t.far_pitch_cdeg = -3000; t.track_pitch_cdeg = 0; t.near_pitch_cdeg = 2000;
        Rig rig(t); rig.tick();
        rig.task.startup_action_ = PushTask::StartupAction::Approach;
        rig.task.startup_pitch_ = t.track_pitch_cdeg;
        auto& in = rig.input;
        in.camera_pitch_cdeg = t.track_pitch_cdeg;
        in.gripper_done = true; in.gripper_feedback_open = 0;
        in.sent_velocity_valid = true;
        in.startup_visible_classes = 1;
        in.startup_scene_us = in.now_us + kTick;
        in.startup_lower_corners_occupied = true;
        auto out = rig.tick();
        assert(out.reason == "startup_pitch_turn_ready");
        in.startup_lower_corners_occupied = false;
        in.startup_scene_us = 0; // Committed maneuver no longer needs a visible target.
        bool moved = false, intervened = false, turned = false;
        for (int frame = 0; frame < 250; ++frame) {
            out = rig.motionTick(out);
            assert(out.motion.camera_pitch_cdeg == t.track_pitch_cdeg && rig.task.frame_raised_ == 0);
            if (out.motion.vx_mps > 0) {
                moved = true;
                assert(out.motion.vx_mps <= t.approach_speed && out.motion.wz_rps == 0);
            }
            if (!intervened && rig.task.startup_pitch_turn_.leg == Leg::Forward &&
                rig.plant_position >= .025f) {
                intervened = true;
                if (scenario == 1) {
                    in.run = false;
                    out = rig.motionTick(out, false);
                    assert(out.run_status == RunStatus::Paused && out.motion.vx_mps == 0);
                    const float retained = rig.task.startup_pitch_turn_.translation_completed_m;
                    assert(retained > .015f && retained < .08f);
                    for (int n = 0; n < 8; ++n) out = rig.motionTick(out, false);
                    assert(rig.task.startup_pitch_turn_.translation_completed_m == retained);
                    in.run = true;
                }
                if (scenario == 2 || scenario == 5) {
                    in.path_safe = scenario != 2;
                    const float before = rig.task.startup_pitch_turn_.translation_imu.distance();
                    for (int n = 0; n < 25; ++n) {
                        out = rig.motionTick(out, false);
                        assert(out.motion.wz_rps == 0);
                        if (scenario == 2) assert(out.motion.vx_mps == 0);
                        assert(rig.task.startup_pitch_turn_.leg == Leg::Forward);
                    }
                    assert(std::abs(rig.task.startup_pitch_turn_.translation_imu.distance() - before) < 1e-6f);
                    in.path_safe = true;
                }
                if (scenario == 3) {
                    for (int n = 0; n < 8; ++n) {
                        out = rig.motionTick(out, true, false);
                        if (out.reason == "startup_corner_forward_imu_gap") break;
                    }
                    assert(out.reason == "startup_corner_forward_imu_gap");
                    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
                    break;
                }
                if (scenario == 4) {
                    in.sent_velocity_valid = false;
                    out = rig.tick();
                    assert(out.reason == "startup_corner_forward_feedback_missing");
                    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
                    break;
                }
            }
            if (out.motion.wz_rps != 0) {
                assert(moved && rig.plant_position >= .09f && rig.plant_position < .12f);
                assert(out.motion.vx_mps == 0);
                assert(out.motion.wz_rps == (zone == 2 || zone == 3 ? -1.f : 1.f));
                turned = true;
                break;
            }
        }
        assert(moved && intervened);
        assert(turned == (scenario != 3 && scenario != 4));
    }
    std::cout << "PASS UI120 corners -> measured forward 0.1m -> first half turn, including pause/clearance/IMU feedback\n";
}

void startupForwardZoneBias() {
    // Exercise actual Startup output in all four zones: initial advance,
    // UI90 guide, UI120 single guide and UI120 pile guide.
    for (int zone : {1, 2, 3, 4}) for (int stage = 0; stage < 4; ++stage) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = zone;
        t.pixel_grasp_enabled = true;
        Rig rig(t);
        rig.tick();
        auto& in = rig.input;
        in.gripper_done = in.sent_velocity_valid = true;
        in.gripper_feedback_open = 0;
        in.directional_clearance_valid = in.arc_safe = in.turn_safe = true;
        if (stage) {
            rig.task.startup_action_ = PushTask::StartupAction::Approach;
            rig.task.startup_pitch_ = in.camera_pitch_cdeg =
                stage == 1 ? t.far_pitch_cdeg : t.track_pitch_cdeg;
            in.target_valid = in.target_center_px_valid = true;
            in.target_id = stage == 3 ? kStartupPileTargetId : 7;
            in.label = "ordinary_supply";
            in.target_center_px = {640, 400};
            in.pixel_grasp_error_px = 0;
            in.pixel_grasp_view = in.pixel_grasp_target_valid = stage >= 2;
            in.startup_visible_classes = 1;
        }
        const auto sample = [&] {
            in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
            return rig.tick();
        };
        PushOutput out;
        for (int i = 0; i < 30; ++i) {
            out = sample();
            if (out.motion.vx_mps > 0) break;
        }
        const float sign = zone == 2 || zone == 3 ? -1.f : 1.f;
        assert(out.motion.vx_mps > 0);
        assert(std::abs(out.motion.wz_rps - sign * (stage == 0 ? .08f : .05f)) < 1e-6f);
        // Zone preference yields to the existing arc and forward clearances.
        in.arc_safe = false;
        out = sample();
        assert(out.motion.vx_mps > 0 && out.motion.wz_rps == 0);
        in.path_safe = false;
        out = sample();
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    }
    std::cout << "PASS startup advances bias right in zones 2/3 and left in 1/4, respecting clearance\n";
}

void startupVisualManeuver(bool stepping = false) {
    for (int zone : {1, 2, 3, 4}) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.startup_advance_speed = .1f;
        t.start_zone = zone;
        t.pixel_grasp_enabled = true;
        t.far_pitch_cdeg = -3000;
        t.track_pitch_cdeg = 0;
        t.near_pitch_cdeg = 2000;
        t.max_wz = .55f;
        Rig rig(t);
        rig.task.setPhaseStepping(stepping);
        const float sign = zone == 2 || zone == 3 ? -1.f : 1.f;
        auto out = rig.tick();
        // No forward motion until the closed command has been acknowledged.
        out = rig.tick();
        assert(out.reason == "startup_gripper_close_wait");
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        rig.input.gripper_done = true;
        rig.input.gripper_feedback_open = 0;
        uint64_t forward_us = 0;
        for (int i = 0; i < 90; ++i) {
            out = rig.tick();
            assert(rig.task.frame_raised_ == 0);
            if (out.motion.vx_mps > 0) {
                assert(std::abs(out.motion.vx_mps - .1f) < 1e-6f &&
                       std::abs(out.motion.wz_rps - sign * .08f) < 1e-6f);
                forward_us += kTick;
            }
            if (rig.task.startup_action_ == PushTask::StartupAction::Observe)
                break;
        }
        assert(forward_us == 2000000 && out.startup_commanded_us == 2000000);
        rig.input.startup_visible_classes = 1;
        rig.input.startup_visible_count = 1;
        rig.input.startup_scene_us = rig.input.now_us;
        out = rig.tick();
        assert(out.motion.wz_rps == sign * .3f);
        rig.input.startup_visible_classes = zone % 2 ? 1 : 15; // same-class pair or four classes
        rig.input.startup_visible_count = zone % 2 ? 2 : 4;
        rig.input.startup_scene_us = rig.input.now_us - t.frame_timeout_us;
        out = rig.tick();
        assert(out.motion.wz_rps == sign * .3f);
        rig.input.startup_scene_us = rig.input.now_us + kTick;
        out = rig.tick();
        assert(out.reason == "startup_two_objects_seen" && !out.immediate_motion_stop);
        assert(out.motion.camera_pitch_cdeg == t.far_pitch_cdeg); // stay UI90 to approach
        rig.input.target_valid = rig.input.target_center_px_valid = true;
        rig.input.target_id = 7;
        rig.input.label = "dangerous_object"; // ordinary not required for closed approach
        rig.input.target_center_px = {640, 400};
        rig.input.pixel_grasp_error_px = 0;
        rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
        out = rig.tick();
        assert(out.reason == "startup_approach_far" && out.motion.vx_mps > 0);
        assert(rig.task.frame_raised_ == 0);
        const auto handoff = [&](int16_t from, int16_t to) {
            rig.input.camera_pitch_cdeg = from;
            rig.input.pixel_grasp_view = from == t.track_pitch_cdeg;
            rig.input.pixel_grasp_target_valid = rig.input.pixel_grasp_view;
            rig.input.pixel_grasp_near_view = false;
            rig.input.pixel_view_handoff_ready = from == t.far_pitch_cdeg;
            rig.input.startup_lower_corners_occupied = from == t.track_pitch_cdeg;
            rig.input.startup_visible_classes = 1;
            rig.input.target_center_px.y = from == t.far_pitch_cdeg ? 560 : 400;
            rig.input.target_valid = rig.input.target_center_px_valid = true;
            const bool half_turn = from == t.track_pitch_cdeg;
            rig.input.heading_valid = half_turn; // UI90 -> UI120 needs no IMU turn.
            bool advanced = false, turned = false, reversed = false, second_turn = false;
            float angle = 0, reverse_start = 0;
            float forward_origin = rig.plant_position;
            for (int i = 0; i < 400; ++i) {
                // Count the maneuver's in-place turns, excluding the preceding guide arc.
                if (out.motion.vx_mps == 0)
                    angle += sign * out.motion.wz_rps * (kTick * 1e-6f);
                const bool turning =
                    rig.task.startup_action_ == PushTask::StartupAction::TurnBeforePitch ||
                    rig.task.startup_action_ == PushTask::StartupAction::StopBeforePitch;
                // The guide is absent throughout forward travel, the two turns and reverse.
                rig.input.target_valid = rig.input.target_center_px_valid = !turning;
                rig.input.pixel_grasp_sample_us = turning ? 0 : rig.input.now_us + kTick;
                rig.input.startup_scene_us = rig.input.now_us + kTick;
                out = rig.motionTick(out);
                if (out.motion.vx_mps > 0) {
                    assert(half_turn && !turned && out.motion.wz_rps == 0);
                    assert(out.motion.vx_mps <= t.approach_speed);
                    // Exclude travel while braking the preceding guide approach.
                    if (!advanced) forward_origin = rig.plant_position;
                    advanced = true;
                }
                if (out.motion.wz_rps != 0 && !turned) {
                    assert(advanced && rig.plant_position - forward_origin >= .09f);
                    assert(rig.plant_position - forward_origin < .12f);
                }
                if (out.motion.vx_mps < 0) {
                    assert(half_turn && angle >= float(CV_PI));
                    assert(angle < float(CV_PI) + .1f && out.motion.wz_rps == 0);
                    if (!reversed) reverse_start = rig.plant_position;
                    reversed = true;
                }
                if (reversed && out.motion.wz_rps != 0) {
                    assert(reverse_start - rig.plant_position >= .09f);
                    second_turn = true;
                }
                assert(rig.task.frame_raised_ == 0);
                if (out.motion.wz_rps != 0) {
                    assert(out.motion.wz_rps == sign * 1.f);
                    assert(out.motion.camera_pitch_cdeg == from);
                    turned = true;
                }
                if (half_turn ? rig.task.startup_action_ == PushTask::StartupAction::SelectOrdinary
                              : out.motion.camera_pitch_cdeg == to)
                    break;
                assert(out.motion.camera_pitch_cdeg == from);
            }
            if (half_turn) {
                assert(advanced && turned && reversed && second_turn && angle >= 2.f * float(CV_PI));
                assert(angle < 2.f * float(CV_PI) + .15f);
                assert(reverse_start - rig.plant_position >= .09f);
                assert(reverse_start - rig.plant_position < .12f);
            } else {
                assert(!advanced && !turned && !reversed && angle == 0);
            }
            assert(out.motion.camera_pitch_cdeg == to);
            assert(out.motion.wz_rps == 0);
            if (!half_turn) {
                rig.input.target_valid = rig.input.target_center_px_valid = true;
                rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
                out = rig.tick(); // old camera feedback must hold all chassis motion
                assert(out.reason == "camera_pitch_wait");
                assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            } else {
                assert(rig.task.startupOrdinaryOnly());
                assert(rig.task.startupSightTargetId() == -1);
            }
            rig.input.startup_lower_corners_occupied = false;
            rig.input.heading_valid = true;
        };
        handoff(t.far_pitch_cdeg, t.track_pitch_cdeg);
        rig.input.camera_pitch_cdeg = t.track_pitch_cdeg;
        rig.input.pixel_grasp_view = rig.input.pixel_grasp_target_valid = true;
        rig.input.pixel_view_handoff_ready = false;
        rig.input.target_center_px.y = 400;
        bool advanced = false;
        for (int i = 0; i < 30; ++i) {
            rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
            out = rig.tick();
            assert(rig.task.frame_raised_ == 0);
            if (out.motion.vx_mps > 0) { advanced = true; break; }
        }
        assert(advanced && out.reason == "startup_approach_120");
        handoff(t.track_pitch_cdeg, t.track_pitch_cdeg);
        assert(out.reason == "startup_track_observe");
        rig.input.camera_pitch_cdeg = t.track_pitch_cdeg;
        rig.input.pixel_grasp_near_view = false;
        rig.input.pixel_grasp_view = true;
        rig.input.pixel_view_handoff_ready = false;
        rig.input.pixel_grasp_target_valid = true;
        rig.input.startup_visible_count = 2;
        rig.input.startup_visible_classes = 9;
        rig.input.startup_scene_us = rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
        out = rig.tick();
        assert(out.reason == "startup_find_ordinary" && out.motion.vx_mps == 0);
        assert(out.motion.camera_pitch_cdeg == t.track_pitch_cdeg);
        if (zone % 2) { // no selected ordinary yet; continue the safe search turn
            rig.input.startup_scene_us = rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
            out = rig.tick();
            assert(out.reason == "startup_find_ordinary");
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == sign * .3f);
            assert(rig.task.frame_raised_ == 0);
            rig.input.startup_visible_count = 1;
            rig.input.startup_scene_us = rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
            out = rig.tick();
            assert(out.reason == "startup_find_ordinary" && out.motion.wz_rps == sign * .3f);
        }
        // Ordinary nearest either before the first turn, or while turning, opens at rest.
        rig.input.target_valid = rig.input.target_center_px_valid = true;
        rig.input.label = "ordinary_supply";
        rig.input.target_id = 8;
        rig.input.target_center_px = {640, 280};
        rig.input.startup_visible_count = 2;
        for (int i = 0; i < 30; ++i) {
            rig.input.startup_scene_us = rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
            out = rig.tick();
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            if (out.reason == "startup_nearest_ordinary_open") break;
        }
        assert(out.reason == "startup_nearest_ordinary_open" && rig.task.frame_raised_ == 1);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        assert(rig.input.sent_velocity_valid && rig.input.sent_vx_mps == 0 &&
               rig.input.sent_wz_rps == 0); // Open only after the actual sent command is stopped.
        rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
        out = rig.tick();
        assert(out.reason == "gripper_feedback_wait" && out.motion.vx_mps == 0);
        rig.input.gripper_feedback_open = 1;
        advanced = false;
        for (int i = 0; i < 30; ++i) {
            rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
            out = rig.tick();
            if (out.motion.vx_mps > 0) { advanced = true; break; }
        }
        assert(advanced && out.reason == "startup_open_approach_120");
        assert(out.motion.camera_pitch_cdeg == t.track_pitch_cdeg);
        // Switch the open approach directly to UI140 at the original handoff.
        rig.input.target_box = {600, 520, 80, 80};
        rig.input.pixel_view_handoff_ready = true;
        rig.input.target_center_px.y = 560;
        for (int i = 0; i < 30; ++i) {
            rig.input.startup_scene_us = rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
            out = rig.tick();
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            assert(rig.task.frame_raised_ == 1);
            if (out.reason == "startup_switch_to_140") break;
        }
        assert(out.reason == "startup_switch_to_140");
        assert(out.motion.camera_pitch_cdeg == t.near_pitch_cdeg);
        assert(rig.task.startup_action_ == PushTask::StartupAction::OpenApproach);
        rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
        out = rig.tick();
        assert(out.reason == "camera_pitch_wait" && out.motion.vx_mps == 0);
        rig.input.camera_pitch_cdeg = t.near_pitch_cdeg;
        rig.input.pixel_grasp_near_view = true;
        // Near-view acquisition stays in Startup until the Y337 grasp line.
        rig.input.pixel_grasp_ready = false;
        rig.input.capture_scene_valid = true;
        rig.input.capture_scene_us = rig.input.now_us + kTick;
        rig.input.capture_class_regions_px["ordinary_supply"] = {342, 226, 689, 263};
        rig.input.capture_image_objects = {{8, "ordinary_supply", {600,240,80,80},true}};
        for (int frame = 0; frame < 3; ++frame) {
            rig.input.capture_scene_us = rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
            out = rig.tick();
            if (frame < 2) assert(out.phase == TaskPhase::Startup && !out.phase_waiting);
        }
        assert(out.immediate_motion_stop && !out.phase_waiting);
        assert(out.phase == TaskPhase::Startup && rig.task.startup_first_capture_);
        assert(out.target_id == 8 && rig.task.frame_raised_ == 1);
        rig.input.target_grasp_range_valid = true;
        rig.input.sent_velocity_valid = true;
        rig.input.capture_scene_valid = rig.input.capture_path_valid = true;
        rig.input.layout_count_region_px = {342, 226, 689, 263};
        rig.input.capture_path_px = {342, 0, 689, 489};
        rig.input.capture_area_px = {342,226,689,263};
        rig.input.target_grasp_range_lateral_measured = true;
        rig.input.target_grasp_range_inside = rig.input.target_grasp_range_lateral = true;
        rig.input.target_center_px = {647.5f, 337};
        rig.input.pixel_grasp_error_px = 0;
        rig.input.pixel_grasp_ready = true;
        rig.input.corridor.ordinary = 1;
        rig.input.corridor_complete = rig.input.corridor_occlusion_free = true;
        rig.input.capture_image_objects = {{8, "ordinary_supply", {608,297,80,80},true}};
        for (int i = 0; i < 30; ++i) {
            rig.input.capture_scene_us = rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
            out = rig.tick();
            assert(out.motion.camera_pitch_cdeg == t.near_pitch_cdeg);
            if (out.phase_waiting || out.phase == TaskPhase::Capture) break;
        }
        if (stepping) {
            assert(out.phase_waiting && out.next_phase == TaskPhase::Capture);
            out = rig.tick();
        }
        assert(out.state == PushState::LOWER_FRAME && out.phase == TaskPhase::Capture);
        rig.input.reset = true;
        out = rig.tick();
        assert(rig.task.startup_action_ == PushTask::StartupAction::Advance);
        assert(!rig.task.startup_first_capture_);
        assert(out.startup_commanded_us == 0);
    }
    TaskTuning invalid;
    invalid.startup_advance_us = 3000000;
    Rig rig(invalid);
    rig.tick();
    const auto out = rig.tick();
    assert(out.reason == "startup_zone_missing" && out.motion.vx_mps == 0);
}

void startupAnyOrdinaryInsideFrame() {
    for (int mode = 0; mode < 3; ++mode) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        Rig rig(t);
        if (mode == 1) rig.task.setPhaseStepping(true);
        if (mode == 2) rig.task.setSinglePhase("startup");
        rig.tick();
        rig.task.startup_action_ = PushTask::StartupAction::Approach;
        rig.task.startup_pitch_ = t.near_pitch_cdeg;
        rig.task.trip_context_.target_id_ = 7;
        rig.task.trip_context_.label_ = "dangerous_object";
        rig.input.camera_pitch_cdeg = t.near_pitch_cdeg;
        rig.input.pixel_grasp_near_view = true;
        rig.input.capture_scene_valid = true;
        rig.input.capture_class_regions_px["ordinary_supply"] = {342, 226, 689, 263};
        // The old locked target has disappeared; an unaligned ordinary is at the front edge.
        rig.input.target_valid = false;
        rig.input.pixel_grasp_ready = rig.input.pixel_grasp_aligned = false;
        rig.input.capture_image_objects = {{8, "ordinary_supply", {302,186,80,80},true}};
        rig.input.capture_scene_us = rig.input.now_us - t.frame_timeout_us;
        auto out = rig.tick();
        assert(out.phase == TaskPhase::Startup && !out.phase_waiting && !out.single_phase_done);
        rig.input.capture_scene_us = rig.input.now_us + kTick;
        rig.input.pixel_grasp_near_view = false;
        out = rig.tick();
        assert(out.phase == TaskPhase::Startup && !out.phase_waiting && !out.single_phase_done);
        rig.input.pixel_grasp_near_view = true;
        rig.input.capture_scene_us = rig.input.now_us + kTick;
        rig.input.camera_pitch_stable = false;
        out = rig.tick();
        assert(out.phase == TaskPhase::Startup && !out.phase_waiting && !out.single_phase_done);
        rig.input.camera_pitch_stable = true;
        for (const auto box : {cv::Rect(301,186,80,80), cv::Rect(992,186,80,80),
                               cv::Rect(302,185,80,80), cv::Rect(302,450,80,80)}) {
            rig.input.capture_image_objects[0].box = box;
            rig.input.capture_scene_us = rig.input.now_us + kTick;
            out = rig.tick();
            assert(out.phase == TaskPhase::Startup && !out.phase_waiting && !out.single_phase_done);
        }
        rig.input.capture_image_objects[0].box = {302,186,80,80};
        rig.input.capture_image_objects[0].label = "core_supply";
        rig.input.capture_scene_us = rig.input.now_us + kTick;
        out = rig.tick();
        assert(out.phase == TaskPhase::Startup && !out.phase_waiting && !out.single_phase_done);
        rig.input.capture_image_objects[0].label = "ordinary_supply";
        rig.input.capture_scene_us = rig.input.now_us + kTick;
        out = rig.tick();
        assert(out.reason == "startup_near_target_selected" && out.immediate_motion_stop);
        assert(!out.phase_waiting && !out.single_phase_done);
        assert(rig.task.trip_context_.target_id_ == 8 && rig.task.frame_raised_ == 1);
        rig.input.target_valid = rig.input.target_center_px_valid = true;
        rig.input.pixel_grasp_view = rig.input.pixel_grasp_target_valid = true;
        rig.input.target_id = 8;
        rig.input.label = "ordinary_supply";
        rig.input.target_center_px = {647.5f, 337};
        rig.input.pixel_grasp_error_px = 0;
        rig.input.pixel_grasp_ready = true;
        rig.input.target_grasp_range_valid = rig.input.target_grasp_range_inside = true;
        rig.input.target_grasp_range_lateral = rig.input.target_grasp_range_lateral_measured = true;
        rig.input.sent_velocity_valid = rig.input.gripper_done = true;
        rig.input.gripper_feedback_open = 1;
        rig.input.capture_path_valid = true;
        rig.input.capture_area_px = rig.input.layout_count_region_px = {342,226,689,263};
        rig.input.capture_path_px = {342,205,689,284};
        rig.input.capture_image_objects = {{8,"ordinary_supply",{608,297,80,80},true}};
        for (int i = 0; i < 30 && !out.phase_waiting && out.phase == TaskPhase::Startup; ++i) {
            rig.input.capture_scene_us = rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
            out = rig.tick();
        }
        if (mode == 1) {
            assert(out.phase_waiting && out.next_phase == TaskPhase::Capture);
            out = rig.tick();
        }
        if (mode == 2) assert(out.single_phase_done && out.phase == TaskPhase::Startup);
        else assert(out.phase == TaskPhase::Capture && out.state == PushState::LOWER_FRAME);

    }
}


// Replay the UI90 handoff failure: Y550, detection gap, Y519, then another ID.
// Once braking starts, target recovery must never request another approach/turn.
void startupFarHandoffSurvivesJitterAndReplacement() {
    for (int interruption = 0; interruption < 6; ++interruption) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        Rig rig(t);
        rig.tick();
        rig.task.startup_action_ = PushTask::StartupAction::Approach;
        rig.task.startup_pitch_ = t.far_pitch_cdeg;
        rig.task.trip_context_.target_id_ = 25;
        rig.task.trip_context_.label_ = "ordinary_supply";
        auto& in = rig.input;
        in.gripper_done = in.sent_velocity_valid = true;
        in.gripper_feedback_open = 0;
        in.target_valid = in.target_center_px_valid = true;
        in.target_id = 25;
        in.label = "ordinary_supply";
        in.target_center_px = {551.5f, 550};
        in.pixel_grasp_error_px = 88.5f;
        in.pixel_view_handoff_ready = true;
        in.sent_vx_mps = .047f;
        in.pixel_grasp_sample_us = in.now_us + kTick;
        auto out = rig.tick();
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        assert(out.motion.camera_pitch_cdeg == t.far_pitch_cdeg);
        for (int i = 0; i < 2; ++i) {
            in.target_valid = in.target_center_px_valid = false;
            in.pixel_grasp_sample_us = 0;
            out = rig.tick();
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            assert(out.motion.camera_pitch_cdeg == t.far_pitch_cdeg);
        }
        in.sent_vx_mps = 0;
        in.target_valid = in.target_center_px_valid = true;
        in.target_center_px = {516, 519.5f};
        in.pixel_grasp_error_px = 124;
        in.pixel_view_handoff_ready = false;
        in.pixel_grasp_sample_us = in.now_us + kTick;
        out = rig.tick();
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        // A fresh replacement starts stationary confirmation immediately, without
        // waiting for the old ID's 1.5-second disappearance timer.
        in.target_id = 42;
        for (int i = 0; i < 4; ++i) {
            in.pixel_grasp_sample_us = in.now_us + kTick;
            out = rig.tick();
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            assert(out.motion.camera_pitch_cdeg == t.far_pitch_cdeg);
        }
        const uint64_t frozen_sample = in.pixel_grasp_sample_us;
        for (int i = 0; i < 40; ++i) {
            in.target_valid = in.target_center_px_valid = interruption != 0;
            in.camera_pitch_stable = interruption != 3;
            in.sent_vx_mps = interruption == 4 ? .04f : 0;
            in.pixel_grasp_sample_us = interruption == 0 ? 0
                : interruption == 1 ? in.now_us - t.frame_timeout_us
                : interruption == 2 ? in.now_us + 2 * kTick
                : interruption == 5 ? frozen_sample : in.now_us + kTick;
            out = rig.tick();
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            assert(out.motion.camera_pitch_cdeg == t.far_pitch_cdeg);
            assert(out.reason != "startup_target_reselect");
            assert(out.reason != "startup_alignment_reselect");
        }
        in.target_valid = in.target_center_px_valid = in.camera_pitch_stable = true;
        in.sent_vx_mps = 0;
        bool switched = false;
        const uint64_t recovered_us = in.now_us;
        for (int i = 0; i < 20; ++i) {
            in.pixel_grasp_sample_us = in.now_us + kTick;
            out = rig.tick();
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            if (out.reason == "startup_switch_to_120") {
                switched = true;
                break;
            }
        }
        assert(switched && in.now_us - recovered_us < kStartupTargetLossHoldUs);
        assert(out.motion.camera_pitch_cdeg == t.track_pitch_cdeg);
        assert(rig.task.frame_raised_ == 0);
    }
    std::cout << "PASS UI90 handoff survives jitter/replacement and requires fresh stopped images\n";
}

void startupLockLossAndRecovery() {
    for (bool near : {false, true}) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        t.track_pitch_cdeg = 0;
        t.near_pitch_cdeg = 2000;
        Rig rig(t);
        rig.tick();
        rig.task.startup_action_ = near ? PushTask::StartupAction::OpenApproach
                                       : PushTask::StartupAction::Approach;
        rig.task.startup_pitch_ = near ? t.near_pitch_cdeg : t.track_pitch_cdeg;
        rig.task.trip_context_.target_id_ = 7;
        rig.task.trip_context_.label_ = "ordinary_supply";
        rig.task.frame_raised_ = near ? 1 : 0;
        rig.input.camera_pitch_cdeg = rig.task.startup_pitch_;
        rig.input.pixel_grasp_near_view = near;
        rig.input.gripper_done = true;
        rig.input.gripper_feedback_open = near ? 1 : 0;
        assert(rig.task.startupSightTargetId() == 7);
        // A short gap requests a ramped stop without changing frame state or identity.
        for (int i = 0; i < 4; ++i) {
            auto out = rig.tick();
            assert(out.reason == "startup_locked_target_wait");
            assert(!out.immediate_motion_stop && out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            assert(rig.task.startupSightTargetId() == 7);
            assert(rig.task.frame_raised_ == (near ? 1 : 0));
        }
        // Recovery resets the lost timer and resumes the same identity.
        rig.input.target_valid = rig.input.target_center_px_valid = true;
        rig.input.target_id = 7;
        rig.input.label = "ordinary_supply";
        rig.input.pixel_grasp_view = rig.input.pixel_grasp_target_valid = true;
        rig.input.target_center_px = {640, 280};
        rig.input.pixel_grasp_error_px = 0;
        rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
        rig.tick();
        assert(rig.task.startup_target_missing_since_us_ == 0);
        assert(rig.task.startupSightTargetId() == 7);
        // A confirmed same-object re-ID keeps the alignment episode and timeout.
        const auto alignment_since = rig.task.startup_alignment_since_us_;
        rig.input.target_id = 70;
        rig.input.startup_rebound_from = 7;
        rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
        rig.tick();
        assert(rig.task.startupSightTargetId() == 70);
        assert(rig.task.startup_alignment_since_us_ == alignment_since);
        rig.input.startup_rebound_from = -1;
        rig.task.trip_context_.target_id_ = 7;
        rig.task.startup_alignment_target_id_ = 7;
        // Even a fresh different identity cannot silently replace the lock.
        rig.input.target_id = 8;
        PushOutput out;
        for (int i = 0; i <= int(kStartupTargetLossHoldUs / kTick); ++i) {
            rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
            out = rig.tick();
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        }
        assert(out.reason == "startup_target_reselect" && !out.immediate_motion_stop);
        assert(rig.task.startupSightTargetId() == -1);
        assert(rig.task.trip_context_.capture_target_id_ == -1);
        assert(rig.task.frame_raised_ == (near ? 1 : 0));
        assert(rig.task.pixel_alignment_.stage == PushTask::PixelAlignment::Stage::Correct);
        // A fresh replacement starts a new alignment, at both UI120 and UI140.
        rig.input.startup_visible_classes = 1;
        rig.input.startup_visible_count = 1;
        rig.input.startup_scene_us = rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
        rig.input.gripper_feedback_open = near ? 1 : 0;
        rig.tick();
        assert(rig.task.startupSightTargetId() == 8);
    }
}

// Logged UI120 failure: the same centred ID disappears briefly every few frames.
// ID changes of a confirmed physical object must retain both moving and rest progress.
void startupRebindKeepsAlignment() {
    for (bool near : {false, true}) {
        for (bool advancing : {false, true}) {
            TaskTuning t;
            t.startup_advance_us = 2000000;
            t.start_zone = 3;
            t.pixel_grasp_enabled = true;
            Rig rig(t);
            rig.tick();
            rig.task.startup_action_ = near ? PushTask::StartupAction::OpenApproach
                                           : PushTask::StartupAction::Approach;
            rig.task.startup_pitch_ = near ? t.near_pitch_cdeg : t.track_pitch_cdeg;
            rig.task.trip_context_.target_id_ = 7;
            rig.task.trip_context_.label_ = "ordinary_supply";
            rig.task.frame_raised_ = near ? 1 : 0;
            auto& in = rig.input;
            in.camera_pitch_cdeg = rig.task.startup_pitch_;
            in.pixel_grasp_near_view = near;
            in.gripper_done = true;
            in.gripper_feedback_open = near ? 1 : 0;
            in.target_valid = in.target_center_px_valid = true;
            in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
            in.target_id = 70;
            in.startup_rebound_from = 7;
            in.label = "ordinary_supply";
            in.target_center_px = {640, 280};
            in.pixel_grasp_error_px = 0;
            in.pixel_grasp_sample_us = in.now_us + kTick;
            auto& a = rig.task.pixel_alignment_;
            a.target_id = 7;
            a.label = in.label;
            a.pitch = in.camera_pitch_cdeg;
            a.stage = advancing ? PushTask::PixelAlignment::Stage::Advance
                                : PushTask::PixelAlignment::Stage::Settle;
            a.last_sample_us = in.now_us;
            a.stable_since_us = in.now_us - t.pixel_align_stable_us;
            a.stable_frames = t.pixel_align_stable_frames;
            a.last_center_y_px = 280;
            const auto out = rig.tick();
            assert(rig.task.startupSightTargetId() == 70);
            assert(a.target_id == 70 && a.stage == PushTask::PixelAlignment::Stage::Advance);
            assert(out.motion.vx_mps > 0 && !out.immediate_motion_stop);
        }
    }
}

// Exercise the actual sender ramp at both UI120 and UI140, plus hard-stop overrides.
void startupGapRampAndSafety() {
    for (bool near : {false, true}) {
        for (int blocked = 0; blocked < 7; ++blocked) {
            TaskTuning t;
            t.startup_advance_us = 2000000;
            t.start_zone = 3;
            t.pixel_grasp_enabled = true;
            Rig rig(t);
            rig.tick();
            rig.task.startup_action_ = near ? PushTask::StartupAction::OpenApproach
                                           : PushTask::StartupAction::Approach;
            rig.task.startup_pitch_ = near ? t.near_pitch_cdeg : t.track_pitch_cdeg;
            rig.task.trip_context_.target_id_ = 7;
            rig.task.trip_context_.label_ = "ordinary_supply";
            rig.input.camera_pitch_cdeg = rig.task.startup_pitch_;
            rig.input.pixel_grasp_near_view = near;
            rig.input.directional_clearance_valid = true;
            rig.input.turn_safe = rig.input.arc_safe = true;
            if (blocked == 1) rig.input.path_safe = false;
            if (blocked == 2) rig.input.retreat_safe = false;
            if (blocked == 3) rig.input.turn_safe = false;
            if (blocked == 4) rig.input.arc_safe = false;
            if (blocked == 5) rig.input.opponent_zone_clear = false;
            if (blocked == 6) rig.input.safety_ok = false;
            MotionLinkConfig c;
            MotionVelocityRamp ramp(c);
            MotionCommand sent;
            sent.vx_mps = .07f;
            sent.wz_rps = .2f;
            ramp.accept(sent, rig.input.now_us);
            rig.input.sent_velocity_valid = true;
            rig.input.sent_vx_mps = sent.vx_mps;
            rig.input.sent_wz_rps = sent.wz_rps;
            const auto out = rig.tick();
            // Runtime hard-stop gating remains independent of the task stop flag.
            const bool hard_stop = !rig.input.run || !rig.input.safety_ok ||
                                   !rig.input.opponent_zone_clear;
            const auto next = ramp.next(out.motion, rig.input.now_us,
                                        out.immediate_motion_stop || hard_stop);
            if (blocked) {
                assert(out.immediate_motion_stop || hard_stop);
                assert(next.vx_mps == 0 && next.wz_rps == 0);
            } else {
                assert(!out.immediate_motion_stop);
                assert(next.vx_mps > 0 && next.vx_mps < sent.vx_mps);
                assert(next.wz_rps > 0 && next.wz_rps < sent.wz_rps);
                ramp.accept(next, rig.input.now_us);
                sent = next;
                rig.task.frame_raised_ = near ? 1 : 0;
                rig.input.gripper_done = true;
                rig.input.gripper_feedback_open = near ? 1 : 0;
                rig.input.sent_velocity_valid = true;
                rig.input.target_valid = rig.input.target_center_px_valid = true;
                rig.input.pixel_grasp_view = rig.input.pixel_grasp_target_valid = true;
                rig.input.target_id = 7;
                rig.input.label = "ordinary_supply";
                rig.input.target_center_px = {640,280};
                rig.input.pixel_grasp_error_px = 0;
                bool reached_rest = false;
                for (int i = 0; i < 30; ++i) {
                    rig.input.sent_vx_mps = sent.vx_mps;
                    rig.input.sent_wz_rps = sent.wz_rps;
                    rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
                    const auto recovered = rig.tick();
                    assert(!recovered.immediate_motion_stop);
                    const auto decelerated = ramp.next(recovered.motion, rig.input.now_us, false);
                    assert(decelerated.vx_mps <= sent.vx_mps && decelerated.wz_rps <= sent.wz_rps);
                    assert(sent.vx_mps - decelerated.vx_mps <= c.linear_decel_mps2 * .04f + 1e-6f);
                    assert(sent.wz_rps - decelerated.wz_rps <= c.angular_decel_rps2 * .04f + 1e-6f);
                    ramp.accept(decelerated, rig.input.now_us);
                    sent = decelerated;
                    if (std::abs(sent.vx_mps) <= t.pixel_align_stopped_vx &&
                        std::abs(sent.wz_rps) <= t.pixel_align_stopped_wz) {
                        reached_rest = true;
                        break;
                    }
                }
                assert(reached_rest);
            }
        }
    }
}

void startupIntermittentDetectionProgress() {
    TaskTuning t;
    t.startup_advance_us = 2000000;
    t.start_zone = 3;
    t.pixel_grasp_enabled = true;
    t.track_pitch_cdeg = 0;
    Rig rig(t);
    rig.tick();
    rig.task.startup_action_ = PushTask::StartupAction::Approach;
    rig.task.startup_pitch_ = t.track_pitch_cdeg;
    rig.input.camera_pitch_cdeg = t.track_pitch_cdeg;
    rig.input.pixel_grasp_view = true;
    rig.input.gripper_done = true;
    rig.input.gripper_feedback_open = 0;
    rig.input.sent_velocity_valid = true;
    int recovered = 0;
    bool advanced = false, retained = false;
    for (int i = 0; i < 60; ++i) {
        const bool gap = i % 2 == 1;
        rig.input.target_valid = rig.input.target_center_px_valid = !gap;
        rig.input.pixel_grasp_target_valid = !gap;
        rig.input.target_id = gap ? -1 : 66;
        rig.input.label = gap ? "" : "injured_person";
        rig.input.target_center_px = {636.5f,195.f};
        rig.input.pixel_grasp_error_px = 3.5f;
        rig.input.pixel_grasp_sample_us = gap ? 0 : rig.input.now_us + kTick;
        const auto old_sample = rig.task.pixel_alignment_.last_sample_us;
        const auto out = rig.tick();
        if (gap) {
            assert(!out.immediate_motion_stop && out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            retained |= old_sample && rig.task.pixel_alignment_.last_sample_us == old_sample;
        } else {
            ++recovered;
            if (recovered < t.pixel_align_stable_frames)
                assert(out.motion.vx_mps == 0);
        }
        advanced |= out.motion.vx_mps > 0;
        if (advanced) break;
    }
    assert(retained && advanced && rig.task.startupSightTargetId() == 66);
}

void startupConfirmationTimeoutAndSearch() {
    TaskTuning t;
    t.startup_advance_us = 2000000;
    t.start_zone = 3;
    t.pixel_grasp_enabled = true;
    t.track_pitch_cdeg = 0;
    t.near_pitch_cdeg = 2000;
    for (bool near : {false,true}) {
        Rig rig(t);
        rig.tick();
        rig.task.startup_action_ = near ? PushTask::StartupAction::OpenApproach
                                       : PushTask::StartupAction::Approach;
        rig.task.startup_pitch_ = near ? t.near_pitch_cdeg : t.track_pitch_cdeg;
        rig.input.camera_pitch_cdeg = rig.task.startup_pitch_;
        rig.input.pixel_grasp_view = true;
        rig.input.pixel_grasp_near_view = near;
        rig.input.gripper_done = true;
        rig.input.gripper_feedback_open = near ? 1 : 0;
        rig.task.frame_raised_ = near ? 1 : 0;
        PushOutput out;
        for (int i = 0; i < 80; ++i) {
            // Every other detection is missing, so the continuous loss timer never expires.
            const bool seen = i % 2 == 0;
            rig.input.target_valid = rig.input.target_center_px_valid = seen;
            rig.input.pixel_grasp_target_valid = seen;
            rig.input.target_id = seen ? 7 : -1;
            rig.input.label = seen ? "ordinary_supply" : "";
            // Visible samples keep moving: there is no stationary confirmation.
            rig.input.pixel_grasp_error_px = i % 4 == 0 ? -60.f : 60.f;
            rig.input.target_center_px = {640 - rig.input.pixel_grasp_error_px,280};
            rig.input.pixel_grasp_sample_us = seen ? rig.input.now_us + kTick : 0;
            out = rig.tick();
            assert(out.motion.vx_mps == 0);
            if (out.reason == "startup_alignment_reselect") break;
        }
        assert(out.run_status == RunStatus::Running);
        assert(out.reason == "startup_alignment_reselect");
        assert(!out.immediate_motion_stop && rig.task.frame_raised_ == (near ? 1 : 0));
        assert(rig.task.startupSightTargetId() == -1);
        assert(rig.task.startupSightRejectedId(rig.input.now_us) == 7);
        assert(rig.task.startupSightRejectedId(rig.input.now_us + 1000000) == -1);
        rig.input.target_valid = rig.input.target_center_px_valid = false;
        rig.input.gripper_feedback_open = near ? 1 : 0;
        rig.input.startup_visible_count = 1;
        rig.input.startup_visible_classes = 4;
        rig.input.startup_scene_us = rig.input.now_us + kTick;
        rig.input.directional_clearance_valid = true;
        rig.input.turn_safe = false;
        out = rig.tick();
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        rig.input.turn_safe = true;
        out = rig.tick();
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps < 0);
        rig.input.reset = true;
        rig.tick();
        assert(rig.task.startupSightRejectedId(rig.input.now_us) == -1);
    }
}

void startupNearPredictiveBrakeAndPulse() {
    TaskTuning t;
    t.startup_advance_us = 2000000;
    t.start_zone = 3;
    t.pixel_grasp_enabled = true;
    t.near_pitch_cdeg = 2000;
    Rig rig(t);
    rig.tick();
    rig.task.startup_action_ = PushTask::StartupAction::OpenApproach;
    rig.task.startup_pitch_ = t.near_pitch_cdeg;
    rig.task.frame_raised_ = 1;
    rig.input.camera_pitch_cdeg = t.near_pitch_cdeg;
    rig.input.pixel_grasp_near_view = rig.input.pixel_grasp_view = true;
    rig.input.target_valid = rig.input.target_center_px_valid = true;
    rig.input.pixel_grasp_target_valid = true;
    rig.input.target_id = 114;
    rig.input.label = "ordinary_supply";
    rig.input.gripper_done = true;
    rig.input.gripper_feedback_open = 1;
    rig.input.sent_velocity_valid = true;
    const auto sample = [&](float error) {
        rig.input.pixel_grasp_error_px = error;
        rig.input.target_center_px = {640-error,280};
        rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
        return rig.tick();
    };
    auto out = sample(100);
    assert(out.motion.wz_rps > 0);
    // Still outside the 12 px tolerance, but measured image drift predicts crossing soon.
    rig.input.sent_wz_rps = out.motion.wz_rps;
    out = sample(70);
    assert(out.reason == "pixel_align_brake" && out.motion.wz_rps == 0);
    rig.input.sent_wz_rps = 0;
    bool fine_moved = false, pulse_stopped = false;
    int consecutive_turns = 0;
    for (int i = 0; i < 30; ++i) {
        out = sample(35);
        assert(out.motion.vx_mps == 0);
        if (out.motion.wz_rps != 0) {
            fine_moved = true;
            ++consecutive_turns;
            assert(std::abs(out.motion.wz_rps) <= .1f);
            assert(consecutive_turns <= 2); // 100 ms at the 50 ms test tick
        } else if (fine_moved) {
            pulse_stopped = true;
            break;
        }
        rig.input.sent_wz_rps = out.motion.wz_rps;
    }
    assert(fine_moved && pulse_stopped);
    rig.input.sent_wz_rps = 0;
    bool advanced = false;
    for (int i = 0; i < 25; ++i) {
        out = sample(0);
        advanced |= out.motion.vx_mps > 0;
        if (advanced) break;
    }
    assert(advanced && out.reason == "startup_open_approach_140");
    // A gap while advancing must require fresh rest confirmation on recovery.
    rig.input.target_valid = rig.input.target_center_px_valid = false;
    rig.input.pixel_grasp_sample_us = 0;
    out = rig.tick();
    assert(!out.immediate_motion_stop && out.motion.vx_mps == 0);
    rig.input.target_valid = rig.input.target_center_px_valid = true;
    out = sample(0);
    assert(out.motion.vx_mps == 0);
    // A large same-ID box jump restarts the stationary evidence rather than accepting it.
    rig.input.target_valid = false;
    rig.input.pixel_grasp_sample_us = 0;
    rig.tick();
    rig.input.target_valid = true;
    out = sample(100);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    assert(rig.task.pixel_alignment_.stage == PushTask::PixelAlignment::Stage::Brake);
}

// Reuse the existing split-controller plant model: 150 ms response lag and
// 2000 px/rad. This catches a controller that passes static samples but oscillates.
void startupNearDelayedChassisConverges() {
    for (float start_error : {-180.f, -100.f, -40.f, 40.f, 100.f, 180.f}) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = 3;
        t.pixel_grasp_enabled = true;
        t.near_pitch_cdeg = 2000;
        Rig rig(t);
        rig.tick();
        rig.task.startup_action_ = PushTask::StartupAction::OpenApproach;
        rig.task.startup_pitch_ = t.near_pitch_cdeg;
        rig.task.frame_raised_ = 1;
        rig.input.camera_pitch_cdeg = t.near_pitch_cdeg;
        rig.input.pixel_grasp_near_view = rig.input.pixel_grasp_view = true;
        rig.input.target_valid = rig.input.target_center_px_valid = true;
        rig.input.pixel_grasp_target_valid = true;
        rig.input.target_id = 114;
        rig.input.label = "ordinary_supply";
        rig.input.gripper_done = true;
        rig.input.gripper_feedback_open = 1;
        rig.input.sent_velocity_valid = true;
        float error = start_error;
        std::array<float,3> delay{};
        PushOutput out;
        bool advanced = false;
        for (int n = 0; n < 65; ++n) {
            error -= delay[0] * .05f * 2000.f;
            rig.input.pixel_grasp_error_px = error;
            rig.input.target_center_px = {640-error,280};
            rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
            out = rig.tick();
            assert(out.reason != "startup_alignment_reselect");
            if (out.motion.vx_mps > 0) {
                assert(std::abs(error) <= t.pixel_align_tolerance_px);
                advanced = true;
                break;
            }
            rig.input.sent_wz_rps = out.motion.wz_rps;
            delay[0] = delay[1]; delay[1] = delay[2]; delay[2] = out.motion.wz_rps;
        }
        assert(advanced);
        std::cout << "startup_delay initial_error=" << start_error << " final_error=" << error
                  << " elapsed_us=" << rig.input.now_us - kStart << '\n';
    }
}

// UI120 log 20261104-190214: corrections cross the centre before the chassis stops.
// At 0.2 rad/s the image moves about 800-1100 px/s. Use 4000 px/rad
// with the existing 150 ms delayed plant; no hardware is opened by this test.
void startupTrackDelayedChassisConverges() {
    for (int mode : {0, 1, 2}) {
        for (float start_error : {-582.f, -241.f, -40.f, 40.f, 241.f, 582.f}) {
            TaskTuning t;
            t.startup_advance_us = 2000000;
            t.start_zone = 3;
            t.pixel_grasp_enabled = true;
            Rig rig(t);
            rig.tick();
            rig.task.startup_action_ = mode == 0 ? PushTask::StartupAction::Approach
                : mode == 1 ? PushTask::StartupAction::SelectOrdinary
                            : PushTask::StartupAction::OpenApproach;
            rig.task.startup_pitch_ = t.track_pitch_cdeg;
            rig.task.frame_raised_ = mode == 2;
            auto& in = rig.input;
            in.camera_pitch_cdeg = t.track_pitch_cdeg;
            in.pixel_grasp_view = in.pixel_grasp_target_valid = true;
            in.target_valid = in.target_center_px_valid = true;
            in.target_id = mode == 0 ? kStartupPileTargetId : 101;
            in.label = "ordinary_supply";
            in.startup_visible_classes = 1;
            in.sent_velocity_valid = in.gripper_done = true;
            in.gripper_feedback_open = mode == 2;
            float error = start_error;
            std::array<float, 3> delay{};
            bool completed = false;
            int reversals = 0, last_direction = 0;
            for (int n = 0; n < 65; ++n) {
                error -= delay[0] * .05f * 4000.f;
                in.pixel_grasp_error_px = error;
                in.target_center_px = {640-error, 280};
                in.startup_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
                const auto out = rig.tick();

                const int direction = out.motion.wz_rps > 0 ? 1 : out.motion.wz_rps < 0 ? -1 : 0;
                if (direction && last_direction && direction != last_direction) ++reversals;
                if (direction) last_direction = direction;
                if (out.motion.vx_mps > 0 || out.reason == "startup_nearest_ordinary_open") {
                    assert(std::abs(error) <= (mode == 1 ? t.pixel_align_approach_tolerance_px
                                                        : t.pixel_align_approach_restart_px));
                    completed = true;
                    break;
                }
                in.sent_wz_rps = out.motion.wz_rps;
                delay[0] = delay[1]; delay[1] = delay[2]; delay[2] = out.motion.wz_rps;
            }
            std::cout << "startup_ui120 mode=" << mode << " initial_error=" << start_error
                      << " final_error=" << error << " elapsed_us=" << in.now_us-kStart
                      << " reversals=" << reversals << " completed=" << completed << std::endl;
            assert(completed && reversals <= 2);
        }
    }
}

// Replay the 20:28 UI120 failure: Y178 -> Y535 -> Y585, then a clipped/missing lock.
// Drive the real task entry, camera handshake and selector events without hardware output.
void searchUi120StopsBeforeBottom(bool speed_only) {
    for (int scenario = 0; scenario < (speed_only ? 1 : 9); ++scenario) {
        TaskTuning t;
        t.startup_advance_us = 0;
        t.require_multi_view = false;
        t.pixel_grasp_enabled = true;
        t.approach_speed = .07f;
        t.far_pitch_cdeg = -3000;
        t.intermediate_pitch_cdeg = t.track_pitch_cdeg = 0;
        t.near_pitch_cdeg = 2000;
        t.pixel_align_brake_motion_px = 30;
        Rig rig(t);
        rig.tick();
        rig.task.enter(PushState::APPROACH, rig.input.now_us, "search_ui120_log_replay");
        rig.task.search_phase_.approach_pitch_ = t.track_pitch_cdeg;
        rig.task.trip_context_.target_id_ = rig.task.trip_context_.capture_target_id_ = 5;
        rig.task.trip_context_.label_ = "ordinary_supply";
        rig.task.setInitialFrameOpen(true);
        auto& in = rig.input;
        in.camera_pitch_cdeg = t.track_pitch_cdeg;
        in.sent_velocity_valid = in.gripper_done = in.pixel_selection_managed = true;
        in.gripper_feedback_open = 1;
        in.pixel_grasp_view = in.pixel_locked = true;
        in.capture_scene_valid = true;
        const int height = scenario == 4 || scenario == 8 ? 1080 : 720;
        in.pixel_grasp_image_height_px = height;
        PushOutput out;
        const auto sample = [&](float y, bool visible = true, bool fresh = true, int id = 5) {
            in.target_valid = in.target_center_px_valid = in.pixel_grasp_target_valid = visible;
            in.target_id = visible ? id : -1;
            in.label = visible ? "ordinary_supply" : "";
            in.target_center_px = {640, y};
            in.target_box = {590, int(y) - 50, 100, 100};
            in.pixel_grasp_error_px = in.pixel_grasp_steering = 0;
            in.pixel_view_handoff_ready = y >= .75f * height;
            in.pixel_grasp_sample_us = visible ? in.now_us + kTick : 0;
            if (fresh)
                in.capture_scene_us = in.now_us + kTick;
            in.capture_image_objects.clear();
            if (visible)
                in.capture_image_objects.push_back({id, "ordinary_supply", in.target_box, true});
            out = rig.tick();
            return out;
        };
        for (int i = 0; i < 22; ++i)
            sample(178);
        assert(out.reason == "pixel_center_approach_120" && out.motion.vx_mps > 0);
        if (speed_only) {
            assert(out.motion.vx_mps <= .04f + 1e-6f);
            std::cout << "PASS search UI120 approach capped at 0.04 m/s\n";
            continue;
        }
        // A stale detection at the new line must not commit a handoff.
        in.now_us += kTick;
        in.target_center_px.y = .625f * height;
        in.pixel_grasp_sample_us = in.now_us - t.frame_timeout_us - 1;
        out = rig.task.update(in);
        assert(out.motion.camera_pitch_cdeg == t.track_pitch_cdeg);
        for (int i = 0; i < 22; ++i)
            sample(178);
        in.sent_vx_mps = .04f;
        const float line = .625f * height;
        sample(line - 1);
        assert(out.motion.vx_mps > 0); // centre, not box bottom, controls the crossing
        sample(line);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        assert(!out.immediate_motion_stop); // retain the ordinary sender deceleration ramp
        assert(out.motion.camera_pitch_cdeg == t.track_pitch_cdeg);
        const auto committed = in.now_us;
        bool switched = false;
        const auto repeated_scene = in.capture_scene_us;
        for (int i = 0; i < 38; ++i) {
            in.sent_vx_mps = scenario == 3 ? .04f : (i < 3 ? .02f : 0.f);
            // The selector can release the old lock during braking; this is not a new approach.
            in.pixel_released_id = scenario == 1 ? 5 : -1;
            const bool fresh = scenario != 2;
            if (!fresh)
                in.capture_scene_us = repeated_scene;
            sample(i % 2 ? line - 15 : line + 10, scenario != 1, fresh);
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            if (out.motion.camera_pitch_cdeg == t.near_pitch_cdeg) {
                assert(scenario != 2 && scenario != 3);
                assert(in.now_us - committed >= 300000);
                switched = true;
                break;
            }
            if (out.state == PushState::SAFE_STOP || out.reason == "pixel_search_handoff_research")
                break;
        }
        if (scenario == 2 || scenario == 3) {
            assert(!switched && out.state == PushState::APPROACH);
            assert(out.reason == "pixel_search_handoff_research" && out.target_id == -1);
            assert(!rig.task.search_phase_.view_handoff_.started_us);
            // An unconfirmed sender stop cannot authorize renewed movement.
            if (scenario == 3)
                for (int i = 0; i < 8; ++i) {
                    sample(178);
                    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
                }
            in.sent_vx_mps = 0;
            bool reacquired = false;
            for (int i = 0; i < 30 && !reacquired; ++i) {
                sample(178);
                assert(out.state != PushState::SAFE_STOP);
                reacquired = out.target_id == 5;
            }
            assert(reacquired);
            continue;
        }
        assert(switched && out.reason == "pixel_center_switch_to_140");
        in.pixel_released_id = -1;
        // Moving camera and an absent near-view target never permit chassis motion.
        in.camera_pitch_stable = false;
        sample(337, false);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        in.camera_pitch_cdeg = t.near_pitch_cdeg;
        in.camera_pitch_stable = true;
        in.pixel_grasp_near_view = true;
        if (scenario == 5 || scenario == 6 || scenario == 8) {
            const auto old_scene = in.capture_scene_us;
            for (int i = 0; i < 38 && out.reason != "pixel_search_handoff_research"; ++i) {
                if (scenario == 6)
                    in.capture_scene_us = old_scene;
                sample(337, scenario == 6, scenario != 6);
                assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            }
            assert(out.state == PushState::APPROACH && out.target_id == -1);
            assert(out.reason == "pixel_search_handoff_research");
            assert(!rig.task.search_phase_.view_handoff_.started_us);
            // Replay 21:08: returning at the same UI120 image position must
            // approach, rather than repeat the failed pitch switch with a new ID.
            in.camera_pitch_cdeg = t.track_pitch_cdeg;
            in.pixel_grasp_near_view = false;
            bool approaching = false;
            for (int i = 0; i < 36; ++i) {
                sample(line + (i % 2 ? 10 : 0), true, true, 23);
                assert(out.motion.camera_pitch_cdeg == t.track_pitch_cdeg);
                approaching |= out.reason == "pixel_center_approach_120" &&
                               out.motion.vx_mps > 0;
            }
            assert(approaching);
            // A stale progress frame is insufficient; a fresh image beyond the
            // existing motion tolerance may commit the next stopped handoff.
            in.target_center_px.y = line + t.pixel_align_brake_motion_px + 1;
            in.pixel_grasp_sample_us = in.now_us - t.frame_timeout_us - 1;
            in.now_us += kTick;
            out = rig.task.update(in);
            assert(out.motion.camera_pitch_cdeg == t.track_pitch_cdeg);
            bool retried = false;
            for (int i = 0; i < 24 && !retried; ++i) {
                sample(line + t.pixel_align_brake_motion_px + 1, true, true, 23);
                retried = out.reason == "pixel_center_switch_to_140";
            }
            assert(retried && out.target_id == -1);
        } else {
            // The next view supplies its own confirmed cargo, without an old-ID event.
            bool acknowledged = false;
            for (int i = 0; i < 16; ++i) {
                sample(337, true, true, scenario == 7 ? 15 : 5);
                assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
                if (out.reason == "search_view_target_selected") {
                    acknowledged = true;
                    in.pixel_reselected_from = -1; // no old-ID acknowledgment is needed
                }
                if (out.reason == "pixel_search_handoff_ready") break;
            }
            assert(out.reason == "pixel_search_handoff_ready");
            assert(rig.task.search_phase_.pixel_handoff_retry_height_ == 0);
            if (scenario == 7) assert(acknowledged && out.target_id == 15);
        }
    }
    if (!speed_only)
        std::cout
            << "PASS UI120 early handoff: jitter, lock release, repeated frames, moving sender, "
               "scaled image, near-target loss, stale near images, retry progress and independent view selection\n";
}

// Exercise task -> sender ramp, including a lost lock, replacement and UI120 ordinary-search
// handoff.
void startupTransitionsDecelerate() {
    TaskTuning t;
    t.startup_advance_us = 2000000;
    t.start_zone = 3;
    t.pixel_grasp_enabled = true;
    t.track_pitch_cdeg = 0;
    Rig rig(t);
    rig.tick();
    rig.task.startup_action_ = PushTask::StartupAction::Approach;
    rig.task.startup_pitch_ = t.track_pitch_cdeg;
    rig.task.trip_context_.target_id_ = 7;
    rig.task.trip_context_.label_ = "ordinary_supply";
    rig.input.camera_pitch_cdeg = t.track_pitch_cdeg;
    rig.input.gripper_done = true;
    rig.input.gripper_feedback_open = 0;
    rig.input.sent_velocity_valid = true;
    rig.input.directional_clearance_valid = true;
    rig.input.turn_safe = rig.input.arc_safe = true;
    MotionLinkConfig c;
    MotionVelocityRamp ramp(c);
    MotionCommand sent;
    sent.vx_mps = .07f;
    sent.wz_rps = .2f;
    ramp.accept(sent, rig.input.now_us);
    bool gap_decelerated = false, replacement_accepted = false, handed_off = false;
    bool alignment_stopped = false, approach_resumed = false;
    const int centered_tick = int(kStartupTargetLossHoldUs / kTick) + 30;
    for (int i = 0; i < 800; ++i) {
        rig.input.heading_rad = wrapAngle(rig.input.heading_rad + sent.wz_rps * (kTick * 1e-6f));
        rig.input.sent_vx_mps = sent.vx_mps;
        rig.input.sent_wz_rps = sent.wz_rps;
        if (i >= 10) {
            rig.input.target_valid = rig.input.target_center_px_valid = true;
            rig.input.target_id = 8;
            rig.input.label = "ordinary_supply";
            rig.input.pixel_grasp_view = rig.input.pixel_grasp_target_valid = true;
            // Keep this ramp/braking case in the coarse correction band at any restart setting.
            rig.input.pixel_grasp_error_px = i < centered_tick
                ? -3.f*t.pixel_align_approach_restart_px : 0.f;
            rig.input.target_center_px = {640.f-rig.input.pixel_grasp_error_px, 400.f};
            rig.input.pixel_view_handoff_ready = false;
            rig.input.startup_lower_corners_occupied = approach_resumed;
            rig.input.startup_visible_classes = 1;
            rig.input.startup_scene_us = rig.input.now_us + kTick;
            rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
        }
        rig.supplyImu(sent.vx_mps, sent.wz_rps);
        const auto out = rig.tick();
        const auto next = ramp.next(out.motion, rig.input.now_us, out.immediate_motion_stop);
        if (out.immediate_motion_stop) {
            assert(next.vx_mps == 0 && next.wz_rps == 0);
            if (i == centered_tick) {
                assert(out.reason == "pixel_align_brake" && out.target_id == 8);
                assert(sent.wz_rps < -t.pixel_align_stopped_wz);
                alignment_stopped = true;
            }
        } else {
            const float dt = c.period_ms * .001f; // sender caps elapsed time per write
            assert(std::abs(next.vx_mps - sent.vx_mps) <= c.linear_decel_mps2 * dt + 1e-6f);
            assert(std::abs(next.wz_rps - sent.wz_rps) <= c.angular_decel_rps2 * dt + 1e-6f);
        }
        approach_resumed |= i > centered_tick && out.target_id == 8 && out.motion.vx_mps > 0;
        if (i == 0) {
            assert(out.reason == "startup_locked_target_wait");
            assert(!out.immediate_motion_stop);
            gap_decelerated = next.vx_mps > 0 && next.vx_mps < sent.vx_mps &&
                          next.wz_rps > 0 && next.wz_rps < sent.wz_rps;
        }
        replacement_accepted |= out.target_id == 8;
        if (rig.task.startup_action_ == PushTask::StartupAction::SelectOrdinary) {
            assert(out.motion.camera_pitch_cdeg == t.track_pitch_cdeg);
            assert(std::abs(sent.vx_mps) <= t.pixel_align_stopped_vx);
            assert(std::abs(sent.wz_rps) <= t.pixel_align_stopped_wz);
            handed_off = true;
        }
        ramp.accept(next, rig.input.now_us);
        sent = next;
        if (handed_off) break;
    }
    assert(gap_decelerated && replacement_accepted && handed_off);
    assert(alignment_stopped && approach_resumed);
    // Restore an approach lock to test a blocked gap, independently of the
    // completed half turn (which deliberately released its old image target).
    rig.task.startup_pitch_ = t.track_pitch_cdeg;
    rig.task.startup_action_ = PushTask::StartupAction::Approach;
    rig.task.trip_context_.target_id_ = 8;
    rig.task.trip_context_.label_ = "ordinary_supply";
    rig.input.target_valid = false;
    rig.input.startup_lower_corners_occupied = false;
    rig.input.path_safe = false;
    const auto out = rig.tick();
    assert(out.immediate_motion_stop);
    const auto stopped = ramp.next(out.motion, rig.input.now_us, out.immediate_motion_stop);
    assert(stopped.vx_mps == 0 && stopped.wz_rps == 0);
}

void startupPitchTurnFeedbackAndClearance() {
    for (int zone : {1, 2, 3, 4}) {
        { // UI120 maneuver finishes in the same view, searching only ordinary cargo.
            TaskTuning t;
            t.startup_advance_us = 2000000;
            t.start_zone = zone;
            t.pixel_grasp_enabled = true;
            t.max_wz = .2f; // only this half-turn command is allowed to request 1 rad/s
            t.far_pitch_cdeg = -3000;
            t.track_pitch_cdeg = 0;
            t.near_pitch_cdeg = 2000;
            Rig rig(t);
            rig.tick();
            const auto from = t.track_pitch_cdeg;
            const auto to = t.track_pitch_cdeg;
            const float sign = zone == 2 || zone == 3 ? -1.f : 1.f;
            rig.task.startup_pitch_ = from;
            rig.task.startup_action_ = PushTask::StartupAction::TurnBeforePitch;
            rig.input.camera_pitch_cdeg = from;
            rig.input.gripper_done = true;
            rig.input.gripper_feedback_open = 0;
            rig.input.sent_velocity_valid = true;
            rig.input.directional_clearance_valid = true;
            rig.input.turn_safe = rig.input.arc_safe = true;
            // Intentionally no image target throughout this turn.
            rig.input.heading_valid = false;
            auto out = rig.tick();
            assert(out.reason == "startup_pitch_turn_heading_wait");
            assert(out.immediate_motion_stop && out.motion.wz_rps == 0);
            rig.input.heading_valid = true;
            rig.input.heading_rad = std::numeric_limits<float>::quiet_NaN();
            out = rig.tick();
            assert(out.reason == "startup_pitch_turn_heading_wait");
            assert(out.immediate_motion_stop && out.motion.wz_rps == 0);
            rig.input.heading_rad = 3.f;
            out = rig.tick();
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == sign);
            assert(out.motion.camera_pitch_cdeg == from);
            rig.input.turn_safe = false;
            out = rig.tick();
            assert(out.motion.wz_rps == 0 && out.immediate_motion_stop);
            assert(rig.task.startup_pitch_turn_.angle == 0);
            rig.input.turn_safe = true;
            // Wrong-way motion must subtract from the net half turn.
            rig.input.heading_rad = wrapAngle(3.f - sign * .2f);
            out = rig.tick();
            assert(std::abs(rig.task.startup_pitch_turn_.angle + .2f) < 1e-5f);
            rig.input.heading_rad = 3.f;
            rig.tick();
            assert(std::abs(rig.task.startup_pitch_turn_.angle) < 1e-5f);
            // A stale IMU stops immediately; the next valid heading is a new reference.
            rig.input.sent_wz_rps = sign;
            rig.input.heading_valid = false;
            out = rig.tick();
            assert(out.motion.wz_rps == 0 && out.immediate_motion_stop);
            rig.input.sent_wz_rps = 0;
            rig.input.heading_valid = true;
            rig.input.heading_rad = wrapAngle(3.f + sign * .3f);
            rig.tick();
            const float before_pause = rig.task.startup_pitch_turn_.angle;
            rig.input.run = false;
            out = rig.tick();
            assert(out.run_status == RunStatus::Paused && out.motion.wz_rps == 0);
            rig.input.heading_rad = wrapAngle(rig.input.heading_rad + sign * .5f);
            rig.input.run = true;
            rig.tick(); // resume the Startup state
            rig.tick(); // establish its heading reference without counting the pause
            assert(std::abs(rig.task.startup_pitch_turn_.angle - before_pause) < 1e-5f);
            for (int i = 0; i < 70; ++i) {
                rig.input.heading_rad = wrapAngle(rig.input.heading_rad + sign * .1f);
                rig.input.sent_wz_rps = sign;
                out = rig.tick();
                assert(out.motion.vx_mps == 0 && out.motion.camera_pitch_cdeg == from);
                assert(rig.task.frame_raised_ == 0);
                if (rig.task.startup_action_ == PushTask::StartupAction::StopBeforePitch)
                    break;
                assert(out.motion.wz_rps == sign);
            }
            assert(rig.task.startup_pitch_turn_.angle >= float(CV_PI));
            assert(rig.task.startup_pitch_turn_.angle < float(CV_PI) + .11f);
            assert(rig.task.startup_action_ == PushTask::StartupAction::StopBeforePitch);
            assert(out.immediate_motion_stop && out.motion.wz_rps == 0);
            // Still moving: even after a half turn the camera must not switch.
            for (int i = 0; i < 8; ++i) {
                out = rig.tick();
                assert(out.motion.camera_pitch_cdeg == from && out.immediate_motion_stop);
            }
            rig.input.sent_wz_rps = 0;
            for (int i = 0; i < 5; ++i) {
                out = rig.tick();
                assert(out.motion.camera_pitch_cdeg == from);
            }
            out = rig.tick(); // first half turn must lead to reverse, not a pitch switch
            assert(out.motion.camera_pitch_cdeg == from && out.motion.wz_rps == 0);
            assert(out.reason == "startup_pitch_reverse_calibrate");
            // No elapsed command time can substitute for displacement samples.
            for (int i = 0; i < 20; ++i) {
                out = rig.tick();
                assert(out.motion.vx_mps == 0 && out.motion.camera_pitch_cdeg == from);
            }
            // Calibrate with real device-timed stationary samples.
            for (int i = 0; i < 20 && out.motion.vx_mps == 0; ++i)
                out = rig.motionTick(out);
            assert(out.motion.vx_mps < 0 && out.motion.wz_rps == 0);
            // A blocked rear corridor stops reverse and never starts the second turn.
            rig.input.retreat_safe = false;
            out = rig.motionTick(out, false);
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            rig.input.retreat_safe = true;
            for (int i = 0; i < 30; ++i) {
                out = rig.motionTick(out, false);
                assert(out.motion.wz_rps == 0 && out.motion.camera_pitch_cdeg == from);
            }
            assert(out.motion.vx_mps < 0); // commands alone made no reverse progress
            const float reverse_origin = rig.plant_position;
            for (int i = 0; i < 10; ++i)
                out = rig.motionTick(out);
            assert(reverse_origin - rig.plant_position > .01f);
            // Pause preserves measured distance and requires fresh stationary calibration.
            rig.input.run = false;
            out = rig.motionTick(out, false);
            assert(out.run_status == RunStatus::Paused && out.motion.vx_mps == 0);
            rig.input.run = true;
            bool second_turn = false;
            float second_angle = 0;
            for (int i = 0; i < 250; ++i) {
                second_angle += sign * out.motion.wz_rps * (kTick * 1e-6f);
                out = rig.motionTick(out);
                assert(out.motion.vx_mps <= 0 && rig.task.frame_raised_ == 0);
                if (out.motion.wz_rps != 0) {
                    second_turn = true;
                    assert(out.motion.wz_rps == sign && out.motion.camera_pitch_cdeg == from);
                }
                if (rig.task.startup_action_ == PushTask::StartupAction::SelectOrdinary) break;
                assert(out.motion.camera_pitch_cdeg == from);
            }
            assert(second_turn && second_angle >= float(CV_PI));
            assert(second_angle < float(CV_PI) + .1f);
            assert(reverse_origin - rig.plant_position >= .09f);
            assert(reverse_origin - rig.plant_position < .12f);
            assert(out.motion.camera_pitch_cdeg == to && out.motion.wz_rps == 0);
            assert(rig.task.startup_action_ == PushTask::StartupAction::SelectOrdinary);
            assert(rig.task.startupOrdinaryOnly());
            rig.input.reset = true;
            rig.tick();
            assert(rig.task.startup_pitch_turn_.angle == 0);
            assert(!rig.task.startup_pitch_turn_.heading_valid);
        }
    }
}

void imuCalibrationKeepsOtherPhaseLimits() {
    using Status = ImuTranslation::CalibrationStatus;
    const auto check = [](float gyro, float noise, bool startup, bool expected) {
        ImuTranslation estimator;
        Status last_rejection = Status::Collecting;
        uint64_t now = kStart;
        for (int i = 0; i < 100; ++i) {
            now += 10000;
            ImuTranslationSample sample;
            sample.sequence = i + 1;
            sample.received_us = now;
            sample.device_time_ms = now / 1000;
            sample.acceleration_mps2 = {.01f + (i % 2 ? noise : -noise), 0, 9.8f};
            sample.angular_velocity_rps = {0, 0, gyro};
            sample.quaternion_wxyz = {1, 0, 0, 0};
            sample.valid = true;
            const bool ready = startup
                ? estimator.calibrate({sample}, now, 0, .15f, .30f, 500000)
                : estimator.calibrate({sample}, now, 0);
            if (ready) {
                assert(expected);
                return estimator.calibrationStatus();
            }
            if (estimator.calibrationStatus() != Status::Collecting)
                last_rejection = estimator.calibrationStatus();
        }
        assert(!expected);
        return last_rejection;
    };
    assert(check(.10f, 0, false, false) == Status::AngularMotion);
    assert(check(0, .20f, false, false) == Status::AccelerationNoise);
    assert(check(.10f, 0, true, true) == Status::Ready);
    assert(check(0, .20f, true, true) == Status::Ready);
    assert(check(.10f, .20f, true, true) == Status::Ready);
    assert(check(.20f, 0, true, false) == Status::AngularMotion);
    assert(check(0, .50f, true, false) == Status::AccelerationNoise);
}

// Reproduce the 22:54 run's post-turn noise: the old calibration never let
// Startup issue a reverse command. These are synthetic noise levels informed
// by the log, rather than a replay of unavailable raw IMU samples.
void startupReverseCalibratesWithPostTurnNoise() {
    for (int zone : {1, 2, 3, 4}) {
        TaskTuning t;
        t.startup_advance_us = 2000000;
        t.start_zone = zone;
        t.pixel_grasp_enabled = true;
        t.far_pitch_cdeg = -3000;
        t.track_pitch_cdeg = 0;
        t.near_pitch_cdeg = 2000;
        Rig rig(t);
        rig.tick();
        rig.task.startup_pitch_ = t.track_pitch_cdeg;
        rig.task.startup_action_ = PushTask::StartupAction::TurnBeforePitch;
        rig.input.camera_pitch_cdeg = t.track_pitch_cdeg;
        rig.input.gripper_done = true;
        rig.input.gripper_feedback_open = 0;
        rig.input.directional_clearance_valid = true;
        rig.input.turn_safe = rig.input.arc_safe = true;
        rig.gyro_noise_rps = .10f;
        rig.accel_noise_mps2 = .20f;
        PushOutput out;
        bool reversed = false, second_turn = false;
        float origin = 0;
        for (int i = 0; i < 400; ++i) {
            out = rig.motionTick(out);
            assert(out.motion.camera_pitch_cdeg == t.track_pitch_cdeg);
            assert(rig.task.frame_raised_ == 0);
            if (out.motion.vx_mps < 0 && !reversed) {
                reversed = true;
                origin = rig.plant_position;
            }
            second_turn |= reversed && out.motion.wz_rps != 0;
            if (rig.task.startup_action_ == PushTask::StartupAction::SelectOrdinary) break;
        }
        assert(reversed && second_turn);
        assert(out.motion.camera_pitch_cdeg == t.track_pitch_cdeg);
        assert(rig.task.startup_action_ == PushTask::StartupAction::SelectOrdinary);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        assert(origin - rig.plant_position > .085f);
        assert(origin - rig.plant_position < .12f);
    }
    std::cout << "PASS Startup post-turn gyro/acceleration noise -> reverse 0.1m -> second half turn\n";
}

// Field trace: a visible UI90 lock with valid ground geometry but unknown
// zone membership must not be cleared every hold_grace_frames ticks.
// First delivery completed, fresh visible lock, no own-zone sighting: approach must run.
void searchFarUnseenZoneApproaches() {
    for (const char* label : {"ordinary_supply", "core_supply", "injured_person"}) {
        TaskTuning tuning;
        tuning.pixel_grasp_enabled = tuning.field_carry = true;
        tuning.far_pitch_cdeg = -3000;
        Rig rig(tuning);
        rig.task.first_ = true;
        rig.task.enter(PushState::VERIFY_DELIVERY, kStart, "delivery_test");
        rig.task.phase_outcome_ = PhaseOutcome::SearchReady;
        rig.task.applyPhaseOutcome(kStart);
        auto& in = rig.input;
        in.target_id = 172;
        in.label = label;
        in.target_valid = in.geometry_valid = in.target_center_px_valid = true;
        in.target_region_valid = false;
        in.target_box = {717,512,86,127};
        in.target_center_px = {760,575.5f};
        in.distance_m = .40f;
        in.heading_error = -.12f;
        in.pixel_selection_managed = in.pixel_locked = true;
        in.pixel_reason = "pixel_ratio_locked";
        in.gripper_done = true;
        in.gripper_feedback_open = 1;
        in.search_target_region = PushObservation::SearchRegion::Outside;
        in.search_region_reason = "zone_not_visible_assumed_outside";
        PushOutput out;
        bool approached = false;
        for (int i=0; i<20; ++i) {
            in.search_region_sample_us = in.pixel_grasp_sample_us = in.now_us + kTick;
            out = rig.tick();
            if (out.reason == "pixel_far_approach") {
                assert(out.motion.vx_mps > 0 || out.motion.wz_rps != 0);
                approached = true;
                break;
            }
        }
        assert(approached && out.target_id == 172);
        // Once the zone becomes visible and contains this target, stop and release it.
        in.search_target_region = PushObservation::SearchRegion::Inside;
        in.search_region_reason = "zone_box_inside";
        in.search_region_sample_us = in.pixel_grasp_sample_us = in.now_us + kTick;
        out = rig.tick();
        assert(out.state == PushState::SCAN && out.target_id == -1);
        assert(out.reason == "search_target_in_zone" && out.immediate_motion_stop);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    }
    // Split discovery at UI120 also checks the selector's lock before its spin branch.
    TaskTuning tuning;
    tuning.pixel_grasp_enabled = tuning.field_carry = true;
    tuning.track_pitch_cdeg = 0;
    Rig split(tuning);
    split.task.first_ = true;
    split.task.enter(PushState::APPROACH,kStart,"split_test");
    split.task.frame_raised_ = 1;
    split.task.search_phase_.approach_pitch_ = tuning.track_pitch_cdeg;
    split.task.search_phase_.first_trip_split_.search_started_us = kStart;
    auto& in = split.input;
    in.camera_pitch_cdeg = tuning.track_pitch_cdeg;
    in.target_id = 172; in.label = "injured_person";
    in.target_valid = in.pixel_locked = in.pixel_selection_managed = true;
    in.search_target_region = PushObservation::SearchRegion::Inside;
    in.search_region_sample_us = in.now_us + kTick;
    in.gripper_done = true; in.gripper_feedback_open = 1;
    const auto out = split.tick();
    assert(out.reason == "search_target_in_zone" && out.target_id == -1);
    assert(out.immediate_motion_stop && out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    std::cout << "PASS post-delivery unseen-zone lock approaches; inside target releases before split discovery\n";
}

void searchFarRegionUnknownKeepsLock() {
    for (int scenario = 0; scenario < 6; ++scenario) {
        TaskTuning tuning;
        tuning.pixel_grasp_enabled = tuning.field_carry = true;
        tuning.far_pitch_cdeg = -3000;
        tuning.track_pitch_cdeg = 0;
        tuning.near_pitch_cdeg = 2000;
        Rig rig(tuning);
        rig.task.first_ = true;
        rig.task.enter(PushState::VERIFY_DELIVERY, kStart, "delivery_test");
        rig.task.phase_outcome_ = PhaseOutcome::SearchReady;
        rig.task.applyPhaseOutcome(kStart);
        assert(rig.task.state_ == PushState::SCAN);
        auto& in = rig.input;
        in.target_id = 438;
        in.label = "core_supply";
        in.target_valid = in.geometry_valid = in.target_center_px_valid = true;
        in.target_region_valid = false;
        in.target_box = {1152, 454, 61, 52};
        in.target_center_px = {1182.5f, 480};
        in.distance_m = .69f;
        in.heading_error = -.59f;
        in.pixel_selection_managed = in.pixel_locked = true;
        in.pixel_reason = "pixel_ratio_locked";
        in.sent_velocity_valid = in.gripper_done = true;
        in.gripper_feedback_open = 0;
        const auto step = [&] {
            in.pixel_grasp_sample_us = in.now_us + kTick;
            return rig.tick();
        };
        PushOutput out;
        for (int i = 0; i < 10; ++i) {
            out = step();
            if (out.state == PushState::APPROACH) break;
        }
        assert(out.state == PushState::APPROACH && out.target_id == 438);
        const auto deadline = rig.task.search_phase_.pixel_approach_started_us_;
        for (int i = 0; i < tuning.hold_grace_frames + 4; ++i) {
            out = step();
            assert(out.state == PushState::APPROACH && out.target_id == 438);
            assert(out.reason == "pixel_far_region_wait");
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            assert(out.immediate_motion_stop);
            assert(rig.task.actionProgress().misses == 0);
            assert(rig.task.search_phase_.pixel_approach_started_us_ == deadline);
        }
        if (scenario == 0) {
            in.target_region_valid = true;
            out = step();
            assert(out.reason == "pixel_far_approach" && out.motion.wz_rps < 0);
            assert(rig.task.search_phase_.far_region_wait_started_us_ == 0);
            assert(out.target_id == 438);
        } else if (scenario == 1) {
            const auto waiting_at = in.now_us;
            const auto started = rig.task.search_phase_.far_region_wait_started_us_;
            int jitter = 0;
            while (in.now_us - waiting_at <= tuning.prepare_budget_us + kTick) {
                in.geometry_valid = (++jitter % 2) == 0;
                out = step();
                if (out.state == PushState::APPROACH)
                    assert(rig.task.search_phase_.far_region_wait_started_us_ == started);
                assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
                if (out.state == PushState::SCAN) break;
            }
            assert(out.state == PushState::SCAN && out.reason == "pixel_far_region_timeout");
            assert(out.target_id == -1 && out.immediate_motion_stop);
            const auto rejected = rig.task.rejectedTargets(in.now_us);
            assert(std::find(rejected.begin(), rejected.end(), 438) != rejected.end());
            // A still-visible old lock cannot immediately renew the wait window.
            for (int i = 0; i < 20; ++i) {
                out = step();
                assert(out.state == PushState::SCAN && out.target_id == -1);
            }
            in.target_id = 439;
            in.geometry_valid = true;
            in.target_region_valid = true;
            for (int i = 0; i < 10; ++i) {
                out = step();
                if (out.state == PushState::APPROACH) break;
            }
            assert(out.state == PushState::APPROACH && out.target_id == 439);
        } else if (scenario == 2) {
            in.pixel_released_id = 438;
            in.target_valid = false;
            out = step();
            assert(out.reason == "pixel_target_released" && out.target_id == -1);
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        } else if (scenario == 4) {
            in.target_in_zone = in.target_region_valid = true;
            out = step();
            assert(out.state == PushState::SCAN && out.target_id == -1);
            assert(out.reason == "search_target_in_zone" && out.immediate_motion_stop);
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        } else {
            if (scenario == 3) in.geometry_valid = false;
            if (scenario == 5) in.target_id = 439; // unrelated identity is still lost
            for (int i = 0; i <= tuning.hold_grace_frames; ++i) {
                out = step();
                assert(out.reason != "pixel_far_region_wait");
                assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            }
            assert(out.state == PushState::SCAN && out.reason == "pixel_far_target_lost");
        }
    }
    std::cout << "PASS post-delivery FAR region wait, recovery, timeout cooldown and real loss\n";
}

void transportReturnsToFarScan() {
    for (bool jaw : {false,true}) {
        TaskTuning tuning;
        tuning.startup_advance_us = 2000000;
        tuning.pixel_grasp_enabled = jaw;
        tuning.field_carry = true;
        tuning.far_pitch_cdeg = -3000;
        tuning.track_pitch_cdeg = 0;
        tuning.near_pitch_cdeg = 2000;
        Rig rig(tuning);
        rig.task.startup_action_ = PushTask::StartupAction::Approach;
        rig.task.enter(PushState::VERIFY_DELIVERY, kStart, "delivery_test");
        rig.task.phase_outcome_ = PhaseOutcome::SearchReady;
        rig.task.applyPhaseOutcome(rig.input.now_us);
        assert(rig.task.state_ == (jaw ? PushState::SCAN : PushState::TURN_SCAN));
        assert(rig.task.desiredPitch() == tuning.far_pitch_cdeg);
        if (!jaw) rig.task.actionProgress().turned = tuning.turn_min_rad;
        const auto out = rig.tick();
        assert(out.phase == TaskPhase::Search && out.state == PushState::SCAN);
        assert(rig.task.desiredPitch() == tuning.far_pitch_cdeg);
    }
}

void prepareTransfersInventoryAndRushLimit(bool stepping = false) {
    TaskTuning tuning;
    tuning.require_multi_view = false;
    tuning.corridor_confirm_frames = 1;
    Rig rig(tuning);
    rig.task.first_ = true; // the two-object handoff is a later trip
    rig.task.setPhaseStepping(stepping);
    rig.task.enter(PushState::PREPARE, kStart, "prepare_test");
    rig.task.actionProgress().step = 1;
    rig.task.frame_raised_ = 1;
    rig.task.pitch_cmd_ = tuning.track_pitch_cdeg;
    rig.input.camera_pitch_cdeg = tuning.track_pitch_cdeg;
    rig.input.gripper_done = true;
    rig.input.gripper_feedback_open = 1;
    rig.input.target_valid = rig.input.geometry_valid = rig.input.target_region_valid = true;
    rig.input.target_id = rig.task.trip_context_.target_id_ = 7;
    rig.input.label = rig.task.trip_context_.label_ = "ordinary_supply";
    rig.input.distance_m = .3f;
    rig.input.corridor.ordinary = 2;
    rig.input.corridor_complete = rig.input.corridor_occlusion_free = true;
    rig.task.transport_phase_.progress.travel = .77f;
    auto out = rig.tick();
    if (stepping) {
        assert(out.phase == TaskPhase::Search && out.next_phase == TaskPhase::Capture);
        assert(out.phase_waiting && out.run_status == RunStatus::Paused);
        const auto frame = out.motion.gripper_offset;
        const auto txn = out.motion.frame_transaction;
        const auto pitch = out.motion.camera_pitch_cdeg;
        rig.input.run = rig.input.safety_ok = false;
        rig.input.now_us += 300000000; // human inspection cannot exhaust capture timers
        out = rig.tick();
        assert(out.phase_waiting && out.phase == TaskPhase::Search && out.target_id == 7);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        assert(out.motion.gripper_offset == frame && out.motion.frame_transaction == txn);
        assert(out.motion.camera_pitch_cdeg == pitch && out.batch_size == 2);
        rig.input.run = rig.input.safety_ok = true;
        out = rig.tick();
        assert(!out.phase_waiting && rig.task.actionProgress().started_us == rig.input.now_us);
    }
    assert(out.phase == TaskPhase::Capture && out.capture_action == CaptureAction::Advance);
    assert(out.batch_size == 2 && rig.task.trip_context_.pending_.ordinary == 2);
    assert(out.capture_evidence == CaptureEvidence::None);
    const float limit = .3f - tuning.grasp_trigger_y_m + tuning.rush_extra_m;
    assert(std::abs(rig.task.capture_phase_.progress.travel_limit - limit) < 1e-6f);
    assert(rig.task.capture_phase_.progress.travel == 0);
    assert(rig.task.transport_phase_.progress.travel == .77f);
    assert(out.corridor_observation_requested && !out.navigation_requested);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
}

void pauseKeepsCargoAndRecoveryKeepsItsOwner() {
    TaskTuning tuning;
    tuning.require_multi_view = false;
    Rig rig(tuning);
    rig.task.enter(PushState::CARRY, kStart, "carry_test");
    rig.task.trip_context_.trip_.ordinary = 1;
    rig.task.capture_phase_.evidence = CaptureEvidence::Verified;
    rig.input.run = false;
    auto out = rig.tick();
    assert(out.phase == TaskPhase::Transport && out.run_status == RunStatus::Paused);
    assert(out.batch_size == 1 && out.capture_evidence == CaptureEvidence::Verified);
    assert(!out.navigation_requested && out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    rig.input.run = true;
    out = rig.tick();
    assert(out.phase == TaskPhase::Transport && out.transport_action == TransportAction::Carry);
    assert(out.batch_size == 1 && out.capture_evidence == CaptureEvidence::Verified);
    assert(rig.task.transport_phase_.progress.step == 1);

    rig.task.enter(PushState::LOST_HOLD, rig.input.now_us, "hold_lost");
    out = rig.tick();
    assert(out.phase == TaskPhase::Transport && out.recovering);
    assert(out.transport_action == TransportAction::Recover && !out.navigation_requested);
    rig.input.gripper_done = true;
    rig.input.gripper_feedback_open = 1;
    for (int n = 0; n < 20 && out.phase != TaskPhase::Search; ++n)
        out = rig.tick();
    assert(out.phase == TaskPhase::Search && out.search_action == SearchStatusAction::Scan);
    assert(!out.recovering && out.batch_size == 0);
    assert(out.capture_evidence == CaptureEvidence::None);
    rig.task.fail("test_fault");
    out = rig.tick();
    assert(out.phase == TaskPhase::Search && out.run_status == RunStatus::Fault);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
}
void checkpointStartupCaptureAndTransport() {
    Rig startup;
    startup.task.setPhaseStepping(true);
    startup.tick();
    auto out = startup.tick();
    assert(out.phase_waiting && out.phase == TaskPhase::Startup &&
           out.next_phase == TaskPhase::Search);
    startup.input.run = startup.input.safety_ok = false;
    for (int n = 0; n < 5; ++n) {
        out = startup.tick();
        assert(out.phase_waiting && out.phase == TaskPhase::Startup);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    }
    startup.input.run = startup.input.safety_ok = true;
    out = startup.tick();
    assert(!out.phase_waiting && out.phase == TaskPhase::Search);

    TaskTuning tuning;
    tuning.require_multi_view = true;
    Rig capture(tuning);
    capture.task.setPhaseStepping(true);
    capture.task.enter(PushState::VERIFY_CAPTURE, kStart, "capture_test");
    capture.task.trip_context_.pending_.ordinary = 1;
    capture.task.trip_context_.label_ = "ordinary_supply";
    capture.input.multi_view_finished = true;
    capture.input.multi_view_verdict = 1;
    capture.input.multi_view_inventory.ordinary = 1;
    out = capture.tick();
    assert(out.phase_waiting && out.phase == TaskPhase::Capture &&
           out.next_phase == TaskPhase::Transport);
    assert(out.batch_size == 1 && out.capture_evidence == CaptureEvidence::Verified);
    capture.input.run = capture.input.safety_ok = false;
    capture.input.now_us += 300000000;
    out = capture.tick();
    assert(out.phase_waiting && out.capture_evidence == CaptureEvidence::Verified);
    capture.input.run = capture.input.safety_ok = true;
    out = capture.tick();
    assert(out.phase == TaskPhase::Transport && !out.phase_waiting);
    assert(out.batch_size == 1 && capture.task.transport_phase_.carry_us_ == capture.input.now_us);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);

    TaskTuning delivery_tuning;
    delivery_tuning.require_multi_view = false;
    delivery_tuning.delivery_budget_us = 1;
    Rig delivery(delivery_tuning);
    delivery.task.setPhaseStepping(true);
    delivery.task.enter(PushState::VERIFY_DELIVERY, kStart, "delivery_test");
    delivery.task.trip_context_.trip_.ordinary = 1;
    out = delivery.tick();
    assert(out.phase_waiting && out.phase == TaskPhase::Transport &&
           out.next_phase == TaskPhase::Search);
    assert(out.delivery_expected == 1 && out.delivery_credited == 0);
    delivery.input.run = delivery.input.safety_ok = false;
    for (int n = 0; n < 5; ++n) {
        out = delivery.tick();
        assert(out.phase_waiting && out.delivered_total == 0);
    }
    delivery.input.run = delivery.input.safety_ok = true;
    out = delivery.tick();
    assert(!out.phase_waiting && out.phase == TaskPhase::Search);
    assert(out.search_action == SearchStatusAction::TurnAway);
    delivery.input.reset = true;
    out = delivery.tick();
    assert(!out.phase_waiting && out.phase == TaskPhase::Startup);
}
void directPhasesSkipPrecedingPhases() {
    Rig startup;
    startup.task.setSinglePhase("startup");
    startup.task.setInitialFrameOpen(true);
    auto out = startup.tick();
    assert(out.phase == TaskPhase::Startup && out.motion.gripper_offset == 0);
    out = startup.tick();
    assert(out.single_phase_done && out.phase == TaskPhase::Startup);
    out = startup.tick();
    assert(out.single_phase_done && out.motion.vx_mps == 0 && out.motion.wz_rps == 0);

    Rig search;
    search.task.setSinglePhase("search");
    out = search.tick();
    assert(out.phase == TaskPhase::Search && out.state == PushState::SCAN);
    assert(!out.single_phase_done && out.motion.vx_mps == 0);

    TaskTuning tuning;
    tuning.require_multi_view = false;
    tuning.corridor_confirm_frames = 1;
    tuning.capture_frames = 1;
    Rig capture(tuning);
    capture.task.setSinglePhase("capture");
    out = capture.tick();
    assert(out.phase == TaskPhase::Capture && out.state == PushState::RUSH);
    assert(out.corridor_observation_requested && out.motion.vx_mps == 0);
    capture.input.camera_pitch_cdeg = tuning.near_pitch_cdeg;
    out = capture.tick();
    assert(capture.task.single_phase_initializing_ && out.batch_size == 0);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    capture.input.gripper_done = true;
    capture.input.gripper_feedback_open = 1;
    capture.input.target_valid = capture.input.geometry_valid = capture.input.target_region_valid =
        true;
    capture.input.target_id = 7;
    capture.input.label = "ordinary_supply";
    capture.input.distance_m = .3f;
    capture.input.corridor.ordinary = 1;
    capture.input.corridor_complete = capture.input.corridor_occlusion_free = true;
    out = capture.tick();
    assert(!capture.task.single_phase_initializing_ && out.target_id == 7 && out.batch_size == 1);
    assert(out.phase == TaskPhase::Capture && out.motion.vx_mps == 0);
    assert(capture.task.actionProgress().travel_limit > 0);
    capture.task.enter(PushState::VERIFY_CAPTURE, capture.input.now_us, "test_enclosure");
    capture.input.hold_observable = capture.input.captured = capture.input.held_complete = true;
    capture.input.held.ordinary = 1;
    out = capture.tick();
    assert(out.single_phase_done && out.phase == TaskPhase::Capture);
    assert(out.capture_evidence == CaptureEvidence::Verified);

    Rig transport(tuning);
    transport.task.setSinglePhase("transport");
    out = transport.tick();
    assert(out.phase == TaskPhase::Transport && out.state == PushState::CARRY);
    assert(out.capture_observation_requested && !out.navigation_requested);
    transport.input.camera_pitch_cdeg = tuning.near_pitch_cdeg;
    transport.input.gripper_done = true;
    transport.input.gripper_feedback_open = 0;
    out = transport.tick();
    assert(transport.task.single_phase_initializing_ && out.batch_size == 0);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    transport.input.hold_observable = transport.input.captured = transport.input.held_complete =
        true;
    transport.input.held.ordinary = 1;
    out = transport.tick();
    assert(!transport.task.single_phase_initializing_ && out.batch_size == 1);
    assert(out.phase == TaskPhase::Transport && out.capture_evidence == CaptureEvidence::Verified);
    assert(transport.task.transport_phase_.carry_us_ == transport.input.now_us);

    TaskTuning multi_tuning;
    multi_tuning.require_multi_view = true;
    Rig multi(multi_tuning);
    multi.task.setSinglePhase("transport");
    multi.tick();
    multi.input.camera_pitch_cdeg = multi_tuning.near_pitch_cdeg;
    multi.input.gripper_done = true;
    multi.input.gripper_feedback_open = 0;
    multi.input.multi_view_finished = true;
    multi.input.multi_view_verdict = 1;
    multi.input.multi_view_inventory.dangerous = 1;
    out = multi.tick();
    assert(multi.task.single_phase_initializing_ && out.batch_size == 0);
    multi.input.multi_view_inventory = {};
    multi.input.multi_view_inventory.ordinary = 1;
    out = multi.tick();
    assert(!multi.task.single_phase_initializing_ && out.batch_size == 1);
    assert(out.phase == TaskPhase::Transport && out.motion.vx_mps == 0);

    Rig absent(tuning);
    absent.task.setSinglePhase("capture");
    absent.tick();
    absent.input.camera_pitch_cdeg = tuning.near_pitch_cdeg;
    for (int n = 0; n < 165; ++n)
        out = absent.tick();
    assert(out.run_status == RunStatus::Fault && !out.single_phase_done);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
}

void directCaptureAfterFirstKeepsEvidenceChecks() {
    for (bool pixel : {false, true}) for (int scenario = 0; scenario < 3; ++scenario)
        for (bool after_first : {false, true}) {
            TaskTuning tuning;
            tuning.require_multi_view = false;
            tuning.pixel_grasp_enabled = pixel;
            tuning.corridor_confirm_frames = tuning.capture_frames = 1;
            Rig rig(tuning);
            rig.task.setSinglePhase("capture", after_first);
            auto out = rig.tick();
            assert(out.first_ordinary_delivered == after_first && out.delivered_total == 0);
            // RESET must retain this session's explicit initial history.
            rig.input.reset = true;
            out = rig.tick();
            assert(out.first_ordinary_delivered == after_first && out.delivered_total == 0);
            rig.input.reset = false;
            rig.tick();
            auto& in = rig.input;
            in.camera_pitch_cdeg = tuning.near_pitch_cdeg;
            in.gripper_done = true;
            in.gripper_feedback_open = 1;
            in.target_valid = in.geometry_valid = in.target_region_valid = true;
            in.target_id = 1;
            in.label = scenario == 0 ? "injured_person" :
                       scenario == 1 ? "ordinary_supply" : "core_supply";
            in.distance_m = .3f;
            if (scenario == 0) in.corridor.injured = 1;
            else { in.corridor.ordinary = 1; in.corridor.core = 1; }
            in.corridor_complete = in.corridor_occlusion_free = true;
            in.pixel_grasp_view = in.pixel_grasp_near_view = true;
            in.pixel_grasp_target_valid = in.target_center_px_valid = true;
            in.target_center_px = {640, 350};
            in.target_grasp_range_valid = in.target_grasp_range_inside = true;
            in.sent_velocity_valid = true;
            in.capture_scene_valid = in.capture_path_valid = true;
            in.capture_area_px = in.layout_count_region_px = {200, 220, 880, 300};
            PushObservation::CaptureImageObject primary;
            primary.id = 1; primary.label = in.label;
            primary.box = {600, 310, 80, 80}; primary.complete = true;
            in.capture_image_objects = {primary};
            if (scenario != 0) {
                auto other = primary;
                other.id = 2; other.box.x = 800;
                other.label = scenario == 1 ? "core_supply" : "ordinary_supply";
                in.capture_image_objects.push_back(other);
            }
            in.capture_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
            out = rig.tick();
            if (!after_first) {
                assert(rig.task.single_phase_initializing_);
                assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
                assert(out.capture_evidence == CaptureEvidence::None);
                continue;
            }
            assert(!rig.task.single_phase_initializing_ && out.phase == TaskPhase::Capture);
            assert(rig.task.trip_context_.pending_ == in.corridor);
            assert(!out.single_phase_done && out.capture_evidence == CaptureEvidence::None);
            rig.task.enter(PushState::VERIFY_CAPTURE, in.now_us, "test_after_first_enclosure");
            // A legal plan does not replace observed captured cargo.
            in.capture_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
            out = rig.tick();
            assert(!out.single_phase_done && out.capture_evidence == CaptureEvidence::None);
            in.hold_observable = in.captured = in.held_complete = true;
            in.held = in.corridor;
            in.capture_scene_us = in.pixel_grasp_sample_us = in.now_us + kTick;
            out = rig.tick();
            assert(out.single_phase_done && out.capture_evidence == CaptureEvidence::Verified);
            assert(out.batch_size == (scenario == 0 ? 1 : 2));
            assert(out.first_ordinary_delivered && out.delivered_total == 0);
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        }
    // The seeded history still enforces one injured alone, two supplies maximum,
    // and rejection of dangerous or unknown cargo.
    for (int extra = 0; extra < 4; ++extra) {
        TaskTuning tuning;
        tuning.require_multi_view = false;
        tuning.corridor_confirm_frames = 1;
        Rig rig(tuning);
        rig.task.setSinglePhase("capture", true);
        rig.tick();
        auto& in = rig.input;
        in.camera_pitch_cdeg = tuning.near_pitch_cdeg;
        in.gripper_done = true; in.gripper_feedback_open = 1;
        in.target_valid = in.geometry_valid = in.target_region_valid = true;
        in.target_id = 1; in.label = "ordinary_supply"; in.distance_m = .3f;
        in.corridor.ordinary = 1;
        if (extra == 0) in.corridor.injured = 1;
        if (extra == 1) in.corridor.core = 2;
        if (extra == 2) in.corridor.dangerous = 1;
        if (extra == 3) in.corridor.unknown = 1;
        in.corridor_complete = in.corridor_occlusion_free = true;
        const auto out = rig.tick();
        assert(rig.task.single_phase_initializing_);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    }
}

void metricOvercaptureDoesNotStartSplit(bool single, int extra_kind,
                                       bool multiview = false, bool raw_held = false) {
    TaskTuning tuning;
    tuning.require_multi_view = multiview;
    tuning.field_rush = tuning.verify_held_contents = raw_held;
    tuning.held_recheck = false;
    tuning.capture_frames = 1;
    Rig rig(tuning);
    if (single)
        rig.task.setSinglePhase("capture");
    rig.task.enter(PushState::VERIFY_CAPTURE, kStart, "metric_overcapture");
    rig.task.trip_context_.label_ = "ordinary_supply";
    rig.task.trip_context_.target_id_ = 7;
    rig.task.trip_context_.pending_.ordinary = 1;
    rig.input.hold_observable = rig.input.captured = rig.input.held_complete = true;
    rig.input.held.ordinary = 1;
    if (extra_kind == 0)
        ++rig.input.held.ordinary;
    if (extra_kind == 1)
        ++rig.input.held.core;
    if (extra_kind == 2)
        ++rig.input.held.injured;
    if (extra_kind == 3)
        ++rig.input.held.dangerous;
    if (extra_kind == 4)
        ++rig.input.held.unknown;
    if (multiview && !raw_held) {
        rig.input.multi_view_finished = true;
        rig.input.multi_view_verdict = 1;
        rig.input.multi_view_inventory = rig.input.held;
    }
    rig.input.camera_pitch_cdeg = rig.task.desiredPitch();
    auto out = rig.tick();
    if (raw_held)
        out = rig.tick();
    assert(!rig.task.search_phase_.first_trip_split_.pending);
    assert(!rig.task.search_phase_.first_trip_split_.capture_retry);
    assert(out.capture_split_runs == 0 && !out.first_ordinary_delivered);
    assert(out.capture_evidence != CaptureEvidence::Verified);
    if (single)
        assert(out.run_status == RunStatus::Fault);
    else
        assert(out.state == PushState::ABORT_DROP && out.recovering);
}

void metricInitialCaptureRejectsIllegalCorridor(int extra_kind) {
    TaskTuning tuning;
    tuning.require_multi_view = false;
    tuning.corridor_confirm_frames = 1;
    Rig rig(tuning);
    rig.task.setSinglePhase("capture");
    rig.tick();
    rig.input.camera_pitch_cdeg = rig.task.desiredPitch();
    rig.input.gripper_done = true;
    rig.input.gripper_feedback_open = 1;
    rig.input.target_valid = rig.input.geometry_valid = rig.input.target_region_valid = true;
    rig.input.target_id = 1;
    rig.input.label = "ordinary_supply";
    rig.input.distance_m = .4f;
    rig.input.corridor.ordinary = 1;
    if (extra_kind == 0)
        ++rig.input.corridor.ordinary;
    if (extra_kind == 1)
        ++rig.input.corridor.core;
    if (extra_kind == 2)
        ++rig.input.corridor.injured;
    if (extra_kind == 3)
        ++rig.input.corridor.dangerous;
    if (extra_kind == 4)
        ++rig.input.corridor.unknown;
    rig.input.corridor_complete = rig.input.corridor_occlusion_free = true;
    for (int n = 0; n < 12; ++n) {
        const auto out = rig.tick();
        assert(rig.task.single_phase_initializing_);
        assert(!out.recovering && !out.single_phase_done);
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        assert(out.capture_split_runs == 0);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    }
}

void splitEvidenceAndLifetime() {
    TaskTuning tuning;
    tuning.require_multi_view = false;
    tuning.capture_frames = 1;
    for (bool later : {false, true}) {
        Rig rig(tuning);
        rig.task.first_ = later;
        rig.task.enter(PushState::VERIFY_CAPTURE, kStart, "split_evidence");
        rig.task.trip_context_.target_id_ = 7;
        rig.task.trip_context_.label_ = "ordinary_supply";
        rig.task.trip_context_.pending_.ordinary = later ? 2 : 1;
        rig.input.camera_pitch_cdeg = rig.task.desiredPitch();
        rig.input.hold_observable = rig.input.captured = true;
        rig.input.held.ordinary = 2;
        // Incomplete held evidence cannot start separation.
        rig.tick();
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        rig.input.held_complete = true;
        auto out = rig.tick();
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        if (later)
            assert(out.phase == TaskPhase::Transport && out.batch_size == 2);
        rig.input.reset = true;
        rig.tick();
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        assert(!rig.task.search_phase_.first_trip_split_.capture_retry);
    }
    Rig absent(tuning);
    absent.task.enter(PushState::VERIFY_CAPTURE, kStart, "no_ordinary");
    absent.task.trip_context_.label_ = "ordinary_supply";
    absent.input.camera_pitch_cdeg = absent.task.desiredPitch();
    absent.input.hold_observable = absent.input.captured = absent.input.held_complete = true;
    absent.input.held.core = 1;
    absent.input.held.dangerous = 1;
    absent.tick();
    assert(!absent.task.search_phase_.first_trip_split_.pending);
}

void initialCaptureNeedsFreshCompleteEvidence() {
    TaskTuning tuning;
    tuning.pixel_grasp_enabled = true;
    tuning.require_multi_view = false;
    tuning.corridor_confirm_frames = 1;
    for (int missing = 0; missing < 6; ++missing) {
        Rig rig(tuning);
        rig.task.setSinglePhase("capture");
        rig.tick();
        rig.input.camera_pitch_cdeg = tuning.near_pitch_cdeg;
        rig.input.gripper_done = true;
        rig.input.gripper_feedback_open = 1;
        rig.input.target_valid = true;
        rig.input.target_id = 1;
        rig.input.label = "ordinary_supply";
        rig.input.pixel_grasp_view = rig.input.pixel_grasp_near_view = true;
        rig.input.pixel_grasp_target_valid = true;
        rig.input.pixel_bearing_valid = true;
        rig.input.pixel_bearing = .247f;
        rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
        rig.input.corridor.ordinary = 2;
        rig.input.corridor_complete = rig.input.corridor_occlusion_free = true;
        if (missing == 0)
            rig.input.heading_valid = false;
        if (missing == 1)
            rig.input.pixel_bearing_valid = false;
        if (missing == 2)
            rig.input.pixel_grasp_sample_us = rig.input.now_us - tuning.frame_timeout_us;
        if (missing == 3)
            rig.input.camera_pitch_stable = false;
        if (missing == 4)
            rig.input.corridor_complete = false;
        if (missing == 5)
            rig.input.gripper_done = false;
        const auto out = rig.tick();
        assert(rig.task.single_phase_initializing_);
        assert(!rig.task.search_phase_.first_trip_split_.pending);
        assert(!out.recovering && out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    }
}
} // namespace
void a7ResetDiscardsEveryPhase() {
    for (const auto phase : {TaskPhase::Startup, TaskPhase::Search,
                             TaskPhase::Capture, TaskPhase::Transport}) {
        TaskTuning t;
        t.start_zone = 3;
        t.startup_advance_us = 2000000;
        t.pixel_grasp_enabled = true;
        Rig rig(t);
        rig.tick();
        rig.task.phase_ = phase;
        rig.task.trip_context_.target_id_ = 42;
        rig.task.trip_context_.label_ = "ordinary_supply";
        rig.task.trip_context_.trip_.ordinary = 2;
        rig.task.total_ = 4;
        rig.task.first_ = true;
        rig.task.startup_commanded_us_ = 1900000;
        rig.task.startup_alignment_error_count_ = 3;
        rig.task.startup_alignment_best_error_px_ = 8;
        rig.task.startup_sight_.label = "ordinary_supply";
        rig.task.fault_ = true;
        rig.task.failure_source_ = "previous_round";
        rig.task.frame_transaction_ = rig.task.wire_txn_ = 19;
        rig.task.wire_last_ = 0;
        rig.task.phase_waiting_ = true;
        rig.input.reset = true;
        auto out = rig.tick();
        assert(out.phase == TaskPhase::Startup && out.state == PushState::WAIT_START);
        assert(out.run_status == RunStatus::Waiting && !out.phase_waiting);
        assert(out.target_id == -1 && out.batch_size == 0 && out.delivered_total == 0);
        assert(!out.first_ordinary_delivered && out.failure_source.empty());
        assert(out.startup_commanded_us == 0 && out.capture_split_runs == 0);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        assert(rig.task.startup_sight_.label.empty());
        assert(rig.task.startup_alignment_error_count_ == 0);
        assert(out.motion.frame_transaction == 19);
        rig.input.reset = false;
        rig.input.gripper_done = true;
        rig.input.gripper_feedback_open = 0;
        out = rig.tick();
        assert(out.phase == TaskPhase::Startup && out.state == PushState::START_ADVANCE);
        out = rig.tick();
        assert(out.phase == TaskPhase::Startup && out.motion.vx_mps > 0);
        assert(out.startup_commanded_us < 200000);
    }
    std::cout << "PASS A7 reset from all four phases returns to fresh startup\n";
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--a7-reset") {
        a7ResetDiscardsEveryPhase();
        return 0;
    }
    a7ResetDiscardsEveryPhase();

    if (argc == 2 && std::string(argv[1]) == "--search-ui120-speed") {
        searchUi120StopsBeforeBottom(true);
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--search-ui120-handoff") {
        searchUi120StopsBeforeBottom(false);
        return 0;
    }
    searchUi120StopsBeforeBottom(true);
    searchUi120StopsBeforeBottom(false);
    if (argc == 2 && std::string(argv[1]) == "--startup-zone-bias") {
        startupForwardZoneBias(); return 0;
    }
    startupForwardZoneBias();
    if (argc == 2 && std::string(argv[1]) == "--search-unseen-zone") {
        searchFarUnseenZoneApproaches(); return 0;
    }
    searchFarUnseenZoneApproaches();
    if(argc==2 && std::string(argv[1])=="--startup-logged-jitter") {
        startupOrdinaryLoggedJitterUsesOneStoppedWindow();return 0;
    }
    startupOrdinaryLoggedJitterUsesOneStoppedWindow();
    if (argc == 2 && std::string(argv[1]) == "--search-region-wait") {
        searchFarRegionUnknownKeepsLock(); return 0;
    }
    searchFarRegionUnknownKeepsLock();
    if (argc == 2 && std::string(argv[1]) == "--startup-observation-handoff") {
        startupNearHandoffObservesBeforeEntry(); return 0;
    }
    startupNearHandoffObservesBeforeEntry();
    if (argc == 2 && std::string(argv[1]) == "--startup-region-handoff") {
        startupHandoffUsesRegionWithoutIdentity(); return 0;
    }
    startupHandoffUsesRegionWithoutIdentity();
    if (argc == 2 && std::string(argv[1]) == "--startup-settle-deadline") {
        startupOpenFinishesPendingSettle(); return 0;
    }
    startupOpenFinishesPendingSettle();
    if (argc == 2 && std::string(argv[1]) == "--startup-open-drift") {
        startupOpenDriftBeforeFirstAdvance();
        startupOpeningApproachToleranceBoundary();
        startupOpenJitterCannotRenewProgressForever();
        return 0;
    }
    startupOpenDriftBeforeFirstAdvance();
    startupOpeningApproachToleranceBoundary();
    startupOpenJitterCannotRenewProgressForever();

    if (argc == 2 && std::string(argv[1]) == "--startup-open-handoff") {
        startupOpenHandoffCommitsBeforeAlignmentTimeout();
        startupOpenHandoffRequiresFreshStoppedImages(); return 0;
    }
    startupOpenHandoffCommitsBeforeAlignmentTimeout();
    startupOpenHandoffRequiresFreshStoppedImages();
    if (argc == 2 && std::string(argv[1]) == "--ordinary-handoff") {
        startupNearHandoffReselectsAndReviewsCore();
        startupNearHandoffSelectsEligibleOrdinary();
        startupNearHandoffUsesOrdinaryRegion(); return 0;
    }
    startupNearHandoffReselectsAndReviewsCore();
    startupNearHandoffSelectsEligibleOrdinary();
    startupNearHandoffUsesOrdinaryRegion();
    if (argc == 2 && std::string(argv[1]) == "--first-target-progress") {
        startupOpenApproachRetainsFirstTarget(); return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--open-pulse-sent") {
        startupOpenFinePulseWaitsForSentMotion(); return 0;
    }
    startupOpenApproachRetainsFirstTarget();
    startupOpenFinePulseWaitsForSentMotion();
    if (argc == 2 && std::string(argv[1]) == "--startup-ordinary-fine") {
        startupOrdinaryFineCorrectionConverges(); return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--startup-open-progress") {
        startupOpeningStartsNewProgressWindow(); return 0;
    }
    startupOrdinaryFineCorrectionConverges();
    startupOpeningStartsNewProgressWindow();
    if (argc == 2 && std::string(argv[1]) == "--startup-ui120-delay") {
        startupTrackDelayedChassisConverges(); return 0;
    }
    startupTrackDelayedChassisConverges();
    if (argc == 2 && std::string(argv[1]) == "--startup-corner-forward") {
        startupCornerForwardDistance(); return 0;
    }
    directCaptureAfterFirstKeepsEvidenceChecks();
    if (argc == 2 && std::string(argv[1]) == "--capture-after-first") return 0;
    if (argc == 2 && std::string(argv[1]) == "--startup-far-handoff") {
        startupFarHandoffSurvivesJitterAndReplacement(); return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--startup-lock-gap") {
        startupTargetLockSurvivesDetectionGap(); return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--startup-lock-progress") {
        startupTargetLockTracksAlignmentProgress(); return 0;
    }
    startupTargetLockSurvivesDetectionGap();
    startupTargetLockTracksAlignmentProgress();
    if (argc == 2 && std::string(argv[1]) == "--startup-gripper-center") {
        startupOpensOnlyAfterCentering();
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--startup-gripper-reselect") {
        startupReselectionKeepsJawOpen();
        return 0;
    }
    startupOpensOnlyAfterCentering();
    startupReselectionKeepsJawOpen();
    {
        Rig rig;
        rig.input.run = false;
        rig.task.last_vx_ = .4f; // target speed before the UART ramp
        rig.task.last_wz_ = .2f;
        rig.input.sent_velocity_valid = true;
        rig.input.sent_vx_mps = .03f;
        rig.input.sent_wz_rps = .02f;
        rig.tick();
        assert(std::abs(rig.task.odo_ - .03f * .05f) < 1e-6f);
    }
    // Metric entry points reject illegal cargo without starting the removed cut.
    for (int extra_kind = 0; extra_kind < 5; ++extra_kind)
        metricInitialCaptureRejectsIllegalCorridor(extra_kind);
    initialCaptureNeedsFreshCompleteEvidence();
    for (bool single : {false, true})
        for (int extra_kind = 0; extra_kind < 5; ++extra_kind)
            metricOvercaptureDoesNotStartSplit(single, extra_kind);
    metricOvercaptureDoesNotStartSplit(true, 1, true);
    for (int extra_kind = 0; extra_kind < 5; ++extra_kind)
        metricOvercaptureDoesNotStartSplit(true, extra_kind, true, true);
    splitEvidenceAndLifetime();
    startupDisabledDoesNotRepeat();
    startupLowerCornersTrigger();
    startupCornerForwardDistance();
    startupVisualManeuver();
    startupVisualManeuver(true);
    startupAnyOrdinaryInsideFrame();
    startupFarHandoffSurvivesJitterAndReplacement();
    startupLockLossAndRecovery();
    startupTransitionsDecelerate();
    startupPitchTurnFeedbackAndClearance();
    startupReverseCalibratesWithPostTurnNoise();
    imuCalibrationKeepsOtherPhaseLimits();
    startupRebindKeepsAlignment();
    startupGapRampAndSafety();
    startupIntermittentDetectionProgress();
    startupConfirmationTimeoutAndSearch();
    startupNearPredictiveBrakeAndPulse();
    startupNearDelayedChassisConverges();
    transportReturnsToFarScan();
    prepareTransfersInventoryAndRushLimit();
    prepareTransfersInventoryAndRushLimit(true);
    checkpointStartupCaptureAndTransport();
    directPhasesSkipPrecedingPhases();
    pauseKeepsCargoAndRecoveryKeepsItsOwner();
    std::cout << "four-phase contracts passed\n";
}
