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
                assert(std::abs(out.motion.vx_mps - .1f) < 1e-6f && out.motion.wz_rps == 0);
                forward_us += kTick;
            }
            if (rig.task.startup_action_ == PushTask::StartupAction::Observe)
                break;
        }
        assert(forward_us == 2000000 && out.startup_commanded_us == 2000000);
        rig.input.startup_visible_classes = 1;
        rig.input.startup_scene_us = rig.input.now_us;
        out = rig.tick();
        assert(out.motion.wz_rps == sign * .3f);
        rig.input.startup_visible_classes = zone % 2 ? 12 : 15; // two or all four classes
        rig.input.startup_scene_us = rig.input.now_us - t.frame_timeout_us;
        out = rig.tick();
        assert(out.motion.wz_rps == sign * .3f);
        rig.input.startup_scene_us = rig.input.now_us + kTick;
        out = rig.tick();
        assert(out.reason == "startup_two_classes_seen" && !out.immediate_motion_stop);
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
            rig.input.pixel_view_handoff_ready = true;
            rig.input.target_center_px.y = 560;
            rig.input.target_valid = rig.input.target_center_px_valid = true;
            const bool half_turn = from == t.track_pitch_cdeg;
            rig.input.heading_valid = half_turn; // UI90 -> UI120 needs no IMU turn.
            bool turned = false, reversed = false, second_turn = false;
            float angle = 0, reverse_start = 0;
            for (int i = 0; i < 400; ++i) {
                angle += sign * out.motion.wz_rps * (kTick * 1e-6f);
                const bool turning =
                    rig.task.startup_action_ == PushTask::StartupAction::TurnBeforePitch ||
                    rig.task.startup_action_ == PushTask::StartupAction::StopBeforePitch;
                // The guide is absent throughout the two turns and reverse.
                rig.input.target_valid = rig.input.target_center_px_valid = !turning;
                rig.input.pixel_grasp_sample_us = turning ? 0 : rig.input.now_us + kTick;
                out = rig.motionTick(out);
                assert(out.motion.vx_mps <= 0);
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
                assert(turned && reversed && second_turn && angle >= 2.f * float(CV_PI));
                assert(angle < 2.f * float(CV_PI) + .15f);
                assert(reverse_start - rig.plant_position >= .09f);
                assert(reverse_start - rig.plant_position < .12f);
            } else {
                assert(!turned && !reversed && angle == 0);
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
        rig.input.pixel_view_handoff_ready = true;
        rig.input.target_center_px.y = 560;
        for (int i = 0; i < 30; ++i) {
            rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
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
        // An ordinary inside the frame finishes startup before the Y337 grasp line.
        rig.input.pixel_grasp_ready = false;
        rig.input.capture_scene_valid = true;
        rig.input.capture_scene_us = rig.input.now_us + kTick;
        rig.input.capture_class_regions_px["ordinary_supply"] = {342, 226, 689, 263};
        rig.input.capture_image_objects = {{8, "ordinary_supply", {440,240,80,80},true}};
        rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
        out = rig.tick();
        assert(out.immediate_motion_stop);
        if (stepping) {
            assert(out.phase_waiting && out.phase == TaskPhase::Startup);
            assert(out.next_phase == TaskPhase::Search);
            out = rig.tick();
        }
        assert(out.phase == TaskPhase::Search && out.state == PushState::APPROACH);
        assert(out.target_id == 8 && rig.task.frame_raised_ == 1);
        assert(rig.task.desiredPitch() == t.near_pitch_cdeg);
        // Startup's Search entry retains UI140 and the selected target, unlike normal SCAN.
        rig.input.target_grasp_range_valid = true;
        rig.input.sent_velocity_valid = true;
        rig.input.capture_scene_valid = rig.input.capture_path_valid = true;
        rig.input.layout_count_region_px = {342, 226, 689, 263};
        rig.input.capture_path_px = {342, 0, 689, 489};
        rig.input.capture_area_px = {342,226,689,263};
        rig.input.target_grasp_range_lateral_measured = true;
        rig.input.target_grasp_range_inside = rig.input.target_grasp_range_lateral = true;
        rig.input.target_center_px = {480, 337};
        rig.input.pixel_grasp_error_px = 167.5f;
        rig.input.capture_image_objects = {{8, "ordinary_supply", {440,297,80,80},true}};
        for (int i = 0; i < 30; ++i) {
            rig.input.capture_scene_us = rig.input.pixel_grasp_sample_us = rig.input.now_us + kTick;
            out = rig.tick();
            assert(out.motion.camera_pitch_cdeg == t.near_pitch_cdeg);
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            if (out.state == PushState::PREPARE) break;
        }
        assert(out.state == PushState::PREPARE && out.phase == TaskPhase::Search);
        rig.input.reset = true;
        out = rig.tick();
        assert(rig.task.startup_action_ == PushTask::StartupAction::Advance);
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
        assert(rig.task.phase_outcome_reason_ == "startup_complete" && out.immediate_motion_stop);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        assert(rig.task.trip_context_.target_id_ == 8 && rig.task.frame_raised_ == 1);
        if (mode == 1) {
            assert(out.phase_waiting && out.next_phase == TaskPhase::Search);
            out = rig.tick();
        }
        if (mode == 2) assert(out.single_phase_done && out.phase == TaskPhase::Startup);
        else {
            assert(out.phase == TaskPhase::Search && out.state == PushState::APPROACH);
            assert(rig.task.desiredPitch() == t.near_pitch_cdeg);
        }
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

// Exercise task -> sender ramp, including a lost lock, replacement and UI120 ordinary-search handoff.
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
            rig.input.pixel_grasp_error_px = i < centered_tick ? -100.f : 0.f;
            rig.input.target_center_px = {i < centered_tick ? 740.f : 640.f, approach_resumed ? 560.f : 400.f};
            rig.input.pixel_view_handoff_ready = approach_resumed;
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

void transportReturnsToFarScan() {
    TaskTuning tuning;
    tuning.startup_advance_us = 2000000;
    tuning.pixel_grasp_enabled = true;
    tuning.far_pitch_cdeg = -3000;
    tuning.track_pitch_cdeg = 0;
    tuning.near_pitch_cdeg = 2000;
    Rig rig(tuning);
    // Even a previous completed Startup cannot change the transport return path.
    rig.task.startup_action_ = PushTask::StartupAction::Approach;
    rig.task.enter(PushState::VERIFY_DELIVERY, kStart, "delivery_test");
    rig.task.phase_outcome_ = PhaseOutcome::SearchReady;
    rig.task.applyPhaseOutcome(rig.input.now_us);
    assert(rig.task.state_ == PushState::TURN_SCAN);
    assert(rig.task.desiredPitch() == tuning.far_pitch_cdeg);
    rig.task.actionProgress().turned = tuning.turn_min_rad;
    const auto out = rig.tick();
    assert(out.phase == TaskPhase::Search && out.state == PushState::SCAN);
    assert(rig.task.desiredPitch() == tuning.far_pitch_cdeg);
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
int main(int argc, char** argv) {
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
