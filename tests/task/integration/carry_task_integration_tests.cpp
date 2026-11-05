#include "rescue/hardware/motion_link.hpp"
#include "rescue/task/transport_rules.hpp"
#include "rescue/geometry/zone_dead_reckoning.hpp"
#include "rescue/geometry/planner.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <map>
#include <limits>
#include <optional>
#include <string>
#include <vector>
#define private public
#include "rescue/task/push_task.hpp"
#undef private

namespace {
using namespace rescue;
constexpr float kDt = .04f;

TaskTuning carryTuning(bool field_carry = true, bool assume_all_safe = false) {
    TaskTuning t;
    t.field_carry = field_carry;
    t.assume_all_safe = assume_all_safe;
    t.require_multi_view = false;
    t.carry_speed = .05f;
    t.max_wz = .55f;
    t.min_turn_wz = .4f;
    return t;
}

struct Rig {
    TaskTuning tuning = carryTuning();
    PushTask task{tuning};
    PushObservation input;
    MotionVelocityRamp ramp{MotionLinkConfig{}};
    MotionCommand sent;
    PushOutput output;
    float yaw, x = tuning.supply_half_x_m, y = -1.2f, actual_rate = 0;
    uint64_t frame = 0;

    explicit Rig(float initial_yaw, bool field_carry = true, bool assume_all_safe = false)
        : tuning(carryTuning(field_carry, assume_all_safe)), task(tuning), yaw(initial_yaw) {
        input.now_us = 1000000;
        input.run = input.safety_ok = input.path_safe = input.retreat_safe = true;
        input.opponent_zone_clear = input.camera_pitch_stable = input.gripper_done = true;
        input.gripper_feedback_open = 0;
        input.heading_valid = input.sent_velocity_valid = true;
        task.trip_context_.trip_.ordinary = 1;
        task.frame_raised_ = task.wire_last_ = 0;
        task.frame_transaction_ = task.wire_txn_ = 41;
        task.enter(PushState::CARRY, input.now_us, "carry_steering_regression");
        task.transport_phase_.carry_us_ = input.now_us;
        task.last_us_ = input.now_us;
        task.pitch_cmd_ = input.camera_pitch_cdeg = tuning.far_pitch_cdeg;
        task.pitch_wait_us_ = input.now_us;
        ramp.accept(sent, input.now_us);
    }

    void sightOwnZone(bool visible = true) {
        input.zone_sight_valid = visible;
        input.zone_sight_enemy = false;
        const float dx = tuning.supply_half_x_m - x, dy = -y;
        const float body_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
        const float body_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
        input.zone_sight_bearing = std::atan2(-body_x, body_y);
    }

    void tick(float yaw_response, bool visible = true, float visual_noise = 0, int points = 6) {
        // Deterministic plant: the field log shows a faster yaw response than sent_wz.
        // Exercise both nominal and fast responses with 120 ms mechanical settling, using
        // the real sender ramp. This is a regression envelope, not a chassis calibration.
        actual_rate += (yaw_response * sent.wz_rps - actual_rate) * (kDt / .12f);
        yaw += actual_rate * kDt;
        x -= sent.vx_mps * std::sin(yaw) * kDt / 4;
        y += sent.vx_mps * std::cos(yaw) * kDt / 4;
        input.now_us += 40000;
        input.heading_rad = yaw;
        input.sent_vx_mps = sent.vx_mps;
        input.sent_wz_rps = sent.wz_rps;
        ZoneEstimate z;
        z.valid = true;
        z.frame_id = ++frame;
        z.timestamp_us = z.observed_us = input.now_us;
        z.yaw_body_rad = -yaw + visual_noise;
        z.origin_body_m = {-std::cos(yaw) * x - std::sin(yaw) * y,
                           std::sin(yaw) * x - std::cos(yaw) * y};
        input.vref_valid = visible;
        input.vref_points = visible ? points : 0;
        input.vref_zone = z;
        output = task.update(input);
        assert(output.state == PushState::CARRY);
        assert(output.motion.frame_transaction == 41);
        sent = ramp.next(output.motion, input.now_us);
        ramp.accept(sent, input.now_us);
    }
};

void convergesWithoutRepeatedReversals(float initial_yaw, float response, bool intermittent) {
    Rig rig(initial_yaw);
    int last_sign = 0, reversals = 0;
    float tail_rate = 0;
    for (int i = 0; i < 500; ++i) {
        // A 600 ms visual gap exercises own-zone heading memory, never pose-axis fallback.
        const bool visible = !intermittent || i % 50 < 35;
        rig.sightOwnZone(visible);
        rig.tick(response, visible, intermittent ? .005f * std::sin(float(i)) : 0);
        const int sign = rig.sent.wz_rps > .02f ? 1 : rig.sent.wz_rps < -.02f ? -1 : 0;
        if (sign && last_sign && sign != last_sign)
            ++reversals;
        if (sign)
            last_sign = sign;
        if (i >= 450)
            tail_rate = std::max(tail_rate, std::abs(rig.sent.wz_rps));
    }
    std::cout << "carry yaw=" << initial_yaw << " response=" << response
              << " intermittent=" << intermittent << " reversals=" << reversals
              << " tail_rate=" << tail_rate << " forward=" << rig.y + 1.2f << std::endl;
    assert(reversals <= 1);
    assert(tail_rate <= .02f);
    assert(std::abs(rig.yaw) < .08f);
    assert(rig.y > -1.1f); // eliminating sway must still carry the cargo forward
}

void smallErrorsDoNotRequestTheTurnFloor() {
    for (float yaw : {-.08f, .08f}) {
        Rig rig(yaw);
        rig.sightOwnZone();
        rig.tick(1);
        assert(rig.output.motion.vx_mps > 0);
        assert(std::abs(rig.output.motion.wz_rps) < .05f);
        assert(rig.output.motion.wz_rps * yaw < 0);
    }
}


// Actual PushTask lifecycle with device-timed IMU batches. The plant's velocity
// changes generate acceleration; command time/travel cannot complete the deposit.
struct DepositRig {
    Rig rig{0};
    uint64_t sequence = 0;
    float velocity = 0;
    float position = 0, gyro_noise = 0;
    PushOutput out;
    DepositRig() {
        rig.task.t_.pixel_grasp_enabled = true;
        rig.task.beginCarryPlacement(rig.input.now_us, "deposit_test");
    }
    PushOutput tick(bool moving = true, bool samples = true, float drift = 0, float physical_scale = 1) {
        auto& in = rig.input;
        in.imu_translation_samples.clear();
        const float sent_target = moving ? out.motion.vx_mps : 0;
        const float target = sent_target * physical_scale;
        const float accel = (target - velocity) / .04f;
        for (int i = 0; i < 4; ++i) {
            in.now_us += 10000;
            const float next = velocity + accel * .01f;
            position += .5f * (velocity + next) * .01f;
            velocity = next;
            if (samples) {
                ImuTranslationSample sample;
                sample.sequence = ++sequence;
                sample.received_us = in.now_us;
                sample.device_time_ms = in.now_us / 1000;
                sample.valid = true;
                sample.acceleration_mps2 = {accel + .01f + drift, 0, 9.8f};
                sample.angular_velocity_rps = {0,0,sample.sequence%2 ? gyro_noise : -gyro_noise};
                sample.quaternion_wxyz = {1, 0, 0, 0};
                in.imu_translation_samples.push_back(sample);
            }
        }
        in.sent_vx_mps = sent_target;
        in.sent_wz_rps = 0;
        in.camera_pitch_cdeg = rig.task.desiredPitch();
        out = rig.task.update(in);
        return out;
    }
};

void floorPlacementUsesColourInsteadOfFifteenCentimetres() {
    DepositRig plant;
    auto& rig = plant.rig;
    rig.task.t_.pushboard_floor_enabled = true;
    auto step = [&](bool valid, bool inside, bool new_image = true) {
        auto& f = rig.input.pushboard_floor;
        f.valid = valid; f.inside = inside;
        if (new_image) f.sample_us = rig.input.now_us + 40000;
        return plant.tick(true, false); // colour placement needs no translation integration
    };
    for (int i = 0; i < 140; ++i) {
        auto out = step(true, false);
        assert(out.state == PushState::CARRY);
        assert(rig.task.desiredPitch() == rig.task.t_.near_pitch_cdeg);
        assert(out.motion.gripper_offset == 20);
    }
    assert(plant.position > .15f && plant.out.motion.vx_mps > 0);
    assert(plant.out.pushboard_floor_active);
    auto out = step(true,true);
    assert(out.motion.vx_mps == 0 && out.immediate_motion_stop);
    // A repeated image cannot accumulate confirmation time/frames.
    for (int i=0; i<4; ++i) {
        out=step(true,true,false);
        assert(out.state == PushState::CARRY);
    }
    // Lost evidence holds closed and resets confirmation.
    out=step(false,false);
    assert(out.motion.vx_mps == 0 && out.motion.gripper_offset == 20);
    assert(rig.task.transport_phase_.floor_frames_ == 0);
    out=step(true,false);
    assert(out.motion.vx_mps > 0 && out.state == PushState::CARRY);
    for (int i=0;i<15;++i) {
        out=step(true,true);
        assert(out.motion.vx_mps == 0 && out.immediate_motion_stop);
        if(out.state == PushState::RAISE_RELEASE) {
            assert(out.reason == "deposit_floor_complete");
            assert(out.motion.gripper_offset == 20);
            assert(rig.task.desiredPitch() == rig.task.t_.far_pitch_cdeg);
            return;
        }
    }
    assert(false && "floor confirmation did not finish");
}

void floorPlacementCannotReleaseOnMissingOrStaleEvidence() {
    for (bool stale : {false,true}) {
        DepositRig plant; auto& rig=plant.rig;
        rig.task.t_.pushboard_floor_enabled=true;
        for(int i=0;i<30;++i) {
            auto& f=rig.input.pushboard_floor;
            f.valid=stale;f.inside=true;f.sample_us=rig.input.now_us-300000;
            auto out=plant.tick(true,false);
            assert(out.state==PushState::CARRY && out.motion.vx_mps==0);
            assert(out.motion.gripper_offset==20);
        }
    }
}

void depositKeepsMappedTransportView() {
    DepositRig plant;
    bool advanced = false;
    for (int i = 0; i < 400; ++i) {
        const auto pitch = plant.rig.task.desiredPitch();
        // Reproduce the live mapping gate: UI140/internal2000 is uncalibrated.
        plant.rig.input.path_safe = pitch == plant.rig.task.t_.far_pitch_cdeg;
        auto out = plant.tick();
        assert(out.state != PushState::SAFE_STOP);
        assert(pitch == plant.rig.task.t_.far_pitch_cdeg);
        advanced |= out.motion.vx_mps > 0;
        if (out.state == PushState::RAISE_RELEASE) {
            assert(advanced && plant.position >= .145f);
            assert(plant.rig.task.desiredPitch() == pitch);
            return;
        }
    }
    assert(false && "mapped deposit did not reach release");
}

void depositUsesImuAndStopsBeforeOpening() {
    DepositRig plant;
    bool advanced = false, braking = false;
    for (int i = 0; i < 400; ++i) {
        auto out = plant.tick();
        if (out.state == PushState::RAISE_RELEASE) {
            assert(advanced && braking);
            assert(plant.rig.task.transport_phase_.deposit_imu_.distance() >= .15f);
            assert(plant.position >= .145f && plant.position < .165f);
            assert(out.reason == "deposit_imu_complete");
            assert(out.motion.gripper_offset == 20);
            out = plant.tick();
            assert(out.motion.gripper_offset == 0);
            return;
        }
        assert(out.state == PushState::CARRY);
        assert(out.motion.gripper_offset == 20);
        if (out.reason == "deposit_stop_imu_calibrate")
            assert(out.motion.vx_mps == 0);
        if (out.motion.vx_mps > 0) {
            advanced = true;
            assert(out.motion.vx_mps <= .05f);
        }
        if (out.reason == "deposit_distance_reached_stop") {
            braking = true;
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            assert(out.immediate_motion_stop);
        }
    }
    assert(false && "IMU deposit did not complete");
}

// 17:28 field failure: no forward permission and zero UART velocity, but
// a small post-calibration acceleration bias accumulated until the jaws opened.
void blockedStationaryImuDriftCannotRelease() {
    DepositRig plant;
    for (int i = 0; i < 30 && plant.out.motion.vx_mps == 0; ++i)
        plant.tick(false);
    assert(plant.out.motion.vx_mps > 0); // calibration completed, still physically stationary
    plant.rig.input.directional_clearance_valid = true;
    plant.rig.input.turn_safe = plant.rig.input.arc_safe = false;
    plant.rig.input.path_safe = false;
    for (int i = 0; i < 200; ++i) {
        const auto out = plant.tick(false, true, .006f);
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        assert(out.state != PushState::RAISE_RELEASE && out.state != PushState::BACK_OUT);
        assert(out.motion.gripper_offset == 20);
    }
}

void zeroSentVelocityImuDriftCannotRelease() {
    DepositRig plant;
    for (int i = 0; i < 30 && plant.out.motion.vx_mps == 0; ++i)
        plant.tick(false);
    assert(plant.out.motion.vx_mps > 0);
    // Motion is permitted and requested, but the UART still reports zero velocity.
    for (int i = 0; i < 200; ++i) {
        const auto out = plant.tick(false, true, .006f);
        assert(out.state == PushState::CARRY);
        assert(out.motion.gripper_offset == 20);
    }
    assert(std::abs(plant.rig.task.transport_phase_.deposit_imu_.distance()) < .001f);
}

void depositImuGapStopsAlreadyRequestedMotion() {
    DepositRig plant;
    for (int i = 0; i < 30 && plant.out.motion.vx_mps == 0; ++i)
        plant.tick();
    assert(plant.out.motion.vx_mps > 0);
    plant.tick(); // positive sender feedback and actual IMU movement
    for (int i = 0; i < 8; ++i) {
        const auto out = plant.tick(true, false);
        if (out.state == PushState::SAFE_STOP) {
            assert(out.reason == "deposit_imu_gap");
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            assert(out.motion.gripper_offset == 20);
            assert(out.immediate_motion_stop);
            return;
        }
    }
    assert(false && "missing IMU did not stop forward motion");
}


void depositWaitsForSentStopAndImuCalibration() {
    DepositRig plant;
    plant.rig.input.sent_velocity_valid = false;
    for (int i = 0; i < 15; ++i) {
        const auto out = plant.tick();
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
        assert(!plant.rig.task.transport_phase_.deposit_imu_.ready());
    }
    plant.rig.input.sent_velocity_valid = true;
    for (int i = 0; i < 5; ++i) {
        const auto out = plant.tick();
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    }
    for (int i = 0; i < 8; ++i) plant.tick();
    assert(plant.out.motion.vx_mps > 0);
}

void depositCannotUseCommandTravelOrMissingImu() {
    DepositRig plant;
    for (int i = 0; i < 80; ++i) {
        const auto out = plant.tick(false);
        assert(out.state == PushState::CARRY);
        assert(out.motion.gripper_offset == 20);
    }
    assert(plant.out.motion.vx_mps > 0);
    assert(std::abs(plant.rig.task.transport_phase_.deposit_imu_.distance()) < .001f);
    plant.rig.input.path_safe = false;
    auto out = plant.tick(false);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
    for (int i = 0; i < 8; ++i) out = plant.tick(false, false);
    assert(out.state == PushState::SAFE_STOP);
    assert(out.motion.gripper_offset == 20);
    assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
}

void nearEdgeLargeTurnRetainsInPlaceAlignment() {
    Rig rig(0);
    rig.x -= .15f;
    rig.y = -.1f;
    rig.input.zone_sight_valid = true;
    rig.input.zone_sight_bearing = .2f; // Conflicts with the required near-edge alignment.
    rig.tick(1);
    assert(rig.output.motion.vx_mps == 0);
    assert(rig.output.motion.wz_rps < 0);
    assert(std::abs(rig.output.motion.wz_rps) >= rig.tuning.min_turn_wz);
}

void forwardClearanceStillStopsTheCarry() {
    Rig rig(-.2f);
    rig.sightOwnZone();
    rig.input.path_safe = false;
    rig.tick(1);
    assert(rig.output.motion.vx_mps == 0 && rig.output.motion.wz_rps == 0);
}
void enemyZonesDoNotStartAManeuver() {
    for (const auto& mode : {std::pair{true, false}, std::pair{false, false},
                            std::pair{false, true}}) {
        Rig rig(0, mode.first, mode.second);
        rig.input.zone_sight_valid = rig.input.zone_sight_enemy = true;
        rig.input.enemy_zone_seen = true;
        rig.input.zone_sight_bearing = .3f;
        for (int i = 0; i < 8; ++i) {
            rig.tick(1, false);
            assert(rig.output.reason.find("enemy_zone_") == std::string::npos);
            assert(rig.output.motion.vx_mps == 0);
        }
        // A fresh own-zone box takes effect immediately, even after repeated enemy sightings.
        rig.input.zone_sight_enemy = rig.input.enemy_zone_seen = false;
        rig.input.zone_sight_bearing = 0;
        rig.tick(1, false);
        assert(rig.output.reason == "carry_drive_to_sighted_zone");
        assert(rig.output.motion.vx_mps > 0);
        assert(rig.output.motion.wz_rps == 0);
    }
}

// Log 20261031-151624: nearby enemy-only frames erased a fresh own-zone
// heading, reversing correction into search or the start-line fallback.
void enemySightCannotEraseOwnCarryHeading() {
    int cases = 0;
    for (const auto& mode : {std::pair{true, false}, std::pair{false, false},
                            std::pair{false, true}}) {
        for (float bearing : {-.5f, 0.f, .5f}) {
            for (bool memory_leg_started : {false, true}) {
                Rig rig(0, mode.first, mode.second);
                rig.tick(0, false); // Record the actual starting heading before seeing our zone.
                rig.yaw = bearing > 0 ? -1.8f : 1.8f;
                rig.input.zone_sight_valid = true;
                rig.input.zone_sight_bearing = bearing;
                rig.tick(0, false);
                assert(rig.task.zone_mem_);
                const auto own_seen_us = rig.task.zone_mem_us_;
                const float own_yaw = rig.task.zone_mem_yaw_;
                if (memory_leg_started) {
                    rig.input.zone_sight_valid = false;
                    rig.tick(0, false);
                }
                for (int i = 0; i < 70; ++i) {
                    // Alternate enemy detections at nearly the same bearing and empty frames.
                    // No new own sight: enemy evidence must not erase or refresh its memory.
                    rig.input.zone_sight_valid = i % 3 == 0;
                    rig.input.zone_sight_enemy = rig.input.enemy_zone_seen = true;
                    rig.input.zone_sight_bearing = bearing + .02f;
                    rig.tick(0, false);
                    assert(rig.task.zone_mem_);
                    assert(rig.task.zone_mem_us_ == own_seen_us);
                    assert(std::abs(wrapAngle(rig.task.zone_mem_yaw_ - own_yaw)) < 1e-6f);
                    assert(rig.output.reason != "searching_own_zone");
                    assert(rig.output.reason != "carry_drive_start_line");
                    if (bearing == 0)
                        assert(rig.output.motion.wz_rps == 0);
                    else
                        assert(rig.output.motion.wz_rps * bearing >= 0);
                    if (i >= 20) {
                        if (bearing == 0)
                            assert(rig.sent.wz_rps == 0);
                        else
                            assert(rig.sent.wz_rps * bearing >= 0);
                    }
                }
                rig.input.path_safe = false;
                rig.tick(0, false);
                assert(rig.output.motion.vx_mps == 0 && rig.output.motion.wz_rps == 0);
                ++cases;
            }
        }
    }
    std::cout << "own/enemy carry heading memory cases=" << cases << " OK\n";
}

// Log 20261031-132602: blue box moves left while a retained pose still steers right.
// Cover both a newly accepted conflicting pose and two-point fixes rejected by PushTask.
void ownZoneSightOverridesConflictingFarAnchor() {
    for (const auto& mode : {std::pair{true, false}, std::pair{false, false},
                            std::pair{false, true}}) {
        for (float bearing : {-.2f, 0.f, .2f}) {
            for (bool fresh_pose : {false, true}) {
                Rig rig(bearing < 0 ? -.2f : .2f, mode.first, mode.second);
                rig.tick(1); // Establish a valid anchor before own-color detection appears.
                if (!fresh_pose)
                    for (int i = 0; i < 15; ++i)
                        rig.tick(1, true, 0, 2);
                rig.input.zone_sight_valid = true;
                rig.input.zone_sight_bearing = bearing;
                for (int i = 0; i < 30; ++i) {
                    rig.tick(1, true, 0, fresh_pose ? 6 : 2);
                    assert(rig.output.reason == "carry_drive_to_sighted_zone");
                    assert(rig.output.motion.vx_mps > 0);
                    if (bearing == 0)
                        assert(rig.output.motion.wz_rps == 0);
                    else
                        assert(rig.output.motion.wz_rps * bearing > 0);
                }
                if (bearing == 0)
                    assert(rig.sent.wz_rps == 0);
                else
                    assert(rig.sent.wz_rps * bearing > 0);
                rig.input.path_safe = false;
                rig.tick(1, true, 0, fresh_pose ? 6 : 2);
                assert(rig.output.motion.vx_mps == 0 && rig.output.motion.wz_rps == 0);
            }
        }
    }
}

// Log 20261031-143024: intermittent own-zone sight requests right while
// the retained far-zone axis requests left. The old axis must never regain motion.
void ownZoneDropoutsCannotRestoreFarAnchorSteering() {
    for (const auto& mode : {std::pair{true, false}, std::pair{false, false},
                            std::pair{false, true}}) {
        for (float bearing : {-.6f, 0.f, .6f}) {
            Rig rig(bearing < 0 ? -.2f : .2f, mode.first, mode.second);
            rig.tick(0); // Seed the conflicting pose; no own-zone sight has been seen.
            assert(rig.output.reason == "searching_own_zone");
            assert(rig.output.motion.vx_mps == 0);
            rig.sent.wz_rps = bearing < 0 ? .15f : -.15f;
            rig.ramp.accept(rig.sent, rig.input.now_us);
            for (int i = 0; i < 90; ++i) {
                // Single sight frames followed by gaps; finish with a 2.4 s dropout.
                rig.input.zone_sight_valid = i < 30 && i % 3 == 0;
                rig.input.zone_sight_bearing = bearing;
                rig.tick(0, false);
                assert(rig.output.reason != "assumed_visual_carry");
                assert(rig.output.reason != "assumed_visual_dead_reckoning");
                if (bearing == 0)
                    assert(rig.output.motion.wz_rps == 0);
                else {
                    // Strict mode may stop for its scheduled holding inspection.
                    // A stationary tick is allowed; steering against own sight is not.
                    assert(rig.output.motion.wz_rps * bearing >= 0);
                    if (rig.input.zone_sight_valid)
                        assert(rig.output.motion.wz_rps * bearing > 0);
                }
                if (i >= 30) {
                    if (bearing == 0)
                        assert(rig.sent.wz_rps == 0);
                    else
                        assert(rig.sent.wz_rps * bearing >= 0);
                }
            }
            rig.input.path_safe = false;
            rig.tick(0, false);
            assert(rig.output.motion.vx_mps == 0 && rig.output.motion.wz_rps == 0);
        }
    }
}

void ownSightDoesNotReplacePoseArrival() {
    for (const auto& mode : {std::pair{true, false}, std::pair{false, true}}) {
        Rig rig(0, mode.first, mode.second);
        rig.input.zone_sight_valid = true;
        rig.input.zone_sight_bearing = .2f;
        rig.input.now_us += 40000;
        rig.input.heading_rad = 0;
        rig.input.vref_valid = true;
        rig.input.vref_points = 6;
        auto& z = rig.input.vref_zone;
        z.valid = true;
        z.frame_id = 1;
        z.timestamp_us = z.observed_us = rig.input.now_us;
        z.yaw_body_rad = 0;
        z.origin_body_m = {-rig.x, -.06f};
        const auto out = rig.task.update(rig.input);
        assert(out.state == (rig.tuning.pixel_grasp_enabled ? PushState::CARRY
                                                           : PushState::RAISE_RELEASE));
        assert(out.reason == "assumed_drop_point_visual");
    }
}

void ownZoneBoxesDriveWithoutPoseKeypoints() {
    for (const auto& mode : {std::pair{true, false}, std::pair{false, false},
                            std::pair{false, true}}) {
        for (float bearing : {-.2f, 0.f, .2f}) {
            Rig rig(0, mode.first, mode.second);
            rig.input.zone_sight_valid = true;
            rig.input.zone_sight_bearing = bearing;
            rig.tick(1, false);
            assert(rig.output.reason == "carry_drive_to_sighted_zone");
            assert(rig.output.motion.vx_mps > 0);
            if (bearing != 0)
                assert(rig.output.motion.wz_rps * bearing > 0);
            rig.input.path_safe = false;
            rig.tick(1, false);
            assert(rig.output.motion.vx_mps == 0);
        }
    }
}
void carryPlacesInSelectedHalfBeforeOpening() {
    for (bool field : {false}) {
        for (int kind : {0, 1, 2}) {
            Rig rig(0, field);
            rig.tuning.pixel_grasp_enabled = rig.task.t_.pixel_grasp_enabled = !field;
            rig.task.clearTrip();
            rig.task.first_ = true;
            Inventory cargo;
            if (kind == 0) cargo.ordinary = 1;
            if (kind == 1) cargo.core = 1;
            if (kind == 2) cargo.injured = 1;
            rig.task.trip_context_.trip_ = cargo;
            rig.task.enter(PushState::CARRY, rig.input.now_us, "selected_half_trip");
            rig.task.transport_phase_.carry_us_ = rig.input.now_us;
            rig.task.transport_phase_.hold_seen_us_ = rig.input.now_us;
            auto& input = rig.input;
            input.zone_valid = input.zone_own = input.zone_identity_verified = true;
            input.zone_counts_valid = input.zone_inventory_complete = true;
            input.zone_supply_count = 4;
            input.zone_injured_count = 2;
            input.drop_plan_valid = input.carry_plan_valid = true;
            input.hold_observable = input.captured = input.held_complete = true;
            input.held = cargo;
            const float half = kind == 2 ? rig.tuning.injured_half_x_m
                                         : rig.tuning.supply_half_x_m;
            input.drop_centre_zone = {half, rig.tuning.deposit_y_m};
            input.carry_waypoint_body = {0, 0};
            auto& z = input.zone_estimate;
            z.valid = true;
            z.source = ZoneEstimate::Source::MULTI_POINT;
            z.geometry_id = "selected-half-test";
            z.zone_label = "red_safe_zone";
            z.inlier_ids = {0, 2, 3, 5};
            z.residual_m = .001f;
            z.position_sigma_m = .002f;
            z.yaw_sigma_rad = .01f;
            z.origin_body_m = {-half, rig.tuning.gate_clearance_m + rig.task.holdCenter()};
            const auto tick = [&] {
                input.now_us += 40000;
                ++z.frame_id;
                z.timestamp_us = z.observed_us = input.now_us;
                input.navigation_timestamp_us = input.now_us;
                input.camera_pitch_cdeg = rig.task.desiredPitch();
                return rig.task.update(input);
            };
            PushOutput out;
            for (int i = 0; i < 20 && !rig.task.transport_phase_.placing_; ++i) {
                out = tick();
                assert(out.state == PushState::CARRY);
                assert(out.motion.gripper_offset == 20);
                assert(out.cargo_injured == (kind == 2));
            }
            assert(rig.task.transport_phase_.placing_);
            assert(out.transport_action == TransportAction::Carry && out.navigation_release);
            assert(rig.task.transport_phase_.baseline_ == (kind == 2 ? 2 : 4));
            assert(rig.task.transport_phase_.baseline_other_ == (kind == 2 ? 4 : 2));
            assert(rig.task.halfX() == half);
            out = tick();
            assert(out.state == PushState::CARRY && out.motion.vx_mps > 0);
            assert(out.motion.gripper_offset == 20);
            z.origin_body_m.y = rig.task.holdCenter() - rig.tuning.deposit_y_m - .01f;
            out = tick();
            assert(out.state == PushState::RAISE_RELEASE);
            assert(out.transport_action == TransportAction::Release);
            assert(out.reason == "enclosed_load_at_drop");
            assert(out.motion.gripper_offset == 20); // opening begins on the following tick
            out = tick();
            assert(out.motion.gripper_offset == 0);
        }
    }
}

// Public transport lifecycle: FAR direction control remains available until the
// own-zone image fills the view, independently of pose and centre-Y handoffs.
struct JawRig {
    Rig rig{0};
    float gap = .5f, yaw = .2f;
    bool region_inside = false, region_found = true, repeat_region_frame = false;
    uint64_t region_stamp = 0;
    bool pose = true, fresh = true, heading = true, view = true;
    bool half_entrance = false, stale_half = false;
    bool ramped_dead_zone = false;
    float angular_response = 1;
    MotionCommand wire;
    bool sight = true, enemy = false, stale_sight = false, repeat_sight_frame = false;
    uint64_t sight_stamp = 0;
    cv::Rect sight_box{136,430,1124,260};
    float bearing = 0, metric_yaw_bias = 0;
    float imu_offset = 0, capture_yaw_offset = 0;
    bool repeat_pose_frame = false, capture_heading = true;
    uint64_t pose_stamp = 0;
    float forced_vx = 0;
    bool force_sent = false;
    bool view_evidence = true, inside = true;
    bool supply_visible = true, injured_visible = true;
    float robot_x = -.15f, image_error = 0;
    float left_margin = -1, right_margin = -1;
    float edge_override = std::numeric_limits<float>::quiet_NaN();
    PushOutput out;
    JawRig() {
        auto& t = rig.task.t_;
        t.pixel_grasp_enabled = true;
        t.far_pitch_cdeg = -3000; t.track_pitch_cdeg = 0; t.near_pitch_cdeg = 2000;
        rig.task.transport_phase_.approach_pitch_ = rig.task.pitch_cmd_ = -3000;
    }
    PushOutput step() {
        auto& in = rig.input;
        in.now_us += 40000;
        const float rate = ramped_dead_zone
            ? (std::abs(wire.vx_mps) > .001f || std::abs(wire.wz_rps) > .25f ? wire.wz_rps : 0.f)
            : out.motion.wz_rps;
        yaw = wrapAngle(yaw + angular_response * rate * .04f);
        in.heading_rad = wrapAngle(yaw+imu_offset); in.heading_valid = heading;
        if (!repeat_pose_frame || !pose_stamp) pose_stamp=in.now_us;
        in.zone_observed_heading_valid = heading && capture_heading;
        in.zone_observed_heading_rad = wrapAngle(yaw+imu_offset-capture_yaw_offset);
        in.zone_observed_heading_us = pose_stamp;
        in.camera_pitch_cdeg = rig.task.desiredPitch(); in.camera_pitch_stable = view;
        in.sent_vx_mps = force_sent ? forced_vx : out.motion.vx_mps;
        in.sent_wz_rps = ramped_dead_zone ? wire.wz_rps : out.motion.wz_rps;
        auto& z = in.zone_estimate;
        z.valid = pose; z.source = ZoneEstimate::Source::MULTI_POINT;
        z.frame_id++; z.timestamp_us = z.observed_us = pose_stamp;
        z.zone_label = "blue_safe_zone"; z.geometry_id = "test-entry";
        z.inlier_ids = {0,1,2}; z.residual_m = z.position_sigma_m = z.yaw_sigma_rad = .001f;
        z.yaw_body_rad = -yaw+metric_yaw_bias+capture_yaw_offset;
        const float captured_yaw=yaw-capture_yaw_offset;
        z.origin_body_m = {-std::cos(captured_yaw)*robot_x+std::sin(captured_yaw)*gap,
                           std::sin(captured_yaw)*robot_x+std::cos(captured_yaw)*gap};
        in.subzone_view_us = stale_half ? in.now_us-1000000 : in.now_us;
        in.supply_view = {view_evidence && supply_visible,inside,-.15f,image_error,image_error*(gap+.15f)/.625f};
        in.injured_view = {view_evidence && injured_visible,inside,.15f,image_error,image_error*(gap+.15f)/.625f};
        // Pitched-camera projection: image edge slope follows measured chassis yaw.
        in.supply_view.edge_slope=in.injured_view.edge_slope=.5f*std::tan(captured_yaw);
        if(std::isfinite(edge_override)) in.supply_view.edge_slope=in.injured_view.edge_slope=edge_override;
        for (auto* v : {&in.supply_view, &in.injured_view}) {
            v->entrance_valid = (pose || half_entrance) && v->valid;
            v->entrance_origin_body_m = z.origin_body_m;
            v->entrance_yaw_rad = half_entrance ? -captured_yaw : z.yaw_body_rad;
            v->left_margin_fraction=left_margin;v->right_margin_fraction=right_margin;
        }
        in.zone_own = in.zone_identity_verified = true;
        in.zone_valid = pose;
        in.zone_sight_valid = sight; in.zone_sight_enemy = enemy;
        in.zone_sight_bearing = bearing;
        in.zone_sight_box = sight_box; in.zone_sight_image_size = {1280,720};
        if (!repeat_sight_frame || !sight_stamp) sight_stamp=in.now_us;
        in.zone_sight_sample_us = stale_sight ? in.now_us - 1000000 : sight_stamp;
        if (!repeat_region_frame || !region_stamp) region_stamp=in.now_us;
        in.transport_floor = {};
        in.transport_floor.sample_us=fresh ? region_stamp : in.now_us-1000000;
        in.transport_floor.valid=true;
        in.transport_floor.own_fraction=.1f; // Low area must not veto a complete region.
        in.transport_floor.region_valid=region_found;
        in.transport_floor.inside=region_inside;
        out = rig.task.update(in);
        if (ramped_dead_zone) {
            wire = rig.ramp.next(out.motion, in.now_us, out.immediate_motion_stop);
            rig.ramp.accept(wire, in.now_us);
        }
        return out;
    }
    PushOutput waitForHalfSearch() {
        const auto start=rig.input.now_us;
        for(int i=0;i<100;++i) {
            const auto result=step();
            assert(result.state==PushState::CARRY && result.motion.vx_mps==0);
            if(result.reason=="carry_half_reacquire_turn" && result.motion.wz_rps!=0) {
                assert(rig.input.now_us-start>=250000);
                return result;
            }
        }
        assert(false && "near-entry landmark loss did not start a stationary search");
        return out;
    }
    PushOutput waitForDirectionalRecovery() {
        using Recovery = TransportPhaseData::JawSequence::Recovery;
        const auto start = rig.input.now_us;
        for (int i = 0; i < 90; ++i) {
            auto result = step();
            if (rig.task.transport_phase_.jaw_sequence_.recovery == Recovery::Turn) {
                assert(rig.input.now_us-start >= 1500000);
                return result;
            }
            assert(result.state == PushState::CARRY);
            assert(result.motion.vx_mps == 0 && result.motion.wz_rps == 0);
            assert(result.motion.gripper_offset == 20);
        }
        assert(false && "persistent landmark loss did not start recovery");
        return out;
    }
    void reachColourLeg() {
        gap = .20f; yaw=.65f;
        bool aligned = false;
        for (int i=0;i<180;++i) {
            step();
            assert(out.state == PushState::CARRY && out.motion.gripper_offset == 20);
            aligned |= out.reason == "carry_entry_image_align" || out.reason == "carry_entry_pose_imu_align" ||
                       out.reason == "carry_entry_forward_correction";
            if (out.motion.vx_mps > 0 && rig.task.desiredPitch()==0) {
                assert(aligned && std::abs(yaw) <= std::atan(.60f)+.00001f);
                assert(rig.task.desiredPitch() == 0);
                return;
            }
        }
        assert(false && "entry normal alignment did not reach UI120 advance");
    }
};
// The selected half remains usable when the whole-zone consensus is invalid.
void independentHalfEntranceAcceptsDeliverableEntry() {
    for (bool injured : {false,true}) for (float offset : {-.08f,.08f}) {
        JawRig p; p.pose=false; p.half_entrance=true;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true; d.half_injured_=injured; d.half_x_=injured ? .15f : -.15f;
        p.supply_visible=!injured; p.injured_visible=injured;
        p.gap=.2f; p.yaw=offset>0 ? .25f : -.25f; p.robot_x=d.half_x_+offset;
        bool accepted=false;
        for (int i=0;i<45;++i) {
            const float next_yaw=p.yaw+p.out.motion.wz_rps*.04f;
            p.image_error=(d.half_x_-p.robot_x)/(p.gap+.15f)*.625f+std::tan(next_yaw)*.625f;
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            assert(out.motion.vx_mps==0);
            if(std::abs(p.image_error)>.10f)
                assert(p.image_error*out.motion.wz_rps<=0);
            assert(d.jaw_sequence_.recovery==TransportPhaseData::JawSequence::Recovery::Idle);
            if(out.reason=="carry_pre_entry_aligned") {
                assert(std::abs(p.image_error)<.10f && std::abs(p.rig.input.supply_view.edge_slope)<=.30f);
                accepted=true;break;
            }
        }
        assert(accepted && p.step().motion.vx_mps>0);
    }
    // Full-zone heading errors cannot change an observed selected-half command.
    JawRig a,b; a.half_entrance=b.half_entrance=true;
    a.metric_yaw_bias=-.6f; b.metric_yaw_bias=.6f;
    a.pose=false; b.pose=true;
    const auto x=a.step(),y=b.step();
    assert(x.reason==y.reason);
    assert(std::abs(x.motion.vx_mps-y.motion.vx_mps)<1e-6f);
    assert(std::abs(x.motion.wz_rps-y.motion.wz_rps)<1e-6f);
    std::cout<<"PASS selected-half entrance confirms a deliverable close approach without reversing\n";
}


// Replay the 11:23 log: selected supply half is 25% right of centre,
// but its negative edge slope asks for a left turn. Geometry and image
// lateral estimates disagree, bypassing the old 3 cm joint-entry gate.
void offsetImageCannotStartATurnAwayFromTheSelectedHalf() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for (bool injured : {false,true}) for (float side : {1.f,-1.f})
        for (auto stage : {Stage::EntryStop,Stage::AlignStop}) for (bool metric : {true,false}) {
            JawRig p; p.gap=.2f; p.yaw=-.55f*side;
            p.image_error=.251739f*side; p.edge_override=-.35f*side;
            p.inside=false; p.bearing=-.409557f*side; p.pose=metric;
            auto& d=p.rig.task.transport_phase_;
            d.half_locked_=true; d.half_injured_=injured; d.half_x_=injured ? .15f : -.15f;
            p.robot_x=d.half_x_-.1f*side;
            p.supply_visible=!injured; p.injured_visible=injured;
            auto& q=d.jaw_sequence_; q.stage=stage;
            for (int i=0;i<45;++i) {
                const auto out=p.step();
                assert((out.state==PushState::CARRY || out.state==PushState::SAFE_STOP) && out.motion.gripper_offset==20);
                assert(p.rig.task.desiredPitch()==-3000);
                assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
            }
            assert(p.out.state==PushState::SAFE_STOP && p.out.reason=="carry_entry_reposition_required");
            // New ranging cannot restart the removed reverse manoeuvre.
            p.pose=true;
            const auto out=p.step();
            assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        }
    std::cout<<"PASS logged off-centre left/right halves hold without reversing or turning away\n";
}

// Requested flow: one stationary confirmation, then switch directly to UI120.
void confirmedAlignmentDirectlyStartsNearViewPush() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for (bool injured : {false,true}) for (float side : {-1.f,1.f})
        for (float offset : {.0438403f,.07f,.09999f,.10f,.10001f,.11999f,.12f,.12001f,.15f}) {
            JawRig p; p.gap=.35f; p.yaw=0; p.image_error=side*offset;
            p.edge_override=0;
            p.sight=false; // No box-centre handoff is required after alignment.
            auto& d=p.rig.task.transport_phase_;
            d.half_locked_=true; d.half_injured_=injured; d.half_x_=injured ? .15f : -.15f;
            p.robot_x=d.half_x_; p.supply_visible=!injured; p.injured_visible=injured;
            d.jaw_sequence_.stage=Stage::AlignStop;
            bool accepted=false;
            for (int i=0;i<35;++i) {
                const auto out=p.step();
                assert(out.reason!="carry_entry_forward_correction");
                assert(out.reason!="carry_pre_entry_confirm");
                if (out.reason=="carry_pre_entry_aligned") {
                    assert(offset<.10f);
                    assert(p.rig.input.now_us>=1360000);
                    assert(d.jaw_sequence_.stage==Stage::ColourAdvance);
                    assert(p.rig.task.desiredPitch()==0);
                    assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
                    accepted=true; break;
                }
                assert(out.motion.vx_mps==0 && out.motion.wz_rps*side<=0);
                assert(p.rig.task.desiredPitch()==-3000);
            }
            assert(accepted==(offset<.10f));
            if (!accepted) {
                assert(p.out.state==PushState::CARRY);
                continue;
            }
            p.view=false;
            assert(p.step().motion.vx_mps==0);
            p.view=true; p.view_evidence=false; p.pose=false;
            p.fresh=false;
            assert(p.step().motion.vx_mps==0);
            p.fresh=true; p.region_inside=false;
            auto out=p.step();
            assert(out.reason=="carry_ui120_region_advance");
            assert(std::abs(out.motion.vx_mps-.06f)<1e-6f);
            assert(d.jaw_sequence_.stage==Stage::ColourAdvance);
            p.region_inside=true;
            out=p.step();
            for(int i=0;i<8;++i) {
                assert(out.motion.vx_mps==.06f && out.reason=="carry_ui120_region_overrun");
                out=p.step();
            }
            assert(out.motion.vx_mps==0 && out.immediate_motion_stop);
            assert(d.jaw_sequence_.stage==Stage::ColourStop);
            assert(out.motion.gripper_offset==20);
        }
    std::cout<<"PASS strict <10% confirmation directly switches to UI120 push; no correction or box gate\n";
}

void directNearViewPushKeepsColourSpeed() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for (bool sight : {false,true}) {
        JawRig p; p.yaw=0; p.gap=.35f; p.sight=sight;
        p.sight_box={10,10,100,40}; // Old centre-Y threshold must not delay switching.
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        p.rig.task.t_.max_speed=.40f;
        p.rig.task.t_.carry_speed=.05f;
        p.rig.task.t_.pushboard_floor_speed=.04f;
        bool aligned=false;
        for(int i=0;i<80;++i) {
            const auto out=p.step();
            assert(out.motion.vx_mps==0);
            if(q.stage==Stage::ColourAdvance) { aligned=true; break; }
            assert(p.rig.task.desiredPitch()==-3000);
        }
        assert(aligned && p.rig.task.desiredPitch()==0);
        p.view=false;
        assert(p.step().motion.vx_mps==0);
        p.view=true; p.region_inside=false;
        auto out=p.step();
        assert(std::abs(out.motion.vx_mps-.06f)<1e-6f);
        p.region_inside=true;
        out=p.step();
        const auto reached_us=p.rig.input.now_us;
        for(int i=0;i<8;++i) {
            assert(p.rig.input.now_us-reached_us<300000);
            assert(out.motion.vx_mps==.06f && !out.immediate_motion_stop);
            assert(out.motion.gripper_offset==20);
            // Use a final 20 ms tick to check the exact 300 ms boundary.
            if(i==7) p.rig.input.now_us-=20000;
            out=p.step();
        }
        assert(p.rig.input.now_us-reached_us==300000);
        assert(out.motion.vx_mps==0 && out.immediate_motion_stop);
        assert(q.stage==Stage::ColourStop && out.motion.gripper_offset==20);
    }
    std::cout << "PASS alignment switches to UI120 0.06m/s -> reached contour 0.06m/s for 0.3s -> brake\n";
}

void transportScanUsesConfiguredAngularSpeed() {
    for (float scan_wz : {.25f, .4f, .6f, .9f, 1.1f}) {
        JawRig plant;
        plant.pose = plant.sight = plant.view_evidence = false;
        plant.rig.task.t_.scan_wz = scan_wz;
        if (scan_wz >= .9f) {
            plant.rig.task.t_.transport_scan_wz = scan_wz;
            plant.rig.task.t_.transport_align_min_wz = .6f;
        }
        plant.rig.task.t_.min_turn_wz = .2f;
        for (int i = 0; i < 20; ++i) {
            const auto out = plant.step();
            assert(out.state == PushState::CARRY && out.reason == "searching_own_zone");
            assert(out.motion.vx_mps == 0);
            assert(std::abs(out.motion.wz_rps - scan_wz) < 1e-6f);
            assert(out.motion.gripper_offset == 20);
        }
    }
    std::cout << "PASS transport scan uses configured angular speed\n";
}


// Reproduce the logged sender ramp together with the documented stationary-turn dead zone.
void tiltedImageConvergesThroughSenderRamp() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(float direction : {-1.f,1.f}) {
        JawRig p;p.pose=false;p.gap=.2f;p.yaw=.70f*direction;
        p.ramped_dead_zone=true;p.rig.task.t_.scan_wz=.4f;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        q.stage=Stage::EntryStop;
        bool reached=false, effective=false;
        for(int i=0;i<800;++i) {
            const auto out=p.step();
            assert(out.motion.vx_mps>=0);
            effective |= std::abs(p.wire.wz_rps)>.25f;
            if(p.rig.task.desiredPitch()==0) {
                assert(std::abs(p.yaw)<=std::atan(.60f)+.00001f); reached=true;break;
            }
        }
        assert(effective && reached);
    }
    std::cout<<"PASS tilted image converges with real sender ramp and stationary dead zone\n";
}

void visibleTiltTakesPriorityOverMetricHeading() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(float direction : {-1.f,1.f}) {
        JawRig p;p.gap=.2f;p.yaw=.70f*direction;p.metric_yaw_bias=.8f*direction;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        q.stage=Stage::Align;q.normal_valid=true;q.normal_heading=.8f*direction;
        q.normal_seen_us=p.rig.input.now_us;q.normal_geometry_id="test-entry";
        bool image_turn=false;
        for(int i=0;i<50;++i) {
            const auto out=p.step();
            assert(out.motion.vx_mps==0);
            assert(out.motion.wz_rps*direction<=0);
            if(out.reason=="carry_entry_image_align") {image_turn=true;break;}
        }
        assert(image_turn && q.normal_rejected);
    }
    std::cout<<"PASS visible tilt prevents a conflicting metric heading from taking over\n";
}

void imagePulseMustFinishBrakingBeforeAnotherCorrection() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    // The pulse turns right toward the right-side target; this case tests
    // braking after a permitted image correction, not an away-turn request.
    JawRig p;p.pose=false;p.yaw=0;p.image_error=.15f;
    p.rig.task.transport_phase_.jaw_sequence_.stage=Stage::Align;
    p.edge_override=.0756f;
    bool turned=false,braked=false;
    uint64_t brake_started=0;
    for(int i=0;i<80;++i) {
        if(turned && !braked) p.edge_override=0;
        else if(braked) p.edge_override=.0286f;
        const auto out=p.step();
        if(!turned && out.motion.wz_rps!=0) {turned=true;continue;}
        if(turned && !braked && out.motion.wz_rps==0) {
            braked=true;brake_started=p.rig.input.now_us;continue;
        }
        if(braked && p.rig.input.now_us-brake_started<250000)
            assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        if(braked && p.rig.input.now_us-brake_started>=250000)break;
    }
    assert(turned && braked);
    std::cout<<"PASS image correction preserves the full stationary observation window\n";
}

void rejectedNormalCannotReturnDuringTheSameEntry() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    JawRig p;p.gap=.2f;p.yaw=0;p.image_error=-.15f;
    auto& q=p.rig.task.transport_phase_.jaw_sequence_;
    q.stage=Stage::Align;
    q.normal_valid=true;q.normal_heading=0;q.normal_seen_us=p.rig.input.now_us;
    q.normal_geometry_id="test-entry";
    p.edge_override=.35f;
    bool rejected=false;
    for(int i=0;i<100;++i) {
        if(rejected) {
            p.yaw=0;
            p.edge_override=i%2 ? 0.f : .35f;
        }
        auto out=p.step();
        if(out.reason=="carry_entry_pose_image_conflict") {rejected=true;continue;}
        if(rejected) assert(!q.normal_valid);
    }
    assert(rejected);
    std::cout<<"PASS a rejected metric normal stays rejected for this entry\n";
}

void detectorBearingConvergesInTheBodyFrame() {
    for(float target_heading : {-.35f,.35f}) {
        JawRig p; p.yaw=0; p.pose=false; p.view_evidence=false;
        for(int i=0;i<100;++i) {
            const float next_yaw=wrapAngle(p.yaw+p.out.motion.wz_rps*.04f);
            p.bearing=wrapAngle(target_heading-next_yaw); // left-positive body bearing
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            assert(out.motion.wz_rps*p.bearing>=-1e-6f);
        }
        assert(std::abs(wrapAngle(target_heading-p.yaw))<.12f);
    }
    std::cout<<"PASS detection-only transport converges to left/right body bearing\n";
}

void stablePoseNormalSurvivesLandmarkDropout() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for (float initial : {-.70f,.70f}) {
        JawRig p; p.gap=.2f; p.yaw=initial;
        p.rig.task.transport_phase_.jaw_sequence_.stage=Stage::EntryStop;
        // Establish the stationary zone normal before removing all pose landmarks.
        bool turning=false;
        for(int i=0;i<22;++i) {
            const auto out=p.step();
            turning |= out.motion.wz_rps*initial<0;
        }
        assert(turning && std::abs(p.yaw)>.05f);
        p.pose=p.view_evidence=false;
        p.bearing=initial>0 ? -.4f : .4f; // bbox direction conflicts with the measured normal
        for(int i=0;i<15;++i) {
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            assert(out.motion.vx_mps==0);
            assert(out.motion.wz_rps*initial<0);
            assert(!out.immediate_motion_stop);
            assert(p.rig.task.transport_phase_.jaw_sequence_.recovery==
                   TransportPhaseData::JawSequence::Recovery::Idle);
        }
        // IMU may finish turning, but cannot authorize entry without fresh centring.
        p.sight=false;
        for(int i=0;i<25;++i) {
            const auto out=p.step();
            assert(out.motion.vx_mps==0 && p.rig.task.desiredPitch()==-3000);
        }
    }
    std::cout<<"PASS stable pose normal uses IMU across landmark dropout\n";
}

void poseNormalRejectsJumpsAndUsesCaptureHeading() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(float offset : {0.f, 3.13f, -3.13f}) {
        JawRig p; p.gap=.2f; p.yaw=.70f; p.imu_offset=offset;
        // A delayed image sees a different chassis yaw. Pair it with capture IMU.
        p.capture_yaw_offset=.08f;
        p.rig.task.transport_phase_.jaw_sequence_.stage=Stage::EntryStop;
        for(int i=0;i<22;++i) p.step();
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        assert(q.normal_valid && std::abs(wrapAngle(q.normal_heading-offset))<.001f);
        const float target=q.normal_heading;
        // Isolate the metric/IMU fallback before forward correction is selected.
        q.stage=Stage::Align;
        p.view_evidence=false;
        assert(p.step().motion.vx_mps==0); // brake the preceding forward correction
        for(int i=0;i<12;++i) {
            p.metric_yaw_bias=i%2 ? .02f : -.02f;
            const auto out=p.step();
            assert(out.motion.vx_mps==0 && out.motion.wz_rps<0);
            assert(std::abs(wrapAngle(q.normal_heading-target))<.001f);
        }
        // Even a quality-gated pose can jump; one jump cannot replace the target.
        p.metric_yaw_bias=-.4f;
        const auto out=p.step();
        assert(out.motion.wz_rps<0 && std::abs(wrapAngle(q.normal_heading-target))<.001f);
    }
    // A repeated image is one observation, regardless of task update frequency.
    JawRig repeated; repeated.yaw=.2f; repeated.gap=.2f; repeated.repeat_pose_frame=true;
    repeated.rig.task.transport_phase_.jaw_sequence_.stage=Stage::Align;
    for(int i=0;i<4;++i) repeated.step();
    assert(!repeated.rig.task.transport_phase_.jaw_sequence_.normal_valid);
    // Current yaw must never substitute for missing capture-time IMU.
    JawRig no_capture; no_capture.gap=.2f; no_capture.capture_heading=false;
    for(int i=0;i<20;++i) no_capture.step();
    assert(!no_capture.rig.task.transport_phase_.jaw_sequence_.normal_valid);
    std::cout<<"PASS capture-time yaw, wraparound, noise, outliers and unique-frame gating\n";
}

void delayedImageLateralCompensationUsesCaptureYaw() {
    JawRig p; p.gap=.8f; p.yaw=.3f; p.capture_yaw_offset=.04f;
    // The chassis is exactly on its selected centreline. Its image error
    // comes solely from the yaw at capture, not a real lateral displacement.
    for(int i=0;i<6;++i) {
        const float captured_yaw=p.yaw+p.out.motion.wz_rps*.04f-p.capture_yaw_offset;
        p.image_error=.625f*std::tan(captured_yaw);
        const auto out=p.step();
        assert(out.state==PushState::CARRY && out.motion.wz_rps<0);
    }
    std::cout<<"PASS delayed image yaw does not create false lateral steering\n";
}

void stableImageConflictFallsBackFromBiasedMetricNormal() {
    for(float direction : {-1.f,1.f}) {
        JawRig p; p.gap=.2f; p.yaw=.3f*direction;
        p.metric_yaw_bias=.08f*direction;
        p.rig.task.transport_phase_.jaw_sequence_.stage=TransportPhaseData::JawSequence::Stage::EntryStop;
        bool reached=false;
        for(int i=0;i<500;++i) {
            p.gap-=std::cos(p.yaw)*p.out.motion.vx_mps*kDt;
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            if(p.rig.task.desiredPitch()==0) {
                assert(std::abs(p.yaw)<=std::atan(.60f)+.00001f); reached=true; break;
            }
        }
        assert(reached);
    }
    std::cout<<"PASS consistent image conflict falls back from biased metric normal\n";
}

void poseNormalExpiresAndStillNeedsFreshCentring() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    JawRig p; p.gap=.2f; p.yaw=.70f;
    p.rig.task.transport_phase_.jaw_sequence_.stage=Stage::EntryStop;
    for(int i=0;i<22;++i)p.step();
    p.pose=p.view_evidence=p.sight=false;
    const auto seen=p.rig.task.transport_phase_.jaw_sequence_.normal_seen_us;
    for(int i=0;i<38;++i) {
        const auto out=p.step();
        assert(out.state==PushState::CARRY && out.motion.vx_mps==0);
        assert(p.rig.task.desiredPitch()==-3000 && out.motion.gripper_offset==20);
        if(p.rig.input.now_us-seen>1500000) {
            assert(out.motion.wz_rps==0 && out.immediate_motion_stop);
            assert(!p.rig.task.transport_phase_.jaw_sequence_.normal_valid);
        }
    }
    p.view_evidence=p.sight=true;
    p.image_error=.1f; p.inside=false;
    // Two displaced captures cannot prove a persistent alignment failure.
    for(int i=0;i<2;++i) {
        p.step(); assert(p.rig.task.desiredPitch()==-3000);
    }
    // Fresh, centred evidence resumes the normal entry and colour endpoint.
    p.pose=true; p.image_error=0; p.inside=true; p.yaw=0;
    bool resumed=false;
    // Faster alignment may leave an off-centre view in the confirmation window.
    // Model translation through the approach and final entry.
    for(int i=0;i<600;++i) {
        p.gap-=p.out.motion.vx_mps*.04f;
        p.step();
        if(p.rig.task.desiredPitch()==0) { resumed=true; break; }
    }
    assert(resumed);
    // A new trip must discard the previous trip's heading reference.
    p.rig.task.enter(PushState::CARRY,p.rig.input.now_us,"next_pose_trip");
    assert(!p.rig.task.transport_phase_.jaw_sequence_.normal_valid);
    std::cout<<"PASS bounded dropout, fresh centring, reacquisition and trip reset\n";
}

void entryLandmarkLossTurnsTowardFreshOwnZone() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(auto stage : {Stage::EntryStop,Stage::Align,Stage::AlignStop}) {
        for(float bearing : {-.4f,.4f}) {
            JawRig p; p.yaw=0; p.pose=false; p.view_evidence=false; p.bearing=bearing;
            p.rig.task.transport_phase_.jaw_sequence_.stage=stage;
            p.waitForDirectionalRecovery();
            for(int i=0;i<12;++i) {
                auto out=p.step();
                assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
                assert(out.motion.vx_mps==0 && out.motion.wz_rps*bearing>0);
                assert(!out.immediate_motion_stop);
                assert(p.rig.task.desiredPitch()==-3000);
            }
            const float target=p.rig.task.transport_phase_.jaw_sequence_.recovery_heading;
            assert(std::abs(wrapAngle(target)-(bearing >= 0 ? float(CV_PI)*.5f : -float(CV_PI)*.25f))<.001f);
            p.view_evidence=true; p.bearing=-bearing; p.sight=false;
            for(int i=0;i<12;++i) {
                auto out=p.step();
                assert(out.motion.vx_mps==0 && out.motion.wz_rps*bearing>0);
                assert(p.rig.task.transport_phase_.jaw_sequence_.recovery_heading==target);
            }
        }
        for(int invalid=0;invalid<6;++invalid) {
            JawRig p; p.yaw=0; p.view_evidence=false; p.bearing=-.4f;
            p.rig.task.transport_phase_.jaw_sequence_.stage=stage;
            if(invalid==0)p.sight=false;
            if(invalid==1)p.enemy=true;
            if(invalid==2)p.stale_sight=true;
            if(invalid==3)p.bearing=0;
            if(invalid==4)p.bearing=std::numeric_limits<float>::quiet_NaN();
            if(invalid==5) {p.force_sent=true;p.forced_vx=.02f;}
            for(int i=0;i<50;++i) {
                auto out=p.step();
                assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
                assert(out.immediate_motion_stop && out.motion.gripper_offset==20);
            }
        }
        JawRig blocked; blocked.view_evidence=false; blocked.bearing=-.4f;
        blocked.rig.task.transport_phase_.jaw_sequence_.stage=stage;
        blocked.rig.input.directional_clearance_valid=true;
        blocked.rig.input.turn_safe=false;
        auto out=blocked.step();
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0 && out.immediate_motion_stop);
    }
    std::cout << "PASS entry landmark loss: latched relative right45/left90 turn\n";
}

void imageAlignmentIgnoresMetricYawAndSurvivesMappingLoss() {
    for(float yaw : {-.70f,0.f,.70f}) {
        JawRig p; p.yaw=yaw; p.gap=.2f;
        using Stage=TransportPhaseData::JawSequence::Stage;
        p.rig.task.transport_phase_.jaw_sequence_.stage=Stage::EntryStop;
        p.pose=false; // No ground mapping or PnP at the alignment entry.
        bool advanced=false, turned=false;
        for(int i=0;i<600;++i) {
            auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            if(out.motion.wz_rps!=0) {
                assert(out.motion.wz_rps*p.yaw<0);
                turned=true;
            }
            if(p.rig.task.desiredPitch()==0) {
                assert(std::abs(p.yaw)<=std::atan(.60f)+.00001f);
                advanced=true; break;
            }
        }
        assert(advanced && (yaw==0 || turned));
    }
    // A bogus metric normal cannot turn an already image-aligned robot.
    JawRig p;p.yaw=0;p.gap=.2f;p.metric_yaw_bias=.8f;
    p.rig.task.transport_phase_.jaw_sequence_.stage=TransportPhaseData::JawSequence::Stage::Align;
    p.rig.task.transport_phase_.jaw_sequence_.heading=1.f;
    auto out=p.step();
    assert(out.motion.wz_rps==0);
    // Missing image evidence must hold, rather than finish from IMU alone.
    p.sight=false;p.view_evidence=false;
    for(int i=0;i<15;++i) {
        out=p.step();
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        assert(p.rig.task.desiredPitch()==-3000);
    }
    // Fresh image landmarks end recovery search even when the metric solver is unavailable.
    JawRig recovery;recovery.pose=false;recovery.yaw=0;
    using Recovery=TransportPhaseData::JawSequence::Recovery;
    recovery.rig.task.transport_phase_.jaw_sequence_.recovery=Recovery::Search;
    for(int i=0;i<20;++i) {
        out=recovery.step();
        assert(out.motion.wz_rps==0);
        if(recovery.rig.task.transport_phase_.jaw_sequence_.recovery==Recovery::Idle)break;
    }
    assert(recovery.rig.task.transport_phase_.jaw_sequence_.recovery==Recovery::Idle);
    std::cout<<"PASS image-only entry alignment without metric pose\n";
}

void selectedHalfAloneCanRecoverAndAlign() {
    using Stage = TransportPhaseData::JawSequence::Stage;
    using Recovery = TransportPhaseData::JawSequence::Recovery;
    for (bool injured : {false, true}) {
        const auto select = [&](JawRig& p) {
            auto& d = p.rig.task.transport_phase_;
            d.half_locked_ = true;
            d.half_injured_ = injured;
            d.half_x_ = injured ? .15f : -.15f;
            p.robot_x = d.half_x_;
            p.supply_visible = !injured;
            p.injured_visible = injured;
            p.pose = false;
            p.yaw = 0;
            p.gap = .2f;
        };
        JawRig recovery;
        select(recovery);
        auto& q = recovery.rig.task.transport_phase_.jaw_sequence_;
        q.recovery = Recovery::Search;
        bool reacquired = false;
        for (int i = 0; i < 30; ++i) {
            const auto out = recovery.step();
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            assert(out.motion.gripper_offset == 20);
            if (q.recovery == Recovery::Idle) {
                assert(out.reason == "carry_pose_recovery_reacquired");
                reacquired = true;
                break;
            }
        }
        assert(reacquired && "selected half must recover without the other half");
        // The same single-half input must also pass stationary image alignment.
        JawRig alignment;
        select(alignment);
        auto& aligned = alignment.rig.task.transport_phase_.jaw_sequence_;
        aligned.stage = Stage::EntryStop;
        for (int i = 0; i < 60 && aligned.stage != Stage::ColourAdvance; ++i) {
            const auto out = alignment.step();
            assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0);
            assert(aligned.recovery == Recovery::Idle);
        }
        assert(aligned.stage == Stage::ColourAdvance);
        assert(alignment.rig.task.desiredPitch()==0);
        assert(alignment.step().motion.vx_mps>0);
        // Seeing only the other half cannot authorize recovery completion.
        JawRig wrong;
        select(wrong);
        wrong.supply_visible = injured;
        wrong.injured_visible = !injured;
        wrong.sight = false;
        auto& missing = wrong.rig.task.transport_phase_.jaw_sequence_;
        missing.recovery = Recovery::Search;
        for (int i = 0; i < 30; ++i) {
            wrong.step();
            assert(missing.recovery != Recovery::Idle);
        }
    }
    std::cout << "PASS selected supply/injured half alone recovers and aligns; other half cannot substitute\n";
}

void shortZoneDropoutHoldsBeforeScanning() {
    for (bool metric_first : {false, true}) {
        JawRig p;p.gap=.65f;p.yaw=0;
        p.pose=metric_first;p.view_evidence=true;
        p.step(); // No stable trajectory has yet been acquired.
        p.pose=p.sight=p.view_evidence=false;
        for(int i=1;i<=5;++i) {
            const auto out=p.step();
            assert(out.motion.vx_mps==0 && out.motion.gripper_offset==20);
            if(i<5) {
                assert(out.reason=="carry_selected_half_wait" && out.motion.wz_rps==0);
            } else assert(out.reason=="carry_half_reacquire_brake" && out.motion.wz_rps==0);
        }
        p.sight=p.view_evidence=true;
        const auto reacquired=p.step();
        assert(reacquired.reason.rfind("carry_half_reacquire_",0)==0 && reacquired.motion.vx_mps==0);
        p.stale_sight=true;p.view_evidence=false;
        for(int i=1;i<=5;++i) {
            const auto out=p.step();
            // An image without entrance distance cannot restart the stopped handoff.
            assert(out.reason.rfind("carry_half_reacquire_",0)==0 && out.motion.vx_mps==0);
        }
    }
    std::cout<<"PASS missing half without a planned route waits four frames before searching\n";
}

void missingZoneScanRemainsContinuous() {
    for (float initial_vx : {0.f, .05f}) {
        JawRig plant;
        plant.pose = plant.sight = plant.view_evidence = false;
        plant.rig.task.t_.scan_wz = .4f;
        plant.rig.task.t_.min_turn_wz = .2f;
        MotionVelocityRamp ramp{MotionLinkConfig{}};
        MotionCommand sent;
        sent.vx_mps = initial_vx;
        ramp.accept(sent, plant.rig.input.now_us);
        bool scanning = false;
        for (int i=0; i<60; ++i) {
            plant.out.motion = sent; // Actual sender feedback, including acceleration and hard stops.
            const auto out = plant.step();
            if (std::abs(sent.vx_mps)>=.001f) {
                assert(out.motion.wz_rps==0 && out.immediate_motion_stop);
            } else {
                assert(out.reason=="searching_own_zone");
                assert(out.motion.vx_mps==0 && out.motion.wz_rps==.4f);
                assert(!out.immediate_motion_stop);
                scanning = true;
            }
            sent=ramp.next(out.motion,plant.rig.input.now_us,out.immediate_motion_stop);
            ramp.accept(sent,plant.rig.input.now_us);
        }
        assert(scanning && std::abs(sent.wz_rps-.4f)<1e-5f);
        plant.out.motion=sent;
        plant.rig.input.directional_clearance_valid=true;
        plant.rig.input.turn_safe=false;
        const auto blocked=plant.step();
        assert(blocked.motion.vx_mps==0 && blocked.motion.wz_rps==0);
    }
    std::cout << "continuous missing-zone scan OK\n";
}

void twentyCentimetreEntryAndColourRelease() {
    JawRig plant; plant.yaw=0;
    plant.gap = .351f;
    auto out = plant.step();
    assert(out.state == PushState::CARRY && out.motion.vx_mps > 0);
    plant.gap = .20f;
    // A 15 cm range jump now requires three fresh captures before braking.
    for(int i=0;i<2;++i) {
        out=plant.step();
        assert(out.motion.vx_mps>0);
    }
    out = plant.step();
    assert(out.motion.vx_mps == 0 && out.immediate_motion_stop);
    assert(plant.rig.task.desiredPitch() == -3000);
    plant.reachColourLeg();
    plant.region_inside=false;
    for (int i=0;i<10;++i) assert(plant.step().motion.vx_mps > 0);
    plant.fresh = false;
    out = plant.step(); assert(out.motion.vx_mps == 0 && out.state == PushState::CARRY);
    plant.fresh = true; plant.region_inside=true;
    for(int i=0;i<8;++i) {
        out=plant.step();
        assert(out.motion.vx_mps==.06f && out.motion.gripper_offset==20);
    }
    plant.force_sent = true; plant.forced_vx = .05f;
    for (int i=0;i<12;++i) {
        out = plant.step();
        assert(out.motion.vx_mps == 0 && out.motion.wz_rps == 0 && out.immediate_motion_stop);
        assert(out.state == PushState::CARRY && out.motion.gripper_offset == 20);
    }
    plant.region_inside=false; // A lost extent revokes release confirmation.
    plant.forced_vx=0;
    out=plant.step();
    assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
    plant.region_inside=true;
    plant.forced_vx = 0;
    for (int i=0;i<45 && out.state == PushState::CARRY;++i) out=plant.step();
    assert(out.state == PushState::RAISE_RELEASE && plant.rig.task.desiredPitch()==0);
    assert(out.reason == "carry_ui120_region_stopped");
    out=plant.step(); assert(out.motion.gripper_offset==0 && out.state==PushState::RAISE_RELEASE);
    plant.rig.input.gripper_feedback_open=1;
    out=plant.step(); assert(out.state==PushState::BACK_OUT);
    assert(plant.rig.task.desiredPitch()==0);
    out=plant.step();
    assert(out.motion.vx_mps==0 && out.immediate_motion_stop);
    assert(out.reason=="jaw_retreat_stop_imu_calibrate");
    plant.rig.task.enter(PushState::CARRY,plant.rig.input.now_us,"next_trip");
    assert(!plant.rig.task.transport_phase_.jaw_sequence_.active);
}

// Replay the right-side box bearing in the 10:36 board log at t=2618250800us.
void loggedRightSideRecoveryTurnsRight() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    using Recovery=TransportPhaseData::JawSequence::Recovery;
    for (bool retry : {false,true}) {
        JawRig p; p.yaw=0; p.pose=false; p.view_evidence=false;
        p.bearing=-.497541f;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        q.stage=Stage::AlignStop;
        if (retry) { q.recovery=Recovery::Search; p.step(); }
        const auto out=retry ? p.step() : p.waitForDirectionalRecovery();
        assert(out.reason=="carry_pose_recovery_turn");
        assert(out.motion.wz_rps<0);
        assert(std::abs(wrapAngle(q.recovery_heading-p.yaw)+float(CV_PI)*.25f)<.001f);
    }
    std::cout<<"PASS logged right-side bearing turns right on first recovery and retry\n";
}

void carryStationaryTurnsUseSearchRate() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    using Recovery=TransportPhaseData::JawSequence::Recovery;
    for (float direction : {-1.f,1.f}) for (float cap : {.4f,.32f,.9f}) {
        JawRig p; p.yaw=0; p.pose=false; p.view_evidence=false; p.sight=false;
        p.rig.task.t_.scan_wz=.4f; p.rig.task.t_.max_wz=cap;
        if (cap == .9f) {
            p.rig.task.t_.max_wz = .6f; // Other phases keep their original cap.
            p.rig.task.t_.transport_scan_wz = .9f;
            p.rig.task.t_.transport_align_min_wz = .6f;
        }
        p.rig.task.t_.pixel_align_max_wz=.2f;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        q.recovery=Recovery::Turn; q.recovery_heading=direction;
        auto out=p.step();
        assert(std::abs(out.motion.wz_rps-direction*cap)<.001f);
        q.recovery=Recovery::Idle; q.stage=Stage::Align;
        q.normal_valid=true; q.normal_heading=direction; q.normal_seen_us=p.rig.input.now_us;
        out=p.step();
        assert(out.reason=="carry_entry_pose_imu_align");
        assert(std::abs(out.motion.wz_rps-direction*cap)<.001f);
    }
    std::cout<<"PASS carry IMU alignment and recovery use search rate within angular limit\n";
}

void approachWithoutLocalEvidenceUsesZoneDirection() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    using Recovery=TransportPhaseData::JawSequence::Recovery;
    for(float bearing : {-.4f,.4f}) {
        JawRig p; p.gap=.8f; p.yaw=0;
        p.pose=false; p.view_evidence=false; p.bearing=bearing;
        auto out=p.step();
        assert(out.reason=="carry_drive_to_sighted_zone");
        assert(out.motion.wz_rps*bearing>0);
        assert(p.rig.task.transport_phase_.jaw_sequence_.recovery==Recovery::Idle);
        // Whole-zone ranging cannot bypass the stopped selected-half handoff.
        p.pose=true; p.bearing=0; p.gap=.36f;
        out=p.step();
        assert(out.reason.rfind("carry_half_reacquire_",0)==0 && out.motion.vx_mps==0);
        assert(p.rig.task.transport_phase_.jaw_sequence_.recovery==Recovery::Idle);
        p.gap=.20f; out=p.step();
        assert(out.motion.vx_mps==0);
        assert(p.rig.task.transport_phase_.jaw_sequence_.stage==Stage::Approach);
        p.view_evidence=true;
        bool parked=false;
        for(int i=0;i<25;++i) {
            out=p.step();assert(out.motion.vx_mps==0);
            if(out.reason=="carry_pre_entry_stop") {parked=true;break;}
        }
        assert(parked);
        assert(p.rig.task.transport_phase_.jaw_sequence_.stage==Stage::EntryStop);
        p.pose=p.view_evidence=false; p.bearing=bearing; out=p.waitForHalfSearch();
        assert(out.reason=="carry_half_reacquire_turn");
        assert(out.motion.wz_rps*bearing>0);
    }
    JawRig p; p.yaw=0; p.gap=.2f;
    p.step();
    p.rig.task.transport_phase_.jaw_sequence_.stage=Stage::Align;
    p.pose=false; p.view_evidence=false; p.bearing=.4f;
    auto entry=p.waitForHalfSearch();
    assert(entry.reason=="carry_half_reacquire_turn" && entry.motion.wz_rps>0);
    assert(p.rig.task.transport_phase_.jaw_sequence_.recovery==Recovery::Idle);
    p.pose=true; p.view_evidence=true; p.bearing=-.4f;p.image_error=0;p.edge_override=0;
    bool recovered=false;
    for(int i=0;i<30;++i) {
        const auto out=p.step();assert(out.motion.vx_mps==0);
        if(out.reason=="carry_half_reacquired") {recovered=true;break;}
    }
    assert(recovered && p.rig.task.transport_phase_.jaw_sequence_.stage==Stage::AlignStop);
    JawRig repeat; repeat.yaw=0; repeat.pose=false; repeat.view_evidence=false;
    repeat.bearing=-.4f;
    repeat.rig.task.transport_phase_.jaw_sequence_.recovery=Recovery::Search;
    auto out=repeat.step();
    assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
    assert(repeat.rig.task.transport_phase_.jaw_sequence_.recovery==Recovery::Turn);
    out=repeat.step();
    assert(out.reason=="carry_pose_recovery_turn" && out.motion.wz_rps<0);
    std::cout<<"PASS approach direction, stationary entry search and stopped selected-half recovery\n";
}

void poseLossStartsDirectionalRecovery() {
    JawRig p; p.gap=.2f; p.yaw=0;
    p.step();
    p.pose=false; p.view_evidence=false; p.bearing=.4f;
    auto out=p.waitForHalfSearch();
    assert(out.reason=="carry_half_reacquire_turn");
    assert(out.motion.vx_mps==0 && out.motion.wz_rps>0);
    out=p.step();
    assert(out.motion.vx_mps==0 && out.motion.wz_rps>0);
    assert(out.motion.gripper_offset==20);
}

void poseLossMeasuredRecoveryAndSearch() {
    using Recovery=TransportPhaseData::JawSequence::Recovery;
    for(float bearing : {-.4f,.4f}) {
        JawRig p; p.gap=.5f; p.yaw=0;
        p.step(); // legacy recovery outside the near-entry parking branch
        p.rig.task.transport_phase_.jaw_sequence_.stage=TransportPhaseData::JawSequence::Stage::AlignStop;
        p.pose=false; p.view_evidence=false; p.bearing=bearing;
        uint64_t seq=0;
        float velocity=0, position=0;
        bool drove=false, searched=false;
        for(int n=0;n<650;++n) {
            auto& in=p.rig.input;
            in.imu_translation_samples.clear();
            const float target=p.out.motion.vx_mps;
            const float acceleration=(target-velocity)/.04f;
            const float sample_yaw=p.yaw+p.out.motion.wz_rps*.04f;
            for(int i=1;i<=4;++i) {
                const float next=velocity+acceleration*.01f;
                position+=(velocity+next)*.005f; velocity=next;
                ImuTranslationSample a;
                a.sequence=++seq; a.received_us=in.now_us+i*10000;
                a.device_time_ms=a.received_us/1000; a.valid=true;
                a.acceleration_mps2={acceleration+.01f,0,9.8f};
                a.quaternion_wxyz={std::cos(sample_yaw*.5f),0,0,std::sin(sample_yaw*.5f)};
                in.imu_translation_samples.push_back(a);
            }
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            if(out.motion.vx_mps>0) {
                drove=true;
                assert(std::abs(wrapAngle(p.yaw-(bearing >= 0 ? float(CV_PI)*.5f : -float(CV_PI)*.25f)))<.04f);
            }
            if(out.reason=="carry_pose_recovery_retry_direction") {
                searched=true;
                assert(drove && position>=.145f && position<.17f);
                assert(std::abs(p.yaw)<.04f);
                assert(p.rig.task.transport_phase_.jaw_sequence_.recovery==Recovery::Turn);
                assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
                break;
            }
        }
        assert(drove && searched);
        p.sight=false;
        p.rig.task.transport_phase_.jaw_sequence_.recovery=Recovery::Search;
        for(int i=0;i<30;++i) {
            auto out=p.step();
            assert(out.reason=="carry_pose_recovery_search");
            assert(out.motion.vx_mps==0 && out.motion.wz_rps!=0 && !out.immediate_motion_stop);
        }
        // Reacquisition brakes, confirms fresh observations and rejoins entry.
        p.sight=true; p.pose=true; p.view_evidence=true;
        for(int i=0;i<20;++i) {
            p.step();
            if(p.rig.task.transport_phase_.jaw_sequence_.recovery==Recovery::Idle)break;
        }
        assert(p.rig.task.transport_phase_.jaw_sequence_.recovery==Recovery::Idle);
    }
    // Full landmarks cannot bypass returning to the original direction.
    {
        JawRig p; p.yaw=-.6f; p.pose=true; p.view_evidence=true; p.sight=false;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        q.recovery=Recovery::Return; q.recovery_original_heading=0;
        bool returned=false;
        for(int i=0;i<300;++i) {
            const auto out=p.step();
            assert(out.motion.vx_mps==0);
            if(q.recovery==Recovery::Search) { returned=true; break; }
            assert(q.recovery==Recovery::Return);
        }
        assert(returned && std::abs(p.yaw)<.04f);
        p.view_evidence=false; p.step();
        assert(q.recovery==Recovery::Search);
        p.view_evidence=true;
        for(int i=0;i<20 && q.recovery!=Recovery::Idle;++i) p.step();
        assert(q.recovery==Recovery::Idle && q.stage==TransportPhaseData::JawSequence::Stage::Approach);
    }
    // Missing/enemy/stale detections cannot begin a measured leg.
    for(int kind=0;kind<3;++kind) {
        JawRig p;p.step();p.pose=false;p.view_evidence=false;
        if(kind==0)p.sight=false;
        if(kind==1)p.enemy=true;
        if(kind==2)p.stale_sight=true;
        for(int i=0;i<15;++i) {
            auto out=p.step();assert(out.motion.vx_mps==0);
            assert(p.rig.task.transport_phase_.jaw_sequence_.recovery==Recovery::Idle);
        }
    }
    // IMU data is mandatory, not a command-duration estimate.
    JawRig noimu;noimu.gap=.5f;noimu.yaw=0;noimu.step();
    noimu.rig.task.transport_phase_.jaw_sequence_.stage=TransportPhaseData::JawSequence::Stage::AlignStop;
    noimu.pose=false;noimu.view_evidence=false;noimu.bearing=.4f;
    for(int i=0;i<350;++i) {
        auto out=noimu.step();assert(out.motion.vx_mps==0);
    }
    assert(noimu.rig.task.transport_phase_.jaw_sequence_.recovery==Recovery::Calibrate);
    std::cout<<"PASS directional IMU15cm -> re-evaluate zone direction -> fresh pose reacquisition"<<std::endl;
}

void selectedSubzoneMustBeCentred() {
    for (int cargo : {0,1}) for (float offset : {-.15f,.15f}) {
        JawRig p; p.yaw=0; p.gap=.20f;
        p.rig.task.transport_phase_.half_locked_=true;
        p.rig.task.transport_phase_.half_injured_=cargo;
        p.rig.task.transport_phase_.half_x_=cargo ? .15f : -.15f;
        p.robot_x=p.rig.task.halfX()+offset;
        p.image_error=-offset;
        bool turned=false;
        for(int i=0;i<40;++i) {
            auto o=p.step();
            assert(o.motion.vx_mps==0 && p.image_error*o.motion.wz_rps<=0);
            turned|=o.motion.wz_rps!=0;
            assert(p.rig.task.desiredPitch()==-3000);
            assert(o.motion.gripper_offset==20 && (o.state==PushState::CARRY || o.state==PushState::SAFE_STOP));
        }
        assert(turned && "a tolerated edge permits turning toward the selected half");
    }
    for (int invalid : {0,1,2}) {
        JawRig p; p.yaw=0; p.gap=.20f;
        if(invalid==0)p.view_evidence=false;
        if(invalid==1)p.inside=false;
        if(invalid==2)p.image_error=.15f;
        for(int i=0;i<40;++i) {
            p.step();
            assert(p.rig.task.desiredPitch()==-3000);
        }
    }
    // A visual gap does not erase physical stopped time, but needs 3 new valid frames.
    JawRig p; p.yaw=0; p.gap=.20f; p.view_evidence=false;
    p.rig.task.transport_phase_.jaw_sequence_.stage=TransportPhaseData::JawSequence::Stage::EntryStop;
    for(int i=0;i<15;++i)p.step();
    p.view_evidence=true;
    for(int i=0;i<2;++i) { p.step(); assert(p.rig.task.desiredPitch()==-3000); }
    bool advanced=false;
    for(int i=0;i<20;++i) { p.step(); advanced|=p.rig.task.desiredPitch()==0; }
    assert(advanced);
}
// The 11:35 board trace repeatedly approached at yaw=-0.55 with the
// selected half still right of centre: the 0.55 bias cap cancelled the normal.
void loggedOffsetApproachConvergesWithoutRetreatLoop() {
    for (bool injured : {false,true}) for (float side : {-1.f,1.f}) {
        JawRig p; p.gap=.65f; p.yaw=-.55f*side;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true; d.half_injured_=injured; d.half_x_=injured ? .15f : -.15f;
        p.supply_visible=!injured; p.injured_visible=injured;
        const float target=d.half_x_;
        p.robot_x=target-.6f*side;
        p.rig.task.t_.carry_budget_us=90000000;
        MotionVelocityRamp ramp{MotionLinkConfig{}};
        MotionCommand sent;
        ramp.accept(sent,p.rig.input.now_us);
        bool advanced=false, reversing=false;
        int reverse_legs=0;

        for(int i=0;i<2200;++i) {
            p.robot_x-=std::sin(p.yaw)*sent.vx_mps*kDt;
            p.gap-=std::cos(p.yaw)*sent.vx_mps*kDt;
            p.out.motion=sent;
            const float next_yaw=wrapAngle(p.yaw+sent.wz_rps*kDt);
            const float dx=target-p.robot_x,depth=p.gap+.15f;
            const float bx=std::cos(next_yaw)*dx+std::sin(next_yaw)*depth;
            const float by=-std::sin(next_yaw)*dx+std::cos(next_yaw)*depth;
            p.image_error=.625f*bx/by;
            const auto out=p.step();
            if(i==0) {
                assert(p.image_error*side>.01f);
                assert(out.reason=="carry_entry_path_review");
                assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
            }
            const bool reverse=out.motion.vx_mps<0;
            if(reverse && !reversing) ++reverse_legs;
            reversing=reverse;
            assert(reverse_legs<=1 && "backoff must not repeat on the same entrance");
            if(reverse) {
                assert(d.jaw_sequence_.stage==TransportPhaseData::JawSequence::Stage::Backoff);
                assert(out.motion.vx_mps>=-.05f && out.motion.wz_rps==0);
            }
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            if(p.rig.task.desiredPitch()==0) {
                assert(std::abs(p.yaw)<=std::atan(.60f)+.00001f && std::abs(p.image_error)<.10f);
                advanced=true; break;
            }
            sent=ramp.next(out.motion,p.rig.input.now_us,out.immediate_motion_stop);
            ramp.accept(sent,p.rig.input.now_us);
        }
        if(!advanced) std::cerr<<"logged offset failed side="<<side<<" injured="<<injured
            <<" x="<<p.robot_x<<" gap="<<p.gap<<" yaw="<<p.yaw
            <<" error="<<p.image_error<<" reason="<<p.out.reason<<std::endl;
        assert(advanced);
        std::cout<<"PASS logged offset converges side="<<side<<" injured="<<injured
                 <<" reverse_legs="<<reverse_legs<<" x_error="<<p.robot_x-target<<"\n";
    }
}

void centredApproachClosedLoop() {
    for (int cargo : {0,1}) for(float offset : {-.08f,.08f}) for(float camera_offset : {0.f,.025f}) {
        JawRig p; p.yaw=0; p.gap=.65f;
        // Match the controlled-field runtime's 90 s carry budget.
        p.rig.task.t_.carry_budget_us=90000000;
        p.rig.task.transport_phase_.half_locked_=true;
        p.rig.task.transport_phase_.half_injured_=cargo;
        p.rig.task.transport_phase_.half_x_=cargo ? .15f : -.15f;
        const float target=p.rig.task.halfX();
        p.robot_x=target+offset;
        bool advanced=false;
        // Image-led correction includes a full stationary observation after
        // every bounded turn, before the final forward approach.
        for(int i=0;i<1000;++i) {
            p.robot_x-=std::sin(p.yaw)*p.out.motion.vx_mps*.04f;
            p.gap-=std::cos(p.yaw)*p.out.motion.vx_mps*.04f;
            const float next_yaw=wrapAngle(p.yaw+p.out.motion.wz_rps*.04f);
            const float dx=target-p.robot_x+camera_offset, depth=p.gap+.15f;
            const float bx=std::cos(next_yaw)*dx+std::sin(next_yaw)*depth;
            const float by=-std::sin(next_yaw)*dx+std::cos(next_yaw)*depth;
            p.image_error=bx/by*.625f; // simulated 800 px focal length / 1280 width
            auto o=p.step();
            if(p.rig.task.desiredPitch()==0) {
                assert(std::abs(p.yaw)<=std::atan(.60f)+.00001f && std::abs(p.image_error)<.10f);
                advanced=true;break;
            }
            assert(o.state==PushState::CARRY && o.motion.gripper_offset==20);
        }
        if(!advanced)std::cerr<<"closed loop failed cargo="<<cargo<<" offset="<<offset
            <<" robot_x="<<p.robot_x<<" gap="<<p.gap<<" yaw="<<p.yaw
            <<" image="<<p.image_error<<" reason="<<p.out.reason<<std::endl;
        assert(advanced);
    }
    // Once the reference is latched, IMU alignment survives metric dropout.
    JawRig p;p.gap=.2f;
    for(int i=0;i<20;++i) {
        p.step();
        if(p.rig.task.transport_phase_.jaw_sequence_.stage==TransportPhaseData::JawSequence::Stage::Align)break;
    }
    p.pose=false; p.half_entrance=true;
    bool advanced=false;
    for(int i=0;i<350;++i) {p.step();advanced|=p.rig.task.desiredPitch()==0;}
    assert(advanced);
    // Close-range landmark loss keeps the already-confirmed line and region extent endpoint.
    JawRig close;close.reachColourLeg();close.view_evidence=false;close.region_inside=true;
    PushOutput released;
    for(int i=0;i<30;++i) {
        released=close.step();
        if(released.state==PushState::RAISE_RELEASE)break;
    }
    assert(released.state==PushState::RAISE_RELEASE);
    // Near-view keypoint errors cannot veto the contour of the already-aligned half.
    JawRig wrong;wrong.reachColourLeg();wrong.image_error=.10f;wrong.inside=false;wrong.region_inside=true;
    for(int i=0;i<30;++i) {
        auto o=wrong.step();
        if(o.state==PushState::RAISE_RELEASE)break;
    }
    assert(wrong.out.state==PushState::RAISE_RELEASE);
}
void metricAndHeadingAreRequiredForEntry() {
    JawRig plant;
    plant.pose = false; plant.view_evidence = false;
    for (int i=0;i<20;++i) {
        auto out=plant.step();
        assert(out.state==PushState::CARRY && out.motion.vx_mps==0);
        assert(out.immediate_motion_stop);
        assert(out.reason=="carry_zone_metric_wait");
        assert(out.motion.gripper_offset==20 && plant.rig.task.desiredPitch()==-3000);
    }
    // Invalid direction evidence cannot authorize forward travel without ranging.
    for (int kind=0;kind<4;++kind) {
        JawRig invalid;
        invalid.pose=false; invalid.view_evidence=false;
        invalid.sight=kind!=0;
        invalid.enemy=kind==1;
        invalid.stale_sight=kind==2;
        if(kind==3) invalid.bearing=std::numeric_limits<float>::quiet_NaN();
        auto out=invalid.step();
        assert(out.motion.vx_mps==0 && out.state==PushState::CARRY);
    }
    for(float bearing : {-.2f,.2f}) {
        JawRig turning; turning.pose=false; turning.view_evidence=false; turning.bearing=bearing;
        auto out=turning.step();
        assert(out.motion.wz_rps*bearing>0 && out.motion.gripper_offset==20);
    }
    plant.pose=plant.view_evidence=true; plant.gap=.299f;
    auto out=plant.step();
    assert(out.motion.vx_mps==0 && out.immediate_motion_stop);
    plant.heading=false;
    out=plant.step();
    assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
    assert(out.state==PushState::SAFE_STOP && out.motion.gripper_offset==20);
}

void deliveryReturnsDirectlyToSearch() {
    for (bool active : {false, true}) {
        JawRig plant;
        plant.rig.task.transport_phase_.jaw_sequence_.active=active;
        plant.rig.task.enter(PushState::VERIFY_DELIVERY,plant.rig.input.now_us,"delivery-test");
        const auto out=plant.step();
        assert(out.state==PushState::SCAN && out.phase==TaskPhase::Search);
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        assert(plant.rig.task.trip_context_.trip_.total()==0);
    }
    std::cout << "PASS delivery enters search without a return turn\n";
}

// The 16:11 run spent almost all of BACK_OUT's wall budget calibrating,
// then faulted while completing the final 250 ms stopped confirmation.
void retreatUsesIndependentStageBudgets() {
    DepositRig p;
    auto& task=p.rig.task;
    task.t_.back_speed=task.t_.max_speed=.4f;
    task.t_.retreat_budget_us=600000;
    task.enter(PushState::BACK_OUT,p.rig.input.now_us,"logged-calibration-delay");
    task.commandFrame(1,p.rig.input.now_us);
    p.rig.input.gripper_feedback_open=1;
    const auto start=p.rig.input.now_us;
    for(int i=0;i<28;++i) {
        auto out=p.tick(true,false);
        assert(out.state==PushState::BACK_OUT && out.motion.vx_mps==0);
    }
    // Waiting without samples must not count toward a 500 ms stationary window.
    for(int i=0;i<10;++i) {
        const auto out=p.tick();
        assert(out.state==PushState::BACK_OUT && out.motion.vx_mps==0);
    }
    for(int i=0;i<100 && p.out.state==PushState::BACK_OUT;++i) p.tick();
    assert(p.out.state==PushState::VERIFY_DELIVERY);
    assert(p.rig.input.now_us-start>2000000);
    assert(p.position<=-.145f && p.position>=-.18f);
    std::cout<<"PASS retreat calibration, movement and stopped confirmation own their budgets\n";
}

void retreatTimeoutsIdentifyTheFailedStage() {
    for(int mode=0;mode<3;++mode) {
        DepositRig p;auto& task=p.rig.task;
        task.t_.back_speed=task.t_.max_speed=.4f;
        task.t_.retreat_budget_us=600000;
        task.enter(PushState::BACK_OUT,p.rig.input.now_us,"retreat-stage-timeout");
        task.commandFrame(1,p.rig.input.now_us);
        p.rig.input.gripper_feedback_open=1;
        for(int i=0;i<160;++i) {
            // Keep acceleration at zero despite reverse commands for mode 1.
            if(mode==2 && task.transport_phase_.jaw_retreat_.stage==
                    TransportPhaseData::JawRetreat::Stage::Stopping)
                p.out.motion.vx_mps=-.02f; // sender never reports zero
            p.tick(true,mode!=0,0,mode==1 ? 0.f : 1.f);
            if(p.out.state!=PushState::BACK_OUT)break;
        }
        const char* reasons[]={"jaw_retreat_calibration_timeout","jaw_retreat_motion_timeout",
                               "jaw_retreat_stop_timeout"};
        assert(p.out.state==PushState::SAFE_STOP && p.out.reason==reasons[mode]);
        assert(p.out.motion.vx_mps==0 && p.out.immediate_motion_stop);
    }
    // Field-calibration candidate: .10 rad/s gyro and .12 m/s² alternating
    // projected acceleration noise, with unchanged 200 ms freshness requirements.
    DepositRig p;auto& task=p.rig.task;
    p.gyro_noise=.10f;
    task.enter(PushState::BACK_OUT,p.rig.input.now_us,"quiet-noisy-imu");
    task.commandFrame(1,p.rig.input.now_us);
    for(int i=0;i<25 && p.out.motion.vx_mps>=0;++i)
        p.tick(true,true,i%2 ? -.12f : .12f);
    assert(p.out.motion.vx_mps<0 && p.out.state==PushState::BACK_OUT);
    std::cout<<"PASS calibration/movement/stop timeouts are distinct; stopped noise converges\n";
}

void retreatPreparationSurvivesReleaseAndRejectsOpeningMotion() {
    for(bool disturbed : {false,true}) {
        DepositRig p;
        auto& task=p.rig.task;auto& in=p.rig.input;
        task.enter(PushState::CARRY,in.now_us,"precalibrated-release");
        task.transport_phase_.placing_=false;
        task.transport_phase_.jaw_sequence_.stage=TransportPhaseData::JawSequence::Stage::ColourStop;
        task.transport_phase_.jaw_sequence_.heading=0;
        task.pitch_cmd_=in.camera_pitch_cdeg=task.t_.track_pitch_cdeg;
        for(int i=0;i<40;++i) {
            in.transport_floor.valid=in.transport_floor.region_valid=in.transport_floor.inside=true;
            in.transport_floor.sample_us=in.now_us+40000;
            p.tick();
            if(p.out.state==PushState::RAISE_RELEASE)break;
        }
        assert(p.out.state==PushState::RAISE_RELEASE);
        // Delay the open acknowledgement while continuing to supply quiet samples.
        in.gripper_feedback_open=0;
        for(int i=0;i<10;++i) p.tick();
        assert(task.transport_phase_.jaw_retreat_.imu.ready());
        if(disturbed) {
            // A strong acceleration during opening invalidates the cached bias.
            for(int i=0;i<4;++i) p.tick(true,true,(i%2 ? -2.f : 2.f));
        }
        in.gripper_feedback_open=1;
        p.tick();assert(p.out.state==PushState::BACK_OUT);
        const auto ack=in.now_us;
        p.tick();
        if(disturbed) assert(p.out.motion.vx_mps==0);
        for(int i=0;i<30 && p.out.motion.vx_mps>=0;++i) p.tick();
        assert(p.out.state==PushState::BACK_OUT && p.out.motion.vx_mps<0);
        if(!disturbed) assert(in.now_us-ack<=80000);
    }
    std::cout<<"PASS stopped placement prepares retreat; opening motion revokes preparation\n";
}

void releasedCargoRetreatsImuFifteenCentimetres() {
    for (float physical_scale : {1.f, .5f}) {
        DepositRig plant;
        auto& rig=plant.rig;
        rig.task.enter(PushState::BACK_OUT,rig.input.now_us,"retreat-test");
        rig.task.commandFrame(1,rig.input.now_us);
        rig.input.gripper_feedback_open=1;
        bool reversed=false;
        PushOutput out;
        for(int i=0;i<1000;++i) {
            out=plant.tick(true,true,0,physical_scale);
            assert(out.motion.wz_rps==0 && out.motion.gripper_offset==0);
            reversed |= out.motion.vx_mps<0;
            if(out.state!=PushState::BACK_OUT)break;
        }
        assert(reversed && plant.position<=-.145f && plant.position>=-.17f);
        assert(out.state==PushState::VERIFY_DELIVERY);
        assert(out.immediate_motion_stop && out.motion.vx_mps==0);
        std::cout << "PASS IMU retreat physical_scale=" << physical_scale
                  << " position=" << plant.position << "\n";
    }
    // Nonzero sent commands and arbitrarily large command travel cannot finish a blocked leg.
    DepositRig stuck;
    stuck.rig.task.enter(PushState::BACK_OUT,stuck.rig.input.now_us,"stuck-retreat");
    stuck.rig.task.commandFrame(1,stuck.rig.input.now_us);
    stuck.rig.input.gripper_feedback_open=1;
    bool requested=false;
    for(int i=0;i<80;++i) {
        stuck.rig.task.actionProgress().travel=10;
        const auto out=stuck.tick(true,true,0,0);
        assert(out.state==PushState::BACK_OUT);
        requested |= out.motion.vx_mps<0;
    }
    assert(requested && stuck.position==0);
    // No valid acceleration samples means no initial reverse request.
    DepositRig missing;
    missing.rig.task.enter(PushState::BACK_OUT,missing.rig.input.now_us,"missing-imu");
    missing.rig.task.commandFrame(1,missing.rig.input.now_us);
    for(int i=0;i<25;++i) {
        const auto out=missing.tick(true,false);
        assert(out.state==PushState::BACK_OUT && out.motion.vx_mps==0);
    }
    // Lost IMU data while reversing must stop, never fall back to sent-speed travel.
    DepositRig interrupted;
    interrupted.rig.task.enter(PushState::BACK_OUT,interrupted.rig.input.now_us,"imu-gap");
    interrupted.rig.task.commandFrame(1,interrupted.rig.input.now_us);
    for(int i=0;i<25 && interrupted.out.motion.vx_mps>=0;++i) interrupted.tick();
    assert(interrupted.out.motion.vx_mps<0);
    for(int i=0;i<8 && interrupted.out.state!=PushState::SAFE_STOP;++i)
        interrupted.tick(true,false);
    assert(interrupted.out.state==PushState::SAFE_STOP);
    assert(interrupted.out.reason=="jaw_retreat_imu_gap" && interrupted.out.motion.vx_mps==0);

    // A stopped sender cannot accumulate acceleration drift into retreat distance.
    DepositRig zero_sent;
    zero_sent.rig.task.enter(PushState::BACK_OUT,zero_sent.rig.input.now_us,"zero-sent");
    zero_sent.rig.task.commandFrame(1,zero_sent.rig.input.now_us);
    for(int i=0;i<25 && zero_sent.out.motion.vx_mps>=0;++i) zero_sent.tick();
    assert(zero_sent.out.motion.vx_mps<0);
    for(int i=0;i<60;++i) {
        const auto out=zero_sent.tick(false,true,1.f);
        assert(out.state==PushState::BACK_OUT);
    }
    assert(zero_sent.rig.task.transport_phase_.jaw_retreat_.imu.distance()==0);

    // A pause retains measured progress, then requires a new stationary calibration.
    DepositRig paused;
    paused.rig.task.enter(PushState::BACK_OUT,paused.rig.input.now_us,"paused-retreat");
    paused.rig.task.commandFrame(1,paused.rig.input.now_us);
    for(int i=0;i<80 && paused.position>-.06f;++i) paused.tick();
    assert(paused.position<-.05f && paused.position>-.1f);
    paused.rig.input.run=false;
    auto out=paused.tick();
    assert(out.state==PushState::WAIT_START && out.motion.vx_mps==0);
    const float retained=paused.rig.task.transport_phase_.jaw_retreat_.completed_m;
    assert(retained>.04f && retained<.1f);
    for(int i=0;i<10;++i) paused.tick();
    paused.rig.input.run=true;
    paused.tick();
    assert(paused.out.state==PushState::BACK_OUT && paused.out.motion.vx_mps==0);
    assert(paused.rig.task.transport_phase_.jaw_retreat_.completed_m==retained);
    for(int i=0;i<300 && paused.out.state==PushState::BACK_OUT;++i) paused.tick();
    assert(paused.out.state==PushState::VERIFY_DELIVERY);
    assert(paused.position<=-.145f && paused.position>=-.18f);
    std::cout << "PASS IMU gap stops; zero sent holds distance; pause retains IMU progress\n";
    std::cout << "PASS sent travel cannot replace IMU samples or physical displacement\n";
}

void completeJawTransportLifecycle(bool inspected = false) {
    DepositRig plant;
    auto& rig=plant.rig;
    auto& in=rig.input;
    auto& task=rig.task;
    task.t_.far_pitch_cdeg=-3000; task.t_.track_pitch_cdeg=0; task.t_.near_pitch_cdeg=2000;
    task.enter(PushState::CARRY,in.now_us,"complete-jaw-trip");
    task.transport_phase_.carry_us_=in.now_us;
    task.pitch_cmd_=in.camera_pitch_cdeg=-3000;
    in.heading_rad=0;
    bool opened=false,reversed=false,verified=false;
    float release_position=0;
    for (int i=0;i<1800;++i) {
        const auto before=plant.out.state;
        auto& z=in.zone_estimate;
        z.valid=true;z.source=ZoneEstimate::Source::MULTI_POINT;
        z.zone_label="blue_safe_zone";z.geometry_id="full-trip";
        z.frame_id++;z.timestamp_us=z.observed_us=in.now_us+40000;
        z.inlier_ids={0,1,2};z.residual_m=z.position_sigma_m=z.yaw_sigma_rad=.001f;
        z.origin_body_m={-task.t_.supply_half_x_m,.21f-plant.position};
        in.zone_valid=in.zone_own=in.zone_identity_verified=true;
        in.subzone_view_us=in.now_us+40000;
        in.supply_view={true,true,task.halfX(),0};
        in.supply_view.entrance_valid=true;
        in.supply_view.entrance_origin_body_m=z.origin_body_m;
        in.supply_view.entrance_yaw_rad=z.yaw_body_rad;
        in.zone_sight_valid=true;in.zone_sight_sample_us=in.now_us+40000;
        in.zone_sight_box={136,430,1124,260};
        in.zone_sight_image_size={1280,720};
        in.transport_floor={};
        in.transport_floor.sample_us=in.now_us+40000;
        in.transport_floor.valid=in.transport_floor.region_valid=true;
        in.transport_floor.inside=plant.position>.025f;
        if (inspected) {
            in.transport_checks_enabled = true;
            in.pixel_grasp_near_view = task.desiredPitch() == task.t_.near_pitch_cdeg;
            in.capture_scene_valid = true;
            in.capture_scene_us = in.now_us + 40000;
            in.pixel_grasp_view = task.desiredPitch() == task.t_.track_pitch_cdeg ||
                                  in.pixel_grasp_near_view;
            in.transport_review_clear = task.desiredPitch() == kTransportReviewPitchCdeg;
        }
        if(opened)in.gripper_feedback_open=1;
        if(before==PushState::VERIFY_DELIVERY) {
            // Completed IMU retreat hands off directly to search without image acceptance.
            in.delivery_observed=false;in.delivery_box_count=0;
            in.delivery_box_sample_us=0;
            verified=true;
        }
        const auto out=plant.tick();
        if(out.state==PushState::SAFE_STOP) std::cerr<<"transport lifecycle failed: "<<out.reason<<std::endl;
        assert(out.state!=PushState::SAFE_STOP);
        if(out.motion.gripper_offset==0 && !opened) {
            assert(before==PushState::RAISE_RELEASE);
            opened=true;release_position=plant.position;
            assert(plant.position>.025f && in.camera_pitch_cdeg==0);
        }
        if(out.motion.vx_mps<0) {
            assert(opened && out.state==PushState::BACK_OUT && out.motion.wz_rps==0);
            reversed=true;
        }
        assert(out.state!=PushState::TURN_AROUND && out.state!=PushState::TURN_SCAN);
        if(out.state==PushState::SCAN) {
            assert(opened && reversed && verified);
            assert(plant.position-release_position<=-.145f);
            assert(task.total_==1 && out.motion.vx_mps==0 && out.motion.wz_rps==0);
            std::cout<<"PASS complete 20cm -> IMU normal -> UI120 half contour -> open ack -> IMU15cm -> search"<<std::endl;
            return;
        }
    }
    assert(false && "complete transport lifecycle did not return to search");
}

void destinationIsLockedWhenCarryStarts() {
    for (int kind : {0, 1, 2}) {
        Rig rig(0);
        rig.task.clearTrip();
        rig.task.first_ = true;
        auto& inventory = rig.task.trip_context_.trip_;
        inventory = {};
        if (kind == 0) inventory.ordinary = 1;
        if (kind == 1) inventory.core = 1;
        if (kind == 2) inventory.injured = 1;
        rig.task.enter(PushState::CARRY, rig.input.now_us, "destination_test");
        const auto& transport = rig.task.transport_phase_;
        assert(transport.half_locked_);
        assert(transport.half_injured_ == (kind == 2));
        assert(transport.half_x_ == (kind == 2 ? rig.tuning.injured_half_x_m
                                              : rig.tuning.supply_half_x_m));
        // Selection happens before any safe-zone observation or navigation plan.
        assert(!rig.input.zone_valid && !rig.input.drop_plan_valid);
        assert(!transport.placing_ && transport.action == TransportAction::Carry);
        const auto event = rig.task.takeDropZoneEvent();
        assert(event.find(kind == 2 ? "half=injured" : "half=supply") != std::string::npos);
        assert(event.find("source=carry_start") != std::string::npos);
        rig.task.lockDropHalf("later_observation");
        assert(rig.task.takeDropZoneEvent().empty());
    }
}

} // namespace


// 11:47 trace: the selected half moved from +4% to -30% while geometry
// and box control alternated. A visible off-centre half must retain priority.
void loggedApproachKeepsSelectedHalfAsSteeringTarget() {
    using Recovery=TransportPhaseData::JawSequence::Recovery;
    for (bool injured : {false,true}) for (float side : {-1.f,1.f}) {
        JawRig p; p.gap=.8f; p.yaw=0;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true; d.half_injured_=injured; d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_;
        p.supply_visible=!injured; p.injured_visible=injured;
        p.image_error=-.18f*side; p.bearing=.5f*side;
        // Isolate half priority on a feasible curve; large inconsistent offsets
        // now stop for the separate stationary path review.
        p.metric_yaw_bias=.12f*side; p.edge_override=-.045f*side;
        for (int i=0;i<12;++i) {
            // Fresh image remains available while metric projection drops out.
            p.pose=i%2==0;
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            assert(p.pose ? out.motion.vx_mps>0 : out.motion.vx_mps==0);
            assert(out.motion.wz_rps*side>0 && "approach must steer toward its selected half");
            assert(d.jaw_sequence_.recovery==Recovery::Idle);
        }
        p.view_evidence=false; p.pose=false; p.bearing=-.5f*side;
        const auto out=p.step();
        assert(out.reason=="carry_entry_route_hold" || out.reason=="carry_selected_half_wait");
        assert(out.motion.wz_rps*side>=0 &&
               "a missing half must not switch to an opposing box target");
    }
    // A single projected-normal jump after convergence cannot take over steering.
    for (float side : {-1.f,1.f}) {
        JawRig p; p.gap=.8f; p.yaw=0; p.image_error=-.18f*side;
        for(int i=0;i<4;++i) p.step();
        p.metric_yaw_bias=.6f*side;
        p.edge_override=-.045f*side;
        const auto out=p.step();
        assert(out.reason=="carry_approach_selected_half");
        assert(out.motion.wz_rps*side>0);
    }
    for (int invalid : {0,1,2}) {
        JawRig p; p.yaw=0; p.gap=.8f; p.image_error=.08f;
        for(int i=0;i<4;++i) p.step();
        p.pose=false; p.view_evidence=false; p.bearing=.5f;
        p.enemy=invalid==1; p.stale_sight=invalid==2;
        for(int i=1;i<=16;++i) {
            const auto out=p.step();
            // Confirmed enemy evidence still revokes the route immediately;
            // otherwise the accepted entrance expires after 500 ms.
            if(i<(invalid==1 ? 5 : 13)) {
                assert(out.reason=="carry_entry_route_hold" || out.reason=="carry_selected_half_wait");
                if(invalid==1 && i>=3) assert(out.motion.vx_mps==0);
            } else {
                assert(out.reason.rfind("carry_half_reacquire_",0)==0 && out.motion.vx_mps==0);
            }
        }
    }
    std::cout<<"PASS logged approach keeps selected-half steering through metric and image dropouts\n";
}

void intermittentHalfApproachConvergesThroughSender() {
    for(bool injured : {false,true}) for(float side : {-1.f,1.f}) {
        JawRig p; p.gap=.8f; p.yaw=.12f*side;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true; d.half_injured_=injured; d.half_x_=injured ? .15f : -.15f;
        p.supply_visible=!injured; p.injured_visible=injured;
        p.robot_x=d.half_x_+.08f*side;
        p.rig.task.t_.carry_budget_us=90000000;
        MotionVelocityRamp ramp{MotionLinkConfig{}};
        MotionCommand sent;
        ramp.accept(sent,p.rig.input.now_us);
        bool advanced=false;
        for(int i=0;i<2000;++i) {
            p.robot_x-=std::sin(p.yaw)*sent.vx_mps*kDt;
            p.gap-=std::cos(p.yaw)*sent.vx_mps*kDt;
            p.out.motion=sent;
            const float next_yaw=wrapAngle(p.yaw+sent.wz_rps*kDt);
            const float dx=d.half_x_-p.robot_x,depth=p.gap+.15f;
            p.image_error=.625f*(std::cos(next_yaw)*dx+std::sin(next_yaw)*depth)/
                (-std::sin(next_yaw)*dx+std::cos(next_yaw)*depth);
            const bool approaching=d.jaw_sequence_.stage==TransportPhaseData::JawSequence::Stage::Approach;
            p.pose=!approaching || i%7!=3;
            p.view_evidence=!approaching || i%19!=8;
            p.bearing=-.5f*side; // opposing box direction must not steal a short dropout
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            if(p.rig.task.desiredPitch()==0) {
                assert(std::abs(p.yaw)<=std::atan(.60f)+.00001f && std::abs(p.image_error)<.10f);
                advanced=true;break;
            }
            sent=ramp.next(out.motion,p.rig.input.now_us,out.immediate_motion_stop);
            ramp.accept(sent,p.rig.input.now_us);
        }
        if (!advanced) std::cerr << "intermittent approach failed injured=" << injured
            << " side=" << side << " gap=" << p.gap << " yaw=" << p.yaw
            << " x=" << p.robot_x << " image=" << p.image_error
            << " reason=" << p.out.reason << std::endl;
        assert(advanced && "intermittent approach must reach centred, normal-aligned entry");
    }
    std::cout<<"PASS intermittent selected-half approach converges with the real sender ramp\n";
}


void entryDropoutWaitsForStationaryObservation() {
    using Recovery=TransportPhaseData::JawSequence::Recovery;
    for(bool returns : {false,true}) {
        JawRig p;p.yaw=0;p.gap=.2f;
        assert(p.step().reason=="carry_pre_entry_stop");
        p.pose=p.view_evidence=false;p.bearing=.24f;
        p.force_sent=true;p.forced_vx=.01f;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        for(int i=0;i<12;++i) {
            const auto out=p.step();
            assert(q.recovery==Recovery::Idle);
            assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        }
        p.forced_vx=0;
        const auto out=p.waitForHalfSearch();
        assert(out.motion.wz_rps>0 && q.recovery==Recovery::Idle);
        if(returns) {
            p.pose=p.view_evidence=true;p.edge_override=0;p.image_error=0;
            bool aligned=false;
            for(int i=0;i<40;++i) {
                const auto next=p.step();assert(next.motion.vx_mps==0 && q.recovery==Recovery::Idle);
                if(next.reason=="carry_pre_entry_aligned") {aligned=true;break;}
            }
            assert(aligned);
        } else {
            for(int i=0;i<20;++i) {
                const auto next=p.step();assert(next.motion.vx_mps==0 && q.recovery==Recovery::Idle);
            }
        }
    }
    std::cout<<"PASS entry dropout brakes sent translation, searches in place and confirms fresh recovery\n";
}

// 12:21 trace: fresh half errors -5.25%, -8.83%, -10.90% were
// still driven at -0.2 rad/s by a held target during metric-normal dropout.
void freshHalfOverridesHeldApproachHeading() {
    for (bool injured : {false,true}) for (float side : {-1.f,1.f})
        for (float error : {0.f,.0273186f,.0525365f}) {
        JawRig p; p.gap=.5f; p.yaw=0;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true; d.half_injured_=injured; d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_;
        p.supply_visible=!injured; p.injured_visible=injured;
        p.image_error=-.10f*side; p.metric_yaw_bias=.15f*side; p.edge_override=-.04f*side;
        for(int i=0;i<5;++i) p.step();
        assert(d.jaw_sequence_.selected_half_seen_us);
        p.image_error=error*side; p.edge_override=0;
        p.metric_yaw_bias=.157235f*side;
        const auto out=p.step();
        assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
        assert(p.image_error*out.motion.wz_rps<=0 &&
               "fresh selected half must override a held turn that now moves away");
        if(error==0) assert(out.motion.wz_rps==0);
    }
    std::cout<<"PASS fresh selected-half image overrides stale approach heading after crossing centre\n";
}


// 12:29 trace: ordinary image offsets and tiny heading jitter repeatedly
// interrupted retreat and sent an already visible half back through entry.
void efficientEntryAcceptsSmallOffsets() {
    for(bool injured : {false,true}) for(float side : {-1.f,1.f}) {
        JawRig p;p.yaw=0;p.gap=.2f;p.image_error=.035f*side;
        p.edge_override=.025f*side;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_;
        bool advanced=false;
        for(int i=0;i<80;++i) {
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            assert(out.motion.vx_mps>=0 &&
                   "small in-half offset must not trigger another retreat");
            if(out.motion.vx_mps>0) {advanced=true;break;}
        }
        assert(advanced && "small offset must reach forward entry without precision-turn loops");
    }
    std::cout<<"PASS small offsets enter without retreat or precision-turn loops\n";
}

void outOfToleranceEntryCentresWithoutForwardCorrection() {
    for(bool injured : {false,true}) for(float side : {-1.f,1.f})
        for(float error : {.10f,.10001f,.12f,.15f,.18f}) {
            JawRig p;p.yaw=0;p.gap=.30f;p.image_error=error*side;p.edge_override=0;
            auto& d=p.rig.task.transport_phase_;
            d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
            p.robot_x=d.half_x_;
            for(int i=0;i<40;++i) {
                const auto out=p.step();
                assert(out.motion.vx_mps==0 && out.motion.wz_rps*side<=0);
                assert(p.rig.task.desiredPitch()==-3000);
                assert(out.reason!="carry_entry_forward_correction");
            }
            assert(p.out.state==PushState::CARRY);
            p.image_error=.049f*side;
            bool accepted=false;
            for(int i=0;i<30;++i) {
                const auto out=p.step();
                assert(out.motion.vx_mps==0);
                if(out.reason=="carry_pre_entry_aligned") {accepted=true;break;}
            }
            assert(accepted && p.rig.task.desiredPitch()==0);
        }
    std::cout<<"PASS persistent >=10% entry centres without forward correction\n";
}

void earlyApproachScalesTurnBeforeTheStopLine() {
    for (bool injured : {false,true}) for (float side : {-1.f,1.f})
        for (bool metric : {false,true}) {
        JawRig p; p.gap=.65f; p.yaw=0; p.pose=metric;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true; d.half_injured_=injured; d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_-.20f*side;
        p.supply_visible=!injured; p.injured_visible=injured;
        p.rig.task.t_.max_wz=.4f;
        p.image_error=.20f*side;
        auto out=p.step();
        if(metric) {
            assert(out.reason=="carry_entry_path_review" && out.motion.vx_mps==0);
            for(int i=0;i<20 && !d.jaw_sequence_.reposition;++i)out=p.step();
            assert(d.jaw_sequence_.reposition);
            out=p.step();
        }
        assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
        assert(metric ? out.motion.vx_mps>0 : out.motion.vx_mps==0);
        assert(out.motion.wz_rps*side<0);
        if(!metric) assert(std::abs(out.motion.wz_rps)>.2f);
        // A measured forward target uses an arc; image-only direction still
        // cannot authorize translation across an unmeasured parking line.
        assert(std::abs(out.motion.wz_rps)<=.4f);
        // A nearly aligned approach keeps speed and a proportional small turn.
        JawRig aligned; aligned.gap=.65f; aligned.yaw=0; aligned.pose=metric;
        aligned.robot_x=aligned.rig.task.halfX(); aligned.image_error=.002f;
        auto small=aligned.step();
        assert(metric ? small.motion.vx_mps>=out.motion.vx_mps : small.motion.vx_mps==0);
        assert(std::abs(small.motion.wz_rps)<.1f);
        // Runtime angular limits still apply to the stronger approach.
        p.rig.task.t_.max_wz=.12f;
        const auto limited=p.step();
        assert(std::abs(limited.motion.wz_rps)<=.12f);
        p.rig.input.path_safe=false;
        const auto blocked=p.step();
        assert(blocked.motion.vx_mps==0 && blocked.motion.wz_rps==0);
    }
    std::cout<<"PASS early approach turns harder and advances less for large offsets\n";
}


// 13:18 trace: losing the current three-frame normal gate switched from
// compensated centreline steering to the opposite image-only bearing.
void confirmedApproachNormalSurvivesIntermittentCaptures() {
    for (bool injured : {false,true}) for (float side : {-1.f,1.f}) {
        // Keep this normal-continuity case on a feasible curve; short large-offset
        // approaches are covered by the forward-reposition regression.
        JawRig p; p.gap=.95f;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true; d.half_injured_=injured; d.half_x_=injured ? .15f : -.15f;
        p.supply_visible=!injured; p.injured_visible=injured;
        p.robot_x=d.half_x_-.35f*side;
        const float yaw=-.55f*side;
        const float dx=d.half_x_-p.robot_x, depth=p.gap+.15f;
        p.image_error=.625f*(std::cos(yaw)*dx+std::sin(yaw)*depth)/
            (-std::sin(yaw)*dx+std::cos(yaw)*depth);
        for(int i=0;i<5;++i) {
            p.yaw=yaw; p.out.motion={}; p.step();
        }
        assert(d.jaw_sequence_.normal_valid);
        for(int i=0;i<30;++i) {
            p.yaw=yaw; p.out.motion={};
            p.capture_heading=i%7!=0;
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            assert(out.motion.wz_rps*side<0 &&
                   "a missing capture-time normal must not reverse confirmed centreline steering");
        }
    }
    std::cout<<"PASS confirmed approach normal bridges capture dropouts without opposite steering\n";
}


void loggedSteeringDropoutsConvergeWithoutRetreatLoop() {
    for(bool injured : {false,true}) for(float side : {-1.f,1.f}) {
        JawRig p; p.gap=.65f; p.yaw=-.55f*side; p.ramped_dead_zone=true;
        p.rig.task.t_.scan_wz=.4f;
        p.rig.task.t_.carry_budget_us=90000000;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true; d.half_injured_=injured; d.half_x_=injured ? .15f : -.15f;
        p.supply_visible=!injured; p.injured_visible=injured;
        p.robot_x=d.half_x_-.35f*side;

        bool advanced=false, confirmed_outside_deadline=false, reversing=false;
        int reverse_legs=0;
        for(int i=0;i<2200;++i) {
            p.robot_x-=std::sin(p.yaw)*p.wire.vx_mps*kDt;
            p.gap-=std::cos(p.yaw)*p.wire.vx_mps*kDt;
            const float rate=std::abs(p.wire.vx_mps)>.001f ||
                std::abs(p.wire.wz_rps)>.25f ? p.wire.wz_rps : 0;
            const float next_yaw=wrapAngle(p.yaw+rate*kDt);
            const float dx=d.half_x_-p.robot_x,depth=p.gap+.15f;
            p.image_error=.625f*(std::cos(next_yaw)*dx+std::sin(next_yaw)*depth)/
                (-std::sin(next_yaw)*dx+std::cos(next_yaw)*depth);
            p.capture_heading=i%7!=3;
            const auto out=p.step();
            if(out.reason=="carry_pre_entry_aligned") {
                assert(p.gap>.20f && "alignment must finish before the 20 cm deadline");
                confirmed_outside_deadline=true;
            }
            if(p.gap<=.20f) assert(confirmed_outside_deadline);
            const bool reverse=out.motion.vx_mps<0;
            if(reverse && !reversing) ++reverse_legs;
            reversing=reverse;
            assert(reverse_legs<=1 && "backoff must not become a retreat loop");
            if(reverse) {
                assert(d.jaw_sequence_.stage==TransportPhaseData::JawSequence::Stage::Backoff);
                assert(out.motion.vx_mps>=-.05f && out.motion.wz_rps==0);
            }
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            if(p.rig.task.desiredPitch()==0) {
                assert(std::abs(p.yaw)<=std::atan(.60f)+.00001f && std::abs(p.image_error)<.10f);
                advanced=true;break;
            }
        }
        if(!advanced) std::cerr<<"logged capture dropout failed injured="<<injured
            <<" side="<<side<<" gap="<<p.gap<<" yaw="<<p.yaw
            <<" x="<<p.robot_x<<" image="<<p.image_error<<" reason="<<p.out.reason<<std::endl;
        assert(advanced && confirmed_outside_deadline);
        std::cout<<"PASS logged capture dropout converges injured="<<injured
                 <<" side="<<side<<" reverse_legs="<<reverse_legs<<"\n";
    }
}


void preEntryAlignmentCompletesAndHoldsItsTarget() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for (bool injured : {false,true}) {
        JawRig p; p.gap=.35f; p.yaw=0;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true; d.half_injured_=injured; d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_; p.supply_visible=!injured; p.injured_visible=injured;
        bool aligned=false;
        for(int i=0;i<40;++i) {
            const auto out=p.step();
            assert(out.motion.vx_mps==0);
            if(out.reason=="carry_pre_entry_aligned") {
                assert(p.gap>.2f && p.rig.task.desiredPitch()==0);
                aligned=true;break;
            }
        }
        assert(aligned && d.jaw_sequence_.stage==Stage::ColourAdvance);
        assert(p.step().motion.vx_mps>0);
        // After switching, keep the confirmed heading through landmark/box loss.
        p.pose=p.view_evidence=p.sight=false; p.bearing=-.5f;
        for(int i=0;i<50;++i) {
            const auto out=p.step();
            assert(std::abs(out.motion.vx_mps-.06f)<1e-6f && out.motion.wz_rps==0);
            assert(d.jaw_sequence_.stage==Stage::ColourAdvance);
            assert(p.rig.task.desiredPitch()==0);
        }
    }
    std::cout<<"PASS one stationary alignment starts near-view push and holds its target\n";
}

void alignmentMustFinishBeforeTwentyCentimetres() {
    JawRig former; former.gap=.40f; former.yaw=0;
    assert(former.step().motion.vx_mps>0 && "40 cm is no longer a parking threshold");
    JawRig unmeasured; unmeasured.pose=false; unmeasured.yaw=0;
    unmeasured.image_error=.08f;
    assert(unmeasured.step().motion.vx_mps==0 && "unmeasured distance cannot authorize crossing entry");

    for (bool injured : {false,true}) for (float side : {-1.f,1.f}) {
        JawRig p; p.gap=.30f; p.yaw=.2f*side;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true; d.half_injured_=injured; d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_; p.supply_visible=!injured; p.injured_visible=injured;
        auto out=p.step();
        assert(out.motion.vx_mps==0 && "final alignment must begin outside the 20 cm line");
        assert(out.reason=="carry_pre_entry_stop");
    }
    // A 9.9% offset is within tolerance; a 10.1% offset still needs centring.
    for (float side : {-1.f,1.f}) for(float offset : {.099f,.101f}) {
        JawRig p; p.gap=.2f; p.yaw=.15f*side; p.image_error=offset*side;
        bool aligned=false;
        for(int i=0;i<50;++i) {
            const auto out=p.step();
            aligned |= out.reason=="carry_pre_entry_aligned";
            assert(out.motion.vx_mps<=0 || (aligned && offset<.10f));
        }
        assert(aligned==(offset<.10f));
    }
    std::cout<<"PASS alignment is required before the 20 cm line\n";
}


// 14:10 field trace: isolated own boxes alternated with the enemy zone and
// repeatedly bought another 1.5 seconds of stationary observation.
void confirmedZoneSightDoesNotRepeatEnemyHolds() {
    JawRig p; p.yaw=0; p.pose=p.view_evidence=false; p.bearing=.3f;
    p.step(); // One own-colour image is not a confirmed dropout window.
    p.sight=false;
    auto out=p.step();
    assert(out.reason=="searching_own_zone" && out.motion.vx_mps==0);
    for(int pulse=0;pulse<5;++pulse) {
        p.sight=true;p.enemy=false;p.step();
        p.enemy=true;
        for(int i=0;i<3;++i) {
            out=p.step();
            assert(out.reason=="searching_own_zone" && out.motion.wz_rps>0);
        }
    }
    p.enemy=false;
    for(int i=0;i<3;++i)p.step();
    p.sight=false;
    out=p.step();
    assert(out.reason=="carry_zone_short_hold" && out.motion.wz_rps==0);
    p.sight=true;p.enemy=true;
    for(int i=0;i<3;++i)out=p.step();
    assert(out.reason=="searching_own_zone" && out.motion.wz_rps>0);
    JawRig repeated;repeated.yaw=0;repeated.pose=repeated.view_evidence=false;
    repeated.repeat_sight_frame=true;
    for(int i=0;i<3;++i)repeated.step();
    assert(repeated.rig.task.transport_phase_.jaw_sequence_.own_zone_seen_us==0);
    repeated.sight=false;
    out=repeated.step();
    assert(out.reason=="searching_own_zone");
    JawRig enemy_duplicate;enemy_duplicate.yaw=0;
    enemy_duplicate.pose=enemy_duplicate.view_evidence=false;
    for(int i=0;i<3;++i)enemy_duplicate.step();
    enemy_duplicate.enemy=enemy_duplicate.repeat_sight_frame=true;
    for(int i=0;i<3;++i) {
        out=enemy_duplicate.step();
        assert(out.reason=="carry_zone_short_hold");
    }
    std::cout<<"PASS isolated/repeated own boxes cannot renew holds; distinct enemy frames resume search\n";
}

void selectedHalfDropoutWithoutRouteWaitsForFiveFrames() {
    for(bool injured : {false,true}) for(float side : {-1.f,1.f}) {
        JawRig p;p.yaw=0;p.pose=false;p.image_error=-.3f*side;
        p.capture_yaw_offset=.06f;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
        p.supply_visible=!injured;p.injured_visible=injured;
        p.step();
        p.view_evidence=false;p.bearing=-.5f*side;
        for(int i=1;i<=6;++i) {
            const auto out=p.step();
            assert(out.motion.vx_mps==0);
            if(i<5) assert(out.reason=="carry_selected_half_wait" && out.motion.wz_rps==0);
            else assert(out.reason.rfind("carry_half_reacquire_",0)==0);
        }
    }
    for(bool missing_heading : {false,true}) {
        JawRig p;p.pose=false;p.yaw=0;p.image_error=-.3f;p.step();
        p.view_evidence=false;
        p.heading=!missing_heading;
        p.rig.input.directional_clearance_valid=true;p.rig.input.turn_safe=missing_heading;
        auto out=p.step();
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
    }
    JawRig missing_capture;missing_capture.pose=false;missing_capture.yaw=0;
    missing_capture.image_error=-.3f;missing_capture.capture_heading=false;missing_capture.step();
    missing_capture.view_evidence=false;
    auto missing=missing_capture.step();
    assert(missing.motion.vx_mps==0 && missing.motion.wz_rps==0);
    // No established route means forward feedback alone cannot authorize travel.
    JawRig moving;moving.yaw=0;moving.image_error=-.3f;moving.step();
    moving.pose=moving.view_evidence=false;
    moving.force_sent=true;moving.forced_vx=.03f;
    auto out=moving.step();
    assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
    moving.forced_vx=0;
    out=moving.step();
    assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
    std::cout<<"PASS missing half without a route cannot reuse an image-only turn\n";
}

void imageTurnContinuesUntilFinalObservation() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(float side : {-1.f,1.f}) {
        JawRig p;p.pose=false;p.yaw=0;p.image_error=-.1f*side;p.edge_override=-.35f*side;
        p.capture_yaw_offset=.04f*side;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        q.stage=Stage::Align;q.image_turn_heading=.1f*side;q.image_turn_wz=.4f*side;
        for(int i=0;i<16;++i) {
            const auto out=p.step();
            assert(out.motion.vx_mps==0 && out.motion.wz_rps*side>0);
            assert(q.stage==Stage::Align);
            const float expected=wrapAngle(p.rig.input.zone_observed_heading_rad+.1f*side);
            assert(std::abs(wrapAngle(q.image_turn_heading-expected))<.001f);
        }
        // A clipped half cannot interrupt a still-fresh measured turn endpoint.
        p.view_evidence=false;
        auto out=p.step();
        assert(out.motion.vx_mps==0 && out.motion.wz_rps*side>0);
        p.view_evidence=true;p.image_error=p.edge_override=0;
        bool braked=false;
        uint64_t brake_us=0;
        for(int i=0;i<30;++i) {
            out=p.step();
            if(!braked && out.motion.wz_rps==0) {
                braked=true;brake_us=p.rig.input.now_us;
            }
            if(braked && p.rig.input.now_us-brake_us<250000)
                assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        }
        assert(braked && q.stage==Stage::ColourAdvance);
    }
    std::cout<<"PASS image turns continue across fresh corrections; final stationary gate remains\n";
}

void unalignedEntryCentresWithoutTranslation() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for (bool injured : {false,true}) for (float side : {-1.f,1.f})
        for (auto stage : {Stage::EntryStop,Stage::AlignStop})
        for (float gap : {.2f,.35f}) {
            JawRig p;p.gap=gap;p.yaw=0;p.image_error=.15f*side;p.edge_override=0;
            auto& d=p.rig.task.transport_phase_;
            d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
            p.robot_x=d.half_x_;d.jaw_sequence_.stage=stage;
            for (int i=0;i<35;++i) {
                const auto out=p.step();
                assert(out.motion.gripper_offset==20);
                assert(out.motion.vx_mps==0 && out.motion.wz_rps*side<=0);
                assert(p.rig.task.desiredPitch()==-3000);
            }
            assert(p.out.state==PushState::CARRY);
            p.image_error=.049f*side;
            bool recovered=false;
            for(int i=0;i<30;++i) {
                const auto out=p.step();
                assert(out.motion.vx_mps==0);
                if(out.reason=="carry_pre_entry_aligned") {recovered=true;break;}
            }
            assert(recovered);
            // A transient bad view before the stationary gate is not a failure.
            JawRig transient;transient.gap=gap;transient.yaw=0;
            transient.image_error=.15f*side;transient.edge_override=0;
            transient.rig.task.transport_phase_.jaw_sequence_.stage=stage;
            for(int i=0;i<4;++i) assert(transient.step().state==PushState::CARRY);
            transient.image_error=.049f*side;
            bool confirmed=false;
            for(int i=0;i<40;++i) {
                const auto out=transient.step();
                confirmed |= out.reason=="carry_pre_entry_aligned";
            }
            assert(confirmed && transient.rig.task.desiredPitch()==0);
        }
    std::cout<<"PASS unaligned entry centres without translation; fresh confirmation permits entry\n";
}

// A conflict outside the relaxed centre and edge bounds must terminate
// without authorizing entry or losing cargo on either half or mirrored side.
void outOfToleranceStationaryConflictTerminates() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(bool injured : {false,true}) for(float side : {-1.f,1.f})
        for(auto stage : {Stage::EntryStop,Stage::AlignStop}) {
            JawRig p; p.yaw=0; p.gap=.35f;
            p.image_error=.15f*side; p.edge_override=-.35f*side;
            p.metric_yaw_bias=.337376f*side;
            auto& d=p.rig.task.transport_phase_;
            d.half_locked_=true;d.half_injured_=injured;
            d.half_x_=injured ? .15f : -.15f;
            p.robot_x=d.half_x_;p.supply_visible=!injured;p.injured_visible=injured;
            d.jaw_sequence_.stage=stage;
            bool failed=false;
            for(int i=0;i<50;++i) {
                const auto out=p.step();
                assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
                assert(out.motion.gripper_offset==20 && p.rig.task.desiredPitch()==-3000);
                if(out.state==PushState::SAFE_STOP) {
                    assert(out.reason=="carry_entry_reposition_required");
                    assert(out.immediate_motion_stop);
                    failed=true;break;
                }
            }
            assert(failed && "stationary alignment conflict must not repeat zero-speed cycles");
            p.image_error=p.edge_override=p.metric_yaw_bias=0;
            const auto out=p.step();
            assert(out.state==PushState::SAFE_STOP && out.motion.vx_mps==0 && out.motion.wz_rps==0);
        }
    std::cout<<"PASS out-of-tolerance stationary conflict terminates with cargo held\n";
}

void jointApproachReachesParkingPoseThroughSender() {
    for(bool injured : {false,true}) for(float side : {-1.f,1.f})
        for(float response : {1.f,6.f}) for(float speed : {.05f,.2f}) {
        JawRig p;p.gap=.8f;p.yaw=.2f*side;p.ramped_dead_zone=true;
        p.rig.task.t_.carry_budget_us=90000000; // actual controlled-field entry budget
        p.angular_response=response;p.rig.task.t_.carry_speed=speed;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_+.08f*side;
        p.supply_visible=!injured;p.injured_visible=injured;
        bool parked=false,confirmed=false;
        for(int i=0;i<2500;++i) {
            p.robot_x-=std::sin(p.yaw)*p.wire.vx_mps*kDt;
            p.gap-=std::cos(p.yaw)*p.wire.vx_mps*kDt;
            const float rate=std::abs(p.wire.vx_mps)>.001f ||
                std::abs(p.wire.wz_rps)>.25f ? p.wire.wz_rps : 0;
            const float next_yaw=wrapAngle(p.yaw+response*rate*kDt);
            const float dx=d.half_x_-p.robot_x,depth=p.gap+.15f;
            p.image_error=.625f*(std::cos(next_yaw)*dx+std::sin(next_yaw)*depth)/
                (-std::sin(next_yaw)*dx+std::cos(next_yaw)*depth);
            const auto out=p.step();
            if(out.state!=PushState::CARRY) {
                std::cerr<<"joint parking failed gap="<<p.gap<<" x="<<p.robot_x
                    <<" yaw="<<p.yaw<<" reason="<<out.reason<<std::endl;
            }
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            assert(out.motion.vx_mps>=0);
            if(out.reason=="carry_pre_entry_stop") {
                parked=true;
                assert(std::abs(d.half_x_-p.robot_x)<.02f);
                assert(std::abs(p.yaw)<.07f && "position and heading must converge before parking");
            }
            if(out.reason=="carry_pre_entry_aligned") {
                assert(parked && p.gap>.20f && std::abs(p.image_error)<.10f);
                confirmed=true;break;
            }
        }
        assert(confirmed && "joint approach must reach stationary confirmation");
    }
    std::cout<<"PASS joint approach reaches selected parking pose through sender\n";
}


void jointApproachPreservesClearanceAndStationaryEvidence() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(bool confirmed_normal : {false,true}) {
        JawRig p;p.yaw=0;p.image_error=.08f;p.gap=.8f;
        if(confirmed_normal) for(int i=0;i<4;++i)p.step();
        p.rig.input.directional_clearance_valid=true;
        p.rig.input.arc_safe=false;p.rig.input.turn_safe=true;
        auto out=p.step();
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0 && out.immediate_motion_stop);
        assert(out.reason=="carry_approach_arc_blocked");
    }
    // One displaced capture inside the final window cannot fault the trip;
    // three distinct centred captures are still required after it.
    JawRig p;p.yaw=0;p.gap=.35f;
    auto& q=p.rig.task.transport_phase_.jaw_sequence_;q.stage=Stage::AlignStop;
    for(int i=0;i<30 && q.visual_frames==0;++i)p.step();
    assert(q.visual_frames==1);
    p.image_error=.15f;p.step();p.image_error=0;
    bool confirmed=false;
    for(int i=0;i<30;++i) {
        const auto out=p.step();
        assert(out.state==PushState::CARRY);
        if(out.reason=="carry_pre_entry_aligned") {confirmed=true;break;}
    }
    assert(confirmed);
    std::cout<<"PASS joint approach clearance and transient stationary evidence\n";
}


void selectedHalfDropoutCannotParkFromWholeZone() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    JawRig unseen;unseen.gap=.2f;unseen.yaw=0;unseen.view_evidence=false;
    const auto stopped=unseen.step();
    assert(stopped.motion.vx_mps==0 && stopped.motion.wz_rps==0);
    assert(stopped.reason=="carry_selected_half_range_wait");
    assert(unseen.rig.task.transport_phase_.jaw_sequence_.stage==Stage::Approach);
    for(bool injured : {false,true}) {
        JawRig p;p.gap=.65f;p.yaw=0;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_;p.supply_visible=!injured;p.injured_visible=injured;
        for(int i=0;i<4;++i)p.step();
        // The selected half was at 65 cm. Its dropout exposes an unrelated
        // whole-zone 25 cm fix; that fix must not latch the parking stage.
        p.view_evidence=false;p.gap=.25f;
        const auto out=p.step();
        assert(out.motion.vx_mps>0 && out.motion.wz_rps==0);
        assert(d.jaw_sequence_.stage==Stage::Approach);
        assert(out.reason=="carry_entry_route_hold");
        p.view_evidence=true;p.gap=.65f;
        assert(p.step().motion.vx_mps>0);
    }
    std::cout<<"PASS whole-zone fallback cannot park a lost selected half\n";
}

void stationaryOffsetWithSpaceResumesJointApproach() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(bool injured : {false,true}) for(float side : {-1.f,1.f}) {
        JawRig p;p.gap=.55f;p.yaw=0;
        p.image_error=.15f*side;p.edge_override=0;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_-.08f*side;
        p.supply_visible=!injured;p.injured_visible=injured;
        auto& q=d.jaw_sequence_;q.stage=Stage::AlignStop;
        bool replanned=false;
        for(int i=0;i<40;++i) {
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
            if(out.reason=="carry_entry_resume_approach") {
                assert(q.stage==Stage::Approach && p.rig.input.now_us>=1360000);
                replanned=true;break;
            }
        }
        assert(replanned && "three stable offset captures with space must replan before turning");
        p.edge_override=std::numeric_limits<float>::quiet_NaN();
        const auto out=p.step();
        assert(out.state==PushState::CARRY && out.motion.vx_mps>=0);
        assert(out.motion.vx_mps>0 || out.motion.wz_rps!=0);
        assert(p.rig.task.desiredPitch()==-3000);
        p.rig.input.directional_clearance_valid=true;
        p.rig.input.path_safe=p.rig.input.arc_safe=p.rig.input.turn_safe=false;
        const auto blocked=p.step();
        assert(blocked.motion.vx_mps==0 && blocked.motion.wz_rps==0);
    }
    std::cout<<"PASS stable offset with measured space resumes the joint approach\n";
}


void reapproachNeedsStableRangeAndRoom() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(int scenario=0;scenario<3;++scenario) {
        JawRig p;p.yaw=0;p.image_error=.15f;p.edge_override=0;
        p.robot_x=p.rig.task.halfX()-.08f;
        p.rig.task.transport_phase_.jaw_sequence_.stage=Stage::AlignStop;
        if(scenario==0)p.pose=false;
        for(int i=0;i<40 && p.out.state!=PushState::SAFE_STOP;++i) {
            p.gap=scenario==2 ? .38f : (scenario==1 && i%2 ? .54f : .43f);
            const auto out=p.step();
            assert(out.motion.vx_mps==0);
            if(scenario<3)assert(out.motion.wz_rps<=0);
            assert(out.reason!="carry_entry_resume_approach");
        }
        // Missing range, >10 cm jitter, and insufficient forward room cannot
        // authorize translation. Jitter waits instead of declaring unreachable.
        assert(p.out.state==PushState::CARRY);
    }
    std::cout<<"PASS reapproach waits for stable range and sufficient forward room\n";
}

void resumedApproachConvergesThroughSender() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(float side : {-1.f,1.f}) for(float response : {1.f,6.f}) {
        JawRig p;p.gap=.65f;p.yaw=.05f*side;p.ramped_dead_zone=true;
        p.angular_response=response;
        p.rig.task.t_.scan_wz=.4f;
        p.rig.task.t_.carry_budget_us=120000000;
        const float target=p.rig.task.halfX();
        p.robot_x=target-.16f*side;
        p.rig.task.transport_phase_.jaw_sequence_.stage=Stage::AlignStop;
        bool resumed=false,confirmed=false;
        for(int i=0;i<2800;++i) {
            p.robot_x-=std::sin(p.yaw)*p.wire.vx_mps*kDt;
            p.gap-=std::cos(p.yaw)*p.wire.vx_mps*kDt;
            const float rate=std::abs(p.wire.vx_mps)>.001f ||
                std::abs(p.wire.wz_rps)>.25f ? p.wire.wz_rps : 0;
            const float next=wrapAngle(p.yaw+response*rate*kDt);
            // Simulate 80 ms of capture-to-control yaw latency.
            p.capture_yaw_offset=response*rate*.08f;
            const float captured=wrapAngle(next-p.capture_yaw_offset);
            const float dx=target-p.robot_x,depth=p.gap+.15f;
            p.image_error=.625f*(std::cos(captured)*dx+std::sin(captured)*depth)/
                (-std::sin(captured)*dx+std::cos(captured)*depth);
            const auto out=p.step();
            resumed |= out.reason=="carry_entry_resume_approach";
            if(out.state!=PushState::CARRY)
                std::cerr<<"resume failed gap="<<p.gap<<" x="<<p.robot_x
                    <<" yaw="<<p.yaw<<" image="<<p.image_error<<" reason="<<out.reason<<std::endl;
            assert(out.state==PushState::CARRY && out.motion.vx_mps>=0);
            if(out.reason=="carry_pre_entry_aligned") {confirmed=true;break;}
        }
        if(!confirmed)
            std::cerr<<"resume incomplete gap="<<p.gap<<" x="<<p.robot_x
                <<" yaw="<<p.yaw<<" reason="<<p.out.reason<<std::endl;
        assert(resumed && confirmed);
    }
    std::cout<<"PASS resumed joint approach reaches stationary acceptance through sender\n";
}

void imageTurnBrakesBeforeNextImuStep() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(float side : {-1.f,1.f}) {
        JawRig p;p.pose=p.view_evidence=false;p.yaw=0;
        p.rig.task.t_.scan_wz=.4f;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        q.stage=Stage::Align;q.image_turn_heading=.05f*side;q.image_turn_wz=.4f*side;
        q.image_turn_sample_us=p.rig.input.now_us;
        p.out.motion.wz_rps=.4f*side;
        const auto first=p.step();
        assert(first.motion.wz_rps*side>0);
        const auto second=p.step();
        assert(second.motion.vx_mps==0 && second.motion.wz_rps==0);
        assert(second.immediate_motion_stop && q.stage==Stage::AlignStop);
        assert(second.reason=="carry_entry_align_brake");
    }
    std::cout<<"PASS image turn brakes before the next measured IMU increment\n";
}


// The 16:03 trace's 8.24% in-half offset now confirms without a turn.
void loggedInHalfEntryAcceptsWithoutTurning() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(bool injured : {false,true}) for(float side : {-1.f,1.f})
        for(bool biased_normal : {false,true}) {
        JawRig p;p.gap=.3237f;p.yaw=0;
        p.edge_override=-.0225989f*side;p.image_error=.082391f*side;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_;p.supply_visible=!injured;p.injured_visible=injured;
        auto& q=d.jaw_sequence_;q.stage=Stage::AlignStop;
        if(biased_normal) {
            q.normal_valid=true;q.normal_heading=.3f*side;
            q.normal_seen_us=p.rig.input.now_us;q.normal_geometry_id="test-entry";
        }
        bool turned=false,accepted=false;uint64_t brake_us=0;
        for(int i=0;i<150;++i) {
            const float next_yaw=p.yaw+p.out.motion.wz_rps*.04f;
            p.image_error=.082391f*side+.625f*std::tan(next_yaw);
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            assert(out.motion.vx_mps==0);
            if(out.motion.wz_rps!=0) {
                assert(out.motion.wz_rps*side<0);
                turned=true;brake_us=0;
            } else if(turned && !brake_us) brake_us=p.rig.input.now_us;
            if(out.reason=="carry_pre_entry_aligned") {
                assert(!turned && std::abs(p.image_error)<.10f);
                assert(p.rig.input.now_us>=1360000);
                assert(q.stage==Stage::ColourAdvance);
                accepted=true;break;
            }
        }
        assert(!turned && accepted);
    }
    std::cout<<"PASS 16:03 in-half offset confirms without another precision turn\n";
}

void entryEdgeThirtyPercentBoundary() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(float gap : {.2f,.35f}) for(float side : {-1.f,1.f})
        for(float slope : {.05f,.07f,.26062f,.29999f,.30f,.30001f}) {
        JawRig p;p.gap=gap;p.yaw=0;p.image_error=0;p.edge_override=side*slope;
        p.pose=false;
        p.rig.task.transport_phase_.jaw_sequence_.stage=Stage::AlignStop;
        bool accepted=false,turned=false;
        for(int i=0;i<30;++i) {
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.vx_mps==0);
            turned|=out.motion.wz_rps!=0;
            if(out.reason=="carry_pre_entry_aligned") {accepted=true;break;}
            if(turned)break;
        }
        assert(accepted==(slope<=.30f));
        assert(turned==(slope>.30f));
    }
    std::cout<<"PASS entry slope accepts inclusive 0.30 at both distances\n";
}

void centreTurnBrakesOnBadSlopeOrBlockedClearance() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(int fault=0;fault<4;++fault) {
        JawRig p;p.pose=false;p.yaw=0;p.image_error=.15f;p.edge_override=-.0226f;
        p.rig.task.transport_phase_.jaw_sequence_.stage=Stage::AlignStop;
        bool turned=false;
        for(int i=0;i<30;++i) {
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.vx_mps==0);
            if(out.motion.wz_rps<0) {turned=true;break;}
        }
        assert(turned);
        if(fault==0)p.edge_override=-.30001f;
        if(fault==1) {
            p.rig.input.directional_clearance_valid=true;p.rig.input.turn_safe=false;
        }
        if(fault==2) {p.image_error=-.06f;}
        if(fault==3) {p.view_evidence=false;p.repeat_pose_frame=true;}
        for(int i=0;i<20;++i) {
            const auto out=p.step();
            assert(out.motion.vx_mps==0);
            if(fault!=3 || i>=15)assert(out.motion.wz_rps==0);
            if(fault==0 && out.state==PushState::SAFE_STOP)break;
            // Reversal is evaluated only after the full stationary window.
            if(fault==2 && i>=6)break;
        }
    }
    std::cout<<"PASS centring brakes for excess slope, blocked turn, crossing and expired views\n";
}


void briefMissingHalvesKeepPlannedRouteBeforeSearching() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(bool injured : {false,true}) for(float side : {-1.f,1.f}) {
        JawRig p;p.gap=.8f;p.yaw=0;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_+.08f*side;
        p.supply_visible=!injured;p.injured_visible=injured;
        // Image-only centring asks for the opposite turn to this parking arc.
        p.image_error=.12f*side;
        for(int i=0;i<6;++i)p.step();
        assert(p.out.reason=="carry_entry_020m_approach" && p.out.motion.vx_mps>0);
        assert(p.out.motion.wz_rps*side>0);
        p.view_evidence=false;p.pose=false;p.bearing=-.5f*side;
        const auto first=p.step();
        assert(first.reason=="carry_entry_route_hold" && first.motion.vx_mps>0);
        assert(first.motion.wz_rps*side>0 && !first.immediate_motion_stop);
        // Repeated control ticks cannot renew the accepted entrance timestamp.
        for(int i=0;i<8;++i) {
            p.rig.input.now_us+=1000;
            const auto duplicate=p.rig.task.update(p.rig.input);
            assert(duplicate.reason=="carry_entry_route_hold" && duplicate.motion.vx_mps>0);
        }
        bool curvature_updated=false;
        for(int missing=2;missing<=12;++missing) {
            const auto out=p.step();
            assert(out.reason=="carry_entry_route_hold" && out.motion.vx_mps>0);
            assert(out.motion.wz_rps*side>0 && !out.immediate_motion_stop);
            assert(d.jaw_sequence_.stage==Stage::Approach);
            curvature_updated |= std::abs(out.motion.wz_rps-first.motion.wz_rps)>1e-5f;
        }
        assert(curvature_updated && "route tracking must use current heading instead of replaying a command");
        const auto search=p.step();
        assert(search.reason=="carry_half_reacquire_brake" && search.motion.vx_mps==0);
        assert(search.motion.wz_rps==0 && search.immediate_motion_stop);
        p.view_evidence=p.pose=true;
        bool reacquired=false;
        for(int i=0;i<20;++i) {
            const auto out=p.step();
            assert(out.motion.vx_mps==0);
            if(out.reason=="carry_half_reacquired") {reacquired=true;break;}
        }
        assert(reacquired);
        p.step(); // A fresh measured route is established after the stopped handoff.
        assert(p.out.motion.vx_mps>0);
        p.view_evidence=p.pose=false;
        const auto recovered=p.step();
        assert(recovered.reason=="carry_entry_route_hold" && recovered.motion.vx_mps>0);
        p.rig.input.directional_clearance_valid=true;
        p.rig.input.path_safe=p.rig.input.arc_safe=p.rig.input.turn_safe=false;
        const auto blocked=p.step();
        assert(blocked.motion.vx_mps==0 && blocked.motion.wz_rps==0);
    }
    std::cout<<"PASS missing half keeps its planned route for 500ms, then brakes for directed search; fresh half confirms; duplicates and blockers respected\n";
}

// 17:13:58 trace: this observed in-half offset still delivers into the safe zone.
void loggedDeliverableEntryStartsNearViewPush() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(bool injured : {false,true}) for(float side : {-1.f,1.f})
        for(bool whole_pose : {false,true}) {
        JawRig p;p.gap=.3152756f;p.yaw=-.144501f*side;
        p.image_error=.0617388f*side;p.edge_override=-.0574713f*side;
        p.pose=whole_pose;p.half_entrance=true;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_-.096866f*side;
        p.supply_visible=!injured;p.injured_visible=injured;
        auto& q=d.jaw_sequence_;q.stage=Stage::AlignStop;
        bool accepted=false;
        for(int i=0;i<25;++i) {
            const auto out=p.step();
            assert(out.state==PushState::CARRY);
            assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
            if(out.reason=="carry_pre_entry_aligned") {
                assert(p.rig.input.now_us>=1400000);
                assert(q.stage==Stage::ColourAdvance && p.rig.task.desiredPitch()==0);
                accepted=true;break;
            }
        }
        assert(accepted && "17:13 deliverable entry must pass stationary confirmation");
        const auto out=p.step();
        assert(out.reason=="carry_ui120_region_advance" && out.motion.vx_mps>0);
        assert(out.motion.gripper_offset==20);
    }
    std::cout<<"PASS 17:13 entry offset confirms and advances for both halves and sides\n";
}

void nearContourNeedsDistinctStoppedCaptures() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    JawRig p;p.yaw=0;p.reachColourLeg();p.region_inside=true;
    for(int i=0;i<9;++i)p.step();
    assert(p.rig.task.transport_phase_.jaw_sequence_.stage==Stage::ColourStop);
    p.force_sent=true;p.forced_vx=.04f;
    for(int i=0;i<20;++i)assert(p.step().state==PushState::CARRY);
    p.forced_vx=0;
    for(int i=0;i<9;++i)p.step();
    assert(p.out.state==PushState::CARRY);
    p.repeat_region_frame=true;
    for(int i=0;i<4;++i)assert(p.step().state==PushState::CARRY);
    p.fresh=false;assert(p.step().state==PushState::CARRY);
    p.fresh=true;p.repeat_region_frame=false;
    for(int i=0;i<30;++i) {
        if(p.step().state==PushState::RAISE_RELEASE)break;
    }
    assert(p.out.state==PushState::RAISE_RELEASE);
    JawRig missing;missing.yaw=0;missing.reachColourLeg();
    missing.region_inside=true;missing.region_found=false;
    for(int i=0;i<15;++i)assert(missing.step().state==PushState::CARRY);
    std::cout<<"PASS UI120 extent requires a found region, stopped sender and distinct fresh captures\n";
}


void entryRangeReviewAndForwardReposition() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    JawRig jump; jump.gap=.65f; jump.yaw=0;
    for(int i=0;i<4;++i) jump.step();
    jump.gap=.30f;
    auto out=jump.step();
    assert(out.state==PushState::CARRY && out.motion.vx_mps>0);
    assert(jump.step().motion.vx_mps>0);
    out=jump.step();
    assert(out.state==PushState::CARRY && out.motion.vx_mps==0);
    assert(jump.rig.task.transport_phase_.jaw_sequence_.stage==Stage::Approach);
    jump.gap=.65f;
    for(int i=0;i<4;++i) out=jump.step();
    assert(out.state==PushState::CARRY && out.motion.vx_mps>0);
    for(float spread : {.07f,.10f,.11f}) {
        JawRig p;p.yaw=0;p.image_error=.15f;p.edge_override=-.09f;
        p.robot_x=p.rig.task.halfX()-.08f;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        q.stage=Stage::AlignStop;
        bool replanned=false;
        for(int i=0;i<30;++i) {
            p.gap=.55f+(i%2 ? spread : 0);
            out=p.step();
            assert(out.state==PushState::CARRY);
            if(out.reason=="carry_entry_resume_approach") {replanned=true;break;}
            assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        }
        assert(replanned==(spread<=.10f));
        if(!replanned) {
            assert(out.reason=="carry_entry_measurement_unstable");
            p.gap=.55f;
            for(int i=0;i<15;++i) {
                out=p.step();
                if(out.reason=="carry_entry_resume_approach") {replanned=true;break;}
            }
            assert(replanned);
        }
    }
    for(float side : {-1.f,1.f}) {
        JawRig p;p.gap=.55f;p.yaw=0;p.image_error=.3f*side;
        p.rig.task.t_.carry_budget_us=90000000;
        p.edge_override=-.10f*side;
        p.robot_x=p.rig.task.halfX()-.23f*side;
        p.rig.task.transport_phase_.jaw_sequence_.stage=Stage::AlignStop;
        bool replanned=false;
        for(int i=0;i<30;++i) {
            out=p.step();assert(out.state==PushState::CARRY);
            if(out.reason=="carry_entry_resume_approach") {replanned=true;break;}
        }
        assert(replanned);
        p.edge_override=std::numeric_limits<float>::quiet_NaN();
        bool advanced=false;
        for(int i=0;i<1800;++i) {
            p.robot_x-=std::sin(p.yaw)*p.out.motion.vx_mps*.04f;
            p.gap-=std::cos(p.yaw)*p.out.motion.vx_mps*.04f;
            p.image_error=.625f*std::tan(p.yaw-std::atan2(p.robot_x-p.rig.task.halfX(),p.gap+.15f));
            out=p.step();
            assert(out.state==PushState::CARRY && out.motion.vx_mps>=-.05f);
            advanced |= out.motion.vx_mps>0;
            if(advanced && std::abs(p.robot_x-p.rig.task.halfX())<.08f) break;
        }
        assert(advanced && std::abs(p.robot_x-p.rig.task.halfX())<.08f);
        p.rig.input.directional_clearance_valid=true;
        p.rig.input.path_safe=p.rig.input.arc_safe=p.rig.input.turn_safe=false;
        out=p.step();
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
    }
    JawRig unstable; unstable.yaw=0; unstable.image_error=.15f;
    unstable.edge_override=-.09f;
    unstable.rig.task.transport_phase_.jaw_sequence_.stage=Stage::AlignStop;
    for(int i=0;i<30;++i) {
        unstable.gap=.55f+(i%2 ? .11f : 0);
        out=unstable.step();
        assert(out.state==PushState::CARRY && out.motion.vx_mps==0);
    }
    assert(out.reason=="carry_entry_measurement_unstable");
    unstable.rig.task.t_.carry_budget_us=1;
    out=unstable.step();
    assert(out.state==PushState::SAFE_STOP && out.reason=="carry_entry_measurement_unstable");
    std::cout<<"PASS entry range review, 10 cm stability and forward reposition\n";
}

void repositionLossKeepsSearching() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(float side : {-1.f,1.f}) for(bool box : {false,true}) {
        JawRig p;p.gap=.55f;p.yaw=0;p.image_error=.3f*side;
        p.edge_override=-.10f*side;
        p.robot_x=p.rig.task.halfX()-.23f*side;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        q.stage=Stage::AlignStop;
        bool replanned=false;
        for(int i=0;i<30;++i) {
            if(p.step().reason=="carry_entry_resume_approach") {replanned=true;break;}
        }
        assert(replanned && q.reposition);
        assert(p.step().reason=="carry_entry_forward_reposition");
        p.pose=false;p.view_evidence=false;p.sight=box;p.bearing=.3f*side;
        bool directed=false;
        for(int i=0;i<60;++i) {
            const auto out=p.step();
            assert(out.state==PushState::CARRY);
            if(i<12) {
                assert(out.motion.vx_mps>0 &&
                       !p.rig.task.transport_phase_.jaw_sequence_.approach_resume_pending);
            } else assert(out.motion.vx_mps==0);
            if(out.reason=="carry_half_reacquire_turn" && out.motion.wz_rps!=0 && !directed) {
                if(box) assert(out.motion.wz_rps*side>0);
                directed=true;
            }
        }
        assert(directed);
        // Without entrance distance, seeing the half cannot authorize translation.
        p.view_evidence=true;
        auto out=p.step();assert(out.motion.vx_mps==0);
        p.rig.input.directional_clearance_valid=true;
        p.rig.input.turn_safe=p.rig.input.path_safe=p.rig.input.arc_safe=false;
        out=p.step();assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        p.rig.input.turn_safe=p.rig.input.path_safe=p.rig.input.arc_safe=true;
        // Three fresh entrance captures commit the search to a stopped handoff.
        p.pose=true;p.yaw=0;
        for(int i=0;i<3;++i) {
            out=p.step();
            assert(out.motion.vx_mps==0);
            if(i<2) assert(out.reason.rfind("carry_half_reacquire_",0)==0);
            else assert(out.reason=="carry_half_reacquire_brake");
        }
        bool resumed=false;
        for(int i=0;i<25;++i) {
            out=p.step();
            if(out.reason=="carry_entry_forward_reposition" && out.motion.wz_rps!=0) {
                resumed=true;break;
            }
        }
        assert(resumed);
    }
    std::cout<<"PASS reposition loss keeps searching on both sides and resumes fresh measured entry\n";
}


void lateralEntranceJumpsRequireStationaryReview() {
    for (float side : {-1.f,1.f}) {
        JawRig p; p.gap=.60f; p.yaw=0; p.image_error=.15f*side;
        for(int i=0;i<4;++i) p.step();
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        const auto accepted=q.range_view.entrance_origin_body_m;
        // 19:42 trace: transverse pose jumps by >30 cm while range barely changes.
        p.robot_x+=.30f*side;
        for(int i=0;i<2;++i) {
            p.step();
            assert(cv::norm(q.range_view.entrance_origin_body_m-accepted)<.001f);
        }
        auto out=p.step();
        assert(out.reason=="carry_entry_route_hold" && out.motion.vx_mps>0);
        for(int i=0;i<15;++i) {
            p.robot_x=p.rig.task.halfX()+side*(i%2 ? .30f : .50f);
            out=p.step();
            if(q.approach_route.valid(p.rig.input.now_us,500000)) {
                assert(out.reason=="carry_entry_route_hold" && out.motion.vx_mps>0);
            } else {
                assert(out.reason=="carry_entry_measurement_unstable");
                assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
            }
        }
        assert(out.reason=="carry_entry_measurement_unstable");
        p.robot_x=p.rig.task.halfX()+.30f*side;
        for(int i=0;i<6 && q.range_wait;++i) out=p.step();
        assert(!q.range_wait && out.state==PushState::CARRY);
        // Stable range now goes through the early curve review before movement.
        const auto replanned = [&] { return q.reposition || int(q.stage)>=9; };
        for(int i=0;i<25 && !replanned();++i) out=p.step();
        assert(replanned() && out.motion.vx_mps==0);
        bool moved=false;
        for(int i=0;i<160;++i) {
            out=p.step();
            if(out.motion.vx_mps!=0 || out.motion.wz_rps!=0) {moved=true;break;}
        }
        assert(moved);
    }
    std::cout<<"PASS lateral entrance jumps bridge briefly, expire and require three stable captures\n";
}


// Short parking room now stops for image alignment instead of reversing.
void shortParkingEntryUsesStationaryAlignment() {
    for(float side : {-1.f,1.f}) {
        JawRig p;p.gap=.3796839f;p.robot_x=p.rig.task.halfX()+.201325f*side;
        p.yaw=.5f*side;p.image_error=.127826f*side;
        bool turned=false;
        for(int i=0;i<80;++i) {
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.vx_mps==0);
            assert(out.motion.gripper_offset==20);
            if(out.motion.wz_rps!=0) {assert(out.motion.wz_rps*side<0);turned=true;break;}
        }
        assert(turned);
        p.rig.input.directional_clearance_valid=true;p.rig.input.turn_safe=false;
        auto out=p.step();
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0 && out.immediate_motion_stop);
        p.rig.input.turn_safe=true;p.heading=false;
        out=p.step();assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
    }
    std::cout<<"PASS short parking entry turns in place and obeys turn/heading gates\n";
}

void approachRepositionsBeforeReachingParkingLine() {
    for(float side : {-1.f,1.f}) {
        JawRig p;p.gap=.75f;p.yaw=0;
        p.rig.task.t_.carry_budget_us=90000000; // match launcher controlled-field budget
        p.robot_x=p.rig.task.halfX()+.35f*side;
        bool repositioned=false,advanced=false;
        for(int i=0;i<1800;++i) {
            p.robot_x-=std::sin(p.yaw)*p.out.motion.vx_mps*.04f;
            p.gap-=std::cos(p.yaw)*p.out.motion.vx_mps*.04f;
            p.image_error=.625f*std::tan(p.yaw-std::atan2(p.robot_x-p.rig.task.halfX(),p.gap+.15f));
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.vx_mps>=-.05f);
            repositioned |= p.rig.task.transport_phase_.jaw_sequence_.reposition;
            advanced |= out.motion.vx_mps>0;
            if(repositioned && advanced && std::abs(p.robot_x-p.rig.task.halfX())<.04f)break;
        }
        assert(repositioned && advanced && std::abs(p.robot_x-p.rig.task.halfX())<.04f);
        assert(p.gap>.35f);
    }
    std::cout<<"PASS early forward reposition removes large side offsets before parking\n";
}

void lostHalfFindsFreshZoneAndBrakesBeforeResuming() {
    for(float side : {-1.f,1.f}) {
        JawRig p;p.yaw=0;p.gap=.8f;
        for(int i=0;i<6;++i)p.step();
        p.pose=p.view_evidence=false;p.bearing=.4f*side;
        bool directed=false;
        for(int i=0;i<18;++i) {
            const auto out=p.step();assert(out.motion.vx_mps>=0);
            if(out.motion.wz_rps!=0) {
                assert(out.motion.vx_mps==0 && out.motion.wz_rps*side>0);directed=true;
            }
        }
        assert(directed);
        p.pose=p.view_evidence=true;
        p.force_sent=true;p.forced_vx=.01f;
        for(int i=0;i<10;++i) {
            const auto out=p.step();
            assert(out.reason=="carry_half_reacquire_brake");
            assert(out.motion.vx_mps==0 && out.motion.wz_rps==0 && out.immediate_motion_stop);
        }
        p.forced_vx=0;
        for(int i=0;i<7;++i)assert(p.step().motion.vx_mps==0);
        // Multiple control ticks on one image cannot satisfy the three-image gate.
        for(int i=0;i<8;++i) {
            p.rig.input.now_us+=1000;
            const auto out=p.rig.task.update(p.rig.input);assert(out.motion.vx_mps==0);
        }
        bool resumed=false;
        for(int i=0;i<20;++i) {
            const auto out=p.step();
            assert(out.motion.vx_mps>=0);
            if(out.motion.vx_mps>0) {resumed=true;break;}
        }
        assert(resumed);
    }
    std::cout<<"PASS missing half turns toward fresh zone and requires a stopped fresh-image handoff\n";
}

void halfSearchIsBoundedWhenZoneDisappears() {
    for(float response : {1.f,6.f}) for(float heading_offset : {-3.f,0.f,3.f}) {
        JawRig p;p.yaw=0;p.gap=.8f;p.imu_offset=heading_offset;
        p.ramped_dead_zone=true;p.angular_response=response;
        p.rig.task.t_.transport_scan_wz=.9f;
        p.rig.task.t_.transport_align_min_wz=.9f;
        for(int i=0;i<6;++i)p.step();
        p.pose=p.view_evidence=p.sight=false;
        const float initial=p.yaw;
        for(int i=0;i<300;++i) {
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.vx_mps>=0);
            assert(std::abs(wrapAngle(p.yaw-initial))<.85f);
        }
        assert(p.out.motion.vx_mps==0 && p.out.motion.wz_rps==0);
    }
    std::cout<<"PASS selected-half search is bounded with 0.9 rad/s, fast yaw response and heading wrap\n";
}

// 20:24 trace: isolated entrance detections repeatedly killed the search ramp.
void intermittentHalfCannotPulseSearchBrakes() {
    for (float side : {-1.f, 1.f}) {
        JawRig p; p.gap=.8f; p.yaw=0; p.pose=false; p.half_entrance=true;
        p.ramped_dead_zone=true; p.angular_response=0;
        p.rig.task.t_.transport_scan_wz=.6f;
        p.rig.task.t_.transport_align_min_wz=.6f;
        for (int i=0;i<6;++i) p.step();
        p.view_evidence=false; p.bearing=.4f*side;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        bool searching=false;
        for (int i=0;i<18;++i) {
            const auto out=p.step();
            if (out.reason=="carry_half_reacquire_turn") {searching=true;break;}
        }
        assert(searching);
        for (int i=0;i<30;++i) {
            p.view_evidence=i%2==0;
            const auto out=p.step();
            assert(out.motion.vx_mps==0);
            assert(out.reason=="carry_half_reacquire_turn" &&
                   out.motion.wz_rps*side>0 && !out.immediate_motion_stop &&
                   "one recovered entrance frame must not interrupt a directed search");
        }
        assert(std::abs(p.wire.wz_rps)>.25f && "search must get past the sender ramp/dead zone");
        // Repeated control ticks on a single candidate cannot confirm it.
        p.view_evidence=true;
        p.step();
        for (int i=0;i<8;++i) {
            p.rig.input.now_us+=1000;
            const auto out=p.rig.task.update(p.rig.input);
            assert(out.reason=="carry_half_reacquire_turn" && !out.immediate_motion_stop);
        }
        bool braking=false;
        for (int i=0;i<4;++i) {
            const auto out=p.step();
            if(out.reason=="carry_half_reacquire_brake") {braking=true;break;}
        }
        assert(braking && p.out.immediate_motion_stop);
        // A brief dropout during the stopped handoff must not restart the turn.
        p.view_evidence=false;
        for(int i=0;i<3;++i) {
            const auto out=p.step();
            assert(out.reason=="carry_half_reacquire_brake" && out.immediate_motion_stop);
            assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        }
        // Sustained loss releases the handoff back to the same bounded search.
        bool turned_again=false;
        for(int i=0;i<30;++i) {
            const auto out=p.step();
            assert(out.motion.vx_mps==0);
            if(out.reason=="carry_half_reacquire_turn") {turned_again=true;break;}
        }
        assert(turned_again);
        // Clearance still vetoes search with or without a candidate entrance.
        p.rig.input.directional_clearance_valid=true;
        p.rig.input.turn_safe=false;
        for(bool candidate : {false,true}) {
            p.view_evidence=candidate;
            const auto out=p.step();
            assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        }
        p.rig.input.turn_safe=true;
        p.view_evidence=true; p.bearing=0; p.image_error=0;
        bool resumed=false;
        for(int i=0;i<40;++i) {
            const auto out=p.step();
            if(out.motion.vx_mps>0) {resumed=true;break;}
        }
        assert(resumed && !q.approach_resume_pending);
    }
    std::cout<<"PASS intermittent entrance keeps search ramp; distinct candidates brake once; short loss holds; prolonged loss searches; stable entrance resumes\n";
}

void forwardEntryRegression() {
    shortParkingEntryUsesStationaryAlignment();
    approachRepositionsBeforeReachingParkingLine();
    lostHalfFindsFreshZoneAndBrakesBeforeResuming();
    halfSearchIsBoundedWhenZoneDisappears();
}


// 20:41 trace: short entrance dropouts during reposition and noisy distance
// reviews must not restart the search/brake cycle on every forward segment.
void approachContinuityAcrossBriefEntranceNoise() {
    for (bool reposition : {false,true}) for (bool injured : {false,true}) {
        JawRig p;p.gap=1.2f;p.yaw=0;p.ramped_dead_zone=true;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_;p.supply_visible=!injured;p.injured_visible=injured;
        auto& q=d.jaw_sequence_;
        for(int i=0;i<6;++i)p.step();
        q.reposition=reposition;
        if(reposition) p.robot_x-=.04f;
        for(int i=0;i<6;++i)p.step();
        for(int cycle=0;cycle<12;++cycle) {
            for(int i=0;i<3;++i) {
                p.pose=false;p.view_evidence=cycle%2==0;
                const auto out=p.step();
                assert(out.state==PushState::CARRY && !out.immediate_motion_stop);
                assert((out.motion.vx_mps>0 || out.motion.wz_rps!=0) &&
                       "brief entrance dropout must not pulse a search brake");
                assert(!q.approach_resume_pending);
                assert(p.wire.vx_mps>0 && "the real sender ramp must keep advancing");
            }
            p.pose=true;p.view_evidence=true;
            for(int i=0;i<6;++i) {
                const auto out=p.step();
                assert(out.state==PushState::CARRY && !out.immediate_motion_stop);
            }
        }
    }
    std::cout<<"PASS approach/reposition bridge repeated brief entrance dropouts for both halves\n";
}

// 13:39/13:44 board traces: 0.2--0.4 s range/half dropouts repeatedly
// reset forward motion, although the last accepted entrance still had room.
void initialApproachBridgesLoggedNoiseBursts() {
    for (bool reposition : {false,true}) for (bool injured : {false,true})
        for (bool image_missing : {false,true}) {
        JawRig p;p.gap=1.2f;p.yaw=0;p.ramped_dead_zone=true;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_-(reposition ? .08f : 0.f);
        p.supply_visible=!injured;p.injured_visible=injured;
        auto& q=d.jaw_sequence_;
        q.reposition=reposition;
        for(int i=0;i<16;++i)p.step();
        for(int cycle=0;cycle<6;++cycle) {
            const auto accepted=q.route_fix_us;
            for(int i=0;i<10;++i) {
                p.pose=false;p.view_evidence=!image_missing;
                const auto out=p.step();
                assert(out.state==PushState::CARRY && out.motion.vx_mps>0 &&
                       !out.immediate_motion_stop &&
                       "400 ms projection/half dropout must keep the accepted approach moving");
                assert(!q.approach_resume_pending && q.route_fix_us==accepted);
                assert(p.wire.vx_mps>0);
                if(i>=5)assert(out.motion.vx_mps<=.030001f);
            }
            p.pose=p.view_evidence=true;
            for(int i=0;i<8;++i) {
                const auto out=p.step();
                assert(out.motion.vx_mps>0 && !out.immediate_motion_stop);
            }
        }
        // Neither a repeated capture nor continuing box sightings renew the route.
        p.pose=false;p.view_evidence=!image_missing;
        for(int i=0;i<14;++i) {
            const auto out=p.step();
            assert(i<12 ? out.motion.vx_mps>0 : out.motion.vx_mps==0);
        }
        assert(p.out.motion.vx_mps==0);
        assert(!q.approach_route.valid(p.rig.input.now_us,500000));
        if(reposition || image_missing)assert(q.approach_resume_pending);
    }
    std::cout<<"PASS initial approach/reposition bridge repeated 400ms range/half loss at reduced speed, then stop on expiry\n";
}

void initialApproachFiltersRangeNoiseAndHonoursStops() {
    for(bool reposition : {false,true}) for(float side : {-1.f,1.f}) {
        JawRig p;p.gap=1.2f;p.yaw=0;p.angular_response=0;
        const float base_x=p.rig.task.halfX()-(reposition ? .08f : 0.f)*side;
        p.robot_x=base_x;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        for(int i=0;i<8;++i)p.step();
        q.reposition=reposition;
        for(int i=0;i<8;++i)p.step();
        for(int cycle=0;cycle<6;++cycle) {
            // Both range and lateral projection fluctuate, like the 13:44 trace.
            for(int i=0;i<12;++i) {
                const bool outlier=i<9;
                p.gap=outlier ? (i%2 ? 1.45f : 1.7f) : 1.2f;
                p.robot_x=base_x+(outlier ? (i%2 ? .3f : .55f)*side : 0.f);
                const auto out=p.step();
                assert(out.motion.vx_mps>0 && !out.immediate_motion_stop &&
                       "360 ms range jumps followed by stable captures must not pulse the brakes");
                assert(!q.approach_resume_pending);
            }
        }
    }
    // Small accepted position changes refine the trajectory rather than snap it.
    JawRig blend;blend.yaw=0;blend.gap=1.2f;
    for(int i=0;i<8;++i)blend.step();
    auto& route=blend.rig.task.transport_phase_.jaw_sequence_.approach_route;
    const float before=route.predicted(blend.rig.input.now_us).bodyToZone({0,0}).x;
    blend.robot_x+=.04f;
    blend.step();
    const float correction=route.predicted(blend.rig.input.now_us).bodyToZone({0,0}).x-before;
    assert(correction>.005f && correction<.03f);
    // Missing evidence never overrides current heading/feedback/clearance or close range.
    for(int fault=0;fault<6;++fault) {
        JawRig p;p.yaw=0;p.gap=fault==5 ? .42f : 1.2f;p.angular_response=0;
        p.robot_x=p.rig.task.halfX()-.08f;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        for(int i=0;i<8;++i)p.step();
        q.reposition=true;p.step();
        if(fault==5)p.gap=.34f;
        else {p.pose=false;p.view_evidence=true;}
        p.rig.input.directional_clearance_valid=true;
        if(fault==0)p.rig.input.path_safe=false;
        if(fault==1)p.rig.input.arc_safe=false;
        if(fault==2)p.heading=false;
        if(fault==3)p.rig.input.sent_velocity_valid=false;
        if(fault==4) {
            p.force_sent=true;p.forced_vx=std::numeric_limits<float>::quiet_NaN();
        }
        const auto out=p.step();
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0 &&
               "route hold must honour immediate clearance/feedback/entry stops");
    }
    std::cout<<"PASS range-jump bursts stay continuous, accepted poses blend and immediate stops still win\n";
}

void approachRangeReviewKeepsRecentTrajectory() {
    JawRig p;p.gap=.9f;p.yaw=0;
    for(int i=0;i<6;++i)p.step();
    for(int cycle=0;cycle<12;++cycle) {
        // Three outliers used to force an immediate stop, even when the next
        // three captures restored the same entrance well inside route expiry.
        for(int i=0;i<6;++i) {
            p.gap=i<3 ? 1.15f : .9f;
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.vx_mps>0 &&
                   !out.immediate_motion_stop && "short range review must preserve a fresh route");
        }
    }
    // Persistent uncertainty must expire rather than renew its own prediction.
    for(int i=0;i<20;++i) {
        p.gap=i%2 ? 1.15f : 1.4f;
        p.step();
    }
    assert(p.out.motion.vx_mps==0 && p.out.motion.wz_rps==0);
    assert(p.out.immediate_motion_stop);
    std::cout<<"PASS short range review continues; persistent inconsistent measurements stop\n";
}


// A fresh longer range must not restart translation after near-entry parking.
void nearEntryKeepsAlignmentAfterRangeChange() {
    for(bool injured : {false,true}) for(float side : {-1.f,1.f}) {
        JawRig p;p.gap=.38f;p.yaw=0;p.image_error=.15f*side;p.edge_override=0;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_-.15f*side;
        p.supply_visible=!injured;p.injured_visible=injured;
        bool turned=false;
        for(int i=0;i<50;++i) {
            const auto out=p.step();assert(out.state==PushState::CARRY && out.motion.vx_mps==0);
            if(out.motion.wz_rps!=0) {turned=true;break;}
        }
        assert(turned && d.jaw_sequence_.stationary_entry);
        p.gap=.70f;
        for(int i=0;i<35;++i) {
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.vx_mps==0);
            assert(out.reason!="carry_entry_resume_approach");
            assert(d.jaw_sequence_.recovery==TransportPhaseData::JawSequence::Recovery::Idle);
        }
        p.rig.task.t_.carry_budget_us=1;
        const auto out=p.step();
        assert(out.state==PushState::SAFE_STOP && out.reason=="carry_entry_alignment_timeout");
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0 && out.immediate_motion_stop);
    }
    std::cout<<"PASS near-entry range changes cannot restart translation; timeout stops explicitly\n";
}


// 01:08:32 trace, frame 92607348: the first usable posture preceded the
// first reverse. Reposition must brake here instead of turning through centre.
void loggedPreBackoffPostureHoldsAcrossFrameNoise() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(bool injured : {false,true}) for(float side : {-1.f,1.f}) {
        JawRig p;p.gap=.633669f;p.yaw=-.126544f*side;
        p.image_error=.0587386f*side;p.edge_override=-.040404f*side;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;
        d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_-.121052f*side;
        p.supply_visible=!injured;p.injured_visible=injured;
        d.jaw_sequence_.reposition=true;
        auto out=p.step();
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0 &&
               "first deliverable posture must brake before it triggers backoff");
        assert(out.immediate_motion_stop);
        const auto started=p.rig.input.now_us;
        // Changes below observed maxima rounded up to .03 and .05 retain the
        // accepted decision, including a frame just outside the old 7% gate.
        for(int i=0;i<25;++i) {
            p.image_error=(i%2 ? .0717023f : .0639241f)*side;
            p.edge_override=(i%2 ? -.0421053f : -.040404f)*side;
            out=p.step();
            assert(out.state==PushState::CARRY && out.motion.vx_mps==0 && out.motion.wz_rps==0);
            assert(d.jaw_sequence_.stage!=Stage::BackoffReview);
            if(out.reason=="carry_pre_entry_aligned")break;
        }
        assert(out.reason=="carry_pre_entry_aligned");
        assert(p.rig.input.now_us-started>=250000);
        assert(d.jaw_sequence_.stage==Stage::ColourAdvance);
        out=p.step();
        assert(out.reason=="carry_ui120_region_advance" && out.motion.vx_mps>0);
        assert(out.motion.gripper_offset==20);
    }
    std::cout<<"PASS logged first deliverable posture holds frame jitter and advances without backoff on both halves/sides\n";
}


void entryPostureHoldRespectsChangeAndFreshEvidence() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    const auto seed=[](JawRig& p) {
        p.gap=.633669f;p.yaw=-.126544f;
        p.image_error=.0587386f;p.edge_override=-.040404f;
        p.robot_x=p.rig.task.halfX()-.121052f;
        p.rig.task.transport_phase_.jaw_sequence_.reposition=true;
        assert(p.step().reason=="carry_entry_posture_brake");
    };
    // Exercise both measured bands almost to their boundary.
    JawRig band;seed(band);
    band.image_error+=.0299f;band.edge_override-=.0499f;
    for(int i=0;i<25 && band.out.reason!="carry_pre_entry_aligned";++i)band.step();
    assert(band.out.reason=="carry_pre_entry_aligned");
    // Small per-frame drift accumulates against the accepted baseline.
    for(int axis=0;axis<4;++axis) {
        JawRig p;seed(p);
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        if(axis<2) {
            for(int i=1;i<=3;++i) {
                if(axis==0)p.image_error=.0587386f+.009f*i;
                else p.edge_override=-.040404f-.016f*i;
                p.step();assert(q.entry_posture_held);
            }
            if(axis==0)p.image_error=.0587386f+.031f;
            else p.edge_override=-.040404f-.051f;
        } else if(axis==2) p.inside=false;
        else p.stale_half=true;
        const auto out=p.step();
        assert(!q.entry_posture_held && q.stage!=Stage::ColourAdvance);
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
    }
    // A held image must still wait for sent translation to stop.
    JawRig moving;seed(moving);moving.force_sent=true;moving.forced_vx=.01f;
    for(int i=0;i<20;++i) {
        const auto out=moving.step();
        assert(out.reason!="carry_pre_entry_aligned" && out.motion.vx_mps==0);
    }
    moving.forced_vx=0;
    auto& q=moving.rig.task.transport_phase_.jaw_sequence_;
    for(int i=0;i<20 && q.visual_frames==0;++i)moving.step();
    assert(q.visual_frames==1);
    const int frames=q.visual_frames;
    // Repeated processing of one capture cannot buy three-frame permission.
    for(int i=0;i<3;++i) {
        auto& in=moving.rig.input;in.now_us+=20000;
        const auto out=moving.rig.task.update(in);
        assert(q.visual_frames==frames && out.reason!="carry_pre_entry_aligned");
    }
    moving.rig.input.path_safe=false;
    for(int i=0;i<20;++i) {
        const auto out=moving.step();
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
    }
    std::cout<<"PASS posture hold bands, accumulated drift, lost/stale half, sent-stop and distinct-capture/clearance gates\n";
}


// 01:58:33 trace: a differential-drive reposition must turn beyond image
// centre before translating sideways. The final stationary alignment gate
// must not cancel that manoeuvre on an instantaneous image-error sign.
void differentialDriveRepositionAllowsTransientImageDeviation() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(bool injured : {false,true}) for(float side : {-1.f,1.f}) {
        JawRig p;p.gap=.7467f;p.yaw=-.54454f*side;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;
        d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_-.363158f*side;
        p.image_error=-.0743061f*side;p.edge_override=-.0897436f*side;
        p.supply_visible=!injured;p.injured_visible=injured;
        d.jaw_sequence_.reposition=true;
        auto out=p.step();
        assert(out.state==PushState::CARRY && d.jaw_sequence_.stage==Stage::Approach);
        assert(out.reason=="carry_entry_forward_reposition" &&
               "transient image deviation must not cancel differential-drive reposition");
        assert(out.motion.vx_mps>0 && out.motion.wz_rps*side<0);
        assert(std::abs(out.motion.wz_rps)<p.rig.task.t_.min_turn_wz);
        assert(p.image_error*out.motion.wz_rps>0);
        p.heading=false;
        out=p.step();
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        p.heading=true;
        p.rig.input.sent_velocity_valid=false;
        out=p.step();
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        p.rig.input.sent_velocity_valid=true;
        p.rig.input.path_safe=false;
        out=p.step();
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
    }
    // Both the initial path review and stationary reapproach must choose
    // reposition from available room, rather than the same image sign veto.
    for(bool stationary : {false,true}) for(float side : {-1.f,1.f}) {
        JawRig p;p.gap=.9153f;p.yaw=-.7822f*side;
        p.robot_x=p.rig.task.halfX()-.411554f*side;
        p.image_error=-.248218f*side;p.edge_override=-.0641025f*side;
        p.angular_response=0;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        if(stationary) q.stage=Stage::AlignStop;
        for(int i=0;i<30 && !q.reposition;++i) {
            const auto out=p.step();
            assert(out.state==PushState::CARRY && q.stage!=Stage::BackoffReview);
        }
        assert(q.reposition && q.stage==Stage::Approach);
    }
    // Exercise the real sender ramp and a nonholonomic plant: continuously
    // curve forward, obtain lateral displacement, then restore entrance heading.
    for(bool injured : {false,true}) for(float side : {-1.f,1.f}) {
        JawRig p;p.gap=.75f;p.yaw=0;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;
        d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_+.35f*side;
        p.supply_visible=!injured;p.injured_visible=injured;
        p.rig.task.t_.carry_budget_us=90000000;
        MotionVelocityRamp ramp{MotionLinkConfig{}};
        MotionCommand sent;
        ramp.accept(sent,p.rig.input.now_us);
        bool curved=false,translated=false,temporary_deviation=false,aligned=false;
        int reverses=0;bool reversing=false;
        for(int i=0;i<2200;++i) {
            p.robot_x-=std::sin(p.yaw)*sent.vx_mps*kDt;
            p.gap-=std::cos(p.yaw)*sent.vx_mps*kDt;
            p.out.motion=sent;
            const float next_yaw=wrapAngle(p.yaw+sent.wz_rps*kDt);
            p.image_error=.625f*std::tan(next_yaw-
                std::atan2(p.robot_x-d.half_x_,p.gap+.15f));
            p.inside=std::abs(p.image_error)<.1f;
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            const bool reverse=out.motion.vx_mps<0;
            if(reverse && !reversing) ++reverses;
            reversing=reverse;
            assert(reverses<=1 && "reposition must not repeat a backoff loop");
            if(out.reason=="carry_entry_forward_reposition") {
                curved |= out.motion.vx_mps>0 && std::abs(out.motion.wz_rps)>.01f;
                translated |= out.motion.vx_mps>0;
                temporary_deviation |= std::abs(p.image_error)>.10f &&
                    p.image_error*out.motion.wz_rps>0;
            }
            if(out.reason=="carry_pre_entry_aligned") {
                assert(std::abs(p.robot_x-d.half_x_)<.08f);
                assert(std::abs(p.yaw)<=std::atan(.60f)+.0001f);
                assert(std::abs(p.image_error)<.10f && p.gap>.20f);
                aligned=true;break;
            }
            sent=ramp.next(out.motion,p.rig.input.now_us,out.immediate_motion_stop);
            ramp.accept(sent,p.rig.input.now_us);
        }
        if(!aligned) std::cerr<<"differential reposition side="<<side<<" injured="<<injured
            <<" x_error="<<p.robot_x-d.half_x_<<" gap="<<p.gap<<" yaw="<<p.yaw
            <<" reason="<<p.out.reason<<std::endl;
        assert(curved && translated && temporary_deviation && aligned);
        std::cout<<"PASS differential reposition half="<<(injured ? "injured" : "supply")
                 <<" side="<<side<<" lateral_error_m="<<p.robot_x-d.half_x_
                 <<" yaw_rad="<<p.yaw<<" reverse_legs="<<reverses<<"\n";
    }
    std::cout<<"PASS differential-drive reposition tolerates transient image deviation and completes lateral travel/heading restoration\n";
}


// Stationary transport can retain IMU vibration after stopping. Exercise the
// real pre-push calibration with noise above the old capture thresholds.
void prePushCalibrationAcceptsStoppedImuNoise() {
    using Stage=TransportPhaseData::JawSequence::PrePushReview::Stage;
    for(int scenario=0;scenario<3;++scenario) {
        JawRig p;p.yaw=0;
        auto& q=p.rig.task.transport_phase_.jaw_sequence_;
        q.active=true;
        q.stage=TransportPhaseData::JawSequence::Stage::ColourAdvance;
        q.prepush.stage=Stage::BackReady;
        q.prepush.started_us=p.rig.input.now_us;
        q.prepush.heading=0;
        const auto started=p.rig.input.now_us;
        uint64_t sequence=0;
        bool ready=false;
        for(int n=0;n<40;++n) {
            auto& in=p.rig.input;
            in.imu_translation_samples.clear();
            for(int i=1;i<=4;++i) {
                ImuTranslationSample sample;
                sample.sequence=++sequence;
                sample.received_us=in.now_us+i*10000;
                sample.device_time_ms=sample.received_us/1000;
                sample.valid=true;
                const float noise=scenario==2 ? .35f : .20f;
                sample.acceleration_mps2={sequence%2 ? noise : -noise,0,9.8f};
                sample.angular_velocity_rps={0,0,scenario==1 ? .16f : .10f};
                sample.quaternion_wxyz={1,0,0,0};
                in.imu_translation_samples.push_back(sample);
            }
            const auto out=p.step();
            assert(out.state==PushState::CARRY);
            assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
            if(q.prepush.stage==Stage::Back) {
                ready=true;
                assert(q.prepush.imu.ready());
                assert(in.now_us-started>=750000);
                assert(in.now_us-started<=1000000);
                break;
            }
            assert(q.prepush.stage==Stage::BackReady);
        }
        assert(ready==(scenario==0));
    }
    std::cout<<"PASS transport pre-push accepts stationary IMU noise and rejects excess gyro/acceleration\n";
}


// 2026-11-05 05:17:16: reposition turned away until its own entrance
// clipped, then searched back and repeated the same turn without translation.
void loggedRepositionAdvancesWithinCameraView() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(bool injured : {false,true}) for(float side : {-1.f,1.f}) {
        JawRig p;p.gap=.633881f;p.yaw=-.890489f*side;
        p.pose=false;p.half_entrance=true;p.ramped_dead_zone=true;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_-.381314f*side;
        p.supply_visible=!injured;p.injured_visible=injured;
        d.jaw_sequence_.reposition=true;
        p.rig.task.t_.carry_budget_us=60000000;
        p.rig.task.t_.transport_scan_wz=.9f;
        p.rig.task.t_.max_wz=.6f;
        p.rig.task.t_.transport_align_min_wz=.6f;
        int searches=0,reverse_legs=0,moving_ticks=0;
        bool searching=false,reversing=false,aligned=false;
        for(int i=0;i<1200;++i) {
            p.robot_x-=std::sin(p.yaw)*p.wire.vx_mps*kDt;
            p.gap-=std::cos(p.yaw)*p.wire.vx_mps*kDt;
            const float rate=std::abs(p.wire.vx_mps)>.001f ||
                std::abs(p.wire.wz_rps)>.25f ? p.wire.wz_rps : 0;
            const float next_yaw=wrapAngle(p.yaw+rate*kDt);
            const float dx=d.half_x_-p.robot_x;
            // Synthetic pinhole camera: the entrance needs both near corners.
            // The whole-zone box can remain visible after a corner clips.
            const float left=-next_yaw-std::atan2(dx-.15f,p.gap);
            const float right=-next_yaw-std::atan2(dx+.15f,p.gap);
            p.view_evidence=std::max(std::abs(left),std::abs(right))<.70f;
            const float left_x=.5f-.5f*std::tan(left)/std::tan(.70f);
            const float right_x=.5f-.5f*std::tan(right)/std::tan(.70f);
            p.left_margin=std::min(left_x,right_x);
            p.right_margin=1.f-std::max(left_x,right_x);
            p.bearing=-next_yaw-std::atan2(dx,p.gap);
            p.sight=std::abs(p.bearing)<.85f;
            p.image_error=.625f*std::tan(next_yaw+std::atan2(dx,p.gap+.15f));
            p.inside=std::abs(p.image_error)<.10f;
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
            const bool search=out.reason.rfind("carry_half_reacquire_",0)==0;
            if(search && !searching) ++searches;
            searching=search;
            const bool reverse=out.motion.vx_mps<0;
            if(reverse && !reversing) ++reverse_legs;
            reversing=reverse;
            moving_ticks+=p.wire.vx_mps>.001f;
            if(out.reason=="carry_pre_entry_aligned") {
                assert(p.gap>.20f && std::abs(p.image_error)<.10f);
                const auto& selected=injured ? p.rig.input.injured_view : p.rig.input.supply_view;
                assert(selected.inside && std::abs(selected.edge_slope)<=.30f);
                assert(std::abs(p.robot_x-d.half_x_)<.381314f*.5f);
                aligned=true;break;
            }
            assert(p.gap>.20f && d.jaw_sequence_.stage!=Stage::ColourAdvance);
        }
        std::cout<<"logged continuous reposition injured="<<injured<<" side="<<side
                 <<" aligned="<<aligned<<" searches="<<searches<<" reverse_legs="<<reverse_legs
                 <<" moving_ticks="<<moving_ticks<<" gap="<<p.gap
                 <<" offset="<<p.robot_x-d.half_x_<<" yaw="<<p.yaw<<std::endl;
        assert(aligned && reverse_legs==0 && searches<=1 &&
               "logged reposition must advance and align instead of repeating turn/search cycles");
    }
}


void continuousRepositionRespectsLimitsAndRouteExpiry() {
    for(float side : {-1.f,1.f}) {
        const auto seed=[&](JawRig& p) {
            p.gap=.8f;p.yaw=0;p.robot_x=p.rig.task.halfX()-.2f*side;
            p.image_error=.25f*side;
            p.rig.task.transport_phase_.jaw_sequence_.reposition=true;
        };
        JawRig curve;seed(curve);
        curve.rig.task.t_.max_wz=.01f;
        const auto limited=curve.step();
        assert(limited.motion.vx_mps>0 && limited.motion.vx_mps<.05f);
        assert(std::abs(limited.motion.wz_rps)<=.010001f);
        JawRig normal;seed(normal);
        const auto unlimited=normal.step();
        const float expected=unlimited.motion.wz_rps/unlimited.motion.vx_mps;
        assert(std::abs(limited.motion.wz_rps/limited.motion.vx_mps-expected)<1e-3f);

        // Keep the full measured half in view instead of driving a corner out.
        JawRig border;seed(border);
        border.left_margin=side>0 ? .5f : .01f;
        border.right_margin=side>0 ? .01f : .5f;
        const auto inward=border.step();
        assert(inward.motion.vx_mps>0 && inward.motion.wz_rps*side<0);
        // The same path without a border constraint still has the route's sign.
        // Here image and route agree; use a logged opposite-sign posture too.
        JawRig conflict;seed(conflict);
        conflict.yaw=-.89f*side;
        conflict.robot_x=conflict.rig.task.halfX()-.38f*side;
        conflict.gap=.634f;conflict.image_error=-.3f*side;
        conflict.left_margin=side>0 ? .01f : .5f;
        conflict.right_margin=side>0 ? .5f : .01f;
        const auto guarded=conflict.step();
        assert(guarded.motion.vx_mps>0 && guarded.motion.wz_rps*side>0);

        for(int fault=0;fault<3;++fault) {
            JawRig blocked;seed(blocked);
            blocked.rig.input.directional_clearance_valid=true;
            blocked.rig.input.arc_safe=fault!=0;
            blocked.rig.input.path_safe=fault!=1;
            blocked.heading=fault!=2;
            const auto out=blocked.step();
            assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        }
        // Brief missing geometry keeps the accepted route, not an opposite
        // box bearing. Persistent loss expires into a stopped bounded search.
        JawRig loss;seed(loss);
        loss.step();loss.view_evidence=loss.pose=false;loss.bearing=.5f*side;
        for(int i=0;i<3;++i) {
            const auto out=loss.step();
            assert(out.motion.vx_mps>0 && !out.immediate_motion_stop);
            assert(!loss.rig.task.transport_phase_.jaw_sequence_.approach_resume_pending);
        }
        loss.view_evidence=loss.pose=true;
        assert(loss.step().motion.vx_mps>0);
        loss.view_evidence=loss.pose=false;
        for(int i=0;i<14;++i)loss.step();
        assert(loss.out.motion.vx_mps==0);
        assert(loss.rig.task.transport_phase_.jaw_sequence_.approach_resume_pending);
    }
    std::cout<<"PASS continuous reposition preserves curvature at speed limits, image margins, clearance, short dropout and search expiry\n";
}


// 2026-11-05 06:06:49: only 2.7 cm remained before the parking line.
// The observed centre and edge both improve under a stationary turn.
void loggedNearEntryTurnsWithoutRetreat() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(bool injured : {false,true}) for(float side : {-1.f,1.f})
        for(bool reposition : {false,true}) {
        JawRig p;p.gap=.376977f;p.yaw=-.550276f*side;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_-.142585f*side;
        p.supply_visible=!injured;p.injured_visible=injured;
        d.jaw_sequence_.reposition=reposition;
        const float start_yaw=p.yaw;
        bool turned=false,accepted=false;
        for(int i=0;i<200;++i) {
            const float next_yaw=p.yaw+p.out.motion.wz_rps*.04f;
            const float delta=wrapAngle(next_yaw-start_yaw);
            p.image_error=-.184901f*side+.625f*delta;
            p.edge_override=-.192547f*side+.5f*delta;
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.vx_mps==0);
            assert(d.jaw_sequence_.stage!=Stage::BackoffReview);
            assert(out.motion.gripper_offset==20);
            if(out.motion.wz_rps!=0) {
                assert(out.motion.wz_rps*side>0);turned=true;
            }
            if(out.reason=="carry_pre_entry_aligned") {
                assert(turned && std::abs(p.image_error)<.10f && std::abs(p.edge_override)<=.30f);
                accepted=true;break;
            }
        }
        assert(accepted && "logged near entrance must align by turning, without retreat");
        assert(p.step().motion.vx_mps>0 && p.rig.task.desiredPitch()==0);
    }
    std::cout<<"PASS logged 37.7 cm entry turns in place and confirms for both halves/sides\n";
}

void nearEntryNeedsStoppedDistinctImages() {
    JawRig p;p.gap=.377f;p.yaw=0;p.image_error=0;p.edge_override=0;
    p.robot_x=p.rig.task.halfX()-.143f;
    p.force_sent=true;p.forced_vx=.01f;
    for(int i=0;i<20;++i) {
        const auto out=p.step();assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        assert(out.reason!="carry_pre_entry_aligned");
    }
    p.forced_vx=0;
    auto& q=p.rig.task.transport_phase_.jaw_sequence_;
    for(int i=0;i<30 && q.visual_frames==0;++i)p.step();
    assert(q.stationary_entry && q.visual_frames==1);
    for(int i=0;i<5;++i) {
        p.rig.input.now_us+=10000;
        const auto out=p.rig.task.update(p.rig.input);
        assert(q.visual_frames==1 && out.motion.vx_mps==0 && out.motion.wz_rps==0);
        assert(out.reason!="carry_pre_entry_aligned");
    }
    p.step();assert(q.visual_frames==2 && p.out.reason!="carry_pre_entry_aligned");
    assert(p.step().reason=="carry_pre_entry_aligned");
    p.rig.input.path_safe=false;
    assert(p.step().motion.vx_mps==0);
    std::cout<<"PASS near-entry requires sent stop and three distinct fresh images before forward motion\n";
}

void nearEntryReacquiresWithoutTranslation() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(bool frozen : {false,true}) for(bool wide : {false,true}) {
        JawRig p;p.gap=.377f;p.yaw=-.55f;p.image_error=-.185f;p.edge_override=-.193f;
        p.robot_x=p.rig.task.halfX()-.143f;
        bool aligning=false;
        for(int i=0;i<30;++i) {
            const auto out=p.step();assert(out.motion.vx_mps==0);
            if(out.reason=="carry_entry_image_align" || out.reason=="carry_entry_centre_align") {aligning=true;break;}
        }
        assert(aligning);
        p.view_evidence=p.pose=false;p.sight=false;
        p.repeat_pose_frame=p.stale_half=frozen;
        bool searched=false;int wide_ticks=0;
        for(int i=0;i<300;++i) {
            const auto out=p.step();assert(out.motion.vx_mps==0);
            searched|=out.reason=="carry_half_reacquire_turn" && out.motion.wz_rps!=0;
            assert(out.state==PushState::CARRY);
            if(out.reason=="carry_entry_half_search") {
                assert(out.motion.wz_rps!=0 && !out.immediate_motion_stop);++wide_ticks;
            }
            if((!wide && searched) || wide_ticks>=30)break;
        }
        assert(searched);
        if(wide) assert(wide_ticks>=30);
        p.rig.input.directional_clearance_valid=true;p.rig.input.turn_safe=false;
        for(int i=0;i<4;++i) {
            const auto out=p.step();assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        }
        p.rig.input.turn_safe=true;
        // Image recovery does not require a working metric entrance.
        p.view_evidence=true;p.image_error=0;p.edge_override=0;
        p.repeat_pose_frame=p.stale_half=false;
        bool recovered=false;
        for(int i=0;i<35;++i) {
            const auto out=p.step();assert(out.motion.vx_mps==0);
            if(out.reason=="carry_half_reacquired") {recovered=true;break;}
        }
        assert(recovered && p.rig.task.transport_phase_.jaw_sequence_.stage==Stage::AlignStop);
        p.repeat_pose_frame=true;p.stale_half=true;
        for(int i=0;i<10;++i) {
            const auto out=p.step();assert(out.motion.vx_mps==0 && out.reason!="carry_pre_entry_aligned");
        }
        p.stale_half=false;p.repeat_pose_frame=false;
        bool accepted=false;
        for(int i=0;i<50;++i) {
            const auto out=p.step();assert(out.motion.vx_mps==0);
            if(out.reason=="carry_pre_entry_aligned") {accepted=true;break;}
        }
        assert(accepted);
    }
    std::cout<<"PASS near-entry loss searches without translation, expands its search and restores image alignment\n";
}


// A visible selected half can temporarily lose its ground projection. Treat
// that as missing range, not as losing the visual target and starting a turn.
void visibleHalfRecoversWithoutEntranceRange() {
    for(bool intermittent_range : {false,true})
        for(bool injured : {false,true}) for(float side : {-1.f,1.f}) {
        JawRig p;p.yaw=0;p.gap=.8f;p.pose=false;p.half_entrance=true;
        p.ramped_dead_zone=true;p.angular_response=0;p.bearing=.4f*side;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=injured;d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_;p.supply_visible=!injured;p.injured_visible=injured;
        for(int i=0;i<6;++i)p.step();
        p.view_evidence=false;p.half_entrance=false;
        bool searching=false;
        for(int i=0;i<18;++i) {
            const auto out=p.step();
            searching |= out.reason=="carry_half_reacquire_turn";
        }
        assert(searching);
        p.view_evidence=true;
        p.step();
        // One capture reused by many control ticks must not finish search.
        for(int i=0;i<8;++i) {
            p.rig.input.now_us+=1000;
            const auto out=p.rig.task.update(p.rig.input);
            assert(out.reason!="carry_half_reacquired" && out.motion.vx_mps==0);
        }
        bool recovered=false;
        for(int i=0;i<25;++i) {
            p.half_entrance=intermittent_range && i%2==0;
            const auto out=p.step();
            assert(out.state==PushState::CARRY && out.motion.vx_mps==0);
            assert(out.motion.gripper_offset==20);
            if(out.reason=="carry_half_reacquired") {recovered=true;break;}
        }
        if(!recovered)std::cerr<<"visible half still searching without range: "<<p.out.reason<<std::endl;
        assert(recovered && "visible half must stop search without waiting for entrance projection");
        p.half_entrance=false;
        for(int i=0;i<12;++i) {
            const auto out=p.step();
            assert(out.reason=="carry_selected_half_range_wait");
            assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
            assert(d.jaw_sequence_.stage==TransportPhaseData::JawSequence::Stage::Approach);
        }
        p.half_entrance=true;
        bool advanced=false;
        for(int i=0;i<12;++i)advanced |= p.step().motion.vx_mps>0;
        assert(advanced && "fresh entrance range must resume approach");
    }
    std::cout<<"PASS visible selected half stops search through missing projection; range still gates travel\n";
}


// Requested 13:19 camera postures and the 13:18 stopped transport trace.
void requestedRelaxedEntryPostures() {
    using Stage=TransportPhaseData::JawSequence::Stage;
    for(bool injured : {false,true}) for(float side : {-1.f,1.f})
        for(auto stage : {Stage::EntryStop,Stage::AlignStop})
        for(const auto sample : {cv::Point2f(.009375f,.1368932f),
                                 cv::Point2f(.075390625f,.26062f),
                                 cv::Point2f(.0999155f,.11039f),
                                 cv::Point2f(.09999f,.30f)}) {
            JawRig p; p.yaw=.52f*side; p.gap=.35f;
            p.image_error=sample.x*side; p.edge_override=sample.y*side;
            auto& d=p.rig.task.transport_phase_;
            d.half_locked_=true;d.half_injured_=injured;
            d.half_x_=injured ? .15f : -.15f;
            p.robot_x=d.half_x_;p.supply_visible=!injured;p.injured_visible=injured;
            d.jaw_sequence_.stage=stage;
            bool accepted=false;
            for(int i=0;i<40;++i) {
                const auto out=p.step();
                assert(out.state==PushState::CARRY && out.motion.gripper_offset==20);
                assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
                if(out.reason=="carry_pre_entry_aligned") {
                    assert(p.rig.input.now_us>=1360000);
                    assert(d.jaw_sequence_.stage==Stage::ColourAdvance);
                    assert(p.rig.task.desiredPitch()==0);
                    accepted=true;break;
                }
                assert(p.rig.task.desiredPitch()==-3000);
            }
            assert(accepted && "requested in-half posture must enter without precision turns");
        }
    // The relaxed image thresholds still need the selected half and fresh,
    // distinct captures after actual zero-speed feedback.
    for(int missing=0;missing<5;++missing) {
        JawRig p;p.yaw=0;p.gap=.35f;p.image_error=.095f;p.edge_override=.11f;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true;d.half_injured_=false;d.half_x_=-.15f;
        d.jaw_sequence_.stage=Stage::AlignStop;
        if(missing==0) p.inside=false;
        if(missing==1) p.stale_half=true;
        if(missing==2) p.repeat_pose_frame=true;
        if(missing==3) {p.force_sent=true;p.forced_vx=.02f;}
        if(missing==4) p.view=false;
        // Repeat both image stamps while the control clock continues.
        if(missing==2) p.step();
        for(int i=0;i<25;++i) {
            PushOutput out;
            if(missing==2) {
                p.rig.input.now_us+=40000;
                out=p.rig.task.update(p.rig.input);
            } else out=p.step();
            assert(out.reason!="carry_pre_entry_aligned");
            assert(p.rig.task.desiredPitch()==-3000 && out.motion.vx_mps==0);
        }
    }
    std::cout<<"PASS requested camera postures accept <10% centre / 0.30 edge with stopped fresh-half evidence\n";
}

// 18:07 trace: range review remained latched after the entrance vanished,
// although a fresh own-zone box still supplied a direction.
void unstableRangeLossTurnsTowardOwnZone() {
    for (bool injured : {false,true}) for (float side : {-1.f,1.f})
        for (bool image_only : {false,true}) {
        JawRig p; p.gap=.65f; p.yaw=0;
        auto& d=p.rig.task.transport_phase_;
        d.half_locked_=true; d.half_injured_=injured; d.half_x_=injured ? .15f : -.15f;
        p.robot_x=d.half_x_; p.supply_visible=!injured; p.injured_visible=injured;
        auto& q=d.jaw_sequence_;
        for (int i=0;i<6;++i) p.step();
        p.gap=.30f;
        for (int i=0;i<8 && !q.range_wait;++i) p.step();
        assert(q.range_wait && q.stage==TransportPhaseData::JawSequence::Stage::Approach);
        p.pose=false; p.view_evidence=image_only; p.bearing=.687961f*side;
        q.approach_route.reset(); q.route_fix_us=0; // no recent accepted anchor
        p.rig.task.t_.transport_scan_wz=.6f;
        p.rig.task.t_.transport_align_min_wz=.6f;
        // Finish braking translation before using image direction to turn.
        p.force_sent=true; p.forced_vx=.03f;
        auto out=p.step();
        assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        p.forced_vx=0;
        bool turned=false;
        for (int i=0;i<12;++i) {
            out=p.step();
            assert(out.state==PushState::CARRY && out.motion.vx_mps==0);
            assert(out.motion.gripper_offset==20);
            if (out.motion.wz_rps!=0) {
                assert(out.reason=="carry_half_reacquire_turn" && out.motion.wz_rps*side>0);
                assert(std::abs(out.motion.wz_rps)<=.600001f);
                turned=true; break;
            }
        }
        assert(turned && "range loss must recover toward the own-zone box instead of staying stopped");
        p.rig.input.directional_clearance_valid=true; p.rig.input.turn_safe=false;
        out=p.step(); assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        p.rig.input.turn_safe=true; p.heading=false;
        out=p.step(); assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
        p.heading=true;
        if (!image_only) {
            // Exhausted local search still follows a fresh zone box; a stale
            // box pauses it, and a new box restores it without an external reset.
            q.half_search_leg=3;
            out=p.step();
            assert(out.motion.vx_mps==0 && out.motion.wz_rps*side>0);
            p.stale_sight=true;
            out=p.step(); assert(out.motion.vx_mps==0 && out.motion.wz_rps==0);
            p.stale_sight=false;
            out=p.step(); assert(out.motion.vx_mps==0 && out.motion.wz_rps*side>0);
        }
        p.pose=p.view_evidence=true; p.rig.input.arc_safe=true;
        p.bearing=0; p.image_error=0; p.gap=.65f;
        bool resumed=false;
        for (int i=0;i<50;++i) {
            out=p.step();
            if (out.motion.vx_mps>0) { resumed=true; break; }
        }
        assert(resumed && !q.range_wait && !q.approach_resume_pending);
    }
    std::cout<<"PASS unstable range loss turns toward own-zone direction, honours stop/clearance/IMU and resumes after fresh captures\n";
}

int main(int argc, char** argv) {
    unstableRangeLossTurnsTowardOwnZone();
    if(argc==2 && std::string(argv[1])=="--unstable-range-loss") return 0;
    if(argc==2 && std::string(argv[1])=="--retreat-timing") {
        retreatUsesIndependentStageBudgets();
        retreatTimeoutsIdentifyTheFailedStage();
        retreatPreparationSurvivesReleaseAndRejectsOpeningMotion();return 0;
    }
    retreatUsesIndependentStageBudgets();
    retreatTimeoutsIdentifyTheFailedStage();
    retreatPreparationSurvivesReleaseAndRejectsOpeningMotion();
    if(argc==2 && std::string(argv[1])=="--initial-approach-continuity") {
        initialApproachBridgesLoggedNoiseBursts();
        initialApproachFiltersRangeNoiseAndHonoursStops();return 0;
    }

    if(argc==2 && std::string(argv[1])=="--relaxed-entry") {requestedRelaxedEntryPostures();return 0;}
    requestedRelaxedEntryPostures();
    if(argc==2 && std::string(argv[1])=="--visual-half-recovery") {visibleHalfRecoversWithoutEntranceRange();return 0;}
    visibleHalfRecoversWithoutEntranceRange();
    if(argc==2 && std::string(argv[1])=="--near-entry-turn") {
        loggedNearEntryTurnsWithoutRetreat();nearEntryNeedsStoppedDistinctImages();nearEntryReacquiresWithoutTranslation();return 0;
    }
    loggedNearEntryTurnsWithoutRetreat();nearEntryNeedsStoppedDistinctImages();nearEntryReacquiresWithoutTranslation();
    if(argc==2 && std::string(argv[1])=="--continuous-reposition") {
        loggedRepositionAdvancesWithinCameraView();continuousRepositionRespectsLimitsAndRouteExpiry();return 0;
    }
    loggedRepositionAdvancesWithinCameraView();
    continuousRepositionRespectsLimitsAndRouteExpiry();
    if(argc==2 && std::string(argv[1])=="--prepush-calibration") {
        prePushCalibrationAcceptsStoppedImuNoise();return 0;
    }
    prePushCalibrationAcceptsStoppedImuNoise();
    if(argc==2 && std::string(argv[1])=="--diff-drive-reposition") {
        differentialDriveRepositionAllowsTransientImageDeviation();return 0;
    }
    differentialDriveRepositionAllowsTransientImageDeviation();
    if(argc==2 && std::string(argv[1])=="--entry-frame-hold") {
        loggedPreBackoffPostureHoldsAcrossFrameNoise();entryPostureHoldRespectsChangeAndFreshEvidence();return 0;
    }
    loggedPreBackoffPostureHoldsAcrossFrameNoise();
    entryPostureHoldRespectsChangeAndFreshEvidence();

    if (argc==2 && std::string(argv[1])=="--transport-inspection") {
        completeJawTransportLifecycle(true); return 0;
    }
    if (argc==2 && std::string(argv[1])=="--imu-retreat") { releasedCargoRetreatsImuFifteenCentimetres(); deliveryReturnsDirectlyToSearch(); return 0; }
    if(argc==2 && std::string(argv[1])=="--reverse-reposition") {nearEntryKeepsAlignmentAfterRangeChange();return 0;}
    nearEntryKeepsAlignmentAfterRangeChange();
    initialApproachBridgesLoggedNoiseBursts();
    initialApproachFiltersRangeNoiseAndHonoursStops();
    if(argc==2 && std::string(argv[1])=="--approach-continuity") {
        approachContinuityAcrossBriefEntranceNoise();
        approachRangeReviewKeepsRecentTrajectory();return 0;
    }
    approachContinuityAcrossBriefEntranceNoise();
    approachRangeReviewKeepsRecentTrajectory();
    if(argc==2 && std::string(argv[1])=="--intermittent-half-search") {intermittentHalfCannotPulseSearchBrakes();return 0;}
    intermittentHalfCannotPulseSearchBrakes();
    if(argc==2 && std::string(argv[1])=="--forward-entry-fix") {forwardEntryRegression();return 0;}
    forwardEntryRegression();
    if(argc==2 && std::string(argv[1])=="--lateral-range-review") {lateralEntranceJumpsRequireStationaryReview();return 0;}
    lateralEntranceJumpsRequireStationaryReview();
    if(argc==2 && std::string(argv[1])=="--reposition-loss") {repositionLossKeepsSearching();return 0;}
    repositionLossKeepsSearching();
    if(argc==2 && std::string(argv[1])=="--entry-range-review") {entryRangeReviewAndForwardReposition();return 0;}
    entryRangeReviewAndForwardReposition();

    if(argc==2 && std::string(argv[1])=="--ui120-region") {
        directNearViewPushKeepsColourSpeed();
        nearContourNeedsDistinctStoppedCaptures();
        twentyCentimetreEntryAndColourRelease();
        completeJawTransportLifecycle();
        return 0;
    }
    nearContourNeedsDistinctStoppedCaptures();
    if(argc==2 && std::string(argv[1])=="--logged-deliverable-entry") {
        loggedDeliverableEntryStartsNearViewPush();return 0;
    }
    loggedDeliverableEntryStartsNearViewPush();
    if(argc==2 && std::string(argv[1])=="--half-loss-five-frames") {
        briefMissingHalvesKeepPlannedRouteBeforeSearching();return 0;
    }
    briefMissingHalvesKeepPlannedRouteBeforeSearching();
    if(argc==2 && (std::string(argv[1])=="--entry-tolerance" || std::string(argv[1])=="--five-percent-entry")) {
        loggedInHalfEntryAcceptsWithoutTurning();
        entryEdgeThirtyPercentBoundary();
        centreTurnBrakesOnBadSlopeOrBlockedClearance();return 0;
    }
    loggedInHalfEntryAcceptsWithoutTurning();
    entryEdgeThirtyPercentBoundary();
    centreTurnBrakesOnBadSlopeOrBlockedClearance();
    if(argc==2 && std::string(argv[1])=="--parking-source") {
        selectedHalfDropoutCannotParkFromWholeZone();return 0;
    }
    if(argc==2 && std::string(argv[1])=="--parking-replan") {
        stationaryOffsetWithSpaceResumesJointApproach();
        reapproachNeedsStableRangeAndRoom();resumedApproachConvergesThroughSender();return 0;
    }
    if(argc==2 && std::string(argv[1])=="--turn-brake") {
        imageTurnBrakesBeforeNextImuStep();return 0;
    }
    selectedHalfDropoutCannotParkFromWholeZone();
    stationaryOffsetWithSpaceResumesJointApproach();
    reapproachNeedsStableRangeAndRoom();resumedApproachConvergesThroughSender();
    imageTurnBrakesBeforeNextImuStep();

    if(argc==2 && std::string(argv[1])=="--entry-loop-fix") {
        outOfToleranceStationaryConflictTerminates(); jointApproachReachesParkingPoseThroughSender();
        jointApproachPreservesClearanceAndStationaryEvidence(); return 0;
    }
    outOfToleranceStationaryConflictTerminates();
    jointApproachReachesParkingPoseThroughSender();
    jointApproachPreservesClearanceAndStationaryEvidence();
    if(argc==2 && std::string(argv[1])=="--direct-push") { confirmedAlignmentDirectlyStartsNearViewPush(); return 0; }
    confirmedAlignmentDirectlyStartsNearViewPush();
    unalignedEntryCentresWithoutTranslation();
    if(argc==2 && std::string(argv[1])=="--zone-hold-fix") {
        confirmedZoneSightDoesNotRepeatEnemyHolds(); return 0;
    }
    if(argc==2 && std::string(argv[1])=="--half-turn-fix") {
        selectedHalfDropoutWithoutRouteWaitsForFiveFrames(); imageTurnContinuesUntilFinalObservation(); return 0;
    }
    confirmedZoneSightDoesNotRepeatEnemyHolds();
    selectedHalfDropoutWithoutRouteWaitsForFiveFrames();
    imageTurnContinuesUntilFinalObservation();

    if(argc==2 && std::string(argv[1])=="--pre-entry") { alignmentMustFinishBeforeTwentyCentimetres(); preEntryAlignmentCompletesAndHoldsItsTarget(); return 0; }
    alignmentMustFinishBeforeTwentyCentimetres();
    preEntryAlignmentCompletesAndHoldsItsTarget();
    if(argc==2 && std::string(argv[1])=="--logged-steering-loop") {
        loggedSteeringDropoutsConvergeWithoutRetreatLoop(); return 0;
    }
    loggedSteeringDropoutsConvergeWithoutRetreatLoop();

    if(argc==2 && std::string(argv[1])=="--steering-dropout") {
        confirmedApproachNormalSurvivesIntermittentCaptures(); return 0;
    }
    confirmedApproachNormalSurvivesIntermittentCaptures();


    if(argc==2 && std::string(argv[1])=="--early-approach") {
        earlyApproachScalesTurnBeforeTheStopLine(); return 0;
    }
    earlyApproachScalesTurnBeforeTheStopLine();
    if(argc==2 && std::string(argv[1])=="--efficient-entry") {
        efficientEntryAcceptsSmallOffsets();
    outOfToleranceEntryCentresWithoutForwardCorrection(); return 0;
    }
    efficientEntryAcceptsSmallOffsets();
    outOfToleranceEntryCentresWithoutForwardCorrection();
    if(argc==2 && std::string(argv[1])=="--fresh-half-overrides-hold") { freshHalfOverridesHeldApproachHeading(); return 0; }
    freshHalfOverridesHeldApproachHeading();
    if(argc==2 && std::string(argv[1])=="--logged-approach-target") { loggedApproachKeepsSelectedHalfAsSteeringTarget(); return 0; }
    if(argc==2 && std::string(argv[1])=="--entry-dropout-window") { entryDropoutWaitsForStationaryObservation(); return 0; }
    if(argc==2 && std::string(argv[1])=="--intermittent-half-loop") { intermittentHalfApproachConvergesThroughSender(); return 0; }
    loggedApproachKeepsSelectedHalfAsSteeringTarget();
    intermittentHalfApproachConvergesThroughSender();
    entryDropoutWaitsForStationaryObservation();
    if(argc==2 && std::string(argv[1])=="--logged-offset-loop") { loggedOffsetApproachConvergesWithoutRetreatLoop(); return 0; }
    loggedOffsetApproachConvergesWithoutRetreatLoop();
    if(argc==2 && std::string(argv[1])=="--offset-direction") {
        offsetImageCannotStartATurnAwayFromTheSelectedHalf(); return 0;
    }
    offsetImageCannotStartATurnAwayFromTheSelectedHalf();
    if(argc==2 && std::string(argv[1])=="--independent-half") { independentHalfEntranceAcceptsDeliverableEntry(); return 0; }
    independentHalfEntranceAcceptsDeliverableEntry();
    if(argc==2 && std::string(argv[1])=="--tilted-entry") {
        visibleTiltTakesPriorityOverMetricHeading();
        tiltedImageConvergesThroughSenderRamp();
        return 0;
    }
    visibleTiltTakesPriorityOverMetricHeading();
    tiltedImageConvergesThroughSenderRamp();
    if(argc==2 && std::string(argv[1])=="--turn-fix") {
        loggedRightSideRecoveryTurnsRight();
        carryStationaryTurnsUseSearchRate();
        entryLandmarkLossTurnsTowardFreshOwnZone();
        approachWithoutLocalEvidenceUsesZoneDirection();
        poseLossStartsDirectionalRecovery();
        poseLossMeasuredRecoveryAndSearch();
        stablePoseNormalSurvivesLandmarkDropout();
        poseNormalRejectsJumpsAndUsesCaptureHeading();
        imagePulseMustFinishBrakingBeforeAnotherCorrection();
        return 0;
    }
    if(argc==2 && std::string(argv[1])=="--logged-right-recovery") { loggedRightSideRecoveryTurnsRight(); return 0; }
    if(argc==2 && std::string(argv[1])=="--carry-turn-speed") { carryStationaryTurnsUseSearchRate(); return 0; }
    loggedRightSideRecoveryTurnsRight();
    carryStationaryTurnsUseSearchRate();
    selectedHalfAloneCanRecoverAndAlign();
    if (argc == 2 && std::string(argv[1]) == "--selected-half") return 0;
    if(argc==2 && std::string(argv[1])=="--joint-entry") {

        centredApproachClosedLoop();
        selectedSubzoneMustBeCentred();
        twentyCentimetreEntryAndColourRelease();
        return 0;
    }


    directNearViewPushKeepsColourSpeed();
    if(argc==2 && std::string(argv[1])=="--centre-handoff") {
        twentyCentimetreEntryAndColourRelease();
        completeJawTransportLifecycle();
        return 0;
    }
    if(argc==2 && std::string(argv[1])=="--log-alignment") {
        transportScanUsesConfiguredAngularSpeed();
        imagePulseMustFinishBrakingBeforeAnotherCorrection();
        rejectedNormalCannotReturnDuringTheSameEntry();
        return 0;
    }
    transportScanUsesConfiguredAngularSpeed();

    if(argc==2 && std::string(argv[1])=="--align-braking") {
        imagePulseMustFinishBrakingBeforeAnotherCorrection(); return 0;
    }
    if(argc==2 && std::string(argv[1])=="--normal-rejection") {
        rejectedNormalCannotReturnDuringTheSameEntry(); return 0;
    }
    imagePulseMustFinishBrakingBeforeAnotherCorrection();
    rejectedNormalCannotReturnDuringTheSameEntry();
    if(argc==2 && std::string(argv[1])=="--zone-direction") {
        detectorBearingConvergesInTheBodyFrame(); return 0;
    }
    detectorBearingConvergesInTheBodyFrame();
    if(argc==2 && std::string(argv[1])=="--pose-imu") {
        stablePoseNormalSurvivesLandmarkDropout();
        poseNormalRejectsJumpsAndUsesCaptureHeading();
        stableImageConflictFallsBackFromBiasedMetricNormal();
        delayedImageLateralCompensationUsesCaptureYaw();
        poseNormalExpiresAndStillNeedsFreshCentring(); return 0;
    }
    stablePoseNormalSurvivesLandmarkDropout();
    poseNormalRejectsJumpsAndUsesCaptureHeading();
    stableImageConflictFallsBackFromBiasedMetricNormal();
    delayedImageLateralCompensationUsesCaptureYaw();
    poseNormalExpiresAndStillNeedsFreshCentring();
    entryLandmarkLossTurnsTowardFreshOwnZone();
    if(argc==2 && std::string(argv[1])=="--entry-landmark-loss") return 0;
    if(argc==2 && std::string(argv[1])=="--approach-local-loss") { approachWithoutLocalEvidenceUsesZoneDirection(); return 0; }
    approachWithoutLocalEvidenceUsesZoneDirection();
    if(argc==2 && std::string(argv[1])=="--image-align") { imageAlignmentIgnoresMetricYawAndSurvivesMappingLoss(); return 0; }
    imageAlignmentIgnoresMetricYawAndSurvivesMappingLoss();
    shortZoneDropoutHoldsBeforeScanning();
    if(argc==2 && std::string(argv[1])=="--pose-loss-recovery") { poseLossStartsDirectionalRecovery(); poseLossMeasuredRecoveryAndSearch(); return 0; }
    poseLossStartsDirectionalRecovery(); poseLossMeasuredRecoveryAndSearch();
    if(argc==2 && std::string(argv[1])=="--continuous-zone-scan") { missingZoneScanRemainsContinuous(); return 0; }
    missingZoneScanRemainsContinuous();
    if(argc==2 && std::string(argv[1])=="--entry-colour-return") { centredApproachClosedLoop(); selectedSubzoneMustBeCentred(); twentyCentimetreEntryAndColourRelease(); metricAndHeadingAreRequiredForEntry(); deliveryReturnsDirectlyToSearch(); releasedCargoRetreatsImuFifteenCentimetres(); completeJawTransportLifecycle(); return 0; }
    if (argc == 2 && std::string(argv[1]) == "--deposit-drift-blocked") {
        blockedStationaryImuDriftCannotRelease();
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--deposit-drift-zero-sent") {
        zeroSentVelocityImuDriftCannotRelease();
        return 0;
    }
    centredApproachClosedLoop(); selectedSubzoneMustBeCentred(); twentyCentimetreEntryAndColourRelease();
    metricAndHeadingAreRequiredForEntry();
    deliveryReturnsDirectlyToSearch();
    releasedCargoRetreatsImuFifteenCentimetres();
    completeJawTransportLifecycle();
    carryPlacesInSelectedHalfBeforeOpening();
    destinationIsLockedWhenCarryStarts();
    ownZoneDropoutsCannotRestoreFarAnchorSteering();
    ownZoneSightOverridesConflictingFarAnchor();
    ownSightDoesNotReplacePoseArrival();
    enemySightCannotEraseOwnCarryHeading();
    enemyZonesDoNotStartAManeuver();
    ownZoneBoxesDriveWithoutPoseKeypoints();
    for (float yaw : {-.2f, .2f, -.8f, .8f})
        for (float response : {1.f, 6.f})
            for (bool intermittent : {false, true})
                convergesWithoutRepeatedReversals(yaw, response, intermittent);
    smallErrorsDoNotRequestTheTurnFloor();
    blockedStationaryImuDriftCannotRelease();
    zeroSentVelocityImuDriftCannotRelease();
    depositImuGapStopsAlreadyRequestedMotion();
    depositWaitsForSentStopAndImuCalibration();
    floorPlacementUsesColourInsteadOfFifteenCentimetres();
    floorPlacementCannotReleaseOnMissingOrStaleEvidence();
    depositKeepsMappedTransportView();
    depositUsesImuAndStopsBeforeOpening();
    depositCannotUseCommandTravelOrMissingImu();
    nearEdgeLargeTurnRetainsInPlaceAlignment();
    forwardClearanceStillStopsTheCarry();
    std::cout << "carry task steering OK\n";
}
