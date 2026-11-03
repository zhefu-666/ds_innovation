#include "../support/push_task_sim.hpp"
#include "../../src/task/task_tick.hpp"
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <stdexcept>
#define private public
#include "rescue/hardware/uart_controller.hpp"
#undef private
#include "rescue/app/observation_builder.hpp"
#include "rescue/geometry/subzone_entrance.hpp"
#include "rescue/perception/perception_adapter.hpp"
#include "rescue/app/runtime_config.hpp"
#include "rescue/hardware/sensor_fusion.hpp"
#include "rescue/control/motion_watch.hpp"
#include "rescue/control/controlled_field.hpp"

void independentHalfEntranceIgnoresDividerDisagreement() {
    rescue::CameraCalibration camera;
    camera.setIntrinsics((cv::Mat_<double>(3,3) << 900,0,640, 0,900,360, 0,0,1),
                         cv::Mat::zeros(1,5,CV_64F));
    camera.setGroundHomography((cv::Mat_<double>(3,3) << .001,0,-.64, 0,-.001,1, 0,0,1),-3000);
    rescue::ZoneGeometry zone;
    zone.width_m=zone.landmark_width_m=.6f; zone.confirmed=true;
    rescue::SensorState imu; imu.imu_valid=true;
    for (int chosen : {0,1}) {
        rescue::ZoneHalves halves;
        for (int i=0;i<2;++i) {
            auto& h=halves[i]; h.valid=true; h.score=.95f; h.confidence.fill(.95f);
            const float x=i==chosen ? 490.f : (chosen==0 ? 800.f : 180.f);
            const float y=i==chosen ? 0.f : -40.f;
            h.keypoints={cv::Point2f{x,200+y},cv::Point2f{x,500+y},
                         cv::Point2f{x+300,200+y},cv::Point2f{x+300,500+y}};
        }
        const auto merged=rescue::zoneHalvesToKeypoints(halves,.6f,{1280,720});
        for(const auto& point : merged) assert(point.id!=1 && point.id!=4);
        rescue::PushObservation in; in.now_us=1000000;
        const auto measure = [&] {
            rescue::applySubzoneView(in,halves,zone,{1280,720},true,true,-.15f,.15f,.6f,in.now_us);
            auto& view=chosen==0 ? in.supply_view : in.injured_view;
            rescue::measureSubzoneEntrance(view,halves[chosen],zone,camera,-3000,&imu);
            assert(view.valid && view.entrance_valid && view.inside);
            rescue::ZoneEstimate frame;
            frame.origin_body_m=view.entrance_origin_body_m;
            frame.yaw_body_rad=view.entrance_yaw_rad;
            const auto robot=frame.bodyToZone({0,0});
            assert(std::abs(robot.x-view.target_x)<1e-5f && std::abs(robot.y+.5f)<1e-5f);
            assert(std::abs(view.error_fraction)<1e-6f && std::abs(view.edge_slope)<1e-6f);
            return view;
        };
        const auto baseline=measure();
        halves[1-chosen].valid=false;
        const auto absent=measure();
        assert(cv::norm(baseline.entrance_origin_body_m-absent.entrance_origin_body_m)<1e-6f);
        auto invalid=absent;
        rescue::measureSubzoneEntrance(invalid,halves[chosen],zone,camera,0,&imu);
        assert(!invalid.entrance_valid && invalid.valid);
    }
    std::cout<<"PASS raw selected-half entrance ignores mismatched or absent other-half divider\n";
}

int main(int argc, char** argv) {
    independentHalfEntranceIgnoresDividerDisagreement();
    if(argc==2 && std::string(argv[1])=="--independent-half") return 0;
    const bool near_test = argc == 2 && std::string(argv[1]) == "--near-selection-only";
    rescue::ResolvedRuntimeConfig cfg;
    cfg.request.frame_width = 1280;
    cfg.request.frame_height = 720;
    cfg.request.confidence = .25f;
    cfg.capture.pixel_grasp_enabled = true;
    cfg.capture.grasp_center_ranges = {
        {"ordinary_supply", 2000, 1280, 720, {342, 226, 1031, 489}, {647.5f, 337}, true}};
    if (near_test) {
        cfg.capture.pixel_center_membership = true;
        cfg.capture.grasp_center_ranges[0].area = {209, 180, 1071, 500};
    }
    cfg.task.pixel_grasp_enabled = true;
    cfg.task.require_multi_view = false;
    PushTask task{cfg.task};
    task.trip_context_.target_id_ = task.trip_context_.capture_target_id_ = 1;
    task.trip_context_.label_ = "ordinary_supply";
    PushOutput previous;
    previous.phase = TaskPhase::Capture;
    previous.state = PushState::RUSH;
    previous.run_status = RunStatus::Running;
    previous.target_id = -1; // Acquire through the real selector before the task adopts its ID.
    previous.motion.vx_mps = previous.motion.wz_rps = 0;
    CameraCalibration camera;
    ZoneGeometry zone;
    ZoneColorClassifier colors;
    ControlledField field(zone, .1f);
    SensorFusion fusion;
    MotionWatch watch;
    UARTController feedback;
    // Feed causal history in memory; no serial port, sender or reader is opened.
    feedback.running_.store(true);
    rescue::app::ObservationBuilder builder(cfg, nullptr, camera, false, zone, colors, field, fusion);
    auto* active_builder = &builder;
    cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar(0));
    rescue::PerceptionResult vision;
    int sequence = 0;
    uint64_t last_feedback_us = 0;
    int camera_pitch = 2000;
    std::vector<SegDetection> selection_scene;
    auto step = [&](int danger_x, bool link = true, cv::Rect danger_box = cv::Rect{},
                    bool zone_box = false, cv::Rect safe_box = {400,200,450,350}) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        const uint64_t stamp = uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        if (!sequence)
            for (int i = 5; i > 0; --i) {
                ActuatorFeedback sample;
                sample.timestamp_us = stamp - uint64_t(i) * 40000;
                sample.camera_pitch_cdeg = camera_pitch;
                sample.valid = true;
                feedback.publishActuatorFeedback(sample);
            }
        ActuatorFeedback sample;
        sample.camera_pitch_cdeg = camera_pitch;
        sample.valid = true;
        if (near_test) {
            sample.gripper_action_id = 1;
            sample.gripper_open = 1;
        }
        // Real servo feedback arrives faster than the image loop. Supply a causal
        // intermediate sample so the 50 ms skew gate does not depend on sleep jitter.
        if (!last_feedback_us)
            last_feedback_us = stamp - 200000;
        for (uint64_t t = last_feedback_us + 40000; t < stamp; t += 40000) {
            sample.timestamp_us = t;
            feedback.publishActuatorFeedback(sample);
        }
        sample.timestamp_us = stamp;
        feedback.publishActuatorFeedback(sample);
        last_feedback_us = stamp;
        SegDetection target;
        target.track_id = 1; target.label = "ordinary_supply";
        target.box = {500, 235, 213, 264}; target.confidence = .95f; target.timestamp_us = stamp;
        SegDetection danger;
        danger.track_id = 29; danger.label = "dangerous_object";
        danger.box = danger_box.empty() ? cv::Rect{danger_x, 94, 254, 264} : danger_box; danger.confidence = .95f; danger.timestamp_us = stamp;
        std::vector<SegDetection> detections{target, danger};
        if (zone_box) {
            SegDetection safe;
            safe.label = cfg.request.team + "_safe_zone";
            safe.confidence = .95f; safe.timestamp_us = stamp;
            safe.box = safe_box;
            detections.push_back(safe);
        }
        if (near_test) {
            detections = selection_scene;
            for (auto& d : detections)
                d.timestamp_us = stamp;
        }
        return active_builder->build(frame, vision, detections, ++sequence, stamp,
                             {task, previous, &feedback, nullptr, watch, link}).input;
    };
    if(argc==2 && std::string(argv[1])=="--zone-sight-only") {
        cfg.request.team="blue";
        previous.phase=TaskPhase::Transport;
        ZoneColorConfig measured_color; measured_color.measured=true;
        ZoneColorClassifier measured_colors(measured_color);
        camera.setIntrinsics((cv::Mat_<double>(3,3) << 900,0,640, 0,900,360, 0,0,1),
                             cv::Mat::zeros(1,5,CV_64F));
        rescue::app::ObservationBuilder pose_builder(cfg,nullptr,camera,true,zone,measured_colors,field,fusion);
        active_builder=&pose_builder;
        vision.pose_ran=true;
        for(int half=0;half<2;++half) {
            auto& h=vision.halves[half];
            h.valid=true;h.score=.95f;h.confidence.fill(.95f);
            const float x=200+200*half;
            h.keypoints={cv::Point2f{x,200},cv::Point2f{x,500},
                         cv::Point2f{x+200,200},cv::Point2f{x+200,500}};
            h.box={x,200,200,300};
        }
        frame.setTo(cv::Scalar(160,100,60));
        PushObservation pose;
        for(int i=0;i<6;++i) pose=step(1040);
        assert(pose.zone_sight_valid && !pose.zone_sight_enemy && pose.zone_sight_bearing>0);
        assert(pose.zone_sight_sample_us && pose.zone_sight_sample_us<=pose.now_us);
        assert(pose.now_us-pose.zone_sight_sample_us<=200000 && !pose.geometry_valid);
        frame.setTo(cv::Scalar(0,0,200));
        pose=step(1040);
        assert(pose.zone_sight_valid && pose.zone_sight_enemy && pose.zone_sight_sample_us);
        frame.setTo(cv::Scalar(150,150,150));
        pose=step(1040);
        assert(!pose.zone_sight_valid && !pose.zone_sight_sample_us);
        std::cout<<"PASS real builder pose-only sight has capture timestamp and measured identity\n";
        return 0;
    }
    if (near_test) {
        feedback.done_flag_ = feedback.gripper_sent_ = true;
        feedback.gripper_action_id_ = 1;
        feedback.gripper_target_ = frameAngle(FrameAction::Open);
        const auto block = [](int id, cv::Rect box) {
            SegDetection d;
            d.track_id = id;
            d.label = "ordinary_supply";
            d.confidence = .95f;
            d.box = box;
            return d;
        };
        // Logged 00:06 capture scene: clipped #3 is larger than complete, in-frame #4.
        auto logged_core = block(2, {592, 339, 152, 148});
        logged_core.label = "core_supply";
        auto logged_injured = block(1, {80, 276, 507, 305});
        logged_injured.label = "injured_person";
        selection_scene = {block(3, {1052, 109, 228, 254}),
                           block(4, {354, 134, 244, 184}),
                           block(5, {0, 124, 152, 305}),
                           logged_core,
                           logged_injured};
        PushObservation chosen;
        for (int i = 0; i < 6; ++i)
            chosen = step(1040);
        std::cout << "logged scene selected=" << chosen.target_id << " expected=4" << std::endl;
        assert(chosen.target_valid && chosen.target_id == 4);
        assert(chosen.target_grasp_range_inside);
        std::cout << "PASS logged near scene prefers complete in-frame candidate\n";
        previous.target_id = 4;
        selection_scene.push_back(block(6, {550, 250, 300, 220}));
        for (int i = 0; i < 5; ++i)
            assert(step(1040).target_id == 4); // Keep a valid inside lock.
        selection_scene[1].box = {1052, 109, 228, 254};
        previous.motion.vx_mps = .02f;
        for (int i = 0; i < 5; ++i)
            assert(step(1040).target_id == 4); // Never switch while moving.
        previous.motion.vx_mps = 0;
        assert(step(1040).target_id == 4);
        assert(step(1040).target_id == 4);
        chosen = step(1040);
        assert(chosen.target_id == 6 && chosen.pixel_reselected_from == 4);
        assert(chosen.pixel_rebound_from == -1 && chosen.gripper_done);
        // A different physical object must discard old scene/entry/alignment confirmations.
        task.enter(PushState::RUSH, chosen.now_us - 50000, "near_reselection_fixture");
        task.run_status_ = RunStatus::Running;
        task.frame_raised_ = 1;
        task.pitch_cmd_ = 2000;
        task.last_us_ = chosen.now_us - 50000;
        task.trip_context_.target_id_ = task.trip_context_.capture_target_id_ = 4;
        task.trip_context_.label_ = "ordinary_supply";
        task.trip_context_.pending_.ordinary = 2;
        task.capture_scene_.objects.push_back({99, "ordinary_supply", {400, 250, 100, 100}, true});
        task.capture_scene_.started_us = chosen.now_us - 1000000;
        task.capture_scene_.waiting = true;
        task.search_entry_.confirmed = true;
        task.pixel_alignment_.target_id = 4;
        task.capture_phase_.pixel_forward_us_ = chosen.now_us - 50000;
        chosen.run = chosen.safety_ok = true;
        const auto switched = task.update(chosen);
        assert(switched.target_id == 6 && switched.reason == "near_target_reselected");
        assert(stopped(switched.motion) && switched.immediate_motion_stop);
        assert(task.capture_scene_.objects.empty() && !task.search_entry_.confirmed);
        assert(task.pixel_alignment_.target_id == -1 && !task.capture_phase_.pixel_forward_us_);
        assert(task.trip_context_.pending_.total() == 0);
        assert(task.capture_scene_.started_us == chosen.now_us - 1000000);
        previous.target_id = 6;
        auto reviewed = step(1040);
        assert(reviewed.pixel_reselected_from == -1); // Event acknowledged once.
        TaskTick mixed_tick(reviewed);
        assert(!task.reviewFirstTripCapture(mixed_tick, "approach"));
        assert(task.capture_scene_.status == "risk" && mixed_tick.immediate_motion_stop);
        assert(stopped(mixed_tick.motion)); // In-frame selection never authorizes mixed closure.
        std::cout << "PASS stopped three-frame replacement resets task evidence and keeps budget\n";
        // Exercise the selector with calibrated image sizes, class gates and far-view fallback.
        const auto select = [&](std::vector<SegDetection> scene,
                                int16_t pitch,
                                bool first = false,
                                const std::string& prefer = "") {
            PixelSelector selector;
            PixelSelection out;
            for (int i = 0; i < 3; ++i) {
                const uint64_t now = 1000000 + i * 50000;
                for (auto& d : scene)
                    d.timestamp_us = now;
                out = selector.update(scene,
                                      {1280, 720},
                                      pitch,
                                      true,
                                      now,
                                      first,
                                      .25f,
                                      {},
                                      prefer,
                                      std::numeric_limits<float>::quiet_NaN(),
                                      1.f,
                                      &cfg.capture);
            }
            return out;
        };
        auto clipped = block(3, {1052, 109, 228, 254});
        auto inside = block(4, {354, 134, 244, 184});
        auto outside = block(7, {1090, 200, 100, 150});
        assert(select({clipped, outside}, 2000).id == 7);
        assert(!select({clipped}, 2000).locked);
        assert(select({clipped, inside}, 0).id == 3); // Far/UI120 retains its area policy.
        auto danger = inside;
        danger.track_id = 8;
        danger.label = "dangerous_object";
        assert(!select({danger}, 2000).locked);
        auto core = inside;
        core.track_id = 9;
        core.label = "core_supply";
        assert(!select({core}, 2000).locked); // First trip only ordinary.
        cfg.capture.grasp_center_ranges.push_back(
            {"core_supply", 2000, 1280, 720, {268.5f, 236, 1017, 555}, {647.5f, 337}, true});
        core.box = {600, 340, 150, 140};
        assert(select({core}, 2000, true).id == 9);
        assert(select({core, inside}, 2000, true, "core_supply").id == 9);
        std::cout << "PASS complete fallback, clipped-only wait, far policy and trip class gates\n";
        PushTask stalled(cfg.task);
        stalled.enter(PushState::RUSH, chosen.now_us, "clipped_review_fixture");
        stalled.frame_raised_ = 1;
        stalled.trip_context_.target_id_ = stalled.trip_context_.capture_target_id_ = 3;
        stalled.trip_context_.label_ = "ordinary_supply";
        auto evidence = chosen;
        evidence.target_id = 3;
        evidence.label = "ordinary_supply";
        evidence.target_valid = true;
        evidence.heading_valid = true;
        evidence.heading_rad = 0;
        CaptureMonitor monitor(cfg.capture);
        for (int i = 0; i < 8 && !stalled.search_phase_.first_trip_split_.pending; ++i) {
            evidence.now_us += 50000;
            clipped.timestamp_us = evidence.now_us;
            monitor.update(evidence, {clipped}, evidence.now_us);
            TaskTick tick(evidence);
            assert(!stalled.reviewFirstTripCapture(tick, "approach"));
            assert(tick.immediate_motion_stop && stopped(tick.motion));
        }
        assert(stalled.search_phase_.first_trip_split_.pending);
        assert(stalled.reason_ == "first_trip_scene_reobserve_turn");
        std::cout
            << "PASS clipped locked target without alternative enters bounded reobservation\n";
        return 0;
    }
    PushObservation input;
    for (int i = 0; i < 6; ++i)
        input = step(1040);
    if (!input.target_valid || !input.capture_path_valid || !input.camera_pitch_stable)
        std::cerr << "builder fixture target=" << input.target_valid
                  << ",path=" << input.capture_path_valid << ",stable=" << input.camera_pitch_stable
                  << ",pitch=" << input.camera_pitch_cdeg << ",id=" << input.target_id
                  << ",label=" << input.label << "\n";
    assert(input.target_valid && input.capture_path_valid && input.camera_pitch_stable);
    previous.target_id = input.target_id;
    input = step(1021);
    assert(input.capture_dangerous_side_pending);
    for (int i = 0; i < 8; ++i)
        input = step(1040);
    assert(input.sent_velocity_valid && input.sent_vx_mps == 0 && input.sent_wz_rps == 0);
    assert(!input.capture_dangerous_side_pending);
    input = step(1021);
    assert(input.capture_dangerous_side_pending);
    previous.motion.vx_mps = .02f; // Actual sent braking speed is not a settled stop.
    for (int i = 0; i < 8; ++i)
        input = step(1040);
    assert(input.sent_vx_mps == .02f && input.capture_dangerous_side_pending);
    previous.motion.vx_mps = 0;
    for (int i = 0; i < 8; ++i)
        input = step(1040, false);
    assert(!input.sent_velocity_valid && input.capture_dangerous_side_pending);
    for (int i = 0; i < 8; ++i)
        input = step(1040);
    assert(!input.capture_dangerous_side_pending);
    // Use the real builder and its actual sent velocity wiring for the upper edge.
    // Establish a shallow observation first, then confirm the current Y176 bottom
    // through top-clipped images; complete remains false for those detections.
    input = step(879, true, {879, 8, 244, 173});
    assert(input.capture_dangerous_side_pending);
    for (int i = 0; i < 8; ++i)
        input = step(889, true, i % 2 ? cv::Rect{889, 13, 244, 163}
                                    : cv::Rect{889, 0, 244, 176});
    assert(!input.capture_dangerous_side_pending);
    input = step(879, true, {879, 8, 244, 173});
    assert(!input.capture_dangerous_side_pending);
    assert(!input.captureInventoryOverlaps({879, 8, 244, 173}, "dangerous_object"));
    assert(!input.capturePathOverlaps({879, 8, 244, 173}, "dangerous_object"));
    previous.motion.vx_mps = .02f;
    input = step(879, true, {879, 8, 244, 173});
    assert(input.sent_velocity_valid && input.sent_vx_mps == .02f);
    assert(input.capture_dangerous_side_pending);
    previous.motion.vx_mps = 0;
    for (int i = 0; i < 8; ++i)
        input = step(889, true, {889, 0, 244, 176});
    assert(!input.capture_dangerous_side_pending);
    input = step(879, false, {879, 8, 244, 173});
    assert(!input.sent_velocity_valid && input.capture_dangerous_side_pending);
    input = step(1040, true, {}, true);
    assert(input.delivery_observed && input.delivery_box_count == 1);
    assert(input.delivery_box_sample_us && !input.geometry_valid);
    input = step(1040);
    assert(!input.delivery_observed && input.delivery_box_count == 0);
    // The raw UI140 image reaches the real task observation, before drawing overlays.
    cfg.pushboard_floor.enabled = true;
    frame.setTo(cv::Scalar(160,100,60)); // HSV around100: blue floor
    input = step(1040);
    assert(input.pushboard_floor.valid && input.pushboard_floor.inside);
    assert(input.pushboard_floor.sample_us < input.now_us);
    assert(input.pushboard_floor.windows[0].own > .99f);
    frame.setTo(cv::Scalar(150,150,150));
    input = step(1040);
    assert(input.pushboard_floor.valid && !input.pushboard_floor.inside);
    cfg.pushboard_floor.enabled = false;
    // Real builder: sighted own-zone recovery needs no metric pose; UI120 uses the unpainted image.
    // No geometry pipeline or serial device exists; only run/IMU/gripper acknowledgements
    // and heading are supplied by this fixture. Never seed path_safe, turn_safe or arc_safe.
    cfg.request.controlled_empty_field = true;
    cfg.task.far_pitch_cdeg = -3000;
    cfg.task.track_pitch_cdeg = 0;
    cfg.task.near_pitch_cdeg = 2000;
    camera.setIntrinsics((cv::Mat_<double>(3,3) << 900,0,640, 0,900,360, 0,0,1),
                         cv::Mat::zeros(1,5,CV_64F));
    rescue::app::ObservationBuilder transport_builder(cfg, nullptr, camera, true,
                                                       zone, colors, field, fusion);
    active_builder = &transport_builder;
    task.t_.field_carry = true;
    task.t_.far_pitch_cdeg = -3000;
    task.t_.track_pitch_cdeg = 0;
    task.t_.near_pitch_cdeg = 2000;
    task.t_.pushboard_floor_enabled = true;
    task.t_.carry_speed = .05f;
    task.clearTrip();
    task.trip_context_.trip_.ordinary = 1;
    task.frame_raised_ = task.wire_last_ = 0;
    const auto now = uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    task.enter(PushState::CARRY, now, "transport_clearance_regression");
    task.transport_phase_.carry_us_ = now;
    task.last_us_ = now;
    task.pitch_cmd_ = task.transport_phase_.approach_pitch_ = -3000;
    previous = {};
    previous.phase = TaskPhase::Transport;
    previous.state = PushState::CARRY;
    previous.run_status = RunStatus::Running;
    auto transportStep = [&](cv::Rect box) {
        camera_pitch = task.desiredPitch();
        input = step(1040, true, {}, true, box);
        assert(!input.geometry_valid);
        input.run = input.safety_ok = input.gripper_done = true;
        input.gripper_feedback_open = 0;
        input.heading_valid = true; input.heading_rad = 0;
        previous = task.update(input);
        assert(previous.state == PushState::CARRY || previous.state == PushState::RAISE_RELEASE);
        assert(previous.motion.gripper_offset == 20);
        return previous.motion;
    };
    for(int i=0;i<20;++i) {
        transportStep({540,440,200,200});
        assert(task.desiredPitch()==-3000);
    }
    // A box supplies direction, not a measured recovery turn. Exercise the
    // calibration hold explicitly without inventing IMU translation evidence.
    task.transport_phase_.jaw_sequence_.recovery = TransportPhaseData::JawSequence::Recovery::Calibrate;
    for (int i=0;i<15;++i) {
        auto motion=transportStep({2,20,1277,681});
        // No raw IMU translation samples exist in this builder fixture:
        // recovery calibration cannot authorize travel.
        assert(motion.vx_mps==0 && motion.wz_rps==0);
        assert(previous.state==PushState::CARRY && previous.reason=="carry_pose_recovery_calibrate");
        assert(task.transport_phase_.jaw_sequence_.stage == TransportPhaseData::JawSequence::Stage::Approach);
    }
    task.transport_phase_.jaw_sequence_.recovery = TransportPhaseData::JawSequence::Recovery::Idle;
    task.transport_phase_.jaw_sequence_.stage = TransportPhaseData::JawSequence::Stage::ColourAdvance;
    task.transport_phase_.jaw_sequence_.heading = 0;
    task.transport_phase_.approach_pitch_ = 0;
    frame.setTo(cv::Scalar(160,100,60));
    bool colour_seen=false;
    for (int i=0;i<12;++i) {
        camera_pitch=0;
        input=step(1040,true,{},true);
        if (input.camera_pitch_stable) {
            assert(input.transport_floor.valid && input.transport_floor.region_valid && input.transport_floor.inside);
            assert(input.transport_floor.sample_us < input.now_us);
            assert(input.path_safe && input.turn_safe && input.arc_safe);
            colour_seen=true;
        }
    }
    assert(colour_seen);
    cfg.request.team="red";
    input=step(1040,true,{},true);
    assert(input.transport_floor.valid && !input.transport_floor.region_valid && !input.transport_floor.inside);
    frame.setTo(cv::Scalar(0,0,200));
    input=step(1040,true,{},true);
    assert(input.transport_floor.valid && input.transport_floor.region_valid && input.transport_floor.inside);
    // Real builder: raw per-half landmarks produce centreline evidence even
    // with no camera calibration or geometry pipeline (both are absent here).
    zone.width_m=.6f;zone.confirmed=true;
    vision.pose_ran=true;
    for(int half=0;half<2;++half) {
        auto& h=vision.halves[half];h.valid=true;h.score=.95f;h.confidence.fill(.95f);
        float x=490+300*half;
        h.keypoints={cv::Point2f{x,200},cv::Point2f{x,500},
                     cv::Point2f{x+300,200},cv::Point2f{x+300,500}};
        h.box={x,200,300,300};
    }
    input=step(1040,true,{},true);
    assert(input.supply_view.valid && input.supply_view.inside);
    assert(std::abs(input.supply_view.error_fraction)<1e-5f);
    assert(input.injured_view.valid && input.injured_view.error_fraction>.01f);
    vision.halves[1].valid=false;
    input=step(1040,true,{},true);
    assert(input.supply_view.valid && !input.injured_view.valid);
    std::cout<<"PASS real builder selected-half centreline without calibration"<<std::endl;
    std::cout << "PASS real builder metric-free recovery calibration and UI120 full-image own-colour evidence" << std::endl;
    // UI120 ordinary search must ignore a lower non-ordinary box, while the
    // earlier closed guide approach still selects the lowest box of any class.
    previous.phase = task.phase_ = TaskPhase::Startup;
    previous.state = PushState::START_ADVANCE;
    previous.target_id = -1;
    task.trip_context_.target_id_ = task.trip_context_.capture_target_id_ = -1;
    task.trip_context_.label_.clear();
    camera_pitch = cfg.task.track_pitch_cdeg;
    vision.pose_ran = false;
    for (auto action : {PushTask::StartupAction::SelectOrdinary,
                        PushTask::StartupAction::OpenApproach,
                        PushTask::StartupAction::Approach}) {
        task.startup_action_ = action;
        for (int i = 0; i < 6; ++i)
            input = step(1040, true, {900, 400, 100, 100});
        assert(input.camera_pitch_stable && !input.pixel_grasp_near_view);
        assert(input.target_valid && input.target_center_px_valid);
        assert(input.label == (action == PushTask::StartupAction::Approach
                                   ? "dangerous_object" : "ordinary_supply"));
    }
    std::cout << "PASS real builder UI120 ordinary search and open approach filter candidates\n";
    std::cout << "PASS real ObservationBuilder shares upper-edge review and actual sent feedback\n";
    std::cout << "PASS real ObservationBuilder passes actual sent feedback before capture clearance review\n";
}
