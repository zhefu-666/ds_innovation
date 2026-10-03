#pragma once
#include "rescue/transport_rules.hpp"
#include "rescue/types.hpp"
#include "rescue/zone_estimate.hpp"
#include "rescue/planner.hpp"
#include <map>
#include <string>

namespace rescue {
// Capture-and-carry task (gripper frame):
// SCAN -> APPROACH (closed) -> PREPARE (stop, open, inventory the rush corridor) -> RUSH ->
// CLOSE -> VERIFY_CAPTURE (held set vs. transport rules) -> CARRY -> GATE (correct half) ->
// OPEN_RELEASE -> ENTER (push in) -> BACK_OUT -> VERIFY_DELIVERY -> TURN_SCAN -> SCAN ...
// CAPTURE_FAIL / LOST_HOLD / ABORT_DROP open the gripper and reverse away before rescanning.
// Camera pitch follows the phase: FAR to search and carry, TRACK for a close target and the zone
// entrance, NEAR (whole gripper frame) to enclose and verify the load. Each phase waits until the
// servo reads back the preset and holds still; CARRY stops for a NEAR hold check periodically and
// GATE once more before opening. A holding region out of view is "not seen", never "lost".
enum class PushState {
    WAIT_START, START_ADVANCE, SCAN, APPROACH, PREPARE, RUSH, CLOSE, VERIFY_CAPTURE, CARRY, GATE,
    OPEN_RELEASE, ENTER, BACK_OUT, VERIFY_DELIVERY, TURN_SCAN,
    CAPTURE_FAIL, LOST_HOLD, ABORT_DROP, SAFE_STOP
};
// Observations are robot body frame (x right, y forward, metres). Every field
// defaults to "no evidence"; the task never treats a missing input as success.
struct PushObservation {
    uint64_t now_us = 0;
    bool run = false, reset = false, safety_ok = false;
    // Primary candidate chosen by the perception adapter.
    bool target_valid = false, geometry_valid = false;
    int target_id = -1;
    std::string label;
    float distance_m = 0, heading_error = 0; // heading positive left/CCW
    // Positive evidence that the candidate already lies in our zone (never pull it out).
    bool target_region_valid = false; // unknown is not evidence of being outside
    bool target_in_zone = false;
    // Direct visual evidence that the released target has entered our own zone.
    bool delivery_observed = false;
    // path_safe: forward motion and in-place turns, including the swept gripper and load.
    // retreat_safe: reversing (the rear is blind; only an explicit adapter may set it).
    bool path_safe = false, retreat_safe = false;
    // The swept forward path and any released load stay outside the opponent safe zone
    // (anything moved there scores for them). Reversing adapters must include it in retreat_safe.
    bool opponent_zone_clear = false;
    // Everything the open gripper frame would sweep on the rush to the primary.
    Inventory corridor;
    bool corridor_complete = false, corridor_occlusion_free = false;
    // Holding region (CaptureMonitor): objects enclosed and moving with the robot.
    // hold_observable: the region is in view this frame (calibrated holding pitch, servo still);
    // without it captured/held carry no information about the load.
    bool hold_observable = false, captured = false, held_complete = false;
    Inventory held;
    // Camera servo readback at capture time (A6, 0.01 deg, positive down) and whether it stayed
    // within tolerance for the stability window (PitchHistory).
    int16_t camera_pitch_cdeg = kCameraPitchInvalid;
    bool camera_pitch_stable = false;
    // Lower-controller completion for the latest gripper command
    // (gripper_feedback_open: -1 none, 0 closed, 1 open target of the acknowledged action).
    bool gripper_done = false;
    int gripper_feedback_open = -1;
    // Our own safe zone.
    bool zone_identity_verified = false; // independent identity evidence, never --team alone
    bool zone_valid = false, zone_own = false; // diagnostic; quality is checked independently
    ZoneEstimate zone_estimate;
    std::string zone_class; // primary target's expected half, diagnostic only
    // Objects fully inside each half of our zone, off the fence and at rest.
    bool zone_counts_valid = false;
    int zone_supply_count = 0, zone_injured_count = 0;
    // Relative IMU yaw, only used to measure the turn after a delivery.
    // Navigation adapter output: timestamped, validated waypoint and selected drop centre.
    // Missing route/occupancy never falls back to driving straight to a fixed gate.
    NavigationScene navigation_scene;
    bool zone_inventory_complete = false;
    std::vector<PlannerObstacle> zone_occupied; // fixed zone frame, excluding this carried load
    bool carry_plan_valid = false, drop_plan_valid = false;
    uint64_t navigation_timestamp_us = 0;
    cv::Point2f carry_waypoint_body, drop_centre_zone;
    bool heading_valid = false;
    float heading_rad = 0;
};
struct PushOutput {
    PushState state = PushState::WAIT_START;
    MotionCommand motion; // includes gripper_open and camera_pitch_cdeg
    int target_id = -1, batch_size = 0, delivered_total = 0;
    bool first_ordinary_delivered = false;
    bool cargo_injured = false;
    bool drop_locked = false;
    cv::Point2f drop_centre_zone;
    RuleVerdict verdict = RuleVerdict::OK;
    std::string reason;
    std::string match_state, match_reason;
    uint64_t match_remaining_us = 0;
    bool hardware_output_enabled = false;
};
// Body geometry, speeds and budgets. Lengths marked "measure" are placeholders
// until the gripper is measured on the robot.
struct TaskTuning {
    float approach_speed = .08f, rush_speed = .12f, carry_speed = .08f;
    float enter_speed = .04f, back_speed = .04f, max_speed = .2f;
    float heading_gain = 1.5f, lateral_gain = 1.0f;
    float scan_wz = .25f, turn_wz = .3f, max_wz = .3f, rotate_in_place_rad = .45f;
    float startup_advance_speed = .1f;
    uint64_t startup_advance_us = 10000000; // 0.1 m/s for 10 s; phase bound, not measured distance
    float rush_start_m = .35f, rush_heading_rad = .06f, rush_extra_m = .15f, rush_abort_heading_rad = .35f;
    float grasp_trigger_y_m = .20f, hold_center_y_m = .12f, injured_hold_center_y_m = .14f, mouth_y_m = .21f;
    float supply_half_x_m = -.15f, injured_half_x_m = .15f; // zone frame, supply half on the left
    float gate_clearance_m = .25f, gate_tolerance_m = .04f, gate_yaw_rad = .05f, gate_lateral_m = .06f;
    // Release: push the held centre to deposit_y (half depth); afterwards reverse until the
    // open mouth is mouth_clear_y outside the entrance.
    float deposit_y_m = .15f, enter_lateral_m = .09f, mouth_clear_y_m = -.05f, abort_back_m = .20f;
    // Turn away from the zone before accepting a new target, so nothing is re-picked from it.
    float turn_min_rad = 1.6f;
    int confirm_frames = 3, capture_frames = 4, delivery_frames = 6, hold_grace_frames = 6;
    uint64_t frame_timeout_us = 200000, gripper_timeout_us = 2500000;
    uint64_t approach_budget_us = 15000000, prepare_budget_us = 4000000, rush_budget_us = 6000000;
    uint64_t verify_budget_us = 1500000, carry_budget_us = 25000000, gate_budget_us = 4000000;
    // Enter covers gate_clearance + deposit_y + tolerance (~0.48 m) at enter_speed; retreat ~0.3 m back.
    uint64_t enter_budget_us = 15000000, retreat_budget_us = 10000000, delivery_budget_us = 2500000;
    uint64_t turn_budget_us = 15000000, blacklist_us = 5000000;
    // Camera presets, 0.01 deg positive down; ground mapping still requires pitch-specific validation.
    int16_t far_pitch_cdeg = 500, track_pitch_cdeg = 500, near_pitch_cdeg = 4000;
    int16_t pitch_tolerance_cdeg = 100;
    float track_near_m = .6f; // approach switches FAR -> TRACK once the target is this close
    // Carry looks ahead at FAR; the hold is re-checked at NEAR after this distance or time.
    float hold_check_m = .3f;
    int hold_check_frames = 2;
    uint64_t hold_check_interval_us = 3000000, pitch_timeout_us = 2000000;
};
class PushTask {
public:
    explicit PushTask(TaskTuning tuning = {}) : t_(tuning) {}
    PushOutput update(const PushObservation &in);
    static const char *name(PushState state);
    std::vector<int> rejectedTargets(uint64_t now_us) const;
private:
    TaskTuning t_;
    PushState state_ = PushState::WAIT_START, stopped_from_ = PushState::WAIT_START;
    int target_id_ = -1, confirmations_ = 0, misses_ = 0, step_ = 0;
    int total_ = 0, baseline_ = 0, baseline_other_ = 0, hold_misses_ = 0;
    std::string label_, reason_;
    Inventory trip_, pending_, seen_;
    RuleVerdict verdict_ = RuleVerdict::OK;
    bool first_ = false, fault_ = false, track_close_ = false, delivery_seen_ = false;
    uint8_t gripper_cmd_ = 0;
    int16_t pitch_cmd_ = t_.far_pitch_cdeg;
    uint64_t last_us_ = 0, phase_us_ = 0, gripper_us_ = 0, carry_us_ = 0, pitch_wait_us_ = 0, hold_seen_us_ = 0;
    // Dead-reckoned from the previous command; only bounds a phase, never proves progress.
    float travel_ = 0, travel_limit_ = 0, turned_ = 0, last_heading_ = 0, carried_ = 0;
    float last_vx_ = 0, last_wz_ = 0;
    bool drop_locked_ = false;
    cv::Point2f drop_centre_zone_;
    bool heading_seen_ = false;
    bool retreat_origin_valid_ = false;
    cv::Point2f retreat_origin_zone_, retreat_forward_zone_;
    std::string retreat_geometry_id_;
    std::map<int, uint64_t> rejected_; // primary track id -> blacklist expiry

    void enter(PushState s, uint64_t now, const char *why);
    void clearTrip();
    void blacklist(uint64_t now);
    void fail(const char *why);
    void commandGripper(uint8_t open, uint64_t now);
    int gripperWait(const PushObservation &in, uint8_t open, uint64_t now) const; // 1 done, 0 pending, -1 timeout
    bool candidate(const PushObservation &in, uint64_t now) const;
    bool tracking(const PushObservation &in) const;
    bool zoneOk(const PushObservation &in) const;
    bool holding(const PushObservation &in) const;
    int holdSeen(const PushObservation &in) const; // 1 seen, 0 in view but not held, -1 out of view
    int16_t desiredPitch() const;
    int pitchWait(const PushObservation &in, uint64_t now); // 1 ready, 0 moving, -1 timeout
    float halfX() const;
    float holdCenter() const;
    PushState resumeState() const;
};
} // namespace rescue
