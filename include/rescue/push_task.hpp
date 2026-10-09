#pragma once
#include "rescue/transport_rules.hpp"
#include "rescue/types.hpp"
#include "rescue/zone_estimate.hpp"
#include "rescue/zone_dead_reckoning.hpp"
#include "rescue/planner.hpp"
#include <map>
#include <string>

namespace rescue {
// Frame enclosure and transport (no gripping):
// PREPARE opens frame (0 deg) -> RUSH positions the object inside -> LOWER_FRAME / CLOSE (20 deg)
// -> VERIFY_CAPTURE checks the enclosed inventory -> CARRY -> GATE -> ENTER with frame DOWN
// -> RAISE_RELEASE at the drop point -> BACK_OUT -> VERIFY_DELIVERY.
// Failure recovery raises the frame before retreat. Stop commands preserve the last position.
// Legacy gripper_done / gripper_feedback_open observations retain the unchanged A6 binary
// interface: assumed raw bits configured after acceptance; fresh action-ID matching is required.
// Camera phases and visual enclosure checks remain mandatory; A6 is not proof of capture.
enum class PushState {
    WAIT_START, START_ADVANCE, SCAN, LOST_SEARCH, MID_REACQUIRE, MID_APPROACH, NEAR_REACQUIRE, SELECT_CARGO, CUE_APPROACH, CUE_PREPARE, CLEAR_PILE, CLEAR_PAUSE, APPROACH, PREPARE, RUSH, LOWER_FRAME, VERIFY_CAPTURE, CARRY, GATE,
    RAISE_RELEASE, ENTER, BACK_OUT, VERIFY_DELIVERY, TURN_SCAN,
    CAPTURE_FAIL, LOST_HOLD, ABORT_DROP, SAFE_STOP
};
// Observations are robot body frame (x right, y forward, metres). Every field
// defaults to "no evidence"; the task never treats a missing input as success.
struct PushObservation {
    uint64_t now_us = 0;
    bool run = false, reset = false, safety_ok = false;
    // Primary candidate chosen by the perception adapter.
    bool target_valid = false, geometry_valid = false;
    bool multi_view_finished=false;
    int multi_view_verdict=0; // 0 uncertain, 1 enclosed, 2 empty
    int16_t multi_view_pitch=kCameraPitchInvalid;
    Inventory multi_view_inventory;
    double pixel_ratio_raw=0,pixel_ratio_smoothed=0;
    std::string pixel_mode="none",pixel_reason,multi_view_reason;
    bool pixel_locked=false;int observation_view=0,observation_frames=0;
    uint64_t observation_round=0;
    bool target_is_search_cue = false; // search cue only, never a transport target
    bool gripper_closed_observed = false; // fresh causal A6 lowered state, not a new-action acknowledgement
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
    bool clear_push_safe=false; // fresh geometry, known zone boundary and closed-front contact corridor
    bool path_safe = false, retreat_safe = false;
    bool clearance_checks_disabled = false; // permissions overridden, not observed safe space
    bool directional_clearance_valid=false, turn_safe=false, arc_safe=false, jaw_open_safe=false;
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
    // TEMP_ASSUMPTION visual zone reference (coarse, >=2 pose points); never a safety gate.
    bool vref_valid = false;
    int vref_points = 0;
    ZoneEstimate vref_zone;
    // TEMP_ASSUMPTION：本帧可见货物近端接地点（机体系）；用于判断货物是否进区/卡住，不作安全判据。
    std::vector<cv::Point2f> cargo_body; uint64_t cargo_ts_us = 0;
    // Pixel-space view of each cargo_body entry (same order): the box bottom against the zone's front landmark line.
    // clipped = the box touches the image bottom, so the ground contact (and any depth derived from it) is biased deep.
    struct CargoImg { float bottom_px = 0, height_px = 0, front_px = 0; bool clipped = false, front_valid = false; };
    std::vector<CargoImg> cargo_img;
    // On-end injured blocks (never transport targets), body-frame ground points of the current frame.
    std::vector<cv::Point2f> upright_body; uint64_t upright_ts_us = 0;
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
    uint64_t imu_blocked_us = 0; // IMU: continuous time the chassis ignored a commanded turn while driving (0 = moving or unknown)
};
struct PushOutput {
    PushState state = PushState::WAIT_START;
    MotionCommand motion; // includes gripper_offset and camera_pitch_cdeg
    int target_id = -1, batch_size = 0, delivered_total = 0;
    bool first_ordinary_delivered = false;
    int search_cue_id = -1, capture_target_id = -1;
    bool target_is_search_cue = false;
    bool cue_contact_allowed = false;
    int clearing_attempts = 0;
    uint64_t startup_commanded_us = 0, clear_commanded_us = 0;
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
    bool require_multi_view = true;
    bool demo_search_only=false, demo_carry_once=false;
    // TEMP_ASSUMPTION（2026-10-08用户要求）：缺失的抓取/区域/投放证据视为成立，用有界盲动作跑通决策链。
    bool assume_all_safe=false;
    bool field_carry=false; // main-path: use the demo transport/release/push/verify logic without the assume-only capture/rush assumptions
    bool field_rush=false; // main-path: scaled rush limit, blind finish after the target drops below view, ID rebound after the frame opens
    float rebind_max_shift_m=.15f; // field_rush only: a re-ID'd target must sit this close to the last tracked one
    bool assume_injured_trip=false; // 演示伤员趟：first_ 初始为真，不改变比赛规则默认值
    float assume_push_m=.55f; // 假设模式：松开后下压框向区内再推的指令行程
    float assume_motion_ratio=4.f, assume_rush_trigger_m=.15f; // 假设模式：指令/实际位移比；放框时块前向距离
    float assume_carry_m=.10f, assume_back_m=.10f; // bounded blind legs; still inside approach/retreat limits
    uint64_t assume_zone_search_us=4000000;
    float assume_prepush_back_m=.20f,assume_align_zone_m=.18f,assume_dr_pessimism=1.7f; // 假设模式：松开后先后退一小段（实际米），再下压框推入
    int16_t assume_view_pitch_cdeg=2000; // 假设模式：松开/后退/推入/核验阶段保持能同时看到区域角点与货物的俯仰角
    float assume_cargo_in_y_m=.05f, assume_cargo_done_y_m=.09f, assume_cargo_x_tol_m=.10f, assume_overshoot_m=.12f, assume_stuck_move_m=.015f; // 货物近端距区前沿：判进区/推到位
    uint64_t assume_stuck_us=2000000; int assume_repush_max=2; uint64_t approach_rebind_window_us=4000000;
    float assume_repush_back_m=.10f, assume_repush_speed_gain=.25f, assume_lateral_back_gain=3.f, assume_lateral_back_max_m=.25f, assume_goal_depth_m=.12f, assume_push_angle_max_rad=.45f, assume_push_dx_deadband_m=.03f; // 卡住重推：加长助跑/提速/按块位置斜推
    float assume_release_gap_m=-.04f, assume_push_depth_m=.16f, assume_ratio_max=14.f, assume_lookahead_m=.35f, assume_stall_near_m=.06f, assume_stall_progress_m=.01f; uint64_t assume_stall_us=1500000; // 视觉闭环：松开时块距区前沿；推入目标深度
    // 假设模式：翻框(松开)前须确认货物真进了安全区；未确认则继续下压，直到硬上限。默认关闭。
    bool assume_enter_confirm=false; float assume_enter_true_y_m=.07f, assume_enter_hold_y_m=.03f, assume_enter_max_hold_y_m=.30f; uint64_t assume_enter_extra_us=8000000;
    // 核验只认视觉定位(>=3点)：无定位时先等；等不到先切到远俯仰找角点，仍无则不记视觉通过。推入阶段块要“进区”须框自身也已越过前沿一定深度。
    uint64_t assume_imu_block_us=600000; float assume_imu_block_near_m=.25f; int assume_inside_after_release_obs=3;
    float assume_img_inside_frac=.25f, assume_img_outside_frac=.10f; int assume_img_verify_obs=3; // 无定位核验：块下沿高于前沿landmark线的比例（占框高）
    // 目标前方扫掠通道里有竖放伤员时，先低框原地小角度转向把它扫开（IMU量角，转出后转回）。
    bool sweep_upright=true; float sweep_corridor_half_m=.25f, sweep_extra_m=.20f, sweep_min_sep_m=.05f;
    float sweep_turn_rad=.60f, sweep_return_tol_rad=.06f; int sweep_confirm=2, sweep_max_attempts=2; float sweep_wz=.55f;
    uint64_t sweep_budget_us=8000000, sweep_fresh_us=500000;
    // 张框距离内航向略超 rush_heading_rad 时，小 wz 底盘不转(实测 0.25rad/s 陀螺≈0)：改用 IMU 量角的 rush_pulse_wz 短脉冲转到位，停稳后再由视觉复核。
    bool rush_pulse_turn=false; float rush_pulse_wz=.55f, rush_pulse_max_rad=.20f, rush_pulse_done_frac=.85f;
    uint64_t rush_pulse_budget_us=2500000, rush_pulse_settle_us=500000; int rush_pulse_max=4;
    bool assume_probe=true; uint64_t assume_probe_first_us=1200000, assume_probe_len_us=900000, assume_probe_gap_us=1500000; // 直推时周期性小偏航探测，让IMU能看出堵转
    uint64_t assume_verify_fix_wait_us=7000000, assume_verify_far_after_us=1500000; float assume_inside_min_hold_y_m=.06f;
    // 假设模式：后退后、前推前先用视觉把方框对准货物（原地转向），推进时按货物方位纯追踪，斜推角收小；货物丢失后保持丢失前航向。默认关闭。
    bool assume_align_push=false; float assume_align_tol_rad=.06f, assume_align_wz=.30f, assume_slant_max_rad=.45f; uint64_t assume_align_budget_us=3000000;
    uint64_t attempt_budget_us=0; // enabled by schema 2, never reset by target reselection
    float approach_limit_m=0, retreat_limit_m=0;
    bool enable_search_cues = false;
    bool enable_short_push = false;
    bool enable_lost_search = false;
    float cue_approach_speed = .4f, cue_prepare_distance_m = .36f;
    float clear_speed = .4f, clear_extra_m = .08f, clear_max_commanded_m = .22f;
    float closed_front_y_m = .225f, clear_heading_limit_rad = .15f;
    int max_clear_attempts = 2;
    uint64_t cue_approach_budget_us = 15000000, clear_wall_budget_us = 7000000, clear_pause_us = 500000;

    float approach_speed = .4f, rush_speed = .4f, carry_speed = .08f;
    float enter_speed = .04f, back_speed = .04f, max_speed = .4f;
    float min_turn_wz = .4f; // nonzero commanded yaw rate, rad/s
    float heading_gain = 1.5f, lateral_gain = 1.0f;
    // Approach steering: below this bearing drive straight, so min_turn_wz never turns an aligned approach.
    float heading_deadband_rad = .05f;
    float scan_wz = .25f, turn_wz = .3f, max_wz = .4f, rotate_in_place_rad = .45f;
    float startup_advance_speed = .1f;
    uint64_t startup_advance_us = 10000000; // 0.1 m/s for 10 s; phase bound, not measured distance
    float rush_start_m = .35f, rush_heading_rad = .06f, rush_extra_m = .15f, rush_abort_heading_rad = .35f;
    float grasp_trigger_y_m = .20f, hold_center_y_m = .12f, injured_hold_center_y_m = .14f, mouth_y_m = .21f;
    float supply_half_x_m = -.15f, injured_half_x_m = .15f; // zone frame, supply half on the left
    float gate_clearance_m = .25f, gate_tolerance_m = .04f, gate_yaw_rad = .05f, gate_lateral_m = .06f;
    // Release: push the held centre to deposit_y (half depth); afterwards reverse until the
    // open mouth is mouth_clear_y outside the entrance.
    float deposit_y_m = .15f, enter_lateral_m = .09f, mouth_clear_y_m = -.05f, abort_back_m = .20f;
    // Release retreat without zone geometry: after waiting this long for the zone, reverse straight
    // by dead reckoning, at most abort_back_m and never more than was driven forward since the
    // lock; stop if the IMU heading drifts beyond blind_retreat_yaw_rad or goes missing.
    uint64_t blind_retreat_wait_us = 500000;
    float blind_retreat_yaw_rad = .15f;
    // Strict PREPARE corridor check: identical inventory on this many consecutive frames. A corridor holding
    // other objects keeps the frame up, reverses corridor_back_m (real distance; commanded = x motion ratio,
    // skipped when retreat_safe is false) and searches again; after corridor_retry_max such back-offs in a
    // row the target is blacklisted.
    int corridor_confirm_frames = 3, corridor_retry_max = 2; // live main raises the frame count (main.cpp)
    float corridor_back_m = .20f;
    // Turn away from the zone before accepting a new target, so nothing is re-picked from it.
    float turn_min_rad = 1.6f;
    int confirm_frames = 3, capture_frames = 4, delivery_frames = 6, hold_grace_frames = 6;
    uint64_t frame_timeout_us = 200000, gripper_timeout_us = 2500000;
    uint64_t approach_budget_us = 15000000, prepare_budget_us = 4000000, rush_budget_us = 6000000;
    uint64_t verify_budget_us = 1500000, carry_budget_us = 25000000, gate_budget_us = 4000000;
    uint64_t zone_search_budget_us = 45000000; // includes stopped NEAR checks while searching
    // Enter covers gate_clearance + deposit_y + tolerance (~0.48 m) at enter_speed; retreat ~0.3 m back.
    uint64_t enter_budget_us = 15000000, retreat_budget_us = 10000000, delivery_budget_us = 2500000;
    uint64_t turn_budget_us = 15000000, blacklist_us = 5000000;
    // Camera presets, 0.01 deg positive down; ground mapping still requires pitch-specific validation.
    int16_t far_pitch_cdeg = 500, track_pitch_cdeg = 500, near_pitch_cdeg = 4000;
    int16_t pitch_tolerance_cdeg = 100;
    int16_t intermediate_pitch_cdeg = kCameraPitchInvalid; // optional 20deg observation stage
    float intermediate_to_near_m = .4f;
    // Open jaw at 20deg: a target lost within this range ran off the image bottom at the
    // mouth; stop and hand off to the 40deg view instead of a lost-target SAFE_STOP.
    float open_jaw_handoff_m = .32f;
    float track_near_m = .6f; // approach switches FAR -> TRACK once the target is this close
    // Carry looks ahead at FAR; the hold is re-checked at NEAR after this distance or time.
    float hold_check_m = .3f;
    int hold_check_frames = 2;
    uint64_t hold_check_interval_us = 3000000, pitch_timeout_us = 2000000;
};
class PushTask {
public:
    explicit PushTask(TaskTuning tuning = {}) : t_(tuning) { first_ = t_.assume_injured_trip; }
    PushOutput update(const PushObservation &in);
    const std::string &anchorDebug() const { return anchor_dbg_; }
    std::string takeDropZoneEvent() { std::string s; s.swap(dropzone_evt_); return s; }
    static const char *name(PushState state);
    std::vector<int> rejectedTargets(uint64_t now_us) const;
private:
    TaskTuning t_;
    PushState state_ = PushState::WAIT_START, stopped_from_ = PushState::WAIT_START;
    bool cue_mode_ = false;
    int clear_attempts_ = 0;
    int near_failures_ = 0;
    uint64_t clear_commanded_us_ = 0, clear_planned_us_ = 0;
    float clear_heading_rad_ = 0;
    int search_cue_id_ = -1, capture_target_id_ = -1;
    int target_id_ = -1, confirmations_ = 0, misses_ = 0, step_ = 0;
    float last_range_ = 0; // last tracked range in MID_APPROACH, inf when none
    int total_ = 0, baseline_ = 0, baseline_other_ = 0, hold_misses_ = 0;
    int delivery_delta_ = 0;
    uint64_t delivery_frame_ = 0, delivery_stable_since_ = 0;
    std::string delivery_geometry_id_;
    std::string label_, reason_;
    Inventory trip_, pending_, seen_;
    RuleVerdict verdict_ = RuleVerdict::OK, seen_verdict_ = RuleVerdict::OK;
    bool first_ = false, fault_ = false, track_close_ = false, delivery_seen_ = false;
    uint8_t frame_raised_ = 0; // logical open, independent of wire angle
    uint64_t frame_transaction_=0,attempt_started_=0;
    float bounded_approach_=0,bounded_retreat_=0;
    int16_t multi_view_pitch_=kCameraPitchInvalid;
    int16_t pitch_cmd_ = t_.far_pitch_cdeg;
    uint64_t rebind_since_us_ = 0;
    bool approach_rebind_ = false; // 分级下压后允许改绑跟踪器新ID一次
    int16_t approach_pitch_ = kCameraPitchInvalid; // APPROACH分级下压：远处保持当前角，贴近时 5°→20°→40°
    uint64_t startup_commanded_us_ = 0;
    uint64_t zone_search_started_us_ = 0;
    float carry_best_y_ = 0; uint64_t carry_progress_us_ = 0;
    uint64_t last_us_ = 0, phase_us_ = 0, gripper_us_ = 0, carry_us_ = 0, pitch_wait_us_ = 0, hold_seen_us_ = 0;
    // Dead-reckoned from the previous command; only bounds a phase, never proves progress.
    float travel_ = 0, travel_limit_ = 0, turned_ = 0, last_heading_ = 0, carried_ = 0;
    float last_vx_ = 0, last_wz_ = 0;
    bool drop_locked_ = false;
    bool prepush_back_ = false; float prepush_hold_y_ = 0; // 假设模式：松开后的“先退再推”阶段
    std::string anchor_dbg_;
    bool half_locked_ = false, half_injured_ = false; float half_x_ = 0; // 本趟目标半区（进入搬运时锁定，整趟不变）
    std::string dropzone_evt_;
    void lockDropHalf(const char *source);
    void noteDropZone(const char *event, const cv::Point2f &hold);
    bool cargoZone(const PushObservation &in, uint64_t now, cv::Point2f &body, cv::Point2f &rear, bool &fix, int *idx = nullptr) const;
    static PushObservation::CargoImg imgAt(const PushObservation &in, int idx) { return idx >= 0 && size_t(idx) < in.cargo_img.size() ? in.cargo_img[idx] : PushObservation::CargoImg{}; }
    void noteCargo(const char *event, const cv::Point2f &rear, bool fix);
    void trackCargo(const PushObservation &in, uint64_t now);
    int sweep_phase_ = 0, sweep_attempts_ = 0, sweep_seen_ = 0; float sweep_yaw0_ = 0, sweep_sign_ = 1; uint64_t sweep_start_us_ = 0, sweep_t0_us_ = 0;
    int rush_pulse_phase_ = 0, rush_pulse_count_ = 0; float rush_pulse_yaw0_ = 0, rush_pulse_delta_ = 0; uint64_t rush_pulse_start_us_ = 0, rush_pulse_t0_us_ = 0;
    bool uprightInCorridor(const PushObservation &in, uint64_t now, float &lateral) const;
    bool cargoVisible(uint64_t now) const { return cargo_seen_us_ && now - cargo_seen_us_ <= 1500000; }
    float backTarget(uint64_t now) const;
    bool rushLogic() const { return t_.assume_all_safe || t_.field_rush; }
    bool carryLogic() const { return t_.assume_all_safe || t_.field_carry; }
    bool assumeView() const { return carryLogic() && t_.assume_view_pitch_cdeg != kCameraPitchInvalid; }
    int repush_n_ = 0, verify_in_ = 0, verify_out_ = 0; bool verify_far_view_ = false; uint64_t verify_fix_us_ = 0; int inside_obs_ = 0; uint64_t inside_obs_ts_ = 0;
    uint64_t cargo_last_ts_ = 0, cargo_ref_us_ = 0, cargo_seen_us_ = 0, verify_ts_ = 0;
    cv::Point2f cargo_ref_body_, cargo_ref_zone_; bool cargo_fix_=false, cargo_clipped_=false; float cargo_rear_x_ = 0, cargo_rear_y_ = 0;
    cv::Point2f cargo_body_; bool align_pending_ = false, push_hold_valid_ = false; uint64_t align_start_us_ = 0; float push_hold_phi_ = 0;
    uint64_t probe_until_us_ = 0, probe_next_us_ = 0; float probe_sign_ = 1.f; bool delivery_visual_ = false;
    bool enter_unconfirmed_ = false; // 本趟推入已因“未确认进区”延长过
    bool assume_pushed_ = false; // 假设模式：已完成“松开后再用下压姿态向区内推”
    bool assume_blind_ = false; // TEMP_ASSUMPTION blind leg active in this phase
    float assume_rush_end_ = 0;
    cv::Point2f last_target_xy_; bool last_target_valid_ = false; // body-frame position of the locked target on its last tracked frame
    // 假设模式：YOLO-pose粗略区域位姿锚定在里程系，视野丢失后用IMU航向+缩放后的指令位移继续推算。
    AnchoredZone zone_;
    float odo_ = 0, ratio_est_ = 0, ratio_odo_ = 0, last_imu_yaw_ = 0;
    cv::Point2f ratio_hv_;
    uint64_t vref_fix_us_ = 0, vref_seen_us_ = 0;
    bool ratio_snap_ = false, imu_seen_ = false, vref_used_ = false;
    float drRatio(uint64_t now) const { const bool stale = now - vref_seen_us_ > 400000; return motionRatio() * ((stale && (state_ == PushState::CARRY || state_ == PushState::BACK_OUT)) ? t_.assume_dr_pessimism : 1.f); }
    float motionRatio() const { return ratio_est_ > 0 ? ratio_est_ : t_.assume_motion_ratio; }
    cv::Point2f drop_centre_zone_;
    bool heading_seen_ = false;
    int lost_round_ = 0; // LOST_SEARCH: 0 FAR sweep, 1 intermediate (20deg) sweep for close cargo
    bool retreat_origin_valid_ = false;
    bool corridor_retreat_ = false; int corridor_retries_ = 0; // LOST_HOLD reused as a no-blacklist back-off from a blocked corridor
    int retreat_mode_ = 0; // 0 undecided, 1 zone-measured, 2 blind (dead reckoning + IMU heading)
    uint64_t retreat_zone_us_ = 0;
    float retreat_heading_ = 0, approach_forward_ = 0; // forward commanded since the trip started
    cv::Point2f retreat_origin_zone_, retreat_forward_zone_;
    std::string retreat_geometry_id_;
    std::map<int, uint64_t> rejected_; // primary track id -> blacklist expiry

    void enter(PushState s, uint64_t now, const char *why);
    void clearTrip();
    void blacklist(uint64_t now);
    void fail(const char *why);
    void commandFrame(uint8_t open, uint64_t now);
    int frameWait(const PushObservation &in, uint8_t open, uint64_t now) const; // 1 done, 0 pending, -1 timeout
    bool candidate(const PushObservation &in, uint64_t now) const;
    bool tracking(const PushObservation &in) const;
    bool midPitch() const { return t_.intermediate_pitch_cdeg>t_.far_pitch_cdeg && t_.intermediate_pitch_cdeg<t_.track_pitch_cdeg; }
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
