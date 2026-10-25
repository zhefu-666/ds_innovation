#pragma once
#include "rescue/transport_rules.hpp"
#include "rescue/types.hpp"
#include "rescue/zone_estimate.hpp"
#include "rescue/zone_dead_reckoning.hpp"
#include "rescue/planner.hpp"
#include "rescue/task/capture_verification_stage.hpp"
#include "rescue/task/delivery_verification_stage.hpp"
#include "rescue/task/release_retreat_stage.hpp"
#include "rescue/task/carry_approach_stage.hpp"
#include "rescue/task/carry_search_stage.hpp"
#include "rescue/task/sweep_trigger_stage.hpp"
#include "rescue/task/enter_stage.hpp"
#include "rescue/task/push_in_stage.hpp"
#include "rescue/task/sweep_ledger.hpp"
#include "rescue/task/disperse_turn_stage.hpp"
#include "rescue/task/disperse_swing_stage.hpp"
#include "rescue/task/disperse_rush_stage.hpp"
#include "rescue/task/recovery_stage.hpp"
#include "rescue/task/search_stage.hpp"
#include "rescue/task/sweep_stage.hpp"
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
    WAIT_START,
    START_ADVANCE,
    SCAN,
    LOST_SEARCH,
    MID_REACQUIRE,
    MID_APPROACH,
    NEAR_REACQUIRE,
    SELECT_CARGO,
    CUE_APPROACH,
    CUE_PREPARE,
    CLEAR_PILE,
    CLEAR_PAUSE,
    APPROACH,
    PREPARE,
    RUSH,
    LOWER_FRAME,
    VERIFY_CAPTURE,
    CARRY,
    GATE,
    RAISE_RELEASE,
    ENTER,
    BACK_OUT,
    VERIFY_DELIVERY,
    TURN_SCAN,
    CAPTURE_FAIL,
    LOST_HOLD,
    ABORT_DROP,
    SAFE_STOP,
    SCATTER,
    PILE
};
// Observations are robot body frame (x right, y forward, metres). Every field
// defaults to "no evidence"; the task never treats a missing input as success.
struct PushObservation {
    uint64_t now_us = 0;
    bool run = false, reset = false, safety_ok = false;
    // Primary candidate chosen by the perception adapter.
    bool target_valid = false, geometry_valid = false;
    bool multi_view_finished = false;
    int multi_view_verdict = 0; // 0 uncertain, 1 enclosed, 2 empty
    int16_t multi_view_pitch = kCameraPitchInvalid;
    Inventory multi_view_inventory;
    double pixel_ratio_raw = 0, pixel_ratio_smoothed = 0;
    std::string pixel_mode = "none", pixel_reason, multi_view_reason;
    bool pixel_locked = false;
    int observation_view = 0, observation_frames = 0;
    float pixel_confidence = 0; // detector confidence of the locked box
    bool pixel_bearing_valid = false;
    float pixel_bearing = 0; // bearing of the locked box centre from its pixels alone (positive
                             // left); needs no ground projection
    uint64_t observation_round = 0;
    bool target_is_search_cue = false; // search cue only, never a transport target
    bool gripper_closed_observed =
        false; // fresh causal A6 lowered state, not a new-action acknowledgement
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
    bool clear_push_safe =
        false; // fresh geometry, known zone boundary and closed-front contact corridor
    bool path_safe = false, retreat_safe = false;
    bool clearance_checks_disabled = false; // permissions overridden, not observed safe space
    bool directional_clearance_valid = false, turn_safe = false, arc_safe = false,
         jaw_open_safe = false;
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
    struct HeldObject {
        int track_id = -1;
        std::string label;
        float confidence = 0;
    };
    std::vector<HeldObject> held_objects; // detections behind `held` (identity/confidence for the
                                          // VERIFY_CAPTURE re-check)
    // Camera servo readback at capture time (A6, 0.01 deg, positive down) and whether it stayed
    // within tolerance for the stability window (PitchHistory).
    int16_t camera_pitch_cdeg = kCameraPitchInvalid;
    bool camera_pitch_stable = false;
    // Lower-controller completion for the latest gripper command
    // (gripper_feedback_open: -1 none, 0 closed, 1 open target of the acknowledged action).
    bool gripper_done = false;
    int gripper_feedback_open = -1;
    // Our own safe zone.
    bool zone_identity_verified = false;       // independent identity evidence, never --team alone
    bool zone_valid = false, zone_own = false; // diagnostic; quality is checked independently
    // This frame measured the zone colour as the enemy's (never --team or an assumption).
    bool enemy_zone_seen = false;
    ZoneEstimate zone_estimate;
    // Coarse sighting of a safe zone at any range (no geometry fix needed): body bearing, left
    // positive. enemy: measured enemy colour.
    bool zone_sight_valid = false, zone_sight_enemy = false;
    float zone_sight_bearing = 0;
    // TEMP_ASSUMPTION visual zone reference (coarse, >=2 pose points); never a safety gate.
    bool vref_valid = false;
    int vref_points = 0;
    ZoneEstimate vref_zone;
    // TEMP_ASSUMPTION：本帧可见货物近端接地点（机体系）；用于判断货物是否进区/卡住，不作安全判据。
    std::vector<cv::Point2f> cargo_body;
    uint64_t cargo_ts_us = 0;
    // Pixel-space view of each cargo_body entry (same order): the box bottom against the zone's
    // front landmark line. clipped = the box touches the image bottom, so the ground contact (and
    // any depth derived from it) is biased deep.
    struct CargoImg {
        float bottom_px = 0, height_px = 0, front_px = 0;
        bool clipped = false, front_valid = false;
    };
    std::vector<CargoImg> cargo_img;
    // On-end injured blocks (never transport targets), body-frame ground points of the current
    // frame.
    std::vector<cv::Point2f> upright_body;
    uint64_t upright_ts_us = 0;
    cv::Point2f target_center_body;
    bool target_center_valid = false; // filled by geometry_pipeline.cpp for the locked target
    // Every other confident, ground-located detection (any class) except the locked target; only
    // read when clear_neighbours is on.
    std::vector<cv::Point2f> clutter_body;
    uint64_t clutter_ts_us = 0;
    std::vector<cv::Point2f>
        clutter_ordinary;   // subset of clutter_body that are ordinary supplies (exact same points)
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
    uint64_t imu_spin_stall_us =
        0; // IMU: continuous time an in-place spin was commanded but the gyro stayed ~0
    uint64_t imu_blocked_us = 0; // IMU: continuous time the chassis ignored a commanded turn while
                                 // driving (0 = moving or unknown)
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
    bool demo_search_only = false, demo_carry_once = false;
    // TEMP_ASSUMPTION（2026-10-08用户要求）：缺失的抓取/区域/投放证据视为成立，用有界盲动作跑通决策链。
    bool assume_all_safe = false;
    bool field_carry = false; // main-path: use the demo transport/release/push/verify logic without
                              // the assume-only capture/rush assumptions
    bool field_rush = false;  // main-path: scaled rush limit, blind finish after the target drops
                              // below view, ID rebound after the frame opens
    bool verify_held_contents =
        false; // main-path: judge the frame contents seen in VERIFY_CAPTURE (dangerous never;
               // injured only after a delivery and alone; target must be inside)
    float rebind_max_shift_m =
        .15f; // field_rush only: a re-ID'd target must sit this close to the last tracked one
    bool assume_injured_trip = false; // 演示伤员趟：first_ 初始为真，不改变比赛规则默认值
    float assume_push_m = .55f; // 假设模式：松开后下压框向区内再推的指令行程
    float assume_motion_ratio = 4.f,
          assume_rush_trigger_m = .15f; // 假设模式：指令/实际位移比；放框时块前向距离
    float assume_carry_m = .10f,
          assume_back_m = .10f; // bounded blind legs; still inside approach/retreat limits
    uint64_t assume_zone_search_us = 4000000;
    float assume_prepush_back_m = .20f, assume_align_zone_m = .18f,
          assume_dr_pessimism = 1.7f; // 假设模式：松开后先后退一小段（实际米），再下压框推入
    int16_t assume_view_pitch_cdeg =
        2000; // 假设模式：松开/后退/推入/核验阶段保持能同时看到区域角点与货物的俯仰角
    float assume_cargo_in_y_m = .05f, assume_cargo_done_y_m = .09f, assume_cargo_x_tol_m = .10f,
          assume_overshoot_m = .12f, assume_stuck_move_m = .015f; // 货物近端距区前沿：判进区/推到位
    uint64_t assume_stuck_us = 2000000;
    int assume_repush_max = 2;
    uint64_t approach_rebind_window_us = 4000000;
    float assume_repush_back_m = .10f, assume_repush_speed_gain = .25f,
          assume_lateral_back_gain = 3.f, assume_lateral_back_max_m = .08f,
          assume_goal_depth_m = .12f, assume_push_angle_max_rad = .45f,
          assume_push_dx_deadband_m = .03f; // 卡住重推：加长助跑/提速/按块位置斜推
    float assume_release_gap_m = -.04f, assume_push_depth_m = .16f, assume_ratio_max = 14.f,
          assume_lookahead_m = .35f, assume_stall_near_m = .06f, assume_stall_progress_m = .01f;
    uint64_t assume_stall_us = 1500000; // 视觉闭环：松开时块距区前沿；推入目标深度
    // 假设模式：翻框(松开)前须确认货物真进了安全区；未确认则继续下压，直到硬上限。默认关闭。
    bool assume_enter_confirm = false;
    float assume_enter_true_y_m = .07f, assume_enter_hold_y_m = .03f,
          assume_enter_max_hold_y_m = .30f;
    uint64_t assume_enter_extra_us = 8000000;
    // 核验只认视觉定位(>=3点)：无定位时先等；等不到先切到远俯仰找角点，仍无则不记视觉通过。推入阶段块要“进区”须框自身也已越过前沿一定深度。
    uint64_t assume_imu_block_us = 600000;
    float assume_imu_block_near_m = .25f;
    int assume_inside_after_release_obs = 3;
    // 核验(有定位)：块近端越过前沿 verify_in_y 即算进区；低于 verify_out_y
    // 才算在区外，两者之间不计数；连续 verify_out_obs 帧在区外且区内看不到任何块才重推。
    float assume_cargo_verify_in_y_m = .01f, assume_cargo_verify_out_y_m = -.01f;
    int assume_verify_out_obs = 8;
    float assume_img_inside_frac = .25f, assume_img_outside_frac = .10f;
    int assume_img_verify_obs = 3; // 无定位核验：块下沿高于前沿landmark线的比例（占框高）
    // 目标前方扫掠通道里有竖放伤员时，先低框原地小角度转向把它扫开（IMU量角，转出后转回）。
    bool sweep_upright = true;
    float sweep_corridor_half_m = .25f, sweep_extra_m = .20f, sweep_min_sep_m = .05f;
    float sweep_turn_rad = .90f, sweep_return_tol_rad = .06f;
    int sweep_confirm = 2, sweep_max_attempts = 2;
    float sweep_wz = 1.2f;
    uint64_t sweep_budget_us = 8000000, sweep_fresh_us = 500000;
    // The frame front only reaches y=.171 m: creep up to the obstacle (leaving sweep_creep_gap_m)
    // before turning so the sweep actually moves it.
    bool sweep_creep = true;
    float sweep_creep_speed = .2f, sweep_creep_max_m = .12f, sweep_frame_front_m = .171f,
          sweep_creep_gap_m = .03f;
    uint64_t sweep_creep_budget_us = 2500000;
    // Dense pile: rush in and swing the frame fast to scatter it (pushing injured/dangerous aside
    // is not penalised), then carry on. Deeper rush, wider swings and a longer forward creep while
    // swinging: the frame front is only ~0.17 m from the body centre, so a short in-place swing
    // barely moves anything.
    bool disperse = true;
    int disperse_dense_min = 3, disperse_swings = 6, disperse_passes = 1;
    float disperse_radius_m = .30f, disperse_speed = .4f, disperse_depth_m = .15f,
          disperse_max_m = .55f, disperse_wz = 3.2f,
          disperse_turn_rad = 1.0f; // the base turns at ~0.3x the command: 3.2 is ~1 rad/s
    float disperse_swing_vx = .14f, disperse_swing_creep_m = .18f, disperse_swing_tol_rad = .08f,
          disperse_pass_extra_m = .15f;
    uint64_t disperse_rush_budget_us = 5000000, disperse_swing_budget_us = 5000000,
             disperse_cooldown_us = 25000000;
    // Sweep ledger shared by every group-sweep trigger (SCAN / far drive / lost-target / APPROACH
    // dense): a gap after each sweep, a per-trip count and time budget, and a growing density
    // requirement per finished sweep (a pile that a sweep did not open up needs more evidence
    // before the next one). Non-supply sweeps (injured / dangerous in the way) bypass all of it.
    // While the ledger blocks, a pickable lock is grabbed instead.
    uint64_t sweep_gap_us = 20000000, sweep_time_budget_us = 45000000;
    int sweep_total_max = 3, sweep_escalate_max = 2;
    // Sweep chain: a far-group drive plus the near sweep that follows it (re-entered within
    // sweep_chain_link_us) is ONE sweep. sweep_op_budget_us is the wall-clock deadline of the whole
    // chain (a far->near hand-over does not re-arm it); sweep_time_budget_us (per trip) and
    // sweep_match_budget_us (per match) are checked WHILE a sweep runs, not only before it starts.
    // sweep_confirm_wait_us: a locked ordinary supply that is still being confirmed
    // (confirm_frames) holds back the cluster decision (sweep_confirm frames) for at most this
    // long.
    uint64_t sweep_op_budget_us = 28000000, sweep_chain_link_us = 3000000,
             sweep_match_budget_us = 60000000, sweep_confirm_wait_us = 1200000;
    // No ordinary supply in view but a cluster of other objects is (they may wrap it): turn to the
    // cluster, plough through it and swing, then search again.
    bool lost_search_rescan = true;
    bool blind_scatter = true;
    bool blind_scatter_need_ordinary = true;
    int blind_scatter_min = 3, blind_scatter_max = 4, blind_scatter_passes = 1,
        blind_scatter_swings = 3;
    float blind_scatter_min_y_m = .10f, blind_scatter_max_y_m = 1.4f, blind_scatter_half_x_m = .9f,
          blind_scatter_depth_m = .20f, blind_scatter_max_m = 1.3f,
          blind_scatter_align_tol_rad = .10f;
    // A lock beyond blind_scatter_far_chase_m is unreliable (class / position off): it is driven
    // toward whatever it is. The lock plus the objects within far_radius_m must reach far_lock_min
    // (1 = a lone lock is chased too); a lock-less cluster needs far_min. The drive ends in a sweep
    // once a group is inside blind_scatter_max_y_m, or in the normal approach once the lock is.
    float blind_scatter_far_radius_m = .60f, blind_scatter_far_chase_m = .80f;
    int blind_scatter_far_min = 3, blind_scatter_far_lock_min = 1;
    float blind_scatter_far_max_y_m = 1.4f, blind_scatter_approach_stop_m = .42f;
    int blind_scatter_approach_max = 3;
    uint64_t blind_scatter_approach_budget_us =
        14000000; // clusters beyond blind_scatter_max_y_m: drive closer first
    uint64_t blind_scatter_wait_us = 2500000, blind_scatter_align_budget_us = 6000000,
             blind_scatter_budget_us = 30000000, blind_scatter_cooldown_us = 3000000;
    float blind_scatter_align_wz = 2.4f; // align turn command (far); half of it near the bearing
    // Only clusters within +-max_bearing of straight ahead are ploughed (the pick is stable, not
    // the densest anywhere); the first one waits for first_turn_rad of scanning. After the swings
    // the base backs out the distance it drove in (blind_scatter_return), so it ends near where it
    // started instead of deep in the pile.
    float blind_scatter_max_bearing_rad = .70f, blind_scatter_first_turn_rad = 4.7f;
    // The back-out covers only this fraction of the distance driven in: a full retreat leaves the
    // pushed objects farther away than the sweep found them.
    bool blind_scatter_return = true;
    float blind_scatter_back_frac = .35f, blind_scatter_back_speed = .30f;
    uint64_t blind_scatter_back_budget_us = 6000000;
    // First far search of the match, once only: the first SCAN-triggered far-group drive (>=
    // first_burst_min_drive_m of real distance) runs at first_burst_speed until first_burst_slow_m
    // of real distance remains, then at approach_speed; the first near scatter after it swings the
    // frame at first_burst_swing_wz instead of disperse_wz.
    bool first_burst_full_impact = false; // live opening: no slowdown through the first near-pile
                                          // impact; later sweeps remain normal
    bool first_burst = false;
    float first_burst_speed = 1.0f, first_burst_slow_m = .35f, first_burst_min_drive_m = .60f,
          first_burst_swing_wz = 5.0f;
    bool post_clear_mid_view = true; // after a scatter/disperse the cargo is close: search at the
                                     // 20deg view instead of 5deg/40deg
    bool sweep_view_handoff = true;  // SCATTER uses the carry camera ladder (5deg -> 20deg at
                                     // track_near_m -> 40deg at intermediate_to_near_m), down only
    bool sweep_view_cap_mid = false; // SCATTER's pre-sweep camera stops at the 20deg stage (a near
                                     // group starts there), never 40deg
    bool sweep_far_judge = false;    // the swing itself (rush in, swing, back out) runs at the far
                                  // (5deg) view; the close-range observation guard is skipped there
    bool swing_avoid_outside = false;
    float swing_avoid_radius_m = .45f,
          swing_avoid_margin_rad =
              .15f; // swing sides stop short of blocks outside the group (judged at the far view)
    bool approach_dense_burst_swing = false; // APPROACH dense-pile sweep swings at
                                             // first_burst_swing_wz like the opening SCATTER sweep
    uint64_t relock_window_us = 1500000;
    float relock_gate_frac =
        .05f; // PixelSelector: re-lock prefers the candidate nearest the vanished lock for this
              // long; grace gate as a fraction of image width
    int lock_max_visible =
        0; // SCAN: with more than this many targets in view (lock included) do not lock, open-frame
           // sweep the nearest pair+ group instead (0 = off)
    int lock_multi_group_min = 2;
    float lock_multi_radius_m =
        .60f; // group size / radius that counts as 'several targets together' for that rule
    bool dense_multi_ignore_gap = false; // the multi-target sweep ignores the sweep_gap_us cooldown
                                         // (total / time budgets still apply)
    // After the rush-in: stop, lift the camera to the 20deg stage, then turn once in place toward
    // the side with fewer blocks in view (no oscillation, no re-centring). The turn runs until at
    // least disperse_turn_min_rad AND the view ahead of the frame mouth holds <=
    // disperse_turn_clear_max blocks for disperse_turn_clear_frames fresh frames, never past
    // disperse_turn_rad.
    bool disperse_single_turn = false;
    float disperse_turn_min_rad = .60f, disperse_turn_view_range_m = 1.0f,
          disperse_turn_view_half_rad = .60f;
    int disperse_turn_clear_max = 1, disperse_turn_clear_frames = 2;
    uint64_t disperse_turn_look_us = 1200000;
    bool row_select = false;
    int row_min_n = 3;
    float row_resid_m = .03f, row_max_tilt_rad = .6f, row_half_along_m = .09f,
          row_half_across_m = .06f; // PixelSelector row mode: middle block of a left-right row
    float disperse_turn_push_m = 0.f;
    uint64_t disperse_turn_push_budget_us = 3000000; // after the single turn: drive straight on
                                                     // this far (0 = off) before the sweep ends
    // Frame-zone check (needs the 20deg stage): once the view ahead is thin and
    // disperse_turn_min_rad is reached, the camera drops to track_pitch (lowest) and the turn goes
    // on until the blocks seen in the zone 0 < y <= disperse_turn_frame_range_m, |x| <=
    // disperse_turn_frame_half_m (body frame) number <= disperse_turn_frame_max for
    // disperse_turn_clear_frames fresh frames, never past disperse_turn_rad.
    bool disperse_turn_frame_check = false;
    int disperse_turn_frame_max = 1;
    float disperse_turn_frame_range_m = .30f, disperse_turn_frame_half_m = .14f,
          disperse_turn_frame_extra_rad =
              .25f; // extra: in-frame blocks rotate with the frame, so cap the turn at this much
                    // past the thin-view point
    uint64_t occlusion_hold_us = 0;
    float occlusion_near_m = .8f; // APPROACH: a target lost this close is assumed hidden by the
                                  // frame; stand still this long waiting for its new id
    // PILE: the corridor ahead of the frame mouth holds many blocks. Look far (5deg) for an empty
    // bearing, carry the pile there with the frame open while turning a large angle, then (one
    // layer left) back off a little, measure the remaining cluster and split it with one side edge
    // of the frame.
    bool pile_clear = false;
    int pile_corridor_min = 3, pile_confirm = 5, pile_max_runs = 2;
    float pile_trigger_max_m = .75f;
    uint64_t pile_budget_us = 40000000, pile_cool_us = 15000000;
    float pile_advance_m = .5f, pile_advance_speed = .25f, pile_turn_wz = 2.0f,
          pile_align_tol_rad = .22f, pile_block_half_m = .03f;
    uint64_t pile_advance_budget_us = 12000000, pile_back_budget_us = 9000000;
    float pile_cluster_radius_m = .45f, pile_cluster_half_x_m = .55f, pile_view_back_m = .20f,
          pile_frame_half_m = .14f, pile_cut_depth_m = .2f, pile_cut_speed = .25f;
    uint64_t pile_measure_us = 700000, pile_cut_budget_us = 8000000;
    int pile_cut_max_steps =
        1; // PILE cut: while the enclosed set still mixes in a non-ordinary block, try 2..N
           // block-widths of turn to each side (1 = one block only)
    // PILE chain: a disperse sweep that really swung hands over to PILE instead of the plain
    // re-search. While the cluster ahead of the frame is deeper than pile_layer_depth_m (more than
    // one layer), rush in pile_chain_rush_m and turn the big angle again (the disperse step), then
    // back off pile_chain_back_frac of the way in and re-measure; at most pile_chain_max rounds,
    // then the PILE cut splits the last layer. The big-angle turn leaves the pushed-out blocks
    // outside the camera's field of view, so after the back-off the car first looks where they went
    // (the middle of the turn, then the heading it rushed in on) before it gives the cluster up.
    bool pile_chain = false;
    int pile_chain_max = 3;
    float pile_layer_depth_m = .08f, pile_chain_rush_m = .15f, pile_chain_back_frac = .35f,
          pile_chain_min_back_m = .2f;
    bool dense_no_lock = false;   // SCAN: a group of several objects is split by a committed sweep
                                  // instead of locking one of them
    bool sweep_non_supply = true; // a locked target that is not ordinary_supply is swept aside
                                  // (rush + swings) instead of grabbed, then vetoed
    // Open-jaw corridor rejected because a few neighbours sit beside the target: nudge them aside
    // with a small closed-frame swing, then re-acquire.
    bool mid_neighbour_clear = true;
    int mid_neighbour_clear_max = 2, mid_neighbour_clear_swings = 2;
    // Selection-stage corridor pre-check (APPROACH, before opening the frame): a target whose
    // corridor already shows a fatal verdict is vetoed so another one is picked. Capped per match
    // so it can never become a permanent search deadlock; beyond the cap PREPARE (6-frame
    // confirmation) alone decides, as before. RUSH no-progress watchdog (field 2026-10-12 01:31:
    // distance sat at 0.18 m for ~9 s under a forward command while a static neighbour also stayed
    // fixed in the body frame). Window of fresh, identity-consistent, camera-stable samples without
    // rush_stall_progress_m of closing; the cause (blocked, pushing, ranging error, intervention)
    // is not distinguished here.
    uint64_t rush_stall_us = 2000000, rush_stall_gap_us = 600000, rush_stall_reobserve_us = 600000,
             rush_stall_reobserve_max_us = 2000000;
    float rush_stall_progress_m = .01f;
    int rush_stall_min_samples = 8, rush_stall_reobserve_frames = 3;
    // Enabled by live main; recorded legacy replays retain their original motion contract.
    bool precise_rush = false, sweep_observation_guard = false;
    bool demo_capture_motion =
        false; // demo opening/pitch/rush sequence, independent of assume_all_safe
    float rush_align_deadband_rad = .04f, rush_align_wz = .25f, rush_near_speed = .16f,
          rush_slow_range_m = .24f;
    uint64_t sweep_reobserve_us = 600000;
    bool frame_open_default =
        false; // the frame stays open in every search/approach/sweep state with nothing held; it
               // closes only to capture (LOWER_FRAME) and stays closed while carrying
    bool sweep_frame_open = false; // sweeps (SCATTER, disperse swings, upright nudges) run with the
                                   // frame open; cargo logic still sees the frame as closed
    // VERIFY_CAPTURE: a core/injured seen in the frame when it is illegal (core/injured before the
    // first delivery) is a possible false class (field 2026-10-12 01:31: core_supply conf .54 boxed
    // on a cable connector). Hold still and re-check before dropping; dangerous objects and
    // detections without identity keep the prompt two-frame abort. Unresolved never counts as a
    // capture.
    bool held_recheck = true;
    int held_recheck_confirm_frames = 4, held_recheck_strong_frames = 2,
        held_recheck_clear_frames = 3;
    float held_recheck_strong_conf = .75f;
    uint64_t held_recheck_us = 1000000;
    bool corridor_precheck = true;
    int corridor_precheck_confirm = 3, corridor_precheck_max = 4;
    float mid_neighbour_clear_turn_rad = .45f, mid_neighbour_clear_gap_m = .04f,
          mid_neighbour_clear_max_rush_m = .12f, mid_neighbour_clear_creep_m = 0.f;
    bool weak_target_scatter = true;
    float weak_pixel_ratio = .0032f;
    uint64_t weak_scatter_wait_us =
        600000; // few-pixel ordinary sighting: plough through instead of chasing
    // 走廊判定不合法时（物资过多/危险物/遮挡等），把目标前方走廊内的邻居（任意类别，只扫不搬）用同一套低框原地转向扫开。默认关闭。
    bool clear_neighbours = false;
    float clear_half_m = .14f, clear_front_gap_m = .03f;
    // 张框距离内航向略超 rush_heading_rad 时，小 wz 底盘不转(实测 0.25rad/s 陀螺≈0)：改用 IMU
    // 量角的 rush_pulse_wz 短脉冲转到位，停稳后再由视觉复核。
    bool rush_pulse_turn = false;
    float rush_pulse_wz = .55f, rush_pulse_max_rad = .20f, rush_pulse_done_frac = .85f;
    uint64_t rush_pulse_budget_us = 2500000, rush_pulse_settle_us = 500000;
    int rush_pulse_max = 4;
    bool assume_probe = true;
    uint64_t assume_probe_first_us = 1200000, assume_probe_len_us = 900000,
             assume_probe_gap_us = 1500000; // 直推时周期性小偏航探测，让IMU能看出堵转
    uint64_t assume_verify_fix_wait_us = 7000000, assume_verify_far_after_us = 1500000;
    float assume_inside_min_hold_y_m = .06f,
          assume_blocked_deep_hold_y_m =
              .10f; // 视觉定不了货物(被遮/截断)时，IMU堵转且框已深入此深度→视为推到位，不重推
    // 假设模式：后退后、前推前先用视觉把方框对准货物（原地转向），推进时按货物方位纯追踪，斜推角收小；货物丢失后保持丢失前航向。默认关闭。
    bool assume_align_push = false;
    float assume_align_tol_rad = .06f, assume_align_wz = .30f, assume_slant_max_rad = .45f;
    uint64_t assume_align_budget_us = 3000000;
    uint64_t attempt_budget_us = 0; // enabled by schema 2, never reset by target reselection
    float approach_limit_m = 0, retreat_limit_m = 0;
    bool enable_search_cues = false;
    bool enable_short_push = false;
    bool enable_lost_search = false;
    float cue_approach_speed = .4f, cue_prepare_distance_m = .36f;
    float clear_speed = .4f, clear_extra_m = .08f, clear_max_commanded_m = .22f;
    float closed_front_y_m = .225f, clear_heading_limit_rad = .15f;
    int max_clear_attempts = 2;
    uint64_t cue_approach_budget_us = 15000000, clear_wall_budget_us = 7000000,
             clear_pause_us = 500000;

    float approach_speed = .4f, rush_speed = .4f, carry_speed = .08f;
    float enter_speed = .04f, back_speed = .04f, max_speed = .4f;
    float min_turn_wz = .4f; // nonzero commanded yaw rate, rad/s
    float heading_gain = 1.5f, lateral_gain = 1.0f;
    // Approach steering: below this bearing drive straight, so min_turn_wz never turns an aligned
    // approach.
    float heading_deadband_rad = .05f;
    float scan_wz = .25f, turn_wz = .3f, max_wz = .4f, rotate_in_place_rad = .45f;
    // SCAN: a block locked for visual_chase_frames frames (more than 3) at confidence >=
    // visual_chase_min_conf that cannot be projected yet (far / no ground contact) stops the spin
    // and is driven at by its pixel bearing until the ground geometry works and the normal lock
    // enters APPROACH. Up to visual_chase_miss_frames lost frames are tolerated.
    bool visual_chase = true;
    int visual_chase_frames = 4, visual_chase_miss_frames = 4;
    float visual_chase_min_conf = .5f;
    float visual_chase_speed = .3f;
    uint64_t visual_chase_budget_us = 6000000, visual_chase_cool_us = 4000000,
             visual_chase_hold_us = 600000;
    // SCAN: the spin (about 9 deg per frame) carries a block out of view before the pixel lock (3
    // frames) and visual_chase (4 frames) can complete. While the pixel selector has a candidate /
    // lock (or visual_chase has recent hits) the base stops turning, for at most scan_hold_max_us;
    // a hold that ends without a chase is followed by scan_hold_cool_us of spinning.
    bool scan_sighting_hold = true;
    uint64_t scan_hold_max_us = 1500000, scan_hold_lost_us = 400000, scan_hold_cool_us = 1500000;
    float inplace_turn_boost =
        1.f; // multiplies in-place turn rate in approach-type states, after the max_wz clamp
    float startup_advance_speed = .1f;
    float startup_hold_gain = 1.5f,
          startup_hold_deadband_rad = .035f; // IMU yaw hold while START_ADVANCE crosses bumps
    uint64_t startup_advance_us = 10000000;  // 0.1 m/s for 10 s; phase bound, not measured distance
    // 原地转圈卡死（SCAN/LOST_SEARCH指令转向但陀螺≈0）：先后退再前进各一段，晃动脱困。
    bool spin_unstick = true;
    uint64_t spin_stall_trigger_us = 1500000, spin_unstick_leg_us = 800000;
    float spin_unstick_speed = .15f;
    // 盲冲（目标落入5°视野盲区后）用IMU偏航保持最后一次可见时的目标方位；落框前偏航偏差超限则先原地对正，超时放弃本次（不拉黑）。
    bool rush_yaw_hold = true;
    float rush_lower_yaw_tol_rad = .08f, rush_realign_wz = .3f;
    uint64_t prelower_realign_us = 1500000;
    float rush_start_m = .35f, rush_heading_rad = .06f, rush_extra_m = .15f,
          rush_abort_heading_rad = .35f;
    float grasp_trigger_y_m = .20f, hold_center_y_m = .12f, injured_hold_center_y_m = .14f,
          mouth_y_m = .21f;
    float supply_half_x_m = -.15f, injured_half_x_m = .15f; // zone frame, supply half on the left
    float gate_clearance_m = .25f, gate_tolerance_m = .04f, gate_yaw_rad = .05f,
          gate_lateral_m = .06f;
    // Release: push the held centre to deposit_y (half depth); afterwards reverse until the
    // open mouth is mouth_clear_y outside the entrance.
    float deposit_y_m = .15f, enter_lateral_m = .09f, mouth_clear_y_m = -.05f, abort_back_m = .20f;
    // Release retreat without zone geometry: after waiting this long for the zone, reverse straight
    // by dead reckoning, at most abort_back_m and never more than was driven forward since the
    // lock; stop if the IMU heading drifts beyond blind_retreat_yaw_rad or goes missing.
    uint64_t blind_retreat_wait_us = 500000;
    float blind_retreat_yaw_rad = .15f;
    // Strict PREPARE corridor check: identical inventory on this many consecutive frames. A
    // corridor holding other objects keeps the frame up, reverses corridor_back_m (real distance;
    // commanded = x motion ratio, skipped when retreat_safe is false) and searches again; after
    // corridor_retry_max such back-offs in a row the target is blacklisted.
    int corridor_confirm_frames = 3,
        corridor_retry_max = 2; // live main raises the frame count (main.cpp)
    float corridor_back_m = .20f;
    // Turn away from the zone before accepting a new target, so nothing is re-picked from it.
    float turn_min_rad = 1.6f;
    int confirm_frames = 3, capture_frames = 4, delivery_frames = 6, hold_grace_frames = 6;
    uint64_t frame_timeout_us = 200000, gripper_timeout_us = 2500000;
    uint64_t approach_budget_us = 15000000, prepare_budget_us = 4000000, rush_budget_us = 6000000;
    uint64_t verify_budget_us = 1500000, carry_budget_us = 25000000, gate_budget_us = 4000000;
    // Searching our zone with a load and no fix: spin carry_search_turns turns (or until a
    // sighting), then drive a leg toward the sighted/remembered zone (else back along the start
    // line) and spin carry_search_respin_turns again. Sightings older than carry_sight_mem_us are
    // dropped.
    float carry_search_turns = 2.f, carry_search_respin_turns = 1.f;
    uint64_t carry_leg_us = 6000000, carry_leg_turn_cap_us = 20000000, carry_spin_cap_us = 40000000,
             carry_sight_fresh_us = 1500000, carry_sight_mem_us = 40000000;
    uint64_t zone_search_budget_us = 45000000; // includes stopped NEAR checks while searching
    // Holding a load while searching our zone: after enemy_evade_frames consecutive enemy-colour
    // frames, turn about (IMU-measured, in the scan direction) and drive a long straight leg away
    // (enemy_evade_m is real distance; commanded = x motion ratio).
    bool enemy_evade = true;
    int enemy_evade_frames = 3;
    float enemy_evade_turn_rad = 2.9f, enemy_evade_m = 1.0f, enemy_evade_speed = .4f;
    uint64_t enemy_evade_turn_budget_us = 15000000, enemy_evade_drive_budget_us = 14000000,
             enemy_evade_cool_us = 3000000;
    // Enter covers gate_clearance + deposit_y + tolerance (~0.48 m) at enter_speed; retreat ~0.3 m
    // back.
    uint64_t enter_budget_us = 15000000, retreat_budget_us = 10000000, delivery_budget_us = 2500000;
    uint64_t turn_budget_us = 15000000, blacklist_us = 20000000;
    // Camera presets, 0.01 deg positive down; ground mapping still requires pitch-specific
    // validation.
    int16_t far_pitch_cdeg = 500, track_pitch_cdeg = 500, near_pitch_cdeg = 4000;
    int16_t pitch_tolerance_cdeg = 100;
    int16_t intermediate_pitch_cdeg = kCameraPitchInvalid; // optional 20deg observation stage
    float intermediate_to_near_m = .4f;
    uint64_t approach_stall_us = 3000000;
    float approach_stall_progress_m = .01f,
          approach_stall_accept_m = .32f; // approach_to_prepare_range: range flat for this long
                                          // within accept range -> treat as arrived
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
    explicit PushTask(TaskTuning tuning = {}) : t_(tuning) {
        first_ = t_.assume_injured_trip;
    }
    PushOutput update(const PushObservation& in);
    const std::string& anchorDebug() const {
        return anchor_dbg_;
    }
    std::string takeDropZoneEvent() {
        std::string s;
        s.swap(dropzone_evt_);
        return s;
    }
    static const char* name(PushState state);
    // Sweep ledger read-out (diagnostics / tests): finished sweeps this trip, ledger time including
    // a running sweep, and whether a sweep chain is open.
    int sweepDone() const {
        return sweep_ledger_.done();
    }
    uint64_t sweepSpentUs(uint64_t now) const {
        return sweep_ledger_.spentUs(now);
    }
    bool sweepChainOpen() const {
        return sweep_ledger_.chainOpen();
    }
    std::vector<int> rejectedTargets(uint64_t now_us) const;
    struct RejectedBearing {
        float world_yaw;
        float range_m;
        uint64_t until_us;
    };
    std::vector<RejectedBearing> rejectedBearings(uint64_t now_us) const;

  private:
    TaskTuning t_;
    PushState state_ = PushState::WAIT_START, stopped_from_ = PushState::WAIT_START;
    bool cue_mode_ = false;
    int clear_attempts_ = 0;
    int near_failures_ = 0;
    uint64_t clear_commanded_us_ = 0, clear_planned_us_ = 0;
    int search_cue_id_ = -1, capture_target_id_ = -1;
    int target_id_ = -1, confirmations_ = 0, misses_ = 0, step_ = 0;
    float last_range_ = 0; // last tracked range in MID_APPROACH, inf when none
    int total_ = 0, baseline_ = 0, baseline_other_ = 0, hold_misses_ = 0;
    std::string delivery_geometry_id_;
    std::string label_, reason_;
    Inventory trip_, pending_, seen_;
    RuleVerdict verdict_ = RuleVerdict::OK;
    bool first_ = false, fault_ = false, track_close_ = false;
    uint8_t frame_raised_ = 0; // logical open, independent of wire angle
    bool sweep_open_ = false, sweep_open_done_ = false, sweep_open_disabled_ = false,
         sweep_close_wait_ = false; // sweep_frame_open overlay on the wire angle only
    uint64_t sweep_open_us_ = 0, sweep_close_us_ = 0;
    uint64_t frame_transaction_ = 0, attempt_started_ = 0;
    uint64_t inplace_since_us_ = 0,
             inplace_last_us_ =
                 0; // continuous in-place turn tracking (demo APPROACH/RUSH turn escalation)
    int wire_last_ = -1;
    uint64_t wire_txn_ = 0; // wire-level target/transaction: only a real open<->close change is a
                            // new action for the MCU
    float bounded_approach_ = 0, bounded_retreat_ = 0;
    int16_t multi_view_pitch_ = kCameraPitchInvalid;
    int16_t pitch_cmd_ = t_.far_pitch_cdeg;
    // PILE state
    int pile_phase_ = 0, pile_runs_ = 0, pile_seen_ = 0;
    uint64_t pile_phase_us_ = 0, pile_cool_until_us_ = 0;
    float pile_yaw0_ = 0, pile_push0_ = -1.f;
    int pile_chain_n_ = 0;
    int approach_last_id_ = -1;
    float approach_last_heading_ = 0.f, approach_last_distance_ = 0.f,
          approach_last_world_yaw_ = 0.f;
    bool approach_last_world_valid_ = false;
    bool approach_rebind_ = false; // 分级下压后允许改绑跟踪器新ID一次
    int16_t approach_pitch_ =
        kCameraPitchInvalid; // APPROACH分级下压：远处保持当前角，贴近时 5°→20°→40°
    int16_t sweep_pitch_ = kCameraPitchInvalid;
    bool sweep_non_supply_active_ = false; // SCATTER的同款分级下压；非物资目标的扫动标志
    uint64_t startup_commanded_us_ = 0;
    float startup_yaw0_ = 0;
    bool startup_yaw0_set_ = false;
    uint64_t zone_search_started_us_ = 0;
    float run_yaw0_ = 0;
    bool run_yaw0_set_ = false; // heading at the first update after reset: the start line
    bool zone_mem_ = false, zone_enemy_ = false;
    float zone_mem_yaw_ = 0, zone_enemy_yaw_ = 0;
    uint64_t zone_mem_us_ = 0;
    uint64_t last_us_ = 0, phase_us_ = 0, gripper_us_ = 0, carry_us_ = 0, pitch_wait_us_ = 0,
             hold_seen_us_ = 0;
    // Dead-reckoned from the previous command; only bounds a phase, never proves progress.
    float travel_ = 0, travel_limit_ = 0, turned_ = 0, last_heading_ = 0, carried_ = 0;
    float last_vx_ = 0, last_wz_ = 0;
    bool drop_locked_ = false;
    std::string anchor_dbg_;
    bool half_locked_ = false, half_injured_ = false;
    float half_x_ = 0; // 本趟目标半区（进入搬运时锁定，整趟不变）
    std::string dropzone_evt_;
    void lockDropHalf(const char* source);
    void noteDropZone(const char* event, const cv::Point2f& hold);
    bool cargoZone(const PushObservation& in,
                   uint64_t now,
                   cv::Point2f& body,
                   cv::Point2f& rear,
                   bool& fix,
                   int* idx = nullptr) const;
    bool cargoInZone(const PushObservation& in, uint64_t now) const;
    static PushObservation::CargoImg imgAt(const PushObservation& in, int idx) {
        return idx >= 0 && size_t(idx) < in.cargo_img.size() ? in.cargo_img[idx]
                                                             : PushObservation::CargoImg{};
    }
    void noteCargo(const char* event, const cv::Point2f& rear, bool fix);
    void trackCargo(const PushObservation& in, uint64_t now);
    int scatter_approach_n_ = 0;
    bool scatter_far_ = false;
    float scatter_approach_m_ = 0;
    bool burst_drive_used_ = false, burst_fast_ = false, burst_swing_armed_ = false,
         burst_swing_ = false; // first_burst: drive spent / this far drive is fast / swing waiting
                               // for the next near scatter / this scatter swings fast
    int scatter_count_ = 0, scatter_seen_ = 0;
    bool scatter_commit_ = false;
    uint64_t scatter_cool_until_us_ = 0, scatter_idle_since_us_ = 0;
    float scatter_yaw_ = 0, scatter_range_ = 0, scan_turn_acc_ = 0;
    float scan_mid_limit_ = 0;
    bool post_disperse_mid_ = false;
    bool disperse_small_ = false;
    int mid_clear_n_ = 0;
    int precheck_seen_ = 0, precheck_rejects_ = 0;
    bool rush_stall_phase_ = false;
    int rush_stall_id_ = -1, rush_stall_samples_ = 0, rush_stall_obs_frames_ = 0;
    float rush_stall_ref_m_ = 0;
    uint64_t rush_stall_ref_us_ = 0, rush_stall_last_us_ = 0, rush_stall_obs_us_ = 0;
    int16_t rush_stall_pitch_ = kCameraPitchInvalid;
    int sweep_phase_ = 0, sweep_attempts_ = 0, sweep_seen_ = 0;
    float sweep_travel0_ = 0;
    float disperse_rush_m_ = 0, disperse_sign_ = 1;
    uint64_t disperse_cool_until_us_ = 0;
    float sweep_yaw0_ = 0;
    uint64_t sweep_start_us_ = 0, sweep_t0_us_ = 0;
    int unstick_step_ = 0;
    uint64_t unstick_us_ = 0;
    bool rush_yaw_ref_set_ = false;
    float rush_yaw_ref_ = 0;
    uint64_t prelower_us_ = 0;
    int rush_pulse_phase_ = 0, rush_pulse_count_ = 0;
    bool uprightInCorridor(const PushObservation& in,
                           uint64_t now,
                           float& lateral,
                           float* along_out = nullptr) const;
    bool neighbourInCorridor(const PushObservation& in,
                             uint64_t now,
                             float& lateral,
                             float* along_out = nullptr) const;
    bool cargoVisible(uint64_t now) const {
        return cargo_seen_us_ && now - cargo_seen_us_ <= 1500000;
    }
    float backTarget(uint64_t now) const;
    bool rushLogic() const {
        return t_.assume_all_safe || t_.field_rush;
    }
    bool carryLogic() const {
        return t_.assume_all_safe || t_.field_carry;
    }
    bool assumeView() const {
        return carryLogic() && t_.assume_view_pitch_cdeg != kCameraPitchInvalid;
    }
    int repush_n_ = 0;
    uint64_t cargo_last_ts_ = 0, cargo_ref_us_ = 0, cargo_seen_us_ = 0;
    cv::Point2f cargo_ref_body_, cargo_ref_zone_;
    bool cargo_fix_ = false, cargo_clipped_ = false;
    float cargo_rear_x_ = 0, cargo_rear_y_ = 0;
    cv::Point2f cargo_body_;
    bool delivery_visual_ = false;
    bool assume_pushed_ = false; // 假设模式：已完成“松开后再用下压姿态向区内推”
    bool assume_blind_ = false; // TEMP_ASSUMPTION blind leg active in this phase
    float assume_rush_end_ = 0;
    cv::Point2f last_target_xy_;
    bool last_target_valid_ =
        false; // body-frame position of the locked target on its last tracked frame
    CaptureVerificationStage capture_verification_;
    DeliveryVerificationStage delivery_verification_;
    ReleaseRetreatStage release_retreat_;
    PushInStage push_in_;
    EnterStage enter_;
    CarryApproachStage carry_approach_;
    CarrySearchStage carry_search_;
    SweepTriggerPlanner sweep_trigger_;
    SweepLedger sweep_ledger_;
    DisperseRushStage disperse_rush_;
    DisperseTurnStage disperse_turn_;
    DisperseSwingStage disperse_swing_;
    RecoveryStage recovery_;
    SearchStage search_;
    SweepStage sweep_;
    // 假设模式：YOLO-pose粗略区域位姿锚定在里程系，视野丢失后用IMU航向+缩放后的指令位移继续推算。
    AnchoredZone zone_;
    float odo_ = 0, ratio_est_ = 0, ratio_odo_ = 0, last_imu_yaw_ = 0;
    cv::Point2f ratio_hv_;
    uint64_t vref_fix_us_ = 0, vref_seen_us_ = 0;
    bool ratio_snap_ = false, imu_seen_ = false, vref_used_ = false;
    float drRatio(uint64_t now) const {
        const bool stale = now - vref_seen_us_ > 400000;
        return motionRatio() *
               ((stale && (state_ == PushState::CARRY || state_ == PushState::BACK_OUT))
                    ? t_.assume_dr_pessimism
                    : 1.f);
    }
    float motionRatio() const {
        return ratio_est_ > 0 ? ratio_est_ : t_.assume_motion_ratio;
    }
    cv::Point2f drop_centre_zone_;
    bool heading_seen_ = false;
    int lost_round_ = 0; // LOST_SEARCH: 0 FAR sweep, 1 intermediate (20deg) sweep for close cargo
    bool corridor_retreat_ = false;
    int corridor_retries_ =
        0; // LOST_HOLD reused as a no-blacklist back-off from a blocked corridor
    float approach_forward_ = 0; // forward commanded since the trip started
    std::vector<RejectedBearing> rejected_bearings_;
    std::map<int, uint64_t> rejected_; // primary track id -> blacklist expiry

    void enter(PushState s, uint64_t now, const char* why);
    void clearTrip();
    void blacklist(uint64_t now);
    void fail(const char* why);
    void commandFrame(uint8_t open, uint64_t now);
    bool sweepFramePhase() const;
    bool defaultOpenState() const;
    void syncFrameOverlay(uint64_t now);
    int frameWait(const PushObservation& in,
                  uint8_t open,
                  uint64_t now) const; // 1 done, 0 pending, -1 timeout
    bool candidate(const PushObservation& in, uint64_t now) const;
    bool weakCandidate(const PushObservation& in) const;
    float approachHeadingShift(
        const PushObservation& in) const; // bearing change vs the last tracked frame, corrected for
                                          // the body's own rotation since
    bool sameObject(const PushObservation& in,
                    uint64_t now) const; // re-ID'd detection of the locked target (same class, body
                                         // position near the last tracked one)
    bool tracking(const PushObservation& in) const;
    bool midPitch() const {
        return t_.intermediate_pitch_cdeg > t_.far_pitch_cdeg &&
               t_.intermediate_pitch_cdeg < t_.track_pitch_cdeg;
    }
    bool zoneOk(const PushObservation& in) const;
    bool holding(const PushObservation& in) const;
    int holdSeen(const PushObservation& in) const; // 1 seen, 0 in view but not held, -1 out of view
    int16_t desiredPitch() const;
    int pitchWait(const PushObservation& in, uint64_t now); // 1 ready, 0 moving, -1 timeout
    float halfX() const;
    float holdCenter() const;
    PushState resumeState() const;
};
} // namespace rescue
