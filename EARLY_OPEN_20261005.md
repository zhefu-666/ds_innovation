# 提前张爪和受阻重选

20°接近中，距离≤0.35m且方向对准后停车。连续3次通道库存检查通过才发张爪；必须动作编号和开爪状态反馈匹配才恢复低速前进。到0.20m停车转40°，重识别期间保持已张开的夹爪，候选通道须与开爪前批次一致，禁止盲目闭爪。

闭爪接近阶段受阻，立即停车并将当前货物ID放入已有5秒黑名单，转回20°重选；共用有限失败预算。已经张爪后遇到阻挡、通道变化或目标丢失，保持开爪停车，不在未知物体附近自动闭爪或旋转。

按方向放宽蓝色碰撞限制尚未启用：缺少机器人原点到前后左右边界和夹爪完整展开范围；现有0.32m圆形保护保留。因此现场蓝色仍在0.32m内时仍可能无法通行，此修改不声称已经解决所有障碍布局。需用户补充实测外形，才能完成动作扫掠碰撞模型。

历史夹爪编号不匹配不能凭同一开闭状态忽略；实际电控执行尚未验证。本次不启动实车。

## 尺寸已补充并接入

用户提供前0.22m、后0.10m、左/右各0.15m、张开直径0.18m（按最大横向宽度解释）。张开前端暂沿用0.22m。启用矩形前进扫掠检查、0.20m后退段扫掠、静止张爪空间检查；旋转仍用0.32m包络，弧线使用更保守的全行程旋转包络，后退转弯不放行。蓝色投影是近侧边，按4cm立方体对角线包住完整占地，这是物体大小不是额外0.15m保护距离。无定位/未知类别继续否决。普通直行不再由单一径向距离统一拦截；实际位于车体前进范围内的蓝色仍拒绝。20°提前张爪和受阻重选保持。软件验证不代替实际制动和开爪外形验收。

## 最小转向速度

用户确认小角速度不能带动底盘，非零wz最低幅值改为0.25rad/s（约14.3度/秒），正负方向保持。作用于任务状态机的原地转向及带转向的前进指令；0保持0，路径/旋转检查拒绝时不抬升速度，上限仍优先。实车是否克服静摩擦及是否出现摆动需验证，本次不自动运动。

## 2026-10-05 补充：张爪后通道复核、航向死区、首趟规则

- 航向死区 `TaskTuning::heading_deadband_rad = 0.05`：MID_APPROACH/APPROACH 中 |航向误差| ≤ 0.05 rad 时 wz=0 直行，不再被 `min_turn_wz`(0.4) 放大成大幅摆头，把通道边缘的邻近物资扫进通道。
- 张爪后通道变化：与张爪时批次不同，先原地停车复核 `confirm_frames` 帧（reason=`open_jaw_corridor_recheck`）。
  - 抖动一帧后恢复原批次：继续前进。
  - 连续确认为合法新批次：采用新批次继续。
  - 连续确认为非法（危险品/未知/超量/不完整）：SAFE_STOP `open_jaw_corridor_changed`。
- 首趟规则：首趟只能运送 1 个普通物资（`checkTrip` 在 `!first && supplies>1` 时返回 TOO_MANY_SUPPLIES）；之后每趟最多 2 个物资。
  - 因此首趟若邻近普通物资确实进入通道，仍会在静止复核后 SAFE_STOP。

## 2026-10-05 07:40 修订：首趟改为最多 2 个普通物资

实车日志 capture-scan-20261005-073355-13214：张爪后直行（wz=0），邻近普通物资 #8 横向由 0.127m 测到 0.109m 进入通道（半宽 0.11），复核 3 帧后因"首趟只运 1 个"停车。用户选择首趟允许 2 个普通物资：首趟仍只允许普通物资，核心/危险/未知/伤员照旧拦截；首趟仍要求通道无遮挡，否则 OCCLUDED 停车。

## 2026-10-05 08:10 hold check and blind retreat (log 075241-19907)

- Field: closed on #8 at y=0.158 m, VERIFY_CAPTURE timed out (capture_unverified). Blocks #11/#13 at
  y~0.29-0.31 m cross the top of the 40deg holding region [440,210,930,720] and made the set ambiguous
  (inferred by projection; the log had no hold fields). Then CAPTURE_FAIL opened but never reversed:
  the zone was not visible at FAR, so the retreat had no measurement -> retreat_unverified.
- A: CaptureMonitor ignores boxes whose measured, untruncated ground contact is beyond
  mouth_y_m + 0.03 m (mouth from the task calibration, 0.22 m). Unknown position stays ambiguous.
- B: new `[HOLD]` log line (observable/captured/complete/held counts, ambiguous/outside/unfollowed
  track ids); `[TARGET]` now has conf and box px.
- C1: CAPTURE_FAIL/LOST_HOLD/ABORT_DROP without zone geometry for 0.5 s reverse straight by dead
  reckoning at back_speed, distance min(abort_back_m=0.20, forward commanded since the scan lock).
  IMU heading required; drift > 0.15 rad or missing heading -> SAFE_STOP. Measured retreat unchanged
  when the zone is visible. Not yet field-tested.
- Backup: backups/hold-retreat-20261005. ctest 16/16, --check OK.
