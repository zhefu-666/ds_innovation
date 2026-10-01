# 实车第1步：串口下发与夹爪反馈（2026-10-01）

## 改动

- 新增 `MotionLink`（`include/rescue/motion_link.hpp`、`src/motion_link.cpp`）：后台线程管理 0x56 包下发。
  - `submit()` 立即写一包，之后每 40 ms（25 Hz）重发最近命令。
  - 命令超过 150 ms 未更新时改发零速度包，夹爪和 pitch 保持不变，计入 `stale`。
  - 第一次 `submit()` 前不发包。`stop()` 和析构都会补发一包零速度包，异常退出时也会发。
  - `healthy()` 需要同时满足：150 ms 内有新命令、最近一次写入成功、A6 反馈有效。不满足时否决 `safety_ok`。
- `UARTController`：
  - `syncGripperActionId()`：启动时读取下位机当前动作编号，首个夹爪包用“当前编号+1”，避免上位机重启后编号与下位机旧编号相同而被误判完成。
  - `gripperAck()`：在同一把锁内返回完成状态和对应目标。
  - 串口独占（flock+TIOCEXCL）扩展到读写模式。
- 主程序新增 `--hardware`：
  - 必须同时带 `--imu`，不能与 `--dry-run`、`--pitch-feedback`、回放或单图模式同用。
  - 启动时等待 A6 反馈 500 ms，没有反馈就退出，不发任何字节。
  - 每帧把 `PushTask` 的输出交给 `MotionLink`。夹爪 Done 时填入 `gripper_done/gripper_feedback_open`。
  - 每 500 ms 打印一行 `[MCU]` 状态。
  - 画面显示红字 “PUSH LIVE - MCU OUTPUT ENABLED”。

## 测试

- `build-arm64-telemetry` 构建退出码为 0，ctest 9/9 通过。`rescue_sensor_protocol_tests` 连跑 5 次全部通过。
- 伪终端覆盖以下行为：独占打开、编号同步（7→8）、首次 submit 前不发包、立即写入和周期重发、CRC、旧编号判为 NotDone、停更后改发零速、改目标时编号递增、stop 补发零速、stop 后不再发包、无反馈时同步超时。
- 参数校验在 `imu_tests` 中覆盖：单独 `--hardware`、与 dry-run/pitch-feedback/push-replay 组合、端口与 IMU 相同。

## 实车读回与联机

- 只读监视：A6 反馈有效，约 12 ms 一帧。
- 第一次 7 s 联机（板子重启前）：发送 214 包，failed=0，但下位机始终回报 fb_id=4、pitch 0.25°，执行器没动。后来执行器上电后此现象消失，判断为执行器未上电。
- 第二次手动测试（执行器上电，车轮命令始终为 0）：
  - `rescue_upper_host --hardware --imu --no-show`：发送 568 包，failed=0，stale_zero=0，healthy=1，Ctrl+C 后补发零速。
  - 夹爪关闭（编号 8，下位机原编号 7 → 同步后 +1）约 0.3 s 收到 `fb_id=8 fb_done=1`，`gripper ack=done`。**上位机下发和夹爪关闭反馈在实车上验证通过。**
  - 旧脚本 `gripper_open.sh`、`pitch_set.sh 12` 都报超时，但用户确认动作**在实车上实际发生了**。
- 下位机反馈问题（固件侧，上位机不能代填）：
  1. 夹爪**张开**完成后从不回报 done=1（编号 1/3/5/7/8 都是这样）：有时回报新编号但 done=0，有时仍回报旧编号。关闭（编号 2/4/6/8）正常。
  2. pitch 读回不反映实际角度：命令 12°（1200）时读回一直是 0.25°（25），主程序全程 `pitch_cmd=1200 pitch_rb=25`；命令 0° 时读回 0.00°。
- 影响：`PushTask` 的张开等待（OPEN_RELEASE 等）会在 2.5 s 超时；`cameraPitchResult` 永远不是 Done，依赖 pitch 的几何映射会被拒绝。
- 需要固件修复：
  - 收到新编号立即采用；张开动作结束后置 done=1（与关闭同一路径）。
  - A6 的 pitch 字段回报舵机实际（或已执行的目标）角度，单位 0.01°，而不是固定值。

## 剩余阻塞

- 实车上 `PushTask` 不会离开 WAIT_START：`safety_ok`（除 IMU/link 否决外）、`path_safe`、`retreat_safe`、`opponent_zone_clear` 和 `ground_contact_valid` 都没有生产者，`run` 需要 `--auto-run`。
- 建议增加台架执行器模式：车轮悬空，按脚本依次验证夹爪开合、pitch 三档（1200/2200/3500）和小速度正反转。
- `tools/` 脚本仍按 ±25° 截断 pitch，下位机已放宽到 ±35°。

启动命令（实车，车轮悬空）：

```bash
./build-arm64-telemetry/rescue_upper_host --hardware --imu --no-show
```
