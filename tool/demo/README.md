# 实机演示与远程查看

板卡部署目录：`/home/cat/ds_innovation/tool/demo`。
本地脚本副本：`/home/liu/ds_innovation/tool/demo`。

## 查看图像与数据（不发送运动控制）

板卡终端：
```bash
cd /home/cat/ds_innovation
tool/demo/view.sh --seconds 3600
```
电脑终端（保持打开）：
```bash
cd /home/liu/ds_innovation
tool/demo/connect.sh
```
浏览器：http://127.0.0.1:18080/
数据：http://127.0.0.1:18080/status
图像流：http://127.0.0.1:18080/stream.mjpg
Foxglove WebSocket：ws://127.0.0.1:18765

查看包含相机检测、目标决策、A6与IMU只读输入。每60秒预览进程自动重新启动，切换时可能短暂显示离线；总时限最多3600秒。数据过期保留离线标记，不假报在线。网页与旧8080服务隔离。

停止：查看终端Ctrl+C，或板卡运行 `tool/demo/view_stop.sh`。电脑隧道终端Ctrl+C。`tool/demo/stop.sh`仅停止动作/预览，不负责整个查看服务；停止整个服务用view_stop。

## 其他入口

`tool/demo/recognize.sh --run --seconds 30`：仅相机识别。
`tool/demo/search.sh --preview --seconds 30`：相机、MCU、IMU只读预览。
`tool/demo/frame_status.sh --run`：A6只读反馈。
`tool/demo/preflight.sh`：检查所有动作前提。
开闭、相机、四向点动、搬运、回放入口均位于同目录，默认不访问设备；运动命令仍需显式run和对应验收。运行其他入口前结束view会话，避免相机/串口争用。

配置：`tool/demo/runtime/config/demo_20261007.json`。
板卡二进制：`tool/demo/runtime/bin/rescue_upper_host`（ARM64，仅板卡安装）。
模型和库读取 `/home/cat/ds_innovation/models` 与 `benchmark_results`。
内部脚本日志：`tool/demo/runtime/telemetry_logs/demo/`。
查看会话日志：`tool/demo/logs/web.log`、`preview.log`。

## 本轮验证

从新目录启动成功；经电脑SSH隧道验证首页HTTP200、status JSON含检测结果且camera_ok/robot_data_connected为true、MJPEG含有效JPEG帧。IMU字段可读，部分采样超过200ms会显示fresh=false（不降低有效性阈值）。只读预览hardware_output_enabled=false。
正式寻找/搬运仍因验收缺项拒绝，这属于功能限制，不声称全动作验收通过。原旧只读识别进程正常结束后新入口取得相机；旧8080网页服务保留。

## 方框、移动和相机脚本清单

运行 `tool/demo/list.sh` 可在终端查看全部命令。

| 功能 | 脚本 | 参数示例（仅检查） |
|---|---|---|
| 抬起/开框0° | `tool/demo/frame_open.sh` | 无 |
| 放下/关框20° | `tool/demo/frame_close.sh` | 无 |
| 原始反馈 | `tool/demo/frame_status.sh` | `--run`只读 |
| 前进 | `tool/demo/move_forward.sh` | `--speed 0.05 --seconds 0.5` |
| 后退 | `tool/demo/move_backward.sh` | `--speed 0.05 --seconds 0.5` |
| 左转 | `tool/demo/turn_left.sh` | `--wz 0.20 --seconds 0.5` |
| 右转 | `tool/demo/turn_right.sh` | `--wz 0.20 --seconds 0.5` |
| 相机前方档 | `tool/demo/camera.sh` | `--offset-deg 5`，RX45° |
| 相机观察档 | `tool/demo/camera.sh` | `--offset-deg 20`或`40`，RX60°/80° |

默认是检查；执行动作需添加 `--run --site-clear`。先运行 `tool/demo/view_stop.sh` 释放相机/串口。使用已验收的自定义配置添加 `--profile /绝对路径/demo.local.json`。当前默认配置仍为pending，脚本存在且参数解析正常不代表动作前提已验收。

完成位模式不能从A6反推当前框角，保持动作需要本轮已知角度与编号，通过 `--known-frame-angle` 和 `--known-frame-id` 提供；不要套用历史固定编号。通用移动脚本仍限0.10m/s，先前0.4m/s独立测试没有改写这里的限速。

## 20261008 搬运演示（TEMP_ASSUMPTION，非验收）

使用 `--assume-all-safe`（含 `--assume-injured-trip`）跑通完整决策链：找到→接近→开框→进入→关框(+20°)→确认→搬运→释放(开框0°)→后退→核验→下一趟。蓝方，布局“左供给、右伤员”，伤员块横放。假设路径安全、对方区无人、伤员趟可进行，结果不作为验收。比赛时限3分钟，因此提高了速度。

### 运行结果

| 趟 | 目标 | 结果 | 说明 |
|---|---|---|---|
| 27 | 普通物资 | OK | 完整闭环 |
| 28 | 普通物资 | 误跑供给 | 非伤员块 |
| 29 | 伤员 | 失败 | 区域回退逻辑错误（已修，见下） |
| 30 | 伤员 | delivered_visual | 视觉核验通过 |
| 31 | 伤员 | wall_budget_exhausted | 搬运阶段顶着安全区边框反复推，耗尽预算 |
| 32 | 伤员 | delivered_assumed | 停滞释放生效（约28.5s触发），日志因断电丢失 |
| 33 | 伤员 | delivered_assumed，53.7s | 日志在 `tool/demo/logs/run33/carry.txt`，二进制 md5 前缀 1aa47c08 |

第33趟节奏：启动后约23.6s `assumed_capture`；约33s 搬运停滞触发 `assumed_drop_point_stalled`（原先顶边框约15-17s，现约1-2s）；随后释放、局部推入（`assumed_push_in_*`）、盲退（`assumed_blind_back_out`）、核验。核验2.5s内仅1次货区检测（需≥3），退出后目标/几何均丢失，回落为 `delivered_assumed`。第30趟同流程得到 delivered_visual，因此不是确定性错误。

### 已知问题（本轮未改，按第33趟现状同步）

- 核验视野丢失：退出后 `target_valid=0`、`points<3`。候选调整：核验预算2.5→4s。
- 首次后退（视觉判断）达到上限0.45m，由 `assume_lateral_back_max_m=.25` 封顶。候选调整：.25→.08。
- 停滞时长1.5s，候选调整：0.8s。
- 距离来源：`travel_`/`odo_` 积分的是指令速度×dt，不是实测位移（实际约为指令的1/4）；IMU只提供yaw、航向误差、转向和倾倒急停，与后退距离无关。

### 启动脚本

每趟前准备（板卡 `/home/cat/ds_innovation`）：

```bash
tool/demo/view_stop.sh
tool/demo/frame_close.sh          # rc=1 无害
tool/demo/frame_open.sh
tool/demo/camera.sh --offset-deg 5
export RESCUE_DEMO_PROFILE=$PWD/runtime/config/demo_20261008.assume.json
tool/demo/carry_once.sh --run --site-clear --assume-all-safe --assume-injured-trip --seconds 60
```

底层主程序参数：

```
rescue_upper_host --demo-mode carry_once --demo-seconds 60 --no-show --camera 0 --width 1280 --height 720 --fps 30
  --startup-advance-ms 0 --startup-speed .10 --scan-wz .25 --turn-wz .25 --match-seconds 60 --telemetry --imu ...
  --team blue --hardware --auto-run --controlled-empty-field --assume-all-safe --assume-injured-trip
  --allow-mechanical-pitch-model --pitch-presets 500,4000,4000 --known-frame-angle 0 --known-frame-id 2
```

日志务必写磁盘（`tool/demo/logs/runNN/`），板卡 `/tmp` 是tmpfs，断电即丢。抽帧需先启动8080网页桥（`web.sh`），重启后不会自动运行。

### 关键决策代码（`src/push_task.cpp`）

- 普通物资与伤员趟只差参数：`injured_half_x_m=.15` vs `supply_half_x_m=-.15`，`injured_hold_center_y_m=.14` vs `hold_center_y_m=.12`，`zone_injured_count` vs `zone_supply_count`。
- 首次后退目标：`assume_prepush_back_m(.20) + min(assume_lateral_back_max_m(.25), 3*(|rear_x-halfX|-.03))`，以 `prepush_hold_y_ - hold_y` 视觉判断是否到位。
- 末段盲退：`travel_ >= assume_back_m`（指令距离0.50m，按 `.85*back_speed*retreat_budget` 取小）。
- 释放阈值 `hold.y >= +0.04` 因边框阻挡物理上不可达（约 -0.03…+0.01），因此加入停滞释放：

```cpp
if(!carry_progress_us_ || hold.y > carry_best_y_ + t_.assume_stall_progress_m) { carry_best_y_ = hold.y; carry_progress_us_ = now; }
if (hold.y >= -t_.assume_stall_near_m && now - carry_progress_us_ >= t_.assume_stall_us) {
    noteDropZone("release_stall", hold);
    enter(PushState::RAISE_RELEASE, now, "assumed_drop_point_stalled"); break;
}
```

参数：`assume_stall_near_m=.06`，`assume_stall_progress_m=.01`，`assume_stall_us=1500000`。
- 核验 `VERIFY_DELIVERY`：预算2.5s，需 `verify_in_>=3`（货区检测 `rear.y>=assume_cargo_in_y_m(.05)` 且 `|x-halfX|<=.10`），否则 `delivered_assumed`。
- 区域回退门控：`assume_injured_trip` 开启时 `field_previous.first_ordinary_delivered = previous.delivered_total>0`（修复第29趟）。

### 主程序与演示版代码差异

主程序为板卡 `/home/cat/ds_innovation`，演示版为 `/home/cat/ds_innovation_candidates/demo-20261007/source`。演示版由主程序演化而来，是其超集。

| 文件 | 演示版相对主程序（+/-行） | 内容 |
|---|---|---|
| src/push_task.cpp | +398/-30 | assume 决策：假设搬运、停滞释放、局部推入、盲退、核验 |
| src/main.cpp | +155/-29 | demo-mode、assume 提速参数、区域回退门控、框协议启动 |
| src/task_calibration.cpp | +67/-2 | 演示标定读取 |
| src/config.cpp | +45/-1 | `--demo-mode`、`--assume-*`、`--known-frame-*` 等参数 |
| src/uart_controller.cpp | +42/-9 | 框协议完成位、开闭 |
| include/rescue/push_task.hpp | +71/-4 | assume 调参与成员 |
| 其余头/源 | 少量 | uart、types、config、capture_monitor、telemetry、perception、geometry |
| 新增 | - | demo_policy / multi_view_capture / pixel_selector / zone_dead_reckoning / zone_visual_reference .hpp，tests/demo_mode_tests.cpp、mechanical_integration_tests.cpp、test_demo_launchers.py，tools/demo、tools/frame_semantics.py |

电脑端 `/home/liu/ds_innovation` 比板卡主程序更旧（push_task.cpp 为09-22），仅作文档副本，不可作为基线。

### 同步到主程序（20261008）

将上述演示版逻辑覆盖到板卡主程序 `/home/cat/ds_innovation`（`src include tests tools config CMakeLists.txt`，不删除文件），同步前备份：`backups_sync/main-before-demo-sync-20261008.tgz`。`--assume-all-safe`、`--assume-injured-trip`、`--demo-mode` 保持显式参数，默认关闭（`config.hpp` 默认 false/none），不带参数时主程序行为沿用比赛默认流程。在独立目录 `build-sync-20261008` 构建，二进制与第33趟部署版逐字节一致（md5 前缀 1aa47c08），ctest 21/21 通过（含新增“边框阻挡停滞释放”用例）。既有 `build/` 与 `tool/demo/runtime/bin` 未改动。上述三项参数调整未同步。
