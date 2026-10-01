# 阶段 HW2：相机外参与地面接触（ground_contact_valid）

日期：2026-10-01。改动前备份：`~/ds_innovation_pre_hw2_20261001.tgz`。未提交 git。

## 1. 结论

- `ground_contact_valid` 现在由 `GeometryPipeline::process()` 生成。输入检测里带的值一律清零，`PushObservation.geometry_valid/distance_m/heading_error` 只取这个结果。
- 判据需要已验收的 PITCH_MODEL 外参：把规则 p.38 的实物尺寸按当前 pitch 与 IMU 投影，再与检测框比对。只有 FIXED_PITCH 单应的标定没有外参，结果一律是 `no_extrinsic_model`。
- `calibrate.py` 新增 `pitch-range` 子命令：在 1200/2200/3500 等实测 pitch 下用独立地面点复核后，才放宽 `pitch_model_min/max_cdeg`。
- `pitch_ctl.py` / `gripper_ctl.py` 的限幅改为 ±3500，与舵机实际限位及 `types.hpp` 一致。
- **实车仍然得不到接触证据**：下位机 pitch 读回卡在 25 cdeg（相机实际已到 12°），运行时每帧都是 `pitch_not_calibrated`。上位机不伪造读回，需修下位机固件。

## 2. 判据与拒绝原因

先做整帧检查，原因与安全区几何共用，按顺序：`stale_frame`、`invalid_calibration`、`calibration_image_size_mismatch`、`imu_not_synchronized`、`pitch_not_synchronized`、`pitch_moving`、`pitch_not_calibrated`、`body_tilt_untrusted`。整帧被拒时，`ground_position_valid` 和 `ground_contact_valid` 都为 false。

整帧通过后逐个检测判断，`ground_contact_reason` 为空表示通过：

| 原因 | 含义 |
|---|---|
| `frame_mismatch` | 检测与几何帧的 frame_id/时间戳不一致 |
| `box_outside_image` | 框越界或为空 |
| `not_projected` | 框底中点无法投到地面（地平线以上等） |
| `out_of_range` | y≤0 或距离 >3 m，同时清除位置 |
| `unknown_label` | 不是四类物块之一 |
| `low_confidence` | 低于 `rescue.yaml` 的 `confidence`（位置仍输出，不给接触） |
| `box_at_image_edge` | 框距图像边 <4 px，轮廓可能被截断 |
| `no_extrinsic_model` | 没有已验收的 PITCH_MODEL 外参 |
| `size_mismatch` | 框宽或框高不在实物投影范围内（±35%） |

尺寸模型取自规则 p.38：普通物资和危险物都是 40 mm 立方体；伤员 80×40×40，可平躺也可立起；核心物资是边长 40 mm 的正四面体，以任一面着地。在框底估计位置后方约 2 cm 处放置实物，每 10° 一个偏航角投影，取框宽、框高的最小与最大值。

危险物也会给出接触，仅用于避障，`targetSelectable` 从不选它。

已知局限：
- 位置用框底中点估计，落在物块近侧边缘，比中心近 2–3 cm（测试中 y=0.58 对应中心 0.60）。
- 叠放、被抬高约 15% 相机高度以上、两块相邻合并成一个框，都会因尺寸不符被拒绝。更小的抬高和轻度遮挡从单个框看不出来，所以不声称能识别。
- 判据只说明“框与贴地实物一致”，不能代替 ToF 等独立证据。`path_safe`、`safety_ok`、`retreat_safe` 仍然没有生产者。

遥测里每个检测新增 `ground_contact_reason` 字段，可以直接看拒绝原因。

## 3. 测试

在上位机 `build-arm64-telemetry` 中构建，退出码 0，`ctest` 9/9 通过。`rescue_stage01_tests` 新增以下用例：
- 四类物块正例；
- 输入自带 contact 被忽略；
- 未知类别、低置信度、贴边、两块合并、上下叠放；
- pitch 超出验证范围、pitch 移动中、IMU 无效、帧不匹配、超距；
- FIXED_PITCH 标定没有接触；
- 非法配置抛异常；
- 全视场扫描：x 从 −1 到 1 m、y 从 0.35 到 2.5 m、任意偏航，三种实物体共 1000 多个框，误拒为 0。

`tools/camera_calibration`：`python3 -m unittest test_calibrate test_guided` 15/15 通过。新用例按真实相机姿态生成 12° 和 35° 的检查点，验证放宽到 1200..3500；同时验证测量错 5 cm 被拒、验证角间隔过大被拒，两种情况都不写出 camera.yaml。

## 3.5 过渡配置：固件只有三档（2026-10-01）

下位机固件目前只支持 −25°/0°/+25° 三档（电控还没改完），1200/2200/3500 都到不了。上位机先这样适配：

- 新参数 `--pitch-presets FAR,TRACK,NEAR`（cdeg），默认 `0,2500,2500`。要求 −3500 ≤ FAR ≤ TRACK ≤ NEAR ≤ 3500。PushTask 和 CaptureMonitor 的夹爪区域视角都用这个 NEAR。
- `TaskTuning` 里的 1200/2200/3500 保留不动，供仿真测试和 push replay 夹具使用（该 ctest 显式传入 `--pitch-presets 1200,2200,3500`）。
- 新增仿真用例：0/2500 两档能完整跑完一趟，并且只下发 0 和 2500。
- `pitch_ctl.py` / `gripper_ctl.py` 限幅暂时改为 ±2500。目标不是 −25/0/25 时给出提示。如果读回刚好等于目标的度数（例如目标 2500，读回 25），会提示“可能按度上传”。
- `CaptureMonitor` 的夹爪区域像素还是 1280×720 的占位值，需要在 NEAR=2500 下重新标定。之前的记录是“25° 看不全闭合框”，这一条要在实车上重新确认。

在这个阶段，标定改为参考角 2500、验证角 0（两档相差 2500，需要放宽间隔限制）：

```bash
python3 tools/camera_calibration/calibrate.py pitch-range --camera calibration_runs/extr_2500/camera.yaml --check 0:checks_0.json --max-gap-cdeg 2500 --output calibration_runs/pitch_range_3preset
```

前提不变：读回必须是真实角度，单位为 cdeg（+25° 应为 `C4 09`）。固件支持任意角度后，去掉 `--pitch-presets` 或改回 `1200,2200,3500`，并按第 4 节补做中间角度。

## 4. 标定操作（手动）

前提：
- 车轮离地，或底盘断电保持静止；
- 相机焦距、分辨率（1280×720）不变；
- 拍照时舵机读回已稳定，**并且读回值等于真实角度**。
当前固件读回错误，所以第 4.3 步之前必须先修固件。在那之前拍的照片只能作为候选，不能部署。

### 4.1 内参

镜头和焦距没有变时，可以沿用 `calibration_runs/circles_20260927_222220_c0d353/intrinsics.yaml`（10×7 旧板，RMS 0.22 px，无警告）。改过焦距就用 9×6 圆点板重拍：`python3 tools/camera_calibration/guided.py --stop-preview`，详见 `tools/camera_calibration/GUIDED_CIRCLES.md`。

### 4.2 参考 pitch 外参（建议 2200）

1. 把 9×6 圆点板平放在车前地面上，量取：
   - 圆心间距 `square-mm`：用卷尺量第 1 到第 9 个圆心，除以 8；备用 PDF 是 12 mm，但必须实测；
   - 第 0 个圆点的 `origin-x/y`（m）；
   - `yaw-deg`；
   - `board-height-mm`（板厚）。
   另放 ≥3 个不共线的独立地面检查标记，量出各自坐标。
2. 把相机转到 2200，等读回稳定后记录读回值，然后在上位机拍照：
   ```bash
   python3 tools/camera_calibration/calibrate.py snapshot --camera /dev/video0 --width 1280 --height 720 --output calibration_runs/ground_2200.png
   ```
3. 把图片拷到本地电脑（上位机没有显示器），在本地运行选点，再把结果拷回上位机：
   ```bash
   python3 tools/camera_calibration/calibrate.py pick --image ground_2200.png --world world_2200.json --output checks_2200.json
   ```
4. 计算外参：
   ```bash
   python3 tools/camera_calibration/calibrate.py extrinsics --pattern circles --cols 9 --rows 6 --intrinsics <内参yaml> --image calibration_runs/ground_2200.png --square-mm <实测> --origin-x <实测> --origin-y <实测> --yaw-deg <实测> --board-height-mm <实测> --confirm-order --check-points checks_2200.json --camera-pitch-cdeg <读回值> --output calibration_runs/extr_2200
   ```
   检查 `corners_numbered.png` 的编号方向，并确认 `extrinsics.json` 里 `validated: true`、检查点误差 ≤20 mm。

### 4.3 放宽 pitch 范围（PushTask 需要 1200..3500）

车和检查标记都不动，或重新布置并实测。相机依次转到 1200 和 3500，每个角度拍一张照，分别用 pick 选出 ≥3 个检查点，然后运行：

```bash
python3 tools/camera_calibration/calibrate.py pitch-range --camera calibration_runs/extr_2200/camera.yaml --check 1200:checks_1200.json --check 3500:checks_3500.json --output calibration_runs/pitch_range_01
```

`pitch-range` 用与 C++ 相同的舵机模型（绕相机 x 轴转动）预测每个角度下的地面点。全部误差 ≤20 mm、相邻验证角间隔 ≤1500 cdeg 时，才在新目录写出放宽后的 `camera.yaml`。如果误差随角度增大，说明转轴不过光心：量出转轴在参考相机坐标系中的位置，用 `--pivot-camera-m x y z` 指定后重做。

### 4.4 部署与复核

```bash
cp config/camera.yaml config/camera.yaml.backup 2>/dev/null || true
cp calibration_runs/pitch_range_01/camera.yaml config/camera.yaml
```

部署后在已知位置放一块实物，在遥测里核对三项：`ground_contact_reason` 为空、`body_xy_m` 误差在 3 cm 以内、拒绝原因与实际情况相符。

## 5. 后续

1. 修下位机固件：pitch 读回要反映实际角度，OPEN 完成要报 done=1。修好后重测夹爪和 pitch，再做第 4 节的标定。
2. 第 3 步：感知证据（`path_safe`、`safety_ok`、`retreat_safe`、`opponent_zone_clear`、`captured` 等）的生产者。
