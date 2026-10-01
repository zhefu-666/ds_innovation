# 相机自动标定工具

依赖：Python 3、OpenCV、NumPy。板子已有这些依赖。本工具不会操作电机，也不会改写当前生效配置；每次输出目录必须不存在。

## 当前推荐流程

本项目使用 9×6 对称圆点板（2026-09-29 起，替换旧 10×7 平板图案）。内参通过网页分步拍摄后，推荐直接按项目根目录的《相机圆点棋盘一体化标定操作指南》执行 `calibrate.py extrinsics --pattern circles`，一次完成外参、地面单应矩阵和独立地面点验收。下面的 `ground` 模式保留作已有测量点或特殊场景的手动映射备用流程，不是圆点棋盘一体化标定的必需步骤。

## 人工准备

1. 使用 **9×6 内角点**棋盘（10×7 方格），打印后贴在平整硬板上。用尺实际量方格边长，示例假定 25 mm；不要直接相信打印比例。
2. 固定镜头焦距/焦点，选择实车分辨率。默认 640×480。标定时不能变焦；之后相机位置改变需重做地面标定。
3. 确认相机设备号，例如 /dev/video0。停止占用同一摄像头的预览/遥测服务。将车轮架空或断开动力，保持机器人静止。

## 一、自动内参标定（SSH 无窗口可用）

在板子项目根目录运行：

```bash
python3 tools/camera_calibration/calibrate.py intrinsics \
  --camera /dev/video0 --width 640 --height 480 \
  --cols 9 --rows 6 --square-mm 25 --views 25 \
  --output calibration_runs/intrinsics_01
```

人工拿着棋盘缓慢移动：依次覆盖中央、四角和边缘，改变距离并左右/上下倾斜。每次保持约两秒，完整棋盘应在画面内。不要让棋盘一直正对相机，也不要让棋盘或相机抖动。

程序自动检测、细化角点，排除过小、模糊或位置相近的画面，每次成功会打印 Accepted。默认五分钟超时，收到 25 张自动结束；不足 15 张不生成内参。若 SSH 窗口没有画面，可先确认相机视野后关闭预览，或在板子桌面终端加 `--preview`，按 Q 提前结束。

产物：
- `samples/*.png`：被选中的原图，可复查或离线重新计算。
- `intrinsics.yaml`：内参和畸变，不包含地面映射，不能单独作为完整 camera.yaml。
- `report.json`：总 RMS、每张图误差、剔除图片及覆盖警告。

RMS 尽量低于 0.5 像素，超过 1 像素应重新检查。程序只生成候选结果，低 RMS 不能证明姿态足够多样或实测精度达标。内参必须由不同倾角、位置和距离的照片求解。

离线重算（已有照片时）：

```bash
python3 tools/camera_calibration/calibrate.py intrinsics \
  --images 'calibration_runs/intrinsics_01/samples/*.png' \
  --square-mm 25 --output calibration_runs/intrinsics_02
```

## 二、地面映射（需要人工测量）

把相机装回最终位置并固定。建立地面坐标：原点为底盘中心地面投影，x 向右，y 向前，单位米。这与项目 body_xy_m 约定一致。摆放至少 6 个拟合标记点和 3 个独立检查点，覆盖实际工作范围，不能排在一条直线上。

复制 `world.example.json` 为自己的 `world.json`，用卷尺测量并填写每个标记的真实 `world_m`。示例坐标仅示意，必须按实车重测。fit 点用于拟合，check 点用于独立验收，不可复用同一批点。坐标最好精确到毫米，测量基准始终相同。

在板子拍一张所有地面点清晰可见的原图：

```bash
python3 tools/camera_calibration/calibrate.py snapshot \
  --camera /dev/video0 --width 640 --height 480 --output ground.png
```

把 ground.png、world.json 和 intrinsics.yaml 复制到有桌面的电脑，运行选点工具（也可以直接在板子桌面运行）：

```bash
python3 tools/camera_calibration/calibrate.py pick \
  --image ground.png --world world.json --output ground_points.json
```

按窗口提示的实际坐标，依次点击对应标记的中心，先 fit 后 check。U 撤销，Q 取消，全部完成后 Enter 保存。不要缩放、裁剪或矫正 ground.png 后再选点；窗口按原图坐标记录。超出屏幕时应换合适显示设备，不要改变图像分辨率。

然后自动计算并验收：

```bash
python3 tools/camera_calibration/calibrate.py ground \
  --intrinsics calibration_runs/intrinsics_01/intrinsics.yaml \
  --points ground_points.json --max-error-mm 20 \
  --camera-pitch-cdeg 2000 \
  --output calibration_runs/ground_01
```

程序先去畸变，再拟合地面单应矩阵；检查点最大误差不超过 20 mm 且至少 6 个拟合内点时才输出 `camera.yaml`。否则仅保留 `ground_report.json`，需要人工检查点位对应、测量或相机固定状况后重做。20 mm 是初调门槛，实际推行余量可能要求更严格。

## 三、文件接入与限制

输出 camera.yaml 包含 camera_matrix、dist_coeffs、ground_homography、图像尺寸和像素域标记，可由项目 CameraCalibration::load() 读取。C++ 组件已适配该标记，在像素到地面转换前先去畸变；旧无标记文件仍按旧的原图坐标变换处理。

`--camera-pitch-cdeg` 必填：拍照时相机舵机的**读回**pitch（A6反馈，0.01°，正值向下；示例2000只是占位）。单应矩阵只在这个pitch下成立，文件记录为 `ground_camera_pitch_cdeg`，C++ 运行时舵机读回偏离超过 ±0.5°（默认 `ground_pitch_tolerance_cdeg: 50`）就拒绝换算；没有该字段的旧 camera.yaml 会被拒绝加载，需重新生成或手工补写实测值。extrinsics 模式同时写入 `pitch_model_reference_cdeg`，验收通过后 C++ 端可按读回pitch由内参+外参实时重算单应；可用范围 `pitch_model_min_cdeg`/`pitch_model_max_cdeg` 默认只含参考角，须在其他pitch下做独立地面点复核后再放宽：用 `calibrate.py pitch-range --camera camera.yaml --check 1200:checks_1200.json --check 3500:checks_3500.json --output 新目录`，各pitch误差都 ≤20 mm 且相邻验证角间隔 ≤15° 才写出放宽后的 camera.yaml（不改原文件）。地面映射的倾斜可信范围与车体12°停车阈值分开：固定单应默认2°，外参模型用IMU补偿默认8°，可用 `ground_tilt_limit_deg` 覆盖；倾斜以 `imu_reference_roll_rad`/`imu_reference_pitch_rad`（标定时静止姿态，默认0）为零点。

只有在**相同原图分辨率、焦点、镜头、安装姿态**下才可以使用地面映射。当前组件不会自动检查调用者的图像尺寸，接入者必须验证。测量范围外尤其地平线附近不能盲目外推。

标定后的独立地面点应再做一次人工实测复核。目标有高度时不能把检测框中心直接映射为地面位置；需使用可信接触点或轮廓。倾斜地面、相机晃动或机器人明显俯仰时应拒绝使用平面标定。

本工具不自动启用实车运动，也不自动覆盖 config/camera.yaml。现有主程序还需要接入标定结果、传感器、安全区判断和底盘协议。

## 开发验证

```bash
python3 tools/camera_calibration/test_calibrate.py
```

测试覆盖合成棋盘检测、已知内参恢复、畸变下地面映射和独立检查失败时拒绝输出。合成测试不替代真实相机标定。

## 四、用棋盘或圆点板标定外参（extrinsics）

此模式自动识别方格棋盘内角点或对称圆点阵列，用平面 PnP 求标定板到相机的位姿，再根据你测量的标定板摆放位置，换算为机器人与相机之间的完整 4×4 变换，同时计算地面单应矩阵。相机必须已完成内参标定并固定在最终位置。标定板必须**平放、水平**，不能手持倾斜来求这一步的机器人外参。

使用当前 9×6 圆点板时，指定 `--pattern circles --cols 9 --rows 6`。圆点间距就是 `--square-mm`，应使用实际测量值。默认 `--pattern chessboard` 保持原有方格棋盘流程不变。

### 需要人工测量的参数

- `--square-mm`：实际方格边长，毫米。
- `--origin-x / --origin-y`：编号 0 **内角点**相对于底盘中心地面投影的位置，单位米。不是纸张外角，也不是棋盘中心。
- `--yaw-deg`：棋盘 +X（编号 0→1）相对于机器人 +x（向右）的角度；从向右转向前为正。0 度表示棋盘 +X 向机器人右侧，棋盘 +Y（编号 0→9，9 列时）向前。90 度表示棋盘 +X 向前、+Y 向左。
- `--board-height-mm`：印刷棋盘表面相对地面的高度，包含垫板厚度。纸贴在 4 mm 板上可按实测填 4；程序会求真实 z=0 地面映射，而不是把棋盘表面误当作地面。

机器人坐标为 x 向右、y 向前、z 向上；相机坐标为 x 图像向右、y 图像向下、z 沿光轴向前。所有变换平移单位是米。

### 先拍照，再计算候选

使用前述 snapshot 命令获取 `ground_board.png`，照片分辨率必须与内参一致。可同时在棋盘外放好独立测量点，避免拍第二张时设备移动。

以下位置、角度和板厚**仅是示例，必须换成实测值**：

```bash
python3 tools/camera_calibration/calibrate.py extrinsics \
  --intrinsics calibration_runs/intrinsics_01/intrinsics.yaml \
  --image ground_board.png --cols 9 --rows 6 --square-mm 25 \
  --origin-x -0.10 --origin-y 0.55 --yaw-deg 0 --board-height-mm 4 \
  --camera-pitch-cdeg 2000 \
  --output calibration_runs/extrinsics_candidate_01
```

圆点板示例：

```bash
python3 tools/camera_calibration/calibrate.py extrinsics \
  --pattern circles --cols 9 --rows 6 \
  --intrinsics calibration_runs/circles_01/intrinsics.yaml \
  --image ground_board.png --square-mm 实测圆心间距 \
  --origin-x -0.10 --origin-y 0.55 --yaw-deg 0 --board-height-mm 4 \
  --confirm-order --check-points extrinsics_checks.json --max-error-mm 20 \
  --camera-pitch-cdeg 2000 \
  --output calibration_runs/extrinsics_circles_candidate_01
```

输出 `corners_numbered.png`，请人工打开核对：
- 0 是你测量的基准内角点。
- 0→1 是棋盘 +X。
- 0→9 是棋盘 +Y（9 列时），两轴应符合上面的 yaw 定义。

普通棋盘没有唯一方向标识，不能默认 OpenCV 编号就是你想要的方向。如有需要，使用 `--flip-cols`、`--flip-rows` 反转列/行编号，再用新的输出目录重算并检查图片。改变基准点后，其位置必须重新测量或正确换算，不能只翻编号不检查坐标。

候选输出包括：

| 文件/字段 | 含义 |
|---|---|
| `camera_candidate.yaml` | 内参、畸变、地面 H、坐标变换；未通过现场独立验证，不能直接部署 |
| `extrinsics.json` | 标定板类型、参数、角点方向设置、重投影 RMS、平面 PnP 候选误差及验证状态 |
| `rvec_board_to_camera` / `tvec_board_to_camera_m` | 棋盘坐标到相机坐标的旋转向量和平移 |
| `T_camera_from_robot` | 机器人点转换到相机坐标 |
| `T_robot_from_camera` | 相机点转换到机器人坐标，和前项互逆 |
| `camera_position_robot_m` | 相机光心在机器人坐标中的 x/y/z |

变换约定：`p_destination = T_destination_from_source @ p_source`，使用齐次列向量。投影误差默认须不超过 0.5 px，相机光心须在机器人地面以上，否则拒绝生成候选 YAML。

### 独立验收与生成正式文件

标定板外另测至少 3 个地面点（应分布在近远左右，不能全部挤在一起），不要拿标定板角点充当独立验收点。按照原有 pick 模式选点即可；world 文件可以只包含空 fit 和至少三个 check：

```json
{
  "fit": [],
  "check": [
    {"world_m": [-0.20, 0.40]},
    {"world_m": [0.20, 0.60]},
    {"world_m": [0.00, 1.00]}
  ]
}
```

上面的坐标同样需要实测。保存为 `extrinsics_world.json` 后，在有桌面的电脑运行：

```bash
python3 tools/camera_calibration/calibrate.py pick \
  --image ground_board.png --world extrinsics_world.json \
  --output extrinsics_checks.json
```

核对角点方向后，用与候选完全一致的棋盘参数（包含任何翻转选项）重新运行，并添加：

```bash
python3 tools/camera_calibration/calibrate.py extrinsics \
  --intrinsics calibration_runs/intrinsics_01/intrinsics.yaml \
  --image ground_board.png --cols 9 --rows 6 --square-mm 25 \
  --origin-x -0.10 --origin-y 0.55 --yaw-deg 0 --board-height-mm 4 \
  --confirm-order --check-points extrinsics_checks.json --max-error-mm 20 \
  --camera-pitch-cdeg 2000 \
  --output calibration_runs/extrinsics_validated_01
```

只有角点方向已确认、位姿检查通过、独立地面点最大误差不超过阈值时，才输出 `camera.yaml`。否则不输出正式文件；若只是缺少确认或检查点，则只输出候选并提示待验收。即使方向或测量填写错误，棋盘重投影误差仍可能很小，所以不能省略独立实测。

验收后，可备份原配置再将生成的 `camera.yaml` 放入项目 `config/camera.yaml`。无需手抄矩阵；现有 C++ 标定组件读取其中 K/D/H 并先去畸变。完整三维变换保存在文件中，供后续传感器/机器人坐标适配使用。主程序的实车标定接入和运动输出仍需后续工作。

本模式不支持倾斜标定板的完整已知六自由度摆放描述；需要它时应扩展 board→robot 参数，而不能假装标定板水平。平面 PnP 存在姿态歧义，报告列出候选误差，实际仍应结合测量的相机高度/朝向和独立检查点核验。
