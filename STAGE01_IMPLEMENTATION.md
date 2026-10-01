# 阶段 0 / 阶段 1：几何观测链与质量门控

本次实现对应 2026-10-01 的阶段 0、1，不包含阶段 2 的 ZoneTracker、DropPlanner、丢目标保持和交付顺序改造，也不包含阶段 3 的关键点网络。主程序仍拒绝非 dry-run 实车运行；本次没有启动或验证真实车辆运动。

## 已落实的接口链

```text
图像帧（frame_id、主机取帧时刻、原始分辨率）
  + 同时刻之前的 IMU 历史样本
  + 同时刻之前的实际 MCU pitch 反馈与稳定窗口
  + 经地面测量验收的相机标定
  → GeometryPipeline
      → 目标地面位置估计
      → 同帧关键点 KeypointFilter
      → GroundPoseFitter（二维刚体拟合，固定尺度）
      → ZoneEstimate（点数、米制残差、不确定度、新鲜度、递推预算）
      → IPPE 可用时交叉校验
      → 预计停留点转换到安全区坐标并判半区
  → PushObservation → PushTask 的质量门控
```

只有 PushTask 是当前构建/主入口的任务状态机。旧 RescueStateMachine 的删除和已有 IMU 适配在本次改动之前已经存在，本次保留并接续这些未提交修改。UART::buildMotionPacket 保留 ±0.2 m/s 硬限幅和非有限速度归零。

`zone_valid=true` 不再足以通过任务层门控：PushTask 同时检查 `zone_estimate.trusted(now_us)`。旧观测 JSON 若没有质量字段，不能再凭几个布尔值进入推送；已更新示例回放。

## 规则几何与关键点标注 v1

依据本地 `intelligence20260617.pdf` 第 37 页：内尺寸 600×300 mm，外尺寸 660×360 mm。用户已确认：面对入口，物资半区在左、伤员半区在右；红蓝两区各自使用这个入口视角。相反方向的两区不能共用图像中的左右。

正式配置：`config/zone_geometry.json`。原点为前沿**内侧地面边界**与隔板中心线交点；x 向面对入口时的右方，y 向区内后沿，z 向上，长度单位 m。矩形是内侧净区域，禁止把外侧 660×360 的角点用作内侧点。

| ID | 名称 | 安全区坐标 (x,y,z)，m |
|---|---|---|
| 0 | 前沿内侧左地面角 | (-0.300, 0.000, 0.000) |
| 1 | 前沿隔板中心线地面点 | (0.000, 0.000, 0.000) |
| 2 | 前沿内侧右地面角 | (0.300, 0.000, 0.000) |
| 3 | 后沿内侧左地面角 | (-0.300, 0.300, 0.000) |
| 4 | 后沿隔板中心线地面点 | (0.000, 0.300, 0.000) |
| 5 | 后沿内侧右地面角 | (0.300, 0.300, 0.000) |

这些点全部位于 z=0。不能标围栏顶角、板顶端或检测框角点。隔板有厚度时，ID 1/4 是其前/后沿地面接触线的中点；只有接触线可见且中点可定位时才标为 visible。遮挡点标 `visible=0`，不外推标签。标注使用原图去缩放前的像素坐标；程序负责去畸变。红/蓝 geometry_id 不相同，禁止跨安全区混合对应点。

PDF 没有明确标出隔板宽度。配置中的 `divider_exclusion_half_width_m=0.02` 是可调整的保守禁投带半宽，**不是隔板实测宽度**；现场确认时应至少覆盖隔板半宽并加余量。几何的 `confirmed=1` 表示尺寸和布局已按本次规则及用户确认固定，不表示实际相机已经标定，也不表示场地制造偏差已经实测。

## 坐标、时间与 pitch

- 地面坐标沿用已有标定工具：x 右、y 前、z 上。IMU 姿态仍使用 FLU 约定：正 yaw 是左转/逆时针。ZoneEstimate 表示 `p_body = R(theta) * p_zone + origin_body`；机器人左转时静止安全区相对车体的 theta 减小。
- 同一帧的检测/关键点必须使用同一数字 frame_id、capture_us 和原始图像尺寸。它与遥测中的坐标系名称 `camera_optical` 是不同字段。
- 在线 capture_us 当前是 OpenCV 取帧完成时的主机单调时钟，不是相机硬件曝光时刻。请求 V4L2 缓冲深度 1，但驱动可能忽略；现场还需测量 USB 相机缓冲/曝光延迟。这里实现的是明确、有上限的主机时间匹配，不能声称完成硬件同步。
- IMU 与 pitch 只选 capture_us 之前的历史样本，最大偏差 50 ms，不用推理完成时的新姿态代替旧帧姿态。帧处理年龄上限 200 ms。
- pitch 需要连续覆盖 150 ms 的反馈窗口，间隔不超过 50 ms，窗口内摆幅不超过 0.5°。未稳定、过期、无效读回、越出标定范围一律拒绝映射。
- FIXED_PITCH 模式只接受标定角容差内的反馈；PITCH_MODEL 使用已经验收的参考外参、转轴和实际 pitch 算 H，并在已验收角度范围内补偿小幅车体倾斜。接口拒绝奇异 H、非刚体外参和非法内参。
- 相机标定图像分辨率必须与采集图像完全一致，不进行隐式内参缩放。

`--pitch-feedback` 使用 MCU 串口 **O_RDONLY** 和独占接收，`sendMotion()` 在该实例上强制返回 false；伪终端测试确认没有 TX 字节。它不会自动转动相机或夹爪。需要调整 pitch 时，先停止该接收进程，再用现有控制工具调到已经标定的角度，然后重新预览；两个程序不能争用同一串口。

`--dry-run` 默认仍不打开 MCU；只有显式 `--pitch-feedback` 才打开 MCU 接收，`--imu` 独立打开 IMU 接收。2026-10-01 起非 dry-run 须显式 `--hardware`（见 STAGE_HW1_SERIAL_OUTPUT.md），其 pitch 读回来自同一读写串口，不再需要 `--pitch-feedback`。

## 过滤、拟合和质量判定

默认参数是初始工程阈值，后续阶段 4 需使用独立实拍数据调整：

- 关键点置信度 ≥0.60，ID 唯一且属于 0..5；至少一条已知基线 ≥0.10 m。
- 间距容差 0.025 m + 标准间距的 10%。投影局部灵敏度由像素扰动评估，单点米制不确定度上限 0.04 m，距离上限 3 m；地平线附近放大的不可靠投影会被拒绝。
- ≥3 个过滤后点时，对至多六个 ID 的所有最小两点样本做确定性的穷举 RANSAC。至少三内点且内点比例不低于 60%，再加权拟合 SE(2)；不拟合任意尺度或仿射变形。
- 只有两点时必须有 350 ms 内的可靠多点锚。使用 IMU 相对 yaw 检查朝向变化，并检查位置跳变；两点结果不会续期这个锚，避免无限延长低冗余定位。
- 位姿默认门槛：残差 ≤0.03 m、位置不确定度 ≤0.05 m、角度不确定度 ≤0.15 rad。ZoneEstimate 同时携带时间、帧号、来源、内点 ID 和错误原因。
- 为下一阶段预留累计绝对递推距离，默认门槛 0.30 m，预测年龄 ≤350 ms；**本阶段没有执行速度积分，也没有生成预测位姿**。
- 大跳变必须连续三帧一致才重新捕获；缺失/无效观测打断连续确认。不再由永不失效的旧位姿永久否决新观测。
- IPPE 至少需要四个非退化共面点，检查正深度、有限值与重投影 RMS。动态外参可用时与主拟合的相机系位姿交叉校验；不足四点或缺少外参时不要求 PnP。view 与隔板当前可见性不再是放行判据。

半区判定输入必须是规划器给出的**预计停止位置**，不是图像框中心。转换到 zone 后看 x 的正负，并用目标保守半径、位置和角度误差拒绝跨隔板或压边的结果。这只是计划几何判定，不代替实际交付验证。

## 编译与无硬件验证

在远程 `/home/cat/ds_innovation`：

```bash
cmake -S . -B build-arm64-telemetry -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DRESCUE_ENABLE_TELEMETRY=ON
cmake --build build-arm64-telemetry -j4
ctest --test-dir build-arm64-telemetry --output-on-failure
./build-arm64-telemetry/rescue_upper_host \
  --geometry-replay tests/fixtures/stage01_geometry.json \
  --calibration tests/fixtures/stage01-camera.synthetic.json \
  --zone-geometry config/zone_geometry.json
```

回放预期最后输出 `Geometry replay frames=3 accepted=2 hardware_output=disabled`。测试标定文件明确是合成相机，**禁止用到真实摄像头测距**。回放走真实的像素→H→过滤→拟合→PnP校验→半区判定流程；第三帧用过期 IMU 验证拒绝行为。

完整测试包括已有任务、协议、IMU 和遥测测试，新增斜看/反向视角半区一致性、错误点、像素噪声、两点锚超时、IMU 转向符号、跳变重获、同帧检查、pitch 稳定历史、只读串口无输出、质量布尔值不可绕过以及非法标定拒绝。

## 使用真实标定的预览

必须先生成并用独立地面测量验收真实的 `config/camera.yaml`。本次只发现历史内参采集结果，没有伪造或生成实车外参。

```bash
./build-arm64-telemetry/rescue_upper_host --dry-run --no-show --telemetry \
  --imu --imu-port /dev/ttyUSB0 \
  --pitch-feedback --port /dev/ttyACM0 \
  --calibration config/camera.yaml --zone-geometry config/zone_geometry.json
```

也可把这些参数附到已有 `manage_project.py start` 后面。默认无关键点来源时显示 `no_keypoints`，目标地面投影在传感器和标定有效时仍可更新。缺失标定会明确报错；不生成虚假相机数据。

阶段 3 的解码器直接调用 `GeometryPipeline::process` 传入同帧 `KeypointFrame` 即可。额外的 `--keypoints-file PATH` 仅是同帧观测适配入口：生产者必须在本帧处理前原子写入匹配元数据的 JSON；不等待上一帧网页显示结果，不自动把旧文件匹配给新图像。常规离线标注验证应使用 geometry-replay，而非用静态文件驱动实时相机。

同帧关键点 JSON 的结构参见 `tests/fixtures/stage01_geometry.json` 每个 frame 的 `frame_id/capture_us/image_width/image_height/geometry_id/zone_label/points`。可选 `expected_stop` 含 `target_id/body_m/radius_m`；本阶段不自动生成 DropPlan。

`GeometryPipeline::apply` 只更新目标米制位置、ZoneEstimate 和有输入时的半区结果。`path_safe`、外部操作安全许可、区域对齐、稳定/完整入区和交付数量仍必须来自对应真实模块，因此当前主程序保持 WAIT_START/预览并且物理输出禁用。这是阶段边界，不应通过把布尔值改为 true 绕过。

目标框底边中点仅用于地面位置预览。`ground_position_valid` 表示投影成功；`ground_contact_valid` 只由 `GeometryPipeline` 按实测证据生成（输入里的值一律清零），任务输入的 `geometry_valid` 只取这个结果；判据与拒绝原因见 STAGE_HW2_GROUND_CONTACT.md。
