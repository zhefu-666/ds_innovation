# 阶段3：检测 + 安全区 pose 双模型接入（2026-10-01）

## 模型
| 用途 | 文件（板上） | 来源 | 输出 |
|---|---|---|---|
| 目标检测 | models/detect_fp.rknn | blocks_v1/outputs_20261001_141203 (blocks_v3) | [1,8,8400]，类别 blue,orange,green,black |
| 安全区 pose | models/zone_pose_fp.rknn | zone_pose_v1/outputs_20261001_043738 (zone_pose_v2) | [1,18,8400]，zone_left/zone_right × 4 角点 |

类别映射：blue→dangerous_object（不搬运，永远不会成为推送目标），orange→injured_person，green→ordinary_supply，black→core_supply。

安全区 pose 模型的类别顺序及用途：

| 类别 ID | 模型类别 | 对应半区 | 允许投放 |
|---|---|---|---|
| 0 | `zone_left` | 左半区，物资安全区 | 普通物资、核心物资 |
| 1 | `zone_right` | 右半区，伤员安全区 | 单独运送的伤员 |

这里的左右沿用安全区坐标系：站在入口面向区内，`x<0` 为左半区，`x>0` 为右半区；不是画面中检测框的左右顺序。红色、蓝色安全区都采用相同的物资左/伤员右映射。pose 类别只表示半区几何，不证明它属于红方或蓝方；区域颜色仍需独立观测确认。

## 运行
```
./build-arm64-telemetry/rescue_upper_host --detect-image IMG \
  --model models/detect_fp.rknn --pose-model models/zone_pose_fp.rknn [--parallel-infer] [--detect-core 1 --pose-core 2]
```
实时模式同样加 `--pose-model`；带 `--calibration` 时 pose 角点直接进入 GeometryPipeline（替代 --keypoints-file），二者互斥。

## 设计
- 两个独立 RKNN 上下文（RknnModel），共用一次 640 letterbox，同一 frame_id / capture_us。
- 角点映射：0=L.near_left，1=mean(L.near_right,R.near_left)，2=R.near_right，3=L.far_left，4=mean(L.far_right,R.far_left)，5=R.far_right。隔板点需两半都可见且相距 <25% 半宽；缺失的点不补。
- 几何（PDF p.37）：标注点在围栏顶面（前沿斜坡脊线、后围栏外上沿，z=20 mm），landmark 模型 660×330 mm @ z=0.02；投放判定仍用内框 600×300。投影改为按高度的平面单应（FIXED_PITCH 只支持 z=0）。
- `--team` 选择己方颜色，实际区域身份仍需独立颜色观测；pose 模型不区分红蓝。红蓝两区都按安全区坐标系左半区为物资区、右半区为伤员区，不能直接用机器人或图像的左右代替。

## 验证
- 本地 x86 与板上 build 均无错误；ctest 本地 8/8、板上 9/9 通过。新增单测：640 letterbox、新类别、pose 解码、角点映射、隔板拒绝、高度投影。
- ONNX（onnxruntime）原始输出喂同一 C++ 解码器，4 张安全区图 6 个点全部输出，与标注偏差多为 <10 px；板上 RKNN 结果与 ONNX 相差 ≈1 px。
- 板上耗时（RK3588，librknnrt 2.3.2，含前后处理）：

| 模式 | detect | pose | 总计 |
|---|---|---|---|
| 串行 | 42 ms | 46 ms | 89–91 ms |
| 串行 + 绑核 | 42 ms | 45 ms | 89 ms |
| 并行 | 43–45 ms | 48–66 ms | 51–70 ms |

## 未完成 / 注意
- 数据里只有蓝色安全区，红区未覆盖；隔板宽度 PDF 未标注。
- 围栏顶面结论只基于一帧的放大核查，建议再抽几帧确认。
- 实时地面测距仍缺 config/camera.yaml（外参）；没有标定时 pose 只画点、不出距离。
- v3 pose / 新检测模型转好 rknn 后直接替换 models/ 下文件即可（程序会校验输入尺寸和通道数）。
- 板上源码备份：~/ds_innovation_pre_stage3_20261001.tgz；未 git commit。
