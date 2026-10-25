# 视觉接口与实车接入状态

## 模型契约

默认模型为 benchmark_results/best_fp16.rknn，运行库为 benchmark_results/librknnrt.so。模型 SHA256：3fccc42d12e0afdff550343e1e259034fb063309d8c678749a11822a1aa94687。路径相对于项目工作目录。

- 输入原图为 BGR uint8 三通道。等比例缩放到 448×448，居中补 114 灰边，转 RGB。
- RKNN 输入：NHWC uint8 0–255，pass_through=0；模型编译时包含 /255 归一化，不能重复除以 255。
- ONNX 输入：RGB NCHW float32 0–1。两条后端共用类别、去重和坐标还原代码，但不保证当前 OpenCV 可以加载该 ONNX 图。
- 输出：单个 float32 [1,11,4116] 张量（RKNN 底层 FP16 由 Runtime 转 float32）；前四通道是输入像素域 cx/cy/w/h，后七通道是分类分数，没有额外 objectness 通道。
- 分类别 NMS，框恢复到原图像素坐标并裁剪边界；无效数值和无面积框丢弃。张量/输入规格不匹配时直接报错。
- 当前模型只输出框，不输出实例掩码。--require-masks 会报错，不能伪造掩码。
- OpenCV 双线性缩放与历史 Python/Pillow 缩放存在数值差异。验证新后端时使用相同预处理比较，不声称与旧报告逐像素相同。

| 原始 ID | 模型类别 | 推行接口 label |
|---|---|---|
| 0 | core | core_supply |
| 1 | wounded | injured_person |
| 2 | red | red_safe_zone |
| 3 | dangerous | dangerous_object |
| 4 | normal | ordinary_supply |
| 5 | main | unmapped_main（含义待确认，不能成为推行目标） |
| 6 | blue | blue_safe_zone |

SegDetection 保留 class_id 和 model_label，另提供映射后的 label、confidence、原图 box、timestamp_us；跟踪器追加 track_id。makePushObservation 仅传递新鲜有效的任务候选身份，几何/路径/安全/交付证据继续保持无效，直至相应模块接入。当前候选适配是预览阶段，不是多目标批次锁定或实车决策器。

## 当前可验证能力

1. ARM64 C++ 原生构建；检测后端动态加载 RKNN，ONNX 共用后处理接口。
2. 单图 NPU 推理与相机检测预览；预览可录像，推行目标名称与模型分类对齐。
3. 目标关联跟踪组件；检测身份可进入推行观测接口。
4. 七状态推行规则：首单一件普通物资、后续批次限制、目标丢失/过期停车、合法分区与数量检查、稳定交付和脱离确认。
5. 不依赖模型或硬件的完整交付回放，以及核心、推行和视觉单元测试。
6. 标定、简单候选路径规划、安全监督、串口包构造等已有可复用组件，但它们尚未组成正式实车运行链路。

## 距离完整实车调试的缺口

### 优先级一：感知与决策闭环

- 确认 main 类语义；确认训练标签、置信度阈值和实际场地的一致性。已有训练验证集存在同源视频泄漏的历史记录，需要独立实拍视频验证泛化。
- 标定相机内参、畸变、安装姿态与地面变换，获得米制目标位置及推板接触区域；现有框中心不能代替地面接触点。
- 实现安全区围栏、隔板和合法物资/伤员半区检测；目标轮廓完全进入、脱离围栏、静止及机器人脱离都需真实证据。
- 接入障碍与危险区、路线及推行方向判断；完善固定目标身份与多物体批次成员管理。当前只取高置信度候选不能用于锁定后的目标切换。

### 优先级二：硬件输出与独立停车

- 核对 STM32 协议、底盘结构、米/秒与电机量的转换、角速度方向和速度上限。旧电机包不能直接替代新 MotionCommand。
- 接入 IMU/测距/编码器数据、时间戳和通信健康信息，完善独立安全监督。
- 下位机实现/确认命令超时停车；上位机阻塞、摄像头断开、NPU 异常或进程退出都必须能停。
- 新主入口保持串口关闭和运动输出禁用，直到上述适配器可测试。参数调低不能替代这一步。

### 优先级三：分阶段实机验收

先验证连续视频检测和映射，再验证标定与安全区几何；随后架空车轮检查协议、方向、急停和断线停车；最后低速单件普通物资、核心/伤员分区、多件批次、遮挡与失败恢复。需要持续运行时延/温度测试。当前测试通过不代表这些实车步骤已完成。

## 命令

```bash
cd /home/cat/ds_innovation
cmake -S . -B build-arm64 -DCMAKE_BUILD_TYPE=Debug
cmake --build build-arm64 -j4
ctest --test-dir build-arm64 --output-on-failure
./build-arm64/rescue_upper_host --detect-image benchmark_results/npu/sample.jpg --conf 0.25
./build-arm64/rescue_upper_host --dry-run --camera 0
./build-arm64/rescue_upper_host --push-replay tests/fixtures/push_delivery.json
```

这些命令均不发送实车运动指令。相机编号须按实际设备选择。

## 本次验证结果（2026-09-21）

- 本机 x86-64 与板子 ARM64 均构建成功，CTest 4/4 通过：核心组件、推行规则、完整交付回放、视觉接口。
- 板子样例图实际 RKNN 推理：id=4，raw=normal，task=ordinary_supply，confidence=0.749512，原图框 x=246,y=165,width=158,height=164。
- 在相同 OpenCV 预处理下，与 Python RKNN 基准实现对照：样例图 1 个目标、黑图 0 个目标、321×257 缩放图 1 个目标；类别与整数框一致，置信度差小于 1e-5。
- 未在本轮运行连续相机、实车底盘或真实安全区投放测试。三张输入一致性检查不是全数据集精度评估。
