# 智能救援遥测可视化 AI Agent 构建规格

## 0. 目标

在 `/home/liu/ds_innovation` C++17 工程中新增一个**单向遥测发布模块**，把相机图像、检测结果、状态机状态和传感器数据实时传到本机笔记本，用 Lichtblick 查看，用于：

```text
现场 3 分钟调试期内快速定位视觉/状态机问题
比赛过程中给裁判和队友展示机器人内部状态
赛后完整回放复盘
```

本模块**只做可观测性**。它不参与控制，不接收任何来自笔记本的指令，不改变状态机的任何行为。移除本模块后，上位机的运动逻辑必须完全不变。

## 1. 不可违反的规则

来源：《2027 年中国大学生工创实践与创新能力大赛 智能+工程创新赛道 竞赛命题解析》第 3 页。

> 允许与笔记本电脑进行通讯，比赛中**不能触碰**笔记本电脑，**不允许任何形式的遥控**。

由此推出四条硬性设计约束，任何一条被破坏都必须视为需求失败：

1. **单向**。只从机器人推数据出去，绝不接受任何输入。实现层面必须让反向通道不存在，而不是靠"我们不去点它"。
2. **旁路**。发布链路的任何阻塞、丢包、断开都不得影响视觉推理、状态机和 UART 控制。链路断开时机器人必须照常自主运行。
3. **不自证违规**。不得把感知或决策卸载到笔记本。机器人必须自主完成救援任务，且供电必须来自车上电池。
4. **可审计**。必须能向裁判一句话说明并当场展示代码：发布端只调用 `log()`，没有任何订阅/回调/服务注册。

### 1.1 闭环反向通道的具体关法

`foxglove-sdk` 的 `WebSocketServerCapabilities` 是这套约束的天然开关，默认值就是 `None`：

```cpp
enum class WebSocketServerCapabilities : uint8_t {
  None = 0,
  ClientPublish     = 1 << 0,  // 允许客户端向服务端发数据 —— 禁止
  ConnectionGraph   = 1 << 1,
  Parameters        = 1 << 2,  // 允许客户端读写参数 —— 禁止
  Time              = 1 << 3,
  Services          = 1 << 4,  // 允许客户端调用服务 —— 禁止
  Assets            = 1 << 5,
  PlaybackControl   = 1 << 6,  // 允许客户端控制播放 —— 禁止
};
```

`WebSocketServerOptions::capabilities` 保持默认 `None`。**不要注册**以下任何一项（名字均以 `cpp/foxglove/include/foxglove/websocket.hpp` 实际定义为准）：

```text
WebSocketServerOptions::callbacks.onPlaybackControlRequest
WebSocketServerOptions::callbacks.onClientAdvertise        // 需 ClientPublish
WebSocketServerOptions::callbacks.onMessageData            // 需 ClientPublish，客户端发来的数据
WebSocketServerOptions::callbacks.onClientUnadvertise      // 需 ClientPublish
WebSocketServerOptions::callbacks.onGetParameters          // 需 Parameters
WebSocketServerOptions::callbacks.onSetParameters          // 需 Parameters
WebSocketServerOptions::callbacks.onParametersSubscribe
WebSocketServerOptions::callbacks.onParametersUnsubscribe
WebSocketServerOptions::parameter_handler                 // 需 Parameters
WebSocketServerOptions::fetch_asset                       // 注册后会自动宣告 Assets 能力
```

`onSubscribe` / `onUnsubscribe` 是可选的只读观测回调，仅在需要统计客户端连接数时使用，不影响单向性。

本规格中不需要出现任何参数回写路径。

如果后续为了 3D 面板的 URDF 必须开 `Assets`，只开这一个，并在代码注释里写明原因。其他能力一律不开。

## 2. 硬件与网络假设

- 上位机运行在 RK3588S（LubanCat-4）。若实际板子不同，本规格第 7 节的带宽算法不变，只需替换编码能力判断。
- 相机 720p USB 单目，沿用现有 `CameraCapture`。
- 板子与笔记本通过 WiFi 或交换机连接，同一网段。笔记本实测 IP 与板子互通。
- 笔记本运行 Lichtblick（桌面版或 Docker），不使用 Foxglove 闭源版。
- **假设链路会断**。场地内有两台机器人同场对抗，不允许使用电子干扰设备，但 2.4G 拥塞、遮挡、对手链路都会造成丢包。所有设计必须按"链路随时可能消失"来写。

## 3. 现有代码接入点

本工程已经具备全部数据源，不要重复实现：

| 数据源 | 位置 | 可用内容 |
|---|---|---|
| `CameraCapture` | `include/rescue/camera_capture.hpp` | `latest(CameraFrame&)` 给出 `cv::Mat` + `timestamp_us` + `frame_number`，已含独立采集线程与最新帧策略，另有 `droppedFrames()` |
| `IDetector` | `include/rescue/yolo_detector.hpp` | 抽象基类，返回 `std::vector<SegDetection>` |
| `makeDetector()` | `include/rescue/detector.hpp` | `std::unique_ptr<IDetector> makeDetector(const Config&)`；`YoloRknnDetector` 亦定义于此 |
| `NearestNeighborTracker` | `include/rescue/tracker.hpp` | 填充 `SegDetection::track_id` |
| `RescueStateMachine` | `include/rescue/rescue_state_machine.hpp` | `RescueState` 枚举（14 态），见下 |
| `SensorFusion` | `include/rescue/sensor_fusion.hpp` | `SensorState` |
| `ZoneLayout` | `include/rescue/zone_layout.hpp` | 物资区/伤员区语义（struct） |
| `SafeZonePoseEstimator` | `include/rescue/safe_zone_pose.hpp` | PnP 位姿与重投影误差 |
| `LocalPlanner` | `include/rescue/planner.hpp` | 接近点与候选路线 |
| `EventLogger` | `include/rescue/logger.hpp` | `event()` / `sensor()` / `command()`，已有 `timestamp_us` 口径 |

`RescueState` 完整枚举，命名与 `AI_AGENT_BUILD_SPEC.md` 第 5.9 节一致，不要另造：

```text
WAIT_START  DEPART  SEARCH_TARGET  SELECT_TARGET  APPROACH_TARGET
ALIGN_PUSH  PUSH_TARGET  NAVIGATE_DROP_ZONE  IDENTIFY_DROP_SUBZONE
ALIGN_DROP  VERIFY_DELIVERY  RETURN_SEARCH  RECOVERY_SEARCH  EMERGENCY_STOP
```

数据结构定义在 `include/rescue/types.hpp`：

```cpp
struct SegDetection {
    int track_id; std::string label; float confidence;
    cv::Rect box; cv::Mat mask;
    cv::Point2f ground_point_px, body_xy_m;
    int class_id; uint64_t timestamp_us;
};

struct SensorState {
    uint64_t timestamp_us;
    float yaw_rad, pitch_rad, roll_rad;
    float tof_fl_m, tof_fr_m, tof_rl_m, tof_rr_m;
    bool imu_valid; std::array<bool,4> tof_valid; bool encoder_valid;
};

struct MotionCommand {
    uint8_t header = 0x56;
    float vx_mps = 0.0f, wz_rps = 0.0f;
};
```

状态名沿用 `AI_AGENT_BUILD_SPEC.md` 第 5.9 节的既有命名，不要另造一套。

**注意**：`USAGE_GUIDE.md` 第 1 节已声明新模块尚未全部接入 `main.cpp` 旧版循环。本模块接入的是**新状态机**，不要挂到旧循环上。

## 4. 依赖与构建

新增依赖：`foxglove/foxglove-sdk`（MIT，C++ 部分在 `cpp/`，是对 `c/` 库的上层封装）。

关键事实：

- C++ SDK 是 C 库的 wrapper，**必须先构建并链接 `c/` 库**，并把生成的 includes 加入头文件搜索路径（见 `cpp/README.md`）。
- 依赖默认从本机包管理器查找，找不到时经 FetchContent 从源码构建，由 `USE_PACKAGE_MANAGER_DEPENDENCIES` 控制（默认 `ON`）。在 RK3588S 上源码构建 nlohmann_json / Catch2 很慢且吃内存，**建议在本机或 CI 上交叉编译好再拷到板子**。
- 头文件内嵌了一份 tl-expected 的 `expected.hpp`。
- 官方示例 `cpp/examples/rgb-camera-visualization/main.cpp`（146 行）是最小可用模板，但**它用的是 `RawImageChannel`，按第 7 节必须换掉**。

CMake 方面新增可执行目标或静态库用于发布模块，并保持 `-DBUILD_TESTING=ON` 下现有 `ctest` 全部通过。发布模块必须能在**不链接 SDK** 时通过编译开关整体关闭：

```cmake
option(RESCUE_ENABLE_TELEMETRY "Build telemetry publisher" OFF)
```

默认 `OFF`，比赛安全版本可完全不带该模块构建。

## 5. 通道映射

同一份数据同时进网络和进磁盘，靠 SDK 的 sink 机制实现：`Context` 关联日志与 sink，`WebSocketServer` 和 `McapWriter` 都是 sink，而 `log()` 的 `sink_id` 省略时**消息会发往所有 sink**。因此只需创建一次通道，网络与录制自动同时生效。

| 通道 topic | 消息类型 | 数据源 | 客户端面板 |
|---|---|---|---|
| `/camera/image` | `CompressedImage` | `CameraCapture::latest()` 缩放后 JPEG | Image |
| `/detections` | `ImageAnnotations` | `IDetector` + `Tracker` | 叠加在 Image 上 |
| `/fsm/state` | `KeyValuePair` | `RescueStateMachine` | Plot / Raw Messages |
| `/fsm/events` | `Log` | 状态转移事件 | Log |
| `/sensors/imu` | `KeyValuePair` | `SensorState` yaw/pitch/roll | Plot |
| `/sensors/tof` | `KeyValuePair` | `SensorState` 四路 ToF | Plot |
| `/cmd/motion` | `KeyValuePair` | `MotionCommand` | Plot |
| `/safezone/pose` | `FrameTransform` | `SafeZonePose` | 3D |
| `/camera/calibration` | `CameraCalibration` | 标定文件 | Image 去畸变 |

统一 frame_id 约定，避免客户端坐标错乱：

```text
camera_optical   相机光心，+x 右 +y 下 +z 入平面（CompressedImage 规定）
base_link        车体
field            场地
zone             安全区局部
```

`ImageAnnotations` / `KeyValuePair` / `FrameTransform` / `CameraCalibration` 的具体结构体字段以 SDK 头文件为准，实施时先读 `cpp/foxglove/include/foxglove/messages.hpp`，**不要凭记忆写字段名**。

## 6. 发布模块设计

新增 `include/rescue/telemetry_publisher.hpp` 与 `src/telemetry_publisher.cpp`。

```cpp
namespace rescue {

struct TelemetryOptions {
    std::string host = "0.0.0.0";      // 必须 0.0.0.0，见下
    uint16_t port = 8765;
    std::string mcap_path;             // 空字符串则不录制
    int image_publish_fps = 10;        // 与视觉帧率解耦
    int image_width = 640;
    int image_height = 480;
    int jpeg_quality = 70;
    bool publish_images = true;
};

class TelemetryPublisher {
public:
    static std::unique_ptr<TelemetryPublisher> create(const TelemetryOptions& options);
    ~TelemetryPublisher();

    // 全部非阻塞，内部只做入队/覆盖，绝不等待网络
    void publishFrame(const cv::Mat& bgr, uint64_t timestamp_us);
    void publishDetections(const std::vector<SegDetection>& detections, uint64_t timestamp_us);
    void publishState(const std::string& state_name, uint64_t timestamp_us);
    void publishEvent(const std::string& state, const std::string& message, uint64_t timestamp_us);
    void publishSensors(const SensorState& sensors);
    void publishMotion(const MotionCommand& command, uint64_t timestamp_us);
    void publishSafeZone(const SafeZonePose& pose, uint64_t timestamp_us);
};

} // namespace rescue
```

实现要求：

- **独立发布线程**。所有 `publish*()` 只把数据放进内部缓冲并返回，真正的编码、序列化和 `log()` 调用在发布线程里做。主循环调用 `publish*()` 的耗时必须是常数级且极小。
- **图像最新帧覆盖**。缓冲只保留最新一帧，发布线程取走即清空；来不及发的帧直接丢弃并累加一个计数，通过 `/fsm/state` 暴露 `dropped_publish_frames`，便于判断是不是网络拖累了链路。
- **结构化数据不丢**。检测、状态、传感器是低带宽高价值数据，允许排队但必须限长（建议 256 条上限），超出丢最旧。
- **编码在发布线程做**。`cv::resize` + `cv::imencode(".jpg", ...)` 放在发布线程，不要在主循环做。
- **帧率解耦**。视觉推理按 `config/rescue.yaml` 的 `detect_interval` 照常跑，图像只按 `image_publish_fps` 往外发。两者互不影响。
- **异常不外抛**。发布线程内任何异常都只记录并继续，绝不终止进程、绝不影响控制。
- `host` 必须设成 `0.0.0.0`。官方示例里写的是 `127.0.0.1`，那样笔记本连不上。

### 6.1 发布线程伪代码

```text
loop:
    if stop_requested: break
    sleep_until(next_tick)                     # 固定 image_publish_fps 节拍
    if images_enabled:
        frame = grab_latest_frame()            # 无新帧则跳过
        if frame:
            small = resize(frame, 640x480)
            ok, buf = imencode(".jpg", small, quality=70)
            if ok:
                msg.data = as_bytes(buf); msg.format = "jpeg"
                msg.timestamp = to_timestamp(frame.timestamp_us)
                msg.frame_id = "camera_optical"
                image_channel.log(msg)
    drain_structured_queues()                  # 检测/状态/传感器/运动
```

## 7. 带宽与性能要求

**这是本方案最容易失败的地方，不要跳过。**

`RawImage` 是未压缩原始帧。720p BGR8 单帧字节数：

```text
1280 × 720 × 3 = 2,764,800 B ≈ 2.64 MiB
```

| 方案 | 单帧/单秒带宽 | 弱板 CPU | 结论 |
|---|---:|---|---|
| `RawImage` 720p @60fps | ≈ 158 MiB/s | 低（但网络爆炸） | **禁用** |
| `RawImage` 720p @30fps | ≈ 79 MiB/s | 低 | **禁用** |
| `CompressedImage` JPEG 720p q70 @10fps | ≈ 0.8–1.5 MiB/s | 中 | 可用 |
| `CompressedImage` JPEG 640×480 q70 @10fps | ≈ 0.3–0.5 MiB/s | 低 | **推荐起点** |
| `CompressedVideo` H.264 硬编 720p @30fps | ≈ 0.25–0.5 MiB/s | 极低 | 进阶 |

要求：

1. 一律使用 `CompressedImageChannel`，**禁止** `RawImageChannel`。这是官方示例与本规格最大的差异点，实施时务必确认没有沿用示例的 `RawImageChannel`。
2. 起点参数：640×480、JPEG 质量 70、10 fps。跑通后可按实测调整。
3. 图像缩放在发布线程做，不要降低相机采集分辨率（视觉推理仍需要 720p 输入）。
4. `CompressedVideoChannel` + RK3588S 硬件编码器（rockchip mpp）作为第二阶段可选优化。**先不要做**，JPEG 路径没跑通之前不要引入 mpp 的时间戳对齐复杂度。
5. 必须实测并记录：开启/关闭遥测两种情况下的状态机帧率差异。若状态机帧率下降超过 5%，视为需求失败，回到第 6 节检查是否真的做到了非阻塞。

## 8. 录制（MCAP）

`McapWriter` 与 `WebSocketServer` 同为 sink，创建两者即可让同一批通道同时进网络和进磁盘：

```cpp
foxglove::McapWriterOptions mcap_options;
mcap_options.path = options.mcap_path;
mcap_options.compression = foxglove::McapCompression::Zstd;   // 默认
auto writer = foxglove::McapWriter::create(mcap_options).value();
```

要求：

- 录制路径带时间戳，单场一个文件，避免跨场比赛混在一起。
- 录制开启时用 `sink_channel_filter` 排除 `/camera/image`，或把图像录制帧率再降一档，避免 eMMC 写入压力。**图像不是复盘的必要条件，状态和传感器才是。**
- 退出时调用 `flush()` 与 `close()`，确保文件可读。
- MCAP 文件落地后可用本机 Lichtblick 直接打开回放，不依赖机器人。

## 9. 客户端配置（Lichtblick）

实施时给出一份可导入的 Lichtblick 布局，包含并只包含以下面板：

```text
Image          绑定 /camera/image，叠加 /detections
Plot           /fsm/state、/sensors/imu、/sensors/tof、/cmd/motion 的数值字段
Log            /fsm/events
Raw Messages   排查用
3D             /safezone/pose（可选）
```

**布局中不得包含任何发布类、Teleop 类或参数编辑面板。** 交付时明确写清这一点，避免队友误操作把遥测变成遥控。

连接地址：

```text
ws://<板子IP>:8765
```

## 10. Agent 实施顺序

### Phase 1：本机跑通 SDK

1. 在本机（非板子）克隆并构建 `foxglove/foxglove-sdk`，构建 C++ 示例。
2. 运行 `example_rgb_camera_visualization`，用 Lichtblick 连 `ws://127.0.0.1:8765` 确认能看到画面。
3. 记录完整的构建命令与依赖版本到本规格末尾的"实施记录"。
4. 此阶段不碰 `/home/liu/ds_innovation` 任何代码。

### Phase 2：压缩图像改造

1. 把示例的 `RawImageChannel` 换成 `CompressedImageChannel`，加入 resize 与 JPEG 编码。
2. 用 `iftop`、`nload` 或 SDK 自带统计实测带宽，确认落在第 7 节预期区间。
3. 确认所有反向能力保持关闭（`capabilities == None`）。

### Phase 3：接入工程

1. 新增 `telemetry_publisher.*`，实现第 6 节接口。
2. 加 `RESCUE_ENABLE_TELEMETRY` 编译开关，默认 `OFF`。
3. 接入 `CameraCapture::latest()`，只推图像通道，先不推其他数据。
4. 实测状态机帧率无退化。

### Phase 4：结构化通道

1. 依次接入 `IDetector`、`RescueStateMachine`、`SensorFusion`、`MotionCommand`。
2. 每接入一个通道就在 Lichtblick 里确认一次显示正确，不要一次全接。
3. 校对时间戳口径与 frame_id 约定。

### Phase 5：录制与交付

1. 接入 `McapWriter`，验证录出的文件能在本机打开回放。
2. 交付 Lichtblick 布局文件与连接说明。
3. 输出第 12 节要求的交付物。

## 11. 验收标准

### 合规

- `WebSocketServerOptions::capabilities` 为 `None`，或仅 `Assets` 且有注释说明。
- 代码中不存在任何客户端消息处理回调、服务注册、参数处理器。
- 交付的 Lichtblick 布局不含发布类面板。
- 断网后机器人行为与开启遥测前完全一致，可重复验证。

### 性能

- 关闭遥测与开启遥测，状态机帧率差异 < 5%。
- 720p 图像不进入网络，实测带宽符合第 7 节预期。
- 发布线程阻塞、Lichtblick 断开、WiFi 断连三种情况下，主循环帧率均无退化。

### 功能

- Lichtblick 能同时看到图像、检测框、状态曲线、传感器曲线。
- 状态转移在 Log 面板可读。
- `dropped_publish_frames` 计数可见，可判断丢帧原因。
- MCAP 文件可在本机完整回放。

### 工程

- `-DRESCUE_ENABLE_TELEMETRY=OFF` 时工程照常编译，且不引入 SDK 依赖。
- 现有 `ctest` 全部通过。
- 删除本模块后运动逻辑无任何变化。

## 12. Agent 工作规则

- 每次只修改一个可验证模块，接入一个通道就在客户端确认一次。
- 修改前先读取相关头文件和调用关系，不假设 SDK 字段名，以 `messages.hpp` 实际定义为准。
- 不删除或修改 `main.cpp` 现有控制逻辑。本模块只增不减。
- 任何 `publish*()` 都不得阻塞、不得抛异常、不得影响状态机。
- 不把遥测链路写入控制回路，任何情况下控制逻辑不得等待遥测。
- 不以"编译通过"代替实机验收，不以"能连上"代替带宽实测。
- 不实现任何反向控制能力，即使看起来方便。
- 每完成一个 Phase，运行编译、`ctest`、以及与上一 Phase 的帧率对比。
- 输出修改文件、接口变化、实测带宽数据、测试命令和未解决风险。

## 13. 实施记录

> 由实施 Agent 在 Phase 1 完成后填写，不要预先填充。

```text
SDK 版本 / commit:
本机构建命令:
C++ 示例可执行路径:
依赖版本（OpenCV / CMake / 编译器）:
板子型号与实测 IP:
Lichtblick 版本与部署方式:
实测带宽:
状态机帧率（关遥测 / 开遥测）:
```

## 14. 附：与现有工程文档的关系

- `AI_AGENT_BUILD_SPEC.md`：上位机主体功能与规则约束，本规格不重复其内容，状态名沿用其第 5.9 节。
- `README.md` / `USAGE_GUIDE.md`：现有运行方式。**注意 `README.md` 第 96 节起的旧状态机是抓取流程，与 2027 规则"不能采用机械臂抓取、不能将救援目标放在机器人上"冲突**（规则见命题解析第 42 页）。本规格的遥测应挂在新状态机上，不要挂旧循环。若旧循环最终被移除，本规格不受影响。
