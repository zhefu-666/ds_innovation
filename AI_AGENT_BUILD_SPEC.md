# 智能救援上位机 AI Agent 构建规格

## 0. 目标

在当前 `/home/liu/ds_innovation` C++17 项目基础上，构建 RK3588S 上位机程序，实现：

```text
单目视觉识别目标
→ IMU/ToF 辅助定位和安全控制
→ 目标选择与局部路径规划
→ 地面推送目标
→ PnP 定位安全区
→ 区分物资区/伤员区
→ 倒退脱离并确认有效送达
→ 返回搜索区域
```

第一阶段只要求稳定完成一个绿色普通物资的单目标闭环。

## 1. 硬件假设

- RK3588S，正式推理优先使用 RKNN/NPU；开发阶段保留 ONNX/OpenCV 后端。
- 720p/60 FPS USB 单目相机。
- 1 个底盘 IMU，至少提供 yaw、pitch、roll 和时间戳。
- 4 个 ToF：左前、右前、左后、右后。
- 下位机负责电机闭环和实时急停。
- 若有编码器，作为短程里程计接入；没有编码器时使用区域状态和短动作复定位。

## 2. 不可违反的规则

- 普通物资、核心物资、伤员是公共目标。
- 必须先有效送达至少一个普通物资，才允许转运核心物资和伤员。
- 禁止机械臂抓取，禁止把目标放在机器人上。
- 伤员单独转运，一次一个；第一版所有目标一次一个。
- 危险目标不得推动、出场或进入任一安全区。
- 不得进入对方安全区。
- 物资区和伤员区颜色相近，不能靠颜色单独区分。

## 3. 目标类别

```text
ordinary_supply   绿色普通物资
core_supply       黑色核心物资
injured_person    橘色伤员
dangerous_object  浅蓝危险物
red_safe_zone     红色安全区整体
blue_safe_zone    蓝色安全区整体
robot             对方机器人，可选
```

正式部署模型流程：

```text
best.pt → best.onnx → best.rknn
```

模型接口必须抽象，使 ONNX 和 RKNN 可替换：

```cpp
class IDetector {
public:
    virtual std::vector<SegDetection> infer(const cv::Mat& frame) = 0;
    virtual ~IDetector() = default;
};
```

目标紧挨时必须支持实例 mask；如果第一阶段暂时使用检测框，必须标记为实验功能，不能作为比赛安全版本。

## 4. 核心数据结构

```cpp
struct SensorState {
    uint64_t timestamp_us;
    float yaw_rad, pitch_rad, roll_rad;
    float tof_fl_m, tof_fr_m, tof_rl_m, tof_rr_m;
    bool imu_valid;
    std::array<bool,4> tof_valid;
    bool encoder_valid;
};

struct SegDetection {
    int track_id = -1;
    std::string label;
    float confidence = 0.0f;
    cv::Rect box;
    cv::Mat mask;
    cv::Point2f ground_point_px;
    cv::Point2f body_xy_m;
};

struct SafeZonePose {
    bool valid = false;
    std::string label;
    cv::Mat rvec, tvec;
    float reprojection_error_px = 999.0f;
    float distance_m = 0.0f;
    float heading_error_deg = 0.0f;
    enum View { FRONT, OBLIQUE, SIDE, PARTIAL, UNKNOWN } view = UNKNOWN;
};

struct MotionCommand {
    uint8_t header = 0x56;
    float vx_mps = 0.0f, wz_rps = 0.0f;
};
```

所有视觉和传感器数据必须带时间戳。过期数据不得参与运动控制。

## 5. 模块拆分

### 5.1 `camera_capture.*`

- 打开 1280×720/60 FPS 相机。
- 独立采集线程，使用最新帧策略，不能阻塞控制线程。
- 记录帧时间戳和丢帧计数。

### 5.2 `detector.*`

- 保留现有 `YoloOnnxDetector` 作为开发后端。
- 新增 `YoloRknnDetector` 接口占位或实现。
- 统一输出 `SegDetection`。
- 支持检测间隔 15～25 Hz，跟踪 30～60 Hz。

### 5.3 `tracker.*`

- 使用 ByteTrack、SORT 或轻量最近邻跟踪。
- 目标类别变化、遮挡、重新出现时不得随意复用 track_id。
- 跟踪丢失超过阈值进入停车复核。

### 5.4 `camera_calibration.*`

- 读取相机内参和畸变参数。
- 读取地面单应矩阵。
- 将目标接地点转换为车体坐标。
- IMU pitch/roll 超过阈值时降低定位可信度。

### 5.5 `safe_zone_pose.*`

- 检测安全区外框角点、隔板点和可选围栏上下边缘。
- 使用 `solvePnPRansac` 求安全区位姿。
- 输出重投影误差和 FRONT/SIDE/PARTIAL 状态。
- 角点不足、误差过大或姿态跳变时返回无效。

### 5.6 `zone_layout.*`

使用配置定义安全区子区域语义，不通过图像左右猜测：

```cpp
struct ZoneLayout {
    std::string zone_label;
    enum Side { LEFT, RIGHT } supply_side;
    enum Side injured_side;
    float field_heading_deg;
};
```

只有安全区完整且朝向已知时，才能将目标类别映射到具体子区。

### 5.7 `planner.*`

- 维护区域状态：START、CENTER、OBSERVE、DROP、UNKNOWN。
- 根据目标位置 `T`、投送点 `G` 计算接近点 `A`：

```text
A = T - d × normalize(G - T)
```

- 生成直达、左绕、右绕、换观察点候选路线。
- 检查车体、机构、目标和危险物膨胀区。
- 危险物碰撞风险是硬约束。
- 不要求第一版实现完整 SLAM。

### 5.8 `sensor_fusion.*`

- 合并视觉、IMU、ToF、编码器数据。
- IMU 用于航向保持和越障稳定判断。
- ToF 用于近场减速、停车和倒车防撞。
- ToF 触发安全停车优先级高于普通视觉运动命令。

### 5.9 `rescue_state_machine.*`

实现以下状态：

```text
WAIT_START
DEPART
SEARCH_TARGET
SELECT_TARGET
APPROACH_TARGET
ALIGN_PUSH
PUSH_TARGET
NAVIGATE_DROP_ZONE
IDENTIFY_DROP_SUBZONE
ALIGN_DROP
VERIFY_DELIVERY
RETURN_SEARCH
RECOVERY_SEARCH
EMERGENCY_STOP
```

状态机必须非阻塞；禁止使用长时间 sleep 阻塞暂停、ToF 和急停处理。

## 6. 安全区投送算法

### 6.1 识别

```text
检测安全区整体
→ 提取外框和隔板
→ PnP 解算位姿
→ 判断是否正面观察
→ 侧面/局部视角移动到正面观察点
```

侧面检测只能表示“发现安全区”，不能直接投送。

### 6.2 子区选择

```text
ordinary_supply/core_supply → supply_side
injured_person              → injured_side
dangerous_object            → reject
```

### 6.3 投送确认

```text
目标进入指定子区
→ 停止推进
→ 低速倒退 0.15～0.25 m
→ 重新 PnP 和目标定位
→ 转换到安全区坐标系
→ 比较倒退前后目标相对位置
→ 连续 8～15 帧确认
```

有效送达必须满足：

- 目标整体在正确子区；
- 目标不跨隔板；
- 目标不压住围栏或边界；
- 机器人已脱离接触；
- 目标相对安全区位置稳定。

## 7. 巡航和恢复

不使用无限固定路线。定义至少：

```text
中心观察点
左侧观察点
右侧观察点
备用观察点
```

每个观察点：

```text
停车 → 图像稳定 → 原地扫描 -45° 到 +45° → 更新目标列表
```

安全区或目标丢失：

```text
停车
→ 保持最后有效 IMU 航向
→ 短距离回退/回到上一个观察点
→ 扇区扫描
→ 仍失败则回到中心区域
→ 设置位置 UNKNOWN 并重新规划
```

## 8. 通信要求

上位机发送：

```text
版本、序号、状态、模式、速度、目标航向、速度上限、急停、有效时间、CRC
```

下位机返回：

```text
序号、执行状态、IMU、四路 ToF、编码器、故障、通信状态
```

安全约束：

- 控制命令 50 Hz；
- 下位机闭环 100 Hz；
- 超过 200 ms 未收到新命令自动停车；
- 急停由下位机立即执行；
- 旧序号命令不得重放；
- 推送时前向 ToF需按当前目标状态处理，左右后方仍保持防撞。

## 9. Agent 实施顺序

### Phase 1：代码基线

1. 阅读现有 `README.md`、`config`、`main`、`vision_logic`、`controller`、`uart_controller`。
2. 编译现有项目并记录基线错误。
3. 不删除旧功能，先创建新模块接口。
4. 添加配置文件和日志系统。

### Phase 2：视觉离线验证

1. 接入新类别模型。
2. 完成相机标定和地面坐标转换。
3. 完成安全区外框、隔板和 PnP。
4. 输出可视化视频，显示目标 mask、PnP 坐标和投送子区。

### Phase 3：传感器和下位机联调

1. 接入 IMU 和四路 ToF。
2. 验证时间戳、单位和坐标方向。
3. 验证急停、通信超时、倒车防撞。
4. 验证 IMU 航向控制和越障状态。

### Phase 4：单目标闭环

1. 只允许绿色普通物资。
2. 完成搜索、接近、推送、PnP 安全区定位。
3. 完成物资区投送和倒退确认。
4. 完成返回搜索区域。

### Phase 5：扩展比赛功能

1. 核心物资。
2. 伤员单独转运。
3. 危险物紧邻和遮挡。
4. 对方机器人动态避让。
5. 多物资转运。

## 10. 验收标准

### 视觉

- 新类别与代码名称完全一致。
- 目标紧挨时能保持独立实例。
- 安全区侧视时不会直接进入投送状态。
- PnP 重投影误差和位姿稳定性可记录。

### 定位

- 目标车体坐标误差满足近距离推送要求。
- IMU 航向控制不持续漂移。
- ToF 在停车、倒车和侧向防护中可靠触发。

### 规则

- 危险物不会被选为目标。
- 首个普通物资未有效送达时不能选择核心物资或伤员。
- 物资和伤员不会投送到错误子区。
- 机器人不进入对方安全区。

### 闭环

- 单个绿色物资完成率达到预设目标。
- 目标丢失、隔板遮挡、ToF触发、通信中断均能安全停车。
- 送达确认不依赖固定时间动作。
- 所有状态切换、传感器值和控制命令可回放分析。

## 11. Agent 工作规则

- 每次只修改一个可验证模块。
- 修改前先读取相关头文件和调用关系。
- 不假设下位机协议，协议不明确时使用适配层和 TODO。
- 不把未验证的成功判定写死。
- 不以编译通过代替实车验收。
- 任何涉及运动的默认动作必须有超时、急停和传感器保护。
- 每完成一个阶段，运行编译、静态检查和对应离线/硬件测试。
- 输出修改文件、接口变化、测试命令和未解决风险。
