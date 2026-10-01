# 2026-10-01 阶段 0/1 更新

已实现的六点规范、帧传感器接口、ZoneEstimate、IPPE 和半区坐标判定以 [STAGE01_IMPLEMENTATION.md](STAGE01_IMPLEMENTATION.md) 为准。以下历史清单中的 FRONT/完整隔板视角门控不再代表当前实现。阶段 2/3 和实车标定验收仍未完成。

# ds_innovation 文件改动清单

> **2026-09-25 后续更新：** 已按电控确认将发送改为10字节浮点包、接收改为4字节执行器反馈，修复协议编译阻塞；远端最新ARM64完整构建及5项CTest通过。已提供新发送接口，主程序实车闭环尚未开放。详见[接口调整记录](/home/liu/ds_innovation/reports/protocol-update-2026-09-25/REPORT.md)。下文保留此前核查时的状态与改动建议。

核查日期：2026-09-25。

依据：[实现现状总结](/home/liu/ds_innovation/PROJECT_IMPLEMENTATION_SUMMARY.md)、[方案评审](/home/liu/ds_innovation/ALGORITHM_PLAN_DECISION_VISION_REVIEW.md)，以及当前远端文件指纹。

本次通过 SSH 只读核验了 `cat@192.168.1.123:/home/cat/ds_innovation`：历史清单中的 74 个文件全部存在且 SHA256 一致；`src/`、`include/`、`tests/`、`config/` 内未发现清单之外的新文件。因此可继续用 [2026-09-24 远端源码快照](/home/liu/ds_innovation/reports/audit-2026-09-24/remote-source) 定位问题。本次没有重新编译、运行硬件程序或修改远端文件。

以下路径均相对于远端 `/home/cat/ds_innovation/`。本机同名旧源码不能直接覆盖远端。表内 `.hpp + .cpp` 表示需要同步修改接口与实现；新增模块名称是建议，实施时可以合并，但对应职责不可省略。

## 1. 第一批：恢复协议与构建基线

这批应先于真实运动接入。当前 `SensorPacket` 实际为 4 字节，但断言和接收逻辑仍使用 9 字节姿态帧，属于已确认的源码冲突。

| 现有文件 | 需要改什么 | 验收要点 |
|---|---|---|
| `include/rescue/types.hpp` | 分离执行器线上反馈与 IMU 业务数据；修正包大小、CRC 偏移、字段和注释；为执行器反馈定义独立状态和接收时间 | 必须先确认固件实际帧格式；不能只将断言从 9 改为 4 |
| `include/rescue/uart_controller.hpp` | 将反馈解析器、反馈缓存及读取接口与姿态数据解耦；明确兼容接口的用途 | 执行器反馈不能更新 IMU 有效性或新鲜度 |
| `src/uart_controller.cpp` | 消除已删除的 `yaw_mrad/pitch_mrad/roll_mrad` 引用；同步接收循环、帧长、CRC、范围校验、缓存及过期逻辑 | 拆包、粘包、噪声、坏 CRC、半帧超时后能恢复；无效帧不刷新有效数据 |
| `src/sensor_monitor.cpp` | 当前调用 `latestSensors()` 并打印姿态，应改为与反馈通道一致的监视输出，或明确区分监视模式 | 不将舵机反馈显示为 IMU 测量 |
| `tests/sensor_protocol_tests.cpp` | 替换旧姿态帧构造和 IMU 断言，保留并更新伪终端收发与坏帧恢复测试 | 新反馈帧、坏帧、过期、混入旧协议数据均有覆盖 |
| `SENSOR_PROTOCOL.md` | 分别描述执行器反馈与独立 HiPNUC 输入，明确反馈状态含义与时效 | 不能继续把旧 A6 模板称为真实 IMU 协议 |
| `VELOCITY_PROTOCOL.md` | 与接收协议统一动作含义、到位条件及命令关联限制 | 如果只有单个 `done` 位，必须写明其无法单独证明“本次命令完成” |
| `CMakeLists.txt` | 根据类型/模块拆分更新目标依赖，并给仍依赖 `assert` 的核心测试启用有效断言 | 干净 ARM64 构建及已有 5 项 CTest 可执行；不能引用旧二进制作为验证 |

如果保留现有 `SensorState` 作为独立 IMU 业务结构，姿态消费者可以暂时保持接口；如果重命名或拆分它，还必须联动检查 `camera_calibration.*`、`sensor_fusion.*`、`logger.*`、`tests/rescue_core_tests.cpp`。这些是接口迁移的影响范围，并非都要在第一批重写算法。

依据：[类型冲突](/home/liu/ds_innovation/reports/audit-2026-09-24/remote-source/include/rescue/types.hpp:45)、[旧接收解析](/home/liu/ds_innovation/reports/audit-2026-09-24/remote-source/src/uart_controller.cpp:45)、[监视器调用](/home/liu/ds_innovation/reports/audit-2026-09-24/remote-source/src/sensor_monitor.cpp:24)。

## 2. 第二批：配置、独立控制与执行器流程

| 现有文件 | 需要改什么 |
|---|---|
| `src/main.cpp` | 装配配置、输入、任务、安全监督和控制循环；保留单图、预览、回放模式；仅在完整接入条件满足后开放硬件模式，不能直接删掉当前拒绝启动检查 |
| `include/rescue/config.hpp`、`src/config.cpp` | 真正加载 YAML，明确 CLI 覆盖关系，校验字段/范围/版本；增加独立串口、机构参数、队伍/区域、标定文件、控制周期和命令有效期 |
| `config/rescue.yaml` | 配置真实支持的参数；启动时打印最终生效值；无效或缺失标定不能假定有效 |
| `include/rescue/uart_controller.hpp`、`src/uart_controller.cpp` | 为 `MotionCommand` 增加明确的 6 字节发送入口，处理短写、失败、重连和零速；隔离 `execute()` 当前发送的旧 13 字节接口 |
| `include/rescue/push_task.hpp`、`src/push_task.cpp` | 接入“停车→框住→等待本次到位→推行→释放确认”；执行器超时/故障不能进入下一运动阶段 |
| `include/rescue/sensor_fusion.hpp`、`src/sensor_fusion.cpp` | 区分 IMU、执行器及可选 ToF 各自健康与时效；将已有倾斜保护纳入最终输出裁决 |
| `CMakeLists.txt` | 登记新增控制、安全和执行器模块及其测试 |

建议新增：

| 建议文件 | 职责 |
|---|---|
| `include/rescue/control_loop.hpp`、`src/control_loop.cpp` | 独立固定周期发送、决策有效期检查、停机处理；控制周期不依赖推理返回 |
| `include/rescue/actuator_controller.hpp`、`src/actuator_controller.cpp` | 执行器动作状态机，区分目标状态、运动中、到位和故障；拒绝旧反馈确认新命令 |
| `include/rescue/safety_supervisor.hpp`、`src/safety_supervisor.cpp` | 统一裁决前进/后退/旋转许可、视觉与定位健康、扫掠空间、许可时效及最终速度 |

独立控制线程只能处理它自身仍正常运行时的决策断流。整进程卡死、串口断开后的停车依赖下位机超时机制；该固件实现未在本次项目资料中核实。若现有反馈协议无法关联命令，可靠关联可能需要下位机同步升级，不能凭空增加上位机字段后宣称解决。

依据：[主入口硬件模式拒绝](/home/liu/ds_innovation/reports/audit-2026-09-24/remote-source/src/main.cpp:73)、[旧发送入口](/home/liu/ds_innovation/reports/audit-2026-09-24/remote-source/src/uart_controller.cpp:170)。

## 3. 第三批：真实输入、时间证据与记录

| 现有文件 | 需要改什么 |
|---|---|
| `include/rescue/camera_capture.hpp`、`src/camera_capture.cpp` | 接入主流程；统一图像帧号、采集时间和时钟域，区分软件收帧时间与曝光时间；设置并回读 MJPG/曝光/增益/白平衡；完善断流退出与帧丢弃统计 |
| `include/rescue/types.hpp` | 按需补证据元数据：捕获/处理时间、来源、有效期、误差；不要把每次读取缓存当成新观测 |
| `include/rescue/push_task.hpp`、`src/push_task.cpp` | 将单个 `now_us` 扩展为来源明确的证据；候选确认和交付确认只消费不同帧，并检查持续时间 |
| `include/rescue/perception_adapter.hpp`、`src/perception_adapter.cpp` | 从只填写目标身份扩展为汇总真实几何、区域和安全证据；保持缺失证据默认无效；不能用 `true` 或零距离占位 |
| `include/rescue/logger.hpp`、`src/logger.cpp` | 接入真实运行链路，记录图像索引、原始 IMU、反馈、决策、最终下发指令、证据时间及拒绝原因 |
| `src/main.cpp`、`tests/fixtures/push_delivery.json` | 更新回放字段和版本；保留人工规则回放，另行增加原始输入重算流程 |

建议新增：

- `include/rescue/hipnuc_imu.hpp`、`src/hipnuc_imu.cpp`：实际型号的帧与 CRC 解析、时钟映射、坐标变换、模式校验、过期拒绝。
- `include/rescue/evidence.hpp`：一致观测快照及各证据有效期，可按职责拆分到现有类型中。
- `tools/replay/` 或独立回放模块：同步读取原始图像、IMU、执行器反馈，重新计算算法结果。

依据：[现有适配器](/home/liu/ds_innovation/reports/audit-2026-09-24/remote-source/src/perception_adapter.cpp:4)、[观测结构](/home/liu/ds_innovation/reports/audit-2026-09-24/remote-source/include/rescue/push_task.hpp:10)。

## 4. 第四批：几何、近场推送与单件交付闭环

| 现有文件 | 需要改什么 |
|---|---|
| `include/rescue/camera_calibration.hpp`、`src/camera_calibration.cpp` | 接入真实内参/畸变/外参/地面映射；使用有效 IMU；检查接触点可信度、距离相关误差及倾角 |
| `include/rescue/safe_zone_pose.hpp`、`src/safe_zone_pose.cpp` | 用旋转和入口法线区分安全区朝向与位置方位；处理平面歧义、地标身份和退化视角 |
| `include/rescue/zone_layout.hpp`、`src/zone_layout.cpp` | 统一有方向的入口局部坐标、隔板与左右半区，加入本队身份及镜像/斜视一致性检查 |
| `include/rescue/planner.hpp`、`src/planner.cpp` | 在现有后方接近点之外，规划接触后完整推送走廊、中转和退出路径，检查物体及机构扫掠 |
| `include/rescue/perception_adapter.hpp`、`src/perception_adapter.cpp` | 将同帧目标接触几何、入口/半区几何、保守占用及误差送入任务层 |
| `include/rescue/push_task.hpp`、`src/push_task.cpp` | 补接近、对准、可见停车门控、进区/越栏、释放、受许可退离、退开复核；按真实进展处理超时；保留首单规则并加入交付去重 |
| `src/main.cpp`、`CMakeLists.txt` | 将新几何与控制模块装配到运行链路并编入测试 |

建议新增：

| 建议文件 | 职责 |
|---|---|
| `include/rescue/zone_geometry.hpp`、`src/zone_geometry.cpp` | 提取隔板/入口/可靠角点，建立半区坐标与边界，输出可见性和几何质量 |
| `include/rescue/zone_servo.hpp`、`src/zone_servo.cpp` | 半区中心线闭环、横向/航向误差控制、入口门控、速度与角速度限幅 |
| `include/rescue/ground_occupancy.hpp`、`src/ground_occupancy.cpp` | 由可信接触边和形状尺寸估计保守地面占用；禁止将立体检测框四角直接当作地面轮廓 |
| `config/camera.yaml` | 由真实标定生成；不能复制示例矩阵或合成测试输出充当实机标定 |

`tools/camera_calibration/calibrate.py` 和 `test_calibrate.py` 已有基础能力，不应仅因主程序未接入就重写；在运行时文件格式、坐标约定或验收指标改变时再同步调整，补充真实多距离验证记录。

危险识别与最小可用定位是单件实车推送的前置条件，不能等到全部性能优化完成后才处理。

依据：[当前 FRONT 判定](/home/liu/ds_innovation/reports/audit-2026-09-24/remote-source/src/safe_zone_pose.cpp:43)、[总结中的证据生产者要求](/home/liu/ds_innovation/PROJECT_IMPLEMENTATION_SUMMARY.md)。

## 5. 第五批：完整比赛、世界状态与规划

| 现有文件 | 需要改什么 |
|---|---|
| `include/rescue/tracker.hpp`、`src/tracker.cpp` | 与世界坐标关联接轨，支持类别证据、遮挡/丢失、任务锁；可保留像素跟踪为前端，不必强塞全部世界逻辑 |
| `include/rescue/planner.hpp`、`src/planner.cpp` | 增加栅格 A*、未知区域、场边/围栏/禁区、动态障碍、整车及机构扫掠；保留近场几何能力 |
| `include/rescue/push_task.hpp`、`src/push_task.cpp` | 增加重定位/恢复/避让/安全停止接口，批次成员身份及逐件交付；与比赛级状态明确分工 |
| `include/rescue/config.hpp`、`src/config.cpp`、`config/rescue.yaml` | 增加规则表、场地/出发区、队伍、机构尺寸、限时与策略参数，区分比赛规则和保守策略 |
| `src/main.cpp`、`CMakeLists.txt` | 装配和构建比赛级模块 |

建议新增 `.hpp + .cpp` 配对模块，头文件放 `include/rescue/`，实现放 `src/`：

- `localization`：指令预测与 IMU/视觉修正、位姿协方差、历史状态和延迟更新、退化与重定位。
- `world_model`：世界目标关联、分类证据、动态障碍、已交付目标/区域占用。
- `task_planner`：按收益、时间、成功率和风险选择合法目标及任务。
- `match_manager`：180 秒生命周期、硬截止信号、终局、裁判介入和恢复。
- `delivery_ledger`：逐目标/逐批交付去重，数量/分值/扣分账本，暂停与重置语义。

正式需求还包括跨围栏可行性、各形状稳定推送和完整一局实测。仅新增状态枚举不能证明这些能力已完成。

## 6. 第六批：识别质量、模型迁移与遥测

| 现有文件 | 需要改什么 |
|---|---|
| `include/rescue/model_io.hpp`、`src/model_io.cpp` | 统一任务语义映射、张量布局和解码版本；支持经验证的新模型契约；接入危险优先、unknown/uncertain 保守处理 |
| `include/rescue/detector.hpp`、`src/detector.cpp` | 校验模型哈希/输入/输出/类别/量化契约；后续增加有界多 context 调度及核分配 |
| `include/rescue/yolo_detector.hpp`、`src/yolo_detector.cpp` | 与 RKNN 后端同步标签和输入输出约定，验证 ONNX 兼容性；若明确不支持某模型，应清楚报错 |
| `include/rescue/tracker.hpp`、`src/tracker.cpp`、`src/perception_adapter.cpp` | 累积分类证据，危险新证据覆盖旧普通判定，过滤已交付/禁入目标 |
| `include/rescue/config.hpp`、`src/config.cpp`、`config/rescue.yaml` | 同步模型路径、类别顺序和版本；不能只把 448 改为 640、7 类改为 8 类 |
| `foxglove_telemetry/src/foxglove_publisher.cpp`、`foxglove_telemetry/schemas/rescue_channels.json` | 将占位图像、固定姿态和示例状态改为真实采集/检测/定位/任务/安全数据 |
| `foxglove_telemetry/src/telemetry_smoke.cpp`、`foxglove_telemetry/CMakeLists.txt` | 增加真实连接与消息时效验证，处理 C++20 遥测工程与 C++17 主程序的集成边界 |

建议新增 `include/rescue/color_check.hpp`、`src/color_check.cpp`，完成经标定的颜色/形状复核、危险否决和普通双重确认。危险复核的可用版本必须在真实推送前完成；本阶段主要指系统化精度验收和性能升级。

模型升级时增加版本化模型清单（例如 `config/model_manifest.yaml`），配套真实导出/量化模型及分场景验收数据；具体新模型文件名须在产物确定后登记。现有 `best_fp16.rknn`、`best.onnx`、`best.pt` 不是应手工编辑的源码。

`benchmark_results/` 中转换/并发脚本和 `tools/remote_camera_telemetry/` 属于配套工具：仅在模型输入、运行契约或数据接口变化时同步调整，不是恢复构建的前置改动。

## 7. 测试与文档应随对应批次同步

| 现有文件 | 必须补充或同步的覆盖 |
|---|---|
| `tests/sensor_protocol_tests.cpp` | 新反馈协议、坏帧恢复、时效、串口异常；动作关联测试放执行器模块测试 |
| `tests/push_task_tests.cpp` | 同帧不能重复计数、证据过期、旧到位反馈、释放超时、退开复核、首单/批次身份、防重复交付 |
| `tests/model_io_tests.cpp` | 新旧模型契约与标签映射、未知拒绝、危险否决、观测适配器不能产生虚假有效证据 |
| `tests/rescue_core_tests.cpp` | 入口法线、投影时效、规划扫掠、传感器拆分与安全保护；保证断言始终有效 |
| `tests/fixtures/push_delivery.json` | 跟随证据/状态接口升级；另增遮挡、跨区、释放失败、视觉断流等回放夹具 |
| `README.md`、`USAGE_GUIDE.md` | 更新实际支持的模式、构建/配置方法、接入范围和未验证项 |
| `VISION_INTEGRATION.md` | 更新模型契约、几何输入、颜色/危险策略、数据与实测边界 |
| `SENSOR_PROTOCOL.md`、`VELOCITY_PROTOCOL.md` | 与实际固件和上位机同版本更新 |

随新增模块补针对性的测试，例如 `hipnuc_imu_tests.cpp`、`actuator_controller_tests.cpp`、`control_loop_tests.cpp`、`safety_supervisor_tests.cpp`、`zone_geometry_tests.cpp`、`zone_servo_tests.cpp`、`localization_tests.cpp`、`world_model_tests.cpp`、`match_manager_tests.cpp`。用伪终端/仿真/录制数据验证接口与故障，再逐阶段安排实机验收；不能以离线通过代替实机通过。

## 8. 暂不直接改动的部分与实施前待落实事项

- `src/controller.cpp`、`include/rescue/controller.hpp` 是旧抓取控制器，不是当前正式任务入口；不建议在其上另造第二套正式控制链路。
- 2026-09-29：历史原型 `rescue_state_machine.*` 已删除，`PushTask` 是唯一任务状态机。
- `REMOTE_REPAIR_REPORT.md`、历史审计快照和历史基准报告保留原始记录；新验证另附日期和版本，不能改写旧结果使其看起来已覆盖新代码。
- `build/`、`build-arm64/`、根目录 `rescue_upper_host` 是构建产物，通过重新构建生成，不手工修改。
- `turtle-crane/` 与救援程序改进无关。

实施前需要确认的外部事实：实际下位机反馈帧/CRC/到位语义、是否支持命令关联与超时停车；HiPNUC 型号/固件/输出模式；底盘运动学和机构尺寸；真实标定及首单取出/越栏可行性；新模型类别顺序和张量契约。这些不影响先完成文件范围分析，但决定具体实现，不能由现有占位字段推断。

建议按“协议构建基线 → 配置/独立控制/执行器 → 可信输入 → 近场几何和单件闭环 → 完整比赛 → 模型/性能/遥测升级”分批实施。第一批重点是第 1 节列出的 9 个文件；后续功能按依赖推进，不一次性改动全部模块。
