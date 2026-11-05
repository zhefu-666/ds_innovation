# ds_innovation · 智能救援机器人上位机

运行于 LubanCat / RK3588S 的智能救援机器人上位机，使用 C++17、OpenCV 和 RKNN 完成视觉感知、任务决策和 MCU 控制，Python/Shell 提供标定、调试和遥测工具。

**本文件是唯一日常维护文档。** 以下说明以板端 `/home/cat/ds_innovation` 为准，更新于 2026-10-29。下发控制包固定 **15 字节**；IMU 在上位机内部参与观测、导航和健康检查，不下发 yaw 或目标转角。软件回归与整车实机验收分开。

目录：[项目与代码](#project) · [编译与测试](#build) · [启动与查看](#run) · [运行完整流程](#full-run) · [四阶段独立测试](#phase-tests) · [四阶段逐段调试](#phase-stepping) · [停止与退出](#stop) · [夹爪、相机与底盘控制](#manual-control) · [配置](#config) · [通信与 IMU](#protocol) · [夹取与相机标定](#calibration) · [工具与排错](#tools) · [开发参考](#development)

| 日常操作 | 入口 |
| --- | --- |
| 第一次构建、修改后编译 | [编译与测试](#build) |
| 分别测试 startup / search / capture / transport | [四阶段独立测试命令](#phase-tests) |
| 同一进程逐阶段停车，人工 next | [四阶段逐段调试](#phase-stepping) |
| 整场运行、查看图像和日志 | [运行完整流程](#full-run) |
| Ctrl+C、暂停或结束 | [停止与退出](#stop) |
| 夹爪开关、相机角度、短程底盘测试 | [手动控制](#manual-control) |

<a id="project"></a>
## 项目与代码

上位机接收相机、IMU 和 MCU 状态，识别目标、估计几何、推进任务状态，再由 MotionLink 向 MCU 发送底盘和机构请求；遥测在旁路发布图像和状态。

| 检测类别（模型顺序） | 任务含义 | 处理方式 |
| --- | --- | --- |
| blue | dangerous_object，危险物 | 不作为搬运目标 |
| orange | injured_person，伤员 | 单独送入伤员半区 |
| green | ordinary_supply，普通物资 | 送入物资半区 |
| black | core_supply，核心物资 | 送入物资半区 |
| zone_blue | blue_safe_zone，蓝色安全区 | 区域归属与寻区，不作为搬运物或物块遮挡 |
| zone_red | red_safe_zone，红色安全区 | 区域归属与寻区，不作为搬运物或物块遮挡 |

Pose 模型的 `zone_left` 是物资半区，`zone_right` 是伤员半区；左右以站在安全区入口面向区内定义，不等于画面左右。队伍归属优先由与 pose 可见角点匹配的 zone_blue / zone_red 检测框确定；没有匹配检测时沿用已标定的 HSV 颜色判断。相互矛盾的区域检测保持未知。检测框可提供粗略寻区方向，精确几何与交付位置仍只使用 pose 关键点。

当前默认检测模型为 detect_v5_int8.rknn，类别 ID 0–5 顺序为 blue、orange、green、black、zone_blue、zone_red。检测输出支持单张量 [1,10,N] 或 boxes [1,4,N] + scores [1,6,N]；旧四分类模型仍可通过显式 --model 与 --classes blue,orange,green,black 使用。pose 默认文件为 models/a_rawxy_20261105/A_rawxy_INT8_EXPERIMENTAL.rknn；五路 INT8 输出经 CPU 恢复两组角点，再进入 [1,26,8400] 双分支解码。左右类别及各 4 点语义不变，旧 [1,18,N] 模型仍支持显式选择。models/detect_v4_manifest.json 仅描述旧模型，不是 v5 元数据。

~~~text
相机 → 检测 + 安全区 pose → 跟踪与几何 → 任务观测 → PushTask
IMU + MCU反馈 ────────────────────────↑               ↓
比赛许可 / 健康检查 → MotionLink → MCU → 底盘与机构
任务与感知快照 → 遥测文件 → 网页 / Foxglove
~~~

业务主流程为 `Startup（冲出）→ Search（搜索）→ Capture（捕获）→ Transport（搬运）→ Search`。

| 业务阶段 | 阶段内动作 | 实现 |
| --- | --- | --- |
| Startup | 0.1 m/s前进2秒；看到至少两个物块后闭爪接近、逐档切UI120/UI140；UI90→120停稳直接切档，UI120两侧触发后先前进0.1m，再以1 rad/s自转半周、后退0.1m、再自转半周；UI120选普通物资并开爪接近后切UI140，见普通物资后停车并直接进入搜索 | `src/task/startup_phase.cpp` |
| Search | Startup完成后保留UI140和普通目标继续近视角核验；运输结束后离区转向，再从UI90扫描；另含候选重找、准备及与Capture共用的图像分堆复查 | `src/task/search_phase.cpp`，`search_stage.cpp`，`first_trip_capture.cpp`，`recovery_phase.cpp` |
| Capture | 前冲、落框、捕获核验 | `src/task/capture_phase.cpp` |
| Transport | Carry 选定放置分区并搬运到位、释放、后退、交付核验、异常恢复 | `src/task/transport_phase.cpp` |

搜索动作精简（2026-11-04）：删除 `CueApproach`、`CuePrepare`、`ClearPile`、`ClearPause`、`Scatter` 及旧 `Pile` 动作，不再执行搜索线索闭爪推清、盲扫冲散、邻物摆扫或旧分层分堆。搜索只确认可选货物并接近；图像库存或抓取通道不合法时，调用与 Capture 相同的 `reviewFirstTripCapture` / `beginPixelSplit`，保留UI140后退、UI120转向、上角预处理和UI140复查。分堆继续遵守现有新鲜图像、夹爪反馈、净空、次数及总时间预算。旧 `PushState` 数字编号仅保留用于历史日志，进入已删除动作会停车报 `removed_search_action`。

搜索逐视角选物（2026-11-05）：搜索中的ID只作同一视角内接近和对齐的短期参考，不作为跨视角身份约束。UI90→UI120、UI120→UI140切换时清除旧目标，新视角按本趟需求、完整性和区域规则重新选物并连续确认；允许ID和所需类别变化，不要求旧ID重绑定或释放应答。完整的所需物块尚未进入夹取范围时可重新对正、接近，不能直接当作抓取就绪。UI140没有合格物块时停车结束本次交接并回到UI120搜索，旧目标消失不触发交接故障。抓取阶段仍保留动作期间的目标关联、夹取范围、库存及安全复核。


Capture 混装评分（2026-11-04）：首个普通物资已确认送达后，框内伤员与普通／核心混装时，停车用新鲜完整图像确认，再比较合法组合收益：伤员单独1个15分，普通每个5分、核心每个10分、物资最多2块，同分优先伤员。例如普通1＋核心1与伤员同为15分，保留伤员；核心2为20分，保留两个核心。选择约束持续到分离、UI120重新接近、UI140合框核验；危险、未知和直立伤员不作为收益目标。已经合框时先记录选择再开框复查。确认货物实际离框后重新评分；曾在框内的物块突然消失或被遮挡不能视为分离成功。每次合框核对实际整组数量，复查及分离沿用原有预算。日志 `[CAPTURE_SCORE]` 记录比较分数及保留组合，`[TASK]` 的 `score_supply`、`score_injured` 和 `keep_*` 可查看当前计划。

Transport 的动作是 `Carry → Release → BackOut → Verify → TurnAround`（非夹爪图像模式保留原有四动作），异常进入 `Recover`。进入 `Carry` 时即按本趟已确认货物锁定目的分区：普通／核心物资去物资放置区，伤员去伤员放置区；不再有独立 `Gate` 或 `Enter` 动作。夹爪图像运输在 `Carry` 内先按UI90新鲜本方区域方向接近，无需测距即可起步；获得可信区域几何后接近入口，车体旋转中心到入口边界40cm时停车；连续三帧一致的可信区域法向与采图时刻IMU建立目标航向，IMU闭环对齐并经新鲜图像确认居中后切UI120保持航向前进，对应半区整体轮廓的宽、高、下边缘位置同时达标即停车，停稳250ms后以三张新图确认，再进入 `Release` 开爪，再执行 `BackOut` 的IMU后退15cm。交付核验后在Transport内按IMU净航向旋转180°，停稳直接进入Search扫描。其他模式的入口对准及补推仍在 `Carry` 内完成。

`include/rescue/task/task_phase.hpp` 定义四个 `TaskPhase`、阶段内动作及运行状态；`src/task/push_task.cpp` 的 `PushTask::update()` 负责公共输入处理、运行许可和阶段调度。阶段通过 `PhaseOutcome` 请求交接，由主调度统一应用。`SearchPhaseData`、`CapturePhaseData`、`TransportPhaseData` 分别保存动作进度和阶段数据，`TripContext` 保存跨阶段目标与本趟货物；`TaskTick` 只存本次更新的输入、权限与输出。

暂停/等待/故障由独立 `RunStatus` 表示，不增加业务阶段。失败恢复保留所属阶段并运行 `src/task/recovery_phase.cpp`，完成后回到搜索；暂停恢复仍执行原有重新观察/安全退回规则。捕获结果区分 `Verified` 与 `Assumed`，交付结果分别记录应交数量、记账数量、视觉确认，视觉通过不自动等于计分成功。保留分堆时间预算、限时、黑名单、帧事务及核验策略。

当前 Startup 在 `startup_phase.cpp` 实现。出发区由 `behavior.start_zone` / `--start-zone 1|2|3|4` 指定，板端当前为3区：2、3区顺时针（负wz），1、4区逆时针（正wz），寻找目标时自转速度0.3 rad/s，UI120寻找普通物资前两次半圈自转均请求1 rad/s。夹爪闭合应答完成后，以0.1 m/s前进2秒；起步前进及UI90/UI120闭爪引导接近时叠加0.05 rad/s的出发区偏向（2、3区向右，1、4区向左），仍受现有转速上限、图像纠偏及弧线净空许可约束；组合动作中的IMU定距前进/后退、开爪后普通物资对准接近保持原规则；UI90旋转过程中同一新鲜稳定画面内识别到伤员、核心、普通、危险中的至少两个物块后停止扫描并开始闭爪接近，同类物块分别计数，重复跟踪ID或相同检测框只算一块，安全区不算。接近目标在四类中按同一视角画面中心Y最大选择，作为最近物块的图像代理，不优先选择远处普通物资。沿用当前下四分之一（1280×720时中心Y≥540）视角交接条件：停车确认新鲜稳定帧后，UI90直接切到UI120，不自转且不额外等待IMU航向；继续闭爪接近；UI120闭爪接近时，朝向使用同一新鲜稳定画面中四类有效物块检测框与画面交集的整体外接框中心，不再选择或锁定某一单块作为方向目标；该物块堆引导使用独立图像标识，单块跟踪ID变化不会触发换目标或丢失等待，无有效物块时仍停车等待。UI90单块引导、组合动作后的普通物资选择及UI140抓取规则保持原有流程。UI120改为同一新鲜稳定画面内左下角、右下角同时出现两个不同物块的检测框时触发组合动作，不再使用选中目标中心Y阈值。角区为画面下方四分之一与左右各三分之一的交集（1280×720时左区X[0,426)、右区X[853,1280)、Y[540,720)），检测框与角区有面积交集即计入，单个横跨画面的物块不同时计为两块；沿用四类有效物块和置信度/新鲜度过滤，不依赖原锁定目标是否仍可见。触发后立即停车并确认发送速度停止至少250ms，静止采样校零后保持UI120和闭爪先前进0.1m（IMU位移积分，速度不超过approach_speed，末段减速）；前进到位并停稳至少250ms后，再在UI120闭爪按出发区方向以1 rad/s自转半周，IMU累计有符号净航向达到180°后停车至少250ms，静止采样校零后以现有后退速度上限后退0.1m（IMU位移积分，不按命令时间估距），停稳至少250ms，再按原方向自转半周，IMU净转角重新累计达到180°并停稳至少250ms后保持UI120寻找普通物资；全程保持UI120和闭爪，新增前进遵守现有前方净空许可，后退遵守现有后方净空许可，IMU位移样本或发送反馈无效时停车失败，暂停后保留当前前进或后退段的已测距离并重新静止校零；自转期间目标离开画面不触发丢失重选，不提前切档，反向转动抵消进度；IMU航向无效时停车等待，暂停恢复后保留已累计转角并重新建立航向参考；此条件是图像策略，不是已完成的重叠区域标定。每次切换均等待相机反馈到位且稳定。组合动作结束后在UI120只从有效普通物资中按中心Y最大选取目标，选中后保持ID，先按现有严格像素容差居中，再停车确认新鲜稳定帧后开爪；未选到普通物资时按出发区方向和现有净空许可继续搜索，不再要求画面至少2个物块。开爪应答完成后先在UI120对准接近所选普通物资；到达原下四分之一视角交接位置（中心Y≥画高75%）后停车确认新鲜稳定帧，直接切到UI140，不重复自转和后退；UI140继续对准接近，小误差使用最高0.1 rad/s、100ms短步修正，并根据新图像误差趋势提前150ms制动；UI140使用稳定的新鲜画面重新选择完整、可选的普通物资，优先标定夹取区，再考虑Y180起的观察区；选择不要求沿用UI120目标身份，也不代表已到达Y337抓取线。UI140重新选中普通物资后继续留在Startup，保持开爪，按原像素对准与Y337停车线接近到位，再直接交给Capture闭爪并核验实际持有货物；观察区的其他普通物资不触发抓前数量分离。单个普通物资验证通过才运输，多抓、未抓到或证据不足进入现有释放/重试流程；危险物等其他类别以及净空、目标有效性、到位条件仍执行。独立startup或逐阶段模式在接近到位、交给Capture之前停车；夹爪运输结束后直接从UI90进入常规Search扫描，不再执行投放后180°掉头或旧离区转向。跳过Startup或独立测试search仍从SCAN开始。前进计时累计已请求非零前进的控制周期，停车/等待/暂停不计时；不是轮速或实测位移保证。`--startup-advance-ms 0` 跳过整个启动动作。Startup（含独立 `--phase startup`）原有前进和图像接近不以IMU就绪/新鲜度作为运行许可条件；UI120寻找普通物资前两次半圈自转必须有新鲜有效IMU航向，无效时停车等待，真实状态仍记录；受控清场场景不因IMU依赖的地面映射缺失被拦住，仍执行现有障碍物和机构条件检查。进入Search后恢复原有IMU约束。正常完成或暂停恢复不会重做Startup，显式reset才回启动等待。

Startup后退前IMU校零（2026-11-03）：仅此入口使用至少500ms、至少10个有效样本的平均窗口，每轴角速度上限0.15 rad/s、沿后退方向加速度标准差上限0.30m/s²，容纳转后噪声并降低零偏估计误差；其他阶段继续使用原250ms、0.05 rad/s和0.05m/s²。无效/过期样本、设备时间间隔检查、后方净空许可继续生效。校零日志以 `startup_pitch_reverse_calibrate` 开头，拒绝原因分别附加 `_gyro`、`_accel`、`_invalid_sample`、`_sample_gap`，基础名称表示仍在收集样本。

Startup锁定目标漏检时通过发送端正常减速；普通物资阶段有其他新鲜、可选的普通候选时，等待300ms后释放重选，其他情况保留最多1.5秒找回窗口；同视角下跟踪ID变化时，只接受同类别、重叠度至少45%、中心位移不超过画宽5%与较大检测框对角线25%两者最大值的唯一候选；多个同物关联候选有歧义时不直接重绑定，按上述丢失窗口释放后重新选择；150ms以内的同ID短缺口保留停车确认进度，但缺失帧不计数，恢复后需至少三个有效新帧，框位置跳变超过30像素或视角/类别变化重新确认。前进或新鲜图像确认横向绝对误差累计改善至少现有图像漂移门限（当前30像素）均计为进展；跟踪重编号、重复帧或误差来回抖动不重置进展计时。连续3秒没有前进或居中进展则释放该目标锁定，1秒内排除它以重选其他候选。普通物资居中确认完成、进入开爪接近时重新开始3秒进展计时，不沿用开爪前已经消耗的居中时间；开爪后仍保留无进展重选。普通物资重选保留当前夹爪开闭目标：首次开爪前保持关闭，已经开爪后，漏检、确认超时或类别变化只停车并重新选择/对齐，不反复关爪再开爪；搜索运动前仍须等待现有夹爪指令应答。上述近中心转向参数是初始控制参数，仍需实机核验制动效果，抓取标定范围保持不变。

遥测提供 `decision.phase`、`decision.action`、`state.run_status`、恢复标志和捕获/交付结果。感知、导航与运动就绪层改读明确的请求字段。`PushState` 暂保留为兼容动作编号，用于既有硬件辅助逻辑、日志及离线回放；主流程已按四阶段调度，但本轮没有删除全部旧动作兼容代码。

回归入口 `rescue_task_phase_tests` 验证四区启动方向、至少两个物块后闭爪接近、两次视角交接、UI120两侧触发后的IMU前进0.1m及原组合动作、组合动作后普通物资选择与自转/开爪条件、开爪接近后的UI140交接、2秒前进、Startup在UI140完成首抓对准/接近并直接交给Capture闭爪、运输后回UI90扫描及搜索到捕获的行程/货物交接、暂停保货、恢复归属和故障停止；捕获/交付集成测试覆盖真实证据与假设证据、全部/部分/未确认交付。历史快照及 `tools/runtime/record_baseline.py` 保留供手动查阅和比较，不再作为自动回归；自动回归检查当前配置、回放结果及阶段行为。

### 六类检测与夹取后寻区

目标检测类别中的 zone_blue / zone_red 分别映射为 blue_safe_zone / red_safe_zone，是安全区地标；blue 仍是危险物块。安全区检测框不参与物块选择、邻居拥挤度、夹取/持有清单、区域内物块计数或未知物体阻挡判断。

搜索、接近及捕获前，普通物资、核心物资或伤员物块的检测框与本方安全区框重叠，或两框最短边缘距离不超过图像短边的 3%（1280×720 时为 21.6 像素），该物块直接排除，不继续接近。蓝队使用蓝区、红队使用红区；只使用 200ms 内且置信度达标的检测框，无需安全区姿态关键点。已锁定目标也立即撤销，跟踪 ID 变化或长期未交付的放宽策略不会绕过此规则。日志标记为 [ZONE_TARGET_EXCLUSION]。运输阶段不执行此目标排除，原始物块仍保留在夹取通道及持有核验中。阈值定义于 perception_adapter.hpp 的 kTargetZoneGapImageFrac。

夹取成功进入 CARRY 后，利用安全区框中心寻找本方区域。检测框和姿态关键点的方向统一为机体系左正方位角，运输直接按该方向纠偏，不再在夹爪分支额外取反；只有关键点而无安全区检测框时，方向携带真实采图时间戳并受图像新鲜度限制，不能用于测距或开爪。控制日志中的 zone_sight、zone_enemy、zone_bearing、zone_sample_us、zone_source 记录实际方向输入。寻区规则：--team blue 选蓝区，--team red 选红区；同时出现两色时优先本方；已删除运输敌区避让，只有敌方框时继续原有寻区流程，不再执行敌区掉头或驶离。框提供方向线索，不要求同时检测到区域姿态关键点；仍要求检测新鲜、置信度达标和相机稳定。没有可用检测框时，已确认颜色的姿态观测仍可提供方向。非夹爪像素分支的场地模式、严格模式及 assume_all_safe 模式可按本方检测框方向接近；夹爪像素运输先按新鲜本方区域方向接近，不以测距作为起步门槛；可信入口几何可用后接入40cm测距停车；已删除远距离运输时粗略姿态锚点的转向回退：当前帧有本方方向就按本方方向接近；检测断帧时使用已有的本方区域航向记忆及 IMU 航向，不再切回区域中轴追踪，即使旧锚仍有效或两点定位被拒也一样；仅出现敌方检测时也不清除、改写或续期本方方向记忆，仍沿用本方最后一次观测的航向和既有有效期；无本方方向记忆时沿用寻区流程。上述几何近区对正保留于非像素运输模式；夹爪像素运输按下文40cm入口停车及UI120对应半区整体轮廓尺寸条件投放。方向偏差大时沿用原地对准，允许前进时按检测方向纠偏，路径净空和姿态保护继续生效；夹爪图像运输以40cm入口停车/IMU法向对齐和UI120对应半区整体轮廓尺寸及停稳后三帧确认触发到达停车与开爪，交付记分仍经过原有货物核验，普通远处安全区框不授权开爪或交付成功。

遥测 decision.zone_sight_valid、zone_sight_enemy、zone_sight_bearing_rad 与 enemy_zone_seen 可核对寻区方向；夹取前的物块目标仍通过 target_label/target_id 查看。

| 位置 | 职责 |
| --- | --- |
| `src/main.cpp`、`src/app/` | 入口、配置加载、生命周期、感知流水线、观测和离线任务 |
| `src/task/` | PushTask 与捕获、交付、退区、搜索及图像分堆复查 |
| `src/perception/` | 检测、RKNN、跟踪、颜色、像素和目标选择 |
| `src/geometry/` | 相机标定、地面映射、安全区定位和规划 |
| `src/hardware/` | 相机、MCU/UART、IMU、传感器融合、MotionLink |
| `src/control/`、`src/config/` | 比赛许可、控制服务、运行配置和标定策略 |
| `src/core/`、`src/telemetry/`、`src/tools/` | 基础类型、日志、遥测、独立监视器 |
| `include/rescue/` | 与实现职责对应的头文件 |
| `config/`、`models/` | 统一配置、相机 YAML、模型文件 |
| `tools/`、`tests/` | 按场景组织的工具、按模块组织的软件测试 |

<a id="build"></a>
## 编译与测试

### 进入板端

~~~bash
ssh cat
whoami
hostname
cd /home/cat/ds_innovation
~~~

用户和主机应分别为 `cat`、`lubancat`；连接地址使用当前已核实 SSH 配置。不要用旧 IP 或本地旧副本覆盖板端。

依赖：C++17 编译器、OpenCV 4.x、nlohmann/json、CMake 3.21+、Ninja、Python 3；工具测试还需要 `cv2`、`numpy`、`websockets`。

~~~bash
sudo apt install build-essential cmake ninja-build pkg-config libopencv-dev nlohmann-json3-dev python3 python3-venv v4l-utils
~~~

RKNN 模型和 SDK 资源独立部署，Git 克隆不包含这些被忽略的文件：

~~~text
models/detect_v5_int8.rknn
models/a_rawxy_20261105/A_rawxy_INT8_EXPERIMENTAL.rknn
benchmark_results/librknnrt.so
benchmark_results/rknn_api.h
~~~

### 搜索目标保持与重认

STARTUP 从 UI120 切换 UI140 后，若无法唯一关联原普通物资，则结束旧身份并重新选择：从普通物资标定区域内完整、可选的候选中，按中心 Y 最大的近端规则选一个（同 Y 按 ID 排序），连续确认三个新鲜停稳画面；确认期间只要候选仍合格就保持选择，不因其他候选出现或检测顺序变化重新切换。多个合格普通物资本身不导致交接超时。重新选择后仍停车进入 Search 的库存和危险物复查，不直接授权闭框。没有合格候选时保留原等待超时。任务故障同时反馈到 match 状态，显示原故障原因及 ready=0、permit=0，传感器健康不会解除任务故障。


像素搜索首次选取在全部合格候选中按局部每趟收益排序，同分优先伤员，再在同等收益候选中比较拥挤度和面积，并确认三个新图像帧；选定后不再被面积更大的邻块替换。短暂漏检保留身份 300 ms，底盘停车；超时或目标被明确拒绝时释放，由任务确认后重新选择。同视角的新 ID 必须同类、在位置/尺度门限内且匹配唯一，已知邻块不能继承锁定。相机换角度保留目标意图，停止期间不比较跨视角像素；到位后依据新鲜 IMU 航向和有效地面方位/距离连续确认三帧，证据不足则停车等待，300 ms 后释放重选。相机运动仍使用原有到位超时。

分散后重新搜索时，若目标在 UI120 切换到 UI140 后丢失并释放，程序停车切回 UI120，完成新鲜静止画面确认后恢复旋转搜索；搜索不前进、不增加分散次数，也不重置本轮搜索总截止时间。仅剩画面截断候选时同样走此恢复流程。无设备回归：`ctest --preset cat-release -R '^rescue_capture_postsplit-search$' --output-on-failure`。

日志 [PIXEL_SELECTION] 的 event、task_id、held_id、released_id、rebound_from 区分首次接纳、保持、歧义、同物体重认和释放；释放/重认事件保留到任务确认。可用 ctest --preset cat-release -R 'rescue_(mechanical|pixel_capture)_tests' 在不操作设备的情况下回归。

UI140 危险物侧边擦线复核：仅在左右边界已标定、危险物中心在侧边界外且横向侵入宽度不超过自身检测框宽度的 5% 时，标为“边缘待确认”，不会由此判定框外或抓取成功。闭爪前保持停车等待；闭爪后保持停车及当前爪状态，在既有 verify_budget_us 内复核。必须同一跟踪 ID、相机稳定、已发送速度为零，并有至少 3 个独立新鲜帧、持续 250 ms 显示检测框完全退出库存区和通道，才恢复正常核验；远离统计区的图像外侧裁切不抹去清晰可见的入侵侧边缘。重复帧、缺失、身份变化、运动、视角变化或时间倒退不能累计退出证据。明显侵入仍按原危险物规则拒绝，已见明显侵入不会被一帧浅侵入清除，持续待确认在现有预算结束后受控失败。普通物资、核心物资、伤员和 unknown 保留原有整框接触规则。5% 是像素判定的工程容差，不是机械安全验收值；该复核不能证明实际夹持范围或实物内外。日志 capture_scene_detail 给出危险物 ID、侵入比例、pending/outside 状态及退出帧数。


UI140 危险物上边界复核：朝向库存区/通道的检测框底边必须可见，并同时退出库存、通道及已配置的危险物类别区域；图像上沿 Y0 截断不改变 complete 标志，但不会否定仍清晰可见的朝内底边。仍须同一 ID、稳定同视角、实际已发送速度为零、至少 3 个独立新鲜帧且持续 250 ms 才确认区外。首次仅接触上边界或浅入时为 pending，不授权闭爪；已确认区外后，同一有效当前观测仅在 720 高参考图的 2 px 工程抖动带内浅入，可维持 outside，不新增等待窗口。该参数随参考图像高度缩放，不是机械安全距离，也不改变标定线。超过抖动带的明确侵入仍保留 risk，浅入不能清除 risk，须重新真正区外确认。缺失、重复/逆序/过期图像、身份/目标/视角变化、朝内底边不可见、相机不稳或无有效停稳反馈均不能维持区外授权；闭后沿用停车保爪及原有 4 帧/1.5 s 验收预算。日志追加 boundary、bottom_y、upper_penetration_px、upper_jitter 和 reset，区分当前边缘证据、滞回与重置原因。左右 5% 待确认和其他类别整框规则保持原义。


首趟普通物资切分在 UI120 转动结束后，停车返回 UI140，并明确结束旧目标选择。相机到位后的新鲜画面在普通物资计数区域内指定中心 Y 最大的完整普通物资，由像素选择器连续确认三个新图像帧，再同步任务、捕获及切分目标 ID；无需继承转动前的旧 ID。重新选择期间停车，全部检测框仍参与计数；仍有多个普通物资则重新后退、转动和复查，只有一个且通过现有抓取范围与停车确认才继续捕获。库存区内无候选、但区外仍有完整且本趟可选的物块时，先确认停车后的独立新鲜帧至少250 ms，再切回UI120正常重选和接近，返回UI140后重新验收分堆及捕获；不将目标离开库存区视为分离或夹取成功。日志pixel_split_target_outside_inventory和pixel_split_resume_ui120标识该恢复。无完整可选目标或观测不完整时停车等待，沿用本次切分总超时预算。日志 `split_reselect`、`split_candidate` 和 `first_trip_split_target_accepted` 显示交接进度。

直接进入 capture 时，若普通物资计数区域内有多块物资，先按同一近端规则重新确认目标，避免较大的远块被锁定后卡在首次复查。邻块整框与普通物资计数区域重叠时判为多物资风险，进入停车确认与拆分；这不授予闭框许可。普通物资已于2026-10-30补齐横向边界，通道使用X209～1071 px；若使用横向边界仍未测量的其他配置，仅与整幅图像占位通道重叠而不与计数区域重叠的侧向邻块仍等待横向证据。

### 像素抓取停车与视角交接（2026-10-29）

UI90→UI120 沿用目标中心达到画高75%（720高时Y≥540）的交接线。搜索阶段 UI120→UI140 提前到目标中心达到画高62.5%（720高时Y≥450），UI120直线接近速度上限为0.04 m/s；这是待实机确认的首轮参数。先请求零速度，通过现有发送层减速后，使用至少3张独立新鲜场景图像、至少200ms的停车指令窗口确认，再切换UI140；旧目标短暂丢失、被选择器释放或中心抖回交接线上方，不取消已经触发的交接，也不恢复前进/转向。停车确认不将零速度指令当作实测轮速。UI140到位后，完整、有效的候选目标还须通过现有新鲜图像稳定窗口；换目标沿用选择器明确的重认/重选事件与入口复查。停车和UI140目标确认各最多1.5秒，且仍受原15秒接近总预算约束；超时停车，分别记录pixel_search_handoff_stop_timeout或pixel_search_handoff_target_timeout。成功过程依次记录pixel_search_handoff_stop、pixel_center_switch_to_140、pixel_search_handoff_confirm、pixel_search_handoff_ready。交接不要求横向精确居中。UI140 普通物资、伤员和核心物资按各类已标定可抓取边界接近，达到 Y≥337 且仍在前后边界内时停稳闭框；搜索、抓取和闭框前复查使用相同位置条件。普通物资当前左右运行边界X209～1071 px，前后边界Y226～489 px；配置不再使用整幅图像横向占位范围。通道检查和闭框后的抓取验证保留。

开局 UI90/UI120 在前后及转动空间许可时，目标丢失、换目标、纠偏刹车和视角交接停车沿用发送层加减速斜坡，避免速度瞬间清零后反复起步；仍须等发送速度降到停车阈值并确认新鲜稳定图像，才允许下一动作或切换相机。空间受阻、急停和故障立即停车。UI140 近端抓取及其他阶段的像素停车保留 immediate_motion_stop，绕过普通速度减速斜坡；发送零速度后仍确认新鲜图像稳定，不能把命令零速当成机械已停止。一次接近沿用现有 15 秒预算，覆盖视角切换、重认和等待，超时停车退出。CONTROL_CHANGE 增加中心坐标、横向标定标志、抓取就绪、对齐阶段和实际发送速度，便于区分判定等待与停车余速。

Startup UI90→UI120切角准备（2026-11-04）：新鲜目标中心首次达到画高75%后，进入独立停车交接状态；框中心短暂抖回交接线上方或跟踪ID变化不再恢复前进、纠偏或进入旧目标的1.5秒丢失重选。停车确认可使用当前新鲜有效物块，换ID会重新确认图像稳定；丢帧、过期/未来帧、重复帧和相机不稳定不提供新的切角依据。仍须发送速度降到停车阈值，并通过原有制动稳定窗口及停车后的新鲜图像确认后才切UI120。UI120后续接近、组合动作及UI140抓取规则保持原流程。

### 混合物块验收与局部分堆（2026-10-30）

像素捕获在 UI140 的接近验收、闭爪前复查、每轮分堆后复查及闭爪后持有核验均覆盖普通物资、核心物资、伤员和危险物。首趟（尚未确认普通物资交付）只允许1个普通物资；后续趟允许普通/核心合计最多2个，或单独1名伤员。危险物、伤员与物资混装、超量物资不能通过验收；未知类别或不完整图像停车等待，不凭缺失的检测当作已分离。

普通库存区保留原横向范围、Y180～489。四类物块的完整检测框与库存区相交或碰边均计入候选库存；核心/伤员额外使用各自已有标定区域，以覆盖超出普通物资前后范围的候选。闭爪前、分堆后和闭爪后使用相同库存口径；危险及未知物块仍参与抓取通道检查。图像候选库存不替代闭爪后真实持有证据。

停车及新鲜图像确认本趟库存不合法后进入局部分堆：开爪应答完成 -> UI140以0.01 m/s直线后退 -> 本轮不需要的干扰物完整检测中心Y≤280且离开独立分堆库存区及抓取通道 -> 停车并用独立新鲜帧确认至少250 ms -> UI120稳定后原地转动0.3 rad/s、0.5秒 -> 停车返回UI140复查。后退参考为本轮抓取目标之外、与库存区或抓取通道相交的干扰物，包括同类多余物块；后退过程中不按最深物块改换需要抓取的目标。每轮入口记录不需要的物块，后退仅检查这些参考物；所需物块离开视野、检测截断或换ID不会拦住后退，也不将其新ID误作排出对象。未到Y280的参考物允许通过同类、邻近位置的唯一匹配更新ID，丢失、不完整或匹配不明确则停车；停车速度反馈不超过0.001 m/s与0.001 rad/s、稳定近视角的新鲜独立帧达到现有确认帧数且持续至少250 ms后，转入已有UI120转向重观测，再回UI140重新选目标并复核，不将漏检当作已越过Y280，也不执行上角盲推。该恢复共用原始时间预算与本轮次数，速度反馈异常、画面过期或相机不稳定会重置停车确认。日志first_trip_split_unwanted_reobserve及scene_blocker_id/scene_association标明该恢复；已经完整观察到Y≤280的物块即使后来从图像上沿消失，也保留其已越线状态，若同ID完整物块重新出现且Y>280则恢复检查。所需物块在转向结束后沿用原流程重新选取和确认。日志split_retreat_reference_id、split_retreat_reference_y记录当前后退参考。转向方向及上角动作仍由需要保留的目标位置决定，转动后沿用原有目标重选和闭爪检查。

方向根据本轮保留目标相对其标定夹取中心线的位置决定：右下方后退左转（wz=+0.3），左下方后退右转（wz=-0.3）；中心±12 px以内沿用上一轮方向，首轮默认右转。方向在每轮开始时锁定，后退和相机切换过程中不随检测抖动翻转；下一轮再按复查后的目标位置决定。

左右上角目标（中心的图像Y小于该类标定框的纵向中点、偏离夹取中心线超过12 px；包括图像Y小于标定框上沿的目标）在本轮入口锁定后，复用上述0.01 m/s、Y≤280和停车确认的后退流程；随后UI120右转90°，按实际发送速度的时间积分前推0.036 m（3.6 cm指令量），后退指令积分与本次前推指令积分配平，再左转80°，最终朝向相对后退结束时向右偏10°。角度使用IMU航向；本段直线动作不依赖IMU平移积分，其漂移或样本缺失不触发返程故障。推行速度上限取approach_speed与rush_near_speed的较小值，并按剩余指令积分减速，末段速度下限0.01 m/s。前后指令配平后换向停稳至少250 ms，转回后清除旧目标并进入UI120重新搜索、选择及接近，再于UI140复核抓取；动作超时也重新进入UI120搜索。指令量不表示实测行程。上角判定在后退过程中不改变，下角沿用原转动流程；净空、航向及实际发送反馈检查保留。上角分堆入口及后退停车不要求目标中心已进入夹取Y范围；例如21:41日志中的中心Y195.5、夹取Y226～489可以先分堆。闭爪仍须通过原标定位置、通道和库存检查。

转动后结束旧目标选择，UI140独立新鲜图像从可保留候选中指定中心Y最大的完整物块，通过像素选择器连续三帧确认，同步类别、任务目标及捕获目标；核心/伤员只在首趟普通物资已交付后可被选中。若仍有危险物、混装或超量物块，则重复后退、转动和复查，每轮计入原pile_max_runs及pile_budget_us限制；无目标、未知类别、画面过期、相机不稳定或运动许可不足时停车，暂停后从UI140后退定位重新开始。

上角预处理与普通后退转向分堆默认不限次数（`pile_max_runs=0`），持续切分直至通过目标分离与抓取验收；显式配置正数时，才分别限制两类动作次数。`split_runs` 仍记录两类动作的累计总次数；各类次数在本趟内保留，交付清理本趟后重置，原有时间预算、净空和抓取验收继续生效。

分堆后本趟库存合法，目标仍在标定范围内且连续新鲜帧、至少250 ms停车确认通过时，进入当前位置捕获；不再为达到Y337前进，以免把已分开的物块重新带入。闭爪前再次核验实际库存、目标位置和通道，闭爪后核验实际持有清单；确认通过才进入运输阶段。闭爪后发现混入物块则停车开爪、重新验收分堆，不将需要保留的目标直接拉黑，也不记作捕获成功。

独立 --phase capture 与整场像素捕获使用相同验收分堆流程；独立捕获成功后结束在捕获阶段，不自动运输。分堆和捕获不会提前更新first_ordinary_delivered，仍须交付验证成功后更新。非像素捕获和连续扫堆（PILE chain）沿用原有流程，不启用此局部分堆。

### 标准构建与回归

~~~bash
cmake --preset cat-release
cmake --build --preset cat-release
ctest --preset cat-release
~~~

板端 `CMakeUserPresets.json` 的 `cat-release` 继承共享 `release`：Ninja、Release、RKNN、遥测和测试，输出为 `build-release/`。板端预设包含本机依赖路径；其他机器使用自己的依赖配置，不直接复制。已有构建缓存沿用原生成器。

~~~bash
# 只构建主程序
cmake --build --preset cat-release --target rescue_upper_host
# 列出测试，按模块或层次筛选
ctest --preset cat-release -N
ctest --preset cat-release -L protocol
ctest --preset cat-release -L task
ctest --preset cat-release -L calibration
ctest --preset cat-release -L integration
# 单项回归
ctest --preset cat-release -R '^rescue_push_tests$'
~~~

最终验收使用未筛选的全量 CTest。测试使用合成观测、虚拟串口、临时文件和本机回环服务，不启动真实运动。

2026-10-29 测试精简：移除历史整段标准输出快照比较 `rescue_runtime_legacy_baseline` 及其比较器自测 `rescue_runtime_baseline_comparator_tests`，不再以冻结日志中的旧速度等参数约束当前调参。删除只要求配置导出字段数量超过固定门槛的断言；配置值、解析失败、设备初始化前退出、回放、四阶段行为、协议与虚拟串口测试继续执行。历史快照和手动比较工具保留，未重录基线。

<a id="run"></a>
## 启动与查看

<a id="full-run"></a>
### 运行完整流程（整场）

以下流程在板端运行完整比赛；启动器当前固定蓝队，默认3号出发区，比赛运动许可180秒。仅检查或查看图像时使用后面的无设备检查、只读预览入口。

**1．登录板端，进入项目。** 本机使用当前已核实的 SSH 别名，不沿用历史 IP：

~~~bash
ssh cat
cd /home/cat/ds_innovation
~~~

**2．首次运行或修改后编译、测试。** 后续命令均在板端项目目录执行；已有最新且验证通过的构建时可跳过本步。

~~~bash
cmake --preset cat-release
cmake --build --preset cat-release
ctest --preset cat-release
~~~

**3．释放设备并做静态检查。** 停止按键监听、预览，并退出其他占用相机或串口的工具。若按键服务正在运行比赛，stop 服务也会终止该比赛。

~~~bash
systemctl --user stop rescue-start-listener.service
python3 tools/remote_camera_telemetry/manage_project.py stop
bash tools/match/match_main.sh --check --frame close --start-zone 3
~~~

检查应显示实际主程序路径为 `/home/cat/ds_innovation/build-release/rescue_upper_host`，并以“完成：未打开设备”结束；本步不发送控制。若设置了 `RESCUE_BINARY`，核对它指向本次编译结果。相机、MCU、IMU 应连接并供电；MCU 默认为 `/dev/ttyACM0`，IMU 使用配置中的固定 by-id 串口。

**4．确认场地后提前准备整场。** 将机器人放在实际出发区，确认前后、旋转和机构扫掠空间已清空，场内无人员和对手。下面命令会操作夹爪和相机，底盘等待启动命令；换区时将检查和运行命令的 `--start-zone 3` 都改成实际的1～4区。

~~~bash
bash tools/match/match_main.sh --run --site-clear --frame close --start-zone 3
~~~

启动器先只读查询 MCU 的 A6 状态并检查配置，再将方框先打开、后关闭，相机预置 UI150（内部偏移 +30°）；
重新检查配置并初始化主程序、加载模型和持续采集。日志出现 `[PREPARED] ready=1` 后等待启动，
底盘不动，180秒尚未计时。此时按 A7 启动按键，或在另一板端终端执行：

~~~bash
cd /home/cat/ds_innovation
python3 tools/match/match_ctl.py status
python3 tools/match/match_ctl.py start
~~~

`status` 中 `WAITING`、`ready=1 permit=0 elapsed_ms=0` 表示已就绪。准备完成前的A7按键不排队，
就绪后需重新按键。整场 `--start-on-a7` 模式下，每次新的有效A7都会先停车、清空本轮任务与观察历史，在零速下确认夹爪关闭和startup视角到位，再从startup开始新一轮180秒；运行、暂停、任务故障及上一轮结束后均可重新开始。交接期间的额外按键不排队；硬件预检失败或准备超时会保持停止，恢复后需再次按键。stop/finish可取消待启动请求。整场需要预检通过后自动开始时显式加 `--auto-run`。单阶段和逐段调试不使用 A7，使用下文带 `--auto-run` 的调试命令。
业务循环为：

~~~text
开局：Startup 启动、对准接近 → Capture 闭爪检验 → Transport 运输交付
后续：Search 搜索、接近     → Capture 捕获检验 → Transport 运输交付
          ↑                                      │
          └────────── 继续搜索 ────────────────────┘
~~~

Startup 完成出发、视觉扫描和首个普通物资的UI140对准接近，直接进入Capture闭爪检验；后续Search选择、接近合法目标；Capture 前冲、关框并核验货物；Transport 寻找本方安全区，将货物送往对应半区，开框、后退并核验交付，再转向继续搜索。首趟先交付一个普通物资，后续按现有货物规则选择物资或伤员。比赛许可到180秒撤销运动许可，进程可能仍保留；提前准备模式默认允许持续等待；显式 `--auto-run` 的外层进程默认上限600秒，`MATCH_MAX_RUN_S` 可覆盖进程总时长。

**5．查看图像、状态和日志。** 启动器自动启动或复用只读网页桥，无需再开第二个预览进程。

| 入口 | 地址 / 内容 |
| --- | --- |
| 网页图像与状态 | `http://<当前板子IP>:8080/` |
| 状态数据 | `http://<当前板子IP>:8080/status` |
| Foxglove | `ws://<当前板子IP>:8765` |
| 整场日志 | `telemetry_logs/match-main-<日期时间>-<PID>.log` |

地址使用当前 SSH 配置核实且本机可达的板端地址；桥接脚本打印的 IP 可能是历史值，不应直接照用。运行日志的 `RUN_HEADER` 记录实际二进制路径、MD5、构建时间和参数。

**6．查询、暂停、恢复或结束。** 另开板端终端，进入项目目录后按需选择一条：

~~~bash
cd /home/cat/ds_innovation
python3 tools/match/match_ctl.py status
python3 tools/match/match_ctl.py stop
python3 tools/match/match_ctl.py resume
python3 tools/match/match_ctl.py finish
~~~

`status` 查询状态与就绪标志，`start` 从就绪等待开始，`stop` 暂停并保留进程，`resume` 按恢复规则继续，`finish` 结束本次运行。也可在启动终端按 Ctrl+C 退出；程序尝试发送零速收尾，夹爪与相机保持最后目标。180秒到时或 `finish` 后整场进程未退出时，可用 Ctrl+C 收尾。详细行为见[停止与退出](#stop)。

需要逐阶段确认时，将第4步命令加上 `--step-phases`，每到阶段交接前停车，在另一终端执行 `python3 tools/match/match_ctl.py next` 继续；此模式禁用180秒总限时，详见[四阶段逐段调试](#phase-stepping)。使用[独立阶段测试](#phase-tests)则跳过前序阶段，完成所选阶段后停车退出。

### 无设备检查

~~~bash
./build-release/rescue_upper_host --config config/project.json#/runtime_profiles/push_replay
./build-release/rescue_upper_host --config config/project.json --dry-run --imu --pitch-feedback --dump-config /tmp/rescue-effective-config.json
tools/match/match_main.sh --check
~~~

回放不打开相机或串口，成功摘要包含 `delivered=1 first=1`。配置导出在设备初始化前退出。比赛入口检查应打印 `build-release/rescue_upper_host` 的实际路径、摘要和构建时间；`RESCUE_BINARY` 可显式覆盖，应核对它没有指向旧程序。

夹爪模式的运输顺序为：闭爪携货接近本方安全区 → 入口外35cm开始最终对准 → 原地对正、实际零速停稳250ms后用三个独立新鲜目标半区图像确认（横向误差严格小于画宽5%） → 直接切UI120保持航向向区内推入，至本方地面颜色占比70%停车 → 停稳后开爪并等待完成反馈 → 保持打开后退15cm → 核验投放。删除停车后的前移横向修正、修正后再次确认和UI90对准后接近交接段；运输和核验不再执行松开后重新闭合推入的动作。

运输 CARRY 中的前进纠偏按前进速度和现有前视距离计算曲率转速，小误差落入航向死区时直行，不再将非零纠偏强制抬到 0.4 rad/s，避免低速运输时左右反复修正。远处边走边转采用相同计算；近区大角度原地对正继续使用原有转速和净空检查。`rescue_carry_task_integration_tests` 使用真实任务及串口速度渐变器，覆盖双向误差、快/慢航向响应、视觉短暂丢失及重新定位，验证纠偏收敛和持续前进，并覆盖连续敌区检测不触发避让、无姿态关键点时按本方框前进及通道阻挡停车；新增三种运输模式、左右/居中方向的断帧冲突回归，验证旧锚不再接管、2.4 秒无检测时仍不产生反向转向请求，且串口渐变最终完成正确换向。另覆盖三模式×左右/居中×寻区腿启动前后共18组“本方→相近敌方→空帧”序列，验证敌方检测不清除或刷新本方记忆、不反转为搜索/返回起始线，串口发送保持本方方向且通道阻挡仍停车。

### 主程序只读预览

相机和串口应没有被其他进程占用。当前 MCU 默认 `/dev/ttyACM0`，IMU 默认 `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_7a068e3dffefef1180ad4f5a6d9880ab-if00-port0`，均为 115200。网页桥需要 `websockets` 10.x；缺少时可在虚拟环境中安装：

~~~bash
python3 -m venv .venv-telemetry
. .venv-telemetry/bin/activate
python -m pip install 'websockets>=10,<11'
~~~

启动只读预览前，先停止开机按键监听服务，释放 MCU 串口 `/dev/ttyACM0`。该服务运行 `start_on_a7.py`，占用串口时预览会报 `Failed to open /dev/ttyACM0: Device or resource busy`。

**运行观测启动命令**（在板端执行；采集视觉、IMU 和 MCU 相机角度反馈，不发送运动控制）：

~~~bash
cd /home/cat/ds_innovation
systemctl --user stop rescue-start-listener.service
python3 tools/remote_camera_telemetry/manage_project.py start \
  --config config/project.json \
  --dry-run --no-show --imu --pitch-feedback --vision-log-dir none
~~~

启动前退出相机角度控制台；观测运行期间仍会占用 MCU 串口。查看运行状态：

~~~bash
python3 tools/remote_camera_telemetry/manage_project.py status
~~~

| 查看入口 | 地址 / 内容 |
| --- | --- |
| 网页 | `http://<当前板子IP>:8080/`，图像与状态；`/status` 返回状态数据 |
| Foxglove WebSocket | `ws://<当前板子IP>:8765` |
| 主程序日志 | `telemetry_logs/main.log` |
| 桥接日志 | `telemetry_logs/bridge.log` |
| 板上监看 | `python3 tools/board_monitor/monitor.py --open`；本机 `http://127.0.0.1:18080/` |

常用通道：`/camera/image`、`/detections`、`/fsm/state`、`/decision/live`、`/runtime/config`、`/cmd/motion`、`/system/health`、`/imu/data`。任务计算速度不等于实际车速，图像刷新也不代表 IMU 或 MCU 反馈新鲜。

~~~bash
python3 tools/remote_camera_telemetry/manage_project.py stop
~~~

比赛程序已运行时直接连接它的查看服务，不再启动第二个预览。`project_bridge.py` 读取 `/dev/shm/rescue-telemetry.bin`，不打开相机、不发送控制；独立 `server.py` 会打开相机。Foxglove 可能保留最后一帧，应同时查看连接状态和数据年龄。

#### 观测启动失败：残留 match socket

若启动时显示 `Project preview and viewing service stopped`，随后提示 `A process exited; see telemetry_logs/main.log and bridge.log`，先查看日志；前一行是启动失败后的自动清理提示，具体原因以日志为准。

~~~bash
cd /home/cat/ds_innovation
tail -n 80 telemetry_logs/main.log
tail -n 80 telemetry_logs/bridge.log
~~~

如果主程序日志包含以下报错，表示控制 socket 已被占用，或上次退出后留下了文件：

~~~text
[FATAL] Cannot bind match socket (already running or stale path): /tmp/rescue-match.sock
~~~

先确认是否仍有主程序或 socket 监听者：

~~~bash
pgrep -af '[r]escue_upper_host'
ss -xlpn | grep -F /tmp/rescue-match.sock
ls -l /tmp/rescue-match.sock
~~~

仅在前两条均无输出、确认主程序已经退出且 socket 文件仍存在时，删除残留文件并重新启动观测。有运行进程时先正常停止对应程序，不要删除正在使用的 socket。

~~~bash
rm /tmp/rescue-match.sock
python3 tools/remote_camera_telemetry/manage_project.py start \
  --config config/project.json \
  --dry-run --no-show --imu --pitch-feedback --vision-log-dir none
~~~

### 比赛与机构入口

| 命令 / 模式 | 行为 |
| --- | --- |
| `--dump-config`、`--check-config` | 配置导出或检查，不打开设备 |
| `--dry-run` | 可采集与推理，不发送 MCU 控制 |
| `tools/match/match_main.sh --check` | 默认静态检查 |
| `tools/match/match_main.sh --status` | 只读 MCU 状态，需串口独占 |
| `tools/match/match_main.sh --run --site-clear …` | 实机比赛运行，执行底盘和机构动作 |

比赛启动需要选择方框准备方式并满足运行许可。`--frame open/close` 会先发反向再发目标动作，`--frame keep` 需要核对已有状态和编号；整场入口默认 `close`（先开再关）。通过 `bash tools/match/match_main.sh --help` 查看参数（当前帮助输出退出码为 2）。

从登录、编译、释放设备到整场运行、查看和停止，按[运行完整流程](#full-run)执行。

启动器固定蓝队，打开硬件输出、IMU、遥测、异步采集和并行推理；默认将子进程限制在 CPU 4–7，可用 `RESCUE_CPUSET=all` 取消或指定其他核。先只读查询 A6、检查配置，完成方框/相机准备后再次检查，再启动主程序提前初始化。整场默认在实时预检通过、打印 `[PREPARED]` 后等待 A7 或 `match_ctl.py start`；单阶段和逐段调试命令显式带 `--auto-run`，预检通过后直接开始，不等待 A7。整场入口及 startup/search 在准备时预置相机 UI150；收到启动后由任务切换比赛视角，capture/transport 由阶段自行选择观察角度。

整场比赛许可默认 180 秒；到时撤销运动许可，不代表进程立即退出。整场默认允许持续准备等待；显式 `--auto-run` 时外层进程默认上限 600 秒，超时发送 SIGTERM，再过 10 秒仍未退出则 SIGKILL。`MATCH_MAX_RUN_S` 可覆盖进程上限。运行日志为 `telemetry_logs/match-main-<日期时间>-<PID>.log`；RUN_HEADER 记录二进制路径、MD5、构建时间和参数，MATCH_MAIN_EXIT_CODE 记录正常收尾后的退出码。

旧的系统级 `match-main.service` 保持禁用，`match_autostart.sh` 仍直接退出。新的用户级 `rescue-start-listener.service` 开机即调用 `match_main.sh --run --site-clear --frame close` 提前完成机构及视觉准备，主程序持续持有串口，首次准备后等待 CRC 正确的 A7 start=1 或本地 START；整场模式每次新的A7按键都建立新一轮并从startup开始，本地START/RESUME仍沿用原有语义。安装、日志、停止及重新待机命令见 [A7 按键启动服务](tools/match/README.start-listener.md)。查看新服务用 `systemctl --user status rescue-start-listener.service`；以下命令仅查看旧服务：

~~~bash
systemctl status match-main.service --no-pager
systemctl cat match-main.service
~~~

<a id="debug-without-a7"></a>
### 整体流程调试（绕过 A7 启动包）

A7 只是启动触发条件；调试完整流程无需模拟串口启动包，也无需修改代码。以下命令均在板端项目目录执行，出发区 `--start-zone 3` 按实际位置修改。

**方式一：已提前准备，用软件发令。** 主程序已启动并打印 `[PREPARED] ready=1` 后，在另一板端终端执行；不要另起第二个主程序：

~~~bash
cd /home/cat/ds_innovation
python3 tools/match/match_ctl.py status
python3 tools/match/match_ctl.py start
~~~

`status` 显示 `WAITING`、`ready=1 permit=0` 后可用 `start` 启动完整流程，无需 A7。命令通过本地控制 socket 交给主程序，不直接访问 MCU 串口。

**方式二：启动后自动运行完整流程。** 手动启动前，按[运行完整流程第3步](#full-run)停止按键监听服务和预览，释放设备；若服务正在运行比赛，停止服务也会终止该比赛。确认场地及机构扫掠空间清空后执行：

~~~bash
cd /home/cat/ds_innovation
bash tools/match/match_main.sh --run --site-clear --frame close --start-zone 3 --auto-run
~~~

`--auto-run` 使主程序在实时预检通过后自动开始实机动作，不等待 A7；仍执行方框、相机准备和相机、IMU、MCU 等预检，整场运动许可仍为180秒。启动器会在 `--auto-run` 和 `--start-on-a7` 中选择其一；直接调用主程序时也不能同时指定这两个参数。

**方式三：完整流程逐阶段调试。** 同样先释放设备并确认场地，再加 `--step-phases`：

~~~bash
cd /home/cat/ds_innovation
bash tools/match/match_main.sh --run --site-clear --frame close --start-zone 3 --auto-run --step-phases
~~~

该模式在同一进程中保留目标、捕获证据和货物状态，每次阶段交接前停车。在另一板端终端查询并继续：

~~~bash
cd /home/cat/ds_innovation
python3 tools/match/match_ctl.py status
python3 tools/match/match_ctl.py next
~~~

阶段断点使用 `next` 继续；运行中可用 `stop` 暂停、`resume` 恢复，`finish` 结束本次调试。逐阶段模式禁用整场180秒限时，保留阶段预算和健康检查，默认进程上限3600秒；停车点及状态含义见[四阶段逐段调试](#phase-stepping)。

<a id="phase-tests"></a>
### 四阶段独立测试（直接指定阶段）

每条命令是独立进程，跳过前序业务阶段，完成所选阶段或出现故障后停车退出，不需要 `next`。先退出占用相机、MCU 串口的预览或其他程序，确认场地和机构扫掠空间清空，再按需执行一条：

单阶段调试不使用 A7。以下命令均显式带 `--auto-run`，完成机构准备及实时预检后直接执行所选阶段。当前脚本不会仅因指定 `--phase` 就自动开始，调试时不要省略 `--auto-run`。

~~~bash
cd /home/cat/ds_innovation
# 启动准备 + Startup 前进/视觉自转/视角接近
bash tools/match/match_main.sh --run --site-clear --auto-run --phase startup
# 搜索和接近，进入捕获前停车退出
bash tools/match/match_main.sh --run --site-clear --auto-run --phase search
# 捕获与核验，不继续运输（默认首趟，只允许1个普通物资）
bash tools/match/match_main.sh --run --site-clear --auto-run --phase capture
# 跳过首趟投放，直接测试1个伤员，或1个普通＋1个核心；按需摆放后运行
bash tools/match/match_main.sh --run --site-clear --auto-run --phase capture --capture-after-first
# 已装货且已关框后执行运输与交付核验
bash tools/match/match_main.sh --run --site-clear --auto-run --phase transport
~~~

若日志持续显示 `WAIT_START`、`task_reason=not_running`、`match=waiting_start`，且 MCU `sent=0`、相机保持原角度，先核对实际启动参数是否缺少 `--auto-run`。首次启动许可发出前，主程序不会下发相机或夹爪命令；调试应使用上述自动开始命令，无需发送 A7。

以上命令按需要选一条，不要整段连续运行。startup 执行开局方框准备、相机预置UI150及完整启动动作（当前默认3区，换区时加 `--start-zone N`）；search 直接搜索接近，准备捕获前停止；capture 直接进入近视野，在原地确认当前目标和完整通道后前冲、关框、核验，完成后不运输；transport 从已装货且已关框开始，在原地核验货物，再寻区、入区、释放、后退、交付核验，结束后不转向搜索。

整场及startup默认 --frame close（先开再关）；search/capture默认 --frame open（先关再开）；transport 强制 --frame keep，避免开框丢失已有货物。capture 前将目标放在近视野的可捕获范围内；transport 前先装入合法货物并关框。独立会话默认保留首趟普通物资规则。`--capture-after-first` 仅允许与 `--phase capture` 同用，将本次测试的首个普通物资已送达状态置真，不增加实际交付计数，不开启 `--assume-all-safe`。单测伤员时抓取范围内只能有1个伤员且不能混入物资；普通＋核心测试放各1个，两者均需进入各自标定抓取范围。仍执行现有通道/分堆及抓取核验，成功后停车退出，不继续运输。该状态仅用于本次进程，不会给后续独立 transport 或整场运行保留历史。捕获/运输初始证据8秒内不足会停车失败，不虚构目标或货物。

UI140 的危险物判定在现有范围外增加前沿中心点风险区：`config/gripper-image-reference.json` 的 `dangerous_front_risk_region_xyxy_px=[209,160,1071,180]`（1280×720 原图，含边界）。这是依据 2026-11-04 漏判日志增加的风险范围，不是新的抓取标定；普通/核心/伤员的抓取范围不变。该区域统一参与通道、分堆、关框前及关框后核验，`--capture-after-first` 不绕过它；日志记录 `dangerous_front_region`。

单阶段结束会记录已应答的方框角度及编号，下一次 transport 仍核对下位机编号。若记录缺失/过期，现场确认后用 --known-frame-angle 20 --known-frame-id N 明确提供已关框状态；用 --status 只读查询编号。运输不执行开局方框翻转；capture/transport 不先回相机90°，由所选阶段控制观察角度。

将 --run --site-clear 改为 --check 可仅检查配置、不打开设备。日志 [PHASE_RESULT] 包含阶段、完成标志、核验结果；故障或交付未确认返回非零退出码。单阶段禁用180秒总限时，保留阶段预算、健康检查和默认600秒进程上限。Ctrl+C 提前结束。原 --step-phases 仍用于连续流程断点调试。

<a id="phase-stepping"></a>
### 四阶段逐段调试

在同一进程中保留目标、捕获证据及货物状态。调试命令显式带 `--auto-run`，预检通过后直接开始，不等待 A7；每次阶段交接前停车，使用 `match_ctl.py next` 继续：

~~~bash
cd /home/cat/ds_innovation
# 仅静态检查，不打开设备
bash tools/match/match_main.sh --check --frame open --step-phases
# 退出由管理器启动的预览，释放相机和 MCU 串口
python3 tools/remote_camera_telemetry/manage_project.py stop
# 清场确认后自动开始调试，无需 A7；仍执行开局关框→开框及相机回90°
bash tools/match/match_main.sh --run --site-clear --auto-run --frame open --step-phases
~~~

| 停车点 | 已完成 | next 后执行 |
| --- | --- | --- |
| STARTUP → SEARCH | 开局准备及完整 Startup 启动动作 | 搜索、接近、准备 |
| SEARCH → CAPTURE | 目标选择与接近 | 前冲、关框、捕获核验 |
| CAPTURE → TRANSPORT | 捕获核验，保留货物信息 | 寻区、入区、释放、后退、交付核验 |
| TRANSPORT → SEARCH | 本趟交付核验 | 离区转向及下一轮搜索 |

另一个板端终端执行（每次只发一条）：

~~~bash
cd /home/cat/ds_innovation
python3 tools/match/match_ctl.py status
python3 tools/match/match_ctl.py next
~~~

日志的 [PHASE_STEP] 显示完成阶段、下一阶段和结果；status 返回 phase_waiting=1 及下一阶段。next 仅在阶段断点可用，运行中重复发送不会排队。失败恢复返回搜索也会停在交接点，检查 reason，不将阶段结束视为捕获/交付成功。

中途 stop 使用原有暂停逻辑，resume 使用原有恢复规则；阶段断点必须用 next。finish 结束本次调试并退出，也可 Ctrl+C。等待时保持方框和相机指令，下一阶段计时从 next 开始；操作期间不要搬动目标或底盘。阶段模式禁用180秒总限时，保留阶段预算和健康检查，进程默认上限3600秒（MATCH_MAX_RUN_S 可覆盖）。不带 --step-phases 的整场入口保持原行为。

<a id="stop"></a>
### 停止、暂停与 Ctrl+C

四条 `--phase` 命令共用主程序退出路径。主程序运行期间在启动终端按 `Ctrl+C`，SIGINT 处理器设置退出标志；当前一轮处理结束后，主循环调用 `MotionLink::stop()`，等待发送线程退出，再尝试下发最后一个 `vx=0、wz=0` 包。这次清零绕过正常速度斜坡；不是按键瞬间在信号处理器里写串口，没有固定的按键到停车时延保证。

零速收尾保持最后的夹爪与相机目标，不自动开爪或复位；尚未到位的机构可能继续完成原目标。命令提交停顿超过 150 ms 时，仍在运行的发送线程也会将过期命令替换成零速度。尚未提交过命令时，stop() 不凭空发送机构目标。

退出日志 `[MCU] stopped: final zero-velocity packet sent` 目前没有根据最终写入结果区分成功和失败，不能独立证明下位机已接收或车轮已静止。串口断开、掉电、SIGKILL 等不能依赖这条正常收尾路径；本节说明软件行为，不代表实车急停验收。启动前的方框/相机准备由独立工具执行，主程序收尾路径适用于主程序已启动后的运行。

主程序仍运行时，可从另一个板端终端选择一条命令：

~~~bash
cd /home/cat/ds_innovation
python3 tools/match/match_ctl.py status  # 只读状态
python3 tools/match/match_ctl.py stop    # 撤销运动许可并暂停，保留进程
python3 tools/match/match_ctl.py resume  # 按原恢复规则继续；阶段断点须用 next
python3 tools/match/match_ctl.py finish  # 结束本次运行；单阶段/逐段模式随后退出
~~~

这些命令通过 `/tmp/rescue-match.sock` 请求主程序处理，不直接访问串口。OK 是主程序受理，不是 MCU 停车应答；主程序已退出时不能用它补发零速。`demo.py stop` 只管理 demo 会话，不替代比赛控制命令。

<a id="manual-control"></a>
## 夹爪、相机与底盘控制

### 夹爪打开、关闭与状态

下面带 `/frame/assume` 的打开命令已由用户在实机确认有效。该配置使用 A6 状态 `1=动作应答`，开、关都会返回 1，不能据此判断夹爪开闭；TX 偏移为 `+5=开、-5=关`，开合各等待配置中的 100 ms。配置仍标记为临时假设，不代表整车验收。

~~~bash
# 打开夹爪
python3 /home/cat/ds_innovation/tools/frame/frame_ctl.py open \
  --frame-config '/home/cat/ds_innovation/config/project.json#/frame/assume'

# 关闭夹爪
python3 /home/cat/ds_innovation/tools/frame/frame_ctl.py close \
  --frame-config '/home/cat/ds_innovation/config/project.json#/frame/assume'

# 只读状态，不发送控制包
python3 /home/cat/ds_innovation/tools/frame/frame_ctl.py status
~~~

打开、关闭命令同时发送零底盘速度，并保持首次 A6 反馈中的相机角度。需要只读反馈并预览控制包时，在开合命令末尾加 `--dry-run`。省略 `--frame-config` 会读取尚未验收的 `/frame/pending`，报错 `A6 / physical action mapping has not been accepted`。

### 相机角度控制

先退出占用 MCU 串口的主程序或其他串口工具。通过遥测管理器启动的预览可用 `python3 tools/remote_camera_telemetry/manage_project.py stop` 停止；`--dry-run` 主程序仍会占用串口。

~~~bash
cd /home/cat/ds_innovation

# 只读当前相机角度，不发送控制包（同样需要串口独占）
sudo python3 tools/remote_debug/camera_angle_console.py --status

# 启动交互式角度控制台
sudo python3 tools/remote_debug/camera_angle_console.py
~~~

出现 `相机角度>` 提示后，输入 `90..150` 内的角度并回车，例如 `120` 回中位、`130` 向下调整；角度增大向下。输入 `s` 读取状态，`q` 退出，退出时不自动复位相机。控制包中的底盘速度固定为零，并保持指定夹爪偏移。

如果提示“编号非0，无法推断夹爪偏移”，需显式提供当前已知的夹爪偏移（范围 `-5..5`）；按上述已确认的夹爪配置，打开为 `5`、关闭为 `-5`。例如，已确认夹爪当前保持打开时：

~~~bash
sudo python3 tools/remote_debug/camera_angle_console.py --hold-gripper-offset 5
~~~

不要用猜测的偏移代替当前值，以免同时改变夹爪。默认串口为 `/dev/ttyACM0`、波特率 115200，可通过 `--port` 和 `--baud` 指定。若出现 `[Errno 11] Resource temporarily unavailable`，先用 `fuser /dev/ttyACM0` 或 `lslocks` 查找占用者并退出对应程序；`sudo` 不会绕过串口独占锁。

### 底盘前进、后退与转向

**`--check` 只检查配置，不打开串口、不发送运动指令，底盘不会前进。没有 `BLOCKED` 表示静态检查通过，不表示已经执行动作。实际运动必须使用 `--run --site-clear`，并去掉 `--check`。** 省略 `--run` 也默认只检查。

**1. 选择演示配置（每个新终端执行一次）**

该 profile 使用 `/frame/assume` 中的夹爪协议映射和临时演示验收配置；`/frame/gripper` 的 A6 也采用动作应答语义，切换几何配置不能消除未知开闭目标的问题。

~~~bash
export RESCUE_DEMO_PROFILE='/home/cat/ds_innovation/config/project.json#/profiles/demo_assume'
~~~

**2. 仅在缺少夹爪记录时建立记录**

若提示“未知夹爪当前开闭目标/动作编号”或 A6 编号为 0，先由操作者执行一次已确认可用的夹爪开合命令。下面命令会实际打开夹爪，底盘速度为零；希望夹爪关闭时将 `open` 换成 `close`。若同一用户已经成功执行过开合且记录仍有效，可跳过此步。

~~~bash
python3 /home/cat/ds_innovation/tools/frame/frame_ctl.py open \
  --frame-config '/home/cat/ds_innovation/config/project.json#/frame/assume'
~~~

A6 的 `状态=1` 只表示动作应答，开、关都会返回 1，不能据此判断夹爪开闭。成功后会输出 `KNOWN_FRAME angle=… id=…` 并保存默认记录；底盘演示会自动读取，无需手工抄写编号。

**3. 静态检查（不会运动）**

~~~bash
python3 /home/cat/ds_innovation/tools/demo/demo.py move-forward \
  --speed 0.05 --seconds 0.5 --check
~~~

输出 `模式：CHECK_NO_DEVICES` 属于正常的检查结果。没有 `BLOCKED` 后，继续执行下一步的实际运动命令；检查通过本身不会发出运动指令，也不验证实时 A6 编号。

**4. 实际运动（确认运动空间清空后执行）**

`--site-clear` 表示操作者已确认运动空间清空。下面每条命令独立执行一次动作，不要将整个代码块一次性粘贴执行。

~~~bash
# 前进：0.05 m/s，持续 0.5 秒，随后发送零速度
python3 /home/cat/ds_innovation/tools/demo/demo.py move-forward --speed 0.1 --seconds 0.5 --run --site-clear

# 后退：0.05 m/s，持续 0.5 秒
python3 /home/cat/ds_innovation/tools/demo/demo.py move-backward --speed 0.05 --seconds 0.5 --run --site-clear

# 原地左转（逆时针）：0.20 rad/s，持续 0.5 秒
python3 /home/cat/ds_innovation/tools/demo/demo.py turn-left --wz 0.20 --seconds 0.5 --run --site-clear

# 原地右转（顺时针）：0.20 rad/s，持续 0.5 秒
python3 /home/cat/ds_innovation/tools/demo/demo.py turn-right --wz 0.20 --seconds 0.5 --run --site-clear
~~~

实际执行时显示 `模式：EXECUTE`。按指令速度计算，0.05 m/s 持续 0.5 秒的理论距离约为 **2.5 厘米**；实际距离取决于底盘响应，不以该计算值作为实测结果。

直线速度 `--speed` 范围为 `(0, 0.10]` m/s，转向速度 `--wz` 为 `(0, 0.25]` rad/s；方向由子命令决定，参数填写正数。显式 `--seconds` 范围为 `(0, 1]` 秒。

**实际运动命令带 `--run --site-clear` 且去掉 `--seconds 0.5` 后会持续运动**，直到当前终端按 `Ctrl+C`，或在另一板端终端执行停止命令。查看与停止本套 demo 会话：

~~~bash
# 查看本套 demo 工具启动的会话
python3 /home/cat/ds_innovation/tools/demo/demo.py status

# 停止本套 demo 工具启动的会话
python3 /home/cat/ds_innovation/tools/demo/demo.py stop
~~~

底盘工具用已记录的夹爪开闭目标和初始 A6 动作编号保持夹爪，并保持反馈中的相机角度；正常结束或中断时补发零速度。同一板端用户通过 `frame_ctl.py open/close` 成功执行的默认记录可直接被 demo 读取，按串口隔离；自定义 `--state-file` 的记录不自动读取。`--known-frame-angle 0|20` 保留兼容参数名，表示开/关的内部业务值，不是夹爪物理角度；手动指定时必须同时提供实际非零 `--known-frame-id`，不能填占位编号。运行前会核对实时 A6 状态为 1 且编号与记录一致，不匹配则拒绝发送。`stop` 只管理本套 demo 会话，不会停止其他独立程序；没有会话记录不等于实物已停车。

### 底盘速度平滑（2026-10-28）

主程序 MotionLink 在实际串口发送前限制线速度和角速度变化，首次从零起步，反向先减到零再反向加速；按发送间隔计算，不随视觉帧率跳变。当前初始调参为线加速 0.30 m/s²、线减速 0.40 m/s²、角加速 0.80 rad/s²、角减速 1.00 rad/s²，集中于 MotionLinkConfig 并导出到有效配置；它们是软件调参，不是机械实测极限。0→0.16 m/s 至少约0.53秒，+0.2→−0.2 rad/s 至少约0.45秒。

目标短暂丢失且通道仍安全时平滑制动；故障/STOP/结束、指令过期、净空失效、相机/夹爪等待和精确夹取停车直接归零，保留 IMU 倾斜保护及故障锁定；当前根配置 `behavior.imu_tilt_limit_deg=30.0`（C++ 默认值为 12°），运行值以有效配置为准。正常制动存在余程：0.16 m/s 按0.40 m/s²制动的指令积分约3.2厘米，因此没有将精确夹取终点改为渐停。夹爪编号和相机目标不经过速度滤波。

主程序统一在 MotionLink 发送层进行角速度响应补偿（2026-11-05 更新）：`hardware.angular_command_scale` 当前为 `0.177`，也可用 `--angular-command-scale K` 覆盖，范围 `(0,1]`，`1` 为关闭补偿。业务转速、控制限幅和加减速斜坡仍以 rad/s 计算，完成斜坡后仅对串口角速度乘一次系数；线速度、夹爪、相机及立即停车逻辑不变。Search 与 Transport 直接使用斜坡后的角速度乘系数：首趟搬运释放、后退完成，切入 `SEARCH/SCAN` 后，请求 `0.6` 最终发送 `0.1062`，不再通过 `1/K` 恢复到 `0.6`；搜索阶段其他转向及后续趟次同样适用，不再额外叠加 `0.20` 搜索限速。Startup 与 Capture 保留原调参恢复：先除以当前系数，再统一乘系数，最终串口速度及加减速过程仍与原调参一致。阶段选择由主程序提交到 MotionLink，同一命令的周期重发不重复缩放。日志 `legacy_angular_phases=startup,capture`、`scaled_angular_phases=search,transport`，`sent_wz` 为业务指令，`precomp_wz` 为发送层补偿前值，`wire_wz` 为实际串口值；这些值不代表 IMU 实测角速度。该换算不作用于独立直发串口工具。初值来自三轮日志：发送0.4时IMU航向变化约2.23–2.30 rad/s，暂取0.4/2.26≈0.177。

搬运接近与原地修正（2026-11-03）：入口法向经三帧确认后，在原有1.5秒有效期和图像一致性约束内持续使用，不因当前帧确认计数跌落而切换相反转向公式；采集时IMU缺失不会延长该有效期。搬运原地修正采用不超过扫描/全局限幅的最低有效转速（当前比赛入口为0.4 rad/s），前进弧线仍为比例控制。后退保向误差超过0.06 rad才开始纠正，进入纠正后须由IMU确认误差不超过0.02 rad并停车稳定250 ms，才恢复后退。统一角速度补偿仍为0.177，Search 与 Transport 均不使用 Startup/Capture 的原调参恢复增益。验证：搬运集成测试覆盖间歇采集、左右两侧/两种目标半区、发送斜坡及原地转向死区；实机效果须由下一轮运行日志确认。

[MCU] requested_vx/requested_wz 为任务目标，sent_vx/sent_wz 为最近成功发送对应的业务速度（经过斜坡、角速度尚未乘补偿系数）；precomp_wz 为补偿前角速度：Startup/Capture 为 sent_wz÷angular_command_scale，Search/Transport 为 sent_wz；wire_wz 为实际串口浮点角速度，始终等于 precomp_wz×angular_command_scale。CONTROL_CHANGE 同样记录 phase、precomp_wz 和 wire_wz。指令里程、静止观测和运动监控使用业务速度；IMU 为独立实测，sent_wz 和 wire_wz 都不是实测转速。[IMU_SAFETY] 在每次安全判定变化时记录失败帧的时间、姿态、有效性和串口计数；周期 estop 复用同一次实际安全判定，避免旧时间检查新样本造成误报。

IMU接收恢复（2026-11-04）：CP210x出现USB接收端点停滞时，串口可能不报断开但不再收帧。连续未收到设备时间推进的HI91帧达到 max(1000ms, 2×IMU新鲜度期限)，或串口报告poll/read错误或断开时，接收线程关闭并重开同一配置路径；首次重试间隔250ms，打开失败后每秒重试。恢复时清空旧测量和历史半包，仅保留最后收帧时间供安全判定，保持累计帧号/计数；重开本身不恢复有效航向，须重新收到满足新鲜度和数值检查的测量。`[IMU_RECOVERY]` 记录 lost/open_failed/reopened/resumed，周期 `[IMU]` 持续记录接收计数与 reconnects。Startup组合动作继续按有效IMU航向放行；本机制恢复接收，不代表已消除USB故障的物理触发原因。

2026-10-28 历史板端验证：修改前授权短程复现记录俯仰12.23°、28.088毫秒后恢复；修改后一次授权运行含准备动作8.75秒，已记录最大横滚/俯仰4.44°，未触发倾斜故障。两次目标与路线不同，不能视为同工况对照或完整机械验收。

<a id="config"></a>
## 配置

`config/project.json` 是运行参数、演示 profile、机构与区域资源的统一入口；相机矩阵继续使用 OpenCV YAML。

| 分组 / 资源 | 用途 |
| --- | --- |
| `hardware` | 相机模式、MCU 和 IMU 端口、波特率、反馈开关 |
| `models` | 检测/pose 模型、类别、阈值、NPU 核与并行推理 |
| `calibration` | 相机 YAML、区域/任务/颜色资源、俯仰预设 |
| `behavior`、`runtime` | 行为参数、队伍、运行模式、遥测与日志 |
| `profiles` | demo 工具 profile，与主程序 runtime profile 区分 |
| `runtime_profiles/preview`、`runtime_profiles/push_replay` | 独立完整的主程序配置片段 |
| `frame`、`acceptance`、`task_calibration`、`zone_color`、`zone_geometry` | 测量、假设与验收资源，不能仅凭路径当作已验收 |
| `config/camera.yaml` | 当前唯一运行相机参数：2026-10-27 手眼旋转与尺量平移，固定 UI 90°（内部 -3000 cdeg），地面验证已由用户确认完成 |

主程序与所有 demo profile 统一使用上述相机文件；旧 5°/40° 参数仅在 `tests/fixtures/legacy_config/` 作为冻结回归输入保留，不用于启动。原始标定会话保留在 `calibration_runs/`。

请求按“默认值 → JSON → 显式命令行”解析，之后兼容策略和标定还可能调整最终值；用 `--dump-config` 同时检查请求、来源和最终参数。导出快照不是可直接回读的输入 profile。

~~~bash
./build-release/rescue_upper_host --config config/project.json#/runtime_profiles/preview --dump-config -
RESCUE_DEMO_PROFILE="$PWD/config/project.json#/profiles/demo_default" python3 tools/demo/demo.py recognize
~~~

JSON 片段是完整 schema v1 配置，不与根节点叠加；文件路径相对 `project.json` 所在目录，命令行路径相对当前目录，不展开 `~` 或环境变量。未知键、重复键、类型错误和非法数字会报错。`schema_version` 为整数 1，`behavior_policy` 为 `legacy_field_v1` 或用于任务回放的 `legacy_replay_v1`。

配置字段的完整映射见 `src/app/config_loader.cpp`，运行参数见 `./build-release/rescue_upper_host --help`。旧分散配置只在 `tests/fixtures/legacy_config/` 保留冻结字节，不作为当前启动配置。

当前根配置使用已确认地面验证的 2026-10-27 外参，`calibration.pitch_presets_cdeg=[-3000,-3000,-3000]`，地面投影只在 UI90（内部 -3000）已标定。图像参考 `runtime_enabled=true` 时，UI120/140 像素夹取会在运行时选择相应视角，不能用请求预设判断主程序始终停在 UI90；用 `--dump-config` 查看最终任务参数。

当前根配置另设 `behavior.imu_tilt_limit_deg=30.0`，与 C++ 默认 12°不同；MCU/IMU 端口为 `/dev/ttyACM0`、`/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_7a068e3dffefef1180ad4f5a6d9880ab-if00-port0`。比赛入口还会覆盖硬件输出、IMU、显示、队伍和运行模式等设置。`--gripper` 会选择候选机械尺寸、UI90/120/140 预设及 `config/camera-overlap.yaml`，仍受候选标定门槛限制，不是启用普通像素夹取的必需参数。

<a id="protocol"></a>
## 通信与 IMU

### 上位机下发：固定 15 字节

MCU 串口默认 115200、8N1。小端、1 字节对齐，浮点为 IEEE754 float32。

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | uint8 | 帧头 `0x56` |
| 1..4 | float32 | `vx`，m/s；正前进、负后退 |
| 5..8 | float32 | `wz`，rad/s；正逆时针、负顺时针 |
| 9 | int8 | 夹爪偏移：+5 开、-5 关、0 为 85° 中位 |
| 10 | uint8 | 动作编号，1..255 循环；0 表示无动作 |
| 11..12 | int16 | 相机整数度偏移，-30..+30；0 对应绝对舵机角 120° |
| 13..14 | uint16 | CRC16/Modbus，覆盖字节 0..12，低字节在前 |

CRC 初值 `0xFFFF`、多项式 `0xA001`，无最终异或。包内没有 IMU yaw、目标转角或其他扩展字段，15 字节布局不与历史 17/19 字节包混用。

业务层仍用方框逻辑值 `0=开、20=关`，编码器映射为 +5/-5；不能把业务层 0 当作线上中位。MCU 按偏移变化计数，上位机重试同一目标保持编号，重启后通过 A6 同步。协议编码器透传有限速度；任一速度为 NaN/Inf 时两轴归零，上层运动许可和限速仍生效。

MotionLink 在业务提交时立即发送，默认每 40 ms 重发；命令超过 150 ms 或许可撤销时替换为零速度，保持机构目标。首次提交前不发送，退出补发零速度。转向仍由 `wz` 请求，上位机不发送相对角度控制请求。

金向量（零速度、相机中位）：

~~~text
开夹爪、编号1：56 00 00 00 00 00 00 00 00 05 01 00 00 9B 71
关夹爪、编号2：56 00 00 00 00 00 00 00 00 FB 02 00 00 5A 99
~~~

`tools/send_velocity.py` 默认 `latest15`，仅预览，`--send` 被既有工具策略拒绝；旧 `legacy6` 仅用于明确指定的六字节预览。`--yaw`、`--target-angle` 和旧 `latest17/latest19` 选项已移除。方框、俯仰、demo 和相机控制台均使用相同 15 字节布局。

### MCU 反馈：A6 固定 8 字节

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | uint8 | `0xA6` |
| 1 | uint8 | 状态/完成位；当前 frame.assume、frame.gripper 按动作应答解释，1 表示完成，不能推断开闭 |
| 2 | uint8 | 动作编号 |
| 3..4 | int16 | 相机绝对舵机角，90..150° |
| 5..6 | uint16 | CRC16/Modbus，覆盖 0..4，低字节在前 |
| 7 | uint8 | 帧尾 `0x0A` |

当前使用 `a6_semantics=done_flag`：开、关都可回 1，开闭目标须结合已知状态记录、动作编号及操作者确认。其他配置若采用位置状态映射，按该配置解释，不能把历史 0关/1开直接套用到当前入口。A6 状态与 TX 有符号偏移不能混用；状态、编号匹配不独立证明夹持或交付成功。解析按帧头、定长、CRC、帧尾进行，不能按换行切包。默认反馈超过 200 ms 失效；非法角度转换为 `-32768`。

主程序和标定内部角度是偏移百分之一度：`pitch_cdeg=(A6绝对角−120)×100`，例如 125° 对应 500；发送时除以 100 并限幅 ±30°。俯仰工具使用绝对角，详见各工具 `--help`。

### IMU 保留在上位机内部

IMU 端口使用当前 CP2102N 的固定 USB 序列号链接，USB 重连导致 ttyUSB 编号变化时无需修改配置。更换 IMU USB 转串口设备后须更新该链接；此修改不提供运行中断线自动恢复。

HI91 独立接 `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_7a068e3dffefef1180ad4f5a6d9880ab-if00-port0`，不在 A6 中。`HipnucImu::snapshot()` 提供原始及车体姿态、样本序号、新鲜度和数值有效性，供观测、导航、健康判断及遥测使用。

当前安装补偿为绕 Y 轴 180°：设备到 `base_link`（X 前、Y 左、Z 上）使用 `diag(-1,+1,-1)`；加速度/角速度变为 `(-x,+y,-z)`，四元数按 `q_body=q_device⊗(0,0,1,0)`，再提取姿态。不要额外叠加设备内部旋转补偿，也不要把几何模块的 x右/y前坐标与 base_link 直接混用。

默认 200 ms 无推进样本即过期；原始状态字、CRC、序号和数据年龄保留用于诊断。IMU 接收有效不等于安装和动态姿态已经实机验收。

<details>
<summary>HI91 原始包参考</summary>

小端 82 字节：`5A A5` 帧头、uint16 载荷长度 76、uint16 CRC、类型 `0x91`、状态、温度、气压、设备时间、加速度、角速度、磁场、欧拉角及四元数。CRC16/XMODEM 初值 0、多项式 `0x1021`，覆盖 0..3 和 6..81，跳过 CRC 本身。

| 偏移 | 数据 |
| --- | --- |
| 6、7..8、9 | 类型、uint16 状态、int8 温度 |
| 10..13、14..17 | float32 气压 Pa、uint32 设备时间 ms |
| 18..29、30..41、42..53 | float32[3] 加速度 g、角速度 °/s、磁场 µT |
| 54..65、66..81 | float32[3] roll/pitch/yaw °、float32[4] 四元数 w/x/y/z |

实现和异常处理见 `src/hardware/hipnuc_imu.cpp`，回归见 `tests/hardware/imu_tests.cpp`。

</details>

<a id="calibration"></a>
## 夹取与相机标定

### 左右夹爪候选与三档重叠标定（2026-10-27）

实测夹爪候选为 config/project.json#/frame/gripper：净宽90 mm，夹持前后边界180/70 mm，闭爪目标中心100 mm，张开外宽240 mm；同轴、两侧合拢，可容纳左右或前后排列的多个物体。物体尺寸与车体包络沿用原配置；满载几何中心为125 mm，与100 mm闭爪触发点不是同一个量。A6为动作应答，开闭各等待100 ms；重复同目标沿用原动作编号。闭爪补推保留。

候选使用UI 90/120/140（内部-3000/0/2000），140档单视角连续帧核验完整夹持清单；120档只有半个夹爪，不作完整清单证据。旧35/40度多边形不能复用。当前正常入口尚未切换到这套机械尺寸候选；现行 UI120/140 像素流程见下节。先完成下述标定，再使用 bash tools/match/match_main.sh --gripper --check --frame open。--gripper --run同样沿用原入口的现场确认要求；未完成标定时静态检查会拒绝，不发送动作。

离线标定入口不打开相机或串口，不执行机构动作：

~~~bash
python3 tools/camera_calibration/overlap.py init --output /tmp/gripper-overlap-points.json
python3 tools/camera_calibration/overlap.py solve --input /tmp/gripper-overlap-points.json --base config/camera.yaml --output /tmp/gripper-overlap-result --max-error-mm <按停止余量确定的误差上限>
~~~

填写原图像素：每组机器人保持静止，同一地面标记在90/120或120/140图中的一一对应；建议每对6–12个分散的非共线点。另在每档填写至少3个未参与拟合、尺量过机器人坐标的验证点，包含100 mm闭爪附近范围。填写140档夹内多边形和实测停止余量。先去畸变，再以90档既有米制标定传递两组单应矩阵，不使用PnP。输出独立验证报告与候选camera-overlap.yaml，不自动覆盖当前标定或启用实机。

地面距离直接使用各档重叠单应矩阵；现有物体高度/安全区栏顶校正使用该矩阵与内参分解得到的离散外参，也不调用PnP。仍需验证栏顶和真实目标，不把地面验证视作全部实机验收。当前config/camera.yaml已关闭可选PnP交叉检查。

待完成：将验证后的文件放入config/camera-overlap.yaml，将真实140档多边形填入frame.gripper.frame_views，测量停止余量并完成对应确认。保留的40 mm最大物体深度在100 mm触发点与70 mm后边界之间只剩10 mm余量，旧15 mm余量不适用，不能直接改小并声称验证通过。

### 图像触发位置参考（2026-10-28）

用户新增的140°三张截图按最前边界、正确位置、最后边界保存于 calibration_runs/gripper_image_20261028/reference_screenshots。原始检测坐标为1280×720：前边界框[541,99,193,254]，正确位置框[541,210,213,254]，后边界框[546,377,244,224]；检测框中心分别约为(637.5,226)、(647.5,337)、(668,489) px。坐标取截图检测表，不能用界面缩放后的像素替代。

frame.gripper.image_trigger_reference 指向版本管理内的 config/gripper-image-reference.json。运行 python3 tools/camera_calibration/image_trigger.py 可离线复核三张参考；也可使用 --bbox 541 210 213 254 --image-size 1280 720 --pitch-ui-deg 140 检查一个检测框。仅判断前后位置及相对正确位置的像素差，无距离换算和PnP，不连接相机、串口或控制机构。

这三张是单个居中普通物块的静态样本，中心Y的226–489只代表该样本的前后位置参考；337是正确位置参考线，尚不是经过动态停车验证的控制阈值。普通物块左右最大容许范围已于2026-10-30补齐，见下文；左右/前后多物块示例和动态停车仍待确认。不得将整个检测框包络当作夹持区域，也不得把所有检测框中心落在该区间的物体判为可闭爪。

2026-10-28 新增伤员和核心物块的前/后/左/右最大夹取边界，原始截图保存于同一 reference_screenshots 目录，配置记录原始文件名、SHA-256 和检测坐标。沿用上述 UI 140°、1280×720 视角，使用检测框中心且边界包含等号：伤员 X=395–906、Y=236–479 px；核心 X=268.5–1017、Y=236–555 px。这些是四个轴向边界样本，角点组合未单独采样。离线工具支持 --label injured_person 或 --label core_supply；正确位置参考线继续复用普通物块 Y=337 px，并非新增的实测停车位置。

2026-10-30 新增普通物资左右最大夹取边界，截图保存于 calibration_runs/gripper_image_20261030/reference_screenshots。左边界检测框[206,253,272,262]，中心(342,384)；右边界检测框[895,263,272,282]，中心(1031,404)。沿用UI140、1280×720原图检测框中心口径，补齐普通物资X342–1031、Y226–489 px（含边界）；目标参考点仍为(647.5,337)。配置保留原始文件名、SHA-256及检测坐标。

2026-10-31 用户确认18:59切分后框内普通物资仍应重新选取，并要求左右一致。按1280宽原图中线X640镜像，将普通物资运行中心区域从X342–1031改为X209–1071 px，覆盖该轮中心X1061–1071及左侧对称位置；Y226–489与目标(647.5,337)不变。此为用户确认后的运行区域调整，原始左右截图、框坐标和历史中心仍保留；伤员/核心范围不改，危险物禁抓及其他运动/复核条件继续生效。

2026-10-31 补充用户确认的八个不可夹取位置，截图及原图检测框坐标保存在 `calibration_runs/gripper_image_20261031/reference_screenshots` 和 `config/gripper-image-reference.json` 的 `non_graspable_samples`。其检测框中心依次为 (93,365.5)、(250.5,145.5)、(435.5,85.5)、(595.5,85.5)、(725.5,105.5)、(925.5,78)、(1210.5,305.5)、(1111,145.5)，全部在已有三类中心范围外；用户确认这些外侧贴爪位置对其他种类同样不可夹取。普通物资中心可夹运行范围为 X209–1071、Y226–489，伤员 X395–906、Y236–479，核心 X268.5–1017、Y236–555，均包含边界；范围外为当前位置不可夹取，范围内也仍须满足目标资格、对正、停车、新鲜图像及闭前/闭后复核。

实际像素参考新增 `membership_feature=bbox_center_xy`：稳定UI140下，普通/核心/伤员库存和闭后持有计数使用各自已测中心范围，不再把检测框角部触碰中心标定矩形当作已夹入。危险物没有抓取资格，其可能夹入区取这三类中心范围的并集；区内仍按危险物拒绝，区外不因外接矩形角部相交产生危险库存。接近通道与当前目标的区域仍独立，按中心进入现有通道判断；“不可夹取”不等于可以无条件前进、转动或闭爪。低置信度/unknown保留整框保守规则；相关物体缺失、身份不明、当前图像过期或相机未稳仍阻断复核。该设置采用用户确认的同类位置推广，不是新完成的米制或闭爪扫掠标定。未启用该成员坐标口径的旧配置仍使用原整框及5%/2px边界复核；下文旧整框规则仅适用于该旧配置。


主程序 frame.assume 和 frame.gripper 读取同一图像参考文件；runtime_enabled=true 时，正常入口使用UI120/140像素夹取流程，不要求这两档地面投影参数。UI90保留远景接近，中心Y≥540 px后先停止前进，在UI90水平对中到画面中心±40 px，经过制动和稳定帧确认后再停车切UI120；即使框碰到下沿导致地面几何无效，也允许原地对中后切视角。UI120根据中心对准画面中央并以近距速度接近，中心Y≥540 px才停车切UI140。540是720高画面下四分之一的视角交接策略，尚待实机验证，不是夹取边界或地面标定；UI140稳定后重新锁定、开爪并核对图像走廊，再以现有近距速度前进。三类物块中心Y达到337 px、满足各自已标定前后与左右边界时停车闭爪，横向对齐容差为独立的12 px，按下文“像素对齐控制”的制动和稳定窗口确认，发出闭爪前再次复核。未检测到目标、过期帧或转动中的相机不产生前进/闭爪依据，超时退出；不使用米制距离、行程耗尽或目标消失代替闭爪条件。

普通物块使用中心范围X209–1071、Y226–489 px；伤员、核心使用各自前后与左右边界。图像走廊按已测边界的横向包络计入可见邻居，并保留首趟、危险物禁夹、混装和伤员独运规则。闭爪动作应答后，UI140用连续稳定检测中心和类别清单核验夹持；UI120不提供夹持成功证据。此流程不伪造geometry_valid或米制距离；已配置的受控场地许可用于像素接近，远景导航和交付仍保留原有几何要求。

有效配置的task.pixel_grasp_enabled、capture.pixel_grasp_enabled、capture.grasp_center_ranges显示启用状态、原图边界和目标中心；遥测capture_position_source=bbox_center_pixels及pixel_grasp_ready可核对实际判定。geometry_reason=pitch_not_calibrated仍如实说明该角度没有地面投影，但不再阻塞120/140像素接近和夹取。337 px来自普通物块静态正确位置样本，动态停车效果仍需低速实机验证。本次软件验证不运行机构；上节--gripper候选的机械尺寸验收与当前像素方案是独立事项。

### 像素对齐控制（2026-10-28）

Startup 开爪接近的 UI120→UI140 交接（2026-11-05）：普通物资的新鲜画面达到中心Y≥画高75%时，先提交停车交接。提交后不再要求原目标ID存在、同物重绑定或继续对中；夹爪开到位应答、发送速度停止、稳定UI120及新鲜场景满足原配置的停车稳定时长和帧数后切UI140。停车交接独立限时1.5秒，失败报 startup_handoff_stop_timeout。UI140优先选择普通物资标定夹取框内的合格物块；框内没有合格物块时，允许从现有普通物资观察区（横向和后边界沿用配置，上沿Y180）选择完整、可选的普通物资。连续三个不同采图时刻存在合格候选即可交接，不要求与原物块同一身份，期间换ID不重置确认；夹取框内候选优先于仅在观察区内的候选，同优先级保留已确认中的候选。无合格候选仍在1.5秒后报 startup_handoff_region_timeout。重新选择不代表已进入夹取区：继续留在Startup，沿用UI140对正、低速接近及Y337停车闭爪线；观察区其他普通物资交由抓后的实际持有检验，不作为抓前数量分离条件。非普通物资风险、标定夹取范围、运动净空和闭爪到位检查保持有效。日志 observation_candidates 和各候选 observation 字段区分观察区候选。无设备回归：./build-release/rescue_task_phase_tests --startup-region-handoff、--startup-open-handoff、--ordinary-handoff、--startup-observation-handoff。

Startup 选择普通物资、开爪前的居中容差沿用 `pixel_align_approach_tolerance_px`（当前±30 px，UI120/UI140一致），允许日志中的16/26 px偏差在停车确认后开爪，不再单独要求±10 px。UI120近中心修正沿用100ms短脉冲，转速取原半增益结果并限制在0.06～0.1 rad/s（仍受配置最大转速限制）。这一短脉冲不被单帧误差差分的预测制动提前截断；进入相应阶段的对齐容差、越过中心、目标/相机/发送反馈异常仍按现有流程停车。粗对齐和UI140继续使用原预测制动；停车新鲜帧及稳定确认要求保持原值。回归覆盖150ms响应延迟、快慢两种图像响应、双向误差和10 px检测抖动，以及居中后跨过旧超时点的开爪接近；这些模拟结果不替代实机耗时测量。

Startup UI120 整堆对准修复（2026-11-05，14:09:23日志）：整堆近中心纠偏沿用普通物资的有效短脉冲规则，转速取半增益并限制在0.06～0.1 rad/s（仍受配置上限限制），100ms从反馈中已发送的同向有效角速度开始计算。单帧检测框抖动造成的误差改善不提前结束该脉冲；进入原±30px容差、越过中线或运动反馈异常仍立即制动。原停车新鲜图像确认、40px重新纠偏门限、3秒无进展复看及6秒整堆对准上限继续生效。无硬件回归 `build-release/rescue_task_revision_tests startup` 覆盖双向抖动、串口爬坡、150ms图像响应延迟、快慢响应下持续前进及左右下角触发后的停车交接；未执行实机动作。

UI120 对齐画面中心，UI140 对齐各类别已有标定中心。参数集中在
config/gripper-image-reference.json 的 pixel_alignment（首轮初始值，尚未实机标定）：
比例增益维持 0.004 rad/s/px、转速上限 0.20 rad/s。UI120 使用独立粗对齐参数：
pixel_align_approach_tolerance_px=30、pixel_align_approach_restart_px=40；进入 ±30 px 后停止转向，
制动、稳定确认和前进均保持滞回，超过 ±40 px 才重新纠偏。UI140 保留 ±12 px 精对齐，
前进时允许 ±24 px，准备及闭爪时仍须 ±12 px。视角交接由中心 Y≥画面高度的 75% 触发；UI120 对同一目标开始停车交接后保持该意图，中心在阈值附近抖动不再取消交接或重置稳定窗口。仍须通过停车、新鲜图像及漂移稳定确认才切 UI140；目标丢失、过期、身份或视角变化、相机不稳定仍重置对齐状态。
越过中心或进入对齐范围先制动，发送速度达到 |vx|≤0.01 m/s、|wz|≤0.02 rad/s 后，
只接受此停车时刻之后的新图像：至少 3 张，覆盖 pixel_align_brake_us=150000，窗口误差
最大值与最小值之差不超过 pixel_align_brake_motion_px=30，才允许重新纠偏或开始稳定确认。
漂移超限重新计时；重复图像不推进窗口；发送速度回升重新等待停车。
随后在当前视角保持范围内至少 3 张新图像、持续 200 ms 才允许前进。
发送速度并非实测轮速，图像稳定是视觉代理证据；上述阈值仍待实机验收。
重复图像不累计确认，检测失效、目标/类别/视角变化或停止会重置对齐状态。
UI140 到达闭爪线后重新确认 ±12 px 和停车条件，原抓取边界及闭爪前复核仍生效。
遥测新增 pixel_grasp_error_px、pixel_grasp_sample_us、pixel_align_state 和
pixel_align_stable_frames；状态为 idle / correct / brake / settle / advance。
上述像素对齐阈值尚未完成实机运动验收。

### 相机—IMU 旋转外参：固定板、多轴运动 AX=XB

现在外参采集可使用网页的 handeye 模式。保留原有内参和地面测量入口；本模式只求相机光学系到原始 IMU 设备系的旋转，不把未知平移设为零，也不自动覆盖 config/camera.yaml。

~~~bash
cd /home/cat/ds_innovation
python3 tools/camera_calibration/guided.py --mode handeye \
  --intrinsics config/camera.yaml --imu-port /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_7a068e3dffefef1180ad4f5a6d9880ab-if00-port0 --imu-baud 115200 \
  --stop-preview
~~~

浏览器打开 http://<当前板端地址>:8081/。默认 1280×720，必须与内参一致；Foxglove 仍为 8766，沿用 /calibration/image、/calibration/status、/calibration/result。仅预览不记录 IMU；点击开始后连续记录，程序不发送底盘、相机舵机或其他机构动作。

1. 将实物 9×6 圆点板固定竖立在距相机约 1 m 处；填写实测横向 8 间距和纵向 5 间距。板太小或识别失败时使用更大板或调整距离，不能假定 1 m 一定满足像素要求。
2. 相机与 IMU 刚性连接，保持焦点、相机舵机角度固定。使用稳定支撑，人工整体改变偏航、俯仰、横滚；板保持不动，不使用舵机代替整体俯仰。不具备多轴动作条件时无法完成本模式。
3. 开始后整体静止至少 5 秒，第一张估计陀螺仪零偏。之后按网页指引转动约 10°～30°，每次停稳约 2 秒再拍摄。两张间隔不超过 45 秒。允许回到之前姿态；相邻两张至少转动 5°，相邻转角不超过 80°。
4. 共 25 张：前 17 张拟合，后 8 张组成独立验证序列，拟合与验证之间不构造交叉运动对。全部样本都记录图像时间戳、圆心、原图以及完整 IMU JSONL，撤销照片时仍保留原记录。
5. 自动检查零偏静止段、IMU 数值和序号、设备时间回绕/倒退/100 ms 断流、两路转角一致性、PnP 重投影与单轴退化；对称圆点倒序按姿态连续性处理。拟合/验证分别要求至少 6 段有效转动，旋转轴第二/第一奇异值比至少 0.15。
6. 默认残差检查为拟合及验证最大 2°、验证中位数 1°；它们是软件初始质量门限，不代表实机精度保证。记录过程使用陀螺仪原始 imu_device 轴、rad/s，积分采用设备时钟间隔；图像使用读取完成的主机单调时间，停稳端点降低时间差影响。--camera-time-offset-ms 将固定毫秒偏移加到图像时间，不自动估计延迟；若存在同步问题，应修正后重采。

输出目录为 calibration_runs/handeye_日期_时间_随机编号/，包含 intrinsics.yaml 快照、imu.jsonl、imu_stderr.log、samples/、detected/、逐帧 JSON、session.json、camera_imu_rotation.yaml、report.json 和使用说明。结果定义为 v_imu_device = R_imu_from_camera · v_camera_optical；同时保存逆矩阵 R_camera_from_imu，translation_estimated=0、extrinsics_validated=0。有告警时状态为 needs_recapture，否则仍为 candidate_requires_validation，需要重复采集和实机独立验证。

~~~bash
# 保留原始记录，用当前求解器重算完整会话；模式自动从 session.json 识别
python3 tools/camera_calibration/guided.py --recalculate calibration_runs/<handeye会话目录>
# 单独检查只读 IMU 记录入口；不连接 MCU
./build-release/rescue_imu_record --help
ctest --preset cat-release -L calibration
~~~

主程序的 T_camera_from_robot、ground_homography 需要平移、地面关系及适用俯仰范围，本结果不能直接替代它们。相机舵机改变角度后 R_imu_from_camera 也会改变，必须在同一机构角度使用或另行建模。项目已确认 IMU→base_link FLU 为 diag(-1,1,-1)；转换车体旋转时可左乘，但地面几何的 robot 坐标约定需另外核对，不混用 FLU 与 x右/y前。

### 内参与地面标定：9×6 对称圆点板

1. 固定镜头焦点、分辨率和相机安装姿态；避免其他预览占用摄像头。
2. 在板端启动 `python3 tools/camera_calibration/guided.py --stop-preview`，打开 `http://<当前板子IP>:8081/`；圆点板地址为 `/board.pdf`。默认 1280×720，须与实际主程序一致。
3. 平板显示完整圆点板，锁定缩放、亮度和旋转。量横向第 1 至第 9 圆心跨度、纵向第 1 至第 6 圆心跨度，单位 mm；分别除以 8 和 5 得间距。两方向差异超过 1% 时先修正显示或测量。
4. 按网页完成 25 个姿态，停稳后保存原始 PNG；前 22 张拟合，后 3 张独立验证。采集期间不能改变焦点、分辨率或图案缩放。
5. 检查 `report.json` 和去畸变结果。`intrinsics.yaml` 只有内参，不可直接当完整 `camera.yaml` 使用。
6. 将圆点板水平放在地面，测量其相对车体的位置，拍照求外参，再用板外独立地面点验收。
7. 通过后备份原相机配置，部署正式 `camera.yaml`，在所用 profile 的 `calibration.calibration_file` 中明确选择，并用配置导出核对。

网页采集结果在 `calibration_runs/circles_日期_时间_随机编号/`，含原图、检测图、会话、内参与报告；中断后照片保留，半途会话不自动恢复。完整会话可用 `guided.py --recalculate <会话目录>` 重算。

标定 Foxglove 为 `ws://<当前板子IP>:8766`，通道 `/calibration/image`、`/calibration/status`、`/calibration/result`；拍摄由网页操作，Foxglove 只读。退出标定后按“主程序只读预览”重新启动查看。

<details>
<summary>外参、地面检查与角度范围</summary>

几何坐标为 x右/y前/z上，平移单位 m；相机坐标为 x图像右/y图像下/z光轴前。测量参数：

| 参数 | 要求 |
| --- | --- |
| `--square-mm` | 实测圆心间距 mm |
| `--origin-x/--origin-y` | 编号 0 圆心相对底盘中心地面投影的位置 m |
| `--yaw-deg` | 板 +X（0→1）从车体 +x 向 +y 的转角 |
| `--board-height-mm` | 圆点表面离地高度，包含垫板 |
| `--camera-pitch-cdeg` | 当前相机偏移百分之一度，例如 A6 125° 填 500 |

先保存原分辨率图像并计算候选。以下尖括号均须替换为自己的文件和实测值：

~~~bash
python3 tools/camera_calibration/calibrate.py snapshot --camera /dev/video0 --width 1280 --height 720 --output ground_board.png
python3 tools/camera_calibration/calibrate.py extrinsics \
  --pattern circles --cols 9 --rows 6 \
  --intrinsics <会话目录>/intrinsics.yaml --image ground_board.png \
  --square-mm <间距mm> --origin-x <x米> --origin-y <y米> \
  --yaw-deg <方向角> --board-height-mm <表面高度mm> \
  --camera-pitch-cdeg <偏移百分之一度> --output <新的候选目录>
~~~

核对 `corners_numbered.png` 的编号 0、0→1 和 0→9 是否符合测量基准。可用 `--flip-cols/--flip-rows` 调整编号；当板 +Y 朝车后时使用 `--reverse-board-y`。改变编号时同时复核坐标。

在板外实测至少 3 个分布于近远左右的地面检查点，创建 `world.json`（`fit: []`，`check` 中每项含实测 `world_m: [x,y]`），在原图上选点：

~~~bash
python3 tools/camera_calibration/calibrate.py pick --image ground_board.png --world world.json --output checks.json
~~~

使用原候选的同一组参数和新输出目录重新执行 `extrinsics`，增加 `--confirm-order --check-points checks.json --max-error-mm 20`。只有方向确认、位姿检查及独立误差通过才产生正式 `camera.yaml`；否则保留候选。默认重投影 RMS 门槛 0.5 px，地面点最大误差 20 mm；小 RMS 不替代独立测量。

单一姿态标定只适用于其验证角度。扩展范围须在其他俯仰角重新量取独立地面点，用 `calibrate.py pitch-range --camera <已验收camera.yaml> --check <偏移cdeg>:<checks.json> --output <新目录>` 验证；可重复 `--check`，默认相邻验证角间隔不超过 15°。旧 40° 结果不能作为当前 30° 的标定。

手工平面映射备用流程为 `snapshot → pick → ground`，至少 6 个拟合点与 3 个独立检查点。棋盘内参备用入口为 `calibrate.py intrinsics`；全部参数以对应子命令 `--help` 为准。

同一相机的焦点、分辨率、安装位置或俯仰改变后应复核适用范围；标定工具不会自动覆盖当前生效配置。

</details>

<a id="tools"></a>
## 工具与排错

| 工具 | 用途 |
| --- | --- |
| `tools/match/` | 正式比赛、只读 MCU 状态、遥测桥管理 |
| `tools/demo/demo.py` | recognize/search/carry_once 等分项入口，读取 demo profile |
| `tools/frame/frame_ctl.py` | open/close/status；up/down 为兼容别名 |
| `tools/gripper/` | 方框工具的旧名称兼容入口 |
| `tools/camera_pitch/pitch_ctl.py` | 相机俯仰，保持反馈中的夹爪状态 |
| `tools/remote_debug/camera_angle_console.py` | 零底盘速度的相机控制台，仍会操作相机机构 |
| `tools/send_velocity.py` | 15 字节包预览，不打开串口 |
| `tools/board_monitor/monitor.py` | 板上浏览器监看；`install_desktop.py` 安装桌面入口 |
| `tools/runtime/record_baseline.py` | 冻结行为基线记录与比较 |

方框工具的 `status` 与 `--dry-run` 只读串口；执行 open/close 时速度为零，但机构会动作。它保持首次新鲜 A6 相机角，要求有效状态和编号，超时停止重发且不反向动作。相机/方框工具与主程序须独占串口。传入 `--help` 查看用法不会执行动作。

包预览示例：

~~~bash
python3 tools/send_velocity.py --protocol latest15 --vx 0.1 --wz -0.25 --frame-offset 0 --pitch 120
~~~

| 现象 | 检查方向 |
| --- | --- |
| SSH 名称失效 | 检查当前 SSH 别名和网络解析，连接后核对用户与主机 |
| 编译缺依赖 | 检查板端预设、OpenCV/nlohmann、RKNN 头和库；工具测试需 cv2/numpy/websockets |
| 刚编译却像旧程序 | 查看比赛入口 identity、`RESCUE_BINARY` 和实际构建目录 |
| 相机/串口被占用 | 查询原服务和进程，正常停止后使用；不同时启动多个读取器 |
| 无 A6 或 CRC 失败 | 核对 115200/8N1、A6 定长 8 字节和 TX 定长 15 字节；反馈状态与发送偏移分开 |
| pitch_not_calibrated | 核对 A6 绝对角→内部偏移换算、实际选中的 YAML 和已验收角度范围 |
| 网页有旧图/IMU失效 | 查主程序及桥日志、frame_age、IMU age、连接和有效标志 |
| 修改参数没变化 | 导出最终配置，检查所选片段、CLI 和后续兼容策略 |
| 冻结回归失败 | 先检查资源摘要、历史夹具和差异，不能重录预期消除失败 |

<a id="development"></a>
## 开发参考

### 测试组织与格式

`tests/CMakeLists.txt` 统一注册测试。`task/stages` 为单阶段，`task/integration` 为阶段接入，`task/regression` 为跨阶段；其余按 perception、geometry、hardware、config、control、telemetry、tools、regression 分类。`fixtures`、`samples` 保留冻结数据与样本。工具本地测试继续位于对应工具目录。

`rescue_push_tests` 保留统一入口和场景顺序，共享模拟器位于 `tests/support/push_task_sim.hpp`。新增测试登记源码、依赖、标签与超时；主题相近不代表测试重复。

<a id="formatting"></a>
C/C++ 使用 `.clang-format`，固定 `clang-format 18.1.8`：4 空格、100 列、Attach 大括号、不排序 include。工具优先选择 `.tools/venv/bin/clang-format`。

~~~bash
# 缺少固定版本工具时安装
python3 -m venv .tools/venv
.tools/venv/bin/python -m pip install clang-format==18.1.8
# 检查；需要格式修复时用 --fix
python3 tools/format_code.py --check
cmake --build build-release --target format-check
~~~

普通构建不会重写源码；格式工具仅处理 src/include/tests/foxglove_telemetry/src 的 C/C++，不改 Python、Shell、配置或第三方文件。每次项目修改在板端编译并运行相关验证。

<details>
<summary>独立 Foxglove / MCAP 工程</summary>

主程序使用共享内存快照和 Python 桥，无需此独立 SDK publisher。需要独立协议检查时：

~~~bash
cmake -S foxglove_telemetry -B build/foxglove_telemetry
cmake --build build/foxglove_telemetry -j2
./build/foxglove_telemetry/rescue_telemetry_smoke
~~~

构建真实 publisher 时增加 `-DRESCUE_BUILD_FOXGLOVE_PUBLISHER=ON -DFOXGLOVE_SDK_DIST=<Foxglove SDK cpp/dist>`，程序为 `rescue_foxglove_publisher`。该工程使用 C++20；只读订阅，不接受客户端控制。

~~~bash
python3 -m venv .venv-mcap
. .venv-mcap/bin/activate
pip install -r foxglove_telemetry/tools/requirements.txt
python3 foxglove_telemetry/tools/generate_sample_mcap.py rescue_sample.mcap
~~~

生成的 MCAP 可由兼容客户端回放。上游依赖为 Foxglove SDK、MCAP Python writer，代码与协议入口保留在 `foxglove_telemetry/`。

</details>

### 当前限制与历史证据

当前配置含假设的机构映射；相机地面标定已由用户确认完成；识别/几何资格、机构、急停、底盘、捕获搬运和完整交付仍需按现场条件验收。软件回归只证明测试输入下的行为。旧报告中的主机异常、吞吐量和实测记录是历史状态，不直接作为当前结论。

仅保留以下三份不可编辑的原始测试证据，内容可能描述历史协议；**当前使用只看本文**：

- [ARM 基线证据](tests/fixtures/doc_evidence/CAT_HOST_20261020.md)
- [历史实车验证证据](tests/fixtures/doc_evidence/CAT_VEHICLE_ACCEPTANCE_20261024.md)
- [历史协议快照](tests/fixtures/doc_evidence/通信协议.md)

冻结证据保留原始字节和历史链接；它们不是现行操作手册。基线补充文件只改证据位置，不重录预期输出。比赛资料 `intelligence20260617.pdf`、标定图案 PDF、测试夹具和标定数据保留。

2026-10-26 本次整理前文件及校验清单位于板端项目外 `/home/cat/ds_protocol15_docs_20261026_183706/`。已移出的旧标定记录位置由 `calibration_runs/README.archive.txt` 指示。后续操作说明集中更新本文；验证日志留在项目外，不再新增逐轮报告文档。

### 后退旋转后的捕获（2026-10-30）

现行规则见“混合物块验收与局部分堆”：四类物块均参与验收，右下目标后退左转、左下目标后退右转；只有合法批次才能进入当前位置闭爪。日志scene_status、scene_detail、task_reason与split_before_objects可核对本轮库存及动作；pixel_split_ui120_left表示左转，first_trip_split_ui120_right表示右转。原first_trip前缀的状态名为兼容历史日志保留，也用于后续趟的像素分堆。

关联复核恢复：对准过程中，相关邻居丢失、截断或关联不明确时，先停车等待新的稳定近视角帧。满足现有确认帧数且停稳至少250ms后，进入已有UI120转向、UI140重新选目标和复核流程，不因“邻居消失”直接放行闭爪。转向继续使用现有方向净空检查；本次复核与恢复共用原始时间预算和分堆次数上限。恢复后仍须经过横向稳定、夹取范围、通道及闭爪前复核。日志新增scene_blocker_id、scene_blocker_box、scene_association，区分missing、clipped、ambiguous及identity_reused。

### 搜索 UI140 首块进框（2026-10-30）

UI90/UI120 沿用现有接近及视角切换。UI140 使用各类别已有可夹范围，
以检测框中心首次进入范围判断进框，不再选左右角区，不要求到 Y337 或居中。
记录第一个进入的物块 ID；所需目标先进入后立即停车，用现有新鲜图像稳定确认后复查库存。
若其他物块先进入，或第一张有效图像已有非本趟可选类别的物块在框内、无法区分先后，停车走现有分堆。
同时进入的物块若全部属于本趟可选类别，也可继续库存复查。后续其他物块同时在框内不抹去已观察到的首块顺序；数量、类别、整框邻块及通道检查继续决定能否闭框。
分堆后回 PREPARE 重新观察；捕获复查位置和库存，不重新居中接近。
独立 capture 保留自身核验。entry_target_id/label/first_id/stage/position_ok/confirmed/stable_frames
记录目标、首块及交接状态。重复图像不能完成停稳确认，相机切换与目标失效清除首块记录。
默认测试只使用模拟输入，不驱动底盘。

### 回转后的目标重选（2026-10-31）

局部分堆返回 UI120 接近时，先保持停车并暂停像素锁定；只有停车指令已发送、相机稳定、有有效航向时变化不超过 0.01rad，且新鲜场景帧覆盖至少 500ms 后，才重新开始三帧选目标确认。滑行、重复图像和相机切换不计入该观察窗口。

UI120 接近过程中，若连续 2.5s 未发出有效前进指令且目标发生至少三次丢失，或连续 4s 未发出有效前进指令，同时画面中存在完整可见、符合本趟类别规则、检测框面积至少为当前目标两倍的候选，则停车释放旧锁并重新观察选择。旧 ID 暂避 4s，原接近总超时预算继续累计；正常前进时保留原锁，缺少合适替代物块时不盲目换目标。新目标仍须经过原有视角交接、进框、库存和闭爪核验。

日志 `pixel_reselect_observe` / `pixel_reselect_ready` 表示停车观察及完成，`pixel_approach_reselect` 表示停滞后主动重选；`PIXEL_SELECTION selection_paused=1` 表示观察期间不接受锁定。回归使用 00:44:51 现场的 #28/#31 检测框，覆盖整场、独立 capture、正常前进保锁以及截断候选不替换；验证过程不驱动实机。

运输夹爪模式先按UI90新鲜本方区域方向靠近，不要求测距有效才起步；有可信区域几何后测量车体旋转中心到安全区入口边界的垂直距离，40cm停车。连续三个不同采集时刻的可信本方区域朝向，在IMU参考系内相差不超过0.06rad时，建立入口法向目标；使用采图时刻的IMU航向与区域相对角相加，避免推理延迟混入转角。接近时以该法向和横向偏差纠偏，转向时锁定目标，用当前IMU闭环对正至误差≤0.02rad。单帧朝向跳变不改写目标；入口对正时短暂丢点最多保留1.5秒的角度参考，期间只允许原地对正，不用旧姿态推算距离或授权入区。缺失同步IMU或可信几何时沿用原图像对正；IMU对正后连续三帧新鲜图像仍显示边线未对齐，则本趟禁用该几何法向并回退图像对正，后续新帧不能重新激活已拒绝的参考；新一趟CARRY才重置。参考过期后恢复原有等待/寻区流程。图像转向使用三帧边线斜率的中位数，锁定转速执行至多100ms后完整停车250ms，再收集停车窗口后的三帧新图像；单帧抖动不能打断停车或立即反向微调。边线对齐后若目标分区偏差超过画面宽度1%或中心线不在分区内，有新鲜几何距离时执行原有后退重定位，保持图像确认的法向重新接近；无测距则停车等待。切UI120仍需三帧居中且边线中位数对齐，IMU单独不能完成此确认。随后保持该航向前进，本方地面颜色占整幅画面≥70%立即停车，确认已发送零速250ms后开爪；蓝队只认蓝色、红队只认红色。其他显式进入的推板颜色投放流程保留。

推板地面停车参数位于 config/gripper-image-reference.json 的 pushboard_floor：1280×720原图左窗[410,530,75,45]、右窗[790,530,75,45]，其他分辨率等比例缩放；只在有效稳定UI140反馈下取样。HSV区分白地、横向紫带、本方蓝/红地；旁边紫色不参与。剔除非安全区检测框（各物块类别包括同地面颜色物块）及绿色普通物资、橙色伤员、深色核心前景像素。两窗有效可见面积均至少60%、可识别地面至少60%，本方颜色均至少75%时立即停车；至少3个不同新鲜图像、连续200 ms满足，且已发送零速度持续250 ms后，才进入开爪。未知像素计入颜色占比分母，敌方色超过10%不提供有效地面。
重复帧不能累计确认，图像年龄超过200 ms不使用，新帧间隔超过150 ms重置连续确认；遮挡/未知/单侧到位不作为完成。观测无效时停车保爪等待，预算耗尽为deposit_floor_timeout；有效且尚未到位时低速推进。画面显示FLOOR L/R窗口、本方色占比和可见比例，日志[PUSHBOARD_FLOOR]记录各色比例。初始窗口与阈值已用用户2026-10-31两张白地/蓝地截图验证，红区及三类物块完整入区对应的物理余量仍需实机标定；截图不是运动验收。关闭pushboard_floor.enabled时仍保留旧FAR视角IMU前进0.15 m流程，不能把该兼容流程当作颜色模式的后备完成条件。


夹爪 carry 投放顺序（2026-11-01）：先按UI90新鲜本方区域方向靠近，再由可信且本方身份确认的非预测区域几何提供入口距离和法向，车体旋转中心距入口40cm停车、IMU对齐入口法向，切UI120后沿该航向前进。本方地面颜色占全画面≥70%立即停车；物块检测框和前景颜色不算地面，遮挡及未知像素仍计入全画面分母；相机不稳、过期或无效图像不提供前进或投放依据。停稳250ms后开爪并等待动作应答，保持开爪使用独立ImuTranslation后退0.15m。后退前采集至少250ms静止样本校零，设备时间积分水平加速度，发送速度仅作许可、不作距离；许可受阻或零发送不累计漂移。后退达到0.15m并停稳250ms后完成运输，直接进入Search扫描，不再执行投放后180°掉头或旧离区转向。缺少可信入口几何时，有新鲜本方区域方向则先按方向接近；方向缺失、过期、非有限值或只有敌方区域时停车后寻区。不使用bbox尺寸猜测40cm，方向接近本身不触发开爪；IMU、航向、反馈无效时停车。

普通物资的夹爪 carry 送达核验使用本方颜色安全区检测框完整包含普通物资检测框（四条边全部在内，重合边界允许）；两个框必须来自同一新鲜图像，且相机稳定。开爪反馈完成、底盘已发送零速度后，仅累计后退结束后的独立图像，沿用 delivery_frames 和至少 300 ms 的稳定确认。包含的普通物资数必须达到本趟数量；敌方框、部分重叠、过期图像、仅地面坐标/前沿估计均不替代此证据。超时记录 delivery_unverified，不按 assume_all_safe 自动计入本趟；成功记录 delivered_box_contained。框包含仅用于送达核验，不赋予运动许可。日志 retreat_stop_imu_calibrate、retreat_wait_reverse_sent、jaw_back_out_015m_imu / jaw_backed_out_015m_imu 标记后退校零、等待发送及行程。

分堆后若 UI140 新鲜静止复核未见可选物块，或返回 UI120 后未锁定目标，保持开爪在 UI120 原地自转搜索；发现并锁定物块后先制动，再按原像素对准、接近与抓取流程执行。搜索使用现有分堆时间预算，接近预算从发现目标后开始；UI120 短暂失框期间停车等待选择器确认，目标锁释放后在 UI120 重新搜索，再次发现目标仍先制动；UI140 交接与近区抓取仍沿用原流程。重搜沿用本轮首次搜索的截止时间，不因反复锁定/丢失而续期；转向受阻、图像过期或视角未稳定时停车。

运输图像接近（2026-11-01）：先按UI90新鲜本方区域方向接近，无需测距起步；获得可信区域几何后40cm停车/IMU垂直对齐，交接UI120全画面本方地面颜色70%停车；不再使用UI90近满屏、bbox中心Y75%或UI140两窗颜色作为该运输分支的投放条件。

分堆独立复核（2026-10-31）：UI140 分堆计数范围保留 Y180 到普通物资后界及已标定横界，分堆候选与复核不得缩成夹取范围。邻块中心离开夹取范围、但仍在分堆计数范围内时，继续按现有批次规则复核；未通过不能闭爪或验收成功。后退净空、重选和转后复核使用相同分堆范围；仍保留新鲜帧、停车确认、原分堆次数和时间上限。夹取位置与实际持有数量继续使用各类别夹取范围，不把分堆范围内的远处物块计作已抓取。


运输UI120纠偏策略已由FAR单视角运输替代（2026-10-31）：运输不再调用物资UI120/UI140像素对准与交接。物资搜索/启动/抓取自身的像素阈值、预测制动、短脉冲及视角交接保留。
运输使用40cm入口几何距离、IMU法向对齐和UI120整幅画面本方颜色70%判据，保留停车及开爪应答；不使用30/40px横向验收、12px近区验收或近满屏替代测距。
CONTROL_CHANGE补充carry_align、carry_error_px、carry_center_px、carry_sample_us、carry_tolerance_px、carry_acceptance_px、carry_brake_reason、carry_stable_frames；运输对准期间align_stage来自carry_alignment_，不再误报idle。原error_px/center_px仍是物资输入字段，判断运输须使用carry_字段。



转后UI140重选时，若可选目标均已离开独立分堆库存区，画面边缘裁切不再重置返回接近的停车确认。仍须稳定近视角、有效航向、新鲜独立图像及已发送零速度达到现有确认帧数并持续250 ms，才停车切回UI120重新搜索/接近；区域内不完整框或危险物pending仍等待。该切换不代表分堆验收或抓取成功，返回后仍执行原UI120停车观察、选目标及UI140闭爪前完整复核；原分堆次数和时间预算保留。

普通分堆已取消额外500 ms后退：split_margin_budget_us默认0。视觉净空后仅执行原有至少250 ms停车及独立新鲜帧确认，再切UI120转向、返回UI140复核；不再追加后退及其二次停车阶段。原视觉退让速度、上角动作、转后裁切恢复、分堆范围、次数和总预算保持。

分堆后退时，未确认净空的干扰物连续漏检3个独立新图像帧后，才触发原有停车及重观处理。前两帧沿用该物块最后位置继续原退让，不将漏检视为净空；重复帧不累计，重新检测或唯一同类近邻关联成功后清零连续漏检计数。裁切、关联歧义、无效/过期画面及原运动许可门控保持原处理。


运输分区联合对齐（2026-11-01）：40cm停车距离保留。图像居中指画面的竖直中心线对准本趟对应物资/伤员半区，不要求画面二维中心点落地；直接使用本方半区四个新鲜图像关键点，不需要新增相机标定，也不用整区检测框中心代替。已有几何负责入口40cm测距和法向；40cm线外用短前视点收敛横向位置。若停车后仍偏区，先停车，在原退让/转向净空许可下转正并退至入口外65cm，再重新接近，不增加横移命令。原25秒运输预算仍生效。
停车后物理250ms零发送计时与视觉确认分开。新鲜可信入口法向锁存IMU目标；几何短暂失效不打断已锁存的IMU对齐，位置重调仍需可信几何。入口航向误差不超过0.035rad、图像横向偏差不超过画宽1%、图像估计横向修正不超过2cm且中心线落入对应半区，三个独立新鲜帧及停稳确认同时通过才切UI120。图像横向米值仅按观测半区宽度估算，用于调整，不替代原几何测距；最终中心线验收使用图像坐标。
UI120保持已确认分区中线与IMU航向；有新鲜关键点时复核对应半区，发现偏区回UI90重对齐；近距离关键点出画不新增停车等待或重搜门槛，终点继续使用原本方地面颜色判据；本方颜色70%不能覆盖错区判定。保留开爪应答和IMU后退15cm，停稳后完成运输并直接进入Search，不再执行投放后180°掉头。诊断输出[CARRY_SUBZONE]包含分区、横向/yaw/画面误差、中心线落区与确认帧数。

运输偏区转向修正（2026-11-03）：40cm停车后，若对应半区横向误差超过画宽1%，且入口边缘转正方向会使目标继续远离画面中线，优先沿当前视线朝向后退至原65cm重调距离，再联合接近；不因图像横向估计与几何横向估计相差超过3cm而绕过该处理。停车入口及三帧静止复核后均检查这一方向冲突，重调时清除旧图像转向目标。缺少新鲜对应半区测距时停车等待，不盲退；原转向、后退净空及40cm距离/投放验收保留。


### 安全区关键点一致性与叠图
模型关键点语义为围栏内沿。公共分隔角仅在双方预测相距不超过半区宽度5%且不超过12原图像素时合并；距离图像边缘不超过2像素及画外点不进入在线几何计算。其余点沿用正式几何流程的间距检查与鲁棒拟合，点不足时不生成可信位姿。
青色框仅投影正式流程接受的当前帧位姿，不再独立拟合全部点；无有效当前帧几何时显示失败原因。橙色叉表示未被该位姿接受的原始模型预测（画外点在边缘标记），紫色保留模型关键点。PnP及标定参数未改变。


### 安全区位姿使用地面映射（2026-11-01）
当前camera.yaml设置enable_zone_pnp_primary=0、enable_pnp_crosscheck=0，安全区位姿解算和投影均不使用PnP。YOLO-pose提供围栏内沿关键点及对应半区图像居中观测；地面映射与正式鲁棒几何拟合负责入口40cm距离和入口法向，锁存后由IMU执行航向闭环。青色框显示正式接受的当前帧地面几何结果，保留关键点一致性与边缘点筛选。UI120仍使用原图像居中复核及颜色终点流程，未新增标定。


下角分堆后退更新：左右下角均以需要保留的目标物块检测框中心 Y≤270 px 为后退结束条件，不再要求干扰物退出分堆区域或抓取通道。保持原后退速度、250 ms停稳及新鲜帧确认、左右转向和转后完整抓取复核。目标连续漏检3帧、裁切或关联歧义仍停车重观；上角仍观察干扰物并执行原右转90度、推20cm、返回及左转80度。

运输pose丢失恢复（2026-11-01）：仅在UI90接近阶段，对应半区局部对齐信息不可用、但存在新鲜本方安全区方向时启动；无需此前已获得局部信息。底盘无横移接口，安全区在图像左侧就先左转、右侧就先右转，朝向安全区后停车采集至少250ms IMU静止样本校零，再沿锁定航向前进0.15m（独立IMU位移积分，不以命令时间估距）。保持夹爪关闭，到距停车并停稳250ms后重新判断：局部信息仍不可用但本方安全区仍可见，则按最新方向开始下一段0.15m；局部信息恢复则停稳确认三帧后返回40cm接近流程；安全区不可见则沿用原地搜索。已进入入口停车、图像对齐或重定位阶段时，局部信息缺失只停车等待，不用本恢复抢占对齐。前进中的方向失效、IMU/航向/发送反馈异常及运动许可受阻均停车。UI120已确认后的关键点丢失仍沿用原颜色终点。保留运输整体超时。

运输丢点恢复（2026-11-01更新）：UI90接近、入口停车和图像对齐阶段丢失半区关键点时，按当时本方安全区左右方向，锁定自身当前IMU航向加/减90度；完成转动并停稳校零后沿锁定航向前进0.15m（IMU积分），停车后再观察。期间关键点重现或检测框方向变化不抢占本段动作，安全区离开画面不取消已锁定动作。仍执行运动净空、IMU及反馈检查。

运输丢点恢复方向修正（2026-11-03）：zone_sight_bearing使用车体转向符号，正值向左、负值向右，恢复分支不再额外镜像取反；正bearing对应自身当前IMU航向左转90度（加90度），负bearing对应右转45度（减45度）。首次恢复与后续重试均使用此角度，完成后仍按IMU积分前进0.15m。入口IMU对准、恢复及重定位航向修正的原地转向上限使用min(scan_wz,max_wz)，比赛启动参数当前为0.6 rad/s，不再受夹取pixel_align_max_wz=0.2的限制；接近目标仍按角差比例减速，图像短脉冲保持原精细限幅。

运输执行顺序（当前，2026-11-03更新）：安全区入口前20cm必须完成对应半区及法向对准；UI90接近到入口外35cm时开始最终对准。实际发送零速至少250ms后，用三个独立新鲜目标半区观测确认，三帧横向误差均须严格小于画宽7%，中心线均在目标半区内；边线斜率绝对值统一按不超过0.07验收（7%高差/横向跨度，包含等于门限）。确认通过后锁定当前IMU航向，直接切UI120，相机稳定且地面颜色图像有效后以0.06m/s向区内推入。删除停车后的前移横向修正、修正后再次确认、确认后继续UI90接近20cm和检测框中心Y560/720交接；超出横向门限时按下述联合纠偏及不可达处理执行，不执行停车后前移修正。本方地面颜色占全幅≥70%立即停车，随后沿用停稳250ms、开爪应答、后退15cm及交付核验。检测框只提供找区方向，没有入口测距时不得据此前进；相机不稳、无效或过期颜色图像时停车。UI120保留7%错区复核门限及IMU航向保持。本段替代历史条目中的2.5%/4%横向确认、前移修正及UI90检测框位置交接流程。


运输找区与缺角转向（当前，2026-11-03，14:10日志）：己方区域的短暂丢失停车窗口须由连续三个不同采集时刻的己方观测建立，单帧或重复帧不能刷新1.5秒窗口；连续三个新鲜帧仅见敌方区域且无可信己方测量时取消该窗口，继续原地搜索。UI90接近中选定半区短暂缺角时，先等待已发送前进速度归零，再允许最多600ms沿最近该半区的采图时IMU方向原地转向；当前IMU扣除已转角，不能切换到整区框或复用旧前进速度，方向过期/航向无效/净空受阻仍停止本次保持。日志以 carry_selected_half_turn_hold 标记该转向。图像对齐中的新鲜同方向边线可连续延长有界IMU转角目标，使用采图时航向补偿推理延迟，减少每转0.1rad即停稳重拍；接近目标、方向变化或观测失效后仍制动，最终保留实际零速250ms和三个独立新鲜图像确认。入口35cm预对准和20cm界限保留；最终横向门限及UI120交接以“运输执行顺序（当前）”为准。本段替代历史条目中半区丢失时复用前进速度和逐段图像转向必停的描述。

运输视角交接更新（2026-11-03，当前）：最终停稳三帧确认通过后，直接从UI90切换UI120并向区内推入；不再等待本方检测框中心Y达到560/720，不再保留切换前的UI90前进段。相机稳定且颜色图像有效后，以0.06m/s前进，保持IMU航向及原错区复核；本方地面颜色占全幅≥70%立即停车，随后沿用停稳250ms、开爪应答及交付流程。

运输联合纠偏补充（2026-11-03）：接近阶段将入口法向误差和横向误差合成同一条前进转向指令，合成航向误差超过0.75rad才先原地转向。40cm停车后，若新鲜图像与可信几何一致确认横向偏差超过2cm（两者相差不超过3cm），不再先原地转正；先保持当前观察航向直退到入口外65cm，再联合接近，最后完成法向与三帧图像验收。退让航向独立于入口法向保存，避免将尚未转正的航向当成法向；退让仍须新鲜图像、可信测距及后退净空许可。

运输目标半区确认（2026-11-03）：本趟运输物资只检查物资半区，运输伤员只检查伤员半区。丢点恢复结束时仅要求对应半区的新鲜有效图像、本方身份、停稳至少250ms及连续三帧确认，不再要求另一半区同时可见；仅另一半区可见不能替代本趟目标。入口图像对齐继续使用同一目标半区。


运输接近连续性修复（2026-11-03，11:47运行日志）：对应半区图像可用但入口投影闪断或法向跳变时，继续以同一半区为转向目标，不切到整区检测框。几何横向/法向联合纠偏须有连续三个不同采集时刻、IMU参考系内一致的入口法向；接近阶段保留大角差的稳定法向采样，原地精对准的采样门槛不变。已形成的接近目标只在原帧超时范围内保持IMU参考航向，保持不会刷新原采集时间，且仍需新鲜半区、可信本方测量或本方区域方向授权；不缓存或外推距离、不提高最近一次许可速度，新的40cm距离仍优先停车。目标半区短暂消失且角度参考不可用时停车等待最多1.5秒，避免逐帧切换到反向整区目标。入口停车/对齐阶段缺少半区及可用法向时，必须实际发送零速并连续观察1.5秒后才允许原45/90度恢复；观测恢复、发送非零速或反馈无效均重置缺失计时。40cm门槛、三帧最终对齐、UI120颜色70%与停稳开爪流程保留。CARRY_SUBZONE增加normal_frames、normal_error_rad，CONTROL_CHANGE用carry_approach_selected_half、carry_approach_heading_hold、carry_selected_half_wait区分图像纠偏、短时航向保持和丢点停车。


运输新鲜半区观测优先级修复（2026-11-03，12:21日志）：接近阶段已有新鲜对应半区图像时，入口法向暂时不一致或投影闪断不得继续执行旧的 approach_heading。日志中曾出现对应半区误差为 -5.25%、-8.83%、-10.90% 时仍由 carry_approach_heading_hold 请求 -0.2 rad/s，把已接近中线的目标再次转偏，继而反复重定位。现在新鲜半区优先重新计算纠偏，图像居中且无法建立一致法向时不延续旧转向；仅对应半区图像丢失时，按原帧超时和新鲜本方证据短时保持航向。40cm测距停车、入口250ms停稳/三帧确认、UI120颜色70%及停稳开爪条件保留。回归用例覆盖物资/伤员两半区、左右方向、越过中线与恰好居中；原发送器闭环与完整运输生命周期一并验证。


运输对齐效率调整（2026-11-03，12:29日志）：本轮日志有103次后退命令段，反复穿插极小航向修正和等待。入口仍在可信几何测得40cm时停车，并保留250ms实际零速和三个独立新鲜图像的确认。对应半区包含画面中心线时，横向误差放宽至画宽4%，边线斜率容差0.035；小偏差无需反复精调。半区内横向误差不超过12%、边线斜率不超过0.12时，允许在UI90阶段边前进边转向：修正越大，前进速度越低，转向随当前图像误差变化；修正期间失去对应半区图像则停车，不靠旧图像盲走。修正回容差后保持当前IMU航向，沿用本方检测框中心Y560/720的UI120交接。UI120复核使用7%回差，中心线离开对应半区仍立即停止并回UI90；颜色终点、停稳开爪及15cm交付后退保持原流程。
必要重定位不再一律退到65cm：按实际横向修正量选择入口外48～65cm的目标；后退航向容差由0.008rad放宽到0.06rad，避免亚角度波动打断后退；首次达到目标即锁存制动，测距回跳不重新倒车。后退仍需新鲜可信测距和后方净空，原地转向、前进弧线仍受现有净空许可约束。日志 carry_entry_forward_correction 标记前进纠偏；CARRY_SUBZONE新增 acceptance_fraction、forward_correcting、retreat_target_m、retreat_braking。新说明替代以上旧条目中的1%验收、固定65cm退距和极小后退航向容差。


运输停车对准循环修复（2026-11-03，15:20日志）：35cm线外使用对应半区入口坐标中的横向位置与入口法向构造到停车位的三次曲线，终点横向斜率为零，联合收敛位置和朝向；航向采用采图时法向与当前IMU参考，图像居中仍独立负责最终验收。按轨迹曲率同时限制线速度与角速度，弧线净空受阻时停车，不退化为继续直行。35cm停车、20cm前完成对准、实际零速250ms、三个独立新鲜目标半区观测、严格小于7%的图像误差保留，边线斜率按下述7%规则验收。斜率超出7%且三帧持续偏区、转正会进一步偏区时，以 carry_entry_alignment_unreachable 进入故障停车；斜率已合格但中心偏移时先原地居中复核。不可达故障保持夹爪及相机角度，单阶段入口报告失败并退出，不再在 EntryStop/AlignStop 间零速循环。混合合格/不合格帧重新采样，缺少新鲜证据不判定为不可达。图像诊断的 lateral_m 保留，新增 pose_lateral_m 与 parking_remaining_m 区分入口坐标横向误差和距35cm停车位的剩余距离。本段替代历史的“偏区后一直停车等待自动恢复”，不恢复停车后前移或后退重调。UI120颜色终点、开爪应答、15cm后退及交付核验流程不变。


运输入口7%容差与居中修正（当前，2026-11-03，17:13日志）：17:13日志中的中心误差6.17%、边线斜率5.75%已确认能够送入安全区，UI90最终对齐的边线斜率绝对值统一放宽为≤0.07，不再区分入口外0.02与近入口0.035；沿用三个独立新鲜帧边线斜率的中位数抗单帧抖动。三帧中心误差仍须均严格小于画宽7%，且画面中心线均在本趟目标半区内。边线合格而中心持续偏右/偏左时，保持前进速度为零，分别右转/左转居中，不再仅因中心偏移就报不可达。居中与修正边线使用独立用途标记，复用有效转速下限、采图时IMU参考及每段最多0.1rad的目标；居中进入范围、越过中线或新图像斜率超7%时先刹车，重新等待实际零速250ms及三个新鲜图像验收。图像两项均合格后锁定当前IMU航向，位姿法向分歧不单独否决入区。图像缺失仅沿用现有600ms有界IMU转向保持，过期或转向净空受阻则停车；原运输预算及UI120颜色终点、开爪/后退流程保留。本段替代历史“边线已平但持续偏区即故障退出”的处理。

### UI120 运输结束判定：对应半区整体轮廓（2026-11-03）

近点不再用全图本方颜色 70%，也不依赖近点关键点模型。UI90 完成对应半区对齐后，UI120
使用独立 HSV 范围提取本方地面，排除物资检测框；蓝色范围避开紫色围边和暗色中间隔板。
从覆盖车体图像中线的地面连通区域中选择最大区域，使用外轮廓的凸包填补内部物资遮挡。
不同连通半区不会拼接；已完全遮挡、无法从可见地面恢复的外边界不外推。

config/gripper-image-reference.json 的 pushboard_floor.transport_region 控制阈值和 HSV。
初始阈值为图像宽度的 65%、高度的 55%、区域下边缘达到图像高度的 95%，三项必须同时满足。
这些是根据用户认可的近点画面设置的初值，需要通过未送入/刚送入的实车画面对照调整。
同一配置可通过 --dump-config 核对，不使用像素绝对尺寸，支持预览分辨率变化。

UI120 推入速度为 0.06 m/s；首次满足后仍以 0.06 m/s 及原转向速度继续前进 0.1 s，再停车；发送速度确认停稳 250 ms 后，必须再取得至少三张不同时间戳的新图，
且每张仍满足区域条件，才进入开爪。失效或过期观测会清空确认，重复帧不累计。
近点关键点丢失或偏移不再把此过程拉回 UI90 对齐；开爪反馈、后退和后续验证流程保留。
预览叠加 UI120 HALF W/H/B 轮廓，日志 [TRANSPORT_REGION] 记录尺寸、阈值和判定，
own_fraction_diagnostic 仅供诊断，不触发释放。

无动作验证命令：

    cmake --build --preset release
    build-release/rescue_pushboard_floor_tests --ui120-reference tests/samples/ui120-left-half.jpg
    build-release/rescue_carry_task_integration_tests --ui120-region
    tools/match/match_main.sh --check --phase transport --frame keep --known-frame-angle 20 --known-frame-id 3

运输初始接近连续性更新（2026-11-05，13:39/13:44日志）：对应半区入口已有已确认轨迹、航向及发送反馈有效且净空许可满足时，短暂半区/测距丢失或测距跳变沿该轨迹保持最多500ms；超过200ms未获得接受的入口测量后，前进速度上限降为0.03m/s。保持窗口只由接受的新采集帧刷新，预测、重复帧及被拒收的测量不能续期；5帧短暂缺失不直接启动接近阶段的重捕获。预测入口必须仍在35cm停车线外至少5cm，新鲜近入口测量仍可停车；持续丢失超时后按原流程停车、寻找对应半区并停稳确认。新测量相对经IMU航向和已发送行程推进的入口轨迹检查位置跳变，采图时入口先补偿至当前航向，再平滑融合到轨迹；静止近入口确认和最终UI120推送流程保持现有要求。回归覆盖两半区、正常接近/前进重定位、重复400ms观测丢失、360ms位置跳变、轨迹过期、净空/航向/发送反馈异常和新鲜近入口停车；这些是无硬件模拟验证，实机连续性需用后续运行日志确认。

运输释放后退更新（2026-11-04，替代2026-11-03的速度积分方案）：区域达标后以0.06m/s继续前进0.3s，再执行停稳及三帧确认。开爪应答后保持释放时相机角度，先停车并采集至少250ms静止IMU样本校零，使用独立ImuTranslation按设备时间将水平加速度积分为速度和位移，后退目标仍为0.15m，速度上限仍为现有back_speed（现场0.4m/s）。发送速度仅用于确认已下发后退，不能提供位移；零发送时不累计漂移，缺少IMU或航向/发送反馈时不得用速度积分替代。达到IMU后退距离后立即停车，确认发送速度停稳250ms后完成运输。暂停保留已测后退距离，恢复后重新静止校零。此后直接进入Search扫描，删除投放后180°掉头，不叠加旧离区转向。沿用已确认释放后直退的净空豁免及现有运行安全停车条件。

运输入区调整更新（2026-11-03）：本段替代历史偏区后退及不可达处理。距离相对最近可信入口测量跳变超过10cm，须连续三个不同采图时刻均跳变才停车复核；前两帧沿用最近可信入口，不采用跳变距离切入停车阶段。复核三帧距离跨度不超过10cm后恢复。正常接近仍在入口前35cm停车。静止250ms后采集三帧，距离和横向位置跨度分别允许10cm；不稳定则保持停车、重新采集，不直接报不可达，阶段预算耗尽时报 carry_entry_measurement_unstable。有稳定对应半区测量、距35cm停车线仍多于5cm且横向偏差大于3cm时，重新接近；现有曲线不可行时，先转向目标半区，再以最高0.05m/s前进调整位置，保留转向/路径/弧线净空检查及停车线。中心和边线修正冲突且确无可用前进调整空间时，以 carry_entry_reposition_required 停车，当前不自动后退。入区中心与边线7%验收门限不变。

运输重定位丢点修复（2026-11-03，19:34日志）：Approach 的 forward_reposition 缺少目标半区入口测距时不再零速等待。禁止前进，优先按新鲜目标半区图像或己方区域方向原地转向；没有可用纠偏方向时持续原地搜索。清除旧距离稳定缓存及接近路线，重新获取半区测距后按当前入口几何恢复重定位；原有净空、急停和入口停车条件保留。

运输后退重定位（2026-11-05差速车修正）：停车线前不足5cm且无法完成原接近曲线时，进入独立的后退调整流程；前进重定位期间的图像误差增大不再单独触发后退。先实际零发送停稳250ms，以三个不同采图时刻的本趟目标半区入口确认距离/横向跨度不超过10cm、IMU参考法向相邻差不超过0.06rad。最终静止对齐发现图像与入口法向冲突时保持故障停车；已获准的几何后退流程不再按图像中心误差瞬时符号否决转正。转正时，锁定采图时IMU航向加入口法向，原地转正并再次停稳250ms，再沿该航向直线后退，速度不超过0.05m/s。目标入口距离为35cm加横向偏差绝对值的两倍，限定65～165cm；无横向偏差时至少退至65cm。后退每帧要求新鲜目标半区入口、有效IMU/发送反馈及后方净空；丢失即停车，不切换整区框寻向，不使用预测距离授权后退。航向偏差超过0.06rad先停车、重新对正后再退，不发送倒车弧线。到目标距离锁存制动，停稳250ms并取得三个新鲜半区观测后清除旧路线与重定位状态，重新执行接近。保持夹爪关闭、UI90、原35cm停车线、7%图像验收、UI120投放和运行入口原有运输预算。以上替代历史“禁止后退”的相关说明。


### UI140 近景候选选择与重选

搜索、抓取阶段在稳定且尺寸匹配的 UI140 中，复用各类别的中心点夹取标定与图像边缘完整性规则：优先框内完整候选，其次框外完整候选；截断候选不参与新锁定。只有同级候选才沿用面积及拥挤程度排序。远景和 UI120 规则保持原有流程。

已锁定的框内完整目标保持稳定。停车且夹爪确认打开、尚未交接或闭合时，较高优先级的新候选连续三帧确认后，可替换框外或截断的旧目标。`near_target_reselected` 与同物体跟踪 ID 重绑分开处理；任务清除旧目标的入框、对准和场景确认，保留原有执行预算，并重新检验混装和路径风险。截断旧目标没有可替换候选时，进入现有有界重新观察流程。

`PIXEL_SELECTION` 记录候选的 `inside`、`complete`、`priority`、`area`、`reason`，并记录 `reselected_from`。无设备回归入口：`ctest --test-dir build-release --output-on-failure -R '^rescue_near_target_selection_tests$'`。


### 保留伤员时剔除其他物块（2026-11-04）

得分决策选择伤员并开始分堆后，本趟固定只保留一名伤员，普通、核心、危险物和多余物块继续参与库存及通道验收。临时丢失伤员、换ID或重新搜索不会改选物资，分堆次数继续累计；正常动作沿用本次预算，放弃无法确认场景后的重搜按下述恢复规则重建观察计时。

稳定UI140确认需要分堆后，先按目标中心与该类标定框纵向中点比较（伤员当前X395–906、Y236–479，中点Y357.5 px）。中心Y小于中点的上方/前半部伤员，无论是否已进入抓取范围，均使用上角清干扰物流程；锁定框内及通道内非目标物块，按干扰物中心Y≤280后退并停稳，右转90度、按实际发送速度积分前推0.036 m、停稳后按本次前推积分配平退回，再左转80度并进入UI120搜索。无可信干扰物时只重新观察，不授权推送。中心在中点及下方且位于抓取范围内的伤员使用先转向、停稳再后退流程：以标定中心线X647.5及±12 px中间带划分，左侧和中间带先右转，右侧先左转；本轮锁定IMU相对转角目标0.15 rad。小角度转向速度上限0.15 rad/s，不叠加split_wz_increment_rps，按剩余角差减速，并按航向转速预留150 ms制动量。到达或越过目标先锁存制动；实际发送速度为零、航向转速≤0.03 rad/s且250 ms内总漂移≤0.005 rad后，角差≤0.02 rad才进入后退，否则停车后低速修正。暂停、IMU丢失及相机切换清除旧采样和停稳计时。后退阶段以新鲜稳定UI140中唯一完整伤员为参考，伤员中心Y≤270后停稳并复查。首次与重试均按最新停车场景决定动作，每轮执行中不切换分支。

观察失败的停车恢复（2026-11-05）：初始抓取或分堆阶段的目标缺失、关联不清、库存不完整、候选确认及重观察等待累计1.5 s，且实际零发送已持续250 ms时，清除旧目标、干扰物参考及场景关联，保持已决定的保留类别，重新进入UI120停车观察及搜索。观察导致的换角和重观察转动不重置该等待期限；新的观察尝试重置旧得分复核计时，避免回到搜索后立即被旧期限判停；观察等待期间即使旧复核预算先到期，也保持零速交由恢复流程处理。恢复时一并清除UI120→UI140旧交接状态，近景确认的新目标先由正常重选分支接收，避免旧锁定拦截新ID。近景交接缺少可确认目标1.5 s后同样返回UI120重新搜索。单阶段capture入口允许内部搜索，重新锁定后仍执行正常UI140混装、通道及闭框复核。UI120搜索窗口到期先零速重置观察窗口后继续搜索，不再因空视野触发pixel_split_search_timeout；净空、急停、相机到位和传感器有效性要求仍生效。日志pixel_split_observation_research、pixel_split_search_restart、pixel_search_handoff_research标识恢复；split_branch、split_target_center_y、split_midpoint_y、split_turn_goal/error/rate/braking/stopped_us标识分支与制动。无硬件回归：ctest --test-dir build-release --output-on-failure -R '^rescue_capture_(front-split-recovery|injured-removal|upper-corner-motion|corner-turn-brake|postsplit-search)$'。

搜索近景交接失败后的接近恢复（2026-11-05，21:08日志）：UI120→UI140切角时释放旧ID，由UI140独立选择完整、可选物块；原有夹取范围内优先、连续新鲜帧确认和库存/通道复核保持。若UI140没有可确认目标而超时返回UI120，本轮记住切角时的目标中心高度；下一次切角须看到目标中心Y至少增加现有pixel_align_brake_motion_px（当前30px，以画高比例保存），否则继续原有最高0.04m/s的对准和接近流程，不能因换ID或Y450触发线重复原地切角。过期画面不授权重试；成功交接、结束本趟或重新分堆时清除该进展要求。该要求不是前进授权，原停车观察、净空、相机稳定、接近预算及闭爪前核验继续生效。无硬件回归：ctest --test-dir build-release --output-on-failure -R 'rescue_search_ui120_(speed|handoff)_tests|rescue_capture_search-view-selection|rescue_capture_postsplit-search'。

分堆后若仍有混装、危险物或超量，按伤员最新位置开始下一轮；仅一名伤员且夹取位置与通道通过才允许进入闭框前核验。UI120重搜只选择伤员；闭框后仍核验实际持有清单。转向或组合动作完成后，得分复核保留类别决策和原始起始时间，等待实际停车至少250 ms及连续新鲜稳定UI140帧后建立新场景，不再要求转后物块逐一匹配转前像素位置与旧ID。同一观察阶段内仍保留丢失、截断、未知类别及关联歧义检查，新场景建立本身不代表分堆成功。新增日志injured_split_turn_left/right、injured_split_turn_brake、injured_split_switch_ui140_retreat、injured_split_retreat_stopped与capture_score_new_view_confirm_wait可定位阶段。角度、位移和剔除效果尚须实机验证。

运输完成判定更新（2026-11-04，当前实机夹爪运输流程）：删除释放、后退后的交付图像验收，不再要求检测框完整落区、交付数量变化或额外确认窗口；开爪应答和IMU后退15cm完成并停稳后直接进入Search扫描，不再执行原IMU净转180度或旧离区转向。单阶段入口按动作流程是否完成决定退出码，不再在开始下一趟清零计数后比较 expected/credited，也不再输出该阶段的 accepted 判定。抓取确认、入区对准、释放应答及运动安全停车条件保留；运输累计数量表示已执行运输的货物数。本段替代历史条目中当前实机流程的“交付核验”要求。

并行推理内存修复（2026-11-04）：板端优化构建的内存检查在主程序静态图片入口复现了 std::async / future 结果交接中的栈越界。检测与关键点模型仍并行运行，改用显式线程和返回前 join；检测或关键点推理异常时也先结束工作线程，再向调用者抛出异常，避免共享图像与结果对象提前销毁。增加实际 RKNN 模型的主程序入口、连续推理/销毁和异常传播回归测试；测试不打开相机、串口或 IMU。


固定动作后的首块跟进修复（2026-11-04，21:20:55 日志）：UI120 开爪接近离开前进容差后，从本次误差重新建立纠偏进展基准；连续三个新鲜图像误差均改善至少 10px 才刷新三秒进展窗口。普通物资纠偏连续三秒无有效进展时，以 startup_alignment_reselect 释放当前目标并继续选择，任务保持 RUNNING；清除旧视觉参考和对准状态，失败ID在一秒内跳过。旧目标丢失且有新鲜、可选的其他普通物资时，等待300ms后释放重选；没有可用替代时沿用1.5秒找回窗口，到期继续寻物。重选沿用已有减速和新目标对准流程，保持夹爪状态；同物换ID仍保留当前纠偏计时。无硬件回归：build-release/rescue_task_revision_tests ordinary-reselect。UI120 普通物资选择和开爪接近均使用 0.06～0.10rad/s 细调短脉冲，100ms 预算从已下发有效同向角速度开始计算。切换 UI140 后按普通物资标定框内的完整、可选物块重新选择，连续三个新鲜画面存在合格物块即可交接，不比较原目标身份和相邻物资顺序；换ID不重置区域确认，无合格物块超时报 startup_handoff_region_timeout。后续库存、危险物、闭爪及抓取验证照常执行。

上角分堆指令积分与恢复（2026-11-05）：本动作前推目标为实际发送速度累计积分 0.036 m（3.6 cm 指令量），返程按后退指令积分与本次前推指令积分配平；接近两端时降速，换向及转回前停稳至少 250 ms。直线动作不依赖 IMU 平移积分，不因其漂移或样本缺失触发 pixel_split_corner_return_position_uncertain / pixel_split_corner_imu_gap；右转90度及左转80度仍使用 IMU 航向。配平后转回并进入 UI120 搜索、重新选择目标；本次上角转向、前推或返程超时也清除旧目标和分堆 pending 状态，重新进入 UI120 搜索，不锁死为 FAULT。split_command_forward_m / split_command_reverse_m 始终记录本动作指令积分，不能解释为实测行程。仅修改上角分堆，运输释放等其他后退流程及净空、航向、发送反馈安全条件保留。

### 2026-11-04 日志问题修正

- 开局 UI120 物堆参考保留同组物块的整体图像运动，新增/丢失成员造成的中心变化逐帧限制；
  原始检测框、场景数量和角点证据不使用该滤波值。
- 物堆虚拟目标对准连续 3 秒无改善时停车复看一次，单次对准总预算 6 秒；
  跟踪编号变化和短时前进不能刷新预算，持续对准前进 300 ms 后结束本次对准。
- 像素运输保留捕获时的本趟载荷清单。运输途中不做额外物块审查或绕行规划；
  只在最终对准完成、即将向安全区执行UI120前进之前，停车使用UI130审查一次。
  按正前方物块类别判断能否送入目标半区，并检查首趟普通物资数量上限，不要求对应旧编号、旧位置或旧载荷数量；
  发现禁止送入的类别或首趟普通物资超量时，后退0.10m并按趟类型转向10°后继续推送。正常路径不再因转向、2秒或2cm重复审查。
- 首次普通物资交付后，搜索和装载共用 chooseTrip 的局部每趟收益：
  普通 5、核心 10、单伤员 15，同分倾向伤员；不同位置的物块不能合计成一趟。
  已接受且仍可见的锁定继续保留，首次交付前仍只选普通物资。

### 推送前一次审查及后退转向推送（2026-11-05）

删除运输路径障碍20°绕行规划。最终入口对准完成后，在首次UI120向安全区前进前停车切UI130（俯仰偏移1000，即向下10°），
等待实际发送零速250ms，以三张独立新鲜图像检查正前方是否有不能送入目标半区的物块。
正前方沿用既有横向像素范围的整高UI130图像带，仅作审查策略，不作为UI130夹取标定，按检测框中心判断；不要求生成夹取区或前进通道，
不匹配旧载荷编号、位置、数量或完整框。物资趟允许普通物资，以及首次普通物资交付后的核心物资；
伤员趟允许伤员。危险物、未知类别、另一半区的物块及首趟核心物资禁止送入。
首趟（尚未完成首次普通物资交付）还统计当前审查图像带内的普通物资：最多1个；
框内已有1个、车前另有1个时，禁止直接推送，触发下述后退转向动作。
该计数只使用当前图像，不再叠加旧载荷数量，不要求完整检测框；图像带外的物块不计入。
没有禁止物块且首趟数量未超限时（包括仅见需要的物块或当前无检测框）切回UI120并等待相机稳定，沿已对准航向执行原入区推送，
不在运输途中或后续推送中重复审查；图像缺失、过期、相机未稳或缺少横向范围仍等待，3秒未通过则停车。

发现正前方有禁止送入目标半区的物块或首趟普通物资超量时，保持闭爪和本趟载荷，
使用UI120，停车并采集250ms静止IMU样本校零，以最高0.05m/s沿当前航向直退0.10m。
后退距离由IMU积分确认，发送速度和固定时间不替代实测距离；到距后停车并等待实际发送零速250ms。
避让触发后，校零、后退及后退停稳阶段不再因相对触发时的航向变化超限而停车；仍使用IMU航向完成后续相对10°转向。
转向侧由本次UI130审查图像中的禁止物块横向位置决定：以所有当前禁止检测框的最左、最右边缘为整体范围，
比较该范围到推送审查通道左右边界的像素间距，向剩余横向空间较大的一侧转10°；物块偏左时右转，偏右时左转。
两侧间距相同时沿用原逻辑：物资趟（普通或核心物资）右转，伤员趟左转。
方向在审查通过三帧后锁定，后退中物块消失、画面变化或暂停恢复不重选方向；转角从后退停稳时的IMU航向计算。
该横向比较使用图像像素，不将UI130无效地面投影当作米制距离。
转向角由IMU航向确认，停稳后把最终推送的航向保持目标更新为该新航向，继续原UI120颜色终点推送。
此次处理不收集车前物块、不开爪侧放、不清空本趟载荷、不返回Search，也不重复推送前审查。
正常到达安全区后的开爪交付及退车流程保留；单运输阶段仍在正常运输动作完成后退出。
急停、相机反馈、IMU及原有转向净空约束继续生效；后退转向流程保留45秒动作预算。
暂停恢复时保留已测后退距离和转向目标，剩余后退重新静止校零后执行。
日志使用carry_avoid_back_calibrate / carry_avoid_back_010m / carry_avoid_back_stop /
carry_avoid_turn_10 / carry_avoid_complete；TRANSPORT_AVOID记录选定转向侧、横向像素偏移、左右剩余间距、是否等距，以及0.1m后退和10°转角。

无动作回归：`ctest --preset release --output-on-failure -R 'rescue_revision_(transport|reject)|rescue_transport_inspected_lifecycle'`。


运输摆位姿态保持（2026-11-05，01:08:32日志）：首次后退前，中心误差5.87386%、边线斜率-0.040404已进入原7%图像容差。在现有最小后退目标距离65cm内，前进摆位一旦达到该容差且画面中心线在本趟半区内，立即刹停，沿用实际发送零速250ms及三个独立新鲜采图确认，再进入UI120颜色终点推进；不再为满足入口位姿曲线而继续转过已可送入的姿态。该段相邻帧中心误差变化最大0.02592753、边线斜率变化最大0.0434783，分别向上取0.03（画宽3个百分点，1280像素下38.4px）与0.05作为图像判断保持带。新帧相对首次接受姿态的两项差值均不超过保持带且中心线仍在本半区内时，保持已接受判断；累计漂移超过任一保持带、半区图像失效/过期或中心线离开半区则撤销保持并重新复核。保持基准不随小幅抖动逐帧移动，不冻结测距、净空或传感器安全判断。日志CARRY_ENTRY_HOLD及CARRY_SUBZONE的posture_held/centre_hold_delta/edge_hold_delta记录该决策。本段仅调整运输摆位转入最终图像确认的时机及其保持判断。


运输差速重定位（2026-11-05，01:58:33日志）：图像防偏离判断仅用于最终静止对齐，不再用于接近路线复核、正在执行的前进重定位或停稳后恢复接近的路线选择。差速车允许先原地转向，再前进取得横向位移，横向偏差收敛到3cm内后恢复原接近曲线并回正；重定位中间的画面中心误差暂时增大或越过7%不会直接切换后退。前进摆位仍要求有效IMU与有限的实际发送速度反馈；短暂半区丢失沿用原有新鲜路线保持/五帧寻回机制，持续测量跳变仍停车复核，前进、旋转、弧线、后退净空及运输总预算继续生效。35cm停车线、最终7%中心/边线验收及现有近距离可送入姿态保持条件不变。CARRY_SUBZONE增加reposition字段以区分重定位与普通接近。无实机动作回归：build-release/rescue_carry_task_integration_tests --diff-drive-reposition，覆盖日志瞬时冲突、初始/静止路线选择、左右镜像及物资/伤员半区的真实发送斜坡差速运动、缺失反馈和通道阻塞。

运输近入口原地对齐（2026-11-05，06:06:49日志）：接近路线静止复核判定前进曲线不可行、且距35cm停车线剩余不超过5cm时，改为停稳后原地按本趟半区图像中心与边线对齐，不再由该判断触发退至65cm重规划。到达35cm停车线同样锁定原地对齐；后续测距变化不能重新启动前进摆位或旧的15cm位移恢复。已建立的有效前进路线继续沿用原接近规则。实际发送零速至少250ms及三个独立新鲜半区图像仍是切UI120推进的条件，原7%中心/边线验收及可送入姿态保持规则继续生效。半区连续缺失五帧或观测过期时，制动后原地定向寻找；局部三段搜索结束后继续按新鲜本方区域方向转向，缺少方向时连续旋转扫描，日志原因为carry_entry_half_search。恢复不要求入口测距，三个新鲜半区图像及停稳确认后返回AlignStop重新对齐，不直接前进。持续转向空间阻塞、传感器无效、无法同时满足图像对齐条件或运输总预算耗尽仍受现有停车/故障约束；总预算耗尽报告carry_entry_alignment_timeout并立即发送零速。本段替代历史近入口因停车空间不足自动后退及局部搜索结束后原地等待的说明。无实机动作回归：`build-release/rescue_carry_task_integration_tests --near-entry-turn`，覆盖37.7cm日志场景、两种半区与左右镜像、实际发送零速、重复/过期图像、时间戳冻结、持续搜索和恢复。


运输半区寻回连续性（2026-11-05，12:34:01日志）：本轮37.3秒控制记录中，半区寻回转向与刹停累计20.4秒；相机采集超时结束本轮，未完成交付。接近阶段的寻回判定改用本趟选定半区的新鲜图像，不再把入口投影失败等同于半区丢失。连续三个不同采图时刻看到半区即提交停车交接；已有的零速250ms及停车后三帧确认继续生效。入口测距持续缺失或隔帧闪断不会反复启动寻区，交接后按原规则等待入口测量，不能凭图像恢复前进。实际半区丢失、过期、错误半区、净空和传感器条件仍按原流程处理。无设备回归：`build-release/rescue_carry_task_integration_tests --visual-half-recovery`，覆盖物资/伤员半区、左右镜像、缺失/间歇测距、重复帧不得确认及测距恢复后继续接近。


### 2026-11-05 A_INT8 模型接入记录

已添加 `models/a_int8_20261105/A_INT8_confidence_fixed_EXPERIMENTAL.rknn` 的主程序四输出双分支解码兼容，并完成RK3588 NPU实跑。使用 `--pose-model models/a_int8_20261105/A_INT8_confidence_fixed_EXPERIMENTAL.rknn` 可选择候选模型；离线图片模式需加 `--calibration '' --detect-image <图片> --no-show`。

该四路文件当时虽然可加载，板端关键点与SDK参考严重不一致，近景多点落在(0,0)，所以当时保持原默认；当前默认已由下节 raw-XY 修复版替换，业务验收仍未完成。详细证据和复现命令见 `telemetry_logs/model-import-20261105/BOARD_INTEGRATION_REPORT.md`，校验工具为 `tools/model_import/verify_a_int8.py`。


运输相对姿态容差更新（2026-11-05，13:19截图）：按用户确认的两种可送入姿态，UI90最终图像验收的中心误差由严格小于画宽7%放宽至严格小于12%，边线斜率绝对值由≤0.07放宽至≤0.30；覆盖截图附近中心误差约0.94%/7.54%、斜率约0.137/0.261，以及13:18:13因中心误差约9.99%、斜率0.110而报carry_entry_reposition_required的场景。前进摆位的可送入姿态刹停、最终静止验收和转向结束条件统一使用新门限。画面中心线仍须在本趟目标半区内；若误差已小于12%但中心线仍在半区外，继续原地居中，不提前结束纠偏。保留实际发送零速250ms、三个独立新鲜半区观测、测距稳定性、IMU、净空、UI120颜色终点及开爪后退流程；图像已合格时不因位姿法向分歧继续精调。CARRY_SUBZONE输出acceptance_fraction=0.12及edge_acceptance=0.3。本段替代此前最终中心/边线7%门限的说明。无实机动作回归：`build-release/rescue_carry_task_integration_tests --relaxed-entry`；完整运输回归覆盖12%中心及0.30边线边界、物资/伤员、左右镜像、过期/重复图像、未停稳与半区外拒绝。

运输横向验收阈值收紧（2026-11-05）：最终中心误差由严格小于画宽12%改为严格小于10%（acceptance_fraction=0.10）；边线斜率绝对值仍为≤0.30。该中心门限同时用于前进摆位姿态刹停、静止验收和居中纠偏结束判断，替代上段12%中心门限；其余流程保持不变。

### 2026-11-05 raw-XY 修复版替换

默认 pose 已切换到 `models/a_rawxy_20261105/A_rawxy_INT8_EXPERIMENTAL.rknn`，SHA256 为 `d0249684fed94195f56e4ce273acb6f9997aaae04f6af3c7d8bbb6e7db2458ea`。根配置、preview 片段、三个 demo profile 及三个显式 pose 启动脚本均使用该文件。检测模型保持 v5。上一节四路候选和旧默认文件保留供显式回退。

RKNN 加载时核对五路名称、形状及 INT8 类型；以 `want_float=1` 读取，在输出 release 前经交付的 `rawxy_to_26.hpp` 完成 CPU 网格/步长坐标恢复。RGB uint8 letterbox、固定 0.5 检测/可见性阈值、0.45 NMS、面积路由及原有平滑参数保留。关键点原图坐标保留越界值供诊断；框的显示裁剪仍用于原有面积路由。平滑默认 off，可用 `--pose-preview overlay` 比较 raw/filtered，控制继续使用 raw。

板端 Release 编译及布局/路由/平滑回归通过；六张同输入图片通过真实 NPU、Python 解码及主程序管线核对，每张合并的 218400 个浮点元素与已有真实 NPU 参考逐元素相等，解码误差不超过 0.000062 原图像素。证据位于 `telemetry_logs/pose-rawxy-20261105/`。此次是离线接入验证，未打开相机/串口或运行车辆；交付报告中的远景重复实例、定位与实车业务验收问题仍存在。

无设备验证：`ctest --preset cat-release -R 'rescue_(rknn_output_layout|dual_expert_pose|pose_smoothing)_tests|rescue_rawxy_pose_cli' --output-on-failure`。默认主程序图片入口：`build-release/rescue_upper_host --config config/project.json --calibration '' --detect-image models/a_int8_20261105/fixtures/near_f000000.png --no-show --parallel-infer`。显式回退可加 `--pose-model models/zone_pose_int8.rknn`。

两个旧调试脚本 `start_capture_scan.sh --check` 与 `start_controlled_empty_field.sh --check` 仍因原有标定项缺失返回 2；用替换前脚本及旧模型对照时结果相同。本次仅更新其模型参数。正式 `match_main.sh --check` 和默认主程序图片入口已通过。

2026-11-05 开局普通物资与放货后退调整：UI120 重新纠偏门槛为 60 像素，开框当前画面的对齐要求仍为 30 像素。轻微超界先保持零速，连续至少 3 个新画面且持续 100 毫秒才撤销已完成的对齐；大幅偏离、丢失目标及安全条件失效仍立即停车。普通物资的刹车和稳定确认共用至少 250 毫秒的连续静止图像窗口，图像漂移上限仍取现有配置。UI140 精对齐与首次核心物资/危险物复核保持原有要求。

放货终点停车时提前采集后退 IMU 零偏，开框应答期间持续校验；开框冲击、转动、发送速度非零、无效或过期样本撤销缓存。后退仍以设备时间 IMU 位移达到 0.15 米结束，不以命令速度积分代替位移。校准、实际后退、停车确认分别计时：默认预算 2 秒、10 秒、1 秒，停车仍需连续 250 毫秒零速发送确认；低速配置沿用现有实际后退预算下限。超时分别为 `jaw_retreat_calibration_timeout`、`jaw_retreat_motion_timeout`、`jaw_retreat_stop_timeout`，数据中断仍为 `jaw_retreat_imu_gap`。`[JAW_RETREAT_IMU]` 记录校准状态变化，`[JAW_RETREAT_TUNING]` 记录参数；当前后退校准采用 500 毫秒窗口、0.15 rad/s 各轴角速度上限及 0.30 m/s² 投影加速度标准差上限，属于待实机位移验证的参数。

上述回归不打开设备：`build-release/rescue_task_phase_tests --startup-logged-jitter`、`build-release/rescue_carry_task_integration_tests --retreat-timing`。


搜索目标规则（当前，2026-11-05）：现场伤员统一按躺卧物块处理，删除橙色轮廓高宽比直立阈值、几何直立／未确认躺卧拒选及相关多帧投票；伤员继续遵守识别置信度、图像完整性、单独运输和夹取条件。搜索按当前视角确认所需物块，再用当前有效画面判断物块与己方安全区的关系：没有己方安全区检测及有效己方区域关键点时，按目标在区外继续接近；区域可见时优先使用区域几何，否则完整包含于区域检测框的目标按区内释放、与所有区域框分离的目标按区外接近，重叠但归属不明沿用4秒等待预算。等待预算绑定本次锁定，几何信息间歇失效不重置；重新确认目标在区内立即停车并返回搜索。上述区外默认只服务搜索，不产生区域位姿、投放成功或路径净空证据。本段替代历史伤员直立拒选及首轮投放后看不到区域仍等待的搜索规则。


2026-11-05 删除成排目标选择：搜索不再拟合左右物块排、不再按排中/排端选点，排布结果也不能覆盖局部每趟收益排序；删除 row_select 及其配套参数。首次普通物资交付前仍只选普通物资；交付后普通物资每个5分、核心物资每个10分、单名伤员15分，最多两件物资，同分优先伤员，两件核心物资20分仍优先于伤员。已接受的可见目标保持锁定，近景夹取区域筛选、首轮前后两件普通物资先近端接近、库存和危险物复核继续生效。日志不再输出 ROW[...]。无设备回归：build-release/rescue_task_revision_tests score（含17:48实测日志采样的排外伤员场景）及 rescue_mechanical_tests。

末段续推（2026-11-05 核对）：近视角地面区域首次到达后进入 `ColourOverrun`，保持 `0.06 m/s` 前进 `300 ms`，在到期后的首次控制更新立即请求停车，再停车稳定并确认地面区域后释放、后退。计时起点只在进入该阶段时记录，既有观测与停止条件继续生效；该 300 ms 是控制阶段时长。现有搬运集成测试覆盖恰好 300 ms 的停车边界。


上角分堆转向制动（2026-11-05）：右转90°与返程左转80°保持原目标航向，末段按剩余角度比例降速（2倍角差、最低0.08 rad/s，上限仍受原分堆转速与全局限幅限制），不再在接近目标时持续请求0.5 rad/s。使用相邻有效IMU航向估算实际转速，按150 ms旋转余量提前制动；越过目标时也先停车，再进行小角度修正。只有已发送速度停车、航向变化率不超过0.03 rad/s且250 ms内航向漂移不超过0.005 rad，同时最终角差不超过0.02 rad，才前推或返回搜索；航向无效或采样间隔超过200 ms重置停稳确认。原前推/配平退回、净空门控、目标抢占与总超时保留。回归覆盖末段减速、反向前制动、发送指令为零但航向仍变化、航向中断及100 ms底盘响应仿真；参数与实际超调改善仍需实机运行确认。


分堆初始后退减速（2026-11-06，01:10:57日志）：修复预测提前急停后反复补退造成的走走停停。完整参考物在相邻新鲜图像中的中心Y变化率与实际已发送后退速度共同决定减速上限；按200 ms底盘停止余量加当前图像年龄，在两个停止预测时域内提前连续减速，速度从最高0.05 m/s降到端点0.01 m/s。同一段后退的上限只降不升，重复图像、采图间隔超过200 ms或换ID不更新速度估计。预测不再触发零速，也不作为到位证据；只有实际中心达到下角Y270／上角Y280才开始最终停车确认。停车后仍要求已发送速度归零、新鲜图像Y漂移不超过12 px、达到原确认帧数并持续至少250 ms；若实际退出条件撤回，只能以本段已降低的速度补退。默认额外后退余量仍为0，显式开启时保持原独立余量流程；净空、机构反馈、原始总预算及闭爪核验保留。CONTROL_CHANGE记录连续速度和最终first_trip_split_retreat_brake。无设备回归：ctest --test-dir build-release -R 'rescue_capture_retreat-(continuity|overshoot)' --output-on-failure，覆盖11月6日启停位置模式、21:37位置序列及含66 ms相机延迟、80/120 ms底盘响应与99 ms命令延迟的仿真；要求退出前无预测零速、最终实际退出且完成停车确认、仿真比0.01 m/s基准更快且停止位置额外过冲不超过12 px。图像坐标和仿真均不表示实测位移，实机效果需实际运行确认。

Startup首抓回归：`./build-release/rescue_first_trip_capture_review_tests tests/fixtures/first_trip_20261029_212327_boxes.txt startup-first-grab`。回放2026-11-06 01:32轮次的#178/#179检测框，覆盖保持Startup、对准接近、单抓通过、多抓后释放、危险物和无效数据停车。
