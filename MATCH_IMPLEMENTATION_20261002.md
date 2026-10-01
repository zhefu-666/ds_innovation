# 比赛流程代码与实车标定清单（2026-10-02）

## 当前结论

本次以 9a08066 为基础。比赛时长按用户确认设为 180 秒，--team red/blue 每场决定己方颜色。
修改实现比赛管理、发送层许可、区域身份/有效性、目标锁定、标定加载、携带路径和落点规划接口。
这不是已经可以自主完成全场的宣告：路径/倒车/完整区内占用证据仍缺实车生产者，实测尺寸和外参还没有完成。
测试只使用单元测试、伪串口和合成回放；未启动真实底盘、夹爪或相机动作，也未修改固件。

## 已实现

- MatchControl：WAITING/RUNNING/PAUSED/FAULT/FINISHED；180 秒使用单调时钟，停止/故障期间不重置比赛时钟；FINISHED 不可恢复。
- start 只能开始一次；stop 锁存停车；resume 明确恢复暂停/健康故障；finish 结束本场。新比赛需退出程序再启动，避免误清累计送达信息。
- --auto-run 只在首次自检就绪时请求一次开始，不自动恢复故障。
- MatchServer 使用 0600 本机 Unix socket 独立监听，避免视觉循环阻塞时收不到停止指令；新进程不能覆盖旧端点。
- MotionLink 每次发送/重发都复核许可；开始前不发送执行器命令，停车后只保留最后一次获准的夹爪/pitch 目标并发送零速度。底层 write/tcdrain 仍可能受驱动阻塞，MCU 自身超时停车必须实测。
- 15 秒缺少可信平移证据会锁存故障，不用指令积分或单独 IMU 航向充当位移。暂定 2 cm 加两端位置不确定度为移动门槛；丢失参照、换参照和未来时间戳不算移动。当前视觉位移只接受已识别身份、非预测的安全区位姿；没有该视野时不能证明移动。旋转/夹取等待是否计入的正式规则尚未确认，当前策略是保守停车，不是裁判规则的最终实现。
- target_region_valid 区分区外/区内/未知；未知和压线不进入抓取。
- 原地转向同时要求 path_safe 和 opponent_zone_clear。
- 已锁定目标不会被每帧新候选替换，失败目标黑名单参与候选筛选。
- 颜色分类只采样已检测到的两个半区内部，排除物体检测框；红/蓝一致且达到阈值后才给出身份。--team 仅决定归属。颜色阈值需用真实场地验证；色彩不足、遮挡或半区不一致不给身份。当前只建立所见区域的归属，尚未构成全场己方/对方区域地图。
- --task-calibration 加载实测夹爪尺寸、载荷半径和与 NEAR 一致的像素区域；无标定时保持 holding 不可观测，禁止自动把旧像素框改名为另一个角度的标定。
- LocalPlanner::planCarry：在明确已知的凸区域内，按车体/夹爪/载荷外接圆检查路径，绕开膨胀障碍；未知空间、过期场景、缺少对方区证据都不出路线。复杂场景无路时停止，最多16个障碍参与绕行图，非全场搜索器。
- DropPlanner：在正确半区选择避开现有物体/边界/分隔线的落点，包含位置与角度不确定度；优先向后填放，给入口留空间。
- CarryNavigator 已接到主循环与 CARRY；没有有效路径不再直奔固定门口。落点锁定后重新验证，不能途中偷偷切换落点；GATE/OPEN_RELEASE/ENTER 要求新鲜落点复核，释放前检查对方区许可。
- 遥测新增比赛状态、剩余时间和缺失证据；硬件输出许可字段不再永久写死为0（表示当帧许可，不是实际轮速反馈）。

## 仍待接入，不允许手动置 true 冒充证据

1. 全场安全空间与障碍生产者：NavigationScene 的 complete、known_region、opponent_region_known、障碍列表及实测 swept_radius_m。
2. path_safe 与 retreat_safe 的运动扫掠判定；特别是后方盲区和经过路径记录。规划路线有效不自动意味着任意转弯速度都安全。
3. zone_inventory_complete、zone_counts_valid 和 zone_occupied：要求整个相关区域被可靠观察，物体完全入区、不压线、静止、类别和身份可信。检测结果为空不等于库存为零。
4. 编码器/可靠定位回传；当前A6只有夹爪done/编号/pitch，没有轮速/轮距。不要改动8字节A6而不同时改固件；应与电控定义版本化反馈。IMU加速度含重力，不可直接二次积分成可靠行驶距离。
5. 抓取的机械/随车运动证据、拥挤遮挡计数，以及全场探索、卡住恢复策略。
6. 相机曝光/采集时间与IMU、pitch同步的实测验证；当前时间戳仍是读取帧后的主机时间。
7. MCU断连/进程卡死时自身停车与动作反馈真值、实际制动距离。
8. 同一场比赛跨进程重启的持久化恢复尚未实现；重启只能作为人工确认的新测试会话，不能把重新开始的180秒当作继续原比赛。

## 控制命令

在远程 /home/cat/ds_innovation 下，主程序运行后另开 SSH 终端：

```bash
python3 tools/match/match_ctl.py status
python3 tools/match/match_ctl.py start
python3 tools/match/match_ctl.py stop
python3 tools/match/match_ctl.py resume
python3 tools/match/match_ctl.py finish
```

REJECTED 表示命令未获准，查看 reason；不能直接修改内存/JSON里的安全布尔值绕过。
若进程异常退出留下 /tmp/rescue-match.sock，先确认旧进程已经退出再清理；启动器不会自动删除别人占用的端点。
比赛时长默认180，可通过 --match-seconds 覆盖。所有新参数仍用命令行传入，config/rescue.yaml 仍不是主程序自动加载的配置。

## 现有标定资料核对

- calibration_runs/circles_20260927_222220_c0d353/intrinsics.yaml 确实存在：1280×720，旧10×7圆点板。
- report.json：拟合RMS约0.218px，3张独立照片约0.262/0.164/0.153px，无警告，但状态仍为 candidate_requires_validation，只是内参，不能改名当作完整 camera.yaml。
- 本次在 config 与 calibration_runs 中未发现已生成的 camera.yaml 或外参候选文件。
- tools/camera_calibration/guided.py、calibrate.py 已支持内参、圆点外参、独立检查与 pitch-range；不应重复编造一套标定流程。
- STAGE_HW2_GROUND_CONTACT.md 记载固件 pitch 读回曾为25而非2500、OPEN完成反馈待修。这是历史记录，本次没有操纵硬件重新确认。
- tools/camera_pitch/pitch_ctl.py 与 tools/gripper/gripper_ctl.py 当前仍限 ±25°，pitch工具写明固件三档。C++可接受±35°并不证明固件已经支持，不能直接用35°开展标定。
- HIPNUC_IMU.md 记录IMU接收/倒装变换验证，但仍留有独立侧倾验证及状态字解释的未决项；请与电控核对是否已有更新记录。

## 按顺序做实测

### A. 执行器与机械尺寸（先架空/断开底盘动力）

停止主程序和所有占用串口的工具。先只查看反馈：

```bash
python3 tools/gripper/gripper_ctl.py status
python3 tools/camera_pitch/pitch_ctl.py status
```

由操作人员分别测试夹爪 open/close，每次核对新编号、实际到位和done=1；至少反复10次，超时或假done不能放行。
pitch工具的 set/repl 会同时保持夹爪张开，不能与夹爪闭合测量同时执行。先选定并稳定NEAR，再退出pitch工具，使用gripper工具关闭并拍照。
在目前固件能力下先验证0°/25°，必须确认真实25°读回为2500 cdeg。固件支持任意角度后再验证12/22/35°及工具限幅。

在底盘旋转中心的地面投影建立原点，x右、y前，量：

- hold_center_y_m：原点到闭合夹持区域中心的前向距离（米）。
- mouth_y_m：原点到张开夹爪前口的前向距离（米），应大于前项。
- corridor_half_width_m：张开夹爪通道半宽，包含合理机械余量。
- load_radius_m：以所选落点中心为基准，能包住最大合法载荷的地面外接圆半径；1/2/3物块、伤员分别量，使用经验证的保守上界。过大导致半区放不下时应分批运输，不能缩小填值骗过规划。
- 另外记录车身、开闭夹爪和载荷相对旋转中心的最大外廓，供后续 NavigationScene.swept_radius_m 使用。

复制 config/task_calibration.example.json 为新的实测文件，填实测值。模板 measured=0 且尺寸为0，程序会拒绝它，这是有意的。

### B. NEAR 夹持像素区域

使用与正式运行相同的1280×720、焦距和安装。保持底盘静止、NEAR真实读回稳定，分别拍：空夹爪、1/2/3块普通物资、伤员、框外紧邻物体、跨边界物体、遮挡物体。

```bash
python3 tools/camera_calibration/calibrate.py snapshot --camera /dev/video0 --width 1280 --height 720 --output calibration_runs/holding_1.png
```

每张换一个新文件名。人工在原始图上测出闭合框内部矩形 x1,y1,x2,y2，而不是物体框。可用新工具复核像素框（将坐标替换为实测）：

```text
python3 tools/gripper/holding_overlay.py --image calibration_runs/holding_1.png --area x1,y1,x2,y2 --output calibration_runs/holding_1_overlay.png
```

同一矩形应能覆盖合法夹持范围，同时不能把框外物体收进来。区域有透视/形状无法用矩形表达时应升级判据，不能强行扩大矩形。
填 holding_views.pitch_cdeg 和 area，至少包括实际使用的NEAR。若25°看不完整闭合框，应调整安装或等待固件支持合适角度。
静止图像仅验证区域和计数，不能证明夹住。之后在受控低速下验证物体随车运动、松开后不再认作持有。

### C. 相机内参、外参、pitch范围

镜头焦距、分辨率和安装未变时，可以从上述既有内参候选开始复核；改变过焦距则按 GUIDED_CIRCLES.md 用9×6板重新采集，不把旧10×7采样当9×6计算。

```bash
python3 tools/camera_calibration/guided.py --stop-preview
```

电脑浏览器打开 http://192.168.1.123:8081/ ，先量真实圆心跨度再拍。此命令由人工在安排好的标定时段执行，本次没有启动该服务。
外参按《相机圆点棋盘一体化标定操作指南》和STAGE_HW2执行：板平放，量圆心间距、编号0点的x/y、yaw、板厚，另测至少3个独立地面点；核对corners_numbered.png方向，不能只看低RMS。
只有 extrinsics.json validated=true 且独立误差达标后使用生成的 camera.yaml，再在实际FAR/TRACK/NEAR逐个复核。固定单应不能代替有外参的pitch模型。
当前三档固件可参考STAGE_HW2第3.5节验证0/2500；固件升级后再补12/22/35°。间隔阈值放宽只是允许验收两档，不证明未测中间角可靠。

### D. 红蓝身份

在真实场地、正常光/阴影/反光、空区/有物体/被遮挡条件下，分别验证红区与蓝区；把 --team red 和 --team blue 对调再测，归属必须随队伍配置改变而实际颜色不变。
config/zone_color.example.json 是未验证阈值模板。算法暂用OpenCV HSV固定红蓝色相窗，S/V/面积比例可调；这并不能代替真实样本验收。两个半区缺失或颜色冲突必须输出unknown，红蓝物块不能单独冒充安全区。
候选阈值只在 --dry-run/离线图像验证，确认后再保存 measured=1 的正式文件。

### E. 位移反馈与低速地面验收

与电控确认是否有电机编码器，以及能否回传左右轮累计计数/距离或轮速、采样时间、序号和有效标志。先定义版本化反馈，保留现有A6兼容性。
卷尺对照前进/后退0.2m与0.5m，转向角与IMU对照；分别测试空载、带载、顶住与打滑。编码器转了但车没动仍不能算有效平移，需要视觉或其他证据识别滑移。
在路径/倒车/计数生产者和标定真正接入前，不执行自主地面比赛。届时先完成架空指令与故障停车验证，再低速单趟，再多趟、伤员和危险物场景，最后180秒整场。

## 本次文件清单

远程项目根目录 `/home/cat/ds_innovation` 下新增或修改以下40个文件；原有临时备份未删除。

```text
CMakeLists.txt
MATCH_IMPLEMENTATION_20261002.md
config/task_calibration.example.json
config/zone_color.example.json
include/rescue/config.hpp
include/rescue/geometry_pipeline.hpp
include/rescue/match_control.hpp
include/rescue/match_server.hpp
include/rescue/motion_link.hpp
include/rescue/navigation_adapter.hpp
include/rescue/perception_adapter.hpp
include/rescue/planner.hpp
include/rescue/push_task.hpp
include/rescue/task_calibration.hpp
include/rescue/zone_color.hpp
include/rescue/zone_geometry.hpp
src/config.cpp
src/geometry_pipeline.cpp
src/main.cpp
src/match_control.cpp
src/match_server.cpp
src/motion_link.cpp
src/navigation_adapter.cpp
src/perception_adapter.cpp
src/planner.cpp
src/push_task.cpp
src/task_calibration.cpp
src/telemetry_publisher.cpp
src/zone_color.cpp
tests/fixtures/push_delivery.json
tests/match_tests.cpp
tests/model_io_tests.cpp
tests/navigation_tests.cpp
tests/push_task_tests.cpp
tests/sensor_protocol_tests.cpp
tests/stage01_tests.cpp
tests/task_calibration_tests.cpp
tests/zone_color_tests.cpp
tools/gripper/holding_overlay.py
tools/match/match_ctl.py
```

## 软件验证

隔离构建完成，13项 CTest 全部通过，包括原9项、比赛控制、实测配置校验、携带规划与红蓝身份测试。串口层新增许可撤销与执行器目标保持测试使用伪串口；不连接实车串口。Python工具通过语法检查。
