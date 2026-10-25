# 重构实施与验证记录

首次实施：2026-10-19；最近更新：2026-10-25。原始应用提交：`bd73a38cea79533807259be1f81be266958dd842`。已完成配置与入口提取，并迁移捕获核验、交付核验、释放后退区、现场推入判定、共享清扫账本、清扫单转、往复摆动、冲入与外侧避障摆动规划、兜底退区、搜索与接近、清扫状态机、严格进区判定、盲扫聚集选择、假设搬运视觉接近、搬运区域搜索与敌方规避以及盲扫触发门；保持既有 PushTask 策略、协议与设备时序。

## 2026-10-24 实车验收检查点

新版协议隔离目录完成 ARM Release 构建与 CTest `41/41`，源码清单 `362/362` 复核通过，构建前后清单一致；隔离二进制 SHA-256 为 `8bc897da951438fb529d3b70e8f23e5da09c3a583e8a60668b3a6eb52990e51a`。新版只读预览确认 IMU、相机、RKNN 推理和 MCU 接收链路工作且全程 `NO_TX`，但未发送任何车轮或机构指令，**实车验收未通过**。

当前状态：

- A6 `90..150°`、`120°` 为零点的协议已同步到实现、工具、测试和协议文档；本地与远端 ARM 离线回归均 `41/41`，冻结基线未改。新版只读实车复验确认 `120° -> pitch=0`。
- 几何仍被 `pitch_not_calibrated` 正确拒绝：`camera.yaml` 只在内部 `+5°`（RX125）标定，现场姿态为 RX120（`0°`）。需重新标定或安全确认后转到 RX125，不能直接改标定范围绕过。
- A6 当前 `state=0/id=0`，没有 `known_frame.json`；方框配置明确为临时假设，不能猜测实际开/关位置或动作编号。
- 新版预览稳态约 `9.22 FPS`（剔除首帧；全样本 `8.735`）、`inference_ms≈95.4`、`detections=0`。原始日志有跨记录拼接，`PIXEL_SELECTION` 只确认 `90` 条完整决策，均为 `no_legal_candidate`；识别、选择与标定资格仍未验收。
- 主机完整性未恢复：EXT4 `errors_count=3`，`dpkg -V` 仍有 `51` 行差异/不可读项；不能把本轮只读链路通过当作主机健康证明。

下一步应先完成 RX120 相机标定与方框物理状态现场确认，再重跑只读预览；完成前不进行受控动作。详细记录见 [CAT_VEHICLE_ACCEPTANCE_20261024](validation/CAT_VEHICLE_ACCEPTANCE_20261024.md)。

## 分批状态

| 批次 | 已完成 | 尚需验收或实施 |
| --- | --- | --- |
| R0 基线 | 原代码离线构建；22 项原测试；固定 7 组应用输出、23 个资源摘要与原二进制摘要；持久化对照纳入 CTest | 在用板卡、模型、现场失败/成功录像、真实完整观测与实车时序基线 |
| R1 配置等价提取 | 原 TaskTuning/标定/演示/现场赋值链移出 main；最终配置只读；完整 389 项任务参数、请求值、覆盖链与假设导出 | 现场模型与标定资格的独立验收；观测内原算法常量仍保持原样 |
| R2 新配置与模式 | 严格 JSON schema v1；文件到显式 CLI 的请求优先级；两种版本化兼容策略；组合校验；导出与运行身份 | 新的最终参数优先级、严格/受控/实验策略的独立语义及部署迁移 |
| R3 入口拆分 | 16 行 main；Runtime、PerceptionPipeline、ObservationBuilder、离线分派及诊断边界；最终配置接入遥测 | 完整观测采样点回放、真实推理/循环延迟、硬件异常退出的现场验收；预检和部分记录仍由 Runtime 管理 |
| R4 阶段迁移 | VERIFY_CAPTURE、VERIFY_DELIVERY、BACK_OUT 判定、ENTER 现场推入与严格进区分支、兜底退区决策、搜索与接近、清扫状态机、共享清扫账本、单转、往复摆动、冲入与外侧避障摆动规划、盲扫聚集选择、假设搬运视觉接近、搬运区域搜索与敌方规避、盲扫触发门均已提取；各批均有独立单测和原/新顶层结果对照 | GATE 的其余复用与零散恢复分支；共享边界仍需逐批拆分，R4 未完成 |
| R5 入口与历史整理 | 当前架构与运行指南；修正 README；归档 README 旧段（`README_DEV_REFERENCE_20261025`）；八份带日期文档移入 `docs/history/` 并修复全部引用；入口统一打印二进制路径与 md5；删除冻结演示运行包与失效的 `match_main_refactor.sh` | 在用部署脚本的实机切换、实车入口检查、剩余历史文档归档与死配置项清理 |

## 可运行入口

从项目根目录执行，二进制路径按实际构建选择：

```sh
# 无模型、无设备的任务回放
./build-offline/rescue_upper_host --config config/runtime/push_replay.json

# 仅解析并导出最终配置；不加载模型、不打开设备
./build-offline/rescue_upper_host --config config/runtime/preview.json --dump-config /tmp/effective_config.json
./build-offline/rescue_upper_host --dump-config -

# 复核固定旧应用的完整输出，不重新生成预期
python3 tools/runtime/record_baseline.py ./build-offline/rescue_upper_host \
  --compare tests/fixtures/runtime_legacy_baseline.json
```

文件按配置文件所在目录解析资源路径；CLI 维持当前工作目录语义。最终快照的 `identity.resource_paths` 提供绝对路径，`identity.executable` 提供实际执行文件，`mode` 区分任务与请求的 MCU 访问方式。dump 和 check 都在设备装配前退出；单图任务仍需实际图片和模型。

兼容策略有意保留旧语义：例如请求启动速度 0.25 m/s、时长 5000 ms，现场最终值为 1 m/s、1250000 us；请求 `turn_wz=0.9` 可能最终成为 0.55。文件和 CLI 优先级决定请求层，后续兼容现场赋值仍可覆盖。不能把新 schema 误认为已完成新的最终优先级迁移。

`--dump-config` 输出全量配置，但不替代标定资格或设备健康检查。`--check-config` 对静态资源返回 0/2，既有错误仍返回 1；两者合用时先输出快照，再做静态检查，诊断不会混入 stdout JSON。旧 CLI 仍可直接使用。

## R4 第十二批：搬运区域搜索、敌方规避、盲扫触发门与死状态清理（2026-10-25）

本批完成三项：

1. `CarrySearchStage` 接管 `carrySearch()` 与 `evadeEnemy()`。敌方规避链与区域搜索链的全部私有历史移入阶段，`reset()` 一并清空；父层继续持有 `zone_mem_*`、`run_yaw0_*`、`travel_` 并按值传入，回传类型化动作、原因、`travel_` 复位与区域搜索起点复位。`scanSpin()`/`steer()`/`drive()` 的公共许可、`reason_` 写入和状态切换留在父层。严格路径先调用只含规避的 `evade()`，规避不占用 tick 才按旧行为原地转一次；假设路径使用合并后的 `update()`，规避结束时搜索在同一 tick 继续运行——这与原两个 lambda 的顺序一致。
2. `SweepTriggerPlanner` 接管所有盲扫触发共享的派生许可（`scatter_ok`、`scatter_multi_ok`、`approach_ok`、`far_y`、聚类新鲜度）。无状态纯函数，条件逐条照搬。
3. 删除只写不读的 `delivery_seen_`（三处写入、零处读取）与同样只写不读的 `enemy_evade_count_`。这是行为无关的死状态清理，不改变任何判定。

| 检查 | 结果与范围 |
| --- | --- |
| 当前源码（ARM64 Release，RKNN ON、遥测 ON） | 全量构建通过；CTest 53 项、52 通过；唯一失败为既有 `rescue_runtime_legacy_baseline` |
| 阶段公开 API | 新增 25 组测试通过：搜索 14 组（旋转、新鲜/记忆/起跑三种驱动、缺航向重转、规避帧门限与转向、驶离复位行程、结束冷却、禁用回退、同 tick 续跑、reset）与触发门 11 组（就绪、禁用、运动许可、航向、次数严格边界、冷却等号、账本与 dense 回退、远窗收缩、聚类新鲜度与未来时间戳） |
| 顶层原/新对照 | 十一组任务级测试完整 stdout 逐字一致；`rescue_push_tests` 摘要仍为 `002ee97645a892ee91217b0c65f05274ac5661b81dd8f0e405fca5495ca351f5` |
| 分支可达性 | 临时插桩统计：搜索动作五种结果全部出现（旋转 1076、起跑线 40、记忆 91、新鲜 31），规避两种全部出现（转向 146、驶离 201），触发门在测试中求值 87520 次；插桩已在提交前移除并重新构建复跑 |
| 构建异常 | 首次全量构建在 `enter_stage.cpp` 触发 `arm_neon.h` 头文件解析错误（与既有记录的 GCC 异常一致），同配置重试通过；之后另有一次因 CMake 源列表遗漏链接失败，已修正 |

构建目录为 `cat:/home/cat/refactor-validation/20261025-r4-search/build-after`，对照 stdout 与摘要位于其 `evidence/`；基线复用同机 `20261025-r4-carry/build-after`。本批未运行真实相机、串口或机构动作。

## R4 第十一批：假设搬运视觉接近（2026-10-25）

本批提取 `CarryApproachStage`，接管 `CARRY` 中已锚定区域时的视觉闭环：追踪区域中轴前视点接近前沿，判定到达释放间隙、卡滞与 IMU 受阻。区域位姿预计算、`lockDropHalf()`、`vref_used_` 与 `zone_search_started_us_` 写入、漏帧、`evadeEnemy()`/`carrySearch()` 分支、`drive()`/`steer()` 公共许可、`noteDropZone()` 与状态切换仍在父层。

阶段独占原父层的卡滞进度时刻与最好深度两项，原先三处清零改调 `reset()`；深度不随之清空，由下一帧按原 `!carry_progress_us_` 分支刷新，保持修改前语义。判定顺序、`>=` 与 `>` 边界、IMU 受阻优先于视觉卡滞、卡滞需要新鲜视野、释放间隙的两种原因、边走边转仅在大角度且远离前沿时生效，均逐条照搬。本批没有改变策略、配置或冻结基线，也未迁移 `carrySearch()`、`evadeEnemy()`、GATE 及其余搬运恢复。

| 检查 | 结果与范围 |
| --- | --- |
| 当前源码（ARM64 Release，RKNN ON、遥测 ON） | 全量构建通过；CTest 51 项、50 通过；唯一失败为既有 `rescue_runtime_legacy_baseline` |
| 阶段公开 API | 新增 15 组测试通过：新鲜/推算两种推进、释放间隙等号边界、无新鲜视野的推算原因、IMU 受阻阈值与前沿条件、IMU 优先于视觉卡滞、卡滞需新鲜视野、进度刷新与阈值等号、边走边转的远端条件与前沿原地对正、已对正不转、`reset()` 重启卡滞时钟 |
| 顶层原/新对照 | 任务主回归、交付、现场推入、释放、导航、清扫、单转、往复摆动、冲入、捕获、stage01 共十一组完整 stdout 逐字一致；`rescue_push_tests` 摘要仍为 `002ee97645a892ee91217b0c65f05274ac5661b81dd8f0e405fca5495ca351f5` |
| 分支可达性 | 临时插桩确认新阶段在 `rescue_push_tests` 中被调用 4280 次，六种结果全部出现（视觉推进 4083、推算推进 164、到达释放间隙 29+1、卡滞 2、IMU 受阻 1）；插桩已在提交前移除并重新构建 |
| 白盒测试调整 | `capture_task_integration_tests` 原先直接读写父层字段，改为通过阶段的 `progressUs()`/`seedProgressUs()` 访问；断言语义与打印格式不变，输出摘要与修改前一致 |

构建目录为 `cat:/home/cat/refactor-validation/20261025-r4-carry/build-after`，对照 stdout 与摘要位于其 `evidence/`；基线复用同机 `20261025-r4-cluster/build-after`。本批未运行真实相机、串口或机构动作。

## R4 第十批：盲扫聚集选择（2026-10-25）

本批提取 `DisperseClusterPlanner`，接管原先内联在 `update()` 的 `isOrdinary`、`findBest`、`evalGroup` 三个 lambda：盲扫在聚类点里挑选最密集且最朝正前方的组，并给出“短窗清扫 / 先靠近 / 不触发”的结论。清扫方向与距离、账本开始、冷却、`beginScatter()` 的字段写入、运动许可和全部状态切换仍由父层管理。

提取并保留了唯一的非纯输入：`ord_extra`（锁定点与弱检测点）和最优组三项。原实现把它们放在 `update()` 作用域，跨该次调用的多个调用点存活；为保持生命周期完全一致，规划器在 `update()` 内按值构造，所有调用点共用同一实例，不跨帧保留。打分公式、同分取近、`radius` 与方位/横向/距离筛选、`blind_scatter_need_ordinary` 判定、升级数只对大于 1 的阈值生效、远窗回退与命中远组才返回 2、未命中清除最优组，均逐条照搬。

| 检查 | 结果与范围 |
| --- | --- |
| 当前源码（ARM64 Release，RKNN ON、遥测 ON） | 全量构建通过；CTest 50 项、49 通过；唯一失败为既有 `rescue_runtime_legacy_baseline` |
| 阶段公开 API | 新增 14 组测试通过：密集组与质心、越界与非有限点剔除、普通物资要求与追加点、半径剔除、密度优先与方位打分、同分取近、短窗与远窗结论、升级数只影响大于 1 的阈值、未命中清空最优组、半径覆盖与回落 |
| 顶层原/新对照 | 任务主回归、交付、现场推入、释放、导航、清扫、单转、往复摆动、冲入、捕获十组完整 stdout 逐字一致；其中往复摆动 `52ebc3929a4b4c21274851c9b96f4968a871f30c32a90b65356a0773cff8de6c`、冲入 `023fda3546fb848ee4809a157ed64d7ce1e0f93a93ada819289f2acbb80055b4` 与前批记录相同；输出包含 `blind_scatter_start`，确认触发路径经过新规划器 |
| 基线对照 | `rescue_runtime_legacy_baseline` 失败内容与本批前逐行一致，未重生成预期 |

构建目录为 `cat:/home/cat/refactor-validation/20261025-r4-cluster/build-after`，对照 stdout 与摘要位于其 `evidence/`；基线构建复用同机 `20261025-r4-enter/build-after`（同一 HEAD `a8f65cd`）。本批未运行真实相机、串口或机构动作。

## R4 第九批：严格进区判定（2026-10-25）

本批提取 `EnterStage`，接管 `ENTER` 中现场推入分支之外的严格进区判定：区域内持有中心越过投放线、横向偏差超限、进区预算超时，以及区域失效后的目标丢失分派。相机门控、闭框命令与反馈、`drive()` 的公共运动许可、漏帧计数存储、`assume_pushed_`/补推次数等共享状态和全部顶层状态切换仍由父层管理。

阶段无私有可变状态；`EnterContext` 按值提供经过时间、区域可用性、父层已判定的目标丢失标记、假设搬运标记、投放点锁定状态与纵坐标、区域坐标下的持有中心、目标半区位置和进区航向误差。判定顺序与原内联分支逐条对应：区域不可用时先看目标是否丢失，丢失后假设搬运转 `RAISE_RELEASE`（`assumed_drop_point_zone_lost`）、否则转 `ABORT_DROP`（`zone_lost`）；区域可用时先复位漏帧计数，再依次比较投放线（保留 `>=`）、横向偏差绝对值（保留严格 `>`）与进区预算（保留严格 `>`）。三项都不触发时输出 `enter_speed` 与 `heading_gain *wrapAngle(yaw) + lateral_gain * lateral`，由父层经公共许可执行。父层仍在阶段之前保持 `lostTarget()` 只在区域不可用时求值的副作用顺序，以及 `in.delivery_observed` 的既有写入。

本批没有改变策略、配置或冻结基线，也未迁移剩余清扫触发/收尾及其余搬运恢复。

| 检查 | 结果与范围 |
| --- | --- |
| 当前源码（ARM64 Release，RKNN ON、遥测 ON） | 全量构建通过；CTest 49 项、48 通过；唯一失败为既有 `rescue_runtime_legacy_baseline` |
| 当前源码（ARM64 Release，RKNN ON、遥测 OFF） | 全量构建通过；CTest 48 项、47 通过；失败项同上 |
| 冻结前源码（同机同配置） | 为取得对照，本批开始在修改前单独构建基线；CTest 48 项、47 通过，失败项同为 `rescue_runtime_legacy_baseline`，失败内容与修改后逐行一致 |
| 阶段公开 API | 新增 12 组测试通过：对准直行、航向与横向增益叠加、航向环绕、未锁定/已锁定投放线（含 `>=`）、半区横向严格边界、负横向绝对值、预算严格边界、区域不可用的漏帧等待、目标丢失两种分派、判定优先级以及终止结果不产生运动 |
| 顶层原/新对照 | 交付、现场推入、释放、任务主回归与导航五组测试的完整 stdout 逐字一致；`rescue_push_tests` 摘要 `002ee97645a892ee91217b0c65f05274ac5661b81dd8f0e405fca5495ca351f5`，与前批记录相同；覆盖 `enter_timeout` 等严格进区分支 |

构建目录为 `cat:/home/cat/refactor-validation/20261025-r4-enter/{build-before,build-after,build-after-notlm}`，对照 stdout 与摘要位于其 `evidence/`。本批未运行真实相机、串口或机构动作。

## R4 第八批：冲入与外侧避障摆动规划（2026-10-25）

本批提取 `DisperseRushStage`，独占原外侧障碍摆动限幅四项（待规划标记、已规划标记、左右限幅），并接管 `sweep_phase_ == 4` 的冲入判定与外侧规划。冲入的行程/预算判定、结束切摆动、公共运动许可、清扫账本、单转与摆动起始、框与相机门控仍由父层应用。

阶段只返回类型化结果（`Continue`/`Complete`）、可选运动与原因、原 `[SWING_PLAN]` 日志数据及首摆方向。规划仅在 `swing_avoid_outside`、SCATTER、新鲜聚类三者同时成立时进行；远视角判定开启时延迟到 `pitch_cmd_ == far_pitch_cdeg` 才规划，到完成帧仍未规划则补做。聚类半径、外侧半径、margin、首摆方向选择、冲击速度覆盖、行程等号与严格 `>` 预算边界均沿用旧表达式。本批没有改变策略、配置或冻结基线。

修改前将整个已有工作树（不含 Git、构建、报告与遥测日志）冻结到 `cat:/home/cat/refactor-validation/20261025-r4-rush/source-before.tar`，SHA-256 为 `c5d83e650cbdc5788ae828c3c597b313e8d8ec614e70e54069b582d2b74d13a1`。

| 检查 | 结果与范围 |
| --- | --- |
| 冻结源码基线（ARM64 Release，RKNN ON、遥测 ON） | 全量构建通过；CTest 43 项、42 通过、0 跳过。唯一失败为本批之前已存在的 `rescue_runtime_legacy_baseline`：工作树中 `config/frame_20261008.assume.json`、`tests/fixtures/frame_mapping.synthetic.json` 与冻结夹具摘要不同，`check_mechanical`/`check_carry_demo` stdout 亦不同。同一失败在本批修改前的 `build-arm64-telemetry` 上复现，与本批无关；未重生成预期或放宽比较 |
| 当前源码（ARM64 Release，RKNN ON、遥测 ON） | 全量构建通过；CTest 45 项、44 通过，失败项与基线为同一既有问题 |
| 当前源码（ARM64 Release，RKNN ON、遥测 OFF） | 全量构建通过；CTest 44 项、43 通过，失败项同上 |
| 阶段公开 API | 6 个测试函数通过，覆盖待规划条件、远视角延迟与完成帧补规划、聚类成员与半径外对象排除、左右限幅与 clamp、首摆方向、冲击速度覆盖、行程/预算严格边界、完成帧无运动及选择性复位 |
| 顶层冻结前/后对照 | 新增集成测试以同一源码分别链接冻结源码与当前实现，23 条摘要与完整 stdout 逐字一致，SHA-256 同为 `023fda3546fb848ee4809a157ed64d7ce1e0f93a93ada819289f2acbb80055b4`；覆盖 SCATTER/APPROACH/MID_APPROACH/PILE 四条冲入路径、行程与预算边界、许可拒绝、外侧规划、过期聚类及非 SCATTER 状态 |
| 既有阶段回归 | `rescue_push_tests` 前/后 stdout 逐字一致（`002ee97645a892ee91217b0c65f05274ac5661b81dd8f0e405fca5495ca351f5`）；单转与往复摆动集成测试前/后一致 |
| 原应用对照 | 7 组应用用例的参数、返回码、stdout、stderr 均与冻结二进制一致；资源摘要未变，持久夹具未修改 |
| 主机异常 | 遥测 OFF 首次构建在 `src/app/runtime_config.cpp:537` 出现 GCC internal compiler error（`段错误`），当次 `errors_count` 为 0；同配置重试构建通过。该次 ICE 与既有主机完整性记录一致，不视为已解决，也不构成主机健康证明 |

构建目录为 `cat:/home/cat/refactor-validation/20261025-r4-rush/{build-before,build-after,build-after-notlm}`，构建日志、CTest 输出与前后 stdout 在 `evidence/`。长期回归由新增两组 CTest 与既有冻结基线承担。

## R4 第七批：往复摆动与归中（2026-10-20）

本批提取 `DisperseSwingStage`，独占原摆动序号、当前趟号、总趟数及总摆数四项。阶段处理单转未激活时的往复摆动、归中和附加趟判定；冲入、方向翻转、共享航向/里程/时钟、外侧障碍规划、框与相机反馈、账本、公共运动许可及四个调用状态的收尾仍由父层管理。

阶段只返回类型化结果、可选运动与时钟事件。冲入完成同帧开始摆动判定；下一摆和下一趟转换帧不赋原因、不请求运动。PILE 只清两种序号，继续继承上次总趟数/总摆数；`enter()`、`clearTrip()`、暂停恢复与显式 reset 不额外清四项历史。角度、容差、预算、慢速前移及快速角速度覆盖均沿用旧表达式和边界，详见 [架构](ARCHITECTURE.md)。本批没有改变策略、配置或冻结基线。

修改前将整个已有工作树（不含 Git/报告）冻结到 `reports/refactor-validation/r4-swing-20261020/source-before.tar`，SHA-256 为 `12a0e646d9f920be3ce47ea36fa20a606b16fd243fffa9ae65f8030982488b71`。冻结源码独立完整构建及 CTest 38/38 通过；原二进制摘要为 `9a32ba2e3c8fc7a4546bd9de54f584b2797e281af495b53578c470c0921ded9e`。已有未提交修改完整保留。

| 检查 | 结果与范围 |
| --- | --- |
| 本机遥测 ON、RKNN OFF | Release 全目标构建与 CTest 40/40 通过；当前环境比赛 socket 测试可运行 |
| 本机遥测 OFF、RKNN OFF | Release 全目标构建与 CTest 39/39 通过 |
| 阶段公开 API | 14 组通过；另以 ASan/UBSan 编译运行通过，覆盖交替/归中、多趟、small、限幅、快速摆动、距离/时间等号、角度跨界、零/倒退时钟及选择性重置 |
| 顶层冻结前/后对照 | 同一测试分别独立链接冻结源码和当前实现，67 条摘要、逐 tick hash 及完整 stdout 逐字一致；覆盖 SCATTER/APPROACH/MID_APPROACH/PILE、真实触发配置、许可拒绝、相机/开闭框、缺观测/航向、预算边界、暂停/reset、单转及 PILE 继承 |
| 原有任务与应用 | 原任务测试完整 stdout 逐字一致；7 项应用参数/返回码/stdout/stderr 和资源摘要与本批冻结记录逐字一致；持久夹具未修改 |
| 独立源码审查 | 四字段全部写点、默认值、生命周期、事件应用顺序和公共运动门控对照无差异；发现的独立测试链接依赖缺漏已修复并通过完整构建 |

构建目录为本地 `reports/refactor-validation/r4-swing-20261020/{build-before,build,build-no-telemetry}`。`evidence/` 保留完整 CTest/JUnit、构建日志及应用对照；`swing-task-{before,after}.stdout` 的 SHA-256 同为 `52ebc3929a4b4c21274851c9b96f4968a871f30c32a90b65356a0773cff8de6c`。报告目录被 Git 忽略，长期回归由新增两组 CTest 与既有冻结基线承担。

## 独立验证批次：精确 ARM 基线补充（2026-10-20）

原 `runtime_legacy_baseline.json` 保持冻结，SHA-256 仍为 `c828d652eb7316c817ab6182ba13f8202496b59f9b66b06dd9abdbd108b4d79f`。新增 `runtime_legacy_aarch64.json` 绑定此摘要、原提交与已验证原版 ARM 二进制摘要，只声明 geometry 第一帧完整行的环境期望：`7.81572e-08` → `7.72043e-08 m`。两行除数值 token 外完全相同，源行必须恰好出现一次；其余 7 项用例的参数/返回码/stdout/stderr 及全部资源摘要仍精确比较。

工具必须显式使用 `--compare ... --platform-baseline ...`；不带选项时保留原逐字比较，误用架构、未知/重复字段、基线摘要或来源不符均拒绝。CMake 仅在 `CMAKE_SYSTEM_PROCESSOR=aarch64` 时明确传入已验证补充文件，不按实际输出学习预期，也未添加浮点容差。`--record` 不能使用该补充。

新增 13 组比较器测试全部通过，覆盖原夹具冻结、架构/摘要/格式拒绝、缺失或重复源行、其他字段和任意微小残差变化仍失败、资源变化及 record 组合拒绝。补充后的完整期望另与存档原版 ARM 和前批 ARM 的 7 项实际输出逐字核对一致。本机最终 CTest 为遥测 ON **41/41**、OFF **40/40**；原 x86 无补充比较仍通过。遥测桥接测试在报告目录内安装 `websockets==10.4` 后 **9/9** 通过，未更改系统 Python 环境。

## 本轮 cat 验证阻断与证据保全（2026-10-20）

用户确认唯一 RK3588 为 `cat`，相机已接入。先用前批 R4-six 独立二进制完成相机串/并行各 30 秒无串口预览、退出后相机重开及真实采集单图的原版/R4-six 对照；短时 FPS 约 9.5/14.4。但内核日志证实同期存在根分区 EXT4 校验失败及 `Data will be lost`，这些结果只能作为受该限制的观察，不能据此认定稳定性能或识别质量通过。

本批 ARM 全构建在原磁盘及可执行 tmpfs 各尝试两次，均失败，累计五处 GCC 内部错误。更关键的是：tmpfs 初始逐文件摘要通过后，未运行任何 CTest 或改写夹具，冻结基线出现两字节非预期变化；另 `cc1plus` 与系统包 MD5 记录不符。已停止远程构建、最终 CTest 和新批相机/信号/无效设备验证，未修复系统、安装包、重启或切换部署。**本批没有完整新 ARM 主程序、最终 CTest 或新主程序 7 项应用对照结果**，不能复用前批 37/38 或把已产生的阶段摘要当成完整验收。

所有异常日志、原/变异摘要和 tmpfs 现场已回收本机；部署 Git 状态及六项受保护资源摘要前后一致。远程全部 30 个 dirty/untracked 文件及三份部署二进制另已流式完整备份，33 项本机/远程前后摘要一致，无排除项。详见 [本轮构建与完整性记录](validation/CAT_SWING_20261020.md) 和 [相机/入口/备份记录](validation/CAT_CAMERA_20261020.md)。

在用比赛入口实际使用远程独有的 hybrid 模型与 `--async-capture` 等参数，尚未建立旧部署二进制到可信源码的完整对应；已完成盘点，但不直接迁移。下一轮先恢复主机存储、工具链及输入完整性，再从本机可信源码完整重建。现有证据不足以确定内存、供电、存储、内核、编译器或其他具体根因，不能仅换 tmpfs 目录后继续验收。

## 主机完整性独立核验与恢复准备（2026-10-20）

本批先处理主机阻断，没有推进 R4 第八批。cat 根分区仍为读写挂载的 eMMC，23:29:43 只读采集时 EXT4 错误计数 2860，23:39:03 已增至 2866，日志中的数据丢失告警仍在持续。用户确认暂时无法现场操作，尚无独立救援启动条件；本批未修复分区、安装系统包、改挂载、重启或替换部署，主机未恢复。

在本机完成 Ubuntu 官方签名 `InRelease → Packages → 精确版本 arm64 .deb` 验证。官方 `cc1plus` SHA-256 为 `110b837d05f403bcd21a63eb1f2341338d613aaf3d977d7db6d2c1661c8afa7d`，cat 当前可读文件仍为 `42528bf65af333350aaea6d21c775b30f22cf9129a498a176548929f5c25490f`，两份同长文件存在 7 字节差异，独立 `cmp` 一致。官方 MD5 与 cat 包记录一致，已排除仅 dpkg 摘要记录出错；损坏环节、时间及是否能解释全部 ICE 仍未确定。

重新核对前批持久证据、原 33 项备份及两份基线夹具摘要。调查发现原备份未包含 Git ignored 的模型/库，本机没有其实际副本，属于备份范围缺口，没有证据表明文件被删除。现已通过 SSH stdout 补存 375 个项目文件（63,415,925 字节）及 11 个当前启动文件（306,141 字节），涵盖全部模型、hybrid、RKNN 库/头文件、源码、配置、演示 runtime、部署二进制和当前 DTB/overlay；两组远程前后清单及本机归档逐文件核验均一致，原部署未变化。远程 HEAD 的本机 Git 对象也完整。

历史 `/tmp/ds-innovation-turn-*` 与 `sweep-*` 消失的调查已确认：其创建与首次缺失之间，本机重启时成功执行了 `D /tmp` 配合 `--remove` 的 tmpfiles 清理；没有逐文件删除审计，保留这一归因限制。第五/六批提取前临时源码与原二进制未找到精确副本；持久应用对照 JSON、第七批前源码归档和二进制仍完好，本轮持久证据不受影响。后续冻结均使用项目内持久报告目录，不再依赖旧临时路径，也未改变本机清理策略。

核验、恢复资源、缺失文件调查和可重现脚本位于 `reports/refactor-validation/cat-integrity-20261020`，按仓库规则忽略；执行边界及离线恢复门槛见 [完整性与恢复记录](validation/CAT_INTEGRITY_20261020.md)。本批只改证据与文档，没有业务代码变更，未重新构建或把历史 ON41/41、OFF40/40 记作本批结果。健康 ARM、新批相机及部署迁移仍待完成。

## ARM64 / NPU 主机验证（2026-10-20，前批历史记录）

用户确认目前只有远程主机 `cat` 可用。本次在 `/home/cat/refactor-validation/20261020-r4-turn` 隔离验证，保留原目录代码及部署二进制；未打开真实相机或串口，未执行底盘或机构动作。完整环境、命令、资源摘要和异常记录见 [cat 主机验证报告](validation/CAT_HOST_20261020.md)。

| 检查 | 结果与范围 |
| --- | --- |
| ARM64 真实后端构建 | GCC 13.3 / OpenCV 4.6，Release，RKNN ON、遥测 ON，全部目标最终构建通过 |
| 完整回归 | CTest 37/38 通过、0 跳过；比赛 socket 测试在主机通过 |
| 固定 x86 基线 | 6/7 用例逐字一致；唯一失败为首帧几何残差 `7.81572e-08` 与 `7.72043e-08 m` 的文本差异，其他输出及 23 项资源摘要一致；持久预期未修改 |
| 同机原/新 ARM 对照 | 从原始提交独立构建原版；7 项用例的参数、返回码、完整 stdout/stderr 与当前版本逐字相同，原版也有相同的 x86 残差差异 |
| 真实双模型推理 | 标准检测及姿态 RKNN 模型，7 张已有图片，当前串行/并行共 14 次均返回 0；原版另跑 7 次串行也成功；除性能行外，原/新及串/并行输出逐字一致 |
| 构建异常留证 | 首次任务测试对象文件两处重定位异常；相同源码和参数重编译后链接通过，额外复编产物逐字相同；原因未确定，异常对象和日志已保留 |

本次只确认 ARM64 主机、真实 SDK 和保存图片路径。识别准确率、颜色身份资格、持续主循环延迟、相机/IMU/MCU、机构和实车流程均未验收；R0、R3 及部署相关待项继续保留。

## R4 第六批：清扫单转（2026-10-20）

本批提取 `DisperseTurnStage`，接管共享清扫中的观察选向、单次转向、低俯仰框口复核和可选直推。阶段按值独占原六个私有字段：阶段、清空确认次数、方向、观察起点、框口去重时刻及框口起始转角。父层继续管理共享动作时间/航向/里程、清扫账本、摆动上限规划、相机与框反馈、公共运动许可、快速角速度覆盖，以及清扫结束后的冷却和顶层转换。

阶段返回类型化结果、运动请求、共享进度事件与框口日志数据；父层先应用起点事件，再输出原格式日志、设置原因并调用 `drive()`。Look 转入 Turn 的同帧计算使用新的航向/时刻；转向结束后的可选 Push 从下一帧才开始运动。FrameCheck 先处理转角和预算上限，再检查相机及框口证据，保留原停车确认、缺首帧等待和日志顺序。

普通 Turn 沿用同一新鲜帧可累计的旧行为，FrameCheck 沿用“与上次接受时间戳不同”的判断。任意 `enter()`（包括暂停、恢复及显式 reset）只取消阶段，不额外清私有历史；`clearTrip()` 不调用阶段清理；`begin()` 仅重设阶段、观察起点与确认计数，PILE 下一轮只清确认计数。等号、零哨兵、无符号时间运算及父层许可拒绝后的原因覆盖均按冻结源码保持，详见 [当前架构](ARCHITECTURE.md)。本批未修改配置、策略或持久基线预期。

生产修改前冻结源码到 `/tmp/ds-innovation-turn-source-20261020`，保留原二进制与任务测试到 `/tmp/ds-innovation-turn-baseline-20261020`。原二进制 SHA-256：`11c7aae6ad79944e98116fc498a72eb337f4887c94bdec8b4b9ba3593829380f`。本轮复用 `/tmp/ds-innovation-sweep-refactor-20261020` 和 `/tmp/ds-innovation-sweep-no-telemetry-20261020` 两个构建目录。

| 检查 | 结果与范围 |
| --- | --- |
| 遥测 ON、RKNN OFF | Release 全目标构建通过；CTest 37/38 通过 |
| 遥测 OFF、RKNN OFF | Release 全目标构建通过；CTest 36/37 通过 |
| 阶段公开 API 单测 | 19 组通过：选向、观察新鲜度与几何边界、重复帧、最小/最大转角及预算、框口去重与到位门控、直推、角度跨界、生命周期、零哨兵与时钟倒退；独立 `-UNDEBUG -Wall -Wextra -Werror` 编译运行通过 |
| 顶层原/新对照 | 同一集成测试分别链接冻结源码和当前实现；51 条摘要、逐 tick 轨迹 hash 和完整日志 stdout 逐字一致，原任务测试 stdout 另行对照一致 |
| 门控与生命周期 | 覆盖 SCATTER/APPROACH/PILE、普通往复摆动回退、规划角度上限、转向与直推许可拒绝、快速角速度、相机与开闭框反馈、外层预算与航向丢失、暂停恢复及显式 reset |
| 原有回归与持久基线 | 原任务、前五批阶段及配置/机械/协议测试通过；7 项原/新应用输出与资源摘要一致，持久基线夹具 hash 未变化 |
| 独立源码审查 | 已核对全部六字段及调用点、父层事件与日志顺序、运动许可、相机/框门控和生命周期，未发现行为差异 |

两种构建唯一失败仍为 `rescue_match_tests`，当前沙箱无法绑定 Unix socket。没有跳过该项；本轮新增测试与原有其他回归均通过。

```sh
cmake -S . -B /tmp/ds-innovation-sweep-refactor-20261020 \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRESCUE_ENABLE_TELEMETRY=ON -DRESCUE_ENABLE_RKNN=OFF
cmake --build /tmp/ds-innovation-sweep-refactor-20261020 -j4
ctest --test-dir /tmp/ds-innovation-sweep-refactor-20261020 --output-on-failure
ctest --test-dir /tmp/ds-innovation-sweep-refactor-20261020 --output-on-failure -R rescue_disperse_turn

# 关闭遥测时使用 /tmp/ds-innovation-sweep-no-telemetry-20261020，
# 将配置项改为 -DRESCUE_ENABLE_TELEMETRY=OFF 后执行相同构建和测试。
```

原/新集成输出保存在 `/tmp/ds-innovation-turn-baseline-20261020/integration.txt` 与 `/tmp/ds-innovation-turn-current-integration-20261020.txt`；应用记录为 `reports/refactor-baseline/turn_before.json` 与 `turn_after.json`（按仓库规则忽略）。长期验收依赖新增两组 CTest 及既有持久基线，不依赖本机临时文件。

该批本地验证未启动相机或真实串口；后续真实 RKNN 样图结果见上文主机验证，台架与实车仍未验收。R4 尚未完成：冲入、多次摆动/归中、附加清扫趟和各状态的触发/收尾仍在顶层；搜索接近、严格 ENTER、其余搬运恢复、最终配置优先级、完整观测回放及部署验收继续按独立范围推进。

## R4 第五批：共享清扫账本（2026-10-20）

本批提取 `SweepLedger`，独占原先分散在 `PushTask` 的 11 个清扫统计与时钟字段：单趟次数、单趟/整场时间、链起点与活动段起点、累计活动时间、暂停时刻、活动/摆动标记、清扫间隔和重新观察截止时刻。现有三个清扫诊断接口继续委托账本。搜索接近、SCATTER、PILE、MID_APPROACH 与 APPROACH 内的清扫动作、目标判定、框反馈及运动执行仍在顶层；没有改变配置或策略。

账本保留原分项生命周期：`close()` 只结算一次活动时间，摆动标记决定是否计次；暂停链在链接窗口之后才自动结算。TURN_SCAN 完成时先关闭链，再只清单趟统计与间隔，整场时间与重新观察等待继续保留。显式 reset 清账本有效状态，普通 `clearTrip()`、`enter()` 和操作员暂停不清账本，也不新增活动链暂停；恢复后的时间统计沿用旧行为。

每次更新先处理遗留活动链，再结算暂停链，随后在相机等待之前生成一次预算快照。非补给清扫、dense multi 与普通清扫继续使用各自原有的许可条件。账本关闭与间隔设置保持分离：结束路径设置间隔，PILE 取较大截止时刻；相机等待续期非零重新观察时刻，指定搜索/接近状态在公共框反馈之后消费等待。整数加法、零哨兵、链接/操作/间隔/观察的等号边界及 SCATTER 超时原因优先级保持原样，详见 [当前架构](ARCHITECTURE.md)。

提取前源码快照位于 `/tmp/ds-innovation-sweep-source-20261020`，原二进制和任务测试位于 `/tmp/ds-innovation-sweep-baseline-20261020`。原任务测试和 7 项固定基线均通过；原二进制 SHA-256：`fe752ff74fcfb4d329908ad538ab2cfa025e33b4631b81e3c864fb6ed02f6e2c`。本次独立源码终审核对了全部字段映射、方法、调用点、每帧快照顺序、重置和跨趟生命周期，未发现行为差异。

| 检查 | 结果与范围 |
| --- | --- |
| 遥测 ON、RKNN OFF | Release 完整构建通过；CTest 35/36 通过 |
| 遥测 OFF、RKNN OFF | Release 完整构建通过；CTest 34/35 通过 |
| 账本公开 API 单测 | 13 组通过：活动/暂停计时、幂等结算、续链等号、整链操作期限、单趟/整场预算、次数与升级、间隔赋值与延长、观察等待、跨趟/reset、时钟倒退与零哨兵；独立 `-UNDEBUG -Wall -Wextra -Werror` 编译运行通过 |
| 顶层原/新对照 | 同一集成测试分别链接冻结源码和当前实现，以 `-UNDEBUG -Wall -Wextra -Werror` 编译运行通过；30 条状态/原因/运动/机构目标/账本/阶段时钟摘要逐字一致 |
| 门控与生命周期 | 覆盖远近续链、无摆动退出、时间/次数/间隔边界、相机等待、重新观察、近视图清扫、非补给例外、普通清趟、操作员暂停、意外路径退出、新趟与显式 reset |
| 原有回归与持久基线 | 原任务、前四批阶段及配置/机械/协议测试通过；两种构建的 7 项持久基线均通过，原/新应用记录的输出与资源摘要相同，持久基线预期未修改 |
| 独立源码审查 | 对照冻结源码，未发现调用顺序、预算边界、计时或生命周期差异 |

两种构建唯一失败仍为 `rescue_match_tests`，当前沙箱无法绑定 Unix socket。没有跳过该项；未启动相机或真实串口，实际 RKNN、台架和实车结果尚未验证。

```sh
cmake -S . -B /tmp/ds-innovation-sweep-refactor-20261020 \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRESCUE_ENABLE_TELEMETRY=ON -DRESCUE_ENABLE_RKNN=OFF
cmake --build /tmp/ds-innovation-sweep-refactor-20261020 -j4
ctest --test-dir /tmp/ds-innovation-sweep-refactor-20261020 --output-on-failure
python3 tools/runtime/record_baseline.py \
  /tmp/ds-innovation-sweep-refactor-20261020/rescue_upper_host \
  --compare reports/refactor-baseline/sweep_before.json

# 关闭遥测时使用独立目录 /tmp/ds-innovation-sweep-no-telemetry-20261020，
# 将配置项改为 -DRESCUE_ENABLE_TELEMETRY=OFF 后执行相同构建和测试。
```

原/新集成摘要为 `/tmp/ds-innovation-sweep-integration-{baseline,refactor}-20261020.stdout`；应用记录为 `reports/refactor-baseline/sweep_before.json` 与 `sweep_after.json`（按仓库规则忽略）。长期验收依赖新增的两组 CTest 和既有持久基线，不依赖这些临时记录。

新增集成测试尚未直接覆盖 PILE 的冷却延长与摆动标记清理，以及 APPROACH 密集清扫的预算截断和观测丢失收尾；账本单测及既有任务测试覆盖其中部分行为。上述路径应在后续清扫动作阶段提取时补充原/新对照，本批不据此声称全部清扫路径均已覆盖。

R4 尚未完成。本批只提取共享账本，清扫动作阶段、搜索接近、严格 ENTER 和其余恢复流程仍需逐批处理。新的最终配置优先级、完整观测回放与实际部署验收也继续保留独立范围。

## R4 第四批：现场推入（2026-10-20）

本批提取 `PushInStage`，处理 `ENTER` 中 `carryLogic() && !assume_pushed_` 的现场搬运/假设推入分支。阶段独占对齐标记与起始时刻、保持航向及有效标记、探测脉冲的结束/下次开始时刻与方向，以及本趟未确认进区记录。严格 `ENTER`、相机/闭框反馈、货物跟踪、区域锚定和几何计算继续由顶层处理。

阶段读取显式时间、里程、区域/半区和货物证据，返回继续、补推或完成结果，并附运动请求、清货物参考请求及有序事件。父层在补推事件前增加共享 `repush_n_`，随后调用退区阶段；事件按原序生成，阶段原因先于公共 `drive()` 应用，保留许可拒绝时的停车和原因覆盖。现有交付/退区测试仅调整迁移字段的访问路径，保留原断言。

`prepare()` 只重置对齐与航向有效标记，保留探测脉冲和未确认历史；`clearTrip()` 清趟时重置这些历史，按旧代码保留无效航向值。普通 `enter()`、暂停与恢复不清阶段历史。对齐转向仍先于深度判断，区内/深处阻塞/卡停仍保持优先级；最终进区确认、盲推速度与距离、400 毫秒新鲜度及探测交替边界均保持原策略。本批未修改配置、协议或持久基线预期。

生产修改前冻结源码到 `/tmp/ds-innovation-enter-source-20261020`，保留二进制与任务测试到 `/tmp/ds-innovation-enter-baseline-20261020`。原任务测试和 7 项固定基线均通过；原二进制 SHA-256：`4efe22e15443cdcefc6a392ed807304cc45aca4c2f89ba6095074ea043b505b4`。

| 检查 | 结果与范围 |
| --- | --- |
| 遥测 ON、RKNN OFF | Release 完整构建通过；CTest 33/34 通过 |
| 遥测 OFF、RKNN OFF | Release 完整构建通过；CTest 32/33 通过 |
| 阶段公开 API 单测 | 13 组通过：完成/卡停优先级、对齐及预算、新鲜度、延长确认与硬上限、保持航向、探测边界、prepare/清趟与无锚定路径；独立 `-Wall -Wextra -Werror` 编译运行通过 |
| 顶层原/新对照 | 同一集成测试分别链接冻结源码和当前实现，以 `-UNDEBUG -Wall -Wextra -Werror` 编译运行通过；51 条状态/原因/运动/机构事务/时钟/货物参考/事件摘要逐字一致 |
| 许可与生命周期 | 覆盖相机及闭框门控、严格 drop 复核、公共运动许可、原流程建立对齐、同状态重入、暂停退区、显式 reset、完成或补推后下一帧的框动作 |
| 原有回归与输出 | 捕获/交付/退区阶段、原任务及配置/机械/协议测试通过；两种构建的 7 项持久基线通过，原/新记录的输出与资源摘要相同 |
| 独立审查 | 核对提前几何快照、货物参考清零时序、补推事件顺序、浮点运算、边界及父层 drive 原因覆盖；未发现行为差异 |

两种构建唯一失败仍为 `rescue_match_tests`，当前沙箱无法绑定 Unix socket。未跳过失败项，也未启动相机或真实串口；实际 RKNN、台架及实车结果尚未验证。

```sh
cmake -S . -B /tmp/ds-innovation-enter-refactor-20261020 \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRESCUE_ENABLE_TELEMETRY=ON -DRESCUE_ENABLE_RKNN=OFF
cmake --build /tmp/ds-innovation-enter-refactor-20261020 -j4
ctest --test-dir /tmp/ds-innovation-enter-refactor-20261020 --output-on-failure
ctest --test-dir /tmp/ds-innovation-enter-refactor-20261020 --output-on-failure -R rescue_push_in

# 关闭遥测时使用独立目录 /tmp/ds-innovation-enter-no-telemetry-20261020，
# 将配置项改为 -DRESCUE_ENABLE_TELEMETRY=OFF 后执行相同构建和测试。
```

原/新摘要保存在 `/tmp/ds-innovation-enter-integration-{baseline,refactor}-20261020.stdout`；应用记录为 `reports/refactor-baseline/enter_before.json` 与 `enter_after.json`（按仓库规则忽略）。可持续验收由新增的两组 CTest 及既有持久基线承担，不依赖临时文件。职责与生命周期详见 [当前架构](ARCHITECTURE.md)。

第四批之后已按上述第五批提取共享清扫账本。R4 尚未完成；搜索/清扫动作阶段、严格搬运、最终配置优先级、完整观测回放和部署验收继续保留独立范围。

## R4 第三批：释放后退区（2026-10-20）

本批提取 `ReleaseRetreatStage`，独占预退模式、起始深度、区内确认次数和货物去重时间戳。阶段读取标量上下文与货物证据，返回继续、推入、交付核验、释放后已在区内或故障结果。`RAISE_RELEASE` 的区域复核、对方区域清空、相机与框反馈顺序仍由顶层处理；`ENTER` 的推入执行也未迁移。

`PushTask` 继续管理货物跟踪、几何计算、机构事务、运动许可、共享补推预算、推入对齐、故障和状态转换。释放后区内事件仍在交付视觉计数清零前生成；暂停保留预退起点和观测历史，但阶段计时、里程和盲退标记按旧 `enter()` 规则重置。预退超时继续推入，普通退区超时故障停机；详细约定见 [当前架构](ARCHITECTURE.md)。本批不修改参数、策略或持久基线预期。

生产修改前冻结源码到 `/tmp/ds-innovation-release-source-20261020`，保留原二进制与任务测试到 `/tmp/ds-innovation-release-baseline-20261020`。原任务测试和 7 项固定基线均通过；原二进制 SHA-256：`b609ca38e7521bc6b84d831f8eec675ca68f538d69f42870ab772aabc2146d7d`。

| 检查 | 结果与范围 |
| --- | --- |
| 遥测 ON、RKNN OFF | Release 完整构建通过；CTest 31/32 通过 |
| 遥测 OFF、RKNN OFF | Release 完整构建通过；CTest 30/31 通过 |
| 阶段公开 API 单测 | 13 组通过：严格退区、盲退锁定与未缩放距离、预退几何/里程、超时边界、区内证据门控与重复/非单调时间戳、补推及分项重置 |
| 顶层原/新对照 | 同一集成测试分别链接冻结源码和新实现，以 `-UNDEBUG -Wall -Wextra -Werror` 编译运行通过；41 条状态/事务/俯仰/运动/时钟/事件摘要逐字一致 |
| 门控与生命周期 | BACK_OUT 相机就绪先于开框、开框反馈先于追踪和退区完成；覆盖共享闭框等待、暂停保留起点/计数、显式 reset、下一帧下框、卡停补推与距离增量 |
| 原有回归与输出 | 捕获/交付阶段、原任务及配置/机械/协议测试通过；两种构建的 7 项固定应用输出与持久基线一致，原/新记录的资源摘要也相同 |
| 独立审查 | 核对浮点边界、只读几何预计算、inside/距离/超时优先级、父层结果应用和事件顺序；未发现行为差异 |

两种构建唯一失败仍为 `rescue_match_tests`，当前沙箱无法绑定 Unix socket；没有跳过该项宣称全部通过。未启动相机或真实串口，没有实际 RKNN、台架或实车验收结论。

```sh
cmake -S . -B /tmp/ds-innovation-release-refactor-20261020 \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRESCUE_ENABLE_TELEMETRY=ON -DRESCUE_ENABLE_RKNN=OFF
cmake --build /tmp/ds-innovation-release-refactor-20261020 -j4
ctest --test-dir /tmp/ds-innovation-release-refactor-20261020 --output-on-failure
ctest --test-dir /tmp/ds-innovation-release-refactor-20261020 --output-on-failure -R rescue_release

# 关闭遥测时使用独立目录 /tmp/ds-innovation-release-no-telemetry-20261020，
# 将配置项改为 -DRESCUE_ENABLE_TELEMETRY=OFF 后执行相同构建和测试。
```

对照输出为 `/tmp/ds-innovation-release-integration-{baseline,refactor}-20261020.stdout`；原/新应用记录为 `reports/refactor-baseline/release_before.json` 与 `release_after.json`（按仓库规则忽略）。长期验收依赖已接入 CTest 的两组新增测试及既有持久基线，不依赖临时文件。

第三批之后已按上述第四批提取 `ENTER` 的现场推入、对齐与卡停补推判定；严格进入和公共机构/运动许可仍在顶层。搜索清扫预算、最终配置优先级迁移、完整观测回放和部署验收仍需分别推进。

## R4 第二批：交付核验（2026-10-20）

本批提取 `VERIFY_DELIVERY` 的库存与视觉证据、稳定窗口、定位等待、超时和结果判断。顶层保留几何选择、机构事务、补推执行与共享预算、总分、首趟许可及演示终止。阶段输出旧格式事件，保留原事件顺序；分项重置约定见 [当前架构](ARCHITECTURE.md)。

生产修改前冻结源码到 `/tmp/ds-innovation-delivery-source-20261020`，保留二进制与原任务测试到 `/tmp/ds-innovation-delivery-baseline-20261020`。原任务测试与 7 项持久基线均通过，原二进制 SHA-256：`190b7de024c429c4b610967292b79bf2570580cb02d845b47836b54b34f62198`。本批没有更新持久基线预期或修改运行配置。

| 检查 | 结果与范围 |
| --- | --- |
| 遥测 ON、RKNN OFF | Release 完整构建通过；CTest 29/30 通过 |
| 遥测 OFF、RKNN OFF | Release 完整构建通过；CTest 28/29 通过 |
| 阶段公开 API 单测 | 13 组通过：计数证据门控、重复/非单调帧、300 毫秒边界、部分交付、超时与晚到定位、视觉阈值、补推、事件文本和分项清理 |
| 顶层原/新对照 | 同一集成测试分别链接冻结源码和新实现，全部断言通过；17 条状态/计分/事务/俯仰/计时/事件摘要逐字一致 |
| 暂停与历史 | 确认帧重新累计但保留稳定窗口；远俯仰期间由原相机门控阻止核验；历史 cargo_fix 阈值与跨 reset 重复货物时间戳保持原行为 |
| 原有回归与输出 | 捕获阶段、原任务测试、机械/协议/配置测试及两种构建的 7 项固定应用输出全部通过 |
| 独立审查 | 核对生命周期、几何只读预计算、结果优先级、事件顺序与父层计分；未发现行为差异 |

两种构建仍只有 `rescue_match_tests` 因当前沙箱无法绑定 Unix socket 失败。未启动相机或真实串口，没有 RKNN、台架或实车验收结论。

```sh
cmake -S . -B /tmp/ds-innovation-delivery-refactor-20261020 \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRESCUE_ENABLE_TELEMETRY=ON -DRESCUE_ENABLE_RKNN=OFF
cmake --build /tmp/ds-innovation-delivery-refactor-20261020 -j4
ctest --test-dir /tmp/ds-innovation-delivery-refactor-20261020 --output-on-failure
ctest --test-dir /tmp/ds-innovation-delivery-refactor-20261020 --output-on-failure -R rescue_delivery

# 关闭遥测时使用独立目录 /tmp/ds-innovation-delivery-no-telemetry-20261020，
# 将配置项改为 -DRESCUE_ENABLE_TELEMETRY=OFF 后执行相同构建和测试。
```

本机对照输出为 `/tmp/ds-innovation-delivery-integration-{baseline,refactor}-20261020.stdout`。原/新应用记录为 `reports/refactor-baseline/delivery_before.json` 与 `delivery_after.json`（按仓库规则忽略）。长期验收依赖新增测试源码和既有持久基线，不依赖这些临时文件。

第二批之后已按上述第三批迁移释放后退区判定；释放反馈与推入执行仍由顶层处理。搜索清扫的跨阶段预算、最终配置优先级、完整观测回放与部署验收仍保留独立范围。

## R4 首批：捕获核验（2026-10-20）

本批只提取 `VERIFY_CAPTURE`，不迁移 PREPARE/RUSH、交付核验或恢复动作。阶段读取观测和配置、维护自己的证据，返回类型化结果；顶层保留黑名单、故障、阶段转换、框事务、搬运计时和计分。进入/重新进入/通过 `enter()` 离开核验及 `clearTrip()` 会重置证据，暂停和显式 reset 沿用原路径。详细所有权约定见 [当前架构](ARCHITECTURE.md)。

改动前冻结源码到 `/tmp/ds-innovation-r4-source-20261020`，从该快照构建原版二进制与任务测试。原 `rescue_push_tests` 通过，7 项固定输出及资源摘要与持久基线完全一致。原版二进制 SHA-256：`c491337f04f0ba09c0302436593b75c6c4dd8ec56388ee620b04bb410fec7f4f`。本批保持原持久预期不变。

| 检查 | 结果与范围 |
| --- | --- |
| 遥测 ON、RKNN OFF | Release 完整构建通过；CTest 27/28 通过 |
| 遥测 OFF、RKNN OFF | Release 完整构建通过；CTest 26/27 通过 |
| 阶段公开 API 单测 | 12 组测试通过，覆盖 8 秒及单视角预算边界、清单精确匹配、field_rush/assume_all_safe 差异、框内证据、非法类别重查与 reset |
| 顶层原/新对照 | 同一集成测试分别链接冻结快照和新实现，全部断言通过，22 条结果摘要逐字一致；检查状态、原因、规则、框事务、黑名单、故障和时间戳 |
| 生命周期 | 核验中的复制/移动保持独立历史；同状态重入、暂停恢复、安全故障后 reset 和新捕获均通过 |
| 原有运行输出 | 两种构建的 7 项固定输出、退出码及资源摘要均与持久基线一致，包含完整合成交付轨迹 |
| 独立代码复核 | 对照原分支顺序、结果应用与重置逻辑，未发现行为差异 |

两种构建唯一失败均为已有的 `rescue_match_tests`，在当前沙箱无法绑定 Unix socket。没有跳过该项宣称全绿。硬件、真实 RKNN 推理与现场时序未验证。

本批可复现命令（从项目根目录执行）：

```sh
cmake -S . -B /tmp/ds-innovation-r4-refactor-20261020 \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRESCUE_ENABLE_TELEMETRY=ON -DRESCUE_ENABLE_RKNN=OFF
cmake --build /tmp/ds-innovation-r4-refactor-20261020 -j4
ctest --test-dir /tmp/ds-innovation-r4-refactor-20261020 --output-on-failure

# 定位新阶段与其顶层契约
ctest --test-dir /tmp/ds-innovation-r4-refactor-20261020 --output-on-failure -R rescue_capture

# 关闭遥测时使用独立目录 /tmp/ds-innovation-r4-no-telemetry-20261020
# 并将配置项改为 -DRESCUE_ENABLE_TELEMETRY=OFF，执行相同构建和测试。
```

原构建目录为 `/tmp/ds-innovation-r4-baseline-20261020`；22 条结果摘要保存在 `/tmp/ds-innovation-capture-integration-{baseline,refactor}-20261020.stdout`。原/新应用记录为 `reports/refactor-baseline/r4_before.json` 与 `r4_after.json`（按仓库规则忽略）。这些本机临时文件不作为长期依赖；可持续运行的验收由新测试源码及既有持久基线承担。

首批之后按上述第二批迁移了交付核验；交付计分、首趟完成标记、补推预算与框事务仍留在顶层。配置最终优先级迁移、完整观测回放和部署验收保持各自独立范围。

## R0 至 R3 验证（2026-10-19）

环境为 GCC 11.4、OpenCV 4.5.4、nlohmann_json 3.10.5、Python 3.10.12，Release 构建并启用断言测试。原应用的离线适配只增加显式关闭 RKNN 的构建路径，不替换厂商 ABI。

| 构建/检查 | 结果 |
| --- | --- |
| 原应用，遥测 ON、RKNN OFF | 完整构建通过，原 CTest 21/22 通过 |
| 重构版，遥测 ON、RKNN OFF | 完整构建通过，CTest 25/26 通过 |
| 重构版，遥测 OFF、RKNN OFF | 完整构建通过，CTest 24/25 通过 |
| 7 组原应用输出对照 | 参数、退出码、stdout、stderr 全部一致；包含整段交付轨迹、几何回放与静态错误 |
| 新配置测试 | 严格类型/未知键/重复键、请求覆盖、路径、归一化、导出及 18 次真实应用调用通过；含已删除 cwd 与裸名动态库回归 |
| 遥测快照测试 | 同一最终配置对象、旧字段、wire header、时间戳及清理回归通过 |
| bridge 扩展 | 两文件 Python 静态编译通过；单测因缺少 `websockets` 无法加载 |
| 实际 RKNN 与相机/串口实车流程 | 未验证；本机没有匹配的 SDK 头文件和板卡资源 |

上述两种新构建和原基线的唯一 CTest 失败均为 `rescue_match_tests`：当前沙箱禁止绑定 Unix socket，错误出现在比赛服务建立 socket 时。串口伪终端、动作 ACK、指令过期与停车回归通过。没有跳过失败来宣称全绿。

固定基线保存在 `tests/fixtures/runtime_legacy_baseline.json`；原二进制 SHA-256 为 `4569d2826d4ce77183bb9e3c9317a2f829ea3906f3819b22d885719fc5fed2cd`。比较器同时校验该基线记录的配置/夹具摘要。原始本机记录在 `reports/refactor-baseline/legacy.json`，该目录按仓库规则忽略；持久夹具保留对照所需证据。

本次构建目录分别为 `/tmp/ds-innovation-baseline-20261019`、`/tmp/ds-innovation-refactor-20261019` 和 `/tmp/ds-innovation-no-telemetry-20261019`。未启动相机、实际串口或硬件运行入口，未生成新的实车验收结论。

## 部署入口盘点

这里只记录源码能确认的调用关系，没有运行带机构动作的脚本，也没有替换现有板卡二进制。

| 入口 | 当前二进制来源 | 注意事项 |
| --- | --- | --- |
| `tools/remote_camera_telemetry/manage_project.py` | `build-arm64-telemetry/rescue_upper_host` | 默认 dry-run 预览，桥接为只读查看 |
| `tools/match/match_main.sh` | `RESCUE_BINARY` 或 `/home/cat/startup-fix-build/rescue_upper_host` | 保留静态检查、框反向准备动作、相机归位和启动顺序；外部二进制来源尚未确认 |
| `tools/match/start_capture_scan.sh`、`start_controlled_empty_field.sh`、`start_hardware_blue.sh` | `build-camera-offset-20261007/rescue_upper_host` | 旧构建目录不能假定包含本轮重构 |
| `tools/demo/demo.py` | 指定演示 profile 的 `binary` | profile 还包含资源与准备/验收信息，不是新的 runtime schema |
| `tools/demo/demo_oneclick.sh` | 指定演示 profile 的 `binary` | 与 `tools/demo/demo.py` 共用统一演示入口，不再维护独立运行包 |

历史 Controller 没有加入当前 CMake 构建；当前 main 没有调用它。暂不删除其他入口和历史资源，后续应先确认实际用户、部署包和脚本引用，再逐项迁移。
