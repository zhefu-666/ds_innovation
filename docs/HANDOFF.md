# 新对话交接清单

更新日期：2026-10-25。项目：`/home/jy/ds_innovation`。

目标：按“配置统一 → 入口拆分 → 状态机分阶段 → 历史入口整理”的顺序优化上位机。每批先保持既有行为，通过原/新对照后，再单独修改策略。当前推进到 R4 第八批，并独立补充严格 ARM 环境基线，整体重构尚未完成。

## 2026-10-25 最新交接（R4 第八批）

- 已提取 `DisperseRushStage`：接管 `sweep_phase_ == 4` 的冲入判定与原 `planSwing` 外侧障碍限幅规划，独占待规划/已规划标记与左右限幅四项；SCATTER、APPROACH、MID_APPROACH、PILE chain 四条调用路径不变。
- cat 上验证：冻结源码基线 CTest 42/43、当前实现遥测 ON 44/45、遥测 OFF 43/44；唯一失败 `rescue_runtime_legacy_baseline` 在本批修改前已存在（工作树 `config/frame_20261008.assume.json`、`tests/fixtures/frame_mapping.synthetic.json` 与冻结夹具不符），没有重生成预期或放宽比较。
- 等价性：新增冲入集成测试在冻结源码与当前实现下完整 stdout 逐字一致；`rescue_push_tests`、单转与往复摆动集成测试、7 组原应用用例同样前/后一致。
- 遥测 OFF 首次构建在 `src/app/runtime_config.cpp:537` 出现一次 GCC ICE，同配置重试通过；`errors_count` 为 0，但主机完整性仍未验收，不能据此宣称健康。
- 下一步建议先提取 `uprightInCorridor`/`neighbourInCorridor` 纯谓词（风险最低），再处理清扫触发聚类、SCATTER 主体与各状态触发/收尾。

## 2026-10-24 最新交接

- 重构工作树仍未提交：13 个已跟踪文件修改、29 个未跟踪文件/目录；不得覆盖或清理。
- 新版协议隔离构建与 CTest `41/41` 已通过；源码清单 `362/362` 复核通过，构建前后一致。旧版 `345/345` 与 `239a807…` 仅作历史对照。
- 新版只读实车预览已确认相机、RKNN、IMU 和 MCU 接收链路工作，A6 `120°` 解析为内部 `0`，全程 `NO_TX`；但几何因 `pitch_not_calibrated` 被拒、`detections=0`，没有发送任何车轮/机构指令，实车验收尚未通过。
- 当前阻断一：`camera.yaml` 只在内部 `+5°`（RX125）标定，现场姿态是 RX120（`0°`）；需重新标定或安全确认后转到 RX125，不能直接改标定范围绕过。
- 当前阻断二：A6 `state=0/id=0`，无 `known_frame.json`；方框实际开/关位置未知，旧配置是临时假设，不能猜方向或编号。
- 当前阻断三：主机 `/sys/fs/ext4/mmcblk0p2/errors_count=3` 且有 EXT4 checksum invalid；`dpkg -V` 仍有 `51` 行差异/不可读项；主机完整性未恢复。
- 下一步先完成 RX120 相机标定和现场方框状态确认，再做只读预览；只有满足安全条件后才允许受控单动作。详见验收文档。
- 完整报告：[validation/CAT_VEHICLE_ACCEPTANCE_20261024.md](validation/CAT_VEHICLE_ACCEPTANCE_20261024.md)。

## 当前状态与约束

- 原始 Git 提交：`bd73a38cea79533807259be1f81be266958dd842`。
- 重构尚未提交；本地工作树有大量修改和未跟踪文件，新对话应先读 `git status`，保留已有改动。
- SSH 别名 `cat` 指向唯一 RK3588 主机 `lubancat`。用户确认已连接相机；现已枚举 USB RYS Camera，`/dev/video0` 为采集、`/dev/video1` 为 metadata，可进行无串口输出的相机与 RKNN 验证。
- 远程 `/home/cat/ds_innovation` 有独立未提交改动和既有程序；不得用本地源码覆盖，也尚未切换部署二进制。
- **远程验收现已阻断，先恢复主机完整性。** 根分区 `mmcblk0p2` 出现 EXT4 checksum invalid、error 117 和 `Data will be lost`；`cc1plus` 与系统包记录的 MD5 不符；可执行 tmpfs 中初始校验通过的冻结基线在编译期间出现两字节非预期变化。不同编译单元反复发生 GCC ICE，迁移 tmpfs 也未消除。已停止新批 ARM 构建/CTest/相机验收，不认定任何硬件根因已解决。未执行 fsck、安装/替换系统文件、改挂载或重启。
- 后续独立核验：Ubuntu 官方签名精确版本包内 `cc1plus` 与 cat 当前可读文件存在 7 字节差异，已排除仅 dpkg 摘要记录错误；尚不能确定磁盘、缓存或内存等损坏环节。23:29 至 23:39 的 EXT4 错误计数从 2860 增至 2866，故障仍在持续。用户确认暂时无法现场操作，未具备离线恢复条件，主机未恢复；详见 `docs/validation/CAT_INTEGRITY_20261020.md`。
- 前批目录为 `cat:/home/cat/refactor-validation/20261020-r4-turn`；本轮 `/home/cat/refactor-validation/20261020-r4-swing` 和 `/dev/shm/r4-swing-validation-20261020` 仅留作异常现场。tmpfs 重启即失，本机已回收关键日志与变异文件，不能直接复用这些目录为验收基点。
- 持久基线 `tests/fixtures/runtime_legacy_baseline.json` 保持冻结，不通过重生成预期或放宽比较掩盖差异。
- 保留现有 `PushObservation` / `PushOutput`、`transport_rules`、日志格式、状态转换、门控、单位和设备时序。阶段提取与策略修改分别验收。

## 已完成

- [x] R0 离线基线：原版构建和原有测试；冻结 7 项应用输出、23 项资源摘要及原版二进制摘要，固定比较纳入 CTest。
- [x] R1 配置等价提取：原 TaskTuning、标定、演示和现场覆盖链移出 main；生成最终只读配置，导出 389 项任务参数、请求值、来源与假设记录。
- [x] R2 首版配置入口：严格 JSON schema v1、`--config`、`--dump-config`、请求层默认值 → 文件 → 显式 CLI、组合校验、绝对资源路径和运行身份。
- [x] R2 兼容策略：`legacy_field_v1`、`legacy_replay_v1` 保留原行为；已有 CLI 继续可用。
- [x] R3 首轮入口拆分：main 收敛为 16 行；已提取 Runtime、PerceptionPipeline、ObservationBuilder、离线任务分派与诊断模块。
- [x] R3 遥测配置：最终配置导出和主程序遥测共用序列化器，保留兼容字段。
- [x] R4 第一批：捕获核验 `CaptureVerificationStage`。
- [x] R4 第二批：交付核验 `DeliveryVerificationStage`。
- [x] R4 第三批：释放后退区 `ReleaseRetreatStage`。
- [x] R4 第四批：现场推入 `PushInStage`，范围仅为 ENTER 内现场搬运/假设推入分支。
- [x] R4 第五批：共享清扫统计与时钟 `SweepLedger`。
- [x] R4 第六批：清扫观察选向、单转、框口复核及可选直推 `DisperseTurnStage`。
- [x] R4 第七批：往复摆动、归中及附加趟判定 `DisperseSwingStage`，独占四项历史，保留 PILE 继承、暂停/reset 语义；14 组 API 测试和 67 条新旧顶层摘要、逐 tick hash 与完整日志一致。
- [x] R4 第八批：冲入与外侧避障摆动规划 `DisperseRushStage`，独占待规划/已规划标记与左右限幅；6 组 API 测试、23 条新旧顶层摘要与完整 stdout 逐字一致；四调用状态、预算/行程边界、许可拒绝、过期聚类均覆盖。
- [x] 本轮本机阶段提取完整回归：遥测 ON 40/40、OFF 39/39；加入独立基线比较器后最终为 ON 41/41、OFF 40/40。原任务 stdout 和 7 项应用输出逐字一致；遥测桥接测试 9/9 通过。
- [x] 严格跨环境比较器：原持久夹具冻结，新增显式 `runtime_legacy_aarch64.json`，绑定原夹具摘要与原版 ARM 来源，仅指定第一帧几何残差整行的精确期望；13 组负向测试，未加入数值容差。
- [x] 只读盘点远程启动入口与三份部署二进制摘要：比赛入口实际使用 `build-school-competition`、hybrid 模型及当前重构不支持的 `--async-capture` 等参数，不能直接替换。来源关系与迁移仍待处理。
- [x] 远程原有全部 30 个 dirty/untracked 文件及三份部署二进制已流式备份到本机 `reports/refactor-validation/r4-swing-20261020/camera/deployment-backup`，33 项摘要与远程备份前/后均一致，无排除项；没有覆盖远程原目录。
- [x] 完整性核验批次补齐 Git 忽略资源的备份范围缺口：375 个项目文件（含 hybrid/全部 RKNN 模型、运行库和头文件、配置、演示 runtime、完整部署源码及二进制）及 11 个当前启动依赖文件回收到 `reports/refactor-validation/cat-integrity-20261020/recovery-assets`。远程前后清单与本机归档逐项一致，原 33 项文件未变化；读取一致不代表故障主机资源已获可信资格。远程 HEAD 的本机 Git 对象完整。
- [x] 从独立官方签名索引核对精确 GCC 包并保留 `.deb`、签名链、原/可信编译器及 7 字节差异；完成离线恢复步骤与验收门槛记录，未在远程安装。
- [x] 已有 R4-six 二进制的相机串/并行各 30 秒短跑，以及真实采集单帧的原版/R4-six 对照已留证；采样时段存在存储错误，只记作有此限制的观察，不代表新批 ARM 验收或长期稳定性。
- [x] 六批阶段均有独立单测和原/新顶层对照；最近一批 19 组阶段 API 检查通过，51 条集成摘要、逐 tick hash 和完整日志一致。
- [x] R5 部分文档：架构、配置运行指南、进度记录、README 更新及 README 旧阶段内容归档；明确主遥测与独立示例工程的区别。
- [x] 前批 cat 隔离构建：R4-six 源码 ARM64 Release，RKNN ON、遥测 ON，完整目标最终构建通过；源码和资源摘要已保存。本批主机异常后未完成最终 ARM 全构建。
- [x] 前批 cat 完整 CTest：37/38 通过、0 跳过；比赛 socket 测试通过。不能记为本批结果或全绿。
- [x] cat 同机原/新离线对照：原版与当前版 7 项用例的参数、返回码、完整 stdout/stderr 逐字相同。
- [x] cat 真实 RKNN 推理：当前版 7 张已有图片，串行/并行共 14 次均成功；同图除性能行外输出一致。
- [x] cat 原/新模型输出对照：原版另跑同 7 张图片串行推理，检测、姿态、关键点、诊断输出及返回码与当前版一致，比较时仅剔除性能行。
- [x] 前批 cat 原部署保护：原工作树状态及两个既有主程序摘要在验证前后一致；前批未打开真实相机或串口。本轮仅相机预览，无串口输出或底盘/机构动作。

## 未完成：软件与结构

- [ ] R2 最终参数优先级：当前只统一了请求来源，兼容现场赋值和标定仍可覆盖请求值；尚未完成新的最终覆盖语义及旧脚本迁移。
- [ ] R2 严格、受控、实验策略的独立语义、假设资格检查与差异验收；不能把现有兼容策略视为已经完成新模式设计。
- [ ] R4 清扫其余动作：各状态的触发与收尾；冲入、外侧障碍摆动规划、往复摆动/归中/附加趟判定已完成。
- [ ] R4 搜索接近：搜索线索、目标选择、丢失重获及接近过程的阶段边界与状态所有权。
- [ ] R4 捕获/搬运其余流程：PREPARE、RUSH、LOWER_FRAME、搬运释放协调等尚留顶层的逻辑，按实际依赖逐批划分。
- [ ] R4 严格 ENTER、其余搬运恢复与异常路径；当前 PushInStage 不覆盖全部 ENTER。
- [ ] R3 完整观测采样与回放：保存影响每帧决策的全部输入、有效配置和运行身份，覆盖主循环真实数据链。
- [ ] R3 评估 Runtime 仍管理的预检、记录和生命周期职责，按实际复杂度继续拆分。
- [ ] R5 确认在用启动脚本 → 实际二进制 → 源码/配置对应关系，迁移部署入口，收敛重复入口。
- [ ] R5 其他历史文档归档、无调用代码标记及确认后清理。

## 未完成：验证与问题

- [ ] 本轮 ARM 完整构建、最终 CTest、显式跨环境基线、新批相机串并/信号退出/无效相机检查：均被主机完整性阻断。原冻结期望不变；补充规则已完成本机及存档原版输出核验，尚缺健康 ARM 主机最终验收。
- [ ] 主机存储、工具链和输入完整性故障的处理及根因：原对象两条重定位类型字节异常、本轮多次 GCC ICE、EXT4 数据丢失告警、cc1plus 包摘要不符与 tmpfs 两字节变化均已留证；不能归结为单一原因或记作已解决。
- [ ] 识别质量验收：检测准确率、误检、关键点质量和区域身份；本次图片未传颜色标定，颜色均为 `color_unmeasured`。
- [ ] 稳定性能验收：持续主循环延迟、稳定 FPS、资源占用、长时间运行；现有单图耗时只是冒烟数据，部分与编译并行。
- [ ] 实测模型/标定资格：相机、机构、俯仰档位、反馈映射及假设文件的可信度，不能用“加载成功”代替现场验收。
- [ ] 相机、MCU、IMU 接入、传感器同步、断连、线程退出及停车时序。
- [ ] 台架/实车：机构、底盘、急停、看门狗、搜索、捕获、搬运、释放和交付全过程；等待设备和现场条件具备。
- [ ] 真实成功/失败录像、完整观测与动作时序基线。
- [ ] 分批整理提交及部署替换；当前尚无本轮重构提交或部署切换。

## 下一轮建议

1. 先具备现场独立 SD/USB 救援启动或外接 eMMC 条件，再恢复 `cat` 的存储、工具链与输入完整性。用户目前无法现场操作；官方精确 GCC 包和部署恢复资源已在本机保全。按 `CAT_INTEGRITY_20261020.md` 先镜像、再离线检查，并独立排查输入变化；不能仅换目录或只替换编译器后验收，不在挂载中的根分区运行修复。
2. 继续 R4 时先提取 `uprightInCorridor`/`neighbourInCorridor` 两个纯 const 谓词（各一个调用点，风险最低），再评估 `isOrdinary`/`findBest`/`evalGroup` 聚类规划；冲入与外侧规划、往复摆动已完成，不重复提取。SCATTER 主体与各状态触发/收尾耦合最高，最后处理。
3. 修改前冻结当前源码和二进制；建立原/新顶层逐帧对照，覆盖许可拒绝、暂停/reset、缺观测和预算边界。
4. 每批保持策略和原持久基线不变，运行针对性测试及必要完整回归，更新进度和架构文档。
5. 新最终配置语义、严格策略、完整观测回放和实车验收分别推进，避免混进结构提取；部署迁移须先解决远程独有参数/模型/源码对应差异。

## 文档与证据入口

- 总方案：`/home/jy/ds_innovation/REFACTORING_PLAN.md`。
- 当前进度：`/home/jy/ds_innovation/docs/REFACTORING_PROGRESS.md`。
- 架构：`/home/jy/ds_innovation/docs/ARCHITECTURE.md`。
- 配置语义：`/home/jy/ds_innovation/docs/RUNTIME_CONFIGURATION.md`。
- cat 验证：`/home/jy/ds_innovation/docs/validation/CAT_HOST_20261020.md`。
- 本轮相机与部署盘点：`/home/jy/ds_innovation/docs/validation/CAT_CAMERA_20261020.md`。
- 本轮构建、基线与主机完整性问题：`/home/jy/ds_innovation/docs/validation/CAT_SWING_20261020.md`。
- 后续主机独立核验、缺失文件调查与离线恢复门槛：`/home/jy/ds_innovation/docs/validation/CAT_INTEGRITY_20261020.md`。
- 主机核验与补充恢复备份：`/home/jy/ds_innovation/reports/refactor-validation/cat-integrity-20261020`。原 30+3 备份没有覆盖 ignored 模型/库，本批已补齐；不能将原范围缺口误记为文件被删除。
- 本轮本地冻结源码、构建与证据：`/home/jy/ds_innovation/reports/refactor-validation/r4-swing-20261020`。
- 本地原始证据：`/home/jy/ds_innovation/reports/refactor-validation/cat-20261020-r4-turn`。
- 远程报告：`cat:/home/cat/refactor-validation/20261020-r4-turn/VALIDATION_REPORT.md`；日志在其 `evidence/`，当前构建在 `build/`，原版在 `legacy-build/`。
- reports 被 Git 忽略；继续使用前检查存在性，需要长期保留时单独归档。
- 旧记录中的 `/tmp/ds-innovation-turn-*` 及 `/tmp/ds-innovation-sweep-*` 已不存在。已核实本机在其创建与首次报缺失之间重启，启动 tmpfiles 服务成功执行 `D /tmp` 配合 `--remove` 的清空规则，足以解释消失；无逐文件 unlink 审计，不能排除额外未记录删除。第五/六批提取前临时源码与原二进制没有找到精确副本；持久应用 JSON 和第七批前源码归档/二进制仍完整。详见完整性记录及 `missing-tmp-investigation.json`；不将后来的快照冒充旧快照。

本机重建离线基线（从项目根目录执行，选择新的构建目录）：

```sh
cmake -S . -B build-handoff-offline -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DRESCUE_ENABLE_TELEMETRY=ON -DRESCUE_ENABLE_RKNN=OFF
cmake --build build-handoff-offline -j4
ctest --test-dir build-handoff-offline --output-on-failure
python3 tools/runtime/record_baseline.py build-handoff-offline/rescue_upper_host \
  --compare tests/fixtures/runtime_legacy_baseline.json
```

以上为新对话复现命令，本次只整理交接文档，未重新执行这些构建或测试。

## 新对话启动文本

> 继续 `/home/jy/ds_innovation` 上位机重构。先读交接、进度、架构、配置、CAT_SWING/CAT_CAMERA 及后续 CAT_INTEGRITY 验证记录。R4 已完成八批（最近为冲入与外侧避障摆动规划 `DisperseRushStage`），cat 上冻结基线 42/43、当前实现 ON 44/45、OFF 43/44，唯一失败 `rescue_runtime_legacy_baseline` 为本批前已存在的工作树资源/夹具差异，原冻结夹具未改、未放宽比较。保留全部未提交修改，不混改策略。cat 唯一 RK3588 已接相机，本批遥测 OFF 构建出现过一次 GCC ICE 后重试通过，主机完整性仍未验收；不能只换目录或替换编译器后继续 ARM/相机验收。用户暂时无法现场操作，离线恢复尚未开始，不修挂载根分区、不改部署。原30脏文件和3二进制已保全，并补齐 ignored 模型/库等375项目文件与11启动文件备份；均在本机 reports/refactor-validation/cat-integrity-20261020。实际部署独有 async-capture/hybrid 模型未迁移。严格 ARM 补充夹具及本机13组比较器测试已完成，健康ARM最终验收仍待补。文件缺失须调查原因，不把备份范围缺口或临时清理当成源码丢失。
