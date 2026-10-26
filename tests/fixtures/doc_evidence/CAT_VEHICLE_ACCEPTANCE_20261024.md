# 实车验收整理：2026-10-24

项目：`/home/jy/ds_innovation`  
远端：`cat:/home/cat/ds_innovation`（`cat` 为 SSH 别名，主机名 `lubancat`）  
隔离验收目录：`cat:/home/cat/refactor-validation/20261024-vehicle-acceptance/`  
新版隔离目录：`cat:/home/cat/refactor-validation/20261024-vehicle-acceptance-protocol/`  
整理时间：2026-10-24（最后复核 18:01）

## 结论

> 更新（2026-10-24 协议同步后）：A6 `90..150°`、`120°` 为零点的口径已完成 C++、工具、测试和协议文档同步，用户已确认不再复核该协议口径。**协议层、离线构建、离线回归和真实图片离线推理已通过；只读相机链路部分通过；主机完整性未通过；整车实车验收未通过。** 本地与远端 ARM 均 `41/41`；7 张真实图片的串行/并行各一轮共 `14/14 rc=0`，剔除 `[PERF]` 后逐字一致；相机-only 真实 PID 采样确认只打开 `/dev/video0`。但当前姿态与相机标定范围不匹配，未取得有效地面几何或现场目标检测，且未执行任何底盘、车轮或机构动作。下文的“旧证据”保留作对照，新证据单列在第 5 节。

当前结论分为四层：

| 层级 | 结论 | 依据 |
| --- | --- | --- |
| A6 协议与离线实现 | **通过** | 本地/远端 `41/41`；`90..150°` 解析、边界与夹爪映射测试通过；新版工具只读打开 |
| 只读实车链路 | **部分通过** | A6 连续读回 `120°` 并解析为内部 `0`；相机-only 真实 PID 采样只打开 `/dev/video0`，未打开 MCU/IMU 串口；IMU、MCU 接收链路工作；`NO_TX` |
| 主机完整性 | **未通过** | EXT4 `errors_count` 已升至 `9`；inode `159737` 对应文件 `Bad message`；`dpkg -V` 仍 `51` 行差异/不可读 |
| 整车实车验收 | **未通过** | 地面几何 `pitch_not_calibrated`；现场检测为 `0`；方框物理状态未知；未做底盘、机构或 `carry_once` |

当前剩余现场前置条件：

1. **相机标定与静止姿态不匹配**：`config/camera.yaml` 的 `ground_camera_pitch_cdeg`、`pitch_model_reference_cdeg`、`pitch_model_min_cdeg`、`pitch_model_max_cdeg` 均为 `500`，即只在内部 `+5°`（RX125）标定有效；现场静止姿态为 RX120（内部 `0°`）。因此 `pitch_not_calibrated` 是正确拒绝，不是协议解析失败，也不能通过直接改标定值宣告通过。
2. **方框实际开/关状态未知**：A6 当前动作编号为 `0`，没有 `known_frame.json`；旧 `frame_20261008.assume.json` 明确标记为临时假设，`a6_open_state/a6_close_state` 均为 `-1`，不能据此猜测当前状态或编号。

3. **主机完整性仍未恢复且本轮继续恶化**：`/sys/fs/ext4/mmcblk0p2/errors_count` 在 `17:37–18:00` 间由 `3` 升至 `4`、`6`、最终 `9`；`dmesg` 的 EXT4 `checksum invalid` 均指向 inode `159737`，对应 `/usr/lib/libreoffice/share/config/soffice.cfg/modules/swriter/ui/printoptionspage.ui`，`stat` 返回 `Bad message`。`dpkg -V` 输出 `51` 行（`20` 个 `??5` 修改、`23` 个 `missing`、`8` 个不可读、其中 `1` 个 `Bad message`），且无 sudo 导致部分检查受限。历史记录中的 `cc1plus` 7 字节差异、GCC ICE 和 EXT4 错误不能视为已恢复。

在上述三项解决前，不应绕过门控去点动底盘或开关方框。本轮结论按“协议/离线通过、只读链路部分通过、主机完整性与整车未通过”分别记录，不能把三者合并成实车通过。

## 已完成的工作

### 1. 隔离构建与回归

- ARM Release 构建成功。
- CTest：`41/41` 通过，总耗时 `9.25 s`。
- 隔离二进制：
  - 路径：`/home/cat/refactor-validation/20261024-vehicle-acceptance/build/rescue_upper_host`
  - SHA-256：`239a807a26f09a936dac1a110ab56d4556e4b352f69737032e0839a4e6ccc34a`
  - 大小：`1301728` bytes
- 证据：
  - `evidence/ctest.log`
  - `evidence/ctest.xml`
  - `evidence/binary.sha256`
  - `evidence/build.log`

### 2. 源码快照与完整性

- 隔离源码清单共 `345` 个文件。
- 从 `source/` 目录用 `sha256sum -c ../evidence/source.sha256` 复核：`345/345 OK`。
- 构建前后清单 `source.sha256` 与 `source-after-build.sha256` 完全一致（`cmp` 返回 0）。
- 本地归档：`reports/refactor-validation/vehicle-acceptance-20261024/source-before.tar.gz`
- 归档 SHA-256：`133169966869d5ec26c13ff758cbf9194f978ab9c5cb6afa101e34d3d2f3fe97`

### 3. 主机与设备预检

本轮只读复查（最后快速复查 `2026-10-24T17:59:44–18:00:57+08:00`，证据 `remote-evidence/current-host-audit/current-host-audit-1801.log`；详细审计窗口 `17:56:02–17:57:16` 及 `17:40` 审计均保留）：

- aarch64，Linux `6.1.118`；根分区 `/dev/mmcblk0p2` 以 ext4 `rw,relatime` 挂载。
- **根文件系统未通过完整性检查且持续恶化**：`/sys/fs/ext4/mmcblk0p2/errors_count` 的观测序列为 `17:37: 3 -> 17:51: 4 -> 17:56 审计: 6 -> 17:57+ 最终复查: 9`；`17:58` 审计文件内 `dmesg` 有 `6` 条、`18:01` 审计文件内也有 `6` 条 `EXT4-fs error ... ext4_lookup ... checksum invalid`，且均指向 inode `159737`。`journalctl -k` 因无 `adm/systemd-journal` 权限显示 `-- No entries --`，不能解释为无错误；可读的 `dmesg` 已保存。
- **损坏 inode 已定位到真实文件**：inode `159737` 对应 `/usr/lib/libreoffice/share/config/soffice.cfg/modules/swriter/ui/printoptionspage.ui`；`stat` 与 `ls` 均返回 `Bad message`，`dpkg -S` 归属 `libreoffice-uiconfig-writer`。这是文件系统元数据/文件损坏，不只是 `dpkg` 摘要记录异常。
- eMMC `life_time=0x01 0x01`、`pre_eol_info=0x01`；这是设备报告的磨损/寿命等级，不等于故障已排除。
- **`dpkg -V` 不是无差异**：输出 `51` 行，按首字段分为 `20` 个 `??5??????`（修改）、`23` 个 `missing`、`8` 个 `?????????`（不可读/未知）；其中 `1` 行含 `Bad message`，`13` 行含 `Permission denied`。因此只能确认“存在多处差异/不可读项”，不能把这 `51` 行全部判为确定损坏文件，也不能判为完整。
- `cc1plus` 当前路径为 `/usr/libexec/gcc/aarch64-linux-gnu/13/cc1plus`，当前 SHA-256 为 `110b837d05f403bcd21a63eb1f2341338d613aaf3d977d7db6d2c1661c8afa7d`，与可信官方包内文件一致；但历史记录曾读到 `42528bf65af333350aaea6d21c775b30f22cf9129a498a176548929f5c25490f` 且与官方包有 `7` 个字节差异。当前一次读取一致**不能**解释历史差异或宣告主机恢复。
- `/dev/video0`、`/dev/video1` 存在；`/dev/ttyACM0` 为 MCHCK USB CDC ACM（MCU）；`/dev/ttyUSB0` 为 CP2102N（IMU）；`match-main.service=inactive`，四个设备均无进程占用。
- RK3588 i2c 控制器 `rk3x-i2c fea90000.i2c` 持续报 timeout；17:37 审计计数为 `4088`，之后一次采样为 `4089`，但 18:00 读取为 `4080`（`dmesg` 窗口/滚动导致计数不可直接比较）。该链路对应 `i2c-1` / `fusb302@22`，不在本次轮式/方框运动链上，只能作为线索，不能据此断定根因。
- 无 `/sys/devices/system/edac`；温度约 `30.5–33.3°C`；`match-main.service=inactive`、`NRestarts=0`；`/dev/video0`、`/dev/video1`、`/dev/ttyACM0`、`/dev/ttyUSB0` 无进程占用。相机为 RYS USB Camera（`0bda:1376`，序列号 `200901010001`）；MCU 为 MCHCK USB CDC ACM；IMU 为 CP2101N/CP210x，`ttyUSB0`。

资源摘要：

```text
db6ef05746699792028de7514420abf07e56cb4f2d8d6eabb1d1233bd25b4966  models/detect_fp.rknn
fb60b26d4390f37df6b6826647a4025018204576c3c75665c86bca88c7d50b12  models/zone_pose_fp.rknn
d31fc19c85b85f6092b2bd0f6af9d962d5264a4e410bfb536402ec92bac738e8  benchmark_results/librknnrt.so
153f9815d2388337ef87985efa7e67e886938d942065019d6b5fe33906568c73  config/camera.yaml
ce34ee54295c8fbb1202569fa346467a93b6d0e1b467ba74aeb28e74ce0e767e  config/zone_geometry.json
```

补充：`fusb302/i2c-1` 相关 timeout 在当前内核日志中持续出现；该链路不在本次运动链上，但尚未证明完全无害。上述设备、温度和服务状态只描述当前读数，不能抵消 EXT4 错误与文件损坏。

### 4. 只读感知预览（协议同步前，历史证据）

> 本节是**协议同步前**的旧证据，仅保留用于对照，不代表当前实现结果。协议同步后的新版复验见第 5 节。

执行隔离二进制的 `search --preview --seconds 10`，同时使用 `--dry-run` 和 `--pitch-feedback`；MCU 串口只读打开，发送路径拒绝 TX。输出包含 `NO_TX`。

观测结果：

```text
[IMU] fresh=1 age_ms=7.283 seq=1025 crc_errors=0
body_rpy_rad=-0.00548032,-0.0381351,3.13969
[MOTION] imu_fresh=1
[GEOMETRY] mapped=0 reason=pitch_not_synchronized
[MATCH] preflight=gripper_calibration_required
[PIXEL_SELECTION] pitch=-32768 reason=invalid_image_or_pitch
[PERF] loop_fps≈9.7 inference_ms≈95 detections=0
```

说明：

- 相机能采集、RKNN 能推理，IMU 与 MCU 接收链路工作。
- 当前画面没有 detections；因为 pitch 无效，地面几何无法建立。
- 循环约 `9.7 FPS`，低于配置的 `30 FPS`，属于下一步性能验收项，不单独解释为故障。
- 进程退出码 `2` 是 `DEMO_RESULT wall_budget_exhausted mode=search` 的预算到期，不是崩溃。
- 证据：`evidence/preview.log`，以及隔离源码中的 `source/telemetry_logs/demo/20261024-165150-search-8772.log`；旧版 profile 在旧归档中，不与新版派生文件混用。

### 5. 协议同步后的新版验证（2026-10-24 17:11–18:01）

#### 5.1 隔离源码、构建与回归

隔离目录：

```text
cat:/home/cat/refactor-validation/20261024-vehicle-acceptance-protocol/
```

结果：

- 源码清单 `362` 个文件，`sha256sum -c` 全部 OK；清单 SHA-256 为 `ebc68ee1fc57dd35d088bdd7504b16ab21c6442a96fbf46fc4b0a9ff868b8620`。
- 构建前后清单 `source.sha256` / `source-after-build.sha256` 完全一致（`cmp` 返回 0）。
- `binary.sha256` 同时记录源码清单 SHA-256 `ebc68ee1fc57dd35d088bdd7504b16ab21c6442a96fbf46fc4b0a9ff868b8620`、二进制 SHA-256 与工具文件哈希。
- 远端 ARM CTest：`41/41 passed`，`15.86 s`。
- 新版隔离二进制：
  - 路径：`/home/cat/refactor-validation/20261024-vehicle-acceptance-protocol/build/rescue_upper_host`
  - SHA-256：`8bc897da951438fb529d3b70e8f23e5da09c3a583e8a60668b3a6eb52990e51a`
  - 大小：`1301776` bytes
- 工具同步后二进制未变化；`tools/camera_pitch/pitch_ctl.py` 与 `tests/test_pitch_prepare5.py` 的 SHA-256 分别为：
  - `1a0b71c5aef93f89927b006fe5f219d1c55e06baa2b8f925c19ddfd6e2813db8`
  - `b46b8bb7f3f584c4335cacb4d7d3308a610678efa0ca860db6db83a249b542e1`
- 本地离线 CTest：`41/41 passed`，`10.34 s`（日志 `/tmp/ds-acceptance-ctest-protocol3.log`）。
- 本地工具测试：`test_pitch_prepare5.py` `8 tests OK`，`test_frame_ctl.py` `14 tests OK`；冻结基线 SHA-256 未变化。
- 证据：`evidence/source.sha256`、`evidence/source-after-build.sha256`、`evidence/binary.sha256`、`evidence/cmake-configure.log`、`evidence/build-pre-tools-sync.log`、`evidence/build.log`、`evidence/ctest-pre-tools-sync.log`、`evidence/ctest.log`。

#### 5.2 A6 只读读回

`match-main.service=inactive`，串口无占用。新版工具以只读方式打开 `/dev/ttyACM0`，未写字节：

```text
当前: 动作编号=0 gripper_open=0 pitch=120.00° (12000 cdeg)  RX: A6 00 00 [78 00] 0E 19 0A
```

`query-lower.py` 只读结果：

```text
state=0 id=0 pitch=120 samples=5 stable=1
```

`pitch-watch` 连续 8 次稳定读回 `120.00°`。新版 C++ 按 `90..150°`、`120°` 为零点解析，得到内部 `pitch=0`；不再出现 `-32768`，也不再是同步失败。A6 帧中的 `00 00`（状态 0、编号 0）仍**不能**解释为方框开或关。

证据：`evidence/pitch-watch-readonly.log`、`evidence/query-lower-readonly.log`。

#### 5.3 新版只读预览结果

执行新版隔离二进制，参数仍为 `--dry-run --pitch-feedback`，`search` 预览 `10 s`；整个预览持续 `NO_TX`。原始日志共 `501` 行，SHA-256 `f223488ecfc5175b40f38d66b09e1923344fa42f095d49c326e6992fe76fb75e`。

关键输出：

```text
[PIXEL_SELECTION] pitch=0 stable=1 id=-1 reason=no_legal_candidate
[GEOMETRY] mapped=0 reason=pitch_not_calibrated
[PERF] ... inference_ms≈95 ... detections=0
[DEMO_SEARCH_PREVIEW] target=-1 locked=0 ... NO_TX
[DEMO_RESULT] wall_budget_exhausted mode=search
process_exit=2
```

从原始日志重算的统计（不是沿用旧日志）：

| 指标 | 全部 19 个 `[PERF]` 样本 | 剔除首帧预热后的 18 个样本 |
| --- | --- | --- |
| `loop_fps` 平均 / 中位 / 最小 / 最大 | `8.735` / `9.539` / `0` / `9.633` | `9.220` / `9.539` / `6.631` / `9.633` |
| `inference_ms` 平均 | `95.373` | `95.429` |
| `detect_ms` 平均 | `44.764` | `44.758` |
| `pose_ms` 平均 | `47.192` | `47.251` |
| `capture_ms` 平均 | `7.462` | `7.483` |
| `detections` 合计 | `0` | `0` |

其他计数：

- `GEOMETRY`：`19/19` 为 `pitch_not_calibrated`。
- `PIXEL_SELECTION`：`93` 行匹配前缀。**只有 `90` 行是单行完整且可解析的选择决策，全部为 `no_legal_candidate`。** 其余 `3` 行不能作为决策样本：`1` 行为独立 `[HOLD]`，`1` 行 `reason=n` 后拼接 `[HOLD]`，`1` 行前缀后直接拼接 `[HOLD]`。上一版文档的“`91` 次正常选择”表述已纠正。
- **原始日志存在跨记录拼接/截断**：1-based 行 `58, 136, 214, 293, 372, 450` 有 `[HOLD]` 拼入其他记录；另有 `91, 169, 247, 326, 405, 472` 为无标签续行/截断片段。`PERF` 和 `GEOMETRY` 各 `19` 行均可完整解析，未受影响；其余逐行计数存在不确定性，不能靠日志修补猜出缺失样本。
- `NO_TX`：`93` 行（`92` 行 `DEMO_SEARCH_PREVIEW` 均含 `NO_TX`，另有 `1` 行结果摘要）。
- `DEMO_RESULT`：`1` 行，`wall_budget_exhausted`；退出码 `2`，不是崩溃。
- IMU 末次快照：`fresh=1 age_ms=6.855 seq=1038 crc_errors=0`。
- profile：本地派生文件 `reports/refactor-validation/vehicle-acceptance-protocol-20261024/remote-evidence/preview-profile.json`（已用 `python3 -m json.tool` 校验；远端 `evidence/` 没有该文件）。

说明：

- A6 反馈已正确转换为 `pitch=0`，但 `camera.yaml` 的可用标定范围是 `500..500 cdeg`，因此 `pitchUsable(0)` 为假；`mapped=0 reason=pitch_not_calibrated` 是标定门控的正确结果。
- 当前画面没有检测目标，`detections=0` 不能用于评价检测准确率、目标选择、区域身份或地面几何精度。
- 帧率仍明显低于配置的 `30 FPS`；本次只有约 `10 s` 的短时样本，不能据此给出长期稳定性能结论。
- 整个预览没有发送车轮、底盘、相机舵机或方框动作。
- 证据：`evidence/preview-rerun.log`、`evidence/preview-fd-flags.log`，以及本地派生 `remote-evidence/preview-profile.json`；旧版对照为 `evidence/preview-protocol-1720.log`、`evidence/preview-fdcheck-old.log`。

#### 5.4 真实图片离线推理串/并行复验

使用新版隔离二进制对 `7` 张真实图片分别执行串行与并行推理，每张各 `2` 次，共 `14/14 rc=0`。证据：`remote-evidence/offline-image-rerun/offline-image-rerun.log`（`192` 行，SHA-256 `8fb787ab2c38fe9b321d09a468e725c52ffb4f5807fee768c6b3e7ec41c95b7a`）及 `offline-image-serial-parallel-comparison.json`（SHA-256 `500bc615a2a1a8a797eb9401aa7aa54aac1efa6ab60c9b5c43284da5eea5e18e`）。

| 图片 | 串行检测数 | 并行检测数 | 串/并行归一化 |
| --- | ---: | ---: | --- |
| `zone_20260930_010257_000020.jpg` | 4 | 4 | 一致 |
| `zone_20260930_211755_000620.jpg` | 0 | 0 | 一致 |
| `zone_20260930_212538_000400.jpg` | 9 | 9 | 一致 |
| `holding_40_injured.png` | 1 | 1 | 一致 |
| `holding_40_block1.png` | 1 | 1 | 一致 |
| `holding_40_block2.png` | 1 | 1 | 一致 |
| `holding_40_block3.png` | 2 | 2 | 一致 |

- 每张图串行与并行输出在**仅剔除以 `[PERF]` 开头的行后**逐字相同；`all_14_normalized_identical=true`。
资源哈希核对：

| 图片 | 当前 SHA-256 | 与旧记录对照 |
| --- | --- | --- |
| `zone_20260930_010257_000020.jpg` | `4352d95cd905dbc0afcfcd179dc556db11dab14427718b1351ae0a57058b7e1a` | 与 `CAT_HOST_20261020` 一致 |
| `zone_20260930_211755_000620.jpg` | `adc8cd90f756f307bbcf106f341688251d6c2e8e4daf31ebac8bf0df7454ce85` | 与 `CAT_HOST_20261020` 一致 |
| `zone_20260930_212538_000400.jpg` | `ee28883f1296dc1b32a08c654745f6760a33541b81062b3135659e2375d90ca2` | 与 `CAT_HOST_20261020` 一致 |
| `holding_40_injured.png` | `7660776538ec20e7339cb7c2e989e4d3aa05fd936059d1ec2b7c761892cf86da` | 与旧记录一致 |
| `holding_40_block1.png` | `1a721125cd8085d9978ab151aa072b46cdb442b971718ab6ae6a549900b35e43` | 旧记录未明确核对 |
| `holding_40_block2.png` | `b8f12c0e99d213f253b2e53c77520fe7c3b13993486cfaf5a94fa0b8673ee206` | 旧记录未明确核对 |
| `holding_40_block3.png` | `bc64c9c6babcafaa64d2226a4a1381e15b0b528e882b06b210b8cba7152c3858` | 与旧记录一致 |

前 `3` 张 zone 图片与旧 `CAT_HOST_20261020` 完全一致；`holding_40_*` 中 `injured` 与 `block3` 与旧记录一致，`block1`、`block2` 未见旧记录明确核对。因此只能说本批串/并行输出稳定一致，**不能**宣称与旧批次逐资源完全等价。
- 颜色诊断全部为 `[ZONE_COLOR] color= verified=0 own=0 reason=color_unmeasured`。
- 该组只验证模型入口、输出稳定性和批内复现性，**不评价检测精度、目标选择、区域颜色/身份或现场地面几何**。

#### 5.5 相机-only 只读预览（真实 PID fd 采样）

使用隔离二进制 `rescue_upper_host`（SHA-256 `8bc897da951438fb529d3b70e8f23e5da09c3a583e8a60668b3a6eb52990e51a`），直接后台启动并采集真实应用 PID，避免把 `timeout` 包装进程当作目标：

```bash
--dry-run --no-show --vision-log-dir none \
--demo-mode recognize --demo-seconds 15 \
--camera 0 --width 1280 --height 720 --fps 60 --team blue
```

未使用 `--pitch-feedback`、`--imu`、`--hardware`，未连接 MCU。结果：

- `PID=43261`；`36` 次 `FD_SAMPLE`。
- `33/36` 个样本出现 `/dev/video0`，flags 为 `0404002`（`O_RDWR`）。
- `/dev/ttyACM0`：`0` 次；`/dev/ttyUSB0`：`0` 次。
- `PROC_EXIT pid=43261 rc=0 samples=38 wall_ms=15925`；`[DEMO_RESULT] wall_budget_exhausted mode=recognize`。
- 证据：`remote-evidence/camera-only-rerun/camera-only-fd-rerun.log`（`1441` 行，SHA-256 `b19df0abcc5fae9c8bbe4fd61ec527ba087c7aa7b8c014044b745b34424cbb16`）、`camera-only-fd-summary.json`。

性能统计（`28` 条完整 `[PERF]`；稳态为剔除首尾各 `3` 条后的 `22` 条）：

| 指标 | 全样本均值 | 稳态均值 | 稳态中位 | 稳态范围 |
| --- | ---: | ---: | ---: | --- |
| `loop_fps` | `9.403` | `9.925` | `9.916` | `9.837–10.004` |
| `inference_ms` | `92.844` | `92.786` | `92.803` | `88.704–96.532` |
| `detect_ms` | `44.317` | `44.228` | `44.192` | `42.420–45.999` |
| `pose_ms` | `46.443` | `46.471` | `46.396` | `43.754–48.590` |
| `capture_ms` | `7.500` | `7.505` | `7.504` | `7.418–7.552` |
| `detections` 合计 | `0` | `0` | — | — |

说明：

- 循环约 `10 FPS`，显著低于配置的 `60 FPS`；主要受同步推理约 `93 ms` 限制，**不能据此宣称实时性能通过**。
- 未提供 pitch，故 `pitch=-32768`、`reason=invalid_image_or_pitch`、`preflight=imu_not_ready` 是预期门控；当前画面 `detections=0`，不能评价现场检测、目标选择、区域颜色或地面几何。
- `remote-originals/` 中的旧远端 `/tmp` 日志只是原件保全；其 fd 采样目标是 `timeout` 包装进程，**不能**据此表述“相机未打开”或“串口未打开”。本节的真实 PID 日志才是有效 fd 证据。
- 整个运行未发送车轮、底盘、相机舵机或方框动作。

#### 5.6 只读句柄证据（带 A6/IMU 预览）

预览进程 `pid=27926` 的 `/proc/<pid>/fdinfo` 采样：

```text
fd=13 path=/dev/ttyACM0 flags=06410000
fd=11 path=/dev/ttyUSB0 flags=02404002
process_exit=2
```

低两位访问位：

- `/dev/ttyACM0`：`00`，即 `O_RDONLY`（MCU 只读）。
- `/dev/ttyUSB0`：`02`，即 `O_RDWR`（IMU 串口读写）。

注意：`preview-fdcheck.log` 只是运行日志，**不包含 fd flags**；fd 证据是 `preview-fd-flags.log`。

#### 5.7 静态配置预检：两组结果

**第一组：原始预检（未带任务标定等资源）**，证据 `preview-check-config.log`：

```text
MISSING task_calibration
MISSING new_frame_mapping_and_required_calibration
MISSING load_footprint_fits_half
OK far_track_ground_calibration
MISSING zone_identity_source
OK zone_pose_source
STATIC_ONLY: ... hardware_output=disabled
```

**第二组：按 `config/runtime/preview.json` 指定完整资源后的只预检**，证据 `preview-check-config-full.log`；命令为：

```bash
/path/to/isolated/rescue_upper_host --check-config \
  --calibration config/camera.yaml \
  --pose-model models/zone_pose_fp.rknn \
  --task-calibration config/task_calibration.40_runtime.json \
  --zone-color-calibration config/zone_color.field.json \
  --zone-geometry config/zone_geometry.json
```

输出：

```text
OK task_calibration
MISSING new_frame_mapping_and_required_calibration
OK load_footprint_fits_half
OK far_track_ground_calibration
OK zone_identity_source
OK zone_pose_source
```

退出码 `2`，唯一缺项是方框已知状态/映射；该检查不打开设备，也不输出动作。

补充说明：

- `OK far_track_ground_calibration` 只表示配置的 `500 cdeg` 落在标定范围内，**不代表当前 RX120 姿态 (`0 cdeg`)** 可用于几何换算。
- 第二组的 `OK` 仅说明静态资源可被解析并满足对应预检条件，不能替代真实像素/颜色/位姿链的现场核对。
- 这些检查是静态配置检查，不能替代现场传感器时序、真实检测、机构动作和长时间运行证据。

## 关键发现：A6 协议不一致（已在协议层解决，并完成新版只读复验）

> 状态：**协议口径与新版只读解析均已通过；整车实车验收未完成。** `90..150°`、`120°` 为零点的解析已同步到 C++、工具、测试和协议文档；本地与远端 ARM 均 `41/41`，新版 A6 读回已正确解析为内部 `0`。本节保留协议同步前的失败采样作为历史证据；新版结果见第 5 节。

现场以 `O_RDONLY` 打开 `/dev/ttyACM0`、写入 0 字节，连续采样 250 帧，结果全部为：

```text
A6 00 00 78 00 0E 19 0A
state=0 id=0 pitch=120
unique-state-id-pitch=[(0, 0, 120)]
```

采样证据：`evidence/a6-readonly-20261024.json`，SHA-256 `2e998cb251a41384a60d89cc848463a50234a81b180faae05463417cffb94fb3`；本地副本为 `reports/refactor-validation/vehicle-acceptance-20261024/remote-evidence/a6-readonly-20261024.json`。

两个工具对同一帧的解释不同：

- `frame_ctl.py status`：显示“相机实际角=120°”
- `pitch_ctl.py status`：显示“pitch=无效 (-32768 cdeg)”
- 本机 C++ `cameraPitchFromWire()` 只接受 `0..80`，`120` 被判为 `kCameraPitchInvalid`

### 根因

`cat:/home/cat/ds_innovation` 的 `school-competition` 分支已有 2026-10-23 的未提交协议升级，本机重构分支尚未移植：

| 项目 | 本机重构分支（旧） | cat 部署分支（新，已实测） |
| --- | --- | --- |
| A6 pitch 范围 | `0..80°` | `90..150°` |
| A6 零点 | `40°` | `120°` |
| 内部零位 | `0 cdeg` | `0 cdeg`（偏移坐标） |
| TX pitch 范围 | `-40..+40°` | `-30..+30°` |
| 夹爪线缆值 | 业务值直接发 `0/20` | 业务 `0/20` 映射为线上 `+5/-5` |
| A6 状态含义 | 旧二值状态 | 当前实测 `state=0`；开闭物理含义仍未重新验收 |

部署分支的实测记录为：保持零速度，依次发送相机偏移 `0/+5/-5/0`，A6 稳定回传 `120/125/115/120`，CRC 均正确，夹爪编号保持 `0`。这只证明反馈数值与发送数值对应，**不证明舵机实际几何角度或视觉标定有效**。

### 影响（协议同步前的历史判断）

- 旧重构二进制无法把现场 A6 反馈转换为有效 pitch。
- 旧地面几何持续 `pitch_not_synchronized`。
- 旧 `frame_ctl.py` 在 pitch 超出 `0..80` 时会拒绝发送方框动作，因此当时连受控 open/close 也无法通过旧工具执行。
- 不能简单把 `120` 改成合法值后继续跑；必须完整移植协议口径并重新验证标定。

### 协议同步后的状态

- 移植已完成，本地与远端 ARM 构建 `41/41` 通过。
- 新版 A6 读回 `120°` 已正确转换内部 `pitch=0`。
- 旧工具的超范围拒绝已不再出现；但当前相机标定只覆盖内部 `+5°`，所以几何仍被 `pitch_not_calibrated` 正确拒绝。
- 下一步不是继续改协议，而是重新标定 RX120，或在安全确认后受控转到 RX125 复核。

## 关键发现：方框状态未知

- A6 当前 `state=0`、`id=0`。
- `/tmp/rescue-demo-1000/known_frame.json` 不存在。
- 主程序启动要求已知 angle `0/20` 且 id `1..255`。
- `config/frame_20261008.assume.json`：
  - `status=TEMP_ASSUMPTION_assume_all_safe_NOT_ACCEPTANCE`
  - `a6_semantics=done_flag`
  - `a6_open_state=-1`
  - `a6_close_state=-1`
  - `action_mapping_source` 虽写有 `open=0 close=+20`，但上层 `pending` 明确说明所有“verified”字段仍是临时假设。

因此不能用旧值直接传 `--known-frame-angle 0 --known-frame-id 1`。需要现场先确认方框物理开/关位置，再建立 A6 编号。

## 重构进度判断

本地 `main` 为 `bd73a38`，工作树有：

- 13 个已跟踪文件修改
- 29 个未跟踪文件/目录
- 重构成果**全部未提交**
- `git diff --check` 通过
- 两个冻结基线的 SHA-256 未变化：
  - `c828d652eb7316c817ab6182ba13f8202496b59f9b66b06dd9abdbd108b4d79f  tests/fixtures/runtime_legacy_baseline.json`
  - `028f36440090e2353b28b3a9f4f069715c088dcae5801fd0004e0302d0987b43  tests/fixtures/runtime_legacy_aarch64.json`

已完成：

- R0 原版离线基线
- R1 配置等价提取
- R2 严格 runtime 配置入口与兼容策略
- R3 入口拆分（`src/main.cpp` 16 行）
- R4 七批：捕获核验、交付核验、释放后退区、现场推入、共享清扫账本、清扫单转、往复摆动与归中

未完成：

- R4 剩余：搜索接近、清扫冲入及触发/收尾、严格 ENTER、其余搬运恢复
- R5：部署入口迁移、二进制对应关系、旧入口与历史代码清理
- `src/task/push_task.cpp` 仍有 2378 行，不能称为 R4 完成
- 协议基线已同步到 `cat` 部署分支的 2026-10-23 口径；新增 `tests/fixtures/runtime_protocol_20261023.json` 作为版本化补充，冻结基线未改

重构不是“推倒重来”，目前是**可构建、可回归的完整快照**，但还不是可部署的实车版本。

## 部署与安全边界

- 不替换 `/home/cat/ds_innovation/build-school-competition/rescue_upper_host`。
- 不使用 `tools/match/match_main.sh` 默认旧二进制；实车验证必须显式指定隔离构建二进制。
- 远程 `school-competition` 分支有大量未提交改动，已保留，不覆盖、不清理。
- 当前未修改源码、未提交、未切换部署。

任何车轮/机构动作前必须同时具备：

- 场地、机构前后和旋转扫掠区清空
- 无人员、无对手进入
- 现场急停可用并由专人值守
- 方框实际状态已由现场确认

**不要猜方框开/关方向，不要使用旧 `--known-frame-angle/id` 假设值。**

## 下一步建议顺序

### 第 0 步：现场确认（不需要运动）

1. 确认方框当前物理开/关位置。
2. 确认 MCU/固件版本。
3. 确认扫掠区已清空、无人员/对手、急停就位。
4. 确认现场允许进行只读采样和受控单动作。

### 第 1 步：协议移植（已完成）

以下项目已同步并纳入 `41/41` 回归，保留作为验收记录，不需要重复移植：

- `include/rescue/core/types.hpp`：`kCameraPitchLimitCdeg = 3000`、`kCameraProtocolZeroDeg = 120`、`cameraPitchFromWire()` 接受 `90..150`
- `src/hardware/uart_controller.cpp`：业务夹爪 `0/20` 映射线上 `+5/-5`
- `tools/camera_pitch/pitch_ctl.py`：界面与 A6 使用 `90..150°`，TX 偏移 `-30..30`
- `tools/frame/frame_ctl.py`：A6 有效范围 `90..150`，动作编号仅在偏移变化时递增
- `tools/send_velocity.py`、`docs/通信协议.md` 与相关测试
- 新增 `tests/fixtures/runtime_protocol_20261023.json` 作为版本化补充；冻结基线未改

### 第 2 步：离线回归（已完成）

本地与远端 ARM 均：

```text
41/41 tests passed
```

覆盖 `89/90/120/150/151/-32768` 的 pitch 边界，以及夹爪 `0 -> +5`、`20 -> -5` 映射。旧冻结基线保持原 SHA-256，未篡改。

### 第 3 步：只读实车复核（协议解析通过，几何被标定门控拒绝）

使用隔离二进制的 `--dry-run --pitch-feedback`：

- **已通过**：A6 回传被解析为 `pitch=0`，且 `[GEOMETRY]` 原因不再是 `pitch_not_synchronized`
- **未通过**：`[GEOMETRY] mapped=0 reason=pitch_not_calibrated`
- **未验证**：真实 `detections>0`、目标选择、区域身份与几何精度
- **已记录**：稳态约 `9.22 FPS`，`inference_ms≈95.4`，`NO_TX=93`，退出码 `2` 为预算到期
- **原因**：`camera.yaml` 只在内部 `+5°`（RX125）标定，当前静止姿态为 RX120（内部 `0°`）

### 第 3a 步：重新建立当前姿态的相机标定

二选一，均需现场条件允许：

1. 在 RX120 静止姿态重新做地面标定，生成绑定 `0 cdeg` 的 `camera.yaml`；或
2. 在安全确认后受控转到 RX125，复核既有 `+5°` 标定。

不得直接把 `pitch_model_min/max` 改成 `0` 来绕过验收；`config/camera.mechanical_5_40.yaml` 明确 `extrinsics_validated=0`、`mechanical_assumption=1`，且 `40deg NOT field validated`，不能作为捷径。

重新标定后，必须重跑只读预览并确认：

- `[GEOMETRY]` 不再出现 `pitch_not_calibrated`
- 画面中放置真实目标时出现 `detections>0`
- 目标选择、区域身份与地面几何有独立核对

### 第 4 步：受控方框动作

只有在第 0 步现场确认完成后：

- 用明确的已知状态/编号建立 `known_frame`
- 先做单次开或单次关，保持底盘零速度
- 现场目视核对物理开/关与 A6 状态、编号
- 不根据旧配置猜测方向

### 第 5 步：小幅底盘点动

- 最小速度、最短时长
- 清空扫掠区、急停值守
- 验证方向、停车、超时和急停链路

### 第 6 步：相机俯仰与 `carry_once`

- 先验证单档 pitch 与视觉几何
- 再验证一次完整搬运
- 最后才跑整场流程

## 证据入口

### 新版（协议同步后，2026-10-24 17:11–18:01）

本地归档：

```text
reports/refactor-validation/vehicle-acceptance-protocol-20261024/
├── remote-evidence/
│   ├── binary.sha256
│   ├── binary-pre-tools-sync.sha256
│   ├── source.sha256
│   ├── source-after-build.sha256
│   ├── source-pre-tools-sync.sha256
│   ├── source-after-build-pre-tools-sync.sha256
│   ├── cmake-configure.log
│   ├── build.log
│   ├── build-tools-sync.log
│   ├── build-pre-tools-sync.log
│   ├── ctest.log
│   ├── ctest-tools-sync.log
│   ├── ctest-pre-tools-sync.log
│   ├── pitch-watch-readonly.log
│   ├── query-lower-readonly.log
│   ├── preview-rerun.log
│   ├── preview-protocol-1720.log
│   ├── preview.log
│   ├── preview-fd-flags.log
│   ├── preview-fdcheck.log
│   ├── preview-fdcheck-old.log
│   ├── preview-check-config.log
│   ├── preview-check-config-full.log
│   ├── preview-profile.json
│   ├── preview-profile-recomputed.json
│   ├── offline-image-rerun/
│   │   ├── offline-image-rerun.log
│   │   └── offline-image-serial-parallel-comparison.json
│   ├── camera-only-rerun/
│   │   ├── README.md
│   │   ├── camera-only-fd-rerun.log
│   │   ├── camera-only-fd-summary.json
│   │   ├── camera-only-perf-summary.json
│   │   ├── camera-only-lsof.log
│   │   ├── camera-only-procfs.log
│   │   ├── camera-only-rerun.log
│   │   └── remote-originals/  (9 个远端 /tmp 原件副本)
│   └── current-host-audit/
│       ├── current-host-audit.log
│       ├── current-host-audit-1758.log
│       ├── current-host-audit-1801.log
│       └── post-audit-storage.log
├── tools/summarize_preview_log.py
├── README.md
├── remote-evidence-summary.json
├── remote-evidence.sha256
├── remote-evidence-remote-originals.sha256
└── source.sha256
```

远程：

```text
cat:/home/cat/refactor-validation/20261024-vehicle-acceptance-protocol/
├── build/rescue_upper_host
├── evidence/
├── source/
└── telemetry_logs/
```

归档说明：

- 远端 `evidence/` 的 `22` 个文件已逐文件回收到本地 `remote-evidence/` 并核对一致；`preview-profile.json` 为本地派生档案，不是远端原件。
- `preview-check-config-full.log` 与 `current-host-audit/` 是本轮新增的本地证据，远端 `evidence/` 尚无同名文件；不能把它们标成远端原件。远端可用 SSH 只读重跑，本轮未向远端写入这两个文件。
- `source.sha256` 中的路径以 `./` 开头，必须在解包后的 `source/` 目录内执行 `sha256sum -c ../source.sha256`（`../evidence/source.sha256` 为远端原位置）；直接在归档根目录校验会全部报找不到文件。
- `remote-evidence.sha256` 覆盖本地归档中的全部证据文件及汇总脚本，共 `48` 项；`remote-evidence-remote-originals.sha256` 单列远端原件副本，共 `22` 项。两者的校验命令分别为 `sha256sum -c remote-evidence.sha256` 与 `sha256sum -c remote-evidence-remote-originals.sha256`，均在归档根目录执行。
- 归档入口与上述校验命令另见 `README.md`；`remote-evidence-summary.json` 是机器可读的结论/哈希摘要，不纳入证据清单。
- `preview-profile-recomputed.json` 由 `tools/summarize_preview_log.py` 从 `preview-rerun.log` 重算；脚本只读、不会补写损坏行。
- 新增 `offline-image-rerun/`、`camera-only-rerun/`（含 `remote-originals/`）与 `current-host-audit/current-host-audit-1758.log`、`current-host-audit-1801.log`；其中 `camera-only-fd-rerun.log` 是通过 SSH stdout 直接形成的本地证据，不是远端 `evidence/` 原件；`remote-originals/` 为远端 `/tmp` 原始副本，来源见其 `README.md`。

### 旧版（协议同步前，仅作历史对照）

- 本地：`reports/refactor-validation/vehicle-acceptance-20261024/`
- 远程：`cat:/home/cat/refactor-validation/20261024-vehicle-acceptance/`
- 旧自测二进制 SHA-256：`239a807a26f09a936dac1a110ab56d4556e4b352f69737032e0839a4e6ccc34a`
- 旧文档保留原结论，不与新版证据混写。

## 一句话交接

**A6 `90..150°`、`120°` 为零点的协议、离线构建、ARM 回归和真实图片离线推理已通过；相机-only 真实 PID fd 采样证明只打开 `/dev/video0`、未打开 MCU/IMU 串口，稳态约 `9.93 FPS`；但主机 EXT4 `errors_count` 已升至 `9`、inode `159737` 真实损坏，RX120 标定、方框状态和现场检测仍未通过，整车实车验收未通过。**
