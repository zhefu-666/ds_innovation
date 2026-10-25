# 重构版实车部署记录（2026-10-24）

> **历史记录**：本文件描述 2026-10-24 的隔离部署目录 `ds_innovation-refactor-20261024`，该目录已不存在。
> 当前入口以 [运行与调试](../运行与调试.md) 为准。下文保留原始事实。


## 结论范围

重构版已部署到车端独立目录，并通过构建、42/42 自动化回归、真实 split RKNN 模型离线推理、无运动相机/RKNN/IMU 冒烟。旧目录与旧二进制保持原样。

**尚未完成实车完整验收。** 本次未启用 `--hardware`、未启动正式整场入口、未发送任何底盘/机构/相机动作指令，也未重新标定地面几何和方框状态。R4/R5 仍以 `docs/REFACTORING_PROGRESS.md`、`docs/validation/CAT_VEHICLE_ACCEPTANCE_20261024.md`、`docs/HANDOFF.md` 的未完成项为准。

## 车端

- 主机：`lubancat`（SSH 别名 `cat`），架构 `aarch64`
- SSH 配置：`Host cat`、`HostName 10.63.153.7`、`User cat`
- 连接命令：`ssh cat`
- 新部署目录：`/home/cat/ds_innovation-refactor-20261024`
- 旧部署目录：`/home/cat/ds_innovation`（未修改）

## 源码与构建

- 源码包：`/tmp/ds-deploy-src-refactor-20261024.tgz`
- 源码包 SHA-256：`ad1e1d61d6adcf85766fc4525f433ba38592d824a3a3599222409f7feed20736`
- 部署目录自包含：模型、运行库、`deps/nlohmann_json/` 均为实际文件；目录内无外部软链接（`find . -type l` 为 0 项）
- 构建目录：`/home/cat/ds_innovation-refactor-20261024/build-arm64-selfcontained`

构建命令：

```bash
cmake -S . -B build-arm64-selfcontained \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRESCUE_ENABLE_TELEMETRY=ON \
  -Dnlohmann_json_DIR="$PWD/deps/nlohmann_json/lib/cmake/nlohmann_json" \
  -DRESCUE_RKNN_INCLUDE_DIR="$PWD/benchmark_results"
cmake --build build-arm64-selfcontained -j4
```

- 产物：`build-arm64-selfcontained/rescue_upper_host`
- 类型：ELF 64-bit LSB PIE，ARM aarch64，动态链接
- 大小：1,304,160 字节
- **最终二进制 SHA-256：`1209044c7198df0c06621cf897fc668f52b85dc7927652259988f3ac50c71274`**
- `ldd` 无 `not found`

构建/回归日志：

| 文件 | SHA-256 |
|---|---|
| `/tmp/deploy2-cmake.log` | `4ce86e6e266782088ae69a66430a33b81691b75d996cbd344a9f95928fab9c17` |
| `/tmp/deploy2-build.log` | `edc58c77db14658d7592a2c111423b6465c63a9a21fe0d399136b5d1b8217d1c` |
| `/tmp/deploy2-ctest.log` | `fa02c2cd7b39d7cc32e7d9c70e4444b45c08bb039e46442c17c6f3a03bbe1733` |
| `/tmp/deploy2-infer-split.log` | `32c3194e586dd7bdd35b7a725c2058d133c7b4b5e43b19aa6d4f144ed1affacc` |

## RKNN 多输出兼容修复

实车 split 模型不是单输出：detector 为 2 输出，pose 为 4 输出。原重构版只接受单输出，首次加载报 `Expected one input and one output`。本次新增并接入：

- `include/rescue/rknn_output_layout.hpp`
- `src/rknn_output_layout.cpp`
- `tests/rknn_output_layout_tests.cpp`
- 修改 `src/rknn_model.cpp`、`CMakeLists.txt`

行为：

- 单输出保持原行为；
- 2 输出按 boxes/4 类 scores 合并为 `[1,8,N]`；
- 4 输出按 boxes、2 类分数和 4 个关键点的 `x,y,visibility` 交错重排为 `[1,18,N]`；
- 校验输出数量、rank、batch、零维、形状和锚点数一致性。

回归：**42/42 通过，0 失败，总耗时 15.96 秒**。新增 `rescue_rknn_output_layout_tests` 已纳入 CTest。

真实 split 模型离线推理（检测 9 个目标，pose 输出 `zone_left`/`zone_right`、`zone_keypoints=6`，`detect_ms=35.2877`、`pose_ms=43.0196`、`total_ms=47.6862`、`parallel=1`）。`ZONE_COLOR=color_unmeasured` 是颜色未标定时的正确门控，不是加载失败。

## 模型/运行库摘要

| 文件 | SHA-256 |
|---|---|
| `models/detect_int8_split_20261022.rknn` | `1bffb403efb6b516a4c1e5a44d0b95945a173be447d442dd28a4eed3168b90f9` |
| `models/zone_pose_int8_split_20261022.rknn` | `54b9420a0c6be709ad1594bd2ffdce394fe18759ef5e0a31441a805ce5f4b3e2` |
| `benchmark_results/librknnrt.so` | `d31fc19c85b85f6091b2bd0f6af9d962d5264a4e410bfb536402ec92bac738e8` |
| `benchmark_results/rknn_api.h` | `c48e11a6f41b451a5fd1e4ad774ea60252d3d94f78bee9b21ea3d21b21deba9a` |

## 新版入口

- 入口：`tools/match/match_main_refactor.sh`
- 入口 SHA-256：`4377de72261ac66423a07b2bcd2adfba9ab5aa876f17e46788baab3e6ba7b952`
- 演示 profile：`config/demo_20261024.assume.json`
- profile SHA-256：`e3cbc4940488712914cbd13036578b6c03aef03056d8b807d8f0a9fd4577be66`

相对旧正式入口保留：

- 默认真实二进制改为 `build-arm64-selfcontained/rescue_upper_host`
- detector/pose 使用 20261022 split INT8 模型
- `--parallel-infer --detect-core 1 --pose-core 2`
- `--fps 60 --telemetry-fps 30`
- `--site-clear`、方框已知状态校验、下位机只读查询、启动锁、`taskset -c 4-7`

明确移除重构版不支持的 `--async-capture`；旧 `tools/match/match_main.sh` 未覆盖。

`--check` 已验证通过（`CHECK_ENTRY_RC=0`），确认使用新目录、split 模型及占位方框状态，且处于 `CHECK_NO_DEVICES`。日志：`/tmp/deploy-refactor-check.log`，SHA-256 `5067b15145ddc058cb68f7e5b29b4d6d02002ec4137c4ba2682e3f071f79d7e8`。

## 无运动实车冒烟

命令要点：`--dry-run --no-show --pitch-feedback`，打开 `/dev/video0`，加载两套 split RKNN，读取 `/dev/ttyUSB0` IMU；未使用 `--hardware`。限时 25 秒，`timeout` 正常终止（rc=124）。

结果：

- 相机采集、检测 RKNN、pose RKNN、IMU、遥测快照均运行；
- `[PERF] loop_fps≈26.8`（`detect_ms≈22.0`、`pose_ms≈23.6`、`capture_ms≈10.9`）；
- IMU `fresh=1`、`numeric_ok=1`、`crc_errors=0`；
- `[SAFETY]` 保持 `state=WAIT_START`，`vx=0 wz=0`；
- `[GEOMETRY] mapped=0 reason=pitch_not_calibrated`，是现有相机/俯仰标定未满足地面几何条件时的正确门控；
- 结束日志 `UART closed`。

冒烟日志：`/tmp/deploy-refactor-smoke-20261024-192142.log`，SHA-256 `b0d62bc870ebbd27a5cfb2c47f9dabeac99dbe51a1f598c51f33b593c1781897`。

首次冒烟因缺少 `--pitch-feedback` 被参数校验门控拒绝，日志 `/tmp/deploy-refactor-smoke-20261024-192134.log`（SHA-256 `1e2bc95eb03f0d86f5de90fa8e83da59553fd10c2e2ef486e8c45edeb3f5d5c6`）；该次未进入运行循环。

## 下位机只读状态核对

`tools/match/match_main_refactor.sh --status`（仅读取 A6，TX=0，不动机构）返回：

- 最近帧：`A6 00 00 78 00 0E 19 0A`
- `LOWER_STATE state=0 id=0 pitch=120 samples=5 stable=1`
- 方框动作编号 0：本次上电后未收到方框动作，A6 不含方框物理位置；本机无 `known_frame.json`
- 相机实际角 RX 120°（内部 0°），与现场 A6 RX120 的已知风险一致

状态日志：`/tmp/deploy-refactor-status-20261024-1926.log`，SHA-256 `5e8de11370b656db48aeae4fe8287580a46265dbbde962a34a6fa296cd4cd57c`。

## 本轮源码资产包

本轮新增/修改的源码、入口与 profile 已另存于工作区，避免未提交改动只存在于内存或临时构建目录；部署记录不把自身纳入包摘要，以免自引用循环。

- 本地源码树：`/home/jy/ds_innovation`（未提交，保留原分支与全部既有改动）
- 本轮相关文件：`src/rknn_output_layout.cpp`、`include/rescue/rknn_output_layout.hpp`、`tests/rknn_output_layout_tests.cpp`、`src/rknn_model.cpp`、`CMakeLists.txt`、`tools/match/match_main_refactor.sh`、`config/demo_20261024.assume.json`

## 回退基线

旧目录保持分支 `school-competition`、HEAD `223c729ad935b0466b4415995d89739b21e69dcd`、51 项未提交改动。

| 文件 | SHA-256 |
|---|---|
| `/home/cat/ds_innovation/build-school-competition/rescue_upper_host` | `8f005fad81325ee1c4f878827ccac52ad25ebe4f55d8b07690ba4ef5dabacffd` |
| `/home/cat/ds_innovation/tools/match/match_main.sh` | `b7b606852272b82ac15a7162641366d0a09c3e9e52f0e56cfee0e8bb63ea47cd` |

回退方式：停止新版入口/主程序后，直接使用旧入口 `tools/match/match_main.sh` 和旧二进制；本次未替换、未删除旧目录任何文件，无需恢复操作。

## 未验证项与启动前门槛

- 未执行 `--hardware` 运动、方框、相机俯仰和整场任务，不得宣称“实车验收通过”；
- 相机 `config/camera.yaml` 仅覆盖内部 `+5°`，现场 A6 曾为 RX120（内部 `0°`）；未重新标定前地面几何结果无效；
- 方框物理状态未知（A6 最近 `state=0/id=0`，无 `known_frame.json`）；
- 正式 `--run` 前必须：清空场地和扫掠空间、确认无人员/对手、急停可用且有人值守、确认方框实际状态，并完成 `--check` 与只读 `--status`；
- 运动或机构动作必须另做现场验收并留存证据。
