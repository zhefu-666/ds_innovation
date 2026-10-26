# cat 构建性能记录 — 2026-10-26

## 环境与方法

实测主机：cat@lubancat；源码：/home/cat/ds_innovation。
Ubuntu 24.04.4、RK3588S（4×Cortex-A55 + 4×Cortex-A76）、约 7.7 GiB RAM、2 GiB swap。
CMake 3.28.3、GCC 13、Ninja；Release（-O3 -DNDEBUG）、C++17、RKNN/遥测/BUILD_TESTING 均 ON。
ccache 4.9.1 来自 Ubuntu arm64 软件包，用户目录安装，不需要 sudo；缓存上限 2 GB。

前后均使用 /home/cat/build-benchmark-20261026/build，同一源码树和 cat-release 配置。
每档并行度使用独立空缓存，先清理构建产物并做冷缓存全量构建，再清理产物并做热缓存全量构建。
每次构建前空闲 5 秒；未绑定 CPU、未调整频率或清理 Linux 页缓存。表中每项为单次实测，不是多次统计。
用时不包含配置、clean 与等待；冷缓存也允许同轮重复源文件命中。热缓存不是 no-op 增量构建。
每 0.5 秒采样系统 MemAvailable、已用 swap、最高 thermal_zone 温度；内存为系统可用值，并非编译器峰值 RSS。

## 结果（秒）

| 并行度 | 重构前冷缓存 | 重构前热缓存 | 重构后冷缓存 | 重构后热缓存 | 同档冷缓存提速 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 4 | 162.550 | 4.763 | 146.125 | 3.757 | 10.1% |
| 6 | 142.269 | 3.514 | 127.483 | 2.907 | 10.4% |
| 8 | 135.176 | 2.904 | 126.672 | 2.534 | 6.3% |

重构后冷缓存资源采样：

| 并行度 | 最低可用内存 MiB | 最大已用 swap MiB | 最高温度 °C |
| --- | ---: | ---: | ---: |
| 4 | 5744.2 | 0.0 | 67.5 |
| 6 | 5424.1 | 0.0 | 69.3 |
| 8 | 5123.1 | 0.0 | 73.9 |

全部 12 个计入结果的构建成功，均未使用 swap。重构后 6→8 并行仅快 0.811 秒（0.64%），
但最高温度由 69.3°C 上升至 73.9°C，最低可用内存减少约 301 MiB；不足以认定 8 并行有稳定优势。
因此 cat-release 默认改为 6 并行，共享 release 预设仍保持 4；临时可用 --parallel 8 覆盖。
重构后 6 并行相对重构前 4 并行冷缓存用时减少约 21.6%。重构后热缓存各档均 179/179 命中。

## 改动

- CMakeLists.txt：自动检测 ccache，以 CMAKE_CXX_COMPILER_LAUNCHER 接入；保留显式 launcher，支持 RESCUE_ENABLE_CCACHE=OFF。
- rescue_test_task_common：19 个公共源码集中为测试静态库，由 32 个测试目标复用。
- 全项目编译单元 416→179（减少 237，约 57.0%）；唯一源文件仍 109。测试侧任务/区域代码编译次数 256→19。
- 公共库保持 -UNDEBUG -Wall -Wextra，主程序继续独立编译；fixture 宏仍留在各测试入口，CTest 名称及超时保持原配置。
- tools/build/benchmark_build.py：可复现冷/热缓存对比，保存配置、用时、缓存统计和完整日志；每组结果立即 fsync。
- README.md：依赖、使用方式、关闭开关和基准复现命令。CMakeUserPresets.json：cat 本机并行度 6。

## 验证与已有失败

正式 build-release 编译成功。重构前和重构后 CTest 均 50/54；失败集合完全一致：
- rescue_runtime_legacy_baseline：4 个基线资源变化，check_mechanical/check_carry_demo 输出不一致。
- frame_virtual_serial_tests：现有 Python 15 字节协议实现与 17 字节测试期望不符等。
- motion_packet_contract_tests：同一协议长度/字段差异。
- demo_launcher_tests：同一协议及虚拟串口行为差异。

所有 C++ 测试通过；上述失败在公共库重构前已存在，本次不修改协议实现或刷新基线。
实际入口 tools/match/match_main.sh --check 成功，输出身份确认使用 build-release/rescue_upper_host，
显示 hardware_output=disabled、CHECK_NO_DEVICES 和“未打开设备”；没有执行实机运动。

测量期间观察到板端重启：第一轮中断记录未计入；随后完整重测。另一次重启发生在重构切换阶段，
已重新确认正式构建与回归。没有足够证据将重启归因于并行度。

## 原始证据（cat 板端）

- /home/cat/build-benchmark-20261026/before-retry/：重构前完整六组结果。
- /home/cat/build-benchmark-20261026/after/：重构后完整六组结果。
- 每个目录含 results.json、metadata.json、CMakeCache.txt、compile_commands.json、各组 .log 与 .ccache.txt。
- /home/cat/build-benchmark-20261026/ctest-before.log、ctest-after.log、entry-after.log。
- /home/cat/build-benchmark-20261026/CMakeLists.original.txt：本次改动前的板端文件备份。
