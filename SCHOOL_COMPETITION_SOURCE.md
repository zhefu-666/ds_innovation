# 校赛源码收拢（2026-10-15）

比赛入口原先默认运行 `/home/cat/startup-fix-build/rescue_upper_host`；其 CMake 源码目录为 `/home/cat/startup-fix-src`，并非本仓库。

本次将该源码目录下 src、include、tests 的差异合入本仓库，新增 row_selector.hpp；两份 CMakeLists.txt 相同。仅去除导入代码的行尾空白，不将备份、历史日志、二进制纳入版本控制。

正式比赛入口 tools/match/match_main.sh 的默认二进制改为项目内 build-school-competition/rescue_upper_host，RESCUE_BINARY 仍可显式覆盖。旧源码和当前旧二进制暂留原位，不在此步骤启动或重启设备。

构建与软件回归：

```bash
cmake -S . -B build-school-competition -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DRESCUE_ENABLE_TELEMETRY=ON
cmake --build build-school-competition -j2
ctest --test-dir build-school-competition --output-on-failure
```

相机、串口、模型、标定、机构和比赛参数沿用现有入口；软件测试不构成实机验收。tool/demo/runtime 仍是独立旧演示运行副本，本次没有覆盖它。旧辅助启动脚本仍使用 build-camera-offset-20261007。

本次验证结果：Release 全量构建通过；CTest 22/22 通过（软件回归与虚拟串口，未做实机验收）。
