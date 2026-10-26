# 板端目录清理（2026-10-26）

目标主机：`cat@lubancat`（通过当前 SSH 别名 `cat` 验证）；项目：`/home/cat/ds_innovation`。

## 清理范围

- 删除三个过期验证构建目录：`build-format-foxglove/`、`build-format-no-telemetry/`、`build-format-verify/`，保留正式 `build-release/`。
- 删除 `benchmark_results/npu_env/`：其 RKNN 编译器是 x86-64 二进制，当前 ARM64 板端构建和运行均不引用它。
- 删除旧 `best.pt`、`best.onnx`、`best_fp16.rknn`、转换脚本、NPU 基准结果及样本；`benchmark_results/` 保留 `librknnrt.so` 和 `rknn_api.h`。
- 删除 `reports/`、`backups/`、两份 `*.before-no-yaw` 脚本副本及项目 Python 字节码缓存。
- 所有删除项均不属于 Git 跟踪文件；启动脚本、源码、现用模型、标定、测试、日志及格式化工具保持原内容。

## 历史资料备份

历史模型、基准资料、旧报告和脚本副本在删除前已复制到开发机：

`/home/jy/ds_innovation_cleanup_20261026_160010/removed-history.tar.gz`

SHA-256：`3ad6369fef5da6f0488de8cc4df6ac728f7b2a01de8c23c080a6e6899584c57b`。

同目录 `manifest.json` 保存完整删除清单与保留文件校验和。旧构建、x86 环境和 Python 缓存可再生成，未纳入历史资料压缩包。压缩包保留项目内原相对路径；需要恢复时只提取所需条目，不整体覆盖当前项目。

## 验证结果

- 删除 30 个指定路径、10,936 个文件，文件内容合计 308,782,620 字节；`du -sh` 项目实际占用从 458 MiB 降至 120 MiB，约减少 74%。
- 330 个保留文件的 SHA-256 与清理前一致，涵盖源码、配置、模型、SDK/运行库、启动工具、测试及重编译后的正式二进制。
- `cmake --preset cat-release` 与 `cmake --build --preset cat-release --clean-first` 成功，完成 228 项编译/链接操作。
- `ctest --preset cat-release`：50/54 通过。失败项为 `rescue_runtime_legacy_baseline`、`frame_virtual_serial_tests`、`motion_packet_contract_tests`、`demo_launcher_tests`。
- 旧基线测试报告配置资源及两条检查输出与快照不一致。三个串口相关测试报告 15/17 字节协议不一致：板端原有未提交改动在 `tools/frame/frame_ctl.py` 和 `tools/demo/demo_serial.py` 中注释了 yaw 写入，而测试仍要求 17 字节。相关文件与清理前校验和一致，本次未修改协议或刷新测试基线。
- `bash tools/match/match_main.sh --check` 成功，实际入口指向重新编译的 `/home/cat/ds_innovation/build-release/rescue_upper_host`，输出 `hardware_output=disabled`、`CHECK_NO_DEVICES`。
- `python3 tools/remote_camera_telemetry/test_project_bridge.py`：9/9 通过。
- `python3 tools/format_code.py --check`：187 个文件通过。
- 未启动实机运动、自启服务或任何机构动作。

正式二进制 SHA-256：`96b19350a29a2e30c9de7ba236a43ceb2912d449ec8bedae1148ed0635d27bce`。

完整日志位于开发机备份目录中的 `board-validation.log` 和 `entry-validation.log`；目录清理记录及 README/模型/协议说明已逐文件同步并核对 SHA-256。
