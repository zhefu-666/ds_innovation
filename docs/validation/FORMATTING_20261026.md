# clang-format 统一整理与验证（2026-10-26）

## 实施范围

实际目录：cat:/home/cat/ds_innovation。采用实施前工作区内容作为基线，保留已有配置整理。
开始时 HEAD：69ebf55aee32f4224784e5c773ea4d8e83ba1248。实施期间其他工作提交为 4ba889e；本任务未创建 Git 提交。

- 固定 clang-format 18.1.8，沿用原 .clang-format。
- 187 个 C++ 文件：81 个格式整理，106 个原合规文件逐字节不变。
- 所有 81 个输出与格式化器对实施前原文生成的结果一致。
- 词法核对去除注释和空白、合并相邻字符串字面量后没有差异。
- Python、Shell、配置数据、第三方 SDK 和构建生成代码未做格式改写。

## 开发入口

- tools/format_code.py --check / --fix。
- CMake format-check / format，共用脚本。
- docs/FORMATTING.md 记录固定版本、安装、范围、退出码及开发流程。
- 没有创建自动提交钩子或 CI；普通构建不会改写源码。

## 已完成验证

| 检查 | 结果 |
| --- | --- |
| 全量格式检查 | 187/187 通过 |
| 重复修复 | 0 个更新，源码 mtime 不变 |
| CMake 检查/修复（BUILD_TESTING=OFF） | 通过 |
| 外部工作目录、含空格文件名 | 通过 |
| Git 已跟踪/未跟踪/忽略/删除文件的选择 | 通过 |
| 工具缺失、版本不符、规则缺失 | 返回明确错误和退出码 2 |
| 检查模式只读、合规文件不重写 | 通过 |
| git diff --check | 通过 |
| Foxglove smoke 构建与运行 | 通过，9 个通道 |
| 格式前主工程 Release / RKNN ON / 遥测 ON | 串行重试构建成功 |
| 格式前 CTest | 53/54 通过，冻结配置基线失败 |
| 格式后主工程 Release / RKNN ON / 遥测 ON | 全量构建成功 |
| 格式后 CTest | 53/54 通过，与格式前结果和失败详情一致 |
| 格式后 Release / RKNN ON / 遥测 OFF | 主程序及诊断目标构建成功 |

## 构建与回归结果

格式后 Release / RKNN ON / 遥测 ON 主工程全量构建成功。CTest 53/54 通过；逐项状态及唯一失败的 6 条差异均与格式前一致，没有新增失败。Release / RKNN ON / 遥测 OFF 构建也已成功（BUILD_TESTING=OFF，验证主程序及诊断目标）。

格式前并行构建在未改写的 src/app/runtime.cpp 上发生 GCC 13.3 内部崩溃，串行重试通过。
格式后并行构建在未改写的 release_retreat_stage.cpp 包含的标准库模板中再次发生 GCC 内部崩溃。串行重试到标定目标时发现生成依赖文件中的错误路径 /usr/inClUde/；保存异常文件、清理该目标生成物并重新运行 CMake 后，全量构建成功。
未为绕过编译问题修改业务代码或编译优化选项。

冻结基线 rescue_runtime_legacy_baseline 在格式前已有以下差异，未修改预期：

- config/demo_20261007.json
- config/demo_20261008.assume.json
- config/frame_20261008.assume.json
- tests/fixtures/frame_mapping.synthetic.json
- check_mechanical 和 check_carry_demo 标准输出

独立 Foxglove 完整 publisher 需要 foxglove-sdk CMake 包；当前主机未找到该包，因此只构建和运行现有 smoke 目标。

## 证据目录

reports/format-20261026-021040/ 包含实施前文件快照、SHA-256 清单、原有差异、格式专用补丁、检查日志和构建/测试日志。
该目录按现有 .gitignore 忽略；本记录可进入版本控制。

## 格式文件清单

### 基础、配置、控制与硬件（26）

- include/rescue/config/demo_policy.hpp
- include/rescue/config/file_reference.hpp
- include/rescue/config/task_calibration.hpp
- include/rescue/control/controlled_field.hpp
- include/rescue/control/match_control.hpp
- include/rescue/control/match_server.hpp
- include/rescue/control/motion_readiness.hpp
- include/rescue/control/motion_watch.hpp
- include/rescue/core/logger.hpp
- include/rescue/hardware/frame_sensors.hpp
- include/rescue/hardware/hipnuc_imu.hpp
- include/rescue/hardware/imu_adapter.hpp
- include/rescue/hardware/sensor_fusion.hpp
- include/rescue/hardware/uart_controller.hpp
- include/rescue/task/transport_rules.hpp
- src/config/task_calibration.cpp
- src/control/controlled_field.cpp
- src/control/match_control.cpp
- src/control/match_server.cpp
- src/core/logger.cpp
- src/hardware/hipnuc_imu.cpp
- src/hardware/imu_adapter.cpp
- src/hardware/sensor_fusion.cpp
- src/task/transport_rules.cpp
- src/tools/imu_monitor.cpp
- src/tools/sensor_monitor.cpp

### 几何与感知（42）

- include/rescue/geometry/camera_calibration.hpp
- include/rescue/geometry/geometry_pipeline.hpp
- include/rescue/geometry/ground_pose_fitter.hpp
- include/rescue/geometry/navigation_adapter.hpp
- include/rescue/geometry/planner.hpp
- include/rescue/geometry/safe_zone_pose.hpp
- include/rescue/geometry/zone_dead_reckoning.hpp
- include/rescue/geometry/zone_estimate.hpp
- include/rescue/geometry/zone_geometry.hpp
- include/rescue/geometry/zone_layout.hpp
- include/rescue/geometry/zone_visual_reference.hpp
- include/rescue/perception/capture_monitor.hpp
- include/rescue/perception/detector.hpp
- include/rescue/perception/keypoint_filter.hpp
- include/rescue/perception/model_io.hpp
- include/rescue/perception/multi_view_capture.hpp
- include/rescue/perception/perception_adapter.hpp
- include/rescue/perception/pixel_selector.hpp
- include/rescue/perception/row_selector.hpp
- include/rescue/perception/silhouette.hpp
- include/rescue/perception/tracker.hpp
- include/rescue/perception/vision_logic.hpp
- include/rescue/perception/yolo_detector.hpp
- include/rescue/perception/zone_color.hpp
- include/rescue/perception/zone_keypoints.hpp
- src/geometry/camera_calibration.cpp
- src/geometry/geometry_pipeline.cpp
- src/geometry/geometry_replay.cpp
- src/geometry/ground_pose_fitter.cpp
- src/geometry/navigation_adapter.cpp
- src/geometry/planner.cpp
- src/geometry/safe_zone_pose.cpp
- src/geometry/zone_estimate.cpp
- src/geometry/zone_geometry.cpp
- src/geometry/zone_layout.cpp
- src/perception/capture_monitor.cpp
- src/perception/keypoint_filter.cpp
- src/perception/perception_adapter.cpp
- src/perception/silhouette.cpp
- src/perception/tracker.cpp
- src/perception/vision_logic.cpp
- src/perception/yolo_detector.cpp

### 测试与独立 Foxglove（13）

- foxglove_telemetry/src/foxglove_publisher.cpp
- foxglove_telemetry/src/telemetry_smoke.cpp
- tests/async_log_tests.cpp
- tests/controlled_field_tests.cpp
- tests/demo_mode_tests.cpp
- tests/imu_tests.cpp
- tests/match_tests.cpp
- tests/mechanical_integration_tests.cpp
- tests/model_io_tests.cpp
- tests/navigation_tests.cpp
- tests/silhouette_tests.cpp
- tests/task_calibration_tests.cpp
- tests/zone_color_tests.cpp
