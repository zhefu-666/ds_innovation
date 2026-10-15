# ds_innovation

RK3588 / LubanCat 智能救援机器人上位机项目，使用 C++17、OpenCV、RKNN，包含视觉识别、任务状态机、IMU 接入、串口控制和遥测查看。

## 当前状态

项目已经历校赛，**底盘运动控制输出已经实现**，支持感知预览和硬件运行模式。校赛经历不代表所有功能及后续重构版本都已完成实机验收。

当前处于调试与重构阶段：

- 当前分支为 `school-competition`。
- 比赛开机自启服务 `match-main.service` 已禁用；自启脚本也保留立即退出保护，比赛启动命令已注释。
- 正式比赛入口为 `tools/match/match_main.sh`，默认使用项目内 `build-school-competition/rescue_upper_host`；支持通过 `RESCUE_BINARY` 显式覆盖。
- 比赛源码已从独立构建源码目录收拢，说明见 [校赛源码收拢](docs/历史记录.md)。

## 通信约定

用户于 2026-10-15 确认：

- 相机发送角度为 **0–80°**，**40°为正常启动中位角度**。
- A6 第二字节持续表示开闭状态：进行一次开动作后持续回传 **1**，关闭后持续回传 **0**。
- 开闭状态不是瞬时动作完成标志；动作确认还要检查编号和反馈新鲜度。

用户于 2026-10-15 确认当前相机下发代码可正常使用，保持现有发送与角度转换实现，本轮不安排修改；不因本文角度表述调整控制代码或启动参数。此可用性依据用户反馈，本次未重新执行硬件验证。

详见 [发送协议](docs/通信协议.md)、[接收协议](docs/通信协议.md)、[相机角度](docs/通信协议.md)、[方框状态](docs/通信协议.md)。

## 构建与软件测试

环境：ARM64 Ubuntu、CMake、C++17 编译器、OpenCV 4.x、Python 3。遥测桥使用 Python websockets。

```bash
cmake -S . -B build-school-competition \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON \
  -DRESCUE_ENABLE_TELEMETRY=ON
cmake --build build-school-competition -j2
ctest --test-dir build-school-competition --output-on-failure
```

源码收拢时已完成 Release 构建和 22/22 项 CTest；这是软件回归结果，不是实机验收。

## 实机运行资源

比赛入口当前使用：

- `models/detect_fp.rknn`：物块检测。
- `models/zone_pose_fp.rknn`：安全区关键点。
- `benchmark_results/librknnrt.so`：RKNN 运行库。
- `config/camera.mechanical_5_40.yaml`：相机配置。
- `config/zone_geometry.json`：区域几何。
- `config/zone_color.field.json`：区域颜色。
- `config/frame_20261008.assume.json`：机构与任务标定配置。

模型二进制和运行库未随 Git 跟踪，下载源码后需要单独准备。运行时不得让多个程序争用同一相机或串口。当前部分标定配置仍含假设，不能仅因文件存在就认定已完成验收。

## 目录与文档

| 位置 | 内容 |
|---|---|
| `src/`、`include/rescue/` | 主程序、接口与算法 |
| `tests/` | 单元测试、虚拟串口及回放 |
| `config/` | 相机、机构、区域和任务配置 |
| `tools/match/` | 比赛启动与控制入口 |
| `tools/remote_camera_telemetry/` | 主程序遥测桥及查看工具 |
| `tools/board_monitor/` | 板上实时监看 |
| `tools/camera_calibration/` | 相机标定工具 |
| `tool/demo/` | 独立演示入口与运行副本，不等同于正式比赛版本 |

- [IMU 接入](docs/IMU接入.md)
- [任务接入](docs/运行与调试.md)
- [相机标定操作](docs/相机标定.md)
- [一体化外参与地面映射](docs/相机标定.md)
- [主程序遥测](docs/遥测与查看.md)
- [板上监看](tools/board_monitor/README.md)
- [独立演示](docs/独立演示.md)

日期命名的阶段文档包含历史设计与测试记录，不能直接作为当前协议或启动命令依据；当前协议以本页所链接的协议文档为准。

## 文档导航

- [运行与调试](docs/运行与调试.md)
- [通信协议](docs/通信协议.md)
- [IMU接入](docs/IMU接入.md)
- [视觉与模型](docs/视觉与模型.md)
- [相机标定](docs/相机标定.md)
- [遥测与查看](docs/遥测与查看.md)
- [任务状态机](docs/任务状态机.md)
- [受控测试](docs/受控测试.md)
- [架构与重构](docs/架构与重构.md)
- [历史记录](docs/历史记录.md)
- [独立演示](docs/独立演示.md)
