# ARM64 远程项目修复记录

> 后续更新：主入口已改为推行，C++ RKNN 后端已接通，默认模型改为现有 best_fp16.rknn。当前状态请见 VISION_INTEGRATION.md；以下保留文件修复时的历史发现。

日期：2026-09-21。主机：cat@192.168.1.123（LubanCat，aarch64，Ubuntu 24.04，Python 3.12）。
本机项目：`/home/liu/ds_innovation`；远程项目：`/home/cat/ds_innovation`。

## 文件修复

已将本机 `tools/remote_camera_telemetry/` 完整补入远程项目，共三个文件：README.md、server.py、verify.mjs。
补齐后，排除 build、npu_env、__pycache__ 和本机隐藏元数据目录，对 91 个项目文件进行了 SHA-256 校验，全部与本机一致，包括源码、配置、模型和测试文件。
相机服务原部署目录 `/home/cat/camera-telemetry/` 保留；不要同时启动两份相机服务。

## 不应直接复制的环境文件

初始差异中的 1,921 个条目属于 `benchmark_results/npu_env/`：1,599 个文件、318 个目录、4 个符号链接。
该环境为本机 x86-64 / Python 3.10 模型转换环境，包含对应架构的二进制扩展及本机解释器链接，不能通过补齐文件变成板子的 ARM64 / Python 3.12 环境。
`convert_npu_model.py` 文件本身明确要求在本机 npu_env 中执行。保留现有远程环境副本供查阅，但不要在板子上激活它或加入 PYTHONPATH。
模型转换应在本机完成，再传输 best_fp16.rknn。板子运行基准脚本通过 ctypes 加载 ARM64 的 librknnrt.so，不依赖 rknn Toolkit、onnxruntime 或该虚拟环境。

实测：使用板子系统 Python，加载 `benchmark_results/best_fp16.rknn` 和 `librknnrt.so`，完成一次真实 NPU 推理，输入 `(448,448,3)`，输出 `(11,4116)`，输出有限值检查通过。Runtime 2.3.2，驱动 0.9.8。
因此，这些虚拟环境差异对已测 NPU 推理和 C++ 核心构建没有直接影响；会影响试图在板子上直接使用本机模型转换环境的操作。

`.agents`、`.codex`、`.git` 在本次本机可见视图中为空目录。未伪造远程 Git 仓库；不影响程序运行。

## 额外发现的运行障碍

1. 原 `build/rescue_upper_host`、`build/rescue_core_tests` 和根目录 `rescue_upper_host` 都是 x86-64 ELF，不能作为板子原生 ARM64 程序运行。原文件保留，新构建使用独立目录 `build-arm64`。
2. 默认模型路径 `models/rescue2025.onnx` 在两端均不存在。现有 `benchmark_results/best.onnx` 是历史基准模型，不应仅重命名充当默认业务模型。
3. 用板子 OpenCV 4.6 实测加载 best.onnx 失败：`/model.24/Add` 节点触发 `parseBias` 的 `blob_0.size == blob_1.size` 断言。这会阻止当前 OpenCV DNN 路径加载该模型。
4. 历史基准报告记录的类别顺序为 core/wounded/red/dangerous/normal/main/blue，与当前 C++ 默认类别定义不同。正式接入必须确认标签映射、输入尺寸与输出解析，不能只修路径。
5. 检查时相机 HTTP 端口 8080 未提供服务；源码语法与 cv2/numpy/websockets 导入检查通过。文件恢复不代表服务已经启动或摄像头已完成实测。

第 2–4 项对完整视觉机器人运行影响较大，属于既有模型部署/兼容性问题，不是远程遗漏源码。此次不替换业务模型、不改变标签语义、不启动机器人控制。

## 原生构建方法

```bash
cd /home/cat/ds_innovation
sudo apt-get install --no-install-recommends cmake libopencv-dev pkg-config
cmake -S . -B build-arm64 -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build build-arm64 -j4
ctest --test-dir build-arm64 --output-on-failure
./build-arm64/rescue_upper_host --help
```

Debug 构建保留现有测试的 assert 检查。核心测试与帮助命令不启动相机/串口控制。

## 构建验证结果

已安装 ARM64 构建依赖，并修复 include/rescue/config.hpp 缺少 <cstdint> 的编译错误（本机与远程同时修复）。

原生 Debug 构建成功；ctest 1/1 通过（assert 启用）；build-arm64/rescue_upper_host --help 正常退出，验证返回码 0。最终使用 build-arm64 下的新程序，原 x86-64 构建产物保留。

补齐并修复后再次校验 91 个项目文件的 SHA-256。模型兼容性问题仍按上文记录，未进行整机运动控制验证。
