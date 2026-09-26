# Foxglove 遥测发布器

旧的 Flask/SSE 原型已移除。这个目录改为基于 GitHub 上游项目的协议实现：C++ publisher 使用 Foxglove SDK WebSocket API，离线文件使用官方 MCAP Python writer，查看端使用兼容 Foxglove 协议的 Lichtblick。

## 上游依据

- Foxglove SDK C++ [`cpp/examples/ws-server/src/main.cpp`](https://github.com/foxglove/foxglove-sdk/blob/main/cpp/examples/ws-server/src/main.cpp)：服务器、channel 和日志发布方式。
- Foxglove SDK C++ [`cpp/examples/ws-stream-mcap/src/main.cpp`](https://github.com/foxglove/foxglove-sdk/blob/main/cpp/examples/ws-stream-mcap/src/main.cpp)：MCAP 作为独立回放源的方式。
- Foxglove SDK C++ [`cpp/examples/rgb-camera-visualization/main.cpp`](https://github.com/foxglove/foxglove-sdk/blob/main/cpp/examples/rgb-camera-visualization/main.cpp)：相机消息发布入口。
- MCAP [`python/examples/raw/writer.py`](https://github.com/foxglove/mcap/blob/main/python/examples/raw/writer.py)：`Writer`、schema、channel 和 message 写入方式。
- Lichtblick [`README.md`](https://github.com/lichtblick-suite/lichtblick/blob/main/README.md)：浏览器/桌面端 Foxglove 兼容可视化工具。

## 当前构建

不安装 Foxglove SDK 也可以先构建协议检查程序：

```bash
cmake -S foxglove_telemetry -B build/foxglove_telemetry
cmake --build build/foxglove_telemetry -j2
./build/foxglove_telemetry/rescue_telemetry_smoke
```

这一步会验证只读通道契约。输出中的 `client publish: disabled` 是设计要求。

## 构建真实 Foxglove publisher

先按 Foxglove SDK 官方说明构建 C++ dist，然后指定 dist 路径：

```bash
cmake -S foxglove_telemetry -B build/foxglove_telemetry \
  -DRESCUE_BUILD_FOXGLOVE_PUBLISHER=ON \
  -DFOXGLOVE_SDK_DIST=/path/to/foxglove-sdk/cpp/dist
cmake --build build/foxglove_telemetry -j2
./build/foxglove_telemetry/rescue_foxglove_publisher
```

在 Lichtblick 中连接 `ws://127.0.0.1:8765`。服务端明确设置 `WebSocketServerCapabilities::None`，不注册客户端发布、参数、服务和播放控制回调，因此浏览器只能查看数据。

## 生成 MCAP 样例

```bash
python3 -m venv .venv-mcap
. .venv-mcap/bin/activate
pip install -r foxglove_telemetry/tools/requirements.txt
python3 foxglove_telemetry/tools/generate_sample_mcap.py rescue_sample.mcap
```

生成的文件可直接拖入 Lichtblick 回放。真实项目接入时，应将 C++ 状态机的数据源替换到 `foxglove_publisher.cpp` 的发布线程，并保持图像最新帧覆盖、结构化数据有界、控制链路完全旁路。
