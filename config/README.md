# 配置入口

`project.json` 是当前运行时、演示 profile、任务标定、区域几何、颜色标定和模型清单的权威配置入口。

主程序：

```bash
build-release/rescue_upper_host --config config/project.json --check-config
```

演示 profile 使用 JSON 片段选择：

```bash
RESCUE_DEMO_PROFILE="$PWD/config/project.json#/profiles/demo_default" \\
  python3 tools/demo/demo.py recognize
```

`project.json#/frame/assume`、`project.json#/task_calibration/runtime` 等引用由 C++ 和 Python 加载器直接读取。相机标定继续使用 OpenCV YAML，因此 `camera.yaml` 和 `camera.mechanical_5_40.yaml` 仍是独立文件。目录中的其他 JSON 是旧启动器和冻结基线的兼容输入；新的脚本不再新增对它们的引用。
