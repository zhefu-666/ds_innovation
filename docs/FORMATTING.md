# C/C++ 格式规范

项目复用根目录 `.clang-format`，固定使用 **clang-format 18.1.8**。
已有风格保持不变：4 空格缩进、100 列、Attach 大括号、短语句展开、禁止 include 排序。
格式整理只调整源码布局，不修改算法、协议、状态机、配置值或编译标准。

## 安装与工具选择

车端已经安装 `.tools/venv/bin/clang-format`。脚本优先使用该工具，其次查找 PATH 中的
clang-format，并要求版本精确匹配 18.1.8。工具缺失或版本不符时退出码为 2。
新环境可以执行：

~~~bash
python3 -m venv .tools/venv
.tools/venv/bin/python -m pip install clang-format==18.1.8
~~~

## 检查和修复

~~~bash
python3 tools/format_code.py --check
python3 tools/format_code.py --fix
~~~

脚本根据自身位置确定项目根目录，也可从其他工作目录以绝对路径调用。
`--check` 不写文件：全部合规退出 0，有格式差异退出 1，工具或执行错误退出 2。
`--fix` 仅写入实际存在格式差异的文件，已合规文件不重写；随后再次检查应全部通过。

已完成 CMake 配置后，可使用相同的文件选择与检查逻辑：

~~~bash
cmake --build build-release --target format-check
cmake --build build-release --target format
~~~

普通构建不会自动改写源文件。格式目标需要 Python 3、Git 和固定版本的 clang-format；
直接运行 Python 脚本不需要先安装 OpenCV/RKNN 开发依赖或完成 CMake 配置。

## 文件范围

包含 Git 跟踪文件及未被 Git 忽略的新文件，限定以下目录的
.c/.cc/.cpp/.cxx/.h/.hh/.hpp/.hxx 文件：

- src/
- include/
- tests/
- foxglove_telemetry/src/

删除的文件跳过；忽略的新文件不纳入。已经被 Git 跟踪的文件仍会检查，即使后来匹配忽略规则。
构建目录、.tools/、第三方 SDK、备份、模型与实验环境不进入上述范围。
Python、Shell、HTML、JavaScript、CMake 和数据文件不使用当前 C++ 格式规则。
主工程 C++17 和独立 Foxglove 工程 C++20 编译设置保持各自原值。

## 开发与验收

提交前运行 `--check`，发现差异后运行 `--fix` 并审阅 diff。新文件自动进入检查范围。
目前仓库没有 CI；这些入口不会自行阻止提交或合并。后续 CI 可直接调用
`python3 tools/format_code.py --check` 并使用其退出码。
格式提交与逻辑改动分开审阅。涉及运行代码时执行已有构建与回归，不调整冻结预期来掩盖失败。
