# ds_innovation 重构检查点提交方案

生成日期：2026-10-24。基线：`main @ bd73a38`。本方案不改变工作树，只定义检查点、文件归属、提交信息与验证门槛。

## 1. 当前结论

- 重构成果全部未提交：13 个已跟踪文件修改，约 60 个未跟踪文件。
- 当前代码在本机 Release、遥测 ON、RKNN OFF 下构建通过，CTest 41/41 通过。
- `src/main.cpp` 已降到 16 行；`src/push_task.cpp` 仍有 2378 行，R4 只完成七批。
- `CMakeLists.txt`、`src/push_task.cpp`、`include/rescue/push_task.hpp`、`tests/push_task_tests.cpp` 同时混有 R1–R4 多批改动。按历史批次硬拆会制造不可编译的中间提交，因此首个检查点采用“完整可构建快照”。
- `reports/` 已被忽略，不能作为提交内容；`tools/runtime/__pycache__/` 已忽略，不加入提交。
- `websockets` 当前未安装，桥接单测需要另外安装 `websockets==10.4` 后再执行；这不影响 C++/CMake 的 41 项测试。

## 2. 推荐提交序列

### C1 重构主检查点（必须首先提交）

**目的**：冻结当前已通过本机回归的完整重构状态，建立可回退、可继续开发的基点。

**提交信息建议**

```text
refactor: split runtime entry and extract push stages

- centralize runtime configuration loading and effective config export
- reduce main to process entry, assembly and exit handling
- extract capture, delivery, release, push-in and sweep stages
- add stage, configuration and frozen-baseline regression tests
- document runtime schema, architecture and current refactor status
```

**纳入文件**：除 C2、C3 所列之外的全部当前修改/未跟踪源码、测试、配置与文档。具体包括：

- 构建：`CMakeLists.txt`
- 入口与配置：`src/main.cpp`、`src/config.cpp`、`src/app/*`、`include/rescue/app/*`、`config/runtime/*`
- 状态机：`src/push_task.cpp`、`include/rescue/push_task.hpp`、`src/task/*`、`include/rescue/task/*`
- 遥测：`src/telemetry_publisher.cpp`、`include/rescue/telemetry_publisher.hpp`、`tests/telemetry_tests.cpp`
- RKNN 离线构建：`src/rknn_model.cpp`、`src/rknn_unavailable.cpp`
- 任务回归：`tests/push_task_tests.cpp` 以及全部 `tests/*stage*_tests.cpp`、`tests/*task_integration_tests.cpp`、`tests/config_loader_tests.cpp`、`tests/runtime_config_tests.cpp`、`tests/runtime_config_integration.py`
- 冻结基线：`tools/runtime/record_baseline.py`、`tools/runtime/test_record_baseline.py`、`tests/fixtures/runtime_legacy_baseline.json`、`tests/fixtures/runtime_legacy_aarch64.json`
- 文档：`README.md`、`REFACTORING_PLAN.md`、`docs/ARCHITECTURE.md`、`docs/RUNTIME_CONFIGURATION.md`、`docs/REFACTORING_PROGRESS.md`、`docs/HANDOFF.md`、`docs/history/README_BEFORE_APP_REFACTOR.md`
- 验证记录：`docs/validation/*.md`（只含文字与摘要，不含 `reports/` 大文件）

**C1 验证门槛**

```sh
git diff --check

cmake -S . -B /tmp/ds-innovation-c1-off \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRESCUE_ENABLE_TELEMETRY=ON -DRESCUE_ENABLE_RKNN=OFF
cmake --build /tmp/ds-innovation-c1-off -j4
ctest --test-dir /tmp/ds-innovation-c1-off --output-on-failure
```

期望：构建成功，CTest `41/41`。当前基线夹具 SHA-256 必须保持：

```text
c828d652eb7316c817ab6182ba13f8202496b59f9b66b06dd9abdbd108b4d79f  tests/fixtures/runtime_legacy_baseline.json
028f36440090e2353b28b3a9f4f069715c088dcae5801fd0004e0302d0987b43  tests/fixtures/runtime_legacy_aarch64.json
```

另需跑一次遥测 OFF 构建，确认配置开关仍独立：

```sh
cmake -S . -B /tmp/ds-innovation-c1-off-telemetry-off \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRESCUE_ENABLE_TELEMETRY=OFF -DRESCUE_ENABLE_RKNN=OFF
cmake --build /tmp/ds-innovation-c1-off-telemetry-off -j4
ctest --test-dir /tmp/ds-innovation-c1-off-telemetry-off --output-on-failure
```

历史记录期望为 `40/40`；必须记录实际数量，不把预期当结果。

**不做的事**：不引入策略变化、不改速度/门控/协议、不删除旧入口、不更新部署二进制。

---

### C2 遥测桥接配置 schema

**目的**：让独立 Foxglove/网页桥接接受主程序新增的 `config.effective` 字段，同时保持旧字段兼容。

**提交信息建议**

```text
fix(telemetry-bridge): accept effective runtime config

Extend the bridge CONFIG_SCHEMA with the exported effective configuration
while preserving all existing flat fields and optional compatibility.
Add schema regression coverage for runtime config and channel identity.
```

**文件**

- `tools/remote_camera_telemetry/project_bridge.py`
- `tools/remote_camera_telemetry/test_project_bridge.py`

**验证门槛**

```sh
python3 -m py_compile tools/remote_camera_telemetry/project_bridge.py \
  tools/remote_camera_telemetry/test_project_bridge.py
```

桥接单测需要在隔离环境中安装依赖，不装入系统 Python：

```sh
python3 -m venv /tmp/ds-innovation-bridge-venv
/tmp/ds-innovation-bridge-venv/bin/pip install 'websockets==10.4'
/tmp/ds-innovation-bridge-venv/bin/python -m unittest discover \
  -s tools/remote_camera_telemetry -p 'test_*.py'
```

期望：9 项桥接测试通过（以实际输出为准）。若离线环境无网络，先记录 `websockets` 缺失，不把它算作通过。

**为何独立**：它是查看端兼容改动，不属于上位机运行逻辑；单独提交便于回退，也便于部署时只更新桥接服务。

---

### C3 清理构建产物与填写检查点结果

**目的**：提交前清掉误生成的缓存，并把 C1/C2 的实际结果写回进度文档，形成闭环。

**建议处理**

- 确认 `tools/runtime/__pycache__/`、`tests/__pycache__/` 等均被 `.gitignore` 覆盖；不应进入 Git。
- 在 `docs/REFACTORING_PROGRESS.md` 增加一节“2026-10-24 检查点复核”，只记录实际执行的命令、环境、测试数量和未验证范围。
- 若 C2 测试因依赖缺失未跑，明确写“未执行”，不要补写为通过。

**提交信息建议**

```text
docs: record validated refactor checkpoint
```

**验证门槛**

```sh
git status --short --ignored | grep -E '^!! .*__pycache__' || true
git diff --check
```

期望：工作树只保留忽略产物；`docs/REFACTORING_PROGRESS.md` 的事实与命令一致。

## 3. 执行顺序与回退

1. 先做 C1，并在提交前记录 `git status --short`、构建目录、CTest 摘要。
2. 在同一工作树完成 C1 提交后，再做 C2 并单独提交。
3. 最后更新 C3 文档。
4. 每个提交都可独立回退；但不要只回退 C1 中的单个文件来模拟阶段回退，因为 CMake 与任务源码已形成整体依赖。
5. 真正的 R4 细分提交从 C1 之后开始，每批只改一个阶段，并按 `REFACTORING_PLAN.md` 的矩阵记录前/后对照。

## 4. 风险与边界

- C1 是大提交，审查成本高于理想状态；这是当前工作树混合多批改动后的现实选择。它的价值是先把“能构建、能回归”的状态固定下来，避免继续在未提交状态上累加风险。
- 不应把 C1 称为“R4 完成”。文档必须继续写明搜索接近、清扫触发/收尾、严格 ENTER、其余搬运恢复和部署迁移未完成。
- `reports/` 中的证据与密钥材料不进入 Git；其路径与摘要保留在进度文档中。
- 远程 `cat` 主机完整性未恢复，C1 不等于 ARM/实车验收，也不授权部署替换。
