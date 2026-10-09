> 2026-10-05策略更新：见 TARGET_SELECTION_20261005.md。旧找物短推已取消；当前先线索接近、停车选货，整批绿色送达后解锁黑色优先。下文旧运行记录仅作历史证据。

# 直接旋转搜索抓取测试

运行位置：cat@192.168.34.7:/home/cat/ds_innovation。
新脚本 tools/match/start_capture_scan.sh 保留原启动脚本，startup-advance-ms=0，不进行开局定时前进。
其他受控空场参数保持原值：蓝方、搜索3 rad/s、找物短推开启、物资每趟最多2件。
仅适用于人工确认唯一己方区、初始目标在区外、前后范围清空且无人进入的场地。

## 使用

```bash
cd /home/cat/ds_innovation
# 静态检查，不打开设备
bash tools/match/start_capture_scan.sh --check
# 识别预览，无控制输出；相机须已实际处于5度
bash tools/match/start_capture_scan.sh --preview
# 准备相机5度并等待人工开始（此准备会发送零轮速执行器命令）
bash tools/match/start_capture_scan.sh --run
```

另一个SSH终端：

```bash
cd /home/cat/ds_innovation
python3 tools/match/match_ctl.py status
python3 tools/match/match_ctl.py start
python3 tools/match/match_ctl.py stop
```

自动开始：`bash tools/match/start_capture_scan.sh --auto-run`，预检通过后立即允许实车动作。
程序运行时终端打印日志路径，使用 `tail -F <该日志绝对路径>` 查看。
画面及状态：http://192.168.34.7:8080/ （桥接服务运行期间可用）。
停止先执行stop，确认permit=0，然后主程序终端Ctrl-C退出。

## 成功判据

应观察到 SCAN → APPROACH → PREPARE → RUSH → CLOSE → VERIFY_CAPTURE → CARRY → GATE → OPEN_RELEASE → ENTER → BACK_OUT → VERIFY_DELIVERY → TURN_SCAN。
抓取通过看 capture_verified；送达看 delivered / partial_delivery 及 delivered 数量。
须同时现场确认物体实际被夹住、随车移动、释放后留在正确半区；日志不能替代物理验收。

## 2026-10-05本次结果

静态检查通过，STARTUP forward_ms=0。
启动实车程序后停在WAIT_START，preflight=startup_clearance_required，未完成搜索/抓取/搬运。
蓝色危险物观测位置约(-0.1334,0.4275)m，距离约0.448m，在0.32+0.15=0.47m的局部否决范围内。
普通物资多次contact=0 reason=size_mismatch，需要核对真实物体尺寸、检测框和地面投影；不能直接放宽阈值。
IMU和相机推理运行，pitch读回500 cdeg；启动准备发送过零轮速、保持夹爪状态的pitch命令。
发送STOP得到 OK PAUSED reason=operator_stop permit=0，随后SIGINT退出本次主程序。
日志：telemetry_logs/capture-scan-20261005-003525-7612.log。
本次143条周期任务状态全部WAIT_START，startup_forward_ms=0，delivered=0。
先现场移开禁入范围内的蓝色物体并检查绿色物资识别，再重新测试。
