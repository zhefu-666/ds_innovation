# A7 提前准备启动服务

此服务以 cat 用户运行；启用 Linger=yes 后无需登录即可在开机后准备。

- 服务启动时立即运行现有整场入口，执行配置检查、方框先开后关、相机预置 UI150（内部偏移 +30°），
  加载模型并持续采集相机、IMU 和 MCU 反馈。准备过程会操作夹爪和相机，需提前清空机构扫掠空间。
- 日志出现 `[PREPARED] ready=1` 后底盘保持停止，等待 CRC 正确的 A7 start=1，
  或 `python3 tools/match/match_ctl.py start`。主程序独占 MCU 串口，无须退出监听后重启项目。
- `status` 返回 `WAITING`、`ready=1 permit=0 elapsed_ms=0` 表示已就绪。
  ready=0 时继续准备；预检或反馈失效时不允许运动。
- A7 按帧长、CRC16/Modbus 与换行帧尾解析，与8字节 A6 混合时整包处理。
  start=0、坏CRC、坏帧尾及准备完成前的按键不会排队触发；就绪后需重新按键。
- 整场 `--start-on-a7` 模式下，每个新的有效A7都请求新一轮：停车并交接发送线程，
  清空任务、目标选择、捕获与导航历史，零速下关框并恢复startup视角；反馈到位、
  新观测预检通过后从startup开始。运行、暂停、任务故障和上一轮结束后均可按键重开。
- 交接阶段状态为 `RESTARTING`，底盘许可为0；交接期间额外按键不排队。
  准备前的按键、硬件异常时的按键同样不排队。硬件异常或准备超时保持停止，恢复后重新按键。
  stop/finish取消待启动请求；本地START/RESUME/NEXT语义保持原样。
- 新一轮清除任务货物记录与送达计数，不改变物体实际位置；需按实际位置使用出发区配置。
  串口和模型保持连接，MCU夹爪动作编号继续递增，不随任务归零。
- 下位机应在消抖后的按下沿仅发送一次 `A7 01 BA 40 0A`，长按不连发。
  日志以 `[A7_RESTART]`、`[RESTART_STOP]` 和 `[RUN_START] run_id=...` 记录交接。
  零速写入成功不代表测得车轮静止；实机动作需现场验收。
- 该功能要求主进程存活。相机等致命错误导致进程退出后无法接收A7，需恢复采集并重启入口。
- 180秒从新一轮准备完成、startup获得运行许可时开始计时；默认不限制首次准备等待时长。
  可用 MATCH_MAX_RUN_S 显式限制包含准备和等待在内的进程总时长。
- 保留 --auto-run 作为显式的预检通过后自动开始方式。默认入口采用提前准备。
- 待机时本服务占用串口、相机与模型；其他设备工具运行前应停止服务。

安装并立即执行准备（会操作夹爪和相机）：

~~~bash
cd /home/cat/ds_innovation
bash tools/match/install_start_listener.sh
~~~

状态、日志和控制（板端 cat 用户）：

~~~bash
systemctl --user status rescue-start-listener.service --no-pager
journalctl --user -u rescue-start-listener.service -f
python3 tools/match/match_ctl.py status
python3 tools/match/match_ctl.py start
systemctl --user stop rescue-start-listener.service
systemctl --user restart rescue-start-listener.service
~~~

restart 会重新执行机构准备。停止服务会终止所属项目进程并执行已有停车收尾。
取消开机启动：`systemctl --user disable --now rescue-start-listener.service`。
卸载：`bash tools/match/install_start_listener.sh --uninstall`。

旧系统级 match-main.service 仍保持禁用。默认服务的 ExecStart 继续使用
start_on_a7.py，但它现在立即交接到准备入口；A7由主程序处理。
--site-clear 沿用用户指定的按键入口参数，设备、配置及实时检查继续生效。
