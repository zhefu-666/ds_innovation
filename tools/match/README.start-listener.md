# A7 按键启动服务

此服务以 cat 用户运行；用户已启用 Linger=yes 时，无需登录即可在开机后启动。

- 待机时只读监听 /dev/ttyACM0，115200、8N1；串口不存在、断开或被占用时每秒重试。
- 按帧长、CRC16/Modbus 和换行帧尾解析混合流，整包跳过 8 字节 A6，仅处理 5 字节 A7。
- start=0 不触发；CRC 有误、帧尾有误或 start 不是 1 均不触发。
- 收到第一个有效 start=1 后，先关闭并释放串口，再立即调用现有整场入口：
  `bash tools/match/match_main.sh --run --site-clear --frame close`。
- 入口执行现有预检、方框/相机准备，再启动 build-release/rescue_upper_host。
  默认蓝队、出发区读取 config/project.json、整场 180 秒；本服务不跳过入口检查。
- 每次服务启动只触发一次；项目运行期间不再监听、不会因重复 A7 启动第二个实例。
  正常结束后服务为 inactive；启动失败则为 failed，不自动重跑比赛。需要再次等待按键时 restart 服务。
- 按键是实机运行触发：启用服务后，按键会启动机构准备和底盘流程。测试监听请用 watch_start.py。
- 待机期间该服务独占 MCU 串口；其他串口工具运行前应停止它。

安装并立即开始等待按键：

```bash
cd /home/cat/ds_innovation
bash tools/match/install_start_listener.sh
```

状态、日志和停止（均在板端 cat 用户下执行）：

```bash
systemctl --user status rescue-start-listener.service --no-pager
journalctl --user -u rescue-start-listener.service -f
systemctl --user stop rescue-start-listener.service
systemctl --user restart rescue-start-listener.service
```

取消开机启动：`systemctl --user disable --now rescue-start-listener.service`。
卸载：`bash tools/match/install_start_listener.sh --uninstall`。
停服务会同时向其正在运行的项目进程发送终止信号，沿用项目现有的停止收尾。

旧的系统级 match-main.service 保持禁用；新的用户服务为 rescue-start-listener.service。
`--site-clear` 随用户要求的按键启动入口传入，实际启动仍执行现有设备、配置和实时预检。
