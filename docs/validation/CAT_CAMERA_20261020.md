# cat 单相机验证与部署保全

日期：2026-10-20。用户确认当前唯一 RK3588 为 SSH 别名 `cat`，且已连接 USB 相机。本次只运行旧 R4 第六批隔离二进制的无串口预览，未切换部署。后续发现主机存储、输入文件及工具链完整性异常，新批 ARM 相机验收已停止，不能将下述旧版观察记作新批验收通过。

## 隔离与设备

- 旧隔离二进制：`/home/cat/refactor-validation/20261020-r4-turn/build/rescue_upper_host`，SHA-256 `e54fcc7a4a3553c0c965f964914c2ebbcbb2a5fa67d82db237b6e6ea525defaf`。
- 旧相机证据：远程 `/home/cat/refactor-validation/20261020-r4-swing-camera/evidence`；本地 `reports/refactor-validation/r4-swing-20261020/camera/evidence`。本地逐文件核对远程摘要，39 个文件全部一致。
- 相机为 RYS / Realtek USB Camera，USB ID `0bda:1376`，序列号 `200901010001`，UVC 1.00，内核驱动 `uvcvideo`。`/dev/video0` 为视频采集，`/dev/video1` 为 UVC 元数据，两者属于同一台相机。
- MJPG 支持 `1280x720` 的 60 / 30 / 25 FPS；YUYV 的该分辨率仅支持 10 FPS。本次选择 V4L2 MJPG、1280x720、请求 60 FPS，遥测确认实际图像尺寸为 1280x720。
- 未执行硬件启动脚本，未启用 `--hardware`、`--imu` 或 `--pitch-feedback`，未操作机构或底盘。每 200 ms 的进程句柄采样只观察到 `/dev/video0`，未观察到串口句柄；全部遥测为 `WAIT_START`、`hardware_output_enabled=0`。

## 旧 R4 第六批相机观察

串行和并行分别运行一次 30 秒识别预览，两组顺序执行，均正常以演示时限退出，返回 0，stderr 为空。第一组退出后第二组成功重新打开相机。两组遥测发布错误均为 0，退出后快照文件均已删除。下表来自约 200 ms 一次的遥测及进程采样，不是逐帧完整性能记录。

| 执行方式 | 总墙钟秒 | 最后帧序号 | loop FPS 中位数 | 推理 ms 中位 / P95 | 采集 ms 中位 | RSS 峰值 KiB | CPU 中位 % |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 串行 | 30.91 | 282 | 9.49 | 95.88 / 97.44 | 9.35 | 149920 | 49.27 |
| 并行 | 30.94 | 421 | 14.38 | 54.72 / 70.57 | 9.37 | 156500 | 85.95 |

CPU 的 100% 表示占满一个核心。请求相机 60 FPS 不表示决策循环达到 60 FPS；本次没有使用远程部署特有的 `--async-capture` 参数。短时数据包含启动过程，未锁定频率或建立性能合格阈值；同期间根分区出现错误，因此仅保留原始观察，不认定稳定性能或长时间运行验收通过。

两组的共同参数如下；并行组仅额外增加 `--parallel-infer`，每组另有独立的遥测文件和比赛 socket。完整 argv、二进制摘要、stdout、stderr、逐次性能行、句柄、RSS、CPU、温度及遥测保存在各组目录。

```sh
rescue_upper_host --dry-run --no-show --vision-log-dir none \
  --demo-mode recognize --demo-seconds 30 \
  --camera 0 --width 1280 --height 720 --fps 60 --team blue \
  --model /home/cat/ds_innovation/models/detect_fp.rknn \
  --pose-model /home/cat/ds_innovation/models/zone_pose_fp.rknn \
  --rknn-library /home/cat/ds_innovation/benchmark_results/librknnrt.so \
  --telemetry --telemetry-fps 10
```

## 真实单帧同图对照

直接从 `/dev/video0` 保存一张完整 MJPEG 帧，SHA-256 为 `725e1f1f69c7b4ba460a04db89a496870f88de8f362c6290dbe688f2a43dbc1e`。使用原始提交的 ARM64 隔离原版及旧 R4 第六批程序，分别执行串行、并行，共 4 次保存图片推理：全部返回 0、stderr 为空，仅删除完整 `[PERF]` 行后，其余 stdout 逐字一致。

当前画面为室内桌面与键盘，没有任务目标；检测数为 0、左右姿态均无有效候选，颜色为 `color_unmeasured`。这只覆盖真实取图和同图模型入口对照，不能据此认定检测准确率、误检、关键点质量、区域身份或现场标定合格。图片和四次完整输出位于 `camera/evidence/camera-still`。

## 部署入口盘点

通过只读检查脚本、profile、CMakeCache 和既有二进制，确认以下默认映射；没有执行这些启动脚本。

| 入口 | 实际二进制或后续入口 | 已确认的配置差异 |
| --- | --- | --- |
| `tools/match/match_main.sh` | `build-school-competition/rescue_upper_host`，可由 `RESCUE_BINARY` 覆盖 | 默认 CPU 4-7；`detect_hybrid_20261015.rknn`；并行、分核、`--async-capture`；机械俯仰与现场颜色文件；MCU/IMU 参数 |
| `tools/match/start_capture_scan.sh` 等旧启动入口 | `build-camera-offset-20261007/rescue_upper_host` | 各自选择相机及任务标定，保留独立启动参数 |
| `tool/demo/demo_oneclick.sh` | `tool/demo/runtime/tools/demo` 和 `tool/demo/runtime/bin/rescue_upper_host` | 使用打包 profile `runtime/config/demo_20261008.assume.json`，资源根目录指回原部署，profile 请求 30 FPS |
| systemd `match-main.service` | `tools/match/match_autostart.sh` | service 为 disabled / inactive，脚本顶部以 `exit 0` 禁用自动启动 |

两个构建目录的 CMakeCache 均指向 `/home/cat/ds_innovation`；该源码树含独立未提交修改，缺少可核验的构建时源码清单，不能据此证明现存二进制精确对应当前源码。三个部署二进制摘要分别为：

- `build-camera-offset-20261007/rescue_upper_host`：`4fde1b8e66fb11898a1b0a067d7eba12c6a74607fd1dbb5ea664e14671001470`。
- `build-school-competition/rescue_upper_host`：`3652eee4d9d51671d2ad9a7bc4e7acf7f5f9cc7f0147ae674b490ddf4f6093aa`。
- `tool/demo/runtime/bin/rescue_upper_host`：`fb46e59a01596619398169d67385fb537ae95ebcf6f30a3939d4f79154cfcb38`。

远程部署还使用本地重构树没有的采集参数和不同检测模型，部署入口迁移与二进制替换均未实施。完整映射、脚本及 profile 原文见 `startup-mapping.json`、`deployment-entry-source-evidence.json` 和 `startup-systemd-evidence.json`。

## 主机异常与保全

原始相机盘点记录已经包含同期间的 `mmcblk0p2` EXT4 checksum invalid、Delayed block allocation failed / error 117 和 `Data will be lost`；system journal 也报告损坏。发现后停止向该根分区写验证产物，并将已有 39 个证据文件同步本机，逐文件 SHA-256 一致。错误原文可从 `storage-warning-from-inventory.json` 查看，未把根分区问题记作已解决。

随后计划在可执行 tmpfs `/dev/shm/r4-swing-camera-20261020` 进行新批测试，并在使用前重新核对检测模型、姿态模型及 RKNN 动态库摘要，三者与旧记录一致。但主机后续构建又出现多次编译器 ICE，且完整摘要核对通过后的 tmpfs 冻结夹具在无 CTest 运行期间出现 2 字节非预期变化，`dpkg -V` 还发现 `cc1plus` MD5 不匹配。主机输入和工具链完整性已不可信，因此新批相机验证没有开始。

**未完成的新批检查**：持续串行/并行、外发 SIGINT / SIGTERM、无效相机 index、退出后相机与遥测锁重开，以及新批同图原版对照。验证脚本虽已准备，其存在不表示执行或通过。没有通过恢复夹具、重新生成预期或重跑掩盖异常；没有修分区、改挂载、重启或切换部署。

为保留远程独有改动，从 `git ls-files --modified --others --exclude-standard` 得到的 30 个文件全部只读备份至本机，共 295929 字节，未排除文件；另备份上述 3 个部署二进制，共 3550776 字节。归档通过 SSH stdout 直接传到本机，没有在坏盘创建 tar。33 个文件的远程备份前后清单完全一致，本机解包文件 SHA-256 全部一致；Git status、diff 与三个部署二进制也和相机盘点时一致。

备份目录：`reports/refactor-validation/r4-swing-20261020/camera/deployment-backup`，含原始 tar、解包文件、Git HEAD / status / diff、前后清单及 `verification.json`。备份 tar SHA-256 为 `941673ed85966e8b585226968d22135a81220f1e85ce945dbbb65ab8cb92bee1`。本机最终核对见 `camera/local-preservation-comparison.json`。`reports` 按仓库规则忽略，需与本地证据归档一起保留。

本机完整相机证据包为 `reports/refactor-validation/r4-swing-20261020/camera-evidence-20261020.tar.gz`，6522242 字节，SHA-256 `6bf440f906329b94ad31cab54d310031bcda7330e872713310af889e83f041c1`；摘要及范围另存同目录 `camera-archive.json`。

相机物理断连、传感器同步、标定资格、机构/底盘/停车时序、实车任务及可靠主机上的新批 ARM 回归仍未验收。远程 tmpfs 中仅保留已上传的准备文件，重启即失；本轮需要长期保留的相机观察和部署备份均已复制至本机。

后续发现原 30+3 备份没有覆盖 Git ignored 的模型和运行库，属于备份范围缺口；这些资源及当前启动依赖已单独流式保全到 `reports/refactor-validation/cat-integrity-20261020/recovery-assets`，远程前后清单及本机归档逐项一致。详见 [完整性与离线恢复准备](CAT_INTEGRITY_20261020.md)。未安装包或更换部署，未恢复相机验收，主机仍等待离线恢复条件。
