# 平板圆点板：分步拍摄相机内参

适用于本项目的 calib.io 对称圆点 PDF：10 列、7 行、原始圆心距 12 mm、圆直径 5 mm。
平板显示后的物理尺寸取决于屏幕缩放，因此必须测量。工具运行在上位机，电脑浏览器负责拍摄，平板只显示 PDF。

## 启动

在 SSH 终端运行：

```bash
cd /home/cat/ds_innovation
python3 tools/camera_calibration/guided.py --stop-preview
```

电脑打开 http://192.168.1.123:8081/ 。平板打开 http://192.168.1.123:8081/board.pdf ，或直接打开原 PDF。
默认相机 `/dev/video0`，分辨率 **1280×720**，与当前主程序默认值一致。如主程序实际使用别的模式，在启动命令追加 `--camera /dev/videoN --width 宽度 --height 高度`。
程序会验证相机实际返回尺寸；不一致会拒绝开始，不能拿不同分辨率的内参直接套用。
仅启动网页不会打开相机；填写测量值并点击「开始」后，`--stop-preview` 会停止项目管理器启动的旧预览和 Foxglove 桥接，再打开相机。
此工具不接串口，不发送运动命令。依赖 Python 3、OpenCV 和 NumPy，远程环境已有。

## 平板准备与量尺

1. 横屏显示完整 PDF，锁定屏幕旋转、缩放和亮度，关闭息屏、自动亮度、夜间模式。避免反光或相机画面中出现屏幕条纹；发现条纹先调整亮度和相机曝光。屏幕应平整，不能弯折。
2. 固定相机焦点，不要在采集中或运行时改变焦距、变焦、裁剪或分辨率。工具不假定相机支持自动关闭对焦，请在相机设置里确认。
3. 量同一行第 1 到第 10 个**圆心**的长度，填横向跨度，单位毫米。量同一列第 1 到第 7 个圆心，填纵向跨度。若中心不易读尺，可量两圆同侧边缘的距离，不能量两圆最外侧总宽。
4. 横跨度除以 9，纵跨度除以 6，得到两方向间距。原尺寸为 108 mm / 72 mm，屏幕上不必等于这个值。两方向算出的间距相差超过 1% 时拒绝开始，请检查测量与拉伸。
5. 之后不要再缩放 PDF。量尺后可移走尺子，避免遮挡圆点。

## 拍摄和保存方式

网页依次提示 25 个姿态：中央及四周 → 远近 → 左右上下倾斜 → 组合倾斜 → 3 张独立验证照片。
每次移动后停稳约 2 秒，看到网页检测快照中完整圆心连线后点击「拍摄并保存这一张」。一次成功只前进一步。
程序拒绝未识别全 70 点、过小、模糊、过期画面和近乎重复的姿态；具体方向、角度仍需按提示人工摆放。
建议画面内保留整块圆点阵列和白边，最大近景占宽约 60%～80%，不要使任何圆点出界。

**无需手机拍照、截图或手动另存。** 每次点击保存的是上位机相机的原始分辨率、未画线 BGR 图像，编码为无损 PNG。
浏览器看到的预览可能缩小，但 samples 中的照片不会缩小。生成的新目录为：

```text
/home/cat/ds_innovation/calibration_runs/circles_日期_时间_随机编号/
  session.json          测量值、分辨率、拍摄进度
  samples/001.png       第一张原图；直到 025.png
  detected/001.png      带圆心连线的复核图；不用于标定
  001.json              对应姿态与圆心像素坐标
  retakes/              撤销的照片归档，不直接删除
  intrinsics.yaml       完成 25 张后生成的内参和畸变
  report.json           拟合与独立验证误差、覆盖情况、警告
  使用说明.txt          参数接入位置
```

网页刷新会恢复当前进度；「撤销上一张」可重拍，最后一张触发计算后不能撤销。
第 1～22 张拟合内参，第 23～25 张只验证，不参与拟合。完成后自动释放相机，页面可以下载结果。
进程中断时已拍照片仍保留；当前不支持恢复半途会话，需要重启完整拍摄，不能改变缩放后混用旧照片。
完成的 25 张可脱离相机重新检测和计算（明确重新生成该次结果）：

```bash
python3 tools/camera_calibration/guided.py --recalculate /home/cat/ds_innovation/calibration_runs/实际会话目录
```

## 结果、质量与填写位置

界面显示 `fx, fy, cx, cy`（像素）和 `[k1,k2,p1,p2,k3]`（畸变系数）。`intrinsics.yaml` 已按项目使用的 OpenCV YAML 格式保存，**无需手填矩阵或从网页抄数字**。
拟合 RMS 希望小于 0.5 px，单张和独立验证误差希望小于 1 px。警告时保留候选文件供排查，建议重拍；不会覆盖生效配置，也不会将候选称作实测验收通过。
低 RMS 不能排除平板反光、摩尔纹、姿态退化或参数不稳定，后续应检查去畸变画面与实测地面距离。

1. 后续地面标定，把本次完整路径填到现有 `calibrate.py ground` 命令的 `--intrinsics` 后面。实测地面点流程见同目录 README。
2. `intrinsics.yaml` 只有 `camera_matrix`、`dist_coeffs` 和尺寸等字段，**不要直接改名为完整 camera.yaml**。项目 `CameraCalibration::valid()` 还要求 `ground_homography`。
3. 地面标定并通过独立测量验收后，完整输出文件部署到 `/home/cat/ds_innovation/config/camera.yaml`。
4. `/home/cat/ds_innovation/config/rescue.yaml` 中对应位置是 `calibration_file: ./config/camera.yaml`。目前主程序尚未读取该 YAML，后续还需要接入加载和尺寸检查；当前填写路径不会自动启用距离计算。
5. 原 `calibrate.py extrinsics` 仍只检测方格棋盘。本工具只新增圆点**内参**流程，后续可直接使用不依赖图案类型的 `ground` 实测地面点模式。

## 退出与恢复原预览

SSH 终端 Ctrl+C 退出，结果不会删除。采集期间原项目预览和 IMU 遥测会暂停；本工具不自动恢复，以免和下一次标定争用相机。
退出后恢复之前的相机、IMU 和 Foxglove：

```bash
cd /home/cat/ds_innovation
python3 tools/remote_camera_telemetry/manage_project.py start --imu --imu-port /dev/ttyUSB0 --imu-baud 115200
```

当前是普通镜头 OpenCV 针孔+畸变模型。超广角/鱼眼镜头如果无法得到稳定参数，需要改用鱼眼模型，不能仅忽略误差警告。

## Foxglove 同步观看

启动命令不变，程序同时提供 `ws://192.168.1.123:8766`。依赖 websockets，远程已安装。
网页会显示连接地址、当前 Foxglove 客户端数和相机是否更新。8766 用于标定，原主程序的 8765 用于原相机/IMU 遥测，两者不共用数据源。

1. Foxglove 选择 Open connection → Foxglove WebSocket，输入 `ws://192.168.1.123:8766`。
2. 添加 Image 面板，选 `/calibration/image`，显示独立的视频预览（最长宽度 960 像素，目标 30 Hz）。原始标定 PNG 仍以 1280×720 保存，不受 Foxglove 影响。
3. 添加 Raw Messages 面板，选 `/calibration/status`，查看 `count`（已拍数量）、`step`（当前提示）、`detected`（完整圆点识别）、`ready`（可拍摄）、`camera_fresh` 和 `frame_age_ms`。
4. 再添加 Raw Messages 面板，选 `/calibration/result`，标定完成后显示 `available: true`、矩阵、畸变、fx/fy/cx/cy 和误差。未完成时是 `available: false`。

可以先点击网页上的「打开相机，仅预览 / Foxglove 观看」，无需填写尺寸即可看相机，不会保存标定照片。正式拍摄前再填写尺寸并开始。尚未打开相机时，Foxglove 只有状态消息。拍摄、撤销、填写尺寸均在网页操作，Foxglove 接口只读。
相机停止或画面过期后不再发送旧图片；Foxglove 可能保留最后一张，判断实时性请看 `camera_fresh`。
程序共用同一次相机采集，不会为 Foxglove 再开一个摄像头。可通过 `--foxglove-port 端口号` 修改监听端口。

## 开发验证

```bash
python3 tools/camera_calibration/test_guided.py
python3 tools/camera_calibration/test_calibrate.py
python3 tools/camera_calibration/test_foxglove_bridge.py
```


## 2026-09-27 预览提速更新

启动命令不变。采集、视频预览与圆点检测分别运行，网页使用连续 MJPEG 视频流，不再每 700 ms 换一张图片。
默认请求相机 60 FPS、预览和 Foxglove 目标 30 FPS、检测上限 10 FPS；实际速度由曝光、USB、算力和网络决定。
本机首次实测采集约 30 FPS、预览约 30 FPS、Foxglove 接收约 25 FPS、圆点检测约 3 FPS。页面分别显示三种实测帧率。

实时预览显示原始画面，圆心连线在网页另一个“检测快照”中显示，避免把旧检测结果画到更新的视频帧上。
点击拍摄保存的是检测快照对应的 1280×720 原图和圆心坐标。检测结果超过 1 秒禁止拍摄，请停稳等待新检测。
算法仍使用原图分辨率的圆心；没有通过缩小标定原图来提速。背景过于杂乱、候选圆点过多时会拒绝匹配，请让完整圆点板占据画面主要区域。

可在启动时设置速度：

```bash
python3 tools/camera_calibration/guided.py --stop-preview \
  --camera-fps 60 --preview-fps 30 --detect-fps 10
```

现有服务运行时新参数不会自动生效；需要退出旧进程后重启。更新后刷新标定网页；Foxglove 如未自动恢复，请重新连接 8766，图像话题仍为 `/calibration/image`。
