# Pose 原生 FP16 输入优化（2026-10-18）

远端：cat@192.168.1.121:/home/cat/ds_innovation。修改 src/rknn_model.cpp，已编译正式 build-school-competition/rescue_upper_host。

## 实现与适用边界

保留640×640 RGB、letterbox、模型、阈值、NMS、检测/pose分核与同帧并行。将当前pose的uint8 RGB以NEON转换为RGB/255 FP16，直接写入由rknn_create_mem分配、rknn_set_io_mem绑定的原生输入缓冲区，每帧执行rknn_mem_sync。缓冲区复用，析构时先释放内存再销毁context。这里仍有CPU转换与写内存，不意味着完全没有数据移动。

仅在aarch64、模型内容FNV-1a指纹592ca2315fd05dde及原生属性FP16 NHWC [1,640,640,3]、2457600字节匹配时默认启用。FNV用于防止误套到其他模型，不是安全校验；模型SHA-256为fb60b26d4390f37df6b6826647a4025018204576c3c75665c86bca88c7d50b12。其他模型继续走原inputs_set路径。模型布局/尺寸异常或内存API失败会报错退出。检测模型路径没有改动。

回退：启动前设置 RESCUE_POSE_NATIVE_FP16=0，即恢复旧输入处理；恢复默认可unset此变量。已通过主程序对照验证该开关。源码及原二进制备份在/home/cat/pose-native-ab-20261018/，分别为rknn_model.before.cpp及rescue_upper_host.before。未修改自启动或启动器。

## 结果

普通标量转换原生路径约6.53ms；NEON转换并同步约1.03–1.06ms；原Runtime inputs_set约10.8–11.1ms。NPU执行仍约36–38ms，本轮没有优化网络计算本身。

9张已有板端实拍图片（3张tests/samples安全区样本、6张calibration_runs图像）对比，原生RGB/255路径与旧Runtime路径的完整原始FP32输出逐字节一致，包括既存非有限值位模式；这不表示模型原有非有限输出问题被修复。未归一化的FP16输入作为负对照出现大量差异，未接入程序。

集成后再次用RknnModel接口对比9张图片，全部bit_equal=1。正式主程序对9张图片各运行旧/新两种路径，除性能行外的stdout（检测、关键点/安全区、颜色等解码结果）全部一致，MAIN_DECODE_AB=PASS。

固定RGB输入、检测core0/pose core1、CPU4–7、OpenCV单线程，3组交替顺序A/B，每组每路径24次、剔除前4次，保留60次/路径：

| 项目 | 旧路径 | 原生FP16 |
|---|---:|---:|
| 并行检测+pose调用平均墙钟 | 49.395ms | 39.029ms |
| 对应模型调用吞吐 | 20.25fps | 25.62fps |

调用延迟约降低21.0%，对应吞吐提高约26.6%。计时不包含相机采集、公共图像预处理、控制、绘图、预览，不能将25.62fps当成真实相机端到端帧率。各组结果见integrated-ab.txt；独立输入路径结果见ab-neon.txt。

## 验证与尚未验收

完整增量构建成功，22/22 CTest通过，git diff --check、bash -n、match_main.sh --check通过；后者没有打开设备。NPU驱动仍绑定，自启动disabled。

本次板端不存在/dev/video0（仅有video-dec0/video-enc0），采集测试明确失败，因此未完成实时摄像头/真实WiFi整链路复测。使用已有实拍样本的离线真实NPU推理，不是新采集或软件模拟NPU。相机恢复后需复测60fps采集条件下的主循环与预览帧率。本轮未连接串口、未执行机械动作。

未在本轮添加跨帧预处理流水线、改变NPU核分配或替换硬件解码。首先完成了已授权方案中的原生输入A/B与通过后接入，进一步优化需基于相机恢复后的瓶颈。
