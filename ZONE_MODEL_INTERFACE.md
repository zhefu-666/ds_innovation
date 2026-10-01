# 安全区两版模型接口

默认蓝色为己方、红色为对方；比赛使用 `--team red` 切换。

当前通用模型继续使用 `--pose-model models/zone_pose_fp.rknn`。
后续两版训练完成并转成 RKNN 后，可以同时配置：

```text
--team blue --pose-model-blue models/zone_blue_fp.rknn --pose-model-red models/zone_red_fp.rknn
```

上述专用文件名仅为未来接口示例，不表示文件已存在。每次只加载所选己方版本，不同时推理两版。
本方专用路径优先于通用路径；本方未配置时回退通用模型。如果只配置了对方模型，则报错，不自动借用。

两版统一参数：安全区框置信度 `--pose-conf 0.25`，关键点置信度 `--pose-kpt-conf 0.5`，NMS `--nms 0.45`，输入尺寸 `--input-size 640`。物块置信度 `--conf 0.5` 不变。
两版必须保持类别顺序 zone_left、zone_right，关键点顺序 far_left、near_left、far_right、near_right，以及现有输出格式。
颜色选择只决定模型路径和己方身份；区域归属仍通过现有颜色观测确认，不能用文件名代替观测。颜色标定仍按现有接口提供。
