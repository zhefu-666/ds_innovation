# 目标丢失原地搜索

接近线索、20°接近、40°接近时连续丢失超过原宽限帧数，或者闭爪重识别3秒未成功，进入LOST_SEARCH。先停车切5°，反馈稳定后确认夹爪闭合、IMU航向有效及旋转空间，再以scan_wz（启动脚本1.0rad/s）原地搜索，前进速度始终为0。

搜索按正常货物规则筛选：首趟绿色，之后黑色优先，蓝色不可成为抓取目标。可使用新编号，但连续3次确认期间停车，确认后重新进入接近流程。每次最多一圈IMU累计角或20秒，先到者停止；不限制累计重搜次数；单次搜索耗尽仍保持SAFE_STOP。已张爪/有载荷不进入转圈。路径阻挡时不会强制转动。

本次只修改编译和软件测试，不启动或恢复运动。此功能不能修复夹爪动作编号反馈不匹配；mid_gripper_open_timeout仍需单独处理。

## 2026-10-05 补充：20°二次搜索

5°一圈（或20秒）未找到合格货物时，不再直接SAFE_STOP：先停车切到20°（intermediate_pitch，需--allow-mechanical-pitch-model且track>20°时启用），反馈稳定后以同样规则再转一圈/20秒，reason=rotating_for_lost_target_mid。20°轮找到且距离≤track_near_m(0.4m)的货物直接进入MID_REACQUIRE保持20°视角；更远的仍走CUE_APPROACH(5°)。20°轮也耗尽才SAFE_STOP lost_search_exhausted。每次新进入LOST_SEARCH都从5°开始。未配置20°时行为与原来一致。
备份：backups/lost-search-mid-pitch-20261005-064132；build-pitch40-telemetry-20261004 编译通过，ctest 16/16。尚未实车验证。
