# 独立底盘检查（2026-10-24）

目标：cat:/home/cat/ds_innovation-refactor-20261024。

本轮未发送真实串口TX，未执行实车运动，也未修改验收配置或冻结基线。

## 结果

- 四向入口存在，独立调用demo_serial.manual_motion，不运行视觉任务。
- 默认move_forward.sh --speed 0.05 --seconds 0.5返回CHECK_NO_DEVICES并拒绝未验收配置，符合预期。
- 默认配置尚缺机构动作映射、0/1实测状态、静止映射试验、<=200ms看门狗测量，以及操作人/日期/证据记录。
- tests/test_demo_launchers.py：17/17通过。
- tests/test_motion_protocol.py：4/4通过。
- tests/test_frame_ctl.py：16/16通过。
- tests/test_pitch_prepare5.py：9/9通过。
- 上述测试使用离线数据或虚拟串口，不构成实物验证。
- build-arm64-selfcontained全量CTest：42/43通过。
- 唯一失败rescue_runtime_legacy_baseline：config/frame_20261008.assume.json、tests/fixtures/frame_mapping.synthetic.json资源不符，check_mechanical、check_carry_demo标准输出不符；与已有MANUAL_CONTROL_20261024.md记录一致。未通过改写基线消除失败。
- 未发现/dev/ttyACM0占用。只读查询30帧稳定：A6 00 00 78 00 0E 19 0A，state=0，id=0，pitch=120。反馈不等于机械到位证据。

## 尚待现场完成

观察零速度单次开关动作，记录真实映射；核对下位机看门狗及测量证据；之后逐条执行低速短时四向点动，观察方向、机构保持及停车。不能将pending或assume配置直接当作实测验收。

当前结论：底盘相关离线回归通过、串口只读反馈正常；实车验证未完成，不能声明没有问题。

## 命令记录与反馈表述修正

- 现场用户确认关框后实物闭合，但后续A6保持state=1、id=2；因此不能将该字节直接显示为夹爪实际开闭位置。
- query_lower和frame_ctl说明改为原始协议字段，明确尚待固件确认。
- frame_ctl和demo_serial在成功写入命令后输出GRIPPER_COMMAND_SENT，记录偏移、编号、速度、相机偏移和完整包，明确不是实测位置。该记录仅用于日志，不参与放行判断。
- 0x56发送布局、速度/时限/超时/中止/停车保护及验收门控未改动。
- 修改前备份：backups/command-reporting-20261024-221150。
- 修改验证：frame 16项、demo 17项、motion contract 4项通过；只读A6显示验证通过。本轮修改未发送真实串口TX。
