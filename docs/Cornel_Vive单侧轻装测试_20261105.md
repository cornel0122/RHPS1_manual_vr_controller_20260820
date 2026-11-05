# Cornel Vive 单侧轻装测试

## 目的

只佩戴腰部 tracker、某一侧 arm tracker 和该侧 Vive controller，快速验证该侧机器人手臂与夹具。
另一侧设备可以全部不佩戴，也不会阻止启动或收到控制命令。

## Controller 名称

Sake2 夹具：

- 左侧：`ANAAvatarControllerRHPS1CornelVive_left_test`
- 右侧：`ANAAvatarControllerRHPS1CornelVive_right_test`

LEAP Hand：

- 左侧：`ANAAvatarControllerRHPS1CornelVive_leap_left_test`
- 右侧：`ANAAvatarControllerRHPS1CornelVive_leap_right_test`

LEAP 单侧版本只加载所选侧的 `LEAPHandPlugin` 配置，另一侧物理手可以不安装。
Control PC 的 `MainRobot` 仍必须与现场实际的 RobotModule/IOB 配置一致。

## 佩戴与操作

以右侧测试为例，只需要：

1. 腰部 tracker；
2. 右臂 tracker；
3. 右手 Vive controller。

进入 retargetting 后：

- 右手 `menu` 短按：开始或继续控制；
- 右手 `menu` 长按 1.5 秒：暂停并保持当前目标；
- 右手 `grip`：切换右侧夹具开合；
- 左侧 Vive 设备、左侧 arm task 和左侧夹具命令均被忽略。

左侧测试的操作完全对称。

## GUI 检查

在 `Avatar / Vive` 页面确认：

- `Active side` 显示 `left` 或 `right`；
- 所选侧 `data`、arm tracker 与 `Waist tracker` 为 true；
- 另一侧为 false 是正常现象；
- `Control state` 在短按 menu 后从 `READY` 变为 `RUNNING`。

单侧测试仍保留机器人硬件急停和 mc_rtc 的站立稳定器，不应把它当作绕过现场安全流程的模式。
