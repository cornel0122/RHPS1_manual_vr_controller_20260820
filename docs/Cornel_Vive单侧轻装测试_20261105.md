# Cornel Vive 单侧轻装测试

## 目的与结构

只佩戴腰部 tracker、某一侧 arm tracker 和该侧 Vive controller，快速验证该侧机器人手臂与夹具。

不增加新的 mc_rtc controller。Operator PC 的 Vive talker 启动脚本发布本次控制模式，Control PC
上的原 controller 接收模式后，再检查该模式要求的设备和 task 是否齐全。另一侧设备可以不佩戴，
且不会收到手臂或夹具命令。

以下原 controller 均支持该模式：

- `ANAAvatarControllerRHPS1CornelVive_manual`
- `ANAAvatarControllerRHPS1CornelVive_leap_manual`
- experiment 版本也能接收模式，但正式双侧实验应固定使用 `both`。

## Operator PC 启动命令

双侧正式控制：

```bash
cd ~/luyisi/rhps1_cornel_vive_controller/operator_pc
./run_vive_talker_rhps1 --control-mode both
```

只测试右侧：

```bash
./run_vive_talker_rhps1 --control-mode right
```

只测试左侧：

```bash
./run_vive_talker_rhps1 --control-mode left
```

脚本在运行 talker 的同时，以可靠、transient-local QoS 发布 `/ana/vive_control_mode`。退出脚本时模式
发布器与 talker 一起停止。也可以用环境变量 `RHPS1_VIVE_CONTROL_MODE` 指定默认值。

## 安全条件与按钮

以 `right` 为例，controller 只在以下条件全部满足时允许开始：

1. 已收到 `right` 模式；
2. 腰部 tracker 在线；
3. 右臂 tracker 在线；
4. 右手柄在线且右侧 hand task 存在。

操作为：

- 右手 `menu` 短按：开始或继续控制；
- 右手 `menu` 长按 1.5 秒：暂停并保持当前目标；
- Sake2 controller：右手 `grip` 切换右侧夹具开合，`trigger` 发送抓取通知；
- LEAP controller：右手模拟 `trigger` 连续控制整手开合，`grip` 发送抓取通知；
- 左侧输入、左侧 arm task 更新和左侧夹具命令均被忽略。
- HMD 头部控制被冻结；头部 task 保持当前目标且目标速度归零。

左侧测试完全对称。`both` 模式保持原有双手 menu 逻辑。

## 模式切换限制

- `RUNNING` 时拒绝改变模式；必须先用当前模式对应的 menu 长按暂停。
- 在 `READY` 或 `PAUSED` 状态收到新模式后，controller 冻结目标、清除旧输入，并重新等待所需设备
  的新数据及 menu 确认。
- controller 每次启动默认是 `both`；没有收到有效条件时不会自行进入单侧模式。
- 只有 `both` 模式允许 `ViveHeadBridge` 使用 HMD 控制头部；`left/right` 均不要求佩戴 HMD。
- Control PC 的 `MainRobot` 仍须与现场 RobotModule、IOB 和实际夹具类型一致。

## GUI 检查

在 `Avatar / Vive` 页面确认：

- `Active side` 显示 `left`、`right` 或 `both`；
- 所选侧手柄、arm tracker 与 `Waist tracker` 为 true；
- 单侧模式下另一侧为 false 是正常现象；
- 短按所选侧 menu 后，`Control state` 从 `READY` 变为 `RUNNING`。

单侧模式不会绕过机器人硬件急停、IOB、RobotModule、夹具注册或 mc_rtc 站立稳定器检查。
