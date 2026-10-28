# Cornel controller：RHPS1 双 LEAP Hands 部署

## Control PC 选择

纯手动验证：

```yaml
MainRobot: RHPS1_leap_leap
Enabled: [ANAAvatarControllerRHPS1CornelVive_leap_manual]
Timestep: 0.005
```

四条件实验：

```yaml
MainRobot: RHPS1_leap_leap
Enabled: [ANAAvatarControllerRHPS1CornelVive_leap_experiment]
Timestep: 0.005
```

实体机器人不得选择名称带 `_MuJoCo` 或 `_Fixed` 的 RobotModule。

## 行为

- VIVE `menu`、双手长按暂停、继续和实验组间复位与 Sake 版本相同；
- VIVE `grip` 每次上升沿在整手张开/闭合之间切换；
- `trigger` 只向意图系统发送“已经抓住物体”的通知；
- controller 启动时不会主动发送手指目标，第一次按 `grip` 后才控制手指；
- 每只 LEAP Hand 必须由 RobotModule 注册 16 个主动关节，否则该侧手指命令被禁用。

## LEAPHandPlugin

生成配置默认使用：

```yaml
LEAPHandPlugin:
  - name: left_leap_hand
    usb_port: /dev/ttyUSB0
    robot: rhps1
    gripper: l_gripper
    simulation: false
    baudrate: 4000000
  - name: right_leap_hand
    usb_port: /dev/ttyUSB1
    robot: rhps1
    gripper: r_gripper
    simulation: false
    baudrate: 4000000
```

现场必须先用设备信息确认左右手串口；如果顺序相反，应修改安装后的 controller
配置，不能在未确认时按 `grip`。Control PC 还必须已经安装 `mc_leap_hand`，并能加载
`LEAPHandPlugin`。

## 首次实体测试

1. 确认 LEAP Hands 供电、USB、硬件急停以及当前 IOB/RobotModule 都是 74 关节 LEAP 版本。
2. 先选择 `ANAAvatarControllerRHPS1CornelVive_leap_manual`，暂不启动 VIVE talker。
3. 启动 MC Control，确认不再出现 74/42 编码器数量不匹配。
4. 日志中左右两侧都必须出现 `profile=leap, active_joints=16` 和 `ready`。
5. 启动 VIVE talker，只验证手臂 menu 开始、暂停和继续，暂不按 `grip`。
6. 将实体手移到无物体、无碰撞位置，先测试单侧一次 `grip`。
7. 确认方向和低速安全保护正确后，再测试张开与另一侧。
8. 手动版本验证完成后，才切换到 `_leap_experiment`。

若日志显示 `expects 16 active joint(s)`，controller 会禁止对应侧的 VIVE grip
命令；此时应修复 `RHPS1_leap_leap` RobotModule/IOB 配置，不应绕过检查。
