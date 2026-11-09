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
- 后部模拟 `trigger` 连续控制整手开合程度：松开为张开，完全扣下为闭合；
- VIVE `grip` 的上升沿只向意图系统发送“已经抓住物体”的通知，不改变手指目标；
- `trigger >= 0.90` 发送 `gripper_closed` 并记录最近物体，`trigger <= 0.10`
  发送 `gripper_opened` 并清除抓取状态；
- RViz `Avatar / Vive / Grasp confirmation` 有左右手备用确认按钮；
- 手指命令只在 VIVE tracking 开始后启用；暂停时保持最后的整手目标，继续后按当前
  `trigger` 位置继续控制，不重新建立零点；
- 模拟输入带 5% 死区和低通滤波，实体夹具仍受 controller 中的低速与安全参数限制；
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
配置，不能在未确认时扣动 `trigger`。Control PC 还必须已经安装 `mc_leap_hand`，并能加载
`LEAPHandPlugin`。

## 首次实体测试

1. 确认 LEAP Hands 供电、USB、硬件急停以及当前 IOB/RobotModule 都是 74 关节 LEAP 版本。
2. 先选择 `ANAAvatarControllerRHPS1CornelVive_leap_manual`，暂不启动 VIVE talker。
3. 启动 MC Control，确认不再出现 74/42 编码器数量不匹配。
4. 日志中左右两侧都必须出现 `profile=leap, active_joints=16` 和 `ready`。
5. 启动 VIVE talker，只验证手臂 menu 开始、暂停和继续，暂不扣动 `trigger`。
6. 将实体手移到无物体、无碰撞位置，先在单侧缓慢扣动并松开 `trigger`，确认整手
   开合方向及比例正确。
7. 将 trigger 拉入闭合区并按一次 `grip`，确认日志/意图系统依次收到
   `gripper_closed` 和 `grasp_notify`，实体手不应因 `grip` 改变目标。
8. 松开 trigger，确认收到 `gripper_opened`。再测试 RViz 对应侧备用确认按钮。
9. 确认低速安全保护正确后，再测试另一侧；手动版本通过后才切换到 `_leap_experiment`。

若日志显示 `expects 16 active joint(s)`，controller 会禁止对应侧的 VIVE 手指
命令；此时应修复 `RHPS1_leap_leap` RobotModule/IOB 配置，不应绕过检查。
