# Cornel Vive 暂停、继续与组间复位

## Controller 名称

纯手动调试：

```text
ANAAvatarControllerRHPS1CornelVive_manual
```

四条件正式实验：

```text
ANAAvatarControllerRHPS1CornelVive_experiment
```

正式实验只在启动 MC Control 前选择一次 experiment controller。四种实验条件和每组之间的
机器人复位均在 controller 运行期间完成，不重启 BodyState、hrpsys、MC Control 或机器人。

## Vive 控制状态

`Avatar -> Vive -> Control state` 显示：

- `READY`：尚未开始；
- `RUNNING`：Vive 正在驱动 retargetting task；
- `PAUSED`：目标保持，不响应 Vive；
- `RESETTING`：正在返回实验起始姿态。

按键：

1. `READY` 或 `PAUSED` 时，左右手柄在 0.5 秒窗口内同时按下 `menu`，立即开始或继续控制。
2. `RUNNING` 时，同时持续按住两个 `menu` 1.5 秒，进入 `PAUSED`。
3. 每次动作后必须松开两侧 `menu`，才能触发下一次动作。
4. 单手 `menu` 不触发开始、暂停或继续。

RHPS1 使用 direct waist-relative 映射。暂停和继续不修改手、臂或腰部零点。暂停期间机器人保持
当前 task target；继续后恢复当前的 waist-relative retargetting，不补偿为新的手部零点。

## 组间机器人复位

Vive bridge 启动时记录：

- 左右 hand task target；
- 左右 arm task target；
- 当时的实际机器人关节姿态（浮动基座不修改）。

该记录称为 `controller start pose`。如果按标准真机流程在 half-sitting 和传感器校准完成后启动
MC Control，它就是本次实验实际使用的安全起始姿态，而不是 walking controller 内部另一套未验证
的 half-sitting 参数。

操作流程：

1. 双手长按 `menu`，确认状态变为 `PAUSED`。
2. 打开 `Avatar -> Experiment Reset`。
3. 点击 `Reset to experiment start pose`。
4. controller 在默认 8 秒内平滑插值 hand/arm task 和关节 posture target。
5. `Reset status` 显示 `complete; waiting for two-hand menu press` 后，切换 Operator PC 条件/脚本。
6. 双手同时短按 `menu`，继续下一组。

运行中点击 reset 会被拒绝。复位过程中可点击 `Abort reset`，机器人保持中止时的当前目标，并继续
处于 `PAUSED`。

## 四条件运行时切换

Operator PC 继续向 `/ana/experiment_condition` 发布 `条件编号:session-id`。映射为：

```text
1 = 0.0 s 延迟，无辅助
2 = 0.0 s 延迟，有辅助
3 = 1.5 s 延迟，无辅助
4 = 1.5 s 延迟，有辅助
```

收到新 session 时，controller 冻结当前目标、清空旧延迟队列并进入等待状态。操作者完成 GUI
复位和脚本切换后，以双手同时短按继续。条件切换本身不会启动机器人运动。

## 真机安全边界

- 首次部署先在无运动条件下确认 GUI 状态和按键计数。
- 第一次复位必须有急停人员，并验证记录的 start pose 确实是本次启动的安全姿态。
- reset 不是通用 walking half-sitting 命令，不会再次进入 `LIPMWalking::Initial`。
- 出现未按键自行运动、连续旋转或目标跳变时立即急停，不用手柄反向纠正。
