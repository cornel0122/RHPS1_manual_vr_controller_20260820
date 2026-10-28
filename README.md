# RHPS1 Vive Teleoperation and Experiment Controller

## Cornel 真机隔离版本

`cornel-vive-controller` 分支为避免与 Control PC 上已有的 ANA/Unreal
controller 冲突，使用独立名称：

```text
ANAAvatarControllerRHPS1CornelVive_manual
ANAAvatarControllerRHPS1CornelVive_experiment
```

对应的 superbuild project target 为：

```text
ANAAvatarControllerRHPS1CornelVive_project
```

本分支导出纯手动和四条件实验 controller；不会覆盖已有的
`ANAAvatarControllerRHPS1_manual`。应把仓库作为独立目录放在
`mc-rtc-superbuild/extensions/` 下构建。

真机实验应在启动时选择 `ANAAvatarControllerRHPS1CornelVive_experiment`，之后在同一次
MC Control 运行中切换四种条件，不需要重启机器人。Vive 操作方式为：

- `READY/PAUSED`：双手同时短按 `menu`，开始或继续 direct waist-relative retargetting；
- `RUNNING`：双手同时长按 `menu` 1.5 秒，暂停并保持当前机器人姿态；
- 暂停/继续不会建立新的手部零点；
- `Avatar -> Experiment Reset` 仅在暂停时允许机器人用 8 秒返回 controller 启动时记录的实验起始姿态。

详细流程见 `docs/Cornel_Vive暂停与组间复位_20261028.md`。

本目录用于把 RHPS1 的 Vive 手动遥操作控制器从 Operator PC 搬运到实验室 Control PC，
并提供一套不必重新编译即可切换四种实验条件的真机实验 controller。

## 目录

```text
controller/
  ANAAvatarControllerRHPS1 源码
ros2_ws/src/vr_interface_msgs/
  Vive controller/tracker 的 ROS 2 自定义消息
operator_pc/
  RHPS1 ZED 图像检查、ZED SDK 转换和 OpenVR HMD overlay
docs/
  中文安装、运行、风险和迁移记录
```

## 当前目标

仅验证以下链路：

```text
Vive -> ROS 2 -> ViveHandBridge/ViveHeadBridge
     -> ANA retargeting tasks -> mc_rtc QP -> RHPS1
```

稳定的纯手动回退版本为：

- 输入延迟：0 秒；
- 线速度前馈：开启，gain=1.0；
- 角速度前馈：关闭；
- 意图识别：关闭；
- 共享自主：关闭；
- YOLO/CHMM：不运行；
- MuJoCo/磁力抓取：不部署。

四条件实验版本为：

```text
ANAAvatarControllerRHPS1CornelVive_experiment
```

Control PC 始终运行这一套 controller；Operator PC 用一个参数选择：

```text
1 = 0.0 s 延迟，无辅助
2 = 0.0 s 延迟，有辅助
3 = 1.5 s 延迟，无辅助
4 = 1.5 s 延迟，有辅助
```

参数到达只会配置条件并让 Vive bridge 进入待命，不会触发机器人运动。左右手柄同时短按
`menu` 后开始或继续 direct waist-relative 控制；重新发送新的实验 session 时也会冻结当前 task
并重新等待双手确认。条件 3/4 同时延迟手/臂/腰输入和 HMD 头部旋转，但不延迟机器人相机画面。
条件 1/3 在控制输出层硬性关闭共享自主，但仍可运行意图识别以记录 belief。

## 阅读顺序

首先阅读：

```text
docs/RHPS1实体机器人_最小手动VR控制器安装摘要_20260820.md
```

需要理解完整电脑分工和后续扩展时再阅读：

```text
docs/RHPS1实体机器人部署与运行说明.md
docs/RHPS1迁移修改记录_20260729.md
```

## 安装材料

Control PC 需要构建：

1. `ros2_ws/src/vr_interface_msgs`；
2. `dependencies/ForceConstrainedTransformTask`；
3. `dependencies/mc_neuron_mocap_plugin`；
4. `controller`。

第二项只是旧 retargetting state 的编译和 datastore 兼容层，不是实际的姿态输入源，
并且不会自动加载。实际输入仍来自 Operator PC 上的 Vive talker。

Operator PC 保留 SteamVR 和 ViveSimpleInterface talker，不需要把 OpenVR talker 安装到 Control PC。

实体夹具与 Vision PC/ZED 画面接入见：

```text
docs/RHPS1真机夹具与ZED画面准备_20260824.md
```

## Superbuild extension 安装

仓库顶层 `CMakeLists.txt` 是 mc-rtc-superbuild extension 入口。在 Control PC 上应将
整个仓库克隆到现有 superbuild 的 `extensions/` 目录，不要直接安装到系统目录。

该分支的 extension 会按以下顺序注册四个项目：

```text
vr_interface_msgs
ForceConstrainedTransformTask
mc_neuron_mocap_plugin
ANAAvatarControllerRHPS1CornelVive_project
```

最后一个是 superbuild 内部的 project target 名；安装后在 mc_rtc 配置中选择的
纯手动 controller 名为 `ANAAvatarControllerRHPS1CornelVive_manual`，四条件实验名称为
`ANAAvatarControllerRHPS1CornelVive_experiment`。

独立构建控制器时请在 CMake 配置参数中加入
`-DINSTALL_DOCUMENTATION=OFF`。源码包不包含生成后的 Doxygen 文档；该参数只关闭文档安装，不影响控制器、状态插件或配置文件。

## 重要安全边界

实体机器人必须选择新增的纯手动 controller：

```text
ANAAvatarControllerRHPS1CornelVive_manual
```

该版本已经跳过 `PrepareRHPS1ForVR`，运行链中也不包含 IntentRecognitionBridge、
UI_SharedAutonomy 或左右 SharedAutonomy。进入手动状态后，Vive bridge 会保持机器人
当前手臂姿态，直到左右手柄同时短按 `menu`；运行中双手长按 1.5 秒可暂停控制。

在实体机器人上开始运动前仍需：

1. 由 RHPS1 工程师确认肩部 local-minimum posture override；
2. 确认真机 RobotModule、夹具名称、安装前缀和 LIPM 稳定器配置；
3. 确认急停人员、脚底接触和初始站姿；
4. 先完成 ROS 2 跨电脑输入检查。

未经以上确认，不要在实体机器人上按下 `Start MC Control`。

完成纯手动链路验证后，四条件实验的安装与运行方法见：

```text
docs/RHPS1实体机器人_四条件实验控制器说明_20261013.md
```

## 来源

- 控制器来源分支：`rhps1-fusion`
- 打包时基准提交：`ce7c879`
- 该分支包含尚未提交的 Vive bridge、RHPS1 配置和实验功能修改；本目录保存的是 2026-08-20 工作区快照，而不是仅由上述提交重建的版本。
- 本目录不包含任何已编译 `.so`、build/install 目录、日志、MuJoCo 模型或嵌套 Git 仓库。
