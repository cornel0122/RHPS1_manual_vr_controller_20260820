# RHPS1 Manual Vive Teleoperation Controller

本目录用于把 RHPS1 最基础的 Vive 手动遥操作控制器从 Operator PC 搬运到实验室 Control PC。

## 目录

```text
controller/
  ANAAvatarControllerRHPS1 源码
ros2_ws/src/vr_interface_msgs/
  Vive controller/tracker 的 ROS 2 自定义消息
docs/
  中文安装、运行、风险和迁移记录
```

## 当前目标

仅验证以下链路：

```text
Vive -> ROS 2 -> ViveHandBridge/ViveHeadBridge
     -> ANA retargeting tasks -> mc_rtc QP -> RHPS1
```

预期实验条件：

- 输入延迟：0 秒；
- 线速度前馈：开启，gain=1.0；
- 角速度前馈：关闭；
- 意图识别：关闭；
- 共享自主：关闭；
- YOLO/CHMM：不运行；
- MuJoCo/磁力抓取：不部署。

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
2. `controller`。

Operator PC 保留 SteamVR 和 ViveSimpleInterface talker，不需要把 OpenVR talker 安装到 Control PC。

独立构建控制器时请在 CMake 配置参数中加入
`-DINSTALL_DOCUMENTATION=OFF`。源码包不包含生成后的 Doxygen 文档；该参数只关闭文档安装，不影响控制器、状态插件或配置文件。

## 重要安全边界

当前代码可以用于编译、安装、controller 发现和 ROS 2 通信验证，但在实体机器人上开始运动前还需要完成：

1. 跳过或替换基于 MuJoCo 桌面绝对坐标的 `PrepareRHPS1ForVR`；
2. 从最小运行 FSM 移除 IntentRecognitionBridge、UI_SharedAutonomy 和左右 SharedAutonomy 状态；
3. 由 RHPS1 工程师确认肩部 local-minimum posture override；
4. 确认真机 RobotModule、夹具名称、安装前缀和稳定器配置。

未经以上确认，不要在实体机器人上按下 `Start MC Control`。

## 来源

- 控制器来源分支：`rhps1-fusion`
- 打包时基准提交：`ce7c879`
- 该分支包含尚未提交的 Vive bridge、RHPS1 配置和实验功能修改；本目录保存的是 2026-08-20 工作区快照，而不是仅由上述提交重建的版本。
- 本目录不包含任何已编译 `.so`、build/install 目录、日志、MuJoCo 模型或嵌套 Git 仓库。
