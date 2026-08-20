# RHPS1 实体机器人部署与运行说明

## 1. 结论先行

实体机器人不运行 `run_ana_rhps1_mujoco_fusion`。该脚本只负责在 Operator PC
启动 MuJoCo、Fixed RobotModule 和本机仿真插件。

真机控制链应为：

```text
Operator PC
  SteamVR + ViveSimpleInterface talker
             |
             | ROS 2：controller / tracker / HMD
             v
Control PC
  ANAAvatarControllerRHPS1 + mc_rtc + mc_openrtm
             |
             | OpenRTM joint state / command ports
             v
  RobotHardwareComp <-> shm_iob <-> io_main <-> EtherCAT <-> RHPS1
```

相机与意图识别是另一条链：

```text
Vision PC: ZED + YOLO/Cornel recognition
      |                     |
      | ROS 2 image/depth   | goal belief / object position
      v                     v
Operator PC overlay      Control PC shared-autonomy state
```

## 2. 四台电脑的职责

### Operator PC（当前这台电脑）

保留：

- Steam 和 SteamVR；
- ViveSimpleInterface 的 `talker`；
- HMD OpenVR overlay；
- 操作者侧录制、校准和必要的可视化工具。

它发布 `/controller_1`、`/controller_2`、`/tracker_*` 和 `/hmd`。它不运行
真机 QP，也不直接向 EtherCAT 下发关节命令。

### Control PC（机器人内部）

安装并运行：

- RHPS1 RobotModule；
- mc_rtc、mc_openrtm 和机器人已有稳定器/观测器；
- 我们的 `ANAAvatarControllerRHPS1` 源码编译结果；
- `ViveHandBridge`、`ViveHeadBridge`；
- 编译这些 bridge 所需的 ROS 2 和 `vr_interface_msgs` 类型支持；
- 后续需要时安装共享自主状态和插件。

这是实际运行 QP、计算目标关节位置的电脑。

### Vision PC（机器人内部）

安装并运行：

- ZED SDK 和 ZED ROS 驱动；
- RGB、depth 和 CameraInfo 发布；
- Cornel/Walid 的 YOLO 与意图识别程序。

Guillaume 固定目标实验可以暂时不依赖 YOLO；Cornel 真机版本需要这里提供感知。

### Terminal PC

主要负责：

- SSH 进入 Control PC 和 Vision PC；
- `bodystat`、`io-main.log`、`nocnoid22.py`；
- `mc_rtc_ticker`/GUI/RViz；
- 现场监视、启动和停止，不承担主要实时计算。

## 3. 不要复制哪些仿真内容

下列内容不应作为真机依赖部署：

- `RHPS1_sake2_sake2_Fixed_MuJoCo`；
- `rhps1_mj_description`；
- MuJoCo 场景、桌子和物体 XML；
- `mc_mujoco_head_camera`；
- MuJoCo adhesion/磁吸执行器；
- fake object truth 和 `RTGR_SIM_BYPASS_YOLO`；
- 本机已经编译好的 `.so` 文件。

`.so` 不能直接复制到 Control PC，因为 mc_rtc、编译器、ROS 和 ABI 版本必须与
Control PC 一致。应复制源码并在目标环境重新构建。

## 4. Control PC 上需要部署的源码

第一阶段（纯遥操作）至少需要：

1. `mc_ana_avatar_controller_rhps1_fusion`；
2. `vr_interface_msgs`（只需要消息包，Control PC 不需要 OpenVR talker）；
3. 控制器 CMake 当前要求的依赖：
   - mc_rtc（带 ROS 支持）；
   - mc_rhps1；
   - ForceConstrainedTransformTask；
   - mc_neuron_mocap_plugin；
   - lipm_walking_controller；
   - ROS 1 compatibility 依赖；
   - ROS 2 Humble、rclcpp、geometry_msgs、std_msgs。

第二阶段（Guillaume 共享自主）再增加：

- AssistedTeleoperationPlugin；
- SharedAutonomy/UI_SharedAutonomy 状态；
- 真机物体 frame/datastore 来源。

第三阶段（Cornel）再连接 Vision PC 的识别输出，不要一开始全部同时部署。

## 5. 推荐安装方式

教程推荐通过 `mc-rtc-superbuild` extension 安装，而不是直接覆盖机器人已有环境。

### 5.1 建立独立用户/会话

由有权限的人员执行：

```bash
sudo useradd -m -s /bin/bash <username>
sudo passwd <username>
su - <username>
```

是否授予 sudo 权限由管理员决定。不要修改已有稳定演示用户的安装。

### 5.2 先记录 Control PC 环境

至少记录：

```bash
uname -a
echo "$ROS_DISTRO"
echo "$ROS_DOMAIN_ID"
echo "$RMW_IMPLEMENTATION"
pkg-config --modversion mc_rtc
```

同时备份：

```text
~/.config/mc_rtc/mc_rtc.yaml
当前 mc-rtc-superbuild 的 remote、branch 和 commit
当前 mc_rhps1 与 mc_ana_avatar_controller 的 commit
```

### 5.3 先构建 ROS 2 消息包

将 `ViveSimpleInterface-main/ros/vr_interface_msgs` 放入 Control PC 的 ROS 2
colcon workspace，单独构建并 source。Control PC 不需要构建依赖 OpenVR 的
`vr_interface_node`。

示意流程：

```bash
source /opt/ros/humble/setup.bash
cd <ros2_workspace>
colcon build --packages-select vr_interface_msgs
source install/setup.bash
```

### 5.4 建立 superbuild extension

把控制器及新增依赖写入一个独立 extension，放入：

```text
mc-rtc-superbuild/extensions/<project-extension>
```

然后在已 source ROS 2 消息 workspace 的终端中，以 `Release` 构建 superbuild。
具体命令必须以 Control PC 当前 superbuild README 为准，因为研究所环境可能有
自己的 wrapper、安装前缀和 drcutil extension。

重要要求：

- `CMAKE_BUILD_TYPE=Release`；
- 不更换已有 mc_rtc 主依赖 branch，除非工程师明确要求；
- 如果确实需要改主依赖，fork superbuild，并为实验建立独立用户环境；
- 构建后确认 controller、states、robot YAML 和 plugin YAML 都安装成功。

## 6. 真机 mc_rtc 配置

真机应使用非 Fixed、非 MuJoCo RobotModule。当前夹具对应的候选名称是：

```yaml
MainRobot: RHPS1_sake2_sake2
Enabled: [ANAAvatarControllerRHPS1]
Timestep: 0.002
```

最终 RobotModule 名称应与现场 `mc_rhps1` 安装和真实末端执行器一致，由工程师确认。

不要使用：

```yaml
MainRobot: RHPS1_sake2_sake2_Fixed_MuJoCo
Enabled: [ANAAvatarControllerRHPS1_none]
```

`_none` 和 Fixed 组合是当前仿真隔离上肢问题的入口。真机需要正常接触、状态估计、
稳定器和安全插件，通常应使用 `ANAAvatarControllerRHPS1`。

## 7. ROS 2 跨电脑通信必须先单独验证

Operator PC 与 Control PC 必须满足：

- 相同或兼容的 ROS 2 发行版和消息定义；
- 相同 `ROS_DOMAIN_ID`；
- 兼容的 `RMW_IMPLEMENTATION`；
- 网络允许 DDS discovery 和数据流，或使用实验室配置的 Zenoh/router；
- `sudo -E` 启动控制器时保留必要 ROS 环境；
- 不私自修改机器人网络固定 IP。

在启动电机控制前，在 Control PC 验证：

```bash
ros2 topic list
ros2 topic echo /controller_1 --once
ros2 topic echo /tracker_1 --once
ros2 topic echo /hmd --once
```

如果 Control PC 看不到这些话题，controller 就不可能收到 Vive 数据。此时先解决
DDS/Zenoh 网络，不要启动机器人运动。

## 8. 现场启动顺序

以下按教程和现有 RHPS1 流程整理；实际动作必须有工程师/老师在场。

1. 检查急停、外部电源、周围空间和安全鞋，禁止单人测试。
2. Terminal PC 窗口 1：SSH 到 Control PC，运行 `bodystat`。
3. Terminal PC 窗口 2：SSH 到 Control PC：

   ```bash
   tail -f /var/log/io-main.log
   ```

4. Operator PC：启动 SteamVR，确认所有 Vive 设备稳定。
5. Operator PC：启动 `run_vive_talker`。
6. Control PC：先确认能看到 Vive ROS 2 话题。
7. Control PC：进入 RHPS1 hrpsys 目录并启动：

   ```bash
   rhps1
   sudo -E ./hrpsys_mc_rtc.sh
   ```

8. Terminal PC 另一个窗口：

   ```bash
   rhps1
   ./nocnoid22.py
   ```

   教程写作 `nocnoid.py`，现场当前使用 `nocnoid22.py`，以机器人现有脚本为准。

9. Terminal PC：启动 GUI/显示：

   ```bash
   ros2 launch mc_rtc_ticker display.launch
   ```

10. 仅在状态、急停、日志和 controller 名称全部正确后，由负责人执行
    `Start MC Control`。
11. 先保持 Vive tracking 未放行，确认机器人稳定站立且 controller 没有异常目标。
12. 操作者摆好姿势后，再按双手确认键开始 Vive 控制。

Vision PC 的 `zed_stream` 和 YOLO 可以在纯遥操作验证完成后再加入。

## 9. 第一次真机验证必须分级

### 级别 A：只验证安装和加载

- controller 能出现在可选列表；
- YAML、state 和 plugin 全部找到；
- 不报 ABI、missing symbol、ROS message type 错误；
- 不启动运动。

### 级别 B：验证输入通信，但不接管

- Control PC 收到手柄、tracker 和 HMD；
- serial 映射正确；
- 数据频率稳定；
- 双键确认之前机器人目标保持不变。

### 级别 C：零延迟、无共享自主、小幅遥操作

- 先单方向、低幅度；
- 先手位置，再手方向，再 arm tracker，再头部；
- 监视 task error、关节限位、碰撞、稳定器和脚底压力；
- 验证夹具时手臂保持静止。

### 级别 D：HMD 相机

- Vision PC 启动 `zed_stream`；
- 将真实 RGB 话题映射/relay 到 overlay 使用的话题；
- Operator PC 单独运行 overlay；
- 先验证 QoS、分辨率、延迟和断流恢复，不与共享自主同时首测。

### 级别 E：1.5 秒输入延迟

确认零延迟遥操作稳定后再开启，并保留立即停止条件。

### 级别 F：共享自主

先 Guillaume，再 Cornel；先只记录 belief，不施加辅助，再允许非零辅助。

## 10. 仿真配置不能直接用于真机的重点

### 自动准备动作

当前 `PrepareRHPS1ForVR` 的手部目标是针对 MuJoCo 桌面调出的绝对世界位置。
真机桌子、机器人基座和初始姿态不同，第一次真机测试前必须：

- 暂时跳过该自动轨迹，或
- 由 RHPS1 工程师提供真机已验证的关节空间准备姿态/轨迹。

不能直接让实体机器人复现仿真中的绝对手部轨迹。

### Fixed base

仿真中固定根部只是为了隔离上肢问题。真机没有这个软件意义上的固定基座，必须
依赖双脚接触、状态观测和稳定器。

### 校准与 offset

Vive 的方向矩阵可以作为初始值，但以下内容必须在现场重新确认：

- waist tracker 相对人体的位置；
- 当前 `waist_reference_offset_body`；
- 手柄到手腕支点的 offset；
- 人体工作姿态与真实 RHPS1 工作空间；
- 真机桌面高度和机器人到桌子的距离。

### Shoulder local minimum workaround

`postureOverride` 当前只应在仿真验证。`SHOULDER_P` 的 `1e6` 权重可能引起快速
肩部运动，未经负责人确认不能直接用于真实机器人。

## 11. HMD 真机画面链路

目前 overlay 程序仍运行在 Operator PC。真机时仅替换图像来源：

```text
MuJoCo head camera -> RHPS1 Vision PC 的 ZED RGB topic
```

建议用 ROS 2 relay/remap 把实际 ZED 话题统一为：

```text
/rtgr/robot/head_camera/rgb/image_raw
/rtgr/robot/head_camera/depth/image_raw
/rtgr/robot/head_camera/camera_info
```

然后 Operator PC 继续运行：

```bash
./run_head_camera_overlay
```

必须先确认 Vision PC 到 Operator PC 的图像带宽和 QoS。控制话题与图像话题最好
分别监控，避免图像流量影响 Vive 控制数据。

## 12. Cornel 真机链路

Cornel 程序在 Vision PC 使用真实 ZED：

1. YOLO/segmentation 检测物体；
2. depth + CameraInfo 估计物体三维位置；
3. 读取机器人真实手 frame 在相机坐标系中的位置；
4. 生成 goal/belief；
5. belief 发送到 Control PC；
6. Control PC 内 Guillaume 的 SharedAutonomy task 对 RHPS1 手施加辅助。

当前 `env_rtgr_robot.sh` 已明确关闭 MuJoCo truth 和 bypass YOLO，但真实 ZED 话题名、
相机 optical frame、外参和跨机 ROS 2 通信仍需现场确认。

## 13. 常见故障定位

- `bodystat` 无法启动：检查共享内存，教程建议确认 `RHPS1init` 是否执行。
- CORBA naming service 异常：在负责人确认后使用 `clear-omninames.sh`。
- controller 找不到：检查安装前缀、`ControllerModulePaths` 和 controller YAML。
- `vr_interface_msgs` invalid：Control PC 未 source 正确 ROS 2 workspace，或消息版本不一致。
- Operator 有话题、Control 没有：DDS domain、RMW、multicast/Zenoh 或防火墙问题。
- ROS warning：检查 `sudo -E` 是否保留环境，以及现有启动脚本的 ROS 配置。
- 动作延迟：所有实时组件用 Release 构建，run loop 中禁止阻塞式 YOLO/网络调用。
- 真机姿态异常：立即停止，区分输入映射、局部极小值、关节极限、任务冲突和稳定器问题。

## 14. 当前仍需向工程师确认的问题

1. 真机准确 RobotModule 名称是否为 `RHPS1_sake2_sake2`；
2. 现场 control user、superbuild 路径和安装前缀；
3. Control PC 的 ROS 2 domain/RMW 与 Zenoh 配置；
4. `hrpsys_mc_rtc.sh` 实际读取哪个用户的 `mc_rtc.yaml`；
5. 真机应启用哪个 ANA controller variant；
6. RHPS1 已验证的 VR 准备姿态/轨迹；
7. shoulder local-minimum workaround 的正确使用时机；
8. ZED 的真实 RGB/depth/CameraInfo 话题和 optical frame；
9. Cornel belief 从 Vision PC 到 Control PC 的正式 topic/interface；
10. 首次真机测试的批准人和急停操作人。

