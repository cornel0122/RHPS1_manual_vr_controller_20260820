# RHPS1 控制器融合说明

## 目标

这份目录是工程师提供的 `topic-back_to_lipm_walking` 分支的独立融合副本。
目标是使用该分支原生面向 RHPS1 的控制器和 retargetting task，同时接入本机
已经完成校准的 ROS2 Vive 手柄、手臂 tracker、腰部 tracker 和 HMD 输入。

原有稳定控制器目录和系统安装未被覆盖：

- 原有控制器：`~/luyisi/guillaume_code/mc_ana_avatar_controller_li_basic`
- 融合源码：`~/luyisi/guillaume_code/mc_ana_avatar_controller_rhps1_fusion`
- 融合安装：`~/luyisi/guillaume_code/install_ana_rhps1_fusion`
- 融合启动器：`~/luyisi/run_ana_rhps1_mujoco_fusion`
- 固定基座 RobotModule：`~/luyisi/guillaume_code/mc_rhps1_fixed_fusion`
- 固定基座 MuJoCo 模型：`~/luyisi/guillaume_code/rhps1_fixed_mujoco`

## 主要改动

1. 控制器改名为 `ANAAvatarControllerRHPS1`，避免与稳定版
   `ANAAvatarController` 冲突。
2. 所有库、状态和配置安装到独立前缀，不写入
   `~/workspace/workspace/install`。
3. 启动流程改为
   `Pause -> ANA::PrepareRHPS1ForVR -> Retargetting`。准备状态先在身体近侧
   弯肘，再向桌面前上方抬手，最后小幅下降到工作姿势；不执行旧版 HRP4CR
   的 `PutHandsAboveTable` 笛卡尔轨迹。
4. 本机的 ISMPC 依赖接口较旧，因此只构建：
   - `ANAAvatarControllerRHPS1`：LIPM 版本；
   - `ANAAvatarControllerRHPS1_none`：无行走版本。
5. 将旧控制器中已验证的 `ViveHandBridge` 和 `ViveHeadBridge` 移入融合版，
   并适配 ROS2 Humble。
6. RHPS1 使用自己的 frame 和关节：
   - 腰部：`BODY`
   - 左右手：`LeftHandSupportPlate`、`RightHandSupportPlate`
   - 夹爪关节：`L_HAND`、`R_HAND`
7. 保留自然站立校准矩阵、手腕支点补偿和线速度前馈；角速度前馈仍关闭。
8. RHPS1 使用 `[-0.10, 0, 0.15] m` 的 BODY 坐标位置补偿，使双手目标后移
   10 cm、上移 15 cm。该补偿不改变已经校准的坐标轴或旋转矩阵。
9. `none` 版本只加载离线的 `mocap_plugin`，因为工程师版 retargetting
   state 初始化时仍通过它取得采样频率。它不会作为动作输入；实际目标由
   Vive bridge 写入。
10. 新增 `RHPS1_sake2_sake2_Fixed_MuJoCo`。它与原 `sake2+sake2` 模型使用
    相同关节、夹具、PD gains 和头部双目相机，只移除根部 `Root` 自由关节。
11. 两个控制器变体都安装专用 `plugins/ROS.yaml`：只发布 control robot，
    不重复发布 real/environment robot，并把发布周期设为 `0.05 s`，避免
    `Full ROS message publishing queue`。

## 当前启动流程

不需要 SteamVR 即可验证 RHPS1 是否正常加载：

```bash
cd ~/luyisi
./run_ana_rhps1_mujoco_fusion
```

该命令现在默认使用固定基座。需要恢复原来的自由浮动动力学时使用：

```bash
ANA_RHPS1_BASE=floating ./run_ana_rhps1_mujoco_fusion
```

固定基座的用途是先验证自动准备轨迹、Vive 输入、夹具、相机和共享自主等上肢
功能，防止轻微桌面或物体碰撞把整机推倒。它不用于评价平衡性能，也不能替代
真实 RHPS1 上的稳定器。

完整 Vive 验证时另外启动 SteamVR 和：

```bash
cd ~/luyisi
./run_vive_talker
```

融合控制器启动后会先自动执行 `ANA::PrepareRHPS1ForVR`。它不是直接插值到
最终姿态，而是依次执行四段关节空间轨迹：

1. `ANA::PrepareRHPS1ClearTable`：在身体近侧把双肘弯到 `-1.2 rad`；
2. `ANA::PrepareRHPS1BendElbows`：继续弯肘到 `-1.5 rad`；
3. `ANA::PrepareRHPS1ReachHigh`：肩俯仰到 `-0.65 rad`，双手抬到桌面前上方；
4. `ANA::PrepareRHPS1LowerHands`：肩俯仰回到 `-0.5 rad`，小幅下降到工作姿态。

最终姿态为双肩俯仰 `-0.5 rad`、左右肩横滚 `+0.4/-0.4 rad`、双肘俯仰
`-1.5 rad`，其余肩、肘和腕关节回到 `0 rad`。四段 posture task 的刚度均为
`0.5`，每段到达误差小于 `0.06` 后进入下一段。

这组目标来自 RHPS1 的 `ForceSensorCalibration` 配置，目的与
`PutHandsAboveTable` 相同，都是在遥操作前形成可用的双臂准备姿态；实现方式
则改为 RHPS1 关节空间运动，避免套用 HRP4CR 的手部 frame 和世界坐标轨迹。
进入 retargetting 后，左右 hand/arm task 会锁定在准备动作结束时的实际姿态。
仍需左右手各自第一次按下复位/确认键，才开始使用 Vive 数据驱动机器人；
接管过程使用 2 秒平滑插值，不会在第一帧跳到人体目标。双键只表示“开始”，
不会重新建立手或腰部零点。

工程师分支原先还为左右肩俯仰设置了 `1e6` 的关节权重，并强制趋向
`-1.5236 rad`。该旧 Unity 姿态偏置会与 Vive 手部目标冲突，RHPS1 融合版已
将其移除。

## 已完成验证

2026-07-31 完成以下自动回归：

- `mc_mujoco` 正确加载 `RHPS1_sake2_sake2_Fixed_MuJoCo`；
- 固定模型与 RobotModule 的 34 项 MuJoCo 参考关节顺序一致；
- 正确加载独立控制器 `ANAAvatarControllerRHPS1_none`；
- 建立左右脚与地面的接触；
- 完成 `Avatar::Initial`、`Pause` 和 `ANA::PrepareRHPS1ForVR`；
- 依次完成避开桌沿、弯肘、高位前伸和从上方下降四个阶段；
- 进入 `Avatar::Retargetting::HeadPlusLeft`；
- 创建左右 hand、arm 和 head retargetting task；
- 成功订阅两个 controller、两个 arm tracker、waist tracker 和 HMD；
- 自动准备状态正常完成，无崩溃或缺失 datastore。
- 55 秒运行期间未再出现 `Full ROS message publishing queue`。
- 新准备姿态的 BODY 相对手部锚点为：
  - 左手 `[0.510, 0.288, 0.422] m`
  - 右手 `[0.510, -0.288, 0.422] m`

固定版启动时可能出现两类非致命信息：

- `BodySensor ... fixed base`：固定根部忽略基座速度更新，符合预期；
- 未运行 `run_vive_talker` 时，进入 retargetting 后会打印 hand task 的
  `Broken cstr`，因为没有新的有效 Vive 目标。

2026-07-31 的低刚度轨迹回归中，四段动作约耗时 `25.2 s`。几何回放确认：

- 桌面顶面高度约为 `0.952 m`；
- 手和腕进入桌面水平范围时，最低高度约为 `1.12 m`；
- 准备动作结束时浮动基座高度为 `0.836 m`，俯仰约为 `-0.16°`。

因此新轨迹不会穿过桌面，且在进入 retargetting 前机器人保持直立。固定模式
不会发生整机倒地；切换到自由浮动模式后仍需观察身体摆动和脚底滑移。

未启动 `run_vive_talker` 时，进入 retargetting 后旧版
`ForceConstrainedTransformTask` 仍可能打印 `Broken cstr`。自动准备状态本身
已经完成，但这些力约束日志仍需在完整 Vive 运行和真实 RHPS1 接入前单独确认。

## 尚需现场验证

以下项目必须连接 Vive 后由操作者确认：

- RHPS1 是否始终保持站立；
- 自动准备姿态的速度、双手最终高度和与桌面的间距；
- 左右 hand/arm/head 的方向和幅度是否适合 RHPS1 身体尺寸；
- 首次双手确认、夹爪按钮和线速度前馈；
- RHPS1 专用位置偏移是否需要重新标定。

RHPS1 与 HRP4CR 的肩到腕长度分别约为 `0.49 m` 和 `0.505 m`，因此当前差异
不是 RHPS1 手臂更长。更显著的几何区别是 RHPS1 肩部基准约向前 4 cm，外加
此前缺少手部位置补偿。

启动器可通过环境变量选择控制器变体。默认仍使用已经验证可以进入
retargetting 的无行走版本：

```bash
ANA_RHPS1_CONTROLLER=ANAAvatarControllerRHPS1_none ./run_ana_rhps1_mujoco_fusion
```

LIPM 对照版本可用以下命令选择，但当前缺少工程师配置要求的 footstep 和安全
插件，并且会停在 `LIPMWalking::Initial`，暂时不作为默认启动方式：

```bash
ANA_RHPS1_CONTROLLER=ANAAvatarControllerRHPS1 ./run_ana_rhps1_mujoco_fusion
```

共享自主已于 2026-08-03 移入该融合控制器，并已替换为 RHPS1 的手部 frame、
关节和当前 MuJoCo 场景目标。详细接入和日志判定方式见工作区根目录的
`RHPS1迁移修改记录_20260729.md`。

## 本次修复回退点

2026-07-31 启动姿态修复前的代码保存在 Git 标签：

```bash
rhps1-fusion-before-start-pose-fix-20260731
```

## 回退

融合版通过独立命令启动。停止它后，继续使用原命令即可回到原系统：

```bash
cd ~/luyisi
./run_scripts_optional/run_ana_rhps1_mujoco
```

不要把融合安装目录手工复制到 `~/workspace/workspace/install`。
