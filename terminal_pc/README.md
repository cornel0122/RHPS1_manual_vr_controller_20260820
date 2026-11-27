# Terminal PC：HMD + SteamVR + VIVE（备用方案，2026-11-27 准备，未在 Terminal PC 上验证）

**什么时候用**：Operator PC（笔记本）上 HMD 画面始终不可用时。笔记本的 VIVE Pro 只能接在 Intel 核显的 DP 口上
（NVIDIA 只有 HDMI），SteamVR 在 Linux + Intel 核显上不是常规配置。11-27 查明之前的“白屏”主要是 HMD 闲置待机
（关屏超时 5 秒，已改 5 分钟），所以**大概率不需要搬**；本目录是以防万一。

## 分工

| Terminal PC（本目录） | Operator PC（不变） |
|---|---|
| SteamVR、HMD、VIVE 手柄/tracker（它们都由接 HMD 的那台电脑上的 SteamVR 管理，必须一起搬） | 唯一的 ZED 接收进程、意图识别、开发、实验控制台、日志 |
| VIVE talker：直接用 `operator_pc/run_vive_talker_rhps1`（Zenoh client → Control PC 路由器，与现在相同） | 新增 `operator_pc/run_hmd_jpeg_forwarder`：把 HMD 用 JPEG 用 TCP 发到 Terminal PC |
| `terminal_pc/run_hmd_terminal`：TCP 接收 JPEG → 本机发布 → Vulkan overlay | |

Control PC 收到的话题与现在完全相同，controller 不用改。图像不经过两台电脑之间的 ROS（不改动 Operator PC
已调好的图像链网络配置），只用一条 TCP（端口 30555，约 3 MB/s）。同一 ZED 流只能有一个接收进程，所以由
Operator PC 接收后转发，Terminal PC 不直接连 Vision PC 的 ZED 流。

## 1. 只读检查（先做这一步，什么都不改）

```bash
cd ~/<放 controller 仓库的位置>/rhps1_cornel_vive_controller   # 没有就先 git clone（见第 2 步）
bash terminal_pc/check_terminal_pc_env.sh
```
把输出（或 `/tmp/rhps1_terminal_pc_check_*.txt`）发给 Claude。关键看：NVIDIA 显卡**有没有 DP 口**（VIVE Pro 只支持 DP）、
ROS 2 Humble 与 `rmw_zenoh_cpp`/`rmw_cyclonedds_cpp`、SteamVR、能否连到 Zenoh 路由器 `10.16.236.199:7447`。

## 2. 安装与编译（检查结果确认可行后，由用户在 Terminal PC 上执行）

```bash
# controller 仓库（含本目录、VIVE 启动脚本、overlay 源码）
git clone -b cornel-vive-controller https://github.com/cornel0122/RHPS1_manual_vr_controller_20260820.git ~/rhps1_cornel_vive_controller
# OpenVR SDK（头文件 + libopenvr_api.so）
git clone --depth 1 https://github.com/ValveSoftware/openvr.git ~/openvr
# 依赖（缺哪个装哪个，见检查结果）
sudo apt install libvulkan-dev libopencv-dev libsdl2-dev libglew-dev ros-humble-rmw-cyclonedds-cpp
# Vulkan overlay
source /opt/ros/humble/setup.bash
cd ~/rhps1_cornel_vive_controller
cmake -S operator_pc/hmd_vulkan_overlay -B operator_pc/hmd_vulkan_overlay/build -DOPENVR_ROOT=$HOME/openvr
cmake --build operator_pc/hmd_vulkan_overlay/build -j4
```
- **ViveSimpleInterface**（`vr_interface_node` talker）：私有仓库 `cornel-ri/vive_simple_interface`，或从 Operator PC 的
  `~/luyisi/ViveSimpleInterface-main` 拷过去，在 Terminal PC 上重新 `colcon build`（不要拷 build/install）。
  启动时用 `VIVE_ROS_SETUP=<它的 install/setup.bash>` 指过去。
- `rmw_zenoh_cpp`：Control PC/Operator PC 已有；Terminal PC 没有的话按同样方式安装（检查结果会显示）。
- SteamVR 设置：“启动/关闭 → 在此之后关闭显示”改为 5 分钟（默认 5 秒，会导致待机“白屏”）；启动选项
  `VALVE_SKIP_RUNTIME_SAFETY=1 %command%`（与笔记本相同，需要时）。
- 搬设备：HMD 的 link box（DP + USB + 电源）、VIVE 手柄/tracker 的无线接收器（Watchman dongle）一起插到 Terminal PC。
  配对信息在 dongle/HMD 上，一般不用重新配对；不行就在 SteamVR 里重新配对。

## 3. 运行

Terminal PC：
```bash
# 终端 1：SteamVR（设备全部变绿）
# 终端 2：VIVE talker（和在笔记本上一样）
VIVE_ROS_SETUP=<ViveSimpleInterface>/ros/install/setup.bash \
  ~/rhps1_cornel_vive_controller/operator_pc/run_vive_talker_rhps1 --control-mode right
# 终端 3：HMD 画面
~/rhps1_cornel_vive_controller/terminal_pc/run_hmd_terminal
#   先用测试图确认显示：RHPS1_OVERLAY_TEST_PATTERN=1 ~/rhps1_cornel_vive_controller/terminal_pc/run_hmd_terminal
```
Operator PC：
```bash
cd ~/luyisi/rhps1_cornel_vive_controller/operator_pc
./run_zed_sdk_ros_bridge_local                                   # 唯一的 ZED 接收进程（不变）
RHPS1_TERMINAL_PC_IP=<Terminal PC 的 IP> ./run_hmd_jpeg_forwarder # 新增：把 HMD 画面发过去
```
`run_hmd_terminal` 每 5 s 打印 `received=`，`run_hmd_jpeg_forwarder` 打印 `received= sent=`，两边数字一起涨就是通了。
连不上：Terminal PC 的防火墙要放行 TCP 30555（检查结果里 `ufw firewall: ENABLED=`）。

## 已验证 / 未验证
- 11-27 在 Operator PC 上用两个本地 ROS domain 模拟两台电脑：测试图 15 Hz → 转发 → 接收，375/375 帧全部收到。
- overlay 在 SteamVR 用 NVIDIA 时自动选 NVIDIA（代码用 `IVRSystem::GetOutputDevice`），未实测。
- 在 Terminal PC 上的安装、编译、SteamVR、VIVE 均**未验证**。
