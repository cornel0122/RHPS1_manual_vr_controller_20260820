# HMD 头部相机 overlay（Vulkan 纹理）

把一个 `sensor_msgs/CompressedImage`（JPEG）话题以 **Vulkan GPU 纹理**的方式显示为 SteamVR overlay。
源码原位于 `~/luyisi/head_camera_openvr_texture_overlay/`（2026-09/10 Codex 编写，未纳入版本管理），
2026-11-24 复制到本仓库备份。日常使用 `operator_pc/run_zed_vulkan_hmd_overlay`。

| 程序 | 说明 |
|---|---|
| `head_camera_openvr_vulkan_overlay` | Vulkan 纹理版（当前使用） |
| `head_camera_openvr_texture_overlay` | OpenGL 纹理版（旧，依赖 SDL2/GLEW） |

## 编译

```bash
source /opt/ros/humble/setup.bash
cmake -S operator_pc/hmd_vulkan_overlay -B operator_pc/hmd_vulkan_overlay/build \
  -DOPENVR_ROOT=$HOME/openvr          # OpenVR SDK：headers/ 与 bin/linux64/libopenvr_api.so
cmake --build operator_pc/hmd_vulkan_overlay/build -j4
```

启动器默认使用 `~/luyisi/head_camera_openvr_texture_overlay/build/head_camera_openvr_vulkan_overlay`，
可用 `HEAD_CAMERA_VULKAN_OVERLAY_BIN` 指向新编译的程序。

## 本机（Operator PC）的限制与已知问题（2026-11-24）

- **VIVE Pro 接在 Intel 集成显卡的 DP 口**（`card1-DP-1`，PCI 00:02.0）。本机只有 HDMI 口走 NVIDIA，
  VIVE Pro 只支持 DisplayPort，所以 SteamVR compositor 只能跑在 Intel Arc 上，改 Steam 启动选项无效。
  overlay 的 Vulkan 纹理必须建在同一块 GPU（Intel）上，启动器已处理。
- **必须用 CycloneDDS**：`libopenvr_api.so` 自带 C++ 运行库，Fast DDS 打开端口时内部抛出并捕获的异常
  会被错误分派而 `std::terminate`（程序一启动即 abort）。
- **SteamVR 10-26 更新后的现象**：`setOverlayRaw` / `setOverlayFromFile` 持续更新几秒后 vrcompositor
  断言崩溃（`Vulkan transfer ring buffer overflow; transfer size 2315040`，与图像大小无关）；
  本程序不崩溃，但 HMD 中大部分时间为白屏，偶尔出现画面（Codex 10-01 验证时的彩色条纹现在也一样）。
- 代码只有**一张**纹理：SteamVR 读取的同时下一帧正在写入。改为 2–3 张纹理轮换是首选修复；
  仍白屏则把 SteamVR 回退到 10-26 之前的版本对照。
- overlay 是单眼平面画面，没有立体深度；需要立体视觉要改为提交左右眼纹理的 VR 应用。
