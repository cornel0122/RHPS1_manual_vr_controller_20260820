# LEAP 右手关节方向核对：测试步骤

## 目的

确认真机 LEAP 右手每个关节的实际转动方向是否与 RViz 模型（URDF）一致。

背景：按代码推算，“按下 trigger”应让模型握拳（关节走到 URDF 上限），但真机是按下张开、松开握拳。
在弄清原因前，不能把直接指定关节角度的方案（ABCD、ring、默认 A 姿势）用于真机。

## 前提

- 机器人按《RHPS1真机启动清单》启动到 Start MC Control，LEAP 配置（`RHPS1_leap_leap` + `_leap_manual`）。
- **不要开始 VIVE tracking**（VIVE 开始后调试界面会自动解除）。
- 急停有人值守；右手周围没有物体；手机录像。
- 每次只动一个关节，每次只动 15°。

## 界面位置

RViz：`Avatar / LEAP Grasp Tuning / Right Hand`
- 按钮在 `Right Hand` 主页面
- 关节滑条在 `Finger 1`、`Thumb` 等子页面，每个关节有 `command [deg]`（当前命令）、`encoder [deg]`（编码器实测）和 `target [deg]`（目标）
  - 2026-11-24 之前的版本只有 `actual [deg]`，**它其实是命令值，不是实测**（测试时会全为 0）。
  - 主页面 `Encoder readings` 显示编码器读数是否可用；不可用时 `encoder [deg]` 为 NaN。

## 步骤

1. `Slow apply duration [s]` 设为 **5**。
2. 点 `ARM RIGHT HAND EDITOR`，确认 `Armed = true`。
3. 点 `CAPTURE ACTUAL INTO EDITOR`（编辑器 = 当前实际姿势）。
4. 截图记录 16 个关节的 `command [deg]` 和 `encoder [deg]`（基准）。
5. 按下表顺序，每个关节做一次：
   1. 把该关节 `target [deg]` 设为 **基准 + 15°**（超出范围就用 基准 − 15°，并在表里注明）。
   2. 点 `APPLY EDITOR SLOWLY`，观察**真机手指**和 **RViz 模型**各往哪个方向动。
   3. 记下动作后的 `encoder [deg]`（编码器实测）。
   4. 点 `RESTORE CAPTURED TO EDITOR`，再点 `APPLY EDITOR SLOWLY`，回到基准。
6. 全部做完点 `DISARM AND HOLD`。

**中止**：任何异常（跳动、异响、卡住、接近极限）立即点 `HOLD NOW`；必要时急停。

## 记录表

模型含义：“+” 表示 URDF 正方向在模型里的动作（由 MuJoCo 正向运动学测得）。

| 关节 | 模型里 “+” 的含义 | RViz 模型实际动作 | 真机实际动作 | 动作后 encoder [deg] |
|---|---|---|---|---|
| `R_MCP_1` | 食指根部弯曲 | | | |
| `R_PIP_1` | 食指向远离拇指方向摆（外展） | | | |
| `R_DIP_1` | 食指中节弯曲 | | | |
| `R_FINGERTIP_1` | 食指指尖弯曲 | | | |
| `R_THUMB_CMC` | 拇指转向手心（对掌） | | | |
| `R_THUMB_PIP` | 拇指绕自身轴转动 | | | |
| `R_THUMB_DIP` | 拇指弯曲 | | | |
| `R_THUMB_FINGERTIP` | 拇指指尖弯曲 | | | |
| `R_MCP_2`（可选） | 中指根部弯曲 | | | |

## 结果判读

| 观察 | 含义 | 处理 |
|---|---|---|
| RViz 与真机方向**相反** | 插件或硬件映射与模型相反 | 修正映射需负责插件的人；ring/ABCD 关节值需按实测关系换算 |
| RViz 与真机方向**一致**，且 “+” = 弯曲 | 模型正确，问题在 trigger→opening 映射 | 在 controller 配置中处理 open/closed 方向（本仓库可改） |
| 只有部分关节相反 | 个别关节标定或映射问题 | 逐个记录，换算时逐关节处理 |
| `encoder` 不跟随 `target` | 反馈或控制异常（或插件不回传该关节编码器） | 停止测试，检查插件与日志 |

---

# 第二部分：ring 抓取上真机

前提：第一部分（方向核对）已完成。ring 代码默认**不启用**（`leap_trigger_path: "linear"`，
且 `direction_verified: false`），安装后真机行为与原来完全相同。

## 0. 安装（Control PC，由用户执行）

```bash
cd ~/workspace/mc-rtc-superbuild/extensions/rhps1_cornel_vive_controller
git switch cornel-vive-controller
git pull --ff-only origin cornel-vive-controller
cd ~/workspace/build/ANAAvatarControllerRHPS1CornelVive_project
ninja install
```

## 1. 填写方向表

把第一部分的结果告诉 Claude，由其修改
`controller/etc/ANAAvatarController.in.yaml` 中 `leap_ring` 的：

- `joint_map`：16 个整数，第 i 个位置（发给电机的槽位）取模型的第几个关节（0–15 的排列；默认不对调）
- `joint_sign`：16 个 +1 / −1
- `joint_offset_deg`：16 个偏移（只有“范围内镜像”这类情况才需要）

规则：`第 i 槽位命令 = joint_sign[i] × 模型角度[joint_map[i]] + joint_offset_deg[i]`。

**2026-11-24 测试结果给出的建议值**（已写在 `ANAAvatarController.in.yaml` 的注释里）：
四指 `MCP_k` 与 `PIP_k` 对调（`joint_map: [1,0,2,3, 5,4,6,7, 9,8,10,11, 12,13,14,15]`），全部 `joint_sign: -1`。
`joint_offset_deg` 尚未测得：用 `encoder [deg]` 在命令 0 和几个已知姿态下读数标定，或先阅读 LEAPHandPlugin 源码。
注意：槽位命令仍按该槽位关节的 URDF 范围钳位（mc_rtc Gripper），偏移选错会让部分姿态被截断，
`Ring Trigger Path` 页面和日志会提示被截断的关节。
提交推送后，按第 0 步重新拉取安装。此时仍保持 `direction_verified: false`。

## 2. RViz 慢速预览 ring 四个姿态（不需要 VIVE）

页面：`Avatar / LEAP Grasp Tuning / Right Hand`

1. `ARM RIGHT HAND EDITOR`
2. 子页面 `Ring Trigger Path`：确认 `Ring status` 为 `preview only: direction table not verified`；
   核对 `A command [deg]`、`D command [deg]` 与方向表一致
3. 点 `LOAD RING A/B/C/D INTO CANDIDATES (OVERWRITES SLOTS)`
4. 子页面 `ABCD Sequence`：四段时长都设为 **5 s**
5. **空手**点 `START CURRENT -> A -> B -> C -> D`，对照仿真图检查每个阶段：
   - A：手张开成 C 形，**拇指转到和食指同一平面并向外伸出**
   - B：食指弯曲包向远侧
   - C：拇指弯过来与食指合成环
   - D：中指、无名指包上来
6. 任一阶段方向不对 → `STOP SEQUENCE / HOLD NOW`，记录是哪个关节，回到第 1 步修改方向表
7. 空手正确后，把瓶子放在手掌前方（不接触），重复第 5 步，观察是否形成环
8. `DISARM AND HOLD`

## 3. 让 trigger 走 ring 路径

1. 第 2 步全部正确后，由 Claude 把 `direction_verified` 改为 `true`，提交推送，按第 0 步重新安装
2. 启动到 Start MC Control，**不要**马上开始 VIVE tracking
3. `Ring Trigger Path` 页面勾选 `Use ring path for the right trigger`（只能在未 RUNNING 时切换），
   确认 `Right trigger path = ring`
4. Operator PC：`./run_vive_talker_rhps1 --control-mode right`，右手 menu 开始
5. **开始后手会以最高 90°/s 平滑地移动到 A 姿势**（松开 trigger = A）
6. 先空手：慢慢扣 trigger，看 `Current segment` 依次经过 A→B→C→D；松开应回到 A
7. 再抓瓶：**手先停稳，再扣 trigger**（仿真中边走边扣成功率较低）

## 中止与回退

- 异常：`HOLD NOW` / menu 长按暂停 / 急停
- 回退到原来的线性 trigger：暂停后取消勾选 `Use ring path...`；或配置改回 `leap_trigger_path: "linear"`

## 已知注意点

- A 姿势拇指向外伸出约 15 cm，移动手臂、放下手臂时注意不要撞到桌面和身体
- 仿真中 B 阶段食指会把瓶子往手心拉，瓶子可能短暂倾斜；装水的瓶子要注意
- 关节值是仿真第一版，真机上可能需要微调（改 `leap_ring` 的 A/B/C/D，不需要改代码）
