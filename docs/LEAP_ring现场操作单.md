# LEAP 右手 ring 抓取：现场操作单（2026-11-27 准备）

目标：松开 trigger = A（拇指在食指平面向外张开、其余手指张开），扣下按 A → B → C → D 依次合拢
（仿真 `operator_pc/leap_grasp_physics_lab.py --strategy ring`）。代码已就绪，只差真机的**关节方向表**。
背景与原理：`docs/LEAP关节方向核对_测试步骤.md`。预计 60–90 分钟，大部分是第 3 步测量。

**安全**：急停有人守；**不要开始 VIVE tracking**（VIVE 开始后调试界面自动解除）；右手周围无物体；
每次只动一个关节、每次 ≤15°、`Slow apply duration` = 5 s；异常 → `HOLD NOW`，必要时急停。

---

## 0. 出发前（Operator PC，5 分钟）

```bash
cd ~/luyisi/rhps1_cornel_vive_controller/operator_pc
python3 tools/leap_ring_calibration.py template --prefill-1124 -o ~/leap_ring_sheet.csv
```
`~/leap_ring_sheet.csv` 已按 11-24 的推测预填（四指根部 MCP/PIP 对调、全部反向），现场只需**核对**并测**零点**。

## 1. 安装最新 controller（Control PC，由用户执行）

```bash
cd ~/workspace/mc-rtc-superbuild/extensions/rhps1_cornel_vive_controller
git switch cornel-vive-controller
git pull --ff-only origin cornel-vive-controller
cd ~/workspace/build/ANAAvatarControllerRHPS1CornelVive_project
ninja install
```
按启动清单启动到 Start MC Control（LEAP：`RHPS1_leap_leap` + `ANAAvatarControllerRHPS1CornelVive_leap_manual`）。

## 2. 只读查看 LEAP 插件配置（2 分钟，Control PC）

```bash
sed -n '1,200p' ~/.config/mc_rtc/plugins/LEAPHandPlugin.yaml
```
拍照或复制给 Claude。看有没有电机顺序 / 方向 / 零点之类的设置（例如 ids、directions、offsets、reverse）。
**只看不改**。如果插件能改，那是比 controller 方向表更“正”的修法（第 4 步会用到这个判断）。

## 3. 逐槽位测量（RViz，40–60 分钟）

界面：`Avatar / LEAP Grasp Tuning / Right Hand`。“槽位”= RViz 里滑条的关节名（发给电机的那个位置）。

**3.1 准备**
1. `Slow apply duration [s]` = 5 → `ARM RIGHT HAND EDITOR`（`Armed = true`）→ `CAPTURE ACTUAL INTO EDITOR`。
2. 把 16 个 `target [deg]` 全部设 **0** → `APPLY EDITOR SLOWLY`。
   **给 RViz 模型截图**（这就是“模型 0 姿态”的参考图），同时给**真机手**拍照。记下主页面 `Encoder readings` 是否可用。

**3.2 每个槽位**（先做食指 4 个 + 拇指 4 个；中指、无名指若与食指规律相同，每指只验 1 个关节即可）
1. 该槽位 `target` 设 **+15**（超范围就 −15，并在 note 里写明）→ `APPLY EDITOR SLOWLY`。
2. 看**真机**：动的是哪根手指的哪个关节 → 填 `moved_joint`（用模型名，见下表）；
   它的方向是不是该关节的“模型 +”方向（见下表）→ 填 `plus_is_model_plus`：是 `Y`，否 `N`。
   （+15 时若方向是“模型 +”的反方向，填 N；用 −15 测的，按“命令增大”的方向换算后再填。）
3. **零点**：调这个槽位的 `target`（每次 5–10°），直到这个真机关节看起来和**参考截图里模型的同一关节**一样
   （伸直 / 不侧摆）→ 把此时的 `command [deg]` 填进 `zero_cmd_deg`。同时看 `encoder [deg]` 是否跟上命令。
4. 该槽位 `target` 改回 0 → `APPLY EDITOR SLOWLY`，进入下一个槽位。

| 模型关节名 | 位置 | “模型 +” 方向 |
|---|---|---|
| `MCP_1` / `_2` / `_3` | 食指 / 中指 / 无名指 根部 | 弯曲 |
| `PIP_1` / `_2` / `_3` | 同上 根部 | 侧摆，远离拇指 |
| `DIP_k` | 中节 | 弯曲 |
| `FINGERTIP_k` | 指尖 | 弯曲 |
| `THUMB_CMC` | 拇指根部 | 转向手心（对掌） |
| `THUMB_PIP` | 拇指 | 绕自身轴转动 |
| `THUMB_DIP` | 拇指中节 | 弯曲 |
| `THUMB_FINGERTIP` | 拇指指尖 | 弯曲 |

**3.3 结束**：`DISARM AND HOLD`。

## 4. 计算方向表（Operator PC，2 分钟）

```bash
python3 tools/leap_ring_calibration.py compute ~/leap_ring_sheet.csv
```
- 输出 `joint_map / joint_sign / joint_offset_deg`（直接贴进 `controller/etc/ANAAvatarController.in.yaml` 的 `leap_ring`），
  以及 A/B/C/D 换算到每个槽位的命令值。
- 带 `*` 的值超出该槽位的 URDF 范围、会被截断（真机姿态与设计不同）。
  - **没有 `*`、显示 OK** → 交给 Claude 写进配置并推送 → 第 1 步方法重新安装（`direction_verified` 仍为 false）。
  - **有 `*`** → 先复查对应槽位的 `zero_cmd_deg`；仍有 → controller 方向表解决不了，要在 LEAP 插件里改电机顺序/方向
    （看第 2 步的配置；修改 Control PC 由用户执行）。
  - 用 11-24 推测值、零点全 0 试算：23 个值被截断——所以零点测量是关键。

## 5. RViz 慢速预览 A/B/C/D（10 分钟，不需要 VIVE）

1. `ARM RIGHT HAND EDITOR`；子页面 `Ring Trigger Path`：`Ring status` 应为 `preview only: direction table not verified`，
   对照第 4 步输出核对 `A command [deg]`、`D command [deg]`。
2. `LOAD RING A/B/C/D INTO CANDIDATES (OVERWRITES SLOTS)`；子页面 `ABCD Sequence` 四段都设 **5 s**。
3. **空手** `START CURRENT -> A -> B -> C -> D`，每段对照仿真：
   - A：手张开成 C 形，拇指转到和食指同一平面并向外伸出（拇指伸出约 15 cm，注意碰撞）
   - B：食指弯曲包向远侧
   - C：拇指弯过来与食指合成环
   - D：中指、无名指包上来
4. 哪一段不对 → `STOP SEQUENCE / HOLD NOW`，记下关节，回到第 3 步复测该槽位。
5. 空手正确后，瓶子放在手掌前方（不接触）重复一次，看能否成环。→ `DISARM AND HOLD`。

## 6. 解锁并用 trigger 走 ring（10 分钟）

1. 第 5 步全部正确 → Claude 把 `direction_verified: true` 写进配置并推送 → 重新安装（第 1 步）。
2. 启动到 Start MC Control，**先不开始 VIVE**；`Ring Trigger Path` 页面勾选 `Use ring path for the right trigger`，
   确认 `Right trigger path = ring`。
3. Operator PC：`./run_vive_talker_rhps1 --control-mode right`，右手 menu 开始。
   **开始后手会以 ≤90°/s 移动到 A 姿态**（松开 trigger = A）。
4. 先空手：慢慢扣 trigger，`Current segment` 依次 A→B→C→D；松开回到 A。
5. 再抓瓶：**手先停稳，再扣 trigger**（仿真中边走边扣成功率低）。

**回退**：暂停后取消勾选 `Use ring path...`，或配置改回 `leap_trigger_path: "linear"`。

---

## 记录表（与 `~/leap_ring_sheet.csv` 相同，可打印手填）

| 槽位 | 预填：动的关节 | 预填：方向 | 实测：动的关节 | 实测：Y/N | 零点 `command` [deg] | encoder 跟上？ | 备注 |
|---|---|---|---|---|---|---|---|
| MCP_1 | PIP_1 | N | | | | | |
| PIP_1 | MCP_1 | N | | | | | |
| DIP_1 | DIP_1 | N | | | | | |
| FINGERTIP_1 | FINGERTIP_1 | N | | | | | |
| MCP_2 | PIP_2 | N | | | | | |
| PIP_2 | MCP_2 | N | | | | | |
| DIP_2 | DIP_2 | N | | | | | |
| FINGERTIP_2 | FINGERTIP_2 | N | | | | | |
| MCP_3 | PIP_3 | N | | | | | |
| PIP_3 | MCP_3 | N | | | | | |
| DIP_3 | DIP_3 | N | | | | | |
| FINGERTIP_3 | FINGERTIP_3 | N | | | | | |
| THUMB_CMC | THUMB_CMC | N | | | | | |
| THUMB_PIP | THUMB_PIP | N | | | | | |
| THUMB_DIP | THUMB_DIP | N | | | | | |
| THUMB_FINGERTIP | THUMB_FINGERTIP | N | | | | | |

预填值来自 11-24 测试（四指根部两个关节对调、方向全反；拇指顺序一致），**全部要现场确认**。
11-24 的记录里无名指与拇指是否对调尚未完全确认，测 `MCP_3`/`THUMB_*` 时特别留意。
