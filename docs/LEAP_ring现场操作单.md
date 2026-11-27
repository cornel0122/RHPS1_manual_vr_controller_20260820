# LEAP 右手 ring 抓取：现场操作单（2026-11-27 版）

**目标**：松开 trigger = A（拇指转到食指平面向外张开、其余手指张开略弯），扣下按扳机数值平滑走 A → B（食指包）→ C（拇指合环）
→ D（其余手指收紧），松开按原路退回。A/B/C/D 的角度和合拢顺序**和 MuJoCo 里看到的 ring 完全一样**，代码已有。

**现场要做的只有一件事**：确认“模型角度 → 真机命令”的对照表对不对。表按 11-24 的测试推算好了（配置里的 H1），
**可能对也可能不对**。你只需要在 RViz 里点慢速预览、看真手、用大白话说哪里不对；对照表由 Claude 改，你重装再看。
**不用拖任何关节滑条，不改 LEAP 插件或硬件配置。** 可能来回 2–3 轮，每轮约 10 分钟。

**安全**：急停有人守；**不要开始 VIVE tracking**（开始后调试界面自动解除）；右手周围无物体（第 2.6 步再放瓶子）；
预览每段 5 s；异常 → `STOP SEQUENCE / HOLD NOW`，必要时急停。A 的拇指向外伸出较多，注意不要撞桌面。

---

## 1. 安装（Control PC，由用户执行；每轮改表后重复）

```bash
cd ~/workspace/mc-rtc-superbuild/extensions/rhps1_cornel_vive_controller
git switch cornel-vive-controller
git pull --ff-only origin cornel-vive-controller
cd ~/workspace/build/ANAAvatarControllerRHPS1CornelVive_project
ninja install
```
按启动清单启动到 Start MC Control（LEAP：`RHPS1_leap_leap` + `ANAAvatarControllerRHPS1CornelVive_leap_manual`）。
controller 终端会打印 `right trigger path: linear (ring ready)`（11-27 起已解锁，但默认仍是线性 trigger，需在 RViz 勾选才走 ring），
以及 `ring keyframe A clips R_THUMB_DIP ...`（预期：A/B 里拇指中节想向后弯 30°，真机做不到，变成伸直，无妨）。

## 2. RViz 慢速预览（每轮约 5 分钟）

界面：`Avatar / LEAP Grasp Tuning / Right Hand`。
1. `Slow apply duration [s]` = 5 → `ARM RIGHT HAND EDITOR`（`Armed = true`）。
2. 子页面 `Ring Trigger Path`：`Ring status` 应为 `ready`。
   点 `LOAD RING A/B/C/D INTO CANDIDATES (OVERWRITES SLOTS)`。
3. 子页面 `ABCD Sequence`：四段时长都设 **5 s** → **空手**点 `START CURRENT -> A -> B -> C -> D`。
4. 每一段对照下表看真手，**拍视频**最好：

| 段 | 应该看到 | 常见错法 → 告诉 Claude 的话 |
|---|---|---|
| A | 食指/中指/无名指张开、略弯、**互相平行不岔开**；拇指转到和食指同一平面、向外伸出 | “手指向两边岔开” / “手指攥着没张开” / “拇指没转过来、贴着手掌” / “拇指往手背方向翻” |
| B | 食指弯曲包向远侧；中指、无名指弯一半；拇指不动 | “食指没弯反而往旁边摆” / “食指是伸直的” / “中指无名指也全弯了” |
| C | 拇指弯过来和食指合成环；其他不动 | “拇指往外翻而不是往里合” / “拇指合不到食指” |
| D | 中指、无名指包上来，整体握紧 | “握不紧，手指还是半开” / “某根手指方向反了” |

5. 不对 → `STOP SEQUENCE / HOLD NOW`，把看到的现象（哪一段、哪根手指、往哪边动）发给 Claude → Claude 改对照表并推送
   → 回到第 1 步重装、再预览。对 → 继续。
6. 把瓶子放到张开的手里（仿真位置：掌心前方、拇指与食指之间）再预览一次，看能否成环握住。→ `DISARM AND HOLD`。

对照表的三类可调项（Claude 改，你不用管）：某根手指**岔开** → 侧摆槽位的偏移；某关节**方向反** → 该槽位的符号；
整体**太开/太紧** → 该槽位的偏移。

## 3. 解锁并用 trigger（10 分钟）

1. 配置已解锁（`direction_verified: true`，2026-11-27），无需再重装。预览**大致正确**即可进入下一步。
2. 启动到 Start MC Control，**先不开始 VIVE**；`Ring Trigger Path` 页面勾选 `Use ring path for the right trigger`，
   确认 `Right trigger path = ring`。
3. Operator PC：`./run_vive_talker_rhps1 --control-mode right`，右手 menu 开始。
   11-27 起 `right` 模式下头部也跟随 HMD（`rhps1.yaml` → `head_control_modes: ["both", "right"]`），戴 HMD 即可。
   **开始后手会以 ≤90°/s 移到 A 姿势**（松开 trigger = A）。
4. 空手慢慢扣 trigger：`Current segment` 依次 A→B→C→D，松开回到 A。
5. 抓瓶：**手先停稳，再扣 trigger**（仿真中边走边扣成功率低）。

**回退**：暂停后取消勾选 `Use ring path...`，或配置改回 `leap_trigger_path: "linear"`（原来的整手线性 trigger）。
**时间不够**：跳过本单，直接用线性 trigger（装新版后：松开张开、扣下握拳）戴 HMD 试抓。

---

## 附录 A：对照表是怎么推算的（H1，2026-11-27）

`controller/etc/ANAAvatarController.in.yaml` → `leap_ring`：槽位 i 的命令 = `joint_sign[i]` × 模型角度[`joint_map[i]`] + `joint_offset_deg[i]`，
再被 mc_rtc 限制在该槽位关节的 URDF 范围内。依据 11-24 测试：① 四指根部“弯曲/侧摆”两个槽位对调；② 所有关节方向与模型相反；
③ 命令 0 时手已弯曲、线性 trigger 能握拳 → “真机角度 = 槽位上限 − 命令”（张开在槽位上限）。于是弯曲类槽位的偏移 = 槽位上限
（PIP 60、DIP 108、指尖 117、拇指 120/139.9/108.8/107.7），侧摆槽位偏移 0。最不确定的是侧摆零点（备选 55 或 128）和拇指。
离线验证（MuJoCo 中加载 controller）：表能加载，只有拇指中节 A/B 被截断，状态 `preview only`，trigger 仍为 linear。

## 附录 B：逐关节标定工具（想精确测量时）

```bash
cd ~/luyisi/rhps1_cornel_vive_controller/operator_pc
python3 tools/leap_ring_calibration.py template --prefill-1124 -o ~/leap_ring_sheet.csv   # 已预填 H1
python3 tools/leap_ring_calibration.py compute ~/leap_ring_sheet.csv                     # 输出对照表与截断检查
```
每个槽位记：真手动的是哪个关节（`moved_joint`）、方向是否为模型“+”（Y/N）、真手与模型 0 姿态一致时的命令值（`zero_cmd_deg`）。
“模型 +”：`MCP_k` 根部弯曲；`PIP_k` 根部侧摆、远离拇指；`DIP_k`/`FINGERTIP_k` 弯曲；`THUMB_CMC` 转向手心；`THUMB_PIP` 绕自身轴转；
`THUMB_DIP`/`THUMB_FINGERTIP` 弯曲。原理见 `docs/LEAP关节方向核对_测试步骤.md`。

## 附录 C：直接示教（对照表怎么都调不对时的兜底）

在 RViz 里把真手摆成“张开”存为 A、套瓶“握紧”存为 D（`SAVE EDITOR AS A/D`，16 个数会打印在 controller 终端），然后
`python3 tools/leap_ring_calibration.py from-poses --open "[...]" --closed "[...]"` 生成 A/B/C/D（恒等对照表）。需要拖滑条，耗时。
