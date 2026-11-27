# LEAP 右手 ring 抓取：现场操作单（2026-11-27）

**目标**：松开 trigger = ring 张开（拇指在食指平面向外张开、其余手指张开）；扣到底 = 握紧；
中间平滑过渡：先食指包、再拇指合、最后其余手指收紧（仿真 `leap_grasp_physics_lab.py --strategy ring` 的顺序）。

**做法（快速版，约 30–40 分钟）**：不去弄清每个电机的方向，直接在真手上**摆出“张开”和“握紧”两个姿势**，
工具自动生成中间两帧。只改我们自己的 controller 配置，**不碰 LEAP 插件或任何硬件配置**。

**安全**：急停有人守；**不要开始 VIVE tracking**（开始后调试界面自动解除）；右手周围无物体（第 3 步再放瓶子）；
`Slow apply duration` = 5 s；每次只改 1–2 个关节、每次 ≤15°；异常 → `HOLD NOW`，必要时急停。

---

## 1. 安装最新 controller（Control PC，由用户执行）

```bash
cd ~/workspace/mc-rtc-superbuild/extensions/rhps1_cornel_vive_controller
git switch cornel-vive-controller
git pull --ff-only origin cornel-vive-controller
cd ~/workspace/build/ANAAvatarControllerRHPS1CornelVive_project
ninja install
```
按启动清单启动到 Start MC Control（LEAP：`RHPS1_leap_leap` + `ANAAvatarControllerRHPS1CornelVive_leap_manual`）。

## 2. 示教“张开”姿势（RViz，10–15 分钟）

界面：`Avatar / LEAP Grasp Tuning / Right Hand`（各关节滑条在 `Finger 1`、`Thumb` 等子页面）。

1. `Slow apply duration [s]` = 5 → `ARM RIGHT HAND EDITOR`（`Armed = true`）→ `CAPTURE ACTUAL INTO EDITOR`。
2. 调 `target [deg]` → `APPLY EDITOR SLOWLY`，看**真手**，反复直到真手是 **ring 张开**：
   拇指转到和食指同一平面并向外伸出，食指/中指/无名指张开、略弯（参考仿真 A 图）。
   滑条对应的真机关节和模型名字不一定一样（11-24：四指根部两个关节对调、方向相反），**以真手为准**。
3. 子页面 `Candidate A` → `SAVE EDITOR AS A`。16 个数会显示在 `Saved values [deg, slot order]`，
   同时打印在 controller 终端（窗口 3）：`LEAP candidate A [deg, slot order ...]: [...]`。**拍照或复制这一行**。

## 3. 示教“握紧”姿势（10–15 分钟）

1. 把瓶子放到张开的手里（仿真的位置：瓶子在掌心前方、拇指与食指之间）。
2. 逐步调 `target` → `APPLY EDITOR SLOWLY`，让真手**握住瓶子**：食指包住远侧、拇指合上与食指成环、其余手指贴上去。
   在“刚好贴紧”之后再多弯 5–10°，扣到底时才有握力（触碰检测会让手指碰到物体后停下）。
3. 子页面 `Candidate D` → `SAVE EDITOR AS D`，同样**拍照或复制那一行**。
4. 慢慢退回张开（`LOAD A INTO EDITOR` → `APPLY EDITOR SLOWLY`），拿走瓶子 → `DISARM AND HOLD`。

## 4. 生成配置（Operator PC，2 分钟）

```bash
cd ~/luyisi/rhps1_cornel_vive_controller/operator_pc
python3 tools/leap_ring_calibration.py from-poses \
  --open   "[A 的 16 个数]" \
  --closed "[D 的 16 个数]"
```
输出 A/B/C/D 四帧（B = 食指先合、C = 再加拇指、D = 全部握紧）和方向表（恒等）。显示 OK 就把输出交给 Claude
写进 `controller/etc/ANAAvatarController.in.yaml` 的 `leap_ring` 并推送；然后按第 1 步重新安装
（此时 `direction_verified` 仍为 false）。想换合拢顺序：`--stages thumb,index,others` 等。

## 5. RViz 慢速预览（5–10 分钟，不需要 VIVE）

1. `ARM RIGHT HAND EDITOR`；子页面 `Ring Trigger Path`：`Ring status` = `preview only: direction table not verified`。
2. `LOAD RING A/B/C/D INTO CANDIDATES (OVERWRITES SLOTS)`；`ABCD Sequence` 四段都设 **5 s**。
3. **空手** `START CURRENT -> A -> B -> C -> D`：A 张开 → B 食指合 → C 拇指合 → D 全握。
4. 再把瓶子放进去重复一次。不对 → `STOP SEQUENCE / HOLD NOW`，回第 2/3 步重新示教。→ `DISARM AND HOLD`。
   （RViz 模型的样子会和真手不一样：关键帧是按槽位直接示教的，只是显示问题，以真手为准。）

## 6. 解锁并用 trigger（10 分钟）

1. Claude 把 `direction_verified: true` 写进配置并推送 → 重新安装（第 1 步）。
2. 启动到 Start MC Control，**先不开始 VIVE**；`Ring Trigger Path` 勾选 `Use ring path for the right trigger`，
   确认 `Right trigger path = ring`。
3. Operator PC：`./run_vive_talker_rhps1 --control-mode right`，右手 menu 开始。
   **开始后手会以 ≤90°/s 移到张开姿势**（松开 trigger = A）。拇指伸出较多，注意碰撞。
4. 空手慢慢扣 trigger：`Current segment` 依次 A→B→C→D，松开回到 A。
5. 抓瓶：**手先停稳，再扣 trigger**（仿真中边走边扣成功率低）。

**回退**：暂停后取消勾选 `Use ring path...`，或配置改回 `leap_trigger_path: "linear"`（原来的整手线性 trigger）。

---

## 附录：逐关节标定（时间充裕、想原样复用仿真设计的 A/B/C/D 时）

逐个滑条测：真手动的是哪个关节（`moved_joint`）、方向是否为模型“+”（`plus_is_model_plus` Y/N）、
真手关节与模型 0 姿态一致时的命令值（`zero_cmd_deg`）：

```bash
python3 tools/leap_ring_calibration.py template --prefill-1124 -o ~/leap_ring_sheet.csv   # 预填 11-24 推测
python3 tools/leap_ring_calibration.py compute ~/leap_ring_sheet.csv
```
输出 `joint_map / joint_sign / joint_offset_deg` 和换算后的 A/B/C/D；带 `*` 的值会被截断，先复查该槽位的零点。
“模型 +”方向：`MCP_k` 根部弯曲；`PIP_k` 根部侧摆、远离拇指；`DIP_k`/`FINGERTIP_k` 弯曲；`THUMB_CMC` 转向手心；
`THUMB_PIP` 绕自身轴转；`THUMB_DIP`/`THUMB_FINGERTIP` 弯曲。原理见 `docs/LEAP关节方向核对_测试步骤.md`。
