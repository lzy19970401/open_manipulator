# OMX 整机参数辨识 — Grill 结论

`/grill-with-docs` 已把 OpenMANIPULATOR-X **System identification** 设计树走通。

**目标（C 混合）**：Phase 1 走通 BIP + 摩擦回归流水线，结果只读；Phase 2 将摩擦写入 `open_manipulator_sysid/config/calibrated_dynamics.yaml`；**Nominal dynamics model** 仍用 URDF 惯性（与知乎原文务实策略一致）。

**范围**：仅 **Arm** 四关节（`joint1`–`joint4`），不含 **Gripper**；不修改现有 GC / bringup yaml。

**包**：仓库根目录独立 ROS 2 包 `open_manipulator_sysid/`。

**工具链**：Python + Pinocchio 离线回归；Phase 1 不做 SymPy BIP 符号重组，直接用 Pinocchio regressor。

**数据采集（D）**：Gazebo 先验收离线流水线 → 实机 **Standard position** 模式 + sysid 自有 controller yaml 录 bag；力矩优先 `joint_states.effort`，回退 `dxl_state` Present Current × 0.00479627 Nm。

**激励（A）**：`excitation.yaml` 多频傅里叶模板；q₀ 为各关节限位中点；幅值 30%；实机速度 cap 1.0 rad/s；软限位内缩 10%。

**回归（B）**：Phase 1 估计 BIP + **Fv** + **Fc**；**Jm** 留 Phase 2。

**构建环境**：所有编译与 launch 在 Docker 容器内执行（`./docker/container.sh enter` → `~/ros2_ws`）。

**Implications**：见 `.scratch/open-manipulator-x-sysid/PRD.md` 与 `docs/adr/0001-omx-sysid-approach.md`；可 `/to-issues` 拆 Phase 1 垂直切片。
