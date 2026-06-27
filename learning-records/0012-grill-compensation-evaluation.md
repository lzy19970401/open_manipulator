# Pinocchio 补偿模式实机评测 — Grill 结论

`/grill-with-docs` 已把三种 **Compensation mode**（G / G+C / G+C+F）的实机对比实验设计走通。

**目标**：在 **Hardware** 上用可重复 bag + 离线指标排序三种前馈，替代纯手感。

**主指标**：**Dynamics residual** RMS、**Compensation wastage power**；静态段用 **Gravity residual**（与模式无关，诊断 URDF）。

**Pose 集**：6 个 **Validation pose**（`init` / `home` / `ready` + sysid q₀ + 两个工作空间补点）；Gripper **Open**。

**动态协议**：**Backdrive trial** `BD01`（init→home→ready→reach→init），每模式 ×3，取 median；可选 **CV01** 恒速扫掠 joint2/3。

**录 bag**：仅 `/joint_states`；effort 经 `0.00479627` N·m/LSB 换算（与 sysid 一致）。

**模型**：Pinocchio 分支与 `pinocchio_gravity_compensation_controller` 一致；不混 KDL。

**产出**：

- 操作清单：`docs/open-manipulator-x-compensation-evaluation.md`
- Pose/trial 配置：`open_manipulator_sysid/config/compensation_evaluation.yaml`
- 术语：`CONTEXT.md` 新增 Compensation mode / Dynamics residual / Backdrive trial / Compensation wastage power

**Implications**：Phase 2 可在 sysid 包加 `comp_eval_report` CLI；不修改 bringup yaml。
