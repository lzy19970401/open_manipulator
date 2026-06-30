# OpenMANIPULATOR-X Pinocchio 任务空间阻抗控制（Phase 1）

OpenMANIPULATOR-X Phase 1 交付 **Task-space impedance control mode**：与 **Joint-space impedance control mode** **完全平行**的独立栈（`om_pinocchio_task_impedance_controller`、`open_manipulator_x_task_impedance_pinocchio`），与标准控制、GC 及关节阻抗 launch **互斥**。

**受控任务**：**End effector** 三维位置 only（不控制姿态）。**Impedance reference position** x_d 与 FK、雅可比均在 **`link1` 基座坐标系**（非 world，实机台架与 link1 重合时等价）。控制律 Phase 1 演进：v1 为 `τ = rnea(q,v,0) + J_p^T F`（G+C 前馈 + 笛卡尔弹簧-阻尼）；完整版叠加 **Task-space impedance dynamics feedforward**（由 x_d 插值得 ẍ_d，**ProxQP** 求 q̈_d 再 rnea）与 **Null-space posture preference**（q_capture 二级目标）。

**参考管理**：`reference_position_mode` 为 `capture_on_activate` | `fixed`；**Reference position interpolator** 对 x_d_active 做 per-axis 速率限制（默认 0.1 m/s）。Phase 1 不接外部 topic 更新 x_d_target。

**QP 选型**：Phase 1 采用 **ProxQP**（`proxsuite` dense QP）求解带 q̈ 盒约束的加权任务/零空间加速度；拒绝纯 DLS-only（用户要求约束 QP）及合并进关节阻抗包（launch 互斥语义模糊）。v1 tracer slice 可先不含 QP 调用，但 CMake/Docker 须预留 `proxsuite` 依赖。

**测试边界**：可测试数学集中于 **task-space impedance control law** 单缝；ROS 插件为薄适配层。CI 做离线 gtest；Gazebo/实机 hold-and-drag 为手动 smoke。

**Considered options:** 扩展现有关节阻抗插件（模式旗标污染互斥语义）；6D 位姿阻抗（4 DOF 欠驱动，延后）；qpOASES（ProxQP 打包受阻时的备选）。
