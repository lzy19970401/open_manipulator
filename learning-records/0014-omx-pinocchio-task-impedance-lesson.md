# 0014 — 任务空间阻抗控制律课（第 12 课）

## Date

2026-06-29

## Context

用户请求 `/teach`：详细讲解 `om_pinocchio_task_impedance_controller` 底层数学与代码对应，作为关节阻抗（第 11 课）的延伸。

## Learned

- 任务空间阻抗在 **link1 系**对 EE **3D 位置**施加 K_p/D_p，反馈经 **τ = J_pᵀ F** 进入关节力矩。
- **a_des = ẍ_d − J̇_p q̇** 来自对 ẋ = J_p q̇ 求导；用于 QP 主任务项 J_p q̈ ≈ a_des。
- **q̈_d** 由加权 QP + 盒约束求出，再 **rnea(q,v,q̈_d)** 前馈；零空间项 **q̈_null = K_null(q_capture − q)** 处理 4→3 冗余。
- ProxQP 的 **H = 2(JᵀWJ + W_null)**、**g = −2(JᵀWa + W_null q̈_null)** 与 `task_impedance_qp_solver.cpp` 一致。
- ROS 插件只做插值 x_d、读 HW、调 `compute_task_impedance_torques`；可测边界在 control law 模块。

## Artifacts

- `lessons/0012-omx-pinocchio-task-impedance.html`
- `reference/pinocchio-task-impedance.html`

## Next (ZPD)

- 在 Gazebo 手动拖动验证 F 方向与 hold 行为
- 调 w_task / w_null / K_null 观察零空间与主任务权衡
- 对照 `test_task_impedance_control_law.cpp` 八个 gtest 与推导

## Open questions

- LOCAL_WORLD_ALIGNED 雅可比与 link1 投影的数值细节（需时另开一小课）
