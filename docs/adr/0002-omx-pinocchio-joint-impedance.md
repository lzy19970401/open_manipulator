# OpenMANIPULATOR-X Pinocchio 关节空间阻抗控制（Phase 1）

OpenMANIPULATOR-X Phase 1 采用 **关节空间阻抗**（非笛卡尔 6D），动力学后端为 Pinocchio **computed torque**（`rnea(q, v, q̈_d)`），叠加虚拟弹簧-阻尼 K_j/D_j 与可选 Fv/Fc 摩擦前馈（参数语义与 Pinocchio GC 相同）。控制器包、配置目录与 launch 与 `om_pinocchio_gravity_compensation_controller` **完全平行**（`om_pinocchio_impedance_controller`、`open_manipulator_x_impedance_pinocchio`），与标准控制及 GC launch 互斥。

参考构型 q_d 默认在 `on_activate` **捕获当前关节角**；yaml 可指定 **Fixed reference pose** 覆盖。Phase 1 不接外部 topic 更新 q_d，但内置 **Reference pose interpolator**（`q_d_active` 向 `q_d_target` 一阶限速，默认 0.5 rad/s/关节），Phase 2 仅需写入 `q_d_target`。Phase 1 交付实机 + Gazebo launch、离线单元测试（力矩计算校验）；Gazebo 仅手动 smoke。

**Considered options:** 笛卡尔阻抗（4 DOF 欠驱动，Phase 2）；仅 G(q) 或 G+C 前馈（手感不如完整 computed torque）；合并进 GC 包（launch/互斥语义模糊）。
