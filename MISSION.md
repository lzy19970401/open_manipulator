# Mission: OpenMANIPULATOR 运动学 + Pinocchio 动力学控制

## Why

在 OpenMANIPULATOR-X 项目上工作（MoveIt、Task-space 控制、Commander 等），需要理解运动学几何，并**能用 Pinocchio 从 URDF 算出动力学前馈与柔顺控制**（重力补偿 g(q)、关节空间阻抗 computed torque），替代或对照现有 KDL 实现（参考 [ROBOTIS open_manipulator_controls](https://github.com/ROBOTIS-GIT/open_manipulator_controls)）。

## Success looks like

- 给定四个关节角，能解释末端大致位置（FK 直觉）
- 理解 Wrench / 空间六维量与 τ = JᵀF 的静力关系
- **从 URDF 构建 Pinocchio Model，调用 `computeGeneralizedGravity` 得到 g(q)** ✓（独立示例 + `om_pinocchio_gravity_compensation_controller` 插件）
- **说清 ROBOTIS KDL 版与 Pinocchio 版的控制环差异（effort 接口、URDF 链、仿真前置条件）** ✓（见第 5 课、第 10 课）
- 能在独立示例程序里验证 g(q) 数量级，并知道如何接到 ros2_control（概念层） ✓（第 10 课走读已落地插件）
- **写出关节阻抗控制律 τ = rnea(q,v,q̈_d) + K_jΔq + D_jΔq̇，并对照 `om_pinocchio_impedance_controller` 源码** ✓（见第 11 课）

## Constraints

- **不修改 open_manipulator 仓库内的机器人包代码**
- 教学示例放在 `teaching-examples/`
- 以 OpenMANIPULATOR-X 为主；参考 ROBOTIS ROS1 仓库作对照

## Out of scope

- 完整 ros2_control 控制器插件实现（KDL 栈已有；Pinocchio 并行栈见第 10 课 / Issue 01）
- 摩擦补偿、leader-follower 同步（见现有 `om_gravity_compensation_controller`）
- 其它型号独立推导
