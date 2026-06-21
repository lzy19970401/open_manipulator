# Mission: OpenMANIPULATOR 运动学 + Pinocchio 重力补偿

## Why

在 OpenMANIPULATOR-X 项目上工作（MoveIt、Task-space 控制、Commander 等），需要理解运动学几何，并**能用 Pinocchio 从 URDF 算出重力补偿力矩 g(q)**，替代或对照现有 KDL 实现（参考 [ROBOTIS open_manipulator_controls](https://github.com/ROBOTIS-GIT/open_manipulator_controls)）。

## Success looks like

- 给定四个关节角，能解释末端大致位置（FK 直觉）
- 理解 Wrench / 空间六维量与 τ = JᵀF 的静力关系
- **从 URDF 构建 Pinocchio Model，调用 `computeGeneralizedGravity` 得到 g(q)**
- **说清 ROBOTIS KDL 版与 Pinocchio 版的控制环差异（effort 接口、URDF 链、仿真前置条件）**
- 能在独立示例程序里验证 g(q) 数量级，并知道如何接到 ros2_control（概念层）

## Constraints

- **不修改 open_manipulator 仓库内的机器人包代码**
- 教学示例放在 `teaching-examples/`
- 以 OpenMANIPULATOR-X 为主；参考 ROBOTIS ROS1 仓库作对照

## Out of scope

- 完整 ros2_control 控制器插件实现（本课给架构与独立示例）
- 摩擦补偿、leader-follower 同步（见现有 `om_gravity_compensation_controller`）
- 其它型号独立推导
