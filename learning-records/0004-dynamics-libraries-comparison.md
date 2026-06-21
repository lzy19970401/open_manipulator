# 动力学库总览

用户完成 Pinocchio 重力补偿课后，要求总结常见动力学库对比。已覆盖：解析动力学（Pinocchio/KDL/RBDL/RBDyn/Drake）vs 仿真（MuJoCo/Gazebo）vs 最优控制（Crocoddyl）。关键区分：g(q) 用解析库，接触 sim 用仿真器，二者互补。

**Implications**：选型决策树已写入第 4 课；若用户继续实践，优先 Pinocchio vs KDL 数值对照实验。
