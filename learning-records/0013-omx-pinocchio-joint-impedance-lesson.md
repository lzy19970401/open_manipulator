# 关节空间阻抗：computed torque + 代码走读文档

用户要求用数学解释 Pinocchio 关节阻抗底层原理并附带代码逻辑，已整理为第 11 课（`lessons/0011-omx-pinocchio-joint-impedance.html`）与速查（`reference/pinocchio-joint-impedance.html`）。覆盖 M(q)q̈+C q̇+g 方程、虚拟 K/D、rnea 前馈、q_d 插值器与 GC 栈对照。

**Implications：** 下一课可练 K/D 实机调参，或对比 Hogan 任务空间阻抗；Phase 2 topic 更新 q_d 时可只讲插值器接线。
