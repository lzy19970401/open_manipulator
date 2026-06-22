# OMX 重力补偿模式 — Grill 结论

`/grill-with-docs` 已把 OpenMANIPULATOR-X 重力补偿控制模式的设计树走通。用途：反驱动示教；实机+Gazebo 并行、实机验收；Phase 1 KDL 复用 `om_gravity_compensation_controller` 打通 effort launch，Phase 2 换 Pinocchio；仅 joint1–4；独立 launch；Phase 1 纯 g(q)，摩擦后调；不录轨迹；退出 torque disable。

**Implications**：可进入 `/to-prd`；Phase 1 工件：`open_manipulator_x_current.ros2_control.xacro`、GC yaml、独立 launch、Gazebo effort transmission。
