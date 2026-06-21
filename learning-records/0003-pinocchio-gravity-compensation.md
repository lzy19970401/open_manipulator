# Pinocchio 重力补偿专题

用户要求学习用 Pinocchio 实现 OpenMANIPULATOR 重力补偿，参考 ROBOTIS ROS1 `open_manipulator_controls`（KDL `JntToGravity`）。已提供 `teaching-examples/` 独立脚本与第 3 课 HTML。尚未在实机验证 g(q) 符号；下一课可对比 KDL 输出或接 ros2_control 概念。

**Evidence**：用户主动指定 Pinocchio + ROBOTIS 仓库；Mission 已更新。

**Implications**：需安装 ros-humble-pinocchio；OMX 默认 position 模式需 effort 配置才能应用 τ。
