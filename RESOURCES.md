# 机器人运动学 Resources

## Knowledge

- [Modern Robotics: Mechanics, Planning, and Control — Lynch & Park（免费预印本 PDF）](https://hades.mech.northwestern.edu/images/0/0c/MR-tablet-v2.pdf)
  Ch.4 FK、Ch.5 静力学/Wrench、Ch.6 IK、Ch.8 动力学。Use for: 运动学与空间向量主线。
- [Modern Robotics 官网](http://modernrobotics.org)
  视频与练习。Use for: 按章自学。
- [Coursera: Modern Robotics Course 2 — Robot Kinematics](https://www.coursera.org/learn/modernrobotics-course2)
  Ch.4–7 系统课。
- [Pinocchio: computeGeneralizedGravity API](https://docs.ros.org/en/humble/p/pinocchio/generated/function_namespacepinocchio_1a79736c0fe06fdd6fc64ab02be0503785.html)
  g(q) 官方定义；等价于 `rnea(model, data, q, 0, 0)`。Use for: 重力补偿核心调用。
- [Pinocchio Overview — URDF + RNEA](https://github.com/stack-of-tasks/pinocchio/blob/master/doc/Overview.md)
  从 URDF 建 Model。Use for: Pinocchio 环境搭建。
- [Pinocchio inverse-dynamics.cpp 样例](https://docs.ros.org/en/noetic/api/pinocchio/html/inverse-dynamics_8cpp_source.html)
  最小 RNEA 脚本。Use for: C++ 模板。
- [ROBOTIS open_manipulator_controls](https://github.com/ROBOTIS-GIT/open_manipulator_controls)
  ROS1 KDL 重力补偿参考：`GravityCompensationController::JntToGravity`。Use for: 控制环对照。
- [awesome-robotics-libraries COMPARISONS.md](https://github.com/jslee02/awesome-robotics-libraries/blob/main/COMPARISONS.md)
  Pinocchio/KDL/RBDL 功能矩阵。Use for: 动力学库横向对比。
- [IEEE RAS TC — Modeling Tools](https://tcoptrob.github.io/resources/modeling_tools/)
  优化与控制社区维护的建模工具列表。
- [Pinocchio vs RBDL 论文 (SII 2019)](https://homepages.laas.fr/ostasse/hugo/publication/int_conf/carpentier-sii-2019/)
  性能与设计模式对比。
- [RBDL GitHub](https://github.com/rbdl/rbdl)
  Featherstone 算法轻量实现；URDF addon。
- [Drake MultibodyPlant](https://drake.mit.edu/)
  系统级建模+仿真+优化；非轻量 g(q) 首选。
- [REP-103: Standard Units](https://www.ros.org/reps/rep-103.html)
  m、rad 约定。

## Wisdom (Communities)

- [ROS Discourse — MoveIt / manipulation](https://discourse.ros.org/c/moveit/)
- [r/robotics](https://reddit.com/r/robotics)

## Gaps

- OpenMANIPULATOR-X 四轴臂无公开 Pinocchio 专用教程；本课 `teaching-examples/` 填补。
- URDF 含 mimic 夹爪时 nq/nv 与 arm 四关节的映射需在实机上校准。
