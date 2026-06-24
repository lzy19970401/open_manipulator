# 第 10 课：Pinocchio GC 控制器插件代码逻辑

Issue 01 已在仓库落地 `om_pinocchio_gravity_compensation_controller` 并行栈。本课把「第 3 课独立脚本算 g(q)」接到 ros2_control 四段生命周期（configure 建模型 → activate 绑接口 → update 算 G 写 effort → deactivate 清零），并强调与 KDL 版的差异：完整 q 含夹爪 mimic 镜像、无 leader/collision 订阅、专用 computeGeneralizedGravity。

**Implications**：MISSION 中「完整 ros2_control 插件」目标已在 Pinocchio 路径达成；下一课可教 Issue 02 摩擦叠加或实机 effort_sign 校准。用户若已掌握第 5 课 KDL 链路，本课是平行对照而非从零入门。
