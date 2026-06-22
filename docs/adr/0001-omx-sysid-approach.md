# OpenMANIPULATOR-X 整机参数辨识采用独立包与 Gazebo-first 混合路线

Status: accepted

OpenMANIPULATOR-X 需要整机 **System identification** 能力（多频傅里叶 **Excitation trajectory**、完整拉格朗日模型 `M(q)q̈ + C(q,q̇)q̇ + G(q) + F(q̇) = τ`、**BIP** 回归），但不能污染现有 GC / bringup 配置，且实机激励有安全风险。

我们决定：在仓库根目录新增独立 ROS 2 包 `open_manipulator_sysid/`，Phase 1 用 **Python + Pinocchio**（非 Simulink）做离线回归；**Gazebo 先跑通**整条流水线后再上实机；实机采集复用现有 `open_manipulator_x_position` ros2_control，通过 sysid 包内自有 controller yaml 与 launch 录 **Identification dataset**，**不修改** `open_manipulator_bringup/config/` 下任何现有文件。Phase 1 回归 **BIP + Fv + Fc**；辨识结果 Phase 1 只读。Phase 2 的 **Calibrated dynamics model** 默认仍采用 **Nominal dynamics model**（URDF 惯性），仅将摩擦参数写入 `open_manipulator_sysid/config/calibrated_dynamics.yaml`。所有构建与运行均在项目 Docker 容器（`~/ros2_ws`）内进行。

## Considered Options

- **Simulink 全流程**：与参考文章一致，但与 ROS 2 栈割裂，且 CI/容器难以复现。
- **复用 GC effort launch 录数**：力矩接口现成，但 GC 语义是 g(q) 前馈，无法跟踪激励轨迹。
- **直接写回 GC yaml**：实现快，但违背「辨识实验与生产配置隔离」；已拒绝。
- **Phase 1 一次回归 BIP + Fv + Fc + Jm**：最完整，但 Jm 与连杆惯量强耦合，4 DOF 桌面臂激励不足；Jm 推迟 Phase 2。

## Consequences

- 新包须声明 Pinocchio / NumPy 依赖；容器内 `colcon build` 验证。
- 实机实验文档须强调人工在场、急停与速度/限位 abort 逻辑。
- GC 摩擦调参（单关节）与 sysid 摩擦结果可对照，但 Phase 2 前互不覆盖。
