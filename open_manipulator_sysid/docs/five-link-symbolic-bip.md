# 五连杆符号化 BIP（论文 Table I 风格）

只辨识 **link1–link5** 的惯性组合参数；**忽略** `gripper_left_link`、`gripper_right_link`、`end_effector_link` 的惯性列。夹爪质量已包含在 URDF 的 `link5` 中。

组合基由激励轨迹堆叠回归器的 **SVD** 导出（Swevers 风格线性组合），参考值 `θ_ref = T @ π_URDF,five_link`。

## 1. 从 URDF 导出最小参数集（BIP + Fv/Fc，功能 1）

```bash
ros2 run open_manipulator_sysid sysid_minimal_parameter_reference \
  --config $(ros2 pkg prefix open_manipulator_sysid)/share/open_manipulator_sysid/config/excitation.yaml \
  --xacro-mapping use_sim:=true \
  --output-yaml /workspace/sysid_results/minimal_parameter_reference.yaml
```

终端输出列：`i | Mathematical form | Physical meaning | Unit | θ (reference)`（含 8 个摩擦参数）。

仅 BIP 参考表（无摩擦）仍可用：

```bash
ros2 run open_manipulator_sysid sysid_five_link_reference \
  --config $(ros2 pkg prefix open_manipulator_sysid)/share/open_manipulator_sysid/config/excitation.yaml \
  --xacro-mapping use_sim:=true \
  --output-yaml /workspace/sysid_results/five_link_reference.yaml
```

终端输出列：`i | Physical meaning | Unit | θ (URDF reference)`。

## 2. GA 优化五阶傅里叶激励（功能 2）

在最小参数集（五连杆 SVD BIP + Fv/Fc）上最小化 `κ([Y_BIP | Y_Fv,Fc])`：

```bash
ros2 run open_manipulator_sysid excitation_ga_optimize \
  --config $(ros2 pkg prefix open_manipulator_sysid)/share/open_manipulator_sysid/config/excitation.yaml
```

目标满秩约为 **22 + 8 = 30**（link2–link5 回归列 + 摩擦；link1 为固定基座不在回归器列中）。迁移至 Fv/Fc 后须重跑 GA；`ga_metadata` 中旧的 `matrix_rank: 35`（Stribeck 时代）作废。

## 3. 从 bag 辨识并与参考对比

```bash
ros2 run open_manipulator_sysid sysid_five_link_identify_compare \
  --bag /workspace/sysid_results/bags/gazebo_YYYYMMDD_HHMMSS \
  --periods 5 \
  --config $(ros2 pkg prefix open_manipulator_sysid)/share/open_manipulator_sysid/config/excitation.yaml \
  --xacro-mapping use_sim:=true \
  --reference-yaml /workspace/sysid_results/five_link_reference.yaml \
  --output-dir /workspace/sysid_results/five_link_gazebo_YYYYMMDD \
  --output-csv /workspace/sysid_results/five_link_compare.csv
```

输出：

- Markdown 对比表（`θ_ref` vs `θ_id`）
- `five_link_identified.yaml`（含 `comparison` 块与 Fv/Fc 摩擦）
- 可选 CSV

## 与辨识/验证 CLI 的关系（ADR-0011 统一后）

`sysid_identify_bag`、`sysid_bip_urdf_compare` 与 `sysid_five_link_identify_compare` 共用 **FiveLinkBaseParameterSet**；`dynamics_identified.yaml` 写入 `five_link_base_parameters`（θ_ref、θ_id）。

| 项目 | 说明 |
|------|------|
| 惯性体 | link2–link5（link1 固定基座无回归列） |
| 物理含义 | SVD **Symbolic base parameter combination** |
| 参考值 | θ_ref = 组合矩阵 × URDF 标准列 |
| yaml 块 | `five_link_base_parameters` + `friction: {Fv, Fc}` |

力矩验收仍建议配合 `sysid_torque_compare_plot`。

详见 [ADR-0007](../../docs/adr/0007-omx-five-link-symbolic-bip.md)。
