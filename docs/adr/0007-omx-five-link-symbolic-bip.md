# Five-link symbolic BIP identification

Status: accepted

辨识与论文 Table I 对齐时，需要 **五连杆（link1–link5）** 的符号化最小惯性参数集，并排除夹爪与 `end_effector_link` 的惯性列。我们决定：

1. 在 Pinocchio 满回归器上只保留 `link1`–`link5` 的 50 个标准惯性列；`gripper_*` 与 `end_effector_link` 不参与 BIP。
2. 对激励轨迹堆叠的 `Y` 做 SVD，取显著奇异值对应的行 `Vh` 作为 **线性组合矩阵**；符号表达式为各标准列的加权和（Swevers 风格），参考值 `θ_ref = T @ π_std_URDF`。
3. bag 离线辨识在同一组合基上联合估计 **Fv/Fc** 摩擦；输出 `five_link_reference.yaml` 与 `five_link_identified.yaml` 及对比表。

## Considered Options

- **继续 QR 单列选参（ADR-0004）**：实现简单，但「物理含义」列是 `joint2.Ixx` 而非论文组合式，且仍可能选中夹爪列。
- **手工推导 OMX 符号 BIP**：最贴文献，维护成本随 URDF 变更陡增。
- **FIGAROH 外部包**：符号化能力强，与当前 Stribeck 栈耦合与 Docker 依赖成本高。

## Consequences

- 新 CLI：`sysid_five_link_reference`、`sysid_five_link_identify_compare`；与 `sysid_identify_bag` 共用 **FiveLinkBaseParameterSet**（2026-07 统一，见 ADR-0011）。
- 组合矩阵依赖所选 `excitation_config` 轨迹；参考表与 bag 辨识应使用同一 config（默认 `excitation.yaml`）。
- 对比表逐行比较的是 **同一组合基** 下的 `θ_ref` 与 `θ_id`；比旧版「单列 vs URDF」更有意义，但仍以力矩残差为最终验收。
