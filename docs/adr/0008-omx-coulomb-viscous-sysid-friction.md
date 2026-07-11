# 系统辨识采用库伦+粘滞摩擦（Fv/Fc），弃用 Stribeck

Status: accepted (supersedes ADR-0004 for friction parameterization)

OMX 控制栈（Pinocchio GC / 阻抗）已统一使用每关节 **Fv、Fc** 线性摩擦前馈。ADR-0004 的完整 Stribeck 七参数/关节模型使 GA 观测矩阵增至 16 列摩擦块（rank 35），与补偿接口不一致，且 v_s/δ/α 在激励设计阶段固定、辨识收益有限。

我们决定：

1. **System identification** 摩擦项改为 **τ_f,j = Fv_j·q̇_j + Fc_j·sign(q̇_j)**，共 8 个线性参数。
2. GA 最小化 **κ([Y_BIP | Y_Fv,Fc])**，目标满秩 **19 + 8 = 27**（五连杆符号 BIP + 摩擦）。
3. 离线辨识与 `dynamics_identified.yaml` 输出 `friction: {Fv, Fc}`，不再写入 `stribeck_friction`。

## Considered Options

- **保留 Stribeck**：可拟合低速非线性，但与 GC/阻抗 Fv/Fc 接口割裂，需额外映射。
- **仅 Fv**：无法解释静摩擦平台，静态 hold 残差解释力弱。
- **Fv + Fc（采纳）**：与补偿模式一致，回归块全线性，秩与条件数更可解释。

## Consequences

- 须重新运行 `excitation_ga_optimize`；旧 `ga_metadata.matrix_rank: 35` 作废。
- `stribeck_friction.py` 已删除；`dynamics_identified.yaml` 仅接受 `five_link_base_parameters` + `friction: {Fv, Fc}`。
