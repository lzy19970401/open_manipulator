# 联合辨识可辨识基参数与完整 Stribeck 摩擦

Status: accepted

Phase 1 原先固定 URDF 惯性、仅线性估计 Fv，且 GA 条件数基于 60+16 列满 Pinocchio 回归器，秩亏导致 κ(W) 虚高（~10⁴），与 Swevers 文献中「最小参数集」口径不一致。

我们决定：

1. 对 **Excitation trajectory** 堆叠 `Y(q,q̇,q̈)`，QR 列枢轴提取 **Identifiable base parameter set**（`Y_base`），仅保留线性独立列；τ = Y_base π_base，不可辨识列在 reachable 运动下不影响力矩。
2. 离线辨识默认 **联合** 估计 π_base 与 **Stribeck friction model** 全部 7×4 参数（含 v_s、δ、α），使用 `scipy.optimize.least_squares`（TRF + 盒约束）。
3. GA 激励优化目标改为最小化 `κ([Y_base | Y_Stribeck_linear])`，其中 Stribeck 形状在 GA 阶段仍用配置初值（线性回归块），与离线非线性辨识分工。

## Considered Options

- **继续固定 URDF + 仅 f_c/f_s/f_v/f_b**：实现简单，但无法标定 Stribeck 形状，且 URDF 误差被摩擦吸收。
- **一次回归 60 列 Pinocchio + 16 列 Stribeck**：列数远大于秩，κ 与最小二乘均不可靠。
- **仅辨识 BIP 符号组合（Swevers 手工 base）**：更贴文献，但 OMX+夹爪 Pinocchio 列映射维护成本高；QR 列选自动化程度更高。

## Consequences

- `dynamics_identified.yaml` 存 `base_parameter_set.column_indices` 与 π_base，而非 per-link 10 参数向量。
- 下游 **Calibrated dynamics model** 须用 Y_base + Stribeck 公式算 τ，不能假设 URDF π 仍有效。
- v_s/δ/α 非线性，需足够速度激励；GA 与多周期 bag 裁剪策略保持不变。
