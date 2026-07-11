# 统一系统辨识 BIP 为五连杆 SVD

Status: accepted

GA 激励优化、`sysid_identify_bag`、`sysid_bip_urdf_compare` 与 `sysid_five_link_identify_compare` 曾使用两套 **Identifiable base parameter set** 口径（QR 单列 vs 五连杆 SVD 组合），导致 rank、对比表语义与 `ga_metadata` 不一致。

我们决定：

1. **FiveLinkBaseParameterSet** 为唯一 canonical BIP；`dynamics_identify` 输出 `five_link_base_parameters`（含 `reference_values`、`identified_values`）。
2. `model_evaluation` / `bip_urdf_compare` 对比 **Symbolic base parameter combination** 的 θ_ref 与 θ_id，不再逐 Pinocchio 列对 URDF。
3. `dynamic_base_parameters`（QR）仅保留诊断与单元测试，不参与生产辨识路径。
4. 删除 `stribeck_friction.py`；摩擦仅 **Coulomb/viscous friction model**（ADR-0008）。

## Consequences

- 旧 `dynamics_identified.yaml`（`base_parameters` QR 块）须重跑辨识。
- ADR-0007「QR 仍可用于力矩验收」废止。
