# Issue 02 离线回归：Y·π=τ 与 4-DOF 可辨识性上限

第 7 课覆盖 `regress.py`：Pinocchio `computeJointTorqueRegressor` + 摩擦列拼接 → 合成 RNEA 数据集 → `lstsq` → 只读 yaml/HTML。关键非显然点：68 维 π 在 OMX 四轴臂上有效秩约 30，单个 BIP 估计可偏离 URDF，但 Fv/Fc 与残差仍可用于 Phase 1 验收；Pinocchio 4 用 `toDynamicParameters()` 替代已移除的 `standardParameters`。

**Implications**：下一课（Issue 03）只需替换 `generate_synthetic_dataset` 的数据源为 rosbag，回归核可复用；不必先教 SymPy BIP 重组。
