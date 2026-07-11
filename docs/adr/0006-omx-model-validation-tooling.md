# OMX 模型验证离线工具链

Status: accepted

辨识产出 **Calibrated dynamics model**（BIP + Stribeck）后，需要可重复的 **Model validation run** 来对照 **Nominal dynamics model** 并可视化力矩拟合。我们决定在 `open_manipulator_sysid` 增加三条独立接缝，不修改 bringup/GC 默认配置：

1. **Test trajectory launch**（`test_trajectory_{gazebo,hardware}.launch.py` + `config/test_trajectory.yaml`）：较短、可复现的激励录 bag。
2. **`sysid_bip_urdf_compare`**：输入 bag → 辨识或加载 yaml → 输出 BIP 与 URDF 同列名义值对比表。
3. **`sysid_torque_compare_plot`**：输入 bag + 标定模型 → 四关节测量 vs 预测力矩 PNG。

共享逻辑放在 `model_evaluation.py`；对比表默认 stdout Markdown，可选 CSV；力矩图依赖 `python3-matplotlib`。

## Considered Options

- **复用完整 sysid excitation 做验证**：周期过长，与「快速验收」目标不符。
- **把对比/绘图并入 `dynamics_identify`**：耦合辨识与验收，不利于「已有 yaml + 新 bag」交叉检查。
- **仅文档描述手工 Jupyter**：不可复现，偏离 ROS 包 CLI 惯例。

## Consequences

- 验证默认 `excitation_periods:=2`，与 `test_trajectory.yaml` 的 `discard_periods: 1` 对齐。
- BIP 对比按 Pinocchio regressor 列索引取 URDF 名义值；**可辨识子集的逐列数值不必接近 URDF**（冗余惯性参数存在等价类）。验收应看力矩 RMSE（`sysid_bip_urdf_compare` 现已打印），而非相对误差百分比。
- bag trim 窗口以 motion t=0（bag 起点）为参考，`identification_end_s` 为绝对上界（非从 bag 末尾回退），避免录制超时后混入回程段。
- 未提供自动 Fv/Fc 映射；下游 GC 仍消费线性摩擦，与 ADR-0004 一致。
