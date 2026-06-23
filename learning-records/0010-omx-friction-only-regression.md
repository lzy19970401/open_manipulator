# 第 8 课：URDF 惯性固定 + 摩擦-only 回归

用户走读当前 `regress.py`：τ_res = τ_meas − RNEA(URDF)，Y_fric 仅 8 列（Fv/Fc），有效秩 8/8；理解混合 Calibrated dynamics 路线与「URDF 错则摩擦背锅」风险。第 7 课（68 维联合回归）降为历史对照。

**Implications**：后续 Gazebo/bag 教学只需换 τ_meas 数据源，回归核不变；不必再教 BIP 秩亏除非用户要做惯性辨识。
