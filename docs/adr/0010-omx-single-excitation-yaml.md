# OMX 系统辨识：单一 excitation.yaml

Status: accepted

系统辨识激励配置从 `excitation.yaml` + `excitation_ga.yaml` 双文件合并为 **`config/excitation.yaml` 单文件**：GA 优化结果（`ga_coefficients`、`ga_metadata`）写回同一文件。在线 `excitation_runner`、launch 与离线工具默认读该文件。`coefficients_source: ga` 与 `home` 段从主配置移除；运动仅围绕 **q₀** 与 GA 傅里叶段。GA 内部仍用解析 template 作种群种子（代码内 `_config_for_template_seed`，非用户配置）。**Model validation run** 保留独立 `test_trajectory.yaml`（`coefficients_source: template`）。

## Consequences

- `default_ga_config_path()` 为 `default_config_path()` 的别名。
- `excitation_ga_optimize` 默认 `--output` 等于 `--config`（原地更新）。
- 辨识与验证流水线（`dynamics_identify`、`model_evaluation`、`test_trajectory_*`）保留不变。
