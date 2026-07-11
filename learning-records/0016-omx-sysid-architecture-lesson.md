# 第 14 课：sysid 域子包架构与三条流水线

用户需要能独立导航 `open_manipulator_sysid` 重组后的子包结构，并知道 Docker/Pinocchio 依赖下从 GA → 录包 → 辨识 → 验证的命令顺序。已产出 `lessons/0014-omx-sysid-architecture-and-usage.html` 与 `reference/sysid-architecture.html`；后续课可默认用户已读过 ADR-0012 子包图，不再重复 flat 模块历史。

**Implications**：教 sysid 时以 CONTEXT 域术语指路径（如「Excitation recording bag → `excitation_recording/bag_reader.py`」）；第 13 课中 `excitation_ga.yaml` 等旧路径需在复习时口头更正为单文件 `excitation.yaml` + 子包路径。
