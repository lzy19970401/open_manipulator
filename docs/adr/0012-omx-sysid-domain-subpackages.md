# open_manipulator_sysid 域对齐子包布局

Status: accepted

`open_manipulator_sysid` Python 代码从 20+ 扁平顶层模块重组为与 CONTEXT.md 对齐的子包；`five_link_identification` 合并入 `five_link_dynamics/reference_tables.py`；实验性 figaroh 脚本与配置迁入 `_experimental/`。

## 子包

| 子包 | 域概念 |
|------|--------|
| `excitation_trajectory/` | Excitation trajectory |
| `excitation_recording/` | Excitation recording bag |
| `five_link_dynamics/` | Five-link dynamics model |
| `system_identification/` | System identification |
| `model_validation/` | Model validation run |
| `pinocchio_support/` | Pinocchio 共享 seam |
| `reference/` | 参考参数表 CLI |
| `_experimental/` | 非生产实验（figaroh 等） |

## Consequences

- `setup.py` entry_points 指向新模块路径。
- `dynamic_base_parameters` → `five_link_dynamics/legacy_qr.py`（QR 诊断 only）。
- `effective_condition_number` 统一在 `pinocchio_support/numerics.py`。
- 旧顶层模块名（`bag_to_dataset`、`dynamics_identify` 等）不再作为文件存在；通过子包 `__init__.py` 或新 entry_points 访问。
