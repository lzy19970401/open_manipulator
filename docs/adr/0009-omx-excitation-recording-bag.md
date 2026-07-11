# OMX 激励录包：单 bag 双 topic 记录指令与反馈

Status: accepted

**System identification** 与 **Model validation run** 的跟踪对比图原先离线重建 yaml 参考轨迹并对齐 `motion_start_s`，在 mock 下会出现反馈「超前」指令的假象（实测与控制器 desired 实际一致）。我们改为：**Excitation recording bag** 在单次 `ros2 bag record` 中同时录制 `/joint_states`（**Measured joint position**）与 `/arm_controller/controller_state` 的 `reference.positions`（**Commanded joint position**）。`sysid_trajectory_compare_plot` 以 measured 时间戳为主轴，将 commanded 插值到同轴绘图；辨识流水线仍只读 `/joint_states`。阶段背景默认关闭；旧单 topic bag 废弃重录，不提供 legacy 回退。

## Considered Options

- **两个独立 bag（feedback/ + commanded/）**：成对管理成本高，两进程启动时刻可能引入额外偏移。
- **离线 yaml 重建 + motion_start 对齐**：无需改录包，但对齐启发式不可靠（Mock ~2 s 相位差）。
- **辨识改用 commanded 作 q**：把跟踪误差混入回归，偏离 **Identification dataset** 语义。

## Consequences

- `excitation_*` 与 `test_trajectory_*` 四套 launch 的 `record_bag` 均录制上述两 topic。
- `excitation_runner` 停止录包时 `pkill` 匹配任意 `ros2 bag record`（单进程录双 topic）。
- 绘图 CLI 去掉 `--periods`/`--config` 主路径依赖；`--show-phases` 可选时仍用 yaml schedule。
- `bag_to_dataset` 与辨识 CLI 行为不变。
