# OMX 模型验证离线工具

在 **Model validation run**（`test_trajectory_*.launch.py`）录 bag 之后，用两个独立 CLI 检查辨识质量。

## 1. 录测试轨迹 bag

**Gazebo**

```bash
ros2 launch open_manipulator_sysid test_trajectory_gazebo.launch.py
```

**实机 / mock**

```bash
ros2 launch open_manipulator_sysid test_trajectory_hardware.launch.py port_name:=/dev/ttyUSB0
```

默认：

- 配置：`config/test_trajectory.yaml`（template 系数，2 个傅里叶周期）
- bag：`/workspace/sysid_results/bags/test_gazebo_YYYYMMDD_HHMMSS/`

## 2. BIP vs URDF 对比表

从 bag 辨识 BIP（或加载已有 `dynamics_identified.yaml`），与 URDF 名义惯性参数对比：

```bash
ros2 run open_manipulator_sysid sysid_bip_urdf_compare \
  --bag /workspace/sysid_results/bags/test_gazebo_YYYYMMDD_HHMMSS

# 可选：复用已有辨识结果，跳过重新辨识
ros2 run open_manipulator_sysid sysid_bip_urdf_compare \
  --bag /workspace/sysid_results/bags/test_gazebo_YYYYMMDD_HHMMSS \
  --identified-yaml /workspace/sysid_results/dynamics_identified.yaml \
  --output-csv /workspace/sysid_results/bip_compare.csv
```

Gazebo bag 需匹配仿真 xacro：

```bash
ros2 run open_manipulator_sysid sysid_bip_urdf_compare \
  --bag <gazebo_bag> --xacro-mapping use_sim:=true
```

终端输出 Markdown 表格；`--output-csv` 可选。

**重要**：对比表为五连杆 **Symbolic base parameter combination** 的 θ_id vs θ_ref（同一组合基），不再逐 Pinocchio 单列对 URDF。CLI 会在表格前打印 **力矩 RMSE**（标定模型 vs 测量、θ_ref + 同摩擦 vs 测量），请以力矩拟合为准。

## 3. 测量力矩 vs 模型预测（四张图）

```bash
ros2 run open_manipulator_sysid sysid_torque_compare_plot \
  --bag /workspace/sysid_results/bags/test_gazebo_YYYYMMDD_HHMMSS \
  --output-dir /workspace/sysid_results/torque_plots

# 使用已有辨识 yaml
ros2 run open_manipulator_sysid sysid_torque_compare_plot \
  --bag <bag> \
  --identified-yaml /workspace/sysid_results/dynamics_identified.yaml
```

输出：`joint1_torque_compare.png` … `joint4_torque_compare.png`。

## 4. 指令 vs 反馈（关节位置，四张图）

从 **Excitation recording bag** 读取 `/arm_controller/controller_state` 的 `reference.positions`（**Commanded joint position**）与 `/joint_states`（**Measured joint position**），在同一时间轴上对比：

```bash
ros2 run open_manipulator_sysid sysid_trajectory_compare_plot \
  --bag /workspace/sysid_results/bags/gazebo_YYYYMMDD_HHMMSS \
  --output-dir /workspace/sysid_results/trajectory_plots/gazebo_YYYYMMDD_HHMMSS
```

输出：`joint1_trajectory_compare.png` … `joint4_trajectory_compare.png`（指令 vs 反馈位置），以及 `joint1_commanded_kinematics.png` … `joint4_commanded_kinematics.png`（每关节三行子图：给定位置、速度、加速度，来自 `controller_state.reference`）。阶段背景默认关闭；需要时可加 `--show-phases`。

旧版仅含 `/joint_states` 的 bag 不再支持，请用更新后的 `excitation_*` / `test_trajectory_*` launch 重录。

## 参数说明

| 参数 | 默认 | 含义 |
|------|------|------|
| `--show-phases` | off | 用 yaml schedule 叠加阶段背景 |
| `--skip-commanded-kinematics` | off | 跳过 `joint*_commanded_kinematics.png` |
| `--periods` | `3` | 仅 `--show-phases` 时使用 |
| `--config` | `excitation.yaml` | 仅 `--show-phases` 时使用 |

完整辨识（5 周期 GA 轨迹）仍用 `excitation_*.launch.py` + `sysid_identify_bag`；验证工具默认对齐 **test** 配置。

## 激励 bag（5 周期 GA）

`excitation_gazebo` 录制的 bag（前缀 `gazebo_`）需显式指定周期与配置：

```bash
ros2 run open_manipulator_sysid sysid_bip_urdf_compare \
  --bag /workspace/sysid_results/bags/gazebo_YYYYMMDD_HHMMSS \
  --periods 5 \
  --config $(ros2 pkg prefix open_manipulator_sysid)/share/open_manipulator_sysid/config/excitation.yaml \
  --xacro-mapping use_sim:=true \
  --identified-yaml /workspace/sysid_results/dynamics_identified.yaml \
  --output-csv /workspace/sysid_results/bip_compare.csv

ros2 run open_manipulator_sysid sysid_torque_compare_plot \
  --bag /workspace/sysid_results/bags/gazebo_YYYYMMDD_HHMMSS \
  --periods 5 \
  --config $(ros2 pkg prefix open_manipulator_sysid)/share/open_manipulator_sysid/config/excitation.yaml \
  --xacro-mapping use_sim:=true \
  --identified-yaml /workspace/sysid_results/dynamics_identified.yaml \
  --output-dir /workspace/sysid_results/torque_plots/gazebo_YYYYMMDD_HHMMSS
```

## 故障排查：`numpy.core.multiarray failed to import`

容器内若曾 `pip install figaroh`（或其它依赖），可能把 `/usr/local` 下的 numpy 升到 2.x，而 apt 的 `python3-scipy` 1.11 仍按 numpy 1.x 编译，导致 `sysid_*` CLI 在 `import scipy` 时崩溃。

快速修复（容器内）：

```bash
/root/ros2_ws/src/open_manipulator/docker/ensure_sysid_python_deps.sh
# 或：pip3 install 'numpy<2' --break-system-packages --force-reinstall
```

验证：

```bash
python3 -c "from scipy.linalg import qr; print('ok')"
cd ~/ros2_ws && pytest src/open_manipulator/open_manipulator_sysid/test/test_scipy_stack.py -q
```

重建镜像时 Dockerfile 已包含 `python3-scipy` 与 `python3-matplotlib`；上述脚本用于**已有运行中容器**在 pip 污染后恢复。
