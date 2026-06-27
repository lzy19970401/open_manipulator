# OpenMANIPULATOR-X — Pinocchio 补偿模式实机评测清单

Domain terms (**Compensation mode**, **Gravity residual**, **Dynamics residual**, etc.) are defined in [CONTEXT.md](../CONTEXT.md).

本文件是 `/grill-with-docs` 结论：在 **Hardware** 上对比三种 **Pinocchio gravity compensation control mode** 前馈组合（仅 G、G+C、G+C+F）的可重复实验与离线指标，不依赖主观手感。

**范围**：**Arm** 四关节（`joint1`–`joint4`）；**Gripper** 不参与。**Operation mode** 为实机 Dynamixel（非 Gazebo / Mock hardware）。

**互斥**：与 **Standard control launch** 及 KDL GC launch 不可同时运行；评测期间只启 `open_manipulator_x_pinocchio_gravity_compensation.launch.py`。

---

## 1. 设计决议（Grill 摘要）

| 决策 | 选择 | 理由 |
|------|------|------|
| 主指标 | **Dynamics residual** RMS + **Compensation wastage power** | 与三种前馈项一一对应；可排序 G / G+C / G+C+F |
| 静态段 | 6 个 **Validation pose**（含 SRDF **Named pose** + 工作空间补点） | 覆盖 URDF 重力方向变化；静态只看 **Gravity residual** |
| 动态段 | 标准化 **Backdrive trial**（固定路径 ×3 次/模式） | 贴近示教；科式/摩擦只在 \(\dot q \neq 0\) 时显现 |
| 恒速段 | 单关节 ±0.5 rad/s 扫掠（可选，推荐） | 分离 Fv/Fc 与科式耦合 |
| 力矩来源 | `/joint_states.effort` → N·m（与 sysid `bag_to_dataset` 相同换算） | GC `open_manipulator_x_current` 已暴露 effort 状态 |
| 模型力矩 | Pinocchio，与控制器同一分支逻辑 | 避免 KDL/Pinocchio 混比 |
| 重复次数 | 每模式动态 trial ×3，取逐指标中位数 | 降低人手零施力不可重复性 |
| 不采用 | 末端六维力传感器、Excitation trajectory 自动跟踪 | 实机 GC 模式无位置跟踪；人手拖动的可重复性足够做 A/B/C |

---

## 2. 三种 Compensation mode

通过 launch 参数切换（每次只改一项，重启 launch）：

| 模式 ID | `enable_coriolis_compensation` | `enable_friction_compensation` | 控制器 \(\tau_{\text{cmd}}\) 模型项 |
|---------|-------------------------------|--------------------------------|-------------------------------------|
| `G` | `false` | `false` | \(G(q)\) |
| `G+C` | `true` | `false` | `nonLinearEffects` → \(G + C\dot q\) |
| `G+C+F` | `true` | `true` | \(G + C\dot q + F_v\dot q + F_c\,\mathrm{sign}(\dot q)\) |

```bash
# 示例：仅重力
ros2 launch open_manipulator_bringup open_manipulator_x_pinocchio_gravity_compensation.launch.py \
  enable_coriolis_compensation:=false enable_friction_compensation:=false

# G+C+F（摩擦系数来自 open_manipulator_x_compensation_pinocchio yaml）
ros2 launch open_manipulator_bringup open_manipulator_x_pinocchio_gravity_compensation.launch.py \
  enable_coriolis_compensation:=true enable_friction_compensation:=true
```

**实验顺序建议**：同一操作者、同一天内完成 `G` → `G+C` → `G+C+F`，每种模式跑完全部 trial 再切换，避免 URDF/温度漂移。

---

## 3. Validation pose 列表（静态 **Static gravity validation**）

操作：用手将 **Arm** 带到目标构型附近，松手后在 GC 下微调至稳态；每个 pose **hold 8 s**（前 2 s 过渡，后 6 s 计入指标）。

| Pose ID | 名称 | joint1 | joint2 | joint3 | joint4 | 说明 |
|---------|------|--------|--------|--------|--------|------|
| P01 | `init` | 0 | 0 | 0 | 0 | SRDF **Init pose** |
| P02 | `home` | 0 | -1.0 | 0.7 | 0.3 | SRDF **Home pose** |
| P03 | `ready` | 0 | -1.0 | 1.0 | 0.0 | bringup **Ready pose** |
| P04 | `sysid_q0` | 0 | 0 | -0.05 | 0.135 | sysid 中性构型 |
| P05 | `reach_forward` | 0 | -0.5 | 0.8 | 0.0 | 前伸，joint2/3 中等负载 |
| P06 | `yaw_plus` | 1.2 | -0.8 | 0.3 | 0.5 | joint1 大角，检验耦合重力 |

Gripper：**Open**（`gripper_left_joint ≈ 0.019 m`），减小夹爪质量对 wrist 重力矩的干扰。

**稳态判据**（hold 窗口内逐样本检查，不满足则重录该 pose）：

- \(|\dot q_i| < 0.02\) rad/s（全部 Arm 关节）
- \(|\ddot q_i| < 0.5\) rad/s²（由离线重采样速度差分估计）

---

## 4. Backdrive trial 路径（动态段）

**Trial ID**：`BD01` — 固定关节空间路径，操作者尽量**零施力**拖动。

| 步骤 | 目标 pose | 建议用时 | 备注 |
|------|-----------|----------|------|
| 1 | P01 `init` | hold 3 s | 起点 |
| 2 | → P02 `home` | 8–12 s | 折叠 |
| 3 | hold P02 | 3 s | |
| 4 | → P03 `ready` | 8–12 s | 抬起 |
| 5 | hold P03 | 3 s | |
| 6 | → P05 `reach_forward` | 8–12 s | 前伸 |
| 7 | hold P05 | 3 s | |
| 8 | → P01 `init` | 8–12 s | 回起点 |

全程约 60–75 s。每种 **Compensation mode** 重复 **3 次**（`run01`–`run03`）。

**可选 Trial `CV01`（恒速扫掠）**：单关节主动匀速，其余关节随动保持自然。

| 关节 | 方向 | 目标 \(\|\dot q\|\) | 持续时间 |
|------|------|---------------------|----------|
| joint2 | + | 0.5 rad/s | 6 s |
| joint2 | − | 0.5 rad/s | 6 s |
| joint3 | + | 0.5 rad/s | 6 s |
| joint3 | − | 0.5 rad/s | 6 s |

恒速段判据：\(|\dot q_i| \in [0.35, 0.65]\) rad/s 的样本占比 > 60%。

---

## 5. Bag 录制规范

### 5.1 Topics

| Topic | 消息类型 | 必须 | 用途 |
|-------|----------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | **是** | `position`, `velocity`, `effort`（Arm 四关节） |
| `/clock` | `rosgraph_msgs/Clock` | 否 | 仅 sim；实机可不录 |

**不录** `/tf`（评测不需要）。若 `effort` 全零，检查 `joint_state_broadcaster` 是否在 effort 接口激活后启动；必要时用 `ros2 topic echo /joint_states` 预检。

### 5.2 命名与目录

```
results/comp_eval/<YYYYMMDD>/
  metadata.yaml                 # 全局：操作者、摩擦系数快照、torque_scaling_factors
  G/
    static/
      P01_init_run01/           # rosbag2 目录
      P02_home_run01/
      ...
    backdrive/
      BD01_run01/
      BD01_run02/
      BD01_run03/
    constant_velocity/          # 可选
      CV01_run01/
  G+C/
    ...
  G+C+F/
    ...
```

### 5.3 录制命令

GC launch 已运行且 controller active 后，**另开终端**：

```bash
ros2 bag record -o results/comp_eval/20250627/G/static/P01_init_run01 \
  /joint_states
```

- 存储：`mcap`（ROS 2 Jazzy 默认）或 `sqlite3` 均可；离线脚本与 sysid 一致。
- 静态 pose：hold 8 s → `Ctrl+C` 停录。
- 动态 trial：路径开始即录，回 P01 后 2 s 停录。

### 5.4 `metadata.yaml` 侧车（每个实验日一份）

```yaml
robot: open_manipulator_x
operation_mode: hardware
port_name: /dev/ttyUSB0
controller: pinocchio_gravity_compensation_controller
update_rate_hz: 100
torque_scaling_factors: [1.0, 0.8, 0.8, 0.8]
effort_sign_flips: [1.0, 1.0, 1.0, 1.0]
viscous_friction_coefficients: [0.49, 1.15, 0.85, 0.19]
coulomb_friction_coefficients: [0.0, 0.0, 0.0, 0.0]
friction_velocity_deadzone: 0.001
urdf_xacro: open_manipulator_x.urdf.xacro
urdf_mappings:
  ros2_control_type: open_manipulator_x_current
  use_mock_hardware: false
operator: <name>
notes: ""
```

---

## 6. 离线处理流程

```
rosbag2 目录
  → 读 /joint_states（复用 open_manipulator_sysid.bag_to_dataset.joint_states_from_bag）
  → effort LSB → N·m（arm_effort_array_to_nm）
  → 统一重采样 dt=0.01 s
  → Pinocchio 计算 τ_model（按 Compensation mode）
  → 指标聚合 → results/comp_eval/<date>/report.html
```

**依赖**：Docker 容器内 `ros-jazzy-pinocchio`、`open_manipulator_sysid`（至少 `bag_to_dataset` 的换算函数）。

---

## 7. Python 计算公式

以下与 `pinocchio_gravity_compensation_controller.cpp` 及 sysid 约定对齐。Arm 关节顺序：`joint1`…`joint4`。

### 7.1 符号

- \(q, \dot q, \ddot q \in \mathbb{R}^{n_v}\)：Pinocchio 广义坐标（含 mimic gripper 镜像，与控制器 `fill_configuration_vector` 一致）
- \(\tau_{\text{meas}} \in \mathbb{R}^4\)：实测关节力矩（N·m）
- \(s_i\)：`effort_sign_flips[i]`（评测 yaml 默认全 1）
- \(k_i\)：`torque_scaling_factors[i]`

### 7.2 模型力矩（与控制器前馈一致）

```python
import numpy as np
import pinocchio as pin

ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')
XM430_NM_PER_LSB = 0.00479627
MAX_REASONABLE_TORQUE_NM = 5.0


def effort_to_nm(effort: np.ndarray) -> np.ndarray:
    """Same rule as open_manipulator_sysid.bag_to_dataset.arm_effort_array_to_nm."""
    if effort.size == 0 or np.max(np.abs(effort)) <= MAX_REASONABLE_TORQUE_NM:
        return effort.astype(float)
    return effort.astype(float) * XM430_NM_PER_LSB


def sign_with_deadzone(velocity: float, deadzone: float) -> float:
    if abs(velocity) < deadzone:
        return 0.0
    return 1.0 if velocity > 0.0 else -1.0


def mirror_gripper_q(model, q: np.ndarray) -> None:
    if model.existJointName('gripper_left_joint') and model.existJointName('gripper_right_joint'):
        left = model.getJointId('gripper_left_joint')
        right = model.getJointId('gripper_right_joint')
        q[model.joints[right].idx_q()] = q[model.joints[left].idx_q()]


def stack_arm_state(model, q_arm, v_arm):
    q = pin.neutral(model)
    v = np.zeros(model.nv)
    for i, name in enumerate(ARM_JOINTS):
        jid = model.getJointId(name)
        q[model.joints[jid].idx_q()] = q_arm[i]
        v[model.joints[jid].idx_v()] = v_arm[i]
    mirror_gripper_q(model, q)
    return q, v


def tau_model_arm(
    model, data, q_arm, v_arm, mode: str, *,
    fv, fc, friction_deadzone, effort_sign_flips, torque_scaling_factors,
) -> np.ndarray:
    """
    mode: 'G' | 'G+C' | 'G+C+F'
    Returns 4-vector τ_model (what controller *intends* before hardware scaling is
    already in commanded effort — compare to τ_meas, not to τ_cmd sent to DXL).
    """
    q, v = stack_arm_state(model, q_arm, v_arm)
    if mode == 'G':
        pin.computeGeneralizedGravity(model, data, q)
        tau_phys = np.array([data.g[model.joints[model.getJointId(j)].idx_v()] for j in ARM_JOINTS])
    else:
        pin.nonLinearEffects(model, data, q, v)
        tau_phys = np.array([data.nle[model.joints[model.getJointId(j)].idx_v()] for j in ARM_JOINTS])

    if mode == 'G+C+F':
        for i in range(4):
            tau_phys[i] += fv[i] * v_arm[i] + fc[i] * sign_with_deadzone(v_arm[i], friction_deadzone)

    return effort_sign_flips * torque_scaling_factors * tau_phys
```

### 7.3 残差

**Gravity residual**（静态 hold 窗口，\( \tau_{\text{model}} = G(q)\) 仅）：

\[
\Delta\tau_g = \tau_{\text{meas}} - G(q)
\]

**Dynamics residual**（动态 trial，按当前 **Compensation mode** 的 \(\tau_{\text{model}}\)）：

\[
\Delta\tau = \tau_{\text{meas}} - \tau_{\text{model}}(\text{mode})
\]

```python
def gravity_residual(tau_meas, q_arm, model, data):
    tau_g = tau_model_arm(model, data, q_arm, np.zeros(4), 'G',
                          fv=np.zeros(4), fc=np.zeros(4), friction_deadzone=0.001,
                          effort_sign_flips=np.ones(4), torque_scaling_factors=np.ones(4))
    return tau_meas - tau_g


def dynamics_residual(tau_meas, q_arm, v_arm, mode, **model_kwargs):
    tau_m = tau_model_arm(model, data, q_arm, v_arm, mode, **model_kwargs)
    return tau_meas - tau_m
```

### 7.4 标量指标

对窗口内 \(N\) 个样本、4 关节：

| 指标 | 公式 | 用途 |
|------|------|------|
| `rms_gravity_residual` | \(\sqrt{\frac{1}{4N}\sum_{t,i}\Delta\tau_{g,i}^2(t)}\) | 静态：URDF 重力 + Fc 误差 |
| `rms_dynamics_residual` | \(\sqrt{\frac{1}{4N}\sum_{t,i}\Delta\tau_i^2(t)}\) | 动态：未补偿动力学 + 人手 |
| `p95_dynamics_residual` | \(\mathrm{percentile}_{95}(\|\Delta\tau\|_2)\) | 峰值拖曳感 |
| `wastage_power` | \(\frac{1}{T}\int_0^T \sum_i |\Delta\tau_i \dot q_i|\,dt\) | 补偿不足导致的「浪费」功率 |
| `corr_vel` | \(\mathrm{mean}_i\,\mathrm{corr}(\Delta\tau_i, \dot q_i)\) | G 模式应高；G+C+F 应低 |

```python
def rms_per_joint(delta_tau: np.ndarray) -> np.ndarray:
    """delta_tau shape (N, 4) → (4,) RMS per joint."""
    return np.sqrt(np.mean(delta_tau ** 2, axis=0))


def rms_norm(delta_tau: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.sum(delta_tau ** 2, axis=1))))


def wastage_power(delta_tau, velocity, dt: float) -> float:
    integrand = np.sum(np.abs(delta_tau * velocity), axis=1)
    return float(np.mean(integrand))  # time-average power (W)


def corr_with_velocity(delta_tau, velocity) -> np.ndarray:
    out = np.zeros(4)
    for i in range(4):
        if np.std(velocity[:, i]) < 1e-6:
            out[i] = np.nan
        else:
            out[i] = float(np.corrcoef(delta_tau[:, i], velocity[:, i])[0, 1])
    return out
```

### 7.5 跨 run 聚合

每种 mode、每种 trial 类型：

```python
def aggregate_runs(metric_values: list[float]) -> dict:
    arr = np.asarray(metric_values, dtype=float)
    return {
        'median': float(np.median(arr)),
        'mean': float(np.mean(arr)),
        'std': float(np.std(arr)),
    }
```

报告以 **median** 为主（3 次 backdrive 的中位数）。

### 7.6 静态窗口提取

对每个 static bag，丢弃前 2 s，后 6 s 计入：

```python
def static_hold_mask(times_s, t_start=2.0, t_end=8.0, vel_max=0.02, acc_max=0.5):
    t0 = times_s[0]
    in_time = (times_s >= t0 + t_start) & (times_s <= t0 + t_end)
    slow = np.all(np.abs(velocity) < vel_max, axis=1)
    # acceleration from np.gradient(velocity, times_s, axis=0)
    return in_time & slow
```

---

## 8. 结果判读（预期排序）

| 实验 | 指标 | G | G+C | G+C+F |
|------|------|---|-----|-------|
| Static hold | `rms_gravity_residual` | ≈ | ≈ | ≈（三者相同，只看 G） |
| Backdrive BD01 | `rms_dynamics_residual` | 高 | 中 | **低** |
| Backdrive BD01 | `wastage_power` | 高 | 中 | **低** |
| Backdrive BD01 | `mean(corr_vel)` | 高（正） | 中 | **≈ 0** |
| CV01 恒速段 | `rms_dynamics_residual` @ joint2/3 | 高 | 中 | **低** |

**通过条件（建议）**：

1. 静态：全部 pose 的 `rms_gravity_residual` < 0.3 N·m（逐关节）— 否则先修 URDF/零位，不比动态。
2. 动态：同一 trial 上 `G+C+F` 的 `rms_dynamics_residual` 与 `wastage_power` 均低于 `G` 和 `G+C`。
3. 若 `G+C` 不优于 `G`，检查 \(\dot q\) 是否足够大（backdrive 太慢则科式项小）。

---

## 9. 安全与前置检查

1. 急停可达；首次 trial 低速。
2. 录 bag 前：`ros2 topic hz /joint_states` ≥ 50 Hz；抽查 `effort` 非全零。
3. 退出 launch 后等待 shutdown 日志确认 Arm **Torque disable**（见 GC 文档）。
4. 与 **System identification** excitation 不同：本评测**不**发轨迹 goal，全靠手拖。

---

## 10. 与现有包的关系

| 组件 | 复用 |
|------|------|
| `bag_to_dataset.joint_states_from_bag` | 读 bag + effort→N·m |
| `bag_to_dataset.resample_uniform` | 0.01 s 网格 |
| `regress.load_pinocchio_model` | URDF 加载（可选） |
| `pinocchio_gravity_compensation_controller` | \(\tau_{\text{model}}\) 逻辑源 |

**不修改** `open_manipulator_bringup/config/` 下现有 yaml；摩擦/Fv 仅记录在 `metadata.yaml` 快照。

Phase 2 可实现 `open_manipulator_sysid` 内 CLI：`comp_eval_report --root results/comp_eval/<date>/`。

---

## 11. 最小实验集（时间紧时）

若只能做一轮：

1. 静态：P01、P02、P06 各 1 次（`G` 模式即可）。
2. 动态：`G` / `G+C` / `G+C+F` 各 1 次 BD01。
3. 只报 `rms_dynamics_residual` 与 `wastage_power` 三列对比。
