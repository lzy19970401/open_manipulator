# OpenMANIPULATOR-X — Gravity Compensation Control Mode

Domain terms (**Gravity compensation control mode**, **Standard control launch**, **Arm**, **Gripper**, **Torque enable**, etc.) are defined in [CONTEXT.md](../CONTEXT.md) at the repository root.

## Overview

**Gravity compensation control mode** lets you backdrive the **Arm** (`joint1`–`joint4`) by hand for teaching preparation. The **gravity compensation controller** computes τ ≈ g(q) via KDL and feeds it to the **Arm** effort interfaces so the chain feels approximately neutral under gravity.

**Phase 1 scope:**

- Backdrivable teaching only — **no** trajectory recording or replay.
- **Arm** only — **Gripper** is not gravity-compensated.
- Do **not** run MoveIt, Commander, or **Standard control launch** on the same **Arm** concurrently.

**Phase 1.5 (current default in GC yaml):** kinetic/static friction compensation enabled on **Arm** four joints for improved backdrive feel. Gravity term remains KDL g(q); leader/collision features stay off. See [Friction tuning (Phase 1.5)](#friction-tuning-phase-15).

**Phase 2 (planned, not current procedure):** replace the KDL dynamics backend with Pinocchio while keeping the same launch and ros2_control contracts. See [Issue 06](../.scratch/open-manipulator-x-gravity-compensation/issues/06-pinocchio-backend-phase2.md).

### Behavioral prior art

This mode follows the same intent as the ROBOTIS ROS 1 reference:

- [ROBOTIS-GIT/open_manipulator_controls](https://github.com/ROBOTIS-GIT/open_manipulator_controls) — `gravity_compensation_controller.launch` (sim: effort transmission + current motor mode; hardware: `sim:=false`).
- [OpenMANIPULATOR-X controller experiment — Gravity Compensation](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_controller_experiment/#gravity-compensation) (e-manual).

The ROS 2 implementation in this repository uses `om_gravity_compensation_controller` and dedicated OpenMANIPULATOR-X bringup launches instead of the legacy ROS 1 stack.

## When to use which launch

| Goal | Launch | ros2_control / controllers |
|------|--------|----------------------------|
| MoveIt, trajectories, position control | **Standard control launch** | `open_manipulator_x_position`; `arm_controller` + `gripper_controller` |
| Hand backdrive / gravity-compensated teaching | **Gravity compensation control mode** | `open_manipulator_x_current`; `gravity_compensation_controller` + `joint_state_broadcaster` |

**Mutual exclusion:** **Standard control launch** and **Gravity compensation control mode** must **never** run at the same time. They claim different command interfaces on the same **Arm** joints (position vs effort). Starting both leads to controller conflicts and unsafe actuator behavior.

| Operation mode | Standard control | GC mode |
|----------------|------------------|---------|
| **Hardware** | `open_manipulator_x.launch.py` | `open_manipulator_x_gravity_compensation.launch.py` |
| **Simulation** | `open_manipulator_x_gazebo.launch.py` | `open_manipulator_x_gravity_compensation_gazebo.launch.py` |

## Launch commands

Build and source the workspace first (`colcon build`, `source install/setup.bash`).

### Hardware (real Dynamixel **Hardware**)

```bash
ros2 launch open_manipulator_bringup open_manipulator_x_gravity_compensation.launch.py
```

Optional arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `port_name` | `/dev/ttyUSB0` | USB serial device for the OpenCR / U2D2 |
| `use_mock_hardware` | `false` | Mock hardware for CI smoke (no physical robot) |
| `prefix` | `""` | Joint/link name prefix |

Example with a non-default port:

```bash
ros2 launch open_manipulator_bringup open_manipulator_x_gravity_compensation.launch.py port_name:=/dev/ttyUSB1
```

Mock hardware smoke (automated / no robot):

```bash
ros2 launch open_manipulator_bringup open_manipulator_x_gravity_compensation.launch.py use_mock_hardware:=true
```

### Simulation (Gazebo)

```bash
ros2 launch open_manipulator_bringup open_manipulator_x_gravity_compensation_gazebo.launch.py
```

Uses the same GC controller yaml and effort-capable ros2_control variant as hardware, with `use_effort_transmission:=true` for **Arm** joints in Gazebo.

## Prerequisites

### Hardware

1. **USB connection:** OpenMANIPULATOR-X connected via U2D2 (or equivalent); default device `/dev/ttyUSB0`. Install udev rules if needed:

   ```bash
   ros2 run open_manipulator_bringup om_create_udev_rules
   ```

2. **Dynamixel operating modes (automatic via launch):** GC mode loads `open_manipulator_x_current` ros2_control, which configures:
   - **Arm** actuators (Dynamixel IDs **11–14**): **Operating Mode 0** (current/torque control), **Goal Current** command interface, **Torque enable** set at init.
   - **Master finger** (ID **15**): unchanged from **Standard control launch** — **Operating Mode 5** (current-based position control).

   You do **not** need to hand-set modes in Dynamixel Wizard each time if you always enter GC through this launch (contrast with the legacy e-manual flow).

3. **Torque enable implications:** While GC mode is active, **Arm** actuators hold **Torque enable** so current commands take effect. On exit, torque is disabled on IDs 11–14 (see [Shutdown safety](#shutdown-safety)).

4. **Physical setup:** Clear workspace; support the **Arm** if testing near singularities. No MoveIt / Commander on the same **Arm** during GC mode.

### Simulation

Per the ROBOTIS reference, Gazebo GC requires **effort transmission** on **Arm** joints (not position-only):

- GC Gazebo launch sets `use_effort_transmission:=true` in `open_manipulator_x.urdf.xacro`.
- **Arm** joints use `EffortJointInterface`; **Gripper** remains position transmission.
- Standard Gazebo launch keeps position transmission — another reason the two simulation launches are mutually exclusive.

Verify `/joint_states` updates and controllers reach active state before relying on sim backdrive feel (sim dynamics differ from hardware).

## What runs in GC mode

| Component | Role |
|-----------|------|
| `ros2_control_node` + `open_manipulator_x_current` | Effort command on **Arm**; hardware or mock |
| `gravity_compensation_controller` | KDL g(q) on `joint1`–`joint4` only |
| `joint_state_broadcaster` | Publishes `/joint_states` |
| `robot_state_publisher` | TF from `robot_description` |
| `gc_shutdown_torque_disable` | Hardware only: **Torque disable** on **Arm** at shutdown |

Config: `open_manipulator_bringup/config/open_manipulator_x_gc/hardware_controller_manager.yaml`.

**Not spawned:** `arm_controller`, `gripper_controller`, MoveIt, Commander, RViz (unless you add them manually — not supported for **Arm** during GC).

## Shutdown safety

When **Gravity compensation control mode** stops (Ctrl+C, controller deactivate, or launch exit):

1. **Effort zero:** `gravity_compensation_controller` deactivates and zeros **Arm** effort commands (existing controller behavior).
2. **Torque disable:** `gc_shutdown_torque_disable` calls `dynamixel_hardware_interface/set_dxl_data` to set **Torque Enable = 0** on **Arm** Dynamixel IDs **11–14**.

**Operator expectation:** After stopping the launch, **wait until shutdown completes** (log lines confirming torque disabled on IDs 11–14) before manually moving the **Arm**. Joints should then move freely with no residual holding torque.

Gazebo GC launch does not run the torque-disable node (sim has no Dynamixel torque register).

Implementation: `open_manipulator_bringup/open_manipulator_bringup/gc_shutdown_torque_disable.py`.

## Manual acceptance checklist (hardware)

Use this after integration changes or before relying on GC mode for teaching.

**Launch**

- [ ] GC hardware launch starts with no controller spawn errors.
- [ ] `ros2 control list_controllers` shows `gravity_compensation_controller` **active**.
- [ ] `/joint_states` publishes for `joint1`–`joint4` (and gripper joints if present).

**Mutual exclusion**

- [ ] **Standard control launch** is **not** running (no `arm_controller` on the same robot).
- [ ] MoveIt / Commander are **not** commanding the same **Arm**.

**Backdrive at named poses**

Manually place or hold the **Arm** near each pose, then try slow hand guidance on each joint.

| Pose | Arm joints (rad) | Checks |
|------|------------------|--------|
| **Home pose** | j1=0, j2=-1, j3=0.7, j4=0.3 | No violent oscillation; no obvious sag under gravity; each joint backdrivable |
| **Ready pose** | j1=0, j2=-1, j3=1, j4=0 | Same as above |

- [ ] No runaway or self-excitation when releasing after a slow push.
- [ ] Subjective “neutral under gravity” — acceptable for Phase 1 pure g(q) without friction tuning.

**Phase 1.5 friction (after pure g(q) baseline passes)**

- [ ] Slow backdrive at **Home** / **Ready** feels smoother than friction-off baseline (less “sticky” when moving, less catch at rest).
- [ ] No new runaway or sustained oscillation vs Phase 1 baseline.
- [ ] Record subjective result in [Issue 05](../.scratch/open-manipulator-x-gravity-compensation/issues/05-friction-tuning-optional.md) operator comment.

**Shutdown**

- [ ] Stop launch with Ctrl+C; wait for shutdown logs.
- [ ] **Arm** is soft — no holding torque; joints move freely by hand.

**Optional: Simulation smoke**

- [ ] Gazebo GC launch activates controllers without fault.
- [ ] `/joint_states` updates in sim.

## Friction tuning (Phase 1.5)

After Phase 1 pure g(q) passes hardware acceptance, GC yaml enables friction compensation on **Arm** joints only. Leader/collision paths in `om_gravity_compensation_controller` remain unused (no follower topics required).

Config file: `open_manipulator_bringup/config/open_manipulator_x_gc/hardware_controller_manager.yaml`.

### Current tuned starting set (OMX Arm, four joints)

| Parameter | joint1 | joint2 | joint3 | joint4 | Notes |
|-----------|--------|--------|--------|--------|-------|
| `kinetic_friction_scalars` | 0.0005 | 0.40 | 0.40 | 0.10 | Pattern from `omx_l_leader_ai` / `omy_l100_leader_ai`, scaled for OMX `torque_scaling_factors` ≈ 1.0 / 0.8 |
| `static_friction_scalars` | 0.02 | 0.03 | 0.03 | 0.02 | Dithering near zero velocity |
| `static_friction_velocity_thresholds` (rad/s) | 0.05 | 0.05 | 0.05 | 0.05 | Applied to scaled velocity |
| `friction_compensation_velocity_thresholds` | 100.0 | 3.5 | 3.5 | 3.5 | Kinetic fade-out; high threshold on base |
| `input_velocity_scaling_factors` | 1.0 | 0.1 | 0.1 | 0.1 | **Must be non-zero** for friction; small values limit Coriolis in KDL |
| `kinetic_friction_torque_scalars` | 0 | 0 | 0 | 0 | Off for simpler tuning |
| `input_acceleration_scaling_factors` | 0 | 0 | 0 | 0 | Keeps KDL q̈ = 0 (no accel feedforward) |
| `torque_scaling_factors` | 1.0 | 0.8 | 0.8 | 0.8 | Unchanged from Phase 1 gravity trim |

Friction adds to KDL torques **before** `torque_scaling_factors`. Do not change leader/collision parameters.

### Retest procedure

1. **Baseline (optional A/B):** temporarily set all `kinetic_friction_scalars` and `static_friction_scalars` to `0.0`, rebuild/install bringup, launch GC hardware mode, backdrive at **Home** and **Ready** — note sticky joints.
2. **Friction on:** restore Phase 1.5 yaml values, `colcon build --packages-select open_manipulator_bringup`, source install, relaunch GC mode.
3. **Feel check:** slow hand guidance on each joint at **Home** (j1=0, j2=-1, j3=0.7, j4=0.3) and **Ready** (j1=0, j2=-1, j3=1, j4=0). Target: less resistance when moving, less “stick-slip” near rest, no violent oscillation when released.
4. **Shutdown safety (Issue 03):** Ctrl+C launch; confirm torque disabled on IDs 11–14; **Arm** moves freely — friction changes must not alter shutdown behavior.
5. **Record:** note per-joint adjustments and subjective improvement in Issue 05 operator comment.

### Tuning guide

| Symptom | Direction |
|---------|-----------|
| Heavy drag when moving a joint | Increase that joint’s `kinetic_friction_scalars` (~10–20% steps) |
| Catch / stick-slip near rest | Increase `static_friction_scalars` slightly (≤ 0.05) |
| Oscillation or hunting after release | Decrease kinetic/static on that joint, or lower `input_velocity_scaling_factors` |
| Base (joint1) over-compensated when rotating | Lower joint1 `kinetic_friction_scalars` (already small) |

Rebuild `open_manipulator_bringup` after yaml edits; no controller code changes required for Phase 1.5.

## Troubleshooting

| Symptom | Likely cause |
|---------|----------------|
| `Overrun might occur` (~5 ms read at 200 Hz) | USB 1 Mbps Dynamixel read exceeds 5 ms budget — GC yaml uses **100 Hz** (same as `omx_l_leader_ai`) |
| `/joint_states` effort always ~0 on Arm | N·m command was truncated to Goal Current raw=0 before `[unit info]` mapping (IDs 11–14; XM430-W350-R Kt=1.783 N·m/A) |
| Arm over/under-compensated after unit fix | Wrong Kt in `[unit info]` multiplier — OMX Arm uses XM430-W350-R (1.783 N·m/A), not XL430 |
| Controller fails to activate | Standard launch or another process still claiming **Arm** interfaces |
| Arm fights gravity heavily | Wrong launch (position mode); or not at expected pose / sign issue — re-check GC launch and yaml |
| Backdrive feels sticky / notchy | Phase 1.5: increase friction scalars per joint; see [Friction tuning](#friction-tuning-phase-15) |
| Oscillation after friction enable | Reduce `kinetic_friction_scalars` or `input_velocity_scaling_factors` on affected joint |
| Arm stiff after exit | Shutdown not finished; torque still enabled — wait for disable logs or power-cycle as last resort |
| No `/joint_states` | Hardware port wrong (`port_name`); USB permissions / udev |
| Gazebo controller timeout | Prior sim instance; retry after killing leftover `gz sim` processes |

## Related files

| Path | Purpose |
|------|---------|
| `open_manipulator_bringup/launch/open_manipulator_x_gravity_compensation.launch.py` | Hardware GC launch |
| `open_manipulator_bringup/launch/open_manipulator_x_gravity_compensation_gazebo.launch.py` | Gazebo GC launch |
| `open_manipulator_description/ros2_control/open_manipulator_x_current.ros2_control.xacro` | Effort + current-mode Dynamixel config |
| `open_manipulator_bringup/config/open_manipulator_x_gc/hardware_controller_manager.yaml` | GC controller parameters |
| `ros2_controller/om_gravity_compensation_controller/` | KDL gravity compensation plugin |

## Out of scope (Phase 1)

- Trajectory recording / replay
- Leader–follower or collision-flag features (OMY leader stack)
- Gripper gravity compensation
- Per-robot friction polish beyond the documented starting set ([Issue 05](../.scratch/open-manipulator-x-gravity-compensation/issues/05-friction-tuning-optional.md))
- Pinocchio backend (Phase 2)
