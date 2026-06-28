# open_manipulator_sysid

OpenMANIPULATOR-X **system identification** package: multi-frequency Fourier **excitation trajectory**, dataset recording, and offline **friction** regression with **fixed URDF inertia**.

> **Build and run only inside the project Docker container.** Do not run `colcon build` on the host — dependencies (ROS 2, Pinocchio) match the container image.

## Docker workflow

From the repository root on the host:

```bash
./docker/container.sh enter
```

Inside the container (`~/ros2_ws`, sources mounted from this repo):

```bash
cd ~/ros2_ws
colcon build --packages-select open_manipulator_sysid --symlink-install
source install/setup.bash
```

## Package layout

| Module | Role |
|--------|------|
| `excitation_trajectory.py` | Fourier trajectory + safety limits (pure math, no ROS) |
| `excitation_runner.py` | ROS node: send excitation goal to `arm_controller` |
| `bag_to_dataset.py` | Rosbag2 → uniform time series |
| `identification_dataset.py` | `IdentificationDataset`, resampling, validation |
| `torque_conversion.py` | Dynamixel Present Current LSB → N·m (live + offline) |
| `measured_torque.py` | ROS node: `/sysid/measured_joint_torque` |
| `hardware_torque_enable.py` | ROS node: one-shot torque enable |
| `regress.py` | Offline friction least squares |
| `comp_eval.py` | Compensation evaluation static gravity residual report |
| `robot_description.py` | URDF + sysid ros2_control for launch |

**Torque units:** use `torque_conversion` only — there is no separate `torque_sources` module.

## Phase 1 — Issue 01: excitation trajectory

### Configuration

`config/excitation.yaml` defines Phase 1 defaults:

- **Arm only**: `joint1`–`joint4` (no gripper)
- **q₀**: joint limit midpoints (`joint1=0`, `joint2=0`, `joint3=-0.05`, `joint4=0.135` rad)
- **Fourier**: N=5 harmonics, period T=10 s, amplitude 30% of each joint range
- **Safety**: soft limits (10% inset from hard limits), velocity cap 1.0 rad/s, acceleration cap 2.0 rad/s²

Hard limits are taken from `open_manipulator_description` URDF values embedded in the yaml.

### CLI

Print one period of joint positions (validates soft limits and velocity cap first):

```bash
ros2 run open_manipulator_sysid excitation_trajectory_print
```

Optional arguments:

```bash
ros2 run open_manipulator_sysid excitation_trajectory_print --samples 21
ros2 run open_manipulator_sysid excitation_trajectory_print --config /path/to/excitation.yaml
```

### Tests

```bash
cd ~/ros2_ws
colcon test --packages-select open_manipulator_sysid
colcon test-result --verbose
```

Or directly:

```bash
pytest src/open_manipulator/open_manipulator_sysid/test
```

## Phase 1 — Issue 02: offline synthetic regression

End-to-end tracer bullet (no Gazebo, no hardware): Fourier excitation → subtract **τ_dyn(URDF)** via RNEA → friction-only regressor (**Fv**/**Fc**) → least squares → read-only outputs under `results/`.

```bash
ros2 run open_manipulator_sysid sysid_regress_synthetic
```

Optional arguments:

```bash
ros2 run open_manipulator_sysid sysid_regress_synthetic --periods 3 --dt 0.01 --output-dir /path/to/results
```

Writes:

- `results/inertia_nominal.yaml` (URDF reference; not estimated)
- `results/friction_estimated.yaml`
- `results/report.html` (residual norm, effective condition number, friction nominal vs estimated)

Requires `ros-jazzy-pinocchio` in the container (`apt install ros-jazzy-pinocchio` if missing).

## Phase 1 — Issue 03: Gazebo record → regress

End-to-end **Simulation** pipeline: Gazebo position mode → excitation → rosbag → offline regression.

### Launch (container)

Build dependencies once (includes `open_manipulator_description`, Gazebo, controllers):

```bash
cd ~/ros2_ws
colcon build --packages-select open_manipulator_sysid open_manipulator_description open_manipulator_bringup --symlink-install
source install/setup.bash
```

Run excitation in Gazebo (records `/joint_states` to `results/bags/` by default, then shuts down):

```bash
ros2 launch open_manipulator_sysid excitation_gazebo.launch.py
```

Optional launch arguments:

```bash
ros2 launch open_manipulator_sysid excitation_gazebo.launch.py \
  excitation_periods:=1 \
  bag_output:=/root/ros2_ws/src/open_manipulator/open_manipulator_sysid/results/bags/my_run
```

Uses **sysid-only** `config/controller_manager_hardware.yaml` (no sim-only `gripper_right_joint`, 100 Hz update rate). Gazebo continues to use `config/controller_manager.yaml`. Does **not** modify bringup or GC yaml.

Excitation sequence (from `config/excitation.yaml`):

1. Slow quintic approach to **q₀** (`approach.duration_s`, default 5 s)
2. Stationary hold (`approach.hold_duration_s`, default 2 s)
3. One or more Fourier excitation periods on **joint1–joint4** only

Live safety: abort if soft limits or velocity cap are breached.

### Offline pipeline

Convert rosbag → uniform dataset (default trim skips approach+hold):

```bash
ros2 run open_manipulator_sysid bag_to_dataset --bag results/bags/<run_dir>
```

Run regression on bag or `.npz` (default robot model: `open_manipulator_x.urdf.xacro` expanded with the same args as hardware excitation launch; override with `--urdf` or `--xacro` / `--xacro-mapping`):

```bash
ros2 run open_manipulator_sysid sysid_regress_bag --bag results/bags/<run_dir>
# legacy static URDF:
ros2 run open_manipulator_sysid sysid_regress_bag --bag results/bags/<run_dir> --urdf /path/to/open_manipulator_x.urdf
# Gazebo bag — match excitation sim flags:
ros2 run open_manipulator_sysid sysid_regress_bag --bag results/bags/<gazebo_run> --xacro-mapping use_sim:=true
# or
ros2 run open_manipulator_sysid sysid_regress_bag --dataset results/datasets/<run_dir>.npz
```

Writes the same read-only outputs under `results/` (`inertia_nominal.yaml`, `friction_estimated.yaml`, `report.html`).

## Phase 1 — Issue 04: Hardware excitation → record → regress

**Prerequisite:** complete the Issue 03 Gazebo loop first (`excitation_gazebo.launch.py` → bag → `sysid_regress_bag` with finite friction estimates). Do not run hardware sysid until simulation validates the pipeline.

### Safety and exclusivity

- **Human operator required** at the robot during excitation; keep the e-stop within reach.
- **Mutually exclusive** with **Gravity compensation control mode** (`open_manipulator_x_gravity_compensation.launch.py`) and standard GC launch on the same **Arm** — both use ros2_control on the same joints; only one control mode may run at a time.
- Excitation aborts automatically when soft limits or the 1.0 rad/s velocity cap are exceeded (same as Gazebo).
- Sequence: slow approach to **q₀** → 2 s hold → Fourier excitation on **joint1–joint4** only.

### Torque source

Regression uses **τ** in this order:

1. `joint_states.effort` from `joint_state_broadcaster` when non-zero.
2. Fallback: `measured_torque_publisher` reads Dynamixel **Present Current** via `dynamixel_hardware_interface/dxl_state` (when `present_current` is available) or `get_dxl_data`, scaled by **0.00479627 N·m** per raw LSB (same factor as GC `open_manipulator_x_current` xacro).

Hardware bags record `/joint_states` and `/sysid/measured_joint_torque`. Offline tools auto-detect the measured-torque topic when bag effort is all zero.

### Launch (container, real hardware)

```bash
cd ~/ros2_ws
colcon build --packages-select open_manipulator_sysid open_manipulator_description --symlink-install
source install/setup.bash

ros2 launch open_manipulator_sysid excitation_hardware.launch.py port_name:=/dev/ttyUSB0
```

Optional arguments:

```bash
ros2 launch open_manipulator_sysid excitation_hardware.launch.py \
  port_name:=/dev/ttyUSB0 \
  excitation_periods:=1 \
  bag_output:=/root/ros2_ws/src/open_manipulator/open_manipulator_sysid/results/bags/my_hardware_run
```

Mock-hardware smoke (no Dynamixel USB):

```bash
ros2 launch open_manipulator_sysid excitation_hardware.launch.py \
  use_mock_hardware:=true mock_sensor_commands:=true
```

Uses **sysid-only** `config/controller_manager.yaml` with `open_manipulator_x_position` ros2_control. Does **not** modify `open_manipulator_bringup` or GC yaml.

### Offline pipeline (same as Gazebo)

```bash
ros2 run open_manipulator_sysid bag_to_dataset --bag results/bags/<hardware_run>
ros2 run open_manipulator_sysid sysid_regress_bag --bag results/bags/<hardware_run>
```

Hardware sign-off is manual: confirm finite BIP/Fv/Fc in `results/report.html` without NaN parameters.

## Compensation evaluation — Issue 01: static gravity residual report

Offline report for **Static gravity validation** holds recorded under one **Compensation mode** folder (`G`, `G+C`, or `G+C+F`). Computes **Gravity residual** RMS per validation pose using Pinocchio \(G(q)\) aligned with the Pinocchio GC controller (gripper mimic, effort sign flips, torque scaling from optional experiment `metadata.yaml`).

### Directory layout

```
results/comp_eval/<YYYYMMDD>/
  metadata.yaml              # optional friction/scaling snapshot
  G/
    static/
      P01_init_run01/        # rosbag2 directory
      P02_home_run01/
      ...
  report/                    # written by comp_eval_report
    metrics.yaml
    report.html
```

### CLI (container)

```bash
cd ~/ros2_ws
colcon build --packages-select open_manipulator_sysid --symlink-install
source install/setup.bash

ros2 run open_manipulator_sysid comp_eval_report \
  --root results/comp_eval/20250627 \
  --mode G
```

Optional arguments:

```bash
ros2 run open_manipulator_sysid comp_eval_report \
  --root results/comp_eval/20250627 \
  --mode G \
  --config /path/to/compensation_evaluation.yaml \
  --metadata /path/to/metadata.yaml \
  --output /path/to/report \
  --xacro-mapping ros2_control_type:=open_manipulator_x_current
```

Exits non-zero when no static bags are found, arm joints are missing from `/joint_states`, or effort is all zero after N·m conversion. Does **not** modify `open_manipulator_bringup` GC yaml.

Operator protocol: [docs/open-manipulator-x-compensation-evaluation.md](../docs/open-manipulator-x-compensation-evaluation.md).

## Package layout

```
open_manipulator_sysid/
├── config/
│   ├── excitation.yaml
│   ├── controller_manager.yaml
│   └── controller_manager_hardware.yaml
├── launch/
│   ├── excitation_gazebo.launch.py
│   └── excitation_hardware.launch.py
├── open_manipulator_sysid/
│   ├── excitation_trajectory.py
│   ├── excitation_runner.py
│   ├── measured_torque.py
│   ├── hardware_torque_enable.py
│   ├── robot_description.py
│   ├── bag_to_dataset.py
│   ├── comp_eval.py
│   └── regress.py
├── test/
└── results/                 # gitignored identification outputs
```

## Isolation

This package does **not** modify `open_manipulator_bringup` or GC controller yaml. See `docs/adr/0001-omx-sysid-approach.md`.

## Related docs

- PRD: `.scratch/open-manipulator-x-sysid/PRD.md`
- Domain terms: root `CONTEXT.md` §参数辨识
