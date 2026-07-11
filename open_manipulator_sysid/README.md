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

## Package layout (domain subpackages)

| Subpackage | Role |
|------------|------|
| `excitation_trajectory/` | Excitation trajectory, GA optimizer, ROS runner, plots |
| `excitation_recording/` | Excitation recording bag → `IdentificationDataset` |
| `five_link_dynamics/` | Five-link SVD BIP, reference tables (`reference_tables.py`) |
| `system_identification/` | Bag/synthetic LS identify (`identify.py`, `friction.py`) |
| `model_validation/` | Model validation run — BIP compare, torque/trajectory plots |
| `pinocchio_support/` | Pinocchio model load, `effective_condition_number` |
| `reference/` | `sysid_minimal_parameter_reference` CLI |
| `_experimental/` | Non-production experiments (figaroh script/config) |

Torque LSB → N·m conversion lives in `excitation_recording/bag_reader.py` (hardware bags).

See [ADR-0012](../docs/adr/0012-omx-sysid-domain-subpackages.md).

## Phase 1 — Issue 01: excitation trajectory

### Configuration

`config/excitation.yaml` — single sysid config (limits, q₀, `ga_coefficients`, `ga_metadata`). GA overwrites the same file.

- **Arm only**: `joint1`–`joint4` (no gripper)
- **q₀**: excitation center pose (not SRDF home)
- **Fourier**: N=5 harmonics, period T=10 s; coefficients from GA (`ga_coefficients`)
- **Safety**: velocity cap 1.0 rad/s, acceleration cap 2.0 rad/s²

Model validation uses separate `config/test_trajectory.yaml` (short template trajectory).

### CLI

Print one period of joint positions (validates soft limits and velocity cap first):

```bash
ros2 run open_manipulator_sysid excitation_trajectory_print
ros2 run open_manipulator_sysid sysid_excitation_trajectory_plot
```

Offline GA optimization (minimize column-scaled regressor κ; overwrites `excitation.yaml` by default):

```bash
ros2 run open_manipulator_sysid excitation_ga_optimize
ros2 run open_manipulator_sysid excitation_ga_optimize --quick   # smoke test
ros2 run open_manipulator_sysid excitation_ga_optimize --generations 200 --population 120
```

Online `excitation_runner` and `excitation_*` launch files default to `config/excitation.yaml`.

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

Run excitation in Gazebo (records **Excitation recording bag** — `/joint_states` + `/arm_controller/controller_state` — to `/workspace/sysid_results/bags/` by default, then shuts down):

```bash
ros2 launch open_manipulator_sysid excitation_gazebo.launch.py
```

Use template coefficients instead of GA defaults:

```bash
ros2 launch open_manipulator_sysid excitation_gazebo.launch.py \
  excitation_config:=$(ros2 pkg prefix open_manipulator_sysid)/share/open_manipulator_sysid/config/excitation.yaml
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

## Package layout

```
open_manipulator_sysid/
├── config/
│   ├── excitation.yaml
│   └── test_trajectory.yaml
├── launch/
│   ├── excitation_gazebo.launch.py
│   └── test_trajectory_gazebo.launch.py
├── open_manipulator_sysid/
│   ├── excitation_trajectory/
│   ├── excitation_recording/
│   ├── five_link_dynamics/
│   ├── system_identification/
│   ├── model_validation/
│   ├── pinocchio_support/
│   ├── reference/
│   └── _experimental/
├── test/
└── results/
```

## Isolation

This package does **not** modify `open_manipulator_bringup` or GC controller yaml. See `docs/adr/0001-omx-sysid-approach.md`.

## Related docs

- PRD: `.scratch/open-manipulator-x-sysid/PRD.md`
- Domain terms: root `CONTEXT.md` §参数辨识
