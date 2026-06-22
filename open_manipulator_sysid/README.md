# open_manipulator_sysid

OpenMANIPULATOR-X **system identification** package: multi-frequency Fourier **excitation trajectory**, dataset recording, and offline **BIP + friction** regression.

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

## Package layout

```
open_manipulator_sysid/
├── config/excitation.yaml
├── open_manipulator_sysid/excitation_trajectory.py
├── test/test_excitation_trajectory.py
└── results/                 # gitignored identification outputs (later issues)
```

## Isolation

This package does **not** modify `open_manipulator_bringup` or GC controller yaml. See `docs/adr/0001-omx-sysid-approach.md`.

## Related docs

- PRD: `.scratch/open-manipulator-x-sysid/PRD.md`
- Domain terms: root `CONTEXT.md` §参数辨识
