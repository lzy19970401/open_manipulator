"""Multi-frequency Fourier excitation trajectory for OMX Arm sysid."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import yaml

ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')


class SafetyViolation(Exception):
    """Raised when a trajectory sample violates configured safety limits."""

#dataclass的意思是定义一个类，这个类是一个数据类，
# frozen=True的意思是这个类是不可变的，也就是说这个类的属性是不可变的,
@dataclass(frozen=True)  
class TrajectorySample:
    """Joint-space state at time t during excitation."""

    time_s: float
    position: Dict[str, float]
    velocity: Dict[str, float]
    acceleration: Dict[str, float]


@dataclass(frozen=True)
class JointSafetyLimits:
    hard_lower: float
    hard_upper: float
    soft_lower: float
    soft_upper: float
    q0: float
    max_velocity: float
    max_acceleration: float


class ExcitationTrajectory:
    """Fourier excitation q(t), q̇(t), q̈(t) for Arm joints joint1–joint4."""

    def __init__(
        self,
        config: Mapping[str, object],  # Mapping[str, object]的意思是映射一个字符串到对象，这个对象可以是任何类型
        *,
        amplitude_scale: Optional[float] = None,
    ) -> None:
        self._joints = tuple(config['arm_joints']) # tuple(config['arm_joints'])的意思是将config['arm_joints']转换为一个元组
        if tuple(self._joints) != ARM_JOINTS:
            raise ValueError(f'arm_joints must be {ARM_JOINTS}, got {self._joints}')

        excitation = config['excitation'] # config['excitation']的意思是获取config中的excitation键的值
        safety = config['safety'] # config['safety']的意思是获取config中的safety键的值
        self._num_harmonics = int(excitation['num_harmonics'])
        self._period_s = float(excitation['period_s']) # float(excitation['period_s'])的意思是将excitation['period_s']转换为浮点数
        self._omega0 = 2.0 * math.pi / self._period_s
        self._amplitude_fraction = float(excitation['amplitude_fraction'])
        inset = float(safety['soft_limit_inset_fraction'])
        self._max_velocity = float(safety['max_velocity_rad_s'])
        self._max_acceleration = float(safety['max_acceleration_rad_s2'])

        q0_cfg = config['q0']
        limits_cfg = config['joint_limits']
        self._limits: Dict[str, JointSafetyLimits] = {}
        for joint in self._joints:
            lower = float(limits_cfg[joint]['lower'])
            upper = float(limits_cfg[joint]['upper'])
            span = upper - lower
            margin = inset * span
            self._limits[joint] = JointSafetyLimits(
                hard_lower=lower,
                hard_upper=upper,
                soft_lower=lower + margin,
                soft_upper=upper - margin,
                q0=float(q0_cfg[joint]),
                max_velocity=self._max_velocity,
                max_acceleration=self._max_acceleration,
            )

        self._amplitudes, self._phases = self._build_harmonics()
        if amplitude_scale is not None:
            self._amplitude_scale = float(amplitude_scale)
        else:
            self._amplitude_scale = self._compute_amplitude_scale()
        self._scaled_amplitudes = {
            joint: [a * self._amplitude_scale for a in amps]
            for joint, amps in self._amplitudes.items()
        }

    @classmethod
    def from_yaml(cls, path: Path | str, **kwargs: object) -> 'ExcitationTrajectory':
        with Path(path).open('r', encoding='utf-8') as handle:
            config = yaml.safe_load(handle)
        return cls(config, **kwargs)

    @classmethod
    def default(cls, **kwargs: object) -> 'ExcitationTrajectory':
        return cls.from_yaml(default_config_path(), **kwargs)

    @property
    def joints(self) -> Sequence[str]:
        return self._joints

    @property
    def period_s(self) -> float:
        return self._period_s

    @property
    def amplitude_scale(self) -> float:
        return self._amplitude_scale

    def sample(self, time_s: float) -> TrajectorySample:
        """Return q, q̇, q̈ at excitation-local time t ∈ [0, period)."""
        t = float(time_s)
        position: Dict[str, float] = {}
        velocity: Dict[str, float] = {}
        acceleration: Dict[str, float] = {}

        for joint in self._joints:
            q, qd, qdd = self._joint_kinematics(joint, t)
            position[joint] = q
            velocity[joint] = qd
            acceleration[joint] = qdd

        return TrajectorySample(
            time_s=t,
            position=position,
            velocity=velocity,
            acceleration=acceleration,
        )

    def validate_sample(self, time_s: float) -> None:
        """Raise SafetyViolation if the sample exceeds soft limits or velocity cap."""
        sample = self.sample(time_s)
        self._validate_state(sample.position, sample.velocity, sample.acceleration)

    def validate_period(self, num_samples: int = 1000) -> None:
        """Validate safety over one excitation period."""
        if num_samples < 2:
            raise ValueError('num_samples must be at least 2')
        for index in range(num_samples + 1):
            t = self._period_s * index / num_samples
            self.validate_sample(t)

    def max_abs_values_over_period(self, num_samples: int = 2000) -> Dict[str, Dict[str, float]]:
        """Return peak |q|, |q̇|, |q̈| per joint over one period."""
        peaks = {
            joint: {'position': 0.0, 'velocity': 0.0, 'acceleration': 0.0}
            for joint in self._joints
        }
        for index in range(num_samples + 1):
            sample = self.sample(self._period_s * index / num_samples)
            for joint in self._joints:
                limits = self._limits[joint]
                peaks[joint]['position'] = max(
                    peaks[joint]['position'],
                    abs(sample.position[joint] - limits.q0),
                )
                peaks[joint]['velocity'] = max(
                    peaks[joint]['velocity'],
                    abs(sample.velocity[joint]),
                )
                peaks[joint]['acceleration'] = max(
                    peaks[joint]['acceleration'],
                    abs(sample.acceleration[joint]),
                )
        return peaks

    def _build_harmonics(self) -> Tuple[Dict[str, List[float]], Dict[str, List[float]]]:
        # 共5个谐波 权重值分别为 1/1, 1/2, 1/3, 1/4, 1/5 相位值都为0
        # 这样使得 速度贡献均匀分布
        harmonic_weights = [1.0 / harmonic for harmonic in range(1, self._num_harmonics + 1)]
        weight_sum = sum(harmonic_weights)

        amplitudes: Dict[str, List[float]] = {}
        phases: Dict[str, List[float]] = {}
        for joint in self._joints:
            limits = self._limits[joint]
            total_amp = self._amplitude_fraction * (
                limits.hard_upper - limits.hard_lower
            )
            amps = [total_amp * weight / weight_sum for weight in harmonic_weights]
            amplitudes[joint] = amps
            phases[joint] = [0.0] * self._num_harmonics
        return amplitudes, phases

    def _compute_amplitude_scale(self) -> float:
        """Uniform scale so analytical bounds satisfy soft limits and velocity cap."""
        scale = 1.0
        for joint in self._joints:
            limits = self._limits[joint]
            amps = self._amplitudes[joint]
            pos_span = sum(amps)
            if pos_span <= 0.0:
                continue

            pos_upper_margin = limits.soft_upper - limits.q0
            pos_lower_margin = limits.q0 - limits.soft_lower
            pos_scale = min(pos_upper_margin, pos_lower_margin) / pos_span

            vel_bound = sum(
                amp * harmonic * self._omega0
                for amp, harmonic in zip(amps, range(1, self._num_harmonics + 1))
            )
            vel_scale = self._max_velocity / vel_bound if vel_bound > 0.0 else 1.0

            acc_bound = sum(
                amp * harmonic * harmonic * self._omega0 * self._omega0
                for amp, harmonic in zip(amps, range(1, self._num_harmonics + 1))
            )
            acc_scale = (
                self._max_acceleration / acc_bound if acc_bound > 0.0 else 1.0
            )
            # 取最小值，确保所有限制都满足
            scale = min(scale, pos_scale, vel_scale, acc_scale)

        if scale <= 0.0 or not math.isfinite(scale):
            raise SafetyViolation('Unable to scale excitation within safety limits')
        return scale

    def _joint_kinematics(self, joint: str, time_s: float) -> Tuple[float, float, float]:
        limits = self._limits[joint]
        amps = self._scaled_amplitudes[joint]
        phases = self._phases[joint]

        q = limits.q0
        qd = 0.0
        qdd = 0.0
        
        # 计算每个谐波的贡献 并累加
        for amp, harmonic, phase in zip(amps, range(1, self._num_harmonics + 1), phases):
            omega = harmonic * self._omega0
            angle = omega * time_s + phase
            sin_a = math.sin(angle)
            cos_a = math.cos(angle)
            q += amp * sin_a
            qd += amp * omega * cos_a
            qdd += -amp * omega * omega * sin_a
        return q, qd, qdd

    def _validate_state(
        self,
        position: Mapping[str, float],
        velocity: Mapping[str, float],
        acceleration: Mapping[str, float],
    ) -> None:
        for joint in self._joints:
            limits = self._limits[joint]
            q = position[joint]
            qd = velocity[joint]
            qdd = acceleration[joint]

            if q < limits.soft_lower or q > limits.soft_upper:
                raise SafetyViolation(
                    f'{joint} position {q:.4f} rad outside soft limits '
                    f'[{limits.soft_lower:.4f}, {limits.soft_upper:.4f}]'
                )
            if abs(qd) > limits.max_velocity + 1e-9:
                raise SafetyViolation(
                    f'{joint} velocity {qd:.4f} rad/s exceeds cap '
                    f'{limits.max_velocity:.4f} rad/s'
                )
            if abs(qdd) > limits.max_acceleration + 1e-9:
                raise SafetyViolation(
                    f'{joint} acceleration {qdd:.4f} rad/s² exceeds cap '
                    f'{limits.max_acceleration:.4f} rad/s²'
                )


def default_config_path() -> Path:
    """Return bundled excitation.yaml (source tree or install share)."""
    source_path = Path(__file__).resolve().parents[1] / 'config' / 'excitation.yaml'
    if source_path.is_file():
        return source_path

    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory('open_manipulator_sysid'))
        installed = share / 'config' / 'excitation.yaml'
        if installed.is_file():
            return installed
    except Exception:
        pass

    raise FileNotFoundError(
        'excitation.yaml not found in source tree or package share directory'
    )


def main(args: Optional[Sequence[str]] = None) -> None:
    """Print trajectory samples for the four Arm joints (CLI entry point)."""
    import argparse   # argparse是Python标准库中的一个模块，用于解析命令行参数

    # parser是argparse.ArgumentParser的一个实例，用于解析命令行参数
    parser = argparse.ArgumentParser(
        description='Print OMX Arm Fourier excitation trajectory samples.'
    )
    parser.add_argument(
        '--config',
        type=Path,
        default=None,
        help='Path to excitation.yaml (default: package config)',
    )
    parser.add_argument(
        '--samples',
        type=int,
        default=11,
        help='Number of samples over one period (default: 11)',
    )
    parsed = parser.parse_args(args)

    config_path = parsed.config or default_config_path()
    trajectory = ExcitationTrajectory.from_yaml(config_path)
    trajectory.validate_period()

    print(f'config: {config_path}')
    print(f'period: {trajectory.period_s:.3f} s, amplitude_scale: {trajectory.amplitude_scale:.4f}')
    print('time_s  ' + '  '.join(f'{joint:>8s}' for joint in trajectory.joints))
    for index in range(parsed.samples):
        t = trajectory.period_s * index / (parsed.samples - 1 if parsed.samples > 1 else 1)
        sample = trajectory.sample(t)
        values = '  '.join(f'{sample.position[joint]:8.4f}' for joint in trajectory.joints)
        print(f'{t:6.3f}  {values}')

    peaks = trajectory.max_abs_values_over_period()
    print('\npeak |q-q0| / |q̇| / |q̈| over one period:')
    for joint in trajectory.joints:
        p = peaks[joint]
        print(
            f'  {joint}: {p["position"]:.4f} rad, '
            f'{p["velocity"]:.4f} rad/s, {p["acceleration"]:.4f} rad/s²'
        )


if __name__ == '__main__':
    main()
