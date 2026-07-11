"""Multi-harmonic Swevers Fourier excitation trajectory for OMX Arm sysid."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import yaml

ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')
COEFFICIENTS_SOURCE_TEMPLATE = 'template'
COEFFICIENTS_SOURCE_GA = 'ga'


class SafetyViolation(Exception):
    """Raised when a trajectory sample violates configured safety limits."""


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


@dataclass(frozen=True)
class RuntimeSchedule:
    """Segment boundaries for the full sysid motion (seconds from goal start)."""

    to_q0_duration_s: float
    ingress_spline_duration_s: float
    excitation_duration_s: float
    egress_spline_duration_s: float
    q0_hold_duration_s: float
    total_duration_s: float
    bag_record_start_s: float
    bag_record_end_s: float
    excitation_start_s: float
    excitation_end_s: float
    identification_start_s: float
    identification_end_s: float
    excitation_period_s: float
    excitation_periods: int
    discard_periods: int


def compute_swevers_kinematics(
    q0: float,
    a_coeffs: Sequence[float],
    b_coeffs: Sequence[float],
    omega_f: float,
    time_s: float,
) -> Tuple[float, float, float]:
    """Paper Fourier excitation (H harmonics, fundamental ω_f = 2π/T).

    q    = q₀ + Σ_l [a_l sin(2πlω_f t)/(2πlω_f) − b_l cos(2πlω_f t)/(2πlω_f)]
    q̇   = Σ_l [a_l cos(2πlω_f t) + b_l sin(2πlω_f t)]
    q̈   = Σ_l [−2πlω_f a_l sin(2πlω_f t) + 2πlω_f b_l cos(2πlω_f t)]

    Period boundaries are generally non-zero; ingress/egress C² splines bridge q₀.
    """
    if len(a_coeffs) != len(b_coeffs):
        raise ValueError('a_coeffs and b_coeffs must have the same length')
    q = q0
    qd = 0.0
    qdd = 0.0
    two_pi = 2.0 * math.pi
    for harmonic_index, (a_l, b_l) in enumerate(
        zip(a_coeffs, b_coeffs),
        start=1,
    ):
        omega_l = two_pi * harmonic_index * omega_f
        angle = omega_l * time_s
        sin_a = math.sin(angle)
        cos_a = math.cos(angle)
        q += a_l * sin_a / omega_l - b_l * cos_a / omega_l
        qd += a_l * cos_a + b_l * sin_a
        qdd += -omega_l * a_l * sin_a + omega_l * b_l * cos_a
    return q, qd, qdd


class ExcitationTrajectory:
    """Swevers (a,b) Fourier excitation for Arm joints joint1–joint4."""

    def __init__(
        self,
        config: Mapping[str, object],
        *,
        coefficient_scale: Optional[float] = None,
    ) -> None:
        self._joints = tuple(config['arm_joints'])
        if tuple(self._joints) != ARM_JOINTS:
            raise ValueError(f'arm_joints must be {ARM_JOINTS}, got {self._joints}')

        excitation = config['excitation']
        safety = config['safety']
        self._num_harmonics = int(excitation['num_harmonics'])
        self._period_s = float(excitation['period_s'])
        self._omega_f = 1.0 / self._period_s
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

        constraints = config.get('constraints', {})
        self._ee_z_min_m = float(constraints.get('end_effector_z_min_m', 0.077))
        self._ee_frame = str(
            constraints.get('end_effector_frame', 'end_effector_link')
        )

        if config.get('ga_coefficients'):
            self._coefficients_source = COEFFICIENTS_SOURCE_GA
            self._a_coeffs, self._b_coeffs = self._load_ga_coefficients(config)
        elif excitation.get('coefficients_source') == COEFFICIENTS_SOURCE_TEMPLATE:
            self._coefficients_source = COEFFICIENTS_SOURCE_TEMPLATE
            self._a_coeffs, self._b_coeffs = self._build_template_coefficients()
        else:
            raise ValueError(
                'Excitation config must contain ga_coefficients (sysid excitation) '
                'or excitation.coefficients_source: template (validation profile).'
            )

        ga_scale = config.get('ga_coefficient_scale')
        if coefficient_scale is not None:
            self._coefficient_scale = float(coefficient_scale)
        elif (
            self._coefficients_source == COEFFICIENTS_SOURCE_GA
            and ga_scale is not None
        ):
            self._coefficient_scale = float(ga_scale)
        else:
            self._coefficient_scale = self._compute_coefficient_scale()

        if self._coefficients_source == COEFFICIENTS_SOURCE_GA and ga_scale is not None:
            self._scaled_a = {joint: list(self._a_coeffs[joint]) for joint in self._joints}
            self._scaled_b = {joint: list(self._b_coeffs[joint]) for joint in self._joints}
        else:
            self._scaled_a = {
                joint: [value * self._coefficient_scale for value in coeffs]
                for joint, coeffs in self._a_coeffs.items()
            }
            self._scaled_b = {
                joint: [value * self._coefficient_scale for value in coeffs]
                for joint, coeffs in self._b_coeffs.items()
            }

    @classmethod
    def from_config(cls, config: Mapping[str, object], **kwargs: object) -> 'ExcitationTrajectory':
        return cls(config, **kwargs)

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
    def omega_f(self) -> float:
        return self._omega_f

    @property
    def coefficient_scale(self) -> float:
        return self._coefficient_scale

    @property
    def coefficients_source(self) -> str:
        return self._coefficients_source

    @property
    def end_effector_z_min_m(self) -> float:
        return self._ee_z_min_m

    @property
    def end_effector_frame(self) -> str:
        return self._ee_frame

    @property
    def q0(self) -> Dict[str, float]:
        return {joint: self._limits[joint].q0 for joint in self._joints}

    @property
    def a_coefficients(self) -> Dict[str, List[float]]:
        return {joint: list(self._scaled_a[joint]) for joint in self._joints}

    @property
    def b_coefficients(self) -> Dict[str, List[float]]:
        return {joint: list(self._scaled_b[joint]) for joint in self._joints}

    def sample(self, time_s: float, *, wrap: bool = True) -> TrajectorySample:
        """Return q, q̇, q̈ at excitation-local time.

        With ``wrap=True`` (default), time is taken modulo one period — used for GA
        and single-period validation. With ``wrap=False``, time is continuous for
        multi-period playback.
        """
        t = float(time_s)
        if wrap:
            t = t % self._period_s
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

    def period_boundary_states(self) -> Tuple[TrajectorySample, TrajectorySample]:
        """Return excitation states at t=0 and t=period (generally not equal to q₀)."""
        return self.sample(0.0, wrap=False), self.sample(self._period_s, wrap=False)

    def validate_sample(self, time_s: float) -> None:
        sample = self.sample(time_s)
        self._validate_state(sample.position, sample.velocity, sample.acceleration)

    def validate_period(self, num_samples: int = 1000) -> None:
        """Check joint position/velocity/acceleration limits over one excitation period."""
        if num_samples < 2:
            raise ValueError('num_samples must be at least 2')
        for index in range(num_samples + 1):
            t = self._period_s * index / num_samples
            self.validate_sample(t)

    def max_abs_values_over_period(self, num_samples: int = 2000) -> Dict[str, Dict[str, float]]:
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

    def _build_template_coefficients(
        self,
    ) -> Tuple[Dict[str, List[float]], Dict[str, List[float]]]:
        harmonic_weights = [1.0 / harmonic for harmonic in range(1, self._num_harmonics + 1)]
        weight_sum = sum(harmonic_weights)
        a_coeffs: Dict[str, List[float]] = {}
        b_coeffs: Dict[str, List[float]] = {}
        for joint_index, joint in enumerate(self._joints):
            limits = self._limits[joint]
            vel_span = self._amplitude_fraction * limits.max_velocity
            a_list: List[float] = []
            b_list: List[float] = []
            for harmonic, weight in enumerate(harmonic_weights, start=1):
                sign = 1.0 if (harmonic + joint_index) % 2 == 0 else -1.0
                a_list.append(sign * vel_span * weight / weight_sum)
                b_list.append(-sign * vel_span * weight / (harmonic * weight_sum))
            a_coeffs[joint] = a_list
            b_coeffs[joint] = b_list
        return a_coeffs, b_coeffs

    def _load_ga_coefficients(
        self,
        config: Mapping[str, object],
    ) -> Tuple[Dict[str, List[float]], Dict[str, List[float]]]:
        ga_coefficients = config.get('ga_coefficients')
        if not isinstance(ga_coefficients, Mapping):
            raise ValueError(
                'ga_coefficients is required when excitation.coefficients_source=ga'
            )

        a_coeffs: Dict[str, List[float]] = {}
        b_coeffs: Dict[str, List[float]] = {}
        for joint in self._joints:
            joint_cfg = ga_coefficients.get(joint)
            if not isinstance(joint_cfg, Mapping):
                raise ValueError(f'ga_coefficients missing joint {joint!r}')
            joint_a = list(joint_cfg.get('a', []))
            joint_b = list(joint_cfg.get('b', []))
            if len(joint_a) != self._num_harmonics:
                raise ValueError(
                    f'{joint} ga_coefficients.a must have '
                    f'{self._num_harmonics} entries, got {len(joint_a)}'
                )
            if len(joint_b) != self._num_harmonics:
                raise ValueError(
                    f'{joint} ga_coefficients.b must have '
                    f'{self._num_harmonics} entries, got {len(joint_b)}'
                )
            a_coeffs[joint] = [float(value) for value in joint_a]
            b_coeffs[joint] = [float(value) for value in joint_b]
        return a_coeffs, b_coeffs

    def _compute_coefficient_scale(self) -> float:
        scale = 1.0
        for _ in range(12):
            violated = False
            for joint in self._joints:
                limits = self._limits[joint]
                for index in range(2001):
                    t = self._period_s * index / 2000
                    q, qd, qdd = compute_swevers_kinematics(
                        limits.q0,
                        [value * scale for value in self._a_coeffs[joint]],
                        [value * scale for value in self._b_coeffs[joint]],
                        self._omega_f,
                        t,
                    )
                    if q < limits.hard_lower or q > limits.hard_upper:
                        violated = True
                        break
                    if abs(qd) > limits.max_velocity + 1e-9:
                        violated = True
                        break
                    if abs(qdd) > limits.max_acceleration + 1e-9:
                        violated = True
                        break
                if violated:
                    scale *= 0.85
                    break
            if not violated:
                break
        if scale <= 0.0 or not math.isfinite(scale):
            raise SafetyViolation('Unable to scale Swevers excitation within safety limits')
        return scale

    def _joint_kinematics(self, joint: str, time_s: float) -> Tuple[float, float, float]:
        limits = self._limits[joint]
        return compute_swevers_kinematics(
            limits.q0,
            self._scaled_a[joint],
            self._scaled_b[joint],
            self._omega_f,
            time_s,
        )

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

            if q < limits.hard_lower or q > limits.hard_upper:
                raise SafetyViolation(
                    f'{joint} position {q:.4f} rad outside limits '
                    f'[{limits.hard_lower:.4f}, {limits.hard_upper:.4f}]'
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


QUINTIC_MAX_VEL_COEFF = 1.875


def required_quintic_duration(
    start_positions: Dict[str, float],
    end_positions: Dict[str, float],
    *,
    joints: Sequence[str],
    max_velocity: float,
    min_duration_s: float,
) -> float:
    if max_velocity <= 0.0:
        raise ValueError('max_velocity must be positive')
    duration = float(min_duration_s)
    for joint in joints:
        delta = abs(end_positions[joint] - start_positions[joint])
        if delta <= 1e-12:
            continue
        needed = delta * QUINTIC_MAX_VEL_COEFF / max_velocity
        duration = max(duration, needed)
    return duration


def quintic_blend(start: float, end: float, alpha: float) -> tuple[float, float, float]:
    alpha = float(max(0.0, min(1.0, alpha)))
    alpha2 = alpha * alpha
    alpha3 = alpha2 * alpha
    alpha4 = alpha3 * alpha
    alpha5 = alpha4 * alpha
    pos_coeff = 10.0 * alpha3 - 15.0 * alpha4 + 6.0 * alpha5
    vel_coeff = 30.0 * alpha2 - 60.0 * alpha3 + 30.0 * alpha4
    acc_coeff = 60.0 * alpha - 180.0 * alpha2 + 120.0 * alpha3
    delta = end - start
    return start + delta * pos_coeff, delta * vel_coeff, delta * acc_coeff


def _quintic_boundary_coefficients(
    p0: float,
    v0: float,
    a0: float,
    p1: float,
    v1: float,
    a1: float,
    duration_s: float,
) -> Tuple[float, float, float, float, float, float]:
    """Polynomial coefficients for p(t)=c0+c1*t+c2*t²+c3*t³+c4*t⁴+c5*t⁵ on [0, duration_s]."""
    if duration_s <= 0.0:
        raise ValueError('duration_s must be positive')
    c0 = p0
    c1 = v0
    c2 = a0 / 2.0
    t = duration_s
    t2 = t * t
    t3 = t2 * t
    t4 = t3 * t
    t5 = t4 * t
    rhs_pos = p1 - c0 - c1 * t - c2 * t2
    rhs_vel = v1 - c1 - 2.0 * c2 * t
    rhs_acc = a1 - 2.0 * c2
    # 3x3 system for c3, c4, c5 from terminal position/velocity/acceleration.
    a11, a12, a13 = t3, t4, t5
    a21, a22, a23 = 3.0 * t2, 4.0 * t3, 5.0 * t4
    a31, a32, a33 = 6.0 * t, 12.0 * t2, 20.0 * t3
    det = (
        a11 * (a22 * a33 - a23 * a32)
        - a12 * (a21 * a33 - a23 * a31)
        + a13 * (a21 * a32 - a22 * a31)
    )
    if abs(det) < 1e-12:
        raise ValueError('Degenerate quintic transition duration')
    c3 = (
        rhs_pos * (a22 * a33 - a23 * a32)
        - a12 * (rhs_vel * a33 - a23 * rhs_acc)
        + a13 * (rhs_vel * a32 - a22 * rhs_acc)
    ) / det
    c4 = (
        a11 * (rhs_vel * a33 - a23 * rhs_acc)
        - rhs_pos * (a21 * a33 - a23 * a31)
        + a13 * (a21 * rhs_acc - rhs_vel * a31)
    ) / det
    c5 = (
        a11 * (a22 * rhs_acc - rhs_vel * a32)
        - a12 * (a21 * rhs_acc - rhs_vel * a31)
        + rhs_pos * (a21 * a32 - a22 * a31)
    ) / det
    return c0, c1, c2, c3, c4, c5


def _eval_quintic(
    coeffs: Tuple[float, float, float, float, float, float],
    time_s: float,
) -> Tuple[float, float, float]:
    c0, c1, c2, c3, c4, c5 = coeffs
    t = time_s
    t2 = t * t
    t3 = t2 * t
    t4 = t3 * t
    t5 = t4 * t
    pos = c0 + c1 * t + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5
    vel = c1 + 2.0 * c2 * t + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t4
    acc = 2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t2 + 20.0 * c5 * t3
    return pos, vel, acc


def required_c2_transition_duration(
    start: TrajectorySample,
    end: TrajectorySample,
    *,
    joints: Sequence[str],
    max_velocity: float,
    max_acceleration: float,
    min_duration_s: float,
) -> float:
    """Conservative duration for a C² spline between two full joint states."""
    if max_velocity <= 0.0 or max_acceleration <= 0.0:
        raise ValueError('max_velocity and max_acceleration must be positive')
    duration = float(min_duration_s)
    for joint in joints:
        delta_q = abs(end.position[joint] - start.position[joint])
        delta_v = abs(end.velocity[joint] - start.velocity[joint])
        delta_a = abs(end.acceleration[joint] - start.acceleration[joint])
        if delta_q > 1e-12:
            duration = max(duration, delta_q * QUINTIC_MAX_VEL_COEFF / max_velocity)
        if delta_v > 1e-12:
            duration = max(duration, delta_v / max_velocity)
        if delta_a > 1e-12:
            duration = max(duration, math.sqrt(delta_a / max_acceleration))
    return duration


def _hold_sample(
    time_s: float,
    positions: Mapping[str, float],
    joints: Sequence[str],
) -> TrajectorySample:
    return TrajectorySample(
        time_s=time_s,
        position={joint: positions[joint] for joint in joints},
        velocity={joint: 0.0 for joint in joints},
        acceleration={joint: 0.0 for joint in joints},
    )


def _quintic_segment_sample(
    time_s: float,
    segment_start_s: float,
    duration_s: float,
    start_positions: Mapping[str, float],
    end_positions: Mapping[str, float],
    joints: Sequence[str],
) -> TrajectorySample:
    alpha = (time_s - segment_start_s) / duration_s
    position: Dict[str, float] = {}
    velocity: Dict[str, float] = {}
    acceleration: Dict[str, float] = {}
    for joint in joints:
        pos, vel, acc = quintic_blend(start_positions[joint], end_positions[joint], alpha)
        position[joint] = pos
        velocity[joint] = vel / duration_s
        acceleration[joint] = acc / (duration_s * duration_s)
    return TrajectorySample(
        time_s=time_s,
        position=position,
        velocity=velocity,
        acceleration=acceleration,
    )


def _c2_transition_segment_sample(
    time_s: float,
    segment_start_s: float,
    duration_s: float,
    start_state: TrajectorySample,
    end_state: TrajectorySample,
    joints: Sequence[str],
) -> TrajectorySample:
    """Clamped quintic B-spline (single-span, C²) matching pos/vel/acc at both ends."""
    local_t = time_s - segment_start_s
    position: Dict[str, float] = {}
    velocity: Dict[str, float] = {}
    acceleration: Dict[str, float] = {}
    for joint in joints:
        coeffs = _quintic_boundary_coefficients(
            start_state.position[joint],
            start_state.velocity[joint],
            start_state.acceleration[joint],
            end_state.position[joint],
            end_state.velocity[joint],
            end_state.acceleration[joint],
            duration_s,
        )
        pos, vel, acc = _eval_quintic(coeffs, local_t)
        position[joint] = pos
        velocity[joint] = vel
        acceleration[joint] = acc
    return TrajectorySample(
        time_s=time_s,
        position=position,
        velocity=velocity,
        acceleration=acceleration,
    )


def describe_runtime_schedule(
    trajectory: ExcitationTrajectory,
    *,
    start_positions: Mapping[str, float],
    to_q0_duration_s: float,
    ingress_spline_duration_s: float,
    num_periods: int,
    egress_spline_duration_s: float,
    q0_hold_duration_s: float,
    discard_periods: int = 1,
) -> RuntimeSchedule:
    excitation_duration_s = num_periods * trajectory.period_s
    to_q0_end_s = to_q0_duration_s
    excitation_start_s = to_q0_end_s + ingress_spline_duration_s
    excitation_end_s = excitation_start_s + excitation_duration_s
    egress_end_s = excitation_end_s + egress_spline_duration_s
    total_duration_s = egress_end_s + q0_hold_duration_s
    bag_record_start_s = to_q0_end_s
    bag_record_end_s = excitation_end_s
    identification_start_s = excitation_start_s + discard_periods * trajectory.period_s
    identification_end_s = excitation_end_s
    return RuntimeSchedule(
        to_q0_duration_s=to_q0_duration_s,
        ingress_spline_duration_s=ingress_spline_duration_s,
        excitation_duration_s=excitation_duration_s,
        egress_spline_duration_s=egress_spline_duration_s,
        q0_hold_duration_s=q0_hold_duration_s,
        total_duration_s=total_duration_s,
        bag_record_start_s=bag_record_start_s,
        bag_record_end_s=bag_record_end_s,
        excitation_start_s=excitation_start_s,
        excitation_end_s=excitation_end_s,
        identification_start_s=identification_start_s,
        identification_end_s=identification_end_s,
        excitation_period_s=trajectory.period_s,
        excitation_periods=num_periods,
        discard_periods=discard_periods,
    )


def build_sysid_runtime_trajectory_messages(
    trajectory: ExcitationTrajectory,
    *,
    start_positions: Dict[str, float],
    to_q0_duration_s: float,
    ingress_spline_duration_s: float,
    num_periods: int,
    egress_spline_duration_s: float,
    q0_hold_duration_s: float,
    sample_dt_s: float,
    discard_periods: int = 1,
) -> Tuple[List[TrajectorySample], RuntimeSchedule]:
    """Build full sysid motion: current→q₀→C² ingress→Fourier→C² egress→hold q₀."""
    if to_q0_duration_s <= 0.0:
        raise ValueError('to_q0_duration_s must be positive')
    if ingress_spline_duration_s <= 0.0 or egress_spline_duration_s <= 0.0:
        raise ValueError('Ingress/egress spline durations must be positive')
    if num_periods < 1:
        raise ValueError('num_periods must be at least 1')
    if sample_dt_s <= 0.0:
        raise ValueError('sample_dt_s must be positive')
    if discard_periods < 0 or discard_periods >= num_periods:
        raise ValueError('discard_periods must satisfy 0 <= discard_periods < num_periods')

    schedule = describe_runtime_schedule(
        trajectory,
        start_positions=start_positions,
        to_q0_duration_s=to_q0_duration_s,
        ingress_spline_duration_s=ingress_spline_duration_s,
        num_periods=num_periods,
        egress_spline_duration_s=egress_spline_duration_s,
        q0_hold_duration_s=q0_hold_duration_s,
        discard_periods=discard_periods,
    )

    q0 = trajectory.q0
    joints = trajectory.joints
    excitation_start, excitation_end = trajectory.period_boundary_states()
    q0_state = _hold_sample(0.0, q0, joints)
    excitation_terminal = trajectory.sample(
        num_periods * trajectory.period_s,
        wrap=False,
    )

    t_to_q0_end = schedule.to_q0_duration_s
    t_ingress_end = schedule.excitation_start_s
    t_excitation_end = schedule.excitation_end_s
    t_egress_end = t_excitation_end + schedule.egress_spline_duration_s

    num_steps = int(math.floor(schedule.total_duration_s / sample_dt_s)) + 1
    samples: List[TrajectorySample] = []

    for step in range(num_steps):
        time_s = min(step * sample_dt_s, schedule.total_duration_s)

        if time_s <= t_to_q0_end:
            sample = _quintic_segment_sample(
                time_s,
                0.0,
                to_q0_duration_s,
                start_positions,
                q0,
                joints,
            )
        elif time_s <= t_ingress_end:
            sample = _c2_transition_segment_sample(
                time_s,
                t_to_q0_end,
                ingress_spline_duration_s,
                q0_state,
                excitation_start,
                joints,
            )
        elif time_s < t_excitation_end - 1e-12:
            excitation_time = time_s - t_ingress_end
            sample = trajectory.sample(excitation_time, wrap=False)
            sample = TrajectorySample(
                time_s=time_s,
                position=dict(sample.position),
                velocity=dict(sample.velocity),
                acceleration=dict(sample.acceleration),
            )
        elif time_s <= t_egress_end:
            sample = _c2_transition_segment_sample(
                time_s,
                t_excitation_end,
                egress_spline_duration_s,
                excitation_terminal,
                q0_state,
                joints,
            )
        else:
            sample = _hold_sample(time_s, q0, joints)

        samples.append(sample)

    return samples, schedule


# Backward-compatible alias used by older call sites during migration.
build_excitation_trajectory_messages = build_sysid_runtime_trajectory_messages


def runtime_config_from_yaml(config: Mapping[str, object]) -> Dict[str, float]:
    runtime = config.get('runtime', {})
    return {
        'to_q0_duration_s': float(runtime.get('to_q0_duration_s', 5.0)),
        'ingress_spline_duration_s': float(runtime.get('ingress_spline_duration_s', 4.0)),
        'egress_spline_duration_s': float(runtime.get('egress_spline_duration_s', 4.0)),
        'q0_hold_duration_s': float(runtime.get('q0_hold_duration_s', 10.0)),
        'discard_periods': int(runtime.get('discard_periods', 1)),
    }


def default_config_path() -> Path:
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


def default_ga_config_path() -> Path:
    """Backward-compatible alias for the single excitation config."""
    return default_config_path()


def main(args: Optional[Sequence[str]] = None) -> None:
    import argparse

    parser = argparse.ArgumentParser(
        description='Print OMX Arm Swevers Fourier excitation trajectory samples.'
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
    print(
        f'period: {trajectory.period_s:.3f} s, ω_f: {trajectory.omega_f:.4f} rad/s, '
        f'coefficient_scale: {trajectory.coefficient_scale:.4f}'
    )
    start_state, end_state = trajectory.period_boundary_states()
    print('period boundary (t=0 → t=T):')
    for joint in trajectory.joints:
        print(
            f'  {joint}: q {start_state.position[joint]:.4f}→{end_state.position[joint]:.4f} rad, '
            f'qd {start_state.velocity[joint]:.4f}→{end_state.velocity[joint]:.4f} rad/s'
        )
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
