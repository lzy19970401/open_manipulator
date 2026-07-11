#!/usr/bin/env python3
"""Plot commanded vs feedback joint positions from an excitation recording bag."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from open_manipulator_sysid.excitation_recording.bag_reader import (
    ARM_CONTROLLER_STATE_TOPIC,
    commanded_reference_from_bag,
    interpolate_positions_at_times,
    joint_states_from_bag,
    overlap_time_mask,
    require_excitation_recording_bag,
)
from open_manipulator_sysid.excitation_recording.dataset import (
    filtered_measured_kinematics_on_times,
)
from open_manipulator_sysid.excitation_trajectory import (
    ARM_JOINTS,
    ExcitationTrajectory,
    RuntimeSchedule,
    TrajectorySample,
    build_sysid_runtime_trajectory_messages,
    default_config_path,
    required_c2_transition_duration,
    required_quintic_duration,
    runtime_config_from_yaml,
)


def _require_matplotlib():
    try:
        import matplotlib
    except ImportError as exc:  # pragma: no cover
        raise ImportError(
            'matplotlib is required for trajectory plots. '
            'Install python3-matplotlib in the Docker container.'
        ) from exc
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    return plt


def detect_motion_start_s(
    times_s: np.ndarray,
    positions: np.ndarray,
    *,
    velocity_threshold_rad_s: float = 0.02,
    hold_samples: int = 5,
) -> float:
    """Estimate trajectory t=0 on the bag time axis from the first sustained motion."""
    if times_s.size < hold_samples + 2:
        return 0.0

    dt = np.diff(times_s)
    dt = np.where(dt > 1e-9, dt, np.nan)
    velocity = np.diff(positions, axis=0) / dt[:, None]
    speed = np.linalg.norm(velocity, axis=1)

    for index in range(max(0, speed.size - hold_samples + 1)):
        window = speed[index : index + hold_samples]
        if np.all(window > velocity_threshold_rad_s):
            return float(times_s[index + 1] - times_s[0])
    return 0.0


def _start_positions_near_motion(
    times_s: np.ndarray,
    positions: np.ndarray,
    motion_start_s: float,
    *,
    lookback_s: float = 0.25,
) -> Dict[str, float]:
    bag_t0 = float(times_s[0])
    anchor = bag_t0 + motion_start_s
    mask = (times_s >= anchor - lookback_s) & (times_s <= anchor)
    if not np.any(mask):
        index = int(np.argmin(np.abs(times_s - anchor)))
        row = positions[index]
    else:
        row = np.mean(positions[mask], axis=0)
    return {joint: float(row[joint_index]) for joint_index, joint in enumerate(ARM_JOINTS)}


def build_reference_runtime(
    trajectory: ExcitationTrajectory,
    *,
    start_positions: Mapping[str, float],
    num_periods: int,
    sample_dt_s: float,
    config: Mapping[str, object],
) -> Tuple[np.ndarray, Dict[str, np.ndarray], RuntimeSchedule]:
    """Rebuild runtime schedule for optional phase backgrounds only."""
    runtime = runtime_config_from_yaml(config)
    max_vel = trajectory._max_velocity  # noqa: SLF001
    max_acc = trajectory._max_acceleration  # noqa: SLF001
    q0 = trajectory.q0

    runtime['to_q0_duration_s'] = required_quintic_duration(
        dict(start_positions),
        q0,
        joints=ARM_JOINTS,
        max_velocity=max_vel,
        min_duration_s=runtime['to_q0_duration_s'],
    )

    excitation_start, _ = trajectory.period_boundary_states()
    q0_state = TrajectorySample(
        time_s=0.0,
        position=dict(q0),
        velocity={joint: 0.0 for joint in ARM_JOINTS},
        acceleration={joint: 0.0 for joint in ARM_JOINTS},
    )
    runtime['ingress_spline_duration_s'] = required_c2_transition_duration(
        q0_state,
        excitation_start,
        joints=ARM_JOINTS,
        max_velocity=max_vel,
        max_acceleration=max_acc,
        min_duration_s=runtime['ingress_spline_duration_s'],
    )
    runtime['egress_spline_duration_s'] = required_c2_transition_duration(
        trajectory.sample(num_periods * trajectory.period_s, wrap=False),
        q0_state,
        joints=ARM_JOINTS,
        max_velocity=max_vel,
        max_acceleration=max_acc,
        min_duration_s=runtime['egress_spline_duration_s'],
    )

    samples, schedule = build_sysid_runtime_trajectory_messages(
        trajectory,
        start_positions=dict(start_positions),
        to_q0_duration_s=runtime['to_q0_duration_s'],
        ingress_spline_duration_s=runtime['ingress_spline_duration_s'],
        num_periods=num_periods,
        egress_spline_duration_s=runtime['egress_spline_duration_s'],
        q0_hold_duration_s=runtime['q0_hold_duration_s'],
        sample_dt_s=sample_dt_s,
        discard_periods=int(runtime['discard_periods']),
    )

    times = np.asarray([sample.time_s for sample in samples], dtype=float)
    positions = {
        joint: np.asarray([sample.position[joint] for sample in samples], dtype=float)
        for joint in ARM_JOINTS
    }
    return times, positions, schedule


def align_commanded_to_measured(
    measured_times_s: np.ndarray,
    measured_positions: np.ndarray,
    commanded_times_s: np.ndarray,
    commanded_positions: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Interpolate commanded positions onto measured timestamps within overlap."""
    mask = overlap_time_mask(measured_times_s, commanded_times_s)
    if not np.any(mask):
        raise ValueError(
            'No overlapping samples between /joint_states and '
            f'{ARM_CONTROLLER_STATE_TOPIC}.'
        )

    aligned_times = measured_times_s[mask]
    aligned_measured = measured_positions[mask]
    aligned_commanded = interpolate_positions_at_times(
        aligned_times,
        commanded_times_s,
        commanded_positions,
    )
    return aligned_times, aligned_measured, aligned_commanded


def _phase_spans_on_bag_axis(
    schedule: RuntimeSchedule,
    motion_start_s: float,
) -> Sequence[Tuple[float, float, str]]:
    t0 = motion_start_s
    return [
        (t0, t0 + schedule.to_q0_duration_s, 'to q₀'),
        (
            t0 + schedule.to_q0_duration_s,
            t0 + schedule.excitation_start_s,
            'ingress',
        ),
        (
            t0 + schedule.excitation_start_s,
            t0 + schedule.excitation_end_s,
            'Fourier',
        ),
        (
            t0 + schedule.excitation_end_s,
            t0 + schedule.excitation_end_s + schedule.egress_spline_duration_s,
            'egress',
        ),
        (
            t0 + schedule.excitation_end_s + schedule.egress_spline_duration_s,
            t0 + schedule.total_duration_s,
            'hold q₀',
        ),
    ]


def plot_joint_position_tracking(
    *,
    aligned_times_s: np.ndarray,
    measured_positions: np.ndarray,
    commanded_positions: np.ndarray,
    output_dir: Path,
    bag_time_origin_s: float,
    title_prefix: str = '',
    phase_spans: Optional[Sequence[Tuple[float, float, str]]] = None,
) -> list[Path]:
    plt = _require_matplotlib()
    output_dir.mkdir(parents=True, exist_ok=True)

    if aligned_times_s.size < 2:
        raise ValueError('Need at least two aligned samples to plot.')

    plot_times = aligned_times_s - bag_time_origin_s

    written: list[Path] = []
    for joint_index, joint_name in enumerate(ARM_JOINTS):
        figure, axis = plt.subplots(figsize=(11, 4.5))
        if phase_spans:
            colors = ('#f5f5f5', '#eef6ff', '#fff7e6', '#f0fff0', '#fafafa')
            for (start_s, end_s, _label), color in zip(phase_spans, colors):
                axis.axvspan(start_s, end_s, color=color, alpha=0.55, linewidth=0)

        axis.plot(
            plot_times,
            commanded_positions[:, joint_index],
            label='commanded (controller reference)',
            linewidth=1.4,
        )
        axis.plot(
            plot_times,
            measured_positions[:, joint_index],
            label='measured (/joint_states)',
            linewidth=1.0,
            alpha=0.85,
        )
        axis.set_xlabel('time since bag start (s)')
        axis.set_ylabel('position (rad)')
        title = f'{title_prefix}{joint_name}: commanded vs measured position'.strip()
        axis.set_title(title)
        axis.legend(loc='best')
        axis.grid(True, alpha=0.3)
        figure.tight_layout()
        out_path = output_dir / f'{joint_name}_trajectory_compare.png'
        figure.savefig(out_path, dpi=150)
        plt.close(figure)
        written.append(out_path)
    return written


def plot_joint_commanded_kinematics(
    *,
    commanded_times_s: np.ndarray,
    commanded_arrays: Mapping[str, np.ndarray],
    output_dir: Path,
    bag_time_origin_s: float,
    title_prefix: str = '',
    phase_spans: Optional[Sequence[Tuple[float, float, str]]] = None,
    measured_arrays: Optional[Mapping[str, np.ndarray]] = None,
    measured_acceleration_lowpass_window: int = 11,
) -> list[Path]:
    """Plot commanded and optional measured q/q̇/q̈ overlay (q̈ may be FIR-smoothed)."""
    plt = _require_matplotlib()
    output_dir.mkdir(parents=True, exist_ok=True)

    if commanded_times_s.size < 2:
        raise ValueError('Need at least two commanded samples to plot.')

    plot_times = commanded_times_s - bag_time_origin_s
    panels = (
        ('position', 'position (rad)'),
        ('velocity', 'velocity (rad/s)'),
        ('acceleration', 'acceleration (rad/s²)'),
    )
    measured_labels = {
        'position': 'measured (/joint_states)',
        'velocity': 'measured (/joint_states)',
        'acceleration': (
            f'measured (q̈ filtered, window={measured_acceleration_lowpass_window}, /joint_states)'
            if measured_acceleration_lowpass_window > 1
            else 'measured (/joint_states)'
        ),
    }

    written: list[Path] = []
    for joint_index, joint_name in enumerate(ARM_JOINTS):
        figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        for axis, (array_key, ylabel) in zip(axes, panels):
            if phase_spans:
                colors = ('#f5f5f5', '#eef6ff', '#fff7e6', '#f0fff0', '#fafafa')
                for (start_s, end_s, _label), color in zip(phase_spans, colors):
                    axis.axvspan(start_s, end_s, color=color, alpha=0.55, linewidth=0)

            axis.plot(
                plot_times,
                commanded_arrays[array_key][:, joint_index],
                label='commanded (controller reference)',
                linewidth=1.2,
            )
            if measured_arrays is not None:
                axis.plot(
                    plot_times,
                    measured_arrays[array_key][:, joint_index],
                    label=measured_labels[array_key],
                    linewidth=1.0,
                    alpha=0.85,
                )
            axis.set_ylabel(ylabel)
            axis.legend(loc='best')
            axis.grid(True, alpha=0.3)

        axes[-1].set_xlabel('time since bag start (s)')
        if measured_arrays is not None:
            title = (
                f'{title_prefix}{joint_name}: commanded vs measured '
                'position, velocity, acceleration'
            ).strip()
        else:
            title = (
                f'{title_prefix}{joint_name}: commanded position, velocity, acceleration'
            ).strip()
        figure.suptitle(title)
        figure.tight_layout()
        out_path = output_dir / f'{joint_name}_commanded_kinematics.png'
        figure.savefig(out_path, dpi=150)
        plt.close(figure)
        written.append(out_path)
    return written


def load_config(path: Optional[Path | str]) -> Tuple[Path, dict]:
    config_path = Path(path) if path is not None else default_config_path()
    with config_path.open('r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle) or {}
    return config_path, config


def main(args: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Plot excitation bag tracking: commanded vs measured positions (four PNG), '
            'plus commanded vs measured q/q̇/q̈ per joint (q̈ FIR-smoothed; four PNG).'
        )
    )
    parser.add_argument('--bag', type=Path, required=True, help='Excitation recording bag')
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path('/workspace/sysid_results/trajectory_plots'),
        help='Directory for joint PNG outputs',
    )
    parser.add_argument(
        '--skip-commanded-kinematics',
        action='store_true',
        help='Skip joint*_commanded_kinematics.png (commanded vs measured kinematics)',
    )
    parser.add_argument(
        '--skip-measured-kinematics-overlay',
        action='store_true',
        help='Plot commanded q/q̇/q̈ only (no filtered /joint_states overlay)',
    )
    parser.add_argument(
        '--measured-acceleration-lowpass-window',
        type=int,
        default=11,
        metavar='N',
        help=(
            'FIR moving-average on measured q̈ only (0 or 1 = no q̈ smoothing; '
            'q and q̇ are unfiltered; default: 11)'
        ),
    )
    parser.add_argument(
        '--measured-kinematics-lowpass-window',
        type=int,
        default=None,
        metavar='N',
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        '--show-phases',
        action='store_true',
        help='Overlay yaml-derived motion phase backgrounds (optional)',
    )
    parser.add_argument(
        '--config',
        type=Path,
        default=None,
        help='Excitation yaml for --show-phases only',
    )
    parser.add_argument(
        '--periods',
        type=int,
        default=3,
        help='Excitation periods for --show-phases only',
    )
    parser.add_argument('--dt', type=float, default=0.01, help='Schedule resample period for phases')
    parser.add_argument(
        '--motion-start-s',
        type=float,
        default=None,
        help='Bag-relative motion t=0 for --show-phases (auto if omitted)',
    )
    parser.add_argument(
        '--velocity-threshold',
        type=float,
        default=0.02,
        help='Auto motion-start detection threshold for --show-phases (rad/s)',
    )
    parsed = parser.parse_args(args)
    if parsed.measured_kinematics_lowpass_window is not None:
        parsed.measured_acceleration_lowpass_window = (
            parsed.measured_kinematics_lowpass_window
        )
    if parsed.measured_acceleration_lowpass_window < 0:
        parser.error('--measured-acceleration-lowpass-window must be >= 0')

    require_excitation_recording_bag(parsed.bag)

    measured_times, measured_arrays = joint_states_from_bag(parsed.bag)
    commanded_times, commanded_arrays = commanded_reference_from_bag(parsed.bag)

    aligned_times, aligned_measured, aligned_commanded = align_commanded_to_measured(
        measured_times,
        measured_arrays['position'],
        commanded_times,
        commanded_arrays['position'],
    )

    bag_time_origin_s = float(measured_times[0])
    title_prefix = f'{parsed.bag.name}: '

    phase_spans = None
    if parsed.show_phases:
        config_path, config = load_config(parsed.config)
        trajectory = ExcitationTrajectory.from_config(config)
        bag_relative = measured_times - measured_times[0]
        if parsed.motion_start_s is None:
            motion_start_s = detect_motion_start_s(
                bag_relative,
                measured_arrays['position'],
                velocity_threshold_rad_s=parsed.velocity_threshold,
            )
        else:
            motion_start_s = float(parsed.motion_start_s)

        start_positions = _start_positions_near_motion(
            bag_relative,
            measured_arrays['position'],
            motion_start_s,
        )
        _reference_times, _reference_positions, schedule = build_reference_runtime(
            trajectory,
            start_positions=start_positions,
            num_periods=parsed.periods,
            sample_dt_s=parsed.dt,
            config=config,
        )
        phase_spans = _phase_spans_on_bag_axis(schedule, motion_start_s)
        print(f'phase config: {config_path}')
        print(f'motion_start_s (bag-relative, phases only): {motion_start_s:.3f}')

    written = plot_joint_position_tracking(
        aligned_times_s=aligned_times,
        measured_positions=aligned_measured,
        commanded_positions=aligned_commanded,
        output_dir=parsed.output_dir,
        bag_time_origin_s=bag_time_origin_s,
        title_prefix=title_prefix,
        phase_spans=phase_spans,
    )

    if not parsed.skip_commanded_kinematics:
        measured_kinematics = None
        if not parsed.skip_measured_kinematics_overlay:
            position, velocity, acceleration = filtered_measured_kinematics_on_times(
                measured_times,
                measured_arrays['position'],
                measured_arrays['velocity'],
                commanded_times,
                sample_dt_s=parsed.dt,
                acceleration_smooth_window=parsed.measured_acceleration_lowpass_window,
            )
            measured_kinematics = {
                'position': position,
                'velocity': velocity,
                'acceleration': acceleration,
            }

        written.extend(
            plot_joint_commanded_kinematics(
                commanded_times_s=commanded_times,
                commanded_arrays=commanded_arrays,
                output_dir=parsed.output_dir,
                bag_time_origin_s=bag_time_origin_s,
                title_prefix=title_prefix,
                phase_spans=phase_spans,
                measured_arrays=measured_kinematics,
                measured_acceleration_lowpass_window=(
                    parsed.measured_acceleration_lowpass_window
                ),
            )
        )

    overlap_duration = float(aligned_times[-1] - aligned_times[0])
    print(f'aligned samples: {aligned_times.size}, overlap duration: {overlap_duration:.1f} s')
    for path in written:
        print(f'Wrote {path}')


if __name__ == '__main__':
    main()
