#!/usr/bin/env python3
"""Plot one-period OMX Swevers Fourier excitation (all joints on one figure)."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from open_manipulator_sysid.excitation_trajectory import (
    ARM_JOINTS,
    ExcitationTrajectory,
    default_config_path,
)


def _require_matplotlib():
    try:
        import matplotlib
    except ImportError as exc:  # pragma: no cover
        raise ImportError(
            'matplotlib is required for excitation trajectory plots. '
            'Install python3-matplotlib in the Docker container.'
        ) from exc
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    return plt


def sample_one_period_positions(
    trajectory: ExcitationTrajectory,
    *,
    num_samples: int = 500,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (times_s, positions) with shape (N,) and (N, 4)."""
    if num_samples < 2:
        raise ValueError('num_samples must be at least 2')
    times_s = np.linspace(0.0, trajectory.period_s, num_samples, dtype=float)
    positions = np.zeros((num_samples, len(ARM_JOINTS)), dtype=float)
    for index, time_s in enumerate(times_s):
        sample = trajectory.sample(float(time_s))
        for joint_index, joint in enumerate(ARM_JOINTS):
            positions[index, joint_index] = sample.position[joint]
    return times_s, positions


def plot_excitation_trajectory(
    trajectory: ExcitationTrajectory,
    *,
    output_path: Path,
    num_samples: int = 500,
    title: Optional[str] = None,
) -> Path:
    """Plot joint positions q(t) for one excitation period (all joints, one figure)."""
    plt = _require_matplotlib()
    times_s, positions = sample_one_period_positions(trajectory, num_samples=num_samples)

    figure, axis = plt.subplots(figsize=(11, 5))
    for joint_index, joint in enumerate(ARM_JOINTS):
        axis.plot(
            times_s,
            positions[:, joint_index],
            label=joint,
            linewidth=1.4,
        )

    axis.set_xlabel('time t (s)')
    axis.set_ylabel('joint position q (rad)')
    axis.set_xlim(0.0, trajectory.period_s)
    axis.grid(True, alpha=0.3)
    axis.legend(loc='best')

    if title is None:
        title = (
            f'Excitation trajectory (1 period, T={trajectory.period_s:.2f} s, '
            f'source={trajectory.coefficients_source})'
        )
    axis.set_title(title)

    figure.tight_layout()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=150)
    plt.close(figure)
    return output_path


def main(args: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Plot one-period Swevers Fourier excitation for all arm joints '
            '(single figure, q vs time).'
        )
    )
    parser.add_argument(
        '--config',
        type=Path,
        default=None,
        help='Excitation yaml with ga_coefficients (default: excitation.yaml)',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('/workspace/sysid_results/excitation_trajectory_one_period.png'),
        help='Output PNG path',
    )
    parser.add_argument(
        '--samples',
        type=int,
        default=500,
        help='Number of time samples over one period (default: 500)',
    )
    parsed = parser.parse_args(args)

    config_path = parsed.config or default_config_path()
    trajectory = ExcitationTrajectory.from_yaml(config_path)
    trajectory.validate_period()

    written = plot_excitation_trajectory(
        trajectory,
        output_path=parsed.output,
        num_samples=parsed.samples,
        title=f'Excitation trajectory — {config_path.name}',
    )
    print(f'config: {config_path}')
    print(f'period: {trajectory.period_s:.3f} s')
    print(f'coefficient_scale: {trajectory.coefficient_scale:.4f}')
    print(f'Wrote: {written}')


if __name__ == '__main__':
    main()
