#!/usr/bin/env python3
"""Plot measured vs calibrated-model predicted joint torques (one figure per joint)."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from open_manipulator_sysid.excitation_trajectory import ARM_JOINTS
from open_manipulator_sysid.excitation_recording.dataset import lowpass_moving_average
from open_manipulator_sysid.model_validation.evaluation import (
    default_test_config_path,
    identify_or_load_model,
    load_dataset_from_bag,
    predict_torque_timeseries,
)
from open_manipulator_sysid.pinocchio_support.model_loader import (
    _add_model_source_arguments,
    _model_source_kwargs_from_parsed,
)


def _require_matplotlib():
    try:
        import matplotlib
    except ImportError as exc:  # pragma: no cover
        raise ImportError(
            'matplotlib is required for torque plots. '
            'Install python3-matplotlib in the Docker container.'
        ) from exc
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    return plt


def plot_joint_torques(
    *,
    times_s,
    measured,
    predicted,
    output_dir: Path,
    title_prefix: str = '',
    measured_torque_lowpass_window: int = 0,
) -> list[Path]:
    plt = _require_matplotlib()
    output_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    measured_plot = np.asarray(measured, dtype=float)
    if measured_torque_lowpass_window > 1:
        measured_plot = lowpass_moving_average(
            measured_plot,
            measured_torque_lowpass_window,
        )
        measured_label = (
            f'measured (low-pass, window={measured_torque_lowpass_window})'
        )
    else:
        measured_label = 'measured'

    for joint_index, joint_name in enumerate(ARM_JOINTS):
        figure, axis = plt.subplots(figsize=(10, 4))
        axis.plot(
            times_s,
            measured_plot[:, joint_index],
            label=measured_label,
            linewidth=1.2,
        )
        axis.plot(
            times_s,
            predicted[:, joint_index],
            label='predicted (calibrated model)',
            linewidth=1.2,
            linestyle='--',
        )
        axis.set_xlabel('time (s)')
        axis.set_ylabel('torque (N·m)')
        title = f'{title_prefix}{joint_name}: measured vs predicted torque'.strip()
        axis.set_title(title)
        axis.legend(loc='best')
        axis.grid(True, alpha=0.3)
        figure.tight_layout()
        out_path = output_dir / f'{joint_name}_torque_compare.png'
        figure.savefig(out_path, dpi=150)
        plt.close(figure)
        written.append(out_path)
    return written


def main(args: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Plot measured joint torques from a bag against the calibrated '
            'dynamics model prediction (BIP + Coulomb/viscous friction). Writes four PNG files.'
        )
    )
    _add_model_source_arguments(parser)
    parser.add_argument('--bag', type=Path, required=True, help='Rosbag2 directory')
    parser.add_argument(
        '--identified-yaml',
        type=Path,
        default=None,
        help='Use existing dynamics_identified.yaml instead of re-identifying',
    )
    parser.add_argument(
        '--config',
        type=Path,
        default=None,
        help='Excitation yaml for trim windows (default: test_trajectory.yaml)',
    )
    parser.add_argument('--periods', type=int, default=2)
    parser.add_argument('--dt', type=float, default=0.01)
    parser.add_argument('--trim-start', type=float, default=None)
    parser.add_argument('--trim-end', type=float, default=None)
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path('/workspace/sysid_results/torque_plots'),
        help='Directory for joint1..joint4 PNG outputs',
    )
    parser.add_argument(
        '--measured-torque-lowpass-window',
        type=int,
        default=0,
        metavar='N',
        help=(
            'Moving-average low-pass on measured torque for plotting only '
            '(0 or 1 = off; try 5–15 at dt=0.01 s). Does not affect identification.'
        ),
    )
    parsed = parser.parse_args(args)
    if parsed.measured_torque_lowpass_window < 0:
        parser.error('--measured-torque-lowpass-window must be >= 0')

    config_path = parsed.config or default_test_config_path()
    dataset = load_dataset_from_bag(
        parsed.bag,
        excitation_config=config_path,
        sample_dt_s=parsed.dt,
        excitation_periods=parsed.periods,
        trim_start_s=parsed.trim_start,
        trim_end_s=parsed.trim_end,
    )
    model, data, calibrated = identify_or_load_model(
        dataset=dataset,
        identified_yaml=parsed.identified_yaml,
        excitation_config=config_path,
        dataset_source=str(parsed.bag),
        **_model_source_kwargs_from_parsed(parsed),
    )
    measured, predicted = predict_torque_timeseries(model, data, dataset, calibrated)
    written = plot_joint_torques(
        times_s=dataset.times_s,
        measured=measured,
        predicted=predicted,
        output_dir=parsed.output_dir,
        title_prefix=f'{parsed.bag.name}: ',
        measured_torque_lowpass_window=parsed.measured_torque_lowpass_window,
    )
    for path in written:
        print(f'Wrote {path}')


if __name__ == '__main__':
    main()
