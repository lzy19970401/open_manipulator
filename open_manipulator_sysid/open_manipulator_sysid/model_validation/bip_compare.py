#!/usr/bin/env python3
"""Compare identified BIP values against URDF nominal parameters from a sysid bag."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Sequence

from open_manipulator_sysid.model_validation.evaluation import (
    build_bip_comparison_rows,
    compute_torque_fit_metrics,
    default_test_config_path,
    format_comparison_markdown,
    format_torque_fit_summary,
    identify_or_load_model,
    load_dataset_from_bag,
    write_comparison_csv,
)
from open_manipulator_sysid.pinocchio_support.model_loader import (
    _add_model_source_arguments,
    _model_source_kwargs_from_parsed,
)


def main(args: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Identify BIP from a sysid/test bag (or load existing yaml) and '
            'compare against URDF nominal inertial parameters.'
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
    parser.add_argument('--periods', type=int, default=2, help='Excitation periods used in the bag')
    parser.add_argument('--dt', type=float, default=0.01)
    parser.add_argument('--trim-start', type=float, default=None)
    parser.add_argument('--trim-end', type=float, default=None)
    parser.add_argument(
        '--output-csv',
        type=Path,
        default=None,
        help='Optional CSV path for the comparison table',
    )
    parsed = parser.parse_args(args)

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
    rows = build_bip_comparison_rows(
        model,
        calibrated.base_set,
        calibrated.base_parameters,
        calibrated.reference_values,
    )
    metrics = compute_torque_fit_metrics(model, data, dataset, calibrated)
    print(format_torque_fit_summary(metrics))
    print()
    table = format_comparison_markdown(rows)
    print(table)
    if parsed.output_csv is not None:
        write_comparison_csv(parsed.output_csv, rows)
        print(f'\nWrote CSV: {parsed.output_csv}')


if __name__ == '__main__':
    main()
