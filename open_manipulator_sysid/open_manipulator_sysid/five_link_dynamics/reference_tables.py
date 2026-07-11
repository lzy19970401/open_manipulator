"""Five-link BIP reference tables and bag identification (canonical BIP path)."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from open_manipulator_sysid.system_identification.identify import (
    IdentificationResult,
    default_results_dir,
    run_identification,
)
from open_manipulator_sysid.excitation_trajectory import default_config_path
from open_manipulator_sysid.five_link_dynamics.base_parameters import (
    FiveLinkBaseParameterSet,
    five_link_base_set_from_trajectory,
    five_link_std_layout,
    reference_base_values,
)
from open_manipulator_sysid.excitation_recording.dataset import IdentificationDataset
from open_manipulator_sysid.model_validation.evaluation import load_dataset_from_bag
from open_manipulator_sysid.pinocchio_support.model_loader import (
    _add_model_source_arguments,
    _model_source_kwargs_from_parsed,
    load_model,
    require_pinocchio,
    resolve_model_load_args,
)
from open_manipulator_sysid.system_identification.friction import friction_params_to_dict


@dataclass(frozen=True)
class FiveLinkComparisonRow:
    index: int
    expression: str
    unit: str
    reference: float
    identified: float

    @property
    def delta(self) -> float:
        return self.identified - self.reference

    @property
    def relative_error_pct(self) -> float:
        denom = abs(self.reference)
        if denom < 1e-12:
            return float('nan')
        return 100.0 * self.delta / denom


@dataclass(frozen=True)
class FiveLinkIdentificationResult:
    base_set: FiveLinkBaseParameterSet
    reference_values: np.ndarray
    identified_values: np.ndarray
    friction_parameters: np.ndarray
    residual_norm: float
    num_samples: int


def derive_five_link_reference_table(
    *,
    urdf_path: Optional[Path | str] = None,
    xacro_path: Optional[Path | str] = None,
    xacro_mappings: Optional[Mapping[str, str]] = None,
    excitation_config: Path | str,
) -> Tuple[object, FiveLinkBaseParameterSet, np.ndarray]:
    require_pinocchio()
    model_path, load_mappings, _ = resolve_model_load_args(
        urdf_path=urdf_path,
        xacro_path=xacro_path,
        xacro_mappings=xacro_mappings,
    )
    model, data = load_model(model_path, xacro_mappings=load_mappings)
    from open_manipulator_sysid.excitation_trajectory import ExcitationTrajectory

    with Path(excitation_config).open('r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle) or {}
    trajectory = ExcitationTrajectory.from_config(config)
    base_set = five_link_base_set_from_trajectory(trajectory, model, data)
    ref_values = reference_base_values(model, base_set)
    return model, base_set, ref_values


def run_five_link_identification(
    dataset: IdentificationDataset,
    *,
    urdf_path: Optional[Path | str] = None,
    xacro_path: Optional[Path | str] = None,
    xacro_mappings: Optional[Mapping[str, str]] = None,
    excitation_config: Path | str,
    base_set: Optional[FiveLinkBaseParameterSet] = None,
) -> FiveLinkIdentificationResult:
    result: IdentificationResult = run_identification(
        dataset,
        urdf_path=urdf_path,
        xacro_path=xacro_path,
        xacro_mappings=xacro_mappings,
        excitation_config=excitation_config,
        output_dir=None,
        dataset_source='five_link_identification',
        base_set=base_set,
    )
    return FiveLinkIdentificationResult(
        base_set=result.base_set,
        reference_values=result.reference_values,
        identified_values=result.base_parameters,
        friction_parameters=result.friction_parameters,
        residual_norm=result.residual_norm,
        num_samples=result.num_samples,
    )


def build_comparison_rows(
    base_set: FiveLinkBaseParameterSet,
    reference_values: Sequence[float],
    identified_values: Sequence[float],
) -> List[FiveLinkComparisonRow]:
    expressions = base_set.symbolic_expressions()
    units = base_set.units()
    rows: List[FiveLinkComparisonRow] = []
    for index, (expr, unit, ref, ident) in enumerate(
        zip(expressions, units, reference_values, identified_values),
        start=1,
    ):
        rows.append(
            FiveLinkComparisonRow(
                index=index,
                expression=expr,
                unit=unit,
                reference=float(ref),
                identified=float(ident),
            )
        )
    return rows


def format_reference_markdown(
    base_set: FiveLinkBaseParameterSet,
    reference_values: Sequence[float],
) -> str:
    expressions = base_set.symbolic_expressions()
    units = base_set.units()
    lines = [
        '| i | Physical meaning | Unit | θ (URDF reference) |',
        '|---|------------------|------|---------------------|',
    ]
    for index, (expr, unit, value) in enumerate(
        zip(expressions, units, reference_values),
        start=1,
    ):
        lines.append(f'| {index} | {expr} | {unit} | {float(value):.6g} |')
    return '\n'.join(lines)


def format_comparison_markdown(rows: Sequence[FiveLinkComparisonRow]) -> str:
    lines = [
        '| i | Physical meaning | Unit | θ_ref | θ_id | Δ | Rel. error % |',
        '|---|------------------|------|-------|------|---|--------------|',
    ]
    for row in rows:
        rel = (
            f'{row.relative_error_pct:.2f}'
            if np.isfinite(row.relative_error_pct)
            else 'n/a'
        )
        lines.append(
            f'| {row.index} | {row.expression} | {row.unit} | {row.reference:.6g} | '
            f'{row.identified:.6g} | {row.delta:.6g} | {rel} |'
        )
    return '\n'.join(lines)


def write_reference_yaml(
    path: Path | str,
    *,
    model_source: str,
    excitation_config: Path | str,
    base_set: FiveLinkBaseParameterSet,
    reference_values: Sequence[float],
) -> None:
    payload = {
        'description': (
            'Five-link (link1–link5) symbolic base inertial parameters derived from '
            'OMX URDF. Gripper and end_effector inertias excluded.'
        ),
        'model_source': model_source,
        'excitation_config': str(excitation_config),
        'five_link_base_parameters': {
            **base_set.to_dict(),
            'reference_values': [float(value) for value in reference_values],
        },
    }
    with Path(path).open('w', encoding='utf-8') as handle:
        yaml.safe_dump(payload, handle, sort_keys=False, default_flow_style=False)


def write_identified_yaml(
    path: Path | str,
    *,
    model_source: str,
    excitation_config: Path | str,
    dataset_source: str,
    result: FiveLinkIdentificationResult,
) -> None:
    rows = build_comparison_rows(
        result.base_set,
        result.reference_values,
        result.identified_values,
    )
    friction_dict = friction_params_to_dict(result.friction_parameters)
    payload = {
        'description': 'Five-link symbolic BIP identification from bag (gripper excluded).',
        'model_source': model_source,
        'excitation_config': str(excitation_config),
        'dataset_source': dataset_source,
        'five_link_base_parameters': {
            **result.base_set.to_dict(),
            'reference_values': [float(value) for value in result.reference_values],
            'identified_values': [float(value) for value in result.identified_values],
        },
        'comparison': [
            {
                'index': row.index,
                'expression': row.expression,
                'unit': row.unit,
                'reference': row.reference,
                'identified': row.identified,
                'delta': row.delta,
                'relative_error_pct': row.relative_error_pct,
            }
            for row in rows
        ],
        'optimizer': {
            'residual_norm': result.residual_norm,
            'num_samples': result.num_samples,
        },
        'friction': {'joints': friction_dict},
    }
    with Path(path).open('w', encoding='utf-8') as handle:
        yaml.safe_dump(payload, handle, sort_keys=False, default_flow_style=False)


def write_comparison_csv(path: Path | str, rows: Sequence[FiveLinkComparisonRow]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w', encoding='utf-8', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                'index',
                'expression',
                'unit',
                'reference',
                'identified',
                'delta',
                'relative_error_pct',
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    row.index,
                    row.expression,
                    row.unit,
                    row.reference,
                    row.identified,
                    row.delta,
                    row.relative_error_pct,
                ]
            )


def main_reference(args: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Derive five-link symbolic base parameters from OMX URDF and print '
            'Swevers-style reference table.'
        )
    )
    _add_model_source_arguments(parser)
    parser.add_argument('--config', type=Path, default=None)
    parser.add_argument('--periods', type=int, default=5)
    parser.add_argument(
        '--output-yaml',
        type=Path,
        default=Path('/workspace/sysid_results/five_link_reference.yaml'),
    )
    parsed = parser.parse_args(args)

    config_path = parsed.config or default_config_path()
    _, _, model_source = resolve_model_load_args(**_model_source_kwargs_from_parsed(parsed))
    _, base_set, ref_values = derive_five_link_reference_table(
        **_model_source_kwargs_from_parsed(parsed),
        excitation_config=config_path,
    )
    print(format_reference_markdown(base_set, ref_values))
    write_reference_yaml(
        parsed.output_yaml,
        model_source=str(model_source),
        excitation_config=config_path,
        base_set=base_set,
        reference_values=ref_values,
    )
    print(f'\nWrote reference yaml: {parsed.output_yaml}')


def main_identify_compare(args: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Identify five-link symbolic BIP from a sysid bag and compare with '
            'URDF reference combinations.'
        )
    )
    _add_model_source_arguments(parser)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=None)
    parser.add_argument('--periods', type=int, default=5)
    parser.add_argument('--dt', type=float, default=0.01)
    parser.add_argument('--reference-yaml', type=Path, default=None)
    parser.add_argument('--output-dir', type=Path, default=None)
    parser.add_argument('--output-csv', type=Path, default=None)
    parsed = parser.parse_args(args)

    config_path = parsed.config or default_config_path()
    output_dir = parsed.output_dir or default_results_dir()
    dataset = load_dataset_from_bag(
        parsed.bag,
        excitation_config=config_path,
        sample_dt_s=parsed.dt,
        excitation_periods=parsed.periods,
    )

    model_path, load_mappings, model_source = resolve_model_load_args(
        **_model_source_kwargs_from_parsed(parsed)
    )
    model, _data = load_model(model_path, xacro_mappings=load_mappings)
    base_set: Optional[FiveLinkBaseParameterSet] = None
    if parsed.reference_yaml is not None:
        with Path(parsed.reference_yaml).open('r', encoding='utf-8') as handle:
            ref_payload = yaml.safe_load(handle) or {}
        layout = five_link_std_layout(model)
        base_set = FiveLinkBaseParameterSet.from_dict(
            ref_payload['five_link_base_parameters'],
            layout,
        )

    result = run_five_link_identification(
        dataset,
        **_model_source_kwargs_from_parsed(parsed),
        excitation_config=config_path,
        base_set=base_set,
    )
    rows = build_comparison_rows(
        result.base_set,
        result.reference_values,
        result.identified_values,
    )
    print(format_comparison_markdown(rows))
    print(f'\nTorque residual_norm: {result.residual_norm:.6g}')

    output_dir.mkdir(parents=True, exist_ok=True)
    write_identified_yaml(
        output_dir / 'five_link_identified.yaml',
        model_source=str(model_source),
        excitation_config=config_path,
        dataset_source=str(parsed.bag),
        result=result,
    )
    print(f'Wrote {output_dir / "five_link_identified.yaml"}')
    if parsed.output_csv is not None:
        write_comparison_csv(parsed.output_csv, rows)
        print(f'Wrote CSV: {parsed.output_csv}')
