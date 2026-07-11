"""Offline model validation: BIP vs URDF comparison and torque prediction from bags."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from io import StringIO
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from open_manipulator_sysid.excitation_recording.bag_reader import bag_to_dataset
from open_manipulator_sysid.five_link_dynamics.base_parameters import (
    FiveLinkBaseParameterSet,
    five_link_std_layout,
    reference_base_values,
)
from open_manipulator_sysid.system_identification.identify import (
    build_sample_kinematics,
    identification_trim_times,
    predict_torque_batch,
    run_identification,
)
from open_manipulator_sysid.excitation_trajectory import default_config_path
from open_manipulator_sysid.excitation_recording.dataset import IdentificationDataset
from open_manipulator_sysid.pinocchio_support.model_loader import (
    arm_velocity_indices,
    require_pinocchio,
    resolve_model_load_args,
)
from open_manipulator_sysid.system_identification.friction import (
    FRICTION_PARAM_NAMES,
    pack_friction_vector,
)


@dataclass(frozen=True)
class BipComparisonRow:
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
class CalibratedModel:
    base_set: FiveLinkBaseParameterSet
    base_parameters: np.ndarray
    reference_values: np.ndarray
    friction_parameters: np.ndarray


def load_identified_yaml(
    path: Path | str,
    *,
    model,
) -> Tuple[CalibratedModel, Dict[str, object]]:
    """Load dynamics_identified.yaml produced by dynamics_identify."""
    with Path(path).open('r', encoding='utf-8') as handle:
        payload = yaml.safe_load(handle) or {}
    base_block = payload.get('five_link_base_parameters')
    if base_block is None:
        if payload.get('base_parameters'):
            raise ValueError(
                'Legacy QR base_parameters yaml is no longer supported; '
                're-run sysid_identify_bag to produce five_link_base_parameters.'
            )
        raise ValueError('dynamics_identified.yaml missing five_link_base_parameters block')

    layout = five_link_std_layout(model)
    base_set = FiveLinkBaseParameterSet.from_dict(base_block, layout)
    base_values = np.asarray(base_block.get('identified_values', []), dtype=float)
    if base_values.size != base_set.num_base:
        raise ValueError(
            f'identified_values length {base_values.size} '
            f'does not match rank {base_set.num_base}'
        )
    reference_values = np.asarray(base_block.get('reference_values', []), dtype=float)
    if reference_values.size != base_set.num_base:
        reference_values = reference_base_values(model, base_set)

    friction_block = payload.get('friction') or {}
    joints = friction_block.get('joints')
    if joints is None:
        raise ValueError('dynamics_identified.yaml missing friction.joints block')
    rows = []
    for joint_name in ('joint1', 'joint2', 'joint3', 'joint4'):
        joint_params = joints.get(joint_name, {})
        rows.append([float(joint_params[name]) for name in FRICTION_PARAM_NAMES])
    friction = pack_friction_vector(np.asarray(rows, dtype=float))
    return (
        CalibratedModel(
            base_set=base_set,
            base_parameters=base_values,
            reference_values=reference_values,
            friction_parameters=friction,
        ),
        payload,
    )


def build_bip_comparison_rows(
    model,
    base_set: FiveLinkBaseParameterSet,
    identified_values: Sequence[float],
    reference_values: Optional[Sequence[float]] = None,
) -> List[BipComparisonRow]:
    if reference_values is None:
        reference_values = reference_base_values(model, base_set)
    expressions = base_set.symbolic_expressions()
    units = base_set.units()
    if len(expressions) != len(identified_values):
        raise ValueError('Expression count does not match identified value count')
    rows: List[BipComparisonRow] = []
    for index, (expr, unit, ref, identified) in enumerate(
        zip(expressions, units, reference_values, identified_values),
        start=1,
    ):
        rows.append(
            BipComparisonRow(
                index=index,
                expression=expr,
                unit=unit,
                reference=float(ref),
                identified=float(identified),
            )
        )
    return rows


def format_comparison_markdown(rows: Sequence[BipComparisonRow]) -> str:
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


def write_comparison_csv(path: Path | str, rows: Sequence[BipComparisonRow]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w', encoding='utf-8', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ['index', 'expression', 'unit', 'reference', 'identified', 'delta', 'relative_error_pct']
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


def load_dataset_from_bag(
    bag_path: Path | str,
    *,
    excitation_config: Path | str,
    sample_dt_s: float = 0.01,
    excitation_periods: int = 5,
    trim_start_s: Optional[float] = None,
    trim_end_s: Optional[float] = None,
) -> IdentificationDataset:
    with Path(excitation_config).open('r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle) or {}
    if trim_start_s is None or trim_end_s is None:
        default_start, default_end = identification_trim_times(
            config,
            num_periods=excitation_periods,
        )
        trim_start_s = default_start if trim_start_s is None else trim_start_s
        trim_end_s = default_end if trim_end_s is None else trim_end_s
    return bag_to_dataset(
        bag_path,
        sample_dt_s=sample_dt_s,
        trim_start_s=trim_start_s,
        trim_end_s=trim_end_s,
        trim_end_reference='bag_start',
    )


def identify_or_load_model(
    *,
    dataset: IdentificationDataset,
    identified_yaml: Optional[Path | str],
    urdf_path: Optional[Path | str] = None,
    xacro_path: Optional[Path | str] = None,
    xacro_mappings: Optional[Mapping[str, str]] = None,
    excitation_config: Path | str,
    dataset_source: str,
) -> Tuple[object, object, CalibratedModel]:
    """Return (model, data, calibrated_model) from yaml or fresh identification."""
    require_pinocchio()
    model_path, load_mappings, _ = resolve_model_load_args(
        urdf_path=urdf_path,
        xacro_path=xacro_path,
        xacro_mappings=xacro_mappings,
    )
    from open_manipulator_sysid.pinocchio_support.model_loader import load_model

    model, data = load_model(model_path, xacro_mappings=load_mappings)

    if identified_yaml is not None:
        calibrated, _payload = load_identified_yaml(identified_yaml, model=model)
        return model, data, calibrated

    result = run_identification(
        dataset,
        urdf_path=urdf_path,
        xacro_path=xacro_path,
        xacro_mappings=xacro_mappings,
        excitation_config=excitation_config,
        output_dir=None,
        dataset_source=dataset_source,
    )
    calibrated = CalibratedModel(
        base_set=result.base_set,
        base_parameters=result.base_parameters,
        reference_values=result.reference_values,
        friction_parameters=result.friction_parameters,
    )
    return model, data, calibrated


def predict_torque_timeseries(
    model,
    data,
    dataset: IdentificationDataset,
    calibrated: CalibratedModel,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return (measured, predicted) torque arrays with shape (num_samples, 4)."""
    arm_indices = arm_velocity_indices(model)
    samples = build_sample_kinematics(
        model,
        data,
        dataset,
        calibrated.base_set,
        arm_indices,
    )
    predicted_flat = predict_torque_batch(
        calibrated.base_parameters,
        calibrated.friction_parameters,
        samples,
    )
    predicted = predicted_flat.reshape(dataset.num_samples, len(arm_indices))
    measured = dataset.torque.copy()
    return measured, predicted


@dataclass(frozen=True)
class TorqueFitMetrics:
    rmse_per_joint: Tuple[float, float, float, float]
    rmse_overall: float
    nominal_rmse_per_joint: Tuple[float, float, float, float]
    nominal_rmse_overall: float
    num_samples: int
    duration_s: float


def compute_torque_fit_metrics(
    model,
    data,
    dataset: IdentificationDataset,
    calibrated: CalibratedModel,
) -> TorqueFitMetrics:
    """Torque RMSE for calibrated model and URDF-nominal BIP (same friction)."""
    measured, predicted = predict_torque_timeseries(model, data, dataset, calibrated)
    rmse_joint = tuple(
        float(np.sqrt(np.mean((measured[:, joint] - predicted[:, joint]) ** 2)))
        for joint in range(measured.shape[1])
    )
    rmse_overall = float(np.sqrt(np.mean((measured - predicted) ** 2)))

    arm_indices = arm_velocity_indices(model)
    samples = build_sample_kinematics(
        model,
        data,
        dataset,
        calibrated.base_set,
        arm_indices,
    )
    pi_base_nominal = calibrated.reference_values
    predicted_nominal = predict_torque_batch(
        pi_base_nominal,
        calibrated.friction_parameters,
        samples,
    ).reshape(dataset.num_samples, len(arm_indices))
    nominal_rmse_joint = tuple(
        float(np.sqrt(np.mean((measured[:, joint] - predicted_nominal[:, joint]) ** 2)))
        for joint in range(measured.shape[1])
    )
    nominal_rmse_overall = float(np.sqrt(np.mean((measured - predicted_nominal) ** 2)))

    duration_s = (
        float(dataset.times_s[-1] - dataset.times_s[0])
        if dataset.num_samples > 1
        else 0.0
    )
    return TorqueFitMetrics(
        rmse_per_joint=rmse_joint,
        rmse_overall=rmse_overall,
        nominal_rmse_per_joint=nominal_rmse_joint,
        nominal_rmse_overall=nominal_rmse_overall,
        num_samples=dataset.num_samples,
        duration_s=duration_s,
    )


BIP_COLUMN_COMPARE_NOTE = (
    'Note: five-link symbolic BIP rows are linear combinations of standard inertia '
    'columns; per-row gaps vs θ_ref are expected even when torque fit is good. '
    'Use torque RMSE below as the primary validation metric.'
)


def format_torque_fit_summary(metrics: TorqueFitMetrics) -> str:
    joint_labels = ('joint1', 'joint2', 'joint3', 'joint4')
    lines = [
        BIP_COLUMN_COMPARE_NOTE,
        (
            f'Dataset window: {metrics.num_samples} samples, '
            f'{metrics.duration_s:.1f} s'
        ),
        f'Calibrated torque RMSE (N·m): overall={metrics.rmse_overall:.4f}  '
        + '  '.join(
            f'{label}={value:.4f}'
            for label, value in zip(joint_labels, metrics.rmse_per_joint)
        ),
        f'URDF-nominal BIP + same friction RMSE (N·m): overall={metrics.nominal_rmse_overall:.4f}  '
        + '  '.join(
            f'{label}={value:.4f}'
            for label, value in zip(joint_labels, metrics.nominal_rmse_per_joint)
        ),
    ]
    return '\n'.join(lines)


def default_test_config_path() -> Path:
    source = Path(__file__).resolve().parents[1] / 'config' / 'test_trajectory.yaml'
    if source.is_file():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = (
            Path(get_package_share_directory('open_manipulator_sysid'))
            / 'config'
            / 'test_trajectory.yaml'
        )
        if installed.is_file():
            return installed
    except Exception:
        pass
    return default_config_path()
