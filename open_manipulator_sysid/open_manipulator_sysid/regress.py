"""Offline system identification: fixed URDF inertia + friction least squares."""

from __future__ import annotations

import argparse
import html
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from open_manipulator_sysid.bag_to_dataset import bag_to_dataset
from open_manipulator_sysid.identification_dataset import (
    IdentificationDataset,
    MAX_REASONABLE_FRICTION,
    validate_identification_dataset,
)
from open_manipulator_sysid.excitation_trajectory import (
    ARM_JOINTS,
    ExcitationTrajectory,
    default_config_path,
)

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover - exercised in Docker with Pinocchio installed
    pin = None
    _PINOCCHIO_IMPORT_ERROR = exc
else:
    _PINOCCHIO_IMPORT_ERROR = None

INERTIA_PARAM_NAMES = (
    'mass',
    'mc_x',
    'mc_y',
    'mc_z',
    'Ixx',
    'Ixy',
    'Iyy',
    'Ixz',
    'Iyz',
    'Izz',
)
SVD_RANK_RTOL = 1e-6
NUM_FRICTION_PARAMS = len(ARM_JOINTS)


@dataclass(frozen=True)
class SyntheticFrictionTruth:
    """Ground-truth viscous friction used to synthesize torque measurements."""

    viscous: Dict[str, float]


@dataclass(frozen=True)
class RegressionResult:
    viscous: np.ndarray
    residual_norm: float
    condition_number: float
    matrix_rank: int
    num_samples: int
    sample_period_s: float
    sample_dt_s: float


def require_pinocchio() -> None:
    if pin is None:
        raise ImportError(
            'Pinocchio is required for offline regression. '
            'Install ros-jazzy-pinocchio inside the Docker container.'
        ) from _PINOCCHIO_IMPORT_ERROR


def _description_share_subpath(*parts: str) -> Path:
    """Resolve open_manipulator_description/... in source tree or install share."""
    source_path = Path(__file__).resolve().parents[2] / 'open_manipulator_description'
    candidate = source_path.joinpath(*parts)
    if candidate.is_file():
        return candidate

    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory('open_manipulator_description'))
        installed = share.joinpath(*parts)
        if installed.is_file():
            return installed
    except Exception:
        pass

    raise FileNotFoundError(
        f'open_manipulator_description file not found: {" / ".join(parts)}'
    )


def default_xacro_path() -> Path:
    """Return bundled OpenMANIPULATOR-X top-level xacro (source tree or install share)."""
    return _description_share_subpath(
        'urdf',
        'open_manipulator_x',
        'open_manipulator_x.urdf.xacro',
    )


def default_urdf_path() -> Path:
    """Return bundled static OpenMANIPULATOR-X URDF (legacy override via --urdf)."""
    return _description_share_subpath(
        'urdf',
        'open_manipulator_x',
        'open_manipulator_x.urdf',
    )


def default_xacro_mappings() -> Dict[str, str]:
    """Default xacro args aligned with hardware excitation launch (Pinocchio uses inertial only)."""
    return {
        'prefix': '',
        'use_sim': 'false',
        'use_mock_hardware': 'false',
        'mock_sensor_commands': 'false',
        'port_name': '/dev/ttyUSB0',
        'ros2_control_type': 'open_manipulator_x_position',
    }


def expand_xacro(
    xacro_path: Path | str,
    mappings: Optional[Mapping[str, str]] = None,
) -> str:
    import xacro

    resolved_mappings = dict(default_xacro_mappings())
    if mappings:
        resolved_mappings.update(mappings)
    document = xacro.process_file(str(xacro_path), mappings=resolved_mappings)
    return document.toprettyxml(indent='  ')


def format_model_source(
    model_path: Path,
    *,
    xacro_mappings: Optional[Mapping[str, str]] = None,
) -> str:
    if xacro_mappings is None:
        return str(model_path)
    pairs = ' '.join(f'{key}:={value}' for key, value in sorted(xacro_mappings.items()))
    return f'{model_path} ({pairs})'


DEFAULT_RESULTS_DIR = Path('/workspace/sysid_results')


def default_results_dir() -> Path:
    DEFAULT_RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    return DEFAULT_RESULTS_DIR


def load_model(
    model_path: Path | str,
    *,
    xacro_mappings: Optional[Mapping[str, str]] = None,
):
    """Load Pinocchio model from expanded xacro (default) or a plain .urdf file."""
    require_pinocchio()
    path = Path(model_path)
    if path.suffix == '.xacro' or xacro_mappings is not None:
        urdf_xml = expand_xacro(path, xacro_mappings)
        model = pin.buildModelFromXML(urdf_xml)
    else:
        model = pin.buildModelFromUrdf(str(path))
    data = model.createData()
    return model, data


def load_default_model():
    """Load the default OMX model from open_manipulator_x.urdf.xacro."""
    return load_model(
        default_xacro_path(),
        xacro_mappings=default_xacro_mappings(),
    )


def resolve_model_load_args(
    *,
    urdf_path: Optional[Path | str] = None,
    xacro_path: Optional[Path | str] = None,
    xacro_mappings: Optional[Mapping[str, str]] = None,
) -> Tuple[Path, Optional[Dict[str, str]], str]:
    if urdf_path is not None and xacro_path is not None:
        raise ValueError('Provide only one of --urdf or --xacro')

    if urdf_path is not None:
        path = Path(urdf_path)
        return path, None, format_model_source(path)

    path = Path(xacro_path) if xacro_path else default_xacro_path()
    resolved_mappings = dict(default_xacro_mappings())
    if xacro_mappings:
        resolved_mappings.update(xacro_mappings)
    return path, resolved_mappings, format_model_source(path, xacro_mappings=resolved_mappings)


def arm_velocity_indices(model) -> List[int]:
    return [model.joints[model.getJointId(joint)].idx_v for joint in ARM_JOINTS]


def nominal_dynamic_parameters(model) -> np.ndarray:
    return np.concatenate(
        [
            model.inertias[body_index].toDynamicParameters()
            for body_index in range(1, model.nbodies)
        ]
    )


def body_names(model) -> List[str]:
    return [model.names[body_index] for body_index in range(1, model.nbodies)]


def nominal_friction_from_urdf(model) -> Tuple[Dict[str, float], Dict[str, float]]:
    """URDF `<dynamics damping>` gives nominal Fv; Coulomb is absent in URDF."""
    viscous: Dict[str, float] = {}
    coulomb = {joint: 0.0 for joint in ARM_JOINTS}
    for joint in ARM_JOINTS:
        joint_id = model.getJointId(joint)
        viscous[joint] = float(model.damping[joint_id])
    return viscous, coulomb


def default_synthetic_friction(model) -> SyntheticFrictionTruth:
    """Synthetic ground truth: URDF viscous + small offset per joint."""
    viscous_nominal, _ = nominal_friction_from_urdf(model)
    viscous = {
        joint: viscous_nominal[joint] + offset
        for joint, offset in zip(
            ARM_JOINTS,
            (0.02, 0.05, 0.01, 0.03),
        )
    }
    return SyntheticFrictionTruth(viscous=viscous)


def build_friction_regressor_row(arm_velocity: np.ndarray) -> np.ndarray:
    """Return Y_fric with shape (4, 4): diag(q̇) for Fv only."""
    return np.diag(arm_velocity)


def dynamics_torque_arm(
    model,
    data,
    q: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    arm_indices: Sequence[int],
) -> np.ndarray:
    """Generalized dynamics torque on Arm joints using fixed URDF inertia (RNEA)."""
    tau_full = pin.rnea(model, data, q, velocity, acceleration)
    return tau_full[arm_indices]


def stack_state_from_sample(
    model,
    sample,
    arm_indices: Sequence[int],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    q = pin.neutral(model)
    velocity = np.zeros(model.nv)
    acceleration = np.zeros(model.nv)
    for joint_index, joint_name in enumerate(ARM_JOINTS):
        idx = arm_indices[joint_index]
        q[idx] = sample.position[joint_name]
        velocity[idx] = sample.velocity[joint_name]
        acceleration[idx] = sample.acceleration[joint_name]
    return q, velocity, acceleration


def generate_synthetic_dataset(
    trajectory: ExcitationTrajectory,
    model,
    data,
    *,
    num_periods: int = 3,
    sample_dt_s: float = 0.01,
    friction_truth: Optional[SyntheticFrictionTruth] = None,
    noise_std: float = 0.005,
    rng_seed: int = 0,
) -> Tuple[np.ndarray, np.ndarray, SyntheticFrictionTruth]:
    """Build Y_fric and τ_res = τ_meas − τ_dyn(URDF) for friction-only regression."""
    if num_periods < 1:
        raise ValueError('num_periods must be at least 1')
    if sample_dt_s <= 0.0:
        raise ValueError('sample_dt_s must be positive')

    arm_indices = arm_velocity_indices(model)
    truth = friction_truth or default_synthetic_friction(model)
    fv = np.array([truth.viscous[joint] for joint in ARM_JOINTS])
    rng = np.random.default_rng(rng_seed)

    rows: List[np.ndarray] = []
    residuals: List[np.ndarray] = []
    period_s = trajectory.period_s
    total_time = num_periods * period_s
    num_steps = int(math.floor(total_time / sample_dt_s)) + 1

    for step in range(num_steps):
        time_s = min(step * sample_dt_s, total_time)
        sample = trajectory.sample(time_s % period_s)
        q, velocity, acceleration = stack_state_from_sample(model, sample, arm_indices)
        arm_velocity = velocity[arm_indices]
        tau_dyn = dynamics_torque_arm(model, data, q, velocity, acceleration, arm_indices)
        tau_friction = fv * arm_velocity
        tau_meas = tau_dyn + tau_friction + rng.normal(0.0, noise_std, len(ARM_JOINTS))
        tau_residual = tau_meas - tau_dyn
        rows.append(build_friction_regressor_row(arm_velocity))
        residuals.append(tau_residual)

    return np.vstack(rows), np.concatenate(residuals), truth


def effective_rank_and_condition(regressor: np.ndarray) -> Tuple[int, float]:
    singular_values = np.linalg.svd(regressor, compute_uv=False)
    if singular_values.size == 0:
        return 0, float('nan')
    threshold = SVD_RANK_RTOL * singular_values[0]
    rank = int(np.count_nonzero(singular_values > threshold))
    if rank == 0:
        return 0, float('inf')
    condition_number = float(singular_values[0] / singular_values[rank - 1])
    return rank, condition_number


def solve_least_squares(regressor: np.ndarray, torque: np.ndarray) -> np.ndarray:
    solution, _, _, _ = np.linalg.lstsq(regressor, torque, rcond=None)
    return solution


def split_viscous_vector(parameter_vector: np.ndarray) -> np.ndarray:
    return parameter_vector[:len(ARM_JOINTS)]


def generate_dataset_regression(
    dataset: IdentificationDataset,
    model,
    data,
) -> Tuple[np.ndarray, np.ndarray]:
    """Build Y_fric and τ_res from recorded (q, q̇, q̈, τ) samples."""
    arm_indices = arm_velocity_indices(model)
    rows: List[np.ndarray] = []
    residuals: List[np.ndarray] = []

    for index in range(dataset.num_samples):
        q = pin.neutral(model)
        velocity = np.zeros(model.nv)
        acceleration = np.zeros(model.nv)
        for joint_index, joint_name in enumerate(ARM_JOINTS):
            idx = arm_indices[joint_index]
            q[idx] = dataset.position[index, joint_index]
            velocity[idx] = dataset.velocity[index, joint_index]
            acceleration[idx] = dataset.acceleration[index, joint_index]

        arm_velocity = velocity[arm_indices]
        tau_dyn = dynamics_torque_arm(model, data, q, velocity, acceleration, arm_indices)
        tau_meas = dataset.torque[index]
        tau_residual = tau_meas - tau_dyn
        rows.append(build_friction_regressor_row(arm_velocity))
        residuals.append(tau_residual)

    return np.vstack(rows), np.concatenate(residuals)


def run_dataset_regression(
    dataset: IdentificationDataset,
    *,
    urdf_path: Optional[Path | str] = None,
    xacro_path: Optional[Path | str] = None,
    xacro_mappings: Optional[Mapping[str, str]] = None,
    excitation_config: Optional[Path | str] = None,
    output_dir: Optional[Path | str] = None,
    dataset_source: Optional[str] = None,
) -> RegressionResult:
    require_pinocchio()
    model_path, load_mappings, model_source_label = resolve_model_load_args(
        urdf_path=urdf_path,
        xacro_path=xacro_path,
        xacro_mappings=xacro_mappings,
    )
    resolved_config = Path(excitation_config) if excitation_config else default_config_path()
    resolved_output = Path(output_dir) if output_dir else default_results_dir()

    model, data = load_model(model_path, xacro_mappings=load_mappings)
    trajectory = ExcitationTrajectory.from_yaml(resolved_config)
    validate_identification_dataset(dataset)
    regressor, torque_residual = generate_dataset_regression(dataset, model, data)

    matrix_rank, condition_number = effective_rank_and_condition(regressor)
    if matrix_rank < NUM_FRICTION_PARAMS:
        raise ValueError(
            f'Friction regressor rank {matrix_rank}/{NUM_FRICTION_PARAMS} — '
            'columns for Fv are not persistently excited. '
            'The bag likely has zero joint velocity (static pose) or constant '
            'effort only. Re-record after a successful excitation run.'
        )

    parameter_vector = solve_least_squares(regressor, torque_residual)
    residual_norm = float(np.linalg.norm(regressor @ parameter_vector - torque_residual))
    viscous = split_viscous_vector(parameter_vector)

    if np.any(~np.isfinite(parameter_vector)) or np.any(
        np.abs(parameter_vector) > MAX_REASONABLE_FRICTION
    ):
        raise ValueError(
            'Least-squares Fv estimate is non-finite or unreasonably large. '
            f'Fv={viscous}. Check dataset motion and regressor rank '
            f'({matrix_rank}/{NUM_FRICTION_PARAMS}).'
        )

    result = RegressionResult(
        viscous=viscous,
        residual_norm=residual_norm,
        condition_number=condition_number,
        matrix_rank=matrix_rank,
        num_samples=regressor.shape[0] // len(ARM_JOINTS),
        sample_period_s=trajectory.period_s,
        sample_dt_s=dataset.sample_dt_s,
    )

    resolved_output.mkdir(parents=True, exist_ok=True)
    viscous_nominal, _ = nominal_friction_from_urdf(model)
    write_inertia_nominal_yaml(
        resolved_output / 'inertia_nominal.yaml',
        model=model,
        urdf_path=Path(model_source_label),
    )
    write_friction_yaml(
        resolved_output / 'friction_estimated.yaml',
        estimated_viscous=result.viscous,
        nominal_viscous=viscous_nominal,
        synthetic_truth=None,
    )
    write_report_html(
        resolved_output / 'report.html',
        model=model,
        result=result,
        urdf_path=Path(model_source_label),
        excitation_config=resolved_config,
        viscous_nominal=viscous_nominal,
        synthetic_truth=None,
        dataset_source=dataset_source,
    )
    return result


def run_synthetic_regression(
    *,
    urdf_path: Optional[Path | str] = None,
    xacro_path: Optional[Path | str] = None,
    xacro_mappings: Optional[Mapping[str, str]] = None,
    excitation_config: Optional[Path | str] = None,
    output_dir: Optional[Path | str] = None,
    num_periods: int = 3,
    sample_dt_s: float = 0.01,
    noise_std: float = 0.005,
    rng_seed: int = 0,
) -> RegressionResult:
    require_pinocchio()
    model_path, load_mappings, model_source_label = resolve_model_load_args(
        urdf_path=urdf_path,
        xacro_path=xacro_path,
        xacro_mappings=xacro_mappings,
    )
    resolved_config = Path(excitation_config) if excitation_config else default_config_path()
    resolved_output = Path(output_dir) if output_dir else default_results_dir()

    model, data = load_model(model_path, xacro_mappings=load_mappings)
    trajectory = ExcitationTrajectory.from_yaml(resolved_config)
    regressor, torque_residual, friction_truth = generate_synthetic_dataset(
        trajectory,
        model,
        data,
        num_periods=num_periods,
        sample_dt_s=sample_dt_s,
        noise_std=noise_std,
        rng_seed=rng_seed,
    )

    parameter_vector = solve_least_squares(regressor, torque_residual)
    residual_norm = float(np.linalg.norm(regressor @ parameter_vector - torque_residual))
    matrix_rank, condition_number = effective_rank_and_condition(regressor)
    viscous = split_viscous_vector(parameter_vector)

    result = RegressionResult(
        viscous=viscous,
        residual_norm=residual_norm,
        condition_number=condition_number,
        matrix_rank=matrix_rank,
        num_samples=regressor.shape[0] // len(ARM_JOINTS),
        sample_period_s=trajectory.period_s,
        sample_dt_s=sample_dt_s,
    )

    resolved_output.mkdir(parents=True, exist_ok=True)
    viscous_nominal, _ = nominal_friction_from_urdf(model)
    write_inertia_nominal_yaml(
        resolved_output / 'inertia_nominal.yaml',
        model=model,
        urdf_path=Path(model_source_label),
    )
    write_friction_yaml(
        resolved_output / 'friction_estimated.yaml',
        estimated_viscous=result.viscous,
        nominal_viscous=viscous_nominal,
        synthetic_truth=friction_truth,
    )
    write_report_html(
        resolved_output / 'report.html',
        model=model,
        result=result,
        urdf_path=Path(model_source_label),
        excitation_config=resolved_config,
        viscous_nominal=viscous_nominal,
        synthetic_truth=friction_truth,
    )
    return result


def inertia_entries(model, values: np.ndarray) -> List[Dict[str, object]]:
    entries: List[Dict[str, object]] = []
    for body_index, body_name in enumerate(body_names(model), start=0):
        offset = body_index * len(INERTIA_PARAM_NAMES)
        chunk = values[offset:offset + len(INERTIA_PARAM_NAMES)]
        entries.append(
            {
                'body': body_name,
                'parameters': {
                    name: float(value)
                    for name, value in zip(INERTIA_PARAM_NAMES, chunk)
                },
            }
        )
    return entries


def write_inertia_nominal_yaml(
    path: Path,
    *,
    model,
    urdf_path: Path,
) -> None:
    payload = {
        'description': (
            'Nominal inertia from URDF (Pinocchio standard dynamic parameters). '
            'Used fixed during regression; not estimated in Phase 1.'
        ),
        'inertia': {
            'use_nominal_urdf': True,
            'source_urdf': str(urdf_path),
        },
        'parameter_names': list(INERTIA_PARAM_NAMES),
        'links': inertia_entries(model, nominal_dynamic_parameters(model)),
    }
    with path.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(payload, handle, sort_keys=False, default_flow_style=False)


def write_friction_yaml(
    path: Path,
    *,
    estimated_viscous: np.ndarray,
    nominal_viscous: Mapping[str, float],
    synthetic_truth: Optional[SyntheticFrictionTruth] = None,
) -> None:
    joints: Dict[str, Dict[str, float]] = {}
    for index, joint in enumerate(ARM_JOINTS):
        entry: Dict[str, float] = {
            'Fv_estimated': float(estimated_viscous[index]),
            'Fv_nominal_urdf': float(nominal_viscous[joint]),
        }
        if synthetic_truth is not None:
            entry['Fv_synthetic_truth'] = float(synthetic_truth.viscous[joint])
        joints[joint] = entry
    payload = {
        'description': (
            'Estimated arm viscous friction (Fv) with URDF inertia fixed. '
            'Phase 2 may copy Fv into calibrated_dynamics.yaml.'
        ),
        'joints': joints,
    }
    with path.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(payload, handle, sort_keys=False, default_flow_style=False)


def _format_float(value: float) -> str:
    if not math.isfinite(value):
        return 'nan'
    return f'{value:.6g}'


def write_report_html(
    path: Path,
    *,
    model,
    result: RegressionResult,
    urdf_path: Path,
    excitation_config: Path,
    viscous_nominal: Mapping[str, float],
    synthetic_truth: Optional[SyntheticFrictionTruth] = None,
    dataset_source: Optional[str] = None,
) -> None:
    timestamp = datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')
    include_truth = synthetic_truth is not None
    title = (
        'OpenMANIPULATOR-X Sysid Report (Synthetic Dataset)'
        if include_truth
        else 'OpenMANIPULATOR-X Sysid Report (Recorded Dataset)'
    )
    friction_rows = []
    for index, joint in enumerate(ARM_JOINTS):
        cells = [
            f'<td>{html.escape(joint)}</td>',
            f'<td>{_format_float(viscous_nominal[joint])}</td>',
        ]
        if include_truth:
            cells.append(f'<td>{_format_float(synthetic_truth.viscous[joint])}</td>')
        cells.append(f'<td>{_format_float(result.viscous[index])}</td>')
        friction_rows.append('<tr>' + ''.join(cells) + '</tr>')

    header_cells = [
        '<th>Joint</th>',
        '<th>Fv nominal (URDF)</th>',
    ]
    if include_truth:
        header_cells.append('<th>Fv synthetic truth</th>')
    header_cells.append('<th>Fv estimated</th>')

    source_line = ''
    if dataset_source:
        source_line = (
            f'<p class="meta">Dataset source: {html.escape(dataset_source)}</p>'
        )

    document = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>OMX Sysid Report</title>
  <style>
    body {{ font-family: sans-serif; margin: 2rem; line-height: 1.4; }}
    table {{ border-collapse: collapse; width: 100%; margin: 1rem 0; }}
    th, td {{ border: 1px solid #ccc; padding: 0.35rem 0.5rem; text-align: right; }}
    th:first-child, td:first-child {{ text-align: left; }}
    h2 {{ margin-top: 2rem; }}
    .meta {{ color: #555; }}
  </style>
</head>
<body>
  <h1>{html.escape(title)}</h1>
  <p class="meta">Generated {html.escape(timestamp)}</p>
  <p class="meta">URDF: {html.escape(str(urdf_path))}</p>
  <p class="meta">Excitation config: {html.escape(str(excitation_config))}</p>
  {source_line}
  <p>Inertia: fixed from URDF via RNEA (<code>τ<sub>res</sub> = τ<sub>meas</sub> − τ<sub>dyn,URDF</sub></code>).
  Only Fv is estimated (4 parameters).</p>

  <h2>Regression quality</h2>
  <table>
    <tr><th>Metric</th><th>Value</th></tr>
    <tr><td>Samples (per arm joint row)</td><td>{result.num_samples}</td></tr>
    <tr><td>Excitation period</td><td>{_format_float(result.sample_period_s)} s</td></tr>
    <tr><td>Sample dt</td><td>{_format_float(result.sample_dt_s)} s</td></tr>
    <tr><td>Residual norm ||Yπ − τ<sub>res</sub>||</td><td>{_format_float(result.residual_norm)}</td></tr>
    <tr><td>Effective matrix rank</td><td>{result.matrix_rank} / {NUM_FRICTION_PARAMS}</td></tr>
    <tr><td>Effective condition number</td><td>{_format_float(result.condition_number)}</td></tr>
  </table>

  <h2>Nominal vs estimated friction</h2>
  <table>
    <tr>
      {''.join(header_cells)}
    </tr>
    {''.join(friction_rows)}
  </table>
</body>
</html>
"""
    path.write_text(document, encoding='utf-8')


def _parse_xacro_mapping_arg(value: str) -> Tuple[str, str]:
    import argparse

    if ':=' not in value:
        raise argparse.ArgumentTypeError(
            f'Xacro mapping must be KEY:=VALUE, got {value!r}'
        )
    key, mapped_value = value.split(':=', 1)
    key = key.strip()
    if not key:
        raise argparse.ArgumentTypeError(
            f'Xacro mapping key must be non-empty, got {value!r}'
        )
    return key, mapped_value.strip()


def _add_model_source_arguments(parser: argparse.ArgumentParser) -> None:
    model_group = parser.add_argument_group('robot model source')
    model_group.add_argument(
        '--urdf',
        type=Path,
        default=None,
        help=(
            'Path to a plain open_manipulator_x.urdf (overrides default xacro). '
            'Mutually exclusive with --xacro.'
        ),
    )
    model_group.add_argument(
        '--xacro',
        type=Path,
        default=None,
        help=(
            'Path to open_manipulator_x.urdf.xacro '
            '(default: open_manipulator_description bundle)'
        ),
    )
    model_group.add_argument(
        '--xacro-mapping',
        action='append',
        default=[],
        metavar='KEY:=VALUE',
        type=_parse_xacro_mapping_arg,
        help=(
            'Extra xacro argument (repeatable), e.g. use_sim:=true. '
            'Merged over package defaults aligned with excitation launch.'
        ),
    )


def _model_source_kwargs_from_parsed(parsed: argparse.Namespace) -> Dict[str, object]:
    return {
        'urdf_path': parsed.urdf,
        'xacro_path': parsed.xacro,
        'xacro_mappings': dict(parsed.xacro_mapping) if parsed.xacro_mapping else None,
    }


def main(args: Optional[Sequence[str]] = None) -> None:
    import argparse

    parser = argparse.ArgumentParser(
        description=(
            'Run synthetic OMX sysid: fixed URDF inertia, friction-only least squares → '
            'results/*.yaml + report.html'
        )
    )
    _add_model_source_arguments(parser)
    parser.add_argument(
        '--config',
        type=Path,
        default=None,
        help='Path to excitation.yaml (default: package config)',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=None,
        help=(
            'Directory for inertia_nominal.yaml, friction_estimated.yaml, report.html '
            f'(default: {DEFAULT_RESULTS_DIR})'
        ),
    )
    parser.add_argument(
        '--periods',
        type=int,
        default=3,
        help='Number of excitation periods to simulate (default: 3)',
    )
    parser.add_argument(
        '--dt',
        type=float,
        default=0.01,
        help='Sample period in seconds (default: 0.01)',
    )
    parser.add_argument(
        '--noise-std',
        type=float,
        default=0.005,
        help='Gaussian torque noise std dev in Nm (default: 0.005)',
    )
    parser.add_argument(
        '--seed',
        type=int,
        default=0,
        help='Random seed for synthetic torque noise (default: 0)',
    )
    parsed = parser.parse_args(args)

    result = run_synthetic_regression(
        **_model_source_kwargs_from_parsed(parsed),
        excitation_config=parsed.config,
        output_dir=parsed.output_dir,
        num_periods=parsed.periods,
        sample_dt_s=parsed.dt,
        noise_std=parsed.noise_std,
        rng_seed=parsed.seed,
    )
    output_dir = Path(parsed.output_dir) if parsed.output_dir else default_results_dir()
    print(f'Wrote results under {output_dir}')
    print(f'samples: {result.num_samples}')
    print(f'residual_norm: {result.residual_norm:.6g}')
    print(f'effective_rank: {result.matrix_rank}/{NUM_FRICTION_PARAMS}')
    print(f'condition_number: {result.condition_number:.6g}')


def main_bag(args: Optional[Sequence[str]] = None) -> None:
    import argparse

    from open_manipulator_sysid.bag_to_dataset import bag_to_dataset

    parser = argparse.ArgumentParser(
        description=(
            'Run OMX sysid on a recorded dataset (.npz) or rosbag2 directory → '
            'results/*.yaml + report.html'
        )
    )
    _add_model_source_arguments(parser)
    parser.add_argument(
        '--dataset',
        type=Path,
        default=None,
        help='Path to .npz dataset produced by bag_to_dataset',
    )
    parser.add_argument(
        '--bag',
        type=Path,
        default=None,
        help='Rosbag2 directory (converted on the fly if --dataset omitted)',
    )
    parser.add_argument(
        '--config',
        type=Path,
        default=None,
        help='Path to excitation.yaml (default: package config)',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=None,
        help=f'Directory for regression outputs (default: {DEFAULT_RESULTS_DIR})',
    )
    parser.add_argument(
        '--dt',
        type=float,
        default=0.01,
        help='Resample period when reading --bag (default: 0.01)',
    )
    parser.add_argument(
        '--trim-start',
        type=float,
        default=7.0,
        help='Seconds trimmed from bag start (default: 7.0 = approach+hold)',
    )
    parser.add_argument(
        '--trim-end',
        type=float,
        default=2.5,
        help='Seconds trimmed from bag end — default covers settle hold (2.0 s) + margin',
    )
    parsed = parser.parse_args(args)

    if parsed.dataset is None and parsed.bag is None:
        parser.error('Provide --dataset or --bag')

    if parsed.dataset is not None:
        dataset = IdentificationDataset.load_npz(parsed.dataset)
        dataset_source = str(parsed.dataset)
    else:
        dataset = bag_to_dataset(
            parsed.bag,
            sample_dt_s=parsed.dt,
            trim_start_s=parsed.trim_start,
            trim_end_s=parsed.trim_end,
        )
        dataset_source = str(parsed.bag)

    result = run_dataset_regression(
        dataset,
        **_model_source_kwargs_from_parsed(parsed),
        excitation_config=parsed.config,
        output_dir=parsed.output_dir,
        dataset_source=dataset_source,
    )
    output_dir = Path(parsed.output_dir) if parsed.output_dir else default_results_dir()
    print(f'Wrote results under {output_dir}')
    print(f'samples: {result.num_samples}')
    print(f'residual_norm: {result.residual_norm:.6g}')
    print(f'effective_rank: {result.matrix_rank}/{NUM_FRICTION_PARAMS}')
    print(f'condition_number: {result.condition_number:.6g}')


if __name__ == '__main__':
    main()
