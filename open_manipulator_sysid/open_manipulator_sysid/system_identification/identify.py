"""Joint identification of base dynamic parameters + Coulomb/viscous friction."""

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
from scipy.optimize import least_squares

from open_manipulator_sysid.excitation_recording.bag_reader import bag_to_dataset
from open_manipulator_sysid.pinocchio_support.numerics import effective_condition_number
from open_manipulator_sysid.five_link_dynamics.base_parameters import (
    FiveLinkBaseParameterSet,
    five_link_base_set_from_excitation_data,
    five_link_base_set_from_trajectory,
    reference_base_values,
)
from open_manipulator_sysid.excitation_trajectory import (
    ARM_JOINTS,
    ExcitationTrajectory,
    default_config_path,
    describe_runtime_schedule,
    runtime_config_from_yaml,
)
from open_manipulator_sysid.excitation_recording.dataset import (
    IdentificationDataset,
    validate_identification_dataset,
)
from open_manipulator_sysid.pinocchio_support.model_loader import (
    arm_velocity_indices,
    default_results_dir,
    load_model,
    resolve_model_load_args,
    stack_state_from_sample,
)
from open_manipulator_sysid.system_identification.friction import (
    FRICTION_PARAM_NAMES,
    NUM_FRICTION_PARAMS,
    CoulombViscousFriction,
    build_coulomb_viscous_block,
    coulomb_viscous_torque_arm,
    default_friction_guess,
    format_friction_equation,
    format_parameter_meanings,
    friction_bounds,
    friction_params_to_dict,
    pack_friction_vector,
    unpack_friction_vector,
)

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover
    pin = None
    _PINOCCHIO_IMPORT_ERROR = exc
else:
    _PINOCCHIO_IMPORT_ERROR = None


@dataclass(frozen=True)
class FullFrictionTruth:
    per_joint: Dict[str, Dict[str, float]]


@dataclass(frozen=True)
class IdentificationResult:
    base_set: FiveLinkBaseParameterSet
    base_parameters: np.ndarray
    reference_values: np.ndarray
    friction_parameters: np.ndarray
    residual_norm: float
    cost: float
    condition_number: float
    matrix_rank: int
    num_samples: int
    sample_dt_s: float
    optimizer_status: int
    optimizer_message: str


def require_pinocchio() -> None:
    if pin is None:
        raise ImportError(
            'Pinocchio is required for dynamics identification. '
            'Install ros-jazzy-pinocchio inside the Docker container.'
        ) from _PINOCCHIO_IMPORT_ERROR


@dataclass(frozen=True)
class SampleKinematics:
    base_regressor: np.ndarray
    arm_velocity: np.ndarray
    torque: np.ndarray


def subsample_dataset(dataset: IdentificationDataset, max_samples: int = 600) -> IdentificationDataset:
    if dataset.num_samples <= max_samples:
        return dataset
    indices = np.linspace(0, dataset.num_samples - 1, max_samples, dtype=int)
    return IdentificationDataset(
        times_s=dataset.times_s[indices],
        position=dataset.position[indices],
        velocity=dataset.velocity[indices],
        acceleration=dataset.acceleration[indices],
        torque=dataset.torque[indices],
        sample_dt_s=dataset.sample_dt_s,
    )


def build_sample_kinematics(
    model,
    data,
    dataset: IdentificationDataset,
    base_set: FiveLinkBaseParameterSet,
    arm_indices: Sequence[int],
) -> List[SampleKinematics]:
    layout = base_set.layout
    five_link_columns = list(layout.five_link_column_indices)
    samples: List[SampleKinematics] = []
    for index in range(dataset.num_samples):
        q = pin.neutral(model)
        velocity = np.zeros(model.nv)
        acceleration = np.zeros(model.nv)
        for joint_index, joint_name in enumerate(ARM_JOINTS):
            idx = arm_indices[joint_index]
            q[idx] = dataset.position[index, joint_index]
            velocity[idx] = dataset.velocity[index, joint_index]
            acceleration[idx] = dataset.acceleration[index, joint_index]
        regressor = pin.computeJointTorqueRegressor(
            model, data, q, velocity, acceleration
        )
        five_link_regressor = regressor[arm_indices, :][:, five_link_columns]
        samples.append(
            SampleKinematics(
                base_regressor=base_set.project_regressor(five_link_regressor),
                arm_velocity=velocity[arm_indices].copy(),
                torque=dataset.torque[index].copy(),
            )
        )
    return samples


def predict_torque_batch(
    pi_base: np.ndarray,
    friction_vector: np.ndarray,
    samples: Sequence[SampleKinematics],
) -> np.ndarray:
    friction = unpack_friction_vector(friction_vector)
    predicted: List[np.ndarray] = []
    for sample in samples:
        tau = sample.base_regressor @ pi_base
        tau = tau + coulomb_viscous_torque_arm(sample.arm_velocity, friction)
        predicted.append(tau)
    return np.concatenate(predicted)


def linear_observation_matrix(
    samples: Sequence[SampleKinematics],
) -> np.ndarray:
    """Linear block [Y_base | Y_Fv,Fc] stacked over samples."""
    rows: List[np.ndarray] = []
    for sample in samples:
        rows.append(
            np.hstack(
                [
                    sample.base_regressor,
                    build_coulomb_viscous_block(sample.arm_velocity),
                ]
            )
        )
    return np.vstack(rows)


def identification_trim_times(
    config: Mapping[str, object],
    *,
    num_periods: int,
) -> Tuple[float, float]:
    trajectory = ExcitationTrajectory.from_config(config)
    runtime = runtime_config_from_yaml(config)
    schedule = describe_runtime_schedule(
        trajectory,
        start_positions=trajectory.q0,
        to_q0_duration_s=runtime['to_q0_duration_s'],
        ingress_spline_duration_s=runtime['ingress_spline_duration_s'],
        num_periods=num_periods,
        egress_spline_duration_s=runtime['egress_spline_duration_s'],
        q0_hold_duration_s=runtime['q0_hold_duration_s'],
        discard_periods=runtime['discard_periods'],
    )
    # Both bounds are seconds from the motion t=0 at bag start (not from bag end).
    return schedule.identification_start_s, schedule.identification_end_s


def run_identification(
    dataset: IdentificationDataset,
    *,
    urdf_path: Optional[Path | str] = None,
    xacro_path: Optional[Path | str] = None,
    xacro_mappings: Optional[Mapping[str, str]] = None,
    excitation_config: Optional[Path | str] = None,
    output_dir: Optional[Path | str] = None,
    dataset_source: Optional[str] = None,
    base_set: Optional[FiveLinkBaseParameterSet] = None,
) -> IdentificationResult:
    require_pinocchio()
    model_path, load_mappings, model_source_label = resolve_model_load_args(
        urdf_path=urdf_path,
        xacro_path=xacro_path,
        xacro_mappings=xacro_mappings,
    )
    config_path = Path(excitation_config) if excitation_config else default_config_path()
    with config_path.open('r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)

    resolved_output = Path(output_dir) if output_dir else default_results_dir()
    model, data = load_model(model_path, xacro_mappings=load_mappings)
    validate_identification_dataset(dataset)
    dataset = subsample_dataset(dataset)
    friction_cfg = CoulombViscousFriction.from_config(config)
    friction_guess = default_friction_guess(friction_cfg)
    trajectory = ExcitationTrajectory.from_config(config)
    if base_set is None:
        base_set = five_link_base_set_from_excitation_data(
            trajectory,
            model,
            data,
            dataset,
        )

    reference_values = reference_base_values(model, base_set)
    arm_indices = arm_velocity_indices(model)
    samples = build_sample_kinematics(model, data, dataset, base_set, arm_indices)
    rows_base = np.vstack([sample.base_regressor for sample in samples])
    torque_vec = np.concatenate([sample.torque for sample in samples])
    pi_base0, _, _, _ = np.linalg.lstsq(rows_base, torque_vec, rcond=None)

    x0 = np.concatenate([pi_base0, pack_friction_vector(friction_guess)])
    friction_lower, friction_upper = friction_bounds(friction_guess)
    lower = np.concatenate([np.full(base_set.num_base, -np.inf), friction_lower])
    upper = np.concatenate([np.full(base_set.num_base, np.inf), friction_upper])

    linear_block = linear_observation_matrix(samples)
    matrix_rank, condition_number = effective_condition_number(linear_block)
    if matrix_rank < base_set.num_base:
        raise ValueError(
            f'Linear observation rank {matrix_rank}/{base_set.num_base} is deficient '
            'for the base parameter set. Re-record with richer excitation.'
        )

    def residual(params: np.ndarray) -> np.ndarray:
        pi_base = params[: base_set.num_base]
        friction = params[base_set.num_base :]
        return predict_torque_batch(pi_base, friction, samples) - torque_vec

    opt = least_squares(
        residual,
        x0,
        bounds=(lower, upper),
        method='trf',
        max_nfev=1000,
        ftol=1e-10,
        xtol=1e-10,
    )

    result = IdentificationResult(
        base_set=base_set,
        base_parameters=opt.x[: base_set.num_base],
        reference_values=reference_values,
        friction_parameters=opt.x[base_set.num_base :],
        residual_norm=float(np.linalg.norm(opt.fun)),
        cost=float(opt.cost),
        condition_number=condition_number,
        matrix_rank=matrix_rank,
        num_samples=dataset.num_samples,
        sample_dt_s=dataset.sample_dt_s,
        optimizer_status=int(opt.status),
        optimizer_message=str(opt.message),
    )

    resolved_output.mkdir(parents=True, exist_ok=True)
    write_identification_yaml(
        resolved_output / 'dynamics_identified.yaml',
        model=model,
        result=result,
        model_source=str(model_source_label),
        excitation_config=config_path,
    )
    write_report_html(
        resolved_output / 'report.html',
        result=result,
        urdf_path=Path(model_source_label),
        excitation_config=config_path,
        dataset_source=dataset_source,
    )
    print_identification_summary(model, result)
    return result


def default_synthetic_friction_truth() -> FullFrictionTruth:
    return FullFrictionTruth(
        per_joint={
            'joint1': {'Fv': 0.02, 'Fc': 0.05},
            'joint2': {'Fv': 0.04, 'Fc': 0.08},
            'joint3': {'Fv': 0.015, 'Fc': 0.04},
            'joint4': {'Fv': 0.01, 'Fc': 0.03},
        }
    )


def friction_truth_vector(truth: FullFrictionTruth) -> np.ndarray:
    rows = []
    for joint in ARM_JOINTS:
        rows.append([truth.per_joint[joint][name] for name in FRICTION_PARAM_NAMES])
    return pack_friction_vector(np.asarray(rows, dtype=float))


def generate_synthetic_dataset(
    trajectory: ExcitationTrajectory,
    model,
    data,
    base_set: FiveLinkBaseParameterSet,
    *,
    num_periods: int = 5,
    discard_periods: int = 1,
    sample_dt_s: float = 0.01,
    friction_truth: Optional[FullFrictionTruth] = None,
    noise_std: float = 0.005,
    rng_seed: int = 0,
) -> Tuple[IdentificationDataset, FullFrictionTruth, np.ndarray]:
    if num_periods <= discard_periods:
        raise ValueError('num_periods must exceed discard_periods')

    truth = friction_truth or default_synthetic_friction_truth()
    friction = unpack_friction_vector(friction_truth_vector(truth))
    pi_base_true = reference_base_values(model, base_set)
    rng = np.random.default_rng(rng_seed)
    arm_indices = arm_velocity_indices(model)

    times: List[float] = []
    positions: List[List[float]] = []
    velocities: List[List[float]] = []
    accelerations: List[List[float]] = []
    torques: List[List[float]] = []

    start_time = discard_periods * trajectory.period_s
    end_time = num_periods * trajectory.period_s
    num_steps = int(math.floor((end_time - start_time) / sample_dt_s)) + 1

    for step in range(num_steps):
        local_time = start_time + min(step * sample_dt_s, end_time - start_time)
        sample = trajectory.sample(local_time)
        q, velocity, acceleration = stack_state_from_sample(model, sample, arm_indices)
        regressor = pin.computeJointTorqueRegressor(
            model, data, q, velocity, acceleration
        )
        five_link_regressor = regressor[arm_indices, :][
            :, list(base_set.layout.five_link_column_indices)
        ]
        tau = base_set.project_regressor(five_link_regressor) @ pi_base_true
        tau = tau + coulomb_viscous_torque_arm(velocity[arm_indices], friction)
        tau = tau + rng.normal(0.0, noise_std, len(ARM_JOINTS))

        times.append(local_time - start_time)
        positions.append([sample.position[joint] for joint in ARM_JOINTS])
        velocities.append([sample.velocity[joint] for joint in ARM_JOINTS])
        accelerations.append([sample.acceleration[joint] for joint in ARM_JOINTS])
        torques.append(tau.tolist())

    dataset = IdentificationDataset(
        times_s=np.asarray(times, dtype=float),
        position=np.asarray(positions, dtype=float),
        velocity=np.asarray(velocities, dtype=float),
        acceleration=np.asarray(accelerations, dtype=float),
        torque=np.asarray(torques, dtype=float),
        sample_dt_s=sample_dt_s,
    )
    return dataset, truth, pi_base_true


def run_synthetic_identification(
    *,
    urdf_path: Optional[Path | str] = None,
    xacro_path: Optional[Path | str] = None,
    xacro_mappings: Optional[Mapping[str, str]] = None,
    excitation_config: Optional[Path | str] = None,
    output_dir: Optional[Path | str] = None,
    num_periods: int = 5,
    discard_periods: int = 1,
    sample_dt_s: float = 0.01,
    noise_std: float = 0.005,
    rng_seed: int = 0,
) -> IdentificationResult:
    config_path = Path(excitation_config) if excitation_config else default_config_path()
    with config_path.open('r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)

    model_path, load_mappings, _ = resolve_model_load_args(
        urdf_path=urdf_path,
        xacro_path=xacro_path,
        xacro_mappings=xacro_mappings,
    )
    model, data = load_model(model_path, xacro_mappings=load_mappings)
    trajectory = ExcitationTrajectory.from_config(config)
    base_set = five_link_base_set_from_trajectory(trajectory, model, data)
    dataset, _, _ = generate_synthetic_dataset(
        trajectory,
        model,
        data,
        base_set,
        num_periods=num_periods,
        discard_periods=discard_periods,
        sample_dt_s=sample_dt_s,
        noise_std=noise_std,
        rng_seed=rng_seed,
    )
    return run_identification(
        dataset,
        urdf_path=urdf_path,
        xacro_path=xacro_path,
        xacro_mappings=xacro_mappings,
        excitation_config=config_path,
        output_dir=output_dir,
        dataset_source='synthetic',
        base_set=base_set,
    )


def write_identification_yaml(
    path: Path,
    *,
    model,
    result: IdentificationResult,
    model_source: str,
    excitation_config: Path,
) -> None:
    friction_dict = friction_params_to_dict(result.friction_parameters)
    payload = {
        'description': (
            'Five-link symbolic BIP (link1–link5, gripper excluded) + '
            'Coulomb/viscous friction (Fv, Fc per joint). '
            'Solved with scipy.optimize.least_squares (TRF).'
        ),
        'model_source': model_source,
        'excitation_config': str(excitation_config),
        'five_link_base_parameters': {
            **result.base_set.to_dict(),
            'reference_values': [float(value) for value in result.reference_values],
            'identified_values': [float(value) for value in result.base_parameters],
        },
        'optimizer': {
            'method': 'scipy.optimize.least_squares (trf, bounded)',
            'status': result.optimizer_status,
            'message': result.optimizer_message,
            'residual_norm': result.residual_norm,
            'cost': result.cost,
        },
        'regression_quality': {
            'linearized_matrix_rank': result.matrix_rank,
            'linearized_condition_number': result.condition_number,
            'num_samples': result.num_samples,
            'sample_dt_s': result.sample_dt_s,
        },
        'friction': {
            'equation': {joint: format_friction_equation(joint) for joint in ARM_JOINTS},
            'joints': friction_dict,
        },
    }
    with path.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(payload, handle, sort_keys=False, default_flow_style=False)


def print_identification_summary(model, result: IdentificationResult) -> None:
    print('\n=== Base dynamics + Coulomb/viscous friction identification ===')
    print(
        f'Five-link BIP count: {result.base_set.num_base} '
        f'(from {result.base_set.layout.num_std} standard inertia columns)'
    )
    print(f'residual_norm: {result.residual_norm:.6g}')
    print(f'cost: {result.cost:.6g}')
    print(
        f'linear_block_rank: {result.matrix_rank} '
        f'(base {result.base_set.num_base} + friction {NUM_FRICTION_PARAMS})'
    )
    print(f'linear_block_condition_number: {result.condition_number:.6g}')
    print(f'friction_params: {NUM_FRICTION_PARAMS} (Fv, Fc per joint)')
    print(f'optimizer: {result.optimizer_message} (status={result.optimizer_status})')

    print('\n--- Model ---')
    print('  tau = Y_base(q,q̇,q̈) pi_base + tau_friction(q̇; Fv, Fc)')
    print('  Unidentifiable Pinocchio columns lie in the regressor null space and do not affect tau.')

    print('\n--- Friction (per joint) ---')
    for line in format_parameter_meanings():
        print(f'  {line}')
    friction = friction_params_to_dict(result.friction_parameters)
    for joint in ARM_JOINTS:
        print(f'\n{joint}: {format_friction_equation(joint)}')
        params = friction[joint]
        print('  Fv={Fv:.5f}, Fc={Fc:.5f}'.format(**params))

    print('\n--- Five-link symbolic BIP (θ_id vs θ_ref) ---')
    for expr, ref, ident in zip(
        result.base_set.symbolic_expressions(),
        result.reference_values,
        result.base_parameters,
    ):
        print(f'  {expr} = {ident:.6g}  (ref {ref:.6g})')


def _format_float(value: float) -> str:
    if not math.isfinite(value):
        return 'nan'
    return f'{value:.6g}'


def write_report_html(
    path: Path,
    *,
    result: IdentificationResult,
    urdf_path: Path,
    excitation_config: Path,
    dataset_source: Optional[str] = None,
) -> None:
    timestamp = datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')
    friction = friction_params_to_dict(result.friction_parameters)
    friction_rows = []
    for joint in ARM_JOINTS:
        params = friction[joint]
        friction_rows.append(
            '<tr>'
            f'<td>{html.escape(joint)}</td>'
            + ''.join(f'<td>{_format_float(params[name])}</td>' for name in FRICTION_PARAM_NAMES)
            + '</tr>'
        )
    source_line = (
        f'<p class="meta">Dataset source: {html.escape(dataset_source)}</p>'
        if dataset_source
        else ''
    )
    document = f"""<!DOCTYPE html>
<html lang="en">
<head><meta charset="utf-8"><title>OMX Sysid Report</title></head>
<body>
  <h1>Base dynamics + Coulomb/viscous friction identification</h1>
  <p class="meta">Generated {html.escape(timestamp)}</p>
  <p class="meta">Model: {html.escape(str(urdf_path))}</p>
  <p class="meta">Excitation config: {html.escape(str(excitation_config))}</p>
  {source_line}
  <p>Base parameters: {result.base_set.num_base} identifiable columns</p>
  <p>Residual norm: {_format_float(result.residual_norm)}</p>
  <h2>Friction parameters (Fv, Fc)</h2>
  <table>
    <tr><th>Joint</th>{''.join(f'<th>{name}</th>' for name in FRICTION_PARAM_NAMES)}</tr>
    {''.join(friction_rows)}
  </table>
</body>
</html>"""
    path.write_text(document, encoding='utf-8')


def _add_model_source_arguments(parser: argparse.ArgumentParser) -> None:
    from open_manipulator_sysid.pinocchio_support.model_loader import _add_model_source_arguments as add_model_args

    add_model_args(parser)


def _model_source_kwargs_from_parsed(parsed: argparse.Namespace) -> Dict[str, object]:
    from open_manipulator_sysid.pinocchio_support.model_loader import _model_source_kwargs_from_parsed

    return _model_source_kwargs_from_parsed(parsed)


def main(args: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        description='Synthetic base-dynamics + Coulomb/viscous friction identification.'
    )
    _add_model_source_arguments(parser)
    parser.add_argument('--config', type=Path, default=None)
    parser.add_argument('--output-dir', type=Path, default=None)
    parser.add_argument('--periods', type=int, default=5)
    parser.add_argument('--discard-periods', type=int, default=1)
    parser.add_argument('--dt', type=float, default=0.01)
    parser.add_argument('--noise-std', type=float, default=0.005)
    parser.add_argument('--seed', type=int, default=0)
    parsed = parser.parse_args(args)
    run_synthetic_identification(
        **_model_source_kwargs_from_parsed(parsed),
        excitation_config=parsed.config,
        output_dir=parsed.output_dir,
        num_periods=parsed.periods,
        discard_periods=parsed.discard_periods,
        sample_dt_s=parsed.dt,
        noise_std=parsed.noise_std,
        rng_seed=parsed.seed,
    )


def main_bag(args: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        description='Identify base dynamics + Coulomb/viscous friction from bag or .npz dataset.'
    )
    _add_model_source_arguments(parser)
    parser.add_argument('--dataset', type=Path, default=None)
    parser.add_argument('--bag', type=Path, default=None)
    parser.add_argument('--config', type=Path, default=None)
    parser.add_argument('--output-dir', type=Path, default=None)
    parser.add_argument('--dt', type=float, default=0.01)
    parser.add_argument('--periods', type=int, default=5)
    parser.add_argument('--trim-start', type=float, default=None)
    parser.add_argument('--trim-end', type=float, default=None)
    parsed = parser.parse_args(args)

    if parsed.dataset is None and parsed.bag is None:
        parser.error('Provide --dataset or --bag')

    config_path = parsed.config or default_config_path()
    with Path(config_path).open('r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)

    if parsed.trim_start is None or parsed.trim_end is None:
        default_trim_start, default_trim_end = identification_trim_times(
            config,
            num_periods=parsed.periods,
        )
        trim_start = parsed.trim_start if parsed.trim_start is not None else default_trim_start
        trim_end = parsed.trim_end if parsed.trim_end is not None else default_trim_end
    else:
        trim_start = parsed.trim_start
        trim_end = parsed.trim_end

    if parsed.dataset is not None:
        dataset = IdentificationDataset.load_npz(parsed.dataset)
        dataset_source = str(parsed.dataset)
    else:
        dataset = bag_to_dataset(
            parsed.bag,
            sample_dt_s=parsed.dt,
            trim_start_s=trim_start,
            trim_end_s=trim_end,
            trim_end_reference='bag_start',
        )
        dataset_source = str(parsed.bag)

    run_identification(
        dataset,
        **_model_source_kwargs_from_parsed(parsed),
        excitation_config=config_path,
        output_dir=parsed.output_dir,
        dataset_source=dataset_source,
    )


if __name__ == '__main__':
    main()
