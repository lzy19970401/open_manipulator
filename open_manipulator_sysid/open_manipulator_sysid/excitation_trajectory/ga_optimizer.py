from __future__ import annotations

"""Genetic-algorithm optimization of Fourier (a,b) excitation coefficients.

Minimizes the column-scaled condition number of the stacked five-link BIP +
Coulomb/viscous friction regressor over one excitation period, subject to joint
limits, velocity/acceleration caps, and end-effector height. Reports raw κ for
diagnostics.
"""

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from open_manipulator_sysid.system_identification.friction import (
    NUM_FRICTION_PARAMS,
    build_coulomb_viscous_block,
)
from open_manipulator_sysid.excitation_trajectory import (
    ARM_JOINTS,
    COEFFICIENTS_SOURCE_TEMPLATE,
    ExcitationTrajectory,
    SafetyViolation,
    default_config_path,
)
from open_manipulator_sysid.five_link_dynamics.base_parameters import (
    FiveLinkBaseParameterSet,
    five_link_base_set_from_stacked_regressor,
    five_link_std_layout,
    stack_five_link_arm_regressor_from_trajectory,
)
from open_manipulator_sysid.excitation_trajectory.kinematics_constraints import (
    default_end_effector_frame,
    end_effector_z_world,
)
from open_manipulator_sysid.pinocchio_support.numerics import SVD_RANK_RTOL, effective_condition_number
from open_manipulator_sysid.pinocchio_support.model_loader import arm_velocity_indices, stack_state_from_sample

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover
    pin = None
    _PINOCCHIO_IMPORT_ERROR = exc
else:
    _PINOCCHIO_IMPORT_ERROR = None

EE_Z_PENALTY_WEIGHT = 1.0e8


@dataclass(frozen=True)
class GAParameters:
    population_size: int = 80
    generations: int = 120
    elite_count: int = 4
    crossover_rate: float = 0.7
    mutation_rate: float = 0.15
    mutation_sigma: float = 0.12
    samples_per_period: int = 60
    tournament_size: int = 4
    seed: int = 42


@dataclass(frozen=True)
class ObservationConditionMetrics:
    matrix_rank: int
    condition_number_raw: float
    condition_number_scaled: float


@dataclass(frozen=True)
class GAResult:
    a_coefficients: Dict[str, List[float]]
    b_coefficients: Dict[str, List[float]]
    coefficient_scale: float
    condition_number_raw: float
    condition_number_scaled: float
    matrix_rank: int
    best_fitness: float
    generations_run: int
    base_parameter_set: Optional[FiveLinkBaseParameterSet] = None

    @property
    def condition_number(self) -> float:
        """Scaled κ (GA objective); kept for backward-compatible callers."""
        return self.condition_number_scaled


def require_pinocchio() -> None:
    if pin is None:
        raise ImportError(
            'Pinocchio is required for GA excitation optimization. '
            'Install ros-jazzy-pinocchio inside the Docker container.'
        ) from _PINOCCHIO_IMPORT_ERROR


def column_scaled_observation_matrix(matrix: np.ndarray) -> np.ndarray:
    """Scale each regressor column to unit L2 norm (experimental-design convention)."""
    column_norms = np.linalg.norm(matrix, axis=0)
    column_norms = np.where(column_norms < 1e-15, 1.0, column_norms)
    return matrix / column_norms


def observation_condition_metrics(matrix: np.ndarray) -> ObservationConditionMetrics:
    """Return rank plus raw and column-scaled condition numbers."""
    rank, condition_number_raw = effective_condition_number(matrix)
    _, condition_number_scaled = effective_condition_number(
        column_scaled_observation_matrix(matrix)
    )
    return ObservationConditionMetrics(
        matrix_rank=rank,
        condition_number_raw=condition_number_raw,
        condition_number_scaled=condition_number_scaled,
    )


def build_observation_matrix(
    trajectory: ExcitationTrajectory,
    model,
    data,
    *,
    num_samples: int,
    base_set: FiveLinkBaseParameterSet,
    include_friction: bool = True,
) -> np.ndarray:
    """Stack [Y_BIP | Y_Fv,Fc] rows over one excitation period (four joints per sample)."""
    require_pinocchio()
    layout = base_set.layout
    arm_indices = arm_velocity_indices(model)
    rows: List[np.ndarray] = []

    for index in range(num_samples):
        time_s = trajectory.period_s * index / num_samples
        sample = trajectory.sample(time_s)
        q, velocity, acceleration = stack_state_from_sample(model, sample, arm_indices)
        regressor = pin.computeJointTorqueRegressor(
            model, data, q, velocity, acceleration
        )
        five_link_regressor = regressor[arm_indices, :][:, list(layout.five_link_column_indices)]
        bip_regressor = base_set.project_regressor(five_link_regressor)
        arm_velocity = velocity[arm_indices]

        if include_friction:
            friction_block = build_coulomb_viscous_block(arm_velocity)
            row = np.hstack([bip_regressor, friction_block])
        else:
            row = bip_regressor
        rows.append(row)

    return np.vstack(rows)


def _coefficient_bounds(config: Mapping[str, object]) -> Tuple[float, List[float], List[float]]:
    """Per-harmonic (a, b) magnitude caps derived from velocity budget."""
    excitation = config['excitation']
    safety = config['safety']
    num_harmonics = int(excitation['num_harmonics'])
    amp_fraction = float(excitation.get('amplitude_fraction', 0.5))
    vel_span = amp_fraction * float(safety['max_velocity_rad_s'])
    a_bounds = [vel_span / harmonic for harmonic in range(1, num_harmonics + 1)]
    b_bounds = list(a_bounds)
    return vel_span, a_bounds, b_bounds


def _decode_individual(
    genes: np.ndarray,
    config: Mapping[str, object],
) -> Tuple[Dict[str, List[float]], Dict[str, List[float]]]:
    excitation = config['excitation']
    num_harmonics = int(excitation['num_harmonics'])
    _, a_bounds, b_bounds = _coefficient_bounds(config)
    a_coeffs: Dict[str, List[float]] = {}
    b_coeffs: Dict[str, List[float]] = {}
    gene_index = 0

    for joint in ARM_JOINTS:
        joint_a: List[float] = []
        joint_b: List[float] = []
        for harmonic_index in range(num_harmonics):
            a_gene = float(genes[gene_index])
            b_gene = float(genes[gene_index + 1])
            gene_index += 2
            a_gene = min(1.0, max(0.0, a_gene))
            b_gene = min(1.0, max(0.0, b_gene))
            a_scale = a_bounds[harmonic_index]
            b_scale = b_bounds[harmonic_index]
            joint_a.append((2.0 * a_gene - 1.0) * a_scale)
            joint_b.append((2.0 * b_gene - 1.0) * b_scale)
        a_coeffs[joint] = joint_a
        b_coeffs[joint] = joint_b

    return a_coeffs, b_coeffs


def _config_for_template_seed(config: Mapping[str, object]) -> Dict[str, object]:
    """Strip GA coefficients so ExcitationTrajectory builds the analytic template."""
    merged = copy.deepcopy(dict(config))
    excitation = dict(merged.get('excitation', {}))
    excitation['coefficients_source'] = COEFFICIENTS_SOURCE_TEMPLATE
    merged['excitation'] = excitation
    merged.pop('ga_coefficients', None)
    merged.pop('ga_coefficient_scale', None)
    merged.pop('ga_metadata', None)
    return merged


def _encode_template_individual(config: Mapping[str, object]) -> np.ndarray:
    """Seed population from normalized template (a,b) coefficients."""
    template = ExcitationTrajectory.from_config(
        _config_for_template_seed(config)
    )
    _, a_bounds, b_bounds = _coefficient_bounds(config)
    genes = np.zeros(2 * len(ARM_JOINTS) * len(a_bounds), dtype=float)
    gene_index = 0
    for joint in ARM_JOINTS:
        for harmonic_index, (a_val, b_val) in enumerate(
            zip(template.a_coefficients[joint], template.b_coefficients[joint])
        ):
            a_bound = a_bounds[harmonic_index]
            b_bound = b_bounds[harmonic_index]
            genes[gene_index] = 0.5 * (a_val / a_bound + 1.0) if a_bound > 0 else 0.5
            genes[gene_index + 1] = 0.5 * (b_val / b_bound + 1.0) if b_bound > 0 else 0.5
            gene_index += 2
    return np.clip(genes, 0.0, 1.0)


def _trajectory_from_genes(
    genes: np.ndarray,
    config: Mapping[str, object],
) -> ExcitationTrajectory:
    a_coeffs, b_coeffs = _decode_individual(genes, config)
    merged = copy.deepcopy(dict(config))
    merged['ga_coefficients'] = {
        joint: {'a': a_coeffs[joint], 'b': b_coeffs[joint]} for joint in ARM_JOINTS
    }
    merged['ga_coefficient_scale'] = 1.0
    excitation = dict(merged.get('excitation', {}))
    excitation.pop('coefficients_source', None)
    merged['excitation'] = excitation
    return ExcitationTrajectory.from_config(merged)


def _end_effector_penalty(
    trajectory: ExcitationTrajectory,
    model,
    data,
    arm_indices: Sequence[int],
    *,
    num_samples: int,
) -> float:
    z_min = trajectory.end_effector_z_min_m
    frame = trajectory.end_effector_frame or default_end_effector_frame()
    violation = 0.0
    for index in range(num_samples):
        time_s = trajectory.period_s * index / num_samples
        sample = trajectory.sample(time_s)
        z = end_effector_z_world(
            model,
            data,
            sample.position,
            arm_indices,
            frame_name=frame,
        )
        if z < z_min:
            violation += (z_min - z) ** 2
    return EE_Z_PENALTY_WEIGHT * violation


def _derive_ga_base_set(
    config: Mapping[str, object],
    model,
    data,
    *,
    num_samples: int,
    rng: np.random.Generator,
) -> FiveLinkBaseParameterSet:
    """Stack template + optional saved GA + probe trajectories for full-rank BIP.

    The analytic template Fourier alone yields a lower SVD rank (~19) on link2–link5
    columns; GA must optimize κ on the richest identifiable combo basis (~22 + 8 friction).
    """
    layout = five_link_std_layout(model)
    regressor_blocks: List[np.ndarray] = []

    template_traj = ExcitationTrajectory.from_config(
        _config_for_template_seed(config)
    )
    regressor_blocks.append(
        stack_five_link_arm_regressor_from_trajectory(
            template_traj,
            model,
            data,
            layout,
            num_samples=num_samples,
        )
    )

    if config.get('ga_coefficients'):
        ga_traj = ExcitationTrajectory.from_config(config)
        regressor_blocks.append(
            stack_five_link_arm_regressor_from_trajectory(
                ga_traj,
                model,
                data,
                layout,
                num_samples=num_samples,
            )
        )

    template_genes = _encode_template_individual(config)
    probes_added = 0
    attempts = 0
    while probes_added < 2 and attempts < 12:
        attempts += 1
        genes = rng.uniform(0.2, 1.0, size=len(template_genes))
        try:
            probe_traj = _trajectory_from_genes(genes, config)
            probe_traj.validate_period(num_samples=max(num_samples, 100))
        except (SafetyViolation, ValueError):
            continue
        regressor_blocks.append(
            stack_five_link_arm_regressor_from_trajectory(
                probe_traj,
                model,
                data,
                layout,
                num_samples=num_samples,
            )
        )
        probes_added += 1

    stacked = np.vstack(regressor_blocks)
    return five_link_base_set_from_stacked_regressor(stacked, model)


def evaluate_genes(
    genes: np.ndarray,
    config: Mapping[str, object],
    model,
    data,
    *,
    num_samples: int,
    base_set: FiveLinkBaseParameterSet,
    arm_indices: Sequence[int],
) -> float:
    """Return fitness (lower is better). Invalid trajectories get a large penalty."""
    try:
        trajectory = _trajectory_from_genes(genes, config)
        trajectory.validate_period(num_samples=max(num_samples, 100))
    except (SafetyViolation, ValueError):
        return 1.0e12

    penalty = _end_effector_penalty(
        trajectory,
        model,
        data,
        arm_indices,
        num_samples=max(num_samples, 40),
    )
    if penalty > 1.0e6:
        return penalty

    matrix = build_observation_matrix(
        trajectory,
        model,
        data,
        num_samples=num_samples,
        base_set=base_set,
    )
    metrics = observation_condition_metrics(matrix)
    min_rank = base_set.num_base + NUM_FRICTION_PARAMS
    if metrics.matrix_rank < min(min_rank, matrix.shape[1]):
        return 1.0e9 + (min_rank - metrics.matrix_rank) * 1.0e6 + penalty
    return metrics.condition_number_scaled + penalty


def _tournament_select(
    population: Sequence[np.ndarray],
    fitness: Sequence[float],
    *,
    tournament_size: int,
    rng: np.random.Generator,
) -> np.ndarray:
    indices = rng.integers(0, len(population), size=tournament_size)
    best_index = min(indices, key=lambda index: fitness[index])
    return population[best_index].copy()


def _crossover(
    parent_a: np.ndarray,
    parent_b: np.ndarray,
    *,
    crossover_rate: float,
    rng: np.random.Generator,
) -> Tuple[np.ndarray, np.ndarray]:
    if rng.random() >= crossover_rate:
        return parent_a.copy(), parent_b.copy()
    point = int(rng.integers(1, len(parent_a)))
    child_a = np.concatenate([parent_a[:point], parent_b[point:]])
    child_b = np.concatenate([parent_b[:point], parent_a[point:]])
    return child_a, child_b


def _mutate(
    individual: np.ndarray,
    *,
    mutation_rate: float,
    mutation_sigma: float,
    rng: np.random.Generator,
) -> np.ndarray:
    mutated = individual.copy()
    for index in range(len(mutated)):
        if rng.random() < mutation_rate:
            mutated[index] += rng.normal(0.0, mutation_sigma)
            mutated[index] = min(1.0, max(0.0, mutated[index]))
    return mutated


def run_genetic_algorithm(
    config: Mapping[str, object],
    model,
    data,
    *,
    ga_params: Optional[GAParameters] = None,
) -> GAResult:
    params = ga_params or GAParameters()
    rng = np.random.default_rng(params.seed)
    template_genes = _encode_template_individual(config)
    from open_manipulator_sysid.pinocchio_support.model_loader import arm_velocity_indices

    arm_indices = arm_velocity_indices(model)
    base_set = _derive_ga_base_set(
        config,
        model,
        data,
        num_samples=params.samples_per_period,
        rng=rng,
    )

    population = [template_genes.copy()]
    while len(population) < params.population_size:
        individual = template_genes.copy()
        for index in range(len(individual)):
            individual[index] = rng.uniform(0.2, 1.0)
        population.append(individual)

    best_genes = population[0].copy()
    best_fitness = float('inf')

    for _generation in range(params.generations):
        fitness = [
            evaluate_genes(
                individual,
                config,
                model,
                data,
                num_samples=params.samples_per_period,
                base_set=base_set,
                arm_indices=arm_indices,
            )
            for individual in population
        ]

        generation_best_index = int(np.argmin(fitness))
        if fitness[generation_best_index] < best_fitness:
            best_fitness = fitness[generation_best_index]
            best_genes = population[generation_best_index].copy()

        sorted_indices = np.argsort(fitness)
        next_population = [
            population[index].copy() for index in sorted_indices[: params.elite_count]
        ]

        while len(next_population) < params.population_size:
            parent_a = _tournament_select(
                population,
                fitness,
                tournament_size=params.tournament_size,
                rng=rng,
            )
            parent_b = _tournament_select(
                population,
                fitness,
                tournament_size=params.tournament_size,
                rng=rng,
            )
            child_a, child_b = _crossover(
                parent_a,
                parent_b,
                crossover_rate=params.crossover_rate,
                rng=rng,
            )
            next_population.append(
                _mutate(
                    child_a,
                    mutation_rate=params.mutation_rate,
                    mutation_sigma=params.mutation_sigma,
                    rng=rng,
                )
            )
            if len(next_population) < params.population_size:
                next_population.append(
                    _mutate(
                        child_b,
                        mutation_rate=params.mutation_rate,
                        mutation_sigma=params.mutation_sigma,
                        rng=rng,
                    )
                )

        population = next_population

    best_trajectory = _trajectory_from_genes(best_genes, config)
    a_coeffs, b_coeffs = _decode_individual(best_genes, config)
    scaled_a = {joint: list(a_coeffs[joint]) for joint in ARM_JOINTS}
    scaled_b = {joint: list(b_coeffs[joint]) for joint in ARM_JOINTS}
    matrix = build_observation_matrix(
        best_trajectory,
        model,
        data,
        num_samples=params.samples_per_period,
        base_set=base_set,
    )
    metrics = observation_condition_metrics(matrix)

    return GAResult(
        a_coefficients=scaled_a,
        b_coefficients=scaled_b,
        coefficient_scale=best_trajectory.coefficient_scale,
        condition_number_raw=metrics.condition_number_raw,
        condition_number_scaled=metrics.condition_number_scaled,
        matrix_rank=metrics.matrix_rank,
        best_fitness=best_fitness,
        generations_run=params.generations,
        base_parameter_set=base_set,
    )


def merge_ga_coefficients_into_config(
    base_config: Mapping[str, object],
    result: GAResult,
) -> Dict[str, object]:
    merged = copy.deepcopy(dict(base_config))
    merged['ga_coefficient_scale'] = 1.0
    excitation = dict(merged.get('excitation', {}))
    excitation.pop('coefficients_source', None)
    merged['excitation'] = excitation
    merged['ga_coefficients'] = {
        joint: {
            'a': result.a_coefficients[joint],
            'b': result.b_coefficients[joint],
        }
        for joint in ARM_JOINTS
    }
    merged['ga_metadata'] = {
        'condition_number': result.condition_number_scaled,
        'condition_number_scaled': result.condition_number_scaled,
        'condition_number_raw': result.condition_number_raw,
        'matrix_rank': result.matrix_rank,
        'generations': result.generations_run,
        'safety_coefficient_scale': result.coefficient_scale,
    }
    if result.base_parameter_set is not None:
        merged['ga_metadata']['base_parameter_set'] = result.base_parameter_set.to_dict()
    return merged


def write_ga_config(
    base_config_path: Path | str,
    output_path: Path | str,
    result: GAResult,
) -> Path:
    with Path(base_config_path).open('r', encoding='utf-8') as handle:
        base_config = yaml.safe_load(handle)
    merged = merge_ga_coefficients_into_config(base_config, result)
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(merged, handle, default_flow_style=False, sort_keys=False)
    return output


def optimize_from_yaml(
    config_path: Path | str,
    output_path: Path | str,
    *,
    ga_params: Optional[GAParameters] = None,
    urdf_path: Optional[Path | str] = None,
    xacro_path: Optional[Path | str] = None,
) -> GAResult:
    from open_manipulator_sysid.pinocchio_support.model_loader import load_model, resolve_model_load_args

    with Path(config_path).open('r', encoding='utf-8') as handle:
        base_config = yaml.safe_load(handle)

    model_path, load_mappings, _ = resolve_model_load_args(
        urdf_path=urdf_path,
        xacro_path=xacro_path,
    )
    model, data = load_model(model_path, xacro_mappings=load_mappings)
    result = run_genetic_algorithm(base_config, model, data, ga_params=ga_params)
    write_ga_config(config_path, output_path, result)
    return result


def main(args: Optional[Sequence[str]] = None) -> None:
    import argparse

    parser = argparse.ArgumentParser(
        description=(
            'Optimize OMX 4-DOF Swevers (a,b) excitation coefficients with a genetic '
            'algorithm (minimize column-scaled regressor condition number).'
        )
    )
    parser.add_argument(
        '--config',
        type=Path,
        default=None,
        help='Base excitation.yaml (default: package config)',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=None,
        help='Output yaml with ga_coefficients (default: overwrite --config)',
    )
    parser.add_argument('--population', type=int, default=GAParameters.population_size)
    parser.add_argument('--generations', type=int, default=GAParameters.generations)
    parser.add_argument('--samples', type=int, default=GAParameters.samples_per_period)
    parser.add_argument('--seed', type=int, default=GAParameters.seed)
    parser.add_argument(
        '--quick',
        action='store_true',
        help='Small GA run for smoke tests (population=20, generations=15)',
    )
    parser.add_argument('--xacro', type=Path, default=None)
    parsed = parser.parse_args(args)

    config_path = parsed.config or default_config_path()
    output_path = parsed.output if parsed.output is not None else config_path

    ga_params = GAParameters(
        population_size=20 if parsed.quick else parsed.population,
        generations=15 if parsed.quick else parsed.generations,
        samples_per_period=parsed.samples,
        seed=parsed.seed,
    )

    result = optimize_from_yaml(
        config_path,
        output_path,
        ga_params=ga_params,
        xacro_path=parsed.xacro,
    )

    print(f'Wrote GA excitation config: {output_path}')
    if result.base_parameter_set is not None:
        bip = result.base_parameter_set.num_base
        print(
            f'base_parameters: {bip} BIP + {NUM_FRICTION_PARAMS} friction '
            f'= {bip + NUM_FRICTION_PARAMS} columns'
        )
    print(f'condition_number_scaled: {result.condition_number_scaled:.6g}')
    print(f'condition_number_raw: {result.condition_number_raw:.6g}')
    print(f'matrix_rank: {result.matrix_rank}')
    print(f'coefficient_scale: {result.coefficient_scale:.4f}')
    for joint in ARM_JOINTS:
        a_vals = ', '.join(f'{value:.5f}' for value in result.a_coefficients[joint])
        b_vals = ', '.join(f'{value:.5f}' for value in result.b_coefficients[joint])
        print(f'  {joint}: a=[{a_vals}] b=[{b_vals}]')


if __name__ == '__main__':
    main()
