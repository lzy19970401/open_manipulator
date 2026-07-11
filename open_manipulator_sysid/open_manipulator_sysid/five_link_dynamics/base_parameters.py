"""Five-link (link1–link5) symbolic base inertial parameters for OMX sysid.

Gripper links and end_effector_link are excluded from the inertial regressor.
Identifiable combinations are derived from an SVD of the stacked arm regressor
over the excitation trajectory (Swevers-style linear combinations, not QR
single-column selection).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from open_manipulator_sysid.excitation_trajectory import ARM_JOINTS, ExcitationTrajectory
from open_manipulator_sysid.excitation_recording.dataset import IdentificationDataset
from open_manipulator_sysid.pinocchio_support.model_loader import (
    INERTIA_PARAM_NAMES,
    SVD_RANK_RTOL,
    arm_velocity_indices,
    body_names,
    nominal_dynamic_parameters,
    require_pinocchio,
    stack_state_from_sample,
)

try:
    import pinocchio as pin
except ImportError:  # pragma: no cover
    pin = None

FIVE_LINK_NAMES: Tuple[str, ...] = ('link1', 'link2', 'link3', 'link4', 'link5')
# Pinocchio OMX model names movable bodies joint1..joint4 (link2..link5 inertia).
# link1 inertia sits on universe (body 0) and has no regressor columns (fixed base).
PINOCCHIO_BODY_TO_LINK: Dict[str, str] = {
    'joint1': 'link2',
    'joint2': 'link3',
    'joint3': 'link4',
    'joint4': 'link5',
}
EXCLUDED_INERTIAL_BODIES: Tuple[str, ...] = (
    'gripper_left_joint',
    'gripper_right_joint',
    'gripper_left_link',
    'gripper_right_link',
    'end_effector_link',
)

PARAM_UNITS: Dict[str, str] = {
    'mass': 'kg',
    'mc_x': 'kg·m',
    'mc_y': 'kg·m',
    'mc_z': 'kg·m',
    'Ixx': 'kg·m²',
    'Ixy': 'kg·m²',
    'Iyy': 'kg·m²',
    'Ixz': 'kg·m²',
    'Iyz': 'kg·m²',
    'Izz': 'kg·m²',
}

SYMBOLIC_TERM_THRESHOLD = 1e-6


@dataclass(frozen=True)
class FiveLinkStdLayout:
    """Maps five-link standard Pinocchio columns into the full model vector."""

    body_names: Tuple[str, ...]
    full_column_indices: Tuple[int, ...]
    five_link_column_indices: Tuple[int, ...]

    @property
    def num_std(self) -> int:
        return len(self.five_link_column_indices)

    def std_labels(self) -> List[str]:
        labels: List[str] = []
        for index in self.five_link_column_indices:
            body_index = index // len(INERTIA_PARAM_NAMES)
            param_index = index % len(INERTIA_PARAM_NAMES)
            pin_name = self.body_names[body_index]
            link_name = PINOCCHIO_BODY_TO_LINK.get(pin_name, pin_name)
            labels.append(f'{link_name}.{INERTIA_PARAM_NAMES[param_index]}')
        return labels


@dataclass(frozen=True)
class FiveLinkBaseParameterSet:
    """Minimum identifiable inertial combinations for link1–link5 only."""

    rank: int
    layout: FiveLinkStdLayout
    combination_matrix: np.ndarray  # (rank, num_std_five_link); rows are combo weights
    singular_values: np.ndarray

    @property
    def num_base(self) -> int:
        return self.rank

    def project_regressor(self, five_link_regressor: np.ndarray) -> np.ndarray:
        """Map (…, num_std) five-link regressor to (…, rank) combo regressor."""
        return five_link_regressor @ self.combination_matrix.T

    def nominal_base_values(self, pi_full_nominal: np.ndarray) -> np.ndarray:
        pi_five = pi_full_nominal[list(self.layout.five_link_column_indices)]
        return self.combination_matrix @ pi_five

    def symbolic_expressions(self) -> List[str]:
        std_labels = self.layout.std_labels()
        expressions: List[str] = []
        for row_index in range(self.rank):
            expressions.append(
                _format_linear_combination(
                    self.combination_matrix[row_index],
                    std_labels,
                )
            )
        return expressions

    def units(self) -> List[str]:
        """Heuristic unit per combo row (mixed terms → kg·m²)."""
        std_labels = self.layout.std_labels()
        result: List[str] = []
        for row_index in range(self.rank):
            result.append(_infer_combo_unit(self.combination_matrix[row_index], std_labels))
        return result

    def to_dict(self) -> dict:
        return {
            'rank': self.rank,
            'num_std_five_link': self.layout.num_std,
            'five_link_bodies': list(FIVE_LINK_NAMES),
        'regressor_bodies': list(PINOCCHIO_BODY_TO_LINK.values()),
        'link1_note': (
            'link1 inertia is on Pinocchio universe (fixed base) and is not a column '
            'in computeJointTorqueRegressor; combinations use link2..link5 only.'
        ),
            'excluded_bodies': list(EXCLUDED_INERTIAL_BODIES),
            'combination_matrix': self.combination_matrix.tolist(),
            'singular_values': self.singular_values.tolist(),
            'symbolic_expressions': self.symbolic_expressions(),
            'units': self.units(),
            'std_labels': self.layout.std_labels(),
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, object], layout: FiveLinkStdLayout) -> 'FiveLinkBaseParameterSet':
        matrix = np.asarray(payload['combination_matrix'], dtype=float)
        singular_values = np.asarray(payload.get('singular_values', []), dtype=float)
        rank = int(payload['rank'])
        if matrix.shape != (rank, layout.num_std):
            raise ValueError(
                f'combination_matrix shape {matrix.shape} does not match rank={rank} '
                f'and num_std={layout.num_std}'
            )
        return cls(
            rank=rank,
            layout=layout,
            combination_matrix=matrix,
            singular_values=singular_values,
        )


def five_link_std_layout(model) -> FiveLinkStdLayout:
    names = tuple(body_names(model))
    full_indices: List[int] = []
    five_indices: List[int] = []
    for body_index, pin_name in enumerate(names):
        offset = body_index * len(INERTIA_PARAM_NAMES)
        chunk = list(range(offset, offset + len(INERTIA_PARAM_NAMES)))
        if pin_name in PINOCCHIO_BODY_TO_LINK:
            full_indices.extend(chunk)
            five_indices.extend(chunk)
        elif pin_name in EXCLUDED_INERTIAL_BODIES:
            continue
        else:
            raise ValueError(
                f'Unexpected Pinocchio body {pin_name!r} in OMX model; '
                f'expected joint1..joint4 or excluded gripper bodies'
            )
    if len(five_indices) != len(PINOCCHIO_BODY_TO_LINK) * len(INERTIA_PARAM_NAMES):
        raise ValueError(
            f'Expected {len(PINOCCHIO_BODY_TO_LINK) * len(INERTIA_PARAM_NAMES)} '
            f'arm link columns, got {len(five_indices)}'
        )
    return FiveLinkStdLayout(
        body_names=names,
        full_column_indices=tuple(full_indices),
        five_link_column_indices=tuple(five_indices),
    )


def five_link_nominal_std_vector(model) -> np.ndarray:
    pi_full = nominal_dynamic_parameters(model)
    layout = five_link_std_layout(model)
    return pi_full[list(layout.five_link_column_indices)]


def stack_five_link_arm_regressor_from_trajectory(
    trajectory: ExcitationTrajectory,
    model,
    data,
    layout: FiveLinkStdLayout,
    *,
    num_samples: int = 200,
) -> np.ndarray:
    require_pinocchio()
    arm_indices = arm_velocity_indices(model)
    rows: List[np.ndarray] = []
    for index in range(num_samples):
        time_s = trajectory.period_s * index / num_samples
        sample = trajectory.sample(time_s)
        q, velocity, acceleration = stack_state_from_sample(model, sample, arm_indices)
        regressor = pin.computeJointTorqueRegressor(model, data, q, velocity, acceleration)
        arm_regressor = regressor[arm_indices, :][:, list(layout.five_link_column_indices)]
        rows.append(arm_regressor)
    return np.vstack(rows)


def stack_five_link_arm_regressor_from_dataset(
    model,
    data,
    dataset: IdentificationDataset,
    layout: FiveLinkStdLayout,
    arm_indices: Sequence[int],
) -> np.ndarray:
    require_pinocchio()
    rows: List[np.ndarray] = []
    for index in range(dataset.num_samples):
        q = pin.neutral(model)
        velocity = np.zeros(model.nv)
        acceleration = np.zeros(model.nv)
        for joint_index, joint_name in enumerate(ARM_JOINTS):
            idx = arm_indices[joint_index]
            q[idx] = dataset.position[index, joint_index]
            velocity[idx] = dataset.velocity[index, joint_index]
            acceleration[idx] = dataset.acceleration[index, joint_index]
        regressor = pin.computeJointTorqueRegressor(model, data, q, velocity, acceleration)
        rows.append(regressor[arm_indices, :][:, list(layout.five_link_column_indices)])
    return np.vstack(rows)


def compute_five_link_base_parameter_set(
    stacked_regressor: np.ndarray,
    *,
    rank_tol: float = SVD_RANK_RTOL,
) -> Tuple[np.ndarray, np.ndarray, int]:
    """Return (combination_matrix, singular_values, rank) from stacked Y."""
    if stacked_regressor.ndim != 2:
        raise ValueError('stacked_regressor must be 2-D')
    _, singular_values, vh = np.linalg.svd(stacked_regressor, full_matrices=False)
    if singular_values.size == 0:
        raise ValueError('Empty regressor matrix')
    threshold = rank_tol * singular_values[0]
    rank = int(np.sum(singular_values > threshold))
    if rank == 0:
        raise ValueError('Regressor has numerical rank zero')
    combination_matrix = vh[:rank, :]
    return combination_matrix, singular_values, rank


def five_link_base_set_from_stacked_regressor(
    stacked_regressor: np.ndarray,
    model,
) -> FiveLinkBaseParameterSet:
    """Build combo base set from a pre-stacked five-link regressor matrix."""
    layout = five_link_std_layout(model)
    combination_matrix, singular_values, rank = compute_five_link_base_parameter_set(
        stacked_regressor
    )
    return FiveLinkBaseParameterSet(
        rank=rank,
        layout=layout,
        combination_matrix=combination_matrix,
        singular_values=singular_values,
    )


def five_link_base_set_from_trajectory(
    trajectory: ExcitationTrajectory,
    model,
    data,
    *,
    num_samples: int = 200,
) -> FiveLinkBaseParameterSet:
    layout = five_link_std_layout(model)
    stacked = stack_five_link_arm_regressor_from_trajectory(
        trajectory,
        model,
        data,
        layout,
        num_samples=num_samples,
    )
    return five_link_base_set_from_stacked_regressor(stacked, model)


def five_link_base_set_from_excitation_data(
    trajectory: ExcitationTrajectory,
    model,
    data,
    dataset: IdentificationDataset,
    *,
    trajectory_samples: int = 200,
    max_dataset_samples: int = 300,
) -> FiveLinkBaseParameterSet:
    layout = five_link_std_layout(model)
    arm_indices = arm_velocity_indices(model)
    dataset_rows = stack_five_link_arm_regressor_from_dataset(
        model,
        data,
        dataset,
        layout,
        arm_indices,
    )
    if dataset_rows.shape[0] > max_dataset_samples:
        indices = np.linspace(0, dataset_rows.shape[0] - 1, max_dataset_samples, dtype=int)
        dataset_rows = dataset_rows[indices]
    stacked = np.vstack(
        [
            stack_five_link_arm_regressor_from_trajectory(
                trajectory,
                model,
                data,
                layout,
                num_samples=trajectory_samples,
            ),
            dataset_rows,
        ]
    )
    combination_matrix, singular_values, rank = compute_five_link_base_parameter_set(stacked)
    return FiveLinkBaseParameterSet(
        rank=rank,
        layout=layout,
        combination_matrix=combination_matrix,
        singular_values=singular_values,
    )


def reference_base_values(model, base_set: FiveLinkBaseParameterSet) -> np.ndarray:
    pi_full = nominal_dynamic_parameters(model)
    return base_set.nominal_base_values(pi_full)


def _format_linear_combination(weights: np.ndarray, labels: Sequence[str]) -> str:
    terms: List[str] = []
    for weight, label in zip(weights, labels):
        if abs(weight) < SYMBOLIC_TERM_THRESHOLD:
            continue
        if abs(weight - 1.0) < 1e-9:
            terms.append(f'+ {label}')
        elif abs(weight + 1.0) < 1e-9:
            terms.append(f'- {label}')
        elif weight < 0:
            terms.append(f'- {abs(weight):.6g}·{label}')
        else:
            terms.append(f'+ {weight:.6g}·{label}')
    if not terms:
        return '0'
    expression = ' '.join(terms)
    if expression.startswith('+ '):
        expression = expression[2:]
    return expression


def _infer_combo_unit(weights: np.ndarray, labels: Sequence[str]) -> str:
    units = set()
    for weight, label in zip(weights, labels):
        if abs(weight) < SYMBOLIC_TERM_THRESHOLD:
            continue
        param = label.split('.')[-1]
        units.add(PARAM_UNITS.get(param, 'kg·m²'))
    if len(units) == 1:
        return units.pop()
    return 'kg·m²'
