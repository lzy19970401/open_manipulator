"""Minimum identifiable (base) dynamic parameter set for OMX sysid."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Mapping, Optional, Sequence, Tuple

import numpy as np
from scipy.linalg import qr

from open_manipulator_sysid.pinocchio_support.numerics import SVD_RANK_RTOL, effective_condition_number
from open_manipulator_sysid.excitation_trajectory import ARM_JOINTS, ExcitationTrajectory
from open_manipulator_sysid.pinocchio_support.model_loader import (
    INERTIA_PARAM_NAMES,
    arm_velocity_indices,
    body_names,
    stack_state_from_sample,
)

try:
    import pinocchio as pin
except ImportError:  # pragma: no cover
    pin = None

@dataclass(frozen=True)
class BaseParameterSet:
    """Column indices into Pinocchio arm regressor selecting an independent set."""

    column_indices: Tuple[int, ...]
    rank: int
    num_full_columns: int

    @property
    def num_base(self) -> int:
        return len(self.column_indices)

    def labels(self, model) -> List[str]:
        names = body_names(model)
        labels: List[str] = []
        for index in self.column_indices:
            body_index = index // len(INERTIA_PARAM_NAMES)
            param_index = index % len(INERTIA_PARAM_NAMES)
            body = names[body_index] if body_index < len(names) else f'body{body_index}'
            labels.append(f'{body}.{INERTIA_PARAM_NAMES[param_index]}')
        return labels

    def project_regressor(self, arm_regressor: np.ndarray) -> np.ndarray:
        return arm_regressor[:, self.column_indices]

    def to_dict(self) -> dict:
        return {
            'rank': self.rank,
            'num_full_columns': self.num_full_columns,
            'column_indices': list(self.column_indices),
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> 'BaseParameterSet':
        indices = tuple(int(value) for value in payload['column_indices'])
        return cls(
            column_indices=indices,
            rank=int(payload['rank']),
            num_full_columns=int(payload['num_full_columns']),
        )


def require_pinocchio() -> None:
    if pin is None:
        raise ImportError('Pinocchio is required for base parameter extraction.')


def stack_arm_regressor_from_trajectory(
    trajectory: ExcitationTrajectory,
    model,
    data,
    *,
    num_samples: int = 200,
) -> np.ndarray:
    """Stack arm regressor rows Y(q, q̇, q̈) over one excitation period."""
    require_pinocchio()
    arm_indices = arm_velocity_indices(model)
    rows: List[np.ndarray] = []
    for index in range(num_samples):
        time_s = trajectory.period_s * index / num_samples
        sample = trajectory.sample(time_s)
        q, velocity, acceleration = stack_state_from_sample(model, sample, arm_indices)
        regressor = pin.computeJointTorqueRegressor(
            model, data, q, velocity, acceleration
        )
        rows.append(regressor[arm_indices, :])
    return np.vstack(rows)


def compute_base_parameter_set(
    stacked_regressor: np.ndarray,
    *,
    rank_tol: float = SVD_RANK_RTOL,
) -> BaseParameterSet:
    """Select independent columns via QR pivoting on the stacked regressor."""
    if stacked_regressor.ndim != 2:
        raise ValueError('stacked_regressor must be 2-D')
    Q, R, pivot = qr(stacked_regressor, pivoting=True, mode='economic')
    diag_r = np.abs(np.diag(R))
    if diag_r.size == 0:
        raise ValueError('Empty regressor matrix')
    threshold = rank_tol * diag_r[0]
    rank = int(np.sum(diag_r > threshold))
    if rank == 0:
        raise ValueError('Regressor has numerical rank zero')
    column_indices = tuple(sorted(int(pivot[index]) for index in range(rank)))
    return BaseParameterSet(
        column_indices=column_indices,
        rank=rank,
        num_full_columns=stacked_regressor.shape[1],
    )


def base_set_from_trajectory(
    trajectory: ExcitationTrajectory,
    model,
    data,
    *,
    num_samples: int = 200,
) -> BaseParameterSet:
    stacked = stack_arm_regressor_from_trajectory(
        trajectory,
        model,
        data,
        num_samples=num_samples,
    )
    return compute_base_parameter_set(stacked)


def initial_base_parameters(
    stacked_regressor: np.ndarray,
    base_set: BaseParameterSet,
    target_torque: np.ndarray,
) -> np.ndarray:
    """Least-squares π_base₀ so Y_base π ≈ τ over stacked samples."""
    base_regressor = stacked_regressor[:, base_set.column_indices]
    solution, _, _, _ = np.linalg.lstsq(base_regressor, target_torque, rcond=None)
    return solution


def stack_dynamics_torque_from_dataset(
    model,
    data,
    dataset,
    arm_indices: Sequence[int],
) -> np.ndarray:
    """Return stacked measured torque (4 * num_samples,) — for init diagnostics."""
    return dataset.torque.reshape(-1)


def stack_inertial_torque_nominal(
    model,
    data,
    dataset,
    arm_indices: Sequence[int],
    pi_nominal: np.ndarray,
) -> np.ndarray:
    """Stack Y_full @ π_nominal for each sample (matches RNEA if π is consistent)."""
    require_pinocchio()
    torques: List[np.ndarray] = []
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
        torques.append(regressor[arm_indices, :] @ pi_nominal)
    return np.concatenate(torques)


def stack_arm_regressor_from_dataset(
    model,
    data,
    dataset: 'IdentificationDataset',
    arm_indices: Sequence[int],
) -> np.ndarray:
    """Stack full-column arm regressor rows from identification samples."""
    require_pinocchio()
    from open_manipulator_sysid.excitation_trajectory import ARM_JOINTS

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
        regressor = pin.computeJointTorqueRegressor(
            model, data, q, velocity, acceleration
        )
        rows.append(regressor[arm_indices, :])
    return np.vstack(rows)


def base_set_from_excitation_data(
    trajectory: ExcitationTrajectory,
    model,
    data,
    dataset,
    *,
    trajectory_samples: int = 200,
    max_dataset_samples: int = 300,
) -> BaseParameterSet:
    """Extract base columns using both one excitation period and recorded data."""
    arm_indices = arm_velocity_indices(model)
    dataset_rows = stack_arm_regressor_from_dataset(model, data, dataset, arm_indices)
    if dataset_rows.shape[0] > max_dataset_samples:
        indices = np.linspace(0, dataset_rows.shape[0] - 1, max_dataset_samples, dtype=int)
        dataset_rows = dataset_rows[indices]
    stacked = np.vstack(
        [
            stack_arm_regressor_from_trajectory(
                trajectory,
                model,
                data,
                num_samples=trajectory_samples,
            ),
            dataset_rows,
        ]
    )
    return compute_base_parameter_set(stacked)

