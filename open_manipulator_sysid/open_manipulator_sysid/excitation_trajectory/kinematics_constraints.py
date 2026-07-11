"""Pinocchio helpers for excitation constraints (end-effector height)."""

from __future__ import annotations

from typing import Mapping, Sequence

import numpy as np

from open_manipulator_sysid.excitation_trajectory import ARM_JOINTS

try:
    import pinocchio as pin
except ImportError:  # pragma: no cover
    pin = None


def require_pinocchio() -> None:
    if pin is None:
        raise ImportError('Pinocchio is required for forward kinematics checks.')


def default_end_effector_frame() -> str:
    return 'end_effector_link'


def arm_configuration_vector(
    model,
    position: Mapping[str, float],
    arm_indices: Sequence[int],
) -> np.ndarray:
    q = pin.neutral(model)
    for joint_index, joint_name in enumerate(ARM_JOINTS):
        q[arm_indices[joint_index]] = position[joint_name]
    return q


def end_effector_z_world(
    model,
    data,
    position: Mapping[str, float],
    arm_indices: Sequence[int],
    *,
    frame_name: str,
) -> float:
    """Return end-effector z coordinate in the world (base) frame."""
    require_pinocchio()
    q = arm_configuration_vector(model, position, arm_indices)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    frame_id = model.getFrameId(frame_name)
    return float(data.oMf[frame_id].translation[2])


def min_end_effector_z_over_samples(
    model,
    data,
    samples: Sequence,
    arm_indices: Sequence[int],
    *,
    frame_name: str,
    num_check_samples: int = 80,
) -> float:
    """Minimum EE z over trajectory samples (uses sample.time_s ordering)."""
    if not samples:
        return float('inf')
    if len(samples) <= num_check_samples:
        indices = range(len(samples))
    else:
        indices = np.linspace(0, len(samples) - 1, num_check_samples, dtype=int)
    z_min = float('inf')
    for index in indices:
        sample = samples[index]
        z = end_effector_z_world(
            model,
            data,
            sample.position,
            arm_indices,
            frame_name=frame_name,
        )
        z_min = min(z_min, z)
    return z_min
