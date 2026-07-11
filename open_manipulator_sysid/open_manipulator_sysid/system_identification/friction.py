"""Linear Coulomb + viscous joint friction for OMX arm sysid (4×2 parameters)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np

from open_manipulator_sysid.excitation_trajectory import ARM_JOINTS

FRICTION_PARAM_NAMES = ('Fv', 'Fc')
NUM_FRICTION_PARAMS_PER_JOINT = len(FRICTION_PARAM_NAMES)
NUM_FRICTION_PARAMS = len(ARM_JOINTS) * NUM_FRICTION_PARAMS_PER_JOINT


@dataclass(frozen=True)
class CoulombViscousFriction:
    """Per-joint viscous Fv and Coulomb Fc reference / initial values (N·m, N·m·s/rad)."""

    fv: np.ndarray
    fc: np.ndarray

    @classmethod
    def from_config(cls, config: Mapping[str, object], num_joints: int = 4) -> 'CoulombViscousFriction':
        friction = config.get('friction', {})
        return cls(
            fv=_as_joint_vector(friction.get('Fv'), num_joints, default=0.0),
            fc=_as_joint_vector(friction.get('Fc'), num_joints, default=0.0),
        )


def _as_joint_vector(
    value: object,
    num_joints: int,
    *,
    default: float,
) -> np.ndarray:
    if value is None:
        return np.full(num_joints, default, dtype=float)
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.size == 1:
        return np.full(num_joints, float(array[0]), dtype=float)
    if array.size != num_joints:
        raise ValueError(f'Expected {num_joints} joint values, got {array.size}')
    return array


def coulomb_viscous_regressor_row(qd: float) -> np.ndarray:
    """Regressor row for [Fv, Fc]: tau_f = Fv * qd + Fc * sign(qd)."""
    sign_qd = math.copysign(1.0, qd) if abs(qd) > 1e-15 else 0.0
    return np.array([float(qd), sign_qd], dtype=float)


def coulomb_viscous_torque(qd: float, fv: float, fc: float) -> float:
    row = coulomb_viscous_regressor_row(qd)
    return float(row @ np.array([fv, fc], dtype=float))


def coulomb_viscous_torque_arm(
    arm_velocity: np.ndarray,
    params_per_joint: np.ndarray,
) -> np.ndarray:
    """Return length-4 friction torque; params_per_joint shape (4, 2) as [Fv, Fc]."""
    torques = np.zeros(len(ARM_JOINTS), dtype=float)
    for joint_index in range(len(ARM_JOINTS)):
        fv, fc = params_per_joint[joint_index]
        torques[joint_index] = coulomb_viscous_torque(
            float(arm_velocity[joint_index]),
            float(fv),
            float(fc),
        )
    return torques


def build_coulomb_viscous_block(arm_velocity: np.ndarray) -> np.ndarray:
    """Return (4, 8) block: joint i uses columns [2i, 2i+1] for [Fv_i, Fc_i]."""
    block = np.zeros(
        (len(ARM_JOINTS), len(ARM_JOINTS) * NUM_FRICTION_PARAMS_PER_JOINT),
        dtype=float,
    )
    for joint_index in range(len(ARM_JOINTS)):
        row = coulomb_viscous_regressor_row(float(arm_velocity[joint_index]))
        offset = joint_index * NUM_FRICTION_PARAMS_PER_JOINT
        block[joint_index, offset : offset + NUM_FRICTION_PARAMS_PER_JOINT] = row
    return block


def default_friction_guess(friction: CoulombViscousFriction) -> np.ndarray:
    """Return (4, 2) initial [Fv, Fc] per joint."""
    guess = np.zeros((len(ARM_JOINTS), NUM_FRICTION_PARAMS_PER_JOINT), dtype=float)
    for joint_index in range(len(ARM_JOINTS)):
        guess[joint_index] = [
            max(float(friction.fv[joint_index]), 0.01),
            max(float(friction.fc[joint_index]), 0.02),
        ]
    return guess


def friction_bounds(initial: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    lower = np.zeros_like(initial)
    upper = np.zeros_like(initial)
    for joint_index in range(len(ARM_JOINTS)):
        lower[joint_index] = [0.0, 0.0]
        upper[joint_index] = [5.0, 5.0]
    return pack_friction_vector(lower), pack_friction_vector(upper)


def pack_friction_vector(params_per_joint: np.ndarray) -> np.ndarray:
    return params_per_joint.reshape(-1)


def unpack_friction_vector(vector: np.ndarray) -> np.ndarray:
    return np.asarray(vector, dtype=float).reshape(len(ARM_JOINTS), NUM_FRICTION_PARAMS_PER_JOINT)


def friction_params_to_dict(friction_vector: np.ndarray) -> dict[str, dict[str, float]]:
    params_per_joint = unpack_friction_vector(friction_vector)
    params: dict[str, dict[str, float]] = {}
    for joint_index, joint in enumerate(ARM_JOINTS):
        params[joint] = {
            name: float(params_per_joint[joint_index, param_index])
            for param_index, name in enumerate(FRICTION_PARAM_NAMES)
        }
    return params


def format_friction_equation(joint: str) -> str:
    return f'tau_f[{joint}] = Fv·q̇ + Fc·sign(q̇)'


def format_parameter_meanings() -> list[str]:
    return [
        'Fv: Viscous friction coefficient (N·m·s/rad)',
        'Fc: Coulomb friction torque magnitude (N·m)',
    ]


def reference_friction_vector(friction: CoulombViscousFriction) -> np.ndarray:
    """Nominal friction vector from config (URDF has no friction; defaults are 0)."""
    rows = [
        [float(friction.fv[joint_index]), float(friction.fc[joint_index])]
        for joint_index in range(len(ARM_JOINTS))
    ]
    return pack_friction_vector(np.asarray(rows, dtype=float))
