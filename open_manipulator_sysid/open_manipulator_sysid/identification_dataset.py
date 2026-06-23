"""Uniform (q, q̇, q̈, τ) dataset for offline friction regression."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import numpy as np

from open_manipulator_sysid.excitation_trajectory import ARM_JOINTS

MIN_POSITION_STD_RAD = 0.01
MIN_VELOCITY_STD_RAD_S = 0.01
MAX_REASONABLE_FRICTION = 100.0


@dataclass(frozen=True)
class IdentificationDataset:
    """Arm joint samples aligned on a uniform time grid."""

    times_s: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    torque: np.ndarray
    sample_dt_s: float

    @property
    def num_samples(self) -> int:
        return int(self.times_s.shape[0])

    def save_npz(self, path: Path | str) -> None:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            target,
            times_s=self.times_s,
            position=self.position,
            velocity=self.velocity,
            acceleration=self.acceleration,
            torque=self.torque,
            sample_dt_s=np.array([self.sample_dt_s]),
            joint_names=np.array(ARM_JOINTS),
        )

    @classmethod
    def load_npz(cls, path: Path | str) -> 'IdentificationDataset':
        with np.load(path, allow_pickle=False) as archive:
            return cls(
                times_s=np.asarray(archive['times_s'], dtype=float),
                position=np.asarray(archive['position'], dtype=float),
                velocity=np.asarray(archive['velocity'], dtype=float),
                acceleration=np.asarray(archive['acceleration'], dtype=float),
                torque=np.asarray(archive['torque'], dtype=float),
                sample_dt_s=float(np.asarray(archive['sample_dt_s']).reshape(-1)[0]),
            )


def validate_identification_dataset(dataset: IdentificationDataset) -> None:
    """Raise ValueError when the dataset cannot support friction regression."""
    if dataset.num_samples < 10:
        raise ValueError(
            f'Identification dataset has only {dataset.num_samples} samples; '
            'need sustained excitation data.'
        )

    low_motion_joints: List[str] = []
    for joint_index, joint_name in enumerate(ARM_JOINTS):
        position_std = float(np.std(dataset.position[:, joint_index]))
        velocity_std = float(np.std(dataset.velocity[:, joint_index]))
        if (
            position_std < MIN_POSITION_STD_RAD
            or velocity_std < MIN_VELOCITY_STD_RAD_S
        ):
            low_motion_joints.append(
                f'{joint_name} (pos_std={position_std:.4g}, vel_std={velocity_std:.4g})'
            )

    if len(low_motion_joints) == len(ARM_JOINTS):
        raise ValueError(
            'Identification dataset has no usable joint motion for friction regression. '
            f'All arm joints are static: {", ".join(low_motion_joints)}. '
            'Common causes: excitation goal was rejected (robot never moved), '
            'bag recorded only approach/hold/settle, or /joint_states lacks velocity. '
            'Re-run excitation launch and confirm '
            '"FollowJointTrajectory goal accepted" before regressing.'
        )


def _moving_average(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return values.copy()
    if window % 2 == 0:
        window += 1
    kernel = np.ones(window, dtype=float) / float(window)
    pad = window // 2
    padded = np.pad(values, ((pad, pad), (0, 0)), mode='edge')
    filtered = np.empty_like(values)
    for column in range(values.shape[1]):
        filtered[:, column] = np.convolve(padded[:, column], kernel, mode='valid')
    return filtered


def differentiate_central(
    values: np.ndarray,
    times_s: np.ndarray,
) -> np.ndarray:
    """Central-difference derivative with forward/backward ends."""
    num_samples, num_joints = values.shape
    derivative = np.zeros_like(values)
    if num_samples < 2:
        return derivative

    for joint in range(num_joints):
        derivative[0, joint] = (values[1, joint] - values[0, joint]) / (
            times_s[1] - times_s[0]
        )
        derivative[-1, joint] = (values[-1, joint] - values[-2, joint]) / (
            times_s[-1] - times_s[-2]
        )
        for index in range(1, num_samples - 1):
            dt = times_s[index + 1] - times_s[index - 1]
            derivative[index, joint] = (
                values[index + 1, joint] - values[index - 1, joint]
            ) / dt
    return derivative


def filtered_acceleration(
    velocity: np.ndarray,
    times_s: np.ndarray,
    *,
    smooth_window: int = 11,
) -> np.ndarray:
    """Differentiate velocity then lightly smooth acceleration."""
    raw = differentiate_central(velocity, times_s)
    return _moving_average(raw, smooth_window)


def resample_uniform(
    times_s: np.ndarray,
    values: np.ndarray,
    *,
    sample_dt_s: float,
) -> Tuple[np.ndarray, np.ndarray]:
    if sample_dt_s <= 0.0:
        raise ValueError('sample_dt_s must be positive')
    if times_s.size == 0:
        raise ValueError('times_s must not be empty')

    start = float(times_s[0])
    end = float(times_s[-1])
    if end <= start:
        raise ValueError('bag timestamps must span a positive interval')

    grid = np.arange(start, end + 0.5 * sample_dt_s, sample_dt_s, dtype=float)
    resampled = np.empty((grid.size, values.shape[1]), dtype=float)
    for joint in range(values.shape[1]):
        resampled[:, joint] = np.interp(grid, times_s, values[:, joint])
    return grid, resampled
