"""Tests for open_manipulator_sysid excitation trajectory."""

from pathlib import Path

import pytest
import yaml

from open_manipulator_sysid.excitation_trajectory import (
    ARM_JOINTS,
    ExcitationTrajectory,
    SafetyViolation,
    default_config_path,
)


CONFIG_PATH = Path(__file__).resolve().parents[1] / 'config' / 'excitation.yaml'


def test_default_config_exists() -> None:
    assert CONFIG_PATH.is_file()
    assert default_config_path().is_file()


def test_config_matches_prd_defaults() -> None:
    with CONFIG_PATH.open('r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)

    assert config['arm_joints'] == list(ARM_JOINTS)
    assert config['q0'] == {
        'joint1': 0.0,
        'joint2': 0.0,
        'joint3': -0.05,
        'joint4': 0.135,
    }
    assert config['excitation']['num_harmonics'] == 5
    assert config['excitation']['period_s'] == 10.0
    assert config['excitation']['amplitude_fraction'] == 0.3
    assert config['safety']['soft_limit_inset_fraction'] == 0.1
    assert config['safety']['max_velocity_rad_s'] == 1.0
    assert config['safety']['max_acceleration_rad_s2'] == 2.0


def test_trajectory_starts_at_q0() -> None:
    trajectory = ExcitationTrajectory.from_yaml(CONFIG_PATH)
    sample = trajectory.sample(0.0)
    for joint in ARM_JOINTS:
        assert sample.position[joint] == pytest.approx(
            trajectory._limits[joint].q0,
            abs=1e-9,
        )


def test_soft_limits_over_one_period() -> None:
    trajectory = ExcitationTrajectory.from_yaml(CONFIG_PATH)
    trajectory.validate_period(num_samples=2000)


def test_velocity_cap_over_one_period() -> None:
    trajectory = ExcitationTrajectory.from_yaml(CONFIG_PATH)
    peaks = trajectory.max_abs_values_over_period()
    for joint in ARM_JOINTS:
        assert peaks[joint]['velocity'] <= trajectory._max_velocity + 1e-6


def test_only_arm_joints_present() -> None:
    trajectory = ExcitationTrajectory.from_yaml(CONFIG_PATH)
    assert trajectory.joints == ARM_JOINTS
    sample = trajectory.sample(1.0)
    assert set(sample.position.keys()) == set(ARM_JOINTS)


def test_validate_sample_detects_velocity_violation() -> None:
    trajectory = ExcitationTrajectory.from_yaml(CONFIG_PATH, amplitude_scale=10.0)
    with pytest.raises(SafetyViolation):
        trajectory.validate_period(num_samples=100)
