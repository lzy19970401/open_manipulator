"""Tests for offline synthetic regression (Issue 02)."""

from pathlib import Path

import numpy as np
import pytest

from open_manipulator_sysid.excitation_trajectory import ARM_JOINTS, default_config_path
from open_manipulator_sysid.regress import (
    NUM_FRICTION_PARAMS,
    build_friction_regressor_row,
    default_results_dir,
    default_urdf_path,
    default_synthetic_friction,
    effective_rank_and_condition,
    load_model,
    require_pinocchio,
    run_synthetic_regression,
    split_friction_vector,
)

pinocchio = pytest.importorskip('pinocchio')


def test_default_paths_exist() -> None:
    assert default_config_path().is_file()
    assert default_urdf_path().is_file()


def test_effective_condition_number_is_finite() -> None:
    matrix = np.diag(np.linspace(1.0, 0.1, 8))
    rank, condition = effective_rank_and_condition(matrix)
    assert rank == 8
    assert np.isfinite(condition)


def test_friction_regressor_shape() -> None:
    row = build_friction_regressor_row(np.array([0.1, -0.2, 0.3, -0.4]))
    assert row.shape == (len(ARM_JOINTS), NUM_FRICTION_PARAMS)


def test_synthetic_regression_writes_results(tmp_path: Path) -> None:
    require_pinocchio()
    result = run_synthetic_regression(
        output_dir=tmp_path,
        num_periods=2,
        sample_dt_s=0.02,
        noise_std=0.002,
        rng_seed=7,
    )

    assert (tmp_path / 'inertia_nominal.yaml').is_file()
    assert (tmp_path / 'friction_estimated.yaml').is_file()
    assert (tmp_path / 'report.html').is_file()
    assert not (tmp_path / 'bip_estimated.yaml').exists()

    assert result.num_samples > 0
    assert np.isfinite(result.residual_norm)
    assert np.isfinite(result.condition_number)
    assert result.condition_number > 0.0
    assert result.matrix_rank <= NUM_FRICTION_PARAMS
    assert len(result.viscous) == len(ARM_JOINTS)
    assert len(result.coulomb) == len(ARM_JOINTS)

    model, _ = load_model(default_urdf_path())
    truth = default_synthetic_friction(model)
    fv_true = np.array([truth.viscous[joint] for joint in ARM_JOINTS])
    fc_true = np.array([truth.coulomb[joint] for joint in ARM_JOINTS])
    assert np.allclose(result.viscous, fv_true, atol=0.01)
    assert np.allclose(result.coulomb, fc_true, atol=0.01)

    report = (tmp_path / 'report.html').read_text(encoding='utf-8')
    assert 'Nominal vs estimated friction' in report
    assert 'fixed from URDF' in report
    assert 'Residual norm' in report
    assert 'condition number' in report.lower()

    inertia_yaml = (tmp_path / 'inertia_nominal.yaml').read_text(encoding='utf-8')
    assert 'use_nominal_urdf: true' in inertia_yaml


def test_split_friction_vector() -> None:
    vector = np.arange(NUM_FRICTION_PARAMS, dtype=float)
    viscous, coulomb = split_friction_vector(vector)
    assert len(viscous) == 4
    assert len(coulomb) == 4
    assert np.allclose(viscous, vector[:4])
    assert np.allclose(coulomb, vector[4:])


def test_results_dir_is_gitignored_package_path() -> None:
    assert default_results_dir().name == 'results'
