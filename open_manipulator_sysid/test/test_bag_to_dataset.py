"""Tests for bag_to_dataset helpers (Issue 03)."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from open_manipulator_sysid.excitation_recording.bag_reader import (
    XM430_PRESENT_CURRENT_NM_MULTIPLIER,
    arm_effort_array_to_nm,
    bag_to_dataset,
    detect_bag_storage_id,
)
from open_manipulator_sysid.excitation_recording.dataset import (
    IdentificationDataset,
    filtered_acceleration,
    resample_uniform,
    validate_identification_dataset,
)
from open_manipulator_sysid.excitation_trajectory import (
    ARM_JOINTS,
    ExcitationTrajectory,
    build_sysid_runtime_trajectory_messages,
    default_config_path,
    runtime_config_from_yaml,
)


def test_arm_effort_array_to_nm_converts_hardware_lsb() -> None:
    raw = np.array([[-2.0, 15.0, -102.0, -5.0]])
    converted = arm_effort_array_to_nm(raw)
    assert converted[0, 2] == -102.0 * XM430_PRESENT_CURRENT_NM_MULTIPLIER


def test_arm_effort_array_to_nm_passes_through_gazebo_values() -> None:
    sim = np.array([[0.1, -0.2, 0.3, 0.0]])
    assert np.allclose(arm_effort_array_to_nm(sim), sim)


def test_detect_bag_storage_id_reads_mcap_metadata() -> None:
    bag_dir = (
        Path(__file__).resolve().parents[1]
        / 'results'
        / 'bags'
        / 'gazebo_20260622_122703'
    )
    if not (bag_dir / 'metadata.yaml').is_file():
        pytest.skip('Gazebo bag fixture not present')
    assert detect_bag_storage_id(bag_dir) == 'mcap'


def test_detect_bag_storage_id_from_mcap_glob(tmp_path: Path) -> None:
    (tmp_path / 'sample_0.mcap').write_bytes(b'\x00')
    assert detect_bag_storage_id(tmp_path) == 'mcap'


def test_validate_rejects_static_dataset() -> None:
    dataset = IdentificationDataset(
        times_s=np.linspace(0.0, 1.0, 101),
        position=np.zeros((101, 4)),
        velocity=np.zeros((101, 4)),
        acceleration=np.zeros((101, 4)),
        torque=np.array([[0.0, -0.44, -0.36, -0.06]] * 101),
        sample_dt_s=0.01,
    )
    with pytest.raises(ValueError, match='no usable joint motion'):
        validate_identification_dataset(dataset)


def test_failed_gazebo_bag_is_rejected() -> None:
    bag_dir = (
        Path(__file__).resolve().parents[1]
        / 'results'
        / 'bags'
        / 'gazebo_20260622_122703'
    )
    if not (bag_dir / 'metadata.yaml').is_file():
        pytest.skip('Failed excitation bag fixture not present')
    with pytest.raises(ValueError, match='no usable joint motion'):
        bag_to_dataset(bag_dir, trim_start_s=7.0, trim_end_s=2.5)


def test_successful_gazebo_bag_passes_validation() -> None:
    bag_dir = (
        Path(__file__).resolve().parents[1]
        / 'results'
        / 'bags'
        / 'gazebo_20260622_123834'
    )
    if not (bag_dir / 'metadata.yaml').is_file():
        pytest.skip('Successful excitation bag fixture not present')
    dataset = bag_to_dataset(bag_dir, trim_start_s=7.0, trim_end_s=2.5)
    validate_identification_dataset(dataset)
    assert dataset.velocity.std(axis=0).min() > 0.01


def test_resample_uniform_interpolates_endpoints() -> None:
    times = np.array([0.0, 0.5, 1.0])
    values = np.array([[0.0], [0.5], [1.0]])
    grid, resampled = resample_uniform(times, values, sample_dt_s=0.25)
    assert grid[0] == pytest.approx(0.0)
    assert grid[-1] == pytest.approx(1.0)
    assert resampled[0, 0] == pytest.approx(0.0)
    assert resampled[-1, 0] == pytest.approx(1.0)


def test_filtered_acceleration_is_finite() -> None:
    times = np.linspace(0.0, 1.0, 101)
    velocity = np.column_stack([np.sin(times), np.cos(times)])
    acceleration = filtered_acceleration(velocity, times, smooth_window=5)
    assert acceleration.shape == velocity.shape
    assert np.all(np.isfinite(acceleration))


def test_build_sysid_runtime_trajectory_includes_phases() -> None:
    config_path = default_config_path()
    with config_path.open('r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)
    runtime = runtime_config_from_yaml(config)
    trajectory = ExcitationTrajectory.from_yaml(config_path)
    samples, schedule = build_sysid_runtime_trajectory_messages(
        trajectory,
        start_positions={joint: 0.0 for joint in trajectory.joints},
        to_q0_duration_s=runtime['to_q0_duration_s'],
        ingress_spline_duration_s=runtime['ingress_spline_duration_s'],
        num_periods=2,
        egress_spline_duration_s=runtime['egress_spline_duration_s'],
        q0_hold_duration_s=runtime['q0_hold_duration_s'],
        sample_dt_s=0.1,
        discard_periods=int(runtime['discard_periods']),
    )
    assert len(samples) > 0
    assert samples[0].time_s == pytest.approx(0.0)
    assert samples[-1].time_s == pytest.approx(schedule.total_duration_s, abs=0.15)


def test_last_trajectory_point_has_zero_end_velocity() -> None:
    """JointTrajectoryController rejects goals whose last point has non-zero velocity."""
    config_path = default_config_path()
    with config_path.open('r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)
    runtime = runtime_config_from_yaml(config)
    trajectory = ExcitationTrajectory.from_yaml(config_path)
    samples, _schedule = build_sysid_runtime_trajectory_messages(
        trajectory,
        start_positions={joint: 0.0 for joint in trajectory.joints},
        to_q0_duration_s=runtime['to_q0_duration_s'],
        ingress_spline_duration_s=runtime['ingress_spline_duration_s'],
        num_periods=2,
        egress_spline_duration_s=runtime['egress_spline_duration_s'],
        q0_hold_duration_s=runtime['q0_hold_duration_s'],
        sample_dt_s=0.01,
        discard_periods=int(runtime['discard_periods']),
    )
    last = samples[-1]
    for joint in ARM_JOINTS:
        assert last.velocity[joint] == pytest.approx(0.0, abs=1e-9)
        assert last.acceleration[joint] == pytest.approx(0.0, abs=1e-9)


def test_dataset_npz_roundtrip(tmp_path: Path) -> None:
    dataset = IdentificationDataset(
        times_s=np.linspace(0.0, 0.1, 11),
        position=np.zeros((11, 4)),
        velocity=np.zeros((11, 4)),
        acceleration=np.zeros((11, 4)),
        torque=np.zeros((11, 4)),
        sample_dt_s=0.01,
    )
    path = tmp_path / 'sample.npz'
    dataset.save_npz(path)
    loaded = IdentificationDataset.load_npz(path)
    assert loaded.num_samples == dataset.num_samples
    assert loaded.sample_dt_s == pytest.approx(dataset.sample_dt_s)


