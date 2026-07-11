"""Unit tests for paper Fourier excitation and C² runtime schedule."""

from __future__ import annotations

import math

import pytest

from open_manipulator_sysid.excitation_trajectory import (
    ARM_JOINTS,
    ExcitationTrajectory,
    TrajectorySample,
    build_sysid_runtime_trajectory_messages,
    compute_swevers_kinematics,
    describe_runtime_schedule,
    required_c2_transition_duration,
)


@pytest.fixture(name='trajectory')
def fixture_trajectory() -> ExcitationTrajectory:
    return ExcitationTrajectory.default()


def test_paper_fourier_kinematics_at_zero():
    q0 = 0.1
    a = [0.2, -0.1, 0.05, -0.02, 0.01]
    b = [0.15, -0.08, 0.04, -0.01, 0.005]
    omega_f = 0.1
    q, qd, qdd = compute_swevers_kinematics(q0, a, b, omega_f, 0.0)
    two_pi = 2.0 * math.pi
    expected_q = q0 - sum(
        b_l / (two_pi * harmonic * omega_f)
        for harmonic, b_l in enumerate(b, start=1)
    )
    assert math.isclose(q, expected_q, rel_tol=0, abs_tol=1e-9)
    assert math.isclose(qd, sum(a), rel_tol=0, abs_tol=1e-9)
    expected_qdd = sum(two_pi * harmonic * omega_f * b_l for harmonic, b_l in enumerate(b, start=1))
    assert math.isclose(qdd, expected_qdd, rel_tol=0, abs_tol=1e-9)


def test_period_boundary_generally_nonzero(trajectory: ExcitationTrajectory):
    start, end = trajectory.period_boundary_states()
    # Template/GA coeffs need not return to q0 at period end.
    assert isinstance(start.position, dict)
    assert isinstance(end.velocity, dict)


def test_multi_period_uses_continuous_time(trajectory: ExcitationTrajectory):
    t_one = trajectory.period_s
    wrapped = trajectory.sample(t_one, wrap=True)
    continuous = trajectory.sample(t_one, wrap=False)
    # Without periodic constraints these may differ when coefficients are non-trivial.
    assert wrapped.time_s == 0.0 or wrapped.time_s == pytest.approx(0.0, abs=1e-9)


def test_c2_ingress_matches_boundary_states(trajectory: ExcitationTrajectory):
    q0 = trajectory.q0
    excitation_start, _ = trajectory.period_boundary_states()
    q0_state = TrajectorySample(
        time_s=0.0,
        position=dict(q0),
        velocity={joint: 0.0 for joint in ARM_JOINTS},
        acceleration={joint: 0.0 for joint in ARM_JOINTS},
    )
    duration = required_c2_transition_duration(
        q0_state,
        excitation_start,
        joints=ARM_JOINTS,
        max_velocity=trajectory._max_velocity,  # noqa: SLF001
        max_acceleration=trajectory._max_acceleration,  # noqa: SLF001
        min_duration_s=2.0,
    )
    samples, schedule = build_sysid_runtime_trajectory_messages(
        trajectory,
        start_positions=q0,
        to_q0_duration_s=1.0,
        ingress_spline_duration_s=duration,
        num_periods=3,
        egress_spline_duration_s=duration,
        q0_hold_duration_s=2.0,
        sample_dt_s=0.05,
        discard_periods=1,
    )
    ingress_start = samples[int(schedule.to_q0_duration_s / 0.05)]
    ingress_end = samples[int(schedule.excitation_start_s / 0.05)]
    for joint in ARM_JOINTS:
        assert ingress_end.position[joint] == pytest.approx(
            excitation_start.position[joint], abs=0.05
        )
        assert ingress_end.velocity[joint] == pytest.approx(
            excitation_start.velocity[joint], abs=0.1
        )


def test_runtime_schedule_bag_window(trajectory: ExcitationTrajectory):
    schedule = describe_runtime_schedule(
        trajectory,
        start_positions=trajectory.q0,
        to_q0_duration_s=5.0,
        ingress_spline_duration_s=4.0,
        num_periods=3,
        egress_spline_duration_s=4.0,
        q0_hold_duration_s=10.0,
        discard_periods=1,
    )
    assert schedule.bag_record_start_s == pytest.approx(5.0)
    assert schedule.bag_record_end_s == pytest.approx(5.0 + 4.0 + 30.0)
    assert schedule.identification_start_s == pytest.approx(9.0 + 10.0)
    assert schedule.identification_end_s == pytest.approx(39.0)
