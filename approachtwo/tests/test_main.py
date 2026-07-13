import numpy as np
import pytest

from approachtwo.allocate import (
    allocate,
    allocated_motor_speeds,
    create_A,
    pseudoinverse,
)
from approachtwo.sim import sim
from approachtwo.continuous import analyze, bode_channel_summary

BALANCED_HOVER_SPEEDS = (
    *((13.75**0.5,) * 4),
    *((11.25**0.5,) * 4),
)


def test_allocate_zero_control_produces_stopped_motors():
    motor_speeds = allocated_motor_speeds((0, 0, 0))

    assert motor_speeds == (0, 0, 0, 0, 0, 0, 0, 0)


def test_generated_allocate_alias_uses_generated_allocator():
    assert allocate((0, 0, 100)) == pytest.approx(allocated_motor_speeds((0, 0, 100)))


def test_manual_pseudoinverse_matches_numpy_for_full_rank_geometry():
    A = create_A()

    assert pseudoinverse(A) == pytest.approx(np.linalg.pinv(A))


def test_allocate_hover_balances_pitch_with_asymmetric_z_arms():
    motor_speeds = allocated_motor_speeds((0, 0, 100))

    assert motor_speeds == pytest.approx(BALANCED_HOVER_SPEEDS)


def test_manual_pseudoinverse_stays_finite_after_motor_loss():
    A = create_A([1, 0, 0, 0, 0, 0, 0, 0])
    A_plus = pseudoinverse(A)

    assert np.isfinite(A_plus).all()


def test_control_allocate_sim_round_trip_is_self_consistent():
    motor_speeds = allocated_motor_speeds((0, 0, 100))

    x_next = sim(*motor_speeds, 0, 0, 0, 1, 0.01)

    assert x_next == pytest.approx((0, 0, 10))


def test_continuous_analysis_uses_per_motor_stack_near_hover():
    result = analyze()

    assert result.controllability_rank == 3
    assert result.observability_rank == 3
    assert result.command_jacobian[0, 0] == pytest.approx(1.0)
    assert result.command_jacobian[1, 1] == pytest.approx(1.0)
    assert result.command_jacobian[2, 2] == pytest.approx(0.05)
    assert all(eig.real < 0 for eig in result.loop_eigenvalues)


def test_approach_two_bode_channel_summary_documents_three_axes():
    result = analyze()

    assert [channel for channel, _ in bode_channel_summary(result)] == [
        "pitch",
        "yaw",
        "airspeed",
    ]
