import numpy as np
import pytest

from approachtwo.model import create_A, pseudoinverse
from approachtwo.main import allocate
from approachtwo.sim import sim


def test_allocate_zero_control_produces_stopped_motors():
    motor_speeds = allocate((0, 0, 0))

    assert motor_speeds == (0, 0, 0, 0, 0, 0, 0, 0)


def test_manual_pseudoinverse_matches_numpy_for_full_rank_geometry():
    A = create_A()

    assert pseudoinverse(A) == pytest.approx(np.linalg.pinv(A))


def test_allocate_equal_thrust_matches_approach_one_style_hover():
    motor_speeds = allocate((0, 0, 100))

    assert motor_speeds == pytest.approx(((100 / 8) ** 0.5,) * 8)


def test_manual_pseudoinverse_stays_finite_after_motor_loss():
    A = create_A([1, 0, 0, 0, 0, 0, 0, 0])
    A_plus = pseudoinverse(A)

    assert np.isfinite(A_plus).all()


def test_control_allocate_sim_round_trip_is_self_consistent():
    motor_speeds = allocate((0, 0, 100))

    x_next = sim(*motor_speeds, 0, 0, 0, 1, 0.01)

    assert x_next == pytest.approx((0, 0, 10))
