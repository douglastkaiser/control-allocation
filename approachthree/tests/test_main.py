import numpy as np
import pytest

from approachthree.model import (
    DEFAULT_REG,
    DEFAULT_S_MAX,
    DEFAULT_WEIGHTS,
    achieved_command,
    allocated_motor_speeds,
    allocated_squared_speeds,
    attainable_moment_set,
    bounded_least_squares,
    bounds,
    convex_hull_2d,
    create_A,
    saturated_motors,
)
from approachthree.sim import sim


def test_allocate_zero_control_produces_stopped_motors():
    assert allocated_motor_speeds((0, 0, 0)) == (0, 0, 0, 0, 0, 0, 0, 0)


def test_hover_matches_equal_thrust_solution():
    # Well inside the polytope, so the QP reproduces the min-effort hover split
    # shared by approaches one and two.
    motor_speeds = allocated_motor_speeds((0, 0, 100))

    assert motor_speeds == pytest.approx(((100 / 8) ** 0.5,) * 8)


def test_feasible_command_is_delivered_exactly():
    # Inside the polytope the QP tracks the command up to the tiny bias from the
    # effort regularisation (the low-priority thrust axis relaxes the most).
    command = (-40, 40, 100)

    assert achieved_command(command) == pytest.approx(command, abs=1e-3)


def test_allocation_never_exceeds_saturation_bounds():
    _, upper = bounds()
    s = allocated_squared_speeds((80, 100, 100))

    assert np.all(s >= -1e-9)
    assert np.all(s <= upper + 1e-9)


def test_saturating_command_is_projected_onto_the_polytope():
    command = (0, 150, 100)  # more yaw torque than the motors can produce
    s = allocated_squared_speeds(command)
    delivered = achieved_command(command)

    # The delivered command falls short of the request but stays feasible, and
    # motors are pinned to their limits rather than silently clamped.
    assert delivered[1] < command[1]
    assert saturated_motors(s).all()
    assert np.all(s <= DEFAULT_S_MAX + 1e-9)


def test_bounded_least_squares_satisfies_kkt_conditions():
    # For a strictly convex box-QP the KKT conditions are necessary and
    # sufficient, so verifying them proves global optimality.
    rng = np.random.default_rng(0)
    for _ in range(200):
        A = rng.normal(size=(3, 8))
        u = rng.normal(size=3) * 5
        lower = np.zeros(8)
        upper = rng.uniform(1, 5, size=8)
        weights = rng.uniform(0.5, 5, size=3)

        s = bounded_least_squares(A, u, lower, upper, weights, DEFAULT_REG)
        H = A.T @ np.diag(weights) @ A + DEFAULT_REG * np.eye(8)
        gradient = H @ s - A.T @ np.diag(weights) @ u

        for i in range(8):
            if s[i] <= lower[i] + 1e-7:
                assert gradient[i] >= -1e-6
            elif s[i] >= upper[i] - 1e-7:
                assert gradient[i] <= 1e-6
            else:
                assert abs(gradient[i]) <= 1e-6


def test_manual_convex_hull_matches_a_known_square():
    points = [(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5), (0.5, 0)]
    hull = convex_hull_2d(points)

    assert {tuple(vertex) for vertex in hull} == {(0, 0), (1, 0), (1, 1), (0, 1)}


def test_motor_loss_shrinks_the_attainable_set():
    nominal = attainable_moment_set()
    degraded = attainable_moment_set([0, 1, 1, 1, 1, 1, 1, 1])

    # Losing motor 0 removes reachable yaw authority on the positive side.
    assert degraded[:, 1].max() < nominal[:, 1].max()


def test_motor_out_column_is_zeroed_and_receives_no_speed():
    motors_active = [0, 1, 1, 1, 1, 1, 1, 1]

    assert np.all(create_A(motors_active)[:, 0] == 0)
    assert allocated_motor_speeds((0, 70, 100), motors_active)[0] == 0


def test_control_allocate_sim_round_trip_is_self_consistent():
    motor_speeds = allocated_motor_speeds((0, 0, 100))

    x_next = sim(*motor_speeds, 0, 0, 0, 1, 0.01)

    assert x_next == pytest.approx((0, 0, 10))


def test_weights_default_prioritises_attitude_over_thrust():
    assert DEFAULT_WEIGHTS[0] == DEFAULT_WEIGHTS[1] > DEFAULT_WEIGHTS[2]
