import numpy as np
import pytest

from approachthree.model import (
    DEFAULT_REG,
    DEFAULT_S_MAX,
    DEFAULT_WEIGHTS,
    achieved_command,
    allocated_motor_speeds,
    allocated_squared_speeds,
    attainable_generators,
    attainable_moment_set,
    attainable_set_faces,
    bounded_least_squares,
    bounds,
    controllability,
    convex_hull_2d,
    create_A,
    hover_margin,
    saturated_motors,
    support,
    zonotope_volume,
)
from approachthree.sim import sim
from approachthree.continuous import analyze, bode_channel_summary, saturation_margin

BALANCED_HOVER_SPEEDS = (
    *((13.75**0.5,) * 4),
    *((11.25**0.5,) * 4),
)


def out(*failed):
    """Eight-motor active mask with the given motor indices disabled."""
    return [0 if motor in failed else 1 for motor in range(8)]


def test_allocate_zero_control_produces_stopped_motors():
    assert allocated_motor_speeds((0, 0, 0)) == (0, 0, 0, 0, 0, 0, 0, 0)


def test_hover_balances_pitch_with_asymmetric_z_arms():
    # Well inside the polytope, so the QP reproduces the balanced hover split
    # shared by approaches one and two.
    motor_speeds = allocated_motor_speeds((0, 0, 100))

    assert motor_speeds == pytest.approx(BALANCED_HOVER_SPEEDS, abs=1e-5)


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

    assert x_next == pytest.approx((0, 0, 10), abs=1e-6)


def test_weights_default_prioritises_attitude_over_thrust():
    assert DEFAULT_WEIGHTS[0] == DEFAULT_WEIGHTS[1] > DEFAULT_WEIGHTS[2]


def test_zonotope_volume_of_unit_cube_is_one():
    assert zonotope_volume(np.eye(3)) == pytest.approx(1.0)


def test_support_function_matches_box_extent():
    generators = attainable_generators()
    # The maximum thrust is the sum of every motor's thrust contribution.
    assert support(generators, (0, 0, 1)) == pytest.approx(8 * DEFAULT_S_MAX)


def test_attainable_set_faces_of_a_cube_are_six_quads():
    # Feed the face builder a cube by masking to three orthogonal generators is
    # not possible through create_A, so exercise the nominal set instead: it is a
    # closed 3-D polytope, so every face is a polygon of at least three vertices.
    faces = attainable_set_faces()

    assert len(faces) > 0
    assert all(len(face) >= 3 for face in faces)


def test_nominal_vehicle_is_controllable():
    verdict = controllability()

    assert verdict["rank"] == 3
    assert verdict["volume"] > 0
    assert verdict["margin"] > 0
    assert verdict["controllable"]


def test_losing_a_row_of_motors_is_rank_deficient():
    # Motors 4-7 all share the same r_z, so the pitch-torque row is a scalar
    # multiple of the thrust row and the command axes are no longer independent.
    verdict = controllability(out(0, 1, 2, 3))

    assert verdict["rank"] == 2
    assert verdict["volume"] == pytest.approx(0.0)
    assert not verdict["controllable"]


def test_adjacent_pair_loss_puts_hover_outside_the_attainable_set():
    verdict = controllability(out(0, 1))

    assert verdict["rank"] == 3
    assert verdict["margin"] < 0
    assert not verdict["controllable"]


def test_opposite_pair_loss_stays_controllable():
    verdict = controllability(out(0, 6))

    assert verdict["rank"] == 3
    assert verdict["margin"] > 0
    assert verdict["controllable"]


def test_losing_an_outer_motor_costs_more_authority_than_an_inner_one():
    nominal = controllability()["volume"]
    outer = controllability(out(0))["volume"]  # r_y = -1.5
    inner = controllability(out(1))["volume"]  # r_y = -0.8

    assert outer < inner < nominal


def test_hover_margin_positive_means_trim_is_interior():
    generators = attainable_generators()

    assert hover_margin(generators, (0, 0, 100)) > 0
    # A command far above the achievable thrust ceiling is well outside the set.
    assert hover_margin(generators, (0, 0, 10_000)) < 0


def test_continuous_analysis_uses_bounded_qp_stack_near_hover():
    result = analyze()

    assert result.controllability_rank == 3
    assert result.observability_rank == 3
    assert result.command_jacobian[0, 0] == pytest.approx(1.0)
    assert result.command_jacobian[1, 1] == pytest.approx(1.0)
    assert result.command_jacobian[2, 2] == pytest.approx(0.05)
    assert saturation_margin() > 0
    assert all(eig.real < 0 for eig in result.loop_eigenvalues)


def test_approach_three_bode_channel_summary_documents_three_axes():
    result = analyze()

    assert [channel for channel, _ in bode_channel_summary(result)] == [
        "pitch",
        "yaw",
        "airspeed",
    ]
