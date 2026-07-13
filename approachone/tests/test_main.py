import pytest

from approachone.allocate import allocate
from approachone.sim import sim
from approachone.continuous import analyze, bode_channel_summary

C = 1
DT = 0.01
BALANCED_HOVER_SPEEDS = (
    *((13.75**0.5,) * 4),
    *((11.25**0.5,) * 4),
)


def run_control_allocate_sim(x, x_cmd, gains):
    """Run one proportional control command through allocation and simulation."""
    u = [gain[0] * (cmd - actual) for actual, cmd, gain in zip(x, x_cmd, gains)]
    motor_speeds = allocate(*u, C)
    x_next = sim(*motor_speeds, *x, C, DT)

    return u, motor_speeds, x_next


def test_allocate_zero_control_produces_stopped_motors():
    motor_speeds = allocate(0, 0, 0, C)

    assert motor_speeds == (0, 0, 0, 0, 0, 0, 0, 0)


def test_control_allocate_sim_round_trip_is_self_consistent():
    x = [0, 0, 0]
    x_cmd = [0, 0, 10]
    gains = [
        [0, 0, 0],
        [0, 0, 0],
        [10, 0, 0],
    ]

    u, motor_speeds, x_next = run_control_allocate_sim(x, x_cmd, gains)

    assert u == [0, 0, 100]
    assert motor_speeds == pytest.approx(BALANCED_HOVER_SPEEDS)
    assert x_next == pytest.approx((0, 0, 10))

    u_next = [
        gain[0] * (cmd - actual) for actual, cmd, gain in zip(x_next, x_cmd, gains)
    ]

    assert u_next == pytest.approx([0, 0, 0])


def test_continuous_analysis_matches_generated_stack_near_hover():
    result = analyze()

    assert result.controllability_rank == 3
    assert result.observability_rank == 3
    assert result.command_jacobian[0, 0] == pytest.approx(1.0)
    assert result.command_jacobian[2, 2] == pytest.approx(0.05)
    assert all(eig.real < 0 for eig in result.loop_eigenvalues)


def test_bode_channel_summary_documents_three_axes():
    result = analyze()

    assert [channel for channel, _ in bode_channel_summary(result)] == [
        "pitch",
        "yaw",
        "airspeed",
    ]
