import pytest

from approachone.allocate import allocate
from approachone.sim import sim


C = 1
DT = 0.01


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
    assert motor_speeds == pytest.approx(((100 / 8) ** 0.5,) * 8)
    assert x_next == pytest.approx((0, 0, 10))

    u_next = [
        gain[0] * (cmd - actual)
        for actual, cmd, gain in zip(x_next, x_cmd, gains)
    ]

    assert u_next == pytest.approx([0, 0, 0])
