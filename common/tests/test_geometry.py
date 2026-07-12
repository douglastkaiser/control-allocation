from common.geometry import (
    ALLOCATION_R_QUADRANT_Y,
    ALLOCATION_R_QUADRANT_Z,
    MOTOR_R_Y,
    MOTOR_R_Z,
)


def test_allocation_quadrant_arms_are_motor_pair_averages():
    expected_y = tuple(
        (MOTOR_R_Y[first_motor] + MOTOR_R_Y[first_motor + 1]) / 2
        for first_motor in range(0, len(MOTOR_R_Y), 2)
    )
    expected_z = tuple(
        (MOTOR_R_Z[first_motor] + MOTOR_R_Z[first_motor + 1]) / 2
        for first_motor in range(0, len(MOTOR_R_Z), 2)
    )

    assert ALLOCATION_R_QUADRANT_Y == expected_y
    assert ALLOCATION_R_QUADRANT_Z == expected_z
