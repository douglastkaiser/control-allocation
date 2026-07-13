"""Shared motor geometry constants for generation and tests."""

N_MOTORS = 8

# Motor positions used by the rigid-body simulator model.
MOTOR_R_Y = (-1.5, -0.8, 1.5, 0.8, -1.5, -0.8, 1.5, 0.8)
MOTOR_R_Z = (-0.9, -0.9, -0.9, -0.9, 1.1, 1.1, 1.1, 1.1)


def _average_motor_pair(values, first_motor):
    """Return the average arm length for a two-motor quadrant."""
    return (values[first_motor] + values[first_motor + 1]) / 2


# Quadrant arm lengths are the average of the two motors in each quadrant.
ALLOCATION_R_QUADRANT_Y = tuple(
    _average_motor_pair(MOTOR_R_Y, first_motor) for first_motor in range(0, N_MOTORS, 2)
)
ALLOCATION_R_QUADRANT_Z = tuple(
    _average_motor_pair(MOTOR_R_Z, first_motor) for first_motor in range(0, N_MOTORS, 2)
)
