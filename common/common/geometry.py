"""Shared motor geometry constants for generation and tests."""

N_MOTORS = 8

# Motor positions used by the rigid-body simulator model.
MOTOR_R_Y = (-1.5, -0.8, 1.5, 0.8, -1.5, -0.8, 1.5, 0.8)
MOTOR_R_Z = (-1, -1, -1, -1, 1, 1, 1, 1)

# Quadrant arm lengths used by the approach-one allocator model.
ALLOCATION_R_QUADRANT_Y = (1, -1, 1, -1)
ALLOCATION_R_QUADRANT_Z = (-1, -1, 1, 1)
