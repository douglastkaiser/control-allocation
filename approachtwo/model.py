"""Approach two model API backed by the generated allocator."""

from approachtwo.allocate import (
    DEFAULT_DAMPING,
    RANK_TOLERANCE,
    allocated_motor_speeds,
    allocated_squared_speeds,
    allocate,
    create_A,
    pseudoinverse,
    pseudoinverse_components,
    pseudoinverse_equations,
)

__all__ = [
    "DEFAULT_DAMPING",
    "RANK_TOLERANCE",
    "allocated_motor_speeds",
    "allocated_squared_speeds",
    "allocate",
    "create_A",
    "pseudoinverse",
    "pseudoinverse_components",
    "pseudoinverse_equations",
]
