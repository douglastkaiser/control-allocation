# This file is auto-generated
"""Pseudoinverse allocation math for approach two."""

import math

import numpy as np
import sympy as sp

from common.geometry import MOTOR_R_Y, MOTOR_R_Z, N_MOTORS


DEFAULT_DAMPING = 1e-9
RANK_TOLERANCE = 1e-12


def create_A(motors_active=None, C=1):
    """Return the command-to-squared-speed allocation matrix.

    Rows are pitch torque, yaw torque, and total thrust. Columns are motors.
    In this coordinate setup, thrust is along body x, so ``r x F`` gives
    ``tau_y = r_z * F`` and ``tau_z = -r_y * F``.
    """
    if motors_active is None:
        motors_active = (1,) * N_MOTORS

    active = np.asarray(motors_active, dtype=float)
    if active.shape != (N_MOTORS,):
        raise ValueError(f"motors_active must contain {N_MOTORS} entries")

    A = np.array(
        [
            MOTOR_R_Z,
            [-r_y for r_y in MOTOR_R_Y],
            [1] * N_MOTORS,
        ],
        dtype=float,
    )

    return C * A * active


def _singularity_robust_reciprocal(sigma, damping):
    """Return a damped reciprocal for one singular value.

    The textbook Moore-Penrose term is ``1 / sigma``. We use
    ``sigma / (sigma**2 + damping**2)`` instead, which converges to the
    textbook value away from singularities but remains finite near lost motor
    authority or rank-deficient geometry.
    """
    if sigma <= RANK_TOLERANCE:
        return 0
    return sigma / (sigma**2 + damping**2)


def pseudoinverse_components(A, damping=DEFAULT_DAMPING):
    """Return the SVD pieces used to assemble the manual pseudoinverse."""
    U, singular_values, Vh = np.linalg.svd(A, full_matrices=False)
    sigma_plus = np.diag(
        [_singularity_robust_reciprocal(sigma, damping) for sigma in singular_values]
    )

    return U, sigma_plus, Vh


def pseudoinverse(A, damping=DEFAULT_DAMPING):
    """Compute a singularity-robust pseudoinverse without ``np.linalg.pinv``."""
    U, sigma_plus, Vh = pseudoinverse_components(A, damping)

    return Vh.transpose() @ sigma_plus @ U.transpose()


def pseudoinverse_equations(A):
    """Return symbolic SVD/pseudoinverse equations matching ``pseudoinverse``."""
    rows, columns = A.shape
    rank = min(rows, columns)

    A_sym = sp.MatrixSymbol("A", rows, columns)
    U = sp.MatrixSymbol("U", rows, rank)
    sigma = sp.MatrixSymbol(r"\Sigma", rank, rank)
    V_t = sp.MatrixSymbol("V^T", rank, columns)
    A_plus = sp.MatrixSymbol("A^+", columns, rows)
    V = sp.MatrixSymbol("V", columns, rank)
    sigma_plus = sp.MatrixSymbol(r"\Sigma_\lambda^+", rank, rank)
    U_t = sp.MatrixSymbol("U^T", rank, rows)

    return (
        sp.Eq(A_sym, sp.MatMul(U, sigma, V_t, evaluate=False)),
        sp.Eq(A_plus, sp.MatMul(V, sigma_plus, U_t, evaluate=False)),
    )


def allocated_squared_speeds(u, motors_active=None, C=1, damping=DEFAULT_DAMPING):
    """Allocate commands to squared motor speeds with the manual pseudoinverse."""
    tau_y, tau_z, thrust = u
    command = np.array([[tau_y], [tau_z], [thrust]], dtype=float)
    A = create_A(motors_active, C)

    return pseudoinverse(A, damping) @ command


def allocated_motor_speeds(u, motors_active=None, C=1, damping=DEFAULT_DAMPING):
    """Allocate commands to non-negative motor speeds."""
    w_sq = allocated_squared_speeds(u, motors_active, C, damping)

    return tuple(math.sqrt(max(float(speed_sq), 0)) for speed_sq in w_sq.flatten())


allocate = allocated_motor_speeds
