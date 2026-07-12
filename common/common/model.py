"""Shared symbolic rigid-body model for control allocation approaches."""

import sympy as sp

from common.geometry import N_MOTORS


n_motors = N_MOTORS


def rigid_body_motion():
    """Build symbolic thrust and torque equations for all motors."""
    # motor positions
    r_x = sp.Matrix(sp.symbols("r_x:%d" % n_motors))
    r_y = sp.Matrix(sp.symbols("r_y:%d" % n_motors))
    r_z = sp.Matrix(sp.symbols("r_z:%d" % n_motors))
    r_expr = [sp.Matrix([r_x[i], r_y[i], r_z[i]]) for i in range(n_motors)]
    # Force of each motor
    F = sp.Matrix(sp.symbols("f0:%d" % n_motors))
    # Force direction from each motor
    xhat = sp.Matrix([1, 0, 0])
    # tau = r x F
    tau_vec = sum(
        (r_expr[i].cross(F[i] * xhat) for i in range(n_motors)), sp.zeros(3, 1)
    )
    tau_y = tau_vec[1]
    tau_z = tau_vec[2]
    thrust = sum(F[i] for i in range(n_motors))

    return tau_y, tau_z, thrust


def single_motor_torque():
    """Build the symbolic torque contribution for one motor."""
    r_x_i, r_y_i, r_z_i, f_i = sp.symbols("r_{x_i} r_{y_i} r_{z_i} f_i")
    r_i = sp.Matrix([r_x_i, r_y_i, r_z_i])
    f_i_vec = sp.Matrix([f_i, 0, 0])

    return r_i, f_i_vec, r_i.cross(f_i_vec)
