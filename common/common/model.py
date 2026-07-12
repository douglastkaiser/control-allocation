"""Shared symbolic rigid-body model for control allocation approaches."""

import sympy as sp

from common.geometry import N_MOTORS


n_motors = N_MOTORS
INERTIA_SYMBOLS = sp.symbols("I_xx I_xy I_xz I_yx I_yy I_yz I_zx I_zy I_zz")


def inertia_matrix():
    """Return the symbolic body inertia matrix."""
    I_xx, I_xy, I_xz, I_yx, I_yy, I_yz, I_zx, I_zy, I_zz = INERTIA_SYMBOLS

    return sp.Matrix(
        [
            [I_xx, I_xy, I_xz],
            [I_yx, I_yy, I_yz],
            [I_zx, I_zy, I_zz],
        ]
    )


def default_inertia_substitutions():
    """Return simple inertia values used by generated examples today."""
    I_xx, I_xy, I_xz, I_yx, I_yy, I_yz, I_zx, I_zy, I_zz = INERTIA_SYMBOLS

    return {
        I_xx: 1,
        I_xy: 0,
        I_xz: 0,
        I_yx: 0,
        I_yy: 1,
        I_yz: 0,
        I_zx: 0,
        I_zy: 0,
        I_zz: 1,
    }


def rigid_body_torque():
    """Build the symbolic torque vector and thrust equation for all motors."""
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
    thrust = sum(F[i] for i in range(n_motors))

    return tau_vec, thrust


def rigid_body_motion():
    """Build symbolic thrust and torque equations for all motors."""
    tau_vec, thrust = rigid_body_torque()
    tau_y = tau_vec[1]
    tau_z = tau_vec[2]

    return tau_y, tau_z, thrust


def angular_acceleration(tau_vec):
    """Return angular acceleration from the full inertia matrix and torque."""
    return inertia_matrix().inv() * tau_vec


def single_motor_torque():
    """Build the symbolic torque contribution for one motor."""
    r_x_i, r_y_i, r_z_i, f_i = sp.symbols("r_{x_i} r_{y_i} r_{z_i} f_i")
    r_i = sp.Matrix([r_x_i, r_y_i, r_z_i])
    f_i_vec = sp.Matrix([f_i, 0, 0])

    return r_i, f_i_vec, r_i.cross(f_i_vec)
