import sympy as sp

n_motors = 8


def rigid_body_motion():
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
