"""Shared generator for the rigid-body ``sim.py`` used by every approach.

All three approaches simulate the same vehicle: squared motor speeds become
forces, forces become body torques through the shared ``common.model`` cross
product, and one Euler step advances pitch rate, yaw rate and airspeed. That
generation logic used to be copy-pasted into each approach's ``gen_sim.py``; it
now lives here so there is a single source of truth. Each approach keeps a tiny
``gen_sim.py`` that calls :func:`write_sim` from its own directory.
"""

import sympy as sp

from common.geometry import MOTOR_R_Y, MOTOR_R_Z, N_MOTORS
from common.model import (
    angular_acceleration,
    default_inertia_substitutions,
    rigid_body_torque,
)
from common.utils import generate_python_function


def sim_expressions():
    """Return ``(args, exprs)`` for the generated rigid-body step function."""
    tau_vec, thrust = rigid_body_torque()
    omega_dot = angular_acceleration(tau_vec)

    motor_speeds = sp.symbols(f"w0:{N_MOTORS}")
    pitch_rate_in, yaw_rate_in, u_air_in = sp.symbols(
        "pitch_rate_in yaw_rate_in u_air_in"
    )
    C = sp.Symbol("C", positive=True)
    dt = sp.Symbol("dt", positive=True)

    forces = [C * motor_speed**2 for motor_speed in motor_speeds]

    T = sum(forces)
    # \(T = D = \frac{1}{2} \rho V^2 S C_D\)
    u_air = sp.sqrt(T / C)

    pitch_accel = omega_dot[1]
    pitch_rate = pitch_rate_in + pitch_accel * dt

    yaw_accel = omega_dot[2]
    yaw_rate = yaw_rate_in + yaw_accel * dt

    force_subs = {}
    for motor_index, force in enumerate(forces):
        force_subs[f"f{motor_index}"] = force

    geometry_subs = {}
    for motor_index, r_y in enumerate(MOTOR_R_Y):
        geometry_subs[f"r_y{motor_index}"] = r_y
    for motor_index, r_z in enumerate(MOTOR_R_Z):
        geometry_subs[f"r_z{motor_index}"] = r_z

    inertia_subs = default_inertia_substitutions()

    pitch_rate = pitch_rate.subs(force_subs).subs(geometry_subs).subs(inertia_subs)
    yaw_rate = yaw_rate.subs(force_subs).subs(geometry_subs).subs(inertia_subs)

    args = (*motor_speeds, pitch_rate_in, yaw_rate_in, u_air_in, C, dt)
    exprs = [pitch_rate, yaw_rate, u_air]

    return args, exprs


def sim_source():
    """Return the full source text of the auto-generated ``sim.py``."""
    args, exprs = sim_expressions()

    source = "# This file is auto-generated\n"
    source += "import math\n"
    source += generate_python_function("sim", args, exprs)

    return source


def write_sim(path="sim.py"):
    """Write the generated ``sim.py`` to ``path`` (defaults to the caller's cwd)."""
    with open(path, "w") as f:
        f.write(sim_source())

    return path
