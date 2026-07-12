import sympy as sp
from common import utils
from common.geometry import (
    ALLOCATION_R_QUADRANT_Y,
    ALLOCATION_R_QUADRANT_Z,
    N_MOTORS,
)


def allocated_motor_speeds(tau_y, tau_z, thrust, C, r_quadrant_y, r_quadrant_z):
    """Build symbolic motor speeds from desired torque and thrust commands."""
    thrust_per_motor = thrust / N_MOTORS
    base_speed = sp.sqrt(sp.Max(thrust_per_motor, 0) / C)
    w = [base_speed for _ in range(N_MOTORS)]

    for tau, quadrant_r in ((tau_y, r_quadrant_y), (tau_z, r_quadrant_z)):
        tau_per_quadrant = tau / 4
        for quadrant, r in enumerate(quadrant_r):
            quadrant_thrust = tau_per_quadrant / r
            speed_delta = sp.sqrt(sp.Abs(quadrant_thrust) / 2 / C) * sp.sign(
                quadrant_thrust
            )
            motor_index = quadrant * 2
            w[motor_index] += speed_delta
            w[motor_index + 1] += speed_delta

    return [sp.Max(wx, 0) for wx in w]


tau_y, tau_z, thrust = sp.symbols("tau_y, tau_z, thrust")
C = sp.Symbol("C", positive=True)

r_quadrant_y = sp.symbols(
    "r_quadrant_0_y r_quadrant_1_y r_quadrant_2_y r_quadrant_3_y"
)
r_quadrant_z = sp.symbols(
    "r_quadrant_0_z r_quadrant_1_z r_quadrant_2_z r_quadrant_3_z"
)

w = allocated_motor_speeds(tau_y, tau_z, thrust, C, r_quadrant_y, r_quadrant_z)

allocation_geometry_subs = {}
for r_symbol, r_value in zip(r_quadrant_y, ALLOCATION_R_QUADRANT_Y):
    allocation_geometry_subs[r_symbol] = r_value
for r_symbol, r_value in zip(r_quadrant_z, ALLOCATION_R_QUADRANT_Z):
    allocation_geometry_subs[r_symbol] = r_value

w = [motor_speed.subs(allocation_geometry_subs) for motor_speed in w]

allocate_file = "# This file is auto-generated\n"
allocate_file += "import math\n"
allocate_file += utils.generate_python_function(
    "allocate",
    (tau_y, tau_z, thrust, C),
    w,
)
with open("allocate.py", "w") as f:
    f.write(allocate_file)
