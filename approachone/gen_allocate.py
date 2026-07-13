import sympy as sp
from common import utils
from common.geometry import (
    ALLOCATION_R_QUADRANT_Y,
    ALLOCATION_R_QUADRANT_Z,
    N_MOTORS,
)


def allocated_motor_speeds(tau_y, tau_z, thrust, C, r_quadrant_y, r_quadrant_z):
    """Build symbolic motor speeds from desired torque and thrust commands."""
    negative_r_z = r_quadrant_z[0]
    positive_r_z = r_quadrant_z[2]
    motors_per_row = N_MOTORS / 2
    negative_row_thrust = (tau_y - positive_r_z * thrust) / (
        motors_per_row * (negative_r_z - positive_r_z)
    )
    positive_row_thrust = (tau_y - negative_r_z * thrust) / (
        motors_per_row * (positive_r_z - negative_r_z)
    )

    w = [
        sp.sqrt(sp.Max(negative_row_thrust, 0) / C)
        if motor_index < motors_per_row
        else sp.sqrt(sp.Max(positive_row_thrust, 0) / C)
        for motor_index in range(N_MOTORS)
    ]

    tau_per_quadrant = tau_z / 4
    for quadrant, r in enumerate(r_quadrant_y):
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
