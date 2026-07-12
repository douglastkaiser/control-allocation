import sympy as sp

from common import utils
from common.geometry import MOTOR_R_Y, MOTOR_R_Z, N_MOTORS


motor_speeds = sp.symbols(f"w0:{N_MOTORS}")
pitch_rate_in, yaw_rate_in, u_air_in = sp.symbols("pitch_rate_in yaw_rate_in u_air_in")
C = sp.Symbol("C", positive=True)
dt = sp.Symbol("dt", positive=True)

forces = [C * motor_speed**2 for motor_speed in motor_speeds]

T = sum(forces)
# \(T = D = \frac{1}{2} \rho V^2 S C_D\)
u_air = sp.sqrt(T / C)

I_y = 1
pitch_accel = sum(r_z * force for r_z, force in zip(MOTOR_R_Z, forces)) / I_y
pitch_rate = pitch_rate_in + pitch_accel * dt

I_z = 1
yaw_accel = sum(-r_y * force for r_y, force in zip(MOTOR_R_Y, forces)) / I_z
yaw_rate = yaw_rate_in + yaw_accel * dt

sim_file = "# This file is auto-generated\n"
sim_file += "import math\n"
sim_file += utils.generate_python_function(
    "sim",
    (*motor_speeds, pitch_rate_in, yaw_rate_in, u_air_in, C, dt),
    [pitch_rate, yaw_rate, u_air],
)
with open("sim.py", "w") as f:
    f.write(sim_file)
