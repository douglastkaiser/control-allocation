import sympy as sp
from common import utils
from common.geometry import MOTOR_R_Y, MOTOR_R_Z
from common.model import (
    angular_acceleration,
    default_inertia_substitutions,
    rigid_body_torque,
)


tau_vec, thrust = rigid_body_torque()
omega_dot = angular_acceleration(tau_vec)

w0, w1, w2, w3, w4, w5, w6, w7 = sp.symbols("w0 w1 w2 w3 w4 w5 w6 w7")
pitch_rate_in, yaw_rate_in, u_air_in = sp.symbols("pitch_rate_in yaw_rate_in u_air_in")
C = sp.Symbol("C", positive=True)
dt = sp.Symbol("dt", positive=True)

motor_speeds = (w0, w1, w2, w3, w4, w5, w6, w7)
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

sim_file = "# This file is auto-generated\n"
sim_file += "import math\n"
sim_file += utils.generate_python_function(
    "sim",
    (w0, w1, w2, w3, w4, w5, w6, w7, pitch_rate_in, yaw_rate_in, u_air_in, C, dt),
    [pitch_rate, yaw_rate, u_air],
)
with open("sim.py", "w") as f:
    f.write(sim_file)
