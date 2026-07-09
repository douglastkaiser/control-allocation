import sympy as sp
from common import utils, model


tau_vec = model.rigid_body_motion()
print(tau_vec)


w0, w1, w2, w3, w4, w5, w6, w7 = sp.symbols("w0 w1 w2 w3 w4 w5 w6 w7")
pitch_rate_in, yaw_rate_in, u_air_in = sp.symbols("pitch_rate_in yaw_rate_in u_air_in")
C = sp.Symbol("C", positive=True)
dt = sp.Symbol("dt", positive=True)

F0 = C * w0**2
F1 = C * w1**2
F2 = C * w2**2
F3 = C * w3**2
F4 = C * w4**2
F5 = C * w5**2
F6 = C * w6**2
F7 = C * w7**2

T = F0 + F1 + F2 + F3 + F4 + F5 + F6 + F7
# \(T = D = \frac{1}{2} \rho V^2 S C_D\)
# C = 1
# dt = 0.01
u_air = sp.sqrt(T / C)

r0_z = r1_z = r2_z = r3_z = -1
r4_z = r5_z = r6_z = r7_z = 1
torque_y = (
    F0 * r0_z
    + F1 * r1_z
    + F2 * r2_z
    + F3 * r3_z
    + F4 * r4_z
    + F5 * r5_z
    + F6 * r6_z
    + F7 * r7_z
)
I_y = 1
pitch_accel = torque_y / I_y
pitch_rate = pitch_rate_in + pitch_accel * dt

r0_y = r4_y = 1.5
r1_y = r5_y = 0.5
r2_y = r6_y = -r1_y
r3_y = r7_y = -r2_y
torque_z = (
    F0 * r0_y
    + F1 * r1_y
    + F2 * r2_y
    + F3 * r3_y
    + F4 * r4_y
    + F5 * r5_y
    + F6 * r6_y
    + F7 * r7_y
)
I_z = 1
yaw_accel = torque_z / I_z
yaw_rate = yaw_rate_in + yaw_accel * dt


sim_file = "# This file is auto-generated\n"
sim_file += "import math\n"
sim_file += utils.generate_python_function(
    "sim",
    (w0, w1, w2, w3, w4, w5, w6, w7, pitch_rate_in, yaw_rate_in, u_air_in, C, dt),
    [pitch_rate, yaw_rate, u_air],
)
with open("sim.py", "w") as f:
    f.write(sim_file)
