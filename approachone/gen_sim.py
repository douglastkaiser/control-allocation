import sympy as sp
from common import utils
from model import rigid_body_motion


tau_y, tau_z, thrust = rigid_body_motion()


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

I_y = 1
pitch_accel = tau_y / I_y
pitch_rate = pitch_rate_in + pitch_accel * dt

I_z = 1
yaw_accel = tau_z / I_z
yaw_rate = yaw_rate_in + yaw_accel * dt


pitch_rate = pitch_rate.subs("f0", F0)
pitch_rate = pitch_rate.subs("f1", F1)
pitch_rate = pitch_rate.subs("f2", F2)
pitch_rate = pitch_rate.subs("f3", F3)
pitch_rate = pitch_rate.subs("f4", F4)
pitch_rate = pitch_rate.subs("f5", F5)
pitch_rate = pitch_rate.subs("f6", F6)
pitch_rate = pitch_rate.subs("f7", F7)
yaw_rate = yaw_rate.subs("f0", F0)
yaw_rate = yaw_rate.subs("f1", F1)
yaw_rate = yaw_rate.subs("f2", F2)
yaw_rate = yaw_rate.subs("f3", F3)
yaw_rate = yaw_rate.subs("f4", F4)
yaw_rate = yaw_rate.subs("f5", F5)
yaw_rate = yaw_rate.subs("f6", F6)
yaw_rate = yaw_rate.subs("f7", F7)


pitch_rate = pitch_rate.subs("r_z0", -1)
pitch_rate = pitch_rate.subs("r_z1", -1)
pitch_rate = pitch_rate.subs("r_z2", -1)
pitch_rate = pitch_rate.subs("r_z3", -1)
pitch_rate = pitch_rate.subs("r_z4", 1)
pitch_rate = pitch_rate.subs("r_z5", 1)
pitch_rate = pitch_rate.subs("r_z6", 1)
pitch_rate = pitch_rate.subs("r_z7", 1)
yaw_rate = yaw_rate.subs("r_z0", -1)
yaw_rate = yaw_rate.subs("r_z1", -1)
yaw_rate = yaw_rate.subs("r_z2", -1)
yaw_rate = yaw_rate.subs("r_z3", -1)
yaw_rate = yaw_rate.subs("r_z4", 1)
yaw_rate = yaw_rate.subs("r_z5", 1)
yaw_rate = yaw_rate.subs("r_z6", 1)
yaw_rate = yaw_rate.subs("r_z7", 1)


pitch_rate = pitch_rate.subs("r_y0", -1.5)
pitch_rate = pitch_rate.subs("r_y1", -0.8)
pitch_rate = pitch_rate.subs("r_y2", 1.5)
pitch_rate = pitch_rate.subs("r_y3", 0.8)
pitch_rate = pitch_rate.subs("r_y4", -1.5)
pitch_rate = pitch_rate.subs("r_y5", -0.8)
pitch_rate = pitch_rate.subs("r_y6", 1.5)
pitch_rate = pitch_rate.subs("r_y7", 0.8)
yaw_rate = yaw_rate.subs("r_y0", -1.5)
yaw_rate = yaw_rate.subs("r_y1", -0.8)
yaw_rate = yaw_rate.subs("r_y2", 1.5)
yaw_rate = yaw_rate.subs("r_y3", 0.8)
yaw_rate = yaw_rate.subs("r_y4", -1.5)
yaw_rate = yaw_rate.subs("r_y5", -0.8)
yaw_rate = yaw_rate.subs("r_y6", 1.5)
yaw_rate = yaw_rate.subs("r_y7", 0.8)


sim_file = "# This file is auto-generated\n"
sim_file += "import math\n"
sim_file += utils.generate_python_function(
    "sim",
    (w0, w1, w2, w3, w4, w5, w6, w7, pitch_rate_in, yaw_rate_in, u_air_in, C, dt),
    [pitch_rate, yaw_rate, u_air],
)
with open("sim.py", "w") as f:
    f.write(sim_file)
