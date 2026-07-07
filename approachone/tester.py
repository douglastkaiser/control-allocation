# The goal here is to take the system being defined for approachone and, using sympy, generate cpp code
# then run and test it here in a python wrap.


from common import sim, utils
from allocate import allocate

dt = 0.01
C = 1

r_quadrant_0_y = 1
r_quadrant_1_y = -1
r_quadrant_2_y = 1
r_quadrant_3_y = -1
r_quadrant_0_z = -1
r_quadrant_1_z = -1
r_quadrant_2_z = 1
r_quadrant_3_z = 1

# t = 0
x = [1, 0, 0]
x_cmd = [0, 0, 0]
integrals = [0, 0, 0]
prev_deltas = [0, 0, 0]
gains = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
u, integrals, prev_deltas = utils.controllers(
    x, x_cmd, integrals, prev_deltas, gains, dt
)
tau_y, tau_z, thrust = u
w0, w1, w2, w3, w4, w5, w6, w7 = allocate(
    tau_y,
    tau_z,
    thrust,
    C,
    r_quadrant_0_y,
    r_quadrant_1_y,
    r_quadrant_2_y,
    r_quadrant_3_y,
    r_quadrant_0_z,
    r_quadrant_1_z,
    r_quadrant_2_z,
    r_quadrant_3_z,
)
pitch_rate, yaw_rate, u_air = sim.sim(w0, w1, w2, w3, w4, w5, w6, w7, 0, 0, 0, C, dt)
