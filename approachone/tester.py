# The goal here is to take the system being defined for approachone and, using sympy, generate cpp code
# then run and test it here in a python wrap.


from common import utils
from allocate import allocate
from sim import sim

dt = 0.01
C = 1

x_cmd = [0, 0, 10]
integrals = [0, 0, 0]
prev_deltas = [0, 0, 0]
gains = [[1, 0, 0], [0, 0, 0], [0, 0, 0]]

# t = 0
print("t = 0")
pitch_rate, yaw_rate, u_air = [1, 0, 0]
print(f" {pitch_rate=}, {yaw_rate=}, {u_air=}")
x = [pitch_rate, yaw_rate, u_air]
u, integrals, prev_deltas = utils.controllers(
    x, x_cmd, integrals, prev_deltas, gains, dt
)
tau_y, tau_z, thrust = u
print(f" {tau_y=}, {tau_z=}, {thrust=}")
w0, w1, w2, w3, w4, w5, w6, w7 = allocate(
    tau_y,
    tau_z,
    thrust,
    C,
)
print(f" {w0=}, {w1=}, {w2=}, {w3=}, {w4=}, {w5=}, {w6=}, {w7=}")
pitch_rate, yaw_rate, u_air = sim(w0, w1, w2, w3, w4, w5, w6, w7, 0, 0, 0, C, dt)
print("t = 1")
print(f" {pitch_rate=}, {yaw_rate=}, {u_air=}")
