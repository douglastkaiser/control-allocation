# The goal here is to make approach two easy to poke at from the command line:
# run one controller step, allocate with the manual pseudoinverse, then simulate
# the resulting vehicle response with the generated rigid-body model.

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))

from common import utils
from approachtwo.model import allocated_motor_speeds
from approachtwo.sim import sim


dt = 0.01
C = 1
motors_active = [0, 1, 1, 1, 1, 1, 1, 1]

x_cmd = [0, 0, 10]
integrals = [0, 0, 0]
prev_deltas = [0, 0, 0]
gains = [[1, 0, 0], [0, 0, 0], [10, 0, 0]]

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

motor_speeds = allocated_motor_speeds(u, motors_active)
print(f" {motor_speeds=}")

pitch_rate, yaw_rate, u_air = sim(*motor_speeds, 0, 0, 0, C, dt)
print("t = 1")
print(f" {pitch_rate=}, {yaw_rate=}, {u_air=}")
