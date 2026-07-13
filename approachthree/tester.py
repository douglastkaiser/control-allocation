# Poke approach three from the command line: send a command through the
# saturation-aware QP allocator, then simulate the vehicle response with the
# generated rigid-body model. Unlike approaches one and two, an over-range
# command is met by the closest achievable command instead of being clamped
# after the fact.

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))

import numpy as np

from approachthree.model import (
    DEFAULT_S_MAX,
    achieved_command,
    allocated_motor_speeds,
    allocated_squared_speeds,
    saturated_motors,
)
from approachthree.sim import sim

dt = 0.01
C = 1


def run(label, command, motors_active=None):
    print(label)
    tau_y, tau_z, thrust = command
    print(f"  requested: {tau_y=}, {tau_z=}, {thrust=}")

    s = allocated_squared_speeds(command, motors_active)
    w = allocated_motor_speeds(command, motors_active)
    delivered = achieved_command(command, motors_active)
    on_limit = int(saturated_motors(s).sum())

    print(f"  squared speeds: {np.round(s, 2)}")
    print(f"  motor speeds:   {tuple(round(value, 3) for value in w)}")
    print(
        f"  delivered command: tau_y={delivered[0]:.2f}, "
        f"tau_z={delivered[1]:.2f}, thrust={delivered[2]:.2f}"
    )
    print(f"  motors on a limit: {on_limit} / {len(w)}  (s_max={DEFAULT_S_MAX:g})")

    pitch_rate, yaw_rate, u_air = sim(*w, 0, 0, 0, C, dt)
    print(f"  one sim step:   {pitch_rate=:.4f}, {yaw_rate=:.4f}, {u_air=:.4f}\n")


run("hover (feasible)", (0, 0, 100))
run("feasible maneuver", (-40, 40, 100))
run("aggressive yaw (saturates)", (0, 150, 100))
run("combined, over-range (saturates)", (80, 100, 100))
run("motor 0 out", (0, 70, 100), [0, 1, 1, 1, 1, 1, 1, 1])
