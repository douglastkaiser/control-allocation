# This file is auto-generated
import math


def allocate(tau_y, tau_z, thrust):
    r_quadrant_0_y = r_quadrant_1_y = r_quadrant_2_y = r_quadrant_3_y = 1
    r_quadrant_0_z = r_quadrant_1_z = r_quadrant_2_z = r_quadrant_3_z = 1
    C = 1
    return (
        max(
            0,
            (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_y / r_quadrant_0_y))
            * (
                0.0
                if tau_y / r_quadrant_0_y == 0
                else math.copysign(1, tau_y / r_quadrant_0_y)
            )
            / math.sqrt(C)
            + (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_z / r_quadrant_0_z))
            * (
                0.0
                if tau_z / r_quadrant_0_z == 0
                else math.copysign(1, tau_z / r_quadrant_0_z)
            )
            / math.sqrt(C)
            + math.sqrt(max(0, (1 / 8) * thrust)) / math.sqrt(C),
        ),
        max(
            0,
            (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_y / r_quadrant_0_y))
            * (
                0.0
                if tau_y / r_quadrant_0_y == 0
                else math.copysign(1, tau_y / r_quadrant_0_y)
            )
            / math.sqrt(C)
            + (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_z / r_quadrant_0_z))
            * (
                0.0
                if tau_z / r_quadrant_0_z == 0
                else math.copysign(1, tau_z / r_quadrant_0_z)
            )
            / math.sqrt(C)
            + math.sqrt(max(0, (1 / 8) * thrust)) / math.sqrt(C),
        ),
        max(
            0,
            (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_y / r_quadrant_1_y))
            * (
                0.0
                if tau_y / r_quadrant_1_y == 0
                else math.copysign(1, tau_y / r_quadrant_1_y)
            )
            / math.sqrt(C)
            + (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_z / r_quadrant_1_z))
            * (
                0.0
                if tau_z / r_quadrant_1_z == 0
                else math.copysign(1, tau_z / r_quadrant_1_z)
            )
            / math.sqrt(C)
            + math.sqrt(max(0, (1 / 8) * thrust)) / math.sqrt(C),
        ),
        max(
            0,
            (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_y / r_quadrant_1_y))
            * (
                0.0
                if tau_y / r_quadrant_1_y == 0
                else math.copysign(1, tau_y / r_quadrant_1_y)
            )
            / math.sqrt(C)
            + (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_z / r_quadrant_1_z))
            * (
                0.0
                if tau_z / r_quadrant_1_z == 0
                else math.copysign(1, tau_z / r_quadrant_1_z)
            )
            / math.sqrt(C)
            + math.sqrt(max(0, (1 / 8) * thrust)) / math.sqrt(C),
        ),
        max(
            0,
            (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_y / r_quadrant_2_y))
            * (
                0.0
                if tau_y / r_quadrant_2_y == 0
                else math.copysign(1, tau_y / r_quadrant_2_y)
            )
            / math.sqrt(C)
            + (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_z / r_quadrant_2_z))
            * (
                0.0
                if tau_z / r_quadrant_2_z == 0
                else math.copysign(1, tau_z / r_quadrant_2_z)
            )
            / math.sqrt(C)
            + math.sqrt(max(0, (1 / 8) * thrust)) / math.sqrt(C),
        ),
        max(
            0,
            (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_y / r_quadrant_2_y))
            * (
                0.0
                if tau_y / r_quadrant_2_y == 0
                else math.copysign(1, tau_y / r_quadrant_2_y)
            )
            / math.sqrt(C)
            + (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_z / r_quadrant_2_z))
            * (
                0.0
                if tau_z / r_quadrant_2_z == 0
                else math.copysign(1, tau_z / r_quadrant_2_z)
            )
            / math.sqrt(C)
            + math.sqrt(max(0, (1 / 8) * thrust)) / math.sqrt(C),
        ),
        max(
            0,
            (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_y / r_quadrant_3_y))
            * (
                0.0
                if tau_y / r_quadrant_3_y == 0
                else math.copysign(1, tau_y / r_quadrant_3_y)
            )
            / math.sqrt(C)
            + (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_z / r_quadrant_3_z))
            * (
                0.0
                if tau_z / r_quadrant_3_z == 0
                else math.copysign(1, tau_z / r_quadrant_3_z)
            )
            / math.sqrt(C)
            + math.sqrt(max(0, (1 / 8) * thrust)) / math.sqrt(C),
        ),
        max(
            0,
            (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_y / r_quadrant_3_y))
            * (
                0.0
                if tau_y / r_quadrant_3_y == 0
                else math.copysign(1, tau_y / r_quadrant_3_y)
            )
            / math.sqrt(C)
            + (1 / 4)
            * math.sqrt(2)
            * math.sqrt(abs(tau_z / r_quadrant_3_z))
            * (
                0.0
                if tau_z / r_quadrant_3_z == 0
                else math.copysign(1, tau_z / r_quadrant_3_z)
            )
            / math.sqrt(C)
            + math.sqrt(max(0, (1 / 8) * thrust)) / math.sqrt(C),
        ),
    )
