# This file is auto-generated
import math


def evaluate_vectors(
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
):
    x0 = 1 / math.sqrt(C)
    x1 = x0 * math.sqrt(max(0, (1 / 8) * thrust))
    x2 = tau_y / r_quadrant_0_y
    x3 = (1 / 4) * math.sqrt(2) * x0
    x4 = tau_z / r_quadrant_0_z
    x5 = max(
        0,
        x1
        + x3 * math.sqrt(abs(x2)) * (0.0 if x2 == 0 else math.copysign(1, x2))
        + x3 * math.sqrt(abs(x4)) * (0.0 if x4 == 0 else math.copysign(1, x4)),
    )
    x6 = tau_y / r_quadrant_1_y
    x7 = tau_z / r_quadrant_1_z
    x8 = max(
        0,
        x1
        + x3 * math.sqrt(abs(x6)) * (0.0 if x6 == 0 else math.copysign(1, x6))
        + x3 * math.sqrt(abs(x7)) * (0.0 if x7 == 0 else math.copysign(1, x7)),
    )
    x9 = tau_y / r_quadrant_2_y
    x10 = tau_z / r_quadrant_2_z
    x11 = max(
        0,
        x1
        + x3 * math.sqrt(abs(x10)) * (0.0 if x10 == 0 else math.copysign(1, x10))
        + x3 * math.sqrt(abs(x9)) * (0.0 if x9 == 0 else math.copysign(1, x9)),
    )
    x12 = tau_y / r_quadrant_3_y
    x13 = tau_z / r_quadrant_3_z
    x14 = max(
        0,
        x1
        + x3 * math.sqrt(abs(x12)) * (0.0 if x12 == 0 else math.copysign(1, x12))
        + x3 * math.sqrt(abs(x13)) * (0.0 if x13 == 0 else math.copysign(1, x13)),
    )
    return (x5, x5, x8, x8, x11, x11, x14, x14)
