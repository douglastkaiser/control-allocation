import math


def allocate(
    tau_y,
    tau_z,
    thrust,
    quadrant_0_yz=[1, -1],
    quadrant_1_yz=[-1, -1],
    quadrant_2_yz=[1, 1],
    quadrant_3_yz=[-1, 1],
):
    """
    Allocate the things!

    Take torque commands and thrust command

    Produce motor speed commands
    """

    thrust_per_motor = thrust / 8

    # F = C*w^2
    C = 1
    w0 = w1 = w2 = w3 = w4 = w5 = w6 = w7 = math.sqrt(max(thrust_per_motor, 0) / C)

    tau_y_per_quadrant = tau_y / 4
    thrust_quadrant_0 = tau_y_per_quadrant / quadrant_0_yz[0]
    thrust_quadrant_1 = tau_y_per_quadrant / quadrant_1_yz[0]
    thrust_quadrant_2 = tau_y_per_quadrant / quadrant_2_yz[0]
    thrust_quadrant_3 = tau_y_per_quadrant / quadrant_3_yz[0]
    # positive tau from back wing thrust
    w0 += math.sqrt(abs(thrust_quadrant_0) / 2 / C) * math.copysign(
        1, thrust_quadrant_0
    )
    w1 += math.sqrt(abs(thrust_quadrant_0) / 2 / C) * math.copysign(
        1, thrust_quadrant_0
    )
    w2 += math.sqrt(abs(thrust_quadrant_1) / 2 / C) * math.copysign(
        1, thrust_quadrant_1
    )
    w3 += math.sqrt(abs(thrust_quadrant_1) / 2 / C) * math.copysign(
        1, thrust_quadrant_1
    )
    w4 += math.sqrt(abs(thrust_quadrant_2) / 2 / C) * math.copysign(
        1, thrust_quadrant_2
    )
    w5 += math.sqrt(abs(thrust_quadrant_2) / 2 / C) * math.copysign(
        1, thrust_quadrant_2
    )
    w6 += math.sqrt(abs(thrust_quadrant_3) / 2 / C) * math.copysign(
        1, thrust_quadrant_3
    )
    w7 += math.sqrt(abs(thrust_quadrant_3) / 2 / C) * math.copysign(
        1, thrust_quadrant_3
    )

    tau_z_per_quadrant = tau_z / 2
    thrust_quadrant_0 = tau_z_per_quadrant / quadrant_0_yz[1]
    thrust_quadrant_1 = tau_z_per_quadrant / quadrant_1_yz[1]
    thrust_quadrant_2 = tau_z_per_quadrant / quadrant_2_yz[1]
    thrust_quadrant_3 = tau_z_per_quadrant / quadrant_3_yz[1]
    w0 += math.sqrt(abs(thrust_quadrant_0) / 2 / C) * math.copysign(
        1, thrust_quadrant_0
    )
    w1 += math.sqrt(abs(thrust_quadrant_0) / 2 / C) * math.copysign(
        1, thrust_quadrant_0
    )
    w2 += math.sqrt(abs(thrust_quadrant_1) / 2 / C) * math.copysign(
        1, thrust_quadrant_1
    )
    w3 += math.sqrt(abs(thrust_quadrant_1) / 2 / C) * math.copysign(
        1, thrust_quadrant_1
    )
    w4 += math.sqrt(abs(thrust_quadrant_2) / 2 / C) * math.copysign(
        1, thrust_quadrant_2
    )
    w5 += math.sqrt(abs(thrust_quadrant_2) / 2 / C) * math.copysign(
        1, thrust_quadrant_2
    )
    w6 += math.sqrt(abs(thrust_quadrant_3) / 2 / C) * math.copysign(
        1, thrust_quadrant_3
    )
    w7 += math.sqrt(abs(thrust_quadrant_3) / 2 / C) * math.copysign(
        1, thrust_quadrant_3
    )

    w0 = max(w0, 0)
    w1 = max(w1, 0)
    w2 = max(w2, 0)
    w3 = max(w3, 0)
    w4 = max(w4, 0)
    w5 = max(w5, 0)
    w6 = max(w6, 0)
    w7 = max(w7, 0)

    return w0, w1, w2, w3, w4, w5, w6, w7
