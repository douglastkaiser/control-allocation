import math


def allocate(
    y,
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

    tau_y, tau_z, thrust = y

    thrust_per_motor = thrust / 8

    # F = C*w^2
    C = 1
    w0 = w1 = w2 = w3 = w4 = w5 = w6 = w7 = math.sqrt(max(thrust_per_motor, 0) / C)

    tau_y_per_quadrant = tau_y / 4
    thrust_quadrant_0 = tau_y_per_quadrant / quadrant_0_yz[0]
    thrust_quadrant_1 = tau_y_per_quadrant / quadrant_1_yz[0]
    thrust_quadrant_2 = tau_y_per_quadrant / quadrant_2_yz[0]
    thrust_quadrant_3 = tau_y_per_quadrant / quadrant_3_yz[0]
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

    tau_z_per_quadrant = tau_z / 4
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


def sim(u, x):
    w0, w1, w2, w3, w4, w5, w6, w7 = u
    pitch_rate, yaw_rate, u_air = x

    dt = 0.01

    C = 1
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
    C = 1
    airspeed = math.sqrt(T / C)

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
    pitch_rate = pitch_rate + pitch_accel * dt

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
    yaw_rate = yaw_rate + yaw_accel * dt

    return pitch_rate, yaw_rate, airspeed
