import numpy as np
import math


def create_A(motors_active):
    # F = C*w^2
    C = 1
    # Moment arms
    r0_yz = [1.5, -1.1]
    r1_yz = [0.5, -1.1]
    r2_yz = [-1.5, -1.1]
    r3_yz = [-0.5, -1.1]
    r4_yz = [1.5, 0.9]
    r5_yz = [0.5, 0.9]
    r6_yz = [-1.5, 0.9]
    r7_yz = [-0.5, 0.9]

    A = np.array(
        [
            [
                r0_yz[0] * C,
                r1_yz[0] * C,
                r2_yz[0] * C,
                r3_yz[0] * C,
                r4_yz[0] * C,
                r5_yz[0] * C,
                r6_yz[0] * C,
                r7_yz[0] * C,
            ],
            [
                r0_yz[1] * C,
                r1_yz[1] * C,
                r2_yz[1] * C,
                r3_yz[1] * C,
                r4_yz[1] * C,
                r5_yz[1] * C,
                r6_yz[1] * C,
                r7_yz[1] * C,
            ],
            [C, C, C, C, C, C, C, C],
        ]
    )

    return A * motors_active


def allocate(u, motors_active=[1, 1, 1, 1, 1, 1, 1, 1]):
    tau_y, tau_z, thrust = u

    # Target vector u (3, 1)
    u = np.array([[tau_y], [tau_z], [thrust]])

    A = create_A(motors_active)

    # Manual Right-Inverse: pinvA = A.T @ inv(A @ A.T)
    # A @ A.T is a 3x3 matrix (Full Rank), which is invertible.
    A_gram = A @ A.transpose()
    # SR - singularity robust alpha identitiy
    alpha_0 = 1
    mu = 1
    m = np.sqrt(np.linalg.det(A @ A.transpose()))
    alpha = alpha_0 * math.exp(-mu * m)
    A_gram += alpha * np.identity(3)

    pinvA_manual = A.transpose() @ np.linalg.inv(A_gram)

    # Solve for squared motor speeds
    w_sq = pinvA_manual @ u

    # Convert squared speeds to w, clamping at 0 to avoid sqrt of negatives
    w = np.sqrt(np.maximum(w_sq, 0))

    return tuple(w.flatten())
