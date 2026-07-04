import math

def allocate(tau_y, tau_z, thrust):
    """
    Allocate the things!

    Take torque commands and thrust command

    Produce motor speed commands
    """

    moment_arm_y = 1
    moment_arm_z = 1

    thrust_per_motor = thrust/8

    # F = C*w^2
    C = 1
    w0 = w1 = w2 = w3 = w4 = w5 = w6 = w7 = math.sqrt(thrust_per_motor/C)

    tau_y_per_wing = tau_y/2
    thrust_per_motor_tau_y = tau_y_per_wing/moment_arm_y/4
    # positive tau from back wing thrust
    w0 -= math.sqrt(thrust_per_motor_tau_y/C)
    w1 -= math.sqrt(thrust_per_motor_tau_y/C)
    w2 -= math.sqrt(thrust_per_motor_tau_y/C)
    w3 -= math.sqrt(thrust_per_motor_tau_y/C)
    w4 += math.sqrt(thrust_per_motor_tau_y/C)
    w5 += math.sqrt(thrust_per_motor_tau_y/C)
    w6 += math.sqrt(thrust_per_motor_tau_y/C)
    w7 += math.sqrt(thrust_per_motor_tau_y/C)

    tau_z_per_side = tau_z/2
    thrust_per_motor_tau_z = tau_z_per_side/moment_arm_z/4
    # positive tau comes from the right side
    w0 -= math.sqrt(thrust_per_motor_tau_y/C)
    w1 -= math.sqrt(thrust_per_motor_tau_y/C)
    w2 += math.sqrt(thrust_per_motor_tau_y/C)
    w3 += math.sqrt(thrust_per_motor_tau_y/C)
    w4 -= math.sqrt(thrust_per_motor_tau_y/C)
    w5 -= math.sqrt(thrust_per_motor_tau_y/C)
    w6 += math.sqrt(thrust_per_motor_tau_y/C)
    w7 += math.sqrt(thrust_per_motor_tau_y/C)

    return w0, w1, w2, w3, w4, w5, w6, w7
