# This file is auto-generated
import math


def allocate(tau_y, tau_z, thrust, C):
    x0 = 1/math.sqrt(C)
    x1 = (1/4)*math.sqrt(2)*x0
    x2 = x1*math.sqrt(abs(tau_z))*(0.0 if tau_z == 0 else math.copysign(1, tau_z))
    x3 = math.sqrt(max(0, (1/8)*thrust))
    x4 = x0*x3
    x5 = x1*math.sqrt(abs(tau_y))*(0.0 if tau_y == 0 else math.copysign(1, tau_y))
    x6 = x4 + x5
    x7 = max(0, -x2 + x6)
    x8 = max(0, x0*x3 - x2 - x5)
    x9 = max(0, x2 + x6)
    x10 = max(0, x2 + x4 - x5)
    return (x7, x7, x8, x8, x9, x9, x10, x10)
