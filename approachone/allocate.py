# This file is auto-generated
import math


def allocate(tau_y, tau_z, thrust, C):
    x0 = 1/math.sqrt(C)
    x1 = math.sqrt(max(0, (1/8)*thrust))
    x2 = math.sqrt(2)*x0
    x3 = 0.25*x2*math.sqrt(abs(tau_z))*(0.0 if tau_z == 0 else math.copysign(1, tau_z))
    x4 = 0.233126202060078*x2*math.sqrt(abs(tau_y))*(0.0 if tau_y == 0 else math.copysign(1, tau_y))
    x5 = x3 + x4
    x6 = max(0, x0*x1 - x5)
    x7 = x0*x1
    x8 = max(0, -x3 + x4 + x7)
    x9 = max(0, x3 - x4 + x7)
    x10 = max(0, x5 + x7)
    return (x6, x6, x8, x8, x9, x9, x10, x10)
