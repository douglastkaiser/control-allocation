# This file is auto-generated
import math


def allocate(tau_y, tau_z, thrust, C):
    x0 = 1/math.sqrt(C)
    x1 = 0.233126202060078*math.sqrt(2)*x0*math.sqrt(abs(tau_z))*(0.0 if tau_z == 0 else math.copysign(1, tau_z))
    x2 = 0.125*tau_y
    x3 = math.sqrt(max(0, 0.1375*thrust - x2))
    x4 = max(0, x0*x3 - x1)
    x5 = max(0, x0*x3 + x1)
    x6 = math.sqrt(max(0, 0.1125*thrust + x2))
    x7 = max(0, x0*x6 - x1)
    x8 = max(0, x0*x6 + x1)
    return (x4, x4, x5, x5, x7, x7, x8, x8)
