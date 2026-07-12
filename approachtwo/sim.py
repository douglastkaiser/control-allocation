# This file is auto-generated
import math


def sim(w0, w1, w2, w3, w4, w5, w6, w7, pitch_rate_in, yaw_rate_in, u_air_in, C, dt):
    x0 = w4**2
    x1 = w5**2
    x2 = w6**2
    x3 = w7**2
    x4 = C*w0**2
    x5 = C*w1**2
    x6 = C*w2**2
    x7 = C*w3**2
    x8 = x4 + x5 + x6 + x7
    x9 = C*x0
    x10 = C*x1
    x11 = C*x2
    x12 = C*x3
    return (dt*(C*x0 + C*x1 + C*x2 + C*x3 - x8) + pitch_rate_in, dt*(0.8*x10 - 1.5*x11 - 0.8*x12 + 1.5*x4 + 0.8*x5 - 1.5*x6 - 0.8*x7 + 1.5*x9) + yaw_rate_in, math.sqrt(x10 + x11 + x12 + x8 + x9)/math.sqrt(C))
