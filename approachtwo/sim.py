# This file is auto-generated
import math


def sim(w0, w1, w2, w3, w4, w5, w6, w7, pitch_rate_in, yaw_rate_in, u_air_in, C, dt):
    x0 = C*w0**2
    x1 = C*w1**2
    x2 = C*w2**2
    x3 = C*w3**2
    x4 = w4**2
    x5 = w5**2
    x6 = w6**2
    x7 = w7**2
    x8 = C*x4
    x9 = C*x5
    x10 = C*x6
    x11 = C*x7
    return (dt*(1.1*C*x4 + 1.1*C*x5 + 1.1*C*x6 + 1.1*C*x7 - 0.9*x0 - 0.9*x1 - 0.9*x2 - 0.9*x3) + pitch_rate_in, dt*(1.5*x0 + 0.8*x1 - 1.5*x10 - 0.8*x11 - 1.5*x2 - 0.8*x3 + 1.5*x8 + 0.8*x9) + yaw_rate_in, math.sqrt(x0 + x1 + x10 + x11 + x2 + x3 + x8 + x9)/math.sqrt(C))
