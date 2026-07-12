from approachone.sim import allocate
import numpy as np


def test_allocate():
    w0, w1, w2, w3, w4, w5, w6, w7 = allocate(np.array([0, 0, 0]))
    assert w0 == w1 == w2 == w3 == w4 == w5 == w6 == w7 == 0

    w0, w1, w2, w3, w4, w5, w6, w7 = allocate(np.array([0, 0, -100]))
    assert w0 == w1 == w2 == w3 == w4 == w5 == w6 == w7 == 0

    w0, w1, w2, w3, w4, w5, w6, w7 = allocate(np.array([0, 0, 100]))
    assert w0 == w1 == w2 == w3 == w4 == w5 == w6 == w7
