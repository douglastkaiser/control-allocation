from common.utils import controllers


def test_controllers():
    pitch_rate = 0
    pitch_rate_cmd = 0
    yaw_rate = 0
    yaw_rate_cmd = 0
    airspeed = 0
    airspeed_cmd = 0
    x = [pitch_rate, yaw_rate, airspeed]
    x_cmd = [pitch_rate_cmd, yaw_rate_cmd, airspeed_cmd]

    integrals = [0, 0, 0]
    prev_deltas = [0, 0, 0]
    gains = [[1, 0, 0], [0, 0, 0], [0, 0, 0]]
    dt = 0.01

    u, integrals, prev_deltas = controllers(x, x_cmd, integrals, prev_deltas, gains, dt)
    tau_y, tau_z, thrust = u
    assert tau_y == tau_z == thrust == 0
