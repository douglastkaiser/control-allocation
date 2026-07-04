
from common.utils import controllers

def test_controllers():
    pitch_rate = 0
    pitch_rate_cmd = 0
    yaw_rate = 0
    yaw_rate_cmd = 0
    airspeed = 0
    airspeed_cmd = 0
    tau_y, tau_z, thrust = controllers(pitch_rate, pitch_rate_cmd, yaw_rate, yaw_rate_cmd, airspeed, airspeed_cmd)
    assert tau_y == tau_z == thrust == 0
