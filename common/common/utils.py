import matplotlib.pyplot as plt
import sympy as sp
from IPython.display import display, Math

def sympy_to_png(sympy, file_path="output.png", dpi=300, fontsize=16):
    latex_to_png(sp.latex(sympy), file_path, dpi, fontsize)

def latex_to_png(latex_str, file_path="output.png", dpi=300, fontsize=16):
    fig = plt.figure(figsize=(0.01, 0.01))
    formula = f"${latex_str}$" if not latex_str.startswith("$") else latex_str
    fig.text(0, 0, formula, fontsize=fontsize, usetex=False)
    fig.savefig(
        file_path,
        dpi=dpi,
        transparent=True,
        bbox_inches='tight',
        pad_inches=0.1
    )
    plt.close(fig)

def controllers(pitch_rate, pitch_rate_cmd, yaw_rate, yaw_rate_cmd, airspeed, airspeed_cmd):
    kp = 1
    torque_y_cmd = kp*(pitch_rate_cmd - pitch_rate)
    torque_z_cmd = kp*(yaw_rate_cmd - yaw_rate)
    thrust_cmd = kp*(airspeed_cmd - airspeed)
    return torque_y_cmd, torque_z_cmd, thrust_cmd

def make_vector(frame, name):
  rx, ry, rz = sp.symbols(f'{name}_x {name}_y {name}_z')
  return rx * frame.x + ry * frame.y + rz * frame.z

def skew_matrix(v):
    return sp.Matrix([
        [    0, -v[2],  v[1]],
        [ v[2],     0, -v[0]],
        [-v[1],  v[0],     0]
    ])
