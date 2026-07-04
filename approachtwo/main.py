import sympy as sp
import sympy.physics.mechanics as me
from IPython.display import display, Math

B = me.ReferenceFrame('B')

def make_vector(frame, name):
  rx, ry, rz = sp.symbols(f'{name}_x {name}_y {name}_z')
  return rx * frame.x + ry * frame.y + rz * frame.z

F_motors = []
r_motors = []
for i in range(8):
  F = make_vector(B, f'F{i+1}')
  # F only acts in the x axis
  F = F.subs(sp.Symbol(f'F{i+1}_y'), 0)
  F = F.subs(sp.Symbol(f'F{i+1}_z'), 0)
  F_motors.append(F)
  r_motors.append(make_vector(B, f'r{i+1}'))

torque_vec = 0
thrust_vec = 0
for i in range(8):
  torque_vec += r_motors[i].cross(F_motors[i])
  thrust_vec += F_motors[i]

print("Net Torque")
latex_str = sp.latex(torque_vec)
display(Math(latex_str))
print("Net Thrust")
latex_str = sp.latex(thrust_vec)
display(Math(latex_str))




def allocate(tau_y, tau_z, thrust):
  w0 = w1 = w2 = w3 = w4 = w5 = w6 = w7 = 0
  return w0, w1, w2, w3, w4, w5, w6, w7
