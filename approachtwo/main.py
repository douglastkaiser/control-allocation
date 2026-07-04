import sympy as sp
import sympy.physics.mechanics as me
from IPython.display import display, Math
import numpy as np

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


import numpy as np
def allocate(tau_y, tau_z, thrust):
  # Setup: u = A*w
  # w = np.array([w0^2, w1^2, w2^2, w3^2, w4^2, w5^2, w6^2, w7^2])
  u = np.array([[tau_y, tau_z, thrust]])

  # F = C*w^2
  # tau = r*F
  C = 1
  # Moment arms
  r0_yz = [1.5, -1]
  r1_yz = [0.5, -1]
  r2_yz = [-1.5, -1]
  r3_yz = [-0.5, -1]
  r4_yz = [1.5, 1]
  r5_yz = [0.5, 1]
  r6_yz = [-1.5, 1]
  r7_yz = [-0.5, 1]

  A = np.array([[r0_yz[0]*C, r1_yz[0]*C, r2_yz[0]*C, r3_yz[0]*C, r4_yz[0]*C, r5_yz[0]*C, r6_yz[0]*C, r7_yz[0]*C],
                [r0_yz[1]*C, r1_yz[1]*C, r2_yz[1]*C, r3_yz[1]*C, r4_yz[1]*C, r5_yz[1]*C, r6_yz[1]*C, r7_yz[1]*C],
                [C, C, C, C, C, C, C, C]])
  
  print(A)

  # w = np.linalg.solve(A, u)
  pinvA = np.linalg.inv(A.transpose() @ A) @ A.transpose()
  w = pinvA*u

  return w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7]
