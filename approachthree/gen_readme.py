import sympy as sp
import sympy.physics.mechanics as me
from IPython.display import display, Math
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))
from common import utils

readme = "# Control Allocation Third Approach\n"
readme += """
Quadratic Programming Polytopes:\n
Same as first two approaches:\n
Pitch rate, yaw rate, airpseed commands come into three PIDs
Torque z and y and thrust commands come out.
Each motor is represented in a matrix that maps those commands to motor speeds
\n
New to this approach:\n
MPT toolbox distributes torques in a way that accounts for motor saturation
"""

# To represent the cross product symbolically for display,
# we use the ReferenceFrame approach seen earlier or the cross function.
B = me.ReferenceFrame('B')

# Alternatively, for just a LaTeX representation of the general formula:
Fi = sp.Symbol('F_i')
ri = sp.Symbol('r_i')
taui = sp.Symbol('\\tau_i')

# We use a custom string or the mechanics cross for the equation
eq = sp.Eq(taui, sp.Symbol('r_i \\times F_i'))

latex_str_cross = sp.latex(eq)
display(Math(latex_str_cross))

# Rotation equation
w_dot = sp.symbols(r'\dot{w}')
tau = sp.symbols(r'\tau')
I = sp.symbols('I')
rotation_eq = sp.Eq(tau, I*w_dot)
latex_str_rot = sp.latex(rotation_eq)

utils.latex_to_png(latex_str_rot)

readme += "```math\n" + latex_str_rot + "\n```\n"

# Use += to ensure this is added to the variable
readme += "\n```math\nSE = \\frac{\\sigma}{\\sqrt{n}}\n```\n"

with open("README.md", "w") as f:
    f.write(readme)
