import sympy as sp

readme = "<!-- This file is auto-generated -->\n"
readme += "# Control Allocation First Approach\n"
readme += """
Linear split:\n
Pitch rate, yaw rate, airpseed commands come into three PIDs
Torque z and y and thrust commands come out.
These are then summed to create Motor spin speed commands
"""

# Define your vectors as symbols
r = sp.symbols(r"\mathbf{r_i}")
F = sp.symbols(r"\mathbf{F_i}")
tau = sp.symbols(r"\boldsymbol{\tau_i}")

# Use /times because it displays as a cross product
torque_latex_str = sp.latex(tau) + r" = " + sp.latex(r) + r" \times " + sp.latex(F)
readme += "```math\n" + torque_latex_str + "\n```\n"

T = sp.symbols(r"T")
thrust_latex_str = sp.latex(T) + r" = \sum_{i=1}^{n} F_i"
readme += "```math\n" + thrust_latex_str + "\n```\n"

ct, rho, n, D = sp.symbols(r"C_T \rho n D")
C = sp.symbols(r"C")
single_prop_thrust = sp.Eq(F, ct * rho * n**2 * D**4)
single_prop_thrust_simplified = sp.Eq(F, C * n**2)

readme += "```math\n" + sp.latex(single_prop_thrust) + "\n```\n"
readme += "```math\n" + sp.latex(single_prop_thrust_simplified) + "\n```\n"

with open("README.md", "w") as f:
    f.write(readme)
