import sympy as sp

allocate_file = "# This file is auto-generated\n"

allocate_file += "import math\n"
allocate_file += "def allocate(tau_y, tau_z, thrust):\n"
allocate_file += (
    "    r_quadrant_0_y = r_quadrant_1_y = r_quadrant_2_y = r_quadrant_3_y = 1\n"
)
allocate_file += (
    "    r_quadrant_0_z = r_quadrant_1_z = r_quadrant_2_z = r_quadrant_3_z = 1\n"
)
allocate_file += "    C = 1\n"

# tau_y, tau_z, thrust = y
tau_y = sp.Symbol("tau_y")
tau_z = sp.Symbol("tau_z")
thrust = sp.Symbol("thrust")

r_quadrant_0_y, r_quadrant_1_y, r_quadrant_2_y, r_quadrant_3_y = sp.symbols(
    "r_quadrant_0_y r_quadrant_1_y r_quadrant_2_y r_quadrant_3_y"
)
r_quadrant_0_z, r_quadrant_1_z, r_quadrant_2_z, r_quadrant_3_z = sp.symbols(
    "r_quadrant_0_z r_quadrant_1_z r_quadrant_2_z r_quadrant_3_z"
)

# actual math
thrust_per_motor = thrust / 8

C = sp.Symbol("C", positive=True)
w0 = w1 = w2 = w3 = w4 = w5 = w6 = w7 = sp.sqrt(sp.Max(thrust_per_motor, 0) / C)

tau_y_per_quadrant = tau_y / 4
thrust_quadrant_0 = tau_y_per_quadrant / r_quadrant_0_y
thrust_quadrant_1 = tau_y_per_quadrant / r_quadrant_1_y
thrust_quadrant_2 = tau_y_per_quadrant / r_quadrant_2_y
thrust_quadrant_3 = tau_y_per_quadrant / r_quadrant_3_y
# positive tau from back wing thrust
w0 += sp.sqrt(sp.Abs(thrust_quadrant_0) / 2 / C) * sp.sign(thrust_quadrant_0)
w1 += sp.sqrt(sp.Abs(thrust_quadrant_0) / 2 / C) * sp.sign(thrust_quadrant_0)
w2 += sp.sqrt(sp.Abs(thrust_quadrant_1) / 2 / C) * sp.sign(thrust_quadrant_1)
w3 += sp.sqrt(sp.Abs(thrust_quadrant_1) / 2 / C) * sp.sign(thrust_quadrant_1)
w4 += sp.sqrt(sp.Abs(thrust_quadrant_2) / 2 / C) * sp.sign(thrust_quadrant_2)
w5 += sp.sqrt(sp.Abs(thrust_quadrant_2) / 2 / C) * sp.sign(thrust_quadrant_2)
w6 += sp.sqrt(sp.Abs(thrust_quadrant_3) / 2 / C) * sp.sign(thrust_quadrant_3)
w7 += sp.sqrt(sp.Abs(thrust_quadrant_3) / 2 / C) * sp.sign(thrust_quadrant_3)

tau_z_per_quadrant = tau_z / 4
thrust_quadrant_0 = tau_z_per_quadrant / r_quadrant_0_z
thrust_quadrant_1 = tau_z_per_quadrant / r_quadrant_1_z
thrust_quadrant_2 = tau_z_per_quadrant / r_quadrant_2_z
thrust_quadrant_3 = tau_z_per_quadrant / r_quadrant_3_z
w0 += sp.sqrt(sp.Abs(thrust_quadrant_0) / 2 / C) * sp.sign(thrust_quadrant_0)
w1 += sp.sqrt(sp.Abs(thrust_quadrant_0) / 2 / C) * sp.sign(thrust_quadrant_0)
w2 += sp.sqrt(sp.Abs(thrust_quadrant_1) / 2 / C) * sp.sign(thrust_quadrant_1)
w3 += sp.sqrt(sp.Abs(thrust_quadrant_1) / 2 / C) * sp.sign(thrust_quadrant_1)
w4 += sp.sqrt(sp.Abs(thrust_quadrant_2) / 2 / C) * sp.sign(thrust_quadrant_2)
w5 += sp.sqrt(sp.Abs(thrust_quadrant_2) / 2 / C) * sp.sign(thrust_quadrant_2)
w6 += sp.sqrt(sp.Abs(thrust_quadrant_3) / 2 / C) * sp.sign(thrust_quadrant_3)
w7 += sp.sqrt(sp.Abs(thrust_quadrant_3) / 2 / C) * sp.sign(thrust_quadrant_3)

w0 = sp.Max(w0, 0)
w1 = sp.Max(w1, 0)
w2 = sp.Max(w2, 0)
w3 = sp.Max(w3, 0)
w4 = sp.Max(w4, 0)
w5 = sp.Max(w5, 0)
w6 = sp.Max(w6, 0)
w7 = sp.Max(w7, 0)

allocate_file += "    return " + sp.pycode((w0, w1, w2, w3, w4, w5, w6, w7))

with open("allocate.py", "w") as f:
    f.write(allocate_file)
