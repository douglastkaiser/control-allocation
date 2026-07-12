import matplotlib.pyplot as plt
import sympy as sp
from sympy.printing import pycode


def sympy_to_png(sympy, file_path="output.png", dpi=300, fontsize=16):
    latex_to_png(sp.latex(sympy), file_path, dpi, fontsize)


def latex_to_png(latex_str, file_path="output.png", dpi=300, fontsize=16):
    fig = plt.figure(figsize=(0.01, 0.01))
    formula = f"${latex_str}$" if not latex_str.startswith("$") else latex_str
    fig.text(0, 0, formula, fontsize=fontsize, usetex=False)
    fig.savefig(
        file_path, dpi=dpi, transparent=True, bbox_inches="tight", pad_inches=0.1
    )
    plt.close(fig)


def controllers(x, x_cmd, integrals, prev_deltas, gains, dt):
    # PID controller for n number of axes
    u = [xi * 0 for xi in x]
    for i in range(len(x)):
        delta = x_cmd[i] - x[i]
        d_delta = (prev_deltas[i] - delta) / dt
        prev_deltas[i] = delta
        integrals[i] += delta * dt
        kp, ki, kd = gains[i]
        u[i] = kp * delta + ki * integrals[i] + kd * d_delta

    return u, integrals, prev_deltas


def make_vector(frame, name):
    rx, ry, rz = sp.symbols(f"{name}_x {name}_y {name}_z")
    return rx * frame.x + ry * frame.y + rz * frame.z


def skew_matrix(v):
    return sp.Matrix([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def generate_python_function(func_name, args, exprs):
    """Generate a Python function from symbolic arguments and expressions."""
    subs, simplified = sp.cse(exprs)

    arg_str = ", ".join(pycode(a) for a in args)
    lines = ["", "", f"def {func_name}({arg_str}):"]

    for var, expr in subs:
        lines.append(f"    {pycode(var)} = {pycode(expr)}")

    lines.append(f"    return {pycode(tuple(simplified))}")

    return "\n".join(lines) + "\n"
