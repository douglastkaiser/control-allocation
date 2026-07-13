"""Generate approach three's README and its polytope diagrams.

Everything numeric in the README -- the allocation matrix, the attainable-set
polygons, and every allocated command -- is computed here from the same
``approachthree.model`` code that runs at allocation time and from the shared
symbolic model in ``common``. The prose is the only hand-written part.
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import sympy as sp  # noqa: E402
from matplotlib.patches import Polygon as PolygonPatch  # noqa: E402

sys.path.append(str(Path(__file__).resolve().parent.parent))

from approachthree.model import (  # noqa: E402
    DEFAULT_REG,
    DEFAULT_S_MAX,
    DEFAULT_WEIGHTS,
    achieved_command,
    allocated_motor_speeds,
    allocated_squared_speeds,
    attainable_moment_set,
    saturated_motors,
)
from common.geometry import MOTOR_R_Y, MOTOR_R_Z, N_MOTORS  # noqa: E402
from common.model import rigid_body_motion, single_motor_torque  # noqa: E402


# ---------------------------------------------------------------------------
# Palette (validated data-viz default palette, light surface).
# ---------------------------------------------------------------------------
SURFACE = "#ffffff"
INK = "#0b0b0b"
INK_SOFT = "#52514e"
GRID = "#e4e3df"
BLUE = "#2a78d6"  # nominal attainable set
ORANGE = "#eb6834"  # degraded (motor-out) attainable set
GREEN = "#0ca30c"  # achieved / feasible command
RED = "#d03b3b"  # requested command that lies outside the set


def math_block(expr):
    """Render a SymPy expression as a Markdown math block."""
    return "```math\n" + sp.latex(expr) + "\n```\n"


def latex_block(latex):
    """Render a raw LaTeX string as a Markdown math block."""
    return "```math\n" + latex + "\n```\n"


def code_block(code):
    """Render a Python snippet as a Markdown code block."""
    return "```python\n" + code.strip() + "\n```\n"


def numeric_matrix_block(matrix, decimals=4):
    """Render a rounded numeric matrix as a Markdown math block."""
    rounded = np.round(np.asarray(matrix, dtype=float), decimals)
    rounded[np.isclose(rounded, 0)] = 0

    return math_block(sp.Matrix(rounded))


def geometry_substitutions():
    """Return substitutions for the shared numeric motor geometry."""
    subs = {}
    for motor_index, r_y in enumerate(MOTOR_R_Y):
        subs[f"r_y{motor_index}"] = r_y
    for motor_index, r_z in enumerate(MOTOR_R_Z):
        subs[f"r_z{motor_index}"] = r_z

    return subs


def allocation_matrix_from_equations(active_motors=None):
    """Build the allocation matrix from the same symbolic equations as the sim."""
    if active_motors is None:
        active_motors = [1] * N_MOTORS

    tau_y, tau_z, thrust = rigid_body_motion()
    equations = [
        tau_y.subs(geometry_substitutions()),
        tau_z.subs(geometry_substitutions()),
        thrust,
    ]
    force_symbols = sp.symbols(f"f0:{N_MOTORS}")

    return sp.Matrix(
        [
            [
                sp.expand(equation).coeff(force) * active
                for force, active in zip(force_symbols, active_motors)
            ]
            for equation in equations
        ]
    )


# ---------------------------------------------------------------------------
# Diagram helpers. Each returns the file name it wrote.
# ---------------------------------------------------------------------------
def _style_axes(ax, title):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=INK, fontsize=13, pad=12)
    ax.set_xlabel(r"pitch torque  $\tau_y$", color=INK_SOFT, fontsize=11)
    ax.set_ylabel(r"yaw torque  $\tau_z$", color=INK_SOFT, fontsize=11)
    ax.axhline(0, color=GRID, lw=1, zorder=0)
    ax.axvline(0, color=GRID, lw=1, zorder=0)
    ax.grid(True, color=GRID, lw=0.6, zorder=0)
    ax.tick_params(colors=INK_SOFT, labelsize=9)
    for spine in ax.spines.values():
        spine.set_color(GRID)
    ax.set_aspect("equal", adjustable="box")


def _fill_polygon(ax, polygon, edge, face_alpha, label, lw=2.2, hatch=None):
    patch = PolygonPatch(
        polygon,
        closed=True,
        facecolor=edge,
        edgecolor=edge,
        alpha=face_alpha,
        lw=0,
        zorder=1,
        hatch=hatch,
    )
    ax.add_patch(patch)
    closed = np.vstack([polygon, polygon[0]])
    ax.plot(closed[:, 0], closed[:, 1], color=edge, lw=lw, zorder=3, label=label)


def _save(fig, name):
    fig.savefig(name, format="svg", facecolor=SURFACE, bbox_inches="tight")
    plt.close(fig)

    return name


def plot_attainable_moment_set(feasible, saturating):
    """Nominal attainable moment set with a feasible and a saturating command."""
    polygon = attainable_moment_set()
    fig, ax = plt.subplots(figsize=(6.4, 5.6))
    _style_axes(ax, "Attainable moment set  (all 8 motors)")
    _fill_polygon(ax, polygon, BLUE, 0.12, "attainable set  $\\mathcal{A}$")

    # Feasible request: delivered exactly, sits inside the polytope.
    fy, fz = feasible["command"][:2]
    ax.plot(
        fy,
        fz,
        "o",
        color=GREEN,
        ms=9,
        zorder=5,
        label="feasible request  (delivered exactly)",
    )

    # Saturating request: projected onto the boundary of the polytope.
    ry, rz = saturating["command"][:2]
    ay, az = saturating["achieved"][:2]
    ax.annotate(
        "",
        xy=(ay, az),
        xytext=(ry, rz),
        arrowprops=dict(arrowstyle="-|>", color=INK_SOFT, lw=1.8, shrinkA=0, shrinkB=0),
        zorder=4,
    )
    ax.plot(
        ry,
        rz,
        "X",
        color=RED,
        ms=11,
        zorder=5,
        label="requested  (outside $\\mathcal{A}$)",
    )
    ax.plot(
        ay, az, "o", color=GREEN, ms=9, zorder=6, label="delivered  (motors saturated)"
    )

    ax.legend(
        loc="upper left",
        fontsize=8.5,
        framealpha=0.95,
        edgecolor=GRID,
        facecolor=SURFACE,
        labelcolor=INK,
    )
    ax.margins(0.16)

    return _save(fig, "attainable_moment_set.svg")


def plot_motor_out(request):
    """Nominal vs motor-0-out moment sets, with a command lost to the failure."""
    nominal = attainable_moment_set()
    degraded = attainable_moment_set([0, 1, 1, 1, 1, 1, 1, 1])
    fig, ax = plt.subplots(figsize=(6.4, 5.6))
    _style_axes(ax, "Motor 0 lost:  the attainable set shrinks")
    _fill_polygon(ax, nominal, BLUE, 0.10, "nominal set  (8 motors)", lw=1.8)
    _fill_polygon(
        ax, degraded, ORANGE, 0.16, "motor-out set  (7 motors)", lw=2.2, hatch="///"
    )

    ry, rz = request["command"][:2]
    ay, az = request["achieved"][:2]
    ax.annotate(
        "",
        xy=(ay, az),
        xytext=(ry, rz),
        arrowprops=dict(arrowstyle="-|>", color=INK_SOFT, lw=1.8, shrinkA=0, shrinkB=0),
        zorder=4,
    )
    ax.plot(
        ry,
        rz,
        "X",
        color=RED,
        ms=11,
        zorder=5,
        label="request  (feasible only with 8 motors)",
    )
    ax.plot(ay, az, "o", color=GREEN, ms=9, zorder=6, label="delivered after failure")

    ax.legend(
        loc="upper left",
        fontsize=8.5,
        framealpha=0.95,
        edgecolor=GRID,
        facecolor=SURFACE,
        labelcolor=INK,
    )
    ax.margins(0.16)

    return _save(fig, "motor_out_moment_set.svg")


def plot_projection_field(thrust=100.0, n_directions=24):
    """Ring of over-large moment requests, each projected onto the polytope."""
    polygon = attainable_moment_set()
    radius = 1.35 * np.max(np.linalg.norm(polygon, axis=1))
    fig, ax = plt.subplots(figsize=(6.4, 5.6))
    _style_axes(ax, "Every out-of-range request maps onto the polytope")
    _fill_polygon(ax, polygon, BLUE, 0.12, "attainable set  $\\mathcal{A}$")

    first = True
    for angle in np.linspace(0, 2 * np.pi, n_directions, endpoint=False):
        req = (radius * np.cos(angle), radius * np.sin(angle), thrust)
        ach = achieved_command(req)
        ax.annotate(
            "",
            xy=(ach[0], ach[1]),
            xytext=(req[0], req[1]),
            arrowprops=dict(
                arrowstyle="-|>",
                color=INK_SOFT,
                lw=1.0,
                alpha=0.65,
                shrinkA=0,
                shrinkB=0,
            ),
            zorder=2,
        )
        ax.plot(
            req[0],
            req[1],
            "X",
            color=RED,
            ms=6,
            zorder=3,
            label="requested" if first else None,
        )
        ax.plot(
            ach[0],
            ach[1],
            "o",
            color=GREEN,
            ms=5,
            zorder=4,
            label="delivered" if first else None,
        )
        first = False

    ax.legend(
        loc="upper left",
        fontsize=8.5,
        framealpha=0.95,
        edgecolor=GRID,
        facecolor=SURFACE,
        labelcolor=INK,
    )
    ax.margins(0.06)

    return _save(fig, "saturation_projection.svg")


# ---------------------------------------------------------------------------
# Single source of truth: compute every demo command once, reuse everywhere.
# ---------------------------------------------------------------------------
def study(command, motors_active=None):
    """Run one command through the allocator and collect what the docs report."""
    s = allocated_squared_speeds(command, motors_active)
    return {
        "command": command,
        "squared_speeds": s,
        "motor_speeds": allocated_motor_speeds(command, motors_active),
        "achieved": achieved_command(command, motors_active),
        "saturated": int(saturated_motors(s).sum()),
        "motors_active": motors_active,
    }


hover = study((0, 0, 100))
feasible = study((-40, 40, 100))
saturating = study((80, 100, 100))
aggressive = study((0, 150, 100))
motor_out = study((0, 70, 100), [0, 1, 1, 1, 1, 1, 1, 1])

nominal_polygon = attainable_moment_set()
motor_out_polygon = attainable_moment_set([0, 1, 1, 1, 1, 1, 1, 1])

set_figure = plot_attainable_moment_set(feasible, saturating)
motor_out_figure = plot_motor_out(motor_out)
projection_figure = plot_projection_field()


# ---------------------------------------------------------------------------
# Symbolic building blocks.
# ---------------------------------------------------------------------------
_, _, tau_i = single_motor_torque()
A = allocation_matrix_from_equations()
motor_out_A = allocation_matrix_from_equations([0, 1, 1, 1, 1, 1, 1, 1])

A_sym = sp.MatrixSymbol("A", 3, N_MOTORS)
s_sym = sp.MatrixSymbol("s", N_MOTORS, 1)
u_sym = sp.MatrixSymbol("u", 3, 1)
W_sym = sp.MatrixSymbol("W", 3, 3)
W_matrix = sp.Matrix(np.diag(DEFAULT_WEIGHTS).astype(int))


def command_row(label, result):
    """Render one command study as a Markdown table row."""
    cy, cz, ct = result["command"]
    ay, az, at = (round(float(value) + 0.0, 1) or 0.0 for value in result["achieved"])
    feasible_flag = "yes" if result["saturated"] == 0 else "**saturated**"

    return (
        f"| {label} | ({cy}, {cz}, {ct}) | ({ay}, {az}, {at}) | "
        f"{result['saturated']} / {N_MOTORS} | {feasible_flag} |\n"
    )


# ---------------------------------------------------------------------------
# Assemble the README.
# ---------------------------------------------------------------------------
readme = "<!-- This file is auto-generated by gen_readme.py -->\n"
readme += "# Control Allocation Third Approach\n\n"
readme += """
Approaches one and two both allocate as if the motors could spin arbitrarily
fast. They solve an unconstrained problem and only clamp the final square root so
a caller never receives an imaginary speed. That clamp hides the moment when a
motor would have to exceed its physical limit, and it silently distorts the
commanded torque.

Approach three keeps the exact same command interface -- pitch torque, yaw
torque and total thrust -- but makes the per-motor **saturation limits** part of
the problem. The set of commands the motors can actually deliver is a **polytope**,
and allocation becomes a small **quadratic program (QP)** that projects the
desired command onto that polytope. A request inside the polytope is delivered
exactly; a request outside it is met by the closest command the motors can
produce.

## Building blocks

Each motor uses the same quadratic propeller model as the earlier approaches, so
we allocate in squared-speed space ``s_i = n_i^2``.
"""
readme += math_block(sp.Eq(sp.Symbol("F_i"), sp.Symbol("C") * sp.Symbol("n_i") ** 2))
readme += (
    "Torque comes from the same SymPy cross product the generated simulator "
    "uses, so the documentation and the flight code never disagree.\n"
)
readme += math_block(sp.Eq(sp.MatrixSymbol(r"\tau_i", 3, 1), tau_i))
readme += (
    "Stacking pitch torque, yaw torque and thrust over all motors gives the "
    "linear allocation map. Working in squared-speed space keeps it exactly "
    "linear, so the command vector is a matrix product.\n"
)
readme += math_block(sp.Eq(u_sym, A_sym * s_sym))
readme += "With the current shared geometry the allocation matrix is:\n"
readme += math_block(sp.Eq(A_sym, A))

readme += """
## Saturation makes the problem a polytope

A real motor cannot spin backwards and cannot exceed its top speed, so every
squared speed is boxed:
"""
readme += latex_block(
    r"0 \le s_i \le s_{\max}, \qquad s_{\max} = " + f"{DEFAULT_S_MAX:g}"
)
readme += (
    "That box is a set in 8-D squared-speed space. Its image under the "
    "allocation matrix is the set of every command the motors can actually "
    "produce -- the **attainable command set**. Because a linear image of a box "
    "is a zonotope, this set is a bounded convex polytope:\n"
)
readme += latex_block(
    r"\mathcal{A} = \{\, A\,s \;:\; 0 \le s \le s_{\max} \,\} \subset "
    r"\mathbb{R}^3"
)
readme += (
    "Projected onto the pitch/yaw torque plane it is the polygon below (its "
    "boundary is traced by the motors sitting on their limits). The vertices "
    "are computed by `approachthree.model.attainable_moment_set`, the same code "
    "the allocator trusts:\n"
)
readme += numeric_matrix_block(nominal_polygon, decimals=1)
readme += f"\n![Attainable moment set]({set_figure})\n\n"
readme += (
    "The green request sits inside the polytope and is delivered exactly. The "
    "red request lies outside it -- no combination of motor speeds can produce "
    "that much torque -- so the allocator delivers the nearest point on the "
    "boundary instead, with several motors driven to saturation.\n"
)

readme += """
## Allocation is a quadratic program

Rather than invert the allocation matrix and hope the answer is feasible, the
allocator solves for the squared speeds directly as a bound-constrained
least-squares problem: get as close to the commanded ``u`` as possible while
staying inside the box.
"""
readme += latex_block(
    r"\begin{aligned}"
    r"\min_{s}\ & \tfrac{1}{2}\,\lVert A s - u \rVert_W^{2}"
    r" + \tfrac{\lambda}{2}\,\lVert s \rVert^{2} \\"
    r"\text{s.t.}\ & 0 \le s \le s_{\max}"
    r"\end{aligned}"
)
readme += (
    f"The weight matrix $W$ prioritises attitude over thrust and the effort "
    f"weight $\\lambda$ is `{DEFAULT_REG:g}` -- the same role approach two's "
    "damping plays, keeping the solution unique and the problem strictly "
    "convex.\n"
)
readme += math_block(sp.Eq(W_sym, W_matrix))
readme += (
    "Expanding the norms turns this into a standard convex QP with a positive "
    "definite Hessian, whose optimum is the projection of $u$ onto the "
    "polytope $\\mathcal{A}$:\n"
)
readme += latex_block(
    r"\min_{s}\ \tfrac{1}{2}\, s^{\top} H s + c^{\top} s \quad"
    r"\text{s.t.}\ 0 \le s \le s_{\max}, \qquad "
    r"H = A^{\top} W A + \lambda I,\ \ c = -A^{\top} W u"
)

readme += """
## Solving the QP ourselves

In the spirit of approach two assembling its own pseudoinverse, we do not call a
QP library. The optimum is found with a primal **active-set** iteration: motors
that want to leave the feasible box are pinned to a bound, the remaining free
motors are driven to the equality-constrained minimum, and a bound is released
only when its KKT multiplier proves the cost can still fall. Because the QP is
strictly convex, the KKT conditions are sufficient, so this fixed point is the
global optimum.
"""
readme += latex_block(
    r"(Hs+c)_i = 0\ \text{(free)},\quad"
    r"(Hs+c)_i \ge 0\ \text{(at } 0),\quad"
    r"(Hs+c)_i \le 0\ \text{(at } s_{\max})"
)
readme += code_block(
    """
s = allocated_squared_speeds(u, motors_active)   # active-set QP, box-constrained
w = allocated_motor_speeds(u, motors_active)      # = sqrt(s), always real & feasible
"""
)

readme += """
## Handling motor saturation

The table below runs a sweep of commands through the allocator. Feasible
commands are delivered exactly; once a command leaves the polytope the allocator
holds the prioritised axes and reports exactly how many motors are pinned to a
limit -- there is no silent clipping.
"""
readme += "\n"
readme += "| command | requested $(\\tau_y, \\tau_z, T)$ | delivered | motors on a limit | status |\n"
readme += "| --- | --- | --- | --- | --- |\n"
readme += command_row("hover", hover)
readme += command_row("feasible maneuver", feasible)
readme += command_row("aggressive yaw", aggressive)
readme += command_row("combined, over-range", saturating)
readme += (
    "\nFor the over-range combined command the QP still returns a fully "
    "feasible squared-speed vector -- some motors floored at zero, others "
    "pinned at ``s_max`` -- rather than an infeasible one:\n"
)
readme += numeric_matrix_block(
    np.array(saturating["squared_speeds"]).reshape(N_MOTORS, 1)
)
readme += "The projection is not special to one direction. Every over-range request maps onto the polytope boundary:\n"
readme += f"\n![Saturation projection field]({projection_figure})\n\n"

readme += """
## Motor-out example

Losing a motor removes its column from the allocation matrix, which shrinks the
attainable polytope. A command that was comfortably feasible with eight motors
can fall outside the degraded set, and the same QP handles it without any
special-casing -- it simply projects onto the smaller polytope.
"""
readme += "The motor-out allocation matrix (motor 0 disabled) is:\n"
readme += math_block(sp.Eq(A_sym, motor_out_A))
readme += "and the attainable moment polygon collapses to:\n"
readme += numeric_matrix_block(motor_out_polygon, decimals=1)
readme += f"\n![Motor-out attainable set]({motor_out_figure})\n\n"
cy, cz, ct = motor_out["command"]
ay, az, at = (round(float(value), 2) for value in motor_out["achieved"])
readme += (
    f"With all eight motors the command ``({cy}, {cz}, {ct})`` is delivered "
    f"exactly. After motor 0 fails it lies outside the degraded polytope, so "
    f"the allocator delivers the closest feasible command ``({ay}, {az}, "
    f"{at})`` with the surviving seven motors and reports "
    f"``{motor_out['saturated']}`` of ``{N_MOTORS}`` motors on a limit.\n"
)

with open("README.md", "w") as f:
    f.write(readme)
