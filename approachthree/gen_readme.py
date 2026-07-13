"""Generate approach three's README and its polytope diagrams.

Everything numeric in the README -- the allocation matrix, the attainable-set
volumes, the controllability verdicts, and every allocated command -- is computed
here from the same ``approachthree.model`` code that runs at allocation time and
from the shared symbolic model in ``common``. The prose is the only hand-written
part.

The diagrams are written as PNGs under ``diagrams/`` so the pull request stays
small; a ``.gitignore`` negation keeps just those files tracked.
"""

import os
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import sympy as sp  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

sys.path.append(str(Path(__file__).resolve().parent.parent))

from approachthree.model import (  # noqa: E402
    DEFAULT_REG,
    DEFAULT_S_MAX,
    DEFAULT_TRIM,
    DEFAULT_WEIGHTS,
    achieved_command,
    allocated_motor_speeds,
    allocated_squared_speeds,
    attainable_generators,
    attainable_set_faces,
    controllability,
    saturated_motors,
    zonotope_volume,
)
from common.geometry import MOTOR_R_Y, MOTOR_R_Z, N_MOTORS  # noqa: E402
from common.model import rigid_body_motion, single_motor_torque  # noqa: E402


# ---------------------------------------------------------------------------
# Palette (validated data-viz default palette, light surface).
# ---------------------------------------------------------------------------
SURFACE = "#ffffff"
INK = "#0b0b0b"
INK_SOFT = "#52514e"
GRID = "#c9c8c3"
BLUE = "#2a78d6"  # controllable attainable set
ORANGE = "#eb6834"  # degraded / uncontrollable attainable set
GREEN = "#0ca30c"  # delivered / feasible trim
RED = "#d03b3b"  # requested-but-infeasible / lost trim

DIAGRAMS = "diagrams"

# Shared axis extents so every scenario diagram is drawn to the same scale and
# the polytope is visibly seen to shrink as motors are lost.
_LIMITS = {
    "x": (-110, 110),
    "y": (-125, 125),
    "z": (0, N_MOTORS * DEFAULT_S_MAX + 10),
}


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
# 3-D polytope diagrams. Each renders one failure scenario to a PNG.
# ---------------------------------------------------------------------------
def _style_3d(ax, title):
    ax.set_title(title, color=INK, fontsize=12, pad=8)
    ax.set_xlabel(r"pitch $\tau_y$", color=INK_SOFT, fontsize=10, labelpad=4)
    ax.set_ylabel(r"yaw $\tau_z$", color=INK_SOFT, fontsize=10, labelpad=4)
    ax.set_zlabel(r"thrust $T$", color=INK_SOFT, fontsize=10, labelpad=4)
    ax.tick_params(colors=INK_SOFT, labelsize=8)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor(SURFACE)
        axis.pane.set_edgecolor(GRID)
        axis.pane.set_alpha(1.0)
    ax.set_xlim(*_LIMITS["x"])
    ax.set_ylim(*_LIMITS["y"])
    ax.set_zlim(*_LIMITS["z"])
    ax.set_box_aspect(
        (
            _LIMITS["x"][1] - _LIMITS["x"][0],
            _LIMITS["y"][1] - _LIMITS["y"][0],
            _LIMITS["z"][1] - _LIMITS["z"][0],
        )
    )
    ax.view_init(elev=20, azim=-60)


def plot_scenario_3d(scenario, verdict, demo=None, trim=DEFAULT_TRIM):
    """Render the 3-D attainable command set for one scenario to a PNG."""
    faces = attainable_set_faces(scenario["mask"])
    fig = plt.figure(figsize=(6.0, 5.2))
    ax = fig.add_subplot(111, projection="3d")

    set_color = BLUE if verdict["controllable"] else ORANGE
    collection = Poly3DCollection(
        faces, facecolor=set_color, edgecolor=INK_SOFT, linewidths=0.35, alpha=0.30
    )
    collection.set_zsort("average")
    ax.add_collection3d(collection)

    trim_color = GREEN if verdict["controllable"] else RED
    ax.scatter(
        *trim,
        color=trim_color,
        s=45,
        depthshade=False,
        edgecolor=INK,
        linewidth=0.5,
        zorder=6,
        label=f"hover trim {tuple(int(v) for v in trim)}",
    )

    if demo is not None:
        requested = np.asarray(demo["requested"], dtype=float)
        delivered = np.asarray(demo["delivered"], dtype=float)
        ax.plot(*zip(requested, delivered), color=INK_SOFT, lw=1.4, zorder=5)
        ax.scatter(
            *requested,
            color=RED,
            marker="X",
            s=55,
            depthshade=False,
            zorder=7,
            label="requested (outside)",
        )
        ax.scatter(
            *delivered,
            color=GREEN,
            s=40,
            depthshade=False,
            zorder=7,
            label="delivered (on surface)",
        )

    _style_3d(ax, scenario["title"])
    ax.legend(
        loc="upper left",
        fontsize=7.5,
        framealpha=0.95,
        edgecolor=GRID,
        facecolor=SURFACE,
        labelcolor=INK,
    )

    path = os.path.join(DIAGRAMS, scenario["slug"] + ".png")
    fig.savefig(path, format="png", dpi=110, facecolor=SURFACE, bbox_inches="tight")
    plt.close(fig)

    return path


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


def mask(failed):
    """Return an eight-motor active mask with ``failed`` indices disabled."""
    return [0 if motor in failed else 1 for motor in range(N_MOTORS)]


hover = study((0, 0, 100))
feasible = study((-40, 40, 100))
saturating = study((80, 100, 100))
aggressive = study((0, 150, 100))

# The failure scenarios documented in the controllability study. The nominal
# case carries the saturation demo; the rest illustrate how losing motors
# reshapes the attainable set.
SCENARIOS = [
    {
        "label": "nominal (8 motors)",
        "title": "Nominal: all 8 motors",
        "slug": "scenario_nominal",
        "mask": mask([]),
        "demo": {
            "requested": saturating["command"],
            "delivered": tuple(saturating["achieved"]),
        },
    },
    {
        "label": "1 out - outer (motor 0)",
        "title": "1 motor out: outer arm (motor 0)",
        "slug": "scenario_outer_single",
        "mask": mask([0]),
    },
    {
        "label": "1 out - inner (motor 1)",
        "title": "1 motor out: inner arm (motor 1)",
        "slug": "scenario_inner_single",
        "mask": mask([1]),
    },
    {
        "label": "2 out - adjacent (motors 0, 1)",
        "title": "2 motors out: adjacent pair (0, 1)",
        "slug": "scenario_adjacent_pair",
        "mask": mask([0, 1]),
    },
    {
        "label": "2 out - opposite (motors 0, 6)",
        "title": "2 motors out: opposite pair (0, 6)",
        "slug": "scenario_opposite_pair",
        "mask": mask([0, 6]),
    },
    {
        "label": "4 out - one r_z row (motors 0-3)",
        "title": "4 motors out: an entire r_z row (0-3)",
        "slug": "scenario_row_out",
        "mask": mask([0, 1, 2, 3]),
    },
]

os.makedirs(DIAGRAMS, exist_ok=True)
NOMINAL_VOLUME = zonotope_volume(attainable_generators())

results = []
for scenario in SCENARIOS:
    verdict = controllability(scenario["mask"])
    figure = plot_scenario_3d(scenario, verdict, demo=scenario.get("demo"))
    results.append({"scenario": scenario, "verdict": verdict, "figure": figure})


# ---------------------------------------------------------------------------
# Symbolic building blocks.
# ---------------------------------------------------------------------------
_, _, tau_i = single_motor_torque()
A = allocation_matrix_from_equations()

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


def verdict_text(verdict):
    """Turn a controllability dict into a short human verdict."""
    if verdict["rank"] < 3:
        return f"**no** - rank {verdict['rank']}, axes coupled"
    if verdict["margin"] <= 1e-6:
        return "**no** - trim on the boundary"
    return "yes"


def controllability_row(result):
    """Render one scenario's controllability metrics as a Markdown table row."""
    scenario = result["scenario"]
    verdict = result["verdict"]
    share = 100 * verdict["volume"] / NOMINAL_VOLUME

    return (
        f"| {scenario['label']} | {verdict['n_active']} | {verdict['rank']} | "
        f"{verdict['volume']:,.0f} | {share:.0f}% | {verdict['margin']:.1f} | "
        f"{verdict_text(verdict)} |\n"
    )


def diagram_grid(entries, columns=2):
    """Lay diagrams out in an HTML grid so the README stays readable."""
    html = "<table>\n"
    for index, result in enumerate(entries):
        if index % columns == 0:
            html += "<tr>\n"
        html += (
            f'<td align="center"><img src="{result["figure"]}" width="100%"><br>'
            f"<sub>{result['scenario']['label']}</sub></td>\n"
        )
        if index % columns == columns - 1 or index == len(entries) - 1:
            html += "</tr>\n"

    return html + "</table>\n"


nominal_result = results[0]
failure_results = results[1:]


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
the problem. The set of commands the motors can actually deliver is a **polytope**
in the three-axis command space, and allocation becomes a small **quadratic
program (QP)** that projects the desired command onto that polytope. A request
inside the polytope is delivered exactly; a request outside it is met by the
closest command the motors can produce. Because the polytope is an explicit
object we can also ask whether the vehicle is still **controllable** after one or
more motors fail.

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
    "is a zonotope, this set is a bounded convex polytope living in the full "
    "$(\\tau_y, \\tau_z, T)$ command space:\n"
)
readme += latex_block(
    r"\mathcal{A} = \{\, A\,s \;:\; 0 \le s \le s_{\max} \,\} \subset \mathbb{R}^3"
)
readme += (
    "Its faces and volume are computed straight from the motor generators "
    "``g_k = s_max * A[:, k]`` by `approachthree.model`, the same code the "
    f"allocator trusts. With all eight motors it has {len(attainable_set_faces())} "
    f"faces and a volume of {NOMINAL_VOLUME:,.0f}. The green point is the hover "
    "trim; the red request below sits outside the set, so the allocator delivers "
    "the nearest point on the surface with motors driven to saturation:\n"
)
readme += f"\n![Nominal attainable command set]({nominal_result['figure']})\n\n"

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

readme += """
## Controllability under motor failure

Losing a motor removes its column from the allocation matrix, which shrinks the
attainable polytope. Because the polytope is explicit, each failure can be
scored for controllability about the hover trim on three criteria:

- **rank** of the active allocation matrix -- all three command axes can be
  actuated independently only when it is 3;
- **volume** of the attainable set -- overall command authority, reported
  relative to the nominal set;
- **hover margin** -- the signed distance from the hover trim to the nearest
  face of the polytope, using the zonotope support function. A positive margin
  means the vehicle can still make a restoring command in every direction.
"""
readme += latex_block(
    r"\text{margin} = \min_{n}\Big( h_{\mathcal{A}}(n) - n^{\top} u_{\text{trim}} "
    r"\Big), \qquad h_{\mathcal{A}}(n) = \sum_k \max\!\big(0,\ n^{\top} g_k\big)"
)
readme += (
    f"The hover trim used here is "
    f"$({DEFAULT_TRIM[0]:g}, {DEFAULT_TRIM[1]:g}, {DEFAULT_TRIM[2]:g})$. "
    "Every row is produced by `approachthree.model.controllability`:\n"
)
readme += "\n"
readme += (
    "| scenario | motors | rank | attainable volume | vs nominal | "
    "hover margin | controllable |\n"
)
readme += "| --- | --- | --- | --- | --- | --- | --- |\n"
for result in results:
    readme += controllability_row(result)

readme += (
    "\nThe geometry tells a clear story. Losing an **outer** arm costs more "
    "authority than an **inner** one. Losing an **adjacent** pair drops the hover "
    "trim exactly onto the boundary -- the vehicle can still hold hover but has "
    "no margin to correct in some direction -- while losing an **opposite** pair "
    "keeps a healthy margin. Losing a whole $r_z$ row leaves the pitch-torque and "
    "thrust rows identical, so the allocation matrix falls to rank 2 and those "
    "axes can no longer be commanded independently: the attainable set collapses "
    "to a flat, zero-volume sheet.\n\n"
)
readme += "A 3-D attainable command set is drawn for each scenario, all to the same scale so the shrinkage is visible:\n\n"
readme += diagram_grid(failure_results)

with open("README.md", "w") as f:
    f.write(readme)
