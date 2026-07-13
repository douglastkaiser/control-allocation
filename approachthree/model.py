"""Saturation-aware QP control allocation for approach three.

Approaches one and two both allocate as if every motor could spin arbitrarily
fast: they solve an unconstrained problem and only clamp the final square root so
callers never see an imaginary speed. That silently distorts the commanded torque
whenever a motor would need to exceed its physical limit.

Approach three keeps the same command interface -- pitch torque, yaw torque and
total thrust -- but treats the per-motor saturation limits as first-class
constraints. Feasible squared speeds live inside a box ``0 <= s <= s_max``. The
image of that box under the allocation matrix ``A`` is the *attainable command
set*: a polytope (a zonotope) of every torque/thrust triple the motors can
actually produce. Allocation becomes a bound-constrained quadratic program that
projects the desired command onto this polytope, so a request that lies outside it
is met by the closest achievable command instead of an infeasible one.

The QP is solved with a self-contained primal active-set method -- we do not call
out to ``cvxpy``/``scipy`` -- in the same spirit as approach two assembling its own
pseudoinverse rather than calling ``np.linalg.pinv``.
"""

import itertools
import math

import numpy as np

from common.geometry import MOTOR_R_Y, MOTOR_R_Z, N_MOTORS


# Per-motor saturation limit on squared speed ``s = w**2`` (so ``w_max = 5``).
# With unit thrust coefficient ``C`` every motor can make at most ``s_max`` of
# force, i.e. a total thrust of ``N_MOTORS * s_max``.
DEFAULT_S_MAX = 25.0

# Command-tracking weights for (tau_y, tau_z, thrust). Attitude authority is
# prioritised over thrust: when the vehicle saturates it holds the commanded
# moments and lets total thrust sag, which is the usual flight-control choice.
DEFAULT_WEIGHTS = (10.0, 10.0, 1.0)

# Tikhonov effort weight. Like approach two's damping it keeps the solution
# unique in the null space of ``A`` (minimum control effort) and the Hessian
# strictly positive definite, without meaningfully perturbing tracking.
DEFAULT_REG = 1e-6

# Hover trim command used for controllability analysis: zero moment while
# carrying a representative thrust. A vehicle is controllable about this trim
# only if it can also make restoring moments in every direction from here.
DEFAULT_TRIM = (0.0, 0.0, 100.0)

_TOL = 1e-9


def create_A(motors_active=None, C=1):
    """Return the command-to-squared-speed allocation matrix.

    Rows are pitch torque, yaw torque and total thrust; columns are motors. This
    is the same map used by approach two: thrust acts along body x, so
    ``tau_y = r_z * F`` and ``tau_z = -r_y * F``. A failed motor zeroes its
    column via the ``motors_active`` mask.
    """
    if motors_active is None:
        motors_active = (1,) * N_MOTORS

    active = np.asarray(motors_active, dtype=float)
    if active.shape != (N_MOTORS,):
        raise ValueError(f"motors_active must contain {N_MOTORS} entries")

    A = np.array(
        [
            MOTOR_R_Z,
            [-r_y for r_y in MOTOR_R_Y],
            [1] * N_MOTORS,
        ],
        dtype=float,
    )

    return C * A * active


def bounds(motors_active=None, s_max=DEFAULT_S_MAX):
    """Return the lower/upper squared-speed bounds for the box constraint.

    Every motor is bounded below by zero (a propeller cannot push backwards) and
    above by ``s_max``. A failed motor is pinned to zero by collapsing its upper
    bound onto its lower bound.
    """
    if motors_active is None:
        motors_active = (1,) * N_MOTORS

    lower = np.zeros(N_MOTORS)
    upper = np.array(
        [s_max if active else 0.0 for active in motors_active], dtype=float
    )

    return lower, upper


def bounded_least_squares(
    A, u, lower, upper, weights=None, reg=DEFAULT_REG, max_iter=200
):
    """Solve ``min (1/2)||A s - u||_W^2 + (reg/2)||s||^2`` s.t. ``lower <= s <= upper``.

    This is a strictly convex quadratic program (the regularisation makes the
    Hessian ``H = A^T W A + reg I`` positive definite), solved with a primal
    active-set iteration: variables are held at a bound while the remaining free
    variables are driven to the equality-constrained minimum, releasing a bound
    only when its KKT multiplier says the objective can still be reduced. For a
    strictly convex QP the KKT conditions are necessary and sufficient, so the
    fixed point this returns is the global optimum.
    """
    A = np.asarray(A, dtype=float)
    u = np.asarray(u, dtype=float).reshape(-1)
    lower = np.asarray(lower, dtype=float).reshape(-1)
    upper = np.asarray(upper, dtype=float).reshape(-1)
    n = A.shape[1]

    W = np.eye(A.shape[0]) if weights is None else np.diag(np.asarray(weights, float))
    H = A.T @ W @ A + reg * np.eye(n)
    c = -A.T @ W @ u

    # Feasible start: the clipped unconstrained minimiser.
    s = np.clip(np.linalg.solve(H, -c), lower, upper)
    at_lower = s <= lower + _TOL
    at_upper = s >= upper - _TOL
    s = np.where(at_lower, lower, s)
    s = np.where(at_upper, upper, s)

    for _ in range(max_iter):
        free = ~(at_lower | at_upper)
        step = np.zeros(n)
        if free.any():
            fixed = ~free
            rhs = -(c[free] + H[np.ix_(free, fixed)] @ s[fixed])
            target = np.linalg.solve(H[np.ix_(free, free)], rhs)
            step[free] = target - s[free]

        if np.linalg.norm(step) <= _TOL:
            # No interior move left: consult the KKT multipliers (the gradient)
            # to see whether releasing a clamped motor could lower the cost.
            gradient = H @ s + c
            release, worst = -1, -_TOL
            for i in range(n):
                if lower[i] >= upper[i]:
                    continue  # a failed motor is pinned, never released
                if at_lower[i] and gradient[i] < worst:
                    release, worst = i, gradient[i]
                elif at_upper[i] and -gradient[i] < worst:
                    release, worst = i, -gradient[i]
            if release < 0:
                break  # every multiplier has the right sign: optimal
            at_lower[release] = at_upper[release] = False
            continue

        # Move as far along the free step as the box allows.
        alpha, block = 1.0, -1
        for i in np.where(free)[0]:
            if step[i] > _TOL:
                limit = (upper[i] - s[i]) / step[i]
            elif step[i] < -_TOL:
                limit = (lower[i] - s[i]) / step[i]
            else:
                continue
            if limit < alpha:
                alpha, block = limit, i

        s = np.clip(s + alpha * step, lower, upper)
        if block >= 0 and alpha < 1.0 - _TOL:
            if step[block] > 0:
                at_upper[block], s[block] = True, upper[block]
            else:
                at_lower[block], s[block] = True, lower[block]

    return np.clip(s, lower, upper)


def allocated_squared_speeds(
    u,
    motors_active=None,
    C=1,
    s_max=DEFAULT_S_MAX,
    weights=DEFAULT_WEIGHTS,
    reg=DEFAULT_REG,
):
    """Allocate a command to saturation-respecting squared motor speeds."""
    A = create_A(motors_active, C)
    lower, upper = bounds(motors_active, s_max)

    return bounded_least_squares(A, u, lower, upper, weights, reg)


def allocated_motor_speeds(
    u,
    motors_active=None,
    C=1,
    s_max=DEFAULT_S_MAX,
    weights=DEFAULT_WEIGHTS,
    reg=DEFAULT_REG,
):
    """Allocate a command to non-negative, saturation-respecting motor speeds."""
    s = allocated_squared_speeds(u, motors_active, C, s_max, weights, reg)

    return tuple(math.sqrt(max(float(value), 0)) for value in s)


def achieved_command(u, motors_active=None, C=1, s_max=DEFAULT_S_MAX, **kwargs):
    """Return the command the motors actually produce, ``A @ s``.

    Inside the attainable polytope this equals ``u``; outside it is the polytope
    point closest (in the weighted norm) to ``u``.
    """
    s = allocated_squared_speeds(u, motors_active, C, s_max, **kwargs)

    return create_A(motors_active, C) @ s


def saturated_motors(s, s_max=DEFAULT_S_MAX):
    """Return a boolean mask of motors sitting on a saturation bound."""
    s = np.asarray(s, dtype=float)

    return (s <= _TOL) | (s >= s_max - _TOL)


def convex_hull_2d(points):
    """Return the counter-clockwise convex hull of 2-D points (monotone chain).

    Implemented directly rather than pulling in ``scipy.spatial`` -- the polytope
    geometry is small and belongs to the single source of truth for the docs.
    """
    unique = sorted({(round(float(x), 9), round(float(y), 9)) for x, y in points})
    if len(unique) <= 2:
        return np.array(unique, dtype=float)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)

    upper = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)

    return np.array(lower[:-1] + upper[:-1], dtype=float)


def attainable_moment_set(motors_active=None, C=1, s_max=DEFAULT_S_MAX):
    """Return the polygon of attainable ``(tau_y, tau_z)`` moments.

    The full attainable command set is the image of the squared-speed box under
    ``A`` -- a 3-D zonotope in ``(tau_y, tau_z, thrust)`` space. Its shadow on the
    moment plane is again a zonotope whose vertices are images of the box
    corners, so we hull the corner images to recover the polygon boundary.
    """
    A = create_A(motors_active, C)
    moment_map = A[:2, :]

    if motors_active is None:
        motors_active = (1,) * N_MOTORS
    active = [i for i, is_active in enumerate(motors_active) if is_active]

    corners = []
    for switches in itertools.product((0.0, s_max), repeat=len(active)):
        s = np.zeros(N_MOTORS)
        for motor_index, value in zip(active, switches):
            s[motor_index] = value
        corners.append(moment_map @ s)

    return convex_hull_2d(corners)


# ---------------------------------------------------------------------------
# 3-D attainable command set (a zonotope in (tau_y, tau_z, thrust) space).
#
# The feasible squared-speed box maps to the Minkowski sum of the segments
# ``[0, g_k]`` where ``g_k = s_max * A[:, k]`` is one motor's generator. That
# zonotope is the full attainable command set. We derive its volume, its facets
# and a support-function containment test directly from the generators, so no
# 3-D convex-hull library is needed.
# ---------------------------------------------------------------------------
def attainable_generators(motors_active=None, C=1, s_max=DEFAULT_S_MAX):
    """Return the zonotope generators of the attainable command set."""
    if motors_active is None:
        motors_active = (1,) * N_MOTORS

    A = create_A(motors_active, C)

    return np.array(
        [s_max * A[:, k] for k in range(N_MOTORS) if motors_active[k]], dtype=float
    )


def zonotope_volume(generators):
    """Return the 3-D volume of a zonotope from its generators.

    A 3-D zonotope's volume is the sum over every generator triple of the
    absolute determinant they span -- zero exactly when the generators fail to
    span all three command axes.
    """
    G = np.asarray(generators, dtype=float)

    return sum(
        abs(float(np.linalg.det(np.array([G[i], G[j], G[k]]))))
        for i, j, k in itertools.combinations(range(len(G)), 3)
    )


def support(generators, direction):
    """Support function ``h(n) = max_{x in Z} n . x`` of the zonotope."""
    G = np.asarray(generators, dtype=float)
    d = np.asarray(direction, dtype=float)

    return float(sum(max(0.0, float(row @ d)) for row in G))


def facet_normals(generators, tol=1e-9):
    """Return the distinct outward facet-normal directions of the zonotope.

    Every facet of a 3-D zonotope is normal to a pair of generators, so the
    candidate normals are the pairwise cross products, de-duplicated by
    direction (both orientations are kept as separate facets).
    """
    G = np.asarray(generators, dtype=float)
    directions = []
    for i, j in itertools.combinations(range(len(G)), 2):
        normal = np.cross(G[i], G[j])
        length = np.linalg.norm(normal)
        if length <= tol:
            continue
        normal = normal / length
        if not any(abs(abs(normal @ other) - 1) < 1e-7 for other in directions):
            directions.append(normal)

    return directions


def hover_margin(generators, trim=DEFAULT_TRIM):
    """Signed distance from ``trim`` to the nearest facet of the zonotope.

    Positive means the command sits strictly inside the attainable set, so the
    vehicle can still trim there and make a restoring command in every
    direction. Non-positive means it is on or outside the boundary.
    """
    trim = np.asarray(trim, dtype=float)
    margins = []
    for normal in facet_normals(generators):
        for oriented in (normal, -normal):
            margins.append(support(generators, oriented) - float(oriented @ trim))

    return min(margins) if margins else float("-inf")


def _order_face(points, normal):
    """Order coplanar points into a simple polygon around their centroid."""
    points = np.unique(np.round(points, 9), axis=0)
    if len(points) < 3:
        return points

    normal = np.asarray(normal, dtype=float)
    normal = normal / np.linalg.norm(normal)
    reference = np.array([1.0, 0.0, 0.0])
    if abs(normal @ reference) > 0.9:
        reference = np.array([0.0, 1.0, 0.0])
    u = reference - (reference @ normal) * normal
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)

    centered = points - points.mean(axis=0)
    angles = np.arctan2(centered @ v, centered @ u)

    return points[np.argsort(angles)]


def attainable_set_faces(motors_active=None, C=1, s_max=DEFAULT_S_MAX, tol=1e-7):
    """Return the ordered polygon faces of the 3-D attainable command set."""
    G = attainable_generators(motors_active, C, s_max)
    faces = []
    for normal in facet_normals(G):
        for oriented in (normal, -normal):
            dots = G @ oriented
            anchor = np.zeros(3)
            in_face = []
            for k, dot in enumerate(dots):
                if dot > tol:
                    anchor = anchor + G[k]
                elif abs(dot) <= tol:
                    in_face.append(k)
            if len(in_face) < 2:
                continue

            corners = []
            for switches in itertools.product((0, 1), repeat=len(in_face)):
                point = anchor.copy()
                for switch, k in zip(switches, in_face):
                    if switch:
                        point = point + G[k]
                corners.append(point)

            ordered = _order_face(np.array(corners), oriented)
            if len(ordered) >= 3:
                faces.append(ordered)

    return faces


def controllability(motors_active=None, trim=DEFAULT_TRIM, C=1, s_max=DEFAULT_S_MAX):
    """Assess whether the vehicle can be controlled about ``trim``.

    Reports the rank of the active allocation matrix (all three command axes are
    independently actuatable only at full rank), the attainable-set volume, and
    the hover margin. The vehicle is deemed controllable when the command axes
    are full rank and the trim sits strictly inside the attainable set.
    """
    if motors_active is None:
        motors_active = (1,) * N_MOTORS

    A = create_A(motors_active, C)
    active_columns = A[:, [k for k in range(N_MOTORS) if motors_active[k]]]
    rank = int(np.linalg.matrix_rank(active_columns, tol=1e-9))

    generators = attainable_generators(motors_active, C, s_max)
    volume = zonotope_volume(generators)
    margin = hover_margin(generators, trim) if rank == 3 else 0.0

    return {
        "n_active": int(sum(motors_active)),
        "rank": rank,
        "volume": volume,
        "margin": margin,
        "controllable": rank == 3 and margin > _TOL,
    }
