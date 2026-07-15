"""Approach three model API backed by the generated bounded-QP allocator."""

import itertools

import numpy as np

from common.geometry import N_MOTORS
from approachthree.allocate import (
    DEFAULT_REG,
    DEFAULT_S_MAX,
    DEFAULT_TRIM,
    DEFAULT_WEIGHTS,
    _TOL,
    achieved_command,
    allocated_motor_speeds,
    allocated_squared_speeds,
    allocate,
    bounded_least_squares,
    bounds,
    create_A,
)


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
