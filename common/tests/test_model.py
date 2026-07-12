import sympy as sp

from common.model import (
    INERTIA_SYMBOLS,
    angular_acceleration,
    default_inertia_substitutions,
    inertia_matrix,
)


def test_inertia_matrix_tracks_full_axis_coupling_before_defaults():
    I = inertia_matrix()

    assert I.shape == (3, 3)
    assert set(I) == set(INERTIA_SYMBOLS)


def test_default_inertia_substitutions_keep_unit_diagonal_only():
    I = inertia_matrix().subs(default_inertia_substitutions())

    assert I == sp.eye(3)


def test_angular_acceleration_uses_full_inertia_before_defaults():
    tau = sp.Matrix([0, sp.Symbol("tau_y"), sp.Symbol("tau_z")])
    omega_dot = angular_acceleration(tau)

    assert any(symbol in omega_dot[0].free_symbols for symbol in INERTIA_SYMBOLS)
    assert omega_dot.subs(default_inertia_substitutions()) == tau
