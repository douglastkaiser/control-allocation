"""Continuous-time small-signal analysis for approach one's sim loop.

The generated allocator and simulator are nonlinear because commands are turned
into motor speeds with square roots and the simulator turns motor speeds back
into thrust with squares.  This module keeps the analysis tied to that runnable
stack by linearizing the actual ``allocate -> sim`` path about the hover trim.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from approachone.allocate import allocate
from approachone.sim import sim

C = 1.0
DT = 1.0e-3
TRIM_COMMAND = np.array([0.0, 0.0, 100.0])
TRIM_STATE = np.array([0.0, 0.0, 10.0])
OUTPUT_NAMES = ("pitch rate q", "yaw rate r", "airspeed u")
COMMAND_NAMES = ("pitch torque tau_y", "yaw torque tau_z", "total thrust T")


@dataclass(frozen=True)
class ContinuousAnalysis:
    """Numerical facts used by the generated README."""

    trim_command: np.ndarray
    trim_state: np.ndarray
    motor_trim: np.ndarray
    allocation_jacobian: np.ndarray
    command_jacobian: np.ndarray
    plant_A: np.ndarray
    plant_B: np.ndarray
    plant_C: np.ndarray
    plant_D: np.ndarray
    loop_A: np.ndarray
    loop_eigenvalues: np.ndarray
    dc_gain: np.ndarray
    controllability_rank: int
    observability_rank: int
    stability_verdict: str
    robustness_notes: tuple[str, ...]


def _loop_output(command: np.ndarray, state: np.ndarray = TRIM_STATE) -> np.ndarray:
    """Run one generated approach-one allocation/simulation step."""
    motors = allocate(*command, C)
    return np.array(sim(*motors, *state, C, DT), dtype=float)


def _finite_difference(func, x0: np.ndarray, step: float = 1.0e-5) -> np.ndarray:
    """Central finite-difference Jacobian for small dense vectors."""
    x0 = np.asarray(x0, dtype=float)
    y0 = np.asarray(func(x0), dtype=float)
    jac = np.zeros((y0.size, x0.size))
    for col in range(x0.size):
        dx = np.zeros_like(x0)
        dx[col] = step
        jac[:, col] = (np.asarray(func(x0 + dx)) - np.asarray(func(x0 - dx))) / (
            2 * step
        )
    return jac


def _allocation_jacobian(step: float = 1.0e-5) -> np.ndarray:
    return _finite_difference(
        lambda command: np.array(allocate(*command, C)), TRIM_COMMAND, step
    )


def _command_jacobian(step: float = 1.0e-5) -> np.ndarray:
    """Return d(output)/d(command) for one sim step, converted to continuous units.

    Pitch and yaw are integrated by ``sim`` with ``dt``, so their rows are divided
    by ``DT`` to recover the continuous command-to-derivative gain.  Airspeed is a
    static output of the current thrust command, so its row is left in output per
    command units.
    """
    jac = _finite_difference(lambda command: _loop_output(command), TRIM_COMMAND, step)
    jac[0:2, :] /= DT
    return jac


def analyze(
    p_gains: tuple[float, float, float] = (1.0, -1.0e-3, 10.0)
) -> ContinuousAnalysis:
    """Linearize the generated approach-one stack around hover."""
    motor_trim = np.array(allocate(*TRIM_COMMAND, C), dtype=float)
    allocation_jac = _allocation_jacobian()
    command_jac = _command_jacobian()

    # Minimal continuous plant used for classical loop reasoning.  The rates are
    # integrators driven by command torque; airspeed is represented as a first
    # order lag whose DC gain is the static thrust-to-airspeed slope from sim.
    A = np.diag([0.0, 0.0, -1.0])
    B = np.zeros((3, 3))
    B[0, :] = command_jac[0, :]
    B[1, :] = command_jac[1, :]
    B[2, :] = command_jac[2, :]
    Cmat = np.eye(3)
    D = np.zeros((3, 3))

    K = np.diag(p_gains)
    loop_A = A - B @ K
    eig = np.linalg.eigvals(loop_A)
    ctrb = np.hstack([B, A @ B, A @ A @ B])
    obsv = np.vstack([Cmat, Cmat @ A, Cmat @ A @ A])
    stable = bool(np.all(np.real(eig) < -1.0e-9))

    notes = (
        "The hover linearization is nearly diagonal: each high-level command primarily moves its matching output.",
        "Yaw allocation uses sqrt(abs(tau_z)), so the exact derivative at zero yaw command is singular; the table reports a small-signal finite-difference slope.",
        "The allocator hides individual motors, so this analysis cannot certify motor-out robustness.",
        "Bode margins are meaningful per channel near hover, but saturation and square-root clipping are outside the linear model.",
    )
    return ContinuousAnalysis(
        trim_command=TRIM_COMMAND.copy(),
        trim_state=TRIM_STATE.copy(),
        motor_trim=motor_trim,
        allocation_jacobian=allocation_jac,
        command_jacobian=command_jac,
        plant_A=A,
        plant_B=B,
        plant_C=Cmat,
        plant_D=D,
        loop_A=loop_A,
        loop_eigenvalues=eig,
        dc_gain=command_jac,
        controllability_rank=int(np.linalg.matrix_rank(ctrb)),
        observability_rank=int(np.linalg.matrix_rank(obsv)),
        stability_verdict=(
            "stable for the documented sign-aware diagonal proportional gains"
            if stable
            else "not stable for the documented gains"
        ),
        robustness_notes=notes,
    )


def bode_channel_summary(analysis: ContinuousAnalysis) -> tuple[tuple[str, str], ...]:
    """Return compact per-axis transfer functions for README text."""
    gains = np.diag(analysis.plant_B)
    return (
        ("pitch", f"{gains[0]:.3g} / s"),
        ("yaw", f"{gains[1]:.3g} / s"),
        ("airspeed", f"{gains[2]:.3g} / (s + 1)"),
    )


if __name__ == "__main__":
    result = analyze()
    print("command Jacobian:\n", result.command_jacobian)
    print("closed-loop eigenvalues:", result.loop_eigenvalues)
