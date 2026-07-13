"""Continuous-time small-signal analysis for approach one's sim loop."""

from __future__ import annotations

import numpy as np

from approachone.allocate import allocate
from approachone.sim import sim
from common.continuous import (
    StateSpaceAnalysis,
    analyze_state_space,
    bode_channel_summary,
    finite_difference,
    save_bode_plot,
)

C = 1.0
DT = 1.0e-3
TRIM_COMMAND = np.array([0.0, 0.0, 100.0])
TRIM_STATE = np.array([0.0, 0.0, 10.0])


def _loop_output(command: np.ndarray, state: np.ndarray = TRIM_STATE) -> np.ndarray:
    motors = allocate(*command, C)
    return np.array(sim(*motors, *state, C, DT), dtype=float)


def _allocation_jacobian(step: float = 1.0e-5) -> np.ndarray:
    return finite_difference(
        lambda command: np.array(allocate(*command, C)), TRIM_COMMAND, step
    )


def _command_jacobian(step: float = 1.0e-5) -> np.ndarray:
    jac = finite_difference(lambda command: _loop_output(command), TRIM_COMMAND, step)
    jac[0:2, :] /= DT
    return jac


def analyze(
    p_gains: tuple[float, float, float] = (1.0, -1.0e-3, 10.0)
) -> StateSpaceAnalysis:
    """Linearize the generated approach-one stack around hover."""
    return analyze_state_space(
        trim_command=TRIM_COMMAND,
        trim_state=TRIM_STATE,
        motor_trim=np.array(allocate(*TRIM_COMMAND, C), dtype=float),
        allocation_jacobian=_allocation_jacobian(),
        command_jacobian=_command_jacobian(),
        p_gains=p_gains,
        robustness_notes=(
            "The hover linearization is nearly diagonal: each high-level command primarily moves its matching output.",
            "Yaw allocation uses sqrt(abs(tau_z)), so the exact derivative at zero yaw command is singular; the table reports a small-signal finite-difference slope.",
            "The allocator hides individual motors, so this analysis cannot certify motor-out robustness.",
            "Bode margins are meaningful per channel near hover, but saturation and square-root clipping are outside the linear model.",
        ),
    )


if __name__ == "__main__":
    result = analyze()
    print("command Jacobian:\n", result.command_jacobian)
    print("closed-loop eigenvalues:", result.loop_eigenvalues)
