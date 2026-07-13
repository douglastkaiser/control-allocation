"""Continuous-time small-signal analysis for approach two's per-motor allocator."""

from __future__ import annotations

import numpy as np

from approachtwo.allocate import allocate, allocated_squared_speeds, create_A
from approachtwo.sim import sim
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
    motors = allocate(command, C=C)
    return np.array(sim(*motors, *state, C, DT), dtype=float)


def _allocation_jacobian(step: float = 1.0e-5) -> np.ndarray:
    return finite_difference(
        lambda command: allocated_squared_speeds(command, C=C).flatten(),
        TRIM_COMMAND,
        step,
    )


def _command_jacobian(step: float = 1.0e-5) -> np.ndarray:
    jac = finite_difference(lambda command: _loop_output(command), TRIM_COMMAND, step)
    jac[0:2, :] /= DT
    return jac


def analyze(
    p_gains: tuple[float, float, float] = (1.0, 1.0, 10.0)
) -> StateSpaceAnalysis:
    """Linearize the generated approach-two stack around hover."""
    allocation_matrix = create_A(C=C)
    command_jacobian = _command_jacobian()
    matrix_rank = int(np.linalg.matrix_rank(allocation_matrix))
    return analyze_state_space(
        trim_command=TRIM_COMMAND,
        trim_state=TRIM_STATE,
        motor_trim=np.array(allocate(TRIM_COMMAND, C=C), dtype=float),
        allocation_jacobian=_allocation_jacobian(),
        command_jacobian=command_jacobian,
        p_gains=p_gains,
        robustness_notes=(
            f"The nominal allocation matrix has rank {matrix_rank}, so the three high-level command axes are locally available.",
            "The pseudoinverse keeps per-motor authority visible and can be recomputed for motor-out masks.",
            "The solution is still unconstrained: negative squared speeds are clipped only after allocation, so saturation robustness is not certified here.",
            "Bode and state-space results describe the local hover loop before any clipping or motor limit is hit.",
        ),
    )


if __name__ == "__main__":
    result = analyze()
    print("command Jacobian:\n", result.command_jacobian)
    print("closed-loop eigenvalues:", result.loop_eigenvalues)
