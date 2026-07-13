"""Continuous-time small-signal analysis for approach three's bounded QP allocator."""

from __future__ import annotations

import numpy as np

from approachthree.model import (
    DEFAULT_S_MAX,
    DEFAULT_TRIM,
    achieved_command,
    allocated_motor_speeds,
    allocated_squared_speeds,
    controllability,
)
from approachthree.sim import sim
from common.continuous import (
    StateSpaceAnalysis,
    analyze_state_space,
    bode_channel_summary,
    finite_difference,
    save_bode_plot,
)

C = 1.0
DT = 1.0e-3
TRIM_COMMAND = np.array(DEFAULT_TRIM, dtype=float)
TRIM_STATE = np.array([0.0, 0.0, 10.0])


def _loop_output(command: np.ndarray, state: np.ndarray = TRIM_STATE) -> np.ndarray:
    motors = allocated_motor_speeds(command, C=C)
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


def saturation_margin(command=TRIM_COMMAND) -> float:
    """Return distance in squared-speed units from a command allocation to a bound."""
    s = np.asarray(allocated_squared_speeds(command, C=C), dtype=float)
    return float(np.min(np.minimum(s, DEFAULT_S_MAX - s)))


def analyze(
    p_gains: tuple[float, float, float] = (1.0, 1.0, 10.0)
) -> StateSpaceAnalysis:
    """Linearize the generated approach-three stack around the hover trim."""
    verdict = controllability(trim=DEFAULT_TRIM)
    margin = saturation_margin(TRIM_COMMAND)
    delivered_trim = achieved_command(TRIM_COMMAND, C=C)
    return analyze_state_space(
        trim_command=TRIM_COMMAND,
        trim_state=TRIM_STATE,
        motor_trim=np.array(allocated_motor_speeds(TRIM_COMMAND, C=C), dtype=float),
        allocation_jacobian=_allocation_jacobian(),
        command_jacobian=_command_jacobian(),
        p_gains=p_gains,
        robustness_notes=(
            f"The hover trim is inside the attainable set with polytope margin {verdict['margin']:.1f}.",
            f"The QP hover allocation is {margin:.2f} squared-speed units away from the nearest motor bound.",
            f"The delivered trim command is ({delivered_trim[0]:.3g}, {delivered_trim[1]:.3g}, {delivered_trim[2]:.3g}) after regularisation.",
            "The local Bode and state-space view applies while the active bound set is unchanged; outside that region the QP projection is piecewise linear.",
            "Unlike approach two, saturation is part of the allocator, so large-signal robustness is studied with attainable-set margin rather than only this hover linearization.",
        ),
    )


if __name__ == "__main__":
    result = analyze()
    print("command Jacobian:\n", result.command_jacobian)
    print("closed-loop eigenvalues:", result.loop_eigenvalues)
