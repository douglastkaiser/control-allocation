"""Shared continuous-time analysis helpers for generated allocation stacks."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import matplotlib.pyplot as plt
import numpy as np


@dataclass(frozen=True)
class StateSpaceAnalysis:
    """Numerical state-space facts used by approach README generators."""

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


def finite_difference(
    func: Callable[[np.ndarray], np.ndarray], x0, step: float = 1.0e-5
):
    """Return a central finite-difference Jacobian for a small dense vector map."""
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


def analyze_state_space(
    *,
    trim_command,
    trim_state,
    motor_trim,
    allocation_jacobian,
    command_jacobian,
    p_gains,
    robustness_notes,
) -> StateSpaceAnalysis:
    """Build a compact continuous plant from stack-level command gains."""
    trim_command = np.asarray(trim_command, dtype=float)
    trim_state = np.asarray(trim_state, dtype=float)
    motor_trim = np.asarray(motor_trim, dtype=float)
    allocation_jacobian = np.asarray(allocation_jacobian, dtype=float)
    command_jacobian = np.asarray(command_jacobian, dtype=float)

    # Pitch and yaw rates are integrators.  Airspeed is represented as a first
    # order lag with the stack-derived static thrust-to-airspeed gain.
    plant_A = np.diag([0.0, 0.0, -1.0])
    plant_B = command_jacobian.copy()
    plant_C = np.eye(3)
    plant_D = np.zeros((3, 3))

    loop_A = plant_A - plant_B @ np.diag(p_gains)
    loop_eigenvalues = np.linalg.eigvals(loop_A)
    ctrb = np.hstack([plant_B, plant_A @ plant_B, plant_A @ plant_A @ plant_B])
    obsv = np.vstack([plant_C, plant_C @ plant_A, plant_C @ plant_A @ plant_A])
    stable = bool(np.all(np.real(loop_eigenvalues) < -1.0e-9))

    return StateSpaceAnalysis(
        trim_command=trim_command,
        trim_state=trim_state,
        motor_trim=motor_trim,
        allocation_jacobian=allocation_jacobian,
        command_jacobian=command_jacobian,
        plant_A=plant_A,
        plant_B=plant_B,
        plant_C=plant_C,
        plant_D=plant_D,
        loop_A=loop_A,
        loop_eigenvalues=loop_eigenvalues,
        dc_gain=command_jacobian,
        controllability_rank=int(np.linalg.matrix_rank(ctrb)),
        observability_rank=int(np.linalg.matrix_rank(obsv)),
        stability_verdict=(
            "stable for the documented diagonal proportional gains"
            if stable
            else "not stable for the documented gains"
        ),
        robustness_notes=tuple(robustness_notes),
    )


def bode_channel_summary(analysis: StateSpaceAnalysis) -> tuple[tuple[str, str], ...]:
    """Return compact per-axis transfer functions for README text."""
    gains = np.diag(analysis.plant_B)
    return (
        ("pitch", f"{gains[0]:.3g} / s"),
        ("yaw", f"{gains[1]:.3g} / s"),
        ("airspeed", f"{gains[2]:.3g} / (s + 1)"),
    )


def save_bode_plot(analysis: StateSpaceAnalysis, output_path, frequencies=None):
    """Save a simple magnitude/phase Bode plot for the three diagonal channels."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    omega = np.asarray(
        frequencies if frequencies is not None else np.logspace(-2, 2, 300)
    )
    gains = np.diag(analysis.plant_B)
    channels = (
        ("pitch", gains[0], 1j * omega),
        ("yaw", gains[1], 1j * omega),
        ("airspeed", gains[2], 1j * omega + 1),
    )

    fig, (mag_ax, phase_ax) = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    for name, gain, denominator in channels:
        response = gain / denominator
        mag_ax.semilogx(
            omega, 20 * np.log10(np.maximum(np.abs(response), 1.0e-12)), label=name
        )
        phase_ax.semilogx(omega, np.angle(response, deg=True), label=name)
    mag_ax.set_ylabel("magnitude [dB]")
    phase_ax.set_ylabel("phase [deg]")
    phase_ax.set_xlabel("frequency [rad/s]")
    mag_ax.grid(True, which="both")
    phase_ax.grid(True, which="both")
    mag_ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path
