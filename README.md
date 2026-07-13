# control-allocation

This repository tells the control-allocation story in three increasingly realistic
steps. Each approach keeps the same high-level controller request -- pitch
torque, yaw torque, and total thrust -- but changes how much of the vehicle and
hardware reality the allocator admits into the problem.

The point of the progression is not just to compare three implementations. It is
to show why each extra layer of math becomes necessary: start with a simple model
that is easy to reason about, replace hand structure with geometry-derived
linear algebra, then add physical limits so the allocator never promises a
command the motors cannot actually produce.

## The story

| Stage | Main idea | Why it is useful | What motivates the next step |
| --- | --- | --- | --- |
| [Approach one](approachone/) | Group the eight motors into four quadrants. | Makes the pitch/yaw/thrust mapping visible and easy to inspect by hand. | The quadrant shortcut hides individual motor authority, failures, and asymmetric behavior. |
| [Approach two](approachtwo/) | Build one allocation-matrix column per motor and solve with a damped pseudoinverse. | Uses the actual per-motor geometry and handles motor-out cases by changing the active columns. | The solution is still unconstrained, so it can request negative or over-limit squared speeds. |
| [Approach three](approachthree/) | Solve a bounded quadratic program over the attainable command set. | Treats saturation and motor failures as part of the allocation problem and exposes controllability margins. | This is the first approach in the series that knows the physical command set. |

## Capability comparison

| Capability | Approach one | Approach two | Approach three |
| --- | --- | --- | --- |
| Shared symbolic rigid-body model | yes | yes | yes |
| Per-motor allocation authority | no, motors are grouped | yes | yes |
| Motor-out handling | not natural | yes, via active columns | yes, via active columns and attainable-set shrinkage |
| Maximum motor-speed limits | no | no | yes |
| Saturation reporting | no | no | yes |
| Controllability analysis | no | limited matrix intuition | yes, via polytope volume and hover margin |

## Reading order

1. Start with [approach one](approachone/) to see the basic physics and the
   deliberately simple quadrant allocator.
2. Move to [approach two](approachtwo/) to see the hand grouping replaced by a
   per-motor allocation matrix and manually assembled pseudoinverse.
3. Finish with [approach three](approachthree/) to see why actuator limits turn
   allocation into a constrained optimization problem and how the attainable
   command polytope explains saturation and failure cases.

## To run

```bash
pip install -e .
```

## Regenerating generated artifacts

Run every `gen_*.py` script, including code, README, and diagram generators, with:

```bash
python run_generators.py
```

Diagram generators write README-linked PNG artifacts under each approach directory (for example `approachone/assets/continuous_bode.png`). These generated images are explicitly unignored so they are added to git with the README updates that reference them.

## Pre-commit

```bash
pre-commit install
```
