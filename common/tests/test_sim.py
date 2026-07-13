from pathlib import Path

from common.sim import sim_expressions, sim_source


def test_sim_source_defines_the_step_function():
    source = sim_source()

    assert source.startswith("# This file is auto-generated")
    assert "def sim(" in source


def test_sim_expressions_return_three_outputs():
    args, exprs = sim_expressions()

    # pitch rate, yaw rate, airspeed
    assert len(exprs) == 3
    # eight motor speeds plus three states, C and dt
    assert len(args) == 13


def test_every_approach_sim_matches_the_single_source():
    expected = sim_source()
    repo_root = Path(__file__).resolve().parents[2]

    for approach in ("approachone", "approachtwo", "approachthree"):
        generated = (repo_root / approach / "sim.py").read_text()
        assert generated == expected, f"{approach}/sim.py is stale; rerun gen_sim.py"
