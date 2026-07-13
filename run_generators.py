"""Run every generated-code script in the repository."""

import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent


for script in sorted(ROOT.rglob("gen_*.py")):
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT)
    subprocess.run([sys.executable, script.name], cwd=script.parent, env=env, check=True)
