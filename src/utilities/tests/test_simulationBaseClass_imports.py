# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""Check Matplotlib import behavior and first plotting use in a fresh process."""

import os
from pathlib import Path
import subprocess
import sys
import textwrap

import Basilisk
from Basilisk.utilities import SimulationBaseClass


def test_matplotlib_imports_and_first_diagram():
    """Defer imports on 3.15 while preserving first-use plotting on all versions."""
    script = textwrap.dedent("""
        import sys

        sys.path.insert(0, sys.argv[1])
        from Basilisk import utilities
        utilities.__path__.insert(0, sys.argv[2])
        from Basilisk.utilities import SimulationBaseClass

        eager = sys.version_info < (3, 15)
        assert ("matplotlib.pyplot" in sys.modules) == eager
        assert ("matplotlib.patches" in sys.modules) == eager

        sim = SimulationBaseClass.SimBaseClass()
        process = sim.CreateNewProcess("testProcess")
        time_step = 1_000_000_000  # [ns]
        process.addTask(sim.CreateNewTask("testTask", time_step))
        figure = sim.ShowExecutionFigure(show_plots=False)
        figure.canvas.draw()
        assert "matplotlib.pyplot" in sys.modules
        assert figure.axes
    """)
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            script,
            str(Path(Basilisk.__path__[0]).resolve().parent),
            str(Path(SimulationBaseClass.__file__).resolve().parent),
        ],
        env=dict(os.environ, MPLBACKEND="Agg", PYTHON_LAZY_IMPORTS="normal"),
        capture_output=True,
        text=True,
        timeout=60,  # [s]
    )
    assert result.returncode == 0, result.stdout + result.stderr
