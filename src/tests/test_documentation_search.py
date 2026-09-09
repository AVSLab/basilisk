# ISC License
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

"""Check Sphinx integration for the documentation search scorer."""

import ast
from pathlib import Path
import subprocess
import sys

import pytest


SCORER = Path(__file__).resolve().parents[2] / "docs/source/_static/js/search-scorer.js"


# Tool-dependent checks run in docs CI after its dependencies are installed.
@pytest.mark.ciSkip
@pytest.mark.docsIntegration
def test_sphinx_embeds_scorer_from_another_working_directory(tmp_path):
    """Verify the configured scorer path is portable and Sphinx packages it."""
    pytest.importorskip("sphinx")
    conf = SCORER.parents[2] / "conf.py"
    setting = next(
        node.value for node in ast.parse(conf.read_text(encoding="utf8")).body
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "html_search_scorer"
                for target in node.targets)
    )
    path = eval(compile(ast.Expression(setting), str(conf), "eval"), {
        "Path": Path, "__file__": str(conf),
    })
    assert Path(path).is_absolute() and Path(path) == SCORER
    source = tmp_path / "source"
    source.mkdir()
    (source / "conf.py").write_text(f"html_search_scorer = {path!r}\n", encoding="utf8")
    (source / "index.rst").write_text("Search example\n==============\n", encoding="utf8")
    output = tmp_path / "html"
    subprocess.run(
        [sys.executable, "-m", "sphinx", "-b", "html", "-W", "-q", str(source), str(output)],
        cwd=tmp_path, capture_output=True, text=True, check=True,
    )
    language_data = (output / "_static/language_data.js").read_text(encoding="utf8")
    assert SCORER.read_text(encoding="utf8").strip() in language_data
