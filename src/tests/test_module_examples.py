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

"""Check example selection without importing Basilisk or executing scenarios."""

import importlib.util
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).resolve().parents[2] / "docs/source/_ext/module_examples.py"
SPEC = importlib.util.spec_from_file_location("module_examples", MODULE_PATH)
EXAMPLES = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXAMPLES)


@pytest.mark.parametrize("source", [
    "from Basilisk.simulation import simpleNav\nx = simpleNav.SimpleNav()",
    "from Basilisk.simulation import simpleNav as nav\nx = nav.SimpleNav()",
    "import Basilisk.simulation.simpleNav as nav\nx = nav.SimpleNav()",
    "import Basilisk.simulation.simpleNav\nx = Basilisk.simulation.simpleNav.SimpleNav()",
    "from Basilisk import simulation as sim\nx = sim.simpleNav.SimpleNav()",
    "from Basilisk.simulation.simpleNav import SimpleNav as Nav\nx = Nav()",
    "def run():\n    from Basilisk.simulation import simpleNav\n    x = simpleNav.SimpleNav()",
])
def test_recognizes_direct_constructors(source):
    """Support qualified names and import aliases in ordinary scenario code."""
    assert EXAMPLES.module_constructions(source) == {("simulation", "simpleNav")}


@pytest.mark.parametrize("source", [
    '# simpleNav.SimpleNav()\n"simpleNav.SimpleNav()"',
    "from Basilisk.simulation import simpleNav",  # An unused import is not usage.
    "from Basilisk.simulation import simpleNav\nsimpleNav.helper()",
    "from other_library import simpleNav\nsimpleNav.SimpleNav()",
    "from Basilisk.simulation import simpleNav\nsimpleNav = other\nsimpleNav.SimpleNav()",
    "from Basilisk.simulation import simpleNav\ndef run(simpleNav):\n    simpleNav.SimpleNav()",
    "def first():\n    from Basilisk.simulation import simpleNav\n"
    "def second():\n    simpleNav.SimpleNav()",
    "from helpers import make_navigation\nmake_navigation()",
])
def test_does_not_infer_mentions_helpers_or_shadowed_names(source):
    """Avoid false links from text, helper calls, unused imports, or lost aliases."""
    assert EXAMPLES.module_constructions(source) == set()


def test_scopes_and_module_packages_are_distinct():
    """A local shadow does not hide valid usage in another function or package."""
    source = (
        "from Basilisk.simulation import simpleNav\n"
        "from Basilisk.fswAlgorithms import mrpPDRust as control\n"
        "def first(simpleNav):\n    simpleNav.SimpleNav()\n"
        "def second():\n    return simpleNav.SimpleNav(), control.mrpPDRust()\n"
        "raise RuntimeError('This source must not be executed')\n"
    )
    assert EXAMPLES.module_constructions(source) == {
        ("simulation", "simpleNav"), ("fswAlgorithms", "mrpPDRust"),
    }


def test_selection_is_limited_documented_and_stable(tmp_path):
    """Prefer three top-level examples and use nested examples only to fill gaps."""
    names = ["nested/scenarioA", "scenarioZ", "scenarioC", "scenarioB", "scenarioA"]
    script = "from Basilisk.simulation import simpleNav\nnav = simpleNav.SimpleNav()\n"
    for name in names + ["scenarioUndocumented", "helper"]:
        path = tmp_path / (name + ".py")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(script, encoding="utf8")
    page = "Documentation/simulation/navigation/simpleNav/simpleNav"
    modules = {page: {"name": "simpleNav"}}
    docs = ["examples/" + name for name in names] + ["examples/helper"]
    index = EXAMPLES.example_usage_index(tmp_path, reversed(docs), modules)
    assert index[page] == ["examples/scenarioA", "examples/scenarioB", "examples/scenarioC"]
    # Renames/removals and source edits must refresh selection without a registry.
    (tmp_path / "scenarioA.py").unlink()
    (tmp_path / "scenarioB.py").write_text("# No longer constructs a module\n", encoding="utf8")
    index = EXAMPLES.example_usage_index(tmp_path, docs, modules)
    assert index[page] == ["examples/scenarioC", "examples/scenarioZ", "examples/nested/scenarioA"]
