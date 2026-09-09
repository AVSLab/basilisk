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

"""Check documentation search ranking with Node.js in the dedicated docs suite."""

import json
from pathlib import Path
import shutil
import subprocess

import pytest


SCORER = Path(__file__).resolve().parents[2] / "docs/source/_static/js/search-scorer.js"
MODULE = "Documentation/fswAlgorithms/attControl/mrpPD"


def rank(query, entries):
    """Execute the real scorer, retaining result paths and numeric scores."""
    node = shutil.which("node")
    if node is None:
        pytest.fail(
            "Documentation search tests require Node.js on PATH; install Node.js to run this suite.",
            pytrace=False,
        )
    script = """
const fs = require('node:fs');
const vm = require('node:vm');
const input = JSON.parse(fs.readFileSync(0, 'utf8'));
const context = {URLSearchParams, window: {location: {search: '?q=' + encodeURIComponent(input.query)}}};
vm.createContext(context);
vm.runInContext(fs.readFileSync(process.argv[1], 'utf8'), context);
const scores = input.entries.map(item => [item[0], context.Scorer.score(item)]);
scores.sort((a, b) => b[1] - a[1]);
process.stdout.write(JSON.stringify(scores));
"""
    result = subprocess.run(
        [node, "-e", script, str(SCORER)],
        input=json.dumps({"query": query, "entries": entries}),
        capture_output=True, text=True, check=True,
    )
    return json.loads(result.stdout)


def entry(path, title, score=15, anchor=""):
    """Construct the six-field result format supported since Sphinx 7."""
    return [path, title, anchor, None, score, path + ".rst"]


@pytest.mark.parametrize("query", ["mrpPD", "MRPPD", " mrpPD "])
def test_exact_module_outranks_related_pages(query):
    """Put the exact module ahead of similarly named modules, tests, and indexes."""
    results = rank(query, [
        entry(MODULE + "/_UnitTest/test_mrpPD", "test_mrpPD", 26),
        entry(MODULE + "/index", "mrpPD", 16),
        entry(MODULE + "/mrpPD", "C Module: mrpPD", 5),
        entry(MODULE + "Rust/mrpPDRust", "Rust Module: mrpPDRust", 20),
        entry("examples/scenarioAttitudeFeedback", "scenarioAttitudeFeedback"),
    ])
    assert results[0][0] == MODULE + "/mrpPD"
    assert results[-1][0] == MODULE + "/_UnitTest/test_mrpPD"


def test_guides_and_examples_precede_internal_validation():
    """General-topic searches retain test results but place user material first."""
    results = rank("attitude control", [
        entry(MODULE + "/_UnitTest/test_mrpPD", "test_mrpPD", 26),
        entry("Learn/makingModules/rustModules", "Making Rust Modules", 5),
        entry("examples/scenarioAttitudeFeedback", "scenarioAttitudeFeedback", 5),
    ])
    assert results[-1][0].endswith("test_mrpPD")
    assert len(results) == 3


@pytest.mark.parametrize("query", ["test_mrpPD", "mrpPD tests", "mrpPD validation"])
def test_explicit_validation_queries_keep_test_priority(query):
    """Do not bury validation documentation when the user explicitly seeks it."""
    results = rank(query, [
        entry(MODULE + "/_UnitTest/test_mrpPD", "test_mrpPD", 26),
        entry("Learn/makingModules/rustModules", "Making Rust Modules", 5),
    ])
    assert results[0][0].endswith("test_mrpPD")


def test_exact_api_query_keeps_symbol_priority():
    """Specific function searches still prioritize their matching API entry."""
    results = rank("Reset_mrpPD", [
        entry(MODULE + "/mrpPD", "Reset_mrpPD", 26, "#reset-mrppd"),
        entry(MODULE + "/mrpPD", "C Module: mrpPD", 5),
    ])
    assert results[0][1] == 106


def test_legacy_folder_name_does_not_block_module_boost():
    """Use the module page name rather than assuming it matches the directory."""
    path = "Documentation/simulation/dynamics/RadiationPressure/radiationPressure"
    assert rank("radiationPressure", [entry(path, "C++ Module: radiationPressure")])[0][1] == 127
