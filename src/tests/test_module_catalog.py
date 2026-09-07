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

"""Exercise catalog generation with an isolated, incremental Sphinx build."""

import ast
from html.parser import HTMLParser
from pathlib import Path
import subprocess
import sys

import pytest


class CatalogRows(HTMLParser):
    """Read table cells without depending on optional HTML parsing packages."""

    def __init__(self, html):
        super().__init__()
        self.rows = []
        self.search_text = []
        self.cell = None
        self.row = []
        self.row_search = ""
        self.feed(html)

    def handle_starttag(self, tag, attrs):
        if tag == "tr":
            self.row = []
            self.row_search = ""
        elif tag == "td":
            self.cell = ""
        elif tag == "span":
            attributes = dict(attrs)
            if "bsk-catalog-search-text" in attributes.get("class", "").split():
                self.row_search = attributes["data-text"]

    def handle_data(self, data):
        if self.cell is not None:
            self.cell += data

    def handle_endtag(self, tag):
        if tag == "td":
            self.row.append(" ".join(self.cell.split()))
            self.cell = None
        elif tag == "tr" and self.row:
            self.rows.append(self.row)
            self.search_text.append(self.row_search)


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
@pytest.mark.parametrize("jobs", [1, 2])
def test_catalog_tracks_modules_and_incremental_edits(tmp_path, jobs):
    """Cover all languages, grouping, exclusions, summary changes, and deletion."""
    pytest.importorskip("sphinx")
    docs = Path(__file__).resolve().parents[2] / "docs/source"
    conf = (docs / "conf.py").read_text(encoding="utf8")
    role = next(node for node in ast.parse(conf).body
                if isinstance(node, ast.FunctionDef) and node.name == "module_type_role")
    source = tmp_path / "source"
    output = tmp_path / "html"
    source.mkdir()
    (source / "conf.py").write_text(
        "import sys\nfrom docutils import nodes\nfrom docutils.parsers.rst import roles\n"
        f"sys.path.insert(0, {str(docs / '_ext')!r})\n"
        + ast.get_source_segment(conf, role)
        + "\nroles.register_local_role('module-type', module_type_role)\n"
        "extensions = ['module_catalog', 'module_examples']\n"
        f"bsk_example_source_root = {str(tmp_path / 'examples')!r}\n"
        f"html_static_path = [{str(docs / '_static')!r}]\n",
        encoding="utf8",
    )
    (source / "index.rst").write_text(
        "Home\n====\n\n.. toctree::\n\n   Documentation/index\n", encoding="utf8"
    )
    (source / "Documentation").mkdir()
    landing = source / "Documentation/index.rst"
    landing.write_text(
        "Documentation\n=============\n\n.. bsk-module-catalog::\n", encoding="utf8"
    )

    def module(path, language="C", summary="A short module summary."):
        """Write a generated-style module page, independently of its folder name."""
        target = source / (path + ".rst")
        target.parent.mkdir(parents=True, exist_ok=True)
        name = target.stem
        title = f":module-type:`{language}` Module: {name}"
        target.write_text(
            f":orphan:\n\n.. _{name}:\n\n{title}\n{'=' * len(title)}\n\n"
            ".. note::\n\n   Not the module summary.\n\n"
            ".. sidebar:: Auxiliary Files\n   :class: bsk-module-auxiliary\n\n"
            "   auxiliary_only_test_token\n\n"
            f"Executive Summary\n-----------------\n\n{summary}\n\n"
            "Detailed Module Description\n---------------------------\n\n"
            "Models a conical **shadow** during an eclipse.\n\n"
            "User Guide\n----------\n\n"
            "- Nested guidance with a literal ``<sensor>`` and \"quoted\" & values.\n\n"
            f".. py:function:: embedded_api_{name}()\n\n"
            "   embedded_api_only_token\n\n"
            ".. bsk-module-guide-end::\n\n"
            "Generated Module API\n--------------------\n\n"
            "generated_api_only_token\n",
            encoding="utf8",
        )
        return target

    controller = module("Documentation/fswAlgorithms/control/LegacyName/mrpPD", summary=(
        "Controls the *attitude* using ``MRPs`` and :ref:`sensor data <demo>`. "
        "Values < 2 & a quoted \"label\" remain readable."
    ))
    rust = module("Documentation/moduleTemplates/rustTemplate/rustTemplate", "Rust", summary=(
        "A longer introductory paragraph. " * 12 + "Horizon visibility is supported."
    ))
    module("Documentation/simulation/sensors/demo/demo", "C++")
    module("Documentation/simulation/navigation/pythonModule/pythonModule", "Python")
    module("Documentation/fswAlgorithms/effectorInterfaces/errorConversion/eulerRotation")
    module("Documentation/fswAlgorithms/effectorInterfaces/errorConversion/mrpRotation")
    module("Documentation/simulation/_GeneralModuleFiles/base")
    module("Documentation/simulation/sensors/demo/_UnitTest/test_demo")
    module("Documentation/architecture/helper")

    def scenario(name, script):
        """Provide a documented scenario source without executing it in Sphinx."""
        python_file = tmp_path / "examples" / (name + ".py")
        python_file.parent.mkdir(parents=True, exist_ok=True)
        python_file.write_text(script, encoding="utf8")
        document = source / "examples" / (name + ".rst")
        document.parent.mkdir(parents=True, exist_ok=True)
        title = Path(name).name
        document.write_text(f":orphan:\n\n{title}\n{'=' * len(title)}\n", encoding="utf8")

    scenario("scenarioAlpha", (
        "from Basilisk.fswAlgorithms import mrpPD\ncontroller = mrpPD.mrpPD()\n"
        "from Basilisk.simulation import pythonModule\nmodule = pythonModule.PythonModule()\n"
    ))
    scenario("nested/scenarioBeta", (
        "from Basilisk.moduleTemplates import rustTemplate\nmodule = rustTemplate.RustTemplate()\n"
    ))
    rust.write_text(rust.read_text(encoding="utf8").replace(
        ".. sidebar:: Auxiliary Files\n   :class: bsk-module-auxiliary\n\n"
        "   auxiliary_only_test_token\n\n", ""
    ), encoding="utf8")

    def build():
        """Reuse the environment to exercise worker merging and stale-row cleanup."""
        result = subprocess.run(
            [sys.executable, "-m", "sphinx", "-b", "html", "-W", "-q", "-j", str(jobs),
             str(source), str(output)], capture_output=True, text=True, check=False,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        return (output / "Documentation/index.html").read_text(encoding="utf8")

    html = build()
    parsed = CatalogRows(html)
    rows = parsed.rows
    assert [row[0] for row in rows] == [
        "demo", "eulerRotation", "mrpPD", "mrpRotation", "pythonModule", "rustTemplate",
    ]
    assert {row[1] for row in rows} == {"C", "C++", "Python", "Rust"}
    assert rows[2][2] == "FSW algorithms / control"
    assert rows[2][3] == (
        'Controls the attitude using MRPs and sensor data. Values < 2 & a quoted “label” remain readable.'
    )
    # Search the full authored guide for all four languages without enlarging
    # the displayed excerpt or including generated APIs/auxiliary navigation.
    assert all("conical shadow" in text for text in parsed.search_text)
    assert all('<sensor> and “quoted” & values' in text for text in parsed.search_text)
    assert all("shadow" not in row[3] for row in rows)
    assert "Horizon visibility" in parsed.search_text[5]
    assert "Horizon visibility" not in rows[5][3]
    for token in ("auxiliary_only_test_token", "generated_api_only_token", "embedded_api_only_token"):
        assert all(token not in text for text in parsed.search_text)
    assert 'href="fswAlgorithms/control/LegacyName/mrpPD.html"' in html
    assert 'class="bsk-catalog-controls" hidden' in html
    assert 'js/module-catalog.js' in html
    assert 'js/module-catalog.js' not in (output / "index.html").read_text(encoding="utf8")
    landing_timestamp = landing.stat().st_mtime_ns
    controller_html = output / "Documentation/fswAlgorithms/control/LegacyName/mrpPD.html"
    assert 'class="bsk-module-examples' in controller_html.read_text(encoding="utf8")
    assert '../../../../examples/scenarioAlpha.html' in controller_html.read_text(encoding="utf8")
    rust_html = (output / "Documentation/moduleTemplates/rustTemplate/rustTemplate.html").read_text(encoding="utf8")
    assert "Auxiliary Files" in rust_html and "scenarioBeta" in rust_html
    python_html = output / "Documentation/simulation/navigation/pythonModule/pythonModule.html"
    assert "scenarioAlpha" in python_html.read_text(encoding="utf8")

    controller.write_text(controller.read_text(encoding="utf8").replace(
        "Controls the *attitude*", "Updated control of the *attitude*"
    ).replace("conical **shadow**", "planetary **penumbra**"), encoding="utf8")
    rust.unlink()
    module("Documentation/simulation/dynamics/NewFolder/alpha", summary="Added automatically.")
    scenario("scenarioAlpha", "# No longer uses these modules\n")
    scenario("scenarioDelta", "from Basilisk.fswAlgorithms import mrpPD\nx = mrpPD.mrpPD()\n")
    parsed = CatalogRows(build())
    rows = parsed.rows
    assert rows[0][0] == "alpha" and rows[0][3] == "Added automatically."
    assert "rustTemplate" not in [row[0] for row in rows]
    assert next(row[3] for row in rows if row[0] == "mrpPD").startswith("Updated control")
    controller_search = parsed.search_text[[row[0] for row in rows].index("mrpPD")]
    assert "planetary penumbra" in controller_search and "conical shadow" not in controller_search
    assert landing.stat().st_mtime_ns == landing_timestamp
    assert "scenarioDelta" in controller_html.read_text(encoding="utf8")
    assert "scenarioAlpha" not in controller_html.read_text(encoding="utf8")
    # The module page must update even though only the example source changed.
    assert 'class="bsk-module-examples' not in python_html.read_text(encoding="utf8")
