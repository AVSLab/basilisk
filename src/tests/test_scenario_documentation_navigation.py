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

"""Exercise scenario grouping through real Sphinx HTML builds."""

from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest


EXTENSIONS = Path(__file__).resolve().parents[2] / "docs/source/_ext"


def sidebar(html):
    """Parse the theme's navigation fragment, including nested link ancestry."""
    fragment = re.search(r'<div class="wy-menu wy-menu-vertical".*?</div>', html, re.S)
    return ET.fromstring(fragment.group())


def item_for(menu, title):
    """Find the list item whose own link has the requested text."""
    return next(item for item in menu.iter("li")
                if "".join(item.find("a").itertext()) == title)


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
def test_scenario_sidebar_preserves_links_and_groups_current_page(tmp_path):
    """Group deep scenarios, preserve other pages, and refresh renamed headings."""
    pytest.importorskip("sphinx")
    pytest.importorskip("sphinx_rtd_theme")
    source = tmp_path / "source"
    output = tmp_path / "html"
    source.mkdir()
    configuration = (
        "import sys\n"
        f"sys.path.insert(0, {str(EXTENSIONS)!r})\n"
        "extensions = ['generated_documentation']\n"
        "html_theme = 'sphinx_rtd_theme'\n"
        "html_theme_options = {'titles_only': True, 'navigation_depth': 4, "
        "'collapse_navigation': True}\n"
    )
    (source / "conf.py").write_text(configuration, encoding="utf8")
    files = {
        "index": "Home\n====\n\n.. toctree::\n\n   Learn\n   guide\n",
        "Learn": "Learn\n=====\n\n.. toctree::\n\n   examples/index\n",
        "guide": "Guide\n=====\n\nGuide details\n-------------\n\nText.\n",
        "examples/index": (
            "Examples\n========\n\nOrbital Simulations\n-------------------\n\n"
            ".. toctree::\n   :maxdepth: 1\n   :numbered:\n\n   Orbit example <orbit>\n\n"
            "Attitude Simulations\n--------------------\n\n"
            "Actuators\n^^^^^^^^^\n\n.. toctree::\n   :maxdepth: 1\n\n"
            "   Wheel example <nested/wheels>\n"
            "   External reference <https://example.org/>\n\n"
            "Support Files\n~~~~~~~~~~~~~\n\n.. toctree::\n\n   support/index\n"
        ),
        "examples/orbit": "Orbit\n=====\n\nOrbit details\n-------------\n\nText.\n",
        "examples/nested/wheels": (
            "Wheels\n======\n\nWheel details\n-------------\n\nText.\n"
        ),
        "examples/support/index": (
            "Support index\n=============\n\n.. toctree::\n\n   helper\n"
        ),
        "examples/support/helper": "Helper\n======\n\nText.\n",
    }
    for name, contents in files.items():
        path = source / (name + ".rst")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding="utf8")

    def build():
        """Build the fixture with warnings treated as failures."""
        result = subprocess.run(
            [sys.executable, "-m", "sphinx", "-b", "html", "-W", "-q",
             "-d", str(tmp_path / "doctrees"), str(source), str(output)],
            capture_output=True, text=True, check=False,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        return {name: (output / (name + ".html")).read_text(encoding="utf8")
                for name in files}

    baseline = build()
    (source / "conf.py").write_text(
        configuration.replace("['generated_documentation']",
                              "['generated_documentation', 'scenario_navigation']"),
        encoding="utf8",
    )
    grouped = build()
    for name in files:
        before = sidebar(baseline[name])
        after = sidebar(grouped[name])
        group_items = [item for item in after.iter("li")
                       if "bsk-scenario-group" in item.get("class", "").split()]
        group_links = {item.find("a") for item in group_items}
        assert [(link.get("href"), "".join(link.itertext()))
                for link in before.iter("a")] == [
                    (link.get("href"), "".join(link.itertext()))
                    for link in after.iter("a") if link not in group_links
                ]
        if not name.startswith("examples/"):
            assert ET.tostring(before) == ET.tostring(after)
        assert "Wheel details" not in ET.tostring(after, encoding="unicode")
        assert "Guide details" not in ET.tostring(after, encoding="unicode")

    overview = sidebar(grouped["examples/index"])
    attitude = item_for(overview, "Attitude Simulations")
    actuators = item_for(attitude, "Actuators")
    assert item_for(actuators, "Wheel example").find("a").get("href") == "nested/wheels.html"
    assert "current" not in attitude.get("class", "").split()
    support = item_for(actuators, "Support Files")
    assert item_for(support, "Support index").find("a").get("href") == "support/index.html"
    helper = sidebar(grouped["examples/support/helper"])
    assert "current" in item_for(helper, "Support Files").get("class", "").split()
    assert item_for(helper, "Helper").find("a").get("href") == "#"

    scenario = sidebar(grouped["examples/nested/wheels"])
    for title in ("Attitude Simulations", "Actuators", "Wheel example"):
        assert "current" in item_for(scenario, title).get("class", "").split()
    assert "current" not in item_for(scenario, "Orbital Simulations").get("class", "").split()
    assert item_for(scenario, "Actuators").find("a").get("href") == "../index.html#actuators"
    assert item_for(scenario, "Wheel example").find("a").get("href") == "#"

    # Only the overview source changes: unchanged scenarios must get fresh groups.
    index = source / "examples/index.rst"
    index.write_text(files["examples/index"].replace("Actuators", "Actuation"), encoding="utf8")
    updated = build()
    scenario = sidebar(updated["examples/nested/wheels"])
    assert item_for(scenario, "Actuation").find("a").get("href") == "../index.html#actuation"
