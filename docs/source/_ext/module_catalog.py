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

"""Build a searchable catalog from the existing, parsed module documentation."""

from html import escape
from pathlib import PurePosixPath
from textwrap import shorten

from docutils import nodes
from docutils.parsers.rst import Directive
from sphinx import addnodes

from bsk_module_io import MODULE_TYPE_STYLES


CATALOG_DOCNAME = "Documentation/index"
AREAS = {
    "fswAlgorithms": "FSW algorithms",
    "simulation": "Simulation",
    "moduleTemplates": "Module templates",
}


class ModuleCatalog(nodes.General, nodes.Element):
    """Placeholder replaced after Sphinx has read all module pages."""


class ModuleCatalogDirective(Directive):
    """Insert the generated catalog; individual modules require no directive."""

    def run(self):
        return [ModuleCatalog()]


class ModuleGuideEndDirective(Directive):
    """Mark the generated API boundary without changing the rendered page."""

    def run(self):
        return [nodes.comment(bsk_module_guide_end=True)]


def _guide_text(section):
    """Collect authored guide text, stopping before appended API documentation."""
    chunks = []

    def visit(node):
        """Return whether the generated end-of-guide marker was reached."""
        if isinstance(node, nodes.comment) and node.get("bsk_module_guide_end"):
            return True
        if isinstance(node, (nodes.comment, nodes.raw, nodes.system_message,
                             addnodes.toctree, addnodes.desc)):
            return False
        if isinstance(node, nodes.Element) and "bsk-module-auxiliary" in node.get("classes", []):
            return False
        if isinstance(node, nodes.Text):
            chunks.append(str(node))
            return False
        return any(visit(child) for child in node.children)

    visit(section)
    return " ".join(" ".join(chunks).split())


def _summary(section):
    """Extract a short paragraph, excluding sidebars, notes, tables, and API data."""
    excluded = (nodes.sidebar, nodes.Admonition, nodes.table, nodes.field_list,
                nodes.definition_list)
    for paragraph in section.findall(nodes.paragraph):
        parent = paragraph.parent
        while parent is not section and parent is not None:
            if isinstance(parent, excluded) or isinstance(parent, nodes.section):
                break
            parent = parent.parent
        if parent is section and paragraph.astext().strip():
            return shorten(paragraph.astext(), width=240, placeholder="…")
    return ""


def module_entry(docname, doctree):
    """Identify a module by its generated title, not by rescanning source code.

    :param docname: Sphinx document name, relative to the documentation root.
    :param doctree: Parsed module page, including its authored RST content.
    :return: Plain catalog metadata, or ``None`` for non-module pages.
    """
    path = PurePosixPath(docname)
    parts = path.parts
    if (len(parts) < 4 or parts[0] != "Documentation" or parts[1] not in AREAS
            or any(part.startswith("_") for part in parts[2:])):
        return None
    section = next((node for node in doctree if isinstance(node, nodes.section)), None)
    if section is None:
        return None
    title = next((node for node in section if isinstance(node, nodes.title)), None)
    if title is None:
        return None
    badge = next((node for node in title.findall(nodes.inline)
                  if "module-type-label" in node.get("classes", [])), None)
    if badge is None or badge.astext() not in MODULE_TYPE_STYLES:
        return None
    language = badge.astext()
    if title.astext().strip() != f"{language} Module: {path.name}":
        return None

    summary = ""
    for child in section.findall(nodes.section):
        heading = next((node for node in child if isinstance(node, nodes.title)), None)
        if heading is not None and heading.astext().strip().casefold() == "executive summary":
            summary = _summary(child)
            break
    summary = summary or _summary(section) or "See the module documentation."
    category = " / ".join([AREAS[parts[1]], *parts[2:-2]])
    return {"docname": docname, "name": path.name, "language": language,
            "category": category, "summary": summary, "search_text": _guide_text(section)}


def collect_module(app, doctree):
    """Cache text records rather than whole module doctrees."""
    records = app.env.__dict__.setdefault("bsk_module_catalog", {})
    entry = module_entry(app.env.docname, doctree)
    if entry is not None:
        records[app.env.docname] = entry


def purge_module(app, env, docname):
    """Discard stale records when a page is changed or removed."""
    getattr(env, "bsk_module_catalog", {}).pop(docname, None)


def merge_modules(app, env, docnames, other):
    """Merge only the records read by each parallel Sphinx worker."""
    records = env.__dict__.setdefault("bsk_module_catalog", {})
    other_records = getattr(other, "bsk_module_catalog", {})
    for docname in docnames:
        if docname in other_records:
            records[docname] = other_records[docname]


def refresh_catalog(app, env):
    """Refresh the landing page even when only a module's guide has changed."""
    return [CATALOG_DOCNAME] if CATALOG_DOCNAME in env.found_docs else []


def _controls(entries):
    """Render escaped, initially hidden controls for progressive enhancement."""
    categories = sorted({entry["category"] for entry in entries}
                        | {entry["category"].split(" / ")[0] for entry in entries})
    category_options = "".join(
        f'<option value="{escape(category, quote=True)}">{escape(category)}</option>'
        for category in categories
    )
    languages = sorted({entry["language"] for entry in entries})
    language_options = "".join(
        f'<option value="{escape(language, quote=True)}">{escape(language)}</option>'
        for language in languages
    )
    return f"""
<form class="bsk-catalog-controls" hidden role="search" aria-label="Module catalog">
  <label class="bsk-catalog-search">Search modules
    <input type="search" name="query" placeholder="Name or topic, e.g. attitude" autocomplete="off">
  </label>
  <label>Category
    <select name="category"><option value="">All categories</option>{category_options}</select>
  </label>
  <label>Language
    <select name="language"><option value="">All languages</option>{language_options}</select>
  </label>
  <button type="reset">Clear filters</button>
</form>
"""


def render_catalog(app, doctree, fromdocname):
    """Render native Sphinx links and a usable table even without JavaScript."""
    for placeholder in list(doctree.findall(ModuleCatalog)):
        entries = sorted(
            (entry for name, entry in getattr(app.env, "bsk_module_catalog", {}).items()
             if name in app.env.found_docs),
            key=lambda entry: (entry["name"].casefold(), entry["docname"]),
        )
        catalog = nodes.container(classes=["bsk-module-catalog"])
        if app.builder.format == "html":
            catalog += nodes.raw("", _controls(entries), format="html")
        catalog += nodes.paragraph("", f"{len(entries)} modules", classes=["bsk-catalog-count"])
        table = nodes.table(classes=["bsk-catalog-table", "colwidths-given"])
        group = nodes.tgroup(cols=4)
        table += group
        for width in (23, 12, 22, 43):
            group += nodes.colspec(colwidth=width)
        head = nodes.thead()
        heading = nodes.row()
        for label in ("Module", "Language", "Category", "Description"):
            heading += nodes.entry("", nodes.paragraph("", label))
        head += heading
        group += head
        body = nodes.tbody()
        group += body
        for entry in entries:
            row = nodes.row()
            name = nodes.paragraph()
            if app.builder.format == "html":
                name += nodes.reference(
                    "", entry["name"], internal=True,
                    refuri=app.builder.get_relative_uri(fromdocname, entry["docname"]),
                )
            else:
                name += nodes.Text(entry["name"])
            if app.builder.format == "html":
                # Keep the full guide out of the visible table and screen readers.
                # An escaped attribute also avoids adding it to site-search text.
                name += nodes.raw("", (
                    '<span hidden class="bsk-catalog-search-text" '
                    f'data-text="{escape(entry["search_text"], quote=True)}"></span>'
                ), format="html")
            row += nodes.entry("", name, classes=["bsk-catalog-name"])
            language = nodes.paragraph()
            language += nodes.inline("", entry["language"], classes=[
                "module-type-label", MODULE_TYPE_STYLES[entry["language"]]["css_class"],
            ])
            row += nodes.entry("", language, classes=["bsk-catalog-language"])
            row += nodes.entry("", nodes.paragraph("", entry["category"]),
                               classes=["bsk-catalog-category"])
            row += nodes.entry("", nodes.paragraph("", entry["summary"]))
            body += row
        catalog += nodes.container("", table, classes=["wy-table-responsive"])
        if app.builder.format == "html":
            catalog += nodes.raw("", """
<p class="bsk-catalog-empty" hidden>No modules match these filters. Try a different search or clear the filters.</p>
<nav class="bsk-catalog-pagination" hidden aria-label="Module catalog pages">
  <button type="button" class="bsk-catalog-previous">Previous</button>
  <span class="bsk-catalog-page"></span>
  <button type="button" class="bsk-catalog-next">Next</button>
</nav>
""", format="html")
        placeholder.replace_self(catalog)


def add_catalog_script(app, pagename, templatename, context, doctree):
    """Load filtering code on the catalog page only."""
    if pagename == CATALOG_DOCNAME:
        app.add_js_file("js/module-catalog.js", defer="defer")


def setup(app):
    """Register automatic collection and parallel-safe incremental rendering."""
    app.add_node(ModuleCatalog)
    app.add_directive("bsk-module-catalog", ModuleCatalogDirective)
    app.add_directive("bsk-module-guide-end", ModuleGuideEndDirective)
    app.connect("doctree-read", collect_module)
    app.connect("env-purge-doc", purge_module)
    app.connect("env-merge-info", merge_modules)
    app.connect("env-updated", refresh_catalog)
    app.connect("doctree-resolved", render_catalog)
    app.connect("html-page-context", add_catalog_script)
    return {"version": "2", "env_version": 2,
            "parallel_read_safe": True, "parallel_write_safe": True}
