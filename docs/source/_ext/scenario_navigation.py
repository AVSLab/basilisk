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

"""Group scenario sidebar links using the authored example index headings."""

from collections import defaultdict, deque

from docutils import nodes
from sphinx import addnodes
from sphinx.environment.adapters.toctree import TocTree
from sphinx.util import url_re


EXAMPLE_INDEX = "examples/index"


def group_scenarios(app, pagename, tree):
    """Wrap resolved scenario links without changing other navigation branches."""
    index_uri = app.builder.get_relative_uri(pagename, EXAMPLE_INDEX)
    index_link = next((
        link for link in tree.findall(nodes.reference)
        if link.get("refuri") == index_uri and not link.get("anchorname")
    ), None)
    if index_link is None:
        return
    index_item = index_link.parent.parent
    original = next((child for child in index_item
                     if isinstance(child, nodes.bullet_list)), None)
    if original is None:
        return  # Respect Sphinx's pruning of inactive branches.

    # Reuse Sphinx's links, titles, numbering, current-page flags, and subtrees.
    # Only headings are new; document depth elsewhere keeps its configured limit.
    links = defaultdict(deque)
    for item in original:
        link = next(item.findall(nodes.reference), None)
        if link is not None:
            links[link["refuri"]].append(item)

    grouped = app.env.tocs[EXAMPLE_INDEX].deepcopy()[0][1]
    grouped["classes"].append("bsk-scenario-tree")
    for toctree in list(grouped.findall(addnodes.toctree)):
        entries = []
        for _, docname in toctree["entries"]:
            uri = (docname if url_re.match(docname) else
                   app.builder.get_relative_uri(pagename, docname))
            if links[uri]:
                entries.append(links[uri].popleft())
        toctree.replace_self(entries)

    # Empty groups can result from excluded or hidden toctree entries.
    for item in reversed(list(grouped.findall(nodes.list_item))):
        link = next(item.findall(nodes.reference), None)
        if link is None or link.get("refuri") != EXAMPLE_INDEX:
            continue
        children = next((child for child in item
                         if isinstance(child, nodes.bullet_list)), None)
        if not children:
            item.parent.remove(item)
            continue
        link["refuri"] = index_uri + link["anchorname"]
        item["classes"].append("bsk-scenario-group")
        if any("current" in child.get("classes", []) for child in children):
            item["classes"].append("current")

    # Retain any entries not represented in the heading tree defensively.
    for remaining in links.values():
        grouped.extend(remaining)

    def set_depth(root, depth):
        """Account for inserted heading levels in the theme's tree classes."""
        for child in root:
            if isinstance(child, nodes.bullet_list):
                set_depth(child, depth + 1)
            elif isinstance(child, (nodes.list_item, addnodes.compact_paragraph)):
                child["classes"] = [name for name in child["classes"]
                                    if not name.startswith("toctree-l")]
                child["classes"].append(f"toctree-l{depth}")
                set_depth(child, depth)

    index_depth = next(int(name.removeprefix("toctree-l"))
                       for name in index_item["classes"]
                       if name.startswith("toctree-l"))
    set_depth(grouped, index_depth + 1)
    index_item.replace(original, grouped)


def add_scenario_navigation(app, pagename, templatename, context, doctree):
    """Provide grouped HTML navigation while leaving the page body unchanged."""
    if app.builder.format != "html" or EXAMPLE_INDEX not in app.env.tocs:
        return

    def toctree(collapse=True, **options):
        """Render the theme's normal tree with scenario groups inserted."""
        options.setdefault("includehidden", False)
        if options.get("maxdepth") == "":
            options.pop("maxdepth")
        tree = TocTree(app.env).get_toctree_for(
            pagename, app.builder, collapse=collapse, **options
        )
        if tree is not None and options.get("titles_only"):
            group_scenarios(app, pagename, tree)
        return app.builder.render_partial(tree)["fragment"]

    context["toctree"] = toctree


def setup(app):
    """Register scenario grouping for the HTML sidebar."""
    app.connect("html-page-context", add_scenario_navigation)
    return {"version": "1", "parallel_read_safe": True, "parallel_write_safe": True}
