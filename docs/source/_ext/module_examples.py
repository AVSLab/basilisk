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

"""Link module guides to a small, automatically selected set of example usages."""

import ast
from collections import defaultdict
from pathlib import Path, PurePosixPath
import re
import tokenize

NAMED_BINDINGS = tuple(getattr(ast, name) for name in ("ExceptHandler", "MatchAs", "MatchStar")
                       if hasattr(ast, name))


def _scope_nodes(body):
    """Walk one lexical scope without mixing imports from separate functions."""
    for node in body:
        yield node
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda)):
            yield from _scope_nodes(ast.iter_child_nodes(node))


def module_constructions(source):
    """Find ordinary Basilisk constructor calls without importing or executing code.

    Recognize qualified imports and aliases, including direct class imports.
    Constructors must match their module name, ignoring capitalization. Skip
    reassigned or ambiguous aliases; do not infer factories or dynamic imports.

    :param source: Python source text for one scenario.
    :return: Set of ``(package, module)`` pairs, such as ``('simulation', 'simpleNav')``.
    """
    found = set()

    def scan(body, inherited, parameters=()):
        scope = list(_scope_nodes(body))
        imports = defaultdict(set)
        shadowed = set(parameters)
        wildcard = False
        for node in scope:
            if isinstance(node, ast.Import):
                for alias in node.names:
                    name = alias.asname or alias.name.split(".")[0]
                    imports[name].add(alias.name if alias.asname else name)
            elif isinstance(node, ast.ImportFrom):
                for alias in node.names:
                    wildcard |= alias.name == "*"
                    path = f"{node.module}.{alias.name}" if not node.level else ""
                    imports[alias.asname or alias.name].add(path)
            elif isinstance(node, ast.Name) and isinstance(node.ctx, (ast.Store, ast.Del)):
                shadowed.add(node.id)
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                shadowed.add(node.name)
            elif isinstance(node, NAMED_BINDINGS) and node.name:
                shadowed.add(node.name)
            elif isinstance(node, (ast.Global, ast.Nonlocal)):
                shadowed.update(node.names)
        bindings = dict(inherited)
        bindings.update({name: None for name in shadowed})
        for name, paths in imports.items():
            bindings[name] = next(iter(paths)) if len(paths) == 1 and name not in shadowed else None
        if wildcard:
            bindings = {}

        for node in scope:
            if isinstance(node, ast.Call):
                parts = []
                function = node.func
                while isinstance(function, ast.Attribute):
                    parts.insert(0, function.attr)
                    function = function.value
                if not isinstance(function, ast.Name) or not bindings.get(function.id):
                    continue
                parts = bindings[function.id].split(".") + parts
                if (len(parts) == 4 and parts[0] == "Basilisk"
                        and parts[1] in ("simulation", "fswAlgorithms", "moduleTemplates")
                        and parts[2].casefold() == parts[3].casefold()):
                    found.add((parts[1], parts[2]))
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                parameters = [argument.arg for argument in ast.walk(node.args)
                              if isinstance(argument, ast.arg)]
                scan(node.body, bindings, parameters)
            elif isinstance(node, ast.ClassDef):
                scan(node.body, bindings)

    scan(ast.parse(source).body, {})
    return found


def example_usage_index(example_root, docnames, modules):
    """Select at most three documented scenarios per module, in stable order."""
    module_pages = defaultdict(list)
    for docname, entry in modules.items():
        module_pages[(PurePosixPath(docname).parts[1], entry["name"])].append(docname)
    index = defaultdict(list)
    scenarios = [PurePosixPath(name) for name in docnames
                 if name.startswith("examples/") and PurePosixPath(name).name.startswith("scenario")]
    scenarios.sort(key=lambda path: (len(path.parts) != 2, path.name.casefold(), str(path)))
    for scenario in scenarios:
        source = Path(example_root) / scenario.relative_to("examples").with_suffix(".py")
        if not source.is_file():
            continue
        with tokenize.open(source) as stream:
            constructions = module_constructions(stream.read())
        for module in constructions:
            for page in module_pages.get(module, ()):
                if len(index[page]) < 3:
                    index[page].append(str(scenario))
    return dict(index)


def refresh_example_usage(app, env):
    """Recheck sources each build and rewrite modules whose example links changed."""
    updated = example_usage_index(
        app.config.bsk_example_source_root, env.found_docs,
        getattr(env, "bsk_module_catalog", {}),
    ) if app.config.bsk_example_source_root else {}
    previous = getattr(env, "bsk_module_examples", {})
    env.bsk_module_examples = updated
    return sorted(page for page in previous.keys() | updated.keys()
                  if page in env.found_docs and previous.get(page) != updated.get(page))


def add_example_usage(app, doctree, fromdocname):
    """Add direct scenario links without giving scenarios another toctree parent."""
    from docutils import nodes

    examples = getattr(app.env, "bsk_module_examples", {}).get(fromdocname, [])
    if app.builder.format != "html" or not examples:
        return
    sidebar = next((node for node in doctree.findall(nodes.sidebar)
                    if "bsk-module-auxiliary" in node.get("classes", [])), None)
    if sidebar is None:
        sidebar = nodes.sidebar(classes=["bsk-module-auxiliary"])
        sidebar += nodes.title("", "Auxiliary Files")
        section = next(node for node in doctree if isinstance(node, nodes.section))
        summary = next((node for node in section.findall(nodes.section)
                        if node.children and isinstance(node[0], nodes.title)
                        and node[0].astext().casefold() == "executive summary"), None)
        if summary is not None:
            summary.parent.insert(summary.parent.index(summary), sidebar)
        else:
            section.insert(1, sidebar)
    sidebar += nodes.paragraph("", "", nodes.strong("", "Example usage"))
    links = nodes.bullet_list(classes=["bsk-module-examples"])
    for example in examples:
        link = nodes.reference("", "", internal=True,
                               refuri=app.builder.get_relative_uri(fromdocname, example))
        # Prefer breaks between words in long scenario names, without changing
        # the visible label or the text copied from the link.
        parts = re.split(r"(?<=[a-z0-9])(?=[A-Z])|(?<=_)", PurePosixPath(example).name)
        for index, part in enumerate(parts):
            if index:
                link += nodes.raw("", "<wbr>", format="html")
            link += nodes.Text(part)
        links += nodes.list_item("", nodes.paragraph("", "", link))
    sidebar += links


def setup(app):
    """Register example discovery after module metadata has been collected."""
    app.add_config_value("bsk_example_source_root", None, "env")
    app.connect("env-updated", refresh_example_usage)
    app.connect("doctree-resolved", add_example_usage)
    return {"version": "1", "env_version": 1,
            "parallel_read_safe": True, "parallel_write_safe": True}
