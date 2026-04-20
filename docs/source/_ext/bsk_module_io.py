#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

from dataclasses import dataclass
from html import escape

from docutils import nodes
from docutils.parsers.rst import Directive, directives
from docutils.statemachine import StringList


CU_GOLD_70 = "#cfb87cb3"


@dataclass
class ModuleMessage:
    direction: str
    name: str
    payload_type: str
    description: str


def _dot_escape(text: str) -> str:
    return text.replace("\\", "\\\\").replace("\"", "\\\"")


def _html_label_escape(text: str) -> str:
    return escape(text, quote=True)


def _table_payload_ref(payload_type: str) -> str:
    if payload_type.startswith(":"):
        return payload_type
    return f":ref:`{payload_type}`"


class BskModuleIODirective(Directive):
    """
    Render a Basilisk module I/O diagram and standard message table.

    The directive body accepts entries of the form::

        input messageName PayloadType
           Message description.

        output messageName PayloadType
           Message description.
    """

    required_arguments = 1
    optional_arguments = 0
    final_argument_whitespace = True
    has_content = True
    option_spec = {
        "caption": directives.unchanged,
    }

    def run(self):
        messages = self._parse_messages()
        if not messages:
            raise self.error("bsk-module-io requires at least one input or output entry.")

        lines = self._build_rst(messages)
        container = nodes.container()
        self.state.nested_parse(StringList(lines), self.content_offset, container)
        return container.children

    def _parse_messages(self):
        messages = []
        index = 0

        while index < len(self.content):
            raw_line = self.content[index]
            line = raw_line.strip()
            index += 1

            if not line:
                continue

            fields = line.split(None, 2)
            if len(fields) != 3 or fields[0] not in ("input", "output"):
                raise self.error(
                    "bsk-module-io entries must use: 'input <name> <payloadType>' "
                    "or 'output <name> <payloadType>'."
                )

            entry_indent = len(raw_line) - len(raw_line.lstrip())
            description_lines = []
            while index < len(self.content):
                next_line = self.content[index]
                next_stripped = next_line.strip()
                next_indent = len(next_line) - len(next_line.lstrip())
                next_fields = next_stripped.split(None, 2)
                if (
                    next_stripped
                    and next_indent <= entry_indent
                    and len(next_fields) == 3
                    and next_fields[0] in ("input", "output")
                ):
                    break
                description_lines.append(next_stripped)
                index += 1

            description = " ".join(part for part in description_lines if part)
            messages.append(ModuleMessage(fields[0], fields[1], fields[2], description))

        return messages

    def _build_rst(self, messages):
        module_name = self.arguments[0]
        caption = self.options.get("caption", "Module I/O Messages")
        graph_name = "".join(char if char.isalnum() else "_" for char in module_name) + "IO"
        graph_lines = [
            ".. graphviz::",
            f"   :alt: {module_name} module input and output messages",
            "   :align: center",
            "",
            f"   digraph {graph_name} {{",
            "      graph [rankdir=LR, bgcolor=\"transparent\", fontname=\"Helvetica\", fontsize=\"11.2\"];",
            "      node [shape=box, style=\"rounded,filled\", fillcolor=\"white\", color=\"#555555\", "
            "fontname=\"Helvetica\", fontsize=\"11.2\"];",
            "      edge [color=\"#555555\", fontname=\"Helvetica\", fontsize=\"11.2\"];",
            "",
            "      \"module\" [",
            f"         label=\"{_dot_escape(module_name)}\",",
            "         style=\"rounded,filled,bold\",",
            f"         fillcolor=\"{CU_GOLD_70}\"",
            "      ];",
            "",
        ]

        for msg_index, message in enumerate(messages):
            node_name = f"msg_{msg_index}"
            graph_lines.extend([
                f"      \"{node_name}\" [",
                "         label=<",
                "            <TABLE BORDER=\"0\" CELLBORDER=\"0\" CELLSPACING=\"0\">",
                "               <TR>",
                f"                  <TD ALIGN=\"LEFT\"><FONT FACE=\"Helvetica\" POINT-SIZE=\"9\">"
                f"{_html_label_escape(message.name)}</FONT></TD>",
                "               </TR>",
                "               <TR>",
                f"                  <TD ALIGN=\"LEFT\"><FONT FACE=\"Helvetica\" POINT-SIZE=\"7\"><I>"
                f"{_html_label_escape(message.payload_type)}</I></FONT></TD>",
                "               </TR>",
                "            </TABLE>",
                "         >",
                "      ];",
            ])

        graph_lines.append("")
        for msg_index, message in enumerate(messages):
            node_name = f"msg_{msg_index}"
            if message.direction == "input":
                graph_lines.append(f"      \"{node_name}\" -> \"module\";")
            else:
                graph_lines.append(f"      \"module\" -> \"{node_name}\";")

        graph_lines.extend([
            "   }",
            "",
            f".. list-table:: {caption}",
            "   :widths: 25 25 50",
            "   :header-rows: 1",
            "",
            "   * - Msg Variable Name",
            "     - Msg Type",
            "     - Description",
        ])

        for message in messages:
            graph_lines.extend([
                f"   * - {message.name}",
                f"     - {_table_payload_ref(message.payload_type)}",
                f"     - {message.description}",
            ])

        return graph_lines


def setup(app):
    app.add_directive("bsk-module-io", BskModuleIODirective)
    return {
        "version": "1.0",
        "parallel_read_safe": True,
        "parallel_write_safe": True,
    }
