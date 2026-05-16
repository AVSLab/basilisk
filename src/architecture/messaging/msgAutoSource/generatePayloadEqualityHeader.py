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
from __future__ import annotations

import json
import sys

from generateSWIGModules import C_TYPE_TO_NPY_ENUM, _INT_CTYPES, _FLOAT_CTYPES


def _normalizedTypeName(typeName: str) -> str:
    return "".join(typeName.split())


def _canUseOperatorEqual(typeName: str) -> bool:
    typeName = _normalizedTypeName(typeName)
    if typeName in C_TYPE_TO_NPY_ENUM or typeName in _INT_CTYPES or typeName in _FLOAT_CTYPES:
        return True
    if typeName in {"bool", "std::string"}:
        return True
    if typeName.startswith("std::vector<") and typeName.endswith(">"):
        return _canUseOperatorEqual(typeName[len("std::vector<"):-1])
    return False


def _appendPayloadComparison(
    lines: list[str],
    field: dict,
    lhsExpr: str,
    rhsExpr: str,
    indent: str = "        ",
    loopDepth: int = 0,
) -> bool:
    """Append C++ statements comparing a field. Returns False if the field is unsupported."""
    kind = field["kind"]

    if kind in ("primitive", "enum"):
        lines.append(f"{indent}if (!({lhsExpr} == {rhsExpr})) {{")
        lines.append(f"{indent}    return false;")
        lines.append(f"{indent}}}")
        return True

    if kind == "array":
        lhsElement = lhsExpr
        rhsElement = rhsExpr
        for dimOffset, dim in enumerate(field["shape"]):
            loopIndex = f"i{loopDepth + dimOffset}"
            lines.append(f"{indent}for (int {loopIndex} = 0; {loopIndex} < {dim}; ++{loopIndex}) {{")
            indent += "    "
            lhsElement += f"[{loopIndex}]"
            rhsElement += f"[{loopIndex}]"
        allSupported = _appendPayloadComparison(
            lines, field["element"], lhsElement, rhsElement, indent,
            loopDepth + len(field["shape"]),
        )
        for _ in field["shape"]:
            indent = indent[:-4]
            lines.append(f"{indent}}}")
        return allSupported

    if kind == "struct":
        for subfield in field.get("fields", []):
            name = subfield["name"]
            if not _appendPayloadComparison(
                lines, subfield,
                f"{lhsExpr}.{name}", f"{rhsExpr}.{name}",
                indent, loopDepth,
            ):
                return False
        return True

    if kind == "unknown":
        typeName = field.get("ctype", "")
        if _normalizedTypeName(typeName).startswith("Eigen::"):
            rowCheck = f"{lhsExpr}.rows() != {rhsExpr}.rows()"
            colCheck = f"{lhsExpr}.cols() != {rhsExpr}.cols()"
            lines.append(f"{indent}if ({rowCheck} || {colCheck}) {{")
            lines.append(f"{indent}    return false;")
            lines.append(f"{indent}}}")
            lines.append(f"{indent}if (({lhsExpr}.array() != {rhsExpr}.array()).any()) {{")
            lines.append(f"{indent}    return false;")
            lines.append(f"{indent}}}")
            return True
        if _canUseOperatorEqual(typeName):
            lines.append(f"{indent}if (!({lhsExpr} == {rhsExpr})) {{")
            lines.append(f"{indent}    return false;")
            lines.append(f"{indent}}}")
            return True

    return False  # pointer or unsupported unknown type


def generateEqualityHeader(meta: dict, payloadType: str, searchDir: str) -> str | None:
    """
    Generate a C++ header with a PayloadEqualityTraits<payloadType> specialization.

    Returns the header text when all fields are supported, or None when any field is
    unsupported (e.g., raw pointers). Callers should write a comment-only placeholder
    when None is returned so CMake custom_command outputs are always satisfied.
    """
    bodyLines: list[str] = []
    for field in meta.get("fields", []):
        if not _appendPayloadComparison(bodyLines, field, f"lhs.{field['name']}", f"rhs.{field['name']}"):
            return None

    lines = [
        "// AUTO-GENERATED — DO NOT EDIT",
        "#pragma once",
        '#include "architecture/messaging/payloadEqualityTraits.h"',
        f'#include "architecture/{searchDir}/{payloadType}.h"',
        "",
        "template<>",
        f"struct PayloadEqualityTraits<{payloadType}> {{",
        "    static constexpr bool supported = true;",
        f"    static bool equal(const {payloadType}& lhs, const {payloadType}& rhs) {{",
    ]
    lines.extend(bodyLines)
    lines += [
        "        return true;",
        "    }",
        "};",
    ]
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    outputPath = sys.argv[1]
    metaJsonPath = sys.argv[2]
    payloadTypeName = sys.argv[3]
    searchDir = sys.argv[4]

    with open(metaJsonPath) as f:
        meta = json.load(f)

    content = generateEqualityHeader(meta, payloadTypeName, searchDir)

    with open(outputPath, "w") as f:
        if content is not None:
            f.write(content)
        else:
            # Always write the output file so CMake's custom_command output is satisfied.
            # Payloads with unsupported fields should define a manual specialization
            # in their header file, guarded by #ifdef __cplusplus.
            f.write(
                "// AUTO-GENERATED — DO NOT EDIT\n"
                "#pragma once\n"
                f"// No PayloadEqualityTraits specialization for {payloadTypeName}:\n"
                "// one or more fields have unsupported types (e.g., raw pointers).\n"
                "// To support recordOnChange() for this payload, add a manual\n"
                f"// PayloadEqualityTraits<{payloadTypeName}> specialization\n"
                "// in the payload header file, guarded by #ifdef __cplusplus, only\n"
                "// when the comparison semantics can be defined safely.\n"
            )
