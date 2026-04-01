"""
generatePayloadMetaJson.py, Parse a C/C++ header with libclang and emit struct metadata as JSON.

Usage (CLI):
    python generatePayloadMetaJson.py <header.h> <TargetStructName> [output.json]
                                      [-- -Iinclude/path -DSOME_MACRO=1 ...]

Usage (API):
    from generatePayloadMetaJson import parseStruct
    meta = parseStruct("MyMsg.h", "MyMsgPayload", compiler_args=["-I./include"])
    # meta is a plain dict, ready for json.dumps()
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
from pathlib import Path

import clang.cindex
from clang.cindex import CursorKind, TypeKind

# ---------------------------------------------------------------------------
# Stub headers
#
# Written to a temp dir and injected via -I so the parser works with only
# `pip install libclang` and no system clang installation.
# Headers irrelevant to struct layout (stdio, stdarg, ...) are not stubbed;
# missing-file errors for them are filtered in _checkDiagnostics.
# ---------------------------------------------------------------------------

_STUBS: dict[str, str] = {
    "stdint.h": """\
#pragma once
typedef signed   char       int8_t;
typedef signed   short      int16_t;
typedef signed   int        int32_t;
typedef signed   long long  int64_t;
typedef unsigned char       uint8_t;
typedef unsigned short      uint16_t;
typedef unsigned int        uint32_t;
typedef unsigned long long  uint64_t;
typedef int8_t   int_least8_t;   typedef uint8_t   uint_least8_t;
typedef int16_t  int_least16_t;  typedef uint16_t  uint_least16_t;
typedef int32_t  int_least32_t;  typedef uint32_t  uint_least32_t;
typedef int64_t  int_least64_t;  typedef uint64_t  uint_least64_t;
typedef int8_t   int_fast8_t;    typedef uint8_t   uint_fast8_t;
typedef int32_t  int_fast16_t;   typedef uint32_t  uint_fast16_t;
typedef int32_t  int_fast32_t;   typedef uint32_t  uint_fast32_t;
typedef int64_t  int_fast64_t;   typedef uint64_t  uint_fast64_t;
typedef long long          intmax_t;
typedef unsigned long long uintmax_t;
typedef long long          intptr_t;
typedef unsigned long long uintptr_t;
#define UINT64_MAX 18446744073709551615ULL
#define INT64_MAX  9223372036854775807LL
#define UINT32_MAX 4294967295U
#define INT32_MAX  2147483647
""",
    "stdbool.h": """\
#pragma once
#ifndef __cplusplus
  typedef _Bool bool;
  #define true  1
  #define false 0
#endif
""",
    "float.h": """\
#pragma once
#define FLT_MAX      3.40282346638528859812e+38F
#define DBL_MAX      1.79769313486231570815e+308
#define FLT_MIN      1.17549435082228750797e-38F
#define DBL_MIN      2.22507385850720138309e-308
#define DBL_MANT_DIG 53
#define FLT_EPSILON  1.19209289550781250000e-7F
#define DBL_EPSILON  2.22044604925031308085e-16
""",
    "stddef.h": """\
#pragma once
typedef long          ptrdiff_t;
typedef unsigned long size_t;
#define NULL     ((void*)0)
#define offsetof(type, member) __builtin_offsetof(type, member)
""",
    "inttypes.h": "#pragma once\n#include <stdint.h>\n",
    "stdlib.h":   "#pragma once\n#include <stddef.h>\n#include <stdint.h>\n",
    "string.h":   "#pragma once\n#include <stddef.h>\n",
    "math.h":     "#pragma once\n#include <float.h>\n#define NAN (__builtin_nanf(\"\"))\n#define INFINITY (__builtin_inff())\n",
    "limits.h": """\
#pragma once
#define CHAR_BIT  8
#define INT_MIN   (-2147483648)
#define INT_MAX   2147483647
#define UINT_MAX  4294967295U
#define LONG_MAX  9223372036854775807L
#define ULONG_MAX 18446744073709551615UL
""",
}


class _StubIncludeDir:
    """Writes stub headers to a temp dir, provides the -I flag, cleans up on exit."""

    def __enter__(self) -> "_StubIncludeDir":
        self._tmpdir = tempfile.TemporaryDirectory(prefix="libclang_stubs_")
        d = Path(self._tmpdir.name)
        for name, content in _STUBS.items():
            (d / name).write_text(content)
        self.flag = f"-I{d}"
        return self

    def __exit__(self, *args):
        self._tmpdir.__exit__(*args)


# ---------------------------------------------------------------------------
# Numeric TypeKind set - used to classify primitive fields
# ---------------------------------------------------------------------------

_NUMERIC_TYPEKINDS: frozenset = frozenset({
    TypeKind.BOOL,
    TypeKind.CHAR_U,  TypeKind.UCHAR,   TypeKind.CHAR16,  TypeKind.USHORT,
    TypeKind.CHAR32,  TypeKind.UINT,    TypeKind.ULONG,   TypeKind.ULONGLONG,
    TypeKind.UINT128,
    TypeKind.CHAR_S,  TypeKind.SCHAR,   TypeKind.SHORT,   TypeKind.INT,
    TypeKind.LONG,    TypeKind.LONGLONG, TypeKind.INT128,
    TypeKind.FLOAT,   TypeKind.DOUBLE,  TypeKind.LONGDOUBLE,
})


# ---------------------------------------------------------------------------
# Diagnostic filtering
# ---------------------------------------------------------------------------

def _checkDiagnostics(diagnostics, header_path: str) -> None:
    """
    Raise for 'unknown type name' errors in the target file only.

    Missing system headers, errors in transitively included files, and
    warnings are all ignored, they do not affect struct layout.
    """
    realPath = os.path.realpath(header_path)
    fatal = [
        d.spelling for d in diagnostics
        if d.severity >= clang.cindex.Diagnostic.Error
        and "unknown type name" in d.spelling
        and d.location.file
        and os.path.realpath(d.location.file.name) == realPath
    ]
    if fatal:
        raise ValueError(
            f"Unresolvable field types in {header_path}:\n" +
            "\n".join(f"  {m}" for m in fatal)
        )


# ---------------------------------------------------------------------------
# Comment extraction
# ---------------------------------------------------------------------------

def _extractComment(cursor: clang.cindex.Cursor) -> str:
    """Return the doc comment for *cursor* as a single space-separated string."""
    rawComment = cursor.raw_comment
    if not rawComment:
        return ""
    lines = []
    for line in rawComment.splitlines():
        line = line.strip()
        for prefix in ("//!<", "///<", "//!", "///", "//", "/*!<", "/**", "/*!", "/*", "*/", "*"):
            if line.startswith(prefix):
                line = line[len(prefix):].strip()
                # Single-line block comments carry a trailing "*/"; strip it.
                if prefix.startswith("/*") and line.endswith("*/"):
                    line = line[:-2].strip()
                break
        if line:
            lines.append(line)
    return " ".join(lines)


# ---------------------------------------------------------------------------
# Token-based type extraction
# ---------------------------------------------------------------------------

def _writtenType(cursor: clang.cindex.Cursor, field_name: str) -> str | None:
    """Return the written type of a field declaration reconstructed from source tokens.

    Searches for the field-name token from the end of the token list (so that
    default initializers like ``= 0`` or ``= {}`` are excluded from the type
    region) and joins all preceding tokens without spaces.

    Returns ``None`` when tokens are unavailable, e.g. for the synthetic
    ``_FakeField`` cursors used for array elements, which have no source location.
    """
    if cursor.location.file is None:
        return None
    tokens = list(cursor.get_tokens())
    if not tokens:
        return None
    nameIdx = next(
        (i for i in range(len(tokens) - 1, -1, -1)
         if tokens[i].spelling == field_name),
        None,
    )
    if nameIdx is None or nameIdx == 0:
        return None
    return "".join(t.spelling for t in tokens[:nameIdx])


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

class FileRegistry:
    def __init__(self, filename: str):
        self.filename = os.path.realpath(filename)
        self.structs: dict[str, clang.cindex.Cursor] = {}
        self.enums:   dict[str, dict[str, int]] = {}

    def populate(self, tu_cursor: clang.cindex.Cursor) -> None:
        for cursor in tu_cursor.walk_preorder():
            if cursor.kind in (CursorKind.STRUCT_DECL, CursorKind.UNION_DECL):
                if cursor.is_definition():
                    name = self._resolve_name(cursor)
                    if name:
                        self.structs[name] = cursor
            elif cursor.kind == CursorKind.ENUM_DECL:
                if cursor.is_definition():
                    name = self._resolve_name(cursor)
                    if name:
                        self.enums[name] = {
                            c.spelling: c.enum_value
                            for c in cursor.get_children()
                            if c.kind == CursorKind.ENUM_CONSTANT_DECL
                        }

    def _resolve_name(self, cursor: clang.cindex.Cursor) -> str | None:
        if cursor.spelling:
            return cursor.spelling
        # Anonymous struct/enum, find the aliasing typedef
        for sibling in cursor.semantic_parent.get_children():
            if sibling.kind == CursorKind.TYPEDEF_DECL:
                decl = sibling.underlying_typedef_type.get_canonical().get_declaration()
                if decl == cursor or decl.canonical == cursor.canonical:
                    return sibling.spelling
        return None


# ---------------------------------------------------------------------------
# Field resolver
# ---------------------------------------------------------------------------

def _unknownField(name: str, ctype: str, offset: int, comment: str) -> dict:
    """A field whose type could not be resolved. Marks the parent struct incomplete."""
    return {
        "kind":         "unknown",
        "name":         name,
        "ctype":        ctype,
        "size_bytes":   None,   # libclang's value is unreliable for unknown types
        "offset_bytes": offset,
        "comment":      comment,
    }


def _isIncomplete(fields: list[dict]) -> bool:
    """Recursively check if any field (or nested field) is unknown."""
    for f in fields:
        if f["kind"] == "unknown":
            return True
        if f["kind"] == "struct" and _isIncomplete(f.get("fields", [])):
            return True
        if f["kind"] == "array" and _isIncomplete([f["element"]]):
            return True
    return False


class _FakeField:
    """Minimal cursor-like object used to resolve array element types.

    Array elements have no independent source location, so ``_writtenType``
    returns ``None`` for them and token-based checks are skipped automatically.
    ``get_tokens()`` is intentionally absent, callers must not invoke it.
    """

    class _Loc:
        file = None

    _loc = _Loc()

    def __init__(self, elem_type: clang.cindex.Type) -> None:
        self.spelling    = "__elem__"
        self.type        = elem_type
        self.raw_comment = ""
        self.location    = _FakeField._loc

    def get_field_offsetof(self) -> int:
        return 0


class StructParser:
    def __init__(self, registry: FileRegistry):
        self.registry = registry

    def parseStruct(self, cursor: clang.cindex.Cursor) -> list[dict]:
        return [
            self._resolveField(child)
            for child in cursor.get_children()
            if child.kind == CursorKind.FIELD_DECL
        ]

    def _resolveField(self, cursor) -> dict:  # cursor: Cursor | _FakeField
        name      = cursor.spelling
        typ       = cursor.type
        comment   = _extractComment(cursor)
        offset    = max(0, cursor.get_field_offsetof()) // 8
        rawSize   = typ.get_size()
        size      = max(0, rawSize)
        canonical = typ.get_canonical()

        if canonical.kind == TypeKind.CONSTANTARRAY:
            return self._array(name, typ, canonical, offset, size, comment)
        if canonical.kind == TypeKind.RECORD:
            return self._record(name, typ, canonical, offset, size, comment)
        if canonical.kind == TypeKind.ENUM:
            return self._enum(name, canonical, offset, size, comment)

        # UNEXPOSED/INVALID covers template instantiations whose headers are absent
        # (e.g. std::vector<double> without <vector>).  typ.spelling may be truncated
        # to the bare template name; reconstruct the full written form from tokens.
        if rawSize < 0 or canonical.kind in (TypeKind.UNEXPOSED, TypeKind.INVALID):
            return _unknownField(name, _writtenType(cursor, name) or typ.spelling,
                                 offset, comment)

        # Detect silent type substitution: libclang maps unknown types to int when
        # their header is missing.  A mismatch between written and resolved spelling
        # exposes the substitution (e.g. Eigen::Vector3d silently becoming int).
        written = _writtenType(cursor, name)
        if written is not None and written != "".join(typ.spelling.split()):
            return _unknownField(name, written, offset, comment)

        if canonical.kind in _NUMERIC_TYPEKINDS:
            return {"kind": "primitive", "name": name, "ctype": typ.spelling,
                    "size_bytes": size, "offset_bytes": offset, "comment": comment}

        if canonical.kind == TypeKind.POINTER:
            return {"kind": "pointer", "name": name,
                    "ctype": typ.spelling,
                    "pointee": canonical.get_pointee().spelling,
                    "size_bytes": size, "offset_bytes": offset, "comment": comment}

        return _unknownField(name, typ.spelling, offset, comment)

    def _array(self, name: str, typ: clang.cindex.Type, canonical: clang.cindex.Type,
               offset: int, size: int, comment: str) -> dict:
        shape     = []
        elemOrig  = typ        # non-canonical walk: preserves typedef names (e.g. int64_t)
        elemCanon = canonical  # canonical walk: drives dimension and kind checks

        while elemCanon.kind == TypeKind.CONSTANTARRAY:
            shape.append(elemCanon.element_count)
            # Peel one array level.  Avoid get_canonical() on elemOrig so typedef
            # names survive (int64_t stays int64_t instead of expanding to long long).
            elemOrig  = (elemOrig.get_canonical() if elemOrig.kind == TypeKind.TYPEDEF
                         else elemOrig).element_type
            elemCanon = elemCanon.element_type.get_canonical()

        return {"kind": "array", "name": name, "shape": shape,
                "element": self._resolveField(_FakeField(elemOrig)),
                "size_bytes": size, "offset_bytes": offset, "comment": comment}

    def _record(self, name: str, typ: clang.cindex.Type, canonical: clang.cindex.Type,
                offset: int, size: int, comment: str) -> dict:
        decl     = canonical.get_declaration()
        typeName = decl.spelling or typ.spelling
        lookup   = typeName.removeprefix("struct ").removeprefix("union ").strip()

        if lookup in self.registry.structs:
            fields = self.parseStruct(self.registry.structs[lookup])
            return {"kind": "struct", "name": name, "typeName": lookup,
                    "fields": fields, "size_bytes": size, "offset_bytes": offset,
                    "comment": comment}

        # Use the full qualified spelling (e.g. "std::vector<double>") rather than
        # the bare declaration name ("vector") so downstream tools get a useful string.
        return _unknownField(name, typ.spelling or typeName, offset, comment)

    def _enum(self, name: str, canonical: clang.cindex.Type,
              offset: int, size: int, comment: str) -> dict:
        typeName = canonical.get_declaration().spelling

        if size not in {1, 2, 4, 8}:
            return _unknownField(name, typeName, offset, comment)

        return {"kind": "enum", "name": name, "typeName": typeName,
                "size_bytes": size, "offset_bytes": offset,
                "values": self.registry.enums.get(typeName, {}), "comment": comment}


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def parseStruct(
    headerPath: str,
    targetStruct: str,
    compiler_args: list[str] | None = None,
) -> dict:
    headerPath = str(Path(headerPath).resolve())

    with _StubIncludeDir() as stubs:
        args = [
            stubs.flag,
            "-include", "stdint.h",
            "-include", "stdbool.h",
            "-include", "float.h",
        ] + list(compiler_args or [])

        index = clang.cindex.Index.create()
        tu = index.parse(
            headerPath,
            args=args,
            options=clang.cindex.TranslationUnit.PARSE_DETAILED_PROCESSING_RECORD,
        )

        _checkDiagnostics(tu.diagnostics, headerPath)

        registry = FileRegistry(headerPath)
        registry.populate(tu.cursor)

        cursor = registry.structs.get(targetStruct)
        if cursor is None:
            # Fallback for payloads declared as anonymous structs aliased via typedef:
            # FileRegistry stores them under the struct tag name, but the caller may
            # supply the typedef name.  Search typedef decls as a second pass.
            for c in tu.cursor.get_children():
                if c.kind == CursorKind.TYPEDEF_DECL and c.spelling == targetStruct:
                    underlying = c.underlying_typedef_type.get_canonical()
                    if underlying.kind == TypeKind.RECORD:
                        decl = underlying.get_declaration()
                        if decl.is_definition():
                            cursor = decl
                            break
        if cursor is None:
            raise ValueError(
                f"Struct {targetStruct!r} not found in {headerPath}.\n"
                f"Available: {sorted(registry.structs)}"
            )

        parser = StructParser(registry)
        fields = parser.parseStruct(cursor)

        return {
            "name":        targetStruct,
            "source_file": os.path.basename(headerPath),
            "kind":        "struct",
            "size_bytes":  max(0, cursor.type.get_size()),
            "complete":    not _isIncomplete(fields),
            "fields":      fields,
        }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    import argparse
    ap = argparse.ArgumentParser(description="Parse a C header and emit struct metadata as JSON.")
    ap.add_argument("header",        help="Path to the .h file")
    ap.add_argument("struct_name",   help="Name of the struct to export")
    ap.add_argument("output",        nargs="?", help="Output .json path (default: stdout)")
    ap.add_argument("compiler_args", nargs=argparse.REMAINDER, metavar="-- CLANG_ARGS")
    args = ap.parse_args()

    extra = args.compiler_args
    if extra and extra[0] == "--":
        extra = extra[1:]

    meta = parseStruct(args.header, args.struct_name, compiler_args=extra)
    outputJson = json.dumps(meta, indent=2)

    if args.output:
        Path(args.output).write_text(outputJson)
    else:
        print(outputJson)


if __name__ == "__main__":
    main()
