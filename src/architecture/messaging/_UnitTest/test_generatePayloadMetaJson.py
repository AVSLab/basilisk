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
"""
Unit tests for generatePayloadMetaJson.py
"""
from __future__ import annotations

import sys
import textwrap
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "msgAutoSource"))
from generatePayloadMetaJson import _extractComment, _isIncomplete, parseStruct


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _h(src: str) -> str:
    """Dedent and strip leading newline from an inline header string."""
    return textwrap.dedent(src).lstrip("\n")


@pytest.fixture()
def hfile(tmp_path):
    """Return a factory: write header content to tmp_path/<name> and return its Path."""
    def _write(content: str, name: str = "TestMsg.h") -> Path:
        p = tmp_path / name
        p.write_text(_h(content))
        return p
    return _write


class _Cursor:
    """Minimal cursor stub for testing _extractComment in isolation."""
    def __init__(self, raw_comment):
        self.raw_comment = raw_comment


# ===========================================================================
# _extractComment - pure-function tests
# ===========================================================================

class TestExtractComment:
    def test_none_returns_empty(self):
        """A cursor with ``raw_comment=None`` must return an empty string."""
        assert _extractComment(_Cursor(None)) == ""

    def test_doxygen_exclamation_inline(self):
        """``//!<`` inline Doxygen comment is stripped to its payload text."""
        assert _extractComment(_Cursor("//!< [N] thrust")) == "[N] thrust"

    def test_doxygen_triple_slash_inline(self):
        """``///<`` inline triple-slash comment is stripped to its payload text."""
        assert _extractComment(_Cursor("///< [s] isp")) == "[s] isp"

    def test_cpp_triple_slash(self):
        """``///`` non-inline triple-slash comment is stripped to its payload text."""
        assert _extractComment(_Cursor("/// description here")) == "description here"

    def test_c_block_comment(self):
        """A plain ``/* ... */`` block comment is unwrapped to its inner text."""
        assert _extractComment(_Cursor("/* brief desc */")) == "brief desc"

    def test_doxygen_block_exclamation(self):
        """A ``/*! ... */`` Doxygen block comment is unwrapped to its inner text."""
        assert _extractComment(_Cursor("/*! value */")) == "value"

    def test_doxygen_block_inline(self):
        """A ``/*!< ... */`` inline Doxygen block comment is unwrapped to its inner text."""
        assert _extractComment(_Cursor("/*!< inline block */")) == "inline block"

    def test_multiline_joined(self):
        """Multiple ``//!<`` continuation lines are joined with a single space."""
        raw = "//!< first line\n//!< second line"
        assert _extractComment(_Cursor(raw)) == "first line second line"

    def test_blank_continuation_lines_skipped(self):
        """Blank ``//!<`` continuation lines are skipped when joining."""
        raw = "//!< first\n//!< \n//!< third"
        assert _extractComment(_Cursor(raw)) == "first third"

    def test_multiline_block(self):
        """A multi-line ``/*! ... */`` block comment is collapsed to a single string."""
        raw = "/*! multi\n * line\n */"
        assert _extractComment(_Cursor(raw)) == "multi line"


# ===========================================================================
# _isIncomplete - pure-function tests
# ===========================================================================

class TestIsIncomplete:
    def test_empty_field_list(self):
        """An empty field list is considered complete (no unknown fields)."""
        assert _isIncomplete([]) is False

    def test_all_primitives(self):
        """A list of only primitive fields is complete."""
        assert _isIncomplete([{"kind": "primitive"}, {"kind": "primitive"}]) is False

    def test_enum_field(self):
        """A single enum field is complete."""
        assert _isIncomplete([{"kind": "enum"}]) is False

    def test_single_unknown(self):
        """A single unknown field marks the struct incomplete."""
        assert _isIncomplete([{"kind": "unknown"}]) is True

    def test_unknown_at_end(self):
        """An unknown field at the end of the list marks the struct incomplete."""
        assert _isIncomplete([{"kind": "primitive"}, {"kind": "unknown"}]) is True

    def test_nested_struct_all_known(self):
        """A nested struct whose own fields are all known is complete."""
        fields = [{"kind": "struct", "fields": [{"kind": "primitive"}]}]
        assert _isIncomplete(fields) is False

    def test_nested_struct_has_unknown(self):
        """A nested struct with an unknown inner field propagates incompleteness."""
        fields = [{"kind": "struct", "fields": [{"kind": "unknown"}]}]
        assert _isIncomplete(fields) is True

    def test_nested_struct_empty_fields_is_complete(self):
        """An empty inner struct (zero fields) must return False, not raise.

        Regression: the old ``or`` chain would evaluate ``[]`` as falsy and fall through.
        """
        assert _isIncomplete([{"kind": "struct", "fields": []}]) is False

    def test_array_with_primitive_element(self):
        """An array whose element is a primitive is complete."""
        assert _isIncomplete([{"kind": "array", "element": {"kind": "primitive"}}]) is False

    def test_array_with_unknown_element(self):
        """An array whose element is unknown marks the struct incomplete."""
        assert _isIncomplete([{"kind": "array", "element": {"kind": "unknown"}}]) is True

    def test_deeply_nested_struct_in_array(self):
        """An unknown element buried inside a struct-in-array propagates incompleteness."""
        fields = [{
            "kind": "struct",
            "fields": [{"kind": "array", "element": {"kind": "unknown"}}],
        }]
        assert _isIncomplete(fields) is True


# ===========================================================================
# parseStruct - primitive types
# ===========================================================================

class TestPrimitives:
    def test_double(self, hfile):
        """A ``double`` field is classified as primitive with 8-byte size at offset 0."""
        meta = parseStruct(str(hfile("typedef struct { double x; } M;")), "M")
        f = meta["fields"][0]
        assert f["kind"] == "primitive"
        assert f["ctype"] == "double"
        assert f["size_bytes"] == 8
        assert f["offset_bytes"] == 0

    def test_float(self, hfile):
        """A ``float`` field is classified as primitive with 4-byte size."""
        f = parseStruct(str(hfile("typedef struct { float x; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["size_bytes"] == 4

    def test_int(self, hfile):
        """A plain ``int`` field is classified as primitive with ctype ``"int"``."""
        f = parseStruct(str(hfile("typedef struct { int n; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["ctype"] == "int"

    def test_bool(self, hfile):
        """A ``bool`` field is classified as primitive with 1-byte size."""
        f = parseStruct(str(hfile("typedef struct { bool b; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["size_bytes"] == 1

    def test_uint32_from_stub(self, hfile):
        """``uint32_t`` (typedef alias from the bundled stdint.h stub) resolves to a 4-byte primitive."""
        f = parseStruct(str(hfile("typedef struct { uint32_t n; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["ctype"] == "uint32_t"
        assert f["size_bytes"] == 4

    def test_int64_from_stub(self, hfile):
        """``int64_t`` from the bundled stdint.h stub is classified as primitive."""
        f = parseStruct(str(hfile("typedef struct { int64_t n; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["ctype"] == "int64_t"


# ===========================================================================
# parseStruct - multi-word types
# Regression: previous code compared only tokens[0] to typ.spelling, which
# misfired on "unsigned int", "long long", "long double", etc.
# ===========================================================================

class TestMultiWordTypes:
    def test_unsigned_int(self, hfile):
        """``unsigned int`` must be classified as primitive, not unknown."""
        f = parseStruct(str(hfile("typedef struct { unsigned int n; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive", "unsigned int must NOT be classified as unknown"
        assert f["ctype"] == "unsigned int"

    def test_long_long(self, hfile):
        """``long long`` must be classified as primitive, not unknown."""
        f = parseStruct(str(hfile("typedef struct { long long n; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive", "long long must NOT be classified as unknown"
        assert f["ctype"] == "long long"

    def test_long_double(self, hfile):
        """``long double`` must be classified as primitive, not unknown."""
        f = parseStruct(str(hfile("typedef struct { long double n; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive", "long double must NOT be classified as unknown"
        assert f["ctype"] == "long double"

    def test_unsigned_long_long(self, hfile):
        """``unsigned long long`` must be classified as primitive with the full multi-word ctype."""
        f = parseStruct(str(hfile("typedef struct { unsigned long long n; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["ctype"] == "unsigned long long"


# ===========================================================================
# parseStruct - typedef aliases
# A typedef alias for a primitive preserves the written name as ctype but
# resolves to the underlying numpy dtype through the canonical type.
# ===========================================================================

class TestTypeAliases:
    def test_typedef_alias_to_int(self, hfile):
        """A typedef alias for ``int`` preserves the alias name as ctype while using canonical size."""
        src = """
            typedef int MyIndex;
            typedef struct { MyIndex idx; } M;
        """
        f = parseStruct(str(hfile(src)), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["ctype"] == "MyIndex"      # preserves the written alias name
        assert f["size_bytes"] == 4         # size still comes from canonical type

    def test_typedef_alias_to_double(self, hfile):
        """A typedef alias for ``double`` preserves the alias name and resolves to 8-byte size."""
        src = """
            typedef double Scalar;
            typedef struct { Scalar val; } M;
        """
        f = parseStruct(str(hfile(src)), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["ctype"] == "Scalar"
        assert f["size_bytes"] == 8

    def test_typedef_alias_to_unsigned_short(self, hfile):
        """A typedef alias for ``unsigned short`` resolves to a 2-byte primitive."""
        src = """
            typedef unsigned short Port;
            typedef struct { Port p; } M;
        """
        f = parseStruct(str(hfile(src)), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["size_bytes"] == 2


# ===========================================================================
# parseStruct - arrays
# ===========================================================================

class TestArrays:
    def test_1d_double(self, hfile):
        """A 1-D ``double`` array is parsed with kind ``"array"``, shape ``[3]``, and correct sizes."""
        f = parseStruct(str(hfile("typedef struct { double v[3]; } M;")), "M")["fields"][0]
        assert f["kind"] == "array"
        assert f["shape"] == [3]
        assert f["element"]["kind"] == "primitive"
        assert f["element"]["ctype"] == "double"
        assert f["size_bytes"] == 24
        assert f["offset_bytes"] == 0

    def test_2d_double(self, hfile):
        """A 2-D ``double`` array is parsed with shape ``[3, 3]`` and total size 72 bytes."""
        f = parseStruct(str(hfile("typedef struct { double m[3][3]; } M;")), "M")["fields"][0]
        assert f["kind"] == "array"
        assert f["shape"] == [3, 3]
        assert f["size_bytes"] == 72

    def test_1d_int(self, hfile):
        """A 1-D ``int`` array carries the correct element ctype."""
        f = parseStruct(str(hfile("typedef struct { int ids[8]; } M;")), "M")["fields"][0]
        assert f["kind"] == "array"
        assert f["shape"] == [8]
        assert f["element"]["ctype"] == "int"

    def test_array_size_from_macro(self, tmp_path):
        """An array whose size is given by a ``#define`` from an included header resolves correctly."""
        (tmp_path / "defs.h").write_text("#pragma once\n#define N_ITEMS 4\n")
        (tmp_path / "M.h").write_text(
            '#include "defs.h"\ntypedef struct { double v[N_ITEMS]; } M;\n'
        )
        f = parseStruct(str(tmp_path / "M.h"), "M", compiler_args=[f"-I{tmp_path}"])["fields"][0]
        assert f["kind"] == "array"
        assert f["shape"] == [4]


# ===========================================================================
# parseStruct - enum fields
# ===========================================================================

class TestEnums:
    def test_basic_enum(self, hfile):
        """A typedef'd enum field is classified with kind ``"enum"`` and its enumerator values."""
        src = """
            typedef enum { OFF = 0, LOW = 1, HIGH = 2 } PowerLevel;
            typedef struct { PowerLevel level; } M;
        """
        f = parseStruct(str(hfile(src)), "M")["fields"][0]
        assert f["kind"] == "enum"
        assert f["typeName"] == "PowerLevel"
        assert f["size_bytes"] == 4
        assert f["values"] == {"OFF": 0, "LOW": 1, "HIGH": 2}

    def test_enum_with_negative_value(self, hfile):
        """An enum with a negative enumerator value is represented faithfully."""
        src = """
            typedef enum { ERR = -1, OK = 0 } Status;
            typedef struct { Status s; } M;
        """
        f = parseStruct(str(hfile(src)), "M")["fields"][0]
        assert f["kind"] == "enum"
        assert f["values"]["ERR"] == -1
        assert f["values"]["OK"] == 0


# ===========================================================================
# parseStruct - pointer fields
# ===========================================================================

class TestPointers:
    def test_int_pointer(self, hfile):
        """An ``int *`` field is classified as a pointer with the correct pointee type."""
        f = parseStruct(str(hfile("typedef struct { int *p; } M;")), "M")["fields"][0]
        assert f["kind"] == "pointer"
        assert f["ctype"] == "int *"
        assert f["pointee"] == "int"
        assert f["size_bytes"] == 8

    def test_void_pointer(self, hfile):
        """A ``void *`` field is classified as pointer with 8-byte size."""
        f = parseStruct(str(hfile("typedef struct { void *p; } M;")), "M")["fields"][0]
        assert f["kind"] == "pointer"
        assert f["size_bytes"] == 8


# ===========================================================================
# parseStruct - nested structs
# ===========================================================================

class TestNestedStructs:
    def test_nested_same_header(self, hfile):
        """A nested struct defined in the same header is inlined with its own fields."""
        src = """
            typedef struct { double x; double y; double z; } Vec3;
            typedef struct { Vec3 pos; Vec3 vel; } M;
        """
        meta = parseStruct(str(hfile(src)), "M")
        pos = meta["fields"][0]
        assert pos["kind"] == "struct"
        assert pos["typeName"] == "Vec3"
        assert len(pos["fields"]) == 3
        assert pos["fields"][0]["name"] == "x"
        assert meta["complete"] is True

    def test_nested_from_included_header(self, tmp_path):
        """A nested struct pulled in via ``#include`` is resolved and inlined correctly."""
        (tmp_path / "vec.h").write_text(
            "typedef struct { double x; double y; } Vec2;\n"
        )
        (tmp_path / "M.h").write_text(
            '#include "vec.h"\ntypedef struct { Vec2 origin; double z; } M;\n'
        )
        meta = parseStruct(str(tmp_path / "M.h"), "M", compiler_args=[f"-I{tmp_path}"])
        origin = meta["fields"][0]
        assert origin["kind"] == "struct"
        assert origin["typeName"] == "Vec2"
        assert len(origin["fields"]) == 2
        assert meta["complete"] is True

    def test_macro_from_included_header(self, tmp_path):
        """A ``#define`` array-size macro from an included header is evaluated correctly."""
        (tmp_path / "config.h").write_text("#pragma once\n#define MAX_N 36\n")
        (tmp_path / "M.h").write_text(
            '#include "config.h"\ntypedef struct { double data[MAX_N]; } M;\n'
        )
        meta = parseStruct(str(tmp_path / "M.h"), "M", compiler_args=[f"-I{tmp_path}"])
        f = meta["fields"][0]
        assert f["kind"] == "array"
        assert f["shape"] == [36]
        assert meta["complete"] is True

    def test_array_of_structs(self, hfile):
        """An array whose element is a struct carries kind ``"struct"`` and typeName in the element dict."""
        src = """
            typedef struct { double x; double y; } Point;
            typedef struct { Point pts[4]; } M;
        """
        meta = parseStruct(str(hfile(src)), "M")
        f = meta["fields"][0]
        assert f["kind"] == "array"
        assert f["shape"] == [4]
        assert f["element"]["kind"] == "struct"
        assert f["element"]["typeName"] == "Point"
        assert meta["complete"] is True

    def test_incomplete_nested_propagates_to_parent(self, hfile):
        """An unknown field inside a nested struct marks the outer struct incomplete."""
        src = """
            struct Fwd;
            typedef struct { struct Fwd val; int ok; } Inner;
            typedef struct { Inner inner; } Outer;
        """
        assert parseStruct(str(hfile(src)), "Outer")["complete"] is False


# ===========================================================================
# parseStruct - unresolvable / incomplete types
# ===========================================================================

class TestUnresolvableTypes:
    def test_forward_declared_struct_becomes_unknown(self, hfile):
        """A forward-declared but never-defined struct produces an unknown field without raising.

        The sibling field must still resolve normally and ``complete`` must be ``False``.
        """
        src = """
            struct Opaque;
            typedef struct {
                struct Opaque val;  //!< incomplete-type field
                int ok;
            } M;
        """
        meta = parseStruct(str(hfile(src)), "M")
        val_f, ok_f = meta["fields"]
        assert val_f["kind"] == "unknown"
        assert val_f["name"] == "val"
        assert ok_f["kind"] == "primitive"   # sibling field still resolves
        assert meta["complete"] is False

    def test_pointer_to_forward_decl_is_known(self, hfile):
        """A pointer to an incomplete type always resolves to an 8-byte pointer field."""
        src = """
            struct Opaque;
            typedef struct { struct Opaque *ptr; } M;
        """
        f = parseStruct(str(hfile(src)), "M")["fields"][0]
        assert f["kind"] == "pointer"
        assert f["size_bytes"] == 8

    def test_unknown_typename_in_target_raises(self, hfile):
        """A completely undefined type name in the target file triggers a ``ValueError``.

        ``_check_diagnostics`` fires on ``"unknown type name"`` diagnostics originating
        in the target file itself, so the parser raises rather than silently emitting
        unknown fields.
        """
        p = hfile("typedef struct { NonExistentType x; int ok; } M;")
        with pytest.raises(ValueError, match="Unresolvable field types"):
            parseStruct(str(p), "M")


# ===========================================================================
# parseStruct - comment extraction
# ===========================================================================

class TestFieldComments:
    def test_doxygen_exclamation(self, hfile):
        """A ``//!<`` field comment is captured in the ``"comment"`` key."""
        p = hfile("typedef struct { double thrust; //!< [N] thrust force\n} M;")
        assert parseStruct(str(p), "M")["fields"][0]["comment"] == "[N] thrust force"

    def test_doxygen_triple_slash(self, hfile):
        """A ``///<`` field comment is captured in the ``"comment"`` key."""
        p = hfile("typedef struct { double isp; ///< [s] specific impulse\n} M;")
        assert parseStruct(str(p), "M")["fields"][0]["comment"] == "[s] specific impulse"

    def test_block_comment(self, hfile):
        """A ``/*!< ... */`` inline block comment is captured in the ``"comment"`` key."""
        p = hfile("typedef struct { double x; /*!< x position */ } M;")
        assert parseStruct(str(p), "M")["fields"][0]["comment"] == "x position"

    def test_no_comment_is_empty_string(self, hfile):
        """A field with no comment produces an empty ``"comment"`` string."""
        assert parseStruct(str(hfile("typedef struct { double x; } M;")), "M")["fields"][0]["comment"] == ""


# ===========================================================================
# parseStruct - struct lookup variants
# ===========================================================================

class TestStructLookup:
    def test_named_struct_tag_same_as_typedef(self, hfile):
        """A struct where the tag and typedef share the same name is found by that name."""
        meta = parseStruct(str(hfile("typedef struct MyMsg { double x; } MyMsg;")), "MyMsg")
        assert meta["name"] == "MyMsg"
        assert len(meta["fields"]) == 1

    def test_anonymous_struct_typedef(self, hfile):
        """An anonymous struct (no tag) is found by its typedef name."""
        meta = parseStruct(str(hfile("typedef struct { double x; double y; } AnonMsg;")), "AnonMsg")
        assert meta["name"] == "AnonMsg"
        assert len(meta["fields"]) == 2

    def test_struct_not_found_raises(self, hfile):
        """Requesting a struct name that does not exist in the header raises ``ValueError``."""
        p = hfile("typedef struct { int x; } Foo;")
        with pytest.raises(ValueError, match="Bar"):
            parseStruct(str(p), "Bar")

    def test_source_file_and_kind_in_metadata(self, hfile):
        """The top-level metadata dict includes the source filename and ``kind="struct"``."""
        p = hfile("typedef struct { int x; } M;", name="MyPayload.h")
        meta = parseStruct(str(p), "M")
        assert meta["source_file"] == "MyPayload.h"
        assert meta["kind"] == "struct"


# ===========================================================================
# parseStruct - memory layout (offsets & sizes)
# ===========================================================================

class TestLayout:
    def test_padding_between_int_and_double(self, hfile):
        """On x86-64, ``int`` + 4-byte pad + ``double`` produces offsets 0/8 and total size 16."""
        meta = parseStruct(str(hfile("typedef struct { int a; double b; } M;")), "M")
        a, b = meta["fields"]
        assert a["offset_bytes"] == 0
        assert a["size_bytes"] == 4
        assert b["offset_bytes"] == 8
        assert b["size_bytes"] == 8
        assert meta["size_bytes"] == 16

    def test_complete_true_all_primitives(self, hfile):
        """A struct with only primitive fields is marked ``complete=True``."""
        meta = parseStruct(str(hfile("typedef struct { double x; int n; } M;")), "M")
        assert meta["complete"] is True

    def test_complete_false_has_unknown(self, hfile):
        """A struct containing an unresolvable field is marked ``complete=False``."""
        src = "struct Fwd; typedef struct { struct Fwd f; } M;"
        assert parseStruct(str(hfile(src)), "M")["complete"] is False


# ===========================================================================
# parseStruct - C++ mode
# ===========================================================================

class TestCppMode:
    def test_default_member_initializer_scalar(self, hfile):
        """C++ default member initializers on scalar fields do not confuse type classification."""
        src = """
            typedef struct {
                double force = 0.0;   //!< [N] force magnitude
                int    count = 0;     //!< item count
            } M;
        """
        meta = parseStruct(str(hfile(src)), "M", compiler_args=["-x", "c++"])
        force, count = meta["fields"]
        assert force["kind"] == "primitive"
        assert force["ctype"] == "double"
        assert count["kind"] == "primitive"
        assert count["ctype"] == "int"

    def test_default_member_initializer_array(self, hfile):
        """C++ default member initializers on array fields do not prevent array classification."""
        src = """
            typedef struct {
                double v[3] = {0.0};  //!< direction vector
            } M;
        """
        meta = parseStruct(str(hfile(src)), "M", compiler_args=["-x", "c++"])
        f = meta["fields"][0]
        assert f["kind"] == "array"
        assert f["shape"] == [3]
        assert meta["complete"] is True

    def test_nested_struct_from_cpp_header(self, tmp_path):
        """A C++ struct included from another header is resolved and inlined as a nested struct."""
        (tmp_path / "state.h").write_text(
            "struct State { double pos[3]; double vel[3]; };\n"
        )
        (tmp_path / "M.h").write_text(
            '#include "state.h"\ntypedef struct { State s; } M;\n'
        )
        meta = parseStruct(
            str(tmp_path / "M.h"), "M",
            compiler_args=["-x", "c++", f"-I{tmp_path}"],
        )
        s = meta["fields"][0]
        assert s["kind"] == "struct"
        assert s["typeName"] == "State"
        assert meta["complete"] is True

    def test_unresolvable_cpp_type_never_silently_primitive(self, hfile):
        """An undeclared C++ namespaced type must not be silently classified as primitive.

        If ``_check_diagnostics`` fires, a ``ValueError`` is acceptable.  Otherwise the
        field must be ``unknown`` and ``complete`` must be ``False``.
        """
        src = "typedef struct { Eigen::Vector3d v; } M;"
        try:
            meta = parseStruct(str(hfile(src)), "M", compiler_args=["-x", "c++"])
            # Did not raise → the field must be unknown, not a silently wrong primitive
            assert meta["complete"] is False
            assert meta["fields"][0]["kind"] == "unknown"
        except ValueError:
            pass  # _check_diagnostics fired - also an acceptable outcome
