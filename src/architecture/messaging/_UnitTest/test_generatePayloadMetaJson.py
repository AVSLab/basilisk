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
        assert _extractComment(_Cursor(None)) == ""

    def test_doxygen_exclamation_inline(self):
        assert _extractComment(_Cursor("//!< [N] thrust")) == "[N] thrust"

    def test_doxygen_triple_slash_inline(self):
        assert _extractComment(_Cursor("///< [s] isp")) == "[s] isp"

    def test_cpp_triple_slash(self):
        assert _extractComment(_Cursor("/// description here")) == "description here"

    def test_c_block_comment(self):
        assert _extractComment(_Cursor("/* brief desc */")) == "brief desc"

    def test_doxygen_block_exclamation(self):
        assert _extractComment(_Cursor("/*! value */")) == "value"

    def test_doxygen_block_inline(self):
        assert _extractComment(_Cursor("/*!< inline block */")) == "inline block"

    def test_multiline_joined(self):
        raw = "//!< first line\n//!< second line"
        assert _extractComment(_Cursor(raw)) == "first line second line"

    def test_blank_continuation_lines_skipped(self):
        raw = "//!< first\n//!< \n//!< third"
        assert _extractComment(_Cursor(raw)) == "first third"

    def test_multiline_block(self):
        raw = "/*! multi\n * line\n */"
        assert _extractComment(_Cursor(raw)) == "multi line"


# ===========================================================================
# _isIncomplete - pure-function tests
# ===========================================================================

class TestIsIncomplete:
    def test_empty_field_list(self):
        assert _isIncomplete([]) is False

    def test_all_primitives(self):
        assert _isIncomplete([{"kind": "primitive"}, {"kind": "primitive"}]) is False

    def test_enum_field(self):
        assert _isIncomplete([{"kind": "enum"}]) is False

    def test_single_unknown(self):
        assert _isIncomplete([{"kind": "unknown"}]) is True

    def test_unknown_at_end(self):
        assert _isIncomplete([{"kind": "primitive"}, {"kind": "unknown"}]) is True

    def test_nested_struct_all_known(self):
        fields = [{"kind": "struct", "fields": [{"kind": "primitive"}]}]
        assert _isIncomplete(fields) is False

    def test_nested_struct_has_unknown(self):
        fields = [{"kind": "struct", "fields": [{"kind": "unknown"}]}]
        assert _isIncomplete(fields) is True

    def test_nested_struct_empty_fields_is_complete(self):
        # An empty inner struct has zero fields - must return False, not raise.
        # Regression: the old `or` chain would evaluate [] as falsy and fall through.
        assert _isIncomplete([{"kind": "struct", "fields": []}]) is False

    def test_array_with_primitive_element(self):
        assert _isIncomplete([{"kind": "array", "element": {"kind": "primitive"}}]) is False

    def test_array_with_unknown_element(self):
        assert _isIncomplete([{"kind": "array", "element": {"kind": "unknown"}}]) is True

    def test_deeply_nested_struct_in_array(self):
        # struct → array whose element is unknown
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
        meta = parseStruct(str(hfile("typedef struct { double x; } M;")), "M")
        f = meta["fields"][0]
        assert f["kind"] == "primitive"
        assert f["ctype"] == "double"
        assert f["size_bytes"] == 8
        assert f["offset_bytes"] == 0

    def test_float(self, hfile):
        f = parseStruct(str(hfile("typedef struct { float x; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["size_bytes"] == 4

    def test_int(self, hfile):
        f = parseStruct(str(hfile("typedef struct { int n; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["ctype"] == "int"

    def test_bool(self, hfile):
        f = parseStruct(str(hfile("typedef struct { bool b; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["size_bytes"] == 1

    def test_uint32_from_stub(self, hfile):
        # uint32_t is a typedef alias defined in the bundled stdint.h stub
        f = parseStruct(str(hfile("typedef struct { uint32_t n; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["ctype"] == "uint32_t"
        assert f["size_bytes"] == 4

    def test_int64_from_stub(self, hfile):
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
        f = parseStruct(str(hfile("typedef struct { unsigned int n; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive", "unsigned int must NOT be classified as unknown"
        assert f["ctype"] == "unsigned int"

    def test_long_long(self, hfile):
        f = parseStruct(str(hfile("typedef struct { long long n; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive", "long long must NOT be classified as unknown"
        assert f["ctype"] == "long long"

    def test_long_double(self, hfile):
        f = parseStruct(str(hfile("typedef struct { long double n; } M;")), "M")["fields"][0]
        assert f["kind"] == "primitive", "long double must NOT be classified as unknown"
        assert f["ctype"] == "long double"

    def test_unsigned_long_long(self, hfile):
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
        src = """
            typedef int MyIndex;
            typedef struct { MyIndex idx; } M;
        """
        f = parseStruct(str(hfile(src)), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["ctype"] == "MyIndex"      # preserves the written alias name
        assert f["size_bytes"] == 4         # size still comes from canonical type

    def test_typedef_alias_to_double(self, hfile):
        src = """
            typedef double Scalar;
            typedef struct { Scalar val; } M;
        """
        f = parseStruct(str(hfile(src)), "M")["fields"][0]
        assert f["kind"] == "primitive"
        assert f["ctype"] == "Scalar"
        assert f["size_bytes"] == 8

    def test_typedef_alias_to_unsigned_short(self, hfile):
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
        f = parseStruct(str(hfile("typedef struct { double v[3]; } M;")), "M")["fields"][0]
        assert f["kind"] == "array"
        assert f["shape"] == [3]
        assert f["element"]["kind"] == "primitive"
        assert f["element"]["ctype"] == "double"
        assert f["size_bytes"] == 24
        assert f["offset_bytes"] == 0

    def test_2d_double(self, hfile):
        f = parseStruct(str(hfile("typedef struct { double m[3][3]; } M;")), "M")["fields"][0]
        assert f["kind"] == "array"
        assert f["shape"] == [3, 3]
        assert f["size_bytes"] == 72

    def test_1d_int(self, hfile):
        f = parseStruct(str(hfile("typedef struct { int ids[8]; } M;")), "M")["fields"][0]
        assert f["kind"] == "array"
        assert f["shape"] == [8]
        assert f["element"]["ctype"] == "int"

    def test_array_size_from_macro(self, tmp_path):
        # Array size given by a #define from an included header
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
        src = """
            typedef enum { OFF = 0, LOW = 1, HIGH = 2 } PowerLevel;
            typedef struct { PowerLevel level; } M;
        """
        f = parseStruct(str(hfile(src)), "M")["fields"][0]
        assert f["kind"] == "enum"
        assert f["type_name"] == "PowerLevel"
        assert f["size_bytes"] == 4
        assert f["values"] == {"OFF": 0, "LOW": 1, "HIGH": 2}

    def test_enum_with_negative_value(self, hfile):
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
        f = parseStruct(str(hfile("typedef struct { int *p; } M;")), "M")["fields"][0]
        assert f["kind"] == "pointer"
        assert f["ctype"] == "int *"
        assert f["pointee"] == "int"
        assert f["size_bytes"] == 8

    def test_void_pointer(self, hfile):
        f = parseStruct(str(hfile("typedef struct { void *p; } M;")), "M")["fields"][0]
        assert f["kind"] == "pointer"
        assert f["size_bytes"] == 8


# ===========================================================================
# parseStruct - nested structs
# ===========================================================================

class TestNestedStructs:
    def test_nested_same_header(self, hfile):
        src = """
            typedef struct { double x; double y; double z; } Vec3;
            typedef struct { Vec3 pos; Vec3 vel; } M;
        """
        meta = parseStruct(str(hfile(src)), "M")
        pos = meta["fields"][0]
        assert pos["kind"] == "struct"
        assert pos["type_name"] == "Vec3"
        assert len(pos["fields"]) == 3
        assert pos["fields"][0]["name"] == "x"
        assert meta["complete"] is True

    def test_nested_from_included_header(self, tmp_path):
        (tmp_path / "vec.h").write_text(
            "typedef struct { double x; double y; } Vec2;\n"
        )
        (tmp_path / "M.h").write_text(
            '#include "vec.h"\ntypedef struct { Vec2 origin; double z; } M;\n'
        )
        meta = parseStruct(str(tmp_path / "M.h"), "M", compiler_args=[f"-I{tmp_path}"])
        origin = meta["fields"][0]
        assert origin["kind"] == "struct"
        assert origin["type_name"] == "Vec2"
        assert len(origin["fields"]) == 2
        assert meta["complete"] is True

    def test_macro_from_included_header(self, tmp_path):
        # A macro that defines array sizes comes from an included header
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
        # Array whose element is a struct (e.g. AccPktDataMsgPayload accPkts[120]).
        # The element dict has kind="struct"/type_name rather than ctype.
        src = """
            typedef struct { double x; double y; } Point;
            typedef struct { Point pts[4]; } M;
        """
        meta = parseStruct(str(hfile(src)), "M")
        f = meta["fields"][0]
        assert f["kind"] == "array"
        assert f["shape"] == [4]
        assert f["element"]["kind"] == "struct"
        assert f["element"]["type_name"] == "Point"
        assert meta["complete"] is True

    def test_incomplete_nested_propagates_to_parent(self, hfile):
        # An unknown field inside a nested struct marks the outer struct incomplete
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
        # A struct that is forward-declared but never defined has incomplete
        # type → the registry has no definition → _record falls back to unknown.
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
        # A pointer to an incomplete type is always 8 bytes - it resolves fine.
        src = """
            struct Opaque;
            typedef struct { struct Opaque *ptr; } M;
        """
        f = parseStruct(str(hfile(src)), "M")["fields"][0]
        assert f["kind"] == "pointer"
        assert f["size_bytes"] == 8

    def test_unknown_typename_in_target_raises(self, hfile):
        # A completely undefined type name directly in the target file triggers
        # _check_diagnostics ("unknown type name" in target file) → raises instead
        # of silently emitting unknown fields.
        p = hfile("typedef struct { NonExistentType x; int ok; } M;")
        with pytest.raises(ValueError, match="Unresolvable field types"):
            parseStruct(str(p), "M")


# ===========================================================================
# parseStruct - comment extraction
# ===========================================================================

class TestFieldComments:
    def test_doxygen_exclamation(self, hfile):
        p = hfile("typedef struct { double thrust; //!< [N] thrust force\n} M;")
        assert parseStruct(str(p), "M")["fields"][0]["comment"] == "[N] thrust force"

    def test_doxygen_triple_slash(self, hfile):
        p = hfile("typedef struct { double isp; ///< [s] specific impulse\n} M;")
        assert parseStruct(str(p), "M")["fields"][0]["comment"] == "[s] specific impulse"

    def test_block_comment(self, hfile):
        p = hfile("typedef struct { double x; /*!< x position */ } M;")
        assert parseStruct(str(p), "M")["fields"][0]["comment"] == "x position"

    def test_no_comment_is_empty_string(self, hfile):
        assert parseStruct(str(hfile("typedef struct { double x; } M;")), "M")["fields"][0]["comment"] == ""


# ===========================================================================
# parseStruct - struct lookup variants
# ===========================================================================

class TestStructLookup:
    def test_named_struct_tag_same_as_typedef(self, hfile):
        # Basilisk pattern: tag and typedef share the same name
        meta = parseStruct(str(hfile("typedef struct MyMsg { double x; } MyMsg;")), "MyMsg")
        assert meta["name"] == "MyMsg"
        assert len(meta["fields"]) == 1

    def test_anonymous_struct_typedef(self, hfile):
        # No struct tag - only the typedef name
        meta = parseStruct(str(hfile("typedef struct { double x; double y; } AnonMsg;")), "AnonMsg")
        assert meta["name"] == "AnonMsg"
        assert len(meta["fields"]) == 2

    def test_struct_not_found_raises(self, hfile):
        p = hfile("typedef struct { int x; } Foo;")
        with pytest.raises(ValueError, match="Bar"):
            parseStruct(str(p), "Bar")

    def test_source_file_and_kind_in_metadata(self, hfile):
        p = hfile("typedef struct { int x; } M;", name="MyPayload.h")
        meta = parseStruct(str(p), "M")
        assert meta["source_file"] == "MyPayload.h"
        assert meta["kind"] == "struct"


# ===========================================================================
# parseStruct - memory layout (offsets & sizes)
# ===========================================================================

class TestLayout:
    def test_padding_between_int_and_double(self, hfile):
        # x86-64: int(4) + 4-byte pad + double(8) → total 16
        meta = parseStruct(str(hfile("typedef struct { int a; double b; } M;")), "M")
        a, b = meta["fields"]
        assert a["offset_bytes"] == 0
        assert a["size_bytes"] == 4
        assert b["offset_bytes"] == 8
        assert b["size_bytes"] == 8
        assert meta["size_bytes"] == 16

    def test_complete_true_all_primitives(self, hfile):
        meta = parseStruct(str(hfile("typedef struct { double x; int n; } M;")), "M")
        assert meta["complete"] is True

    def test_complete_false_has_unknown(self, hfile):
        src = "struct Fwd; typedef struct { struct Fwd f; } M;"
        assert parseStruct(str(hfile(src)), "M")["complete"] is False


# ===========================================================================
# parseStruct - C++ mode
# ===========================================================================

class TestCppMode:
    def test_default_member_initializer_scalar(self, hfile):
        # C++ allows `= expr` in struct fields.  The initializer must not
        # confuse the token-based type-substitution check.
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
        assert s["type_name"] == "State"
        assert meta["complete"] is True

    def test_unresolvable_cpp_type_never_silently_primitive(self, hfile):
        # A namespaced C++ type with no declaration anywhere in the TU.
        # libclang cannot resolve Eigen::Vector3d → the field must be classified
        # as unknown.  Depending on Clang's diagnostic wording the parser may
        # either raise (if the diagnostic says "unknown type name") or return an
        # unknown field - in neither case should it silently produce a primitive.
        src = "typedef struct { Eigen::Vector3d v; } M;"
        try:
            meta = parseStruct(str(hfile(src)), "M", compiler_args=["-x", "c++"])
            # Did not raise → the field must be unknown, not a silently wrong primitive
            assert meta["complete"] is False
            assert meta["fields"][0]["kind"] == "unknown"
        except ValueError:
            pass  # _check_diagnostics fired - also an acceptable outcome
