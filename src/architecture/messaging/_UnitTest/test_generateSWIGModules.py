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
"""Unit tests for message auto-source generators."""

from __future__ import annotations

import json
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

MSG_AUTO_SOURCE_DIR = Path(__file__).parent.parent / "msgAutoSource"
NEW_MESSAGING_TEMPLATE = Path(__file__).parent.parent / "newMessaging.ih"

sys.path.insert(0, str(MSG_AUTO_SOURCE_DIR))
from generatePayloadEqualityHeader import generateEqualityHeader


def _generate_swig_interface(tmp_path, generate_c_info):
    """Generate a SWIG interface file for a simple test message payload."""

    # The generator normally consumes metadata emitted by libclang.  This test
    # only needs enough metadata to exercise the Python binding templates, so a
    # single scalar field keeps the fixture small while still producing a valid
    # payload extension block.
    meta = {
        "complete": True,
        "size_bytes": 8,  # [byte]
        "fields": [
            {
                "name": "value",
                "kind": "primitive",
                "ctype": "double",
                "offset_bytes": 0,  # [byte]
                "size_bytes": 8,  # [byte]
            },
        ],
    }
    meta_path = tmp_path / "CustomMsgPayload.json"
    output_path = tmp_path / "CustomMsgPayload.i"
    meta_path.write_text(json.dumps(meta), encoding="utf-8")

    # Run the real generator instead of duplicating its formatting logic in the
    # test.  The header path does not need to exist because generateSWIGModules.py
    # only splices the path into the SWIG template.
    subprocess.run(
        [
            sys.executable,
            "generateSWIGModules.py",
            str(output_path),
            str(tmp_path / "CustomMsgPayload.h"),
            "CustomMsgPayload",
            str(tmp_path),
            str(generate_c_info),
            str(meta_path),
            "0",
        ],
        cwd=MSG_AUTO_SOURCE_DIR,
        check=True,
    )

    return output_path.read_text(encoding="utf-8")


def _render_read_functor_namespace():
    """Render the generated ``ReadFunctor`` Python helpers for execution."""

    template = NEW_MESSAGING_TEMPLATE.read_text(encoding="utf-8")
    start = template.index("            def subscribeTo(self, source):")
    end = template.index("        %}", start)
    read_functor_methods = template[start:end]
    read_functor_methods = read_functor_methods.replace("messageType ## _C", "CustomMsg_C")
    read_functor_methods = read_functor_methods.replace("messageTypePayload", "CustomMsgPayload")
    read_functor_methods = read_functor_methods.replace("messageType", "CustomMsg")
    read_functor_methods = textwrap.indent(textwrap.dedent(read_functor_methods).strip(), "    ")

    namespace = {}
    class_source = "\n".join([
        "class CustomMsg:",
        "    pass",
        "",
        "class OtherMsg:",
        "    pass",
        "",
        "class CustomMsgReader:",
        "    def __init__(self):",
        "        self.subscribed_to = None",
        "        self.keepalive_source = None",
        "",
        "    def __subscribe_to(self, source):",
        "        self.subscribed_to = source",
        "",
        "    def _install_keepalive(self, source):",
        "        self.keepalive_source = source",
        "",
        "    def __is_subscribed_to(self, source):",
        "        return source is self.subscribed_to",
        "",
        read_functor_methods,
    ])
    exec(class_source, namespace)

    return namespace


def test_generated_message_bindings_use_module_local_classes(tmp_path):
    """Generated subscription helpers do not assume the Basilisk package path."""

    generated = _generate_swig_interface(tmp_path, True)
    new_messaging_template = NEW_MESSAGING_TEMPLATE.read_text(encoding="utf-8")

    assert "from Basilisk.architecture.messaging.messageType" not in new_messaging_template
    assert "if type(source) == messageType ## _C:" in new_messaging_template
    assert "from Basilisk.architecture.messaging import {type}" not in generated
    assert "elif type(source) == CustomMsg:" in generated


def test_keepalive_callbacks_skip_python_finalization():
    """Keep-alive callbacks do not acquire the GIL while Python is finalizing."""

    template = NEW_MESSAGING_TEMPLATE.read_text(encoding="utf-8")

    finalizing_guard = (
        "#if PY_VERSION_HEX >= 0x030D0000 && "
        "(!defined(Py_LIMITED_API) || Py_LIMITED_API+0 >= 0x030D0000)"
    )
    assert finalizing_guard in template
    assert "if (Py_IsFinalizing()) return false;" in template
    assert template.count("if (!_bsk_python_runtime_is_usable()) return;") == 2
    assert "_bsk_readfunctor_acquire(source);" in template


def test_generated_read_functor_without_c_interface_uses_cpp_message(tmp_path):
    """Generated helpers still work when no C-interface symbol exists."""

    generated = _generate_swig_interface(tmp_path, False)

    assert "cMsgCInterface/CustomMsg_C.h" not in generated
    assert "%extend CustomMsg_C" not in generated
    assert "CustomMsg_C" not in generated

    namespace = _render_read_functor_namespace()
    assert "CustomMsg_C" not in namespace

    reader = namespace["CustomMsgReader"]()
    source = namespace["CustomMsg"]()
    reader.subscribeTo(source)

    assert reader.subscribed_to is source
    assert reader.keepalive_source is source
    assert reader.isSubscribedTo(source) is True

    other_source = namespace["OtherMsg"]()
    assert reader.isSubscribedTo(other_source) == 0
    with pytest.raises(Exception, match="tried to subscribe ReadFunctor<CustomMsgPayload>"):
        reader.subscribeTo(other_source)


def test_payload_equality_generates_fixed_array_comparison():
    """Generated equality compares fixed numeric array payloads element-wise."""

    meta = {
        "fields": [
            {
                "name": "dataVector",
                "kind": "array",
                "shape": [3],
                "element": {"kind": "primitive", "ctype": "double"},
            },
        ],
    }

    generated = generateEqualityHeader(meta, "CModuleTemplateMsgPayload", "msgPayloadDefC")

    assert generated is not None
    assert "std::memcmp" not in generated
    assert "struct PayloadEqualityTraits<CModuleTemplateMsgPayload>" in generated
    assert '#include "architecture/msgPayloadDefC/CModuleTemplateMsgPayload.h"' in generated
    assert "static constexpr bool supported = true" in generated
    assert "for (int i0 = 0; i0 < 3; ++i0)" in generated
    assert "if (!(lhs.dataVector[i0] == rhs.dataVector[i0]))" in generated
    assert "return true;" in generated


def test_payload_equality_generates_nested_struct_comparison():
    """Generated equality walks nested struct arrays field by field."""

    meta = {
        "fields": [
            {
                "name": "reactionWheels",
                "kind": "array",
                "shape": [36],
                "element": {
                    "kind": "struct",
                    "typeName": "RWConfigElementMsgPayload",
                    "fields": [
                        {
                            "name": "gsHat_B",
                            "kind": "array",
                            "shape": [3],
                            "element": {"kind": "primitive", "ctype": "double"},
                        },
                        {"name": "Js", "kind": "primitive", "ctype": "double"},
                    ],
                },
            },
        ],
    }

    generated = generateEqualityHeader(meta, "RWConstellationMsgPayload", "msgPayloadDefC")

    assert generated is not None
    assert "for (int i0 = 0; i0 < 36; ++i0)" in generated
    assert "for (int i1 = 0; i1 < 3; ++i1)" in generated
    assert "lhs.reactionWheels[i0].gsHat_B[i1] == rhs.reactionWheels[i0].gsHat_B[i1]" in generated
    assert "lhs.reactionWheels[i0].Js == rhs.reactionWheels[i0].Js" in generated
    assert "return true;" in generated


def test_payload_equality_generates_eigen_comparison():
    """Generated equality compares Eigen dimensions and coefficients."""

    meta = {
        "fields": [
            {"name": "qpos", "kind": "unknown", "ctype": "Eigen::VectorXd"},
        ],
    }

    generated = generateEqualityHeader(meta, "MJSceneStateMsgPayload", "msgPayloadDefCpp")

    assert generated is not None
    assert "lhs.qpos.rows() != rhs.qpos.rows()" in generated
    assert "lhs.qpos.cols() != rhs.qpos.cols()" in generated
    assert "if ((lhs.qpos.array() != rhs.qpos.array()).any())" in generated
    assert "return true;" in generated


def test_payload_equality_uses_operator_equal_for_safe_stl_fields():
    """Generated equality uses ``operator==`` for supported STL payload fields."""

    meta = {
        "fields": [
            {"name": "keyboardInput", "kind": "unknown", "ctype": "std::string"},
            {"name": "samples", "kind": "unknown", "ctype": "std::vector<double>"},
        ],
    }

    generated = generateEqualityHeader(meta, "StringVectorMsgPayload", "msgPayloadDefCpp")

    assert generated is not None
    assert "if (!(lhs.keyboardInput == rhs.keyboardInput))" in generated
    assert "if (!(lhs.samples == rhs.samples))" in generated
    assert "return true;" in generated


def test_payload_equality_returns_none_for_unsupported_fields():
    """Payloads with pointer or unsupported fields return None — no specialization generated."""

    meta = {
        "fields": [
            {"name": "frameNumber", "kind": "primitive", "ctype": "int"},
            {"name": "imagePointer", "kind": "pointer", "ctype": "void *"},
            {
                "name": "vizEventReplies",
                "kind": "unknown",
                "ctype": "std::vector<VizEventReply>",
            },
            {"name": "lateField", "kind": "primitive", "ctype": "int"},
        ],
    }

    generated = generateEqualityHeader(meta, "VizUserInputMsgPayload", "msgPayloadDefCpp")

    assert generated is None
