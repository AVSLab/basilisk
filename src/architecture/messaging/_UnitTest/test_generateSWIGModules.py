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
"""Unit tests for ``generatePayloadEqualityHeader.py``."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "msgAutoSource"))
from generatePayloadEqualityHeader import generateEqualityHeader


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
