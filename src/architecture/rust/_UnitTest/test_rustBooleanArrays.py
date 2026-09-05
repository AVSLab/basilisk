# ISC License
#
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

"""Exercise Boolean aliases through generated Rust, C++, SWIG, and Python accessors."""

import copy

import pytest

from Basilisk import hasBuildFeature
from Basilisk.architecture.bskLogging import BasiliskError

rustModulesEnabled = hasBuildFeature("rustModules")
pytestmark = pytest.mark.skipif(
    not rustModulesEnabled,
    reason="Requires Basilisk built with --rustModules True",
)
if rustModulesEnabled:
    from Basilisk.moduleTemplates import rustModuleTemplate


@pytest.mark.parametrize(
    "fieldName, values, wrongLength",
    [
        ("sampleFlags", [True, False, True], [False, True]),
        ("sampleFlagMatrix", [[True, False], [False, True]], [[True, False]]),
    ],
    ids=["boolean-alias", "alias-chain-matrix"],
)
def test_boolean_alias_array_accessors(fieldName, values, wrongLength):
    """Round-trip aliased arrays without sharing storage or accepting wrong sizes.

    :param fieldName: Configuration property on the template module.
    :param values: Expected vector or matrix of Python Boolean values.
    :param wrongLength: A proposal containing too few elements.
    """
    module = rustModuleTemplate.rustModuleTemplate()
    suffix = fieldName[0].upper() + fieldName[1:]
    getter = getattr(module, f"get{suffix}")
    setter = getattr(module, f"set{suffix}")
    defaults = getattr(module, fieldName)
    flatDefaults = defaults if fieldName == "sampleFlags" else sum(defaults, [])
    assert all(value is False for value in flatDefaults)

    setattr(module, fieldName, values)
    assert getter() == values
    actual = getter()
    flattened = actual if fieldName == "sampleFlags" else sum(actual, [])
    assert all(type(value) is bool for value in flattened)

    modified = copy.deepcopy(values)
    if fieldName == "sampleFlags":
        actual[0] = not actual[0]
        modified[0] = not modified[0]
    else:
        actual[0][0] = not actual[0][0]
        modified[0][0] = not modified[0][0]
    assert getattr(module, fieldName) == values
    setter(modified)
    assert getattr(module, fieldName) == modified

    for proposal in [wrongLength, [], values + values]:
        with pytest.raises(BasiliskError, match="wrong number of values"):
            setattr(module, fieldName, proposal)
        assert getter() == modified
        with pytest.raises(BasiliskError, match="wrong number of values"):
            setter(proposal)
        assert getter() == modified

    # Neither an array update nor its rejected proposals should touch adjacent fields.
    assert module.panicOnUpdate is False
    assert module.dummy == 0.0  # [-]
