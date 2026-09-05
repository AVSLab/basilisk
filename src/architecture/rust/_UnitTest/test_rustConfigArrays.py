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

"""Exercise numeric configuration arrays beyond Rust's standard Default limit."""

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


@pytest.mark.parametrize("useMethods", [False, True], ids=["property", "methods"])
def test_large_numeric_configuration_array(useMethods):
    """Initialize, copy, and validate all 64 values through the generated API.

    :param useMethods: Use explicit setters/getters rather than the property.
    """
    module = rustModuleTemplate.rustModuleTemplate()
    getValues = module.getSampleArray if useMethods else lambda: module.sampleArray
    setValues = (
        module.setSampleArray if useMethods
        else lambda value: setattr(module, "sampleArray", value)
    )
    assert getValues() == [0.0] * 64
    values = [float(index) - 10.5 for index in range(64)]  # [-]
    setValues(values)
    assert getValues() == values

    returned = getValues()
    returned[40] = -2.5  # [-]
    assert getValues() == values
    setValues(returned)
    assert getValues() == returned

    for length in (32, 33, 63, 65):
        with pytest.raises(BasiliskError, match="sampleArray has the wrong number of values"):
            setValues([0.0] * length)
        assert getValues() == returned

    # Reject invalid values beyond the old 32-element boundary, preserving
    # the entire preceding configuration rather than partially updating it.
    for index in (32, 63):
        for invalid in (float("nan"), float("inf"), -float("inf")):
            proposal = returned.copy()
            proposal[index] = invalid
            with pytest.raises(BasiliskError, match="sampleArray components must be finite"):
                setValues(proposal)
            assert getValues() == returned

    other = rustModuleTemplate.rustModuleTemplate()
    assert other.sampleArray == [0.0] * 64
    assert module.increment == 1.0  # [-]
