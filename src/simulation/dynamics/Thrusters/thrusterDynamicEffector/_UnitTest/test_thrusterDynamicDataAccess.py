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

#
#   Unit Test Script
#   Module Name:        thrusterDynamicEffector
#   Author:             robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date:      June 24, 2026
#

"""
Regression test for issue #1254.

The ``thrusterData`` member of ``ThrusterDynamicEffector`` is a
``std::vector<std::shared_ptr<THRSimConfig>>``. After THRSimConfig was changed
to a shared pointer (commit f93eaa97b) the SWIG ``%template`` still targeted the
old value type, so ``thrusterData`` came back to Python as an opaque
``SwigPyObject`` that could not be indexed or iterated. This test pins the
member to behave as an iterable list of THRSimConfig objects.
"""

from Basilisk.simulation import thrusterDynamicEffector


def test_thrusterData_is_iterable_list():
    """Test that ``thrusterData`` exposes ``THRSimConfig`` objects in Python."""
    thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
    maxThrusts = [10.0, 20.0]  # [N]
    for maxThrust in maxThrusts:
        config = thrusterDynamicEffector.THRSimConfig()
        config.thrLoc_B = [[1.0], [0.0], [0.0]]  # [m]
        config.thrDir_B = [[1.0], [0.0], [0.0]]  # [-]
        config.MaxThrust = maxThrust
        thrusterSet.addThruster(config)

    data = thrusterSet.thrusterData

    # Indexing and len must work (previously raised TypeError on an opaque object).
    assert len(data) == len(maxThrusts)
    assert [data[i].MaxThrust for i in range(len(data))] == maxThrusts

    # Iteration must work.
    assert [config.MaxThrust for config in data] == maxThrusts

    # shared_ptr semantics: mutating through the proxy is reflected in the module.
    data[0].MaxThrust = 99.0  # [N]
    assert thrusterSet.thrusterData[0].MaxThrust == 99.0


if __name__ == "__main__":
    test_thrusterData_is_iterable_list()
