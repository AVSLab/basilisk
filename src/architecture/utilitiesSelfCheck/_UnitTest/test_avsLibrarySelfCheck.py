#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#
#   Integrated Unit Test Script
#   Purpose:  Self-check on the AVS C-code support libraries
#   Author:  Hanspeter Schaub
#   Creation Date:  August 11, 2017
#

import os

import pytest
from Basilisk.architecture import avsLibrarySelfCheck
from Basilisk.utilities import simHelpers


SELF_CHECKS = {
    "testRigidBodyKinematics": 1e-10,
    "testOrbitalAnomalies": 1e-10,  # [rad] absolute anomaly tolerance
    "testLinearAlgebra": 1e-10,
    "testOrbitalHill": 1e-4,  # [km] and [km/s] absolute state tolerances
}


@pytest.mark.parametrize("testName", SELF_CHECKS)
# provide a unique test method name, starting with test_
def test_unitDynamicsModes(testName):
    """AVS Library Self Check"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitAVSLibrarySelfCheck(testName)
    assert testResults < 1, testMessage


def unitAVSLibrarySelfCheck(testName):
    """Run a named native self-check; unknown names must never report success."""
    if testName not in SELF_CHECKS:
        raise ValueError(f"Unknown AVS self-check: {testName}")
    testFailCount = getattr(avsLibrarySelfCheck, testName)(SELF_CHECKS[testName])
    testMessages = [f"ERROR: {testName} failed self-check.\n"] if testFailCount else []

    if testFailCount == 0:
        print("PASSED ")
        passFailText = "PASSED"
        colorText = 'ForestGreen'  # color to write auto-documented "PASSED" message in in LATEX
        snippetContent = ""
    else:
        print(testFailCount)
        print(testMessages)
        passFailText = 'FAILED'
        colorText = 'Red'  # color to write auto-documented "FAILED" message in in LATEX
        snippetContent = ""
        for message in testMessages:
            snippetContent += message

    fileName = os.path.basename(os.path.splitext(__file__)[0])
    path = os.path.dirname(os.path.abspath(__file__))

    snippetMsgName = fileName + 'Msg-' + testName
    simHelpers.writeTeXSnippet(snippetMsgName, snippetContent, path + "/../_Documentation/")

    snippetPassFailName = fileName + 'TestMsg-' + testName
    snippetContent = r'\textcolor{' + colorText + '}{' + passFailText + '}'
    simHelpers.writeTeXSnippet(snippetPassFailName, snippetContent, path + "/../_Documentation/")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


@pytest.mark.parametrize("name", SELF_CHECKS)
@pytest.mark.parametrize("error_count", [0, 3])
def test_self_check_dispatch(monkeypatch, name, error_count):
    """Every advertised selector calls native code and propagates its failures."""
    calls = []
    def native_check(tolerance):
        calls.append(tolerance)
        return error_count
    monkeypatch.setattr(avsLibrarySelfCheck, name, native_check)
    monkeypatch.setattr(simHelpers, "writeTeXSnippet", lambda *args: None)
    count, message = unitAVSLibrarySelfCheck(name)
    assert calls == [SELF_CHECKS[name]]
    assert count == error_count
    assert bool(message) == bool(error_count)


@pytest.mark.parametrize("name", ["testOrbitalElements", "testEnvironment", "nonexistent_test"])
def test_self_check_unknown_selector(name):
    """Previously empty test selectors must fail instead of reporting PASSED."""
    with pytest.raises(ValueError, match="Unknown AVS self-check"):
        unitAVSLibrarySelfCheck(name)


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    unitAVSLibrarySelfCheck(
        "testOrbitalHill"
    )
