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

import numpy as np
import pytest

from Basilisk.utilities import deprecated, simHelpers, unitTestSupport


SIM_HELPER_COMPATIBILITY_METHODS = (
    "EigenVector3d2list",
    "EigenVector3d2np",
    "addTimeColumn",
    "checkMethodKeyword",
    "columnToRowList",
    "decimalYearToDateTime",
    "flattenList",
    "getLineColor",
    "getScenarioFigureFileName",
    "np2EigenMatrix3d",
    "np2EigenVectorXd",
    "npList2EigenXdVector",
    "pullVectorSetFromData",
    "removeTimeFromData",
    "samplingTime",
    "saveFigurePDF",
    "saveScenarioFigure",
    "saveScenarioGraphvizFigure",
    "timeStringToGregorianUTCMsg",
    "writeFigureLaTeX",
    "writeTableLaTeX",
    "writeTeXSnippet",
)


def _callCompatibilityMethod():
    """Call a simple compatibility wrapper with deterministic input."""
    timeData = np.array([0.0, 1.0])  # [s]
    data = np.array([[2.0], [3.0]])
    return unitTestSupport.addTimeColumn(timeData, data)


@pytest.mark.parametrize("methodName", SIM_HELPER_COMPATIBILITY_METHODS)
def test_unitTestSupportProvidesSimHelperCompatibilityMethods(methodName):
    """Check that moved helpers remain available from ``unitTestSupport``."""
    compatibilityMethod = getattr(unitTestSupport, methodName, None)

    assert compatibilityMethod is not None
    assert callable(compatibilityMethod)
    assert callable(getattr(simHelpers, methodName))


def test_unitTestSupportCompatibilityMethodCanStillBeCalled():
    """Check a moved helper remains callable from ``unitTestSupport``."""
    # If this test starts emitting a ``BSKUrgentDeprecationWarning``, then the
    # compatibility deadline has passed and these wrappers should be removed.
    with deprecated.ignore(r"Basilisk\.utilities\.unitTestSupport\.addTimeColumn"):
        result = _callCompatibilityMethod()

    np.testing.assert_array_equal(
        result,
        simHelpers.addTimeColumn(np.array([0.0, 1.0]), np.array([[2.0], [3.0]])),
    )
