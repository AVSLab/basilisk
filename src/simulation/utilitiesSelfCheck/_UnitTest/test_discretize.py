''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''

#
#   Unit Test for Discretize Utility
#   Purpose:  Self-check on Discretize
#   Author:  Scott Carnahan
#   Creation Date:  Jan 18, 2018
#   Note that all of this test is really in c++. This script is just a pytest access to those tests.

import pytest
import os
from Basilisk.simulation import discretizeCheck
from Basilisk.utilities import unitTestSupport


@pytest.mark.parametrize("testName"
    , ["testDiscretize"])
# provide a unique test method name, starting with test_
def test_discretize(testName):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitDiscretizeCheck(testName)
    assert testResults < 1, testMessage


def unitDiscretizeCheck(testName):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    testFailCount = discretizeCheck.testDiscretize()
    if testFailCount > 0:
        testMessages.append("ERROR: Discretize Test.\n")

    if testFailCount == 0:
        print "PASSED "
        passFailText = "PASSED"
        colorText = 'ForestGreen'  # color to write auto-documented "PASSED" message in in LATEX
        snippetContent = ""
    else:
        print testFailCount
        print testMessages
        passFailText = 'FAILED'
        colorText = 'Red'  # color to write auto-documented "FAILED" message in in LATEX
        snippetContent = ""
        for message in testMessages:
            snippetContent += message

    fileName = os.path.basename(os.path.splitext(__file__)[0])
    path = os.path.dirname(os.path.abspath(__file__))

    snippetPassFailName = "discretizePass"
    snippetContent = '\\textcolor{' + colorText + '}{' + passFailText + '}'
    unitTestSupport.writeTeXSnippet(snippetPassFailName, snippetContent, path + "/../_Documentation/discretize/AutoTex/")
    print path
    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    unitDiscretizeCheck(
        "testDiscretize"
    )
