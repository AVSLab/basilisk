''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#   Unit Test for Gauss Markov
#   Purpose:  Self-check on Gauss Markov
#   Author:  Scott Carnahan
#   Creation Date:  August 11, 2017
#

import pytest
import os
from Basilisk.simulation import gaussMarkovCheck
from Basilisk.utilities import unitTestSupport


@pytest.mark.parametrize("testName"
    , ["testGaussMarkov"])
# provide a unique test method name, starting with test_
def test_unitGausMarkov(testName):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitGausMarkovSelfCheck(testName)
    assert testResults < 1, testMessage


def unitGausMarkovSelfCheck(testName):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    testFailCount = gaussMarkovCheck.testGaussMarkov(1e-10)
    if testFailCount > 0:
        testMessages.append("ERROR: GaussMarkov Test.\n")

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

    snippetMsgName = fileName + 'Msg-' + testName
    unitTestSupport.writeTeXSnippet(snippetMsgName, snippetContent, path + "/../_Documentation/AutoTex/")

    snippetPassFailName = fileName + 'TestMsg-' + testName
    snippetContent = '\\textcolor{' + colorText + '}{' + passFailText + '}'
    unitTestSupport.writeTeXSnippet(snippetPassFailName, snippetContent, path + "/../_Documentation/AutoTex/")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    unitGausMarkovSelfCheck(
        "testGaussMarkov"
    )
