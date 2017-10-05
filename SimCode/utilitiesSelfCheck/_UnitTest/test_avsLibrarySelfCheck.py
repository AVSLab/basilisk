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
#   Integrated Unit Test Script
#   Purpose:  Self-check on the AVS C-code support libraries
#   Author:  Hanspeter Schaub
#   Creation Date:  August 11, 2017
#

import pytest
import sys, os, inspect



filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('Basilisk')
#sys.path.append(splitPath[0] + '/Basilisk/modules')
#sys.path.append(splitPath[0] + '/Basilisk/PythonModules')

from Basilisk.modules import avsLibrarySelfCheck
from Basilisk.utilities import unitTestSupport





# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

@pytest.mark.parametrize("testRigidBodyKinematics, testOrbitalElements, testOrbitalAnomalies, testLinearAlgebra, testEnvironment", [
      (True, False, False, False, False)
    , (False, True, False, False, False)
    , (False, False, True, False, False)
    , (False, False, False, True, False)
    , (False, False, False, False, True)
])


# provide a unique test method name, starting with test_
def test_unitDynamicsModes(testRigidBodyKinematics, testOrbitalElements, testOrbitalAnomalies, testLinearAlgebra, testEnvironment):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitAVSLibrarySelfCheck(
            testRigidBodyKinematics, testOrbitalElements, testOrbitalAnomalies, testLinearAlgebra, testEnvironment)
    assert testResults < 1, testMessage



def unitAVSLibrarySelfCheck(testRigidBodyKinematics, testOrbitalElements, testOrbitalAnomalies, testLinearAlgebra, testEnvironment):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    if testRigidBodyKinematics:
        errorCount = avsLibrarySelfCheck.testRigidBodyKinematics(1e-10)
        if errorCount:
            testFailCount += errorCount
            testMessages.append("ERROR: Rigid Body Kinematics Library Failed Self Test.\n")
    if testOrbitalElements:
        errorCount = avsLibrarySelfCheck.testOrbitalAnomalies(1e-10)
        if errorCount:
            testFailCount += errorCount
            testMessages.append("ERROR: Orbital Anomalies Library Failed Self Test.\n")
    if testOrbitalAnomalies:
        errorCount = avsLibrarySelfCheck.testOrbitalElements(1e-10)
        if errorCount:
            testFailCount += errorCount
            testMessages.append("ERROR: Orbital Elements Library Failed Self Test.\n")
    if testLinearAlgebra:
        errorCount = avsLibrarySelfCheck.testLinearAlgebra(1e-10)
        if errorCount:
            testFailCount += errorCount
            testMessages.append("ERROR: Linear Algebra Library Failed Self Test.\n")
    if testEnvironment:
        errorCount = avsLibrarySelfCheck.testOrbitalEnvironment(1e-10)
        if errorCount:
            testFailCount += errorCount
            testMessages.append("ERROR: Space Environment Library Failed Self Test.\n")




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
    fileNameString = filename[len(path) + 6:-3]
    print path
    snippetMsgName = fileNameString + 'Msg-' + str(testRigidBodyKinematics) + str(testOrbitalElements) \
                     + str(testOrbitalAnomalies) + str(testLinearAlgebra) + str(testEnvironment)
    unitTestSupport.writeTeXSnippet(snippetMsgName, snippetContent,
                                    path + "/../_Documentation/")
    snippetPassFailName = fileNameString + 'TestMsg-' + str(testRigidBodyKinematics) + str(testOrbitalElements) \
                          + str(testOrbitalAnomalies) + str(testLinearAlgebra) + str(testEnvironment)
    snippetContent = '\\textcolor{' + colorText + '}{' + passFailText + '}'
    unitTestSupport.writeTeXSnippet(snippetPassFailName, snippetContent,
                                    path + "/../_Documentation/")


    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    unitAVSLibrarySelfCheck(
                           True,           # rigidBodyKinematics
                           False,           # orbitalMotion
                           False,           # linearAlgebra
                           False            # environment
                           )

