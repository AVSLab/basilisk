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
import numpy
import pytest
from Basilisk.architecture import sim_model
from Basilisk.simulation import stateArchitecture


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_


@pytest.mark.parametrize("function", ["stateData"
                                      , "stateArchitectureTest"
                                      , "stateProperties"
                                      , "EigenConversions"
                                      ])
def test_stateArchitectureAllTests(show_plots, function):
    """Module Unit Test"""
    [testResults, testMessage] = eval(function + '(show_plots)')
    assert testResults < 1, testMessage


def stateData(show_plots):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    stateUse = [[10.0], [20.0]]
    stateName = "position"
    newState = stateArchitecture.StateData(stateName, stateUse)
    newState.setState(stateUse)
    
    predictedDerivative = [[0.0], [0.0]]

    if(newState.getRowSize() != len(stateUse)):
        testFailCount += 1
        testMessages.append("State row sized incorrectly") 
    if(newState.getColumnSize() != len(stateUse[0])):
        testFailCount += 1
        testMessages.append("State column sized incorrectly") 
    if(newState.getName() != stateName):
        testFailCount += 1
        testMessages.append("State name incorrect")
    if(newState.getState() != stateUse):
        testFailCount += 1
        testMessages.append("State equality check failure.")
    if(newState.getStateDeriv() != predictedDerivative):
        testFailCount += 1
        testMessages.append("State derivative zero check failure.")

    derivativeInc = [[1.0], [2.5]]
    newState.setDerivative(derivativeInc)
    newState.propagateState(0.1)

    predictedDerivativeNum = numpy.array(predictedDerivative) + numpy.array(derivativeInc)
    obsDerivativeNum = numpy.array(newState.getStateDeriv())
    if(obsDerivativeNum.tolist() != predictedDerivativeNum.tolist()):
        testFailCount += 1
        testMessages.append("State derivative update check failure.")

    stateUpdateNum = numpy.array(newState.getState())
    predUpStateNum = numpy.array(stateUse) + predictedDerivativeNum*0.1
    if(stateUpdateNum.tolist() != stateUpdateNum.tolist()):
        testFailCount += 1
        testMessages.append("State propagation update check failure.")
    
    priorState = stateUpdateNum
    scaleFactor = 0.25
    priorState *= scaleFactor
    outState = newState*scaleFactor
    newState.scaleState(scaleFactor)
    stateUpdateNum = numpy.array(newState.getState())
    if(stateUpdateNum.tolist() != priorState.tolist()):
        testFailCount += 1
        testMessages.append("State scaling update check failure.")
    if(outState.getState() != newState.getState()):
        testFailCount += 1
        testMessages.append("State scaling via * operator check failure.")


    dummyState = stateArchitecture.StateData()
    if(dummyState.getRowSize() != 0):
        testFailCount += 1
        testMessages.append("Dummy state row sized incorrectly")
    if(dummyState.getColumnSize() != 0):
        testFailCount += 1
        testMessages.append("Dummy state column sized incorrectly")

    dummyState.setState(newState.getState())
    
    outState = dummyState + newState
    if(outState.getState() != (2.0*stateUpdateNum).tolist()):
        testFailCount += 1
        testMessages.append("Plus operator failed on StateData")


    if testFailCount == 0:
        print("PASSED: " + " State data")
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def stateProperties(show_plots):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    newManager = stateArchitecture.DynParamManager()

    gravList = [[9.81], [0.0], [0.1]]
    gravName = "g_N"
    newManager.createProperty(gravName, gravList)
    
    propRef = newManager.getPropertyReference(gravName)
    
    if propRef != gravList:
        testFailCount += 1
        testMessages.append("Create and property reference matching failed.")
    
    newGravList = [[0.0], [9.81], [-0.1]]
    newManager.setPropertyValue(gravName, newGravList)
    propRef = newManager.getPropertyReference(gravName)
    if propRef != newGravList:
        testFailCount += 1
        testMessages.append("Set and property reference matching failed.")

    newGravList = [[0.0], [9.81*2], [-0.1]]
    newManager.createProperty(gravName, newGravList)
    propRef = newManager.getPropertyReference(gravName)
    if propRef != newGravList:
        testFailCount += 1
        testMessages.append("Set and property reference matching failed.")

    wrongGravList = [[0.0], [9.81], [-0.1]]
    newManager.setPropertyValue(gravName+"Scott", newGravList)
    propRef = newManager.getPropertyReference(gravName+"Scott")
    if propRef != None:
        testFailCount += 1
        testMessages.append("Set and property reference matching failed.")

    massList = [[1500.0]]
    massName = "mass"
    newManager.createProperty(massName, massList)
    massRef = newManager.getPropertyReference(massName)
    if massRef != massList:
        testFailCount += 1
        testMessages.append("1x1 Eigen property creation failed.")


    if testFailCount == 0:
        print("PASSED: " + " State properties")
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def stateArchitectureTest(show_plots):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    newManager = stateArchitecture.DynParamManager()
    
    positionName = "position"
    stateDim = [3, 1]
    posState = newManager.registerState(stateDim[0], stateDim[1], positionName)
    
    velocityName = "velocity"
    stateDim = [3, 1]
    velState = newManager.registerState(stateDim[0], stateDim[1], velocityName)
    
    flexName = "Array1_flex"
    flexDim = [2, 1]
    flexState = newManager.registerState(flexDim[0], flexDim[1], flexName)
    
    if posState.getRowSize() != stateDim[0] or posState.getColumnSize() != stateDim[1]:
        testFailCount += 1
        testMessages.append("Position state returned improper size")
    
    if velState.getName() != velocityName:
        testFailCount += 1
        testMessages.append("Failed to return proper state name for velocity")
    
    if(newManager.registerState(stateDim[0], stateDim[1], positionName).getName() != positionName):
        testFailCount += 1
        testMessages.append("Failed to return proper state name in overload of call")
    newManager.registerState(stateDim[0], stateDim[1]+2, positionName)
    
    positionStateLookup = newManager.getStateObject("Array1_flex")

    if(positionStateLookup.getName() != flexName):
        testFailCount += 1
        testMessages.append("State lookup for solar array flex failed")
    
    vectorFactor = 4.0
    vecStart = [[1.0], [2.0], [3.5]]
    posState.setState(vecStart)
    velState.setState(vecStart)
    vectorStart = newManager.getStateVector()
    vectorComposite = vectorStart + vectorStart*vectorFactor + vectorStart*vectorFactor
    numpyOutput = numpy.array(vecStart) + numpy.array(vecStart)*vectorFactor + numpy.array(vecStart)*vectorFactor
    newManager.updateStateVector(vectorComposite)
    
    if(velState.getState() != numpyOutput.tolist()):
        testFailCount += 1
        testMessages.append("Velocity state update via state-manager failed")

    dt = 1.0;
    posState.setDerivative(vecStart)
    newManager.propagateStateVector(dt)
    numpyOutput += numpy.array(vecStart)*dt
    if(posState.getState() != numpyOutput.tolist()):
        testFailCount += 1
        testMessages.append("Position state propagation via state-manager failed")

    if testFailCount == 0:
        print("PASSED: " + " State manager")
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def EigenConversions(show_plots):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    inputArray = [[3.0], [1.0], [2.0]]
    outputArray = sim_model.new_doubleArray(3)
    stateArchitecture.eigenVector3d2CArray(inputArray, outputArray)
    
    flatList =  [y for x in inputArray for y in x]

    for i in range(len(flatList)):
        if(flatList[i] != sim_model.doubleArray_getitem(outputArray, i)):
            testFailCount += 1
            testMessages.append("3-vector conversion failed")

    inputArray = [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]]
    outputArray = sim_model.new_doubleArray(9)
    stateArchitecture.eigenMatrix3d2CArray(inputArray, outputArray)
    
    flatList =  [y for x in inputArray for y in x]

    for i in range(len(flatList)):
        if(flatList[i] != sim_model.doubleArray_getitem(outputArray, i)):
            print(sim_model.doubleArray_getitem(outputArray, i))
            testFailCount += 1
            testMessages.append("3x3 matrix conversion failed")

    inputArray = [[0.0, 1.0, 0.0, 2.0], [0.0, 0.0, 1.0, 0.5], [1.0, 0.0, 0.0, 2.7]]
    outputArray = sim_model.new_doubleArray(12)
    stateArchitecture.eigenMatrixXd2CArray(inputArray, outputArray)
    
    flatList =  [y for x in inputArray for y in x]

    for i in range(len(flatList)):
        if(flatList[i] != sim_model.doubleArray_getitem(outputArray, i)):
            print(sim_model.doubleArray_getitem(outputArray, i))
            testFailCount += 1
            testMessages.append("3x4 matrix conversion failed")

    if testFailCount == 0:
        print("PASSED: " + " Eigen Conversions")
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    stateArchitectureAllTest(False)
    
